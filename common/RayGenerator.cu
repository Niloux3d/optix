#include "RayGenerator.cuh"
//#include <cuda/helpers.h>
#include <cuda/whitted.h>
#include <cuda/LocalGeometry.h>

#include <sutil/vec_math.h>

extern "C" {
__constant__ Params params;
}

// Lidar：返回坐标，强度
// Radar：返回坐标，发射功率，法向量
//static __forceinline__ __device__ void setPayload(float3 p, float i, float3 n, float intensity, float2 g_speed)
static __forceinline__ __device__ void setPayload(float3 p, float i, float3 n, float intensity)
{
    optixSetPayload_0( float_as_int( p.x ) );
    optixSetPayload_1( float_as_int( p.y ) );
    optixSetPayload_2( float_as_int( p.z ) );
    optixSetPayload_3( float_as_uint( i ) );
    optixSetPayload_4( float_as_uint( n.x ) );
    optixSetPayload_5( float_as_uint( n.y ) );
    optixSetPayload_6( float_as_uint( n.z ) );
    optixSetPayload_7( float_as_uint( intensity ) );
    //optixSetPayload_8( float_as_int( g_speed.x ) );
    //optixSetPayload_9( float_as_int( g_speed.y ) );
}

// [一汽需求] 包络线 增加激光点云hpr 2022.08.17 LS
// 向量旋转
static __forceinline__ __device__ void cuda_vector_rotated( float yaw, float pitch, float roll, float3& _direction )
{
    const double trans_x = 
                            ( 
                                _direction.x * ( cos(yaw) * cos(pitch) ) 
                                + _direction.y * (-sin(yaw) * cos(pitch))   //x后旋转坐标
                                + _direction.z * sin(pitch)
                            );

    const double trans_y = 
                            (
                                _direction.x * (sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll))
                                + _direction.y * (cos(yaw) * cos(roll) - sin(pitch) * sin(yaw) * sin(roll)) //y旋转后的坐标
                                + _direction.z * (-cos(pitch) * sin(roll) ) 
                            );
    const double trans_z =
                            (
                                _direction.x * (sin(yaw) * sin(roll) - cos(yaw) * sin(pitch) * cos(roll))
                                + _direction.y * (cos(yaw) * sin(roll) + sin(pitch) * sin(yaw) * cos(roll)) // z旋转后的坐标
                                + _direction.z * (cos(pitch) * cos(roll))   
                            );

    _direction = make_float3(trans_x, trans_y, trans_z);
}

static __forceinline__ __device__ void computeRay( uint3 idx, float3& origin, float3& direction)
{
    origin    = params.origin;

    
    float horizontal_radian = -(params.hFov/2) + idx.x * params.pulse_divergence_angle;
    float vertical_radian = params.lower_fov + idx.y * params.step_fov;
    float vertical_radian_cos = cosf(vertical_radian);
#ifndef NDEBUG
	// printf("%s:%d %s horizontal_radian(%ld), vertical_radian(%ld), vertical_radian_cos(%ld)\n", __FILE__, __LINE__, __FUNCTION__, horizontal_radian, vertical_radian, vertical_radian_cos);
#endif	

    direction.x = vertical_radian_cos * cosf(horizontal_radian);
    direction.y = vertical_radian_cos * sinf(horizontal_radian);
    direction.z = -sinf(vertical_radian);

#if 1 // [一汽需求] 包络线 增加激光点云hpr 2022.08.17 LS
    cuda_vector_rotated( params.yaw, params.pitch, params.roll, direction);
#endif

}

extern "C" __global__ void __raygen__rg()
{
    // Lookup our location within the launch grid
    const uint3 idx = optixGetLaunchIndex();

    // Map our launch idx to a screen location and create a ray from the camera
    // location through the screen
    float3 ray_origin, ray_direction;
    computeRay( idx, ray_origin, ray_direction);

    // Trace the ray against our scene hierarchy
    // unsigned int p0, p1, p2, p3, p4, p5, p6, p7, p8, p9;
    unsigned int p0, p1, p2, p3, p4, p5, p6, p7;
    optixTrace(
            params.handle,
            ray_origin,
            ray_direction,
            params.minDetectRange,        // Min intersection distance
            params.maxDetectRange,        // Max intersection distance
            0.0f,                // rayTime -- used for motion blur
            OptixVisibilityMask( 255 ), // Specify always visible
            OPTIX_RAY_FLAG_NONE,
            0,                   // SBT offset   -- See SBT discussion
            1,                   // SBT stride   -- See SBT discussion
            0,                   // missSBTIndex -- See SBT discussion
            p0, p1, p2, p3, p4, p5, p6, p7 );
    float4 result;
    result.x = int_as_float( p0 );
    result.y = int_as_float( p1 );
    result.z = int_as_float( p2 );
    result.w = uint_as_float( p3 );

    // Record results in our output raster
    params.cloud_data[idx.y * params.width + idx.x] = result;

    float4 normal;
    normal.x = uint_as_float( p4 );
    normal.y = uint_as_float( p5 );
    normal.z = uint_as_float( p6 );
    normal.w = uint_as_float( p7 );

    params.normal_data[idx.y * params.width + idx.x] = normal;

}    


extern "C" __global__ void __miss__ms()
{
    MissData* miss_data  = reinterpret_cast<MissData*>( optixGetSbtDataPointer() );
    setPayload(  make_float3(0.0f, 0.0f, 0.0f), 0.0f ,make_float3(0.0f, 0.0f, 0.0f), 1.0f);
}


extern "C" __global__ void __closesthit__ch()
{
    HitGroupData* rt_data = (HitGroupData*)optixGetSbtDataPointer();
    const float3 ray_orig  = optixGetWorldRayOrigin();
    const float3 ray_dir   = optixGetWorldRayDirection();
    const float  ray_t     = optixGetRayTmax();
    float3       hit_point = ray_orig + ray_t * ray_dir;

    OptixTraversableHandle gas = optixGetGASTraversableHandle();
    unsigned int primIdx = optixGetPrimitiveIndex();
    const uint3 idx = optixGetLaunchIndex();
    params.mesh_index_data[idx.y * params.width + idx.x] = static_cast<int>(primIdx);
    unsigned int sbtIdx = optixGetSbtGASIndex();
    float time = optixGetRayTime();
    float3 data[3];
    optixGetTriangleVertexData(gas, primIdx, sbtIdx, time, data);
	
    //normal vector
    float3 u = normalize(data[0] - data[1]);
    float3 v = normalize(data[2] - data[1]);
    const float3 N   = ( cross( u ,v) );

    //for test
    float  incidence   = dot( N, -ray_dir );
    if(incidence < 0.0)
    {
        incidence = -incidence;
    }

    // 不同类型的传感器输出数据不一样：1 - Lidar; 6 - Radar
    if ( params.type == 1 )
    {
        //printf("primIdx(%d), reclectance: (%f) ", primIdx, rt_data->reflection);
        setPayload( hit_point, rt_data->reflection, N, incidence);
    }
    else if ( params.type == 6 )
    {
        if ( hit_point.x == 0 && hit_point.y == 0 && hit_point.z == 0 )
        {
            //setPayload( hit_point, 1.0f, N, 1.0f, make_float2(0.0f, 0.0f));
        }
        else
        {
            // 撞击点的水平、垂直夹角
            float3 hit_direction;
            hit_direction.x = hit_point.x - ray_orig.x;
            hit_direction.y = hit_point.y - ray_orig.y;
            hit_direction.z = hit_point.z - ray_orig.z;
            float2 hit_point_radian;
    
            // 坐标旋转, 输出两个向量的夹角
            float3 mEx = make_float3(1, 0, 0); // x轴基向量
            // 旋转基向量
            cuda_vector_rotated(params.egoYaw, params.egoPitch, params.egoRoll, mEx); // 旋转后的x轴基向量
            cuda_vector_rotated(params.yaw, params.pitch, params.roll, mEx); // 旋转后的x轴基向量
            
            // 计算撞击点水平，垂直方向的夹角
            float mulx_y = hit_direction.x * mEx.x + hit_direction.y * mEx.y;
            float mulx_z = hit_direction.x * mEx.x + hit_direction.z * mEx.z;

            float sqrt_hit_y = (float)sqrt((double)( hit_direction.x * hit_direction.x + hit_direction.y * hit_direction.y ));
            float sqrt_hit_z = (float)sqrt((double)( hit_direction.x * hit_direction.x + hit_direction.z * hit_direction.z ));
            float sqrt_mEx_y = (float)sqrt((double)( mEx.x * mEx.x + mEx.y * mEx.y ));
            float sqrt_mEx_z = (float)sqrt((double)( mEx.x * mEx.x + mEx.z * mEx.z ));

            hit_point_radian.x = acos( mulx_y / ( sqrt_hit_y * sqrt_mEx_y ) );
            hit_point_radian.y = acos( mulx_z / ( sqrt_hit_z * sqrt_mEx_z ) );
        
            // 计算射线方向的发射功率
            float transmitterPower = ( 1 - (hit_point_radian.x / (2 * params.hFov)) ) * ( 1 - (hit_point_radian.y / (2 *( params.upper_fov - params.lower_fov ))) ) * params.transmitter_power;

            //setPayload( hit_point, transmitterPower, N, 1.0f, make_float2(hit_point_radian.x, hit_point_radian.y));
        }
    }
    else
    {
        //setPayload( hit_point, 1.0f, N , 1.0f, make_float2(0.0f, 0.0f));
    }
}
