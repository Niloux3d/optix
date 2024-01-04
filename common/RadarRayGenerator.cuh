#ifndef RadarRayGenerator_ch
#define RadarRayGenerator_ch

#ifndef RADAROVERRIDE
    #define radar_ray_override 5
#endif

#include <vector_types.h>
#include <optix.h>


struct RadarParams
{
    float3                  origin;
    int                     type;                   // 传感器类型：1 - Lidar; 6 - Radar
    float                   hFov;                   // 水平视场角
    float                   horizontal_resolution;  // 水平视场角分辨率 = 垂直视场角分辨率
    float                   upper_fov;              // 垂直视场角结束点
    float                   lower_fov;              // 垂直视场角开始点
    float                   step_fov;               // 垂直视场角步长
    float                   minDetectRange;         // 最近探测距离
    float                   maxDetectRange;	        // 最远探测距离（单位：米）
#if 1 // [一汽需求] 包络线 增加激光点云hpr 2022.08.17 LS
    float                   yaw;                    // 传感器航向角
    float                   pitch;                  // 传感器俯仰角
    float                   roll;                   // 传感器横滚角
#endif
    float                   egoYaw;                 // 车辆航向角
    float                   egoPitch;               // 车辆俯仰角
    float                   egoRoll;                // 车辆横滚角
    /* Lidar */
    float                   attenuation;            // 大气衰减率
    float4*                 cloud_data;             // 点云坐标，强度
    int*                    mesh_index_data;        //碰撞到的mesh对应的下标
    /* Radar */
    float                   transmitter_power;      // 发射功率
    float4*                 ray_data;               // xyz，发射功率
    float4*                 normal_data;            // 法向量
    unsigned int            width;                  //2pi / horizontal_resolution
    unsigned int            height;                 //(upper_fov - lower_fov) / step_fov
    OptixTraversableHandle handle;
};

#endif

#ifndef RayGenerator_ch
#define RayGenerator_ch
struct RayGenData
{
    // No data needed
};


struct MissData
{
//    float3 bg_color;
};


struct HitGroupData
{
    float reflection;
};

#endif
