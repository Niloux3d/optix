#include <map>
#include <vector>
#include <cuda_runtime.h>
#include <time.h>

#include "common.h"
// #include "helper_math.h"
#include "sutil/vec_math.h"
#include "CalculateLidarRay.cuh"
#include <pthread.h>
#include <vector_types.h>
#include <curand_kernel.h>

#define LIGHTSPEED (3e8) 	// 光速：299792458米/秒
#define DISCRETEPOINT (2000) 	// 离散点
#define PI (3.141592653589793)	// 弧度

#ifndef MIN
#define MIN(a, b) ((a < b) ? a : b)
#endif
#ifndef MAX
#define MAX(a, b) (a > b ? a : b)
#endif

// static pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
__constant__ LidarSimParams g_d_lidarSimParams;

//计算数据（供外部调用）
__host__ bool calculate_lidar_ray(const std::vector<float4> h_closest_hits_data,const std::vector<float4> h_normal_data, std::vector<float4>& h_raw_data_phy, int h_data_size, LidarSimParams* lidarSimParams,const std::vector<float> h_randDistanceVec,const std::vector<char> h_randClutterVec)
{
#ifndef NDEBUG
	printf("%s:%d %s start.\n", __FILE__, __LINE__, __FUNCTION__);
	printf("%s:%d %s, h_data_size(%d)\n", __FILE__, __LINE__, __FUNCTION__, h_data_size);
#endif	

	if(h_data_size == 0)
	{
#ifndef NDEBUG
		printf("%s:%d %s warning.\n", __FILE__, __LINE__, __FUNCTION__);
#endif	
		return false;
	}

	struct timespec ts_start;
	clock_gettime(CLOCK_MONOTONIC, &ts_start);
#ifndef NDEBUG
	printf("%s:%d %s ts_start.tv_sec(%ld), ts_start.tv_nsec(%ld)\n", __FILE__, __LINE__, __FUNCTION__, ts_start.tv_sec, ts_start.tv_nsec);
#endif	

	// define device var
	float4 *d_closest_hits_data = nullptr;
	float4 *d_normal_data = nullptr;
	float4 *d_raw_data_phy = nullptr;
	float *d_rand_data = nullptr;
	char *d_rand_clutter_data = nullptr;
    float *d_rand_normal_data = nullptr;

	// computer raw data size
	const unsigned int nDataMemorySize = sizeof(float4) * h_data_size;

	// allocate device memory
	CHECK(cudaMalloc((void **)&d_closest_hits_data, nDataMemorySize));
	CHECK(cudaMalloc((void **)&d_normal_data, nDataMemorySize));
	CHECK(cudaMalloc((void **)&d_raw_data_phy, nDataMemorySize));
	CHECK(cudaMalloc((void **)&d_rand_data, sizeof(float) * h_data_size));
	CHECK(cudaMalloc((void **)&d_rand_clutter_data, sizeof(char) * h_data_size));
    CHECK(cudaMalloc((void **)&d_rand_normal_data, h_data_size * sizeof(float)));

	// copy host memory to device
	CHECK(cudaMemcpy(d_closest_hits_data, h_closest_hits_data.data(), nDataMemorySize, cudaMemcpyHostToDevice)); 
	CHECK(cudaMemcpy(d_normal_data, h_normal_data.data(), nDataMemorySize, cudaMemcpyHostToDevice)); 
	CHECK(cudaMemcpyToSymbol(g_d_lidarSimParams, lidarSimParams, sizeof(LidarSimParams)));
	CHECK(cudaMemcpy(d_rand_data, h_randDistanceVec.data(), (sizeof(float) * h_data_size),cudaMemcpyHostToDevice));
	CHECK(cudaMemcpy(d_rand_clutter_data, h_randClutterVec.data(), (sizeof(char) * h_data_size),cudaMemcpyHostToDevice));

	// setup execution parameters
    int dimx = 512;
    dim3 block(dimx, 1);
    dim3 grid((h_data_size + block.x - 1) / block.x, 1);

	// execute the kernel
    generateNormalDisRand<<<grid, block>>>(d_rand_normal_data, h_data_size);    //正太分布随机数初始化
    cuda_calculate_lidar_ray<<<grid, block>>>(d_closest_hits_data,d_normal_data, d_raw_data_phy, h_data_size,d_rand_data,d_rand_clutter_data, d_rand_normal_data);
	CHECK(cudaDeviceSynchronize());	

	// copy results from device to host
	h_raw_data_phy.resize(h_data_size);
	CHECK(cudaMemcpy(h_raw_data_phy.data(), d_raw_data_phy, nDataMemorySize, cudaMemcpyDeviceToHost));
	// release data
	CHECK(cudaFree(d_closest_hits_data));
	CHECK(cudaFree(d_normal_data));
	CHECK(cudaFree(d_raw_data_phy));
    CHECK(cudaFree(d_rand_normal_data));
	
	struct timespec ts_end;
	clock_gettime(CLOCK_MONOTONIC, &ts_end);
#ifndef NDEBUG
	printf("%s:%d %s ts_end.tv_sec(%ld), ts_end.tv_nsec(%ld), delta(%ld)\n", __FILE__, __LINE__, __FUNCTION__, ts_end.tv_sec, ts_end.tv_nsec, (ts_end.tv_sec - ts_start.tv_sec) * 1000 + (ts_end.tv_nsec - ts_start.tv_nsec) / 1000000);		
	printf("%s:%d %s end.\n", __FILE__, __LINE__, __FUNCTION__);
#endif	
	
	return true;
}

//kernel函数：计算数据 (模拟建模过程)
__global__ void cuda_calculate_lidar_ray(float4* d_closest_hits_data,float4* d_normal_data, float4* d_raw_data_phy, unsigned int size,float* d_rand_data,char* d_rand_clutter_data, float* d_rand_normal_data)
{
#ifndef NDEBUG
	//printf("%s:%d %s start.\n", __FILE__, __LINE__, __FUNCTION__);
#endif
	// current thread id
	unsigned int tid = threadIdx.x + blockIdx.x * blockDim.x;
    if (tid > size)
        return;

	//判断是否为杂波
	bool isClutter = d_rand_clutter_data[tid] == 1 ? true : false;
	//天气衰减距离
	float randDistance = d_rand_data[tid];

    //白噪声随机距离
    float randNormalDistance = d_rand_normal_data[tid];
    //printf("randNormalDistance: %f\n", randNormalDistance);

	// parallel handle each ray 
	// 由于最近撞击点是世界坐标系，将其转换成传感器坐标系
	float4 each_ray = d_closest_hits_data[tid];
	float4 each_ray_new = cuda_handle_eachRay(each_ray.x - g_d_lidarSimParams._fLidarPosition.x,
												each_ray.y - g_d_lidarSimParams._fLidarPosition.y, 
												each_ray.z - g_d_lidarSimParams._fLidarPosition.z,
												each_ray.w,d_normal_data[tid],tid,randDistance, isClutter, randNormalDistance);
#if 0 //输出数据需要传感器坐标系，此处代码保留; 之后如需渲染点云仍需要
	// through physical modeling, the new coordinates are obtained
	// 转回世界坐标系坐标
	d_raw_data_phy[tid] = make_float4(each_ray_new.x + g_d_lidarSimParams._fLidarPosition.x,
										each_ray_new.y + g_d_lidarSimParams._fLidarPosition.y,
										each_ray_new.z + g_d_lidarSimParams._fLidarPosition.z,
										each_ray_new.w);
#endif
    d_raw_data_phy[tid] = each_ray_new;

#ifndef NDEBUG
	//printf("%s:%d %s end.\n", __FILE__, __LINE__, __FUNCTION__);
#endif
}

// "Calls to curand_init() are slower than calls to curand() or curand_uniform(). It is much faster to save and restore random generator state than to recalculate the starting state repeatedly. 
// It may be beneficial to separate call to curand_init() and curand() into separate kernels for maximum performance."
__global__ void generateNormalDisRand(float *randNormal, int size)
{
    /* Each thread gets different seed, a different sequence
       number, no offset */
    curandState_t state;    // each thread needs its own state to garantee every random number is generated independently
    curand_init(clock64(), threadIdx.x, 0, &state);

    /* generate normal distributed random number */
    for (int i = blockIdx.x; i < size; i += blockDim.x)
    {
        randNormal[i] = curand_normal(&state);
    }

}

//计算每一跟射线
// 1、float4*data[i]（x,y,z,intensity）=》第i根射线
// 2、计算射线点的距离、方位角、高低角(笛卡尔坐标系=》极坐标系)
// 3、计算TOF
// 4、根据输入参数计算接收功率
// 5、计算接收信号
// 6、筛选接收信号峰值=》t
// 7、根据t-T/2计算出新距离
// 8、新距离+方位角、高低角 =》 更新坐标(极坐标系=》笛卡尔坐标系)
__device__ float4 cuda_handle_eachRay(float d_x, float d_y, float d_z, float w,float4 n,unsigned int index,float randDistance,bool isClutter, float randNormalDistance)
{
    // 1、 白噪声: 生成符合正太分布随机数
    //printf("random normal distance %f\n", randNormalDistance);
	// 2、计算射线点的距离、方位角、高低角(笛卡尔坐标系=》极坐标系)
	// R= √(x² + y²+ z²)
	double disR = sqrt(pow(d_x, 2) + pow(d_y, 2) + pow(d_z, 2)) + randNormalDistance / 50;
	// α = arcsin( y / √(x²+y²)) 
	// float alpha = asin(d_y / sqrt(pow(d_x, 2) + pow(d_y, 2)));
	// θ = arcsin( z / R )
	// float theta = asin(d_z / disR);

    // printf("normal distribution number: %f\n", normal_distri_num);
    // printf("disR: %f\n", disR);

	// 3、计算TOF
	// τ = 2R/c
	double tau = 2 * disR / LIGHTSPEED;
	double halfT = g_d_lidarSimParams._fT / 2;

	float3 dir = normalize(float3{d_x,d_y,d_z});
	float3 normal = float3{n.x,n.y,n.z};
	float cur = abs(dot(normal,-dir));

	//f(c,l,h) = c + (1-c) * ( 1 - l* h )^5 其中c为基准反射率，l为方向向量，h为法向量
	float fbrdf = w + (1 - w) * pow(1- cur,5.0f);

	// 4、根据输入参数计算接收功率  通过disR计算回波功率
	// PR = ( Pt *Gt / ( π * ( R * θB)² ))  *  BRDF参数 * (π / ( 4 * (f/D)² ) ) * A * ηatm * ηsys
	double Pr = g_d_lidarSimParams._fPt * g_d_lidarSimParams._fGt / (PI * pow(disR*g_d_lidarSimParams._fThetaB, 2))
				* fbrdf //* g_d_lidarSimParams._fBRDF 
				* (PI / (4 * pow(g_d_lidarSimParams._ff / g_d_lidarSimParams._fD, 2)))
				* g_d_lidarSimParams._fA *  g_d_lidarSimParams._fEtaatm * g_d_lidarSimParams._fEtasys;

#if 0 //离散模型
	// 5、计算接收信号
	// St = PR * max{ ( 1 - 2/T * | t-2R/c-T/2 | ) , 0}
	double maxSt = 0.0;
	double maxT = 0.0;
	double t1 = 1.0/(g_d_lidarSimParams._ffrequ);   //脉冲时延

    if(g_d_lidarSimParams._pulseWaveForm == 0 || g_d_lidarSimParams._pulseWaveForm == 2)//高斯、三角形
	{
		for (size_t j = 0; j < DISCRETEPOINT; j++)
		{
			double t = j * t1/DISCRETEPOINT;
			double sgma = g_d_lidarSimParams._fT/6.0;
			double a = pow(t - tau - 3 * sgma,2.0);
			double b = 2 * pow(sgma,2.0);
			double curSt = Pr * exp( -(a/b) );
            //功率受天气衰减
			if(isClutter)
			{
				double a1 = pow(t - randDistance - 3 * sgma,2.0);
				double st = Pr * exp( -(a1/b) );
                curSt = curSt + st;
			}

			// 6、筛选接收信号峰值=》t    更新迭代接受最大功率找到峰值以及对应时间
			if (curSt > maxSt)
			{
				maxT = t;
				maxSt = curSt;
			}
		}
	}	
	else if(g_d_lidarSimParams._pulseWaveForm == 1)//矩形
	{
		double lastSt = 0.0;
		double m = 0.0;
		for (size_t j = 0; j < DISCRETEPOINT; j++)
		{
			double t = j * t1/DISCRETEPOINT;
			double sgma = g_d_lidarSimParams._fT/6.0;
			double a = pow(t - tau - 3 * sgma,2.0);
			double b = 2 * pow(sgma,2.0);
			double curSt = Pr * exp( -(a/b) );

			if(isClutter)
			{
				double a1 = pow(t - randDistance - 3 * sgma,2.0);
				double st = Pr * exp( -(a1/b) );
				curSt = curSt + st;
			}
			
			// 6、筛选接收信号峰值=》t
			double ac = abs(curSt - lastSt);
			lastSt = curSt;
			if (ac > m)
			{
				maxT = t;
				m = ac;
				maxSt = curSt;
			}
		}
	}

	// 7、根据t-T/2计算出距离    T脉冲延时(脉冲宽度)
	// TOF' = t - T/2·
	// R' = TOF * c / 2

	double tauN = maxT - halfT;
	if(g_d_lidarSimParams._pulseWaveForm == 1)//矩形
	{
		tauN = maxT;
	}

	double disRN = tauN * LIGHTSPEED / 2;

	// 8、新距离+方位角、高低角 =》 更新坐标(极坐标系=》笛卡尔坐标系)
	// x' = R' * cosα * cosθ
	// y' = R' * sinα * cosθ
	// z' = R' * sinθ
	// 方案1：
	// float xN = disRN * cos(alpha) * cos(theta);
	// float yN = disRN * sin(alpha) * cos(theta);
	// float zN = disRN * sin(theta);

	// 方案2：
	float xN = disRN / disR * d_x;
	float yN = disRN / disR * d_y;
	float zN = disRN / disR * d_z;
	// if(isClutter)
	// 	return make_float4(0,0,0,0);
	return make_float4(xN, yN, zN, maxSt);
#endif

    if(g_d_lidarSimParams._pulseWaveForm == 0 || g_d_lidarSimParams._pulseWaveForm == 2)//高斯、三角形
	{

        if(isClutter)
        {
            //功率受天气衰减
            Pr = Pr * g_d_lidarSimParams._fclutterCoff; //TODO: _fclutterRatio => α*x/N
            double Prc = g_d_lidarSimParams._fPt * g_d_lidarSimParams._fGt / (PI * pow(randDistance *g_d_lidarSimParams._fThetaB, 2))
				* 0.1 //* fbrdf 雨雪雾的反射率 
				* (PI / (4 * pow(g_d_lidarSimParams._ff / g_d_lidarSimParams._fD, 2)))
				* g_d_lidarSimParams._fA *  g_d_lidarSimParams._fEtaatm * g_d_lidarSimParams._fEtasys;
            //对比衰减后功率和雨雪雾反射所产生功率大小
            if(Prc > Pr)
            {
                float xN = randDistance / disR * d_x;
                float yN = randDistance / disR * d_y;
                float zN = randDistance / disR * d_z;
                //return make_float4(xN, yN, zN, Prc);
                return make_float4(xN, yN, zN, 0.1);
            }
        }
        //return make_float4(d_x, d_y, d_z, Pr);
        return make_float4(d_x, d_y, d_z, w);
	}
}