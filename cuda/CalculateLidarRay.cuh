#ifndef _CALCULATE_LIDAR_RAY_H_
#define _CALCULATE_LIDAR_RAY_H_

#include <vector>

// simulation parameters
struct LidarSimParams {
    // 发射峰值功率（Pt）、
    float _fPt;
    // 发射增益（Gt）、
    float _fGt;
    // 光束发散角（θB）、
    float _fThetaB;
    // 接收孔径大小（A）、
    float _fA;
    // 大气消光系数（ηatm）、
    float _fEtaatm;
    // 光学系统消光系数（ηsys）、
    float _fEtasys;
    // 光阑大小（D）、
    float _fD;
    // 接收系统光学焦距（f）、
    float _ff;
    // BRDF参数（暂定，二期开发此模块）
    float _fBRDF;
    // 波长（λ）
    float _fLambda;
    // 脉冲时延（T）
    float _fT;
    // 脉冲频率（f）
    float _ffrequ;
    // 传感器安装位置
    float3 _fLidarPosition;
    //脉冲波形
    int _pulseWaveForm;
    //天气模型
    //脉冲波形 1.0
    int _minDistance;
    //杂波比例 0-1.0
    float _fclutterRatio;
    //杂波衰减系数 0-1.0
    float _fclutterCoff;
  };

//__host__ ============================================
// 供外部调用
// 计算点云数据
__host__ bool calculate_lidar_ray(const std::vector<float4> h_closest_hits_data,const std::vector<float4> h_normal_data, std::vector<float4>& h_raw_data_pyh, int h_raw_data_size, LidarSimParams* lidarSimParams,const std::vector<float> h_randDistanceVec,const std::vector<char> h_randClutterVec);

//__global__ ============================================
// kernel
__global__ void cuda_calculate_lidar_ray(float4* d_closest_hits_data,float4* d_normal_data, float4* d_raw_data_phy, unsigned int size,float* d_rand_data,char* d_rand_clutter_data, float* d_rand_normal_data);

// kernel 生成正太分布随机数
__global__ void generateNormalDisRand(float *randNormal, int size);

//__device__ ============================================
// 处理每一个射线
__device__ float4 cuda_handle_eachRay(float d_x, float d_y, float d_z,float w,float4 n,unsigned int index,float randDistance,bool isClutter, float randNormalDistance);

#endif //_CALCULATE_LIDAR_RAY_H_
