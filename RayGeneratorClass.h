#ifndef RayGenerator_h
#define RayGenerator_h

#include <vector>
#include <sutil/sutil.h>
#include "RayGenerator.cuh"
#include "RadarRayGenerator.cuh"

#include "SensorDescription.h"

using namespace std;

enum MATERIAL_TYPE
{
    MATERIAL_TYPE_UNKNOWN = 0,
    MATERIAL_TYPE_METAL,
    MATERIAL_TYPE_CEMENT,
    MATERIAL_TYPE_ASPHALT,
    MATERIAL_TYPE_GLASS,
    MATERIAL_TYPE_FABRIC,
    MATERIAL_TYPE_SKIN,
    MATERIAL_TYPE_PLANT,
    MATERIAL_TYPE_COUNT,
};

typedef struct
{
    vector<float3> points;      //三个点构成一个三角形
    vector<uint32_t> materials; //一个三角形有一个材质
}ScenarioData;

class RayGenerator
{
public:
	RayGenerator();
    ~RayGenerator();

    bool Initialize(SENSOR_TYPE);
    bool Uninitialize();

	// 设置原点
    void setOrigin(const float3& origin);
	// 设置雷达所安装车辆的姿态角
    void setEgoH(const float& h);
    void setEgoP(const float& p);
    void setEgoR(const float& r);

	// 设置场景数据
    virtual void setScenarioData(ScenarioData& scenarioData);
	// 生产点云数据
    virtual bool makePointCloud(void** data, size_t* size) = 0;

	// 读取PTX文件内容
	std::string readPTXFile(const char* filename);

protected:
	void cuda_malloc(CUdeviceptr* buf, size_t* size, size_t new_size);
	// 启动OPTIX Pipeline
	void runOptix(Params params,
				  unsigned int width,
				  unsigned int height,
				  unsigned int depth);
	void runOptixRadar(RadarParams params,
                    unsigned int width,
                    unsigned int height,
                    unsigned int depth);

protected:
	float3 m_f3Origin;
	float m_fEgoH;
	float m_fEgoP;
	float m_fEgoR;
	
	OptixDeviceContext	_context;
	OptixModule	_module;
	OptixProgramGroup	_raygen_prog_group;
	OptixProgramGroup	_miss_prog_group;
	OptixProgramGroup	_hitgroup_prog_group;
	OptixPipeline	_pipeline;

	CUdeviceptr	_param;
	CUstream _stream;

	size_t _gas_output_buffer_size;
	CUdeviceptr	_gas_output_buffer;

	size_t _temp_buffer_gas_size;
	CUdeviceptr	_temp_buffer_gas;

	size_t _vertices_size;
	CUdeviceptr	_vertices;

	size_t _material_indices_size;
	CUdeviceptr	_material_indices;

	OptixShaderBindingTable _sbt;
	OptixTraversableHandle _gas_handle;

	char log[2048]; // For error reporting from OptiX creation functions

	void* _radar_ray_data;
	size_t _radar_ray_data_size;

	void* _radar_mesh_data;
	size_t _radar_mesh_data_size;

};

#endif
