#ifndef LidarData_h
#define LidarData_h

#include "SensorData.h"
#include <sutil/vec_math.h>
#include <sutil/sutil.h>
#include "LidarDescription.h"

#include <mutex>
#include<vector>
#include <sstream>

using namespace std;

struct LidarRawData
{
    LidarRawData(float3 c, float i)
        : coordination(c), intensity(i)
    {

    }

    float3 coordination;
    float intensity;

};

class LidarData : public SensorData
{
public:
    LidarData();
    
	virtual SensorDescription* get_description();
	virtual void set_description(SensorDescription* description);

    void addRawData(const float4& lrd);
    vector<float4>& getRawData();
    float4 getRawData(int idx);

    void addPhyRawData(const float4& lrd);
    vector<float4>& getPhyRawData();
    
	size_t get_serialize_size();
    size_t serialize(void* buffer, size_t size);
	size_t deserialize(void* buffer, size_t size);
	string to_json();
	
    void addNormalRawData(const float4& lrd);
    vector<float4>& getNormalRawData();
	static std::mutex dataMtx;

private:
	static int _lidar_frame;
	
	//重要：如果追加vector等容器类，或者string成员变量，应放在这之后。
	//并且要修改serialize, deserialize的实现。
    vector<float4> _raw_data;       // 保存光线追踪后最近撞击点的点云
	vector<float4> _raw_data_phy;   // 保存物理建模后的点云
    vector<float4> _normal_raw_data;    
    
	LidarDescription _description;
};

#endif