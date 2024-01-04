#ifndef Lidar_h
#define Lidar_h
#define ENABLE_GPU

#include "LidarData.h"
#include "LidarDescription.h"

#ifdef ENABLE_GPU
#include "LidarRayGenerator.h"
#include "../cuda/CalculateLidarRay.cuh"
#endif


using namespace std;
#define LIDAR_DEBUG 0

class Lidar
{
public:

    Lidar(LidarDescription ld);
    ~Lidar();

	SensorDescription* get_description();
    bool equal_description(SensorDescription*);

	//传感器数据生成前的准备，由渲染线程执行。
	// virtual SensorData* preMakeData();
	void preMakeData();
	void postMakeData();
    SensorData* makeData();
	bool makeDataClosestHits();
	LidarSimParams initializePhyParams();

	void printData(LidarData *lidarData);
	float3 getLidarPosition();
    float getLidarClutterRatio();
	//生成默认的空的点云数据。用于结束时，清空既存的渲染点云。
	virtual SensorData* makeDefaultData();

    virtual double getIntervalOfFrame();
    
    //accquire type
    virtual int getType();
private:
	static float radians(float degrees);

	//雷达走一帧
	void rollOneFrame();

	//获取雷达所在位置，一圈的百分比
	float getRoundRate();
private:
    LidarDescription _lidarDescription;
#ifdef ENABLE_GPU
    LidarRayGenerator _lidarRayGenerator;
#endif
	pthread_mutex_t _mutex;
	LidarData* _lidarData;
	vector<float4> vecData;
	vector<float4> vecDataBackup;
	vector<float4> vecDataPerFrame;
	vector<float4> vecNormalDataPerFrame;

	vector<float4> vecNormalData;
	vector<float4> vecNormalDataBackup;

	vector<float4> phy_vecData;
	vector<float4> phy_vecDataBackup;

	//激光雷达扫描一圈控制
	int _curFrame; //Current frame num by lidar counting
	int _maxFrame;

	vector<float4> _vecFrameData;
	vector<float4> _vecAllData;

	vector<float4> _vecNormalData;
	vector<float4> _vecNormalAllData;

    float clutterRatio;
    int bufferAllocatedSize = 0;
    int totalBufferSize = 0;

#if LIDAR_DEBUG
	vector<float4> _vecFrameRawData;
	vector<float4> _vecAllRawData;
#endif

private:
    bool _enable;

};

#endif