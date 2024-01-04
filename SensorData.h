#ifndef SensorData_h
#define SensorData_h

#include <string.h>
#include <string>
#include <vector>

#include "SensorDescription.h"

using namespace std;

class Sensor;
#if 1
//车辆
#define SENSOR_OBJECT_DETECTION_TYPE_VEHICLE (0x00000001)

//行人
#define SENSOR_OBJECT_DETECTION_TYPE_PEDESTRIAN (0x00000002)

//障碍物
#define SENSOR_OBJECT_DETECTION_TYPE_OBSTACLE (0x00000004)

//交通标识
#define SENSOR_OBJECT_DETECTION_TYPE_TRAFICSIGN (0x00000008)

//道路信息
#define SENSOR_OBJECT_DETECTION_TYPE_LANEINFO (0x00000010)

//道路标识
#define SENSOR_OBJECT_DETECTION_TYPE_ROADMARK (0x00000020)

//交通灯
#define SENSOR_OBJECT_DETECTION_TYPE_TRFICLIGHT (0x00000040)

//所有
#define SENSOR_OBJECT_DETECTION_TYPE_ALL (0xffffffff)
#endif
#if 0
enum SENSOR_TYPE
{
	SENSOR_TYPE_UNKNOWN,	
	SENSOR_TYPE_LIDAR,
	SENSOR_TYPE_CAMERA,
	SENSOR_TYPE_IMU,
	SENSOR_TYPE_BASIC_SENSOR,
	SENSOR_TYPE_GPS,
#if 1  // [一汽需求] 包络线 2022.08.08 LS
	SENSOR_TYPE_RADAR,
	SENSOR_TYPE_ULTRASONIC,
#endif
};
#endif

enum SENSOR_DETECTION_TYPE
{
	SENSOR_DETECTION_TYPE_UNKNOWN,
	SENSOR_DETECTION_TYPE_VEHICLE,
	SENSOR_DETECTION_TYPE_PEDESTRIAN,
	SENSOR_DETECTION_TYPE_ROAD_OBJECT,
};

typedef struct
{
	SENSOR_DETECTION_TYPE type;
	float center[3];	//中心点坐标(x, y, z)，Camera坐标系
	float length;		//长
	float width;		//宽
	float height;		//高
	float h;			//航向角
	float p;			//俯仰角
	float r;			//横滚角
	float alpha;		//物体的观察角度，范围：-pi~pi
	float bbox_left;	//TBD.物体的2维边界框（像素坐标系，单位：像素）
	float bbox_top;
	float bbox_right;
	float bbox_bottom;
	float rotation_y;	//3维物体的空间方向：rotation_y
} SensorDectectedObject;

class SensorData
{
public:
	SensorData();
	
	virtual ~SensorData();

	int get_type();

	int get_frame();

	virtual SensorDescription* get_description() = 0;
	virtual void set_description(SensorDescription* description) = 0;

	unsigned long get_timestamp();

	//返回序列化的数据大小。（不包含目标识别物体）
	//注意：vector等容器类型，包含数据的大小(size_t)和数据
	virtual size_t get_serialize_size();
	
	//序列化（不包含目标识别物体）
	//注意：vector等容器类型或者string，序列化包含数据的大小(size_t)和数据
    virtual size_t serialize(void* buffer, size_t size);

	//反序列化（不包含目标识别物体）
	//注意：vector等容器类型或者string，反序列化要包含数据的大小(size_t)和数据
	virtual size_t deserialize(void* buffer, size_t size);

	//转换成json格式（不包含目标识别物体）
	virtual string to_json();

	//转换成osi格式（不包含目标识别物体）
	virtual string to_osi();
	
	//目标识别物体
	vector<SensorDectectedObject>& get_detected_objects();

protected:
	int _type;
    int _frame;
    unsigned long _timestamp;
	vector<SensorDectectedObject> _detected_objects;
	
};

#endif