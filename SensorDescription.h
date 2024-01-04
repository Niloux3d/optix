#ifndef SensorDescription_h
#define SensorDescription_h

#include <string>

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

enum SENSOR_TYPE
{
	SENSOR_TYPE_UNKNOWN,	
	SENSOR_TYPE_LIDAR,
	SENSOR_TYPE_CAMERA,
	SENSOR_TYPE_IMU,
	SENSOR_TYPE_BASIC_SENSOR,
	SENSOR_TYPE_GPS,
#if 1 // 目标级传感器 2021.12.14 LS
	SENSOR_TYPE_RADAR,
	SENSOR_TYPE_ULTRASONIC,
#endif
#if 1 //liangxu 2023.04.17 [一汽V2X]
	SENSOR_TYPE_V2X_OBU,
	SENSOR_TYPE_V2X_RSU
#else
#if 1 //liangxu V2X传感器 2022.03.10
	SENSOR_TYPE_V2X,
#endif
#endif
};

#if 1	/*2022.10.24 Tang 理想传感器拆分 [start]*/
enum SENSOR_MODEL_TYPE
{
	SENSOR_MODEL_IDEAL,
	SENSOR_MODEL_PROBABLY,
	SENSOR_MODEL_PHYSIC
};
#endif	/*2022.10.24 Tang 理想传感器拆分 [end]*/

struct SensorDescription
{
    int type;
	int sensorOutputType; // 包络线[Add]传感器输出类型 2022.08.22 LS
    char name[128];
	bool enable;
	char dummy[3];
#if 1	/* Tang 2022.10.25 一汽一阶段，传感器拆分[start] */
	float assemblePositionX;
	float assemblePositionY;
	float assemblePositionZ;
	float heading;
	float pitch;
	float roll;
	SENSOR_MODEL_TYPE modelType;	// 传感器模型类型，初始化为IDEAL
	int object_detection_type;	//物体识别类别
#endif	/* Tang 2022.10.25 一汽一阶段，传感器拆分[end] */

#if 1 //hlx 2022.05.16 bug1312
public:
	virtual int size()
	{
		return sizeof(SensorDescription);
	};

	virtual int getFrameRate()
	{
		return 0;
	};
#endif
};



#endif