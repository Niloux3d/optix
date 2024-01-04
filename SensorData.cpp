#include "SensorData.h"

#include <stdio.h>
#include <time.h>


#include <sstream>

SensorData::SensorData()
{

    //printf("%s:%d %s start.\n", __FILE__, __LINE__, __FUNCTION__);
	

	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
    _timestamp = ts.tv_sec * 1000 + ts.tv_nsec / (1000 * 1000);
	
    //printf("%s:%d %s end.\n", __FILE__, __LINE__, __FUNCTION__);	

}

SensorData::~SensorData()
{
}

int SensorData::get_type()
{
	return _type;
}

int SensorData::get_frame()
{
	return _frame;
}

unsigned long SensorData::get_timestamp()
{
	return _timestamp;
}

size_t SensorData::get_serialize_size()
{
	return sizeof(_type) + sizeof(_frame) + sizeof(_timestamp);
}

//序列化
//注意：vector等容器类型或者string，序列化包含数据的大小(size_t)和数据
size_t SensorData::serialize(void* buffer, size_t size)
{
	if((buffer == NULL) || (size < get_serialize_size()))
	{
		return 0;
	}
	
	char* write_pos = (char*)buffer;
	size_t write_size = sizeof(_type);
	memcpy(write_pos, &_type, write_size);
	write_pos += write_size;

	write_size = sizeof(_frame);
	memcpy(write_pos, &_frame, write_size);
	write_pos += write_size;

	write_size = sizeof(_timestamp);
	memcpy(write_pos, &_timestamp, write_size);
	write_pos += write_size;
	
	return (write_pos - (char*)buffer);
}

//反序列化
//注意：vector等容器类型或者string，反序列化要包含数据的大小(size_t)和数据
size_t SensorData::deserialize(void* buffer, size_t size)
{
	if((buffer == NULL) || (size < get_serialize_size()))
	{
		return 0;
	}
	
	char* read_pos = (char*)buffer;
	
	size_t read_size = sizeof(_type);
	memcpy(&_type, read_pos, read_size);
	read_pos += read_size;

	read_size = sizeof(_frame);
	memcpy(&_frame, read_pos, read_size);
	read_pos += read_size;

	read_size = sizeof(_timestamp);
	memcpy(&_timestamp, read_pos, read_size);
	read_pos += read_size;
	
	return (read_pos - (char*)buffer);
	
}
	
string SensorData::to_json()
{
	stringstream sstream;

	sstream << "\"sensor_type\":" << _type << ",";
	sstream << "\"frame\":" << _frame << ",";
	sstream << "\"timestamp\":" << _timestamp << ",";

	return sstream.str();
}

string SensorData::to_osi()
{
	return "";
}

vector<SensorDectectedObject>& SensorData::get_detected_objects()
{
	return _detected_objects;
}
