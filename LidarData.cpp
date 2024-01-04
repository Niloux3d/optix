#include "LidarData.h"


int LidarData::_lidar_frame = 0;
std::mutex LidarData::dataMtx;

LidarData::LidarData()
{

	_type = SENSOR_TYPE_LIDAR;
    _frame = LidarData::_lidar_frame++;
	
}
    
void LidarData::addRawData(const float4& lrd)
{
#ifndef NDEBUG
    //printf("%s:%d %s start.\n"));
#endif
    std::lock_guard<std::mutex> lock(dataMtx);
    _raw_data.push_back(lrd);
#ifndef NDEBUG
    //printf("%s:%d %s end.\n"));
#endif	
}

vector<float4>& LidarData::getRawData()
{
    return _raw_data;
}

float4 LidarData::getRawData(int idx)
{
    return _raw_data[idx];
}

void LidarData::addPhyRawData(const float4 &lrd)
{
	_raw_data_phy.push_back(lrd);
}

vector<float4> &LidarData::getPhyRawData()
{
	return _raw_data_phy;
}

size_t LidarData::get_serialize_size()
{
	return sizeof(LidarData) 
		- sizeof(vector<float4>) + sizeof(size_t) + _raw_data.size() * sizeof(float4);	
	
}

size_t LidarData::serialize(void* buffer, size_t size)
{
	if(buffer == NULL)
	{
		return 0;
	}
	
	//数据长度检查
	if(get_serialize_size() != size)
	{
		return 0;
	}
	
	//序列化基类
	char* write_pos = (char*)buffer;
	size_t write_size = SensorData::serialize(write_pos, size);
	
	//序列化容器类的size
	write_pos += write_size;
	write_size = sizeof(size_t);
	*((size_t*)write_pos) = _raw_data.size();
	
	//序列化容器类的数据
	write_pos += write_size;
	write_size = _raw_data.size() * sizeof(float4);
	memcpy(write_pos, _raw_data.data(), write_size);
	
	write_pos += write_size;

	
	return (write_pos - (char*)buffer);
	
}

size_t LidarData::deserialize(void* buffer, size_t size)
{
	if(buffer == NULL)
	{
		return 0;
	}
	
	//最小数据长度检查
	if(sizeof(LidarData) - sizeof(vector<float4>) + sizeof(size_t) > size)
	{
		return 0;
	}
	
	//反序列化基类
	char* read_pos = (char*)buffer;
	size_t read_size = SensorData::deserialize(read_pos, size);
	
	//反序列化容器类的size
	read_pos += read_size;
	read_size = sizeof(size_t);
	size_t raw_data_size = *((size_t*)read_pos);
	
	//反序列化容器类的数据
	read_pos += read_size;
	if((size - (read_pos - (char*)buffer)) >= raw_data_size * sizeof(float4))
	{
		_raw_data.resize(raw_data_size);
		read_size = raw_data_size * sizeof(float4);
		memcpy(_raw_data.data(), read_pos, read_size);
		
		read_pos += read_size;
		return (read_pos - (char*)buffer);
	}
	//数据长度不一致
	else
	{
		return 0;
	}
	
}

string LidarData::to_json()
{

	stringstream sstream;

	sstream << "{";
	sstream << SensorData::to_json();
	sstream << "\"dummy\":0";
	sstream << "}";


	return sstream.str();
}


SensorDescription* LidarData::get_description()
{
	return &_description;
}

void LidarData::set_description(SensorDescription* description)
{
	memcpy(&_description, description, sizeof(LidarDescription));
}

void LidarData::addNormalRawData(const float4& lrd)
{
	_normal_raw_data.push_back(lrd);
}

vector<float4>& LidarData::getNormalRawData()
{
	return _normal_raw_data;
}