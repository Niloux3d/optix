#include "LidarRayGenerator.h"
#include "RayGenerator.cuh"

#include <cuda_runtime.h>

#include <sutil/CUDAOutputBuffer.h>
#include <sutil/Exception.h>
#include <sutil/sutil.h>

#include <array>
#include <iomanip>
#include <iostream>
#include <string>

#include <sutil/Trackball.h>
#include <vector_types.h>
#include <sutil/vec_math.h>


//接收激光点云数据缓存
static sutil::CUDAOutputBuffer<float4> *_lidar_output_buffer = NULL;
static sutil::CUDAOutputBuffer<float4> *_lidar_output_normal_buffer = NULL;
static sutil::CUDAOutputBuffer<int> *_lidar_mesh_index_buffer = NULL;

static float radians(float degrees)
{
    return degrees * M_PIf / 180.0f;
}

LidarRayGenerator::LidarRayGenerator(LidarDescription ld)
    :m_lidarDescription(ld)
{

	try
	{
        _point_cloud_data = NULL;
        _point_cloud_data_size = 1024 * 1024;
        _point_cloud_data = malloc(_point_cloud_data_size);

        _lidar_output_buffer = new sutil::CUDAOutputBuffer<float4>( sutil::CUDAOutputBufferType::CUDA_DEVICE, int(ld.horizontalFov / ld.fPulseDivergenceAngle), ld.channels );
        _lidar_output_normal_buffer = new sutil::CUDAOutputBuffer<float4>( sutil::CUDAOutputBufferType::CUDA_DEVICE, int(ld.horizontalFov / ld.fPulseDivergenceAngle), ld.channels );
        _lidar_mesh_index_buffer = new sutil::CUDAOutputBuffer<int>( sutil::CUDAOutputBufferType::CUDA_DEVICE, int(ld.horizontalFov / ld.fPulseDivergenceAngle), ld.channels );

        _point_normal_data = NULL;
        _point_normal_data_size = 1024 * 1024;
        _point_normal_data = malloc(_point_normal_data_size);

        _mesh_reflection_idx = NULL;
        _mesh_reflection_idx_size = 1024 * 1024;
        _mesh_reflection_idx = malloc(_mesh_reflection_idx_size);
	}
	catch( std::exception& e )
	{
	}

}

LidarRayGenerator::~LidarRayGenerator()
{
	try
	{
		if(_lidar_output_buffer != NULL)
		{
			delete _lidar_output_buffer;
			_lidar_output_buffer = NULL;
		}

        if(_lidar_output_normal_buffer != NULL)
        {
            delete _lidar_output_normal_buffer;
			_lidar_output_normal_buffer = NULL;
        }

        if(_lidar_mesh_index_buffer != NULL)
        {
            delete _lidar_mesh_index_buffer;
            _lidar_mesh_index_buffer = NULL;
        }

        if (_point_cloud_data != NULL)
        {
            free(_point_cloud_data);
            _point_cloud_data = NULL;
            _point_cloud_data_size = 0;
        }

        if (_point_normal_data != NULL)
        {
            free(_point_normal_data);
            _point_normal_data = NULL;
            _point_normal_data_size = 0;
        }

        if (_mesh_reflection_idx != NULL)
        {
            free(_mesh_reflection_idx);
            _mesh_reflection_idx = NULL;
            _mesh_reflection_idx_size = 0;
        }
    }
	catch( std::exception& e )
	{
	}

}

bool LidarRayGenerator::makePointCloud(void** data, size_t* size,void** normal_data,size_t* normal_size)
{
	if((_lidar_output_normal_buffer == NULL || _lidar_output_buffer == NULL) || _lidar_mesh_index_buffer == NULL || (_pipeline == nullptr))
	{
		return false;
	}

    try
    {
        //width = Lidar水平方向的光的数量。
        //height = Lidar垂直方向的光的数量。
        int width = int(m_lidarDescription.horizontalFov / m_lidarDescription.fPulseDivergenceAngle);
        int height = m_lidarDescription.channels;
		//
        // launch
        //
        {
            Params params;
            params.origin                   = m_f3Origin;
            params.type                     = SENSOR_TYPE_LIDAR;
            params.hFov                     = radians(m_lidarDescription.horizontalFov) * this->rate;
            params.pulse_divergence_angle    = radians(m_lidarDescription.fPulseDivergenceAngle);
#if 0 // [一汽需求] 包络线 修改激光点云vFov 2022.08.17 LS
            params.upper_fov                = radians(_description.upperFov);
            params.lower_fov                = radians(_description.lowerFov);
            params.step_fov                 = radians(_description.upperFov - _description.lowerFov) / (_description.channels - 1);
#else // [一汽需求] 包络线 修改激光点云vFov 2022.08.17 LS
            params.upper_fov                = radians(m_lidarDescription.verticalFov) / 2;
            params.lower_fov                = -radians(m_lidarDescription.verticalFov) / 2;
            params.step_fov                 = radians(m_lidarDescription.verticalFov) / (m_lidarDescription.channels - 1);
#endif
            params.minDetectRange           = m_lidarDescription.minimumDetectRange;
			params.maxDetectRange           = m_lidarDescription.range;
#if 1 // [一汽需求] 包络线 增加激光点云hpr 2022.08.17 LS
            params.yaw                      = radians(m_lidarDescription.heading) + m_fEgoH;
            params.pitch                    = radians(m_lidarDescription.pitch) + m_fEgoP;
            params.roll                     = radians(m_lidarDescription.roll) + m_fEgoR;
#endif
            params.attenuation              = m_lidarDescription.atmosphereAttenuationRate;
            params.cloud_data               = _lidar_output_buffer->map();
            params.normal_data               = _lidar_output_normal_buffer->map();
            params.mesh_index_data          = _lidar_mesh_index_buffer->map();
            params.width                    = (unsigned int)(width * this->rate);
            params.height                   = height;
            params.handle                   = _gas_handle;

			{
                runOptix(params, params.width, height, 1);
            }

            _lidar_output_buffer->unmap();
            _lidar_output_normal_buffer->unmap();
            _lidar_mesh_index_buffer->unmap();
        }

        //
        // get cloud data & size.
        //
        *size          = width * height * sizeof(float4);
		if(_point_cloud_data_size < *size)
		{
			if(_point_cloud_data != NULL)
			{
				free(_point_cloud_data);
			}

			_point_cloud_data_size = *size;
			_point_cloud_data = malloc(_point_cloud_data_size);
		}

		memcpy(_point_cloud_data, _lidar_output_buffer->getHostPointer(), *size);
		*data = _point_cloud_data;

        *normal_size = width * height * sizeof(float4);
		if(_point_normal_data_size < *normal_size)
		{
			if(_point_normal_data != NULL)
			{
				free(_point_normal_data);
			}

			_point_normal_data_size = *normal_size;
			_point_normal_data = malloc(_point_normal_data_size);
		}

		memcpy(_point_normal_data, _lidar_output_normal_buffer->getHostPointer(), *normal_size);
		*normal_data = _point_normal_data;

        if(_mesh_reflection_idx_size != width * height * sizeof(int))
        {
            if(_mesh_reflection_idx != NULL)
            {
                free(_mesh_reflection_idx);
            }
            _mesh_reflection_idx_size = width * height * sizeof(int);
            _mesh_reflection_idx = malloc(_mesh_reflection_idx_size);
        }
        memcpy(_mesh_reflection_idx, _lidar_mesh_index_buffer->getHostPointer(), _mesh_reflection_idx_size);

    }
    catch( std::exception& e )
    {
        return false;
    }

    return true;
}

void *LidarRayGenerator::getMeshIndex()
{
    return _mesh_reflection_idx;
}