#ifndef LidarRayGenerator_h
#define LidarRayGenerator_h

#include "LidarDescription.h"

#include <vector>
#include "RayGeneratorClass.h"

using namespace std;

class LidarRayGenerator : public RayGenerator
{
public:
    LidarRayGenerator(LidarDescription ld);
    ~LidarRayGenerator();

    virtual bool makePointCloud(void** data, size_t* size,void** normal_data,size_t* normal_size);
    virtual bool makePointCloud(void** data, size_t* size){};
    void* getMeshIndex();
    void setRate(float rate){ this->rate = rate; };

private:    
	LidarDescription m_lidarDescription;

	void* _point_cloud_data;
	size_t _point_cloud_data_size;
    void* _point_normal_data;
    size_t _point_normal_data_size;
    float rate;
    void* _mesh_reflection_idx;
    size_t _mesh_reflection_idx_size;

};

#endif
