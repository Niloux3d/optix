#include "Lidar.h"
#include "LidarData.h"
#include "LidarRayGenerator.h"

#include <random>
#include <cmath>
#include <sutil/vec_math.h>
#include <sutil/sutil.h>
#include <memory>

#include <algorithm>


using std::default_random_engine;
using std::uniform_real_distribution;
const float PI = 3.1415926;

const std::array<float, MATERIAL_TYPE_COUNT> material_reflections =
{ {
    0.5,    //unknown
    0.9,	//metal
    0.57,	//cement
    0.17,	//asphalt
    0.05,	//glass
    0.5,	//fabric
    0.7,	//skin
    0.6,	//plant
} };

Lidar::Lidar(LidarDescription ld)
    : _lidarDescription(ld)
#ifdef ENABLE_GPU
	, _lidarRayGenerator(ld)
#endif

{
	_enable = ld.enable;
#ifdef ENABLE_GPU
    _lidarRayGenerator.Initialize(SENSOR_TYPE_LIDAR);
	_lidarRayGenerator.setRate(getIntervalOfFrame());
	
	_maxFrame = (int)_lidarDescription.fSpeed;//TODO: 改倍数
	_curFrame = _maxFrame - 1;
	vecData.reserve(10000);
	vecDataBackup.reserve(10000);
	vecDataPerFrame.reserve(10000);
	vecNormalDataPerFrame.reserve(10000);
	phy_vecData.reserve(10000);
	phy_vecDataBackup.reserve(10000);

	vecNormalData.reserve(10000);
	vecNormalDataBackup.reserve(10000);

#endif
	if(pthread_mutex_init( &_mutex, NULL) != 0)
	{
	}


    int projectId = 'L';
    int bufferNum = 1;
    totalBufferSize = 8 * 1024 * 1024;

}

LidarSimParams Lidar::initializePhyParams()
{
	LidarSimParams lidarSimParams;
	// 发射峰值功率（Pt）、
	//lidarSimParams._fPt = 2.5;
	lidarSimParams._fPt = _lidarDescription.fPeakPower;
	// 发射增益（Gt）、
	//lidarSimParams._fGt = 1.0;
	lidarSimParams._fGt = _lidarDescription.fGainOfEmission;
	// 光束发散角（θB）、
	//lidarSimParams._fThetaB = 0.001;
	lidarSimParams._fThetaB = radians(_lidarDescription.fAngleOfBeamDivergence);
	// 接收孔径大小（A）、
	lidarSimParams._fA = 1.0;
	// 大气消光系数（ηatm）、
	//lidarSimParams._fEtaatm = 1.0;
	lidarSimParams._fEtaatm = _lidarDescription.fAtmosphericExtinctionCoefficient;
	// 光学系统消光系数（ηsys）、
	//lidarSimParams._fEtasys = 1.0;
	lidarSimParams._fEtasys = _lidarDescription.fOpticalSystemExtinctionCoefficient;
	// 光阑大小（D）、
	//lidarSimParams._fD = 1.0;
	lidarSimParams._fD = _lidarDescription.fSizeOfAperture;
	// 接收系统光学焦距 （f）、
	lidarSimParams._ff = 1.0;
	// BRDF参数（暂定，二期开发此模块）
	lidarSimParams._fBRDF = 1.0;
	// 波长（λ）
	//lidarSimParams._fLambda = 1.0;
	lidarSimParams._fLambda = _lidarDescription.fWaveLength * 1e-9;
	// 脉冲时延（T）
	//lidarSimParams._fT = 2e-8;
	lidarSimParams._fT = _lidarDescription.fDelayOfPulse * 1e-9;
	// 脉冲频率（f）
	//lidarSimParams._ffrequ = 5e7;
	lidarSimParams._ffrequ = _lidarDescription.fFrequency  * 1e6;
	//脉冲波形
	lidarSimParams._pulseWaveForm = _lidarDescription.iPulseWaveForm;


	// LidarSimParams lidarSimParams;
	// // 发射峰值功率（Pt）、
	// lidarSimParams._fPt = 2.5;
	// //lidarSimParams._fPt = _lidarDescription.fPeakPower;
	// // 发射增益（Gt）、
	// lidarSimParams._fGt = 1.0;
	// //lidarSimParams._fGt = _lidarDescription.fGainOfEmission;
	// // 光束发散角（θB）、
	// lidarSimParams._fThetaB = 0.001;
	// //lidarSimParams._fThetaB = radians(_lidarDescription.fAngleOfBeamDivergence);
	// // 接收孔径大小（A）、
	// lidarSimParams._fA = 1.0;
	// // 大气消光系数（ηatm）、
	// lidarSimParams._fEtaatm = 1.0;
	// //lidarSimParams._fEtaatm = _lidarDescription.fAtmosphericExtinctionCoefficient;
	// // 光学系统消光系数（ηsys）、
	// lidarSimParams._fEtasys = 1.0;
	// //lidarSimParams._fEtasys = _lidarDescription.fOpticalSystemExtinctionCoefficient;
	// // 光阑大小（D）、
	// lidarSimParams._fD = 1.0;
	// //lidarSimParams._fD = _lidarDescription.fSizeOfAperture;
	// // 接收系统光学焦距 （f）、
	// lidarSimParams._ff = 1.0;
	// // BRDF参数（暂定，二期开发此模块）
	// lidarSimParams._fBRDF = 1.0;
	// // 波长（λ）
	// lidarSimParams._fLambda = 1.0;
	// //lidarSimParams._fLambda = _lidarDescription.fWaveLength * 1e-9;
	// // 脉冲时延（T）
	// lidarSimParams._fT = 2e-8;
	// //lidarSimParams._fT = _lidarDescription.fDelayOfPulse * 1e-9;
	// // 脉冲频率（f）
	// lidarSimParams._ffrequ = 5e5;
	// //lidarSimParams._ffrequ = _lidarDescription.fFrequency  * 1e6;
	// //脉冲波形
	// //lidarSimParams._pulseWaveForm = 2;
	// lidarSimParams._pulseWaveForm = _lidarDescription.iPulseWaveForm;

	lidarSimParams._fLidarPosition = getLidarPosition();
	//杂波比例
	//lidarSimParams._fclutterRatio = 0.3;
    lidarSimParams._fclutterRatio = getLidarClutterRatio();

	//天气模型最小距离
	lidarSimParams._minDistance = 1.0;
	//杂波衰减系数
	//lidarSimParams._fclutterCoff = 0.1;
    lidarSimParams._fclutterCoff = 0.1; //TOOD: 天气程度最大的杂波 对应天气种类的衰减系数

	return lidarSimParams;
}

Lidar::~Lidar()
{

	pthread_mutex_destroy(&_mutex);
#ifdef ENABLE_GPU
    _lidarRayGenerator.Uninitialize();
#endif

}

// 激光雷达传感器生产数据
SensorData* Lidar::makeData()
{
    rollOneFrame();
	// 初始化CUDA全局资源(主要是物理建模用模型参数信息)
	LidarSimParams lidarSimParams = initializePhyParams();

	// 利用光线追踪计算最近撞击点的坐标信息
	if (makeDataClosestHits() == false)
	{
		return NULL;
	}

	// 利用CUDA并行计算，物理建模后各撞击点的新坐标信息
	vector<float4> h_raw_data_phy;  //世界坐标系
	vector<float4> h_raw_data;
	vector<float4> h_normal_raw_data; //法向量
	h_raw_data.reserve(vecDataPerFrame.size() + 10);
	h_normal_raw_data.reserve(vecNormalDataPerFrame.size() + 10);
	for(size_t i = 0;i<vecDataPerFrame.size();i++)
	{
		h_raw_data.push_back(float4{vecDataPerFrame[i].x,vecDataPerFrame[i].y,vecDataPerFrame[i].z,vecDataPerFrame[i].w});
		h_normal_raw_data.push_back(float4{vecNormalDataPerFrame[i].x,vecNormalDataPerFrame[i].y,vecNormalDataPerFrame[i].z,vecNormalDataPerFrame[i].w});
	}

	int nRawDataSize = h_raw_data.size();
	//杂波光线概率模型处理
	std::vector<float> randDistanceVec;
	std::vector<char> randClutterVec;
	randDistanceVec.resize(nRawDataSize,0.0);
	randClutterVec.resize(nRawDataSize,0);
	std::vector<int> vec;
	vec.resize(nRawDataSize);
	for(int i = 0;i<nRawDataSize;i++)
	{
		vec[i] = i;
	}

	random_shuffle(vec.begin(),vec.end());
	int clutterNum = lidarSimParams._fclutterRatio * nRawDataSize * 0.002; //此处系数为依据一汽需求控制噪点数量
    //D_PRINTF(D_INFO, "getClutterRatio:%f",_clutterMapPtr->at("rain") );
	std::default_random_engine generator;
	std::poisson_distribution<int> distribution(80.1);//设置均值为20.1的泊松分布

	for(int i = 0;i<nRawDataSize;i++)
	{
		if( i < clutterNum)
		{
			int index = vec[i];
			randClutterVec[index] = 1;
		}
		
		randDistanceVec[i] = distribution(generator) * 0.03;
        //D_PRINTF(D_INFO, "rand distance[%d]:%d",i, randDistanceVec[i]);
	}

    //调用cuda并行计算点云数据 io:h_raw_data_phy
    calculate_lidar_ray(h_raw_data,h_normal_raw_data, h_raw_data_phy, nRawDataSize, &lidarSimParams,randDistanceVec,randClutterVec);


	for (size_t i = 0; i < nRawDataSize; i++)
	{
		//_lidarData->addPhyRawData(h_raw_data_phy[i]);
        phy_vecData.push_back(float4{h_raw_data_phy[i].x,h_raw_data_phy[i].y,h_raw_data_phy[i].z,h_raw_data_phy[i].w});
	}

	if(_curFrame == _maxFrame - 1)
	{
		phy_vecDataBackup.clear();
		phy_vecDataBackup.reserve(phy_vecData.size() + 10);
		
		for (size_t i = 0; i < phy_vecData.size(); i++)
		{
			phy_vecDataBackup.push_back(float4{phy_vecData[i].x,phy_vecData[i].y,phy_vecData[i].z,phy_vecData[i].w});
		}

		phy_vecData.clear();
	}

	if(phy_vecDataBackup.size() > 0)
	{
		for (size_t i = 0; i < phy_vecDataBackup.size(); i++)
		{
            if(phy_vecDataBackup[i].z >= _lidarDescription.assemblePositionZ * (-1)) //TODO: 临时修复地面下有点云问题，过滤地面下点云
            {
                _lidarData->addPhyRawData(float4{phy_vecDataBackup[i].x,phy_vecDataBackup[i].y,phy_vecDataBackup[i].z,phy_vecDataBackup[i].w});
                //cout << "adding phy raw data: [x: " << phy_vecDataBackup[i].x << " y: " << phy_vecDataBackup[i].y << " z: " << phy_vecDataBackup[i].z << " w: " << phy_vecDataBackup[i].w << "]"<< endl;
            }
		}
		
	}

	// DEBUG调试
	printData(_lidarData);

	return _lidarData;
}

// 光线追踪 计算最近碰撞点的位置
bool Lidar::makeDataClosestHits()
{
	float3 lidar_position = getLidarPosition();//返回lidar所在车辆得传感器动态坐标
    _lidarRayGenerator.setOrigin(lidar_position);

	// 设置雷达所安装车辆的姿态角
	// TODO 设置固定的姿态角
    // _lidarRayGenerator.setEgoH(obj->getH() + 2 * PI * getRoundRate());
    // _lidarRayGenerator.setEgoP(obj->getP());
    // _lidarRayGenerator.setEgoR(obj->getR());
	_lidarRayGenerator.setEgoH(0.0f);
	_lidarRayGenerator.setEgoP(0.0f);
	_lidarRayGenerator.setEgoR(0.0f);

	// 取得传感器周围的mesh信息
	// TODO 通过UE4获取场景mesh信息并传递给ScenarioData
    ScenarioData scenario;
	scenario.points.push_back(make_float3(0, 0, 0));
	scenario.points.push_back(make_float3(1, 1, 1));
	scenario.points.push_back(make_float3(2, 2, 2));
	scenario.materials.push_back(MATERIAL_TYPE_METAL);

	// 设置生成最近撞击点用的场景数据
    _lidarRayGenerator.setScenarioData(scenario);

	// 利用光线追踪，生成最近撞击点的位置信息
    void* point_cloud_data = NULL;    // 点云真值数据
	void* point_normal_data = NULL;
    size_t point_cloud_data_size = 0;    // 点云真值法向量
	size_t point_normal_data_size = 0;
    if(!_lidarRayGenerator.makePointCloud(&point_cloud_data, &point_cloud_data_size,&point_normal_data,&point_normal_data_size))
    {
        return false;
    }

	_lidarData = new LidarData();
    // 建立激光雷达数据对象，此数据由SensorManage管理
	_lidarData->set_description(&_lidarDescription);

	float3 pos = getLidarPosition();
    float4* data = (float4*)point_cloud_data;
	
	float4* normal_data = (float4*)point_normal_data;
	size_t normal_size = point_normal_data_size/sizeof(float4);
    std::cout << "LJY debug point normal cloud data size" << normal_size << std::endl;

	vecDataPerFrame.clear();
	vecNormalDataPerFrame.clear();
	size_t size = point_cloud_data_size / sizeof(float4);

    void* mesh_index_data = NULL;
    mesh_index_data = _lidarRayGenerator.getMeshIndex();
    int* mesh_index_ptr = (int*) mesh_index_data;

    std::cout << "LJY debug point cloud number" << size << std::endl;
    for(size_t i = 0; i < size; i++){
        if(data[i].w > 1e-10 && normal_data[i].w >= 1e-10 )
        {
            //D_PRINTF(D_INFO, "mesh_index_data[%d][%d]",i, mesh_index_ptr[i]);
            int reflection_type = scenario.materials[mesh_index_ptr[i] ];
            float reflectance = material_reflections[reflection_type];
            //D_PRINTF(D_INFO, "mesh_reflection_data[%d][%f]",i, reflectance);

            vecDataPerFrame.push_back(float4{data[i].x,data[i].y,data[i].z,reflectance});
            vecNormalDataPerFrame.push_back(float4{normal_data[i].x,normal_data[i].y,normal_data[i].z,reflectance});
            vecData.push_back(float4{data[i].x,data[i].y,data[i].z,reflectance});
            vecNormalData.push_back(float4{normal_data[i].x,normal_data[i].y,normal_data[i].z,reflectance});
			//_lidarData->addRawData(data[i]);
        }
    }
    std::cout << "LJY debug vecDataPerFrame data size" << vecDataPerFrame.size() << std::endl;
	if(_curFrame == _maxFrame - 1)
	{
		vecDataBackup.reserve(vecData.size() + 10);
		vecDataBackup.clear();
		for(size_t i = 0;i < vecData.size(); i++)
		{
			vecDataBackup.push_back(vecData[i]);
		}
		vecData.clear();

		vecNormalDataBackup.reserve(vecNormalData.size() + 10);
		vecNormalDataBackup.clear();
		for(size_t i = 0;i < vecNormalData.size(); i++)
		{
			vecNormalDataBackup.push_back(vecNormalData[i]);
		}
		vecNormalData.clear();
	}
	if(vecDataBackup.size() > 0)
	{
		for(size_t i = 0;i < vecDataBackup.size(); i++)
		{
			_lidarData->addRawData(float4{vecDataBackup[i].x,vecDataBackup[i].y,vecDataBackup[i].z,vecDataBackup[i].w});
            //cout << "adding raw data: [x: " << vecDataBackup[i].x << " y: " << vecDataBackup[i].y << " z: " << vecDataBackup[i].z << " w: " << vecDataBackup[i].w << "]"<< endl;
		}
        std::cout << "finish RawData insertion: " << vecDataBackup.size() << " for frame: " << _curFrame << std::endl;

		for(size_t i = 0;i < vecNormalDataBackup.size(); i++)
		{
			_lidarData->addNormalRawData(float4{vecNormalDataBackup[i].x,vecNormalDataBackup[i].y,vecNormalDataBackup[i].z,vecNormalDataBackup[i].w});
		}
	}
    return true;
}

double Lidar::getIntervalOfFrame()
{
    return 1.0 / _lidarDescription.fSpeed;
}

SensorDescription* Lidar::get_description()
{
	return (SensorDescription*)&_lidarDescription;
}

bool Lidar::equal_description(SensorDescription* description)
{

	if(description == NULL)	
	{
		return NULL;
	}

	return (memcmp(&_lidarDescription, description, sizeof(LidarDescription)) == 0);

}

int Lidar::getType()
{
    return SENSOR_TYPE_LIDAR;
}

void Lidar::printData(LidarData *lidarData)
{
	float3 pos = getLidarPosition();
	vector<float4> raw_data = lidarData->getRawData();
	vector<float4> normal_raw_data = lidarData->getNormalRawData();
	FILE *pRawDataFile = nullptr;
	char cRawDataFile[100];

	const std::string createDir = "mkdir -p ./output/Sensor/Lidar/ClosestHit";
	(void)std::system(createDir.c_str());

	sprintf(cRawDataFile, "./output/Sensor/Lidar/ClosestHit/LidarData%d.csv", lidarData->get_frame());
	pRawDataFile = fopen(cRawDataFile, "w");
	fprintf(pRawDataFile, "索引,世界坐标X,世界坐标Y,世界坐标Z,反射率,法线x,法线y,法线z,点乘积 %d\n",raw_data.size());
	for (size_t i = 0; i < raw_data.size(); i++)
	{
		fprintf(pRawDataFile, "%d,%f,%f,%f,%f,%f,%f,%f,%f\n",i,raw_data[i].x, raw_data[i].y, raw_data[i].z, raw_data[i].w,
		normal_raw_data[i].x,normal_raw_data[i].y,normal_raw_data[i].z,normal_raw_data[i].w	);
		// float disR = sqrt(pow(raw_data[i].x, 2) + pow(raw_data[i].y, 2) + pow(raw_data[i].z, 2));
		// fprintf(pRawDataFile, "%f,%f \n", disR, raw_data[i].w);
	}
	fclose(pRawDataFile);

	vector<float4> raw_data_phy = lidarData->getPhyRawData();
	FILE *pRawDataPhyFile = nullptr;
	char cRawDataPhyFile[100];

	const std::string createDir1 = "mkdir -p ./output/Sensor/Lidar/PhyRawData";
	(void)std::system(createDir1.c_str());

	sprintf(cRawDataPhyFile, "./output/Sensor/Lidar/PhyRawData/LidarData%d.csv", lidarData->get_frame());
	pRawDataPhyFile = fopen(cRawDataPhyFile, "w");
	//fprintf(pRawDataPhyFile, "索引,世界坐标X,世界坐标Y,世界坐标Z,回波强度 %d\n",raw_data_phy.size());
    fprintf(pRawDataPhyFile, "索引,世界坐标X,世界坐标Y,世界坐标Z,反射率 %d\n",raw_data_phy.size());
	for (size_t i = 0; i < raw_data_phy.size(); i++)
	{
		fprintf(pRawDataPhyFile, "%d,%f,%f,%f,%f\n",i,raw_data_phy[i].x, raw_data_phy[i].y, raw_data_phy[i].z, raw_data_phy[i].w);
		// float disR = sqrt(pow(raw_data_phy[i].x, 2) + pow(raw_data_phy[i].y, 2) + pow(raw_data_phy[i].z, 2));
		// fprintf(pRawDataPhyFile, "%f,%f \n", disR, raw_data_phy[i].w);
	}
	fclose(pRawDataPhyFile);

	//diff

	// if(raw_data_phy.size() == raw_data.size())
	// {
	// 	FILE *pDiffFile = nullptr;
	// 	char cDiffFile[100];
	// 	const std::string createDir2 = "mkdir -p ./output/Sensor/Lidar/diff";
	// 	(void)std::system(createDir2.c_str());
	// 	sprintf(cDiffFile, "./output/Sensor/Lidar/diff/diff%d.csv", lidarData->get_frame());
	// 	pDiffFile = fopen(cDiffFile, "w");
	// 	fprintf(pDiffFile, "索引,世界坐标X差值,世界坐标Y差值,世界坐标Z差值,回波强度差值,距离,%d \n",raw_data_phy.size());

	// 	for (size_t i = 0; i < raw_data_phy.size(); i++)
	// 	{
	// 		float diff = sqrt(pow(raw_data_phy[i].x - raw_data[i].x, 2) + pow(raw_data_phy[i].y  - raw_data[i].y, 2) + pow(raw_data_phy[i].z  - raw_data[i].z, 2));
	// 		fprintf(pDiffFile, "%d,%f,%f,%f,%f,%f\n", 
	// 				i,
	// 				abs(raw_data_phy[i].x - raw_data[i].x), 
	// 				abs(raw_data_phy[i].y  - raw_data[i].y),
	// 				abs(raw_data_phy[i].z  - raw_data[i].z), 
	// 				abs(raw_data_phy[i].w - raw_data[i].w),
	// 				diff );
	// 	}

	// 	fclose(pDiffFile);
	// }

	if(raw_data_phy.size() == raw_data.size())
	{
		FILE *pDiffFile = nullptr;
		char cDiffFile[100];
		const std::string createDir2 = "mkdir -p ./output/Sensor/Lidar/ex";
		(void)std::system(createDir2.c_str());
		sprintf(cDiffFile, "./output/Sensor/Lidar/ex/ex%d.csv", lidarData->get_frame());
		pDiffFile = fopen(cDiffFile, "w");
		fprintf(pDiffFile, "索引,世界坐标X差值,世界坐标Y差值,世界坐标Z差值,回波强度差值,距离,%d \n",raw_data_phy.size());

		for (size_t i = 0; i < raw_data_phy.size(); i++)
		{
			if(abs(raw_data[i].x) > 10 || abs(raw_data[i].y) > 10 || abs(raw_data[i].z)> 10 )
			{
				fprintf(pDiffFile, "%d,%f,%f,%f,%f\n", 
					i,
					abs(raw_data[i].x - pos.x), 
					abs(raw_data[i].y - pos.y),
					abs(raw_data[i].z - pos.z), 
					abs(raw_data[i].w)
					 );
			}
		}

		fclose(pDiffFile);
	}
}

SensorData* Lidar::makeDefaultData()
{
    LidarData* ld = new LidarData();

	LidarDescription description;
	ld->set_description(&description);

	return ld;
}

float Lidar::radians(float degrees)
{
    return degrees * M_PIf / 180.0f;
}

void Lidar::rollOneFrame()
{
    _curFrame++;
    _curFrame = _curFrame%(_maxFrame);
}

float Lidar::getRoundRate()
{
    return (1.0/_maxFrame) * (_curFrame + 0.5); //0.5参数为了矫正渲染效果的激光雷达扫描起始角度向前
}

float3 Lidar::getLidarPosition()
{
	return make_float3(0.0f, 0.0f, 0.0f);
}

float Lidar::getLidarClutterRatio()
{
	return 0.3;
}