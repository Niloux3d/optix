#ifndef LidarDescription_h
#define LidarDescription_h

#include <string>
#include <iostream>
#include <string.h>
#include "SensorDescription.h"

using namespace std;

struct LidarDescription : public SensorDescription
{
	//参照KITTI的velodyne参数，设置默认值
    LidarDescription(string n = "untitled lidar", float aPositionX = 0.0, float aPositionY = 0.0, float aPositionZ = 1.5, int c = 128,
        float aHeading = 0.0, float aPitch = 0.0, float aRoll = 0.0, float hFov = 360.0, float vFov = 15.0, float aMinimumDetectRange = 0.0,
        float r = 250.0, float rFrequency = 10.0, float hResolution = 0.09, float uFov = 10.0, float lFov = -16.8, float aAttenuationRate = 0.004,
        float dGeneralRate = 0.45, float dIntensityLimit = 0.8, float dZeroIntensity = 0.4, float nStddev = 0.0, float apeakOpticalPower = 90.0,
        float aVBeamDivAngle = 1.0, float aHBeamDivAngle = 1.0, float aTAperture = 1.0, float aRAperture = 1.0,
        float aTTransmissivitye = 1.0, float aRTransmissivity = 1.0, float aNEP = 1.0, float aNoiseBandwidth = 1.0,
        float areceiverAperture = 50.0, float areceiverOpticalEfficiency = 0.8, float aunitGain = 10000000.0,
        float aAPD = 100.0, float abandwidth = 18,float aphotonFluxes = 3, float areflectivity = 0.5,
        float aexcessNoiseFactor = 5.0, float afalseAlarmRate = 0.1, int aFrameRate = 10, double aideal_Freq = 10, 
        float angleOfBeamDivergence = 0.001,/* float pulseWireHarness = 64,*/ int pulseWaveForm = 2, float pulseDivergenceAngle = 0.00555, 
        float waveLength = 1000.0, float delayOfPulse = 20.0, float peakPower = 100.0,  float speed = 3.0, float frequency = 10.0,
        float gainOfEmission = 1.0, float sizeOfAperture = 0.01, float rotaryPulseResolution = 00555, float scanningPulseResolution = 00555,
        float atmosphericExtinctionCoefficient = 1.0, float OpticalSystemExtinctionCoefficient = 1.0, int operatingMode = 0,
        float afExistProb = 100, SENSOR_MODEL_TYPE amodelType = SENSOR_MODEL_IDEAL):
#if 0	/* Tang 2022.10.27 一汽一阶段，拆分理想传感器 [start] */
        assemblePositionX(aPositionX),
        assemblePositionY(aPositionY),
        assemblePositionZ(aPositionZ),
		heading(aHeading),
        pitch(aPitch),
        roll(aRoll),
#endif	/* Tang 2022.10.27 一汽一阶段，拆分理想传感器 [end] */
        channels(c), 
#if 1 //bug5507 2022.03.15 dingjun
        horizontalFov(hFov),
        verticalFov(vFov),
        minimumDetectRange(aMinimumDetectRange),
#endif
        range(r), 
        rotationFrequency(rFrequency), 
        horizontalResolution(hResolution), 
        upperFov(uFov), 
        lowerFov(lFov), 
        atmosphereAttenuationRate(aAttenuationRate), 
        dropoffGeneralRate(dGeneralRate), 
        dropoffIntensityLimit(dIntensityLimit), 
        dropoffZeroIntensity(dZeroIntensity), 
        noiseStddev(nStddev),
#if 1 // 添加传感器帧率控制 LS 22.04.24
        frameRate(aFrameRate),
#endif
        fVBeamDivergenceAngle(aVBeamDivAngle),
        fHBeamDivergenceAngle(aHBeamDivAngle),
        fTransmittingAperture(aTAperture),
        fReceivingAperture(aRAperture),
        fTransmitTransmissivity(aTTransmissivitye),
        fReceiveTransmissivity(aRTransmissivity),
        fNEP(aNEP),
        fNoiseBandwidth(aNoiseBandwidth),

#if 1 //激光雷达新增参数 2022.04.07 hlx 
        peakOpticalPower(apeakOpticalPower),
        receiverAperture(areceiverAperture),
        receiverOpticalEfficiency(areceiverOpticalEfficiency),
#if 0 // bug6479 22.04.19 LS
        extinctionCoefficient(aextinctionCoefficient),
#endif
        unitGain(aunitGain),
        APD(aAPD),
        bandwidth(abandwidth),
        excessNoiseFactor(aexcessNoiseFactor),
        reflectivity(areflectivity),
        falseAlarmRate(afalseAlarmRate),
        photonFluxes(aphotonFluxes),
#endif 
        fExistProb(afExistProb)
#if 0	/*2022.10.24 Tang 理想传感器拆分 [start] -> bug17012[delete] LS 23.06.13*/
		ideal_Freq(aideal_Freq)
#endif	/*2022.10.24 Tang 理想传感器拆分 [end]*/
    {
        type = SENSOR_TYPE_LIDAR;
        strcpy(name, n.c_str()); 
        enable = false;
        sensorOutputType = 0; // 包络线[Add] 2022.08.22 LS
        isSavePointCloud = false;
        dumpRawDataType = 0;
        isVisibleCloudPoint = false;
        advSettingsEnable = false;
        bPhySettingsEnable = false; 
        thermalNoise = 0.0;
        darkcurrent = 0.00000002;
        solarNoise = 0.0000001;
        enableConeRender = false; // 包络线[Add] 2022.08.22 LS -> [bug9111] 默认为false LS 08.30
        bExistProbInput = false;
#if 1	/*2022.10.24 Tang 理想传感器拆分 [start]*/
		modelType = amodelType;
		assemblePositionX = aPositionX;
		assemblePositionY = aPositionY;
		assemblePositionZ = aPositionZ;
		heading = aHeading;
		pitch = aPitch;
		roll = aRoll;
		object_detection_type = SENSOR_OBJECT_DETECTION_TYPE_ALL;
#endif	/*2022.10.24 Tang 理想传感器拆分 [end]*/

#if 1   /* 增加 激光雷达 物理参数 */
        // bPhySettingsEnable = false;                                            //物理参数开关
        fAngleOfBeamDivergence = angleOfBeamDivergence;                           //光束发散角
        // fPulseWireHarness = pulseWireHarness;                                     //脉冲线束
        iPulseWaveForm = pulseWaveForm;                                           //脉冲波形
        fPulseDivergenceAngle = pulseDivergenceAngle;                             //脉冲发散角
        fWaveLength = waveLength;                                                 //波长
        fDelayOfPulse = delayOfPulse;                                             //脉冲延时
        fPeakPower = peakPower;                                                   //峰值功率
        fSpeed = speed;                                                           //转速
        fFrequency = frequency;                                                   //频率
        fGainOfEmission = gainOfEmission;                                         //发射增益
        fSizeOfAperture = sizeOfAperture;                                         //光阑大小
        fRotaryPulseResolution = rotaryPulseResolution;                           //旋转式脉冲分辨率
        fScanningPulseResolution = scanningPulseResolution;                       //扫描式脉冲分辨率
        fAtmosphericExtinctionCoefficient = atmosphericExtinctionCoefficient;     //大气消光系数
        fOpticalSystemExtinctionCoefficient = OpticalSystemExtinctionCoefficient; //光学系统消光系数
        iOperatingMode = operatingMode;                                           //运行模式
#endif
    }

#if 0	/* Tang 2022.11.01 一汽一阶段，传感器模型拆分 将安装位置信息移至父类[start]*/
    // string name; 
    float assemblePositionX; 
    float assemblePositionY; 
    float assemblePositionZ;
    float roll;                 // 翻滚角 
    float heading;              // 偏航角
    float pitch;                // 俯仰角
#endif	/* Tang 2022.11.01 一汽一阶段，传感器模型拆分 将安装位置信息移至父类[end]*/
    float horizontalFov; //水平视场角 
    float verticalFov; //垂直视场角 
    float minimumDetectRange;
	int channels;	// 线束
    float range; //最大探测距离
    float rotationFrequency; //扫描帧频
    float horizontalResolution; //水平分辨率
    float upperFov; //上垂直方位角
    float lowerFov; //下垂直方位角
    float atmosphereAttenuationRate; //大气衰减率
    float dropoffGeneralRate; //一般衰减率
    float dropoffIntensityLimit; //衰减强度极限
    float dropoffZeroIntensity; //0衰减率
    float noiseStddev; //加性高斯噪声
    int dumpRawDataType;
#if 1 // 添加传感器帧率控制 LS 22.04.24
    int frameRate;
#endif
#if 1 // [一汽]混合传感器 [Add] LS 23.04.07
    float fVBeamDivergenceAngle;     // 光束垂直发散角
    float fHBeamDivergenceAngle;     // 光束水平发散角
    float fTransmittingAperture;     // 发射孔径
    float fReceivingAperture;        // 接收孔径
    float fTransmitTransmissivity;   // 发射孔径透射率
    float fReceiveTransmissivity;    // 接收孔径透射率
    float fNEP;                      // 噪声等效功率
    float fNoiseBandwidth;           // 噪声带宽
#endif
#if 1 //激光雷达新增参数 2022.03.31 hlx
    float peakOpticalPower; //峰值光功率
    float receiverAperture; //接收器孔径
    float receiverOpticalEfficiency; //接收器光学系统效率
#if 0 // bug6479 22.04.19 LS
    float extinctionCoefficient; //大气消光系数
#endif
    float unitGain; //单位增益响应
    float APD; //APD
    float bandwidth;//带宽
    double darkcurrent; //暗电流
    double solarNoise; //太阳辐照度背景噪声
    float excessNoiseFactor; //过量噪声系数
    double thermalNoise; //热噪声
    float reflectivity; //目标反射率
    float falseAlarmRate; //虚警概率
    float photonFluxes;  //光子通量
    bool  advSettingsEnable; //高级选项
#endif
	bool isVisibleCloudPoint;
    bool isSavePointCloud;
    bool enableConeRender; // 包络线开关
    bool bExistProbInput;     // 自定义目标存在可能性开关
    float fExistProb;        // 目标存在可能性
#if 0 // 补位
    char dummy[1];
#endif

#if 1	/*2022.10.24 Tang 理想传感器拆分 [start]*/
	double ideal_Freq;	// 理想的频率参数
#endif	/*2022.10.24 Tang 理想传感器拆分 [end]*/

#if 1   /* 增加 激光雷达 物理参数 */
    bool bPhySettingsEnable;                   //物理参数开关
    float fAngleOfBeamDivergence;              //光束发散角
    // float fPulseWireHarness;                   //脉冲线束
    int iPulseWaveForm;                        //脉冲波形
    float fPulseDivergenceAngle;               //脉冲发散角
    float fWaveLength;                         //波长
    float fDelayOfPulse;                       //脉冲延时
    float fPeakPower;                          //峰值功率
    float fSpeed;                              //转速
    float fFrequency;                          //频率
    float fGainOfEmission;                     //发射增益
    float fSizeOfAperture;                     //光阑大小
    float fRotaryPulseResolution;              //旋转式脉冲分辨率
    float fScanningPulseResolution;            //扫描式脉冲分辨率
    float fAtmosphericExtinctionCoefficient;   //大气消光系数
    float fOpticalSystemExtinctionCoefficient; //光学系统消光系数
    int iOperatingMode;                        //运行模式
#endif
#if 1 // 刘婧 2023.12.7 [一汽需求][新增lidar 物理参数][start]
    int MaxNumOfReflections;		           //最大反射数（默认为1）	
#endif // 刘婧 2023.12.7 [一汽需求][新增lidar 物理参数][end]

#if 1 //hlx 2022.05.17 bug1312
public:
    int size()
    {
        return sizeof(LidarDescription);
    }

    int getFrameRate()
    {
        return frameRate;
    }
#endif
#if 0	/*2022.10.24 Tang 理想传感器拆分 [start] -> bug17012[delete] LS 23.06.13*/
	double ideal_Freq;	// 理想的频率参数
#endif	/*2022.10.24 Tang 理想传感器拆分 [end]*/
};


#endif