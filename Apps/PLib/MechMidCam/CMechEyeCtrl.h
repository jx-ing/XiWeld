#pragma once
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"

#if ENABLE_MECH_EYE


#include ".\Apps\PLib\BasicFunc\Const.h"
#include <MechEyeApi.h>
#include <SampleUtil.h>
#include <opencv2/highgui/highgui.hpp>

typedef struct MECH_EYE_POINT
{
	double x;
	double y;
	double z;
}MECH_EYE_POINT;

using namespace mmind::api;

class CMechEyeCtrl
{
public:
	CMechEyeCtrl(T_MECHEYE_CAMREA_DRIVER_PARAM tMecheyeCameraDriverPara);
	~CMechEyeCtrl();

	//初始化相机
	int InitMechEyeCamera();

	//数据采集
	int CapturePointXYZ(std::vector<MECH_EYE_POINT>& vpntData);
	int CapturePointXYZ(cv::Mat &mData);
	int CaptureDepth(cv::Mat &mData);
	int CaptureColor(cv::Mat &mData);

	//获取信息
	int GetCameraInternalParam(DeviceIntri& param);//获取相机内参
	int GetCameraResolution(DeviceResolution& param);//获取分辨率
	T_MECHEYE_CAMREA_DRIVER_PARAM GetCameraBaseInfo();

	//参数设置
	MechEyeDevice *GetMechEyeDevice();
	//2D图像曝光模式:	setScan2DExposureMode,	getScan2DExposureMode
	//2D图像曝光时间:	setScan2DExposureTime,	getScan2DExposureTime
	//2D图像ROI:		setScan2DROI,			getScan2DROI
	//3D图像曝光时间:	setScan3DExposure,		getScan3DExposure 
	//3D图像ROI:		setScan3DROI,			getScan3DROI
	//3D图像深度范围:	setDepthRange,			getDepthRange

private:
	MechEyeDevice* m_pMechMindDevice = NULL;
	T_MECHEYE_CAMREA_DRIVER_PARAM m_tMecheyeCameraDriverPara;
};

#endif // ENABLE_MECH_EYE

