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

	//��ʼ�����
	int InitMechEyeCamera();

	//���ݲɼ�
	int CapturePointXYZ(std::vector<MECH_EYE_POINT>& vpntData);
	int CapturePointXYZ(cv::Mat &mData);
	int CaptureDepth(cv::Mat &mData);
	int CaptureColor(cv::Mat &mData);

	//��ȡ��Ϣ
	int GetCameraInternalParam(DeviceIntri& param);//��ȡ����ڲ�
	int GetCameraResolution(DeviceResolution& param);//��ȡ�ֱ���
	T_MECHEYE_CAMREA_DRIVER_PARAM GetCameraBaseInfo();

	//��������
	MechEyeDevice *GetMechEyeDevice();
	//2Dͼ���ع�ģʽ:	setScan2DExposureMode,	getScan2DExposureMode
	//2Dͼ���ع�ʱ��:	setScan2DExposureTime,	getScan2DExposureTime
	//2Dͼ��ROI:		setScan2DROI,			getScan2DROI
	//3Dͼ���ع�ʱ��:	setScan3DExposure,		getScan3DExposure 
	//3Dͼ��ROI:		setScan3DROI,			getScan3DROI
	//3Dͼ����ȷ�Χ:	setDepthRange,			getDepthRange

private:
	MechEyeDevice* m_pMechMindDevice = NULL;
	T_MECHEYE_CAMREA_DRIVER_PARAM m_tMecheyeCameraDriverPara;
};

#endif // ENABLE_MECH_EYE

