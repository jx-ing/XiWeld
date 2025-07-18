/****************************************************************************************
2020��09��22�� ������

ע�⣺
1.��������ڵ�����������Kinect2.0����
2.������Ҫ΢��KinectSDKv2.0֧�֣�
3.��ʱ�俪��ʹ��ʱ�����ڻ�ȡͼƬ֮ǰʹ��IsAvailable()��ȷ�������������������

ʹ�ò��裺
1.����;
2.OpenKinect();
3.��ȡͼƬ(��ʹ��GetColorImage()��GetDepthImage()��GetInfraredImage()��֮��ʹ�ö�Ӧ�ĳ�Ա��������);
4.CloseKinect()��������;
****************************************************************************************/

#pragma once

#include "Apps/PLib/Others/Kinect.h"
#include "K4a.h"
#include <opencv2/opencv.hpp>


class CKinectControl
{
public:
	CKinectControl(T_DEPTH_CAMREA_DRIVER_PARAM tDepthParam);
	~CKinectControl();

public:	
	int OpenKinect(int nWaitTime = 3000);														//��Kinect,�������Ҫһ��ʱ�䣨��ʹ��WaitForAvailable()������
	void WaitForAvailable(int nWaitTime = 3000);												//�豸������Ҫ�㹻��ʱ������������ʱ��Ҫ�ȴ�һ��ʱ��
	bool IsAvailable();																			//�ж�Kinect�Ƿ����(����Azure Kinect����������ͼ����)
	void CloseKinect();																			//�ر�Kinect�����Ե�����ʱ�Զ��ر�
	bool GetColorImage();																		//��ȡ��ͼ
	bool GetDepthImage(BOOL bDistortion = TRUE);												//��ȡ���ͼ
	bool GetInfraredImage(BOOL bDistortion = TRUE);												//��ȡ����ͼ
	void SetImgType(bool bColorImg = true, bool bDepthImg = true, bool bInfraredImg = true);	//����������ͼ��ͼ���ͣ�������ͼ��ʹ����Ч	
	void OpenContinuousImageAcquisition();														//����������ͼ
	void CloseContinuousImageAcquisition();														//�ر�������ͼ
	
	cv::Mat	*m_ColorImage;																		//��ͼ
	cv::Mat	*m_DepthImage;																		//���ͼ
	cv::Mat	*m_InfraredImage;																	//����ͼ
	k4a_device_configuration_t m_K4aConfigMode;

	bool GetKinectSerialNumber(CString sSerialNumber);											//��ȡ������к�
	bool StartCameras(k4a_device_configuration_t K4aConfigMode);								//�������ģʽ���������
	int m_nReNum = 0;
	bool CloseCameras();
	bool GetCapture(int nWaitTime = 1500);														//��ȡһ֡ͼƬ����

	bool GetColorImage_k4a(cv::Mat	*ColorImage);												//��ȡ��ɫͼƬ
	bool GetDepthImage_k4a(cv::Mat	*DepthImage);												//��ȡ���ͼƬ
	bool GetInfraredImage_k4a(cv::Mat	*InfraredImage);										//��ȡ����ͼƬ

	bool GetDepthPointCloud(std::vector<cv::Point3d> &vcPointCloud);							//��ȡ�������
	
	void Distortion(cv::Mat mSrc, cv::Mat &mDst);
	bool convertMatToPcl(const cv::Mat& xyzDepth, std::vector<cv::Point3d> &vcPointCloud);

	int GetDepthCameraType();
	int GetDepthCameraWidth();
	int GetDepthCameraHeight();
	int GetColorCameraWidth();
	int GetColorCameraHeight();
	int GetInfraredCameraWidth();
	int GetInfraredCameraHeight();
	bool GetDepthCameraEnableUndistort();
	cv::Mat GetDepthCameraCameraMatrix();
	cv::Mat GetDepthCameraDistCoeffs();
private:
	bool InitKinect();																			//��ʼ��Kinect

	bool GetOneColorImage_k4a();
	bool GetOneDepthImage_k4a(BOOL bDistortion = TRUE);
	bool GetOneInfraredImage_k4a(BOOL bDistortion = TRUE);

	bool GetOneColorImage_v2();
	bool GetOneDepthImage_v2();
	bool GetOneInfraredImage_v2();

	static UINT ThreadContinuousImageAcquisition(void *pParam);									//������ͼ�߳�
	void ContinuousImageAcquisition();															//����������ͼ

	bool m_bColorImg;																			//������ͼʹ��
	bool m_bDepthImg;																			//������ͼʹ��
	bool m_bInfraredImg;																		//������ͼʹ��
	bool m_bContinuousImageAcquisition;
	IKinectSensor	*m_Sensor;
	HRESULT			m_Hr;	
	IColorFrameSource   *m_pIColorSource;
	IColorFrameReader   *m_pIColorReader;
	IColorFrame			*m_pIColorFrame;
	IDepthFrameSource   *m_pIDepthSource;
	IDepthFrameReader   *m_pIDepthReader;
	IDepthFrame			*m_pIDepthFrame;
	IInfraredFrameSource	*m_pIInfraredSource;
	IInfraredFrameReader	*m_pIInfraredReader;
	IInfraredFrame			*m_pIInfraredFrame;

	k4a_capture_t m_K4aCapture;
	k4a_device_t m_kDevice;

	T_DEPTH_CAMREA_DRIVER_PARAM m_tDepthParam;
	
};

