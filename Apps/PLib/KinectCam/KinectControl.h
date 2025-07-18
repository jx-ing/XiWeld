/****************************************************************************************
2020年09月22日 江文奇

注意：
1.此类仅用于单个深度相机（Kinect2.0）；
2.此类需要微软KinectSDKv2.0支持；
3.长时间开机使用时，请在获取图片之前使用IsAvailable()来确保相机仍在正常工作；

使用步骤：
1.构造;
2.OpenKinect();
3.获取图片(先使用GetColorImage()、GetDepthImage()或GetInfraredImage()，之后使用对应的成员变量即可);
4.CloseKinect()或者析构;
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
	int OpenKinect(int nWaitTime = 3000);														//打开Kinect,这可能需要一点时间（因使用WaitForAvailable()方法）
	void WaitForAvailable(int nWaitTime = 3000);												//设备可能需要足够的时间来开启，此时需要等待一段时间
	bool IsAvailable();																			//判断Kinect是否可用(对于Azure Kinect，仅连续采图可用)
	void CloseKinect();																			//关闭Kinect，可以等析构时自动关闭
	bool GetColorImage();																		//获取彩图
	bool GetDepthImage(BOOL bDistortion = TRUE);												//获取深度图
	bool GetInfraredImage(BOOL bDistortion = TRUE);												//获取红外图
	void SetImgType(bool bColorImg = true, bool bDepthImg = true, bool bInfraredImg = true);	//设置连续采图采图类型，连续采图中使用无效	
	void OpenContinuousImageAcquisition();														//开启连续采图
	void CloseContinuousImageAcquisition();														//关闭连续采图
	
	cv::Mat	*m_ColorImage;																		//彩图
	cv::Mat	*m_DepthImage;																		//深度图
	cv::Mat	*m_InfraredImage;																	//红外图
	k4a_device_configuration_t m_K4aConfigMode;

	bool GetKinectSerialNumber(CString sSerialNumber);											//获取相机序列号
	bool StartCameras(k4a_device_configuration_t K4aConfigMode);								//配置相机模式，开启相机
	int m_nReNum = 0;
	bool CloseCameras();
	bool GetCapture(int nWaitTime = 1500);														//获取一帧图片参数

	bool GetColorImage_k4a(cv::Mat	*ColorImage);												//获取彩色图片
	bool GetDepthImage_k4a(cv::Mat	*DepthImage);												//获取深度图片
	bool GetInfraredImage_k4a(cv::Mat	*InfraredImage);										//获取红外图片

	bool GetDepthPointCloud(std::vector<cv::Point3d> &vcPointCloud);							//获取深度数据
	
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
	bool InitKinect();																			//初始化Kinect

	bool GetOneColorImage_k4a();
	bool GetOneDepthImage_k4a(BOOL bDistortion = TRUE);
	bool GetOneInfraredImage_k4a(BOOL bDistortion = TRUE);

	bool GetOneColorImage_v2();
	bool GetOneDepthImage_v2();
	bool GetOneInfraredImage_v2();

	static UINT ThreadContinuousImageAcquisition(void *pParam);									//连续采图线程
	void ContinuousImageAcquisition();															//开启连续采图

	bool m_bColorImg;																			//连续采图使用
	bool m_bDepthImg;																			//连续采图使用
	bool m_bInfraredImg;																		//连续采图使用
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

