/*********************************************************************
* 版权所有 (C)2019, 哈尔滨行健机器人技术有限公司
**********************************************************************/

//无论哪种方式，均为连续采集

#if !defined DHGIGEIMAGECAPTURE_H
#define DHGIGEIMAGECAPTURE_H

#include "GxIAPI.h"
#include "highgui.h"
#include "cxcore.h"
#include "cv.h"
#include <vector>
#include ".\Apps\PLib\BasicFunc\Const.h"

inline void FilpImage(CvArr* src, CvArr* dst CV_DEFAULT(NULL), E_FLIP_MODE flip_mode = E_FLIP_NONE)
{
	switch (flip_mode)
	{
	case E_FLIP_VERTICAL:
		cvFlip(src, dst, 1);
		return;
	case E_FLIP_HORIZEN:
		cvFlip(src, dst, 0);
		return;
	case E_FLIP_BOTH:
		cvFlip(src, dst, -1);
		return;
	default:
		if (dst == NULL)
		{
			dst = src;
			return;
		}
		cvCopyImage(src, dst);
		return;
	}
}

inline void FilpImagePoint(CvPoint& tPoint, int nWidth, int nHeight, E_FLIP_MODE flip_mode = E_FLIP_NONE)
{
	switch (flip_mode)
	{
	case E_FLIP_VERTICAL:
		tPoint.x = nWidth - tPoint.x;
		tPoint.y = tPoint.y;
		return;
	case E_FLIP_HORIZEN:
		tPoint.x = tPoint.x;
		tPoint.y = nHeight - tPoint.y;
		return;
	case E_FLIP_BOTH:
		tPoint.x = nWidth - tPoint.x;
		tPoint.y = nHeight - tPoint.y;
		return;
	default:
		return;
	}
}

inline void FilpImagePoint(std::vector<CvPoint>& vtPoint, int nWidth, int nHeight, E_FLIP_MODE flip_mode = E_FLIP_NONE)
{
	switch (flip_mode)
	{
	case E_FLIP_VERTICAL:
		for (size_t i = 0; i < vtPoint.size(); i++)
		{
			vtPoint[i].x = nWidth - vtPoint[i].x;
			vtPoint[i].y = vtPoint[i].y;
		}
		return;
	case E_FLIP_HORIZEN:
		for (size_t i = 0; i < vtPoint.size(); i++)
		{
			vtPoint[i].x = vtPoint[i].x;
			vtPoint[i].y = nHeight - vtPoint[i].y;
		}
		return;
	case E_FLIP_BOTH:
		for (size_t i = 0; i < vtPoint.size(); i++)
		{
			vtPoint[i].x = nWidth - vtPoint[i].x;
			vtPoint[i].y = nHeight - vtPoint[i].y;
		}
		return;
	default:
		return;
	}
}

typedef enum
{
	E_ACQUISITION_MODE_SOURCE_SOFTWARE = 0,		//软触发模式
	E_ACQUISITION_MODE_SOURCE_HARD_0 = 1,		//硬触发模式
	E_ACQUISITION_MODE_SOURCE_HARD_1 = 2,		//硬触发模式
	E_ACQUISITION_MODE_SOURCE_HARD_2 = 3,		//硬触发模式
	E_ACQUISITION_MODE_SOURCE_HARD_3 = 4,		//硬触发模式
	E_ACQUISITION_MODE_CONTINUE = 5,			//连续模式
	E_DHGIGE_ACQUISITION_MODE_MAX_NUM,
}E_DHGIGE_ACQUISITION_MODE;

typedef enum
{
	E_CALL_BACK_MODE_OFF = 0,			//关闭回调函数
	E_CALL_BACK_MODE_WAIT_IMAGE,		//等待回调结束
	E_CALL_BACK_MODE_SAVE_IMAGE,		//回调储存图片
	E_DHGIGE_CALL_BACK_MAX_NUM,
}E_DHGIGE_CALL_BACK;

//inline void XI_ReleaseImage(IplImage** img)
//{
//	if (NULL != *img)
//	{
//		cvReleaseImage(img);
//		*img = NULL;
//	}
//}

//图片信息
struct T_GET_CB_IMAGE
{
	int nImageID			= -1;	//图片ID（主动计数）
	int nImageState			= -1;	//图片状态（相机内部获取）
	uint64_t ullFrameID		= 0;    //图片的帧号（相机内部获取）
	uint64_t ullTimestamp	= 0;	//图片时间戳（相机内部获取）
	long long llGetTime		= 0;	//图片获取时间（获取图片时由Xi_clock()函数生成）
	bool bGetImage			= false;//是否成功获取图片
	IplImage *pImage		= NULL;	//图片缓存

	//释放图片缓存
	void ReleaseImage();
	//重置全部数据
	void Clear();
	//生成一个新的图片缓存区
	void CreateImage(int nRoiWidth, int nRoiHeight);
	//设置数据
	void SetImage(const GX_FRAME_DATA& tFrameData);
	void SetImage(const GX_FRAME_CALLBACK_PARAM& tFrameData);
};

class CDHGigeImageCapture
{
public:
	//CDHGigeImageCapture();//小组立暂时禁用此构造函数
	CDHGigeImageCapture(CLog* cLog, T_CAMREA_PARAM tCameraPara);
	virtual ~CDHGigeImageCapture();

	/************************** 初始化 **************************/
	//初始化此类
	void InitPara(CLog* cLog, T_CAMREA_PARAM tCameraPara);

	T_CAMREA_PARAM m_tCameraPara;					//相机参数

	/************************* 连接相机 *************************/
	//初始化相机
	//nTriggerSrc：
	//		0：软触发；
	//		1、2、3、4：对应端口硬触发；
	//		其它：无触发
	bool InitCam(E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF);
	bool InitCam(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY, E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF);
	//释放相机资源
	bool UnInitCam();
	//释放库（最后一个相机退出时要调用）
	void CloseLib();

	E_DHGIGE_ACQUISITION_MODE m_eCameraMode;
	E_DHGIGE_CALL_BACK m_eCallBack;

	/************************* 相机状态 *************************/
	//判断相机是否已经打开
	bool DeviceIsOpen();
	//判断相机是否已经初始化
	bool DeviceIsInit();
	//判断相机是否已经开始采集
	bool DeviceIsAcq();

	/************************* 配置相机 *************************/
	//设置触发模式（只能在开始采集之前调用）
	//0：软触发
	//1、2、3、4：硬触发
	//其它：无触发
	bool SetTriggerMode(int nTriggerSrc);
	//设置ROI（只能在开始采集之前调用）
	bool SetROI(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY);
	//设置帧率（只能在开始采集之前调用）
	bool ChangeFrame(double dFrame);
	//设置增益
	bool SetGain(double dGainLevel = -1.0);
	//设置曝光时间
	bool SetExposure(double dExposureTime = -1.0);
	//开启闪光灯模式
	bool OpenStrobe(int nLineNum);
	//关闭闪光灯模式
	bool CloseStrobe(int nLineNum);
	//设置硬触发IO
	bool SetIOForTrigger(int nLineNum);

	/************************* 采图通用 *************************/
	//开始采集（无论哪种采图方式都需要先调用此函数）
	bool StartAcquisition(E_DHGIGE_CALL_BACK eCallBack);
	bool StartAcquisition();
	//停止采集
	bool StopAcquisition(bool bClearCallBackImage = true);

	/************************* 单张采图 *************************/
	/********************** 仅支持触发模式 **********************/
	//清空采集输出队列（硬触发之前可考虑调用，软触发会自动调用）
	bool FlushQueue();
	//设置采图状态（单张硬触发之前必须调用一次）
	void SetCaptureImageState(bool bState);
	//获取单张图像（触发模式）
	bool CaptureImage(int nTryTimes, unsigned int unTimeout = 2000);
	//获取单张图像（触发模式）
	bool CaptureImage(IplImage*& pImage, int nTryTimes, unsigned int unTimeout = 2000);

	IplImage* m_pImageBuff;							//图片缓存(外部使用可以释放)
	T_GET_CB_IMAGE m_tLatestImage;					//图片缓存(外部使用不可释放)

	/************************* 连续采图 *************************/
	//清除回调采集缓存数据
	void ClearCallBackImage();

	std::vector<T_GET_CB_IMAGE*> m_vtCallBackImage;	//图片缓存（用于获取相机当前图片）

	/************************* 设备信息 *************************/
	//获取相机总帧数
	int ReadCapInfo(bool bIsShowInfo, double* pdCurFrameRate = NULL);
	//获取帧率
	bool GetCurrentFrameRate(double& dCurrentFrameRate);
	//获取图像中心点坐标
	CvPoint GetImageCenter();

	/************************* 异常信息 *************************/
	//弹窗显示异常信息
	void ShowErrorString();
	//获取异常信息
	CString GetErrorString();

	/************************* 图片处理 *************************/
	//去畸变
	void Distortion(cv::Mat mSrc, cv::Mat& mDst);
	void Distortion(IplImage* pImgGray, IplImage* pResultImgGray);
	void Distortion(std::vector<CvPoint> mSrc, std::vector<CvPoint>& mDst);

private:
	/************************* 异常与记录 *************************/
	//按错误码设置异常信息
	void SetErrorString(GX_STATUS emErrorStatus);
	//设置异常信息
	void SetErrorString(CString sError = "");

	CString m_sErrorInfo;							//异常信息
	CLog* m_cLog;									//日志

	/************************* 输入参数 *************************/
	//清除类中的全部缓存空间，重置全部成员变量
	void Clear();

	GX_TRIGGER_MODE_ENTRY m_eTriggerMode;			//触发模式
	GX_TRIGGER_SOURCE_ENTRY m_eTriggerSource;		//触发源
	//int m_nAcqWay;									//采集方式（1：采集单张；2：采集多张；）

	/************************* 数据缓存 *************************/
	GX_FRAME_DATA m_tFrameData;						//图片缓存（相机格式，用于获取相机当前图片）

	/************************* 控制与状态 *************************/
	//初始化库
	bool InitLib();
	//打开相机
	bool OpenDevice(unsigned int unTimeout = 1000);
	//关闭相机
	bool CloseDevice();
	//在打开相机之后设定一些参数
	bool InitAfterOpen();
	//在开始采集之前根据相机当前参数设定缓存区
	bool InitBeforeCapture();
	//设置触发模式
	bool SetTriggerMode(GX_TRIGGER_MODE_ENTRY eTriggerMode, GX_TRIGGER_SOURCE_ENTRY eTriggerSource);
	//回调函数
	static void __stdcall ContiCapCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
	//回调采集 连续
	bool GetImageFromCam_Continuous(GX_FRAME_CALLBACK_PARAM* pFrame = NULL);
	//回调采集 单图
	bool GetImageFromCam_Single(GX_FRAME_CALLBACK_PARAM* pFrame = NULL);
	//软触发采集单张
	bool CaptureImage_Software(unsigned int unTimeout);
	//硬触发采集单张
	bool CaptureImage_Hard(unsigned int unTimeout);
	//转化IO端口参数
	bool GetLineNum(int nLineNum, GX_LINE_SELECTOR_ENTRY& gLineNum);
	//获取当前图片尺寸
	bool GetDeviceParam();
	//像素位深设为8bit
	bool SetPixelFormat8bit();

	GX_DEV_HANDLE m_hDevice;						//设备句柄
	bool m_bDeviceIsInit;							//设备是否已经初始化
	bool m_bDeviceIsAcquisition;					//设备是否已经开始采集
	int64_t m_nPayLoadSize;							//当前图像大小
public:
	int64_t m_nImageWidth;							//当前图像宽
	int64_t m_nImageHeight;							//当前图像高
	int m_nRoiOffsetX;
	int m_nRoiOffsetY;
};
	

#endif