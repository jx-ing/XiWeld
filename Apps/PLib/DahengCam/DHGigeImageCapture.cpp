#include "StdAfx.h"
#include "DHGigeImageCapture.h"
#include "CvvImage.h"
#include ".\Apps\PLib\BasicFunc\Const.h"

#define CHECK_FAILED_RETURN(_Status)\
{\
	GX_STATUS eStatus = _Status;\
	if (eStatus != GX_STATUS_SUCCESS)\
	{\
		SetErrorString(eStatus); \
		return false; \
	}\
}

#define CHECK_IS_OPEN if (!DeviceIsOpen())\
{\
	SetErrorString("相机尚未打开，不能执行：" +  CString(__FUNCSIG__)); \
	return false; \
}

#define CHECK_IS_INIT if (!DeviceIsInit())\
{\
	SetErrorString("相机尚未初始化，不能执行：" +  CString(__FUNCSIG__)); \
	return false; \
}

#define CHECK_IS_ACQ if (!DeviceIsAcq())\
{\
	SetErrorString("相机尚未开始采集，不能执行：" +  CString(__FUNCSIG__)); \
	return false; \
}

void T_GET_CB_IMAGE::ReleaseImage()
{
	if (NULL != this->pImage)
	{
		cvReleaseImage(&(this->pImage));
		this->pImage = NULL;
	}
}

void T_GET_CB_IMAGE::Clear()
{
	this->nImageID = -1;
	this->nImageState = -1;
	this->ullFrameID = 0;
	this->ullTimestamp = 0;
	this->llGetTime = 0;
	this->bGetImage = false;
	ReleaseImage();
}

void T_GET_CB_IMAGE::CreateImage(int nRoiWidth, int nRoiHeight)
{
	this->pImage = cvCreateImage(cvSize(nRoiWidth, nRoiHeight), IPL_DEPTH_8U, 1);
}

void T_GET_CB_IMAGE::SetImage(const GX_FRAME_DATA& tFrameData)
{
	this->nImageID++;
	this->nImageState = tFrameData.nStatus;
	this->ullFrameID = tFrameData.nFrameID;
	this->ullTimestamp = tFrameData.nTimestamp;
	this->llGetTime = XI_clock();
	this->bGetImage = true;
	memcpy(this->pImage->imageData, tFrameData.pImgBuf, this->pImage->imageSize);
}

void T_GET_CB_IMAGE::SetImage(const GX_FRAME_CALLBACK_PARAM& tFrameData)
{
	this->nImageID++;
	this->nImageState = tFrameData.status;
	this->ullFrameID = tFrameData.nFrameID;
	this->ullTimestamp = tFrameData.nTimestamp;
	this->llGetTime = XI_clock();
	this->bGetImage = true;
	memcpy(this->pImage->imageData, tFrameData.pImgBuf, this->pImage->imageSize);
}

//CDHGigeImageCapture::CDHGigeImageCapture()
//{
//	m_cLog = NULL;// XiBase::GetSystemLog();
//	m_pImageBuff = NULL;
//	memset(&m_tFrameData, 0, sizeof(GX_FRAME_DATA));
//	Clear();
//}

CDHGigeImageCapture::CDHGigeImageCapture(CLog* cLog, T_CAMREA_PARAM tCameraPara)
{
	m_pImageBuff = NULL;
	memset(&m_tFrameData, 0, sizeof(GX_FRAME_DATA));
	InitPara(cLog, tCameraPara);
}

CDHGigeImageCapture::~CDHGigeImageCapture()
{
	Clear();
}

void CDHGigeImageCapture::Clear()
{
	if (NULL != m_tFrameData.pImgBuf)
	{
		free(m_tFrameData.pImgBuf);
		m_tFrameData.pImgBuf = NULL;
	}
	if (NULL != m_pImageBuff)
	{
		cvReleaseImage(&m_pImageBuff);
		m_pImageBuff = NULL;
	}
	m_tLatestImage.Clear();

	m_sErrorInfo.Empty();

	m_hDevice = NULL;
	m_bDeviceIsInit = false;
	m_bDeviceIsAcquisition = false;

	m_nPayLoadSize = 0;
	m_nImageHeight = 0;
	m_nImageWidth = 0;
	memset(&m_tFrameData, 0, sizeof(GX_FRAME_DATA));
}

void CDHGigeImageCapture::InitPara(CLog* cLog, T_CAMREA_PARAM tCameraPara)
{
	Clear();
	m_pImageBuff = cvCreateImage(cvSize(tCameraPara.tDHCameraDriverPara.nRoiWidth, tCameraPara.tDHCameraDriverPara.nRoiHeight), IPL_DEPTH_8U, 1);
	m_tLatestImage.CreateImage(tCameraPara.tDHCameraDriverPara.nRoiWidth, tCameraPara.tDHCameraDriverPara.nRoiHeight);
	m_nImageHeight = tCameraPara.tDHCameraDriverPara.nRoiHeight;
	m_nImageWidth = tCameraPara.tDHCameraDriverPara.nRoiWidth;
	m_tCameraPara = tCameraPara;
	m_cLog = cLog;
}

bool CDHGigeImageCapture::InitLib()
{
	CHECK_FAILED_RETURN(GXInitLib());
	return true;
}

void CDHGigeImageCapture::CloseLib()
{
	GXCloseLib();
}

bool CDHGigeImageCapture::OpenDevice(unsigned int unTimeout)
{
	uint32_t  nDevNum = 0;

	//枚举设备
	CHECK_FAILED_RETURN(GXUpdateDeviceList(&nDevNum, unTimeout));

	//判断当前连接设备个数
	if (nDevNum <= 0)
	{
		SetErrorString("Error:未发现设备！");
		return false;
	}

	//如果设备已经打开则关闭，保证相机在初始化出错情况下能再次打开
	CloseDevice();

	//通过IP地址打开设备
	GX_OPEN_PARAM stOpenParam;
	stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
	stOpenParam.openMode = GX_OPEN_IP;
	stOpenParam.pszContent = (LPSTR)(LPCTSTR)(m_tCameraPara.tDHCameraDriverPara.strDeviceAddress);

	//打开设备
	GX_STATUS eStatus = GX_STATUS_ERROR;
	eStatus = GXOpenDevice(&stOpenParam, &m_hDevice);
	if (eStatus != GX_STATUS_SUCCESS)
	{
		SetErrorString(eStatus);
		m_hDevice = NULL;
		return false;
	}
	return true;
}

bool CDHGigeImageCapture::CloseDevice()
{
	if (DeviceIsOpen())
	{
		CHECK_FAILED_RETURN(GXCloseDevice(m_hDevice));
		m_hDevice = NULL;
	}
	m_bDeviceIsInit = false;
	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::DeviceIsOpen()
{
	return m_hDevice != NULL;
}

bool CDHGigeImageCapture::InitAfterOpen()
{
	//设置采集模式为连续模式
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS));

	//将图像数据格式设置为8Bit
	if (!SetPixelFormat8bit())
	{
		return false;
	}

	//设置心跳超时时间为1s
	//针对千兆网相机，程序在Debug模式下调试运行时，相机的心跳超时时间自动设置为5min，
	//这样做是为了不让相机的心跳超时影响程序的调试和单步执行，同时这也意味着相机在这5min内无法断开，除非使相机断电再上电
	//为了解决掉线重连问题，将相机的心跳超时时间设置为1s，方便程序掉线后可以重新连接
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_GEV_HEARTBEAT_TIMEOUT, 10000));

	//关闭帧率控制
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_OFF));

	//设置采集buffer个数
	CHECK_FAILED_RETURN(GXSetAcqusitionBufferNumber(m_hDevice, 30));

	//设置流通道包长
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_GEV_PACKETSIZE, 8192));

	//设置数据块超时时间
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_DS_INT_BLOCK_TIMEOUT, 1000));

	return true;
}

bool CDHGigeImageCapture::InitBeforeCapture()
{
	//获取相机当前参数
	if (!GetDeviceParam())
	{
		return false;
	}

	if (m_nPayLoadSize > 0)
	{
		//根据获取的图像buffer大小m_nPayLoadSize申请buffer
		if (NULL != m_tFrameData.pImgBuf)
		{
			free(m_tFrameData.pImgBuf);
			m_tFrameData.pImgBuf = NULL;
		}
		m_tFrameData.pImgBuf = malloc((size_t)m_nPayLoadSize);
		m_tLatestImage.ReleaseImage();
		m_tLatestImage.CreateImage(m_nImageWidth, m_nImageHeight);

		if (NULL != m_pImageBuff)
		{
			cvReleaseImage(&m_pImageBuff);
			m_pImageBuff = NULL;
		}
		m_pImageBuff = cvCreateImage(cvSize(m_nImageWidth, m_nImageHeight), IPL_DEPTH_8U, 1);
	}

	return true;
}

bool CDHGigeImageCapture::DeviceIsInit()
{
	return DeviceIsOpen() && m_bDeviceIsInit;
}

bool CDHGigeImageCapture::StartAcquisition(E_DHGIGE_CALL_BACK eCallBack)
{
	m_eCallBack = eCallBack;
	return StartAcquisition();
}

bool CDHGigeImageCapture::StartAcquisition()
{
	CHECK_IS_INIT;
	if (m_bDeviceIsAcquisition)
	{
		if (!StopAcquisition(true))
		{
			return false;
		}
		Sleep(30);
	}

	m_bDeviceIsAcquisition = false;
	m_tLatestImage.nImageID = 0;
	ClearCallBackImage();

	if (!InitBeforeCapture())
	{
		return false;
	}

	if (E_CALL_BACK_MODE_WAIT_IMAGE == m_eCallBack
		|| E_CALL_BACK_MODE_SAVE_IMAGE == m_eCallBack)
	{
		//如果注册需要消耗时间，再考虑只注册一次的问题
		CHECK_FAILED_RETURN(GXRegisterCaptureCallback(m_hDevice, this, ContiCapCallbackFun));
	}
	CHECK_FAILED_RETURN(GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_START));
	m_bDeviceIsAcquisition = true;
	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::StopAcquisition(bool bClearCallBackImage)
{
	if (!DeviceIsAcq())
	{
		SetErrorString();
		return true;
	}
	//m_nAcqWay = -1;
	m_bDeviceIsAcquisition = false;
	CHECK_FAILED_RETURN(GXSendCommand(m_hDevice, GX_COMMAND_ACQUISITION_STOP));

	if (GX_TRIGGER_MODE_ON != m_eTriggerMode
		|| GX_TRIGGER_SOURCE_SOFTWARE != m_eTriggerSource)//如果没打开软触发
	{
		CHECK_FAILED_RETURN(GXUnregisterCaptureCallback(m_hDevice));
	}
	if(bClearCallBackImage)
	{
		ClearCallBackImage();
	}
	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::DeviceIsAcq()
{
	return DeviceIsInit() && m_bDeviceIsAcquisition;
}

bool CDHGigeImageCapture::SetTriggerMode(int nTriggerSrc)
{
	switch (nTriggerSrc)
	{
	case GX_TRIGGER_SOURCE_SOFTWARE:
		m_eCameraMode = (E_DHGIGE_ACQUISITION_MODE)nTriggerSrc;
		return SetTriggerMode(GX_TRIGGER_MODE_ON, GX_TRIGGER_SOURCE_SOFTWARE);
	case GX_TRIGGER_SOURCE_LINE0:
		m_eCameraMode = (E_DHGIGE_ACQUISITION_MODE)nTriggerSrc;
		return SetTriggerMode(GX_TRIGGER_MODE_ON, GX_TRIGGER_SOURCE_LINE0);
	case GX_TRIGGER_SOURCE_LINE1:
		m_eCameraMode = (E_DHGIGE_ACQUISITION_MODE)nTriggerSrc;
		return SetTriggerMode(GX_TRIGGER_MODE_ON, GX_TRIGGER_SOURCE_LINE1);
	case GX_TRIGGER_SOURCE_LINE2:
		m_eCameraMode = (E_DHGIGE_ACQUISITION_MODE)nTriggerSrc;
		return SetTriggerMode(GX_TRIGGER_MODE_ON, GX_TRIGGER_SOURCE_LINE2);
	case GX_TRIGGER_SOURCE_LINE3:
		m_eCameraMode = (E_DHGIGE_ACQUISITION_MODE)nTriggerSrc;
		return SetTriggerMode(GX_TRIGGER_MODE_ON, GX_TRIGGER_SOURCE_LINE3);
	default:
		m_eCameraMode = E_ACQUISITION_MODE_CONTINUE;
		return SetTriggerMode(GX_TRIGGER_MODE_OFF, GX_TRIGGER_SOURCE_SOFTWARE);
	}
	return false;
}

bool CDHGigeImageCapture::SetTriggerMode(GX_TRIGGER_MODE_ENTRY eTriggerMode, GX_TRIGGER_SOURCE_ENTRY eTriggerSource)
{
	CHECK_IS_INIT;
	if (DeviceIsAcq())
	{
		SetErrorString("请先停止采集，再更改触发模式");
		return false;
	}

	m_eTriggerMode = eTriggerMode;
	m_eTriggerSource = eTriggerSource;

	if (eTriggerMode == GX_TRIGGER_MODE_ON)
	{
		//设置触发模式为开
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_ON));
		// 设置触发类型
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_SELECTOR, GX_ENUM_TRIGGER_SELECTOR_FRAME_START));

		if (eTriggerSource != GX_TRIGGER_SOURCE_SOFTWARE)
		{
			//设置为上升沿触发
			CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_ACTIVATION, GX_TRIGGER_ACTIVATION_RISINGEDGE));
			//设置上升沿滤波最小值
			CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_TRIGGER_FILTER_RAISING, 50));
			//设置下降沿滤波最小值
			CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_TRIGGER_FILTER_FALLING, 50));
		}
		//选择触发源
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_SOURCE, eTriggerSource));
	}
	else
	{
		//设置触发模式为关
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_TRIGGER_MODE, GX_TRIGGER_MODE_OFF));
	}
	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::SetROI(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY)
{
	CHECK_IS_INIT;
	if (DeviceIsAcq())
	{
		SetErrorString("请先停止采集，再更改ROI");
		return false;
	}

	GXSetInt(m_hDevice, GX_INT_OFFSET_X, nRoiOffsetX);
	GXSetInt(m_hDevice, GX_INT_OFFSET_Y, nRoiOffsetY);
	GXSetInt(m_hDevice, GX_INT_WIDTH, nRoiWidth);
	GXSetInt(m_hDevice, GX_INT_HEIGHT, nRoiHeight);

	//以下两行是解决当宽高变大，偏移值变小时，以上代码一次设置不完全的情况。
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_OFFSET_X, nRoiOffsetX));
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_OFFSET_Y, nRoiOffsetY));
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_WIDTH, nRoiWidth));
	CHECK_FAILED_RETURN(GXSetInt(m_hDevice, GX_INT_HEIGHT, nRoiHeight));

	m_nImageWidth = nRoiWidth;
	m_nImageHeight = nRoiHeight;
	m_nRoiOffsetX = nRoiOffsetX;
	m_nRoiOffsetY = nRoiOffsetY;

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::ChangeFrame(double dFrame)
{
	CHECK_IS_INIT;
	if (DeviceIsAcq())
	{
		SetErrorString("请先停止采集，再更改帧率");
		return false;
	}

	if (dFrame > 0.0)
	{
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON));
		CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, dFrame));
	}
	else
	{
		CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_OFF));
		//CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, dFrame));
	}

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::SetGain(double dGainLevel)
{
	CHECK_IS_INIT;

	if (dGainLevel >= 0)
	{
		CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_GAIN, dGainLevel));
	}
	else
	{
		CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_GAIN, m_tCameraPara.tDHCameraDriverPara.dGainLevel));
	}

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::SetExposure(double dExposureTime)
{
	CHECK_IS_INIT;

	if (dExposureTime > 0)
	{
		CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, dExposureTime));
	}
	else
	{
		CHECK_FAILED_RETURN(GXSetFloat(m_hDevice, GX_FLOAT_EXPOSURE_TIME, m_tCameraPara.tDHCameraDriverPara.dExposureTime));
	}

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::OpenStrobe(int nLineNum)
{
	CHECK_IS_INIT;

	GX_LINE_SELECTOR_ENTRY gLineNum;
	if (!GetLineNum(nLineNum, gLineNum))
	{
		return false;
	}
	if (gLineNum == GX_ENUM_LINE_SELECTOR_LINE1)
	{
		SetErrorString("IO端口1不能作为爆闪灯信号输出口");
		return false;
	}
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_SELECTOR, gLineNum));//选择端口
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_OUTPUT));//设为输出
	CHECK_FAILED_RETURN(GXSetBool(m_hDevice, GX_BOOL_LINE_INVERTER, false));//不反转
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_STROBE));//爆闪灯模式

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::CloseStrobe(int nLineNum)
{
	CHECK_IS_INIT;

	GX_LINE_SELECTOR_ENTRY gLineNum;
	if (!GetLineNum(nLineNum, gLineNum))
	{
		return false;
	}

	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_SELECTOR, gLineNum));
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_SOURCE, GX_ENUM_LINE_SOURCE_OFF));

	SetErrorString();
	return true;
}

bool CDHGigeImageCapture::SetIOForTrigger(int nLineNum)
{
	CHECK_IS_INIT;

	GX_LINE_SELECTOR_ENTRY gLineNum;
	if (!GetLineNum(nLineNum, gLineNum))
	{
		return false;
	}

	if (gLineNum == GX_ENUM_LINE_SELECTOR_LINE1)
	{
		SetErrorString("IO端口1不能作为硬触发信号输入口");
		return false;
	}
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_SELECTOR, gLineNum));//选中指定IO
	CHECK_FAILED_RETURN(GXSetEnum(m_hDevice, GX_ENUM_LINE_MODE, GX_ENUM_LINE_MODE_INPUT));//设置为输入
	CHECK_FAILED_RETURN(GXSetBool(m_hDevice, GX_BOOL_LINE_INVERTER, false));//引脚反转

	SetErrorString();	
	return true;
}

bool CDHGigeImageCapture::FlushQueue()
{
	CHECK_IS_INIT;
	CHECK_FAILED_RETURN(GXFlushQueue(m_hDevice));
	SetErrorString();
	return true;
}

void CDHGigeImageCapture::SetCaptureImageState(bool bState)
{
	m_tLatestImage.bGetImage = bState;
}

bool CDHGigeImageCapture::CaptureImage(int nTryTimes, unsigned int unTimeout)
{
	CHECK_IS_ACQ;

	if (m_eTriggerMode != GX_TRIGGER_MODE_ON)
	{
		SetErrorString("CaptureImage只支持触发模式！");
		return false;
	}

	bool bRtn = false;

	while (nTryTimes > 0)
	{
		//软触发
		if (m_eTriggerSource == GX_TRIGGER_SOURCE_SOFTWARE)
		{
			bRtn = CaptureImage_Software(unTimeout);
		}
		//硬触发或无触发
		else
		{
			bRtn = CaptureImage_Hard(unTimeout);
		}
		if (bRtn)
		{
			SetErrorString();
			return bRtn;
		}
		nTryTimes--;
		Sleep(100);
	}
	return false;
}

bool CDHGigeImageCapture::CaptureImage(IplImage*& pImage, int nTryTimes, unsigned int unTimeout)
{
	bool bRtn = CaptureImage(nTryTimes, unTimeout);
	if (!pImage)
	{
		pImage = cvCreateImage(cvSize(m_nImageWidth, m_nImageHeight), IPL_DEPTH_8U, 1);
	}
	if (bRtn)
	{
		cvCopyImage(m_tLatestImage.pImage, pImage);
	}
	else
	{
		cvZero(pImage);
	}
	return bRtn;
}

void CDHGigeImageCapture::ClearCallBackImage()
{
	for (int nNum = 0; nNum < m_vtCallBackImage.size(); nNum++)
	{
		if (m_vtCallBackImage[nNum] != NULL)
		{
			if (m_vtCallBackImage[nNum]->pImage != NULL)
			{
				cvReleaseImage(&(m_vtCallBackImage[nNum]->pImage));
				m_vtCallBackImage[nNum]->pImage = NULL;
			}
			delete m_vtCallBackImage[nNum];
			m_vtCallBackImage[nNum] = NULL;
		}
	}
	m_vtCallBackImage.clear();
	m_vtCallBackImage.reserve(2000);
}

int CDHGigeImageCapture::ReadCapInfo(bool bIsShowInfo, double* pdCurFrameRate)
{
	CHECK_IS_INIT;

	int64_t nBufferCount = 0;
	int64_t nAllFrameCount = 0;
	int64_t nLostFrameCount = 0;
	int64_t nIncompleteFrameCount = 0;
	double dCurFrameRate = 0.0;
//	GX_STATUS emStatus;
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_DS_INT_ANNOUNCED_BUFFER_COUNT, &nBufferCount));
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_DS_INT_DELIVERED_FRAME_COUNT, &nAllFrameCount));
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_DS_INT_LOST_FRAME_COUNT, &nLostFrameCount));
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_DS_INT_INCOMPLETE_FRAME_COUNT, &nIncompleteFrameCount));
	CHECK_FAILED_RETURN(GXGetFloat(m_hDevice, GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE, &dCurFrameRate));
	WriteLog("Buffer数量：%lld, 总帧数：%lld, 丢帧数：%lld, 残帧数：%lld,当前帧率：%.3lf",
		nBufferCount, nAllFrameCount, nLostFrameCount, nIncompleteFrameCount, dCurFrameRate);
	if (true == bIsShowInfo)
	{
		//已修改
		XUI::MesBox::PopInfo("Buffer数量:{0}, 总帧数：{1}, 丢帧数：{2}, 残帧数：{3},当前帧率：{4}",
			nBufferCount, nAllFrameCount, nLostFrameCount, nIncompleteFrameCount, dCurFrameRate);
		//XiMessageBoxOk("Buffer数量：%lld, 总帧数：%lld, 丢帧数：%lld, 残帧数：%lld,当前帧率：%.3lf",
			//nBufferCount, nAllFrameCount, nLostFrameCount, nIncompleteFrameCount, dCurFrameRate);
	}
	if (NULL != pdCurFrameRate)
	{
		*pdCurFrameRate = dCurFrameRate;
	}
	return nAllFrameCount;
}

bool CDHGigeImageCapture::GetCurrentFrameRate(double& dCurrentFrameRate)
{
	dCurrentFrameRate = 0.0;
	CHECK_IS_INIT;
	CHECK_FAILED_RETURN(GXGetFloat(m_hDevice, GX_FLOAT_CURRENT_ACQUISITION_FRAME_RATE, &dCurrentFrameRate));

	SetErrorString();
	return true;
}

CvPoint CDHGigeImageCapture::GetImageCenter()
{
	return cvPoint(m_tCameraPara.tDHCameraDriverPara.nMaxWidth / 2.0,
		m_tCameraPara.tDHCameraDriverPara.nMaxHeight / 2.0);
}

void CDHGigeImageCapture::ShowErrorString()
{
	if (!m_sErrorInfo.IsEmpty())
	{
		XUI::MesBox::PopError((const char*)m_sErrorInfo);
	}
}

CString CDHGigeImageCapture::GetErrorString()
{
	return m_sErrorInfo;
}


void CDHGigeImageCapture::Distortion(cv::Mat mSrc, cv::Mat& mDst)
{
	if (!m_tCameraPara.tCameraDistortion.bEnableUndistort)
	{
		mSrc.copyTo(mDst);
		return;
	}
	cv::Mat new_matrix; 
	cv::undistort(mSrc, mDst, 
		m_tCameraPara.tCameraDistortion.cvCameraMatrix, 
		m_tCameraPara.tCameraDistortion.cvDistCoeffs,
		new_matrix);
}

void CDHGigeImageCapture::Distortion(std::vector<CvPoint> mSrc, std::vector<CvPoint>& mDst)
{
	if (!m_tCameraPara.tCameraDistortion.bEnableUndistort)
	{
		mDst = mSrc;
		return;
	}
	if (mSrc.size() <= 0)
	{
		return;
	}
	cv::Point2f pnTemp;
	std::vector<cv::Point2f>pntSrc, pntDst;
	pntSrc.reserve(10000);
	pntDst.reserve(10000);
	for (int n = 0; n < mSrc.size(); n++)
	{
		pnTemp = mSrc[n];
		pntSrc.push_back(pnTemp);
	}

	cv::undistortPoints(pntSrc, pntDst, 
		m_tCameraPara.tCameraDistortion.cvCameraMatrix, 
		m_tCameraPara.tCameraDistortion.cvDistCoeffs,
		cv::Mat(), m_tCameraPara.tCameraDistortion.cvCameraMatrix);
	mDst.clear();
	for (int n = 0; n < pntDst.size(); n++)
	{
		CvPoint pntTemp;
		pntTemp.x = (int)(pntDst[n].x + 0.5);
		pntTemp.y = (int)(pntDst[n].y + 0.5);
		mDst.push_back(pntTemp);
	}
	pntSrc.clear();
	pntDst.clear();
}

void CDHGigeImageCapture::Distortion(IplImage* pImgGray, IplImage* pResultImgGray)
{
	if (!m_tCameraPara.tCameraDistortion.bEnableUndistort)
	{
		cvCopyImage(pImgGray, pResultImgGray);
		return;
	}
	IplImage* src;
	cv::Mat mSrc(pImgGray, true);
	cv::Mat mDst;
	Distortion(mSrc, mDst);
	src = &IplImage(mDst);
	cvCopyImage(src, pResultImgGray);
	mSrc.release();
	mDst.release();
	mSrc = NULL;
	mDst = NULL;
	src = NULL;
}

bool CDHGigeImageCapture::InitCam(E_DHGIGE_ACQUISITION_MODE eCameraMode, E_DHGIGE_CALL_BACK eCallBack)
{
	return InitCam(m_tCameraPara.tDHCameraDriverPara.nRoiWidth, m_tCameraPara.tDHCameraDriverPara.nRoiHeight,
		m_tCameraPara.tDHCameraDriverPara.nRoiOffsetX, m_tCameraPara.tDHCameraDriverPara.nRoiOffsetY, eCameraMode, eCallBack);
}

bool CDHGigeImageCapture::InitCam(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY, E_DHGIGE_ACQUISITION_MODE eCameraMode, E_DHGIGE_CALL_BACK eCallBack)
{
	if (m_bDeviceIsInit)
	{
		SetErrorString();
		return true;
	}

	m_eCallBack = eCallBack;

	//打开相机
	m_bDeviceIsInit = false;
	bool bRtn = InitLib();
	bRtn = bRtn && OpenDevice();
	bRtn = bRtn && InitAfterOpen();
	m_bDeviceIsInit = true;

	//初始化部分参数
	bRtn = bRtn && SetROI(nRoiWidth, nRoiHeight, nRoiOffsetX, nRoiOffsetY);//设置ROI
	bRtn = bRtn && SetGain();//设置增益
	bRtn = bRtn && SetExposure();//设置曝光
	bRtn = bRtn && SetTriggerMode((int)eCameraMode);//设置触发模式
	if (E_ACQUISITION_MODE_CONTINUE == eCameraMode && E_CALL_BACK_MODE_SAVE_IMAGE == eCallBack)
	{
		bRtn = bRtn && ChangeFrame(CAPTURE_IMAGE_FRAME_RATE);//设置帧率
	}

	if (!bRtn)
	{
		CloseDevice();
		m_bDeviceIsInit = false;
	}
	else
	{
		SetErrorString();
	}
	return bRtn;
}

bool CDHGigeImageCapture::UnInitCam()
{
	return StopAcquisition(true)
		&& CloseDevice();
}

void CDHGigeImageCapture::SetErrorString(GX_STATUS emErrorStatus)
{
	char* pchErrorInfo = NULL;
	size_t    nSize = 0;
	GX_STATUS emStatus = GX_STATUS_ERROR;

	//获取错误信息长度，并申请内存空间
	emStatus = GXGetLastError(&emErrorStatus, NULL, &nSize);
	pchErrorInfo = new char[nSize];
	if (pchErrorInfo != NULL)
	{
		//获取错误信息描述，并显示
		emStatus = GXGetLastError(&emErrorStatus, pchErrorInfo, &nSize);
		if (emStatus != GX_STATUS_SUCCESS)
		{
			SetErrorString("Error:GXGetLastError接口调用失败！");
		}
		else
		{
			CString str;
			str.Format("Error:%s", pchErrorInfo);
			SetErrorString(str);
		}

		//释放申请的内存空间
		if (NULL != pchErrorInfo)
		{
			delete[] pchErrorInfo;
			pchErrorInfo = NULL;
		}
	}
}

void CDHGigeImageCapture::SetErrorString(CString sError)
{
	m_cLog->Write(sError);
	m_sErrorInfo = sError;
}

void __stdcall CDHGigeImageCapture::ContiCapCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
	CDHGigeImageCapture* pMyObj = (CDHGigeImageCapture*)(pFrame->pUserParam);

	switch (pMyObj->m_eCallBack)
	{
	case E_CALL_BACK_MODE_WAIT_IMAGE: pMyObj->GetImageFromCam_Single(pFrame); break;
	case E_CALL_BACK_MODE_SAVE_IMAGE: pMyObj->GetImageFromCam_Continuous(pFrame); break;
	}
}

bool CDHGigeImageCapture::GetImageFromCam_Continuous(GX_FRAME_CALLBACK_PARAM* pFrame)
{
	if (NULL == pFrame)
	{
		m_cLog->Write("第 %d 采图失败，回调函数返回空指针！", m_tLatestImage.nImageID);
		return false;
	}
	if (pFrame->status != 0)
	{
		m_cLog->Write("第 %d 采图失败，回调函数返回残帧！", m_tLatestImage.nImageID);
		//return false;
	}

	//复制图片数据
	IplImage* pImage = m_tLatestImage.pImage;
	T_GET_CB_IMAGE* tNewImage = new T_GET_CB_IMAGE();
	*tNewImage = m_tLatestImage;
	tNewImage->CreateImage(m_nImageWidth, m_nImageHeight);
	tNewImage->SetImage(*pFrame);
	m_tLatestImage = *tNewImage;
	m_tLatestImage.pImage = pImage;
	//保存到容器里
	m_vtCallBackImage.push_back(tNewImage);

	return true;
}

bool CDHGigeImageCapture::GetImageFromCam_Single(GX_FRAME_CALLBACK_PARAM* pFrame)
{
	if (NULL == pFrame)
	{
		m_cLog->Write("第 %d 采图失败，回调函数返回空指针！", m_tLatestImage.nImageID);
		return false;
	}
	if (pFrame->status != 0)
	{
		m_cLog->Write("第 %d 采图失败，回调函数返回残帧！", m_tLatestImage.nImageID);
		//return false;
	}

	//复制图片数据
	m_tLatestImage.SetImage(*pFrame);
	return true;
}

bool CDHGigeImageCapture::CaptureImage_Software(unsigned int unTimeout)
{
	GX_STATUS nStatus = GX_STATUS_ERROR;
	m_tLatestImage.bGetImage = false;

	//清空采集输出队列
	if (!FlushQueue())
	{
		return false;
	}

	//发送软触发命令
	int nSendCount = 0;
	while (nStatus != GX_STATUS_SUCCESS)
	{
		if (nSendCount > 2)
		{
			SetErrorString("发送软触发命令超时");
			return false;
		}
		nStatus = GXSendCommand(m_hDevice, GX_COMMAND_TRIGGER_SOFTWARE);
		nSendCount++;
		Sleep(10);
	}

	//获取图片
	if (m_bDeviceIsInit)
	{		
		if (m_eCallBack == E_CALL_BACK_MODE_OFF)//回调函数关
		{
			if (GXGetImage(m_hDevice, &m_tFrameData, 2000) != GX_STATUS_SUCCESS)
			{
				SetErrorString("软触发 回调采图超时");
				return false;
			}
			m_tLatestImage.SetImage(m_tFrameData);
			return true;
		}
		else//回调函数开
		{
			long long llStartTime = XI_clock();
			while (true)
			{
				Sleep(5);
				if (m_tLatestImage.bGetImage == true)
				{
					break;
				}
				if (unTimeout < XI_clock() - llStartTime)
				{
					SetErrorString("软触发 回调采图超时");
					return false;
				}
			}
		}
	}
	else
	{
		SetErrorString("必须先初始化相机才可以采集图片");
		return false;
		//nStatus = GXGetImage(m_hDevice, &m_tFrameData, unTimeout);
		//CHECK_FAILED_RETURN(nStatus);
		//m_tLatestImage.SetImage(m_tFrameData);
	}	
	
	//清空采集输出队列
	//if (!FlushQueue())
	//{
	//	return false;
	//}
	return true;
}

bool CDHGigeImageCapture::CaptureImage_Hard(unsigned int unTimeout)
{
	GX_STATUS nStatus = GX_STATUS_ERROR;

	//获取图片
	if (m_bDeviceIsInit)
	{		
		if (m_eCallBack == E_CALL_BACK_MODE_OFF)//回调函数关
		{
			if (GXGetImage(m_hDevice, &m_tFrameData, 2000) != GX_STATUS_SUCCESS)
			{
				SetErrorString("软触发 回调采图超时");
				return false;
			}
			m_tLatestImage.SetImage(m_tFrameData);
			return true;

		}
		else//回调函数开
		{
			long long llStartTime = XI_clock();
			while (true)
			{
				Sleep(5);
				if (m_tLatestImage.bGetImage == true)
				{
					break;
				}
				if (unTimeout < XI_clock() - llStartTime)
				{
					SetErrorString("硬触发 回调采图超时");
					return false;
				}
			}
		}
	}
	else
	{
		SetErrorString("必须先初始化相机才可以采集图片");
		return false;
		//nStatus = GXGetImage(m_hDevice, &m_tFrameData, unTimeout);
		//CHECK_FAILED_RETURN(nStatus);
		//m_tLatestImage.SetImage(m_tFrameData);
	}
	return true;
}

bool CDHGigeImageCapture::GetLineNum(int nLineNum, GX_LINE_SELECTOR_ENTRY& gLineNum)
{
	if (nLineNum >= 0 && nLineNum <= 3)
	{
		gLineNum = (GX_LINE_SELECTOR_ENTRY)nLineNum;
		return true;
	}
	gLineNum = GX_ENUM_LINE_SELECTOR_LINE1;
	SetErrorString("相机IO端口只有0、1、2、3四种，其余端口无效");
	return false;
}

bool CDHGigeImageCapture::GetDeviceParam()
{
	//获取图像大小
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_INT_PAYLOAD_SIZE, &m_nPayLoadSize));

	//获取宽度
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_INT_WIDTH, &m_nImageWidth));

	//获取高度
	CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_INT_HEIGHT, &m_nImageHeight));

	if (m_nImageWidth != m_tCameraPara.tDHCameraDriverPara.nRoiWidth
		|| m_nImageHeight != m_tCameraPara.tDHCameraDriverPara.nRoiHeight)
	{
		m_cLog->Write("第一次获取的图片宽高与初始化的不同，尝试再次获取");
		Sleep(50);
		//获取宽度
		CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_INT_WIDTH, &m_nImageWidth));
		//获取高度
		CHECK_FAILED_RETURN(GXGetInt(m_hDevice, GX_INT_HEIGHT, &m_nImageHeight));
	}

	return true;
}

bool CDHGigeImageCapture::SetPixelFormat8bit()
{
	GX_STATUS emStatus    = GX_STATUS_ERROR;
	int64_t   nPixelSize  = 0;
	uint32_t  nEnmuEntry  = 0;
	size_t    nBufferSize = 0;
	
	GX_ENUM_DESCRIPTION  *pEnumDescription = NULL;
	
	//获取像素位深大小
	CHECK_FAILED_RETURN(GXGetEnum(m_hDevice, GX_ENUM_PIXEL_SIZE, &nPixelSize));
	
	//判断为8bit时直接返回,否则设置为8bit
	if (nPixelSize == GX_PIXEL_SIZE_BPP8)
	{
		return true;
	}
	else
	{
		//获取设备支持的像素格式的枚举项个数
		CHECK_FAILED_RETURN(GXGetEnumEntryNums(m_hDevice, GX_ENUM_PIXEL_FORMAT, &nEnmuEntry));
		
		//为获取设备支持的像素格式枚举值准备资源
		nBufferSize      = nEnmuEntry * sizeof(GX_ENUM_DESCRIPTION);
		pEnumDescription = new GX_ENUM_DESCRIPTION[nEnmuEntry];
		
		//获取支持的枚举值
		emStatus = GXGetEnumDescription(m_hDevice, GX_ENUM_PIXEL_FORMAT, pEnumDescription, &nBufferSize);
		if (emStatus != GX_STATUS_SUCCESS)
		{
			if (pEnumDescription != NULL)
			{
				delete []pEnumDescription;
				pEnumDescription = NULL;
			}
			SetErrorString(emStatus);
			return false;
		}

		//遍历设备支持的像素格式,设置像素格式为8bit,
		//如设备支持的像素格式为Mono10和Mono8则设置其为Mono8
		for (uint32_t i = 0; i<nEnmuEntry; i++)
		{
			if ((pEnumDescription[i].nValue & GX_PIXEL_8BIT) == GX_PIXEL_8BIT)
			{
				emStatus = GXSetEnum(m_hDevice, GX_ENUM_PIXEL_FORMAT, pEnumDescription[i].nValue);
				break;
			}
		}	
		
		//释放资源
		if (pEnumDescription != NULL)
		{
			delete []pEnumDescription;
			pEnumDescription = NULL;
		}
	}
	CHECK_FAILED_RETURN(emStatus);
	return true;
}
