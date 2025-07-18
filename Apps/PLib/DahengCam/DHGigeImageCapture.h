/*********************************************************************
* ��Ȩ���� (C)2019, �������н������˼������޹�˾
**********************************************************************/

//�������ַ�ʽ����Ϊ�����ɼ�

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
	E_ACQUISITION_MODE_SOURCE_SOFTWARE = 0,		//����ģʽ
	E_ACQUISITION_MODE_SOURCE_HARD_0 = 1,		//Ӳ����ģʽ
	E_ACQUISITION_MODE_SOURCE_HARD_1 = 2,		//Ӳ����ģʽ
	E_ACQUISITION_MODE_SOURCE_HARD_2 = 3,		//Ӳ����ģʽ
	E_ACQUISITION_MODE_SOURCE_HARD_3 = 4,		//Ӳ����ģʽ
	E_ACQUISITION_MODE_CONTINUE = 5,			//����ģʽ
	E_DHGIGE_ACQUISITION_MODE_MAX_NUM,
}E_DHGIGE_ACQUISITION_MODE;

typedef enum
{
	E_CALL_BACK_MODE_OFF = 0,			//�رջص�����
	E_CALL_BACK_MODE_WAIT_IMAGE,		//�ȴ��ص�����
	E_CALL_BACK_MODE_SAVE_IMAGE,		//�ص�����ͼƬ
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

//ͼƬ��Ϣ
struct T_GET_CB_IMAGE
{
	int nImageID			= -1;	//ͼƬID������������
	int nImageState			= -1;	//ͼƬ״̬������ڲ���ȡ��
	uint64_t ullFrameID		= 0;    //ͼƬ��֡�ţ�����ڲ���ȡ��
	uint64_t ullTimestamp	= 0;	//ͼƬʱ���������ڲ���ȡ��
	long long llGetTime		= 0;	//ͼƬ��ȡʱ�䣨��ȡͼƬʱ��Xi_clock()�������ɣ�
	bool bGetImage			= false;//�Ƿ�ɹ���ȡͼƬ
	IplImage *pImage		= NULL;	//ͼƬ����

	//�ͷ�ͼƬ����
	void ReleaseImage();
	//����ȫ������
	void Clear();
	//����һ���µ�ͼƬ������
	void CreateImage(int nRoiWidth, int nRoiHeight);
	//��������
	void SetImage(const GX_FRAME_DATA& tFrameData);
	void SetImage(const GX_FRAME_CALLBACK_PARAM& tFrameData);
};

class CDHGigeImageCapture
{
public:
	//CDHGigeImageCapture();//С������ʱ���ô˹��캯��
	CDHGigeImageCapture(CLog* cLog, T_CAMREA_PARAM tCameraPara);
	virtual ~CDHGigeImageCapture();

	/************************** ��ʼ�� **************************/
	//��ʼ������
	void InitPara(CLog* cLog, T_CAMREA_PARAM tCameraPara);

	T_CAMREA_PARAM m_tCameraPara;					//�������

	/************************* ������� *************************/
	//��ʼ�����
	//nTriggerSrc��
	//		0��������
	//		1��2��3��4����Ӧ�˿�Ӳ������
	//		�������޴���
	bool InitCam(E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF);
	bool InitCam(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY, E_DHGIGE_ACQUISITION_MODE eCameraMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, E_DHGIGE_CALL_BACK eCallBack = E_CALL_BACK_MODE_OFF);
	//�ͷ������Դ
	bool UnInitCam();
	//�ͷſ⣨���һ������˳�ʱҪ���ã�
	void CloseLib();

	E_DHGIGE_ACQUISITION_MODE m_eCameraMode;
	E_DHGIGE_CALL_BACK m_eCallBack;

	/************************* ���״̬ *************************/
	//�ж�����Ƿ��Ѿ���
	bool DeviceIsOpen();
	//�ж�����Ƿ��Ѿ���ʼ��
	bool DeviceIsInit();
	//�ж�����Ƿ��Ѿ���ʼ�ɼ�
	bool DeviceIsAcq();

	/************************* ������� *************************/
	//���ô���ģʽ��ֻ���ڿ�ʼ�ɼ�֮ǰ���ã�
	//0������
	//1��2��3��4��Ӳ����
	//�������޴���
	bool SetTriggerMode(int nTriggerSrc);
	//����ROI��ֻ���ڿ�ʼ�ɼ�֮ǰ���ã�
	bool SetROI(int nRoiWidth, int nRoiHeight, int nRoiOffsetX, int nRoiOffsetY);
	//����֡�ʣ�ֻ���ڿ�ʼ�ɼ�֮ǰ���ã�
	bool ChangeFrame(double dFrame);
	//��������
	bool SetGain(double dGainLevel = -1.0);
	//�����ع�ʱ��
	bool SetExposure(double dExposureTime = -1.0);
	//���������ģʽ
	bool OpenStrobe(int nLineNum);
	//�ر������ģʽ
	bool CloseStrobe(int nLineNum);
	//����Ӳ����IO
	bool SetIOForTrigger(int nLineNum);

	/************************* ��ͼͨ�� *************************/
	//��ʼ�ɼ����������ֲ�ͼ��ʽ����Ҫ�ȵ��ô˺�����
	bool StartAcquisition(E_DHGIGE_CALL_BACK eCallBack);
	bool StartAcquisition();
	//ֹͣ�ɼ�
	bool StopAcquisition(bool bClearCallBackImage = true);

	/************************* ���Ų�ͼ *************************/
	/********************** ��֧�ִ���ģʽ **********************/
	//��ղɼ�������У�Ӳ����֮ǰ�ɿ��ǵ��ã��������Զ����ã�
	bool FlushQueue();
	//���ò�ͼ״̬������Ӳ����֮ǰ�������һ�Σ�
	void SetCaptureImageState(bool bState);
	//��ȡ����ͼ�񣨴���ģʽ��
	bool CaptureImage(int nTryTimes, unsigned int unTimeout = 2000);
	//��ȡ����ͼ�񣨴���ģʽ��
	bool CaptureImage(IplImage*& pImage, int nTryTimes, unsigned int unTimeout = 2000);

	IplImage* m_pImageBuff;							//ͼƬ����(�ⲿʹ�ÿ����ͷ�)
	T_GET_CB_IMAGE m_tLatestImage;					//ͼƬ����(�ⲿʹ�ò����ͷ�)

	/************************* ������ͼ *************************/
	//����ص��ɼ���������
	void ClearCallBackImage();

	std::vector<T_GET_CB_IMAGE*> m_vtCallBackImage;	//ͼƬ���棨���ڻ�ȡ�����ǰͼƬ��

	/************************* �豸��Ϣ *************************/
	//��ȡ�����֡��
	int ReadCapInfo(bool bIsShowInfo, double* pdCurFrameRate = NULL);
	//��ȡ֡��
	bool GetCurrentFrameRate(double& dCurrentFrameRate);
	//��ȡͼ�����ĵ�����
	CvPoint GetImageCenter();

	/************************* �쳣��Ϣ *************************/
	//������ʾ�쳣��Ϣ
	void ShowErrorString();
	//��ȡ�쳣��Ϣ
	CString GetErrorString();

	/************************* ͼƬ���� *************************/
	//ȥ����
	void Distortion(cv::Mat mSrc, cv::Mat& mDst);
	void Distortion(IplImage* pImgGray, IplImage* pResultImgGray);
	void Distortion(std::vector<CvPoint> mSrc, std::vector<CvPoint>& mDst);

private:
	/************************* �쳣���¼ *************************/
	//�������������쳣��Ϣ
	void SetErrorString(GX_STATUS emErrorStatus);
	//�����쳣��Ϣ
	void SetErrorString(CString sError = "");

	CString m_sErrorInfo;							//�쳣��Ϣ
	CLog* m_cLog;									//��־

	/************************* ������� *************************/
	//������е�ȫ������ռ䣬����ȫ����Ա����
	void Clear();

	GX_TRIGGER_MODE_ENTRY m_eTriggerMode;			//����ģʽ
	GX_TRIGGER_SOURCE_ENTRY m_eTriggerSource;		//����Դ
	//int m_nAcqWay;									//�ɼ���ʽ��1���ɼ����ţ�2���ɼ����ţ���

	/************************* ���ݻ��� *************************/
	GX_FRAME_DATA m_tFrameData;						//ͼƬ���棨�����ʽ�����ڻ�ȡ�����ǰͼƬ��

	/************************* ������״̬ *************************/
	//��ʼ����
	bool InitLib();
	//�����
	bool OpenDevice(unsigned int unTimeout = 1000);
	//�ر����
	bool CloseDevice();
	//�ڴ����֮���趨һЩ����
	bool InitAfterOpen();
	//�ڿ�ʼ�ɼ�֮ǰ���������ǰ�����趨������
	bool InitBeforeCapture();
	//���ô���ģʽ
	bool SetTriggerMode(GX_TRIGGER_MODE_ENTRY eTriggerMode, GX_TRIGGER_SOURCE_ENTRY eTriggerSource);
	//�ص�����
	static void __stdcall ContiCapCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame);
	//�ص��ɼ� ����
	bool GetImageFromCam_Continuous(GX_FRAME_CALLBACK_PARAM* pFrame = NULL);
	//�ص��ɼ� ��ͼ
	bool GetImageFromCam_Single(GX_FRAME_CALLBACK_PARAM* pFrame = NULL);
	//�����ɼ�����
	bool CaptureImage_Software(unsigned int unTimeout);
	//Ӳ�����ɼ�����
	bool CaptureImage_Hard(unsigned int unTimeout);
	//ת��IO�˿ڲ���
	bool GetLineNum(int nLineNum, GX_LINE_SELECTOR_ENTRY& gLineNum);
	//��ȡ��ǰͼƬ�ߴ�
	bool GetDeviceParam();
	//����λ����Ϊ8bit
	bool SetPixelFormat8bit();

	GX_DEV_HANDLE m_hDevice;						//�豸���
	bool m_bDeviceIsInit;							//�豸�Ƿ��Ѿ���ʼ��
	bool m_bDeviceIsAcquisition;					//�豸�Ƿ��Ѿ���ʼ�ɼ�
	int64_t m_nPayLoadSize;							//��ǰͼ���С
public:
	int64_t m_nImageWidth;							//��ǰͼ���
	int64_t m_nImageHeight;							//��ǰͼ���
	int m_nRoiOffsetX;
	int m_nRoiOffsetY;
};
	

#endif