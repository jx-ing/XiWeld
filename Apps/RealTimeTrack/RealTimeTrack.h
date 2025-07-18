#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>
#include "Apps/PartData/WeldTrack.h"
#include "Apps/PLib/DahengCam/DHGigeImageCapture.h"
#include "Apps/PLib/YaskawaRobot/RobotDriverAdaptor.h"
#include "Apps/RealTimeTrack/AdjustTrack.h"

class CUnit;

/// @brief ʵʱ����
namespace RTT
{
	class RealTimeTrack
	{
	public:
		/// @brief ͼƬ��Ϣ
		struct T_IMG_INFO;

		/// @brief ͼ�����߳�����
		struct T_PRO_IMG_THREAD_INFO;

	public:
		RealTimeTrack();
		virtual ~RealTimeTrack();

		/// @brief �����Ƿ����ã����ڵ���startRun֮ǰ��Ч
		/// @param bEnable �Ƿ�����
		void setEnable(bool bEnable);

		/// @brief �Ƿ�������
		/// @return ��/��
		bool isEnable() const;

		/// @brief ���ÿ��Ƶ�Ԫ����ָ��
		/// @param cUnit ���Ƶ�Ԫ����ָ��
		void setUnit(CUnit& cUnit);

		/// @brief �������ָ��
		/// @param cCamera ���ָ��
		void setCamera(CDHGigeImageCapture& cCamera);

		/// @brief ���û�����ָ��
		/// @param cRobot ������ָ��
		void setRobot(CRobotDriverAdaptor& cRobot);

		/// @brief ���ó�ʼ���ӹ켣
		/// @param cWeldTrack ��ʼ���ӹ켣
		void initTrack(PartData::WeldTrack& cWeldTrack);

		/// @brief ���ü����������
		/// @param tWidgetLaserInfo �����������
		void setLaserLockResult(WidgetLaserInfo& tWidgetLaserInfo);

		/// @brief ���ò�������
		/// @param dNormalComp ���򲹳�
		/// @param dHeightComp �߶Ȳ���
		void setCompensation(double dNormalComp, double dHeightComp);

		/// @brief ����ǰ��ʼ��
		/// @return �ɹ�/ʧ��
		bool initBeforeRun();

		/// @brief ��ʼ����
		void startRun();

		/// @brief ��������
		void endRun();

		/// @brief �ȴ�����
		void waitUntilEnd();

		AdjustTrack m_cAdjustTrack;//�켣����

	protected:
		/// @brief ��ͣ�豸
		void emg();

		/// @brief ��ͣ��ͼ
		void pauseGetImg();

		/// @brief ������ͼ
		void continueGetImg();

		/// @brief ��ͣ�켣����
		void pauseAdjustTrack();

		/// @brief ��������
		void continueAdjustTrack();

		/// @brief ��ȡͼ�����߳�
		/// @param pParam thisָ��
		/// @return ������
		static UINT ThreadGetImg(void* pParam);

		/// @brief ͼ�������߳�
		/// @param pParam T_PRO_IMG_THREAD_INFO����
		/// @return ������
		static UINT ThreadImageProcess(void* pParam);

		/// @brief �켣�������߳�
		/// @param pParam thisָ��
		/// @return ������
		static UINT ThreadAdjustTrack(void* pParam);

		/// @brief ��ȡͼ��
		void getImg();

		/// @brief ����ͼ��
		/// @param tProImgInfo ͼ�������߳���Ϣ
		void processImage(T_PRO_IMG_THREAD_INFO& tProImgInfo);

		/// @brief �ȴ���һ�Ŵ�����ͼƬ
		/// @param tProImgInfo ͼ�������߳���Ϣ
		/// @return ��һ�Ŵ�����ͼƬ��ָ��
		T_IMG_INFO* waitForNextImgForProcess(T_PRO_IMG_THREAD_INFO& tProImgInfo);

		/// @brief ������άת��ά
		/// @param tImgInfo ͼƬ��Ϣ
		void laserPnt2DTo3D(T_IMG_INFO& tImgInfo);

		/// @brief �켣����
		void adjustTrack();

		/// @brief �ȴ���һ�Ŵ������켣��ͼƬ
		/// @param nNextImgNo ��һ��ͼƬ�����
		/// @return true���ɹ�
		/// @return false���Ѿ�û�пɹ�ʹ�õ�ͼƬ
		T_IMG_INFO* waitForNextImgForAdjustTrack(int& nNextImgNo);

	private:
		/// @brief ����ͼ�����ݻ�����
		void createImgBuffer();

		/// @brief ���ͼ�����ݻ�����
		void releaseImgBuffer();

		/// @brief ��ȡ���ݱ����ļ��ĸ�Ŀ¼
		/// @return ���ݱ����ļ��ĸ�Ŀ¼
		CString getDataFileRootFolder() const;

		/// @brief ����ͼƬ
		void saveImg(T_IMG_INFO& tImgInfo);

		/// @brief д��־
		/// @param format ��ʽ���ַ���
		/// @param  ����ʽ������
		void writeLog(const char* format, ...) const;

		/*******************************��������*******************************/
		CUnit* m_pUnit = nullptr;									//���Ƶ�Ԫָ��
		CDHGigeImageCapture* m_pCamera = nullptr;					//�������ָ��
		CRobotDriverAdaptor* m_pRobot = nullptr;					//�����˿���ָ��
		PartData::WeldTrack* m_pWeldTrack = nullptr;				//���ӹ켣
		WidgetLaserInfo* m_pWidgetLaserInfo = nullptr;				//ͼ����������
		double m_dNormalComp = 0.0;									//XYƽ�淨�򲹳�
		double m_dHeightComp = 0.0;									//Z����߶Ȳ���
		bool m_bEnable = true;										//�Ƿ����ù켣����

		/*******************************���Ʋ���*******************************/
		int m_nImgProcessThreadCount = 4;							//ͼ�����߳���Ŀ
		int m_nMinShowImgSampleGap = 10;							//��ʾͼƬ����С�����һ��ʹ����߳�����ȼ���
		int m_nImgBufferCount = 500;								//ͼ�񻺴���Ŀ
		double m_dMaxFrame = 15.0;									//ͼ��ɼ����֡��
		int m_nLockFailTimesThre = 3;								//��������ʧ�ܴ�������
		int m_nGetKeyPntFailThre = 60;								//Ѱ�Ҽ��⽻������ʧ�ܴ�����ֵ
		double m_dDisOfGetPntCloud = 150.0;							//��ȡ���Ƶ��ܾ���

		/***************************��ͼ�߳�ʵʱ״̬***************************/
		std::vector<T_IMG_INFO*> m_vpImgInfo;						//ͼ�����ݻ�����
		std::atomic<bool> m_bEndGetImage = true;					//�Ƿ���ֹ��ͼ
		std::atomic<bool> m_bPauseGetImage = true;					//�Ƿ���ͣ��ͼ
		std::atomic<bool> m_bIsGetImageEnd = true;					//�Ƿ��Ѿ���ֹ��ͼ
		std::atomic<int> m_nNextGetImgNo = 0;						//�´βɼ�ͼƬ�����
		int m_nLockFailTimes = 0;									//��������ʧ�ܴ���
		bool m_bFirstLockComplete = false;							//�����������

		/*************************ͼ�����߳�ʵʱ״̬*************************/
		std::vector<T_PRO_IMG_THREAD_INFO*> m_vpProImgThreadInfo;	//ͼ�����߳���Ϣ

		/*************************�켣�����߳�ʵʱ״̬*************************/
		std::atomic<bool> m_bPauseAdjustTrack = true;				//�Ƿ��Ѿ���ͣ����
		std::atomic<bool> m_bIsAdjustTrackEnd = true;				//�Ƿ��Ѿ���ֹ����

		/*************************�յ��ж��߳�ʵʱ״̬*************************/
		std::atomic<bool> m_bIsStartGetPntCloud = false;			//�Ƿ��Ѿ���ʼ�ɼ�����
		CvPoint3D64f m_tTheoryEndPnt;								//�����յ�
	};
}

