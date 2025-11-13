#pragma once
#include <atomic>
#include <opencv2/opencv.hpp>
#include "Apps/PartData/WeldTrack.h"
#include "Apps/PLib/DahengCam/DHGigeImageCapture.h"
#include "Apps/PLib/YaskawaRobot/RobotDriverAdaptor.h"
#include "Apps/RealTimeTrack/AdjustTrack.h"

class CUnit;

/// @brief 实时跟踪
namespace RTT
{
	class RealTimeTrack
	{
	public:
		/// @brief 图片信息
		struct T_IMG_INFO;

		/// @brief 图像处理线程数据
		struct T_PRO_IMG_THREAD_INFO;

	public:
		RealTimeTrack();
		virtual ~RealTimeTrack();

		/// @brief 设置是否启用，仅在调用startRun之前有效
		/// @param bEnable 是否启用
		void setEnable(bool bEnable);

		/// @brief 是否已启用
		/// @return 是/否
		bool isEnable() const;

		/// @brief 设置控制单元控制指针
		/// @param cUnit 控制单元控制指针
		void setUnit(CUnit& cUnit);

		/// @brief 设置相机指针
		/// @param cCamera 相机指针
		void setCamera(CDHGigeImageCapture& cCamera);

		/// @brief 设置机器人指针
		/// @param cRobot 机器人指针
		void setRobot(CRobotDriverAdaptor& cRobot);

		/// @brief 设置初始焊接轨迹
		/// @param cWeldTrack 初始焊接轨迹
		void initTrack(PartData::WeldTrack& cWeldTrack);

		/// @brief 设置激光锁定结果
		/// @param tWidgetLaserInfo 激光锁定结果
		void setLaserLockResult(WidgetLaserInfo& tWidgetLaserInfo);

		/// @brief 设置补偿参数
		/// @param dNormalComp 法向补偿
		/// @param dHeightComp 高度补偿
		void setCompensation(double dNormalComp, double dHeightComp);

		/// @brief 运行前初始化
		/// @return 成功/失败
		bool initBeforeRun();

		/// @brief 开始运行
		void startRun();

		/// @brief 结束运行
		void endRun();

		/// @brief 等待结束
		void waitUntilEnd();

		AdjustTrack m_cAdjustTrack;//轨迹修正

	protected:
		/// @brief 急停设备
		void emg();

		/// @brief 暂停采图
		void pauseGetImg();

		/// @brief 继续采图
		void continueGetImg();

		/// @brief 暂停轨迹修正
		void pauseAdjustTrack();

		/// @brief 继续修正
		void continueAdjustTrack();

		/// @brief 获取图像子线程
		/// @param pParam this指针
		/// @return 无意义
		static UINT ThreadGetImg(void* pParam);

		/// @brief 图像处理子线程
		/// @param pParam T_PRO_IMG_THREAD_INFO参数
		/// @return 无意义
		static UINT ThreadImageProcess(void* pParam);

		/// @brief 轨迹修正子线程
		/// @param pParam this指针
		/// @return 无意义
		static UINT ThreadAdjustTrack(void* pParam);

		/// @brief 获取图像
		void getImg();

		/// @brief 处理图像
		/// @param tProImgInfo 图像处理子线程信息
		void processImage(T_PRO_IMG_THREAD_INFO& tProImgInfo);

		/// @brief 等待下一张待处理图片
		/// @param tProImgInfo 图像处理子线程信息
		/// @return 下一张待处理图片的指针
		T_IMG_INFO* waitForNextImgForProcess(T_PRO_IMG_THREAD_INFO& tProImgInfo);

		/// @brief 激光点二维转三维
		/// @param tImgInfo 图片信息
		void laserPnt2DTo3D(T_IMG_INFO& tImgInfo);

		/// @brief 轨迹修正
		void adjustTrack();

		/// @brief 等待下一张待修正轨迹用图片
		/// @param nNextImgNo 下一张图片的序号
		/// @return true：成功
		/// @return false：已经没有可供使用的图片
		T_IMG_INFO* waitForNextImgForAdjustTrack(int& nNextImgNo);

	private:
		/// @brief 创建图像数据缓存区
		void createImgBuffer();

		/// @brief 清空图像数据缓存区
		void releaseImgBuffer();

		/// @brief 获取数据保存文件的根目录
		/// @return 数据保存文件的根目录
		CString getDataFileRootFolder() const;

		/// @brief 保存图片
		void saveImg(T_IMG_INFO& tImgInfo);

		/// @brief 写日志
		/// @param format 格式化字符串
		/// @param  被格式化变量
		void writeLog(const char* format, ...) const;

		/*******************************输入数据*******************************/
		CUnit* m_pUnit = nullptr;									//控制单元指针
		CDHGigeImageCapture* m_pCamera = nullptr;					//相机控制指针
		CRobotDriverAdaptor* m_pRobot = nullptr;					//机器人控制指针
		PartData::WeldTrack* m_pWeldTrack = nullptr;				//焊接轨迹
		WidgetLaserInfo* m_pWidgetLaserInfo = nullptr;				//图像处理函数参数
		double m_dNormalComp = 0.0;									//XY平面法向补偿
		double m_dHeightComp = 0.0;									//Z方向高度补偿
		bool m_bEnable = true;										//是否启用轨迹修正

		/*******************************控制参数*******************************/
		int m_nImgProcessThreadCount = 4;							//图像处理线程数目
		int m_nMinShowImgSampleGap = 10;							//显示图片的最小间隔，一般和处理线程数相等即可
		int m_nImgBufferCount = 500;								//图像缓存数目
		double m_dMaxFrame = 15.0;									//图像采集最大帧率
		int m_nLockFailTimesThre = 3;								//连续锁定失败次数上限
		int m_nGetKeyPntFailThre = 60;								//寻找激光交点连续失败次数阈值
		double m_dDisOfGetPntCloud = 150.0;							//获取点云的总距离

		/***************************采图线程实时状态***************************/
		std::vector<T_IMG_INFO*> m_vpImgInfo;						//图像数据缓存区
		std::atomic<bool> m_bEndGetImage = true;					//是否终止采图
		std::atomic<bool> m_bPauseGetImage = true;					//是否暂停采图
		std::atomic<bool> m_bIsGetImageEnd = true;					//是否已经终止采图
		std::atomic<int> m_nNextGetImgNo = 0;						//下次采集图片的序号
		int m_nLockFailTimes = 0;									//连续锁定失败次数
		bool m_bFirstLockComplete = false;							//初次锁定完成

		/*************************图像处理线程实时状态*************************/
		std::vector<T_PRO_IMG_THREAD_INFO*> m_vpProImgThreadInfo;	//图像处理线程信息

		/*************************轨迹修正线程实时状态*************************/
		std::atomic<bool> m_bPauseAdjustTrack = true;				//是否已经暂停修正
		std::atomic<bool> m_bIsAdjustTrackEnd = true;				//是否已经终止修正

		/*************************终点判断线程实时状态*************************/
		std::atomic<bool> m_bIsStartGetPntCloud = false;			//是否已经开始采集点云
		CvPoint3D64f m_tTheoryEndPnt;								//理论终点
	};
}

