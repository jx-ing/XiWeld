#pragma once
#include "Apps/ImageProcess/GenericImage.h"
#include "LocalFiles/ExLib/Vision/include/GsImageProcess.h"
#include "LocalFiles/ExLib/Vision/include/PointCloudWeldingLinesExtraction.h"
#include "LocalFiles/ExLib/Vision/include/RiserManagement.h"

namespace image_process
{
class FlatWeldRealTimeTrack : public GenericImage
{
public:
	FlatWeldRealTimeTrack();
	virtual ~FlatWeldRealTimeTrack();

	/****************************************** 初始化 ******************************************/
	/// @brief 确保数据文件夹存在
	virtual void makesureDataFolderExist() override;

	/// @brief 删除旧数据
	virtual void deleteOldData() override;

	/***************************************** 图片数据 *****************************************/
	/// @brief 获取锁定用原图文件名称
	/// @return 图片文件名称
	CString getLockSrcImageName();

	/// @brief 获取锁定用处理后图片文件名称
	/// @return 图片文件名称
	CString getLockDstImageName();

	/// @brief 获取原图文件名称
	/// @param index 图片索引
	/// @return 图片文件名称
	virtual CString getSrcImageName(int index) override;

	/// @brief 获取处理后图片文件名称
	/// @param index 图片索引
	/// @return 图片文件名称
	virtual CString getDstImageName(int index) override;

	/// @brief 保存原图
	/// @param index 图片索引
	virtual void saveSrcImage(int index) override;

	/// @brief 加载原图
	virtual void loadSrcImage() override;

	/// @brief 保存处理后图片
	/// @param index 图片索引
	virtual void saveDstImage(int index) override;

	/// @brief 加载处理后图片
	virtual void loadDstImage() override;

	/***************************************** 点云数据 *****************************************/
	/// @brief 获取点云文件夹名称
	/// @return 点云文件夹名称
	CString getPointCloudFolder();

	/// @brief 获取点云文件名称
	/// @return 点云文件名称
	CString getPointCloudFileName()
	{
		return m_sPointCloudFileName;
	}

	/***************************************** 图像处理 *****************************************/
	/// @brief 在开始图像处理之前，进行一些准备工作
	/// @return true 成功；false 失败
	bool beforeProcessImage();

	/// @brief 锁定激光
	/// @param bIsContinueWeld 是否继续焊接
	/// @return true 成功；false 失败
	bool lockLaser(bool bIsContinueWeld);

	/// @brief 初始化图像处理参数
	/// @param index 图片索引
	/// @param tCapPulse 采图时脉冲坐标
	/// @param tCapCoord 采图时机器人坐标
	/// @return true 成功；false 失败
	bool initImageProcessPara(int index, const T_ANGLE_PULSE& tCapPulse,
		const T_ROBOT_COORS& tCapCoord);

	/// @brief 处理图像
	/// @return true 成功；false 失败
	virtual bool processImage() override;

	/// @brief 获取处理结果
	/// @return 处理结果
	auto& getProcessResult()
	{
		return m_vtPoints;
	}

	/// @brief 获取关键点
	/// @param tKeyPoint 关键点坐标
	/// @return true 成功；false 失败
	auto getKeyPoint(T_ROBOT_COORS& tKeyPoint) const
	{
		if(m_bGetKeyPoint)
		{
			tKeyPoint = m_tKeyPointCoord;
		}
		return m_bGetKeyPoint;
	}

	/// @brief 获取关键点
	/// @param tKeyPoint 关键点坐标
	/// @return true 成功；false 失败
	auto getKeyPoint(CvPoint& tKeyPoint) const
	{
		if (m_bGetKeyPoint)
		{
			tKeyPoint = m_tKeyPointPixelCoord;
		}
		return m_bGetKeyPoint;
	}

	/// @brief 在图像处理之后，进行一些收尾工作
	/// @return true 成功；false 失败
	bool afterProcessImage();

	/***************************************** 图像显示 *****************************************/
	/// @brief 绘制处理结果
	virtual void drawResult() override;

	/***************************************** 错误处理 *****************************************/
	/// @brief 保存错误数据
	/// @param bAutoSave 是否自动保存
	/// @param sName 错误数据名称
	virtual void saveErrorData(bool bAutoSave, CString sName) override;

	/***************************************** 私有函数 *****************************************/
private:
	/// @brief 图像处理参数
	struct ImageProcessPara
	{
		int index;//图片索引
		T_ANGLE_PULSE tCapPulse;//采图时脉冲坐标
		T_ROBOT_COORS tCapCoord;//采图时机器人坐标
	};

	/// @brief 关闭完整点云文件
	void closeFullPointCloudFile();

	/// @brief 删除旧完整点云文件
	/// @param nRetentionFileCount 保留文件数量
	void deleteOldFullPointCloudFile(int nRetentionFileCount = 20);

	/// @brief 保存图像处理参数
	/// @param index 图片索引
	/// @param para 图像处理参数
	void saveImageProcessPara(int index, const ImageProcessPara& para) const;

	/// @brief 删除激光跟踪工具
	void deleteWidgetLaserTrackTool();

	/// @brief 删除图像处理指针
	void deleteImageProcessPointer();

	/***************************************** 私有变量 *****************************************/
	/// @brief 配置文件夹路径
	CString m_sConfigFolder = _T(".\\Local_Files\\ExtLib\\Vision\\ConfigFiles\\WidgetLaserVis\\");

	/// @brief 激光锁定配置文件名称
	CString m_sLaserLockConfigName = _T("WidgetLaserLock");

	/// @brief 提取激光点配置文件名称
	CString m_sLaserPntConfigName = _T("FindLaserMidPiontEEEEEInTrack_Fix");
	CString m_sLaserPntConfigPath = _T(".\\Local_Files\\ExtLib\\Vision\\ConfigFiles\\TrackDipParam\\FindLaserMidPiontEEEEEInTrack_Fix");

	/// @brief 点云文件名称
	CString m_sPointCloudFileName;

	/// @brief 完整点云文件
	FILE* m_pFullPointCloudFile = nullptr;

	/// @brief 图像处理参数
	ImageProcessPara m_tProcessPara;

	/// @brief 激光锁定信息
	WidgetLaserInfo* m_pSearchLockLaserInfo = new WidgetLaserInfo[10];

	XiLineParamNode m_tFrontLine;
	XiLineParamNode m_tBackLine;

	/// @brief 激光跟踪工具
	WidgetLaserTrackTool* m_pWidgetLaserTrackTool = nullptr;
	GROUP_STAND_DIP_NS::CImageProcess* m_pTraceImgProcess = nullptr;

	/// @brief 三维点（单图处理结果）
	std::list<Three_DPoint> m_vtPoints;
	T_ROBOT_COORS m_tKeyPointCoord;
	CvPoint m_tKeyPointPixelCoord;
	bool m_bGetKeyPoint = false;

};
}
