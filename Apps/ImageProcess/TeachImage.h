/*************************************************************************************************
 * 文件： TeachImage.h
 * 说明： 1.示教图片的采集、处理、保存、显示、读取等操作；
 *		  2.异常时可自动打包数据；
 * 作者： 江文奇
 * 日期： 2025-09-29
 * ToDo： 增加测试功能；
 ************************************************************************************************/
#pragma once
#include "Apps/ImageProcess/GenericImage.h"

namespace image_process
{
/// @brief 示教图片
class TeachImage : public GenericImage
{
public:
	TeachImage();
	virtual ~TeachImage();

	/****************************************** 初始化 ******************************************/
	/// @brief 确保数据文件夹存在
	virtual void makesureDataFolderExist() override;

	/// @brief 删除旧数据
	virtual void deleteOldData() override;

	/***************************************** 图片数据 *****************************************/
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

	/***************************************** 图像处理 *****************************************/
	/// @brief 初始化图像处理参数
	/// @param index 图片索引
	/// @param nMeasureType	示教点类型
	/// @param dExAxlePos 外部轴位置
	/// @param tCapPulse 采图时脉冲坐标
	/// @return true 成功；false 失败
	bool initImageProcessPara(int index, int nMeasureType, double dExAxlePos,
		const T_ANGLE_PULSE& tCapPulse);

	/// @brief 处理图像
	/// @return true 成功；false 失败
	virtual bool processImage() override;

	/// @brief 获取处理结果
	/// @return 处理结果
	auto& getTeachResult()
	{
		return m_tTeachResult;
	}

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
		int measureType;//示教点类型
		bool assembleFilp;//左右镜像
		bool crossFilp;//上下镜像
		CString configFile;//配置文件
		double dExAxlePos;//外部轴位置
		T_ANGLE_PULSE tCapPulse;//采图时脉冲坐标
	};

	/// @brief 保存图像处理参数
	/// @param index 图片索引
	/// @param para 图像处理参数
	void saveImageProcessPara(int index, const ImageProcessPara& para) const;

	/// @brief 加载图像处理参数
	/// @param index 图片索引
	/// @param para 图像处理参数
	void loadImageProcessPara(int index, ImageProcessPara& para) const;

	/// @brief 初始化处理结果
	/// @param dExAxlePos 外部轴位置
	/// @param tCapPulse 采图时脉冲坐标
	void initTeachResult(double dExAxlePos, const T_ANGLE_PULSE& tCapPulse);

	/// @brief 将WidgetLaserInfo转换为TeachResult
	/// @param tWidgetLaserInfo 激光信息
	/// @param result 处理结果
	void WidgetLaserInfo2TeachResult(
		const WidgetLaserInfo& tWidgetLaserInfo, T_TEACH_RESULT& result);

	/// @brief 将2D坐标转换为3D坐标
	/// @param result 处理结果
	void trans2DTo3D(T_TEACH_RESULT& result);

	/***************************************** 私有变量 *****************************************/
	/// @brief 配置文件夹路径
	CString m_sConfigFolder = _T(".\\Local_Files\\ExtLib\\Vision\\ConfigFiles\\WidgetLaserVis\\");

	/// @brief 提取激光点配置文件名称
	CString m_sLaserPntConfigName[3] = {
		_T("WidgetLaserLock"),
		_T("WidgetLaserLock_HeightMeasure"),
		_T("WidgetLaserLock_HeightMeasure")
	};

	/// @brief 图像处理参数
	ImageProcessPara m_tProcessPara;

	/// @brief 处理结果
	T_TEACH_RESULT m_tTeachResult;
};
}
