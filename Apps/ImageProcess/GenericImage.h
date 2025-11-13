#pragma once
#include "Apps/PLib/CtrlUnit/CUnit.h"

namespace image_process
{
class GenericImage
{
public:
	GenericImage();
	~GenericImage();

	/****************************************** 初始化 ******************************************/
	/// @brief 设置显示图片缓存
	/// @param pShowImg 显示图片缓存
	void setShowImage(IplImage** pShowImg);

	/// @brief 设置硬件设备
	/// @param pUnit 控制单元
	/// @param nCameraNo 相机编号
	void setHardware(CUnit* pUnit, int nCameraNo);

	/// @brief 创建图像缓冲区
	void createImageBuffer();

	/// @brief 确保数据文件夹存在
	virtual void makesureDataFolderExist() = 0;

	/// @brief 删除旧数据
	virtual void deleteOldData() = 0;

	/***************************************** 图片数据 *****************************************/
	/// @brief 获取原图
	/// @return 图片数据
	IplImage*& getSrcImage()
	{
		return m_pSrcImage;
	}

	/// @brief 获取处理后图片
	/// @return 图片数据
	IplImage*& getDstImage()
	{
		return m_pDstImage;
	}

	/// @brief 获取原图文件名称
	/// @param index 图片索引
	/// @return 图片文件名称
	virtual CString getSrcImageName(int index) = 0;

	/// @brief 获取处理后图片文件名称
	/// @param index 图片索引
	/// @return 图片文件名称
	virtual CString getDstImageName(int index) = 0;

	/// @brief 保存原图
	/// @param index 图片索引
	virtual void saveSrcImage(int index) = 0;

	/// @brief 加载原图
	virtual void loadSrcImage() = 0;

	/// @brief 保存处理后图片
	/// @param index 图片索引
	virtual void saveDstImage(int index) = 0;

	/// @brief 加载处理后图片
	virtual void loadDstImage() = 0;

	/// @brief 获取原图保存路径
	/// @return 原图保存路径
	CString getSrcImageFolder() const;

	/// @brief 获取处理后图片保存路径
	/// @return 处理后图片保存路径
	CString getDstImageFolder() const;

	/// @brief 获取输出配置文件夹路径
	/// @return 输出配置文件夹路径
	CString getOutPutParaFile() const;

	/// @brief 获取数据文件夹路径
	/// @return 数据文件夹路径
	CString getDataFolder() const;

	/***************************************** 相机操作 *****************************************/
	/// @brief 打开相机
	/// @param eCaptureMode 采集模式
	/// @param eCallBackMode 回调方式
	/// @return true 成功；false 失败
	bool openCamera(E_DHGIGE_ACQUISITION_MODE eCaptureMode, E_DHGIGE_CALL_BACK eCallBackMode);

	/// @brief 关闭相机
	/// @return true 成功；false 失败
	bool closeCamera();

	/// @brief 开始采集图像
	/// @return true 成功；false 失败
	bool startAcquisition();

	/// @brief 采集图像
	/// @param nTryTimes 尝试次数
	/// @return true 成功；false 失败
	bool captureImage(int nTryTimes = 3);

	/***************************************** 图像处理 *****************************************/
	/// @brief 处理图像
	/// @return true 成功；false 失败
	virtual bool processImage() = 0;

	/***************************************** 图像显示 *****************************************/
	/// @brief 绘制处理结果
	virtual void drawResult() = 0;

	/// @brief 显示原图
	void showSrcImage();

	/// @brief 显示处理后图片
	void showDstImage();

	/***************************************** 错误处理 *****************************************/
	/// @brief 保存错误数据
	/// @param bAutoSave 是否自动保存
	/// @param sName 错误数据名称
	virtual void saveErrorData(bool bAutoSave, CString sName) = 0;

	/// @brief 打开错误数据文件夹
	void openErrorDataFolder();

protected:
	/***************************************** 释放资源 *****************************************/
	/// @brief 释放资源
	virtual void releaseResources();

	/// @brief 释放原图
	void releaseSrcImages();

	/// @brief 释放处理后图片
	void releaseDstImages();

	/***************************************** 通用成员 *****************************************/
	/// @brief 控制单元指针
	CUnit* m_pUnit = nullptr;

	/// @brief 相机编号
	int m_nCameraNo = 0;

	/// @brief 相机指针
	CDHGigeImageCapture* m_pCamera = nullptr;

	/// @brief 机器人驱动指针
	CRobotDriverAdaptor* m_pRobot = nullptr;

	/// @brief 传递处理图到界面
	IplImage** m_pShowImg = nullptr;

	/// @brief 数据保存文件夹
	CString m_sDataFolder = _T(".\\WeldData\\RobotA\\GenericImage\\");

	/// @brief 错误数据文件夹路径
	CString m_sErrorDataDir = _T(".\\ErrorData\\");

	/***************************************** 生产用 *****************************************/
	/// @brief 单张原图缓存
	IplImage* m_pSrcImage = nullptr;

	/// @brief 单张处理后图片缓存
	IplImage* m_pDstImage = nullptr;

	/***************************************** 测试用 *****************************************/
	/// @brief 原图缓存
	std::vector<IplImage*> m_vpSrcImages;

	/// @brief 处理后图片缓存
	std::vector<IplImage*> m_vpDstImages;
};
}
