#pragma once
#include "RiserManagement.h"
#include "LocalFiles/ExLib/Vision/include/PointCloudWeldingLinesExtraction.h"

namespace image_process
{
/// @brief 局部点云处理
class PartialPointCloudProcess
{
public:
	PartialPointCloudProcess();
	virtual ~PartialPointCloudProcess();

	/****************************************** 初始化 ******************************************/
	/// @brief 初始化
	/// @param sDataFolder 数据文件夹名称
	void init(CString sDataFolder);

	/// @brief 设置相机向下法向
	/// @param x 
	/// @param y 
	/// @param z 
	void setCameraNorm(double x, double y, double z)
	{
		m_tCameraNorm.x = x;
		m_tCameraNorm.y = y;
		m_tCameraNorm.z = z;
	}

	/// @brief 设置底板向上法向
	/// @param x 
	/// @param y 
	/// @param z 
	void setPlaneHNorm(double x, double y, double z)
	{
		m_tPlaneHNorm.x = x;
		m_tPlaneHNorm.y = y;
		m_tPlaneHNorm.z = z;
	}

	/// @brief 设置参考线起点
	/// @param x 
	/// @param y 
	/// @param z 
	void setRefStartPtn(double x, double y, double z)
	{
		m_tRefLine[0].x = x;
		m_tRefLine[0].y = y;
		m_tRefLine[0].z = z;
	}

	/// @brief 设置参考线终点
	/// @param x 
	/// @param y 
	/// @param z 
	void setRefEndPtn(double x, double y, double z)
	{
		m_tRefLine[1].x = x;
		m_tRefLine[1].y = y;
		m_tRefLine[1].z = z;
	}

	/// @brief 设置配置文件
	/// @param sConfigFile 配置文件名称
	void setConfigFile(CString sConfigFile)
	{
		m_sConfigFile = sConfigFile;
	}

	/***************************************** 处理函数 *****************************************/
	/// @brief 处理函数
	/// @param nImageNoS 起始图片索引
	/// @param nImageNoE 结束图片索引
	/// @param vtPointCloud 点云
	/// @return 成功/失败
	bool process(int nImageNoS, int nImageNoE, std::vector<CvPoint3D64f>& vtPointCloud);

	/// @brief 获取处理结果
	/// @return 处理结果
	std::vector<Three_DPoint>& getResult()
	{
		return m_vtResult;
	}

	/***************************************** 错误处理 *****************************************/
	/// @brief 保存错误数据
	/// @param bAutoSave 是否自动保存
	/// @param sName 错误数据名称
	void saveErrorData(bool bAutoSave, CString sName);

	/// @brief 打开错误数据文件夹
	void openErrorDataFolder();

private:
	/***************************************** 私有函数 *****************************************/
	/// @brief 保存输入参数
	/// @param nImageNoS 起始图片索引
	/// @param nImageNoE 结束图片索引
	/// @param sFileName 文件夹名称
	/// @return 成功/失败
	bool saveInputPara(int nImageNoS, int nImageNoE, CString sFileFolder) const;

	/// @brief 保存点云
	/// @param vtPointCloud 点云
	/// @param sFileName 点云文件夹名称
	/// @return 成功/失败
	bool savePointCloud(const std::vector<CvPoint3D64f>& vtPointCloud, CString sFileFolder);

	/// @brief 保存处理结果
	/// @param sFileFolder 文件夹名称
	/// @return 成功/失败
	bool saveResult(CString sFileFolder);
	
	CString m_sErrorDataDir = _T(".\\ErrorData\\PartialPointCloudProcess\\");// 错误文件夹名称
	CString m_sDataFolder;// 数据文件夹名称
	CvPoint3D32f m_tCameraNorm;// 相机向下法向
	CvPoint3D32f m_tPlaneHNorm;// 底板向上法向
	CvPoint3D32f m_tRefLine[2];// 参考线	
	CString m_sConfigFolder = _T(".\\Local_Files\\ExtLib\\Vision\\ConfigFiles\\");/// 配置文件夹路径
	CString m_sConfigFile;// 配置文件名称
	std::vector<Three_DPoint> m_vtResult;// 处理结果
	int m_nErrorThreshold;// 错误次数阈值
	int m_nErrorCount = 0;// 错误次数
};
}
