/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
*
* 文件名称： ScanWeldLine.h
* 内容摘要： 先测后焊焊缝全扫描焊接类
* 当前版本： 1.0
* 作    者： 刘彦斌
* 完成日期： 2024年8月21日
* 其它说明：
*
* 修改记录1：
*
* 修改记录2：
*
注意：


**********************************************************************/


#pragma once
#ifndef _SCAN_WELD_LINE
#define _SCAN_WELD_LINE

#include "WeldAfterMeasure.h"

class ScanWeldLine : public WAM::WeldAfterMeasure
{
public:
	ScanWeldLine(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
	~ScanWeldLine();

	virtual void SetRecoParam() override;
	virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) override;
	virtual bool WeldSeamGrouping(int& nWeldGroupNum) override;
	virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) override;
	bool CalcMeasureTrack_Circle(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight);
	void adjustCoordinate(double& coord, double& axisCoord, double lowerLimit, double upperLimit, double scanValue);
	bool CalcMeasureTrack_Normal(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight);
	virtual bool DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo = 0) override;
	virtual bool CalcWeldTrack(int nGroupNo) override;
private:
	// 读取轮廓坐标 文件格式:序号 x y z dirx diry dirz 读取结果：vtContour中xyz 方向保存再drx dry drz中; dbx dby dbz为0
	bool LoadContour(int nWeldIndex, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour);
	//
	bool LoadContourByJson(int nWeldIndex, int nGroupNo, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour);
	// 计算并分解坐标所有vtCoord中的世界坐标x y z 到 x y z dbx dby dbz ; dExAxlePos:分解轴的外部轴坐标
	void DecomposeExAxle(std::vector<T_ROBOT_COORS>& vtCoord, double& dExAxlePos, double& dExAxlePos_n, int trackAxisNo);
	// 轨迹滤波(xyz)并添加坐标姿态
	bool TrackFilter(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	bool TrackFilter_EX(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	bool TrackFilter_weld(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	// 保存第nGroupNo组焊缝测量焊枪工具坐标到文件
	void SaveMeasureTrackGunTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord);
	// 保存第nGroupNo组焊缝测量相机工具坐标到文件
	void SaveMeasureTrackCamTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord, const std::vector<T_ANGLE_PULSE>& vtPulse, const std::vector<int>& vtPtnType);
	// 将工具tSrcTool的所有坐标vtCoord，转换为工具的坐标
	bool TransCoordByTool(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool);
	// 在 vtCoord 和 前后添加过渡点坐标 和 类型
	void AddSafeCoord(std::vector<T_ROBOT_COORS>& vtCoord, vector<int>& vnType, double dMaxPartHeightZ);
	//
	void CurveScanSide(WeldLineInfo tWeldInfo, double dExAxlePos, double dExAxlePos_y, std::vector<T_ROBOT_COORS>& vtMeasureCoord, double pExLinepExLine, double angle);

	void CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen);

	// 运动外部轴
	bool MoveExAxleToDst(double dPos, int nAxisNo, double dSpeed);

	// 扫描后从 m_pTraceModel->vtConnerWorldCoords 中 获取结果 保存到 m_vtTeachResult中
	bool GetScanWeldLineResult();

	// 采集线程
	static UINT ThreadMeasure(void* pParam);

	void WeldInfoToJson(Welding_Info tWeldInfo);

	void BackHome();

	// 局部点云处理
	Local_Welding_Info PartPointCloudProcess(vector<T_ROBOT_COORS>& vtCoord, int nGroupNo);

	vector<T_ROBOT_COORS> generateTrajectory(double startX, double startY, double startZ, double endX, double endY, double endZ);

	double m_dScanSpeed; // 扫描测量速度
	double m_dMaxLength; // 分段长度
	int m_isOneWled;
	int m_VisionType;	// 视觉配置文件
	double m_LiRon;		// 角钢上沿宽度
	bool m_bIsCake;     //1接线盒，0基座
	double m_conRz;
	double holesize;
	int m_iscircle;
	bool isMeasureMove;
	int m_nScanNum;
	int m_nWeldHole;//过焊孔
	std::vector<T_ROBOT_COORS> m_vtContour;
};

#endif // !_SCAN_WELD_LINE



