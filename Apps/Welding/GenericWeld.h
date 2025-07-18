/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
*
* 文件名称： GenericWeld.h
* 内容摘要： 通用焊接类
* 当前版本： 1.0
* 作    者： 刘彦斌
* 完成日期： 2023年8月10日
* 其它说明：
*
* 修改记录1：
*
* 修改记录2：
*
注意：
	

**********************************************************************/

#pragma once
#ifndef _GENERIC_WELD
#define _GENERIC_WELD
#include "WeldAfterMeasure.h"

class GenericWeld : public WAM::WeldAfterMeasure
{
public:
	GenericWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
	~GenericWeld();

	virtual void SetRecoParam() override;
	virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) override;
	virtual bool WeldSeamGrouping(int& nWeldGroupNum) override;
	virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) override;
	virtual bool CalcWeldTrack(int nGroupNo) override;

private:
	// 计算测量点所在焊道法向
	void GetWeldLineDir(std::vector<T_ROBOT_COORS> vtMeasureCoord, std::vector<T_LINE_DIR_3D>& vtWeldLineDir);
	// 计算当前工件上板件信息
	void GetBlockPlate(std::vector<T_XIGEO_WELDLINE>& vtCheckBlockPlate);
	// 对vtMeasureCoord按照Rz排序可连续运动 vtSortIdx为排序后的索引, dExAxlePos为初步计算的外部轴位置，内部计算此位置不合适会修改，然后重新计算机器人坐标 vtSortIdx为排序后的顺序
	bool SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordNew_y(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordNew(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordForArc(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	// 将示教结果按照 SortMeasureCoord 函数的排序结果 恢复顺序
	bool RestoreOrderTeachResult();
	// 在连续轨迹vtCoord首位插入收枪下枪坐标
	bool AddSafeCoord(vector<T_ROBOT_COORS> &vtMeasureCoord, vector<int> &vnMeasureType, double dMaxPartHeight, double dSafeDis);
	// 计算焊缝平焊tSeam上的两个标准测量坐标(如果tSeam中起点终点是世界坐标，结果也是世界坐标)
	bool CalcSeamMeasureCoord(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, bool bOffsetHandEyeDis = true);
	// 计算焊缝平焊tSeam上起点或终点定长搜索数据(如果tSeam中起点终点是世界坐标，结果也是世界坐标)
	bool CalcSeamMeasureCoordFix(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE,bool bIsStart = true, bool bOffsetHandEyeDis = true);
	// 计算指定焊缝精确测量所需要的所有测量点信息
	bool CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> tSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// 计算焊缝全扫描测量轨迹
	bool CalcScanMeasureTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// 计算圆弧焊缝精确测量所需要的所有测量点信息
	bool CalcSeamArcTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// 计算闭合圆先测后焊轨迹
	bool CalcTeachDataCircle(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// 计算自由曲线跟踪测量轨迹
	bool CalcTeachDataFreeCurve(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// 计算平焊自由端点测量数据
	bool CalcFreeTeachDataFlat(int nGroupNo, int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dSearchDis, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// 计算平焊干涉端点测量数据
	bool CalcInterfereTeachDataFlat(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// 计算干涉端定长搜索/搜局部点云数据
	bool CalcInterfereTeachDataFlatFix(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// 计算立焊测量数据
	bool CalcTeachDataStand(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// 计算立焊自由端起点测量数据
	bool CalcTeachDataStandFreeStart(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, vector<T_TEACH_DATA>& vtTeachData);
	// 计算长平焊缝侧弯调整测量点
	bool CalcFlatSeamMeasurementEnhancement(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeam, vector<T_TEACH_DATA>& vtTeachData);
	// 输入测量点 所属起点/终点 所属点类型 除所属点外另外一个点的类型 焊缝长度默认0表示属于假设焊缝上的测量点
	void CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen = 0);
	// 使用m_vtTeachData修正焊缝信息
	bool WeldSeamAdjust(int nGroupNo);
	// 使用一个焊缝的测量数据计算起点 终点 法相 ZSide
	bool CalcAccurateEndpoint(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tData, T_ALGORITHM_POINT& tS, T_ALGORITHM_POINT& tE, double& dNorAngle, double& dZSide);
	// 根据 平/立焊 起/终点 干涉/自由端点 判断测量结果数量有效性
	bool CheckTeachDataNumValid(int nNumber, bool bIsFlatSeam, bool bIsStartPoint, bool bIsFreePoint);
	// 计算自由端准确端点
	bool CalcFreePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tStartPtn);
	// 计算干涉端准确端点
	bool CalcInterferePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// 计算立焊干涉起点准确端点
	bool CalcInterferePointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// 计算立焊自由起点准确端点
	bool CalcFreeStartPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// 计算立焊自由终点准确端点
	bool CalcFreeEndPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// 检查是否有干涉端点识别为自由端点 及焊缝长度(先测后焊机头，焊缝长度大于阈值，分成多个焊缝)
	void CheckSeamDataEndpointType(double dDisThreshold);
	
	void WeldSeamSegmentation(WeldLineInfo tWeldSeam, double dLenThreshold, std::vector<WeldLineInfo>& vtWeldSeams);
	// 调试 测试 使用函数
	// 保存每条焊缝的全部测量点
	void SaveTeachDataEverySeam(int nGroupNo, int nSeamNo, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//获取龙门停靠位置
	bool GetGantryPos(vector<LineOrCircularArcWeldingLine>& vtSeamGroup, double& dExAxlePos, double& dPartMaxHeight,double dTrackPos, double dPartMinHeight, CvPoint3D64f tTotalValue, CvPoint3D64f tTotalDirValue);
	// 计算跟踪焊接轨迹，利用初始段轨迹和结尾点补充完整轨迹后保存，便于跟踪类调用数据进行焊接
	bool CalcAndTrackWeldingTry(int nGroupNo);
	// 判断焊缝组内是否存在跟踪焊缝
	bool DetermineGroupMode(int nGroupNo);
	

public:
	// 先测后焊设备，长焊缝，分割为短焊缝
	void Segmentation();
	

};

#endif