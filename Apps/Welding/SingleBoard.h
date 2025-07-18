/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
*
* 文件名称： SingleBoard.h
* 内容摘要： 单板单筋焊接类
* 当前版本： 1.0
* 作    者： 刘彦斌
* 完成日期： 2024年7月17日
* 其它说明：
*
* 修改记录1：
*
* 修改记录2：
*
注意：


**********************************************************************/

#pragma once
#ifndef _SINGLE_BOARD
#define _SINGLE_BOARD
#include "WeldAfterMeasure.h"

class SingleBoard : public WAM::WeldAfterMeasure
{
public:
	SingleBoard(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
	~SingleBoard();

	virtual void SetRecoParam() override;
	virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) override;
	virtual bool WeldSeamGrouping(int& nWeldGroupNum) override;
	virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) override;
	virtual bool DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo = 0) override;
	virtual bool CalcWeldTrack(int nGroupNo) override;
	virtual bool DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara) override;
	//第5个参数是为了减少测量时间加的 单板单筋独有
	bool SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos , bool bIsMeasureLastPoint);
	bool SavePointCloudProcessResult(LineOrCircularArcWeldingLine* pWeldSeams, int nWeldLineNumber);
public:
	void InitParam();
	bool SingleGroupWeld(int nGroupNo);
	//在job中的状态位
	bool SetSingleBoardJobState(bool bState);
	//bool CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	//bool CalcThickMeasureData(bool bNeedMeasureThick, int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//bool CalcWeldMidTeachData(int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//bool SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx);
	//bool AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, double dMaxPartHeight, double dSafeDis);

public:
	bool m_bAutoMeasureThick; // 是否自动测量筋板厚度

	double m_dExAxleOffsetDis; // 测量焊接时外部轴偏移距离 正坐0  倒挂500或其他
	T_ANGLE_PULSE m_tThickMeasureRefPulse; // 板厚测量参考关节坐标
	T_ANGLE_PULSE m_tStartMeasureRefPulse; // 起点测量参考关节坐标
	T_TEACH_RESULT m_tLastTeachResult;  //最后一次示教结果单独保存
};

#endif