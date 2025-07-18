#pragma once
/*********************************************************************
* 版权所有 (C)2018, 哈尔滨行健智能机器人股份有限公司
*
* 文件名称： GrooveWeld.h
* 内容摘要： 坡口焊接类
* 当前版本： 1.0
* 作    者： 刘彦斌
* 完成日期： 2024年5月15日
* 其它说明：
*
* 修改记录1：
*
* 修改记录2：
*
注意：


**********************************************************************/

#pragma once
#ifndef _GROOVE_WELD
#define _GROOVE_WELD
#include "WeldAfterMeasure.h"
#include "Project\ChangeGroovePara.h"
class GrooveWeld : public WAM::WeldAfterMeasure
{
public:
	GrooveWeld(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
	~GrooveWeld();

	virtual void SetRecoParam() override;
	virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) override;
	virtual bool WeldSeamGrouping(int& nWeldGroupNum) override;
	virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) override;
	virtual bool DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo = 0) override;
	virtual bool ScanEndpoint(int nGroupNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& tTeachResult) override;
	virtual bool CalcWeldTrack(int nGroupNo) override;
	virtual bool LoadRealWeldTrack(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE& eWeldSeamType, double& dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int>& vnPtnType);
	virtual bool DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara);
private:
	bool CalcGrooveScanTrack(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	bool SortMeasureCoordNew(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx);
	bool AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, double dMaxPartHeight, double dSafeDis);
	bool GetScanProcResult(vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult);
	bool GetScanProcResult_G(int nGroupNo, int nLayerNo, vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult);
	void SaveTeachResult(int nGroupNo);
	void SaveTeachResult_G(int nGroupNo, int nLayerNo);
	bool GeneralGrooveWeldTrack(int nGroupNo);

	// 计算测量轨迹 拆分后的坐标都保存在vtMeasureCoord中，包括外部轴
	bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dSafeHeight);
	
	// 将nDecomposeAxisNo对应的轴，设置为tFixCoord中xyz对应的值，tFixCoord中xyz都是0表示外部轴固定
	bool DecomposeCoordinate(T_ROBOT_COORS& tRobotCoors, T_ROBOT_COORS tFixCoord, int nDecomposeAxisNo);
	bool DecomposeCoordinate(std::vector<T_ROBOT_COORS>& vtRobotCoors, T_ROBOT_COORS tFixCoord, int nDecomposeAxisNo);

	bool TransCameraToolCoord(std::vector<T_ROBOT_COORS>& vtCoord, int nCameraNo);


	T_ROBOT_COORS m_tFixCoord; // 坡口测量和焊接机器人固定坐标 计算测量轨迹更新 测量 和 焊接使用相同

	// 国焊坡口添加
	//bool DoTeach_G(int nGroupNo,int nLayerNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos);
	bool CalcGrooveScanTrack_G(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	bool AddSafeCoord_G(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, double dMaxPartHeight, double dSafeDis, E_WELD_SEAM_TYPE eWeldSeamType = E_PLAT_GROOVE);
	bool CalcMeasureTrack_G(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dSafeHeight);
	bool ScanEndpoint_G(int nGroupNo, int nLayerNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& vtTeachResult);
	bool GeneralGrooveWeldTrack_G(int nGroupNo); // 只使用首位测量结果 已弃用
	bool GeneralGrooveWeldTrack_G_First(int nGroupNo); // 使用所有测量结果生成所有焊接轨迹 坡口第一次测量使用
	bool GeneralGrooveWeldTrack_G_Other(int nGroupNo, int nLayerNo); // 非第一次测量的第nGroupNo组第nLayerNod道焊接轨迹修正
	//bool AdjustWaveWeldPath(const std::vector<GroovePointInfo>& vtInfoBase, const std::vector<GroovePointInfo>& vtInfoNew, vector<T_ROBOT_COORS>& vtCoord);

	//double CalcUpperWidth(XI_POINT p, int nBasePtnNo, const std::vector<GroovePointInfo>& vtInfoBase);

	XI_POINT MidPtn(CvPoint3D32f p1, CvPoint3D32f p2);
	bool DoWelding_G(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara);
	bool LoadRealWeldTrack_G(CString sWeldTrackFileName, E_WELD_SEAM_TYPE& eWeldSeamType, double& dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int>& vnPtnType);
	// 保证下枪过渡点和第一个测量点外部轴相同，收枪过渡点和最后一个测量点外部轴相同
	void SaveCoordExAxleProcess(std::vector<T_ROBOT_COORS>& vtCoord, std::vector<int> vnType);
	void GenerateJob(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName);

public:
	//坡口函数
	int GetGroovePara(const T_ROBOT_COORS& tRobotStartCoord, const T_ROBOT_COORS& tRobotEndCoord, vector<T_WAVE_PARA>& vtTWavePara);
	int GetFlatGroovePara(vector<T_WAVE_PARA>& vtTWavePara);
	int GetVerGroovePara(vector<T_WAVE_PARA>& vtTWavePara);
	int GetTeachPos(T_ROBOT_COORS& tRobotStartCoord, T_ROBOT_COORS& tRobotEndCoord, int nRobotNo);
	int CalWavePath(T_GROOVE_INFOR tGrooveInfor, T_INFOR_WAVE_RAND tGrooveRand, T_WAVE_PARA tWavePara,int nLayerNo, vector<T_ROBOT_COORS>& vtGrooveWavePath, vector<double>& weldSpeedRate, vector<double> &vdWaveWaitTime);
	int DoGrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate, int nRobotNo);
	int GrooveAutoRandNew(T_GROOVE_INFOR tGrooveInfor, vector<T_WAVE_PARA> vtTWavePara, vector<T_INFOR_WAVE_RAND>& vtGrooveRand, int nRobotDir);
	// 调整机器人nRobotNo, 第nGroupNo组焊缝的起终点（根据nGroupNo组以前焊接轨迹）
	bool AdjustStartEndPtn(int nRobotNo, int nWeldIdx, E_WELD_SEAM_TYPE eWeldSeamType, T_ROBOT_COORS& tStartPtn, T_ROBOT_COORS& tEndPtn);

private:
	E_WELD_SEAM_TYPE m_eWeldSeamType;
	std::vector<double> m_vdGrooveWeldSpeedRate;
	std::vector<double> m_vdWaveWaitTime; // 渐变的停留时间
	//T_ROBOT_COORS m_tFlatGrooveScanPosture = T_ROBOT_COORS(0.0, 500, 1600.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tFlatGrooveWeldPosture = T_ROBOT_COORS(0.0, 500, 1600.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	////T_ROBOT_COORS m_tFlatGrooveScanPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	////T_ROBOT_COORS m_tFlatGrooveWeldPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tStandDownGrooveScanPosture = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tStandDownGrooveWeldPosture = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tStandUpGrooveScanPosture = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tStandUpGrooveWeldPosture = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
};

#endif
