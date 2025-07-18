/*********************************************************************
* ��Ȩ���� (C)2018, �������н����ܻ����˹ɷ����޹�˾
*
* �ļ����ƣ� SingleBoard.h
* ����ժҪ�� ���嵥�����
* ��ǰ�汾�� 1.0
* ��    �ߣ� �����
* ������ڣ� 2024��7��17��
* ����˵����
*
* �޸ļ�¼1��
*
* �޸ļ�¼2��
*
ע�⣺


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
	//��5��������Ϊ�˼��ٲ���ʱ��ӵ� ���嵥�����
	bool SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos , bool bIsMeasureLastPoint);
	bool SavePointCloudProcessResult(LineOrCircularArcWeldingLine* pWeldSeams, int nWeldLineNumber);
public:
	void InitParam();
	bool SingleGroupWeld(int nGroupNo);
	//��job�е�״̬λ
	bool SetSingleBoardJobState(bool bState);
	//bool CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	//bool CalcThickMeasureData(bool bNeedMeasureThick, int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//bool CalcWeldMidTeachData(int nGroupNo, int nSeamNo, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//bool SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx);
	//bool AddSafeCoord(vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, double dMaxPartHeight, double dSafeDis);

public:
	bool m_bAutoMeasureThick; // �Ƿ��Զ����������

	double m_dExAxleOffsetDis; // ��������ʱ�ⲿ��ƫ�ƾ��� ����0  ����500������
	T_ANGLE_PULSE m_tThickMeasureRefPulse; // �������ο��ؽ�����
	T_ANGLE_PULSE m_tStartMeasureRefPulse; // �������ο��ؽ�����
	T_TEACH_RESULT m_tLastTeachResult;  //���һ��ʾ�̽����������
};

#endif