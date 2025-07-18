/*********************************************************************
* ��Ȩ���� (C)2018, �������н����ܻ����˹ɷ����޹�˾
*
* �ļ����ƣ� GenericWeld.h
* ����ժҪ�� ͨ�ú�����
* ��ǰ�汾�� 1.0
* ��    �ߣ� �����
* ������ڣ� 2023��8��10��
* ����˵����
*
* �޸ļ�¼1��
*
* �޸ļ�¼2��
*
ע�⣺
	

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
	// ������������ں�������
	void GetWeldLineDir(std::vector<T_ROBOT_COORS> vtMeasureCoord, std::vector<T_LINE_DIR_3D>& vtWeldLineDir);
	// ���㵱ǰ�����ϰ����Ϣ
	void GetBlockPlate(std::vector<T_XIGEO_WELDLINE>& vtCheckBlockPlate);
	// ��vtMeasureCoord����Rz����������˶� vtSortIdxΪ����������, dExAxlePosΪ����������ⲿ��λ�ã��ڲ������λ�ò����ʻ��޸ģ�Ȼ�����¼������������ vtSortIdxΪ������˳��
	bool SortMeasureCoord(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordNew_y(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double& dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int>& vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordNew(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	bool SortMeasureCoordForArc(double dPartHeight, double dFirstSeamNorAngle, double dExAxleOffsetDis, double &dExAxlePos, vector<T_ROBOT_COORS>& vtMeasureCoord, vector<int> &vnMeasureType, vector<int>& vtSortIdx);
	// ��ʾ�̽������ SortMeasureCoord ������������ �ָ�˳��
	bool RestoreOrderTeachResult();
	// �������켣vtCoord��λ������ǹ��ǹ����
	bool AddSafeCoord(vector<T_ROBOT_COORS> &vtMeasureCoord, vector<int> &vnMeasureType, double dMaxPartHeight, double dSafeDis);
	// ���㺸��ƽ��tSeam�ϵ�������׼��������(���tSeam������յ����������꣬���Ҳ����������)
	bool CalcSeamMeasureCoord(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, bool bOffsetHandEyeDis = true);
	// ���㺸��ƽ��tSeam�������յ㶨����������(���tSeam������յ����������꣬���Ҳ����������)
	bool CalcSeamMeasureCoordFix(LineOrCircularArcWeldingLine tSeam, double dOfficeDis, double dRobotChangeDir, vector<T_ROBOT_COORS>& vtMeasureCoord, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE,bool bIsStart = true, bool bOffsetHandEyeDis = true);
	// ����ָ�����쾫ȷ��������Ҫ�����в�������Ϣ
	bool CalcSeamTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> tSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// ���㺸��ȫɨ������켣
	bool CalcScanMeasureTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// ����Բ�����쾫ȷ��������Ҫ�����в�������Ϣ
	bool CalcSeamArcTeachData(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// ����պ�Բ�Ȳ�󺸹켣
	bool CalcTeachDataCircle(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// �����������߸��ٲ����켣
	bool CalcTeachDataFreeCurve(int nGroupNo, int nSeamNo, double dExAxlePos, vector<LineOrCircularArcWeldingLine> vtSeamGroup, vector<T_TEACH_DATA>& vtTeachData);
	// ����ƽ�����ɶ˵��������
	bool CalcFreeTeachDataFlat(int nGroupNo, int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dSearchDis, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// ����ƽ������˵��������
	bool CalcInterfereTeachDataFlat(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// �������˶�������/�Ѿֲ���������
	bool CalcInterfereTeachDataFlatFix(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dNorAngle, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// ����������������
	bool CalcTeachDataStand(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, vector<LineOrCircularArcWeldingLine> vtSeam, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	// �����������ɶ�����������
	bool CalcTeachDataStandFreeStart(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeamS, LineOrCircularArcWeldingLine* ptAdjoinSeamE, vector<T_TEACH_DATA>& vtTeachData);
	// ���㳤ƽ����������������
	bool CalcFlatSeamMeasurementEnhancement(int nSeamNo, bool bIsFlatSeam, bool bIsStartPtn, LineOrCircularArcWeldingLine tSeam, double dNorAngle, double dExAxlePos, LineOrCircularArcWeldingLine* ptAdjoinSeam, vector<T_TEACH_DATA>& vtTeachData);
	// ��������� �������/�յ� ���������� ��������������һ��������� ���쳤��Ĭ��0��ʾ���ڼ��躸���ϵĲ�����
	void CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen = 0);
	// ʹ��m_vtTeachData����������Ϣ
	bool WeldSeamAdjust(int nGroupNo);
	// ʹ��һ������Ĳ������ݼ������ �յ� ���� ZSide
	bool CalcAccurateEndpoint(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tData, T_ALGORITHM_POINT& tS, T_ALGORITHM_POINT& tE, double& dNorAngle, double& dZSide);
	// ���� ƽ/���� ��/�յ� ����/���ɶ˵� �жϲ������������Ч��
	bool CheckTeachDataNumValid(int nNumber, bool bIsFlatSeam, bool bIsStartPoint, bool bIsFreePoint);
	// �������ɶ�׼ȷ�˵�
	bool CalcFreePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tStartPtn);
	// ��������׼ȷ�˵�
	bool CalcInterferePoint(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// ���������������׼ȷ�˵�
	bool CalcInterferePointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// ���������������׼ȷ�˵�
	bool CalcFreeStartPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// �������������յ�׼ȷ�˵�
	bool CalcFreeEndPointStand(const std::vector<T_TEACH_DATA>& tTeachData, T_ALGORITHM_POINT& tAlgPoint);
	// ����Ƿ��и���˵�ʶ��Ϊ���ɶ˵� �����쳤��(�Ȳ�󺸻�ͷ�����쳤�ȴ�����ֵ���ֳɶ������)
	void CheckSeamDataEndpointType(double dDisThreshold);
	
	void WeldSeamSegmentation(WeldLineInfo tWeldSeam, double dLenThreshold, std::vector<WeldLineInfo>& vtWeldSeams);
	// ���� ���� ʹ�ú���
	// ����ÿ�������ȫ��������
	void SaveTeachDataEverySeam(int nGroupNo, int nSeamNo, double dExAxlePos, vector<T_TEACH_DATA>& vtTeachData);
	//��ȡ����ͣ��λ��
	bool GetGantryPos(vector<LineOrCircularArcWeldingLine>& vtSeamGroup, double& dExAxlePos, double& dPartMaxHeight,double dTrackPos, double dPartMinHeight, CvPoint3D64f tTotalValue, CvPoint3D64f tTotalDirValue);
	// ������ٺ��ӹ켣�����ó�ʼ�ι켣�ͽ�β�㲹�������켣�󱣴棬���ڸ�����������ݽ��к���
	bool CalcAndTrackWeldingTry(int nGroupNo);
	// �жϺ��������Ƿ���ڸ��ٺ���
	bool DetermineGroupMode(int nGroupNo);
	

public:
	// �Ȳ���豸�������죬�ָ�Ϊ�̺���
	void Segmentation();
	

};

#endif