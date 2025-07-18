/*********************************************************************
* ��Ȩ���� (C)2018, �������н����ܻ����˹ɷ����޹�˾
*
* �ļ����ƣ� ScanWeldLine.h
* ����ժҪ�� �Ȳ�󺸺���ȫɨ�躸����
* ��ǰ�汾�� 1.0
* ��    �ߣ� �����
* ������ڣ� 2024��8��21��
* ����˵����
*
* �޸ļ�¼1��
*
* �޸ļ�¼2��
*
ע�⣺


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
	// ��ȡ�������� �ļ���ʽ:��� x y z dirx diry dirz ��ȡ�����vtContour��xyz ���򱣴���drx dry drz��; dbx dby dbzΪ0
	bool LoadContour(int nWeldIndex, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour);
	//
	bool LoadContourByJson(int nWeldIndex, int nGroupNo, CString sContourFile, std::vector<T_ROBOT_COORS>& vtContour);
	// ���㲢�ֽ���������vtCoord�е���������x y z �� x y z dbx dby dbz ; dExAxlePos:�ֽ�����ⲿ������
	void DecomposeExAxle(std::vector<T_ROBOT_COORS>& vtCoord, double& dExAxlePos, double& dExAxlePos_n, int trackAxisNo);
	// �켣�˲�(xyz)�����������̬
	bool TrackFilter(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	bool TrackFilter_EX(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	bool TrackFilter_weld(const std::vector<T_ROBOT_COORS>& vtSrcCoord, std::vector<T_ROBOT_COORS>& vtRstCoord, int nFilterOutInterval = 8, int nCalcPosturePtnStep = 8);
	// �����nGroupNo�麸�������ǹ�������굽�ļ�
	void SaveMeasureTrackGunTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord);
	// �����nGroupNo�麸���������������굽�ļ�
	void SaveMeasureTrackCamTool(int nGroupNo, const std::vector<T_ROBOT_COORS>& vtCoord, const std::vector<T_ANGLE_PULSE>& vtPulse, const std::vector<int>& vtPtnType);
	// ������tSrcTool����������vtCoord��ת��Ϊ���ߵ�����
	bool TransCoordByTool(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool);
	// �� vtCoord �� ǰ����ӹ��ɵ����� �� ����
	void AddSafeCoord(std::vector<T_ROBOT_COORS>& vtCoord, vector<int>& vnType, double dMaxPartHeightZ);
	//
	void CurveScanSide(WeldLineInfo tWeldInfo, double dExAxlePos, double dExAxlePos_y, std::vector<T_ROBOT_COORS>& vtMeasureCoord, double pExLinepExLine, double angle);

	void CalcCorvrAngle(bool bBelongStart, bool bBelongPtnFree, bool bOtherPtnFree, T_ROBOT_COORS& tCoord, double dSeamLen);

	// �˶��ⲿ��
	bool MoveExAxleToDst(double dPos, int nAxisNo, double dSpeed);

	// ɨ���� m_pTraceModel->vtConnerWorldCoords �� ��ȡ��� ���浽 m_vtTeachResult��
	bool GetScanWeldLineResult();

	// �ɼ��߳�
	static UINT ThreadMeasure(void* pParam);

	void WeldInfoToJson(Welding_Info tWeldInfo);

	void BackHome();

	// �ֲ����ƴ���
	Local_Welding_Info PartPointCloudProcess(vector<T_ROBOT_COORS>& vtCoord, int nGroupNo);

	vector<T_ROBOT_COORS> generateTrajectory(double startX, double startY, double startZ, double endX, double endY, double endZ);

	double m_dScanSpeed; // ɨ������ٶ�
	double m_dMaxLength; // �ֶγ���
	int m_isOneWled;
	int m_VisionType;	// �Ӿ������ļ�
	double m_LiRon;		// �Ǹ����ؿ��
	bool m_bIsCake;     //1���ߺУ�0����
	double m_conRz;
	double holesize;
	int m_iscircle;
	bool isMeasureMove;
	int m_nScanNum;
	int m_nWeldHole;//������
	std::vector<T_ROBOT_COORS> m_vtContour;
};

#endif // !_SCAN_WELD_LINE



