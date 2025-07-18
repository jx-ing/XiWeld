/*********************************************************************
* ��Ȩ���� (C)2018, �������н����ܻ����˹ɷ����޹�˾
*
* �ļ����ƣ� WeldAfterMeasure.h
* ����ժҪ�� �Ȳ��ͳһ��ʶ������������̽ӿ� �������㺯�� 
* ��ǰ�汾�� 1.0
* ��    �ߣ� �����
* ������ڣ� 2023��6��15��
* ����˵����
*
* �޸ļ�¼1��
* 
* �޸ļ�¼2��
* 
ע�⣺

**********************************************************************/

#pragma once
#include <vector>
#include <map>

#include ".\Apps\PLib\CtrlUnit\CUnit.h"
#include "XiAlgorithm.h"
#include "AbsCoorTransLib.h"

#include "GsImageProcess.h"
#include "WeldingLinesRecognition.h"
#include "PointCloudWeldingLinesExtraction.h"
#include "RiserManagement.h"

#include "ScanInitModule.h"
//#include "WeldParamProcess.h"
#include "Project/ParamInput.h"
//#include"StatisticalData.h"
#include "Project/WeldParamProcess.h"
#include "Project/IOCtrlDlg.h"
#include ".\Apps\PLib\LeisaiCtrl\StepMoveControlJY.h"
#include ".\LocalFiles\ExLib\ALGO\include\MoveHandle_C_DLL.h"
#include ".\LocalFiles\ExLib\ALGO\include\TrackFilterHandle_C_DLL.h"

#include "xi_export_ROBOTCOLLIDECLOSEDPLANE.h"
#include "robot_collide_closed_plane_interface.h"

using namespace std;

struct WeldSeamAtrribute
{
	int		nWeldSeamIdx; // ��������
	int		nIsDoubleWelding;
	int		nStartWrapType;// ���ɶˣ��ݶ�0����������1����㶨��������յ㲻��������2���յ㶨���������㲻��������3�����յ㶼�������
	int		nEndWrapType;// �����ֶ�����
	double 		nWeldAngleSize;
	double dStartHoleSize;
	double dEndHoleSize;
	int	   nRobotSelete;
	double dThoeryLength;// ���۳���
	double dGroupTrackPos;
	int	   nGroupNo;   // ͬһ���ڲ�ͬС��
	int    nBigGroupNo;// ���飬��ͬ��е��ͬʱ����λ��
	bool   bStartFixScan = false; // �Ȳ��Ĭ��false
	bool   bEndFixScan = false; // �Ȳ��Ĭ��false
	bool   bWeldMode = false; // ���ӷ�ʽ��true:���٣�false:�Ȳ�� 
	int	   nScanEndpointNum = 0; // �Ѷ˵����

};

struct WeldLineInfo
{
	LineOrCircularArcWeldingLine tWeldLine;
	WeldSeamAtrribute tAtrribute;
};

struct TempStruct
{
	int nIdx;
	T_ROBOT_COORS tCoord;
};

template<typename T>
inline bool WeldingLineIsArc(T tValue)
{
	return tValue.IsArc > 0 && tValue.IsArc < 10;
}

namespace WAM
{
	#define CHECK_EXAXLE_RETURN(pMoveCtrl, dExAxlePos) \
	if (false == CompareExAxlePos(pMoveCtrl, dExAxlePos)) \
	{ \
		return false; \
	}

	class WeldAfterMeasure
	{
	public:
		WeldAfterMeasure(CUnit* ptUnit, E_WORKPIECE_TYPE eWorkPieceType);
		~WeldAfterMeasure();

		/********************* ����ͨ�ù��ܺ�����ֲ *********************/
	public:
		// ���������� vtNewTeachData ׷�ӵ� vtTeachData
		void AppendTeachData(vector<T_TEACH_DATA>& vtTeachData, vector<T_TEACH_DATA> vtNewTeachData);	
		// ����������ֽ�Ϊ�ⲿ������ͻ���������(��Ҫ֧�ֶ��ᣬĿǰ��֧��Y���ⲿ��)
		void DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos);
		void DecomposeExAxle(T_ROBOT_COORS& tRobotCoord, double dExAxlePos, int nExAxleNo);
		// ��������tTeachCoord�����Ƿ��Ѵ�����vtTotalTeachCoord�� tThresholdΪ��ֵ ������vtTotalTeachCoord�е����� �����ڷ���-1
		int CheckTeachCoord(const T_ROBOT_COORS& tTeachCoord, const vector<T_ROBOT_COORS>& vtTotalTeachCoord, T_ROBOT_COORS tThreshold);
		// ��ȡ����tTeachData���ַ���
		CString GetTeachDataString(const T_TEACH_DATA& tTeachData);

		long GetMoveExAxisPos(T_ANGLE_PULSE tPulse, int nAxisNo);
		double GetMoveExAxisPos(T_ROBOT_COORS tCoord, int nAxisNo);

		/********************* ��̬ͨ�ú��� *********************/
	public:
		static void SaveWorkpieceType(CRobotDriverAdaptor* pRobotCtrl);
		static void LoadWorkpieceType(CRobotDriverAdaptor* pRobotCtrl, std::map<int, CString> &m_nsPartType);

		/********************* �������ܺ��� *********************/
	public:
		// ����ϵͳ�豸����
		void LoadEquipmentParam();
		// ���غ��Ӳ�������
		void LoadCompParam();
		// ���ô����м����ݱ���·��
		void SetFilePath();
		// ���ú��Ӳ��������۾��� ������ݶ˵���� ���ӺͲ�����������̬Rx Ry
		void SetWeldParam(BOOL *pIsArcOn, BOOL *pIsNaturalPop, BOOL* pIsTeachPop, BOOL* bNeedWrap);
		// ����Ӳ����Դ
		void SetHardware(/*CMoveCtrlModule* pMoveCtrl, CGroupLaserVision* pDHCameraVision, CIOControl* pIOControl, */CScanInitModule* pScanInit);
		// ���ص�������
		void LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, CvPoint3D64f* pPointCloud, int& nPtnNum);
		void LoadContourData(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f> &vtPointCloud);
		void LoadContourData3D(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f> &vtPointCloud);
		void SaveContourData();		//���������ݺϲ���һ���ļ���
		// ������ƴ�����
		bool SavePointCloudProcessResult(LineOrCircularArcWeldingSeam* pWeldSeams, int nWeldLineNumber);
		bool SavePointCloudProcessResult(LineOrCircularArcWeldingLine* pWeldSeams, int nWeldLineNumber);
		bool SavePointCloudProcessResult(const Welding_Info &tWeldInfo);
		bool SaveSplitAfterPointCloudProcessResult(std::vector<WeldLineInfo> vtWeldSeamInfo, CString sFileName = "",bool IsArc = false);
		// ���ص��ƴ�����
        bool LoadCloudProcessResultNew(CString sFileName = "");
		bool LoadCloudProcessResult(CString sFileName, vector<WeldLineInfo>& vtWeldLineInfo); // ����ָ���ļ��еĺ�����Ϣ
		// ��ֵ��ƽ��,����ֱ�ߺ�Բ��
		bool SplitCloudProcessResult(CString sFileName = "");
		// ���溸�������
		void SaveWeldSeamGroupInfo();
		// ����ʾ�̽������
		virtual void SaveTeachResult(int nGroupNo);
		virtual void SaveTeachResultWorld(int nGroupNo);
		// ����ʾ�̽��
		bool LoadTeachResult(int nGroupNo, int nLayerNo = -1);
		bool LoadTeachResult(int nGroupNo, int nLayerNo, std::vector<T_TEACH_RESULT> &vtTeachResult);
		// ���غ��ӹ켣���� ��nGroupNo�� ��nWeldNo������
		virtual bool LoadRealWeldTrack(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE&eWeldSeamType, double &dExAxlePos, vector<T_ROBOT_COORS>& vtRealWeldTrack, vector<int> &vnPtnType);
		// ���켣vtRobotPulse������(XY) �����Ƿ�Զ������ԭ��
		bool CheckFlangeToolCoord(int nGroupNo, int nWeldNo, vector<T_ANGLE_PULSE> vtRobotPulse);

		bool AddMeasurePointsToLongSeam(int nWeldNo, std::vector<std::vector<T_TEACH_DATA>> vvtTeachData, LineOrCircularArcWeldingLine tWeldSeam, 
										double dPtnInterval, double dPostureRx, double dPostureRy, double dWeldDirAngle, E_WELD_SEAM_TYPE eWeldSeamType , vector<T_ROBOT_COORS> &vtWeldCoord );
		// ���溸��켣(�����󺸽ӹ켣)																																									 
		void SaveWeldTrack(int nGroupNo, int nWeldNo, int nLayerNo, E_WELD_SEAM_TYPE eWeldSeamType, double dExAxlePos, vector<T_ROBOT_COORS> vtWeldCoord, vector<int> vnPtnType);
		// �ж���������ǲ�ֵ�Ƿ�С��dThreshold
		bool JudgeDirAngle(double dDir1, double dDir2, double dThreshold =45.0) const;
		bool JudgeAngle(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2, double dThreshold) const;
		// ����ƽ����������ǵĽ�ƽ���� �����(ȡ�����������ֵ)
		double TwoDirAngleMedian(double dDirAngle1, double dDirAngle);
		double TwoAngleMedian(double dPositiveLimit, double dNegativeLimit, double dAngle1, double dAngle2);
		// ����� -> Rz
		double DirAngleToRz(double dDirAngle);
		double DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle);
		// Rz -> �����
		double RzToDirAngle(double dRz);
		double RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz);
		// ����ֱ������
		T_ROBOT_COORS GenerateRobotCoord(CvPoint3D64f tSrcPtn3D, double dRx, double dRy, double dRz);
		T_ROBOT_COORS GenerateRobotCoord(T_ALGORITHM_POINT tAlgPtn3D, double dRx, double dRy, double dRz);
		// ���ɶ������
		void GenerateMultipleCoord(T_ROBOT_COORS tStartCoord, T_ROBOT_COORS tEndCoord, vector<T_ROBOT_COORS>& vtCoords, double dInterval = 1.0);
		void GenerateMultipleCoord(XI_POINT tStartCoord, XI_POINT tEndCoord, vector<XI_POINT>& vtCoords, double dInterval = 1.0);
		// ����tSrcPtn ���tDirSPtnָ���tDirEPtn�ķ���ƫ�� dOffsetDis ��λ:mm
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_ROBOT_COORS tDirSPtn, T_ROBOT_COORS tDirEPtn, double dOffsetDis); 
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, CvPoint3D64f tDirSPtn, CvPoint3D64f tDirEPtn, double dOffsetDis);
		XI_POINT RobotCoordPosOffset(XI_POINT tSrcPtn, XI_POINT tDirSPtn, XI_POINT tDirEPtn, double dOffsetDis);
		T_ROBOT_COORS RobotCoordPosOffset(T_ROBOT_COORS tSrcPtn, T_LINE_PARA tLineParam, double dOffsetDis);
		T_ALGORITHM_POINT RobotCoordPosOffset(T_ALGORITHM_POINT tSrcPtn, T_ALGORITHM_POINT tDirSPtn, T_ALGORITHM_POINT tDirEPtn, double dOffsetDis);
		// ������ֱ������X Y Zƫ��
		void RobotCoordPosOffset(T_ROBOT_COORS& tSrcCoord, double dOffsetDir, double dOffsetDis, double dHeightOffsetDis = 0.0);
		// ��ά������ת��
		T_ALGORITHM_POINT TransPtn3D(XI_POINT tPtn);
		XI_POINT TtansPtn3D(T_ALGORITHM_POINT tPtn);
		// ����������˶��� ����������̬�仯С��180�ȵ� ����ؽ�����
		bool CalcContinuePulse(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse);
		bool CalcContinuePulseForWeld(vector<T_ROBOT_COORS> &vtWeldCoord, vector<T_ANGLE_PULSE>& vtWeldPulse, bool bWeldTrack = true);
		bool CalcContinuePulseForMeasure(vector<T_ROBOT_COORS>& vtRobotCoors, vector<T_ANGLE_PULSE>& vtRobotPulse, bool bWeldTrack = true);
		// �жϵ�nGroupNo�麸�ӹ켣�Ƿ���������˶�-������ɨ���
		bool CheckContinueMove_Offset(int nGroupNo, double dExAxisPos, double yPos, double offSet, double ry, double rx);
		// ʹ���õ���ʶ�������㺸�� �� �ж��Ƿ������������
		bool JudgeTheoryTrackContinueMove(int nGroupNo, /*double dCarPos, */double dExAxlePos, double yPos);
		// �жϵ�nGroupNo�麸�ӹ켣�Ƿ���������˶�
		bool CheckContinueMove(int nGroupNo, /*double dCarPos, */double dExAxisPos, double yPos);
		bool CheckContinueMove_rAngle(int nGroupNo, double dExAxisPos, double yPos, double ry, double rx);
		// �ⲿ������Ա�
		bool CompareExAxlePos(CMoveCtrlModule* pMoveCtrl, double dDstPos, double dLimit = 3.0);
		bool TransCoordByCamera(std::vector<T_ROBOT_COORS>& vtCoord, T_ROBOT_COORS tSrcTool, T_ROBOT_COORS tDstTool);
		// ���ݴ������͵��ò�ͬͼ����ӿ� ���� ��ά������
		bool ProcessTeachImage(int nTeachPtnNo, GROUP_STAND_DIP_NS::CImageProcess* pImgProcess, IplImage* pImg, T_ANGLE_PULSE tCapPulse, int nMeasureType, T_TEACH_RESULT& tTeachResult, E_FLIP_MODE eFlipMode);
		// ���ͼ�����ά����Ч��
		bool CheckImgProPtn2D(int nMeasureType, CvPoint tKeyPoint, vector<CvPoint> vLeftPtn, vector<CvPoint> vRightPtn);
		// ������ת����vtXiPoint׷�ӱ��浽vtAlgPtns
		void XiPointAddToAlgPtn(const vector<XI_POINT>& vtXiPoint, vector<T_ALGORITHM_POINT>& vtAlgPtns);
		// ������ƽ�潻��
		T_ALGORITHM_LINE_PARA PlanePlaneInterLine(T_ALGORITHM_PLANE_PARAM tPlane1, T_ALGORITHM_PLANE_PARAM tPlane2);
		// ����tWeldSeam�� ��� �յ� ��㷨�� �յ㷨��
		bool AdjsutWeldSeam(LineOrCircularArcWeldingLine& tWeldSeam, T_ALGORITHM_POINT tAlgStartPtn, T_ALGORITHM_POINT tAlgEndPtn, double dNorAngle, double dZSide);
		// ����ɨ������ĺ�����Ϣ
		bool AdjustScanWeldSeam(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData);
		// ������nGroupNo�� ��nWeldNo�����죬���κ���
		bool AdjustWeldSeamArc(int nGroupNo, int nWeldNo, std::vector<T_TEACH_DATA>& tTeachData);
		// �ж�LineOrCircularArcWeldingLine ��������
		E_WELD_SEAM_TYPE GetWeldSeamType(const LineOrCircularArcWeldingLine& WeldSeamType);
		// ��ȡ��ǰѡ���ƽ�����������ղ���
		bool GetCurWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, vector<T_WELD_PARA> &vtWeldPara);
		// ��ȡָ��ƽ�������� ָ���������Ƶ� ���ղ���
		bool GetWeldParam(E_WELD_SEAM_TYPE eWeldSeamType, int nWeldAngleSize, vector<T_WELD_PARA> &vtWeldPara);
		// ������ǹ켣 ��ע�ⲻͬ�������;���ʵ�ֲ�ͬ��
		bool CalcWrapTrack(WeldLineInfo tWeldLineInfo, E_WELD_SEAM_TYPE eWeldSeamType, double dStandWeldWrapLen, int nStandWeldDir, vector<T_ROBOT_COORS>& vtWeldCoord, vector<int>& vnPtnType);
		// ���ݷ���� + �߰��Ƿ����� �Զ������������Ƿ���ȱ����Ϣ������ȫͨ�ã�������������ʱ��ע�⣺dDirAngle��ʾ������򷴷���
		void AutoCalcStandWrapOffsetCoord(double dDis, double dDirAngle, bool bHeightBoardIsLeft, T_ROBOT_COORS& tRobotCoord);
		// �켣������Ӳ���ֵ
		void TrackComp(int nGroupNo, E_WELD_SEAM_TYPE eWeldSeamType, const vector<T_ROBOT_COORS>& vtSrcWeldCoord, const T_WELD_PARA& tWeldPara, vector<T_ROBOT_COORS>& vtAdjustWeldCoord);
		// �����nGrouopNo�麸����ߵ�Zֵ
		double CalcBoardMaxHeight(int nGroupNo);
		// �ж��Ƿ������� tVector���취����
		bool IsStandWeldSeam(CvPoint3D64f tVector);
		// �Է�ϰ庸��(С��4������)�Ĳ�ȫ������Ϣ ������ δ��������켣�ṩ��������
		bool ResetWeldSeamGroup(int nGroupNo, vector<LineOrCircularArcWeldingLine>& vtWeldLineSeam);
		// �жϻ�ͷ�͹����Ƿ�����
		bool JudgeCollision(double dRy, double dAngle, double dBackH, double dLH, double dRH, double dFrontBackDis, double dLRDis, double dGunSafeDis, double dSafeDis);
		// ʹ��ʾ�̽��vtTeachResult(����������ֱ�ߵ�ʾ�̽��)
		bool AdjustLineSeamTrack(E_WELD_SEAM_TYPE eWeldSeamType, const vector<XI_POINT>& vtScanPtns, vector<T_ROBOT_COORS>& vtWeldTrack);
		// ��������ؽ�����Ĺ��ɵ�(������ֹ����̬��λ�����˶�����-90��ķ���ƫ��) ������������㹻����
		bool CalcTwoPulseTempPtn(T_ANGLE_PULSE& tTempPulse, T_ANGLE_PULSE tStartPulse, T_ANGLE_PULSE tEndPulse, double dLevelOffsetDis, double dHeightOffset = 0.0);
		// �жϸ��麸���Ƿ�Ϊ��ຸ��(�������찴�ڲ⺸�촦��)
		bool JudgeOutsideWeld(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam);
		// ��������������ƽ�������ţ�������������˳���������(��֤������ӹ켣˳�����Ӻ󼴿���������)
		bool GetContiueWeldOrderFlat(const vector<LineOrCircularArcWeldingLine>& vtWeldSeam, vector<int>& vnFlatWeldNoIdx);

		/********************* �̼߳��̺߳��� *********************/
		bool WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName);
		static UINT ThreadTeachProcess(void* pParam);
		bool TeachProcess(int nTimeOut = 20000); // ����nTimeOut����û��ʾ��ͼ ��ʱ�˳�

        //��¼ʱ��
        double RecordRunTime(CRobotDriverAdaptor *pRobotCtrl, long ltimeBegin, CString str);
		//�������˼�ͣ
		void CheckMachineEmg(CRobotDriverAdaptor *pRobotCtrl);
        bool WeldTrackLock(CRobotDriverAdaptor* pRobotCtrl, T_ROBOT_COORS tStartP, T_ROBOT_COORS tEndP, bool bIsTrack);
		bool IsWorking();

		/********************* �¿���ҵ���� *********************/
		bool FreeWaveGrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate);
		bool DoFreeGrooveWelding(std::vector<T_ROBOT_COORS>& vtWeldPathPoints, T_WAVE_PARA tWavePara, const T_WELD_PARA& tWeldPara, vector<double> weldSpeedRate);
		int m_nGroupNo; // �¿�ʹ�� ��ǰ�������
		int m_nLayerNo; // �¿�ʹ�� ��ǰ���Ӳ��(�˴�ָ����)

		/********************* �˶������ҵ���� *********************/
	public:
		//// �����˶��켣���� �켣������ ���� �ⲿ��λ�õ�ʾ������
		void SetTeachData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, double& dExAxlePos_y);
		//// ִ�в����˶�
		virtual bool DoTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos, int nLayerNo = 0);
		// ִ��������ͼ
		virtual bool SpuriousTriggerTeach(int nGroupNo, const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const vector<int>& vnMeasureType, double dExAxlePos);
		//// ִ�к����˶�
		bool Weld(int nGroupNo);
		//// �Ȳ��ͨ�ú���
		virtual bool DoWelding(int nGroupNo, int nWeldNo, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const vector<int>& vnPtnType, const T_WELD_PARA &tWeldPara);
		// vtPulse:��ǹ ������� �����յ� ��ǹ nDownBackSpeed����ǹ�ٶ� nScanSpeedɨ���ٶ� ����˵㵽tTeachResult.tKeyPtn3D
		virtual bool ScanEndpoint(int nGroupNo, int nCameraNo, vector<T_ANGLE_PULSE> vtPulse, vector<int> vnType, vector<T_TEACH_RESULT>& tTeachResult);
		// ��ȡScanEndpoint�Ĳ������
		bool GetScanProcResult(int nGroupNo, bool bIsScanMode, bool bSinglePointMeasure, vector<T_ROBOT_COORS> vtCoord, vector<T_TEACH_RESULT>& vtTeachResult);
		// �ں�����vtSeamGroup�л�ȡ��nSeamNo����������ں��� ��nSeamNo��ƽ��������ƽ������յ����ڵ�ƽ������  ��nSeamNo���������������������������ں��յ�������������ڵ�ƽ������
		void GetAdjoinSeam(int nSeamNo, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE);
		void GetAdjoinSeam(LineOrCircularArcWeldingLine tSeam, vector<LineOrCircularArcWeldingLine>& vtSeamGroup, LineOrCircularArcWeldingLine** ptAdjoinSeamS, LineOrCircularArcWeldingLine** ptAdjoinSeamE);
		// ������Ǹ������̬�켣ɾ����Ϣ
		void CalcInterfereInfo(int nSeamNo, vector<LineOrCircularArcWeldingLine> vtWeldSeam, vector<T_ROBOT_COORS>& vtWeldCoord,
			double& dChangeDisS, double& dChangeDisE, int& nChangePtnNum, int& nDelPtnNum, double& dWeldHoleSizeS, double& dWeldHoleSizeE);
		// �ж�ʱ��Ϊ��Ǻ���
		bool IsAcuteSeamStand(LineOrCircularArcWeldingLine tSeam, double dAcuteAngleThreshold);
		// ��ȡ��ǰ���� ���к�����Ϣ��ߵ� (�������� �ѿ�����������)
		XI_POINT GetPieceHeight(int nGroupNo, double zOffeset);

		int m_nSmallGroupNoForTeach = -1;//����ʱ����ǰ�ǵڼ�С��ĺ���

		/********************* ��������ʵ�ֵĲ��� *********************/
	public:
		// ����ʶ�����
		virtual void SetRecoParam() = 0;
		// ���ƴ���
		virtual bool PointCloudProcess(bool bUseModel, CvPoint3D64f* pPointCloud = NULL, int PointCloudSize = 0) = 0;
		// ���ƴ��� ģ��
		bool PointCloudProcessWithModel();
		
		// ���ƴ����������������
		virtual bool WeldSeamGrouping(int& nWeldGroupNum) = 0;
		// ��������˶��켣
		virtual bool CalcMeasureTrack(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos, double& dSafeHeight) = 0;
		//// ���㾫ȷ����켣 dWeldHoleSize:�����׳ߴ�
		virtual bool CalcWeldTrack(int nGroupNo) = 0;

		/********************* ��̬���� *********************/
		public:
		// ���ݹ������ͻ�ȡ�������� ���������������� �� �����������
		static bool GetWorkPieceName(E_WORKPIECE_TYPE eWorkPieceType, CString& sWorkPieceName, CString& sWorkPieceTypeName);
		// ����������봰��
		static void InputParaWindow(CString sUnitName, E_WORKPIECE_TYPE eWorkPieceType);
		
		/********************* ������ʵ�ֵĲ��� *********************/
	public:
		// ������յ����꼰��̬����ֱ�����꺸��켣���� bTheoryCheck:�Ƿ���ʹ��ʶ�𺸷��������ۺ��ӹ켣(��������켣��麸�ӹ켣�Ƿ�������˶�ʹ��true)
		virtual bool GenerateWeldLineData(int nGroupNo, double dWeldHoleSize, double dBoardThick, bool bTheoryCheck = false);
		// ����һ������ĺ���켣
		//bool GenerateWeldLineData(const LineOrCircularArcWeldingLine& tWeldSeam, int nGroupNo, int nWeldNo, double dWeldHoleSize, double dBoardThick);
		bool GenerateWeldLineData(const LineOrCircularArcWeldingLine& tWeldSeam, int nGroupNo, int nWeldNo, double dWeldHoleSize, double dBoardThick, std::vector<XI_POINT> vtScanPtns);

		bool GenerateWeldLineData(CvPoint3D64f tStartPtn, CvPoint3D64f tEndPtn, double dInterval, double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors);
		bool GenerateWeldLineData(XI_POINT tStartPtn, XI_POINT tEndPtn, double dInterval, double dRx, double dRy, double dWeldNorAngle, E_WELD_SEAM_TYPE eWeldSeamType, std::vector<T_ROBOT_COORS>& vtRobotCoors);
		// ����Բ�����ӹ켣
		bool GenerateWeldTrackArc(LineOrCircularArcWeldingLine tWeldLine);
		// ���ݹ켣�����ɻ����������
		bool CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
			double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize, double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<T_ROBOT_COORS>& vtCoutCoors);
		//bool ModelMatching(double dCorasrMatrix[][4], std::string ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[][4]);
		bool CalcRobotPostureCoors(vector<XI_POINT> vtOutPoints, double dRX, double dRY, double dStepDis, double dStChangePostureDis, double dStChangeAngle,
			double dEdChangePostureDis, double dEdChangeAngle, bool bStartType, bool bEndType, double dStartHoleSize, double dEndHoleSize, int& nStartChangeStepNo, int& nEndChangeStepNo, vector<XI_POINT>& vtPosture);

		/********************* ���Ժ��� *********************/
	public:
		// ���Ժ���
		void SaveCoordToFile(std::vector<T_ROBOT_COORS> vtCoord, CString sFileName);
		// ��������ʾ�̽������(����ʹ��)
		bool GeneralTeachResult(int nGroupNo, std::vector<T_ROBOT_COORS>& vtMeasureCoord, std::vector<T_ANGLE_PULSE>& vtMeasurePulse, vector<int>& vnMeasureType, double& dExAxlePos);
		void Test();
		// ����ʹ�ó��������Job
		void GenerateJobSpotWeld(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName);
		void GenerateJobLocalVariable(vector<T_ANGLE_PULSE> vtRobotPulse, int nMoveType, CString sJobName, int nStep = 10);
		void GenerateJobCoord(vector<T_ROBOT_COORS> vtRobotCoord, int nMoveType, CString sJobName);
		void GenerateJobForContinueWeld(int nGroupNo, bool bUsedExAxle = false);
		//void GenerateFilePLYPlane(double dExAxisPos, CString sName = "AllWeldSeamPlane");
		void GenerateFilePLYPlane(vector<WeldLineInfo> vtWeldSeamPlane, CString sName = "AllWeldSeamPlane");
		void GenerateFilePLY(int nGroupNo, double dExAxlePos = 0.0, CString sName = "AllWeldSeam");
		void GenerateFilePLY(CvPoint3D64f* pPointCloud, int PointCloudSize, CString sFileName, int nSample = 2, double dExAxlePos = 0.0);

		void SaveToJobBuffer(const vector<T_ANGLE_PULSE> &vtPulse);
		void CleaeJobBuffer();
		vector<vector<T_ANGLE_PULSE>> m_vvtJobPulse;
		// ��������ָ���㼰Z����ת180��
		void PointCloudRotate180(vector<CvPoint3D64f> &vtPtns, double dCenX, double dCenY);
		// ��ʾ�̽����ָ���㼰Z����ת180��
		void TeachResultRotate180(double dCenX, double dCenY);

		// ģ��ƥ��õ���ͬ���������к��켰����������Ϣ
		bool GenerateTemplate();
		void TemplateMatch();

		//����ֵ:��ͬһ����ϵ�´�ԭʼλ�õ�Ŀ��λ�õ�����任����
		//BasePoints:ԭʼλ���ĸ���Ӧ��������ϵ�µ�����
		//TargetPoints:Ŀ��λ���ĸ���Ӧ��������ϵ�µ�����
		/*���ĸ��㹲��,�����BaseCoordinateMatrix������,������󲻴���,�ڼ���BaseCoordinateMatrix_1ʱ���õ�����Ľ��*/
		cv::Mat CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints);
		//����ֵ:ԭʼλ�õ�任��Ŀ��λ�ú�Ľ������ͬһ����ϵ�µ�����
		//BasePoints:ԭʼλ�õ�����
		//TransformationMatrix:����任����
		std::vector<CvPoint3D64f> CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);
		//����ֵ:��ά����ָ����ת�ᰴ��������������ʱ����תָ���ǶȺ����ά����
		//Point:��ά������
		//RotationCenter:��ת���������
		//RotationAxis:��ת����������
		//RotationAngle:��ת��(��λ����)
		CvPoint3D64f RotatePoint3D(CvPoint3D64f Point, CvPoint3D64f& RotationCenter, CvPoint3D64f& RotationAxis, double RotationAngle);
		XI_POINT RotatePoint3D(XI_POINT Point, XI_POINT& RotationCenter, XI_POINT& RotationAxis, double RotationAngle);

		double calculateAngle(XI_POINT vector1, XI_POINT vector2);
		bool CalcLineLineIntersection(T_ROBOT_COORS tStart, double dStartAngle, T_ROBOT_COORS tEnd, double dEndAngle, XI_POINT& intersection);
		void GetChangePosturePare(vector<T_ROBOT_COORS> tLeftTrackCoors, XI_POINT tLeftCenter, double dLeftNormal, bool bLeftIsArc, vector<T_ROBOT_COORS> tRightTrackCoors, XI_POINT tRightCenter, double dRightNormal, bool bRightIsArc, bool bStartOrEnd,
			int& nStartStepNo, double& dStChangeAngle, double dChangePostureThresVal);
	public:
		// �Ƿ���б��ص���
		bool m_bIsLocalDebug;
		// ���ݴ���ͼ������
		IplImage** m_pColorImg;

		vector<int> m_vtImageNum;


		int m_ChangeDisPoint;

	/*protected*/public:
		CUnit* m_ptUnit;
		/*=====�ϸ��������======*/
		TraceModel* m_pTraceModel = new TraceModel();
		/*=====�ϸ��������======*/
		CRobotDriverAdaptor *m_pRobotDriver;
		CScanInitModule* m_pScanInit;
		int m_nCameraOpenNumber; // ��¼��ǰ����򿪵����� ���һ������ص�����ҪCloseLib

		CString m_sPointCloudFileName; // ���������ļ�·�����ļ���
		CString m_sDataSavePath; // �м����ݱ���·��
		std::vector<LineOrCircularArcWeldingLine> m_vtWeldSeamData; // ���ƴ���õ��ĺ�����Ϣ
		std::vector<WeldLineInfo> m_vtWeldSeamInfo; // ����ʶ���������ŵ��޷�ʶ���������Ϣ
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroup; // �����ĺ�����Ϣ
		std::vector<std::vector<WeldLineInfo>> m_vvtWeldLineInfoGroup; // ��m_vvtWeldSeamGroupһһ��Ӧ�ĺ��츽����Ϣ
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroupAdjust; // ��ȷ���� ������ ���������Ϣ
		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtCowWeldSeamGroupAdjust; // ��ȷ���� ������ ���������Ϣ

		std::vector<std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamGroupOrg; // ԭʼ�������� ���������Ϣ�����ҳ����������ʵ��

		std::vector < std::vector<LineOrCircularArcWeldingLine>> m_vvtWeldSeamData; // ������ƴ���õ��ĺ�����Ϣ
		std::vector < std::vector<WeldLineInfo>> m_vvtWeldSeamInfo; // �������ʶ���������ŵ��޷�ʶ���������Ϣ

		double m_WeldSeamGroupAngle;// �ӽ����y����б�Ƕ� ��Ϊǰ�ߺ��

		// ���ƴ������
		bool m_bDeleteSeam;				// �Ƿ�ɾ����帹�庸��
		double m_dZPallet;				// ƽ̨���޹���λ�õ�Zֵ, ���ڹ��˵���
		int m_nScanTrackingWeldEnable;
		double m_dSideBoardThick;		// H�͸ֲ����		// ���ƴ������
		double m_dBoardLen;				// �г������ �� ���н�峤��(�����޽��ʱʹ��)
		double m_dWeldHoleSize;			// �����״�С
		double m_dEndBoardSupportLen;	// �˰�֧�ų���
		double m_dPurlinSupportLen;		// ����֧�ų���
		double m_dPurlinLen;			// ���г���
		double m_dEndBoardLen;			// �˰峤��
		int m_nExPartType;				// ���ָ��⹤������ 1������   2������  (���ָֺ��� 0:С���ӿ� 1:ţ�Ƚӿ�)
		double m_dScanEndpointOffset;	// �˵������������ ������ ������
		double m_dMaxWeldLen;			// ���ָ��Ȳ�󺸺�����󳤶�
		double m_dJointLen;				// ���ָ��Ȳ�󺸽�ͷ������
		int m_nWeldMode;				// ����ģʽ��0ֱ��1���� (���ָ�ר��)

		// ���ٲ���
		double m_dStartDelLen; // ���ɾ������
		double m_dEndDelLen; // �յ�ɾ������

		// ���Ӳ���
		bool m_bWorking;					// �Ƿ����ڹ����� (˽�г�Ա ʹ��IsWorking��������)
		BOOL* m_pIsArcOn;					// �Ƿ���
		BOOL* m_pIsNaturalPop;				// �Ƿ�����ʾ���� 
		BOOL* m_pIsTeachPop;				// �Ƿ���ʾ�̵���
		BOOL* m_bNeedWrap;					// �Ƿ����
		int m_nRobotInstallDir;				// �����˰�װ����
		double m_dGunAngle;					// ��ǹ�Ƕ�
		double m_dGunLaserAngle;			// ��ǹ�ͼ����߼н�
		double m_dGunCameraAngle;			// ��ǹ���������н�
		double m_dRotateToCamRxDir;			// ��ͷת��������Rx�仯���� ����1 ��С-1
		double m_dHandEyeDis;				// ���۾��룺����ǰ���������
		double m_dTrackCamHandEyeDis;		// ����������۾��룺����ǰ���������
		double m_dMeasureDisThreshold;		// ��������뺸������յ������ֵ
		double m_dPlatWeldRx;				// ��׼�����ͺ�����̬Rx
		double m_dPlatWeldRy;				// ��׼�����ͺ�����̬Ry
		double m_dNormalWeldRx;				// ����ƽ�Ǻ�����̬Rx
		double m_dNormalWeldRy;				// ����ƽ�Ǻ�����̬Ry
		double m_dStandWeldRx;				// ��׼������̬Rx
		double m_dStandWeldRy;				// ��׼������̬Ry
        double m_dTransitionsRx;			// ���ȵ���̬Rx
        double m_dTransitionsRy;			// ���ȵ���̬Ry
		double m_dStandWeldScanRx;			// ����ɨ����̬Rx
		double m_dStandWeldScanRy;			// ����ɨ����̬Ry
		double m_dStandWeldScanOffsetRz;	// ��׼����ɨ����̬Rzƫ��
		double m_dGrooveScanRx;				// �¿�ɨ����̬Rx
		double m_dGrooveScanRy;				// �¿�ɨ����̬Ry
		double m_dGrooveScanOffsetRz;		// �¿�ɨ����̬Rzƫ��
		double m_dGrooveWeldRx;				// �¿ں�����̬Rx
		double m_dGrooveWeldRy;				// �¿ں�����̬Ry
		double m_dGrooveWeldRz;				// �¿ں�����̬Rz
		double m_dWeldNorAngleInHome;		// �����˰�ȫλ�ùؽ�����״̬ ��Ӧ�ĺ��취���
		double m_dEndpointSearchDis;		// ���ɶ��������쳤��(�˵��򺸷��ں���ƫ�ƾ���)
		double m_dGunDownBackSafeDis;		// ����ǹ��ȫ����
		double m_dLengthSeamThreshold;		// ����ƽ�峤����ֵ(��������һ��)
		double m_dShortSeamThreshold;		// �Ȳ�����¼������λ�õĶ̱߳�����ֵ
		double m_dPointSpacing;             // ����ƽ����֮�����������
		double m_dCleanGunDis;				// ���Ӷ೤����һ����ǹ��˿ ��λ:��
		bool m_bFlatWeldContinue;			// ��β����ƽ���Ƿ��������ӣ���Ҫ���ǹ�����ɾ�� ����ɾ�� ��ͬ���գ�
		bool m_bCheckCollide;				// ��ͷ������ʹ��
		bool m_bTrackingEnable;				// �Ƿ����ø��ٺ��ӹ���
		double m_dTrackingLengthThreshold;	// �����೤����ʹ�ø��ٺ���
		bool m_bAutoCalcWrapDirStand;		// �Զ������������Ƿ���
		T_ROBOT_COORS m_tRobotHomeCoors;	// �����˰�ȫλ�ùؽ������Ӧ��ֱ������(��ǹ����)�����캯����ʼ��
		bool m_bIsSlope;					//��������
		//UINT* m_pUnTracePointCloudProcess;	// �Ƿ�ʹ�ø��ٵ��ƴ����Ѷ˵�
		UINT* m_pUnWarpBoundWeld;			// �Ƿ������ǹ
		//BOOL* m_pIsSaveTraceScanPic;		// �Ƿ���������ԭͼ
		//�ٶȲ���
		double m_dUltraTransitionSpeed;		//�ߺ��ӹ��ɵ��ٶȣ�����ٶ�
		double m_dSafePosRunSpeed;//��ȫλ���ƶ��ٶȣ�����
		double m_dFastApproachToWorkSpeed;//���ٿ��������� ����
		double m_dSafeApproachToWorkSpeed;//��ȫ�ƶ��������ٶȣ�����
		double m_RobotBiasDis;			  // ������ƫ�þ��룬����ʱʹ������X��Y��̶���m_RobotBiasDisλ��

		// ʾ�̲���
		int m_nTeachPtnNum;					// ʾ�̵���
		double m_dTeachExAxlePos;			// ʾ��ʱ�ⲿ��λ��
		double m_dTeachExAxlePos_y;			// ʾ��ʱy�����ⲿ��λ��
		double m_dPauseExAxlePos;
		double m_dPauseExAxlePos_y;
		vector<int> m_vnTeachIdxWithTemp;	// ���в������ڰ������ɵ�켣�е�����
		vector<int> m_vnMeasurePtnType;		// ������ʾ�̵�����ͬ (���������ɵ�)
		vector<T_ANGLE_PULSE> m_vtTeachPulse;	// ʾ��λ�ùؽ����� ������ʾ�̵�����ͬ(���������ɵ�)
		vector<T_TEACH_RESULT> m_vtTeachResult;	// ʾ�̲����������ά�� ��ά���� ��ͼֱ�� ��ͼ�ؽ� ��ͼ�ⲿ�� 

		// ���ӹ켣����
		map<int, double> m_mdFlatHorComp;	// ƽ��ˮƽ���򲹳�
		map<int, double> m_mdFlatHeightComp;	// ƽ���߶Ȳ���
		map<int, double> m_mdStandLenComp;  // �������쳤�Ȳ���
		map<int, double> m_mdStandVerComp;	// ������˿��ֱ���򲹳�

		/********************* ͨ���Ȳ����� *********************/
		
		vector<T_TEACH_DATA> m_vtTeachData; // �������� �����˶����� �� ����켣ʹ�õ���ز���
		vector<int> m_vnTeachTrackOrder; // ʾ�̹켣�����˶���������� �ָ�˳���������ʾ�̽�������˳��ʹ�ã�
		E_WORKPIECE_TYPE m_eWorkPieceType; // �������ֲ�ͬ���͹������ò�ͬ�Ӿ��ӿ�

		// ���Բ���
		bool m_bWorkpieceShape; // ������״
		double m_dOverflowHoleStart;// ��ˮ��
		double m_dOverflowHoleEnd;// ��ˮ��

		void   WeldSeamTransCoor_Gantry2Robot(std::vector<LineOrCircularArcWeldingLine>& vtWeldSeamData, std::vector<WeldLineInfo>& vtWeldSeamInfo);	//������Ϣ��������ϵת����������ϵ
		// �Զ����飬��ʱ����ʹ�ã�������ԭ���麯��
		bool   WeldSeamGroupingAuto(int& nWeldGroupNum);
		// ���亸�캸�ӷ�ʽ������/�Ȳ��
		void DetermineWeldingMode();
		// ģ��ƥ��
		//bool ModelMatching(TransformMatrix *TransMat, std::string ModelFileName1, std::string ModelFileName2, std::string SaveRoute_weldline_point);
		
		// ���ݽ�β���������ǹ�켣
		bool CalcJumpWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
			std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType);
		// ��ǹ��ǹ�켣
		void GenerateWrapBoundTrack(T_ROBOT_COORS tWrapStart, T_ROBOT_COORS tWrapEnd, vector<T_ROBOT_COORS>& vtRobotCoor,
			vector<int>& vtPtType,double dWorkpieceHight ,double dInsertInDis = 8.0, double dInsertOutDis = 8.0, double dBackGunPercent = (double)(6.0), double dDirRotation = 0.0);
public:
	/********************* ��ײ������ *********************/
	//// ��ʼ����ײ������ �� ���û�ͷģ��
	//void InitCheckCollide(CString sModelFilePath);
	//// ������ײ���ƽ��
	//void SetCheckCollidePlane();
	//// ���ָ�������Ƿ������õ�ƽ����� tCoord:�������� bOutputStep:�Ƿ���������Step�ļ� dCheckDis:������
	//bool CheckIsCollide(T_ROBOT_COORS tCoord, bool bOutputStep = false, double dCheckDis = 1000.0);
	//// ���ָ�������Ƿ������õ�ƽ����� tPulse:�������� bOutputStep:�Ƿ���������Step�ļ� dCheckDis:������
	//bool CheckIsCollide(T_ANGLE_PULSE tPulse, bool bOutputStep = false, double dCheckDis = 1000.0);
	//// ���ָ�������Ƿ������õ�ƽ����� vtCoord:���������� dExAxis:�ⲿ������ bOutputStep:�Ƿ���������Step�ļ� dCheckDis:������
	//bool CheckIsCollide(vector<T_ROBOT_COORS> vtCoord, double dExAxis, bool bOutputStep = false, double dCheckDis = 1000.0);
	//xi::RobotCollideClosedPlaneInterface* m_pCollide;
	//vector<T_RECT_POINT> m_vtPartPlaneRects;


public:
	/********************* �ϵ�������� *********************/
	bool GetPauseWeldTrack(vector<T_ROBOT_COORS>& vtCoord, vector<int> &vnPtnType, E_WELD_SEAM_TYPE eSeamType);		 // ������ͣλ�ý�ȡ�������ӹ켣
	void SavePauseInfo(int nGroupNo, int nWeldNo, int nLayerNo); // ������ͣ״̬��Ϣ
	void LoadPauseInfo();			// ������ͣ״̬��Ϣ
	int m_nPauseGroupNo;			// ��ͣʱ���
	int m_nPauseWeldNo;				// ��ͣʱ�����
	int m_nPauseLayerNo;			// ��ͣʱ���Ӳ��
	T_ANGLE_PULSE m_tPausePulse;	// ��ͣʱ��������������

	//ֱ�߹켣Ԥ��
	bool TrajectoryLineTrack(std::vector<T_ROBOT_COORS>& vtOutputTrack, T_LINE_PARA tRansacLine, T_ROBOT_COORS tRealStart, XI_POINT tDealEnd, int nPointCount, double dAdjustDis);
	std::vector<XI_POINT> m_vtTrackTheoryTrack;//�������۹켣

	//���㺸���ɨ��켣
	bool CalcScanTrack(LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double& dExAxlePos);


	std::vector<T_ROBOT_COORS> CalLineDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, double dStep);
	//jwq��ʱ�����޲����Բ��Բ��
	std::vector<T_ROBOT_COORS> CalArcDividePoints(T_ROBOT_COORS tStart, T_ROBOT_COORS tEnd, T_ROBOT_COORS tCenter, double dStep);
	bool CalcArcMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep, int& nStartPointCount, int& nEndPointCount);

	bool CalcLineMoveTrackBySeamData(int nRobotNo, LineOrCircularArcWeldingLine SeamData, std::vector<T_ROBOT_COORS>& vtMeasureCoord, vector<T_ANGLE_PULSE>& vtMeasurePulse, double dStep);
	// �����˲�
	bool SplineFiltering(double dExAxisPos, vector<XI_POINT> vtWeldTrack, vector<T_ROBOT_COORS>& vtWeldTrackRobot);
	// ������ʵ�켣
	bool CalcRealWeldTrack(WeldLineInfo tWeldSeam, vector<T_ROBOT_COORS>& vtWeldTrackRobot);
	// �ж����ڰ�����ʽ
	int DetermineWarpMode(int nGroupNo);

	// ���ذ��ǲ���
	E_WRAPANGLE_PARAM LoadWarpParam(CString strRobotName, E_WRAPANGLE_TYPE eWarpType, int nWarpNo);
	// ���ݽ�β�������һ�ΰ��ǹ켣
	bool CalcOnceWarpTrack(int nWarpNum, double dBoardChick, double dGunToEyeCompen, double dFlatHorComp, std::vector<T_ROBOT_COORS> vtWeldLineTrack, double dWorkpieceHight,
		std::vector<T_ROBOT_COORS>& vtWarpCoors, std::vector<int>& vtWarpPtType, E_WRAPANGLE_PARAM tWarpParam);

	// ������귨��λ�þ�������ԭ������Ƿ������ֵ
	bool CheckFlangeToolCoord(const T_ROBOT_COORS& tRobotCoord, double dDisThreshold = 300.0);
	bool CheckFlangeToolCoord(const T_ANGLE_PULSE& tRobotPulse, double dDisThreshold = 300.0);
	bool CheckFlangeToolCoord(const vector<T_ROBOT_COORS>& vtRobotCoord, double dDisThreshold = 200.0);
	bool CheckFlangeToolCoord(const vector<T_ANGLE_PULSE>& vtRobotPulse, double dDisThreshold = 200.0);

	void WeldInfoToJson(Welding_Info tWeldInfo);

	void DelPngFile(const CString& directory);

	CvPoint3D64f* SaveRemoveCloud(CRobotDriverAdaptor* pRobotDriver, CString cFileName, vector<CvPoint3D64f>& vtPointCloud, int nPtnNum);

	void BackHome();

	//void LoadWeldInfoToJson(Welding_Info& tWeldInfo, int nWeldIndex, std::vector<T_ROBOT_COORS>& vtContour);

};
}
