// RobotDriverAdaptor.h: interface for the CRobotDriverAdaptor class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_)
#define AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include "XiRobotCtrl.h"
#include ".\Apps\PLib\CtrlUnit\CUnitDriver.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include "AbsCoorTransLib.h"

#define MAXMOVENUM 10
#define MAXPOSVARNO 23
#define MAXPOSVARNO_H 25


#include "Apps/PLib/CrpRobot/ROBOTPCtr.h"


//��̬����
static bool m_bInsterPVal = false;



typedef enum
{
	ROBOT_STATE_IDLE = 0,		//����
	ROBOT_STATE_RUNNING = 1,	//������
	ROBOT_STATE_ERROR = 2,		//�쳣
}E_ROBOT_STATE;

typedef enum 
{
	ROBOT_BRAND_YASKAWA = 0x01,			//����
	ROBOT_BRAND_ESTUN		= 0x02,		//��˹��
	ROBOT_BRAND_CRP			= 0x03			//��ŵ��
}E_ROBOT_BRAND;

struct T_ROBOT_MOVE_INFO
{
	MP_USR_VAR_INFO tCoord;	//����
	MP_USR_VAR_INFO tPulse;	//����
	int nMoveType = MOVJ;	//�ƶ���ʽ
	T_ROBOT_MOVE_SPEED tSpeed;	//�ƶ��ٶ�
	int nMoveDevice = -1;
	int nTrackNo = -1;
	double adBasePosVar[3]; //�ⲿ������(MP_USR_VAR_INFO�����֧��8��)
};

typedef struct
{
	T_ROBOT_COORS tGunTool;
	T_ROBOT_COORS tMagnetTool;
	T_ROBOT_COORS tPolisherTool;
	T_ROBOT_COORS tCameraTool;
}T_ROBOT_TOOLS;

class CRobotMove
{
public:
	void Clear();
	bool InsertCoor(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	bool InsertCoor(T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nExternalAxleType, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	bool DeleteTrack(int nNum);//ɾ���켣ǰ nNum ����
	std::vector< T_ROBOT_MOVE_INFO> GetTrack();
	int GetTrackNum();
private:
	std::vector< T_ROBOT_MOVE_INFO> m_vRobotMoveInfo;
	int m_nTrackNum = 0;
};

class CRobotDriverAdaptor  
{
public:
	CRobotDriverAdaptor(CString strUnitName, CLog *cLog, std::vector<CUnitDriver*>* ppUnitDriver = NULL);
	virtual ~CRobotDriverAdaptor();
	BOOL InitRobotDriver(CString strUnitName, CLog *cLog);

	void SetKinematicsParam(T_KINEMATICS tKinematics, T_AXISUNIT tAxisUnit, T_AXISLIMITANGLE tAxisLimitAngle);
	//�����⣺�ú�������ȷִ������·�� .\Data\SpeedAndSafeHeight.ini  �¿���޴�·��  �ú�������ɶ��壩
	void LoadSpeedAndSafeHeight(); // �����ٶȲ���

	//�����⣺δ���CheckInIO()���壺�ú�������CheckInIO()  �ú�������ɶ��壺ע�������
	bool ContiMoveByJob(std::vector<T_ANGLE_PULSE> vtPulseMove, T_ROBOT_MOVE_SPEED tPulseMove, bool bPulseOrder, CString strJobName = "CONTIMOVE");
	void SetIntValFun(int nIntVal, int nVal);
	

	/****************************************************��������****************************************************/
	//������̬
	void ConfigRobotPosture(unsigned int nRobotPosture);
	//�ŷ��ϵ�
	void ServoOn();	
	//���������Ϣ+
	void CleanAlarm();
	//�ŷ�����
	void ServoOff();
	//�����˼�ͣ
	//void HoldOn(); 	// ��Ϊ�麯���ӿ�
	//�ָ���ͣ�еĻ�����	
	void HoldOff();	
	//�����ֲٺг���
	//void CallJob(char JobName[24]);	// ��Ϊ�麯���ӿ�
	//void CallJob(CString sJobName);	// ��Ϊ�麯���ӿ�
	//�����⣺�Ͽ���޴����أ�
	void CallJob(CString sJobName, int nExternalType);
	//���õ�ǰ����
	void SetRobotToolNo(int nToolNo);
	//2023/12/21 ���Ӽӣ�ע��
	CXiRobotCtrl *GetXiRobotCtrl();

	/****************************************************��Ϣ��ȡ****************************************************/
	//��ѯ�����˵�ǰֱ������
	//T_ROBOT_COORS GetCurrentPos();	// ��Ϊ�麯���ӿ�
	//double GetCurrentPos(int nAxisNo);  	// ��Ϊ�麯���ӿ�
	//��ѯ�����˵�ǰ�ؽ�����
	//T_ANGLE_PULSE GetCurrentPulse();	// ��Ϊ�麯���ӿ�
	//long GetCurrentPulse(int nAxisNo);	// ��Ϊ�麯���ӿ�
	//��ѯ�����˵�ǰ����Ƕ�(�����ⲿ��)
	double GetCurrentDegree(int nAxisNo);
	//�ؽ�����תֱ������
	int ConvAxesToCartPos(long alPulse[6], UINT * unFig_ctrl, double adCoord[6], int nGrpNo = 0, UINT unTool_no = 1);
	//ֱ������ת�ؽ�����														
	int ConvCartToAxesPos(double adCoord[6], long alPulse[6], UINT unFig_ctrl = 4, int nGrpNo = 0, UINT unTool_no = 1, UINT unKinema_type = 0);
	//��ȡ������Ϣ															
	BOOL GetToolData(UINT unToolNo, double adRobotToolData[6], UINT32 unTimeout = 2000);
	//��ȡ������							
	BOOL GetAlarmCode(int &nErrorNo, int &ErrorData, int &AlarmNum, MP_ALARM_DATA *pData, UINT32 unTimeout = 2000);	
	//��ȡ������Ϣ
	CString GetWarningMessage();

	/***********************************************Ŀ��λ�ú˶�***********************************************/
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SetAimPulse(T_ANGLE_PULSE tAimPulse);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SetAimCoord(T_ROBOT_COORS tAimCoord);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool CheckIsAimPulse(T_ANGLE_PULSE tCompareLimit);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool CheckIsAimCoord(T_ROBOT_COORS tCompareLimit);

	bool CompareCurCoord(T_ROBOT_COORS tCoord, T_ROBOT_COORS tCompareLimit);
	bool CompareCurPulse(T_ANGLE_PULSE tPulse, T_ANGLE_PULSE tCompareLimit);
	bool CompareCoord(T_ROBOT_COORS tCoord1, T_ROBOT_COORS tCoord2, T_ROBOT_COORS tCompareLimit);
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, T_ANGLE_PULSE tCompareLimit);
	bool CompareCoords(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit = 5.0, int nCheckType = 3, double dAngleLimit = 4.0);									//����ȶԺ���nCheckType:1���XYZ��2���RXRYRZ��3�����
	bool CompareCoords(T_ROBOT_COORS tCoords1, double dCoordsLimit = 5.0, int nCheckType = 3, double dAngleLimit = 4.0);									//����ȶԺ���nCheckType:1���XYZ��2���RXRYRZ��3�����
	bool CompareXY(T_ROBOT_COORS tCoords1, T_ROBOT_COORS tCoords2, double dCoordsLimit = 20.0);													//����ȶԺ���
	bool ComparePulse(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2, long lLimit = 500);
	bool ComparePulse(T_ANGLE_PULSE tPulse1, long lLimit = 500);

	/****************************************************������д****************************************************/
	//fast write
	BOOL SetMultiVar_H(UINT unCount, MP_USR_VAR_INFO* pVarInfo);
	//��2023/12/21������ռ��
	void SetMultiVar_H(const std::vector<MP_USR_VAR_INFO> vtVarInfo);
	//��2023/12/21������ռ��
	void SetMultiVar_H_WriteSyncIo(int nSyncId, int nSyncIoNo, UINT unCount, MP_USR_VAR_INFO* pVarInfo);

	//һ�������ô���BP������2023/12/21������ռ��
	void SetMultiBasePosVar(UINT unCount, UINT unIndex, long lRobotPulse[][3], int nExternalAxleType);
	//��2023/12/21������ռ��
	void SetMultiBasePosVar(UINT unCount, UINT unIndex, double dPosCoord[][3], int nExternalAxleType, int nPVarType = MP_BASE_COORD);
	//�����⣺�ú�������ɶ��� �Ͽ���޴�����	����2023/12/21������ռ��
	void SetMultiBasePosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, int nExternalAxleType);
	//һ�������ô���P���� ��2023/12/21������ռ��
	void SetMultiPosVar(UINT unIndex, std::vector<T_ANGLE_PULSE> vtRobotPulse, UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//һ�������ô���P���� ��2023/12/21������ռ��
	void SetMultiPosVar(UINT unCount, UINT unIndex, long lRobotPulse[][6], UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//һ�������ô���P���� ��2023/12/21������ռ��
	void SetMultiPosVar(UINT unCount, UINT unIndex, double adPosCoord[][6], UINT unToolNum = 1, UINT unUserNo = 1, UINT unPosture = 4);
	//һ�������ô���P���� ��2023/12/21������ռ��
	void SetMultiPosVar(UINT unCount, T_ROBOT_POSVAR_DATA* pPosVarData);
	//��˹�ٻ�����һ�����ô������� ֱ�� //�ӿڲ��ȶ����������Ǹ� �����⣺�¿���޴����� �Ͽ��δ��ȷ���壩
	bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//�����⣺�ú�������ɶ��� �¿���޴����� �Ͽ��δ��ȷ���壩
	bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);

	//һ���Ի�ȡ����P����
	void GetMultiPosVar(UINT unCount, UINT unIndex, double adRobotPos[][6], UINT *pToolNo = NULL, UINT *pUserNo = NULL, UINT *pPosture = NULL, UINT32 unTimeout = 2000);
	void GetMultiPosVar(UINT unCount, MP_VAR_INFO *mpVarInfo, LONG* pPosVarData, UINT32 unTimeout = 2000);
	//���õ���P���� ++-
	void SetPosVar(int nIndex, double adPosVar[6], int nPVarType);
	void SetPosVar(int nIndex, long alPosVar[6], int nPVarType);
	void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors);
	void SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse);
	//�����⣺�ú�������ɶ��� �Ͽ���޴�����	����2023/12/21������ռ��
	void SetPosVar(int nIndex, long lPosVar[6]);
	
	//��ȡһ��ָ��I����
	int GetIntVar(int nIndex);
	//����һ��ָ��I���� 
	void SetIntVar(int nIndex, int nValue);
	//����I���������ٽӿڣ�
	void SetIntVar_H(int nIndex, int nValue);

	//���ô���������I,B,D,R��
	void SetMultiVar(UINT unCount, unsigned unIndex, LONG lRobotValue[], UINT unType = MP_RESTYPE_VAR_I);
	void SetMultiVar(UINT unCount, MP_VAR_DATA *pVarData);
	
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	������BP������2023/12/21������ռ��
	void SetBasePosVar(int nIndex, double adBasePosVar[3], int nCoordType);
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	����ȡBP������2023/12/21������ռ��
	int GetBasePosVar(long lPvarIndex, double *array, UINT32 unTimeout = 2000);
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	����ȡP������2023/12/21������ռ��
	int GetPosVar(long lPvarIndex, double array[6]);

	/****************************************************�˶�����****************************************************/
	//ֱ������ϵ�����˶�����(�����ⲿ��)
	virtual void PosMove(int nAxisNo, double dDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	virtual void PosMove(double adRobotCoord[10], long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	//�ؽ�����ϵ�����˶�����(�����ⲿ��)
	int AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1);
	void MoveToAbsPluse(long alAbsPulse[6], long lSpeedRatio = 500);
	//ͨ���ƶ�����
	void MoveByJob(double *dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType = PULSE_COORD, CString JobName = "MOVJ");
	//ֱ�������˶�����
	void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	//�ؽ������˶�����
	virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	void MoveByJob(long *alRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	// ������˶� (����ǹ��������˶�) ��2023/12/21��ŵ�ն�ռ��
	bool DropGun(std::vector<double*> pos, std::vector<T_ROBOT_MOVE_SPEED> myspeed, std::vector<int> movType);
	virtual void MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7]);
	virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7]);////Ĭ�� modeֵCfֵΪ��	

	void ThroughTranPointInfo(T_ANGLE_PULSE& tStartPulse, T_ANGLE_PULSE& tEndPulse, vector<T_ROBOT_MOVE_INFO>& tAllPulse);
	bool TransPulse(T_ROBOT_MOVE_INFO& tPoint, T_ANGLE_PULSE &tPulse);
	bool TheAnswer(vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);

	/**************************************************�����ƶ�����**************************************************/
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//��2023/12/21������ռ��
	// �����췢���ݣ�bSafeMove��Ĭ��ʹ�ð�ȫ���ȵ�
	virtual bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo,bool bSafeMove = true, bool bUsePB = false);
	virtual bool SetMoveValue(CRobotMove cMoveInfo);
	//��2023/12/21������ռ��
	int GetMoveStep();
	//��2023/12/21������ռ��
	int CleanMoveStep();
	// �����ⲿ�������ȡд���ʽת��
	void RobotTransCoordToBase(double *adCoord, double *adBasePosVar);
	void RobotTransBaseToCoord(double *adBasePosVar, double *adCoord);

	/**************************************************Ӳ����ר�ú���**************************************************/
	//��������ռ��
	bool ReadSyncCoord(T_ANGLE_PULSE& tPulse, int nSyncId);

	/***********************************************״̬��Ϣ(���ɸ���)***********************************************/
	int m_nExternalAxleType;							//�ⲿ������
	T_EXTERNAL_AXLE_INFO m_tExternalAxle[3];			//�ⲿ����Ϣ

	int m_nRobotType;									//�ؽڱ����ͣ����������໮�֣�
	E_ROBOT_BRAND m_eRobotBrand;						//������Ʒ��

	E_MANIPULATOR_TYPE m_eManipulatorType;				//�ؽڱ����ͣ��������ͺŻ��֣�
	int m_nRobotNo;										//�ؽڱ۱��
	CString m_strRobotName;								//�ؽڱ����ƣ�������ȡ�������ڲ��ã�
	CString m_strCustomName;							//�ؽڱ����ƣ���ʾ�ã�
	T_ROBOT_TOOLS m_tTools;								//�ؽڱ����ù��߼���
	T_ROBOT_COORS m_tFirstTool;							//�ؽڱ�һ�Ź���
	T_ANGLE_PULSE m_tHomePulse;							//�ؽڱ۷�����״̬ʱ�İ�ȫλ��

	T_KINEMATICS m_tKinematics;
	T_AXISUNIT m_tAxisUnit;
	T_AXISLIMITANGLE m_tAxisLimitAngle;
	T_ROBOT_LIMITATION m_tRobotLimitation;				//���������Ʒ�Χ��24/01/23�ϸ�������ӣ�

	std::vector<T_ANGLE_PULSE> m_vtSafePulse;			//���ӱ��Ϲ��ɵ�

	double m_dWeldNorAngleInHome;
	vector<CUnitDriver*> *m_pvpMotorDriver;
	CXiRobotCtrl* m_pYaskawaRobotCtrl;					//ʵ����һ������������
	CROBOTPCtr* m_pCrpRobotCtrl;						//ʵ����һ����ŵ�ջ�����

	T_ROBOT_COORS m_tRobotHomeCoors;
	GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans* m_cXiRobotAbsCoorsTrans;

	// �������ٶȲ���
	T_ROBOT_MOVE_SPEED		m_tBackHomeSpeed;
	T_ROBOT_MOVE_SPEED		m_tTeachSpeed;
	T_ROBOT_MOVE_SPEED		m_tExAxlePulseSpeed;
	T_ROBOT_MOVE_SPEED		m_tPulseHighSpeed;
	T_ROBOT_MOVE_SPEED		m_tPulseLowSpeed;
	T_ROBOT_MOVE_SPEED		m_tCoordHighSpeed;
	T_ROBOT_MOVE_SPEED		m_tCoordLowSpeed;

	//�����⣺���Ͽ����ֲ�ı��� ���³�Ա������	��ʹ�ã�
	int	  m_nRobotInstallDir;
	bool		m_bIsOpenTrack;
	std::vector<T_ROBOT_COORS> m_vtWeldLineInWorldPoints;	//��������

	//��2023/12/21������ռ��
	bool GetManipulatorType(CString strManipulatorType, E_MANIPULATOR_TYPE &eManipulatorType);
	//��2023/12/21������ռ��
	E_ROBOT_MODEL GetManipulatorType(E_MANIPULATOR_TYPE eManipulatorType);
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	��
	bool IsCuttingRobot();
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	��
	bool IsHandlingRobot();
	//�����⣺�ú�������ɶ��� �Ͽ���޴˶���	��
	bool IsPolishingRobot();

	/***********************************************״̬��Ϣ(�ɸ���)***********************************************/
	E_INCISEHEAD_THREAD_STATUS m_eThreadStatus;				//�������¹ؽڱ��������̵߳ĵ�ǰ״̬
	E_INCISEHEAD_THREAD_STATUS m_eLastThreadStatus;			//�������¹ؽڱ��������̵߳�֮ǰ״̬�����ڼ�����
	E_ROBOT_STATE m_eRobotState;							//�ؽڱ۵�ǰ�˶�״̬���������������̣߳�

	CLog *m_cLog;

	/***********************************************��ͣ������д***********************************************/
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SavePausePara(CString sKey, int nValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SavePausePara(CString sKey, bool bValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SavePausePara(CString sKey, double dValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool SavePausePara(CString sKey, CString sValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool LoadPausePara(CString sKey, int &nValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool LoadPausePara(CString sKey, bool &bValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool LoadPausePara(CString sKey, double &dValue);
	//�����⣺ʹ�ú�ROBOT_PAUSE_INI��Ŀ¼��./Data/_RobotName_/RobotPause.ini���ú�������ɶ��� �¿���޴�Ŀ¼�ṹ �Ͽ���޴˶��壩
	bool LoadPausePara(CString sKey, CString &sValue);

	/***********************************************�㷨����***********************************************/
	//��2023/12/21������ռ��
	bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ROBOT_COORS tToolCoors, std::vector<T_ANGLE_PULSE>& vtResultPulse);
	bool RobotInverseKinematics(T_ROBOT_COORS tRobotCoors, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tToolCoors, T_ANGLE_PULSE &tResultPulse, int nExternalAxleType = -1);
	void RobotKinematics(T_ANGLE_PULSE tCurrentPulse, T_ROBOT_COORS tToolCoors, T_ROBOT_COORS &tCurrentRobotCoors, int nExternalAxleType = -1);
	//��2023/12/21������ռ��
	bool MoveToolByWeldGun(T_ROBOT_COORS tWeldGunCoors, T_ROBOT_COORS tWeldGunTools, T_ROBOT_COORS tMagneticCoors, T_ROBOT_COORS tMagneticTools, T_ROBOT_COORS& tGunCoorsMoveMagnetic);
	T_ANGLE_PULSE ThetaToPulse(T_ANGLE_THETA tTheta);
	T_ANGLE_THETA PulseToTheta(T_ANGLE_PULSE tPulse);

	// ����� -> Rz
	double DirAngleToRz(double dDirAngle);
	double DirAngleToRz(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dDirAngle);
	// Rz -> �����
	double RzToDirAngle(double dRz);
	double RzToDirAngle(double dBaseRz, double dBaseDirAngle, double dChangeDir, double dRz);

	/******************************************��ŵ�ջ����˶�ռ*****************************************/
	//Var
	std::map<std::string, int> XiJobCase;

	//Func
	/**
	* @description: ���Ӻ��������������֣����ɺ����ļ������ͺ����ļ��������ˡ�ִ�к����ļ�
	* @param {vector<T_ROBOT_COORS>} vtMeasureCOORS �����ӵ�λ
	* @param {vector<int>} vnPtnType�� ���ӵ����ͣ�MOVJ,MOVL��
	* @param {T_WELD_PARA} Para�� ���Ӳ���
	* @param {T_ROBOT_MOVE_SPEED} tPulseMove�� �����ٶȡ����ٶȡ����ٶ�
	* @param {int} weldType�� ����ͬ�ĺ�������(������ƽ��)��ͬʱҲ����ʹ�õڼ������ӹ���(�磺����ʹ�ù���0�������������0��ƽ��ʹ�ù���1)
	* @return {int}�� ����״̬��
	*/
	//��2023/12/21��ŵ�ն�ռ��
	int Welding(std::vector<T_ROBOT_COORS> vtWeldCOORS, double dRealWeldExPos, std::vector<int> vnPtnType, T_WELD_PARA Para, int weldType, bool arcOn = true);

	//���ݹ켣�����й��ȵ��MOVL ����ɨ����2023/12/21��ŵ�ն�ռ��
	int SendTeachMoveData_L(std::vector<T_ROBOT_COORS> vtMeasureCOORS, std::vector<bool> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//��2023/12/21��ŵ�ն�ռ��
	int SendTeachMoveData_J(std::vector<T_ANGLE_PULSE> vtMeasurePulse, double ExternalAxle, const std::vector<int> veMeasureType, int nDowdSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//��ȡM�ڵ�״̬��2023/12/21��ŵ�ն�ռ��
	int Read_M(int M_Num);
	//����M�ڵ�״̬��2023/12/21��ŵ�ն�ռ��
	int Write_M(int M_num, int Status);
	//��ȡM�ڵ�״̬��2023/12/21��ŵ�ն�ռ��
	int Read_Y(int Y_num);
	//����Y�ڵ�״̬��2023/12/21��ŵ�ն�ռ��
	int Write_Y(int Y_num, int Status);
	//������д�̵�����ָ�� ��2023/12/21��ŵ�ն�ռ��
	//(ע�⣺ ʹ���ӳ�����Ҫ���ӳ���ĵ�һ�н�M500���ó�1,�ڵ����ڶ�����Ҫ��M500����Ϊ0)
	//��2023/12/21��ŵ�ն�ռ��
	void MakeDout(int M_num, bool state, int lineNum, std::string* Dout_str);
	//��2023/12/21��ŵ�ն�ռ��
	void MakeSleep(int SleepTime, int LineNumber, string& Sleep_Str);

	// ���������Ƿ����˶��У�2023/12/21��ŵ�ն�ռ��
	E_ERROR_STATE CheckRobotDone_CRP(int nDelayTime);

	/********************************************�췢����׼��**********************************************/
	//��2023/12/21������ռ��
	MP_USR_VAR_INFO PrepareValData(int nIntValIdx, int nValue);
	//��2024/01/24������ռ��
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, long lBPValue[3], UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//�����
	MP_USR_VAR_INFO PrepareValDataEx(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//
	MP_USR_VAR_INFO PrepareValDataEx(int nPosVarIdx, T_ROBOT_COORS tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);

	//��2023/12/21������ռ��
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, T_ANGLE_PULSE tPulse, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	//��2023/12/21������ռ��
	MP_USR_VAR_INFO PrepareValData(int nPosVarIdx, T_ROBOT_COORS tCoord, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);

	/****************************************************����****************************************************/
	T_ROBOT_COORS m_tRobotRunningRangeMin;//�����˼�������
	T_ROBOT_COORS m_tRobotRunningRangeMax;//�����˼�������
	
	// 10���������� �������͡��岹��ʽ���ٶ� �������˶� (�˶��ڶ�����ǰ���뱣֤�ⲿ���˶���dExAxlePosλ��)
	//int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);	// ��Ϊ�麯���ӿ�
	//�����⣺��ʹ��Ŀ¼ ./Data/RobotAndCar.ini �¿���޴�Ŀ¼�ṹ ��δ���SwitchIO()���壺�ú�������SwitchIO()  �ú�������ɶ��壺ע�������
	//bool CleanGunH(int RobotNum); //1�� 0��
	/***********************************************�ϺϺϺ�***********************************************/
	// ��ȡ�ⲿ�᷽��
	int GetPanasonicExDir(E_EXTERNAL_AXLE_TYPE eExAxisType);
	private:
		T_ANGLE_PULSE m_tAimPulse;
		T_ROBOT_COORS m_tAimCoord;

		void Open(int nSocketPort = 20001);
		void Close();
		//void AppendMotorAxisPos(T_ROBOT_COORS& tCoord);
		//void AppendMotorAxisPos(T_ANGLE_PULSE& tPulse);
		int GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, double &dExPos);
		int GetPanasonicExPos(E_EXTERNAL_AXLE_TYPE eExAxisType, long &lExPos);
	
public:
	// ������������չ��Ҫ����ʵ�ֵ������麯��
	// �����ֲٺг��� ++
	virtual int CallJob(char JobName[24]);
	virtual int CallJob(CString sJobName);
	virtual void WorldCheckRobotDone(int nDelayTime = 1200);
	virtual bool CheckIsReadyRun();	// �����ϵ�״̬ δ�ϵ�ʱ��ִ���ϵ�
	//virtual E_ERROR_STATE CheckRobotDone(int nDelayTime = 1200);		//�����ȴ��˶�ֹͣ
	//virtual DWORD SwitchIO(CString sName, bool bOpen);
	//virtual bool CheckInIO(CString sName); // ��Ĭ��ֵ��ͬ true ��Ĭ��ֵ�෴ false
	//virtual bool CheckOutIO(CString sName); // ��Ĭ��ֵ��ͬ true ��Ĭ��ֵ�෴ false
	virtual double GetCurrentPos(int nAxisNo); //��ѯ�����˵�ǰֱ������
	virtual bool GetInputIOVar(int name, int value);
	virtual bool SetOutIOVar(int name, int value);
	virtual T_ROBOT_COORS GetCurrentPos_ESTUN();
	virtual T_ROBOT_COORS GetCurrentPos(); //��ѯ�����˵�ǰֱ������
	virtual long GetCurrentPulse(int nAxisNo); //��ѯ�����˵�ǰ�ؽ�����
	virtual T_ANGLE_PULSE GetCurrentPulse(); //��ѯ�����˵�ǰ�ؽ�����
	virtual double GetPositionDis(); //��ѯ�ⲿ������ (��ŵ�ջ򰲴������ⲿ��)
	//virtual int scanLaser(int state); //��ɨ���� state�� 1 ���� 0 �ء�
	//virtual int trackLaser(int state); //���ټ��� state�� 1 ���� 0 �ء�
	//virtual bool MoveToSafeHeight(); // ��������������һ���߶�(�밲ȫλ����������������й�)
	//��˹�ٻ�����һ�����ô������� ֱ�� //�ӿڲ��ȶ����������Ǹ�
	//virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_PULSE_MOVE tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//ͨ��FTP���ʹ������� ������300����
	//virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_PULSE_MOVE tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);
	// ����Ӳ����ʾ���˶����� nDownSpeed�ؽڲ岹�ٶ� nTeachSpeed �� nUpSpeed ֱ�߲岹�ٶ�
	//virtual bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);
	//virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_PULSE_MOVE tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	//ֱ�������˶����� + 
	//virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_PULSE_MOVE tPulseMove, int nExternalAxleType, CString JobName = "MOVJ", int config[7] = int(0));
	//�ⲿ���˶�����
	virtual int MoveExAxisForLineScan(double dDist, int nSpeed);
	// �˶�����ɨ���λ��
	virtual int MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove);

	virtual bool WorldIsRunning(); // ��˹��ר��:�ж��ⲿ��ͻ������Ƿ����˶�

	// Ӳ����ʾ���˶� �˶��ⲿ�� �� ������ (��һ������ⲿ��ͬʱ�˶� �� �ȶ��ⲿ���ٶ�������)
	virtual bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);

	// 10���������� �������͡��岹��ʽ���ٶ� �������˶� (�˶��ڶ�����ǰ���뱣֤�ⲿ���˶���dExAxlePosλ��)
	virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo);
	// �����˶� ()
	virtual int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	//virtual bool CleanGunH();
	// ��ͣ�˶�
	virtual	void HoldOn();
	// �˶���ָ���Ĺؽ�����
	//virtual bool MoveByJobJ(double Distence[8], int config[7], double speed, int ifAbsolutM, int ifJoint);

	virtual int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // ����ⲿ���ٶȷ���չ�ⲿ���ٶ�mm/min С��0ʹ��nSpeed
	virtual int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);

	// ��ȡ�ⲿ������
	virtual double			GetExPositionDis(int nMoveXYZAxisNo);


	//��˹�ٸ��ٽӿ�
	//��˹�ٻ����� ����modeֵ,�������ͺţ�50B��20 ������
	virtual void CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7]);
	//��˹�ٻ����� �����������ӣ�����ɳ�ʼ��(���ͳ�ʼ��λ)
	virtual bool EstunTrackInit(const T_ROBOT_COORS& startPos, const CString& IP = "192.168.60.63", const vector<int>& Cfg = vector<int>(7, 0));
	//��˹�ٻ����� ����buffer
	virtual bool EstunClearWeldBuffer(); 
	//��˹�ٻ����� ʵʱ���ٴ������ݺ��������ڽ�������õ������ݴ��ݸ�������
	virtual bool EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg = vector<vector<int>>(2, vector<int>(7, 0)));
	//��˹�ٻ����� ֹͣ����ʱ����
	virtual bool EstunTrackStop();
	CMutex        m_mutex;  //ʵʱ���ٵ���
	bool TrackingState; //�����������״̬

	// �����¿ں������
	T_ROBOT_COORS m_tFlatGrooveScanPosture; // = T_ROBOT_COORS(0.0, 500, 1600.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tFlatGrooveWeldPosture; // = T_ROBOT_COORS(0.0, 500, 1600.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tFlatGrooveScanPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 0.0, 0.0, 66.0, 0.0, 0.0, 0.0);
	//T_ROBOT_COORS m_tFlatGrooveWeldPosture = T_ROBOT_COORS(0.0, 1100.0, 1800.0, 4.0, -10.0, 66.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandDownGrooveScanPosture; // = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandDownGrooveWeldPosture; // = T_ROBOT_COORS(0.0, -1200.0, 1400.0, 52.0, -60.0, 35.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandUpGrooveScanPosture; // = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
	T_ROBOT_COORS m_tStandUpGrooveWeldPosture; // = T_ROBOT_COORS(0.0, -1500.0, 800.0, -60.0, 62.0, -154.0, 0.0, 0.0, 0.0);
	int m_nScanNum;//ɨ�����
};


#endif // !defined(AFX_ROBOTDRIVERADAPTOR_H__59FD46BF_CA09_4E0E_A5E0_C6B55F565624__INCLUDED_)
