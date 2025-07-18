#ifndef __ESTUN_ADAPTOR
#define __ESTUN_ADAPTOR
#include ".\Apps\PLib\CtrlUnit\CUnitDriver.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include "EstunRemoteApiLibWeld.h"
#include "ESTUNRobotCtrl.h"
#include <Apps/PLib/LeisaiCtrl/IOControl.h>
#include "XiRobotCtrl.h"
#include ".\OpenClass\FileOP\ini\opini.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include "AbsCoorTransLib.h"
#include ".\Apps\PLib\CtrlUnit\CContralUnit.h"


#define WELD_TRACK_MAX_NUM			498			// �������켣����
#define WRAP_TRACK_MAX_NUM			2			// �������켣����

class EstunAdaptor : public CRobotDriverAdaptor
{
public:
	EstunAdaptor(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver);
	virtual ~EstunAdaptor();

	CString m_sFlatNormalWelderMode_W;		//ƽ��ֱ������ģʽ����W
	CString m_sFlatNormalWelderMode_MM;		//ƽ��ֱ������ģʽ����MM
	CString m_sFlatPulseWelderMode_W;		//ƽ�����庸��ģʽ����W
	CString m_sFlatPulseWelderMode_MM;		//ƽ�����庸��ģʽ����MM
	CString m_sVerNormalWelderMode_W;		//����ֱ������ģʽ����W
	CString m_sVerNormalWelderMode_MM;		//����ֱ������ģʽ����MM
	CString m_sVerPulseWelderMode_W;		//�������庸��ģʽ����W
	CString m_sVerPulseWelderMode_MM;		//�������庸��ģʽ����MM

	bool getWelderMode(E_WELD_SEAM_TYPE eWeldSeamType, CString& sWelderMode_W, CString& sWelderMode_MM);

	int m_nFlatWelderMode;						//����ģʽ0ֱ��1�������˹��ʹ��
	//int g_bWelderType;						//��������0�������1��̩
	int m_nVerWelderMode;               //��������ģʽ,0ֱ����1����

	/****************************************************��������****************************************************/
	//���������Ϣ+
	void cleanAlarm();
	//���õ�ǰģʽ 0-�ֶ�ģʽ��1-�Զ�ģʽ��2-Զ��ģʽ��-1-����
	virtual void SetSysMode(int mode);
	//����ʾ�����ٶ�
	void SetTpSpeed(int speed);//����TPʾ�����ϵ��ٶ�%

	//���س��� ���ñ���ǰҪ�ȼ��س���
	void LoadUserProgramer(const char* projName, const char* progName);
	//���õ�ǰ���� !+
	virtual bool SetRobotToolNo(int nToolNo);

	/****************************************************��Ϣ��ȡ****************************************************/
	//��ѯ�����˵�ǰ����Ƕ�(�����ⲿ��) +
	double GetCurrentDegree(int nAxisNo);
	//��ȡ������Ϣ	+		
	BOOL GetToolData(UINT unToolNo, double adRobotToolData[6]);

	/****************************************************������д****************************************************/
	//,int nToolNo = 1 scoper,0-ϵͳ��1-ȫ�֣�2-���̣�3-����
	virtual void SetPosVar(int nIndex, T_ROBOT_COORS tRobotCoors, int config[7] = int(0), int scoper = 2);
	virtual void SetPosVar(int nIndex, T_ANGLE_PULSE tRobotPulse, int scoper = 2);
	// ��˹�ٻ�����ר��: �ٶȱ�����д
	void SetMultiSpeed(UINT index, UINT count, double** speedVar);//���ô����ٶ�
	void SetSpeed(int nIndex, double adSpeed[5]); // ���õ����ٶ�
	//�ں�����
	void SetWeaveDate(const char* name, ESTUN_WeaveDate WeaveDate, int scope = 2);
	//��ȡһ��ָ��I����
	int GetIntVar(int nIndex, char* cStrPreFix = "INT");
	//����Real����
	bool SetRealVar(int nIndex, double value, char* cStrPreFix = "REAL", int score = 1);//��Ҫ���ڷ��͵�����ѹ scoper,0-ϵͳ��1-ȫ�֣�2-���̣�3-����

	/****************************************************�˶�����****************************************************/

	//�ؽ�����ϵ�����˶�����(�����ⲿ��)++  (�õ�ǰ�û��͹���)	/*1 - 10000 representing 0.01 to 100.0 %),�Ƽ�500*/
	virtual int AxisPulseMove(int nAxisNo, long lDist, long lRobotSpd, WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 1);
	//ֱ������ϵ�����˶�����(�����ⲿ��)
	virtual void PosMove(int nAxisNo, double dDist, long lRobotSpd,  WORD wCoorType = COORD_REL, int nToolNo = 1, long lCoordFrm = 0);
	//ͨ���ƶ�����
	virtual void MoveByJob(double* dRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, int nPVarType, CString JobName, int config[7]) override;
	// ��˹�ٽǶ�����ר�ú�����ֻ�ܹؽڲ岹��
	void MoveByJobForAngle(double* adRobotAngleCoord, T_ROBOT_MOVE_SPEED tPulseMove, CString JobName = "MOVJ");

	//����IO����
	virtual bool SetOutIOVar(int name, int value);
	//bool GetOutIOVar(int name, int& value);
	virtual bool GetInputIOVar(int name, int& value);

	//��˹�ٻ����� ����modeֵ,�������ͺţ�50B��20 ������
	virtual void CalModeValue(T_ANGLE_PULSE tReferPluse, const char* RobotMode, int config[7]);

public:
	/**
     * @description: ʵʱ���ٴ������ݺ��������ڽ�������õ������ݴ��ݸ�������
     * @param {T_ROBOT_COORS&} WeldPathPoints�� ���ӵ�λ����
     * @param {vector<int>&} Cfg����λ���ݶ�Ӧ��config(Ĭ��ȫ0)
     * @return {*} �޸��Ƿ�ɹ�
     */
	virtual bool EstunSendTrackData(const vector<T_ROBOT_COORS>& WeldPathPoint, const vector<vector<int>>& Cfg = vector<vector<int>>(2, vector<int>(7, 0)));
	/**
	 * @description: �����������ӣ�����ɳ�ʼ��(���ͳ�ʼ��λ)
	 * @param {CString&} IP
	 * @param {T_ROBOT_COORS&} startPos�� ������ʼ��λ����
	 * @param {vector<int>&} Cfg����λ���ݶ�Ӧ��config(Ĭ��ȫ0)
	 * @return {*} �޸��Ƿ�ɹ�
	 */
	virtual bool EstunTrackInit(const T_ROBOT_COORS& startPos,const CString& IP = "192.168.60.63",  const vector<int>& Cfg = vector<int>(7, 0));
	/**
	 * @description: ֹͣ���ٹ���(�����껺�������һ����λ��һ��ʱ����û������Ҳ���Զ��رո��ٷ���)
	 * @return {*} {*} �޸��Ƿ�ɹ�
	 */
	virtual bool EstunTrackStop(); //ֹͣ����ʱ����
	virtual bool EstunClearWeldBuffer(); //����buffer
	EstunRemoteApiLibWeld api;//����api����
    
	//����һ��ָ��I����
	virtual void SetIntVar(int nIndex, int nValue, int score = 2, char* cStrPreFix = "INT");
	/************************************************�˶�״̬���************************************************/
	virtual E_ERROR_STATE CheckRobotDone(int nDelayTime = 1200); //�����ȴ��˶�ֹͣ
	virtual bool WorldIsRunning() override; // �ж��ⲿ��ͻ������Ƿ����˶�
	virtual void WorldCheckRobotDone(int nDelayTime = 1200);
	virtual void HoldOn() override;
	//�ָ���ͣ�еĻ����� !+
	virtual void HoldOff();
	// �����ˣ������ϵ�״̬ δ�ϵ�ʱ��ִ���ϵ�
	bool CheckIsReadyRun();
	/**************************************************����д��**************************************************/
	//��˹�ٻ�����һ�����ô������� ֱ�� //�ӿڲ��ȶ����������Ǹ�
	virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, int config[7] = int(0), UINT unToolNum = 1);
	//ͨ��FTP���ʹ������� ������WELD_TRACK_MAX_NUM����
	virtual bool SetMultiPosVar(UINT unIndex, std::vector<T_ROBOT_COORS> vtRobotJointCoord, T_ROBOT_MOVE_SPEED tPosMove, CString Program_name, int config[7] = int(0), UINT unToolNum = 1);
	// ����Ӳ����ʾ���˶����� nDownSpeed�ؽڲ岹�ٶ� nTeachSpeed �� nUpSpeed ֱ�߲岹�ٶ�
	bool SendTeachMoveData(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, const  std::vector<int>& vnMeasureType, int nDownSpeed, int nTeachSpeed, int nUpSpeed, int nTrigSigTime);

	/**************************************************�����ȡ**************************************************/
	virtual double			GetCurrentPos(int nAxisNo) override;		//��ѯ�����˵�ǰֱ������
	virtual T_ROBOT_COORS GetCurrentPos_ESTUN();
	virtual T_ROBOT_COORS	GetCurrentPos() override;				//��ѯ�����˵�ǰֱ������
	virtual long			GetCurrentPulse(int nAxisNo) override;	//��ѯ�����˵�ǰ�ؽ�����
	virtual T_ANGLE_PULSE	GetCurrentPulse() override;				//��ѯ�����˵�ǰ�ؽ�����	
	virtual double			GetPositionDis();				//��ѯ�ⲿ������	
	// ��ȡ�ⲿ������
	virtual double			GetExPositionDis(int nMoveXYZAxisNo);

	/**************************************************IO���ƺ���**************************************************/
	//virtual bool	CheckOutIO(CString sName);				// ��Ŀǰֻ����ɨ���⡢���ټ��⡢��ɨ���������ٴ�����iOȷ����������ȷ�Ϻú����ȥ
	//virtual int		scanLaser(int state);					//��ɨ���� state�� 1 ���� 0 �ء�
	//virtual int		trackLaser(int state);					//���ټ��� state�� 1 ���� 0 �ء�
	/**************************************************��ͨ�ƶ�����**************************************************/
	virtual int CallJob(char JobName[24]); //�����ֲٺг��� ++
	virtual int CallJob(CString sJobName);
	virtual void MoveByJob(T_ROBOT_COORS tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName, int config[7]) override;////Ĭ�� modeֵCfֵΪ��	
	virtual void MoveByJob(T_ANGLE_PULSE tRobotJointCoord, T_ROBOT_MOVE_SPEED tPulseMove, int nExternalAxleType, CString JobName = "MOVJ");
	/**************************************************�����ƶ�����**************************************************/
	//���֧��ʮ���˶�
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);
	virtual T_ROBOT_MOVE_INFO PVarToRobotMoveInfo(int nVarNo, T_ROBOT_COORS tCoord, T_ROBOT_MOVE_SPEED tSpeed, int nMoveType = MOVL, UINT unToolNum = 1, UINT unUserNo = 0, UINT unPosture = 4);//RefPulseΪtCoord�Ĺؽ�λ�ã���˹�ٻ����˱���Ҫ��ֵ
	virtual bool SetMoveValue(std::vector<T_ROBOT_MOVE_INFO> vtMoveInfo, bool bSafeMove = true, bool bUsePB = false);
	/**************************************************�����ƶ�����**************************************************/
	//�ⲿ���ƶ�����
	virtual int MoveExAxisForLineScan(double dDist, int nSpeed);
	// ��������������һ���߶�(�밲ȫλ����������������й�)
	virtual bool MoveToSafeHeight();
	// Ӳ����ʾ���˶� �˶��ⲿ�� �� ������ (��һ������ⲿ��ͬʱ�˶� �� �ȶ��ⲿ���ٶ�������)
	virtual bool TeachMove(const std::vector<T_ANGLE_PULSE>& vtMeasurePulse, double dExAxlePos, const  std::vector<int>& vnMeasureType, int nTrigSigTime);
	// ��ǹ��˿
	//virtual bool CleanGunH();
	// �˶�����ɨ���λ��
	virtual int MoveToLineScanPos(T_ANGLE_PULSE tPulse, T_ROBOT_MOVE_SPEED tPulseMove);

	// 10���������� �������͡��岹��ʽ���ٶ� �������˶�
	virtual int ContiMoveAny(const std::vector<T_ROBOT_MOVE_INFO>& vtRobotMoveInfo) override;

	// �ްں� �� �����˵�һ�� �����˶�
	int WeldMoveRobotWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	bool SendWeldProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);
	// �Զ����������ǰ� �����˶�
	int WeldMoveTriangleWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);
	// ����ƽ������ ���ǰ�L�� �����켣
	bool CalcWeaveWeldTrack(std::vector<T_ROBOT_COORS>& vtWeldTrack, const T_WELD_PARA& tWeldPara, E_WELD_SEAM_TYPE eWeldSeamType, double& dRealWeldSpeed);

	// �Զ�������L�ں����˶�
	int WeldMoveLWeave(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);

	// �����˶� ()
	virtual int WeldMove(const std::vector<T_ROBOT_COORS>& vtWeldPathPoints, const  std::vector<int>& vnPtnType, const T_WELD_PARA& tWeldPara, double dExAxlePos, E_WELD_SEAM_TYPE eWeldSeamType, bool bIsArcOn);

	bool SendWeldTriangleWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);

	bool SendWeldLWeaveProgram(int nWeldTrackNum, E_WELD_SEAM_TYPE eWeldSeamType);

	// �������꣺�ⲿ���˶�����,�����������ж��ƶ���
	virtual int MoveExAxisFun(double dDist, int nSpeed, int nMoveXYZAxisNo, double dMaxExSpeed = -1); // ����ⲿ���ٶȷ���չ�ⲿ���ٶ�mm/min С��0ʹ��nSpeed
	virtual int XYZAxisNoToSoftAxisNo(int nMoveXYZAxisNo);

private:
	// ������̬��dPosture1->dPosture2 �仯����ΪdRatio����ֵ̬
	double CalcPosture(double dPosture1, double dPosture2, double dRatio);

	ESTUNRobotCtrl* m_pEstunRobot;
	CString m_sRobotMode;
};

#endif // !__ESTUN_ADAPTOR
