#pragma once

#include "Apps\PLib\BasicFunc\BaseStruct.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\PLib\Estun\EstunAdaptor.h"
#include "CUnitDriver.h"
#include ".\Apps\PLib\DahengCam\DHGigeImageCapture.h"
#include ".\Apps\PLib\KinectCam\KinectControl.h"
#include ".\Apps\PLib\LeisaiCtrl\CBasicIOControl.h"
#include ".\Apps\PLib\BasicFunc\ChoiceResources.h"
#include ".\Apps\PLib\MechMidCam\CMechEyeCtrl.h"

#ifndef ON
#define ON	0
#define OFF 1
#endif

#ifndef MAX_IO_NUM
#define MAX_IO_NUM 9999
#endif

#ifndef DELETE_UNIT
#define DELETE_UNIT(obj) if(obj != NULL)\
{\
delete obj;\
}
#endif

#ifndef MESSAGE_BOX_UNIT
#define MESSAGE_BOX_UNIT(data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
XUI::MesBox::PopError((const char*)__str);\
}
#endif

#ifndef WRITE_LOG_UNIT
#define WRITE_LOG_UNIT(data) {CString __str;\
__str.Format("Function:[%s] Line:[%d]	\ndata: %s",__FUNCTION__, __LINE__, data);\
GetLog()->Write(__str);}
#endif

#ifndef CHECK_BOOL_RTN_UNIT
#define CHECK_BOOL_RTN_UNIT(data, str) if(data != TRUE)\
{\
WRITE_LOG_UNIT(str);\
return FALSE;\
}
#endif

#ifndef CHECK_BOOL_BOX_RTN_UNIT
#define CHECK_BOOL_BOX_RTN_UNIT(data, str) if(data != TRUE)\
{\
MESSAGE_BOX_UNIT(str);\
return FALSE;\
}
#endif

#ifndef CHECK_INT_RTN_UNIT
#define CHECK_INT_RTN_UNIT(data, str) {int temp = data;\
if(temp != 0)\
{\
WRITE_LOG_UNIT(str);\
return temp;\
}\
}
#endif

#ifndef CHECK_INT_BOX_RTN_UNIT
#define CHECK_INT_BOX_RTN_UNIT(data, str) {int temp = data;\
if(temp != 0)\
{\
MESSAGE_BOX_UNIT(str);\
return temp;\
}\
}
#endif

#ifndef CHECK_IO_RTN_UNIT
#define CHECK_IO_RTN_UNIT(data) if(data < 0 || data > MAX_IO_NUM)\
{\
MESSAGE_BOX_UNIT(GetStr("IO��Ч,��ţ�%d", data));\
return -1;\
}
#endif

#ifndef CHECK_CTRL_CARD_RTN_UNIT
#define CHECK_CTRL_CARD_RTN_UNIT if(m_pBasicIOControl == NULL)\
{\
MESSAGE_BOX_UNIT("���ƿ�������Ч");\
return -1;\
}
#endif

typedef enum
{
	E_UNIT_TYPE_HANDLING = 0,	//����
	E_UNIT_TYPE_SORTING,		//�ּ�
	E_UNIT_TYPE_CUTTING,		//�и�
	E_UNIT_TYPE_WELD,			//����
	E_UNIT_TYPE_POLISH,			//��ĥ
	E_UNIT_TYPE_FLIP,			//����
	E_UNIT_TYPE_MAX_NUM,
}E_UNIT_TYPE;

typedef struct
{
	int nUnitNo = -1;
	CString strUnitName;
	CString strChineseName;
	CString strUnitType;
	int nContralUnitType = 0;
}T_CONTRAL_UNIT;

class CContralUnit
{
public:
	CContralUnit(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver *pServoMotorDriver = NULL);
	~CContralUnit();

	//**********************************************//
	//*											   *//
	//******************* ��ʼ�� *******************//
	//*											   *//
	//**********************************************//
	/*
	* InitContralResource
	* ��ʼ��������Դ��������+���ƿ��ŷ����+�����
	* param		
	* return	0:�ɹ� ����:�������
	*/
	int InitContralResource();


	// ���ݻ����˺Ŵ���������ָ��(��Ʒ��)
	CRobotDriverAdaptor* InitRobot(CString strUnitName, CLog* cLog, std::vector<CUnitDriver*>* ppUnitDriver);

	/*
	* InitCamera
	* ��ʼ���������
	* param
	* return	0:�ɹ� ����:�������
	*/
	int InitCamera();

	/*
	* InitDHCamera
	* ��ʼ��������
	* param		tCameraPara:����������
	* return	0:�ɹ� ����:�������
	*/
	int InitDHCamera(T_CAMREA_PARAM tCameraPara);

	/*
	* SwitchDHCamera
	* ���ش�����
	* param		
		nCameraNo	:����ܱ��
		bOpen		:true��false�ر� 
		bSwitchLaser:�Ƿ񿪹ع����ļ��� 
		eCaptureMode:�ɼ�ģʽ 
		eCallBackMode:��ͼģʽ
	* return	0:�ɹ� ����:�������
	* ע�⣺���д���������д�������ļ���ǰ�棬�Ǵ�������д��������������
	*/
	int SwitchDHCamera(int nCameraNo, bool bOpen, bool bSwitchLaser = true,
		E_DHGIGE_ACQUISITION_MODE eCaptureMode = E_ACQUISITION_MODE_SOURCE_SOFTWARE, 
		E_DHGIGE_CALL_BACK eCallBackMode = E_CALL_BACK_MODE_OFF);

	/*
	* InitDepthCamera
	* ��ʼ����ȣ�ȫ�������
	* param		tDepthCameraPara:��ȣ�ȫ�����������
	* return	0:�ɹ� ����:�������
	*/
	int InitDepthCamera(T_DEPTH_CAMREA_DRIVER_PARAM tDepthCameraPara);

	/*
	* InitMecheyeCamera
	* ��ʼ��÷���������
	* param		tMecheyeCameraDriverPara:÷�������������
	* return	0:�ɹ� ����:�������
	*/
	int InitMecheyeCamera(T_MECHEYE_CAMREA_DRIVER_PARAM &tMecheyeCameraDriverPara);
	//**********************************************//
	
	//**********************************************//
	//*											   *//
	//*************** �����˻������� ***************//
	//*											   *//
	//**********************************************//

	/*
	* GetRobotCtrl
	* ��ȡ�����˿���ָ��
	* param		
	* return	�����˿���ָ��
	*/
	CRobotDriverAdaptor *GetRobotCtrl();

	/*
	* GetRobotEnableState
	* �õ��Ƿ�ʹ�û����˱�־
	* param		
	* return	true:ʹ�û����� false:û��ʹ�û�����
	*/
	bool GetRobotEnableState();

	//-------��ȡ��Ϣ-------//
	/*
	* GetRobotConnectState
	* �õ�����������״̬��δ���ԣ�
	* param
	* return	TRUE:���� FALSE:δ����
	*/
	BOOL GetRobotConnectState();

	/*
	* GetRobotTool
	* �õ������˹���
	* param		nToolNo:���߱�� tTool:�����˹���
	* return	TRUE:�ɹ� FALSE:ʧ��
	*/
	BOOL GetRobotTool(int nToolNo, T_ROBOT_COORS &tTool);

	/*
	* GetRobotTool
	* �õ������˹���
	* param		nToolNo:���߱��
	* return	�����˹���
	*/
	T_ROBOT_COORS GetRobotTool(int nToolNo);

	/*
	* GetRobotPos
	* �õ������˵�ǰֱ������
	* param		nToolNo:���߱�ţ��ɻ�ȡָ��������ֱ������
	* return	��ǰֱ������
	*/
	T_ROBOT_COORS GetRobotPos(int nToolNo = -1);

	/*
	* GetRobotPulse
	* �õ������˵�ǰ�ؽ�����
	* param		
	* return	��ǰ�ؽ�����
	*/
	T_ANGLE_PULSE GetRobotPulse();
	//-------��������-------//
	//-------�˶����-------//
	//-------���ֹͣ-------//
	/*
	* RobotCheckRunning
	* ������������״̬
	* param
	* return	TRUE:������ FALSE:ֹͣ
	*/
	BOOL RobotCheckRunning();

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		nDelayTime:�ȴ���ʼ�˶�ʱ�䣨ms��
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL RobotCheckDone(int nDelayTime = 1000);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ������ tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩 nTool:������깤��
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL RobotCheckDone(T_ROBOT_COORS tAimCoor, T_ROBOT_COORS tCompareLimit = T_ROBOT_COORS(1, 1, 1, -1, -1, -1, 1, 1, 1), int nTool = -1);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL RobotCheckDone(T_ANGLE_PULSE tAimPulse, T_ANGLE_PULSE tCompareLimit = T_ANGLE_PULSE(10, 10, 10, 10, 10, 10, 10, 10, 10));
	//**********************************************//

	//**********************************************//
	//*											   *//
	//************** �ŷ������������ **************//
	//*											   *//
	//**********************************************//

	/*
	* GetMotorCtrl
	* ��ȡ�ŷ��������ָ��
	* param		nUnitMotorNo:������
	* return	�ŷ��������ָ��
	*/
	CUnitDriver *GetMotorCtrl(int nUnitMotorNo = 0);

	/*
	* GetMotorEnableState
	* �õ��Ƿ�ʹ���ŷ������־
	* param		
	* return	true:ʹ���ŷ���� false:û��ʹ���ŷ����
	*/
	bool GetMotorEnableState();

	/*
	* GetMotorNum
	* ��ȡ�ŷ��������
	* param		
	* return	�ŷ��������
	*/
	int GetMotorNum();
	//-------��ȡ��Ϣ-------//

	/*
	* GetCurrentPosition
	* ��ȡ��ǰ�ŷ�������꣨mm��
	* param		nUnitMotorNo:��Ԫ���ŷ������� dPosition:�ŷ�������꣨mm��
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	int GetCurrentPosition(int nUnitMotorNo, double& dPosition);
	//-------��������-------//
	//-------�˶����-------//

	/*
	* AbsPosMove
	* �ŷ�������������˶�
	* param		nUnitMotorNo:��Ԫ���ŷ������� dAbsPosture:�ŷ�����������꣨mm�� dSpeed:�˶��ٶȣ�mm/s��
	* param		dAcc: dDec: dSParam:
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	int AbsPosMove(int nUnitMotorNo, double dAbsPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	int RelaPosMove(int nUnitMotorNo, double dRelaPosture, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);
	//-------���ֹͣ-------//

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL MotorCheckRunning(int nUnitMotorNo = 0);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL MotorCheckDone_CheckRobot(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);

	/*
	* RobotCheckDone
	* �ȴ��������˶�����
	* param		tAimCoor:���Ŀ��ؽ����� tCompareLimit:������귶Χ��������ʾ���������������ʾ����飩
	* return	TRUE:����ֹͣ FALSE:�쳣ֹͣ
	*/
	BOOL MotorCheckDone(int nUnitMotorNo, double dAbsPosture, double dEarlyEndDistance);
	//**********************************************//

	//**********************************************//
	//*											   *//
	//**************** ����������� ****************//
	//*											   *//
	//**********************************************//
	//�谴��ʵ���������ת��ָ������
	void* GetCameraCtrl(int nCameraNo);
	T_CAMREA_PARAM GetCameraParam(int nCameraNo);

	CDHGigeImageCapture *GetDHCameraCtrl(int nSameTypeNo = 0);
	CKinectControl *GetKinectCameraCtrl(int nSameTypeNo = 0);
#if ENABLE_MECH_EYE
	CMechEyeCtrl* GetMechEyeCameraCtrl(int nSameTypeNo = 0);
#endif

	//-------��ͼ-------//
	BOOL CaptureImage(IplImage * Image, int nSameTypeNo = 0);


	//**********************************************//

	//**********************************************//
	//*											   *//
	//***************** ����I/O���� ****************//
	//*											   *//
	//**********************************************//

	int ReadInbit(int nIONo, WORD& wState);
	int ReadOutbit(int nIONo, WORD& wState);
	int WriteOutbit(int nIONo, WORD wState);

	//-------�������-------//
	int GetAllEmgStopState(WORD& wState);									//��ȡ�ܼ�ͣ�ź�
	int OpenAirSolenoidValue();												//��·��ŷ�����
	int CloseAirSolenoidValue();											//��·��ŷ��ر�
	int GetAirPressureSignal(WORD &wState);									//����ѹǿ�ź�
	int OpenRobotControlCabinetPower();										//�����������ƹ��Դ
	//  ��ʾ�ƿ��أ���1����2����3��
	int SetCautionLightGreen(WORD on_off);//��ʾ����
	int SetCautionLightYellow(WORD on_off);//��ʾ�ƻ�
	int SetCautionLightRed(WORD on_off);//��ʾ�ƺ�

	//-------�����أ�ȫ����-------//
	//��Դ
	int OpenCameraPower(int nCameraNo = 0);										//�������Դ
	int CloseCameraPower(int nCameraNo = 0);									//�ر������Դ

	//-------�����أ���㣩-------//
	//�����/�����
	int OpenSupLED(int nCameraNo = 0);											//�������/����Ƶ�Դ
	int CloseSupLED(int nCameraNo = 0);											//�ر������/����Ƶ�Դ
	int OpenSupLEDCtrlSignalPower(int nCameraNo = 0);							//������ƿ����źŵ�Դ
	int CloseSupLEDCtrlSignalPower(int nCameraNo = 0);							//�ر�����ƿ����źŵ�Դ

	//��ͷ��
	int OpenLensCap(int nCameraNo = 0);										//�򿪾�ͷ��
	int CloseLensCap(int nCameraNo = 0);									//�رվ�ͷ��
	int GetLensCapState(WORD &wState, int nCameraNo = 0);					//��ȡ��ͷ��״̬

	//�߼���
	int GetLaserNum(int nCameraNo);
	int OpenLaser(int nCameraNo = 0, int nLeaserLineNo = 0);					//�򿪼�����
	int CloseLaser(int nCameraNo = 0, int nLeaserLineNo = 0);					//�رռ�����

	//-------���������-------//
	//int OpenRobot();														//���ⲿ����
	//int CloseRobot();														//�ر��ⲿ����
	//int OpenCallJob();														//�򿪳������IO
	//int CloseCallJob();														//�رճ������IO
	//int OpenRobotPause();													//���ⲿ��ͣIO
	//int CloseRobotPause();													//�ر��ⲿ��ͣIO
	//int OpenExRobotEmg();													//�򿪻�е���ⲿ��ͣ
	//int CloseExRobotEmg();													//�رջ�е���ⲿ��ͣ
	//int OpenResetErrors();													//�򿪸�λ��е�۴���/����
	//int CloseResetErrors();													//�رո�λ��е�۴���/����

	//int GetRobotRunningSignal(WORD &wState);								//��ȡ�����������ź�
	int GetRobotServoSignal(WORD &wState);									//��ȡ��е���ŷ���ͨ���ź�
	//int GetRobotErrorsSignal(WORD &wState);									//��ȡ��е�۷�������/�����ź�
	//int GetRobotBatteryWarningSignal(WORD &wState);							//��ȡ��ؾ����ź�
	//int GetRobotOperationalOriginSignal(WORD &wState);						//��ȡ��ҵԭ���ź�
	//int GetRobotMidwayStartSignal(WORD &wState);							//��ȡ����;�����ź�
	//int GetRobotEmgStopSignal(WORD &wState);								//��ȡ��ͣ�����ź�
	//int GetRobotLongRangeModeSignal(WORD &wState);							//��ȡԶ��ģʽ�ź�
	//int GetRobotManualModeSignal(WORD &wState);								//��ȡ�ֶ�ģʽ�ź�
	//int GetRobotTeachModeSignal(WORD &wState);								//��ȡʾ��ģʽ�ź�


	//**********************************************//


	






	//**********************************************//
	//*											   *//
	//****************** Ӳ����Դ ******************//
	//*											   *//
	//**********************************************//
	//����������
	CRobotDriverAdaptor *m_pYasakawaRobotDriver = NULL;//����
	//�ŷ�����
	std::vector < CUnitDriver *> m_pContralCard;//�������ƿ�
	//IO����
	CBasicIOControl *m_pBasicIOControl;//�������ƿ�
	//�������
	std::vector<CDHGigeImageCapture*> m_vpImageCapture;//������
	std::vector < CKinectControl *> m_vpKinectControl;//ȫ�����

#if ENABLE_MECH_EYE
	std::vector < CMechEyeCtrl*> m_vpMechMindDevice;//÷���������
#endif

	//**********************************************//


	//**********************************************//
	//*											   *//
	//****************** ���ز��� ******************//
	//*											   *//
	//**********************************************//
	T_CONTRAL_UNIT m_tContralUnit;
	T_COMMON_IO m_tCommonIO;
	int m_nMeasureCameraNo = -1;
	int m_nTrackCameraNo = -1;
	int m_nLineScanCameraNo = -1;
	std::vector< T_MOTOR_PARAM> m_vtMotorPara;
	std::vector<T_CAMREA_PARAM> m_vtCameraPara;
	BOOL LoadBaseIOParam(CString strUnitName, T_COMMON_IO &m_tCommonIO);//���ػ���IO����
	BOOL LoadRobotIOParam(CString strUnitName, T_YASKAWA_ROBOT_IO &m_tRobotIO);//���ػ�����IO����
	BOOL LoadMotorParam(CString strUnitName, std::vector< T_MOTOR_PARAM> &m_vtMotorPara);//�����ŷ��������

	//****************** ������� ******************//
	BOOL LoadDeviceIO();
	BOOL LoadRobotIOParam();
	DWORD SwitchIO(CString sName, bool bOpen);
	bool CheckCO2IO(CString sName); // ��Ĭ��ֵ��ͬ true ��Ĭ��ֵ�෴ false ������ʱ
	bool CheckInIO(CString sName); // ��Ĭ��ֵ��ͬ true ��Ĭ��ֵ�෴ false
	bool CheckOutIO(CString sName); // ��Ĭ��ֵ��ͬ true ��Ĭ��ֵ�෴ false
	std::map<CString, T_IO_PARAM> m_mtRobotIO; // ����������ֵĶ��IO
	std::map<CString, T_IO_PARAM> m_mtDeviceIO; // �����豸����IO ÿ�����Ƶ�Ԫ������һ���豸IO,��ͨ�����Ƶ�Ԫ����

	//�����������
	bool LoadDHCompen(COPini& opini, XI_POINT& tPoint);
	bool LoadDHCompenNew(COPini& opini, double ppdHandEyeCompen[][4]);
	BOOL LoadCameraParam(CString strUnitName, std::vector<T_CAMREA_PARAM>& vtCameraPara);
	bool LoadDHCameraParam(COPini& opini, bool& bAutoOpen, int &nInstallPos, E_FLIP_MODE &eFilpMode, T_ROBOT_COORS& tCameraTool);
	BOOL LoadDHCameraParam(COPini& opini, T_DH_CAMREA_DRIVER_PARAM& tDHCameraDriverPara);//������
	BOOL LoadDepthCameraParam(COPini& opini, T_DEPTH_CAMREA_DRIVER_PARAM& tPANOCameraDriverPara);//ȫ�����
	BOOL LoadMecheyeCameraParam(COPini& opini, T_MECHEYE_CAMREA_DRIVER_PARAM& tMecheyeCameraDriverPara);//÷���������
	BOOL LoadCameraBaseParam(COPini& opini, T_HAND_EYE_CALI_PARAM& tHandEyeCaliPara);//����ڲ�
	BOOL LoadCameraHandEyeCaliParam(COPini& opini, T_HAND_EYE_CALI_PARAM &tHandEyeCaliPara);//���۹�ϵ
	BOOL LoadCameraTransfomParam(COPini& opini, T_CAMERA_TRANSFORM_PARAM& tCameraTransfomPara);//ת����ϵ
	BOOL LoadCameraDistortionParam(COPini& opini, T_CAMERA_DISTORTION& tCameraDistortion);//�������
	BOOL LoadLaserPlanEquation(COPini& opini, T_LASER_PLAN_EQUATION& tLaserPlanEquation);
	BOOL LoadCameraIOParam(COPini& opini, T_CAMERA_IO& tCameraIO);//IO����
	//**********************************************//

	CLog* GetLog();
//protected:
	CString GetFolderPath();
	CString GetUnitName();

	
private:
	CServoMotorDriver *m_pServoMotorDriver;
	bool m_bInitRobotMark = false;
	CLog* m_cUnitLog = NULL;
};

