// IOControl.h: interface for the CIOControl class.
//˵����
//DMC5400:֧�ֵ���DMC5000ϵ�п�CAN��չ
//IOC0640:֧�ֵ��Ż����IOC0640��
//���ߣ�
//������
//�޸����ڣ�
//20210608
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_)
#define AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_


#include ".\Apps\PLib\BasicFunc\Const.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#define ON	0
#define OFF 1
#define INVALID_IO_NUM 9999

//�����������ڳ�ʼ��ʱ���������ͼ���m_mapCardModel��Ա����
const int IOC0640 = 1;// _T("IOC0640");
const int IOC1280 = 2;// _T("IOC1280");
const int DMC_CAN = 3;// _T("DMC_CAN");
const int DMC_CAT = 4;// _T("DMC_CAT");

typedef struct
{
	//ͨ��
	int nCardModel;		
	int nHardwareCardNo;
	int nSoftwareCardNoStrat;
	int nSoftwareCardNoEnd;
	int nMaxIONum;
	//IOC
	int nIOCFILTER;
	//CAN
	int nCANCount;
	int nCANBaud;
	//CAT
	int nCATCycleTime;
	int nCATStartIONo;
}T_CARD_PARA;



typedef struct
{
	//�����
	int nSupLEDIO;											//������ܿ�IO
	int nSupLED1IO;											//�����1IO
	int nSupLED2IO;											//�����2IO

	//���ɽ�
	int nAirSolenoidValueIO;								//��·��ŷ�IO
	int nAntisplashIO;										//���ɽ�����IO
	int nAntisplashOpenedIO;								//���ɽ������״̬IO
	int nAntisplashClosedIO;								//���ɽ�����ر�״̬IO

	//����
	int nLaserIO;											//����IO

	//������
	int nPlamsaPowerIO;										//�����ӵ�Դ
	int nPlamsaArcIO;										//��������IO
	int nArcFeedbackIO;										//�����ӻ�����IO
	int nSmallArcFeedbackIO;								//С������IO
}T_CUTTING_ROBOT_IO;

typedef struct
{
}T_HANDLING_ROBOT_IO;

typedef struct
{
	T_YASKAWA_ROBOT_IO tYaskawaRobotIO;
	T_CUTTING_ROBOT_IO tCuttingRobotIO;
	T_HANDLING_ROBOT_IO tHandlingRobotIO;
}T_ROBOT_IO;

class CIOControl  
{
public:
    CIOControl();
    virtual ~CIOControl();

private:
	//��ʼ������
	void OpenIoCard();											//��ʼ��IO��
	void CloseIoCard();											//�ر�IO��

	void LoadIO();												//����IO��
	bool LoadRobotIO();
	bool CheckBitNo(WORD bitno);								//���IO���Ƿ���Ч
	void CheckIOPara();	

    //������дIO����
    DWORD ReadInbit(WORD bitno);
    DWORD ReadOutbit(WORD bitno);

public:
    DWORD WriteOutbit(WORD bitno, WORD on_off);
   
	std::vector<CString> m_vstrCardModel;		//IO������
	std::map<CString, int> m_mapCardModel;		//IO�������ļ�����ָ���������
	std::map<int, int> m_mapCardNum;			//SoftwareCardNoָ��m_vtCardPara���±�
	std::vector<T_CARD_PARA> m_vtCardPara;		//IO����Ϣ
	bool m_bIOCCardExists;
	bool m_bDMCCardExists;
	int m_nMagnetizingDelayTime;

public:
	//*************************************************����*************************************************//
	bool TotalEmg(bool bStopArc, bool bPopup = true);									//��ͣ
	void OpenAirconditioner();															//�յ�����
	void CloseAirconditioner();															//�յ��ر�
	void OpenDustRemoval();																//��������
	void CloseDustRemoval();															//�����ر�
	void OpenWarningLights();															//��ʾ����˸����
	void CloseWarningLights();															//��ʾ����˸�ر�	

	int m_nAirconditionerIO;															//�յ�IO
	int m_nDustRemovalIO;																//����IO
	int m_nWarningLightsIO;																//��ʾ��IO
	int m_nTotalEmgIO;																	//�豸��ͣIO

	//************************************************��е��************************************************//
	void OpenRobot(int nRobotNo);														//���ⲿ����
	void CloseRobot(int nRobotNo);														//�ر��ⲿ����
	void OpenCallJob(int nRobotNo);														//�򿪳������IO
	void CloseCallJob(int nRobotNo);													//�رճ������IO
	void ResetErrors(int nRobotNo);														//��е�۴���/������λ
	void ResetExRobotEmg(int nRobotNo);													//���û�е���ⲿ��ͣ
	void SetExRobotEmg(int nRobotNo);													//��е���ⲿ��ͣ

	bool CheckRobotMoveSafe();
	int CheckRobotDone(int nRobotNo, int nDelayTime, int nProximity = 0);				//�˶�����,0���ã�1���ź�ֹͣ��2���ź�ֹͣ,С��0�쳣
	bool RobotServoOn(int nRobotNo, bool bStopArc, bool bPopup = true);					//��е���ŷ���ͨ��
	bool Errors(int nRobotNo);															//��е�۷�������/����	
	bool BatteryWarning(int nRobotNo);													//��ؾ���
	bool OperationalOrigin(int nRobotNo);												//��ҵԭ��
	bool MidwayStart(int nRobotNo);														//����;����

	int m_nRobotMoveSafeIO;
	std::vector<T_ROBOT_IO> m_vtRobotIO;												//��е��IO
	int m_nRobotNum;

	//**********************************************�и��е��**********************************************//
	//�����
	void OpenSupLED(int nRobotNo);											//��������ܵ�Դ
	void CloseSupLED(int nRobotNo);											//�ر�������ܵ�Դ
	void OpenSupLEDCtrlSignalPower(int nRobotNo);							//������ƿ����źŵ�Դ
	void CloseSupLEDCtrlSignalPower(int nRobotNo);							//�ر�����ƿ����źŵ�Դ

	//���ɽ�
	void OpenAntisplash(int nRobotNo);										//�򿪷��ɽ�����
	void CloseAntisplash(int nRobotNo);										//�رշ��ɽ�����
	bool GetAntisplashState(int nRobotNo, bool &bState);					//��ȡ���ɽ�����״̬
	void OpenAirSolenoidValue(int nRobotNo);								//��·��ŷ�����
	void CloseAirSolenoidValue(int nRobotNo);								//��·��ŷ��ر�

	//����
	void OpenLaser(int nRobotNo);											//�򿪼���
	void CloseLaser(int nRobotNo);											//�رռ���

	//������
	DWORD OpenPlamsa(int nRobotNo);											//�򿪵�����
	DWORD ClosePlamsa(int nRobotNo);										//�رյ�����
	DWORD StartPlamsaArc(int nRobotNo);										//��
	DWORD StopPlamsaArc(int nRobotNo);										//ͣ��
	bool ReadPlamsaArc(int nRobotNo);
	bool PlamsaArcFeedback(int nRobotNo);									//������
	bool SmallArcFeedback(int nRobotNo);									//С������

	//**********************************************���˻�е��**********************************************//
	//ȫ�����
	void OpenPanoramaCamera();									//ȫ���������
	void ClosePanoramaCamera();									//ȫ������ر�

	int m_nPanoramaCameraIO;									//ȫ�����IO

	//�����	
	void Demagnetizing();										//������˴�
	void Magnetizing(int nLevel);								//�������ţ���ѡ�����ȼ�
	void LockMagnet();											//���������
	void UnlockMagnet();										//���������
	bool GrabSuccess(int nNum);									//ץȡ�ɹ�
	bool MagnetizingSuccess();									//��ųɹ��ź�
	bool DemagnetizingSuccess();								//�˴ųɹ��ź�
	void MagnetizingLv1();										//�����ȼ�1
	void MagnetizingLv2();										//�����ȼ�2
	void MagnetizingLv3();										//�����ȼ�3
	void OpenMagnet2();											//���õ����2	
	void CloseMagnet2();										//�رյ����2

	int m_nMagnat1GrabSuccessIO;								//�����1��������IO
	int m_nMagnat2GrabSuccessIO;								//�����2��������IO
	int m_nMagnetizingSuccessIO;								//��ųɹ�IO
	int m_nDemagnetizingSuccessIO;								//�˴ųɹ�IO
	int m_nMagnet2ControlIO;									//�����2����IO
	int m_nDemagnetizingIO;										//������˴�IO
	int m_nMagnetUnlockIO;										//�����˴�����IO		
	int m_nMagnetizingLv1IO;									//�����Lv1IO
	int m_nMagnetizingLv2IO;									//�����Lv2IO
	int m_nMagnetizingLv3IO;									//�����Lv3IO

	//**********************************************���˻�е��**********************************************//
	void OpenCoolingForPolisher();
	void CloseCoolingForPolisher();
	void OpenPolisher();
	void ClosePolisher();
	void OpenMagnetForPolish();
	void CloseMagnetForPolish();

	int m_nCoolingForPolisher;
	int m_nOpenPolisher;
	int m_nMagnetForPolish;

	//**********************************************���������**********************************************//
	void PlatformMagnetizing();									//ƽ̨��������
	void PlatformDemagnetizing();								//ƽ̨������˴�	
	bool Door1Close();											//�Ž�1״̬����
	bool Door2Close();											//�Ž�2״̬����
	void OpenDoor1();											//�Ž�1����
	void OpenDoor2();											//�Ž�2����
	void CloseDoor1();											//�Ž�1�ر�
	void CloseDoor2();											//�Ž�2�ر�
	
	int m_nDoor1CloseIO;										//�Ž�1IO
	int m_nDoor2CloseIO;										//�Ž�2IO
	int m_nPlatformMagnatIO;									//ƽ̨���������IO				
	int m_nDoor1ControlIO;										//�Ž�1����IO
	int m_nDoor2ControlIO;										//�Ž�2����IO

	//*************************************************ƽ̨*************************************************//
	bool PlatformUp();											//ƽ̨���������ź�
	bool PlatformDown();                                        //ƽ̨�½������ź�
	void PlatformControl(bool bUpOrDown);						//ƽ̨��������
	bool OpenMagnetInChamferingTable(int iNo);
	bool CloseMagnetInChamferingTable(int iNo);

	int m_nUpOrDownIO;											//ƽ̨��������IO
	int m_nPlatformUpLimitIO;									//ƽ̨��������IO
	int m_nPlatformDownLimitIO;									//ƽ̨�½�����IO
	
	//***********************************************�������***********************************************//
	void OpenExCamera();										//�����������
	void CloseExCamera();										//��������ر�

	int m_nExCameraIO;											//�������IO

	//*********************************************����¿�ר��*********************************************//
	void LoadIOForBiggerPart();
	bool StartPlatform(int nPlatformNo);	
	void StopPlatform(int nPlatformNo);
	void OpenTransducer();
	void CloseTransducer();
	bool GetTransducerState();
	bool GetLightCurtainState();								//��Ļ���������ź�ʱ���ܽ��������ϲ���
	bool PlatformStart(int nPlatformNo);
	bool PlatformDec(int nPlatformNo);
	bool PlatformStop(int nPlatformNo);

	int m_nPlatform1CorotationStartIO;
	int m_nPlatform1CorotationDecIO;
	int m_nPlatform1CorotationStopIO;
	int m_nPlatform2CorotationStartIO;
	int m_nPlatform2CorotationDecIO;
	int m_nPlatform2CorotationStopIO;
	int m_nLightCurtain1IO;
	int m_nLightCurtain2IO;
	int m_nTransducer1ClosedIO;
	int m_nTransducer2ClosedIO;
	int m_nPlatform1MoveIO;
	int m_nPlatform2MoveIO;
	
	int m_nPlatform1CorotationIO;
	int m_nPlatform1ReversalIO;
	int m_nPlatform2CorotationIO;
	int m_nPlatform2ReversalIO;
	int m_nTransducer1OpenIO;
	int m_nTransducer2OpenIO;

	//��̨
	bool TableUpIn(int nTableNo);
	bool TableUpOut(int nTableNo);
	bool CheckForHandling(int nTableNo);
	bool CheckForBlanking(int nTableNo);

	HANDLE m_handleTableMove[2];

	int m_nTableUpIn[2];
	int m_nTableUpOut[2];
	int m_nTableDownIn[2];
	int m_nTableDownOut[2];

	int m_nTableUpInLimit[2];
	int m_nTableUpOutLimit[2];
	int m_nTableUpInMiddle[2];
	int m_nTableDownInLimit[2];

	int m_nTableDelayTime;
	int m_nTableTimeLimit;
};

#endif // !defined(AFX_IOCONTROL_H__492E8CFB_AB04_42AC_8954_92B41A9D6719__INCLUDED_)
