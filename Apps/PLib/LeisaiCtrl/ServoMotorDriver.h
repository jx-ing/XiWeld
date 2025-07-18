///////////////////// �����˶�����-�������� /////////////////////
/*
˵����
1�����з�bool�ͷ���ֵ��0��ʾ�ɹ���!0��ʾ������;bool�ͷ���ֵ��true��ʾ�ɹ���false��ʾʧ�ܣ��޴����룻
2������������Ϊ�������ƿ����أ��������¶��壻
3��Ϊ���ڿ��ƣ����������嵱������Ϊ��1.0����������ģ�
*---*ע�⣺���ֹ��ܵ��ùٷ�APIʱ����������������ơ��͡��������嵱�����ơ��������ã�
4����LTSMC�������ޡ�����������ơ��ӿڣ�
5�����嵱�����㹫ʽΪ��ʵ������/ʵ�ʾ���(pulse/mm)
*/


#pragma once
#include "LTDMC.h"
#include "LTSMC.h"
#include "IOC0640.H"
//#include "DriverAdapter.h"
#include ".\Apps\PLib\PanasonicServo\AbsoluteEncoder.h"
#include "MoveCtrlModule.h"

/*����ֵ�����붨��*/
#define INPUT_INFO_ERROR		-1	//������Ϣ����
#define NO_FOLLOW_FUN			-2	//û�и��湦��
#define	CTRL_CARD_NUM_ERROR		-3	//���ƿ���������
#define	CTRL_CARD_NO_FOUND		-4	//û���ҵ����ƿ�
#define	IO_CARD_NO_CONNECT		-5	//û���ҵ�IO��
#define	ABS_ENCODER_DISENABLE	-6	//���Ա�����û��ʹ��
#define	SERIAL_PORT_DISCONNECT	-7	//���ڴ�ʧ��
#define	NO_FUNITION				-8	//�޴˹���
#define	CAN_DISCONNECT			-9	//Can����û������



#ifndef WRITE_LOG
#define WRITE_LOG(data) {CString __str;\
__str.Format("Function:[%s], Line:[%d]	\n%s",__FUNCTION__, __LINE__, data);\
WriteLog(__str);}
#endif // !WRITE_LOG

#define CHECK_RTN_INT(nRtn) if(0 != nRtn)\
{\
WRITE_LOG(GetStr("�쳣����:%d", nRtn));\
return nRtn;\
}
#define CHECK_RTN_BOOL(bRtn) if(true != bRtn)\
{\
WRITE_LOG("�쳣����");\
return bRtn;\
}

#ifndef ON
#define ON	0
#endif // !ON
#ifndef OFF
#define OFF 1
#endif // !OFF

typedef enum
{
	E_CONNECT_TYPE_DIRECT,//ֱ��
	E_CONNECT_TYPE_NETWORK,//����
	E_CONNECT_TYPE_ETHERCAT,//EtherCAT����
//	E_CONNECT_TYPE_SERIAL_PORT,//����
}E_CONNECT_TYPE;

typedef enum
{
	E_CTRL_CARD_TYPE_DMC5410,
	E_CTRL_CARD_TYPE_DMC5800,
	E_CTRL_CARD_TYPE_DMC5810,
	E_CTRL_CARD_TYPE_SMC604,
	E_CTRL_CARD_TYPE_IOC0640,
	E_CTRL_CARD_TYPE_IOC1280,
	E_CTRL_CARD_TYPE_IMC30G_E,
	E_CTRL_CARD_TYPE_MAX_NUM,
}E_CTRL_CARD_TYPE;

typedef enum
{
	E_SERCO_TYPE_PANASONIC,//�����ŷ�
	E_SERCO_TYPE_INOVANCE,//�㴨�ŷ�
//	E_SERCO_TYPE_YASKAWA,//�����ŷ�
E_SERVO_TYPE_MAX_NUM,
}E_SERVO_TYPE;

typedef enum
{
	E_IO_CARD_TYPE_NONE,
	E_IO_CARD_TYPE_IOC0640,
	E_IO_CARD_TYPE_IOC1280,
	E_IO_CARD_TYPE_CAN,
	E_IO_CARD_TYPE_CAT,
E_IO_CARD_TYPE_MAX_NUM,
}E_IO_CARD_TYPE;

typedef struct
{
	double dSpeed = 0;
	double dAcc = 1;
	double dDec = 1;
	double dSParam = 0.5;
}T_AXIS_SPEED;

typedef struct
{
	int nHardCtrlCardNo = -1;//���ƿ�������
	int nSoftCtrlCardNo = -1;//������ƿ����
	int nPortNo = -1;//SMC�˿ںţ�0,1 ��ʾ CANOpen��2��3 ��ʾ EtherCAT �˿�
	E_IO_CARD_TYPE eIOType = E_IO_CARD_TYPE_NONE;//IO����
	int nLocalIONum = 0;//����IO����
	int nSingleIONum = 0;//���ڵ�IO����
	int nNoteNum = 0;//�ڵ���
	int nCANBaudNo = 0;//CAN���߲�����
	int nCATCycleTime = 0;//CAT����ѭ������
	int nIOCFilter = 1;//IOC���˲�����
}T_IO_CTRL_INFO;

typedef struct
{
	int nHardCtrlCardNo = -1;//���ƿ�������
	int nSoftCtrlCardNo = -1;//������ƿ����
	int nSoftAxisNo = -1;//�������
	int nHardAxisNo = -1;//Ӳ������
	CString strAxisName = "";//������
	E_SERVO_TYPE eServoType = E_SERCO_TYPE_PANASONIC;//�ŷ���������
	double dPulseEquivalent = 0;//���嵱����������/ʵ�ʾ��루pulse/mm ��
	int nPulseMode = 0;//����ģʽ

	bool bEnableAbsEncoder = false;//ʹ�ܾ��Ա�����
	int nAbsEncoderPartNo = -1;//���Ա��������ں�
	int nLapPulse = 2000;//��Ȧ������
	int nAbsEncoderDir = 1;//���Ա���������

	bool bEnableSoftLimit = false;//ʹ������λ
	double dPositiveLimit = 0;//����λ
	double dNegativeLimit = 0;//����λ

	T_AXIS_SPEED tMaxSpeed;//����,dSpeed��λmm/s,dAccdDec��λs
	T_AXIS_SPEED tMidSpeed;//����,dSpeed��λmm/s,dAccdDec��λs
	T_AXIS_SPEED tMinSpeed;//����,dSpeed��λmm/s,dAccdDec��λs

	bool bAlmLogic = true;//������ƽ

	int nGearFollowProfile = 0;//����ģʽ,0:��ֹ��1:�����ᣬ2:�Ӷ���
	int nMasterAxisNo = -1;//�����ţ�����ģʽΪ�Ӷ���ʱʹ��
}T_AXIS_CTRL_INFO;

typedef struct
{
	int nHardCtrlCardNo = -1;//���ƿ�������
	int nSoftCtrlCardNo = -1;//������ƿ����
	int nAxisNum = -1;//������
	E_CONNECT_TYPE eConnectType = E_CONNECT_TYPE_DIRECT;//������
	CString strNetWorkIP = "";//��������IP��ַ
	E_CTRL_CARD_TYPE eCtrlCardType = E_CTRL_CARD_TYPE_DMC5410;//�˶����ƿ�����
	T_IO_CTRL_INFO tIOCtrlInfo;
	std::vector<T_AXIS_CTRL_INFO> vtAxisInfo;
}T_CTRL_CARD_INFO;

class CServoMotorDriver : public CMoveCtrlModule
{
public:
	CServoMotorDriver();
	~CServoMotorDriver();
	std::vector<T_CTRL_CARD_INFO> m_vtCtrlCardInfo;
	std::map<int, CAbsoluteEncoder*> m_mAbsEncoderCtrl;//���Ա���������

	/*******************************  ��ʼ��  *******************************/
	virtual int InitCtrlCard(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo);//��ʼ�����ƿ�
	bool LoadCtrlCardParam(std::vector<T_CTRL_CARD_INFO> &vtCtrlCardInfo);//���ƿ�����
	bool SaveCtrlCardParam(std::vector<T_CTRL_CARD_INFO> vtCtrlCardInfo);

	/*******************************  �ŷ����  *******************************/
	//�ŷ�ʹ��
	virtual int SetSevon(int nSoftAxisNo, bool bEnable);
	virtual bool GetSevon(int nSoftAxisNo, bool &bEnable);
	//�ŷ�׼��
	virtual bool GetSevonRdy(int nSoftAxisNo, bool &bSevonRdy);

	/*******************************  �˶����  *******************************/
	//�����˶�
	virtual int PosMove(int nSoftAxisNo, long lDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//���������˶�,dSpeed��λpulse/s,dAccdDec��λs, nMode�˶�ģʽ��0���������ģʽ��1����������ģʽ
	virtual int PosMove(int nSoftAxisNo, double dDistance, int nMode, double dSpeed, double dAcc, double dDec, double dSParam = 0.1);//����λ���˶�,dSpeed��λmm/s,dAccdDec��λs
	virtual int ContiMove(int nSoftAxisNo, int nDir, double dSpeed, double dAcc, double dSParam = 0.1);//���������˶�,dSpeed��λmm/s,dAcc��λs
	virtual bool CheckAxisRun(int nSoftAxisNo, int *pnError = NULL);//�������״̬

	//ֹͣ���
	virtual bool EmgStop(int *nSize = NULL, int *pnError = NULL);//��ͣ
	virtual int EmgStopCtrlCard(int nSoftCtrlCardNo);
	virtual int EmgStopAxis(int nSoftAxisNo);
	virtual int SetDecelStopTime(int nSoftAxisNo, double dDec);//���ü���ֹͣʱ��,dDec��λs
	virtual int DecelStop(int nSoftAxisNo);//����ֹͣ

	//��ǰ����
	virtual int GetCurrentPosition(int nSoftAxisNo, long &lPosition);//��ǰλ��
	virtual int GetCurrentPosition(int nSoftAxisNo, double &dPosition);
	virtual int SetCurrentPosition(int nSoftAxisNo, long lPosition);
	virtual int SetCurrentPosition(int nSoftAxisNo, double dPosition);

	//Ŀ������
	virtual int GetTargetPosition(int nSoftAxisNo, long &lPosition);//Ŀ��λ��
	virtual int GetTargetPosition(int nSoftAxisNo, double &dPosition);
	virtual int SetTargetPosition(int nSoftAxisNo, long lPosition, bool bupdate = false);//bupdate=true ǿ�Ƹı�λ��
	virtual int SetTargetPosition(int nSoftAxisNo, double dPosition, bool bupdate = false);

	//�ٶ�
	virtual int GetSpeed(int nSoftAxisNo, long &lSpeed);
	virtual int GetSpeed(int nSoftAxisNo, double &dSpeed);
	virtual int ChangeSpeed(int nSoftAxisNo, long lSpeed, double dChangeTime = 0.1);//��;����
	virtual int ChangeSpeed(int nSoftAxisNo, double dSpeed, double dChangeTime = 0.1);
	virtual double GetMaxSpeed(int nSoftAxisNo); // ��ȡ����ٶ� mm/min

	//��ȡ�趨�����ݣ��ǵ�ǰֵ��
	virtual int GetDataPulseEquivalent(int nSoftAxisNo, double &dPulseEquivalent);//���嵱��
	virtual int GetDataSpeedParam(int nSoftAxisNo, T_AXIS_SPEED &tMinSpeedParam, T_AXIS_SPEED &tMidSpeedParam, T_AXIS_SPEED &tMaxSpeedParam);//���嵱��

	/*******************************  ����ģʽ  *******************************/
	virtual int SetGearFollowProfile(int nMasterSoftAxisNo, int nDrivenSoftAxisNo, bool bEnable, double dRatio = 1.0);
	virtual int GetGearFollowProfile(int nDrivenSoftAxisNo, int &nMasterSoftAxisNo, bool &bEnable, double &dRatio);

	/*******************************  IO����  *******************************/
	virtual int ReadInbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID = 0);//nNoteID����0Ϊ����IO,��չ����1��ʼ
	virtual int ReadOutbit(int nSoftCtrlCardNo, int bBitNo, WORD &nStatus, int nNoteID = 0);//nNoteID����0Ϊ����IO,��չ����1��ʼ
	virtual int WriteOutbit(int nSoftCtrlCardNo, int bBitNo, WORD nStatus, int nNoteID = 0);//nNoteID����0Ϊ����IO,��չ����1��ʼ

	/*******************************  ADDA����(CAN����)  *******************************/
	virtual int SetADMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode);//nChannel:ģ��������ͨ���ţ���Χ 0~3, nMode:����ģʽ��0����ѹģʽ��1������ģʽ
	virtual int SetDAMode(int nSoftCtrlCardNo, int nNoteID, int nChannel, int nMode);//nChannel:ģ�������ͨ���ţ���Χ 0~1, nMode:����ģʽ��0����ѹģʽ��1������ģʽ
	virtual int SetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double dValue);//dValuev:���ֵ����λ��mV/mA
	virtual int GetDAChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue);//dValuev:���ֵ����λ��mV/mA
	virtual int GetADChannel(int nSoftCtrlCardNo, int nNoteID, int nChannel, double &dValue);//dValuev:����ֵ����λ��mV/mA

	/*******************************  ���Ա�����  *******************************/
	virtual int OpenAbsEncoder(int nSoftAxisNo);//�򿪱���������
	virtual int CloseAbsEncoder(int nSoftAxisNo);//�رմ���
	virtual int GetAbsData(int nSoftAxisNo, double &dAbsData);//�Ӿ���ֵ��������ȡ��ǰ���꣬��Ҫ 1s ���ң���ȡʱ�ŷ�������뾲ֹ
	virtual int ClearManyLapData(int nSoftAxisNo);//��ն�Ȧ����
	virtual int SetInitLapData(int nSoftAxisNo);//������㵥Ȧ����

	/*******************************  �岹�˶�  *******************************/



	/*******************************  ��ȡ�Ῠ��Ϣ  *******************************/
	bool GetAxisCardInfo(int nSoftAxisNo, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetAxisCardInfo(int nHardCtrlCardNo, int nHardAxisNo, E_CTRL_CARD_TYPE eCtrlCardType, T_AXIS_CTRL_INFO &tAxisInfo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetCtrlCardInfo(int nSoftCtrlCardNo, T_CTRL_CARD_INFO &tCtrlCardInfo);
	bool GetCtrlCardInfo(int nHardCtrlCardNo, E_CTRL_CARD_TYPE eCtrlCardType, T_CTRL_CARD_INFO &tCtrlCardInfo);

	/*******************************  ����  *******************************/
	int GetSoftAxisNo(int nSoftCtrlCardNo);
	void SetSoftAxisNo();
	// ��ȡ���嵱��
	double GetPulseEquivalent(int nSoftAxisNo);
private:

	int InitDirectCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);
	int InitNetWorkCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);
	int InitIOCCtrlCard(T_CTRL_CARD_INFO tCtrlCardInfo, int &nCardNum);

	int InitCanIOCtrl(T_IO_CTRL_INFO tIOInfo);
	int InitCatIOCtrl(T_IO_CTRL_INFO tIOInfo);

	int InitDirectAxisParam(T_AXIS_CTRL_INFO tAxisInfo);
	int InitNetWorkAxisParam(T_AXIS_CTRL_INFO tAxisInfo);



	int m_nCardNum = 0;
	int m_nAxisNum = 0;
};
