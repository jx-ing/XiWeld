// MoveCtrlModule.h: interface for the CMoveCtrlModule class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_)
#define AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000




// const WORD D5800_MAX_AXIS_NO = 7;
// const WORD D5800_LOGIC = 0;
// const WORD D5800_CRD = 0;
// 
// const WORD D2C80_OUTBIT_SERVO_ENABLE = 1;
// 
// const int DMC5800_CONTIMOV_INSTRUCT_NUM = 5000;

#include ".\Apps\PLib\BasicFunc\Const.h"

/*************************** �����˶����� ********************************/
class CMoveCtrlModule  
{
public:
	CMoveCtrlModule();
	virtual ~CMoveCtrlModule();

private:
	
public:
    //��λ�˶�:��λ��dDis:mm,dMinVel��dMaxVel��mm/min��dAcc��dDec��s
	void                        PosMoveDis
	(
		WORD wAxis, double dDis, WORD wPositionMode, 
		double dMinVel, double dMaxVel, double dAcc, 
		double dDec, BOOL bIfCheckDone = FALSE
	);
    /*
    ��������:
    void ContiMove(WORD wAxis, WORD wDir,double dMinVel, double dMaxVel, double dAcc, double dDec);
    ����˵��:
    ����ִ�������˶��˶�
    ����˵��:
    wDir 0:����1:����;
    dAcc��dDec����2610Ϊ�Ӽ���ʱ�䣬����ȡһ����С��������0.2������2c80Ϊ���ٶȺͼ��ٶȣ�����ȡ��㣬��100000.
    */
    void ContiMoveDis(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    
	DWORD                       SetPositionDis(WORD wAxis, double dPos);
    double                      GetPositionDis(WORD wAxis);

	void                        GetMachinePositionDis(double &dCurPosX, double &dCurPosY);

	WORD                        CheckDoneDis(WORD wAxis);
	WORD						CheckDoneAllDis(WORD wAxisNum);
	void                        EmgStopDis(); // �����Ῠ��ͣ

	DWORD						ReadInbitDis(WORD wBitNo);
	//void						EmgStop(WORD wCardNo);

	// ����������������뵥λ��mm(��)���ٶȵ�λmm(��)/min���Ӽ��ٶȵ�λmm(��)/s*min(1���Ӽ��ٵ�����mm(��)/min)
    void                        UnitedMoveDis
	(
		WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode, 
        double dMinVel, double dMaxVel, double dAcc, double dDec, 
		WORD wCardNo = CONTROL_CARD_NO
	);

public:
    // ��λ��dDis : pulse, dMinVel��dMaxVel��pulse / s��dAcc��dDec��s
    /*
    ��������:
    void PosMove(WORD wAxis, long lDist, WORD wPositionMode, double dMinVel, double dMaxVel, double dAcc, double dDec)
    ����˵��:
    ����ִ�е�λ�˶�
    ����˵��:
    lDist ��λ����
    wPositionMode 0:�������ģʽ��1:��������ģʽ;
    dAcc��dDec����2610Ϊ�Ӽ���ʱ�䣬����ȡһ����С��������0.2������2c80Ϊ���ٶȺͼ��ٶȣ�����ȡ��㣬��100000.
    */
    void PosMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    /*
    ��������:
    void ContiMove(WORD wAxis, WORD wDir,double dMinVel, double dMaxVel, double dAcc, double dDec);
    ����˵��:
    ����ִ�������˶��˶�
    ����˵��:
    wDir 0:����1:����;
    dAcc��dDec����2610Ϊ�Ӽ���ʱ�䣬����ȡһ����С��������0.2������2c80Ϊ���ٶȺͼ��ٶȣ�����ȡ��㣬��100000.
    */
    void ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    /*
    ��������:
    long GetPosition(WORD wAxis)
    ����˵��:
    ��ȡָ�����ָ������λ��
    ����˵��:
    wAxis ָ�����
    ����ֵ:
    ָ������λ��
    */
    long GetPosition(WORD wAxis);

    /*
    ��������:
    DWORD SetPosition(WORD wAxis, long lPosition)
    ����˵��:
    ����ָ�����ָ������λ��
    ����˵��:
    wAxis ָ�����
    lPosition ָ������λ�ã���λpulse
    ����ֵ:
    �������
    */
    DWORD SetPosition(WORD wAxis, long lPosition);

    //*************************************************************************************************
    // wLogic���߼���ƽ
    // wDir��1  ���� , 2: ����
    //*************************************************************************************************
    void HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    /*
    ��������:
    WORD CheckDone(WORD wAxis)
    ����˵��:
    ���ָ��������˶�״̬
    ����˵��:
    wAxis ָ�����
    ����ֵ:
    0:�����˶�; 1:ָ������ֹͣ
    */
    WORD CheckDone(WORD wAxis);

    /*
    ��������:
    WORD CheckDoneAll(WORD wAxisNum)
    ����˵��:
    ���ָ����Ŀ�ĵ�����˶�״̬
    ����˵��:
    wAxisNum ָ������
    ����ֵ:
    0:���ٴ���һ�����˶�; 1:ָ����Ŀ������ֹͣ
    */
    WORD CheckDoneAll(WORD wAxisNum);

    /*
    ��������:
    CheckDoneMultiCoor(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD)
    ����˵��:
    �������ϵ���˶�״̬(�����ڲ岹�˶�)
    ����˵��:
    wCardNo ָ������
    wCrd ָ�����ƿ��ϵ�����ϵ�� 0~1
    ����ֵ:
    0:����������; 1:����ֹͣ
    ��ע˵��:
    Ŀǰ֧��5800���Ĳ岹�˶�
    */
    WORD CheckDoneMultiCoor(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD);

    /*
    ��������:
    void MultiCoorStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCrd = D5800_CRD, WORD wStopMode)
    ����˵��:
    ֹͣ����ϵ����������˶�,֧��5800���ƿ�(�����ڲ岹�˶�)
    ����˵��:
    wCardNo ָ������
    wCrd ָ�����ƿ��ϵ�����ϵ�� 0~1
    wStopMode �ƶ���ʽ 0:����ֹͣ��1:����ֹͣ
    */
    void MultiCoorStop(WORD wStopMode, WORD wCrd = D5800_CRD, WORD wCardNo = CONTROL_CARD_NO);

    /*
    ��������:
    WORD EmgStop()
    ����˵��:
    ����ֹͣ������
    */
    void EmgStop();

    /*
    ��������:
    WORD EmgStop(WORD wCardNo)
    ����˵��:
    ����ֹͣ�����ᣬ֧��5800���ƿ�
    ����˵��:
    wMoveControlCardType ���ƿ�����
    */
    void EmgStop(WORD wMoveControlCardType);

    /*
    ��������:
    WORD OpenControlCard()
    ����˵��:
    ��ʼ�����ƿ�
    ����ֵ˵��:
    0:û���ҵ����ƿ������߿��ƿ��쳣;
    1~8:���ƿ���;
    */
    WORD OpenControlCard();

    /*
    ��ע˵��:
    5800�˶����ƿ��Ѿ����ṩ�˹���
    */
    WORD OpenControlCard(WORD wMoveControlCardType);

    /*
    ��������:
    void CloseControlCard()
    ����˵��:
    �رտ��ƿ�
    */
    void CloseControlCard();

    /*
    ��������:
    WORD CloseControlCard(WORD wMoveControlCardType)
    ����˵��:
    �ر�ָ�����ƿ�
    ����˵��:
    wMoveControlCardType ���ƿ�����
    */
    void CloseControlCard(WORD wMoveControlCardType);

    /*
    ��������:
    ConfigELMode(WORD wAxis, WORD wLogic)
    ����˵��:
    ����EL��λ�ź�
    ����˵��:
    wAxis ���
    wLogic 0:������λ�͵�ƽ��Ч��1:������λ�ߵ�ƽ��Ч��2:������Ч��������Ч��3:������Ч��������Ч
    */
    void ConfigELMode(WORD wAxis, WORD wLogic);

    /*
    ��������:
    ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO,WORD wAxisNum = D5800_MAX_AXIS_NO)
    ����˵��:
    ����EMG��ͣ�ź�
    ����˵��:
    wCardNo ���ƿ�����
    wAxisNum ָ�����趨������
    wEnable 0:��ֹ��1:����
    wLogic 0:�͵�ƽ��Ч��1:�ߵ�ƽ��Ч
    */
    void ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO, WORD wAxisNum = D5800_MAX_AXIS_NO);

    /*
    ��������:
    void ConfigAlmMode(WORD wCardNo = CONTROL_CARD_NO, WORD wAxisNum = D5800_MAX_AXIS_NO, WORD wLogic = D5800_LOGIC)
    ����˵��:
    ����ALM�ź�
    ��ע:
    ������һ�汾�ӿ�
    */
    void ConfigAlmMode();

    /*
    ��������:
    void ConfigAlmMode(WORD wCardNo = CONTROL_CARD_NO, WORD wAxis, WORD wLogic)
    ����˵��:
    ����ALM�ź�
    ����˵��:
    wCardNo ���ƿ�����
    wAxis ���
    wLogic 0:�͵�ƽ��Ч��1:�ߵ�ƽ��Ч
    ��ע:
    �Ƽ�ʹ�ô˺���
    */
    void ConfigAlmMode(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void LoadPosition();
    void SavePosition();

    /*
    ��������:
    void SetPulseOutmode(WORD wAxis,WORD wOutmode);
    ��������:
    ����ָ������������ģʽ
    ����˵��:
    wAxis ���
    wOutmode ���������ʽѡ��
    */
    void SetPulseOutmode(WORD wAxis, WORD wOutmode);

    /*
    ��������:
    DWORD WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ����ָ��������ĳ������˿ڵĵ�ƽ
    ����˵��:
    wCardNo ���ƿ�����
    wBitNo ����˿ں� DM5800Ϊ0~15
    wCardType ���ƿ�����
    wOnOff 0:����͵�ƽ��1:����ߵ�ƽ
    ����ֵ˵��:
    �������
    */
    DWORD WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    /*
    ��������:
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ����ָ��������ĳ������˿ڵĵ�ƽ
    ����˵��:
    wCardNo ���ƿ�����
    wBitNo ����˿ں� DM5800Ϊ0~15
    ����ֵ˵��:
    ��������˿ڵ�ƽ 0:�͵�ƽ��1:�ߵ�ƽ
    */
    DWORD ReadOutbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    /*
    ��������:
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ����ָ��������ĳ������˿ڵĵ�ƽ
    ����˵��:
    wCardNo ���ƿ�����
    wBitNo ����˿ں� DM5800Ϊ0~15
    wCardType ���ƿ�����
    ����ֵ˵��:
    ��������˿ڵ�ƽ 0:�͵�ƽ��1:�ߵ�ƽ
    */
    DWORD ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    /*
    ��������:
    DWORD ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ����ָ��������ĳ������˿ڵĵ�ƽ
    ����˵��:
    wCardNo ���ƿ�����
    wCardType ���ƿ�����
    wBitNo ����˿ں� DM5800Ϊ0~15
    ����ֵ˵��:
    ָ������˿ڵ�ƽ 1:�ߵ�ƽ 0:�͵�ƽ
    */
    DWORD ReadInbit(WORD wBitNo, WORD wCardType, WORD wCardNo = CONTROL_CARD_NO);

    DWORD IoStatus(WORD wAxis);

    BOOL CheckEmg(WORD wAxis);

    BOOL CheckAlarm(WORD wAxis);

    BOOL CheckMoveLimitPositive(WORD wAxis);

    BOOL CheckMoveLimitNegative(WORD wAxis);

    BOOL CheckHome(WORD wAxis);

    void EnableServo(WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);
    void EnableServo(WORD wAxis, WORD wLogic, int nServoEnableIo = 1, WORD wCardNo = CONTROL_CARD_NO); // 2C80�ŷ�ʹ����ͨ��IO�ڿ��Ƶģ���Ҫָ��IO

    void SetDefaultAcc(double dAcc);
    void SetDefaultDec(double dDec);
    double GetDefaultAcc();
    double GetDefaultDec();

    double GetCurrentSpeed(WORD wAxis);
    DWORD ChangeCurrentSpeed(WORD wAxis, double dVel);

    /*
    ����˵��:
    void InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ������Ϊ��λ������ֱ�߲岹�˶�
    ��ע˵��:
    ֧������2��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    */
    void InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // ֧������3��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // ֧������4��岹��d2610������֧�֣�������
    void InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80֧������6������в岹
    void InterpolationMove6(WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80֧������1~12������в岹
    void InterpolationMoveN(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);


    //DMC-5800 ����������������뵥λ��mm(��)���ٶȵ�λmm(��)/min���Ӽ��ٶȵ�λmm(��)/s*min(1���Ӽ��ٵ�����mm(��)/min)
    void UnitedMove(WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    //

    /*
    ����˵��:
    void InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    �����嵱��Ϊ��λ��ֱ�߲岹�˶�
    ��ע˵��:
    5800֧������2~6����ֱ�߲岹;
    ��Ҫ��ʹ��void SetEquiv(WORD wCardNo = CONTROL_CARD_NO, WORD wAxis, double dEquiv)Ϊ�����������嵱��
    */
    void InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /* + ************************** �����岹���� ******************************** + */
    /*
    ����˵��:
    void StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    �������岹������,��ʼ�����岹
    */
    void StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    void ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    ִ������ֱ�߲岹�˶�
    ����˵��:
    awAxis ����б�
    lDist  Ŀ��λ�����飬������Ϊ��λ
    wPositionMode �˶�ģʽ, 0:�������ģʽ 1:��������ģʽ
    */
    void ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    BOOL  ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    ��ѯ�����岹������ʣ��岹�ռ�
    ����ֵ˵��:
    TRUE �����岹����������ʣ��
    */
    BOOL  ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    BOOL  CheckContiInterMoveState(WORD wAxisNum)
    ����˵��:
    ��ȡָ������ϵ�Ĳ岹�˶�״̬
    */
    short  CheckContiInterMoveState(WORD CardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    DWORD ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    ��ͣ�����岹,�����岹�˶�������ֹͣ
    ����ֵ˵��:
    ���ش������
    */
    DWORD ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    DWORD ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    ����������ͣδ��ɵ������岹�˶�
    */
    DWORD ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    DWORD ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO);
    ����˵��:
    ֹͣ�岹�˶�,�ú������������в岹�˶�,��ʹ����岹���˶����˳��岹ģʽ
    */
    DWORD ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    /*
    ����˵��:
    void  ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO)
    ����˵��:
    ��̬���������岹�ٶȱ���
    ��ע˵��:
    �������ִ�е���ָ��ʱ����ָ����뻺�������Ӹ�ָ�����һ���˶�������ʼ�˶�ʱ������
    */
    void  ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0); // wRatio��Ԥ���ٶȵı���

	/*
	����˵��:
	DWORD StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO)
	����˵��:
	�ر������岹������
	*/
    DWORD StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    BOOL  CheckContiInterMoveDone(WORD CardNo = CONTROL_CARD_NO);

    void SetBuffNum(int nBuffNum);
    int GetBuffNum();
    // ����ֵ��1��buffer��ȫ��ʹ��, 0��bufferδʹ����
    WORD CheckPreBuff(WORD wPara, WORD wMoveControlCardType); // wPara:����2C80Ϊ���ţ�����2610Ϊ���

                                                              /* - ************************** �����岹���� ******************************** - */

                                                              //dDec ����ֹͣʱ��,��λs
                                                              //wStopMode �ƶ���ʽ 0:����ֹͣ�� 1:����ֹͣ
    void DecelStop(WORD wAxis, double dDec, WORD wStopMode = 0);
    void DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode = 0);

    WORD GetCardAxisNum(WORD wCardNo = CONTROL_CARD_NO);

    int m_nBuffNum;

//private:
public:

    void InitAxisPara();
    void SetPulseMode(WORD wCardNo = CONTROL_CARD_NO);

    /*************************************************************************************************/
    /*                                         D2C80����                                             */
    /*************************************************************************************************/
    // dAcc��dDec������2c80Ϊ���ٶȺͼ��ٶȣ�����ȡ��㣬��100000
    void D2C80PosMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D2C80ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // ֧��ͬ��������2��岹(0~3Ϊ��һ�飬4~5Ϊ�ڶ���)������D2C80����������ƣ����ǽӿڱ���
    void D2C80InterpolationMove2Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧������2��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D2C80InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧��ͬ��������3��岹(0~3Ϊ��һ�飬4~5Ϊ�ڶ���)������D2C80����������ƣ����ǽӿڱ���
    void D2C80InterpolationMove3Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧������3��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D2C80InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ��֧��ǰ4��岹
    void D2C80InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    // ֧������4��岹��d2610������֧�֣�������
    void D2C80InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ��֧��ǰ6��岹
    void D2C80InterpolationMove6(long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // 2c80֧������6������в岹
    void D2C80InterpolationMove6(WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // D2C80֧������1~12������в岹
    void D2C80InterpolationMoveN(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    void D2C80StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO);
    void D2C80ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    BOOL  D2C80ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO);
    DWORD D2C80ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO);
    void  D2C80ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO); // wRatio��Ԥ���ٶȵı���
    DWORD D2C80StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO);
    BOOL  D2C80CheckContiInterMoveDone();

    void D2C80DecelStop(WORD wAxis, double dDec);
    void D2C80DecelStopAll(WORD wAxisNum, double dDec);

    long D2C80GetPosition(WORD wAxis);
    DWORD D2C80SetPosition(WORD wAxis, long lPosition);

    double D2C80GetCurrentSpeed(WORD wAxis);
    DWORD D2C80ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // wLogic���߼���ƽ
    // wDir��1  ���� , 2: ����
    //*************************************************************************************************
    void D2C80HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void D2C80ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D2C80CheckDone(WORD wAxis);
    WORD D2C80CheckDoneAll(WORD wAxisNum);
    // ����ֵ��1������0����
    WORD D2C80CheckPreBuff(WORD wCardNo = CONTROL_CARD_NO);

    void D2C80EmgStop();

    void D2C80ConfigELMode(WORD wAxis, WORD wLogic);
    void D2C80ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D2C80CloseControlCard();

    WORD D2C80OpenControlCard();

    void D2C80SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D2C80WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2C80ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2C80IoStatus(WORD wAxis);

    BOOL D2C80CheckEmg(WORD wAxis);

    BOOL D2C80CheckAlarm(WORD wAxis);

    BOOL D2C80CheckMoveLimitPositive(WORD wAxis);

    BOOL D2C80CheckMoveLimitNegative(WORD wAxis);

    BOOL D2C80CheckHome(WORD wAxis);

    void D2C80EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);


    /*************************************************************************************************/
    /*                                         D2610����                                             */
    /*************************************************************************************************/
    // dAcc��dDec������2610Ϊ�Ӽ���ʱ�䣬����ȡһ����С����
    void D2610PositionMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D2610ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // ֧��ͬ��������2��岹(0~3Ϊ��һ�飬4~5Ϊ�ڶ���)������2C80����������ƣ����ǽӿڱ���
    void D2610InterpolationMove2Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧������2��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D2610InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧��ͬ��������3��岹(0~3Ϊ��һ�飬4~5Ϊ�ڶ���)������2C80����������ƣ����ǽӿڱ���
    void D2610InterpolationMove3Group(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧������3��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D2610InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ��֧��ǰ4��岹
    void D2610InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);
    // ֧������4��岹��d2610������֧�֣�������
    void D2610InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ��֧��ǰ6��岹
    void D2610InterpolationMove6(long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    BOOL  D2610CheckContiInterMoveDone(WORD wAxisNum);

    void D2610DecelStop(WORD wAxis, double dDec);
    void D2610DecelStopAll(WORD wAxisNum, double dDec);

    long D2610GetPosition(WORD wAxis);
    DWORD D2610SetPosition(WORD wAxis, long lPosition);

    double D2610GetCurrentSpeed(WORD wAxis);
    DWORD D2610ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // home_mode����ԭ��ķ�����
    // 1���������ԭ��
    // 2���������ԭ��
    // vel_mode��ѡ���ԭ����ٶȷ�ʽ��
    // 0�����ٻ�ԭ��
    // 1�����ٻ�ԭ�㣬��ԭ���źţ����ٺ�ֹͣ
    //*************************************************************************************************
    void D2610HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dMaxVel, double dAcc, double dDec);
    void D2610ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D2610CheckDone(WORD wAxis);
    WORD D2610CheckDoneAll(WORD wAxisNum);
    // ����ֵ��1������0����
    WORD D2610CheckPreBuff(WORD wAxis);

    void D2610EmgStop();

    void D2610ConfigELMode(WORD wAxis, WORD wLogic);
    void D2610ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D2610CloseControlCard();

    WORD D2610OpenControlCard();

    void D2610SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D2610WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2610ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D2610IoStatus(WORD wAxis);

    BOOL D2610CheckEmg(WORD wAxis);

    BOOL D2610CheckAlarm(WORD wAxis);

    BOOL D2610CheckMoveLimitPositive(WORD wAxis);

    BOOL D2610CheckMoveLimitNegative(WORD wAxis);

    BOOL D2610CheckHome(WORD wAxis);

    void D2610EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);



    /*************************************************************************************************/
    /*                                         D4400����                                             */
    /*************************************************************************************************/
    // dAcc��dDec������4400Ϊ�Ӽ���ʱ�䣬����ȡһ����С����
    void D4400PositionMove(WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D4400ContiMove(WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // ֧������2��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D4400InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ֧������3��岹����Щ������汾��֧�֣�������Щ���ƿ�ʹ�øú����ᵼ��6��岹��������ʱʧЧ
    void D4400InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    // ��֧��ǰ4��岹
    void D4400InterpolationMove4(long lDist1, long lDist2, long lDist3, long lDist4, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO);

    BOOL  D4400CheckContiInterMoveDone(WORD wAxisNum);

    void D4400DecelStop(WORD wAxis, double dDec);
    void D4400DecelStopAll(WORD wAxisNum, double dDec);

    long D4400GetPosition(WORD wAxis);
    DWORD D4400SetPosition(WORD wAxis, long lPosition);

    double D4400GetCurrentSpeed(WORD wAxis);
    DWORD D4400ChangeCurrentSpeed(WORD wAxis, double dVel);

    //*************************************************************************************************
    // home_mode����ԭ��ķ�����
    // 1���������ԭ��
    // 2���������ԭ��
    // vel_mode��ѡ���ԭ����ٶȷ�ʽ��
    // 0�����ٻ�ԭ��
    // 1�����ٻ�ԭ�㣬��ԭ���źţ����ٺ�ֹͣ
    //*************************************************************************************************
    void D4400HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dMaxVel, double dAcc, double dDec);
    void D4400ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D4400CheckDone(WORD wAxis);
    WORD D4400CheckDoneAll(WORD wAxisNum);
    // ����ֵ��1������0����
    WORD D4400CheckPreBuff(WORD wAxis);

    void D4400EmgStop();

    void D4400ConfigELMode(WORD wAxis, WORD wLogic);
    void D4400ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    void D4400CloseControlCard();

    WORD D4400OpenControlCard();

    void D4400SetPulseOutmode(WORD wAxis, WORD wOutmode);

    DWORD D4400WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D4400ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D4400IoStatus(WORD wAxis);

    BOOL D4400CheckEmg(WORD wAxis);

    BOOL D4400CheckAlarm(WORD wAxis);

    BOOL D4400CheckMoveLimitPositive(WORD wAxis);

    BOOL D4400CheckMoveLimitNegative(WORD wAxis);

    BOOL D4400CheckHome(WORD wAxis);

    void D4400EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);

    /*************************************************************************************************/
    /*                                         D5800����                                             */
    /*************************************************************************************************/
    // dAcc��dDec������2c80Ϊ���ٶȺͼ��ٶȣ�����ȡ��㣬��100000
    void D5800PosMove(WORD wCardNo, WORD wAxis, long lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    void D5800ContiMove(WORD wCardNo, WORD wAxis, WORD wDir,
        double dMinVel, double dMaxVel, double dAcc, double dDec);

    // ֧������2��岹
    //�ٶ� mm/min
    void D5800InterpolationMove2(WORD *pwAxis, double *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // ֧������3��岹
    void D5800InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // ֧������4��岹
    void D5800InterpolationMove4(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // 2c80֧������6������в岹
    void D5800InterpolationMove6(WORD *pwAxis, long *plDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    // D5800֧������1~12������в岹
    void D5800InterpolationMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    // D5800֧������2~6������л������嵱����ֱ�߲岹
    void D5800InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);

    void D5800StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    void D5800ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
        double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    BOOL  D5800ContiCheckRemainSpace(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMovePause(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMoveContinue(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    DWORD D5800ContiInterMoveDecelStop(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    void  D5800ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0); // wRatio��Ԥ���ٶȵı���
    DWORD D5800StopContiInterMove(WORD wCardNo = CONTROL_CARD_NO, WORD wCoordinate = 0);
    BOOL  D5800CheckContiInterMoveDone();

    void D5800DecelStop(WORD wCardNo, WORD wAxis, double dDec, WORD wStopMode);
    void D5800DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode);

    long D5800GetPosition(WORD wCardNo, WORD wAxis);
    DWORD D5800SetPosition(WORD wCardNo, WORD wAxis, long lPosition);

    double D5800GetCurrentSpeed(WORD wCardNo, WORD wAxis);
    DWORD D5800ChangeCurrentSpeed(WORD wCardNo, WORD wAxis, double dVel);

    short  D5800CheckContRunState(WORD CardNo, WORD wCoordinate = 0);

    //*************************************************************************************************
    // wLogic���߼���ƽ
    // wDir��1  ���� , 2: ����
    //*************************************************************************************************
    void D5800HomeMove(WORD wCardNo, WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec);
    void D5800ZeroMove(WORD wCardNo, WORD wAxis, double dVel, double dAcc, double dDec);

    WORD D5800CheckDone(WORD wCardNo, WORD wAxis);
    WORD D5800CheckDoneAll(WORD wAxisNum);
    // ����ֵ��1��buffer��ȫ��ʹ��, 0��bufferδʹ����
    WORD D5800CheckPreBuff(WORD wCardNo = CONTROL_CARD_NO);

    void D5800EmgStop(WORD wCardNo);

    void D5800ConfigELMode(WORD wCardNo, WORD wAxis, WORD wLogic);
    void D5800ConfigEmgMode(WORD wEnable, WORD wAxis, WORD wLogic, WORD wCardNo = CONTROL_CARD_NO);
    void D5800ConfigAlmMode(WORD wCardNo, WORD wAxis, WORD wLogic);

    void D5800CloseControlCard();

    WORD D5800OpenControlCard();

    void D5800SetPulseOutmode(WORD wCardNo, WORD wAxis, WORD wOutmode);

    DWORD D5800WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo = CONTROL_CARD_NO);
    int   D5800ReadOutbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D5800ReadInbit(WORD wBitNo, WORD wCardNo = CONTROL_CARD_NO);

    DWORD D5800IoStatus(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckEmg(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckAlarm(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckMoveLimitPositive(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckMoveLimitNegative(WORD wCardNo, WORD wAxis);

    BOOL D5800CheckHome(WORD wCardNo, WORD wAxis);

    void D5800EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo);

    void D5800SetPulseMode(WORD wCardNo = CONTROL_CARD_NO);
private:
    WORD m_wMoveControlCardType;
    int m_nAxisNoOffset;
    WORD m_wPosFileFlag;
    WORD m_wDefaultAcc;
    WORD m_wDefaultDec;

    typedef struct
    {
        int nPhysicalAxisNo;
        int HomeDir;
        int PulseDir;
        double PulseDisRatio;
        std::string AxisName;
    }T_AXIS_INFO;

    std::map <int, T_AXIS_INFO> m_mapCardAxis;

	double                      m_dUnitedMoveAcc;
	double                      m_dUnitedMoveDec;


};

#endif // !defined(AFX_MOVECTRLMODULE_H__6F89151F_1564_4AEC_B08C_2ECA5656300E__INCLUDED_)
