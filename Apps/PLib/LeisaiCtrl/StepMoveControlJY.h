// StepMoveControl.h: interface for the CStepMoveControl class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_STEPMOVECONTROL_H__EA9B29AA_D51F_4B01_932C_CB8F7EE74019__INCLUDED_)
#define AFX_STEPMOVECONTROL_H__EA9B29AA_D51F_4B01_932C_CB8F7EE74019__INCLUDED_

//#include "DriverAdapter.h"
#include "MoveCtrlModule.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000 

#include "LTDMC.h"
//#include <VECTOR>
// using namespace std;

#ifndef STEP_NUM_ACC
#define STEP_NUM_ACC 0
#endif

#ifndef ACC_DEC_RATIO
#define ACC_DEC_RATIO 0.1
#endif

#define DEC 0.01
#define ACC 0.01

typedef enum
{
    STEPMOVE_THREAD_STATUS_INIT,
		STEPMOVE_THREAD_STATUS_START,
        STEPMOVE_THREAD_STATUS_GOING,
        STEPMOVE_THREAD_STATUS_PAUSING,
        STEPMOVE_THREAD_STATUS_PAUSED,
        STEPMOVE_THREAD_STATUS_PAUSED_ONSTEP,
        STEPMOVE_THREAD_STATUS_PAUSED_BYINTERVAL,
        STEPMOVE_THREAD_STATUS_CONTINUING,
        STEPMOVE_THREAD_STATUS_ACCING,
        STEPMOVE_THREAD_STATUS_DECING,
        STEPMOVE_THREAD_STATUS_STOPPED,
        STEPMOVE_THREAD_STATUS_COMPLETE
}E_STEPMOVE_THREAD_STATUS;

typedef enum
{
    STEP_MOVE_DIRECTION_POSITIVE = 1,
        STEP_MOVE_DIRECTION_NEGATIVE = -1,
}E_STEP_MOVE_DIRECTION;

typedef enum
{
    STEP_MOVE_INTERPOLATION_TYPE_NULL, 
        STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE, // 从0开始按轴号顺序插补
        STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS // 任意几个轴的插补
}E_STEP_MOVE_INTERPOLATION_TYPE;

class CStepMoveControlJY  
{
public:
	CStepMoveControlJY(int nCoordinate = 0);
	virtual ~CStepMoveControlJY();

	void SetCallBackFun(void *pObj, void (*pFun)(void *));
    void SetCallBackFunOnPauseByInterval(void *pObj, void (*pFun)(void *));
	
    // 从轴0开始按轴号顺序插补
	void StepMove(std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum);
    void SetStartStopStepNum( WORD wStepNum );
	void SetStartStepNo(long lStartStepNo);

    void RelPositionMove(long alRelPulse[], double dTotalVel, int nAxisNum, long lStepNum);
    void AbsPositionMove(long alAbsPulse[], double dTotalVel, int nAxisNum, long lStepNum);
	
    // 支持任意几个轴的插补（2610驱动暂不支持4轴、5轴、6轴任意顺序插补，仅支持2、3轴的，并且有些驱动版本和控制卡不能用）
//    void StepMove(WORD wAxis[], std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum);//3-28
	void StepMove(WORD wAxis[], std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum, int nMoveDir = 1);
    void RelPositionMove(WORD wAxis[], long alRelPulse[], double dTotalVel, int nAxisNum, long lStepNum);
    void AbsPositionMove(WORD wAxis[], long alAbsPulse[], double dTotalVel, int nAxisNum, long lStepNum);

	// 连续插补暂停
	void ContiMovePause();
	// 连续插补恢复
	void ContiMoveRestore();

    void AdjustData( WORD wAxis, long lDataAdjust );
	void InsertData(std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum);
	
    void Pause(int nDecStepNum = STEP_NUM_ACC);
    void PauseOnStepNo(long lStepNo);
    void PauseByInterval( long lIntervalStepNum );
    void Continue(int nDecStepNum = STEP_NUM_ACC);
    void Accelerate(float fRatio = ACC_DEC_RATIO, int nAccStepNum = STEP_NUM_ACC);
    void Decelerate(float fRatio = ACC_DEC_RATIO, int nDecStepNum = STEP_NUM_ACC);
    BOOL GoBack();
    BOOL GoForward();
    void EmgStop();
	void StopStep();
	
    void AddSwing(WORD wAxisNo, std::vector <long> vlInnerSwingAmp, std::vector <long> vlOuterSwingAmp, 
        std::vector <double> vdInnerStayTime, std::vector <double> vdOuterStayTime, int nSwingPeriod);
    void AddSwing(WORD wAxisNo, long lInnerSwingAmp, long lOuterSwingAmp, 
        double dInnerStayTime, double dOuterStayTime, int nSwingPeriod);

	long GetCurrentMark();

    long GetCurrentStepNo();
    long GetPosition(WORD wAxisNo); // pulse
    double GetVelcecity(WORD wAxisNo); // pulse/s
	double GetVelRatio();
	
    BOOL IsComplete();
    BOOL IsPauseComplete();
    BOOL IsContinueComplete();
    BOOL IsAccelerateComplete();
    BOOL IsDecelerateComplete();

    void SetBuffNum(int nBuffNum);

	E_STEPMOVE_THREAD_STATUS GetStatus();

	BOOL AdjustpvlRelPulseIsNULL(){return NULL == m_pvlRelPulse;}

	//是否分段插入缓存
	BOOL m_bInsterBuffer;
    int m_nCoordinate;
	
private:
    static UINT ThreadStepMove(void *pParam);
    void Start();
    void StepMove();
    void DoPause();
    void DoPauseOnStep();
    void DoPauseByInterval();
    void DoContinue();
    void DoAcc();
    void DoDec();
    void ChangeVel();
    void CalcVelChangeTable(double dVelStart, double dVelEnd);
    void RecordCurrentStepNo();

	void RecordCurrentMark();

    void CompleteStepMove();
    void Minus(std::vector <long> &vlData);
    void InterpolationMove(long alRelPulse[], double dMinVel, double dMaxVel);
    BOOL CheckDoneAll();
    BOOL CheckPreBuff();
    BOOL CheckBeforeInterMove();
    int  GetAxisIdx(WORD wAxis);
    BOOL IsOnPauseStepNo();
    BOOL IsOnIntervalPauseStepNo();
    void Swing();
    void CalcStepTime();
	
private:
    CMoveCtrlModule m_cDrAdap;
    std::vector <long> *m_pvlRelPulse;
    std::vector <double> m_vdStepVel;
    std::vector <double> m_vdStepTime;
    WORD m_awAxis[6];
    int m_nAxisNum;
    long m_lStepNum;
    int m_nCardNo;
	
	long m_lCurrentMark;

    long m_lCurrentStepNo;
    long m_lPauseStepNo;
    long m_lIntervalStepNum;
    double m_dVelRatio;
    E_STEP_MOVE_DIRECTION m_eMoveDir;
    E_STEP_MOVE_INTERPOLATION_TYPE m_eInterpoType;
	
    double m_dStepVelChangeTable[10];
    int m_nVelChangeStepNum;
	
    E_STEPMOVE_THREAD_STATUS m_eThreadStatus;
    BOOL m_bThreadOnOff;
    
	
	
    FILE *m_pFileCurrentStepNo;
	
	FILE *m_pFileCurrentMark;
	
    void (*m_pFun)(void *);
    void *m_pCallObj;

    void (*m_pFunOnPauseByInterval)(void *);
    void *m_pCallObjOnPauseByInterval;
    
    BOOL m_bIfAddSwing;

    std::vector <long> m_vlInnerSwingAmp;
    std::vector <long> m_vlOuterSwingAmp;
    std::vector <double> m_vdInnerStayTime;
    std::vector <double> m_vdOuterStayTime;
    int m_nSwingPeriod;
    WORD m_wSwingAxis;
    long m_lSwingZero;
    long m_lSwingPeak;

    WORD m_wMoveControlCardType;
 

};  

#endif // !defined(AFX_STEPMOVECONTROL_H__EA9B29AA_D51F_4B01_932C_CB8F7EE74019__INCLUDED_)
