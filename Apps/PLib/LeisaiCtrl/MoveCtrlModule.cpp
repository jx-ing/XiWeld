// MoveCtrlModule.cpp: implementation of the CMoveCtrlModule class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "MoveCtrlModule.h"
#include "LTDMC.h"
//#include "AxisParaManage.h"
//#include "DMC2C80.h"
//#include "dmc4400.h"
//#include "DMC2610.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

CMoveCtrlModule::CMoveCtrlModule()
{
    m_wMoveControlCardType = MOVE_CONTROL_CARD_5800;
    //ConfigAlmMode();
}

CMoveCtrlModule::~CMoveCtrlModule()
{
	
}

void CMoveCtrlModule::PosMoveDis
(
	WORD wAxis, double dDis, WORD wPositionMode, double dMinVel, 
	double dMaxVel, double dAcc, double dDec, BOOL bIfCheckDone 
)
{
//    PosMove(wAxis, dDis / m_mapPulseToDis[wAxis], wPositionMode, dMinVel/ m_mapPulseToDis[wAxis]/60, dMaxVel/ m_mapPulseToDis[wAxis]/60, dAcc, dDec);
    if (TRUE == bIfCheckDone)
    {
        while(FALSE == CheckDone(wAxis))
        {
            DoEvent();
            Sleep(5);
        }
    }
}

void CMoveCtrlModule::ContiMoveDis(WORD wAxis, WORD wDir,
    double dMinVel, double dMaxVel, double dAcc, double dDec)
{
  //  ContiMove(wAxis, wDir,dMinVel/m_mapPulseToDis[wAxis] / 60, dMaxVel/m_mapPulseToDis[wAxis] / 60, dAcc, dDec);
}



DWORD CMoveCtrlModule::SetPositionDis( WORD wAxis, double dPos )
{
    return SetPosition(wAxis, dPos/* / m_mapPulseToDis[wAxis]*/);
}


double CMoveCtrlModule::GetPositionDis( WORD wAxis )
{
   return GetPosition(wAxis) /** m_mapPulseToDis[wAxis]*/;
}

void CMoveCtrlModule::GetMachinePositionDis(double &dCurPosX, double &dCurPosY)
{
// 	dCurPosX = GetPosition(AXIS_X) * m_mapPulseToDis[AXIS_X];
// 	dCurPosY = GetPosition(AXIS_Y) * m_mapPulseToDis[AXIS_Y];
}

WORD CMoveCtrlModule::CheckDoneDis(WORD wAxis)
{
	Sleep(200);
	while (false == CheckDone(wAxis))
	{
		Sleep(10);
		DoEvent();
	}
	return CheckDone(wAxis);
}

WORD CMoveCtrlModule::CheckDoneAllDis(WORD wAxisNum)
{
	Sleep(200);
	while (false == CheckDoneAll(wAxisNum))
	{
		Sleep(10);
		DoEvent();
	}
	return CheckDoneAll(wAxisNum);
}

void CMoveCtrlModule::EmgStopDis() // 所有轴卡急停
{
	EmgStop();
	dmc_conti_stop_list(0, 0, 0); // 连续插补停止
}
DWORD CMoveCtrlModule::ReadInbitDis(WORD wBitNo)
{
	return ReadInbit(wBitNo, m_wMoveControlCardType);
}

// 多轴软件联动，距离单位：mm(°)，速度单位mm(°)/min，
// 加减速度单位mm(°)/s*min(1秒钟加速到多少mm(°)/min)
void CMoveCtrlModule::UnitedMoveDis
(
	WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode, 
    double dMinVel, double dMaxVel, double dAcc, double dDec, 
	WORD wCardNo
)
{
	UnitedMove(wAxisNum, pwAxis, dDist, wPositionMode,
		dMinVel, dMaxVel, dAcc, dDec, wCardNo);
}

void CMoveCtrlModule::PosMove(WORD wAxis, long lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec)
{
    WriteLog("位置运动：轴号：%d，位置：%ld，位移模式：%d，起始速度：%f，运行速度：%f，加速度：%f, 减速度：%f。",
        wAxis, lDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec);
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800PosMove(0, wAxis, lDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec);
    }
}

void CMoveCtrlModule::UnitedMove(WORD wAxisNum, WORD *pwAxis, double *dDist, WORD wPositionMode, double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo /*= CONTROL_CARD_NO*/, WORD wCoordinate)
{
    WriteLog("UnitedMove: %d axis UnitedMove", wAxisNum);
    switch (wAxisNum)
    {
    case 1:
        break;
    case 2:
        D5800InterpolationMove2(pwAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
        break;
    case 3:
        D5800InterpolationMoveUnit(3, pwAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
        break;
    case 4:
        D5800InterpolationMoveUnit(4, pwAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
        break;
    case 5:
        D5800InterpolationMoveUnit(5, pwAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
        break;
    case 6:
        D5800InterpolationMoveUnit(6, pwAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
        break;
    default:
        break;
    }
}

void CMoveCtrlModule::ContiMove(WORD wAxis, WORD wDir,
    double dMinVel, double dMaxVel, double dAcc, double dDec)
{
    WriteLog("连续运动：轴号：%d，方向：%ld，起始速度：%f，运行速度：%f，加速度：%f, 减速度：%f。",
        wAxis, wDir, dMinVel, dMaxVel, dAcc, dDec);
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ContiMove(0, wAxis, wDir, dMinVel, dMaxVel, dAcc, dDec);
    }
}

void CMoveCtrlModule::InterpolationMove2(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        //D5800InterpolationMove2(pwAxis, plDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo);
    }
}

void CMoveCtrlModule::InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    WriteLog("3轴插补运动：0轴位移：%ld，1轴位移：%ld，2轴位移：%ld，起始速度：%f，运行速度：%f，加速度：%f, 减速度：%f。",
        plDist[0], plDist[1], plDist[2], dMinVel, dMaxVel, dAcc, dDec);
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800InterpolationMove3(pwAxis, plDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
    }

}

void CMoveCtrlModule::InterpolationMove4(WORD awAxis[], long alDist[], WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800InterpolationMove4(awAxis, alDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
    }

}

void CMoveCtrlModule::InterpolationMove6(WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {

    }
}

void CMoveCtrlModule::InterpolationMoveN(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800InterpolationMove(wAxisNum, awAxis, lDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
    }
}

void CMoveCtrlModule::StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800StartContiInterMove(wAxisNum, pwAxis, wCardNo, wCoordinate);
    }
}

void CMoveCtrlModule::ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ContiInterMove(wAxisNum, awAxis, lDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
    }
}

BOOL CMoveCtrlModule::ContiCheckRemainSpace(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    BOOL bRemainSpace = 0;
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //bRemainSpace = D2C80ContiCheckRemainSpace(wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        bRemainSpace = D5800ContiCheckRemainSpace(wCardNo, wCoordinate);
        //WriteLog("ContiCheckRemainSpace Flage:%d", bRemainSpace);
    }
    return bRemainSpace;
}

DWORD CMoveCtrlModule::ContiInterMovePause(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    DWORD dwRet = 0;
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //dwRet = D2C80ContiInterMovePause(wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        dwRet = D5800ContiInterMovePause(wCardNo, wCoordinate);
    }
    return dwRet;
}

DWORD CMoveCtrlModule::ContiInterMoveContinue(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    DWORD dwRet = 0;
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //dwRet = D2C80ContiInterMoveContinue(wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        dwRet = D5800ContiInterMoveContinue(wCardNo, wCoordinate);
    }
    return dwRet;
}

DWORD CMoveCtrlModule::ContiInterMoveDecelStop(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    DWORD dwRet = 0;
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //dwRet = D2C80ContiInterMoveDecelStop(wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        dwRet = D5800ContiInterMoveDecelStop(wCardNo, wCoordinate);
    }
    return dwRet;
}

void CMoveCtrlModule::ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //D2C80ContiInterMoveChangeSpeed(wRatio, wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ContiInterMoveChangeSpeed(wRatio, wCardNo, wCoordinate);
    }
}

DWORD CMoveCtrlModule::StopContiInterMove(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    DWORD dwRet = 0;
    if (MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        //dwRet = D2C80StopContiInterMove(wCardNo);
    }
    else if (MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
    {
    }
    else if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        dwRet = D5800StopContiInterMove(wCardNo, wCoordinate);
    }
    return dwRet;
}

BOOL CMoveCtrlModule::CheckContiInterMoveDone(WORD wAxisNum)
{
    BOOL bIsDone = TRUE;
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        bIsDone = D5800CheckContiInterMoveDone();
    }
    return bIsDone;
}

short CMoveCtrlModule::CheckContiInterMoveState(WORD CardNo /*= CONTROL_CARD_NO*/, WORD wCoordinate)
{
    short sotRunState = 0;
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        sotRunState = D5800CheckContRunState(CardNo, wCoordinate);
    }
    return sotRunState;
}

void CMoveCtrlModule::DecelStop(WORD wAxis, double dDec, WORD wStopMode)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800DecelStop(0, wAxis, dDec, wStopMode);
    }
}

void CMoveCtrlModule::DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800DecelStopAll(wAxisNum, dDec, wStopMode);
    }

}

WORD CMoveCtrlModule::GetCardAxisNum(WORD wCardNo)
{
    DWORD dwAxisNum = 0;

    dmc_get_total_axes(wCardNo, &dwAxisNum);
    return dwAxisNum;
}

long CMoveCtrlModule::GetPosition(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800GetPosition(0, wAxis);
    }

    return 0;
}

double CMoveCtrlModule::GetCurrentSpeed(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800GetCurrentSpeed(0, wAxis);
    }

    return 0;
}

DWORD CMoveCtrlModule::ChangeCurrentSpeed(WORD wAxis, double dVel)
{

    return 0;
}

void CMoveCtrlModule::HomeMove(WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800HomeMove(0, wAxis, wLogic, wDir, dVel, dAcc, dDec);
    }
}

void CMoveCtrlModule::ZeroMove(WORD wAxis, double dVel, double dAcc, double dDec)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ZeroMove(0, wAxis, dVel, dAcc, dDec);
    }
}

WORD CMoveCtrlModule::CheckDone(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckDone(0, wAxis);
    }

    return 0;
}

WORD CMoveCtrlModule::CheckDoneAll(WORD wAxisNum)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckDoneAll(wAxisNum);
    }

    return 0;
}

void CMoveCtrlModule::EmgStop()
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800EmgStop(0);
    }
}

void CMoveCtrlModule::EmgStop(WORD wMoveControlCardType)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800EmgStop(wMoveControlCardType);
    }
}


void CMoveCtrlModule::SetBuffNum(int nBuffNum)
{
    m_nBuffNum = nBuffNum;
}

int CMoveCtrlModule::GetBuffNum()
{
    return m_nBuffNum;
}


WORD CMoveCtrlModule::CheckPreBuff(WORD wPara, WORD wMoveControlCardType)
{
if (MOVE_CONTROL_CARD_5800 == wMoveControlCardType)
    {
        return D5800CheckPreBuff(wPara);
    }
    else
    {
        return 1;
    }
}

void CMoveCtrlModule::ConfigELMode(WORD wAxis, WORD wLogic)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ConfigELMode(0, wAxis, wLogic);
    }
}

void CMoveCtrlModule::ConfigEmgMode(WORD wEnable, WORD wLogic, WORD wCardNo, WORD wAxisNum)
{
 if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        for (WORD wAxis = 0; wAxis < wAxisNum; ++wAxis)
        {
            D5800ConfigEmgMode(wCardNo, wEnable, wAxis, wLogic);
        }
    }
}

void CMoveCtrlModule::CloseControlCard()
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800CloseControlCard();
    }
}

void CMoveCtrlModule::CloseControlCard(WORD wMoveControlCardType)
{
if (MOVE_CONTROL_CARD_5800 == wMoveControlCardType)
    {
        D5800CloseControlCard();
    }

}

WORD CMoveCtrlModule::OpenControlCard()
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800OpenControlCard();
    }

    return 0;
}

WORD CMoveCtrlModule::OpenControlCard(WORD wMoveControlCardType)
{
    // 	if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    //     {
    //         return D5800OpenControlCard(wMoveControlCardType);
    //     }
    return 0;
}

void CMoveCtrlModule::SetPulseOutmode(WORD wAxis, WORD wOutmode)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800SetPulseOutmode(0, wAxis, wOutmode);
    }
}

DWORD CMoveCtrlModule::SetPosition(WORD wAxis, long lPosition)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800SetPosition(0, wAxis, lPosition);
    }

    return 0;
}

DWORD CMoveCtrlModule::WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardType, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
if (MOVE_CONTROL_CARD_5800 == wCardType)
    {
        return D5800WriteOutbit(wBitNo, wOnOff, wCardNo);
    }

    return 0;
}

DWORD CMoveCtrlModule::ReadInbit(WORD wBitNo, WORD wCardType, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
if (MOVE_CONTROL_CARD_5800 == wCardType)
    {
        return D5800ReadInbit(wBitNo, wCardNo);
    }

    return 0;
}

DWORD CMoveCtrlModule::IoStatus(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800IoStatus(0, wAxis);
    }

    return 0;
}

BOOL CMoveCtrlModule::CheckEmg(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckEmg(0, wAxis);
    }

    return 0;
}

BOOL CMoveCtrlModule::CheckAlarm(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckAlarm(0, wAxis);
    }

    return 0;
}

BOOL CMoveCtrlModule::CheckMoveLimitPositive(WORD wAxis)
{
 if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckMoveLimitPositive(0, wAxis);
    }

    return 0;
}

BOOL CMoveCtrlModule::CheckMoveLimitNegative(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckMoveLimitNegative(0, wAxis);
    }

    return 0;
}

BOOL CMoveCtrlModule::CheckHome(WORD wAxis)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800CheckHome(0, wAxis);
    }

    return 0;
}

void CMoveCtrlModule::EnableServo(WORD wLogic, WORD wCardNo /*= CONTROL_CARD_NO */)
{
    for (int nIndex = 0; nIndex < 6; nIndex++)
    {
        EnableServo(nIndex, wLogic, 1, wCardNo);
    }
}

void CMoveCtrlModule::EnableServo(WORD wAxis, WORD wLogic, int nServoEnableIo /*= 1*/, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800EnableServo(wAxis, wLogic, wCardNo);
    }
}


//////////////////////////////// D5800驱动 //////////////////////////////////////////
void CMoveCtrlModule::D5800PosMove(WORD wCardNo, WORD wAxis, long lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec)
{
    dmc_set_profile_unit(wCardNo, wAxis, dMinVel, dMaxVel, dAcc, dDec, 0);
    dmc_set_s_profile(wCardNo, wAxis, 0, SECTION_S_TIME);
    dmc_pmove_unit(wCardNo, wAxis, lDist, wPositionMode);
}

void CMoveCtrlModule::D5800ContiMove(WORD wCardNo, WORD wAxis, WORD wDir,
    double dMinVel, double dMaxVel, double dAcc, double dDec)
{
    dmc_set_profile(wCardNo, wAxis, dMinVel, dMaxVel, dAcc, dDec, 0);
    dmc_set_s_profile(wCardNo, wAxis, 0, SECTION_S_TIME);
    dmc_vmove(wCardNo, wAxis, wDir);
}

void CMoveCtrlModule::D5800InterpolationMove2(WORD *pwAxis, double *pdDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    int nAxisNum = 2;
    dmc_set_vector_profile_unit(wCardNo, wCoordinate, dMinVel / 60, dMaxVel / 60, dAcc, dDec, dMinVel / 60);
    dmc_set_vector_s_profile(wCardNo, wCoordinate, 0, SECTION_S_TIME);
    dmc_line_unit(wCardNo, wCoordinate, nAxisNum, pwAxis, pdDist, wPositionMode);
}

void CMoveCtrlModule::D5800InterpolationMove3(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_multicoor(wCardNo, wCoordinate, dMinVel, dMaxVel, dAcc, dDec, dMaxVel);
    dmc_line_multicoor(wCardNo, wCoordinate, 3, pwAxis, plDist, wPositionMode);
}

void CMoveCtrlModule::D5800InterpolationMove4(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_multicoor(wCardNo, wCoordinate, dMinVel, dMaxVel, dAcc, dDec, dMaxVel);
    dmc_line_multicoor(wCardNo, wCoordinate, 4, pwAxis, plDist, wPositionMode);
}

void CMoveCtrlModule::D5800InterpolationMove6(WORD *pwAxis, long *plDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_multicoor(wCardNo, wCoordinate, dMinVel, dMaxVel, dAcc, dDec, dMaxVel);
    dmc_line_multicoor(wCardNo, wCoordinate, 6, pwAxis, plDist, wPositionMode);
}

void CMoveCtrlModule::D5800InterpolationMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_multicoor(wCardNo, wCoordinate, dMinVel, dMaxVel, dAcc, dDec, dMaxVel);
    dmc_line_multicoor(wCardNo, wCoordinate, wAxisNum, awAxis, lDist, wPositionMode);
}

void CMoveCtrlModule::D5800InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_unit(wCardNo, wCoordinate, dMinVel / 60, dMaxVel / 60, dAcc, dDec, dMinVel / 60);
    dmc_set_vector_s_profile(wCardNo, wCoordinate, 0, SECTION_S_TIME);
    dmc_line_unit(wCardNo, wCoordinate, wAxisNum, awAxis, dDist, wPositionMode);
}

void CMoveCtrlModule::D5800StartContiInterMove(WORD wAxisNum, WORD *pwAxis, WORD wCardNo, WORD wCoordinate)
{
    dmc_conti_open_list(wCardNo, wCoordinate, wAxisNum, pwAxis);
    dmc_conti_start_list(wCardNo, wCoordinate);
}

void CMoveCtrlModule::D5800ContiInterMove(WORD wAxisNum, WORD awAxis[], long *lDist, WORD wPositionMode,
    double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo, WORD wCoordinate)
{
    dmc_set_vector_profile_unit(wCardNo, wCoordinate, 0 /*/ 60*/, dMaxVel /*/ 60*/, 0.0/*dAcc*/, 0.0/*dDec*/, 5000 /*/ 60*/); // 大车加减速时间
    double adDist[8] = { 0 };
    for (int nAxisNo = 0; nAxisNo < wAxisNum; nAxisNo++)
    {
        adDist[nAxisNo] = lDist[nAxisNo];
    }
    dmc_conti_line_unit(wCardNo, wCoordinate, wAxisNum, awAxis, adDist, wPositionMode, 0); // mark 标号，任意指定，0 表示自动编号	
	
}

BOOL CMoveCtrlModule::D5800ContiCheckRemainSpace(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
	int nUsingBuffNum = 0;
	nUsingBuffNum = DMC5800_CONTIMOV_INSTRUCT_NUM - dmc_conti_remain_space(wCardNo, wCoordinate);
    //WriteLog("D5800ContiCheckRemainSpace is: %d, Set BuffNum is %d", nUsingBuffNum, m_nBuffNum);	
    if (m_nBuffNum > nUsingBuffNum)
    {
        //指令buff未使用完
        return false;

    }
    else
    {
        return true;
    }

}

DWORD CMoveCtrlModule::D5800ContiInterMovePause(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    return dmc_conti_pause_list(wCardNo, wCoordinate);
}

DWORD CMoveCtrlModule::D5800ContiInterMoveContinue(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    return dmc_conti_start_list(wCardNo, wCoordinate);
}

DWORD CMoveCtrlModule::D5800ContiInterMoveDecelStop(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    return dmc_conti_stop_list(wCardNo, wCoordinate, 0);
}

void CMoveCtrlModule::D5800ContiInterMoveChangeSpeed(WORD wRatio, WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    dmc_conti_set_override(wCardNo, wCoordinate, wRatio);
}

DWORD CMoveCtrlModule::D5800StopContiInterMove(WORD wCardNo /*= CARDNO*/, WORD wCoordinate)
{
    dmc_conti_stop_list(0, wCoordinate, 0);
	dmc_conti_reset_list(0, wCoordinate);
    return dmc_conti_close_list(wCardNo, wCoordinate);
}

BOOL CMoveCtrlModule::D5800CheckContiInterMoveDone()
{
    BOOL bIsDone = ContiCheckRemainSpace();
    return bIsDone;
}

void CMoveCtrlModule::D5800DecelStop(WORD wCardNo, WORD wAxis, double dDec, WORD wStopMode)
{
    dmc_set_dec_stop_time(wCardNo, wAxis, dDec);
    dmc_stop(wCardNo, wAxis, wStopMode);
}

void CMoveCtrlModule::D5800DecelStopAll(WORD wAxisNum, double dDec, WORD wStopMode)
{
    for (WORD wAxis = 0; wAxis < wAxisNum; wAxis++)
    {
        D5800DecelStop(0, wAxis, dDec, wStopMode);
    }
}

long CMoveCtrlModule::D5800GetPosition(WORD wCardNo, WORD wAxis)
{
    return dmc_get_position(wCardNo, wAxis);
}

double CMoveCtrlModule::D5800GetCurrentSpeed(WORD wCardNo, WORD wAxis)
{
    return dmc_read_current_speed(wCardNo, wAxis);
}

DWORD CMoveCtrlModule::D5800ChangeCurrentSpeed(WORD wCardNo, WORD wAxis, double dVel)
{
    return dmc_change_speed(wCardNo, wAxis, dVel, 0.1);
}

void CMoveCtrlModule::D5800HomeMove(WORD wCardNo, WORD wAxis, WORD wLogic, WORD wDir, double dVel, double dAcc, double dDec)
{
    T_AXIS_INFO tAxisNo;
    if (m_mapCardAxis.count(wAxis) != 0)
    {
        tAxisNo = m_mapCardAxis.find(wAxis)->second;
        if (wAxis != tAxisNo.nPhysicalAxisNo)
        {
            WriteLog("D5800HomeMove Error,wAxis != nPhysicalAxisNo");
            return;
        }
    }
    double velMode = 0;// 1 高速回零
    dmc_set_home_pin_logic(wCardNo, wAxis, wLogic, 0);
    dmc_set_homemode(wCardNo, wAxis, tAxisNo.HomeDir, velMode, 0, 0);
    dmc_set_profile(wCardNo, wAxis, dVel / 5, dVel, dAcc, dDec, 0);
    dmc_home_move(wCardNo, wAxis);
}

void CMoveCtrlModule::D5800ZeroMove(WORD wCardNo, WORD wAxis, double dVel, double dAcc, double dDec)
{
    D5800PosMove(wCardNo, wAxis, 0, 0, 0, dVel, dAcc, dDec);
}

WORD CMoveCtrlModule::D5800CheckDone(WORD wCardNo, WORD wAxis)
{
    return dmc_check_done(wCardNo, wAxis);
}

WORD CMoveCtrlModule::D5800CheckDoneAll(WORD wAxisNum)
{
    WORD wRet = 1;
    for (WORD wAxis = 0; wAxis < wAxisNum; wAxis++)
    {
        wRet = wRet && D5800CheckDone(0, wAxis);
    }

    return wRet;
}

void CMoveCtrlModule::D5800EmgStop(WORD wCardNo)
{
    dmc_emg_stop(wCardNo);
}

WORD CMoveCtrlModule::D5800CheckPreBuff(WORD wCardNo)
{
    //return (ContiCheckRemainSpace(wCardNo) <= 0);
    return ContiCheckRemainSpace(wCardNo);
}

void CMoveCtrlModule::D5800ConfigELMode(WORD wCardNo, WORD wAxis, WORD wLogic)
{
    dmc_set_el_mode(wCardNo, wAxis, 1, wLogic, 0);
}

void CMoveCtrlModule::D5800ConfigEmgMode(WORD wEnable, WORD wAxis, WORD wLogic, WORD wCardNo)
{
    dmc_set_emg_mode(wCardNo, wAxis, wEnable, wLogic);
}

void CMoveCtrlModule::D5800ConfigAlmMode(WORD wCardNo, WORD wAxis, WORD wLogic)
{
    dmc_set_alm_mode(wCardNo, wAxis, 0, wLogic, 0);
}

void CMoveCtrlModule::D5800CloseControlCard()
{
    dmc_board_close();
}

// void CMoveCtrlModule::D5800CloseControlCard(WORD wMoveControlCardType)
// {
//     dmc_board_close_onecard(wMoveControlCardType);
// }

WORD CMoveCtrlModule::D5800OpenControlCard()
{
    return dmc_board_init();
}

// WORD CMoveCtrlModule::D5800OpenControlCard(WORD wMoveControlCardType)
// {
//     return dmc_board_init_onecard(wMoveControlCardType);
// }

void CMoveCtrlModule::D5800SetPulseOutmode(WORD wCardNo, WORD wAxis, WORD wOutmode)
{
    dmc_set_pulse_outmode(wCardNo, wAxis, wOutmode);
}

DWORD CMoveCtrlModule::D5800SetPosition(WORD wCardNo, WORD wAxis, long lPosition)
{
    return dmc_set_position(wCardNo, wAxis, lPosition);
}

DWORD CMoveCtrlModule::D5800WriteOutbit(WORD wBitNo, WORD wOnOff, WORD wCardNo)
{
    return dmc_write_outbit(wCardNo, wBitNo, wOnOff);
}

int CMoveCtrlModule::D5800ReadOutbit(WORD wBitNo, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    return dmc_read_outbit(wCardNo, wBitNo);
}

DWORD CMoveCtrlModule::D5800ReadInbit(WORD wBitNo, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    return dmc_read_inbit(wCardNo, wBitNo);
}

DWORD CMoveCtrlModule::D5800IoStatus(WORD wCardNo, WORD wAxis)
{
    return dmc_axis_io_status(wCardNo, wAxis);
}

BOOL CMoveCtrlModule::D5800CheckEmg(WORD wCardNo, WORD wAxis)
{
    DWORD dwIoStatus = D5800IoStatus(wCardNo, wAxis);
    if (dwIoStatus & 0x8)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CMoveCtrlModule::D5800CheckAlarm(WORD wCardNo, WORD wAxis)
{
    DWORD dwIoStatus = D5800IoStatus(wCardNo, wAxis);
    if (dwIoStatus & 0x1)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CMoveCtrlModule::D5800CheckMoveLimitPositive(WORD wCardNo, WORD wAxis)
{
    DWORD dwIoStatus = D5800IoStatus(wCardNo, wAxis);
    if (dwIoStatus & 0x2)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CMoveCtrlModule::D5800CheckMoveLimitNegative(WORD wCardNo, WORD wAxis)
{
    DWORD dwIoStatus = D5800IoStatus(wCardNo, wAxis);
    if (dwIoStatus & 0x4)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CMoveCtrlModule::D5800CheckHome(WORD wCardNo, WORD wAxis)
{
    DWORD dwIoStatus = D5800IoStatus(wCardNo, wAxis);
    if (dwIoStatus & 0x10)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void CMoveCtrlModule::D5800EnableServo(WORD wAxis, WORD wLogic, WORD wCardNo /*= CARDNO*/)
{
    dmc_write_sevon_pin(wCardNo, wAxis, wLogic);
}

void CMoveCtrlModule::SetDefaultAcc(double dAcc)
{
    m_wDefaultAcc = dAcc;
}

void CMoveCtrlModule::SetDefaultDec(double dDec)
{
    m_wDefaultDec = dDec;
}

double CMoveCtrlModule::GetDefaultAcc()
{
    return m_wDefaultAcc;
}

double CMoveCtrlModule::GetDefaultDec()
{
    return m_wDefaultDec;
}

void CMoveCtrlModule::ConfigAlmMode()
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        for (int nAxisNo = 0; nAxisNo < D5800_MAX_AXIS_NO; ++nAxisNo)
        {
            D5800ConfigAlmMode(CONTROL_CARD_NO, nAxisNo, D5800_LOGIC);
        }
    }
}

void CMoveCtrlModule::ConfigAlmMode(WORD wAxis, WORD wLogic, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800ConfigAlmMode(wCardNo, wAxis, wLogic);
    }
}

void CMoveCtrlModule::LoadPosition()
{

}

void CMoveCtrlModule::SavePosition()
{

}

DWORD CMoveCtrlModule::ReadOutbit(WORD wBitNo, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        return D5800ReadOutbit(wBitNo, wCardNo);
    }

    return 0;
}

DWORD CMoveCtrlModule::ReadOutbit(WORD wBitNo, WORD wCardType, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    if (MOVE_CONTROL_CARD_5800 == wCardType)
    {
        return D5800ReadOutbit(wBitNo, wCardNo);
    }

    return 0;
}

void CMoveCtrlModule::InterpolationMoveUnit(WORD wAxisNum, WORD awAxis[], double *dDist, WORD wPositionMode, double dMinVel, double dMaxVel, double dAcc, double dDec, WORD wCardNo /*= CONTROL_CARD_NO*/, WORD wCoordinate)
{
    if (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        D5800InterpolationMoveUnit(wAxisNum, awAxis, dDist, wPositionMode, dMinVel, dMaxVel, dAcc, dDec, wCardNo, wCoordinate);
    }
}

WORD CMoveCtrlModule::CheckDoneMultiCoor(WORD wCardNo /*= CONTROL_CARD_NO*/, WORD wCrd /*= D5800_CRD*/)
{
    return dmc_check_done_multicoor(wCardNo, wCrd);
}

void CMoveCtrlModule::MultiCoorStop(WORD wStopMode, WORD wCrd /*= D5800_CRD*/, WORD wCardNo /*= CONTROL_CARD_NO*/)
{
    dmc_stop_multicoor(wCardNo, wCrd, wStopMode);
}

short CMoveCtrlModule::D5800CheckContRunState(WORD CardNo, WORD wCoordinate)
{
    return dmc_conti_get_run_state(CardNo, wCoordinate);
}

void CMoveCtrlModule::InitAxisPara()
{
    COPini cOpini;
    m_mapCardAxis.swap(std::map <int, T_AXIS_INFO>());
    cOpini.SetFileName(".\\Data\\AxisPara.ini");


    std::string sIndexArray[6] = { "0", "1", "2", "3", "4", "5" };
    std::string sKeyWord = "";

    std::vector<std::string> vsAxisName;

    CString sAxisName;
    int nAxisNo;
    int nHomeDir;
    int nPulseDir;
    double dPulseDisRatio;
    for (int nIdx = 0; nIdx < 6; nIdx++)
    {
        T_AXIS_INFO tAxisInfo;

        cOpini.SetSectionName("AxisName");
        sKeyWord = "Axis" + sIndexArray[nIdx];
        cOpini.ReadString(sKeyWord.c_str(), sAxisName);

        cOpini.SetSectionName("PhysicalAxisNo");
        sKeyWord = "Axis" + sAxisName;
        cOpini.ReadString(sKeyWord.c_str(), &nAxisNo);
        tAxisInfo.AxisName = sKeyWord;
        tAxisInfo.nPhysicalAxisNo = nAxisNo;

        cOpini.SetSectionName("HomeDir");
        cOpini.ReadString(sKeyWord.c_str(), &nHomeDir);
        tAxisInfo.HomeDir = nHomeDir;

        cOpini.SetSectionName("PulseDir");
        sKeyWord = "PulseDirAxis" + sAxisName;
        cOpini.ReadString(sKeyWord.c_str(), &nPulseDir);
        tAxisInfo.PulseDir = nPulseDir;

        cOpini.SetSectionName("PulseDisRatio");
        sKeyWord = "KEY_P2D_" + sAxisName;
        cOpini.ReadString(sKeyWord.c_str(), &dPulseDisRatio);
        tAxisInfo.PulseDisRatio = dPulseDisRatio;

        std::pair<int, T_AXIS_INFO> mTempPair = std::make_pair(nAxisNo, tAxisInfo);
        m_mapCardAxis.insert(mTempPair);
    }

    SetPulseMode(CONTROL_CARD_NO);
}

void CMoveCtrlModule::SetPulseMode(WORD wCardNo)
{
    D5800SetPulseMode(wCardNo);
}

void CMoveCtrlModule::D5800SetPulseMode(WORD wCardNo)
{
    for (int nIndex = 0; nIndex < 6; nIndex++)
    {
        dmc_set_pulse_outmode(wCardNo, nIndex, m_mapCardAxis.at(nIndex).PulseDir);
    }
}