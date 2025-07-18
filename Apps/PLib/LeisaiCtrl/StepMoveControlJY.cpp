// StepMoveControl.cpp: implementation of the CStepMoveControl class.
//
//////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "math.h"
#include <FSTREAM>
#include "StepMoveControlJY.h"
//#include "DMC2610.h"
#include "Apps/PLib/BasicFunc/Log.h"
#include ".\Apps\PLib\BasicFunc\Const.h"

#ifdef _DEBUG
#undef THIS_FILE
static char THIS_FILE[]=__FILE__;
#define new DEBUG_NEW
#endif

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

#ifndef SQUARE
#define SQUARE(x) ((x)*(x))
#endif // SQUARE


static CLog s_cStepMoveLog("Monitor\\StepMove\\", "StepLog_");

CStepMoveControlJY::CStepMoveControlJY(int nCoordinate)
    :m_nCoordinate(nCoordinate)
{
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_INIT;
    m_eMoveDir = STEP_MOVE_DIRECTION_POSITIVE;
    m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_NULL;
    m_bThreadOnOff = 0;
    m_pFun = NULL;
    m_pFunOnPauseByInterval = NULL;
    m_nCardNo = CONTROL_CARD_NO;
    m_lPauseStepNo = 0xffffff;
    m_lIntervalStepNum = 0xffffff;
	m_pvlRelPulse = NULL;
    m_nVelChangeStepNum = STEP_NUM_ACC;
    m_bIfAddSwing = FALSE;
	m_wMoveControlCardType = MOVE_CONTROL_CARD_5800;

	m_lCurrentStepNo = 0;
	m_lCurrentMark = 0;
	m_bInsterBuffer = FALSE;
    m_cDrAdap = CMoveCtrlModule();//传输坐标系
}

CStepMoveControlJY::~CStepMoveControlJY()
{
    if(m_bThreadOnOff != 0)
    {
        m_bThreadOnOff = 0;
        while(STEPMOVE_THREAD_STATUS_COMPLETE != m_eThreadStatus) 
        {
            Sleep(5);
        }
    }

    if (NULL != m_pvlRelPulse)
    {
        delete [] m_pvlRelPulse;
        m_pvlRelPulse = NULL;
    }
}

void CStepMoveControlJY::StepMove( std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum )
{
	m_vdStepVel = vdStepVelecity;
    m_nAxisNum = nAxisNum;
    m_lStepNum = vdStepVelecity.size();
    m_pvlRelPulse = new std::vector <long> [nAxisNum];
    for (int nAxisNo = 0; nAxisNo < nAxisNum; nAxisNo++)
    {
		//各轴脉冲、各轴轴号、插补类型
        m_pvlRelPulse[nAxisNo] = avlRelPulse[nAxisNo];
//		m_awAxis[nAxisNo] = nAxisNo;
		m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE;
    }
	m_eThreadStatus = STEPMOVE_THREAD_STATUS_START;
	Start();
}

void CStepMoveControlJY::StepMove( WORD wAxis[], std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum ,int nMoveDir)
{
	s_cStepMoveLog.Write("entry StepMoveControl Interface func of StepMove involve 5 Param");
    m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS;
	//3-28
	m_eMoveDir = (E_STEP_MOVE_DIRECTION)nMoveDir;
	//
    memset(m_awAxis, 0, sizeof(m_awAxis));
    memcpy(m_awAxis, wAxis, sizeof(WORD) * nAxisNum);
    StepMove(avlRelPulse, vdStepVelecity, nAxisNum);
}

UINT CStepMoveControlJY::ThreadStepMove( void *pParam )
{
    CStepMoveControlJY *pMyObj = (CStepMoveControlJY *)pParam;
    pMyObj->m_bThreadOnOff = 1;
    while(pMyObj->m_bThreadOnOff)
    {		
        if (((pMyObj->m_lCurrentStepNo >= pMyObj->m_lStepNum) && (STEP_MOVE_DIRECTION_POSITIVE == pMyObj->m_eMoveDir))&&  false== pMyObj->m_bInsterBuffer
            || ((pMyObj->m_lCurrentStepNo < 0) && (STEP_MOVE_DIRECTION_NEGATIVE == pMyObj->m_eMoveDir)) && false == pMyObj->m_bInsterBuffer)
        {
			s_cStepMoveLog.Write(" m_nCoordinate:%d Interpolation thread:%d %d", pMyObj->m_nCoordinate, pMyObj->m_lCurrentStepNo, pMyObj->m_lStepNum);
            pMyObj->m_bThreadOnOff = 0;
            break;
        }

        if (pMyObj->IsOnPauseStepNo())
        {
			//Useless
			s_cStepMoveLog.Write("Interpolation thread:execution module of PauseOnStep");
            pMyObj->DoPauseOnStep();
        }

        else if (pMyObj->IsOnIntervalPauseStepNo())
        {
			//Useless
			s_cStepMoveLog.Write("Interpolation thread:execution module of PauseByInterval");
            pMyObj->DoPauseByInterval();
        }

        else if (pMyObj->m_eThreadStatus == STEPMOVE_THREAD_STATUS_GOING)
        {
            pMyObj->StepMove();
        }
        
        else if (pMyObj->m_eThreadStatus == STEPMOVE_THREAD_STATUS_PAUSING)
        {
			//Useless
			s_cStepMoveLog.Write("Interpolation thread:execution module of Pause");
            pMyObj->DoPause();
            pMyObj->m_eThreadStatus = STEPMOVE_THREAD_STATUS_PAUSED;
        }
        
        else if (pMyObj->m_eThreadStatus == STEPMOVE_THREAD_STATUS_CONTINUING)
        {
			s_cStepMoveLog.Write("Interpolation thread:execution module of Continue");
            pMyObj->DoContinue();
            pMyObj->m_eThreadStatus = STEPMOVE_THREAD_STATUS_GOING;
        }
        
        else if (pMyObj->m_eThreadStatus == STEPMOVE_THREAD_STATUS_ACCING)
        {
			s_cStepMoveLog.Write("Interpolation thread:execution module of acc");
            pMyObj->DoAcc();
            pMyObj->m_eThreadStatus = STEPMOVE_THREAD_STATUS_GOING;
        }
        
        else if (pMyObj->m_eThreadStatus == STEPMOVE_THREAD_STATUS_DECING)
        {
			s_cStepMoveLog.Write("Interpolation thread:execution module of dec");
            pMyObj->DoDec();
            pMyObj->m_eThreadStatus = STEPMOVE_THREAD_STATUS_GOING;
        }
		else
		{
			s_cStepMoveLog.Write("Interpolation thread:execution module of NULL");
			Sleep(5);
		}
        DoEvent();
        Sleep(20);
    }
    
    pMyObj->CompleteStepMove();
    pMyObj->m_eThreadStatus = STEPMOVE_THREAD_STATUS_COMPLETE;
	s_cStepMoveLog.Write("exit thread");
    return 0;
}

void CStepMoveControlJY::StepMove()
{
	
    long lRelPulse[6] = {0};
	double dVelTotal = 0.0;
	
	if (m_lCurrentStepNo<m_vdStepVel.size()&& m_lCurrentStepNo<m_pvlRelPulse[0].size())
	{
		//WriteLog("m_lCurrentStepNo:%d %d", m_lCurrentStepNo, m_vdStepVel.size());
		
		for (int nIdx = 0; nIdx < m_nAxisNum; nIdx++)
		{
			//WriteLog("m_pvlRelPulse[%d]:%d", nIdx, m_pvlRelPulse[nIdx].size());
			if (1 == nIdx)
			{
				lRelPulse[nIdx] = 0; // 单松下外部轴插补崩溃临时处理
			}
			else
			{
				lRelPulse[nIdx] = m_pvlRelPulse[nIdx][m_lCurrentStepNo] * m_eMoveDir;
			}
		}
		dVelTotal = m_vdStepVel[m_lCurrentStepNo] * m_dVelRatio;
		s_cStepMoveLog.Write("m_nCoordinate:%d m_lCurrentStepNo :%d m_dVelRatio:%lf dVelTotal:%lf %lf", m_nCoordinate, m_lCurrentStepNo, m_dVelRatio,dVelTotal, m_vdStepVel[m_lCurrentStepNo]);
	}
	else
	{
		return;
	}   
    if(TRUE == CheckBeforeInterMove())
    {
		s_cStepMoveLog.Write("m_nCoordinate:%d Interpolation module:StepNo%d %d %d %lf", m_nCoordinate, m_lCurrentStepNo, lRelPulse[0], lRelPulse[1], dVelTotal);
        if (m_eThreadStatus == STEPMOVE_THREAD_STATUS_PAUSING
            || m_eThreadStatus == STEPMOVE_THREAD_STATUS_ACCING
            || m_eThreadStatus == STEPMOVE_THREAD_STATUS_DECING)
        { 
			 s_cStepMoveLog.Write("threadstatus return m_lCurrentStepNo:%d",m_lCurrentStepNo);
             return ;
        }   
		
		long lRemainSpace = dmc_conti_remain_space(0, m_nCoordinate);
		long lCurrentMark = dmc_conti_read_current_mark(0, m_nCoordinate);
		s_cStepMoveLog.Write("m_nCoordinate:%d InterpolationMove before:lRemainSpace:%ld lCurrentMark:%ld", m_nCoordinate,lRemainSpace,lCurrentMark);

        InterpolationMove(lRelPulse, dVelTotal, dVelTotal);

		lRemainSpace = dmc_conti_remain_space(0, m_nCoordinate);
		m_lCurrentMark = dmc_conti_read_current_mark(0, m_nCoordinate);

		s_cStepMoveLog.Write("m_nCoordinate:%d InterpolationMove after:lRemainSpace:%ld lCurrentMark:%ld", m_nCoordinate,lRemainSpace,m_lCurrentMark);

		s_cStepMoveLog.Write("InterpolationMove");
        if (TRUE == m_bIfAddSwing)
        {
            Swing();
        }
        
        RecordCurrentStepNo();

        m_lCurrentStepNo += m_eMoveDir;

        if (NULL != m_pFun)
        {
            m_pFun(m_pCallObj);
        }
    }
}

void CStepMoveControlJY::DoPause()
{
    ChangeVel();
}

void CStepMoveControlJY::DoPauseOnStep()
{
    if (FALSE == IsOnPauseStepNo())
    {
        return ;
    }

    if (m_eThreadStatus == STEPMOVE_THREAD_STATUS_STOPPED)
    {
        return ;
    }

    m_lPauseStepNo = 0xffffff;
    DoPause();
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_PAUSED;
}

void CStepMoveControlJY::DoPauseByInterval()
{
    if (m_eThreadStatus == STEPMOVE_THREAD_STATUS_STOPPED)
    {
        return ;
    }
    
    DoPause();
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_PAUSED;
    if (NULL != m_pFunOnPauseByInterval)
    {
        m_pFunOnPauseByInterval(m_pCallObjOnPauseByInterval);
    }
}

void CStepMoveControlJY::DoContinue()
{
    ChangeVel();
}

void CStepMoveControlJY::DoAcc()
{
    ChangeVel();  
}

void CStepMoveControlJY::DoDec()
{
    ChangeVel();  
}

void CStepMoveControlJY::ChangeVel()
{
    long lRelPulse[6] = {0};

    for (WORD wStepNo = 0; wStepNo < m_nVelChangeStepNum; wStepNo++)
    {
        for (int nAxisNo = 0; nAxisNo < m_nAxisNum; nAxisNo++)
        {
            lRelPulse[nAxisNo] = m_pvlRelPulse[nAxisNo][m_lCurrentStepNo] * m_eMoveDir;
        }
        
        double dVelTotal = m_dStepVelChangeTable[wStepNo];

        CheckBeforeInterMove();

        if ((STEPMOVE_THREAD_STATUS_PAUSING == m_eThreadStatus && (m_nVelChangeStepNum - 1) == wStepNo)
            || (STEPMOVE_THREAD_STATUS_CONTINUING == m_eThreadStatus && 0 == wStepNo))
        {
            InterpolationMove(lRelPulse, 0, dVelTotal);
        }
        else
        {
            InterpolationMove(lRelPulse, dVelTotal, dVelTotal);
        }
        
        RecordCurrentStepNo();
        m_lCurrentStepNo += m_eMoveDir;
        
        if (NULL != m_pFun)
        {
            m_pFun(m_pCallObj);
        }
    }
}

void CStepMoveControlJY::CalcVelChangeTable(double dVelStart, double dVelEnd)
{
    if(m_nVelChangeStepNum > 0)
    {
        double dStep = (dVelEnd - dVelStart) / (m_nVelChangeStepNum + 1);
        
        for (int i = 0; i < m_nVelChangeStepNum; i++)
        {
            m_dStepVelChangeTable[i] = dVelStart + dStep * (i + 1);
        }
    }
}

void CStepMoveControlJY::Start()
{
    m_dVelRatio = 1;
	//3-28
//    m_lCurrentStepNo = 0;
	//
    m_pFileCurrentStepNo = fopen(".\\CurrentStepNo.txt", "w");
	m_pFileCurrentMark = fopen(".\\CurrentMark.txt", "w");
	//3-29
//    m_eMoveDir = STEP_MOVE_DIRECTION_POSITIVE;
	//
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_GOING;
    
    CalcVelChangeTable(0, m_vdStepVel[m_nVelChangeStepNum]);
    WORD wStepNo = 0;
    for (wStepNo = 0; wStepNo < m_nVelChangeStepNum; wStepNo++)
    {	//修改联动时，起步脉冲速度
        m_vdStepVel[wStepNo] = m_dStepVelChangeTable[wStepNo];
    }
    CalcVelChangeTable(m_vdStepVel[m_lStepNum - 1 - m_nVelChangeStepNum], 0);
    for ( wStepNo = m_lStepNum - m_nVelChangeStepNum; wStepNo < m_lStepNum; wStepNo++)
    {
		//修改联动时，停止脉冲速度
        m_vdStepVel[wStepNo] = m_dStepVelChangeTable[wStepNo - (m_lStepNum - m_nVelChangeStepNum)];
    }

    if(MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType || MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
    {
        m_cDrAdap.StartContiInterMove(m_nAxisNum, m_awAxis, m_nCardNo);
    }
	s_cStepMoveLog.Write("entry thread:插补坐标系：%d", m_nCoordinate);
    AfxBeginThread(ThreadStepMove, this);
}

void CStepMoveControlJY::Pause(int nDecStepNum)
{
    if(STEPMOVE_THREAD_STATUS_GOING != m_eThreadStatus)
    {
        return ;
    }
    
    m_nVelChangeStepNum = nDecStepNum;
    CalcVelChangeTable(m_vdStepVel[m_lCurrentStepNo], 0);
    
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_PAUSING;
}

void CStepMoveControlJY::PauseOnStepNo( long lStepNo )
{
    m_lPauseStepNo = lStepNo;
}

void CStepMoveControlJY::Continue(int nAccStepNum)
{
    if((STEPMOVE_THREAD_STATUS_PAUSED == m_eThreadStatus)
        ||(m_eThreadStatus == STEPMOVE_THREAD_STATUS_PAUSED_ONSTEP)
        || (m_eThreadStatus == STEPMOVE_THREAD_STATUS_PAUSED_BYINTERVAL))
    {        
        m_nVelChangeStepNum = nAccStepNum;
        CalcVelChangeTable(0, m_vdStepVel[m_lCurrentStepNo + nAccStepNum * m_eMoveDir]);
        
        m_eThreadStatus = STEPMOVE_THREAD_STATUS_CONTINUING;
    }  
}

void CStepMoveControlJY::Accelerate(float fRatio, int nAccStepNum)
{
    if(STEPMOVE_THREAD_STATUS_GOING != m_eThreadStatus)
    {
        return ;
    }
    
    if (m_dVelRatio >= 10)
    {
        return ;
    }
     
    if (TRUE == m_bIfAddSwing)
    {
        m_cDrAdap.ChangeCurrentSpeed(m_wSwingAxis, m_cDrAdap.GetCurrentSpeed(m_wSwingAxis) * 1.2);
    }

    m_nVelChangeStepNum = nAccStepNum;

    CalcVelChangeTable(m_vdStepVel[m_lCurrentStepNo] * m_dVelRatio, 
            m_vdStepVel[m_lCurrentStepNo + nAccStepNum * m_eMoveDir] * (m_dVelRatio + fRatio));
    
    m_dVelRatio += fRatio;
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_ACCING;
}

void CStepMoveControlJY::Decelerate(float fRatio, int nDecStepNum)
{
    if(STEPMOVE_THREAD_STATUS_GOING != m_eThreadStatus)
    {
        return ;
    }
    
    if (m_dVelRatio <= 0.25)
    {
        return ;
    }
    
    m_nVelChangeStepNum = nDecStepNum;
    CalcVelChangeTable(m_vdStepVel[m_lCurrentStepNo] * m_dVelRatio, 
            m_vdStepVel[m_lCurrentStepNo + nDecStepNum * m_eMoveDir] * (m_dVelRatio - fRatio));
    
    m_dVelRatio -= fRatio;
    m_eThreadStatus = STEPMOVE_THREAD_STATUS_DECING;
}

BOOL CStepMoveControlJY::GoBack()
{
    if (STEPMOVE_THREAD_STATUS_PAUSED != m_eThreadStatus)
    {
        AfxMessageBox("请先暂停后再回退！");
        return FALSE;
    }
    else
    {
        m_eMoveDir = STEP_MOVE_DIRECTION_NEGATIVE;
        m_lCurrentStepNo--;

        Continue();

        return TRUE;
    }
}

BOOL CStepMoveControlJY::GoForward()
{
    if (STEPMOVE_THREAD_STATUS_PAUSED != m_eThreadStatus)
    {
        AfxMessageBox("请先暂停后再回退！");
        return FALSE;
    }
    else
    {
        m_eMoveDir = STEP_MOVE_DIRECTION_POSITIVE;
        m_lCurrentStepNo++;
        
        Continue();

        return TRUE;
    }
}

void CStepMoveControlJY::EmgStop()
{
    if (STEPMOVE_THREAD_STATUS_COMPLETE != m_eThreadStatus
        && STEPMOVE_THREAD_STATUS_INIT != m_eThreadStatus)
    {

        m_eThreadStatus = STEPMOVE_THREAD_STATUS_STOPPED;
        m_bThreadOnOff = 0;
        
        if(MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType || MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType)
        {
            m_cDrAdap.ContiInterMoveDecelStop(m_nCardNo);
            return ;
        }
        else if((MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
        || (MOVE_CONTROL_CARD_4400 == m_wMoveControlCardType) )
        {
            for(int nIdx = 0; nIdx < m_nAxisNum; nIdx++)
            {
                m_cDrAdap.DecelStop(m_awAxis[nIdx], DEC);
            }
        }

		s_cStepMoveLog.Write("EmgStop module:DecelStop");
    }
}

long CStepMoveControlJY::GetPosition( WORD wAxisNo )
{
    return m_cDrAdap.GetPosition(wAxisNo);
}

double CStepMoveControlJY::GetVelcecity( WORD wAxisNo )
{
    return m_cDrAdap.GetCurrentSpeed(wAxisNo);
}

void CStepMoveControlJY::RecordCurrentStepNo()
{
    fprintf(m_pFileCurrentStepNo, "%12d\n", m_lCurrentStepNo);
}

void CStepMoveControlJY::RecordCurrentMark()
{
    fprintf(m_pFileCurrentMark, "%12d\n", m_lCurrentMark);
}

void CStepMoveControlJY::CompleteStepMove()
{
    m_lPauseStepNo = 0xffffff;
    m_lIntervalStepNum = 0xffffff;
    m_bIfAddSwing = FALSE;
    fclose(m_pFileCurrentStepNo);
	fclose(m_pFileCurrentMark);
    if (NULL != m_pvlRelPulse)
    {
        delete [] m_pvlRelPulse;
        m_pvlRelPulse = NULL;
    }
}

BOOL CStepMoveControlJY::IsComplete()
{
    if (STEPMOVE_THREAD_STATUS_INIT == m_eThreadStatus
        || STEPMOVE_THREAD_STATUS_COMPLETE == m_eThreadStatus)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

long CStepMoveControlJY::GetCurrentStepNo()
{

	if ( m_lCurrentStepNo != 0)
	{
		return m_lCurrentStepNo - 1 * m_eMoveDir; // m_lCurrentStepNo指示的是刚发出的那段的下一段
	}
	else
	{
		return m_lCurrentStepNo;
	}

}

long CStepMoveControlJY::GetCurrentMark()
{
	return m_lCurrentMark;
}

E_STEPMOVE_THREAD_STATUS CStepMoveControlJY::GetStatus()
{
	return m_eThreadStatus;
}

double CStepMoveControlJY::GetVelRatio()
{
	return m_dVelRatio;
}

void CStepMoveControlJY::SetCallBackFun(void *pObj, void (*pFun)(void *))
{
    m_pFun = pFun;
    m_pCallObj = pObj;
}

void CStepMoveControlJY::SetCallBackFunOnPauseByInterval( void *pObj, void (*pFun)(void *) )
{
    m_pFunOnPauseByInterval = pFun;
    m_pCallObjOnPauseByInterval = pObj;  
}

void CStepMoveControlJY::Minus( std::vector <long> &vlData )
{
    for (WORD wIdx = 0; wIdx < vlData.size(); wIdx++)
    {
        vlData[wIdx] = -vlData[wIdx];
    }
}

void CStepMoveControlJY::RelPositionMove( long alRelPulse[], double dTotalVel, int nAxisNum, long lStepNum )
{
    std::vector <long> *pvlRelPulse = new std::vector <long> [nAxisNum];
    for (int nAxisNo = 0; nAxisNo < nAxisNum; nAxisNo++)
    {
        pvlRelPulse[nAxisNo].assign(lStepNum, alRelPulse[nAxisNo] / lStepNum);
    }
    std::vector <double> vdStepVel;
    vdStepVel.assign(lStepNum, dTotalVel);
    
    if(STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS != m_eInterpoType)
    {
        m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE;
    }
    StepMove(pvlRelPulse, vdStepVel, nAxisNum);
	delete [] pvlRelPulse;
	pvlRelPulse = NULL;
}

void CStepMoveControlJY::RelPositionMove( WORD wAxis[], long alRelPulse[], double dTotalVel, int nAxisNum, long lStepNum )
{
    m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS;
    memset(m_awAxis, 0, sizeof(m_awAxis));
    memcpy(m_awAxis, wAxis, sizeof(WORD) * nAxisNum);
    RelPositionMove(alRelPulse, dTotalVel, nAxisNum, lStepNum);
}

void CStepMoveControlJY::AbsPositionMove( long alAbsPulse[], double dTotalVel, int nAxisNum, long lStepNum )
{
    std::vector <long> *pvlRelPulse = new std::vector <long> [nAxisNum];
    for (int nAxisNo = 0; nAxisNo < nAxisNum; nAxisNo++)
    {
        pvlRelPulse[nAxisNo].assign(lStepNum, 
            (alAbsPulse[nAxisNo] - m_cDrAdap.GetPosition(nAxisNo)) / lStepNum);
    }
    std::vector <double> vdStepVel;
    vdStepVel.assign(lStepNum, dTotalVel);
    
    if(STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS != m_eInterpoType)
    {
        m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE;
    }
    StepMove(pvlRelPulse, vdStepVel, nAxisNum);
	delete [] pvlRelPulse;
	pvlRelPulse = NULL;
}

void CStepMoveControlJY::AbsPositionMove( WORD wAxis[], long alAbsPulse[], double dTotalVel, int nAxisNum, long lStepNum )
{
    m_eInterpoType = STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS;
    memset(m_awAxis, 0, sizeof(m_awAxis));
    memcpy(m_awAxis, wAxis, sizeof(WORD) * nAxisNum);
    AbsPositionMove(alAbsPulse, dTotalVel, nAxisNum, lStepNum);
}

void CStepMoveControlJY::InterpolationMove( long alRelPulse[], double dMinVel, double dMaxVel )
{
    if(MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType || (MOVE_CONTROL_CARD_5800 == m_wMoveControlCardType))
    {
        m_cDrAdap.ContiInterMove(m_nAxisNum, m_awAxis, alRelPulse, 0,
            dMinVel, dMaxVel, ACC, DEC, m_nCardNo);

    }
    else if((MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
        || (MOVE_CONTROL_CARD_4400 == m_wMoveControlCardType))
    {
        if (5 == m_nAxisNum || 6 == m_nAxisNum)
        {
            m_cDrAdap.InterpolationMove6(m_awAxis, alRelPulse, 0, 
                dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
        }
        else if (4 == m_nAxisNum)
        {
            if (STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove4(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
            else if (STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove4(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
        }
        else if (3 == m_nAxisNum)
        {
            if (STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove3(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
            else if (STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove3(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
        }
        else if (2 == m_nAxisNum)
        {
            if (STEP_MOVE_INTERPOLATION_TYPE_ARBITRARY_AXIS == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove2(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
            else if (STEP_MOVE_INTERPOLATION_TYPE_SEQUENCE == m_eInterpoType)
            {
                m_cDrAdap.InterpolationMove2(m_awAxis, alRelPulse, 0, 
                    dMinVel, dMaxVel, ACC, DEC, m_nCardNo);
            }
        }
        else if (1 == m_nAxisNum)
        {
            m_cDrAdap.PosMove(m_awAxis[0], alRelPulse[0], 0, 
                dMinVel, dMaxVel, ACC, DEC);
        }
    }
}

BOOL CStepMoveControlJY::CheckDoneAll()
{
    BOOL bIsDone = TRUE;
    if(MOVE_CONTROL_CARD_2C80 == m_wMoveControlCardType)
    {
        bIsDone = (128 == m_cDrAdap.ContiCheckRemainSpace(m_nCardNo));
    }
    else if ((MOVE_CONTROL_CARD_2610 == m_wMoveControlCardType)
        || (MOVE_CONTROL_CARD_4400 == m_wMoveControlCardType))
    {
        for (WORD wIdx = 0; wIdx < m_nAxisNum; wIdx++)
        {
		    bIsDone = bIsDone && m_cDrAdap.CheckDone(m_awAxis[wIdx]);
        }
    }
    
    return bIsDone;
}

BOOL CStepMoveControlJY::CheckPreBuff()
{
    BOOL bIsFull = FALSE;
    bIsFull = m_cDrAdap.CheckPreBuff(m_nCardNo, m_wMoveControlCardType);

    return bIsFull;
}

BOOL CStepMoveControlJY::IsPauseComplete()
{
	if (STEPMOVE_THREAD_STATUS_PAUSED == m_eThreadStatus)
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL CStepMoveControlJY::IsContinueComplete()
{
    if (STEPMOVE_THREAD_STATUS_GOING == m_eThreadStatus)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CStepMoveControlJY::IsAccelerateComplete()
{
    if (STEPMOVE_THREAD_STATUS_GOING == m_eThreadStatus)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CStepMoveControlJY::IsDecelerateComplete()
{
    if (STEPMOVE_THREAD_STATUS_GOING == m_eThreadStatus)
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CStepMoveControlJY::CheckBeforeInterMove()
{
    while(CheckPreBuff() == 1)
    {
        if (m_eThreadStatus == STEPMOVE_THREAD_STATUS_STOPPED)
        {
            return FALSE;
        }        
		//s_cStepMoveLog.Write("CheckBeforeInterMove module:loop of checkBuff return 1");
        Sleep(3);
    } 
	s_cStepMoveLog.Write("CheckBeforeInterMove module:loop of checkBuff return 0");
    return TRUE;
}

void CStepMoveControlJY::ContiMovePause()
{
	m_cDrAdap.ContiInterMovePause(m_nCardNo);
	return;
}

void CStepMoveControlJY::ContiMoveRestore()
{
	m_cDrAdap.ContiInterMoveContinue(m_nCardNo);
	return;
}

void CStepMoveControlJY::AdjustData( WORD wAxis, long lDataAdjust )
{
    WORD wIdx = GetAxisIdx(wAxis);
    m_pvlRelPulse[wIdx][m_lCurrentStepNo + 1] += lDataAdjust;
}

void CStepMoveControlJY::InsertData(std::vector <long> avlRelPulse[], std::vector <double> vdStepVelecity, int nAxisNum)
{
	if (m_eThreadStatus == STEPMOVE_THREAD_STATUS_COMPLETE)
	{
		s_cStepMoveLog.Write("InsertData:线程已结束，禁止继续放入数据");
		return;
	}
     m_vdStepVel.insert(m_vdStepVel.end(), vdStepVelecity.begin(), vdStepVelecity.end());
	 s_cStepMoveLog.Write("insert 111 m_nCoordinate：%d nAxisNum:%d m_vdStepVel:%d 速度：%lf", m_nCoordinate,nAxisNum, m_vdStepVel.size(), vdStepVelecity[0]);
	 
     for (int nAxisNo = 0; nAxisNo < nAxisNum; nAxisNo++)
     {
// 		 for (int n = 0;n<avlRelPulse[nAxisNo].size();n++)
// 		 {
// 			 s_cStepMoveLog.Write("avlRelPulse[%d]:%ld",nAxisNo,avlRelPulse[nAxisNo].at(n));
// 		 }
		 if (NULL == m_pvlRelPulse)
		 {
			 s_cStepMoveLog.Write("m_pvlRelPulse为空");
			 break;
		 }		 
		 m_pvlRelPulse[nAxisNo].insert(m_pvlRelPulse[nAxisNo].end(), avlRelPulse[nAxisNo].begin(), avlRelPulse[nAxisNo].end());
		 s_cStepMoveLog.Write("m_nCoordinate:%d m_pvlRelPulse[%d]:size：%d", m_nCoordinate,nAxisNo, m_pvlRelPulse[nAxisNo].size());
     }
	 
	 m_lStepNum = m_lStepNum + vdStepVelecity.size();
	 s_cStepMoveLog.Write("m_nCoordinate:%d m_lStepNum:%d", m_nCoordinate, m_lStepNum);
    	
}

int CStepMoveControlJY::GetAxisIdx( WORD wAxis )
{
    for (WORD wIdx = 0; wIdx < m_nAxisNum; wIdx++)
    {
        if (m_awAxis[wIdx] == wAxis)
        {
            return wIdx;
        }
    }

    AfxMessageBox("修改数据输入轴号错误！");
    return 0;
}

void CStepMoveControlJY::PauseByInterval( long lIntervalStepNum )
{
    m_lIntervalStepNum = lIntervalStepNum;
}

BOOL CStepMoveControlJY::IsOnPauseStepNo()
{
    if (m_lCurrentStepNo == (m_lPauseStepNo - m_nVelChangeStepNum * m_eMoveDir))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

BOOL CStepMoveControlJY::IsOnIntervalPauseStepNo()
{
    if(((m_lCurrentStepNo + m_nVelChangeStepNum * m_eMoveDir) % m_lIntervalStepNum == 0)
        && (m_eThreadStatus != STEPMOVE_THREAD_STATUS_PAUSED)
        && (m_lCurrentStepNo > m_nVelChangeStepNum)
        && (m_lCurrentStepNo < (m_lStepNum - m_nVelChangeStepNum)))
    {
        return TRUE;
    }
    else
    {
        return FALSE;
    }
}

void CStepMoveControlJY::AddSwing( WORD wAxisNo, std::vector <long> vlInnerSwingAmp, std::vector <long> vlOuterSwingAmp, 
                                std::vector <double> vdInnerStayTime, std::vector <double> vdOuterStayTime, 
                                int nSwingPeriod )
{
    m_wSwingAxis = wAxisNo;
    m_vlInnerSwingAmp = vlInnerSwingAmp;
    m_vlOuterSwingAmp = vlOuterSwingAmp;
    m_vdInnerStayTime = vdInnerStayTime;
    m_vdOuterStayTime = vdOuterStayTime;
    m_nSwingPeriod = nSwingPeriod;
    m_lSwingZero = m_cDrAdap.GetPosition(wAxisNo);
    m_bIfAddSwing = TRUE;
    CalcStepTime();
}

void CStepMoveControlJY::AddSwing( WORD wAxisNo, long lInnerSwingAmp, long lOuterSwingAmp, double dInnerStayTime, double dOuterStayTime, int nSwingPeriod )
{
    m_wSwingAxis = wAxisNo;
    m_vlInnerSwingAmp.assign(m_lStepNum, lInnerSwingAmp);
    m_vlOuterSwingAmp.assign(m_lStepNum, lOuterSwingAmp);
    m_vdInnerStayTime.assign(m_lStepNum, dInnerStayTime);
    m_vdOuterStayTime.assign(m_lStepNum, dOuterStayTime);
    m_nSwingPeriod = nSwingPeriod;
    m_lSwingZero = m_cDrAdap.GetPosition(wAxisNo);
    CalcStepTime();
    m_bIfAddSwing = TRUE;
}

void CStepMoveControlJY::Swing()
{
    if(m_lCurrentStepNo % m_nSwingPeriod == 0)
    {
        long lSwingAmplitude = 0;
        double dStayTime = 0;
        if (0 == ((m_lCurrentStepNo / m_nSwingPeriod) % 2))
        {
            lSwingAmplitude = m_vlInnerSwingAmp[m_lCurrentStepNo] * m_eMoveDir;
            dStayTime = m_vdInnerStayTime[m_lCurrentStepNo];
        }
        else
        {
            lSwingAmplitude = m_vlOuterSwingAmp[m_lCurrentStepNo + 1] * m_eMoveDir * (-1);
            dStayTime = m_vdOuterStayTime[m_lCurrentStepNo + 1];
        }
        
        double dSwingTime = 0;
        for (int n = 0; n < m_nSwingPeriod; n++)
        {
            dSwingTime += m_vdStepTime[m_lCurrentStepNo + n];
        }
        dSwingTime /= m_dVelRatio;
        if (dSwingTime > dStayTime)
        {
            dSwingTime -= dStayTime;
        }
        
        m_lSwingPeak = m_lSwingZero + lSwingAmplitude;
        
        double dSwingVel = abs(m_lSwingPeak - m_cDrAdap.GetPosition(m_wSwingAxis)) / dSwingTime;
        
//         while(cDriverAdap.CheckDone(m_wSwingAxis) == FALSE)
//         {
//             Sleep(5);
//         }
        m_cDrAdap.PosMove(m_wSwingAxis, m_lSwingPeak, 1, dSwingVel, dSwingVel, ACC, DEC);
    }
}

void CStepMoveControlJY::CalcStepTime()
{
    m_vdStepTime.clear();
    for (long lStepNo = 0; lStepNo < m_lStepNum; lStepNo++)
    {
        double dStepPulse = 0;
        for (int nIdx = 0; nIdx < m_nAxisNum; nIdx++)
        {
            dStepPulse += SQUARE(m_pvlRelPulse[nIdx][lStepNo]);
        }
        dStepPulse = sqrt(dStepPulse);
        double dStepTime = dStepPulse / m_vdStepVel[lStepNo];
        m_vdStepTime.push_back(dStepTime);
    }
}

void CStepMoveControlJY::SetStartStopStepNum( WORD wStepNum )
{
    m_nVelChangeStepNum = wStepNum;
}

void CStepMoveControlJY::SetStartStepNo(long lStartStepNo)
{
	m_lCurrentStepNo = lStartStepNo;
}

void CStepMoveControlJY::SetBuffNum( int nBuffNum )
{   
	m_cDrAdap.SetBuffNum(nBuffNum);
}

void CStepMoveControlJY::StopStep()
{
	m_cDrAdap.StopContiInterMove();
	m_bInsterBuffer = FALSE;
	s_cStepMoveLog.Write("StopStep module:StopContiInterMove");
}

