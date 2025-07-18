// WeldParamProcess.cpp : ʵ���ļ�
//

#include "stdafx.h"
//#include "XiWeldRobot.h"
#include "res\resource.h"
#include "WeldParamProcess.h"
#include "afxdialogex.h"
#include <ParamInput.h>


// CWeldParamProcess �Ի���

IMPLEMENT_DYNAMIC(CWeldParamProcess, CDialog)

CWeldParamProcess::CWeldParamProcess(std::vector<CUnit*> vpUnit, CWnd* pParent /*=NULL*/)
    : CDialog(IDD_WELD_PARAM, pParent)
    , m_dStartArcCurrent(0)
    , m_dStartArcVoltage(0)
    , m_dStopArcVoltage(0)
    , m_dStopArcCurrent(0)
    , m_dStopWaitTime(0)
    , m_dTrackCurrent(0)
    , m_WeldVelocity(0)
    , m_dTrackVoltage(0)
    , m_dStartWaitTime(0)
    , m_verticalOffset(0)
    , m_dCrosswiseOffset(0)
    , m_strNewWorkpieceName(_T(""))
    , m_dWrapCurrent1(0)
    , m_dWrapCurrent2(0)
    , m_dWrapCurrent3(0)
    , m_dWrapVol1(0)
    , m_dWrapVol2(0)
    , m_dWrapVol3(0)
    , m_dWrapWaitTime1(0)
    , m_dWrapWaitTime2(0)
    , m_dWrapWaitTime3(0)
    , m_bRobotNoSelect(FALSE)
    , m_dStartPointScanDis(0)
    , m_nWrapCondition(1)
	, m_dWeldAngle(45.0)
	, m_dWeldDipAngle(0.0)
    , m_nLayerNo(1)
    , m_nStandWeldDir(1)
    , m_sUnitName(vpUnit[0]->GetRobotCtrl()->m_strRobotName)
    , m_vpUnit(vpUnit)
    , m_nRobotNum(vpUnit.size())
{
    m_bEditOrAdd = false;
	m_bShowImage = false;
    m_nRobotNo = 0;

    LoadParam();
    LoadWeldParam(m_vvvtFlatWeldParam, m_vvvtStandWeldParam);
    LoadAttributeParam(); 
    LoadWeaveParam();
}
CWeldParamProcess::CWeldParamProcess(CString strName, CWnd* pParent)
    : CDialog(IDD_WELD_PARAM, pParent)
    , m_dStartArcCurrent(0)
    , m_dStartArcVoltage(0)
    , m_dStopArcVoltage(0)
    , m_dStopArcCurrent(0)
    , m_dStopWaitTime(0)
    , m_dTrackCurrent(0)
    , m_WeldVelocity(0)
    , m_dTrackVoltage(0)
    , m_dStartWaitTime(0)
    , m_verticalOffset(0)
    , m_dCrosswiseOffset(0)
    , m_strNewWorkpieceName(_T(""))
    , m_dWrapCurrent1(0)
    , m_dWrapCurrent2(0)
    , m_dWrapCurrent3(0)
    , m_dWrapVol1(0)
    , m_dWrapVol2(0)
    , m_dWrapVol3(0)
    , m_dWrapWaitTime1(0)
    , m_dWrapWaitTime2(0)
    , m_dWrapWaitTime3(0)
    , m_bRobotNoSelect(FALSE)
    , m_dStartPointScanDis(0)
    , m_nWrapCondition(1)
    , m_dWeldAngle(45.0)
    , m_dWeldDipAngle(0.0)
    , m_nLayerNo(1)
    , m_nStandWeldDir(1)
    , m_sUnitName(strName)
    , m_nRobotNum(1)
{
    m_bEditOrAdd = false;
    m_bShowImage = false;
    m_nRobotNo = 0;

    LoadParamSingle();
    LoadWeldParam(m_vvtFlatWeldParam, m_vvtStandWeldParam);
    //LoadWeldParam(m_vvvtFlatWeldParam, m_vvvtStandWeldParam);
    LoadAttributeParam();
    LoadWeaveParam();
}

CWeldParamProcess::~CWeldParamProcess()
{
}

void CWeldParamProcess::DoDataExchange(CDataExchange* pDX)
{
    CDialog::DoDataExchange(pDX);
    DDX_Control(pDX, IDC_BOX_SELECT_DRAWINGLIST, m_bSelectDrawing);
    DDX_Text(pDX, IDC_EDI_START_CURRENT, m_dStartArcCurrent);
    DDX_Text(pDX, IDC_EDIT_ST_VOL, m_dStartArcVoltage);
    DDX_Text(pDX, IDC_EDIT_STOP_VOL, m_dStopArcVoltage);
    DDX_Text(pDX, IDC_EDIT_STOP_CURRENT, m_dStopArcCurrent);
    DDX_Text(pDX, IDC_EDIT_STOP_WAIT, m_dStopWaitTime);
    DDX_Text(pDX, IDC_EDIT_TRACK_CURRENT, m_dTrackCurrent);
    DDX_Text(pDX, IDC_EDIT_TRACK_VEL, m_WeldVelocity);
    DDX_Text(pDX, IDC_EDIT_TRACK_VOL, m_dTrackVoltage);
    DDX_Text(pDX, IDC_EDITSTRAT_WAIT, m_dStartWaitTime);
    DDX_Text(pDX, IDC_EDIT_VIR_OFFSET, m_verticalOffset);
    DDX_Text(pDX, IDC_EDIT_CORSS_OFFSET, m_dCrosswiseOffset);
    DDX_Text(pDX, IDC_EDIT_WRAP_CONDITION, m_nWrapCondition);
    DDX_Control(pDX, IDC_COMBO_WELD_TYPE, m_cWeldType);
    DDX_Text(pDX, IDC_EDIT_NEW_WOEKPIECE, m_strNewWorkpieceName);
    //  DDX_Radio(pDX, IDC_RADIO_EDIT, m_bEditOrAddPara);
    DDX_Text(pDX, IDC_EDI_WRAP_CURRENT1, m_dWrapCurrent1);
    DDX_Text(pDX, IDC_EDI_WRAP_CURRENT2, m_dWrapCurrent2);
    DDX_Text(pDX, IDC_EDI_WRAP_CURRENT3, m_dWrapCurrent3);
    DDX_Text(pDX, IDC_EDIT_WRAP_VOL1, m_dWrapVol1);
    DDX_Text(pDX, IDC_EDIT_WRAP_VOL2, m_dWrapVol2);
    DDX_Text(pDX, IDC_EDIT_WRAP_VOL3, m_dWrapVol3);
    DDX_Text(pDX, IDC_EDIT_WRAP_WAIT1, m_dWrapWaitTime1);
    DDX_Text(pDX, IDC_EDIT_WRAP_WAIT2, m_dWrapWaitTime2);
    DDX_Text(pDX, IDC_EDIT_WRAP_WAIT3, m_dWrapWaitTime3);
    DDX_Check(pDX, IDC_CHECK_ROBOTNO, m_bRobotNoSelect);
    DDV_MinMaxDouble(pDX, m_dCrosswiseOffset, -30, 30);
    DDV_MinMaxDouble(pDX, m_verticalOffset, -30, 30);
    DDV_MinMaxInt(pDX, m_nWrapCondition, -1, 255);
    DDX_Text(pDX, IDC_EDIT_START_SCAN_DIS, m_dStartPointScanDis);
    DDX_Control(pDX, IDC_BOX_LAYER_NO, m_cComboSelectLayerNo);
    DDX_Text(pDX, IDC_EDIT_WELD_ANGLE, m_dWeldAngle);
    DDV_MinMaxDouble(pDX, m_dWeldAngle, -20.0, 70.0);
    DDX_Text(pDX, IDC_EDIT_DIP_ANGLE, m_dWeldDipAngle);
    DDV_MinMaxDouble(pDX, m_dWeldDipAngle, -30.0, 30.0);
    DDX_Control(pDX, IDC_BOX_SELECT_ROBOT, m_boxSelectRobot);
    DDX_Control(pDX, IDC_BOX_STAND_DIR, m_ComboBoxStandDir);
    DDX_CBIndex(pDX, IDC_BOX_STAND_DIR, m_nStandWeldDir);
}


BEGIN_MESSAGE_MAP(CWeldParamProcess, CDialog)
	ON_CBN_SELCHANGE(IDC_BOX_SELECT_DRAWINGLIST, &CWeldParamProcess::OnCbnSelchangeBoxSelectDrawinglist)
	ON_BN_CLICKED(IDOK, &CWeldParamProcess::OnBnClickedOk)
	ON_CBN_SELCHANGE(IDC_COMBO_WELD_TYPE, &CWeldParamProcess::OnCbnSelchangeComboWeldType)
	ON_BN_CLICKED(IDC_BTN_DELETE, &CWeldParamProcess::OnBnClickedBtnDelete)
	ON_BN_CLICKED(IDC_BTN_EDIT, &CWeldParamProcess::OnBnClickedBtnEdit)
	ON_BN_CLICKED(IDC_BTN_SAVE, &CWeldParamProcess::OnBnClickedBtnSave)
	ON_BN_CLICKED(IDC_BTN_ADD, &CWeldParamProcess::OnBnClickedBtnAdd)
	ON_BN_CLICKED(IDC_RADIO_EDIT, &CWeldParamProcess::OnBnClickedRadioEdit)
	ON_BN_CLICKED(IDC_RADIO_ADD, &CWeldParamProcess::OnBnClickedRadioAdd)
    ON_BN_CLICKED(IDC_CHECK_ROBOTNO, &CWeldParamProcess::OnBnClickedCheckRobotno)
    ON_WM_PAINT()
	ON_BN_CLICKED(IDCANCEL, &CWeldParamProcess::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON_SAVE_POS, &CWeldParamProcess::OnBnClickedButtonSavePos)
    ON_CBN_SELCHANGE(IDC_BOX_LAYER_NO, &CWeldParamProcess::OnCbnSelchangeBoxLayerNo)
    ON_CBN_SELCHANGE(IDC_BOX_SELECT_ROBOT, &CWeldParamProcess::OnCbnSelchangeBoxSelectRobot)
    ON_CBN_SELCHANGE(IDC_BOX_STAND_DIR, &CWeldParamProcess::OnCbnSelchangeBoxStandDir)
    ON_BN_CLICKED(IDC_BUTTON2, &CWeldParamProcess::OnBnClickedButton2)
END_MESSAGE_MAP()


// CWeldParamProcess ��Ϣ�������
void CWeldParamProcess::SelchangeBoxSelectDrawinglist()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������
    UpdateData(TRUE);
    std::vector<std::vector<T_WELD_PARA>> *pvvtWeldParam;
    int nIndex = m_bSelectDrawing.GetCurSel();
    CString str;
    m_bSelectDrawing.GetLBText(nIndex, str);
    m_strWorkPiece = str;
    if (false == m_bRobotNoSelect)
    {
        pvvtWeldParam = &m_vvvtFlatWeldParam[m_nRobotNo]/*m_vvtFlatWeldParam*/;
    }
    else
    {
        pvvtWeldParam = &m_vvvtStandWeldParam[m_nRobotNo];
    }
    for (int i = 0; i < pvvtWeldParam->size(); i++)
    {
        if (str == pvvtWeldParam->at(i)[0].strWorkPeace)
        {
            m_tWeldSeamPare = pvvtWeldParam->at(i)[0];
            m_nLayerNo = pvvtWeldParam->at(i)[0].nLayerNo;
            m_cComboSelectLayerNo.ResetContent();
            for (int nNo = 0; nNo < pvvtWeldParam->at(i).size(); nNo++)
            {
                CString sLayerNo;
                sLayerNo.Format("%d", pvvtWeldParam->at(i)[nNo].nLayerNo + 1); // ��ʾ��1��ʼ
                m_cComboSelectLayerNo.AddString(sLayerNo);
            }
            m_cComboSelectLayerNo.SetCurSel(m_nLayerNo);
            break;
        }
    }
    OnRefreshPage(m_tWeldSeamPare);
}

void CWeldParamProcess::SelectLayerNoList()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������
    UpdateData(TRUE);
    std::vector<std::vector<T_WELD_PARA>>* pvvtWeldParam;
    int nIndex = m_bSelectDrawing.GetCurSel();
    m_nLayerNo = m_cComboSelectLayerNo.GetCurSel();
    CString str;
    m_bSelectDrawing.GetLBText(nIndex, str);
    m_strWorkPiece = str;
    if (false == m_bRobotNoSelect)
    {
        pvvtWeldParam = &m_vvvtFlatWeldParam[m_nRobotNo]/*m_vvtFlatWeldParam*/;
    }
    else
    {
        pvvtWeldParam = &m_vvvtStandWeldParam[m_nRobotNo];
    }
    for (int i = 0; i < pvvtWeldParam->size(); i++)
    {
        if (str == pvvtWeldParam->at(i)[0].strWorkPeace)
        {
            m_tWeldSeamPare = pvvtWeldParam->at(i)[m_nLayerNo];
            break;
        }
    }
    OnRefreshPage(m_tWeldSeamPare);
}

void CWeldParamProcess::OnCbnSelchangeBoxSelectDrawinglist()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
    SelchangeBoxSelectDrawinglist();
    m_pRightDC->FillSolidRect(m_cRightClientRect, m_pRightDC->GetBkColor());
    ShowWeldGun(m_tWeldSeamPare.CrosswiseOffset, m_tWeldSeamPare.verticalOffset);
    UpdateData(FALSE);
    return;
}

void CWeldParamProcess::OnCbnSelchangeBoxLayerNo()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������
    SelectLayerNoList();
    m_pRightDC->FillSolidRect(m_cRightClientRect, m_pRightDC->GetBkColor());
    ShowWeldGun(m_tWeldSeamPare.CrosswiseOffset, m_tWeldSeamPare.verticalOffset);
    UpdateData(FALSE);
}

void CWeldParamProcess::OnRefreshPage(T_WELD_PARA tWeldPare)
{
    m_dStartArcCurrent = tWeldPare.dStartArcCurrent;
    m_dStartArcVoltage = tWeldPare.dStartArcVoltage;
    m_dStartWaitTime   = tWeldPare.dStartWaitTime;
    m_dTrackCurrent    = tWeldPare.dTrackCurrent;
    m_dTrackVoltage    = tWeldPare.dTrackVoltage;
    m_WeldVelocity     = tWeldPare.WeldVelocity;
    m_dStopArcCurrent  = tWeldPare.dStopArcCurrent;
    m_dStopArcVoltage  = tWeldPare.dStopArcVoltage;
    m_dStopWaitTime    = tWeldPare.dStopWaitTime;
    m_dCrosswiseOffset = tWeldPare.CrosswiseOffset;
    m_verticalOffset   = tWeldPare.verticalOffset;
    m_nWrapCondition   = tWeldPare.nWrapConditionNo;
	m_dWeldAngle	   = tWeldPare.dWeldAngle;
	m_dWeldDipAngle	   = tWeldPare.dWeldDipAngle;
    m_dWrapCurrent1    = tWeldPare.dWrapCurrentt1;
    m_dWrapCurrent2    = tWeldPare.dWrapCurrentt2;
    m_dWrapCurrent3    = tWeldPare.dWrapCurrentt3;
    m_dWrapVol1        = tWeldPare.dWrapVoltage1;
    m_dWrapVol2        = tWeldPare.dWrapVoltage2;
    m_dWrapVol3        = tWeldPare.dWrapVoltage3;
    m_dWrapWaitTime1   = tWeldPare.dWrapWaitTime1;
    m_dWrapWaitTime2   = tWeldPare.dWrapWaitTime2;
    m_dWrapWaitTime3   = tWeldPare.dWrapWaitTime3;
    m_nStandWeldDir    = tWeldPare.nStandWeldDir;
    UpdateData(FALSE);
}
T_WELD_PARA CWeldParamProcess::DownloadPageParam()
{
    UpdateData(TRUE);
    T_WELD_PARA tSeamPare;
    tSeamPare.dStartArcCurrent = m_dStartArcCurrent;
    tSeamPare.dStartArcVoltage = m_dStartArcVoltage;
    tSeamPare.dStartWaitTime = m_dStartWaitTime;
    tSeamPare.dTrackCurrent = m_dTrackCurrent;
    tSeamPare.dTrackVoltage = m_dTrackVoltage;
    tSeamPare.WeldVelocity = m_WeldVelocity;
    tSeamPare.dStopArcCurrent = m_dStopArcCurrent;
    tSeamPare.dStopArcVoltage = m_dStopArcVoltage;
    tSeamPare.dStopWaitTime = m_dStopWaitTime;
    tSeamPare.CrosswiseOffset = m_dCrosswiseOffset;
    tSeamPare.verticalOffset = m_verticalOffset;
    tSeamPare.nWrapConditionNo = m_nWrapCondition;
	tSeamPare.dWeldAngle = m_dWeldAngle;
    m_dWeldDipAngle = m_dWeldDipAngle > 20.0 ? 20.0 : m_dWeldDipAngle;
    m_dWeldDipAngle = m_dWeldDipAngle < -20.0 ? -20.0 : m_dWeldDipAngle;
	tSeamPare.dWeldDipAngle = m_dWeldDipAngle;
    tSeamPare.dWrapCurrentt1 = m_dWrapCurrent1;
    tSeamPare.dWrapCurrentt2 = m_dWrapCurrent2;
    tSeamPare.dWrapCurrentt3 = m_dWrapCurrent3;
    tSeamPare.dWrapVoltage1 = m_dWrapVol1;
    tSeamPare.dWrapVoltage2 = m_dWrapVol2;
    tSeamPare.dWrapVoltage3 = m_dWrapVol3;
    tSeamPare.dWrapWaitTime1 = m_dWrapWaitTime1;
    tSeamPare.dWrapWaitTime2 = m_dWrapWaitTime2;
    tSeamPare.dWrapWaitTime3 = m_dWrapWaitTime3;
    tSeamPare.nLayerNo = m_cComboSelectLayerNo.GetCurSel();
    tSeamPare.nStandWeldDir = m_nStandWeldDir;
    return tSeamPare;
}
void CWeldParamProcess::OnBnClickedOk()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
    //SaveWeldParam(0);                 
    //SaveWeldParam(1);  
    SaveWeldParam();
	m_bShowImage = false;
	CDialog::OnOK();
}

void CWeldParamProcess::OnCbnSelchangeComboWeldType()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(TRUE);
	int nIndex = m_cWeldType.GetCurSel();
	CString str;
	m_cWeldType.GetLBText(nIndex, str);
	m_strWeldSeamType = str;
	if ("����" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.VerticalWeldPare;
	}
	else if ("б��ƽ��" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.SlopeFlatWeldPare;
	}
	else if ("ƽ�Խ�" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.LevelFlatWeldPare;
	}
	else if ("�ߵ�ƽ��" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.FlatSeamingWeldPare;
	}
	else if ("�Ƿ�" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.CornerLineWeldPare;
	}
	else if ("��ǿ��" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.StrengthBoardWeldPare;
	}
	else if ("��Ե����" == str)
	{
        m_tWeldSeamPare = m_tWeldPara.MarginStraightSlitWeldPare;
	}
    else if ("�¿����庸��" == str)
    {
        m_tWeldSeamPare = m_tWeldPara.FrontPlateUnderFrameWeldPare;
    }
	m_dStartArcCurrent = m_tWeldSeamPare.dStartArcCurrent;
	m_dStartArcVoltage = m_tWeldSeamPare.dStartArcVoltage;
	m_dStartWaitTime   = m_tWeldSeamPare.dStartWaitTime;
	m_dTrackCurrent    = m_tWeldSeamPare.dTrackCurrent;
	m_dTrackVoltage    = m_tWeldSeamPare.dTrackVoltage;
	m_WeldVelocity     = m_tWeldSeamPare.WeldVelocity;
	m_dStopArcCurrent  = m_tWeldSeamPare.dStopArcCurrent;
	m_dStopArcVoltage  = m_tWeldSeamPare.dStopArcVoltage;
	m_dStopWaitTime    = m_tWeldSeamPare.dStopWaitTime;
	m_dCrosswiseOffset = m_tWeldSeamPare.CrosswiseOffset;
	m_verticalOffset   = m_tWeldSeamPare.verticalOffset;
    m_nWrapCondition   = m_tWeldSeamPare.nWrapConditionNo;
	m_dWeldAngle	   = m_tWeldSeamPare.dWeldAngle;
	m_dWeldDipAngle	   = m_tWeldSeamPare.dWeldDipAngle;
	UpdateData(FALSE);
}


BOOL CWeldParamProcess::OnInitDialog()
{
	CDialog::OnInitDialog(); 
    m_ComboBoxStandDir.AddString("������(ƽ����Ч)");
    m_ComboBoxStandDir.AddString("������(ƽ����Ч)");
	CButton* pBtn = (CButton*)GetDlgItem(IDC_CHECK_ROBOTNO);
	pBtn->SetCheck(0);// �򿪽���Ĭ��Ϊƽ������
    int i = 0;
	int nSetNo = 0;
    m_nLayerNo = 0;
    m_boxSelectRobot.ResetContent();
    for (i = 0; i < m_nRobotNum; i++)
    {
        CString str;
        str.Format("%d", i);
        m_boxSelectRobot.AddString(str);
    }
    m_boxSelectRobot.SetCurSel(0);
    
	//ˢ�½���
	m_bSelectDrawing.ResetContent();
	
    m_vvtFlatWeldParam = m_vvvtFlatWeldParam[m_nRobotNo];
	for (i = 0; i < m_vvtFlatWeldParam.size(); i++)
	{
		m_bSelectDrawing.AddString(m_vvtFlatWeldParam[i][0].strWorkPeace);
		if (m_strWorkPieceFlat[m_nRobotNo] == m_vvtFlatWeldParam[i][0].strWorkPeace)
		{
			nSetNo = i;
            for (int nNo = 0; nNo < m_vvtFlatWeldParam[i].size(); nNo++)
            {
                CString sLayerNo;
                sLayerNo.Format("%d", m_vvtFlatWeldParam[i][nNo].nLayerNo + 1);
                m_cComboSelectLayerNo.AddString(sLayerNo);
            }
		}
	}
	m_bSelectDrawing.SetCurSel(nSetNo);
    m_cComboSelectLayerNo.SetCurSel(m_nLayerNo);
    SelchangeBoxSelectDrawinglist();

	//���޸�
	// TODO:  �ڴ���Ӷ���ĳ�ʼ��
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
				  // �쳣: OCX ����ҳӦ���� FALSE
}
void CWeldParamProcess::LoadParam()
{
	COPini Opini;
    for (int i = 0; i < m_nRobotNum; i++)
    {
        Opini.SetFileName(DATA_PATH + m_vpUnit[i]->GetRobotCtrl()->m_strRobotName + WELD_PARAM_FILE);
        Opini.SetSectionName("WeldParam");
        CString strFlat, strStand;
        strFlat.Format("strWorkpieceFlat");
        strStand.Format("strWorkpieceStand");
        Opini.ReadString(strFlat, m_strWorkPieceFlat[i]);
        m_strWorkPieceFlat[i] = Utf8ToGBK(m_strWorkPieceFlat[i]);
        Opini.ReadString(strStand, m_strWorkPieceStand[i]);
        m_strWorkPieceStand[i] = Utf8ToGBK(m_strWorkPieceStand[i]);
    }
	//Opini.ReadString("bEditOrAdd", &m_bEditOrAdd);
}

void CWeldParamProcess::LoadParamSingle()
{
    COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
    Opini.SetSectionName("WeldParam");
    Opini.ReadString("strWorkpieceFlat", m_strWorkPieceFlat[0]);
    m_strWorkPieceFlat[0] = Utf8ToGBK(m_strWorkPieceFlat[0]);
    Opini.ReadString("strWorkpieceStand", m_strWorkPieceStand[0]);
    m_strWorkPieceStand[0] = Utf8ToGBK(m_strWorkPieceStand[0]);
    Opini.ReadString("StartPointScanDis", &m_dStartPointScanDis);

}

void CWeldParamProcess::LoadWeldThink(int nRobotNo, double &dWeldParam)
{
    COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
    Opini.SetSectionName("WeldParam");
    if (0 == nRobotNo)
    {
        Opini.ReadString("strWorkpieceFlat", &dWeldParam);
    }
    else
    {
        Opini.ReadString("strWorkpieceStand", &dWeldParam);
    }
}

bool CWeldParamProcess::LoadCurUseParam(std::vector<T_WELD_PARA>& vtWeldParam, bool bIsFlat)
{
    CString sCurUseParamName;
    CString sSectionName = (true == bIsFlat ? "strWorkpieceFlat" : "strWorkpieceStand");
    std::vector<std::vector<T_WELD_PARA>> &vvtWeldParam = (true == bIsFlat ? m_vvtFlatWeldParam : m_vvtStandWeldParam);

    COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
    Opini.SetSectionName("WeldParam");
    Opini.ReadString(sSectionName, sCurUseParamName);

    bool bFound = false;
    for (int i = 0; i < vvtWeldParam.size(); i++)
    {
        if (vvtWeldParam[i][0].strWorkPeace == sCurUseParamName)
        {
            vtWeldParam = vvtWeldParam[i];
            bFound = true;
            break;
        }
    }
    for (int i = 0; i < vtWeldParam.size(); i++)
    {
        //vtWeldParam[i].nWeldMethod = bIsFlat ? 1 : 0;       
        vtWeldParam[i].nWeldMethod = 0 == i ? 0 : 1; // ��һ�����ֱ�� ��������
        if (false == bIsFlat) vtWeldParam[i].nWeldMethod = 0; // ����ֱ��

        memset(&vtWeldParam[i].tWeaveParam, 0, sizeof(vtWeldParam[i].tWeaveParam)); // �ްڶ�
        vtWeldParam[i].tWeaveParam.nType = 1;
        for (int nWeaveNo = 0; nWeaveNo < m_vtWeaveParam.size(); nWeaveNo++)
        {
            if (vtWeldParam[i].nWrapConditionNo == m_vtWeaveParam[nWeaveNo].nNo)
            {
                vtWeldParam[i].tWeaveParam = m_vtWeaveParam[nWeaveNo];
                break;
            }
        }
    }
    return bFound;
}

bool CWeldParamProcess::LoadCurUseParam(int nRobotNo,std::vector<T_WELD_PARA>& vtWeldParam, bool bIsFlat)
{
    CString sCurUseParamName,strFlat,strStand;
    strFlat.Format("strWorkpieceFlat_%d", nRobotNo);
    strStand.Format("strWorkpieceStand_%d", nRobotNo);
    CString sSectionName = (true == bIsFlat ? strFlat : strStand);
    std::vector<std::vector<T_WELD_PARA>>& vvtWeldParam = (true == bIsFlat ? m_vvvtFlatWeldParam[nRobotNo] : m_vvvtStandWeldParam[nRobotNo]);

    COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
    Opini.SetSectionName("WeldParam");
    Opini.ReadString(sSectionName, sCurUseParamName);

    bool bFound = false;
    for (int i = 0; i < vvtWeldParam.size(); i++)
    {
        if (vvtWeldParam[i][0].strWorkPeace == sCurUseParamName)
        {
            vtWeldParam = vvtWeldParam[i];
            bFound = true;
            break;
        }
    }
    return bFound;
}
bool CWeldParamProcess::LoadWeldParam(int nRobotNo, bool bIsFlat, int nWeldAngleSize, std::vector<T_WELD_PARA>& vtWeldParam)
{
    CString sCurUseParamName, strFlat, strStand;
    strFlat.Format("strWorkpieceFlat");
    strStand.Format("strWorkpieceStand");
    CString sSectionName = (true == bIsFlat ? strFlat : strStand);
    std::vector<std::vector<T_WELD_PARA>>& vvtWeldParam = (true == bIsFlat ? m_vvvtFlatWeldParam[nRobotNo] : m_vvvtStandWeldParam[nRobotNo]);


   // CString sCurUseParamName;
    COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
    Opini.SetSectionName("WeldParam");
    Opini.ReadString(sSectionName, sCurUseParamName);
    //sCurUseParamName.Format("%d", nWeldAngleSize);
    //std::vector<std::vector<T_WELD_PARA>>& vvtWeldParam = (true == bIsFlat ? m_vvtFlatWeldParam : m_vvtStandWeldParam);
    if (nWeldAngleSize > 3) {
        sCurUseParamName.Format("%d", nWeldAngleSize);
    }
    bool bFound = false;
    for (int i = 0; i < vvtWeldParam.size(); i++)
    {
        if (vvtWeldParam[i][0].strWorkPeace == sCurUseParamName)
        {
            vtWeldParam = vvtWeldParam[i];
            bFound = true;
            break;
        }
    }
    for (int i = 0; i < vtWeldParam.size(); i++)
    {
        vtWeldParam[i].nWeldMethod = bIsFlat ? 1 : 0;
    }
    return bFound;
}

bool CWeldParamProcess::LoadWeldParam(bool bIsFlat, int nWeldAngleSize, std::vector<T_WELD_PARA>& vtWeldParam)
{
    CString sCurUseParamName;
    sCurUseParamName.Format("%d", nWeldAngleSize);
    std::vector<std::vector<T_WELD_PARA>>& vvtWeldParam = (true == bIsFlat ? m_vvtFlatWeldParam : m_vvtStandWeldParam);

    bool bFound = false;
    for (int i = 0; i < vvtWeldParam.size(); i++)
    {
        if (vvtWeldParam[i][0].strWorkPeace == sCurUseParamName)
        {
            vtWeldParam = vvtWeldParam[i];
            bFound = true;
            break;
        }
    }
    for (int i = 0; i < vtWeldParam.size(); i++)
    {
        vtWeldParam[i].nWeldMethod = bIsFlat ? 1 : 0;
        memset(&vtWeldParam[i].tWeaveParam, 0, sizeof(vtWeldParam[i].tWeaveParam)); // �ްڶ�
        vtWeldParam[i].tWeaveParam.nType = 1;
        for (int nWeaveNo = 0; nWeaveNo < m_vtWeaveParam.size(); nWeaveNo++)
        {
            if (vtWeldParam[i].nWrapConditionNo == m_vtWeaveParam[nWeaveNo].nNo)
            {
                vtWeldParam[i].tWeaveParam = m_vtWeaveParam[nWeaveNo];
                break;
            }
        }
    }
    return bFound;
}

void CWeldParamProcess::SaveParam()
{
	COPini Opini;
	Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
	Opini.SetSectionName("WeldParam");

    if (false == m_bRobotNoSelect)
    {
        CString strName;
        strName.Format("strWorkpieceFlat");
        Opini.WriteString(strName, m_strWorkPiece);
        m_strWorkPieceFlat[m_nRobotNo] = m_strWorkPiece;
    }
    else
    {
        CString strName;
        strName.Format("strWorkpieceStand");
        Opini.WriteString(strName, m_strWorkPiece);
        m_strWorkPieceStand[m_nRobotNo] = m_strWorkPiece;
    }

}

void CWeldParamProcess::OnBnClickedBtnDelete() // ????????? ɾ�����ղ���
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
    if (false == m_bRobotNoSelect)
    {
        m_vvtFlatWeldParam.clear();
        m_vvtFlatWeldParam = m_vvvtFlatWeldParam[m_nRobotNo];
        for (int i = 0; i < m_vvtFlatWeldParam.size(); i++)
        {
            if (m_strWorkPiece == m_vvtFlatWeldParam.at(i)[0].strWorkPeace)
            {
                if (m_nLayerNo != m_vvtFlatWeldParam[i].size() - 1)
                {
                    XiMessageBox("������һ��������ʼɾ��!");
                    return;
                }
				//���޸�
				if (IDOK == XUI::MesBox::PopOkCancel("�Ƿ�ȷ��ɾ������{0}��{1}�� ƽ�����ղ���", m_strWorkPiece.GetBuffer(), m_nLayerNo + 1))
                //if (IDOK == XiMessageBoxGroup(1, "�Ƿ�ȷ��ɾ�� ����%s ��%d�� ƽ�����ղ���", m_strWorkPiece, m_nLayerNo + 1))
                {
                    m_vvtFlatWeldParam[i].erase(m_vvtFlatWeldParam[i].begin() + m_nLayerNo);
                    if (m_vvtFlatWeldParam[i].size() <= 0)
                    {
                        m_vvtFlatWeldParam.erase(m_vvtFlatWeldParam.begin() + i);
                    }
					//���޸�
					XUI::MesBox::PopOkCancel("ƽ������{0}��{1}�� ����ɾ���ɹ�", m_strWorkPiece.GetBuffer(), m_nLayerNo + 1);
                    //XiMessageBoxGroup(1, "ƽ�� ����%s ��%d�� ����ɾ���ɹ�", m_strWorkPiece, m_nLayerNo + 1);
                    m_bSelectDrawing.ResetContent();
                    for (i = 0; i < m_vvtFlatWeldParam.size(); i++)
                    {
                        m_bSelectDrawing.AddString(m_vvtFlatWeldParam[i][0].strWorkPeace);
                    }
                    m_bSelectDrawing.SetCurSel(0);
                    m_nLayerNo = 0;
                }
                m_vvvtFlatWeldParam[m_nRobotNo] = m_vvtFlatWeldParam;
                break;
            }
        }
    }
    else
    {
        m_vvtStandWeldParam.clear();
        m_vvtStandWeldParam = m_vvvtStandWeldParam[m_nRobotNo];
        for (int i = 0; i < m_vvtStandWeldParam.size(); i++)
        {
            if (m_strWorkPiece == m_vvtStandWeldParam.at(i)[0].strWorkPeace)
            {
                if (m_nLayerNo != m_vvtStandWeldParam[i].size() - 1)
                {
                    XiMessageBox("������һ��������ʼɾ��!");
                    return;
                }
				//���޸�
				if (IDOK == XUI::MesBox::PopOkCancel("�Ƿ�ȷ��ɾ������{0}��{1}���������ղ���", m_strWorkPiece.GetBuffer(), m_nLayerNo + 1))
                //if (IDOK == XiMessageBoxGroup(1, "�Ƿ�ȷ��ɾ�� ����%s ��%d�� �������ղ���", m_strWorkPiece, m_nLayerNo + 1))
                {
                    m_vvtStandWeldParam[i].erase(m_vvtStandWeldParam[i].begin() + m_nLayerNo);
                    if (m_vvtStandWeldParam[i].size() <= 0)
                    {
                        m_vvtStandWeldParam.erase(m_vvtStandWeldParam.begin() + i);
                    }
					//���޸�
					XUI::MesBox::PopOkCancel("��������{0}��{1}������ɾ���ɹ�", m_strWorkPiece.GetBuffer(), m_nLayerNo + 1);
                    //XiMessageBoxGroup(1, "���� ����%s ��%d�� ����ɾ���ɹ�", m_strWorkPiece, m_nLayerNo + 1);
                    m_bSelectDrawing.ResetContent();
                    for (i = 0; i < m_vvtStandWeldParam.size(); i++)
                    {
                        m_bSelectDrawing.AddString(m_vvtStandWeldParam[i][0].strWorkPeace);
                    }
                    m_bSelectDrawing.SetCurSel(0);
                    m_nLayerNo = 0;
                }
                m_vvvtStandWeldParam[m_nRobotNo] = m_vvtStandWeldParam;
                break;
            }
        }
    }
}

void CWeldParamProcess::OnBnClickedBtnEdit()
{
    SaveParam();
}

void CWeldParamProcess::OnBnClickedBtnSave()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	UpdateData(TRUE);
	//T_WELD_SEAM_TYPE_PARA tWeldPara;
	T_WELD_PARA tSeamPare;
    tSeamPare = DownloadPageParam();
	if (!m_bEditOrAdd)//����
	{
		int nIndex1 = m_bSelectDrawing.GetCurSel();
		CString str1;
		m_bSelectDrawing.GetLBText(nIndex1, str1);
        tSeamPare.strWorkPeace = str1;
        tSeamPare.nWeldAngleSize = atoi(str1.GetBuffer());
        if (false == m_bRobotNoSelect)
        {
            m_vvtFlatWeldParam = m_vvvtFlatWeldParam[m_nRobotNo];
            for (int i = 0; i < m_vvtFlatWeldParam.size(); i++)
            {
                if (str1 == m_vvtFlatWeldParam.at(i)[0].strWorkPeace)
                {
                    m_vvtFlatWeldParam.at(i)[m_nLayerNo] = tSeamPare;
                    break;
                }
            }
            m_vvvtFlatWeldParam[m_nRobotNo] = m_vvtFlatWeldParam;
			//���޸�
			XUI::MesBox::PopInfo("ƽ��������{0} ��{1}�� ���ո������", str1.GetBuffer(), m_nLayerNo + 1);
           // XiMessageBoxGroup(1, "ƽ��������%s ��%d�� ���ո������", str1, m_nLayerNo + 1);
        }
        else
        {
            m_vvtStandWeldParam = m_vvvtStandWeldParam[m_nRobotNo];
            for (int i = 0; i < m_vvtStandWeldParam.size(); i++)
            {
                if (str1 == m_vvtStandWeldParam.at(i)[0].strWorkPeace)
                {
                    m_vvtStandWeldParam.at(i)[m_nLayerNo] = tSeamPare;
                    break;
                }
            }
            m_vvvtStandWeldParam[m_nRobotNo] = m_vvtStandWeldParam;
			//���޸�
			XUI::MesBox::PopInfo("ƽ��������{0} ��{1}�� ���ո������", str1.GetBuffer(), m_nLayerNo + 1);
           // XiMessageBoxGroup(1, "����������%s ��%d�� ���ո������", str1, m_nLayerNo + 1);
        }
        m_pRightDC->FillSolidRect(m_cRightClientRect, m_pRightDC->GetBkColor());
        ShowWeldGun(tSeamPare.CrosswiseOffset, tSeamPare.verticalOffset);
	}
	else//���
	{
		if ( ""!= m_strNewWorkpieceName)
		{
            tSeamPare.nWeldAngleSize = atoi(m_strNewWorkpieceName.GetBuffer());
			tSeamPare.strWorkPeace = m_strNewWorkpieceName;
			//���޸�
			XUI::MesBox::PopOkCancel("��ӣ�����{0}����", m_strNewWorkpieceName.GetBuffer());
			//XiMessageBoxGroup(1, "��ӣ����� %s ����",m_strNewWorkpieceName);
			
			int i = 0;
            if (false == m_bRobotNoSelect)
            {
                m_vvtFlatWeldParam = m_vvvtFlatWeldParam[m_nRobotNo];
                for (i = 0; i < m_vvtFlatWeldParam.size(); i++)
                {
                    if (m_strNewWorkpieceName == m_vvtFlatWeldParam.at(i)[0].strWorkPeace)
                    {
                        //if (XiMessageBoxGroup(1, "ƽ��������ͬ���գ�%s �Ƿ����", m_strNewWorkpieceName))
                            //{
                            //    m_vvtFlatWeldParam.at(i)[0] = tSeamPare;
                            //}
                        break;
                    }
                }
                if (i == m_vvtFlatWeldParam.size()) // ����º���
                {
                    m_nLayerNo = 0;
                    tSeamPare.nLayerNo = m_nLayerNo;
                    std::vector<T_WELD_PARA> vtWeldParam;
                    vtWeldParam.clear();
                    vtWeldParam.push_back(tSeamPare);
                    m_vvtFlatWeldParam.push_back(vtWeldParam);
                    m_bSelectDrawing.ResetContent();
                    for (i = 0; i < m_vvtFlatWeldParam.size(); i++)
                    {
                        m_bSelectDrawing.AddString(m_vvtFlatWeldParam[i][0].strWorkPeace);
                    }
                    m_bSelectDrawing.SetCurSel(0);
                }
                else // �������к��� ���ൽ����
                {
                    m_nLayerNo = m_vvtFlatWeldParam[i].size();
                    tSeamPare.nLayerNo = m_nLayerNo;
                    m_vvtFlatWeldParam[i].push_back(tSeamPare);
                    m_tWeldSeamPare = tSeamPare;
                    m_bSelectDrawing.SetCurSel(i);
                }
                m_vvvtFlatWeldParam[m_nRobotNo] = m_vvtFlatWeldParam;
				//���޸�
				XUI::MesBox::PopOkCancel("ƽ������{0}mm��{1}��������ӳɹ�", m_strNewWorkpieceName.GetBuffer(), m_nLayerNo + 1);
               // XiMessageBoxGroup(1, "ƽ�� ����%smm ��%d�� ������ӳɹ�", m_strNewWorkpieceName, m_nLayerNo + 1);
            }
            else
            {
                m_vvtStandWeldParam = m_vvvtStandWeldParam[m_nRobotNo];
                for (i = 0; i < m_vvtStandWeldParam.size(); i++)
                {
                    if (m_strNewWorkpieceName == m_vvtStandWeldParam.at(i)[0].strWorkPeace)
                    {
                        //if (XiMessageBoxGroup(1, "��ǹ������ͬ���գ�%s �Ƿ����", m_strNewWorkpieceName))
                        //{
                        //    m_vvtStandWeldParam.at(i)[0] = tSeamPare;
                        //}
                        break;
                    }
                }
                if (i == m_vvtStandWeldParam.size())
                {
                    m_nLayerNo = 0;
                    tSeamPare.nLayerNo = m_nLayerNo;
                    std::vector<T_WELD_PARA> vtWeldParam;
                    vtWeldParam.clear();
                    vtWeldParam.push_back(tSeamPare);
                    m_vvtStandWeldParam.push_back(vtWeldParam);
                    m_bSelectDrawing.ResetContent();
                    for (i = 0; i < m_vvtStandWeldParam.size(); i++)
                    {
                        m_bSelectDrawing.AddString(m_vvtStandWeldParam[i][0].strWorkPeace);
                    }
                    m_bSelectDrawing.SetCurSel(0);
                }
                else // �������к��� ���ൽ����
                {
                    m_nLayerNo = m_vvtStandWeldParam[i].size();
                    tSeamPare.nLayerNo = m_nLayerNo;
                    m_vvtStandWeldParam[i].push_back(tSeamPare);
                    m_tWeldSeamPare = tSeamPare;
                    m_bSelectDrawing.SetCurSel(i);
                }
                m_vvvtStandWeldParam[m_nRobotNo] = m_vvtStandWeldParam;
				//���޸�
				XUI::MesBox::PopOkCancel("��������{0}mm��{1}��������ӳɹ�", m_strNewWorkpieceName.GetBuffer(), m_nLayerNo + 1);
                //XiMessageBoxGroup(1, "���� ����%smm ��%d�� ������ӳɹ�", m_strNewWorkpieceName, m_nLayerNo + 1);
            }
		}
		else
		{
			//���޸�
			XUI::MesBox::PopOkCancel("�¹�������Ϊ�գ��������ȷ�������ٴβ���");
			//XiMessageBoxGroup(1, "�¹�������Ϊ�գ��������ȷ�������ٴβ���");
		}
	}
}

void CWeldParamProcess::OnBnClickedBtnAdd()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
    //UpdateData(TRUE);
    //m_pRightDC->FillSolidRect(m_cRightClientRect, m_pRightDC->GetBkColor());
    //ShowWeldGun(0, 0);
    //XiMessageBox("11");
    ////ShowWeldGun(10, 10);
    //return;
}

std::vector<T_WELD_SEAM_TYPE_PARA> CWeldParamProcess::WeldSeamTypePara()
{
	return m_vtWeldPara;
}


void CWeldParamProcess::OnBnClickedRadioEdit()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	m_bEditOrAdd = false;
	//���޸�
	CString str = "����";
	str = XUI::Languge::GetInstance().translate(str.GetBuffer());
	GetDlgItem(IDC_BTN_SAVE)->SetWindowText(str);
	//GetDlgItem(IDC_BTN_SAVE)->SetWindowText("����");
	((CStatic*)GetDlgItem(IDC_STATIC_NEW))->ShowWindow(FALSE);
	((CEdit*)GetDlgItem(IDC_EDIT_NEW_WOEKPIECE))->ShowWindow(FALSE);

}


void CWeldParamProcess::OnBnClickedRadioAdd()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	m_bEditOrAdd = true;
	//���޸�
	CString str = "���";
	str = XUI::Languge::GetInstance().translate(str.GetBuffer());
	GetDlgItem(IDC_BTN_SAVE)->SetWindowText(str);
	//GetDlgItem(IDC_BTN_SAVE)->SetWindowText("���");
	((CStatic*)GetDlgItem(IDC_STATIC_NEW))->ShowWindow(TRUE);
	((CEdit*)GetDlgItem(IDC_EDIT_NEW_WOEKPIECE))->ShowWindow(TRUE);
}


void CWeldParamProcess::LoadAttributeParam()
{
	COPini Opini;
	Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
	Opini.SetSectionName("WeldParam");
	Opini.ReadString("StartPointScanDis", &m_dStartPointScanDis);
}

//void CWeldParamProcess::LoadWeldParam(std::vector<T_WELD_PARA> &vtWeldParamLeft,std::vector<T_WELD_PARA> &vtWeldParamRight)
//{
//    vtWeldParamLeft.clear();
//    vtWeldParamRight.clear();
//
//    T_WELD_PARA tWeldPara;
//    int nTotalNum = 0;
//    int nTotalNumRight = 0;
//    FILE *ReadParam;
//    FILE *ReadParamRight;
//    if (0 == CheckFileExists("Data\\WeldParamLeft.txt",true))
//    {
//        XiMessageBoxGroup(1, "Data\\WeldParamLeft.txt�ļ�������,����");
//    }
//    ReadParam = fopen("Data\\WeldParamLeft.txt", "r");  
//    fscanf(ReadParam, "TotalNum:%d", &nTotalNum);   
//    for (int i = 0; i < nTotalNum; i++)
//    {
//        char nName[100];
//        fscanf(ReadParam, "%s\n", nName, 100);
//        fscanf(ReadParam, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d", &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage,&tWeldPara.dStartWaitTime,
//            &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage,&tWeldPara.WeldVelocity,&tWeldPara.dStopArcCurrent,&tWeldPara.dStopArcVoltage,&tWeldPara.dStopWaitTime,
//            &tWeldPara.dWrapCurrentt1,&tWeldPara.dWrapVoltage1,&tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
//            &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3,&tWeldPara.CrosswiseOffset,&tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo);
//        tWeldPara.strWorkPeace = nName;
//        vtWeldParamLeft.push_back(tWeldPara);
//    }
//    if (0 == CheckFileExists("Data\\WeldParamRight.txt", true))
//    {
//        XiMessageBoxGroup(1, "Data\\WeldParamRight.txt�ļ�������,����");
//    }
//    ReadParamRight = fopen("Data\\WeldParamRight.txt", "r");
//    fscanf(ReadParamRight, "TotalNum:%d", &nTotalNumRight);
//    for (int i = 0; i < nTotalNumRight; i++)
//    {
//        char nName[100];
//        fscanf(ReadParamRight, "%s\n", nName, 100);
//        fscanf(ReadParamRight, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d", &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage, &tWeldPara.dStartWaitTime,
//            &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage, &tWeldPara.WeldVelocity, &tWeldPara.dStopArcCurrent, &tWeldPara.dStopArcVoltage, &tWeldPara.dStopWaitTime,
//            &tWeldPara.dWrapCurrentt1, &tWeldPara.dWrapVoltage1, &tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
//            &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3, &tWeldPara.CrosswiseOffset, &tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo);
//        tWeldPara.strWorkPeace = nName;
//        vtWeldParamRight.push_back(tWeldPara);
//    }
//    fclose(ReadParam);
//    fclose(ReadParamRight);
//}

void CWeldParamProcess::LoadWeldParam(std::vector<std::vector<T_WELD_PARA>>& vvtFlatWeldParam, std::vector<std::vector<T_WELD_PARA>>& vtStandWeldParam)
{
    vvtFlatWeldParam.clear();
    vtStandWeldParam.clear(); 
    std::vector<T_WELD_PARA> vtWeldParam;
    T_WELD_PARA tWeldPara;

    // ƽ������
    CString sFileName = DATA_PATH + m_sUnitName + WELD_PARAM_FLAT_FILE;
    if (0 == CheckFileExists(sFileName, true))
    {
        XUI::MesBox::PopError("{0}�ļ�������,����", sFileName.GetBuffer());
    }
    FILE* ReadFlatWeldParam;
    ReadFlatWeldParam = fopen(sFileName.GetBuffer(), "r");
    int nPreWeldAngleSize = -1;
    int nReadRst = -1;
    int nDataNum = 26; // ��������
    while (EOF != (nReadRst = fscanf(ReadFlatWeldParam, "%d%d%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %d",
        &tWeldPara.nWeldAngleSize, &tWeldPara.nLayerNo, &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage, &tWeldPara.dStartWaitTime,
        &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage, &tWeldPara.WeldVelocity, &tWeldPara.dStopArcCurrent, &tWeldPara.dStopArcVoltage, &tWeldPara.dStopWaitTime,
        &tWeldPara.dWrapCurrentt1, &tWeldPara.dWrapVoltage1, &tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
        &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3, &tWeldPara.CrosswiseOffset, &tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo,
		&tWeldPara.dWeldAngle, &tWeldPara.dWeldDipAngle, &tWeldPara.nStandWeldDir)))
    {
        if (nDataNum != nReadRst)
        {
            XiMessageBox("��⵽������ƽ�����ղ���������ʧ�ܣ�");
            return;
        }
        tWeldPara.strWorkPeace.Format("%d", tWeldPara.nWeldAngleSize);
        if (tWeldPara.nWeldAngleSize != nPreWeldAngleSize) // �º��Ų���
        {
            if (nPreWeldAngleSize > 0) // ��֤��ȡ��һ��ʱ����PushBackһ��������
            {
                vvtFlatWeldParam.push_back(vtWeldParam);
            }
            vtWeldParam.clear();
            vtWeldParam.push_back(tWeldPara);
            nPreWeldAngleSize = tWeldPara.nWeldAngleSize;
        }
        else
        {
            vtWeldParam.push_back(tWeldPara);
        }
    }
    vvtFlatWeldParam.push_back(vtWeldParam); // ��¼��������
    fclose(ReadFlatWeldParam);

    // �������ղ���
    sFileName = DATA_PATH + m_sUnitName + WELD_PARAM_STAND_FILE;
    if (0 == CheckFileExists(sFileName, true))
    {
        XUI::MesBox::PopError("{0}�ļ�������,����", sFileName.GetBuffer());
    }
    FILE* ReadStandWeldParam;
    ReadStandWeldParam = fopen(sFileName.GetBuffer(), "r");
    nPreWeldAngleSize = -1;
    nReadRst = -1;
    while (EOF != (nReadRst = fscanf(ReadStandWeldParam, "%d%d%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %d",
        &tWeldPara.nWeldAngleSize, &tWeldPara.nLayerNo, &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage, &tWeldPara.dStartWaitTime,
        &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage, &tWeldPara.WeldVelocity, &tWeldPara.dStopArcCurrent, &tWeldPara.dStopArcVoltage, &tWeldPara.dStopWaitTime,
        &tWeldPara.dWrapCurrentt1, &tWeldPara.dWrapVoltage1, &tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
        &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3, &tWeldPara.CrosswiseOffset, &tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo,
		&tWeldPara.dWeldAngle, &tWeldPara.dWeldDipAngle, &tWeldPara.nStandWeldDir)))
    {
		if (nDataNum != nReadRst)
		{
			XiMessageBox("��⵽�������������ղ���������ʧ�ܣ�");
			return;
		}
        tWeldPara.strWorkPeace.Format("%d", tWeldPara.nWeldAngleSize);
        if (tWeldPara.nWeldAngleSize != nPreWeldAngleSize) // �º��Ų���
        {
            if (nPreWeldAngleSize > 0) // ��֤��ȡ��һ��ʱ����PushBackһ��������
            {
                vtStandWeldParam.push_back(vtWeldParam);
            }
            vtWeldParam.clear();
            vtWeldParam.push_back(tWeldPara);
            nPreWeldAngleSize = tWeldPara.nWeldAngleSize;
        }
        else
        {
            vtWeldParam.push_back(tWeldPara);
        }
    }
    vtStandWeldParam.push_back(vtWeldParam); // ��¼��������
    fclose(ReadStandWeldParam);
}

void CWeldParamProcess::LoadWeldParam(std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtFlatWeldParam,
    std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtStandWeldParam)
{
    vvvtFlatWeldParam.clear();
    vvvtStandWeldParam.clear();
    std::vector<std::vector<T_WELD_PARA>> vvtFlatWeldParam;
    std::vector<std::vector<T_WELD_PARA>> vvtStandWeldParam;
    std::vector<T_WELD_PARA> vtWeldParam;
    T_WELD_PARA tWeldPara;

    for (int nRobotNo = 0; nRobotNo < m_nRobotNum; nRobotNo++)
    {
        vvtFlatWeldParam.clear();
        vvtStandWeldParam.clear();
        CString sFileName = DATA_PATH + m_vpUnit[nRobotNo]->GetRobotCtrl()->m_strRobotName + WELD_PARAM_FLAT_FILE;
        if (0 == CheckFileExists(sFileName, true))
        {
            XUI::MesBox::PopError("{0}�ļ�������,����", sFileName.GetBuffer());
        }
        FILE* ReadFlatWeldParam;
        ReadFlatWeldParam = fopen(sFileName.GetBuffer(), "r");
        int nPreWeldAngleSize = -1;
        int nReadRst = -1;
        int nDataNum = 26; // ��������
        while (EOF != (nReadRst = fscanf(ReadFlatWeldParam, "%d%d%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %d",
            &tWeldPara.nWeldAngleSize, &tWeldPara.nLayerNo, &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage, &tWeldPara.dStartWaitTime,
            &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage, &tWeldPara.WeldVelocity, &tWeldPara.dStopArcCurrent, &tWeldPara.dStopArcVoltage, &tWeldPara.dStopWaitTime,
            &tWeldPara.dWrapCurrentt1, &tWeldPara.dWrapVoltage1, &tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
            &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3, &tWeldPara.CrosswiseOffset, &tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo,
            &tWeldPara.dWeldAngle, &tWeldPara.dWeldDipAngle, &tWeldPara.nStandWeldDir)))
        {
            if (nDataNum != nReadRst)
            {
                XiMessageBox("��⵽������ƽ�����ղ���������ʧ�ܣ�");
                return;
            }
            tWeldPara.strWorkPeace.Format("%d", tWeldPara.nWeldAngleSize);
            if (tWeldPara.nWeldAngleSize != nPreWeldAngleSize) // �º��Ų���
            {
                if (nPreWeldAngleSize > 0) // ��֤��ȡ��һ��ʱ����PushBackһ��������
                {
                    vvtFlatWeldParam.push_back(vtWeldParam);
                }
                vtWeldParam.clear();
                vtWeldParam.push_back(tWeldPara);
                nPreWeldAngleSize = tWeldPara.nWeldAngleSize;
            }
            else
            {
                vtWeldParam.push_back(tWeldPara);
            }
        }
        vvtFlatWeldParam.push_back(vtWeldParam); // ��¼��������
        vvvtFlatWeldParam.push_back(vvtFlatWeldParam);
        fclose(ReadFlatWeldParam);

        // �������ղ���
        CString strStandFile;
        strStandFile = DATA_PATH + m_vpUnit[nRobotNo]->GetRobotCtrl()->m_strRobotName + WELD_PARAM_STAND_FILE;
        if (0 == CheckFileExists(strStandFile, true))
        {
            XUI::MesBox::PopError("{0}�ļ�������,����", strStandFile.GetBuffer());
        }
        FILE* ReadStandWeldParam;
        ReadStandWeldParam = fopen(strStandFile.GetBuffer(), "r");
        nPreWeldAngleSize = -1;
        nReadRst = -1;
        while (EOF != (nReadRst = fscanf(ReadStandWeldParam, "%d%d%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %d",
            &tWeldPara.nWeldAngleSize, &tWeldPara.nLayerNo, &tWeldPara.dStartArcCurrent, &tWeldPara.dStartArcVoltage, &tWeldPara.dStartWaitTime,
            &tWeldPara.dTrackCurrent, &tWeldPara.dTrackVoltage, &tWeldPara.WeldVelocity, &tWeldPara.dStopArcCurrent, &tWeldPara.dStopArcVoltage, &tWeldPara.dStopWaitTime,
            &tWeldPara.dWrapCurrentt1, &tWeldPara.dWrapVoltage1, &tWeldPara.dWrapWaitTime1, &tWeldPara.dWrapCurrentt2, &tWeldPara.dWrapVoltage2, &tWeldPara.dWrapWaitTime2,
            &tWeldPara.dWrapCurrentt3, &tWeldPara.dWrapVoltage3, &tWeldPara.dWrapWaitTime3, &tWeldPara.CrosswiseOffset, &tWeldPara.verticalOffset, &tWeldPara.nWrapConditionNo,
            &tWeldPara.dWeldAngle, &tWeldPara.dWeldDipAngle, &tWeldPara.nStandWeldDir)))
        {
            if (nDataNum != nReadRst)
            {
                XiMessageBox("��⵽�������������ղ���������ʧ�ܣ�");
                return;
            }
            tWeldPara.strWorkPeace.Format("%d", tWeldPara.nWeldAngleSize);
            if (tWeldPara.nWeldAngleSize != nPreWeldAngleSize) // �º��Ų���
            {
                if (nPreWeldAngleSize > 0) // ��֤��ȡ��һ��ʱ����PushBackһ��������
                {
                    vvtStandWeldParam.push_back(vtWeldParam);
                }
                vtWeldParam.clear();
                vtWeldParam.push_back(tWeldPara);
                nPreWeldAngleSize = tWeldPara.nWeldAngleSize;
            }
            else
            {
                vtWeldParam.push_back(tWeldPara);
            }
        }
        vvtStandWeldParam.push_back(vtWeldParam); // ��¼��������
        vvvtStandWeldParam.push_back(vvtStandWeldParam);
        fclose(ReadStandWeldParam);
    }
}

void CWeldParamProcess::LoadWeaveParam()
{
    T_WEAVE_PARA tWeaveParam;
    m_vtWeaveParam.clear();
    CString sFileName = DATA_PATH + "RobotA\\WEAV.CND";
    if (!CheckFileExists(sFileName))
    {
        return;
    }
    FILE* pf = fopen(sFileName, "r");

    CString sTemp;
    int nTemp;
    double dTemp;
    double dAmp1, dAmp2, dAmp3;
    while (EOF != fscanf(pf, "%s%d %d,%d,%d %s %lf,%lf,%lf,%lf,%lf,%lf %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf %d,%lf,%d,%lf,%d,%lf\n",
        sTemp, &tWeaveParam.nNo,
        &tWeaveParam.nType, &nTemp, &nTemp,
        sTemp,
        &dAmp1, &dAmp2, &tWeaveParam.dRotAngleX, &tWeaveParam.dRotAngleZ, &dAmp3, &tWeaveParam.dFreq,
        &dTemp, &tWeaveParam.dTimeL, &dTemp, &tWeaveParam.dTimeC, &dTemp, &tWeaveParam.dTimeR, &dTemp, &dTemp,
        &nTemp, &dTemp, &nTemp, &dTemp, &nTemp, &dTemp))
    {

        //tWeaveParam.nType = 1;
        tWeaveParam.nType += 1; // ��һ�� ���ǰ� L��   ����012 ��˹��123
        tWeaveParam.nType = tWeaveParam.nType > 3 ? 1 : tWeaveParam.nType; // ��ֻ֤�����ְڶ���ʽ
        tWeaveParam.nType = tWeaveParam.nType < 1 ? 1 : tWeaveParam.nType; // ��ֻ֤�����ְڶ���ʽ
        tWeaveParam.dAmpL = 1 == tWeaveParam.nType ? dAmp3 : dAmp1;
        tWeaveParam.dAmpR = 1 == tWeaveParam.nType ? dAmp3 : dAmp2;
        tWeaveParam.nDelateTypeL = 1;
        tWeaveParam.nDelateTypeC = 1;
        tWeaveParam.nDelateTypeR = 1;
        tWeaveParam.dRotAngleX = 0.0;
        tWeaveParam.dRotAngleZ = 0.0;
        tWeaveParam.dTimeC = 0.0;
        m_vtWeaveParam.push_back(tWeaveParam);
    }
    fclose(pf);
}

void CWeldParamProcess::WeaveParamDialog()
{
    //��ǰ�ڶ��ļ���
    int curWaveNum = m_nWrapCondition;
    std::vector<double> curWeaveParam;
    std::vector<CString> curWeaveParamName;
    curWeaveParam.clear();
    curWeaveParamName.clear();
    //T_WEAVE_PARA t = LoadCurWeaveParam(curWeaveParam, curWeaveParamName, curWaveNum);
    T_WEAVE_FULLPARA t = LoadCurWeaveParam(curWeaveParam, curWeaveParamName, curWaveNum);
    // �޸�ʶ���������
	//���޸�
	CString title;
	title = XUI::Languge::GetInstance().translate("�ں�����");
	
    ParamInput cParamDlg("�ں�����", curWeaveParamName, &curWeaveParam);
    int nRst = cParamDlg.DoModal();
    if (1 != nRst)
    {
        return;
    }
    t.nNo = curWeaveParam[0];
    t.n1 = curWeaveParam[1];
    t.dFreq = curWeaveParam[2];
    t.dAmpL = curWeaveParam[3];
    t.dAmpR = curWeaveParam[4];
    t.dTimeL = curWeaveParam[5];
    t.dTimeC = curWeaveParam[6];
    t.dTimeR = curWeaveParam[7];
    t.dRotAngleX = curWeaveParam[8];
    t.dRotAngleZ = curWeaveParam[9];

    SaveWeaveParam(t, curWaveNum);
    LoadWeaveParam(); // ˢ��
    return;
}

void CWeldParamProcess::SaveWeaveParam(T_WEAVE_FULLPARA t, int curWaveNum)
{
    m_vtWeaveFullParam[curWaveNum - 1] = t;
    CString sFileName = DATA_PATH + "RobotA\\WEAV.CND";
    if (!CheckFileExists(sFileName))
    {
        return;
    }
    for (int i = 0; i < m_vtWeaveFullParam.size(); i++)
    {
        if (m_vtWeaveFullParam[i].nNo == curWaveNum)
        {
            m_vtWeaveFullParam[i] = t;
       
        }
    }

    FILE *pf = fopen(sFileName, "w"); 

    for (int i = 0; i < m_vtWeaveFullParam.size(); i++)
    {
        fprintf(pf, "//%s %d\n%d,%d,%d\n%s\n%.3lf,%.3lf,%.2lf,%.2lf,%.3lf,%.1lf\n%.1lf,%.1lf,%.1lf,%.1lf,%.1lf,%.1lf,%.1lf,%.1lf\n%d,%.1lf,%d,%.2lf,%d,%.3lf\n",
            "WEAV", m_vtWeaveFullParam[i].nNo,
            m_vtWeaveFullParam[i].n1, m_vtWeaveFullParam[i].n2, m_vtWeaveFullParam[i].n3,
            m_vtWeaveFullParam[i].s8bit,
            m_vtWeaveFullParam[i].dAmpL, m_vtWeaveFullParam[i].dAmpR, m_vtWeaveFullParam[i].dRotAngleX, m_vtWeaveFullParam[i].dRotAngleZ, m_vtWeaveFullParam[i].dAmpL, m_vtWeaveFullParam[i].dFreq,
            m_vtWeaveFullParam[i].d3, m_vtWeaveFullParam[i].dTimeL, m_vtWeaveFullParam[i].d4, m_vtWeaveFullParam[i].dTimeC, m_vtWeaveFullParam[i].d5, m_vtWeaveFullParam[i].dTimeR, m_vtWeaveFullParam[i].d6, m_vtWeaveFullParam[i].d7,
            m_vtWeaveFullParam[i].n4, m_vtWeaveFullParam[i].d8, m_vtWeaveFullParam[i].n5, m_vtWeaveFullParam[i].d9, m_vtWeaveFullParam[i].n6, m_vtWeaveFullParam[i].d10);
    }
    fclose(pf); 
    return;
}

T_WEAVE_FULLPARA CWeldParamProcess::LoadCurWeaveParam(std::vector<double>& curWeaveParam, std::vector<CString>& curWeaveParamName, int curWaveNum)
{
//    CRobotDriverAdaptor* pRobotCtrl;
    T_WEAVE_FULLPARA tReturnParam;
    tReturnParam.nNo = 0;
    tReturnParam.n1 = 0;
    tReturnParam.dFreq = 0;
    tReturnParam.dAmpL = 0;
    tReturnParam.dAmpR = 0;
    tReturnParam.dTimeL = 0;
    tReturnParam.dTimeC = 0;
    tReturnParam.dTimeR = 0;
    tReturnParam.dRotAngleX = 0;
    tReturnParam.dRotAngleZ = 0;

    //CString sFileName = "ConfigFiles\\" + pRobotCtrl->m_strRobotName + "\\WEAV.CND";
    CString sFileName = DATA_PATH + "RobotA\\WEAV.CND";
    if (!CheckFileExists(sFileName))
    {
        return tReturnParam;
    }
    m_vtWeaveFullParam.clear();
    FILE* pf = fopen(sFileName, "r");
    T_WEAVE_FULLPARA tWeaveFullParam;
    while (~fscanf(pf, "%s%d %d,%d,%d %s %lf,%lf,%lf,%lf,%lf,%lf %lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf %d,%lf,%d,%lf,%d,%lf\n",
        tWeaveFullParam.sWave, &tWeaveFullParam.nNo,
        &tWeaveFullParam.n1, &tWeaveFullParam.n2, &tWeaveFullParam.n3,
        tWeaveFullParam.s8bit,
        &tWeaveFullParam.d1, &tWeaveFullParam.dAmpR, &tWeaveFullParam.dRotAngleX, &tWeaveFullParam.dRotAngleZ, &tWeaveFullParam.dAmpL, &tWeaveFullParam.dFreq,
        &tWeaveFullParam.d3, &tWeaveFullParam.dTimeL, &tWeaveFullParam.d4, &tWeaveFullParam.dTimeC, &tWeaveFullParam.d5, &tWeaveFullParam.dTimeR, &tWeaveFullParam.d6, &tWeaveFullParam.d7,
        &tWeaveFullParam.n4, &tWeaveFullParam.d8, &tWeaveFullParam.n5, &tWeaveFullParam.d9, &tWeaveFullParam.n6, &tWeaveFullParam.d10))
    {
        if (tWeaveFullParam.nNo == curWaveNum)
        {
            tReturnParam = tWeaveFullParam;
        }
        m_vtWeaveFullParam.push_back(tWeaveFullParam);
    }
    fclose(pf);


    curWeaveParam.push_back(tReturnParam.nNo);
    curWeaveParam.push_back(tReturnParam.n1); // �ڶ���ʽ
    curWeaveParam.push_back(tReturnParam.dFreq);
    curWeaveParam.push_back(tReturnParam.dAmpL);
    curWeaveParam.push_back(tReturnParam.dAmpR);
    curWeaveParam.push_back(tReturnParam.dTimeL);
    curWeaveParam.push_back(tReturnParam.dTimeC);
    curWeaveParam.push_back(tReturnParam.dTimeR);
    curWeaveParam.push_back(tReturnParam.dRotAngleX);
    curWeaveParam.push_back(tReturnParam.dRotAngleZ);


    curWeaveParamName.push_back("�ڶ��ļ���");
    curWeaveParamName.push_back("��ʽ 0:��һ�� 1:���ǰ� 2:L��");
    curWeaveParamName.push_back("Ƶ�� Hz");
    curWeaveParamName.push_back("���");
    curWeaveParamName.push_back("���ݾ���");
    curWeaveParamName.push_back("ֹͣʱ��1 ��λs");
    curWeaveParamName.push_back("ֹͣʱ��2,4 ��λs");
    curWeaveParamName.push_back("ֹͣʱ��3 ��λs");
    curWeaveParamName.push_back("�ڶ��Ƕ�");
    curWeaveParamName.push_back("�н��Ƕ�");


    return tReturnParam;
}

void CWeldParamProcess::SaveWeldParam(int nRobotNo)
{
    std::vector<std::vector<T_WELD_PARA>> *pvvtWeldParam;
    T_WELD_PARA tWeldPara;
    //int nTotalNum;
    FILE *ReadParam;
    if (0 == nRobotNo)
    {
        CString sFileName = DATA_PATH + m_sUnitName + WELD_PARAM_FLAT_FILE;
        ReadParam = fopen(sFileName.GetBuffer(), "w");
        //nTotalNum = m_vtWeldParamLeft.size();
        pvvtWeldParam = &m_vvtFlatWeldParam;
    }
    else
    {
        CString sFileName = DATA_PATH + m_sUnitName + WELD_PARAM_STAND_FILE;
        ReadParam = fopen(sFileName.GetBuffer(), "w");
        //nTotalNum = m_vtWeldParamRight.size();
        pvvtWeldParam = &m_vvtStandWeldParam;
    }
    //fprintf(ReadParam, "TotalNum:%d\n", nTotalNum);
    //��������
    std::sort(pvvtWeldParam->begin(), pvvtWeldParam->end(), SortWeldParaNew);
    for (int i = 0; i < pvvtWeldParam->size(); i++)
    {
        std::vector<T_WELD_PARA>& vtWeldParam = pvvtWeldParam->at(i);
        std::sort(vtWeldParam.begin(), vtWeldParam.end(),
            [](const T_WELD_PARA& tPara1, const T_WELD_PARA& tPara2) -> bool
            {
                return (tPara1.nLayerNo < tPara2.nLayerNo);
            });

        for (int nNo = 0; nNo < vtWeldParam.size(); nNo++)
        {
            tWeldPara = vtWeldParam.at(nNo);
			fprintf(ReadParam, "%d %d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %d %.3lf %.3lf %d\n",
				tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo,
				tWeldPara.dStartArcCurrent, tWeldPara.dStartArcVoltage, tWeldPara.dStartWaitTime, tWeldPara.dTrackCurrent, tWeldPara.dTrackVoltage, tWeldPara.WeldVelocity,
				tWeldPara.dStopArcCurrent, tWeldPara.dStopArcVoltage, tWeldPara.dStopWaitTime, tWeldPara.dWrapCurrentt1, tWeldPara.dWrapVoltage1, tWeldPara.dWrapWaitTime1,
				tWeldPara.dWrapCurrentt2, tWeldPara.dWrapVoltage2, tWeldPara.dWrapWaitTime2, tWeldPara.dWrapCurrentt3, tWeldPara.dWrapVoltage3, tWeldPara.dWrapWaitTime3,
				tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset, tWeldPara.nWrapConditionNo, tWeldPara.dWeldAngle, tWeldPara.dWeldDipAngle, tWeldPara.nStandWeldDir);
        }
    }
    fclose(ReadParam);
}

void CWeldParamProcess::SaveWeldParam()
{
    std::vector<std::vector<T_WELD_PARA>>* pvvtWeldParam, *pvvtWeldParamStand;
    T_WELD_PARA tWeldPara;
    CString strFlat, strStand;
    for (int i = 0; i < m_nRobotNum; i++)
    {
        FILE* ReadParam, *ReadParamStand;
        strFlat= DATA_PATH + m_vpUnit[i]->GetRobotCtrl()->m_strRobotName + WELD_PARAM_FLAT_FILE;
        ReadParam = fopen(strFlat, "w");
        pvvtWeldParam = &m_vvvtFlatWeldParam[i];
        strStand = DATA_PATH + m_vpUnit[i]->GetRobotCtrl()->m_strRobotName + WELD_PARAM_STAND_FILE;
        ReadParamStand = fopen(strStand, "w");
        pvvtWeldParamStand = &m_vvvtStandWeldParam[i];
        //��������
        std::sort(pvvtWeldParam->begin(), pvvtWeldParam->end(), SortWeldParaNew);

        std::sort(pvvtWeldParamStand->begin(), pvvtWeldParamStand->end(), SortWeldParaNew);

        for (int i = 0; i < pvvtWeldParam->size(); i++)
        {
            std::vector<T_WELD_PARA>& vtWeldParam = pvvtWeldParam->at(i);
            std::sort(vtWeldParam.begin(), vtWeldParam.end(),
                [](const T_WELD_PARA& tPara1, const T_WELD_PARA& tPara2) -> bool
                {
                    return (tPara1.nLayerNo < tPara2.nLayerNo);
                });

            for (int nNo = 0; nNo < vtWeldParam.size(); nNo++)
            {
                tWeldPara = vtWeldParam.at(nNo);
                fprintf(ReadParam, "%d %d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %d %.3lf %.3lf %d\n",
                    tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo,
                    tWeldPara.dStartArcCurrent, tWeldPara.dStartArcVoltage, tWeldPara.dStartWaitTime, tWeldPara.dTrackCurrent, tWeldPara.dTrackVoltage, tWeldPara.WeldVelocity,
                    tWeldPara.dStopArcCurrent, tWeldPara.dStopArcVoltage, tWeldPara.dStopWaitTime, tWeldPara.dWrapCurrentt1, tWeldPara.dWrapVoltage1, tWeldPara.dWrapWaitTime1,
                    tWeldPara.dWrapCurrentt2, tWeldPara.dWrapVoltage2, tWeldPara.dWrapWaitTime2, tWeldPara.dWrapCurrentt3, tWeldPara.dWrapVoltage3, tWeldPara.dWrapWaitTime3,
                    tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset, tWeldPara.nWrapConditionNo, tWeldPara.dWeldAngle, tWeldPara.dWeldDipAngle, tWeldPara.nStandWeldDir);
            }
        }
        fclose(ReadParam);

        for (int i = 0; i < pvvtWeldParamStand->size(); i++)
        {
            std::vector<T_WELD_PARA>& vtWeldParam = pvvtWeldParamStand->at(i);
            std::sort(vtWeldParam.begin(), vtWeldParam.end(),
                [](const T_WELD_PARA& tPara1, const T_WELD_PARA& tPara2) -> bool
                {
                    return (tPara1.nLayerNo < tPara2.nLayerNo);
                });

            for (int nNo = 0; nNo < vtWeldParam.size(); nNo++)
            {
                tWeldPara = vtWeldParam.at(nNo);
                fprintf(ReadParamStand, "%d %d %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %.3lf %d %.3lf %.3lf %d\n",
                    tWeldPara.nWeldAngleSize, tWeldPara.nLayerNo,
                    tWeldPara.dStartArcCurrent, tWeldPara.dStartArcVoltage, tWeldPara.dStartWaitTime, tWeldPara.dTrackCurrent, tWeldPara.dTrackVoltage, tWeldPara.WeldVelocity,
                    tWeldPara.dStopArcCurrent, tWeldPara.dStopArcVoltage, tWeldPara.dStopWaitTime, tWeldPara.dWrapCurrentt1, tWeldPara.dWrapVoltage1, tWeldPara.dWrapWaitTime1,
                    tWeldPara.dWrapCurrentt2, tWeldPara.dWrapVoltage2, tWeldPara.dWrapWaitTime2, tWeldPara.dWrapCurrentt3, tWeldPara.dWrapVoltage3, tWeldPara.dWrapWaitTime3,
                    tWeldPara.CrosswiseOffset, tWeldPara.verticalOffset, tWeldPara.nWrapConditionNo, tWeldPara.dWeldAngle, tWeldPara.dWeldDipAngle, tWeldPara.nStandWeldDir);
            }
        }
        fclose(ReadParamStand);
    }
    
}

bool CWeldParamProcess::SortWeldPara(T_WELD_PARA a, T_WELD_PARA b)
{
    double dA = atof(a.strWorkPeace);
    double dB = atof(b.strWorkPeace);
    return dA < dB;
}

bool CWeldParamProcess::SortWeldParaNew(std::vector<T_WELD_PARA>& a, std::vector<T_WELD_PARA>& b)
{
    if (a.size() <= 0 || b.size() <= 0)
    {
        return false;
    }
    double dA = atof(a[0].strWorkPeace);
    double dB = atof(b[0].strWorkPeace);
    return dA < dB;
}

void CWeldParamProcess::OnBnClickedCheckRobotno()
{
    UpdateData(TRUE);
    std::vector< std::vector<T_WELD_PARA>> *pvvtWeldParam;
    CString m_str;
	//���޸�
	CString str1 = "�������ղ���";
	str1 = XUI::Languge::GetInstance().translate(str1.GetBuffer());
	CString str2 = "ƽ�����ղ���";
	str2 = XUI::Languge::GetInstance().translate(str2.GetBuffer());
    if (m_bRobotNoSelect)
    {
		//���޸�
        GetDlgItem(IDC_CHECK_ROBOTNO)->SetWindowText("str1"); 
        pvvtWeldParam = &m_vvvtStandWeldParam[m_nRobotNo];
        m_str = m_strWorkPieceStand[m_nRobotNo];
        
    }
    else
    {
		//���޸�
        GetDlgItem(IDC_CHECK_ROBOTNO)->SetWindowText("str2");
        pvvtWeldParam = &m_vvvtFlatWeldParam[m_nRobotNo];
        m_str = m_strWorkPieceFlat[m_nRobotNo];
    }
    int nSetNo = 0;
    //ˢ�½���
    m_bSelectDrawing.ResetContent();
    int i = 0;
    for (i = 0; i < pvvtWeldParam->size(); i++)
    {
        m_bSelectDrawing.AddString(pvvtWeldParam->at(i)[0].strWorkPeace);
        if (m_str == pvvtWeldParam->at(i)[0].strWorkPeace)
        {
            nSetNo = i;
        }
    }
    m_bSelectDrawing.SetCurSel(nSetNo);
    OnCbnSelchangeBoxSelectDrawinglist();
}

void CWeldParamProcess::LoadWeldParam(int nRobotNo,double dThink, T_WELD_PARA &tWeldParam,bool ioethick)
{
	if (ioethick==true)
	{
		if (dThink <= 6 && dThink > 3)
		{
			dThink = 4;
		}
		else if (dThink <= 8)
		{
			dThink = 5;
		}
		else if (dThink <= 13)
		{
			dThink = 6;
		}
		else if (dThink <= 15)
		{
			dThink = 7;
		}
		else if (dThink <= 20)
		{
			dThink = 8;
		}
		else
		{
            XUI::MesBox::PopOkCancel("�������������飺{0} ", dThink);
			dThink = 6;
		}
	}
	WriteLog("��������Ϊ :%lf", dThink);
    int i = 0;
    if (0 == nRobotNo)
    {      
        for (i = 0; i < m_vvvtFlatWeldParam[nRobotNo].size(); i++)
        {
            double dThk = atof(m_vvvtFlatWeldParam[nRobotNo][i][0].strWorkPeace);
            if (fabs(dThink- dThk)<0.5)
            {
                tWeldParam = m_vvvtFlatWeldParam[nRobotNo][i][0];
            }
        }
    }
    else
    {
        for (i = 0; i < m_vvvtStandWeldParam[nRobotNo].size(); i++)
        {
            double dThk = atof(m_vvvtStandWeldParam[nRobotNo][i][0].strWorkPeace);
            if (fabs(dThink - dThk)<0.5)
            {
                tWeldParam = m_vvvtStandWeldParam[nRobotNo][i][0];
            }
        }
    }
}

void CWeldParamProcess::InitWeldPage()
{  
    m_pRightWin = GetDlgItem(IDC_STC_WELD_PARAM);
    m_pRightWin->GetClientRect(m_cRightClientRect);
    m_pRightWin->GetWindowRect(m_cRightWindowRect);
    ScreenToClient(m_cRightWindowRect);
    m_pRightDC = m_pRightWin->GetDC();
    m_pRightDC->Rectangle(m_cRightClientRect);
}

void CWeldParamProcess::ShowWeldGun(double dXAxisAdjust, double dZAxisAdjust)
{
    // ������
    CPoint cWeldSeam;
    cWeldSeam.x = m_cRightClientRect.Width() * 1 / 10;
    cWeldSeam.y = m_cRightClientRect.Height() * 9 / 10;
    DrawingWeldSeam(cWeldSeam, 20, false == m_bRobotNoSelect);

    double dX = (dXAxisAdjust * 10.0); // ƽ����ǹƫ��
    double dZ = -(dZAxisAdjust * 10.0);
    if (TRUE == m_bRobotNoSelect) // ������ǹƫ�� nXAxisAdjust���� nZAxisAdjust��˿��ֱ��ʱ�뷽��
    {
		dX = dXAxisAdjust * 10.0; // ������ǹƫ��
		dZ = dZAxisAdjust * 10.0;
        double d1 = dX * CosD(-45.0) + dZ * CosD(-135.0);
        double d2 = dX * SinD(-45.0) + dZ * SinD(-135.0);
        dX = d1;
        dZ = d2;
    }

    CPoint cGunCoor;
    cGunCoor.x = (int)((double)m_cRightClientRect.Width() * 1.0 / 10.0 + dX * (double)m_cRightClientRect.Width() / 600.0);
    cGunCoor.y = (int)((double)m_cRightClientRect.Height() * 9.0 / 10.0 + dZ * (double)m_cRightClientRect.Width() / 600.0);
    int nWeldWearDistance = (int)(sqrt(SQUARE(m_cRightClientRect.Height()) + SQUARE(m_cRightClientRect.Width())) / 5.0);
    CPen pen(PS_SOLID, 2, RGB(255, 0, 0));
    CPen *oldPen = m_pRightDC->SelectObject(&pen);

    m_pRightDC->MoveTo(cGunCoor);
    cGunCoor.x += nWeldWearDistance;
    cGunCoor.y -= nWeldWearDistance;
    m_pRightDC->LineTo(cGunCoor);
    cGunCoor.x += nWeldWearDistance / 8;
    cGunCoor.y += nWeldWearDistance / 8;
    m_pRightDC->MoveTo(cGunCoor);
    cGunCoor.x -= (nWeldWearDistance / 8) * 2;
    cGunCoor.y -= (nWeldWearDistance / 8) * 2;
    m_pRightDC->LineTo(cGunCoor);
    cGunCoor.x += nWeldWearDistance;
    cGunCoor.y -= nWeldWearDistance;
    m_pRightDC->LineTo(cGunCoor);
    cGunCoor.x += (nWeldWearDistance / 8) * 2;
    cGunCoor.y += (nWeldWearDistance / 8) * 2;
    m_pRightDC->LineTo(cGunCoor);
    cGunCoor.x -= nWeldWearDistance;
    cGunCoor.y += nWeldWearDistance;
    m_pRightDC->LineTo(cGunCoor);
    m_pRightDC->SelectObject(oldPen);
    ShowLeftWeldLine();
    m_pRightWin->UpdateWindow();
}

void CWeldParamProcess::DrawingWeldSeam(CPoint tBaseCenterPtn, int nRadius, bool bIsFlatSeam)
{
    CPen RedPen(PS_SOLID, 2, RGB(255, 0, 0));
    CPen BlackPen(PS_SOLID, 2, RGB(0, 0, 0));
    CPen* oldPen = m_pRightDC->SelectObject(&BlackPen);

    CPoint tCenter;
    std::vector<CPoint> vtCenter;
    std::vector<CPoint> vtStartPtn;
    std::vector<int> vnRadius;
    if (true == bIsFlatSeam)
    {
        tCenter = tBaseCenterPtn;
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y)); 
        vnRadius.push_back(nRadius);
        tCenter = tBaseCenterPtn;
        tCenter.x += nRadius;
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y)); 
        vnRadius.push_back(nRadius);
        tCenter = tBaseCenterPtn;
        tCenter.y -= nRadius;
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y));
        vnRadius.push_back(nRadius);
        tCenter = tBaseCenterPtn;
        tCenter.x += (nRadius * 2);
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y));
        vnRadius.push_back(nRadius);
        tCenter = tBaseCenterPtn;
        tCenter.x += nRadius;
        tCenter.y -= nRadius;
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y));
        vnRadius.push_back(nRadius);
        tCenter = tBaseCenterPtn;
        tCenter.y -= (nRadius * 2);
        vtCenter.push_back(tCenter);
        vtStartPtn.push_back(CPoint(tCenter.x + nRadius, tCenter.y));
        vnRadius.push_back(nRadius);

        for (int i = 0; i < 7; i++) // �������ʾ��ʱ����
        {
            vtCenter.push_back(vtCenter.back());
            vtStartPtn.push_back(vtStartPtn.back());
            vnRadius.push_back(vnRadius.back());
        }

  
    }
    else
    {
        tCenter = tBaseCenterPtn;
        for (int i = 10; i < 150; i += 10)
        {
            vtCenter.push_back(tCenter);
            vtStartPtn.push_back(CPoint(tCenter.x + nRadius + i, tCenter.y));
            vnRadius.push_back(nRadius + i);
        }
        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 10, tCenter.y));
        //vnRadius.push_back(nRadius + 10);

        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 20, tCenter.y));
        //vnRadius.push_back(nRadius + 20);

        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 30, tCenter.y));
        //vnRadius.push_back(nRadius + 30);

        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 40, tCenter.y));
        //vnRadius.push_back(nRadius + 40);

        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 50, tCenter.y));
        //vnRadius.push_back(nRadius + 50);

        //vtCenter.push_back(tCenter);
        //vtStartPtn.push_back(CPoint(tCenter.x + nRadius + 60, tCenter.y));
        //vnRadius.push_back(nRadius + 60);
       
    }
    

    // ��ǰ���ղ����ܲ���
    int nCurSelectTotalNum = 1;
    std::vector<std::vector<T_WELD_PARA>>* pvvtWeldParam;
    if (false == m_bRobotNoSelect)
    {
        pvvtWeldParam = &m_vvtFlatWeldParam;
    }
    else
    {
        pvvtWeldParam = &m_vvtStandWeldParam;
    }
    for (int i = 0; i < pvvtWeldParam->size(); i++)
    {
        if (m_strWorkPiece == pvvtWeldParam->at(i)[0].strWorkPeace)
        {
            nCurSelectTotalNum = pvvtWeldParam->at(i).size();
            break;
        }
    }
    // ��ǰѡ�����
    int nCurSelectLayerNo = m_nLayerNo;
    for (int i = 0; i < nCurSelectTotalNum; i++)
    {
        if (i == nCurSelectLayerNo)
        {
            m_pRightDC->SelectObject(&RedPen);
        }
        else
        {
            m_pRightDC->SelectObject(&BlackPen);
        }
        m_pRightDC->MoveTo(vtStartPtn[i]);
        m_pRightDC->AngleArc(vtCenter[i].x, vtCenter[i].y, vnRadius[i], 0.0, 90.0);
    }
    m_pRightDC->SelectObject(&oldPen);
}

void CWeldParamProcess::ShowLeftWeldLine()
{
    CPoint cWeldLineStartCoor;
    cWeldLineStartCoor.x = m_cRightClientRect.Width() * 1 / 10;
    cWeldLineStartCoor.y = m_cRightClientRect.Height() * 1 / 10;
    CPoint cWeldLineMidCoor;
    cWeldLineMidCoor.x = m_cRightClientRect.Width() * 1 / 10;
    cWeldLineMidCoor.y = m_cRightClientRect.Height() * 9 / 10;
    CPoint cWeldLineEndCoor;
    cWeldLineEndCoor.x = m_cRightClientRect.Width() * 9 / 10;
    cWeldLineEndCoor.y = m_cRightClientRect.Height() * 9 / 10;

    CPen pen(PS_SOLID, 2, RGB(0, 0, 0));
    CPen *oldPen = m_pRightDC->SelectObject(&pen);
    m_pRightDC->MoveTo(cWeldLineStartCoor);
    m_pRightDC->LineTo(cWeldLineMidCoor);
    m_pRightDC->LineTo(cWeldLineEndCoor);
    m_pRightDC->SelectObject(oldPen);
}

void CWeldParamProcess::OnPaint()
{
    CPaintDC dc(this); // device context for painting
    if (false == m_bShowImage)
    {
        m_bShowImage = true;
        InitWeldPage();
        ShowWeldGun(0, 0);
    }   
}

void CWeldParamProcess::OnBnClickedCancel()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	m_bShowImage = false;
	CDialog::OnCancel();
}

void CWeldParamProcess::OnBnClickedButtonSavePos()
{
	UpdateData(true); 
	COPini Opini;
    Opini.SetFileName(DATA_PATH + m_sUnitName + WELD_PARAM_FILE);
	Opini.SetSectionName("WeldParam");
	Opini.WriteString("StartPointScanDis",m_dStartPointScanDis);
	//���޸�
	XUI::MesBox::PopInfo("�ɹ�������ʼ����ǹɨ����:{0}", m_dStartPointScanDis);
	//XiMessageBox("�ɹ�������ʼ����ǹɨ����:%.3lf", m_dStartPointScanDis);
}




void CWeldParamProcess::OnCbnSelchangeBoxStandDir()
{
    int n = m_nStandWeldDir;
    XUI::MesBox::PopOkCancel("{0}", m_nStandWeldDir);
    // TODO: �ڴ���ӿؼ�֪ͨ����������
}
void CWeldParamProcess::OnCbnSelchangeBoxSelectRobot()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������
    m_nRobotNo = m_boxSelectRobot.GetCurSel();
    CString str,strName;
    m_boxSelectRobot.GetLBText(m_nRobotNo, str);
    m_sUnitName = m_vpUnit[m_nRobotNo]->GetRobotCtrl()->m_strRobotName;
    strName.Format("%d", m_nRobotNo);
    if (strName != str)
    {
        XUI::MesBox::PopOkCancel("��ѡ�����ˣ�δ��Ӧʵ�ʻ����˱�ţ�ѡ������˱�ţ�{0} ��Ӧ��ţ�{1}", strName, str);
        return;
    }
    OnBnClickedCheckRobotno();
}

void CWeldParamProcess::OnBnClickedButton2()
{
    // TODO: �ڴ���ӿؼ�֪ͨ����������
    WeaveParamDialog();
}