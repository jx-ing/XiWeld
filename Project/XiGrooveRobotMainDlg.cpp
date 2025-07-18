// XiGrooveRobotMainDlg.cpp : implementation file
//
#include "stdafx.h"

#include ".\Project\XiGrooveRobot.h"
#include "XiGrooveRobotMainDlg.h"
//#include ".\Apps\PLib\LeisaiCtrl\IOControl.h"
//#include "IncisePlanePart.h"
#include "Apps/PLib/BasicFunc/Log.h"
//#include "CalibrateDlg.h"
#include ".\Project\SetServoCtrlParam.h"
#include ".\Apps\PLib\CtrlUnit\CContralUnit.h"
#include ".\Apps\PLib\BasicFunc\BaseParam.h"
//#include ".\Project\CAutoCtrlDlg.h"
#include ".\Project\ExternalAxisMoveDlg.h"
#include ".\Apps\PLib\Database\CCommonParaByDatabase.h"
#include ".\Apps\PLib\Database\CSetCommonPara.h"

#include ".\Apps\PLib\MD\md5.h"

//#ifdef _DEBUG
//#define new DEBUG_NEW
//#undef THIS_FILE
//static char THIS_FILE[] = __FILE__;
//#endif


/////////////////////////////////////////////////////////////////////////////
// CAboutDlg dialog used for App About

class CAboutDlg : public CDialog
{
public:
	CAboutDlg();

// Dialog Data
	//{{AFX_DATA(CAboutDlg)
	enum { IDD = IDD_ABOUTBOX };
	//}}AFX_DATA

	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAboutDlg)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL

// Implementation
protected:
	//{{AFX_MSG(CAboutDlg)
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialog(CAboutDlg::IDD)
{
	//{{AFX_DATA_INI(CAboutDlg)
	//}}AFX_DATA_INI
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CAboutDlg)
	//}}AFX_DATA_MAP
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialog)
	//{{AFX_MSG_MAP(CAboutDlg)
		// No message handlers
	//}}AFX_MSG_MAP
END_MESSAGE_MAP()

/////////////////////////////////////////////////////////////////////////////
// CXiGrooveRobotMainDlg dialog

CXiGrooveRobotMainDlg::CXiGrooveRobotMainDlg(CWnd* pParent /*=NULL*/)
	: CDialog(CXiGrooveRobotMainDlg::IDD, pParent)
{
	//{{AFX_DATA_INI(CXiGrooveRobotMainDlg)
		// NOTE: the ClassWizard will add member initialization here
	//}}AFX_DATA_INI
	// Note that LoadIcon does not require a subsequent DestroyIcon in Win32

 	if (!CheckFileAndPathExists())
	{
		return;
	}
	CreatFileAndPath();
	LoadAllBaseParam();
// 	COPini opini;
// 	opini.SetFileName(PROCESS_PARA_INI);
// 	opini.SetSectionName("ProcessPara");
// 	opini.ReadString("InciseCurrent", &m_nInciseCurrent);

	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);

    GetModuleFileName(NULL, szFilePath, MAX_PATH);
    (_tcsrchr(szFilePath, _T('\\')))[1] = 0; //ɾ���ļ�����ֻ���·��
}

CXiGrooveRobotMainDlg::~CXiGrooveRobotMainDlg()
{
	DELETE_UNIT(m_pCtrlCardDriver);
	DELETE_UNIT(m_cOPCClientCtrl);
}

void CXiGrooveRobotMainDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	//{{AFX_DATA_MAP(CXiGrooveRobotMainDlg)
	//}}AFX_DATA_MAP
	DDX_Control(pDX, IDC_COMBO2, m_combLanguageChose);
}

BEGIN_MESSAGE_MAP(CXiGrooveRobotMainDlg, CDialog)
	//{{AFX_MSG_MAP(CXiGrooveRobotMainDlg)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_WM_CTLCOLOR()
	ON_BN_CLICKED(IDC_BUTTON_PART_BEVEL_CUT, OnBtnPartBevelCut)
	//}}AFX_MSG_MAP
	ON_BN_CLICKED(IDCANCEL, &CXiGrooveRobotMainDlg::OnBnClickedCancel)
	ON_BN_CLICKED(IDC_BUTTON_SET_PARA, &CXiGrooveRobotMainDlg::OnBnClickedButtonSetPara)
	ON_BN_CLICKED(IDC_BUTTON_SET_PARA2, &CXiGrooveRobotMainDlg::OnBnClickedButtonSetPara2)
	ON_WM_TIMER()
	ON_WM_TIMER()
	ON_BN_CLICKED(IDC_BUTTON1, &CXiGrooveRobotMainDlg::OnBnClickedButton1)
	ON_BN_CLICKED(IDC_BUTTON3, &CXiGrooveRobotMainDlg::OnBnClickedButton3)
	ON_BN_CLICKED(IDC_BUTTON8, &CXiGrooveRobotMainDlg::OnBnClickedButton8)
	ON_CBN_SELCHANGE(IDC_COMBO2, &CXiGrooveRobotMainDlg::OnCbnSelchangeCombo2)
END_MESSAGE_MAP()

// <start> ******************************* ������غ��� ******************************* <start> //

void CXiGrooveRobotMainDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
    if ((nID & 0xFFF0) == IDM_ABOUTBOX)
    {
        CAboutDlg dlgAbout;
        dlgAbout.DoModal();
    }
    else
    {
        CDialog::OnSysCommand(nID, lParam);
    }
}

// If you add a minimize button to your dialog, you will need the code below
//  to draw the icon.  For MFC applications using the document/view model,
//  this is automatically done for you by the framework.

void CXiGrooveRobotMainDlg::OnPaint() 
{
    if (IsIconic())
    {
        CPaintDC dc(this); // device context for painting
        
        SendMessage(WM_ICONERASEBKGND, (WPARAM) dc.GetSafeHdc(), 0);
        
        // Center icon in client rectangle
        int cxIcon = GetSystemMetrics(SM_CXICON);
        int cyIcon = GetSystemMetrics(SM_CYICON);
        CRect rect;
        GetClientRect(&rect);
        int x = (rect.Width() - cxIcon + 1) / 2;
        int y = (rect.Height() - cyIcon + 1) / 2;
        
        // Draw the icon
        dc.DrawIcon(x, y, m_hIcon);
    }
    else
    {
        CDialog::OnPaint();
    }
}

// The system calls this to obtain the cursor to display while the user drags
//  the minimized window.
HCURSOR CXiGrooveRobotMainDlg::OnQueryDragIcon()
{
    return (HCURSOR) m_hIcon;
}

//=======================================================
//�������ͣ����ʾ��Ϣ
//=======================================================
void CXiGrooveRobotMainDlg::SetToolTip()
{
    m_cButtonHint.Create(this);
    m_cButtonHint.SetDelayTime(200);
}

BOOL CXiGrooveRobotMainDlg::PreTranslateMessage(MSG* pMsg) 
{
	//���ͣ�� ��ʾ��Ϣ
    m_cButtonHint.RelayEvent(pMsg);
    
    //������Ӧ
	if(pMsg->message == WM_KEYDOWN)
	{
		if((pMsg->wParam == VK_RETURN) || (pMsg->wParam == VK_ESCAPE))
		{
			return TRUE;
		}
		else if(pMsg->wParam == VK_F1)
		{
			return TRUE;
		}
		else if(pMsg->wParam == VK_F2)
		{
			return TRUE;
		}
        else if(pMsg->wParam == VK_F3)
        {
            return TRUE;
        }
        else if(pMsg->wParam == VK_F4)
        {
            return TRUE;
        }
        else if(pMsg->wParam == VK_F5)
        {
            return TRUE;
        }
        else if(pMsg->wParam == VK_F6)
        {
            return TRUE;
        }

        if (pMsg->wParam == 'P'
			&& GetKeyState(VK_CONTROL)&0x80000 
			&& GetKeyState(VK_MENU)&0x80000 )
		{
            
			return TRUE;
		}		

        if (pMsg->wParam == 'V'
            && GetKeyState(VK_CONTROL)&0x80000 
            && GetKeyState(VK_MENU)&0x80000 )
        {
			
			return TRUE;
		}		
	}

	return CDialog::PreTranslateMessage(pMsg);
}

HBRUSH CXiGrooveRobotMainDlg::OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor) 
{
    HBRUSH hbr = CDialog::OnCtlColor(pDC, pWnd, nCtlColor);
    
    if(pWnd->GetDlgCtrlID() == IDC_STATIC_TIME || pWnd->GetDlgCtrlID() == IDC_STATIC_URL)
    {
		pDC->SetTextColor(RGB(255, 255, 255));
        pDC->SetBkMode(TRANSPARENT); 
    }
    return m_cBckBrush;
}

void CXiGrooveRobotMainDlg::ShowTime()
{
	CString sTime;
	CTime cSystemTime = CTime::GetCurrentTime();
	sTime.Format("%04d/%02d/%02d ", cSystemTime.GetYear(), cSystemTime.GetMonth(), cSystemTime.GetDay());

	CString strTemp = "";
	int nWeekId = cSystemTime.GetDayOfWeek();
	if (nWeekId == 1)
	{
		strTemp = "������";
	}
	else if (nWeekId == 2)
	{
		strTemp = "����һ";
	}
	else if (nWeekId == 3)
	{
		strTemp = "���ڶ�";
	}
	else if (nWeekId == 4)
	{
		strTemp = "������";
	}
	else if (nWeekId == 5)
	{
		strTemp = "������";
	}
	else if (nWeekId == 6)
	{
		strTemp = "������";
	}
	else if (nWeekId == 7)
	{
		strTemp = "������";
	}
	sTime += strTemp;

	strTemp.Format(" %02d:%02d:%02d", cSystemTime.GetHour(), cSystemTime.GetMinute(), cSystemTime.GetSecond());
	sTime += strTemp;

	SetDlgItemText(IDC_STATIC_TIME, sTime);
}

bool CXiGrooveRobotMainDlg::CheckFileAndPathExists()
{
	bool bExists = true;

	//�ļ�
	bExists = bExists && CheckFile(DEBUG_INI);
	bExists = bExists && CheckFile(OPTIONAL_FUNCTION);
	//bExists = bExists && CheckFile(PS_INI);
	bExists = bExists && CheckFile(SYSTEM_PARA_INI);
	//bExists = bExists && CheckFile(USER_INFO_INI);
	bExists = bExists && CheckFile(XI_ROBOT_CTRL_INI);

	//�ļ���

	return bExists;
}

void CXiGrooveRobotMainDlg::CreatFileAndPath()
{
	//�ļ�

	//�ļ���
	CheckFolder(MONITOR_PATH);
	CheckFolder(SYSTEM_LOG_PATH);
}

bool CXiGrooveRobotMainDlg::CheckFile(CString strFile)
{
	if (!CheckFileExists(strFile))
	{
		XiMessageBox("Error:" + strFile + "ϵͳ�ļ���ʧ��");
		return false;
	}
	return true;
}

// <end> ******************************* ������غ��� ******************************* <end> //

BOOL CXiGrooveRobotMainDlg::OnInitDialog()
{
	CDialog::OnInitDialog();

	// Add "About..." menu item to system menu.

	// IDM_ABOUTBOX must be in the system command range.
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		CString strAboutMenu;
		strAboutMenu.LoadString(IDS_ABOUTBOX);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// Set the icon for this dialog.  The framework does this automatically
	//  when the application's main window is not a dialog
	SetIcon(m_hIcon, TRUE);			// Set big icon
	SetIcon(m_hIcon, FALSE);		// Set small icon
	
	//��ʼ��OPC
	m_cOPCClientCtrl = new COPCClientCtrl;

	CString strIP;
	CString strUser;
	CString strKey;
	CString strDBName;
	int nPort;
	LoadData(strIP, strUser, strKey, strDBName, nPort);
	LoadLanguge();
	WriteLog("�����");
	SetToolTip(); // ���� ���ͣ�� ��ʾ��Ϣ	

    CBitmap bmpBack;  
    bmpBack.LoadBitmap(IDB_BITMAP_BACKGROUND); 
    m_cBckBrush.CreatePatternBrush(&bmpBack);   

    SetTimer(1, 500, NULL); // ��ʾϵͳʱ��
	InitCtrlCard();
	//���޸�
	XUI::Languge::GetInstance().translateDialog(this);
	WriteLog("�����ʼ�����");

	//CMenu menu;
	//menu.LoadMenu(IDR_MENU1);  //IDR_MENU1Ϊ�˵���ID��
	//SetMenu(&menu);

	return TRUE;
}

void CXiGrooveRobotMainDlg::OnTimer(UINT_PTR nIDEvent)
{
	// TODO: �ڴ������Ϣ�����������/�����Ĭ��ֵ
	if (1 == nIDEvent)
	{
		ShowTime();
	}

	CDialog::OnTimer(nIDEvent);
}

void CXiGrooveRobotMainDlg::OnCancel() 
{
	KillTimer(1);
	WriteLog("�����˳�");
	CDialog::OnCancel();
}

BOOL CXiGrooveRobotMainDlg::InitCtrlCard()
{
	m_pCtrlCardDriver = new CServoMotorDriver;
	bool bRtn = m_pCtrlCardDriver->LoadCtrlCardParam(m_pCtrlCardDriver->m_vtCtrlCardInfo);
	if (!bRtn)
	{
		AfxMessageBox("���ؿ��ƿ�����ʧ��");
		return FALSE;
	}
	if (!GetLocalDebugMark())
	{
		m_pCtrlCardDriver->InitCtrlCard(m_pCtrlCardDriver->m_vtCtrlCardInfo);
		if (!bRtn)
		{
			AfxMessageBox("��ʼ�����ƿ�ʧ��");
			return FALSE;
		}
	}
	return TRUE;
}

void CXiGrooveRobotMainDlg::OnBtnPartBevelCut() 
{
	if (!RunPara::GetInstance().load())
	{
		XUI::MesBox::PopError("��ȡ���в���ʧ�ܣ�");
		return;
	}
	if (GetLocalDebugMark())
	{
		//AfxMessageBox("���ص���ģʽ");
	}
	if (m_pcAssemblyWeld != NULL)
	{
		delete m_pcAssemblyWeld;
	}
	m_pcAssemblyWeld = new CAssemblyWeld(m_pCtrlCardDriver, m_cOPCClientCtrl);
	setMainDlg(m_pcAssemblyWeld);
	m_pcAssemblyWeld->DoModal();
	setMainDlg(nullptr);

	//if (m_pcAutoCtrlDlg != NULL)
	//{
	//	delete m_pcAutoCtrlDlg;
	//}
	//m_pcAutoCtrlDlg = new CAutoCtrlDlg(m_pCtrlCardDriver, m_cOPCClientCtrl);
	//m_pcAutoCtrlDlg->DoModal();

}

void CXiGrooveRobotMainDlg::OnBnClickedCancel()
{
	CDialog::OnCancel();
}

void CXiGrooveRobotMainDlg::OnBnClickedButtonSetPara()
{
	CSetServoCtrlParam().DoModal();
}


void CXiGrooveRobotMainDlg::OnBnClickedButtonSetPara2()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	CExternalAxisMoveDlg(m_pCtrlCardDriver).DoModal();
}

void CXiGrooveRobotMainDlg::LoadData(CString& strIP, CString& strUser, CString& strKey, CString& strDBName, int& nPort)
{
	COPini opini;
	opini.SetFileName(SYSTEM_PARA_INI);
	opini.SetSectionName("Database_Debug");
	opini.ReadAddString("���ݿ�IP", strIP, "127.0.0.1");
	opini.ReadAddString("���ݿ�˿ں�", &nPort, 3306);
	opini.ReadAddString("�û���", strUser, "test_user");
	opini.ReadAddString("����", strKey, "mAYa1Eh2dedAB8Ca1iCOcEGO2i4a42");
	opini.ReadAddString("���ݿ���", strDBName, "test");
}

void CXiGrooveRobotMainDlg::OnBnClickedButton1()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������

}

void CXiGrooveRobotMainDlg::OnBnClickedButton3()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������	

	FILE* m_pFileLog;
	XI_fopen_s(&m_pFileLog, "job.txt", "w", _SH_DENYNO);
	int n = 0;
	for (int i = 0; i < 10; i++)
	{
		fprintf_s(m_pFileLog, "'MOVE TO P%d\n", 20 + i);
		fprintf_s(m_pFileLog, "JUMP *%d-1 IF I0%d=0\n", 20 + i, 20 + i);
		fprintf_s(m_pFileLog, "MOVJ P0%d BP0%d VJ=I0%d ACC=I0%d DEC=I0%d\n", 20 + i, 20 + i, 40 + i * 3, 41 + i * 3, 42 + i * 3);
		fprintf_s(m_pFileLog, "JUMP *%d-2\n", 20 + i);
		fprintf_s(m_pFileLog, "*%d-1\n", 20 + i);
		fprintf_s(m_pFileLog, "MOVL P0%d BP0%d V=I0%d ACC=I0%d DEC=I0%d\n", 20 + i, 20 + i, 40 + i * 3, 41 + i * 3, 42 + i * 3);
		fprintf_s(m_pFileLog, "*%d-2\n", 20 + i);
		fprintf_s(m_pFileLog, "INC I008\n");
		fprintf_s(m_pFileLog, "JUMP *END IF I008>=I009\n");
	}
	fflush(m_pFileLog);
	fclose(m_pFileLog);
}

/*
'MOVE TO P20
JUMP *20-2 IF I020=0
MOVJ P020 VJ=I040 NWAIT ACC=I041 DEC=I042
JUMP *20-3
*20-2
MOVL P020 V=I040 NWAIT ACC=I041 DEC=I042
*20-3
INC I008
JUMP *END IF I008>=I009
*/

void CXiGrooveRobotMainDlg::OnBnClickedButton8()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	//CSetCommonPara().DoModal();
}

void CXiGrooveRobotMainDlg::OnCbnSelchangeCombo2()
{
	// TODO: �ڴ���ӿؼ�֪ͨ����������
	int nIndex = m_combLanguageChose.GetCurSel();
	CString str;
	m_combLanguageChose.GetLBText(nIndex, str);
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	CStringA strUtf8 = UnicodeToUtf8(str.GetBuffer());
	//opini.WriteString("Language", strUtf8.GetBuffer());
	opini.WriteString("Language", strUtf8);
	//COPini::WriteString(DEBUG_INI, "Debug", "Language", str);
	//XUI::Languge::GetInstance().loadAllLanguge();
	XUI::Languge::GetInstance().chioceLanguge(str.GetBuffer());//������ܧڧ� ��٧��
}

void CXiGrooveRobotMainDlg::LoadLanguge()
{
	m_combLanguageChose.ResetContent();

	m_combLanguageChose.AddString("English");
	//m_combLanguageChose.AddString("������ܧڧ� ��٧��");
	m_combLanguageChose.AddString("����");

	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");
	CString str;
	opini.ReadAddString("Language", str, NULL);
	str = Utf8ToGBK(str);
	int InDex;
	if (str == "English")
	{
		InDex = 0;
	}
	//else if (str == "������ܧڧ� ��٧��")
	//{
	//	InDex = 1;
	//}
	else if (str == "����")
	{
		InDex = 1;
	}
	m_combLanguageChose.SetCurSel(InDex);
}
