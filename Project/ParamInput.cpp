// ParamInput.cpp: 实现文件
//

#include "StdAfx.h"
//#include "XiWeldRobot.h"
#include "res\resource.h"
#include "afxdialogex.h"
#include "ParamInput.h"


// ParamInput 对话框

IMPLEMENT_DYNAMIC(ParamInput, CDialog)

ParamInput::ParamInput(CString sTitleName, std::vector<CString> vStrName,
	std::vector<double> *vnInputData, CWnd* pParent /*=nullptr*/)
	: CDialog(IDD_PARAM_INPUT, pParent)
	, m_nEdit1(0)
	, m_nEdit2(0)
	, m_nEdit3(0)
	, m_nEdit4(0)
	, m_nEdit5(0)
	, m_nEdit6(0)
	, m_nEdit7(0)
	, m_nEdit8(0)
	, m_nEdit9(0)
	, m_nEdit10(0)
	, m_nEdit11(0)
	, m_nEdit12(0)
	, m_nEdit13(0)
	, m_vStrName(vStrName)
	, m_vnInputData(vnInputData)
	, m_sTitleName(sTitleName)
{
	m_vnStaticID.clear();
	m_vpEdit.clear();
	m_vnData.clear();

	m_vnStaticID.push_back(IDC_STATIC_INPUT1);
	m_vnStaticID.push_back(IDC_STATIC_INPUT2);
	m_vnStaticID.push_back(IDC_STATIC_INPUT3);
	m_vnStaticID.push_back(IDC_STATIC_INPUT4);
	m_vnStaticID.push_back(IDC_STATIC_INPUT5);
	m_vnStaticID.push_back(IDC_STATIC_INPUT6);
	m_vnStaticID.push_back(IDC_STATIC_INPUT7);
	m_vnStaticID.push_back(IDC_STATIC_INPUT8);
	m_vnStaticID.push_back(IDC_STATIC_INPUT9);
	m_vnStaticID.push_back(IDC_STATIC_INPUT10);
	m_vnStaticID.push_back(IDC_STATIC_INPUT11);
	m_vnStaticID.push_back(IDC_STATIC_INPUT12);
	m_vnStaticID.push_back(IDC_STATIC_INPUT13);
	int n = m_vnStaticID.size();

	m_vpEdit.push_back(&m_cEdit1);
	m_vpEdit.push_back(&m_cEdit2);
	m_vpEdit.push_back(&m_cEdit3);
	m_vpEdit.push_back(&m_cEdit4);
	m_vpEdit.push_back(&m_cEdit5); 
	m_vpEdit.push_back(&m_cEdit6);
	m_vpEdit.push_back(&m_cEdit7);
	m_vpEdit.push_back(&m_cEdit8);
	m_vpEdit.push_back(&m_cEdit9);
	m_vpEdit.push_back(&m_cEdit10);
	m_vpEdit.push_back(&m_cEdit11);
	m_vpEdit.push_back(&m_cEdit12);
	m_vpEdit.push_back(&m_cEdit13);
	n = m_vpEdit.size();

	m_vnData.push_back(&m_nEdit1);
	m_vnData.push_back(&m_nEdit2);
	m_vnData.push_back(&m_nEdit3);
	m_vnData.push_back(&m_nEdit4);
	m_vnData.push_back(&m_nEdit5);
	m_vnData.push_back(&m_nEdit6);
	m_vnData.push_back(&m_nEdit7);
	m_vnData.push_back(&m_nEdit8);
	m_vnData.push_back(&m_nEdit9);
	m_vnData.push_back(&m_nEdit10);
	m_vnData.push_back(&m_nEdit11);
	m_vnData.push_back(&m_nEdit12);
	m_vnData.push_back(&m_nEdit13);
	n = m_vnData.size();
}

ParamInput::~ParamInput()
{
}

void ParamInput::DoDataExchange(CDataExchange* pDX)
{
	CDialog::DoDataExchange(pDX);
	DDX_Control(pDX, IDC_EDIT_INPUT1, m_cEdit1);
	DDX_Control(pDX, IDC_EDIT_INPUT2, m_cEdit2);
	DDX_Control(pDX, IDC_EDIT_INPUT3, m_cEdit3);
	DDX_Control(pDX, IDC_EDIT_INPUT4, m_cEdit4);
	DDX_Control(pDX, IDC_EDIT_INPUT5, m_cEdit5);
	DDX_Control(pDX, IDC_EDIT_INPUT6, m_cEdit6);
	DDX_Control(pDX, IDC_EDIT_INPUT7, m_cEdit7);
	DDX_Control(pDX, IDC_EDIT_INPUT8, m_cEdit8);
	DDX_Control(pDX, IDC_EDIT_INPUT9, m_cEdit9);
	DDX_Control(pDX, IDC_EDIT_INPUT10, m_cEdit10);
	DDX_Control(pDX, IDC_EDIT_INPUT11, m_cEdit11);
	DDX_Control(pDX, IDC_EDIT_INPUT12, m_cEdit12);
	DDX_Control(pDX, IDC_EDIT_INPUT13, m_cEdit13);
	DDX_Text(pDX, IDC_EDIT_INPUT1, m_nEdit1);
	DDX_Text(pDX, IDC_EDIT_INPUT2, m_nEdit2);
	DDX_Text(pDX, IDC_EDIT_INPUT3, m_nEdit3);
	DDX_Text(pDX, IDC_EDIT_INPUT4, m_nEdit4);
	DDX_Text(pDX, IDC_EDIT_INPUT5, m_nEdit5);
	DDX_Text(pDX, IDC_EDIT_INPUT6, m_nEdit6);
	DDX_Text(pDX, IDC_EDIT_INPUT7, m_nEdit7);
	DDX_Text(pDX, IDC_EDIT_INPUT8, m_nEdit8);
	DDX_Text(pDX, IDC_EDIT_INPUT9, m_nEdit9);
	DDX_Text(pDX, IDC_EDIT_INPUT10, m_nEdit10);
	DDX_Text(pDX, IDC_EDIT_INPUT11, m_nEdit11);
	DDX_Text(pDX, IDC_EDIT_INPUT12, m_nEdit12);
	DDX_Text(pDX, IDC_EDIT_INPUT13, m_nEdit13);
}

BOOL ParamInput::OnInitDialog()
{
	CDialog::OnInitDialog();

	// TODO:  在此添加额外的初始化
	this->SetWindowText(m_sTitleName);
	int i = 0;
	for (i = 0; i < m_vStrName.size(); i++)
	{
		GetDlgItem(m_vnStaticID[i])->SetWindowText(m_vStrName[i]);
		*(m_vnData[i]) = (*m_vnInputData)[i];
	}
	int n = m_vnStaticID.size();
	for (; i < n; i++)
	{
		GetDlgItem(m_vnStaticID[i])->SetWindowText("无效:");
		GetDlgItem(m_vnStaticID[i])->EnableWindow(false);
		m_vpEdit[i]->EnableWindow(false);
	}
	UpdateData(FALSE);
	//已修改
	XUI::Languge::GetInstance().translateDialog(this);
	return TRUE;  // return TRUE unless you set the focus to a control
	// 异常: OCX 属性页应返回 FALSE
}

BEGIN_MESSAGE_MAP(ParamInput, CDialog)
	ON_BN_CLICKED(IDOK, &ParamInput::OnBnClickedOk)
END_MESSAGE_MAP()


// ParamInput 消息处理程序


void ParamInput::OnBnClickedOk()
{
	// TODO: 在此添加控件通知处理程序代码
	UpdateData(TRUE);
	for (int i = 0; i < m_vnInputData->size(); i++)
	{
		(*m_vnInputData)[i] = (*(m_vnData[i]));
	}
	CDialog::OnOK();
}

//void ParamInput::GetParam(int& nRobotNo, int& nCameraNo, int& nPtnX, int& nPtnY)
//{
//	nRobotNo = m_nEdit1;
//	nCameraNo = m_nEdit2;
//	nPtnX = m_nEdit3;
//	nPtnY = m_nEdit4;
//}

