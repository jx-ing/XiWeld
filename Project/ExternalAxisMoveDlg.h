#pragma once
#include "afxwin.h"

#include ".\Apps\PLib\LeisaiCtrl\ServoMotorDriver.h"


// CExternalAxisMoveDlg �Ի���
#define MAX_AXIS_NUM  16
class CExternalAxisMoveDlg : public CDialogEx
{
	DECLARE_DYNAMIC(CExternalAxisMoveDlg)

public:
	CExternalAxisMoveDlg(CServoMotorDriver* pSetServoCtrl, CWnd* pParent = NULL);
	CExternalAxisMoveDlg(CWnd* pParent = NULL);   // ��׼���캯��
	virtual ~CExternalAxisMoveDlg();

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_DIALOG_EXTERNAL };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_ComBox_ExtAxis;
	CComboBox m_ComBox_AxisMode;
	CComboBox m_ComBox_MoveDirection;
	CComboBox m_ComBox_StopMode;
	WORD m_usCardNo = 0;
	WORD m_Axis = 0;
	WORD m_StopMode;
	WORD m_MoveDirection;
	WORD m_AxisMode;
	UINT m_nFocusEditId = -1;//�༭��id
	double m_dDistancePluse[MAX_AXIS_NUM] = {0};//����
	double m_dVeloci[MAX_AXIS_NUM] = { 2000,2000,2000 };//�ٶ�
	double m_dOriVel[MAX_AXIS_NUM] = { 1000,1000,1000 };//��ʼ�ٶ�
	double m_dADDTime[MAX_AXIS_NUM] = { 2,2,2 };//����ʱ��
	double m_dDesTime[MAX_AXIS_NUM] = { 2,2,2 };//����ʱ��
	virtual BOOL OnInitDialog();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
	//�����Ϣ
	bool ShowExternalAxis0Information();
	bool ShowExternalAxis1Information();
	bool ShowExternalAxis2Information();
	
	bool ShowTransformDistanceToMM(WORD Axis);
	bool ShowTransformVelociToMM(WORD Axis);
	bool ShowTransformDistanceMMToU(WORD Axis);
	bool ShowTransformVelociMMToU(WORD Axis);
	BOOL InitExternalPluseEquivalent();
	BOOL InitExternalArgument();
	

	double m_dAxisPluseEquivalent[4] = {0.0};	//���嵱��
	afx_msg void OnCbnSelchangeCombo1();
	afx_msg void OnCbnSelchangeCombo3();
	afx_msg void OnCbnSelchangeCombo4();
	afx_msg void OnCbnSelchangeCombo2();
	virtual BOOL OnCommand(WPARAM wParam, LPARAM lParam);
	afx_msg void OnBnClickedButton3();
	afx_msg void OnBnClickedButton11();
	afx_msg void OnBnClickedButton14();
	afx_msg void OnBnClickedButton1();
	afx_msg void OnBnClickedButton17();
	virtual BOOL PreTranslateMessage(MSG* pMsg);
	afx_msg void OnBnClickedButton13();

	CServoMotorDriver *m_pcSetServoCtrl;
	afx_msg void OnBnClickedOk();
	afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);

	//��ȫ�Ż�
	int naSpeedTableByRobot[5]={30,80,120,200,300};//�����˵����ĳ�ʼ�ٶ�mm/s
	int naSpeedTableByZ[5] = { 0,0,0,0,0 };//�����и�����˵�������mm/s
	int nSpeedByFree = 0;//ѡ���������30-80�Σ��ɽ��������޸ĵ�����ٶ�ֵ

	bool m_abSignal[3] = { false };
	bool m_abError[3] = { false };
	COLORREF m_cColorGreen = RGB(0, 255, 0);
	COLORREF m_cColorRed = RGB(255, 0, 0);
	COLORREF m_cColorYellow = RGB(255, 255, 0);
	int m_nInput1;
	int m_nInput2;
	int m_nInput3;
	afx_msg void OnEnChangeEdit8();
	afx_msg void OnEnChangeEdit11();
	afx_msg void OnEnChangeEdit12();
	int m_nOutput1;
	int m_nOutput2;
	int m_nOutput3;
	BOOL m_nOutputMark1;
	BOOL m_nOutputMark2;
	BOOL m_nOutputMark3;
	afx_msg void OnBnClickedCheck1();
	afx_msg void OnEnChangeEdit13();
	afx_msg void OnEnChangeEdit14();
	afx_msg void OnEnChangeEdit15();
	afx_msg void OnBnClickedCheck2();
	afx_msg void OnBnClickedCheck4();
	afx_msg void OnBnClickedButton16();
	afx_msg void OnBnClickedButton18();
	afx_msg void OnBnClickedButton12();
	CComboBox m_cBox_ChoiceSpeed;
	afx_msg void OnCbnSelchangeCombo5();
};
