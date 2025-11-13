#pragma once
#include "afxdialogex.h"
#include "./res/resource.h"
#include ".\Apps\PLib\BasicFunc\Const.h"




// ChangeGroovePara 对话框

class ChangeGroovePara : public CDialogEx
{
	DECLARE_DYNAMIC(ChangeGroovePara)

public:
	ChangeGroovePara(CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~ChangeGroovePara();
	virtual BOOL OnInitDialog();


// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_GROOVE_PARA };
#endif

public:
	void InitCombo1();
	void SetGroovePara();
	void LoadGroovePara();
	void ShowGroovePara();
	void LoadAllGroovePara(vector<T_WAVE_PARA>& vtFlatWavePara, vector<T_WAVE_PARA>& vtVerWavePara);

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	CComboBox m_ComboWeldNo;
	CComboBox m_ComboWaveType;
	afx_msg void OnBnClickedButton1();
	double m_StartWave;
	double m_EndWave;
	double m_WaveDistance;
	double m_EndSpeedRate;
	double m_LeftWaitTime;
	double m_RightWaitTime;
	double m_MidWaitTime;
	double m_VerticalAngle;
	double m_NormalOffset;
	double m_HorizontalOffset;
	double m_WaveHeight;
	double m_WaveUpBase;
	vector<T_WAVE_PARA> vtFlatWavePara;
	vector<T_WAVE_PARA> vtVerWavePara;
	afx_msg void OnCbnSelchangeCombo1();
	int m_WeldNo;
	int m_WaveType;
	int m_ArcType;
	CComboBox m_ComboArcType;
	afx_msg void OnCbnSelchangeCombo4();
};
