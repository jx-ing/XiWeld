#pragma once
#include "afxdialogex.h"
#include "Apps\PLib\CtrlUnit\CUnit.h"

#define KEY_DOWN(VK_NONAME) ((GetAsyncKeyState(VK_NONAME) & 0x8000) ? 1:0) //必要的，要背下来
// ChangeScanLineParam 对话框

class ChangeScanLineParam : public CDialogEx
{
	DECLARE_DYNAMIC(ChangeScanLineParam)

public:
	ChangeScanLineParam(CUnit *pUnit, CWnd* pParent = nullptr);   // 标准构造函数
	virtual ~ChangeScanLineParam();
	void DrawTable();
	void DrawScale();
	void DrawArrow();
	void LoadScanParam();
	bool CheckRobotOffsetValid(T_ANGLE_PULSE& tScanPulse);

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_CHANGE_SCANPARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	virtual BOOL OnInitDialog();
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnPaint();
	afx_msg void OnEnChangeEdit8();
	afx_msg void OnEnChangeEdit2();
	afx_msg void OnEnChangeEdit3();
	afx_msg void OnEnChangeEdit5();
	afx_msg void OnEnChangeEdit4();
	afx_msg void OnEnChangeEdit6();
	afx_msg void OnEnChangeEdit7();
	afx_msg void OnBnClickedBtnok();
	afx_msg void OnBnClickedBtnok3();
	afx_msg void OnBnClickedBtnok4();
	afx_msg void OnBnClickedBtnok5();

private:
	CPoint m_ptStart;
	CPoint m_ptEnd;
	CPoint m_ptCenter;
	CRect m_tableRect;
	CDC* m_ptableDC;
	CWnd* m_ptablepWnd;
	int m_tableWidth;
	int m_tableHeight;
	int m_tableSx;
	int m_tableSy;
	int m_tableEx;
	int m_tableEy;
	int m_rTableLength;
	int m_rTableWidth;
	double dMaxScanPos;
	double dMinScanPos;
	double m_ScanOffsetX;
	double m_ScanOffsetZ;
	double m_ScanOffsetRX;
	double m_ScanOffsetRY;
	double m_ScanOffsetRZ;
	int m_ScanStartLocation;
	int m_ScanEndLocation;
	double rStoW;
	bool m_IsInital;
	bool m_bRButtonIsDown; // 鼠标右键是否按下
	bool m_bIsRunning; // 机器人或外部轴是否运行中
	T_ANGLE_PULSE m_tLineScanPulse;
	CUnit *m_pUnit;
};
