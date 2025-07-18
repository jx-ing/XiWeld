#if !defined(AFX_VISIONSHOW_H__4768E130_6C22_430C_9458_94830F0D3FF5__INCLUDED_)
#define AFX_VISIONSHOW_H__4768E130_6C22_430C_9458_94830F0D3FF5__INCLUDED_

#include "res/resource.h"
#include "Dll_XiCV_Image_Processing.h"
#include "AbsPositionCalcAdaptor.h"
#include ".\Apps\PLib\YaskawaRobot\RobotDriverAdaptor.h"
#include ".\Apps\PLib\DahengCam\DHGigeImageCapture.h"
#include "CommonAlgorithm.h"
#include ".\Apps\PLib\BasicFunc\Const.h"
#include ".\OpenClass\FileOP\ini\opini.h"

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include ".\Apps\PLib\CtrlUnit\CUnit.h"

// VisionShow.h : header file
//

/////////////////////////////////////////////////////////////////////////////
// CVisionShow dialog

class CDHCameraCtrlDlg : public CDialog
{
// Construction
public:
	CDHCameraCtrlDlg(std::vector < CUnit *> vpUnit, CWnd* pParent = NULL);   // standard constructor
    ~CDHCameraCtrlDlg();

// Dialog Data
	//{{AFX_DATA(CVisionShow)
	enum { IDD = IDD_CTRL_DH_CAMERA };
	int			m_nAcquisitionMode;	//0:连续采集  1:触发采集
	long		m_lExposureTime;
	double		m_dGainLevel;
	//}}AFX_DATA


// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CVisionShow)
	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV support
	//}}AFX_VIRTUAL
	std::vector < CUnit*> m_vpUnit;
// Implementation
protected:

	// Generated message map functions
	//{{AFX_MSG(CVisionShow)
	virtual BOOL OnInitDialog();
	afx_msg void OnBtnOpenCamera();
	afx_msg void OnBtnCloseCamera();
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedButtonAdjustAxisPos();
	afx_msg void OnBnClickedRadioContinuous();
	afx_msg void OnBnClickedRadioTrigger();
	afx_msg void OnEnChangeEditExposureTime();
	afx_msg void OnEnChangeEditGainLevel();
	afx_msg void OnBnClickedButtonSaveImage();
	afx_msg void OnBnClickedButtonTrigger();
	afx_msg void OnBnClickedCheckLaser();
	afx_msg void OnBnClickedCheckStrobe();
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

private:
    bool ShowImage();
	void SetMostControlState(BOOL bState);		//除图片控件全部设置为指定状态
	void CloseCamera();
	static UINT ThreadShowImage(void *pParam);
	void ChangeExposureTime();


	CRect m_rectShowWholeImage;
	CRect m_rectShowWholeGrayImage;
	CRect m_rectItm;
	XiCV_Image_Processing	*m_pXiLaserCV;
	XiCV_Image_Processing	*m_pXiGrayscaleCV;
	IplImage *m_pcvLaserImage;
	IplImage *m_pcvGrayscaleImage;
	int m_nLaserImageNo;
	bool m_bStartAcquisitionMode;
	bool m_bShowImage;
	bool m_bIsShowImageClosed; 
	bool m_bOpenLaser;
	bool m_bOpenStrobe;
	bool m_bChangeExposureTime;

	std::vector<CRobotDriverAdaptor*> m_vpRobotDriver;
public:
	CComboBox m_cChoiceCtrlUnit;
	CComboBox m_cChoiceCamera;
	afx_msg void OnCbnSelchangeComboCtrlUnit();


	void OpenCameraChangeBox(BOOL bCameraState);
	afx_msg void OnCbnSelchangeComboCamera();
};

//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

#endif // !defined(AFX_VISIONSHOW_H__4768E130_6C22_430C_9458_94830F0D3FF5__INCLUDED_)
