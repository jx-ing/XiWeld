#pragma once
#include "afxwin.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"


// CWeldParamProcess 对话框

class CWeldParamProcess : public CDialog
{
	DECLARE_DYNAMIC(CWeldParamProcess)

public:
	CWeldParamProcess(std::vector<CUnit*> vpUnit, CWnd* pParent = NULL);   // 标准构造函数 获取所有机器人的工艺参数
	CWeldParamProcess(CString strName = "", CWnd* pParent = NULL);   // 标准构造函数 获取指定机器人的工艺参数
	virtual ~CWeldParamProcess();
// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_WELD_PARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnCbnSelchangeBoxSelectDrawinglist();
	afx_msg void OnCbnSelchangeBoxLayerNo();
    void SelchangeBoxSelectDrawinglist();
	void SelectLayerNoList();
	CComboBox m_bSelectDrawing;
	afx_msg void OnBnClickedOk();
	double m_dStartArcCurrent;
	double m_dStartArcVoltage;
	double m_dStopArcVoltage;
	double m_dStopArcCurrent;
	double m_dStopWaitTime;
	double m_dTrackCurrent;
	double m_WeldVelocity;
	double m_dTrackVoltage;
	double m_dStartWaitTime;
	double m_verticalOffset;
	double m_dCrosswiseOffset;
	int m_nWrapCondition;
	double m_dWeldAngle; // 平焊焊丝和底板 或 立焊焊丝和侧板夹角 标准为45°
	double m_dWeldDipAngle; // 焊接倾角 药芯焊丝拉着焊(正数) 实芯焊丝推着焊(负数)
	afx_msg void OnCbnSelchangeComboWeldType();
	virtual BOOL OnInitDialog();
public:
	std::vector<T_WELD_SEAM_TYPE_PARA> m_vtWeldPara;
	T_WELD_SEAM_TYPE_PARA m_tWeldPara;
	T_WELD_SEAM_TYPE_PARA m_tNewWeldPara;
	T_WELD_PARA m_tWeldSeamPare;
	CComboBox m_cWeldType;
	CString m_strWorkPiece;
	CString m_strWeldSeamType;
	CString m_strWorkPieceLeft;
	CString m_strWorkPieceRight;
	CString m_strWorkPieceFlat[10]; //[ROBOT_NUM];
	CString m_strWorkPieceStand[10]; //[ROBOT_NUM];
	bool m_bEditOrAdd;
	int m_nLayerNo; // 全局 多层多道参数 层号索引

	void LoadParam();
	void LoadParamSingle();
	void SaveParam();
    void LoadWeldThink(int nRobotNo, double &dWeldParam);
	bool LoadCurUseParam(std::vector<T_WELD_PARA>& vtWeldParam, bool bIsFlat);
	bool LoadWeldParam(bool bIsFlat, int nWeldAngleSize, std::vector<T_WELD_PARA>& vtWeldParam);
	bool LoadWeldParam(int nRobotNo, bool bIsFlat, int nWeldAngleSize, std::vector<T_WELD_PARA>& vtWeldParam);
	bool LoadCurUseParam(int nRobotNo, std::vector<T_WELD_PARA>& vtWeldParam, bool bIsFlat);
	afx_msg void OnBnClickedBtnDelete();
	afx_msg void OnBnClickedBtnEdit();
	afx_msg void OnBnClickedBtnSave();
	CString m_strNewWorkpieceName;
	afx_msg void OnBnClickedBtnAdd();
	std::vector<T_WELD_SEAM_TYPE_PARA> WeldSeamTypePara();
	afx_msg void OnBnClickedRadioEdit();
	afx_msg void OnBnClickedRadioAdd();
	void LoadAttributeParam();
	//	BOOL m_bEditOrAddPara;
    double m_dWrapCurrent1;
    double m_dWrapCurrent2;
    double m_dWrapCurrent3;
    double m_dWrapVol1;
    double m_dWrapVol2;
    double m_dWrapVol3;
    double m_dWrapWaitTime1;
    double m_dWrapWaitTime2;
    double m_dWrapWaitTime3;
	int m_nStandWeldDir;
	std::vector<CUnit*> m_vpUnit;
	CString m_sUnitName;
	int m_nRobotNum;

    //std::vector<T_WELD_PARA> m_vtWeldParamLeft;
    //std::vector<T_WELD_PARA> m_vtWeldParamRight;
	int m_nRobotNo;
	// 多个机器人参数
	std::vector<std::vector<std::vector<T_WELD_PARA>>> m_vvvtFlatWeldParam;
	std::vector<std::vector<std::vector<T_WELD_PARA>>> m_vvvtStandWeldParam;
	// 单个机器人参数
	std::vector<std::vector<T_WELD_PARA>> m_vvtFlatWeldParam;
	std::vector<std::vector<T_WELD_PARA>> m_vvtStandWeldParam;
	std::vector<T_WEAVE_PARA> m_vtWeaveParam; // 摆动参数
	std::vector<T_WEAVE_FULLPARA> m_vtWeaveFullParam; //安川完整摆动参数
    void                     LoadWeldParam(int nRobotNo, double dThink, T_WELD_PARA &tWeldParam,bool ioethick);
    //void                     LoadWeldParam(std::vector<T_WELD_PARA> &vtWeldParamLeft, std::vector<T_WELD_PARA> &vtWeldParamRight);
	void                     LoadWeldParam(std::vector<std::vector<T_WELD_PARA>>& vvtFlatWeldParam, std::vector<std::vector<T_WELD_PARA>>& vtStandWeldParam);
	void					 LoadWeaveParam(); // 加载摆动参数
	void					 WeaveParamDialog(); //在界面修改摆动参数
	void					 SaveWeaveParam(T_WEAVE_FULLPARA t, int curWaveNum); //保存修改后的摆动参数
	T_WEAVE_FULLPARA		 LoadCurWeaveParam(std::vector<double>& curWeaveParam, std::vector<CString>& curWeaveParamName, int curWaveNum); //加载当前摆动参数
	void					 LoadWeldParam(std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtFlatWeldParam,
		std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtStandWeldParam);
    void                     SaveWeldParam(int nRobotNo);
	void					 SaveWeldParam();// 保存全部焊接工艺
    static bool              SortWeldPara(T_WELD_PARA a, T_WELD_PARA b);
	static bool              SortWeldParaNew(std::vector<T_WELD_PARA>& a, std::vector<T_WELD_PARA>& b);

    BOOL m_bRobotNoSelect;
    afx_msg void OnBnClickedCheckRobotno();

    void                    OnRefreshPage(T_WELD_PARA tWeldPare);
    T_WELD_PARA             DownloadPageParam();

    void                    InitWeldPage();
    void                    ShowWeldGun(double dXAxisAdjust, double dZAxisAdjust);
	void DrawingWeldSeam(CPoint tBaseCenterPtn, int nRadius, bool bIsFlatSeam);
    void                    ShowLeftWeldLine();
    bool                    m_bShowImage;

    CRect m_cRightClientRect;//画布范围
    CRect m_cRightWindowRect;//界面范围
    CWnd *m_pRightWin;//界面窗口
    CDC *m_pRightDC;//显示界面CDC
    afx_msg void OnPaint();
	afx_msg void OnBnClickedCancel();
	double m_dStartPointScanDis;
	afx_msg void OnBnClickedButtonSavePos();
	CComboBox m_cComboSelectLayerNo;
	CComboBox m_boxSelectRobot;
	afx_msg void OnCbnSelchangeBoxSelectRobot();
	// 立焊焊接方向
	CComboBox m_ComboBoxStandDir;
	afx_msg void OnCbnSelchangeBoxStandDir();
	afx_msg void OnBnClickedButton2();
};
