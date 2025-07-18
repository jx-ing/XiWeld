#pragma once
#include "afxwin.h"
#include ".\Apps\PLib\CtrlUnit\CUnit.h"


// CWeldParamProcess �Ի���

class CWeldParamProcess : public CDialog
{
	DECLARE_DYNAMIC(CWeldParamProcess)

public:
	CWeldParamProcess(std::vector<CUnit*> vpUnit, CWnd* pParent = NULL);   // ��׼���캯�� ��ȡ���л����˵Ĺ��ղ���
	CWeldParamProcess(CString strName = "", CWnd* pParent = NULL);   // ��׼���캯�� ��ȡָ�������˵Ĺ��ղ���
	virtual ~CWeldParamProcess();
// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_WELD_PARAM };
#endif

protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV ֧��

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
	double m_dWeldAngle; // ƽ����˿�͵װ� �� ������˿�Ͳ��н� ��׼Ϊ45��
	double m_dWeldDipAngle; // ������� ҩо��˿���ź�(����) ʵо��˿���ź�(����)
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
	int m_nLayerNo; // ȫ�� ��������� �������

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
	// ��������˲���
	std::vector<std::vector<std::vector<T_WELD_PARA>>> m_vvvtFlatWeldParam;
	std::vector<std::vector<std::vector<T_WELD_PARA>>> m_vvvtStandWeldParam;
	// ���������˲���
	std::vector<std::vector<T_WELD_PARA>> m_vvtFlatWeldParam;
	std::vector<std::vector<T_WELD_PARA>> m_vvtStandWeldParam;
	std::vector<T_WEAVE_PARA> m_vtWeaveParam; // �ڶ�����
	std::vector<T_WEAVE_FULLPARA> m_vtWeaveFullParam; //���������ڶ�����
    void                     LoadWeldParam(int nRobotNo, double dThink, T_WELD_PARA &tWeldParam,bool ioethick);
    //void                     LoadWeldParam(std::vector<T_WELD_PARA> &vtWeldParamLeft, std::vector<T_WELD_PARA> &vtWeldParamRight);
	void                     LoadWeldParam(std::vector<std::vector<T_WELD_PARA>>& vvtFlatWeldParam, std::vector<std::vector<T_WELD_PARA>>& vtStandWeldParam);
	void					 LoadWeaveParam(); // ���ذڶ�����
	void					 WeaveParamDialog(); //�ڽ����޸İڶ�����
	void					 SaveWeaveParam(T_WEAVE_FULLPARA t, int curWaveNum); //�����޸ĺ�İڶ�����
	T_WEAVE_FULLPARA		 LoadCurWeaveParam(std::vector<double>& curWeaveParam, std::vector<CString>& curWeaveParamName, int curWaveNum); //���ص�ǰ�ڶ�����
	void					 LoadWeldParam(std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtFlatWeldParam,
		std::vector<std::vector<std::vector<T_WELD_PARA>>>& vvvtStandWeldParam);
    void                     SaveWeldParam(int nRobotNo);
	void					 SaveWeldParam();// ����ȫ�����ӹ���
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

    CRect m_cRightClientRect;//������Χ
    CRect m_cRightWindowRect;//���淶Χ
    CWnd *m_pRightWin;//���洰��
    CDC *m_pRightDC;//��ʾ����CDC
    afx_msg void OnPaint();
	afx_msg void OnBnClickedCancel();
	double m_dStartPointScanDis;
	afx_msg void OnBnClickedButtonSavePos();
	CComboBox m_cComboSelectLayerNo;
	CComboBox m_boxSelectRobot;
	afx_msg void OnCbnSelchangeBoxSelectRobot();
	// �������ӷ���
	CComboBox m_ComboBoxStandDir;
	afx_msg void OnCbnSelchangeBoxStandDir();
	afx_msg void OnBnClickedButton2();
};
