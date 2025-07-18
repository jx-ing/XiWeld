#if !defined(AFX_INCISEHEAD_H__DA65D33B_9ECF_4AE5_A0F5_67DA4502360C__INCLUDED_)
#define AFX_INCISEHEAD_H__DA65D33B_9ECF_4AE5_A0F5_67DA4502360C__INCLUDED_
#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000
// InciseHead.h : header file
#include "afxwin.h"
#include "DHCameraCtrlDlg.h"
#include "FullScreen.h"
#include "OpenClass\COMM\OPC\OPCClientCtrl.h"
#include "Apps\PLib\Others\ShowTrack.h"
#include "Apps\Welding\LaserLineScreen.h"
#include "Apps\Welding\WeldAfterMeasure.h"
#include "Apps\Welding\GenericWeld.h"
#include "Apps\Welding\GrooveWeld.h"
#include "Apps\Welding\SingleBoard.h"
#include "Apps\Welding\ScanWeldLine.h"
#include "Project\RobotCtrlDlg.h"
#include "Project\WrapAngleParam.h"
#include "Project\ChangeScanLineParam.h"
#include "Apps\Welding\DiaphragmWeld.h"
#include "XiBase.h"
//#include "ProgressDialog.h"
#include "LineScanForGantry.h"
//#include "LineScanForPanTeach.h"
#include "LineScanForPanasonicTeach.h"
//#include "ContourFitting_C_DLL.h"
//#include "StiffenPlate.h"
//#include "PurlinHanger.h"
//#include "DiaphragmWeld.h"
//#include"NamedPipeClient.h" //��·��ʱʹ�� �����ܵ�

#include "ChangeGroovePara.h"
#include "XirobotCv.h"
#include "VZenseDriver.h"

class CAssemblyWeld;

typedef struct
{
	int nGroupNo;
	int nLayerNo; // ������� �ڼ�������
	bool bRobotThreadStatus;
	CRobotDriverAdaptor* pRobotCtrl;
	CAssemblyWeld* cIncisePlanePart;
}T_ROBOT_THREAD;

/////////////////////////////////////////////////////////////////////////////
// CAssemblyWeld dialog
class CAssemblyWeld : public CDialog
{
// Construction
public:
	CAssemblyWeld(CServoMotorDriver* pCtrlCardDriver = NULL, COPCClientCtrl* pOPCClientCtrl = NULL);
    ~CAssemblyWeld();

// Dialog Data
	//{{AFX_DATA(CAssemblyWeld)
	enum { IDD = IDD_MAIN_ASSEMBLY_WELD };
	//}}AFX_DATA

// Overrides
	// ClassWizard generated virtual function overrides
	//{{AFX_VIRTUAL(CAssemblyWeld)
public:
	virtual BOOL OnInitDialog(); 
	VOID MainFromArrangement();
	afx_msg void OnTimer(UINT_PTR nIDEvent);
    afx_msg HBRUSH OnCtlColor(CDC* pDC, CWnd* pWnd, UINT nCtlColor);
	//}}AFX_VIRTUAL

// Implementation
public:
	// Generated message map functions
	//{{AFX_MSG(CAssemblyWeld) 
    afx_msg void OnBtnPauseContinue();
	afx_msg void OnBtnClose();
	afx_msg void OnBnClickedButtonSimulateIncise();
	afx_msg void OnBtnSystemPara();
	afx_msg void OnBnClickedButtonSpare4();
	afx_msg void OnBtnVisionShow();
	afx_msg void OnBtnLoadTrack();
	afx_msg void OnBnClickedButtonPanoRecognition();
	afx_msg void OnBnClickedButtonRobotCtrl();
	afx_msg void OnBnClickedButtonAntifeeding();
	afx_msg void OnBnClickedButtonStart();
	afx_msg void OnBnClickedButtonPlasmaCom();
	afx_msg void OnBnClickedButtonAdjustRecog();
	afx_msg void OnBnClickedButtonBackHome();
	afx_msg void OnBnClickedRadioNoarc();
	afx_msg void OnBnClickedRadioArc();
	afx_msg BOOL OnMouseWheel(UINT nFlags, short zDelta, CPoint pt);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnBnClickedButtonCommonlyUsedIO();
	afx_msg void OnBnClickedButtonSpare3();
	afx_msg void OnBnClickedButtonSpare1();
	afx_msg void OnBnClickedCheckNaturalPop();
	afx_msg void OnBnClickedCheckProcessPop();
	afx_msg void OnBnClickedButtonAdvancedContinue();
	afx_msg void OnBnClickedButtonTablePara();
	afx_msg void OnBnClickedCheck3();
	afx_msg void OnBnClickedCheckGray();
	afx_msg void OnBnClickedCheckGray2();
	afx_msg void OnBnClickedCheckGray3();
	afx_msg void OnBnClickedCheckTeachPop();
	afx_msg void OnBnClickedButtonSystemPara2();
	afx_msg void OnCbnSelchangeComboTableGroup2();
	afx_msg void OnBnClickedButtonTablePara2();
	afx_msg void OnBnClickedButtonPanoRecognition2();
	afx_msg void OnBnClickedButtonTablePara3();
	afx_msg void OnBnClickedButtonTablePara4();
	afx_msg void OnBnClickedButtonLoadTrack2();
	afx_msg void OnCbnSelchangeWorkpieceType();
	//}}AFX_MSG
	DECLARE_MESSAGE_MAP()

	/******************** ��ʼ���ͽ��� ********************/
public:
	BOOL InitVariable();
    void InitMoveState();
	void SetHintInfo(CString sHintInfo);
	static UINT ThreadShowMoveState(void *pParam);
    void ShowMoveState(CUnit* pUnit);
	void ShowTeachImage();
	void ShowTeachImage(IplImage *pImage);
	void SetShowImg(IplImage* pImage);
	void SaveLastCtrlState();
	bool LoadLastCtrlState();
	void LoadTableGroupScan();
	void LoadPartType(); // �������й������ͺ͵�ǰѡ�񹤼�
	void LoadDebugPara();
	void LoadOptionalFunctionPara();
	void LoadRobotandCar(CRobotDriverAdaptor* pRobotCtrl, int tablenum);
	//�ذ�ȫλ��
	void BackHome();

	/************************�¿����� 2023-12-28 Start***********************/
	//��ʼ�����п��Ƶ�Ԫ
	BOOL InitAllUnit();
	std::vector<T_CONTRAL_UNIT> m_vtContralUnitInfo;		//���Ƶ�Ԫ��Ϣ
	std::vector<CUnit*> m_vpUnit;							//���Ƶ�Ԫ����
	CServoMotorDriver* m_pServoMotorDriver = NULL;			//���ƿ�	
	COPCClientCtrl* m_pOPCClientCtrl = NULL;
	E_WORKPIECE_TYPE m_tChoseWorkPieceType = SMALL_PIECE;

	/************************�¿����� 2023-12-28 Start***********************/

	/******************** ���ܺ��� ********************/
public:
	// ��������������
	void GetCameraTool(int nRobotNo, int nCameraNo, T_ROBOT_COORS &tCameraTool);
	// ������ɨ�������
	void GetRecogCameraTool(int nRobotNo, T_ROBOT_COORS& tRecogCameraTool);		
	// ��ǹ��˿
	bool CleanGun(CRobotDriverAdaptor *pRobotCtrl);
	//��·��ǹ��˿
	bool CleanGunH(CRobotDriverAdaptor* pRobotCtrl);
	// ��������
	void SaveErrorData(CString strName = _T(""));
	// ����ʹ�õ���Դ
	void CleanUp();
	// ��ɨ�̺߳���
	static UINT ThreadScanLine(void* pParam);
	// ��ɨִ�к���
	bool ScanLine(int nRobotNo);
	bool ScanLineForVzenseCam(int nRobotNo);
	bool GetVzensePointCloud(int nRobotNo, T_CAMREA_PARAM tCamParam, std::vector<CvPoint3D64f>& vtPointCloud);
	// ����ָ���������͵Ķ���ʵ�� ����ָ��
	bool CreateObject(E_WORKPIECE_TYPE ePartType, CUnit* pUnit, WAM::WeldAfterMeasure** pWeldAfterMeasure);
	// �Ȳ���̺߳���
	static UINT ThreadWeldAfterMeasure(void *pParam);
	static UINT ThreadCheckWeldIO(void* pParam);
	int CheckWeldIO(void* pParam);
	// �Ȳ���̺߳���
	static UINT ThreadWeldAfterMeasureMultiMachine(void* pParam);
	bool WeldAfterMeasureMultiMachine();
	// �Ȳ��ִ�к���
	bool WorkWeldAfterMeasure(int nRobotNo, int& nCurGroupNo);
	// ��������ɨ��
	bool ScanWeldTrack(WAM::WeldAfterMeasure* pWeldAfterMeasure, LineOrCircularArcWeldingLine SeamData, int nRobotNo, int nGroupNo, double dSafeHeight);

	// �ַ����ָ�
	vector<string> split(const string& str, const string& delim);

	/******	WorkWeldAfterMeasure������	******/
	//���ӵ��Ⱥ���
	bool WeldSchedule(WeldAfterMeasure *pWeldData ,int nGroupNo);
	bool WeldSchedule_G(WeldAfterMeasure *pWeldData ,int nGroupNo, int nLayerNo);
	bool WeldAfterMeasureMultiMachine_G();
	static UINT ThreadGrooveWeld_G(void* pParam);
	bool FuncGrooveWeld_G(int nRobotNo, int& nCurGroupNo, int nLayerNo);
	/******	WorkWeldAfterMeasure������	******/

	/******************** ���Ժ��� ********************/

	static UINT ThreadTest(void* pParam);

	void TestGroovePointCloudProcess();
	// ���Ե��ƽӿ�������
	void TestGetEndPtnUsePointCloudWRZ();
	// ���Ե��ƽӿ�Τ����
	void TestGetEndPtnUsePointCloud();
	// �����˵������ӿڲ���
	void TestLockProcess(int nRobotNo, int nCamerNo);
	// �¿ڼ������ĵ���ȡ����
	void TestGrooveImageProcess(int nRobotNo, int nCameraNo);
	// �������ĵ���ȡ����
	void TestLaserCenterPtnImageProcess(int nRobotNo, int nCameraNo);
	// �ǵ�ͼ��ӿڲ���
	void TestImageProcess(int nRobotNo, int nCamerNo); 
	// �˵㼰������弤���߽ӿڲ���
	void TestEndpointAndStandBoardLine(int nRobotNo, int nCamerNo);
	// ���ͼ��ӿڲ���
	void TestFindEndPoint(); 
	// ָ����㡢�뾶����㷽��ǡ����ࡢ�켣Բ���Ƕ�(����ԭ360.0) ���Բ����Բ�켣�㣨Zֵ��ͬ��
	void GenetateCirclePath(XI_POINT tStartPtn, double dRadius, double dFirstPtnDirAngle, double dPtnUnit, double dTotalAngle, std::vector<XI_POINT>& vtCirclePtns);
	//����У��
	void testCompenWithPara(CRobotDriverAdaptor* pRobotCtrl, E_CAM_ID camId);
	// �����ڶ������㺸���Ժ���
	void TestContiSwaySpot();

	// vtCoord�˲�ǰ�켣 nSingleTimePtnNum�����˲�ʹ�õĹ켣���� nSampleInterval�˲������������
	bool WeldTrackLineFilter(std::vector<T_ROBOT_COORS>& vtCoord, int nSingleTimePtnNum, int nSampleInterval);

	//���Ժ���
	void TestGetEndPtnUsePointCloud_F();

	void TestGetPicturePtnToPointCloud();

	void TestScanEndptnImageProcess();

	bool BackHome_G();
	static UINT ThreadBackHome(void* pParam);
	int FuncBackHome(int nRobotNo);
	//����ɼ����ƽӿ�(ֻ�����ڰ��� ����job����ΪGETCLOUD ʾ�̵�ʱ�����Ҫ���е����)
	void GetPoinCloud();
	static UINT ThreadGetPointCloud(void *pParamnt);
	//�¿ں���
	bool m_bGrooveTeachWeldRunning = false;
	static UINT ThreadGrooveTeachWeld(void *pParam);
	int FuncGrooceTeachWeld();
	int GetGroovePara(const T_ROBOT_COORS& tRobotStartCoord, const T_ROBOT_COORS& tRobotEndCoord, vector<T_WAVE_PARA>& vtTWavePara);
	int GetFlatGroovePara(vector<T_WAVE_PARA>& vtTWavePara);
	int GetVerGroovePara(vector<T_WAVE_PARA>& vtTWavePara);
	int GetTeachPos(T_ROBOT_COORS& tRobotStartCoord, T_ROBOT_COORS& tRobotEndCoord, int nRobotNo);
	int CalWavePath(T_GROOVE_INFOR tGrooveInfor, T_INFOR_WAVE_RAND tGrooveRand, T_WAVE_PARA tWavePara, vector<T_ROBOT_COORS>& vtGrooveWavePath, vector<double>& weldSpeedRate);
	int GrooveWeld(vector<vector<T_ROBOT_COORS>> vvtGrooveWavePath, vector<T_WAVE_PARA> vtTWavePara, vector<vector<double>> vWeldSpeedRate, int nRobotNo);
	int GrooveAutoRandNew(T_GROOVE_INFOR tGrooveInfor, vector<T_WAVE_PARA> vtTWavePara, vector<T_INFOR_WAVE_RAND>& vtGrooveRand, int nRobotDir);
	bool JudgeGrooveStandWeld(WeldLineInfo tWeldLineInfo);

// ���ض������
	// ���ط�����������
	bool LoadGroupingResult(CString sFileName = "");
	bool LoadCloudProcessResultMultiMachine(CString sFileName = "");


	//��һ�㣺�ع�ÿ��ͣ��ʱ���豸�Ĺ�������
	//�ڶ��㣺�豸���������ڣ�������е�۵Ĺ�������
	//�����㣺������е�۵Ĺ��������ڣ��������
	//���Ĳ㣺һ�麸���и����������Ϣ
	std::vector <std::vector < std::vector<LineOrCircularArcWeldingLine>>> m_vvvtWeldSeamData; // ������ƴ���õ��ĺ�����Ϣ
	std::vector <std::vector < std::vector<WeldLineInfo>>> m_vvvtWeldSeamInfo; // �������ʶ���������ŵ��޷�ʶ���������Ϣ
	
	/******************** ��Ա���� ********************/
	CToolTipCtrl m_tooltip;					//���ͣ�� ��ʾ��Ϣ
	CComboBox m_comboTableGroupleft;
	CComboBox m_ctlWorkpieceType;
	CBrush m_bkBrush;
	CBrush m_Brush;
	CStatic m_static;
	CFont m_cFontCorporation;
	CFont m_cFont;
	CEdit m_cCoorX;
	CEdit m_cCoorY;
	CEdit m_cCoorZ;
	CEdit m_cCoorRX;
	CEdit m_cCoorRY;
	CEdit m_cCoorRZ;
	BOOL m_bIfWindowOn = TRUE;
	BOOL m_bStartArcOrNot;
	BOOL m_bNaturalPop;
	BOOL m_bTeachPop;
	BOOL m_bProcessPop;
	BOOL m_bMeausreThickEnable;		//�Զ����
	BOOL m_bCleanGunEnable;			//�Զ���ǹ
	BOOL m_bNeedWrap;

	UINT m_UnTracePointCloudProcess;			//���ƴ������� 0��ʹ�õ��ƴ��� 1	2��+������� 
	UINT m_UnWarpBoundWeld;						//������ǹ�������ͣ�0����ǹ 1�����ǹ 2�յ���ǹ 3��ͷ��ǹ
	BOOL m_bSaveTraceScanPic;					//�Ƿ�����ԭͼ
	
	bool m_bQuit = false;
	bool m_bThreadShowMoveStateEnd;
	bool m_bShowAllButton;					//��ʾ���а�ť
	bool HSJC;								// ��˿���
	bool m_bIfEmg;							
	bool m_bAutoWeldWorking;				// �����߳��Ƿ���
	BOOL m_bOpenRightRobotRightCam;			// �������״̬
	BOOL m_bOpenRightRobotLeftCam;
	BOOL m_bOpenLeftRobotRightCam;
	BOOL m_bOpenLeftRobotLeftCam;
	WORD m_cLeftCleanGunIOQQ;				//���������ǹ
	WORD m_cLeftCleanGunIOJS;				//������˼�˿
	WORD m_cLeftCleanGunIOJJ;				//������˼н�
	unsigned long long m_ullInitCtrlState;	// ��ť״̬��ʾ
	vector<UINT> m_vnDrawComponentID;			//���滭��ID����
	std::vector<int> m_vnCtrlID;			// �Զ�����ʹ�ܵİ�ťID����
	std::map<int, CString> m_nsPartType;	// �������ļ���ȡ�Ĺ���������Ϣ �����������ͱ��
    CMoveCtrlModule m_cMoveCtrl;			// �������ƿ����Ƶ��ⲿ�����
    vector<CScanInitModule*>m_vpScanInit;		// ����յ�ɨ��ָ��		// ����յ�ɨ��ָ��
	WAM::WeldAfterMeasure* m_pWeldAfterMeasure = NULL;	// �Ȳ��ָ��
	std::vector<IplImage*> m_vpShowLaserImgBuff;		// ͼ����ʾ ���������˺�
	std::vector<CRobotDriverAdaptor*> m_vpRobotDriver;	// ������ָ�� ���������˺�
	std::vector<CLaserLineScreen*> m_vpLaserLineScreen;	// ��ɨ��ָ�� ���������˺�
	std::vector<T_ROBOT_THREAD*> m_vtRobotThread;		// �̲߳��� ����������


	virtual void DoDataExchange(CDataExchange* pDX);
	virtual BOOL PreTranslateMessage(MSG* pMsg);

	void testCompenWithPara(CUnit* ptUnit, int CameraNo);
	afx_msg void OnBnClickedButton2();
	afx_msg void OnBnClickedButton3();
	afx_msg void OnPaint();
	afx_msg void OnDrawItem(int nIDCtl, LPDRAWITEMSTRUCT lpDrawItemStruct);

	CComboBox m_comboWorkpieceType;
	void initWorkpieceType();
	void setWorkpieceType();
	std::vector<CString> m_vsWorkpieceType;
	afx_msg void OnCbnSelchangeWorkpieceType2();
	afx_msg void OnBnClickedCheckModel();
	BOOL m_nUseModel;
};
//{{AFX_INSERT_LOCATION}}
// Microsoft Visual C++ will insert additional declarations immediately before the previous line.

//�ܵ�ͨ��
class PipeCommunication {
public:
	PipeCommunication(const std::string& filePath) : filePath(filePath) {}

	// д�ܵ��ļ�
	void WriteToPipe(const std::string& message) {
		std::ofstream outFile(filePath, std::ios::app);
		if (outFile.is_open()) {
			outFile << message << "\n"; // ÿ��д��һ�У�����ʽ��
			outFile.close();
		}
		else {
			std::cerr << "Unable to open the pipe file for writing.\n";
		}
	}

	// ���ܵ��ļ�
	std::string ReadFromPipe() {
		std::ifstream inFile(filePath);
		std::ostringstream content;
		if (inFile.is_open()) {
			std::string line;
			while (std::getline(inFile, line)) {
				content << line << "\n"; // ÿ�ζ�ȡһ��
			}
			inFile.close();
		}
		else {
			std::cerr << "Unable to open the pipe file for reading.\n";
		}
		return content.str();
	}

	// ��չܵ��ļ�
	void ClearPipe() {
		std::ofstream outFile(filePath, std::ios::trunc);
		if (!outFile.is_open()) {
			std::cerr << "Unable to open the pipe file for clearing.\n";
		}
		outFile.close();
	}

private:
	std::string filePath;
};

#endif // !defined(AFX_INCISEHEAD_H__DA65D33B_9ECF_4AE5_A0F5_67DA4502360C__INCLUDED_)
