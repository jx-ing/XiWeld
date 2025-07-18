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
//#include"NamedPipeClient.h" //鸿路临时使用 有名管道

#include "ChangeGroovePara.h"
#include "XirobotCv.h"
#include "VZenseDriver.h"

class CAssemblyWeld;

typedef struct
{
	int nGroupNo;
	int nLayerNo; // 国焊添加 第几道焊接
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

	/******************** 初始化和界面 ********************/
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
	void LoadPartType(); // 加载所有工件类型和当前选择工件
	void LoadDebugPara();
	void LoadOptionalFunctionPara();
	void LoadRobotandCar(CRobotDriverAdaptor* pRobotCtrl, int tablenum);
	//回安全位置
	void BackHome();

	/************************新框架添加 2023-12-28 Start***********************/
	//初始化所有控制单元
	BOOL InitAllUnit();
	std::vector<T_CONTRAL_UNIT> m_vtContralUnitInfo;		//控制单元信息
	std::vector<CUnit*> m_vpUnit;							//控制单元对象
	CServoMotorDriver* m_pServoMotorDriver = NULL;			//控制卡	
	COPCClientCtrl* m_pOPCClientCtrl = NULL;
	E_WORKPIECE_TYPE m_tChoseWorkPieceType = SMALL_PIECE;

	/************************新框架添加 2023-12-28 Start***********************/

	/******************** 功能函数 ********************/
public:
	// 计算跟踪相机工具
	void GetCameraTool(int nRobotNo, int nCameraNo, T_ROBOT_COORS &tCameraTool);
	// 计算线扫相机工具
	void GetRecogCameraTool(int nRobotNo, T_ROBOT_COORS& tRecogCameraTool);		
	// 清枪剪丝
	bool CleanGun(CRobotDriverAdaptor *pRobotCtrl);
	//鸿路清枪剪丝
	bool CleanGunH(CRobotDriverAdaptor* pRobotCtrl);
	// 保存数据
	void SaveErrorData(CString strName = _T(""));
	// 清理使用的资源
	void CleanUp();
	// 线扫线程函数
	static UINT ThreadScanLine(void* pParam);
	// 线扫执行函数
	bool ScanLine(int nRobotNo);
	bool ScanLineForVzenseCam(int nRobotNo);
	bool GetVzensePointCloud(int nRobotNo, T_CAMREA_PARAM tCamParam, std::vector<CvPoint3D64f>& vtPointCloud);
	// 创建指定工件类型的对象实例 返回指针
	bool CreateObject(E_WORKPIECE_TYPE ePartType, CUnit* pUnit, WAM::WeldAfterMeasure** pWeldAfterMeasure);
	// 先测后焊线程函数
	static UINT ThreadWeldAfterMeasure(void *pParam);
	static UINT ThreadCheckWeldIO(void* pParam);
	int CheckWeldIO(void* pParam);
	// 先测后焊线程函数
	static UINT ThreadWeldAfterMeasureMultiMachine(void* pParam);
	bool WeldAfterMeasureMultiMachine();
	// 先测后焊执行函数
	bool WorkWeldAfterMeasure(int nRobotNo, int& nCurGroupNo);
	// 焊缝整段扫描
	bool ScanWeldTrack(WAM::WeldAfterMeasure* pWeldAfterMeasure, LineOrCircularArcWeldingLine SeamData, int nRobotNo, int nGroupNo, double dSafeHeight);

	// 字符串分割
	vector<string> split(const string& str, const string& delim);

	/******	WorkWeldAfterMeasure主流程	******/
	//焊接调度函数
	bool WeldSchedule(WeldAfterMeasure *pWeldData ,int nGroupNo);
	bool WeldSchedule_G(WeldAfterMeasure *pWeldData ,int nGroupNo, int nLayerNo);
	bool WeldAfterMeasureMultiMachine_G();
	static UINT ThreadGrooveWeld_G(void* pParam);
	bool FuncGrooveWeld_G(int nRobotNo, int& nCurGroupNo, int nLayerNo);
	/******	WorkWeldAfterMeasure主流程	******/

	/******************** 测试函数 ********************/

	static UINT ThreadTest(void* pParam);

	void TestGroovePointCloudProcess();
	// 测试点云接口王润泽
	void TestGetEndPtnUsePointCloudWRZ();
	// 测试点云接口韦富进
	void TestGetEndPtnUsePointCloud();
	// 搜索端点锁定接口测试
	void TestLockProcess(int nRobotNo, int nCamerNo);
	// 坡口激光中心点提取测试
	void TestGrooveImageProcess(int nRobotNo, int nCameraNo);
	// 激光中心点提取测试
	void TestLaserCenterPtnImageProcess(int nRobotNo, int nCameraNo);
	// 角点图像接口测试
	void TestImageProcess(int nRobotNo, int nCamerNo); 
	// 端点及左侧立板激光线接口测试
	void TestEndpointAndStandBoardLine(int nRobotNo, int nCamerNo);
	// 测厚图像接口测试
	void TestFindEndPoint(); 
	// 指定起点、半径、起点方向角、点间距、轨迹圆弧角度(完整原360.0) 输出圆弧或圆轨迹点（Z值相同）
	void GenetateCirclePath(XI_POINT tStartPtn, double dRadius, double dFirstPtnDirAngle, double dPtnUnit, double dTotalAngle, std::vector<XI_POINT>& vtCirclePtns);
	//手眼校验
	void testCompenWithPara(CRobotDriverAdaptor* pRobotCtrl, E_CAM_ID camId);
	// 立焊摆动连续点焊测试函数
	void TestContiSwaySpot();

	// vtCoord滤波前轨迹 nSingleTimePtnNum单次滤波使用的轨迹点数 nSampleInterval滤波输入点采样间隔
	bool WeldTrackLineFilter(std::vector<T_ROBOT_COORS>& vtCoord, int nSingleTimePtnNum, int nSampleInterval);

	//测试函数
	void TestGetEndPtnUsePointCloud_F();

	void TestGetPicturePtnToPointCloud();

	void TestScanEndptnImageProcess();

	bool BackHome_G();
	static UINT ThreadBackHome(void* pParam);
	int FuncBackHome(int nRobotNo);
	//相机采集点云接口(只适用于安川 调用job名称为GETCLOUD 示教的时候必须要运行到起点)
	void GetPoinCloud();
	static UINT ThreadGetPointCloud(void *pParamnt);
	//坡口函数
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

// 加载多机数据
	// 加载分组后点云数据
	bool LoadGroupingResult(CString sFileName = "");
	bool LoadCloudProcessResultMultiMachine(CString sFileName = "");


	//第一层：地轨每次停下时，设备的工作区域
	//第二层：设备工作区域内，各个机械臂的工作区域
	//第三层：各个机械臂的工作区域内，焊缝的组
	//第四层：一组焊缝中各个焊缝的信息
	std::vector <std::vector < std::vector<LineOrCircularArcWeldingLine>>> m_vvvtWeldSeamData; // 多机点云处理得到的焊缝信息
	std::vector <std::vector < std::vector<WeldLineInfo>>> m_vvvtWeldSeamInfo; // 多机包括识别结果及焊脚等无法识别的属性信息
	
	/******************** 成员变量 ********************/
	CToolTipCtrl m_tooltip;					//鼠标停留 提示信息
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
	BOOL m_bMeausreThickEnable;		//自动测厚
	BOOL m_bCleanGunEnable;			//自动清枪
	BOOL m_bNeedWrap;

	UINT m_UnTracePointCloudProcess;			//点云处理类型 0不使用点云处理 1	2存+处理点云 
	UINT m_UnWarpBoundWeld;						//包角跳枪焊接类型：0不跳枪 1起点跳枪 2终点跳枪 3两头跳枪
	BOOL m_bSaveTraceScanPic;					//是否存跟踪原图
	
	bool m_bQuit = false;
	bool m_bThreadShowMoveStateEnd;
	bool m_bShowAllButton;					//显示所有按钮
	bool HSJC;								// 焊丝检测
	bool m_bIfEmg;							
	bool m_bAutoWeldWorking;				// 工作线程是否开启
	BOOL m_bOpenRightRobotRightCam;			// 相机开光状态
	BOOL m_bOpenRightRobotLeftCam;
	BOOL m_bOpenLeftRobotRightCam;
	BOOL m_bOpenLeftRobotLeftCam;
	WORD m_cLeftCleanGunIOQQ;				//左机器人清枪
	WORD m_cLeftCleanGunIOJS;				//左机器人剪丝
	WORD m_cLeftCleanGunIOJJ;				//左机器人夹紧
	unsigned long long m_ullInitCtrlState;	// 按钮状态显示
	vector<UINT> m_vnDrawComponentID;			//界面画布ID集合
	std::vector<int> m_vnCtrlID;			// 自动控制使能的按钮ID集合
	std::map<int, CString> m_nsPartType;	// 从配置文件读取的工件类型信息 索引工件类型编号
    CMoveCtrlModule m_cMoveCtrl;			// 雷赛控制卡控制的外部轴对象
    vector<CScanInitModule*>m_vpScanInit;		// 起点终点扫描指针		// 起点终点扫描指针
	WAM::WeldAfterMeasure* m_pWeldAfterMeasure = NULL;	// 先测后焊指针
	std::vector<IplImage*> m_vpShowLaserImgBuff;		// 图像显示 索引机器人号
	std::vector<CRobotDriverAdaptor*> m_vpRobotDriver;	// 机器人指针 索引机器人号
	std::vector<CLaserLineScreen*> m_vpLaserLineScreen;	// 线扫类指针 索引机器人号
	std::vector<T_ROBOT_THREAD*> m_vtRobotThread;		// 线程参数 索引机器号


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

//管道通信
class PipeCommunication {
public:
	PipeCommunication(const std::string& filePath) : filePath(filePath) {}

	// 写管道文件
	void WriteToPipe(const std::string& message) {
		std::ofstream outFile(filePath, std::ios::app);
		if (outFile.is_open()) {
			outFile << message << "\n"; // 每次写入一行，不格式化
			outFile.close();
		}
		else {
			std::cerr << "Unable to open the pipe file for writing.\n";
		}
	}

	// 读管道文件
	std::string ReadFromPipe() {
		std::ifstream inFile(filePath);
		std::ostringstream content;
		if (inFile.is_open()) {
			std::string line;
			while (std::getline(inFile, line)) {
				content << line << "\n"; // 每次读取一行
			}
			inFile.close();
		}
		else {
			std::cerr << "Unable to open the pipe file for reading.\n";
		}
		return content.str();
	}

	// 清空管道文件
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
