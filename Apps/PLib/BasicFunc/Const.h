#if !defined(AFX_CONST_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_)
#define AFX_CONST_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_

#include "Log.h"
#include "CommonAlgorithm.h"
#include "CvvImage.h"
#include <VECTOR>
#include <MAP>
#include "highgui.h"
#include "cxcore.h"
#include "cv.h"
#include ".\Apps\PLib\BasicFunc\BaseParam.h"
#include "FileProcess.h"
#include ".\Apps\PLib\Database\CCommonParaByDatabase.h"
#include <XiAlgorithm.h>
#include <i_path_planning.h>
#include "Apps/SmallPiece/Infrastructure/UI/MesBox.h"
#include "Apps/PLib/BasicFunc/RunPara.h"

#ifndef SINGLE_ROBOT
//#define SINGLE_ROBOT  //单机器人运行 带外部轴运行注释掉,确定外部轴与机器人轴平行。定义该宏与Y轴平行，反之与X轴平行
#endif

//// 线扫参数
#define CAPTURE_IMAGE_FRAME_RATE		    100     	// 线扫采图帧率
#define MAX_PROCESS_THREAD_NUM				4			// 线扫图像处理线程数量
#define	LINE_SCAN_BINARY_THRESHOLD			30			// 线扫图像处理激光线亮度阈值
#define LINE_SCAN_STEP_2D_POINT				1			// 线扫图像处理二维点采样间隔
#define LINE_SCAN_LASER_WIDTH				10			// 线扫图像处理激光线宽度
#define LINESCAN_CAMERA_NUM					1			//线扫相机个数	
#define SAVE_IMAGE_THREAD_MUN				4			//存图线程数量
//#define LINE_SCAN_SAVE_IMAGE

// 运行信息统计
//判断白班和夜班
enum ShiftType {
	DayShift = 0,
	NightShift = 1,
	InvalidShift = 2,
	SecDay = 3
};

extern int m_nWorkState; //判断当前工作是白班还是夜班
extern CString m_sFirstOpenTime; // 首次打开时间 年/月/日 时:分:秒
extern CString m_sLastCloseTime; // 最后关闭时间 年/月/日 时:分:秒
extern long m_lTotalOpenTime;	// 总打开时间 ms
extern long m_lTotalColseTime;	// 总断开时间 ms

extern long m_lWorkTime;			// 扫描、交互、示教、焊接 等总工作时间
extern long m_lStandbyTime;			// 待机时间 = 总联机时间 - 工作时间
extern long m_lFlatWeldTime;		// 平焊时间
extern long m_lStandWeldTime;		// 立焊时间
extern long m_lScanTime;			// 扫描时间 线扫时间 + 示教时间
extern long m_lScanWeldTime;		// 扫描时间 + 平焊时间 + 立焊时间

extern double m_dFlatWeldLen;	// 平焊长度
extern double m_dStandWeldLen;	// 立焊长度
extern double m_dTotalWeldLen;	// 焊缝总长度 平焊长度 + 立焊长度
extern double m_dTotalScanLen;	// 总线扫扫描长度

extern long lCurOpenTimeStamp; // 本地打开时间戳
extern double m_dCurWeldLenBeforeCleanGun;

template<typename T>
inline auto square(T value)
{
	return value * value;
}

//大恒相机图像翻转模式
typedef enum E_FLIP_MODE {
	E_FLIP_NONE = 0,				//不翻转
	E_FLIP_VERTICAL,				//沿垂直轴翻转（左右翻转）
	E_FLIP_HORIZEN,					//沿水平轴翻转（上下翻转）
	E_FLIP_BOTH						//水平+垂直 翻转
}E_FLIP_MODE;

typedef enum
{
	E_EX_X = 1 << 0,
	E_EX_Y = 1 << 1,
	E_EX_Z = 1 << 2,
	E_EX_YASKAWA_X = 1 << 3,
	E_EX_YASKAWA_Y = 1 << 4,
	E_EX_YASKAWA_Z = 1 << 5,
	E_EX_PANASONIC_X = 1 << 6,
	E_EX_PANASONIC_Y = 1 << 7,
	E_EX_PANASONIC_Z = 1 << 8
}E_EXTERNAL_AXLE_TYPE;

typedef enum
{
	E_DIAPHRAGM		= 0,	// ！隔板
	E_LINELLAE		= 1,	// ！线条 黄埔工件(小部件跟踪)
	SMALL_PIECE		= 2,	// ！小散件(先测后焊)
	STIFFEN_PLATE	= 3,	// ！加劲板
	E_PURLIN_HANGER = 4,	// ！檩托
	E_END_PLATE		= 5,	// ！端板
	E_CORBEL		= 6,	// ！牛腿
	E_CLOSEDARC		= 7,	// 单一闭合弧圈
	E_CAKE			= 8,    // 佳木斯电厂
	E_BEVEL         = 9,    // 坡口
	E_SINGLE_BOARD = 10,	// 单板单筋
	E_SCAN_WELD_LINE = 11,	// 扫描测量
	E_EMPTY_ERROR	= 12
}E_WORKPIECE_TYPE;

typedef struct T_WEAVE_FULLPARA
{
	CString sWave;
	int nNo;
	int n1;
	int n2;
	int n3;
	CString s8bit;
	double d1;
	double d2;
	double dRotAngleX;
	double dRotAngleZ;
	double dAmpL;
	double dAmpR;
	double dFreq;
	double d3;
	double dTimeL;
	double d4;
	double dTimeC;
	double d5;
	double dTimeR;
	double d6;
	double d7;
	int n4;
	double d8;
	int n5;
	double d9;
	int n6;
	double d10;
}T_WEAVE_FULLPARA;

typedef struct
{
	double dDirX;
	double dDirY;
	double dDirZ;
}T_SPACE_LINE_DIR;

//2024/01/03 焊接加
typedef struct
{
	IplImage* pImage;
	int nImageNo;
	CString cstrId;
	CString cstrRobotName;
	BOOL bSaveImageStaus[SAVE_IMAGE_THREAD_MUN];
}T_SAVE_IMAGE;

typedef struct T_LINE_PARA
{
	double dPointX;
	double dPointY;
	double dPointZ;
	double dDirX;
	double dDirY;
	double dDirZ;
	double dLineError;
}T_LINE_PARA;

typedef struct
{
	double dX;
	double dY;
	double dZ;
	double dU;
}T_CART_COOR;

//2023/12/21 焊接加

#define CHECK_BOOL(bRst) \
if (false == bRst) \
{ \
	return; \
}

#define CHECK_BOOL_RETURN(bRst) \
if (false == bRst) \
{ \
	return false; \
}

#define CHECK_BOOL_CONTINUE(bRst) \
if (false == bRst) \
{ \
	continue; \
}

#define CHECK_BOOL_BREAK(bRst) \
if (false == bRst) \
{ \
	break; \
}

#define CHECK_RESULT_NO_RET(re) \
{ \
    if (FALSE == re) \
    { \
        CString str, strTemp; \
        strTemp.Format(" line %d", __LINE__); \
        str = __FILE__ + strTemp + ": 返回值为FALSE"; \
        XiMessageBox(str); \
    } \
}

#define CHECK_PULSE_RETURN(pRobotDriver, tDstRobotPulse) \
if (false == pRobotDriver->ComparePulse(tDstRobotPulse, pRobotDriver->GetCurrentPulse())) \
{ \
	WriteLog("Check Pulse Err1"); \
	return RUNNING_STATUS_CHECK_PULSE_ERROR; \
}

#define CHECK_PULSE_RETURN_BOOL(pRobotDriver, tDstRobotPulse) \
if (false == pRobotDriver->ComparePulse(tDstRobotPulse, pRobotDriver->GetCurrentPulse())) \
{ \
	WriteLog("Check Pulse Err2"); \
	return false; \
}

#define CHECK_COORS_RETURN(pRobotDriver, tDstRobotCoors) \
if(false == pRobotDriver->CompareCoords(tDstRobotCoors, pRobotDriver->GetCurrentPos())) \
{ \
	WriteLog("Check Coors Err1"); \
	return RUNNING_STATUS_CHECK_COORS_ERROR; \
}

#define CHECK_COORS_RETURN_BOOL(pRobotDriver, tDstRobotCoors) \
if(false == pRobotDriver->CompareCoords(tDstRobotCoors, pRobotDriver->GetCurrentPos())) \
{ \
	WriteLog("Check Coors Err2"); \
	return false; \
}


// #define CHECK_RESULT_NO_RET(re) \
// { \
//     if (FALSE == re) \
//     { \
//         CString str, strTemp; \
//         strTemp.Format(" line %d", __LINE__); \
//         str = __FILE__ + strTemp + ": 返回值为FALSE"; \
//         XiMessageBox(str); \
//     } \
// }

#define CHECK_FALSE_RETURN(bResult)\
{\
    if (!bResult)\
    {\
        return false; \
    }\
}

#define CHECK_FILE_POINTER_RET_PARA(p, para) \
{ \
    if (NULL == (p)) \
    { \
        CString str, strTemp; \
        strTemp.Format(" line %d", __LINE__); \
        str = __FILE__ + strTemp + ": 文件打开错误"; \
        XiMessageBox(str); \
        return para; \
    } \
}

#define DELETE_POINTER(p) \
    if (NULL != p) \
{ \
    delete p; \
    p = NULL; \
}

#define DELETE_POINTER_ARRAY(p) \
    if (NULL != p) \
{ \
    delete [] p; \
    p = NULL; \
}

//#define DELETE_POINTER_2D_ARRAY(p, num) \
//    if (NULL != p) \
//{ \
//	for(int i = 0; i < num; i++) \
//	{ \
//		if (NULL != p[i]) \
//		{ \
//			delete[] p[i]; \
//			p[i] = NULL; \
//		}\
//	} \
//	delete [] p; \
//	p = NULL; \
//}

#define  CHECK_OKCANCEL(OkOrCancel)  if (IDCANCEL == OkOrCancel) \
{ \
    return ; \
}

#define GET_CUR_POS GetStr("Function:%s	Line:%d -> ", __FUNCTION__, __LINE__)

#define  RTN_STATE_DATA(tRtnResult, eRtnState, strRtnExplain, nDataType, vData) AddRtnData(tRtnResult, eRtnState, __FUNCTION__, __LINE__, strRtnExplain, nDataType, vData); \
return tRtnResult; 

#define  RTN_STATE_EXPLAIN(tRtnResult, eRtnState, strRtnExplain) AddRtnData(tRtnResult, eRtnState, __FUNCTION__, __LINE__, strRtnExplain); \
return tRtnResult; 

#define  RTN_STATE_RESULT(tRtnResult, eRtnState) AddRtnData(tRtnResult, eRtnState, __FUNCTION__, __LINE__); \
return tRtnResult; 

#define  CHECK_RTN_RESULT(tRtnResult, tRtnData) if (!AddRtnData(tRtnResult, tRtnData)) \
{\
RTN_STATE_RESULT(tRtnResult, tRtnResult.veRtnState.back());\
}


#define  CHECK_RTN_EXPLAIN(tRtnResult, tRtnData, strRtnExplain) if (!AddRtnData(tRtnResult, tRtnData)) \
{\
RTN_STATE_EXPLAIN(tRtnResult, tRtnResult.veRtnState.back(), strRtnExplain);\
}

#define  ADD_RTN_RESULT(tRtnResult, tRtnData) AddRtnData(tRtnResult, tRtnData);

#define  ADD_RTN_STATE(tRtnResult, eRtnState) AddRtnData(tRtnResult, eRtnState, __FUNCTION__, __LINE__);

#define  ADD_RTN_EXPLAIN(tRtnResult, eRtnState, strRtnExplain) AddRtnData(tRtnResult, eRtnState, __FUNCTION__, __LINE__, strRtnExplain);

#define  DEBUG_MESSAGE_RTN_EXPLAIN(tRtnResult, str) if (TRUE == DEBUG_MODE)\
{\
if (IDOK != XiMessageBox(GetStr("Function:[%s]\r\nLine:[%d]\r\n\r\n%s", __FUNCTION__, __LINE__, str)))\
{\
RTN_STATE_EXPLAIN(tRtnResult, RTN_STATUS_MANUAL_CANCEL,str);\
}\
else\
{\
ADD_RTN_EXPLAIN(tRtnResult, RTN_STATUS_MANUAL_CONTINUE, str); \
}\
}

#ifndef SQUARE
#define SQUARE(x) (((double)x)*((double)x))
#endif

#define __max(a,b) (((a) > (b)) ? (a) : (b))
#define __min(a,b) (((a) < (b)) ? (a) : (b))

//得到应用程序所在路径名
void GetModulePath(char *cPosfix);
void GetModulePath(const char *cPosfix);
extern char gcModulePath[500];
extern char gcModuleWholePath[500];

//实时记录当前机器人坐标所用
extern char gcLogModulePath[500];
extern char gcLogModuleWholePath[500];
void GetLogModulePath(char *cPosfix);
void GetLogModulePath(const char *cPosfix);

////////////////////////////        常数      ////////////////////////////////////
const CString FILE_ERROR = _T("文件异常！");

// const CString CUTTING_ROBOT = _T("CuttingRobot");
// const CString HANDLING_ROBOT = _T("HandlingRobot");

const int PULSE_COORD = 1;
const int ROBOT_COORD = 0;

const int PanoImageWidth = 1024;
const int PanoImageHeight = 1024;

const int LaserImageWidth = 2592;
const int LaserImageHeight = 1944;

const int GrayscaleImageWidth = 2448;
const int GrayscaleImageHeight = 2048;

// const int LineScanImageWidth = 2448;
// const int LineScanImageHeight = 2048;

const long    SCREENX        =   1280                               ;   // 屏幕显示最大像素值：长
const long    SCREENY        =   1024                               ;   // 屏幕显示最大像素值：宽

//const double  PI = 3.1415926535897932384626433832795;

const int cnToolRotate = 90;//一圈角度等分份数

////////////////////////////      文件路径     ///////////////////////////////////

//配置文件
const CString CONTRAL_UNIT_INI					=	_T("ConfigFiles\\ContralUnitInfo.ini");
const CString DEBUG_INI							=	_T("ConfigFiles\\Debug.ini");
const CString DEPTH_CAMERA_PARA_INI				=	_T("ConfigFiles\\DepthCameraParam.ini");
const CString INTERFACE_CTRL_INI				=	_T("ConfigFiles\\InterfaceCtrlPara.ini");//需要处理
const CString IO_PARA							=	_T("ConfigFiles\\IOPara.ini");
const CString JOB_INI							=	_T("ConfigFiles\\JOB.ini");
const CString OPTIONAL_FUNCTION					=	_T("ConfigFiles\\OptionalFunction.ini");
const CString ORACLE_INI						=	_T("ConfigFiles\\Oracle.ini");
const CString PAUSE_INI							=	_T("ConfigFiles\\Pause.ini");
const CString PS_INI							=	_T("ConfigFiles\\ps.ini");
const CString ROBOT_TRANS_RELATION				=	_T("\\RobotTransRelation.ini");
const CString CONTRAL_UNIT_INFO_INI				=	_T("ConfigFiles\\ContralUnitInfo.ini");
const CString SYSTEM_PARA_INI					=	_T("ConfigFiles\\SystemPara.ini");
const CString USER_INFO_INI						=	_T("ConfigFiles\\UserInfo.ini");
const CString XI_ROBOT_CTRL_INI					=	_T("Data\\XiRobotCtrl.ini");
const CString PROCESS_PARA_EXCEL				=	_T("ConfigFiles\\工艺表.xlsx");
const CString MAT_GANTRY_ROBOT					=	_T("ConfigFiles\\MatGantryRobot.ini");

//工作台相关
const CString TABLE_PATH						=	_T(".\\ConfigFiles\\Table\\");
const CString TABLE_GROUP_INI					=	_T("ConfigFiles\\Table\\TableGroup.ini");
const CString TABLE_INI							=	_T("ConfigFiles\\Table.ini");//待修改

/*********************** 新文件结构添加 Start ***********************/
const CString DATA_PATH							=	_T(".\\ConfigFiles\\");
const CString SERVO_CTRL_DATA_INI				=	_T("ConfigFiles\\ServoCtrlData.ini");
const CString IO_PARA_DEVICE					=	_T("ConfigFiles\\IOParamDevice.ini");
const CString IO_PARA_ROBOT						=	_T("\\IOParamRobot.ini");
const CString LINE_SCAN_PARAM					=	_T("\\LineScanParamOld.ini");
const CString SYETEM_PARAM_FILE					=	_T("\\SystemParam.ini");
const CString WELD_PARAM_FILE					=	_T("\\WeldParam.ini");
const CString WELD_PARAM_FLAT_FILE				=	_T("\\WeldParamFlat.txt");
const CString WELD_PARAM_STAND_FILE				=	_T("\\WeldParamStand.txt");
const CString WRAP_PARAM_FILE					=	_T("\\WeldWrapAngleParam.ini");
const CString WRAP_LIBRARY_FILE					=	_T("\\WrapAnglelibrary.txt");
const CString WELD_GROOVE_FILE                  =   _T("\\WeldGrooveFlat.txt");
const CString Clean_Gun							= _T("\\CleanGun.ini");
const CString OUTPUT_PATH						=	_T(".\\LocalFiles\\OutputFiles\\");
const CString WELDDATA_PATH						=	_T(".\\WeldData\\");
const CString ERROR_PATH						=	_T("\\Error\\");
const CString TRACEPROCRESULT_PATH				=	_T("\\TraceProcResult\\");
const CString TEACH_SRC_PATH					=	_T("\\Teach\\Src");
const CString TEACH_PRO_PATH					=	_T("\\Teach\\Pro");
const CString SEARCH_SCANLOCK_IMG				=	_T("\\Search\\Scan_lock.jpg");
const CString SEARCH_SRC_PATH					=	_T("\\Search\\Src");
const CString SEARCH_PRO_PATH					=	_T("\\Search\\Pro");
const CString RECOGNITION_FOLDER				=	_T("\\Recognition\\");
const CString RECOGNITION_SRC_PATH				=	_T("\\Recognition\\Src");
const CString RECOGNITION_PRO_PATH				=	_T("\\Recognition\\Pro");
const CString POINT_CLOUD_IMAGE					=	_T("Recognition\\LineScanImage\\");
const CString POINT_CLOUD_FILE					=	_T("Recognition\\PointCloud.txt");
const CString POINT_CLOUD_IDENTIFY_RESULT_JSON	=	_T("Recognition\\PointCloudIdentifyReault.json");
const CString IDENTIFY_RESULT_SAVE_JSON			=	_T("Recognition\\PointCloudIdentifyReaultAfter.json");
const CString POINT_CLOUD_IDENTIFY_RESULT		=	_T("Recognition\\PointCloudIdentifyReault.txt");
const CString IDENTIFY_RESULT_SAVE				=	_T("Recognition\\PointCloudIdentifyReaultAfter.txt");
const CString IDENTIFY_RESULT_SAVE_OLD			=	_T("Recognition\\PointCloudIdentifyReaultAfterOld.txt");
const CString ERROR_DATA_PATH					=	_T("LocalFiles\\OutputFiles\\ERROR");
const CString GROOVE_DATA_PATH					=	_T("\\Groove\\");
const CString REMOVE_CLOUD_PATH					=	_T("\\Recognition\\RemovePointCloud_0.txt");

/*********************** 新文件结构添加 End ***********************/

//以机械臂划分的配置文件
const CString GENERAL_CONTROL_PATH				=	_T(".\\ConfigFiles\\GeneralControl\\");
const CString CAMERA_PARAM_INI					=	_T("\\CameraParam.ini");
const CString MOTOR_PARAM_INI					=	_T("\\MotorParam.ini");
const CString COMMON_IO_PARAM_INI				=	_T("\\IOParam.ini");
const CString FIND_CORNNER_INI					=	_T("\\FindCornnerParam.ini");
const CString PLASMA_PARA_INI					=	_T("\\PlasmaPara.ini");
const CString PROCESS_PARA_INI					=	_T("\\ProcessPara.ini");
const CString PRODUCTION_DATA_INI				=	_T("\\ProductionData.ini");
const CString ROBOT_IO_PARA						=	_T("\\RobotIOPara.ini");
const CString ROBOT_PARA_INI					=	_T("\\RobotPara.ini");
const CString ROBOT_PAUSE_INI					=	_T("\\RobotPause.ini");
const CString SPEED_SAFE_HEIGHT_INI				=	_T("\\SpeedAndSafeHeight.ini");
const CString CLUSTER_DIFF_DEPTH_PARTS_INI		=	_T("\\panoRecognition\\ClusterDiffDepthPartsParam.ini");
const CString COMPOSE_IMAGE_INI					=	_T("\\panoRecognition\\ComposeImage.ini");
const CString FIND_3D_CONTOURS_INI				=	_T("\\panoRecognition\\Find3dContoursParam.ini");
const CString SINGLE_COMPENSATION_INI			=	_T("\\panoRecognition\\SingleCompensation.ini");
const CString WORKBENCH_GRAY_CAMERA_PARA		=	_T("\\panoRecognition\\WorkbenchGrayCameraParam.ini");
const CString CUR_PICTURES						=	_T("Pics\\CurPictures\\");
const CString GRAY_IMAGES						=	_T("Pics\\GrayImages\\");
const CString LASER_CAMERA_CAL					=	_T("Pics\\LaserCameraCal\\");
const CString LASER_IMAGES						=	_T("Pics\\LaserImages\\");
const CString DEPTH_PICTURES					=	_T("ProPics\\DepthPics\\");
const CString GRAYSCALE_PICTURES				=	_T("ProPics\\GrayscalePics\\");
const CString MANUALLY_PICTURES					=	_T("ProPics\\ManuallyPics\\");
const CString ORIGINAL_PICTURES					=	_T("ProPics\\OriginalPics\\");
const CString PROCESSED_PICTURES				=	_T("ProPics\\ProcessedPics\\");

//可以不存在但需要自动生成的路径
const CString DXF_FILES							=	_T("DXF\\GraphData\\");
const CString GRAPH_DATA_FILES					=	_T("GraphData\\");
const CString MONITOR_PATH						=	_T("Monitor\\");
const CString SYSTEM_LOG_PATH					=	_T("Monitor\\SystemLog\\");
const CString OPC_LOG_PATH						=	_T("Monitor\\OPCLog\\");
const CString PRODUCTION_DATA					=	_T("生产数据\\");
const CString LASER_SCAN_CAMERA_PICTURES		=	_T("ProPics\\LaserScanPics\\");

//需要存在的路径
const CString CAD_PROCESS_PARA_PATH				=	_T("ProcessParaTxt\\");
const CString ROBOT_INSTRUCTION					=	_T("说明\\机械臂\\");
const CString DH_CAMERA_INSTRUCTION				=	_T("说明\\跟踪与灰度相机\\");

typedef enum
{
	INCISEHEAD_THREAD_STATUS_START			= 0,					//运行
	INCISEHEAD_THREAD_STATUS_READY			= 1,					//准备
	INCISEHEAD_THREAD_STATUS_ENTER			= 2,					//下枪
	INCISEHEAD_THREAD_STATUS_TEACH			= 3,					//示教
	INCISEHEAD_THREAD_STATUS_INCISE			= 4,					//切割
	INCISEHEAD_THREAD_STATUS_LEAVE			= 5,					//提枪
	INCISEHEAD_THREAD_STATUS_STOPPED		= 6,					//暂停
	INCISEHEAD_THREAD_STATUS_CONTINUE		= 7,					//继续
	INCISEHEAD_THREAD_STATUS_COMPLETE		= 8,					//完成
	INCISEHEAD_THREAD_STATUS_FEEDING		= 9,					//上料
	INCISEHEAD_THREAD_STATUS_BLANKING		= 10,					//下料
	INCISEHEAD_THREAD_STATUS_ANTIFEEDING	= 11,					//反上料
	INCISEHEAD_THREAD_STATUS_DEPTH_RECOG	= 12,					//深度识别
	INCISEHEAD_THREAD_STATUS_GRAY_RECOG		= 13,					//灰度识别
	INCISEHEAD_THREAD_STATUS_BACK_HOME		= 14,					//返回固定位置
	INCISEHEAD_THREAD_STATUS_PICK_UP		= 15,					//抓取
	INCISEHEAD_THREAD_STATUS_TURN_OVER		= 16,					//翻面
	INCISEHEAD_THREAD_STATUS_POLISH			= 17					//打磨
}E_INCISEHEAD_THREAD_STATUS;

////2023/12/21 焊接加
extern E_INCISEHEAD_THREAD_STATUS g_eThreadStatus;

const int INCISEHEAD_THREAD_STATUS_NUM = 18;
const CString INCISEHEAD_THREAD_STATUS_NAME[INCISEHEAD_THREAD_STATUS_NUM] = 
				{ "运行","准备","下枪","示教","切割","提枪","暂停","继续","完成","上料",
				"下料","反上料","深度识别","灰度识别","返回固定位置","抓取","翻面","打磨" };

//只决定IO加载的类型，暂停继续文件的读写项，不决定机械臂的运动
typedef enum
{
	CUTTING_ROBOT = 0x01,
	HANDLING_ROBOT = 0x02,
	POLISHING_ROBOT = 0x04,
}E_ROBOT_TYPE;

//仅适用于RobotDriverAdaptor
typedef enum
{
	ROBOT_AXIS_X = 0,
	ROBOT_AXIS_Y = 1,
	ROBOT_AXIS_Z = 2,
	ROBOT_AXIS_RX = 3,
	ROBOT_AXIS_RY = 4,
	ROBOT_AXIS_RZ = 5,
	ROBOT_AXIS_BPX = 6,
	ROBOT_AXIS_BPY = 7,
	ROBOT_AXIS_BPZ = 8,
}ROBOT_AXIS;

//仅适用于RobotDriverAdaptor
typedef enum 
{
	SAxis = 0,
	LAxis = 1,
	UAxis = 2,
	RAxis = 3,
	BAxis = 4,
	TAxis = 5,
	BPXAxis = 6,
	BPYAxis = 7,
	BPZAxis = 8,
}E_ROBOT_PULSE_AXIS;

typedef enum
{
	Unknow = 0,
	DX200 = 1,
	YRC1000 = 2,
}E_ROBOT_CONTROL_CABINET_MODEL;

typedef enum E_ROBOT_COOR
{
	E_JOINT_COOR = 0,
	E_CART_COOR	
}E_ROBOT_COOR;

typedef enum
{
	E_ROBOT_UNKOWN = -1,
	E_ROBOT_GP12 = 0,
	E_ROBOT_GP25,
	E_ROBOT_GP88,
	E_ROBOT_GP180,
	E_ROBOT_GP225,
	E_ROBOT_GP280L,
	E_ROBOT_GP360L,
	E_ROBOT_MH12,
	E_ROBOT_MH24,
	E_ROBOT_MH50_20,
	E_ROBOT_MH225,
	E_ROBOT_MA2010,
	E_ROBOT_AR2010,
	E_ROBOT_ESTUN,
	E_ROBOT_FANUC2000,
	E_ROBOT_FANUC700,
	E_ROBOT_CROBOTP,
	E_ROBOT_EFORT
}E_MANIPULATOR_TYPE;

typedef enum
{
	E_PART_OBVERSE_CUT,
	E_PART_REVERSE_CUT
}E_PART_CUT_TYPE;

typedef enum
{
	E_PART_ONEMAGNETIC,
	E_PART_TWOMAGNETIC
}E_PART_TURNOVER_TYPE;


typedef struct
{
	double x;
	double y;
	double z;
	double dAngleU;
	double dAngleV;
	double dLength;
	double dThickness;
	int nEntityNo;
}T_PROCESS_POINT;

typedef struct
{
	int nCameraNo;
	int nCameraPos;
	int nTemplateNo;
	double dPartHeight;
	double dAngleRotate;
	double dTranslateX;
	double dTranslateY;
	int nLayerNo;
	int nGroupNo;
}T_RECOGNITION_PARA;


typedef struct
{
	bool bEnable;
	double dPulse;
	long lMaxPulseNum;
	long lMinPulseNum;
}T_EXTERNAL_AXLE_INFO;

typedef struct
{
	int nContourNo;
	int nBevelUpNum;
	int nBevelDownNum;
}T_CONTOUR_BEVEL_DATA;

typedef struct
{
	int  nPartNo;
	std::vector<T_CONTOUR_BEVEL_DATA> vtContourBevelData;
}T_BEVELINFO_PRECISECUT;

typedef struct 
{
	double dDirX;
	double dDirY;
	double dDirZ;
}T_XIGEO_LINE_DIR;

typedef struct
{
	XI_POINT tPoint;
	T_XIGEO_LINE_DIR tLineDir;
}T_XIGEO_LINE_PARA;

//2023/12/21 焊接加
typedef enum
{
	ROBOT_CAR_SUCCESS = 0,
	ROBOT_CAR_STOP = 1,		//空闲
	ROBOT_CAR_RUN = 2,		//运行中
	Y_COOR_OVERRUN = 3,		//Y轴超限
	X_COOR_OVERRUN = 4,		//X轴超限
	Z_COOR_OVERRUN = 5,		//Z轴超限
	WELD_ROBOT_LEFT = 6,		//机器人在X轴左侧
	WELD_ROBOT_RIGHT = 7,		//机器人在X轴右侧
	ROBOT_MOVE_FAILE = 8,     //机器人运动失败
	CAR_MOVE_FAILE = 9         //大车运动失败
}E_ROBOT_CAR_COORS_ERROR;

//2023/12/21 焊接加
typedef enum
{
	E_WRAPANGLE_EMPTY_SINGLE = 0,	// 单面焊接不包脚
	E_WRAPANGLE_EMPTY_DOUBLE,		// 双面焊接不包脚	
	E_WRAPANGLE_TWO_SINGLE,			// 单面焊接分两次包角
	E_WRAPANGLE_TWO_DOUBLE,			// 双面焊接分两次包角
	E_WRAPANGLE_ONCE,				// 双面焊一次包角
	E_WRAPANGLE_JUMP				// 跳枪一次包角
}E_WRAPANGLE_TYPE;

//2023/12/21 焊接加
struct T_BOARD_SIZE
{
	int			BoardNum;		// 板号
	XI_POINT	tStartPoint;	// 起点
	XI_POINT	tEndPoint;		// 终点
	int			dArc;			// 是否为圆弧 
	double		dBoardlength;	// 工件长度
	int			dStartInterf;	// 起点是否干涉
	double		dStartNormal;	// 起点法相
	int			dEndInterf;		// 终点是否干涉
	double		dEndNormal;		// 终点法相
	double		dBoardHeight;	// 工件高度Z
	double		dBoardThick;	// 工件厚度
	int			dOrDouble;		// 双面焊接
	int			dStartWrapType;	// 起点包角类型
	int			dEndWrapType;	// 终点包角类型
};

struct E_WRAPANGLE_PARAM
{
	double dEndpointOffsetDis = 0.0;			// 端点向外偏移长度
	double dStartVerOffset = 0.0;				// 起始端垂直立板偏移长度
	double dStartBackVerOffset = 0.0;			// 起始端反面垂直立板偏移长度

	double dWarpSpeed = 0.0;					// 包角速度
	double dWarpLength = 0.0;					// 起始包角长度
	double dRotateSpeed = 0.0;					// 起始旋转速度
	double dRotateAngle = 0.0;					// 起始旋转角度
	double dRotateStpTime = 0.0;				// 起始旋转角度后停留时间
	double dInputSpeed = 0.0;					// 起始段进入速度
	double dInputRatoi = 0.0;					// 起始段进入比例
	double dInputMaxDis = 10.0;					// 起始段进入极限阈值
	double dBackRotateSpeed = 0.0;				// 起始反面旋转速度
	double dBackRotateAngle = 0.0;				// 起始反面旋转角度
	double dBackRotateStpTime = 0.0;			// 起始旋转角度后停留时间
	double dBackInputSpeed = 0.0;				// 起始段反面进入速度
	double dBackInputRatoi = 0.0;				// 起始段反面进入比例
	double dBackInputMaxDis = 10.0;				// 起始段反面进入极限阈值
	double dJumpSafeHight = 50.0;				// 跳枪安全高度
};

// 测量点属性
typedef enum E_ATTRIBUTE
{
	E_BELONG_START = 0,			// 起点测量点
	E_BELONG_MIDDLE = 1,			// 焊缝中间修正测量点
	E_BELONG_END = 2				// 终点测量点
}E_ATTRIBUTE;

// 焊缝端点类型
typedef enum E_ENDPOINT_TYPE
{
	E_INTERFERE_POINT = 0,			// 干涉端点
	E_FREE_POINT = 1				// 自由端点
}E_ENDPOINT_TYPE;

// 示教结果
typedef struct T_TEACH_RESULT
{
	double	dExAxlePos;					// 采图时外部轴位置
	T_ROBOT_COORS tRobotCoors;			// 采图时机器人直角坐标
	T_ANGLE_PULSE tRobotPulse;			// 采图时机器人关节坐标
	CvPoint tKeyPtn2D;					// 交点二维点
	std::vector<CvPoint> vtLeftPtns2D;		// 左线二维点集合
	std::vector<CvPoint> vtRightPtns2D;		// 右二维点集合
	XI_POINT tKeyPtn3D;					// 交点三维坐标
	std::vector<XI_POINT> vtLeftPtns3D;		// 左线三维坐标集合
	std::vector<XI_POINT> vtRightPtns3D;		// 右线三维坐标集合
	XI_POINT tEndPtn3D;					// 行车梁扫描端点坐标
}T_TEACH_RESULT;

typedef enum E_WELD_SEAM_TYPE
{
	E_FLAT_SEAM = 0,			// 普通平焊
	E_STAND_SEAM = 1,			// 普通立焊
	E_PLAT_MULTIPLE = 2,			// 平焊多层多道
	E_STAND_MULTIPLE = 3,			// 立焊多层多道
	E_PLAT_GROOVE = 4,			// 平焊坡口
	E_STAND_GROOVE = 5				// 立焊坡口
}E_WELD_SEAM_TYPE;

// 示教数据
typedef struct T_TEACH_DATA
{
	int nWeldNo;						// 测量点所属焊缝编号
	int nMeasurePtnNo;					// 测量点编号(同一个测量点可用于计算多个焊缝端点)
	int nMeasureType;					// 测量类型： 先测后焊示教 测量修正 搜索端点 测棱投影
	E_WELD_SEAM_TYPE eWeldSeamType;		// 测量点所属焊缝的焊缝类型: 平焊 立焊 ……
	E_ATTRIBUTE eAttribute;				// 测量点属性：用于测量起点 终点 或焊缝中间修正
	E_ENDPOINT_TYPE eEndpointType;		// 测量点所属端点的端点类型: 自由端点 干涉端点
	T_ROBOT_COORS tMeasureCoordGunTool;	// 测量位置直角坐标 焊枪工具
	T_ROBOT_COORS tMeasureCoordCamTool;	// 测量位置直角坐标 相机工具
}T_TEACH_DATA;

//坡口点特征
typedef struct
{
	double dirX;
	double dirY;
	double dirZ;
	double vecAngleX;
	double vecAngleY;
}DirGroove;

typedef struct
{
	XI_POINT tPoint;
	T_XIGEO_LINE_DIR tPlateVerDir;
}T_XIGEO_PLANE_PARA;

typedef struct
{
	double dCenterX;
	double dCenterZ;
	double dRadius;
	double dThickness;
}T_XIGEO_PARA_PIPE;

typedef struct
{
	T_POINT_3D tUpStartPoint;
	T_POINT_3D tUpEndPoint;
	T_POINT_3D tDownStartPoint;
	T_POINT_3D tDownEndPoint;
}T_XIGEO_WELDLINE;

struct T_ROBOT_COORS_IN_WORLD
{
	double dX;
	double dY;
	double dZ;
	double dRX;
	double dRY;
	double dRZ;

	T_ROBOT_COORS_IN_WORLD()
	{
		memset(this, 0, sizeof(T_ROBOT_COORS_IN_WORLD));
	}
};

typedef struct T_KINEMATICS
{
	double dH1;
	double dA1;
	double dA2;
	double dA3;
	double dD3;
	double dD4;
	double dD5;

	T_KINEMATICS()
	{
		memset(this, 0, sizeof(T_KINEMATICS));
	}
}T_KINEMATICS;

struct T_AXISUNIT
{
	double dSPulse;
	double dLPulse;
	double dUPulse;
	double dRPulse;
	double dBPulse;
	double dTPulse;

	T_AXISUNIT()
	{
		memset(this, 0, sizeof(T_AXISUNIT));
	}
};

struct T_AXISLIMITANGLE
{
	double dMaxPosSAngle;
	double dMaxNegSAngle;

	double dMaxPosLAngle;
	double dMaxNegLAngle;

	double dMaxPosUAngle;
	double dMaxNegUAngle;

	double dMaxPosRAngle;
	double dMaxNegRAngle;

	double dMaxPosBAngle;
	double dMaxNegBAngle;

	double dMaxPosTAngle;
	double dMaxNegTAngle;

	T_AXISLIMITANGLE()
	{
		memset(this, 0, sizeof(T_AXISLIMITANGLE));
	}
};

struct T_ROBOT_BASE_PULSE
{
	long lBXPulse;
	long lBYPulse;
	long lBZPulse;

	T_ROBOT_BASE_PULSE()
	{
		memset(this, 0, sizeof(T_ROBOT_BASE_PULSE));
	}
};

typedef struct  
{
	XI_POINT tWeldGunPos;
	XI_POINT tWeldLinePos;
}T_ABS_POS_IN_BASE;

typedef struct  
{
	XI_POINT tWeldGunPos;
	XI_POINT tWeldLinePos;
}T_ABS_POS_IN_WORLD;

typedef struct
{
	double dDx;
	double dDy;
	double dDz;
	double dCenterDisUToBase;
}T_HAND_EYE_PARAM;

typedef struct
{
	int nWidth;
	int nHeight;
	double dPixelX;
	double dPixelY;
	double dFocal;
	double dBaseLineLength;
	double dCtanBaseLineAngle;
}T_CAMREA_INNER_PARAM;


//多线程搜索模式 2023.2.26 添加定长搜索
typedef enum E_SCANMODE {
	E_SCANMODE_2DPOINT = 0x00,		//搜端点模式
	E_SCANMODE_KINEMATIC = 0x01,	//纯运动模式
	E_SCANMODE_FIXEDSCAN = 0x02		//定长搜索
}E_SCANMODE;

typedef struct
{
	double dMatrix[4][4] = {0};
	double dRotateDegree;
	double dDirX;
	double dDirY;
	double dDirZ;
	double dX;
	double dY;
	double dZ;
}T_CAMERA_TRANSFORM_PARAM;

typedef struct
{
	T_CAMREA_INNER_PARAM tCameraInnerPara;
	T_ROBOT_COORS tCameraReadyRobotCoors;
	T_ANGLE_PULSE tCameraReadyRobotPulse;
	T_ANGLE_THETA tCameraReadyRobotTheta;
	T_ROBOT_COORS tGunMoveToCameraCenterRobotCoors;
	E_MANIPULATOR_TYPE eManipulatorType = E_ROBOT_GP225;
	int nCameraXAxisInRobotCoordinate;
	int nCameraYAxisInRobotCoordinate;
	int nCameraZAxisInRobotCoordinate;
	int nExternalAxisType = 0;
}T_HAND_EYE_CALI_PARAM;

typedef struct
{
	double dMatrix[4][4];
}T_TransMatrix;

typedef struct
{
	double dMatrix[3][3];
}T_TransMatrix_Three;

typedef struct  
{
	T_ROBOT_COORS tTableCenter;
	int nCalTableCenterRobotNo;
	int nCalRobotExternalType;
	double dTableRotateAngleInWorkRobot;
	int nWorkRobotNo;
	int nWorkRobotExternalType;
}T_TABLE_PARA;

typedef struct  
{
	T_ROBOT_COORS tLeftUpTablePoint;
	T_ROBOT_COORS tLeftDownTablePoint;
	T_ROBOT_COORS tRightUpTablePoint;
	T_ROBOT_COORS tRightDownTablePoint;
}T_TABLE_CORNER;

typedef struct  
{
	T_ROBOT_COORS tFirstPointInFirstRobot;
	T_ROBOT_COORS tSecondPointInFirstRobot;
	T_ROBOT_COORS tThirdPointInFirstRobot;
	int nFirstRobotExternalType;
	int nFirstRobotNo;
	T_ROBOT_COORS tFirstPointInSecondRobot;
	T_ROBOT_COORS tSecondPointInSecondRobot;
	T_ROBOT_COORS tThirdPointInSecondRobot;
	int nSecondRobotExternalYype;
	int nSecondRobotNo;
}T_TWO_ROBOT_CALI_PARA;

typedef struct
{
	T_ROBOT_COORS tFirstPointInObverseSide;
	T_ROBOT_COORS tSecondPointInObverseSide;
	T_ROBOT_COORS tThirdPointInObverseSide;
	T_ROBOT_COORS tFirstPointInReverseSide;
	T_ROBOT_COORS tSecondPointInReverseSide;
	T_ROBOT_COORS tThirdPointInReverseSide;
}T_TURNOVERDEVICE_CALI_PARA;


typedef enum
{
	E_LEFT_ROBOT_FORWARD_CAM = 0,
	E_LEFT_ROBOT_BACKWARD_CAM,
	E_LEFT_ROBOT_LEFT_CAM,
	E_LEFT_ROBOT_RIGHT_CAM,
	E_RIGHT_ROBOT_FORWARD_CAM,
	E_RIGHT_ROBOT_BACKWARD_CAM,
	E_RIGHT_ROBOT_LEFT_CAM,
	E_RIGHT_ROBOT_RIGHT_CAM,
}E_CAM_ID;

typedef struct
{
	CString strModelName;						//相机型号名
	CString strID;								//相机ID
	CString strHardwareVersion;					//硬件版本
	CString strFirmwareVersion;					//固件版本
	CString strDeviceAddress;					//IP地址
	int	nPort;									//端口号
}T_MECHEYE_CAMREA_DRIVER_PARAM;//梅卡曼德相机参数

typedef struct
{
	int nCameraType;							//相机类型
	int nColorWidth;							//彩图宽
	int nColorHeight;							//彩图高
	int nInfraredWidth;							//红外图宽
	int nInfraredHeight;						//红外图高
	int nDepthWidth;							//深度宽
	int nDepthHeight;							//深度高
	bool bEnableUndistort;
	cv::Mat cvCameraMatrix;
	cv::Mat cvDistCoeffs;
}T_DEPTH_CAMREA_DRIVER_PARAM;//全景相机型号参数

typedef struct
{
	CString  strDeviceAddress;					//IP地址
	int nCameraAcquisitionMode;					//触发模式
	int nCallBackMode;							//回调模式
	double dExposureTime;						//曝光时间
	double dGainLevel;							//增益
	int nRoiWidth;								//宽
	int nRoiHeight;								//高
	int nMaxWidth;								//宽
	int nMaxHeight;								//高
	int nRoiOffsetX;							//ROI，X方向偏移
	int nRoiOffsetY;							//ROI，Y方向偏移
	int nStrobeLine1;							//闪光灯输出引脚1
	int nStrobeLine2;							//闪光灯输出引脚2

}T_DH_CAMREA_DRIVER_PARAM;//大恒相机参数

typedef struct
{
	bool bEnableUndistort;
	cv::Mat cvCameraMatrix;
	cv::Mat cvDistCoeffs;
}T_CAMERA_DISTORTION;

typedef struct
{
	std::vector<double> vdLineLaserCalibrationResult;
}T_LASER_PLAN_EQUATION;

typedef enum
{
	E_DH_CAMERA = 0,
	E_KINECT_CAMERA,
	E_MECHEYE_CAMERA,
	E_VZENSE_CAMERA,
	E_CAMERA_TYPE_MAX_NUM,
}E_CAMERA_TYPE;

typedef struct
{
	//防飞溅挡板
	int nAntisplashOpenedIO;								//防飞溅挡板打开状态IO
	int nAntisplashClosedIO;								//防飞溅挡板关闭状态IO
}T_CAMERA_IO_INPUT;
typedef struct
{
	//激光线
	//输出信号
	std::vector<int> vnLeaseLineIO;							//同一相机可能有多条激光线

	//补光灯
	int nSupLEDIO;											//补光灯总控IO
	int nSupLED1IO;											//补光灯1IO
	int nSupLED2IO;											//补光灯2IO

	//防飞溅挡板
	int nAirSolenoidValueIO;								//气路电磁阀IO
	int nAntisplashIO;										//防飞溅挡板IO

	//输出信号
	int m_nCameraPowerIO;									//电源开关
}T_CAMERA_IO_OUTPUT;
typedef struct
{
	std::vector<CString> vsLaserIOName;
	T_CAMERA_IO_INPUT tInputIO;
	T_CAMERA_IO_OUTPUT tOutputIO;
}T_CAMERA_IO;

//2023/12/21 焊接加
typedef struct
{
	double drYMaxCar;				//Y方向大车最大位置
	double drYMinCar;				//Y方向大车
	double drYMaxRobot;			//
	double drYMinRobot;			//
	double drXMax;					//
	double drXMin;					//
	double drZMax;					//
	double drZMin;					//
	double drdMinVel;				//
	double drdAcc;					//
	double drdDec;					//
	double drScanSpeed;			//
	double drRunSpeed;			//
	double drTableY;
	double drTableZ;
}T_ROBOT_LIMITATION;



struct T_WAVE_PARA
{
	bool bStandWeld = false;
	double dStartWave; //起点摆弧幅度
	double dEndWave;//终点摆弧幅度
	double dWaveDistance;//摆动间隔
	double dEndSpeedRate;//末端速率
	double dLeftWaitTime;//左侧等待时间	 // 用作起点5mm宽停留时间
	double dRightWaitTime;//右侧等待时间
	double dMidWaitTime;//中间等待时间   // 用作终点12mm宽停留时间
	double dVerticalAngle;//立焊的焊道方向
	int waveType;//立焊的摆动方式
	double dWaveHeight;//三角摆和梯形摆的高
	double dWaveUpBase;//梯形摆的上底
	double dNormalOffset;//法向偏移
	double dHorizontalOffset;//横向偏移
	T_WAVE_PARA()
	{
		memset(this, 0, sizeof(T_WAVE_PARA));
	}
};//多层多道渐变摆弧专用

struct T_GROOVE_INFOR
{
	std::vector<T_ROBOT_COORS> vtStartPoint;//坡口起点信息
	std::vector<T_ROBOT_COORS> vEndPoint;//坡口终点信息
	T_ROBOT_COORS tStartPoint;//坡口起点信息
	T_ROBOT_COORS tEndPoint;//坡口终点信息
	double dStartThickness; // 起点厚度
	double dEndThickness; // 终点厚度
	double dPlateThickness;//板厚度
	double dStartLowerFace;//起点底面宽度
	double dStartUpperFace; //起点上面宽度
	double dEndLowerFace;//终点底面宽度
	double dEndUpperFace;//终点上面宽度
	double weldAngle;//立焊坡口方向，平焊无效
	int nStartChangeType; // 起点处：0不变 1回缩 2伸出
	int nEndChangeType; // 终点处：0不变 1回缩 2伸出
	T_GROOVE_INFOR()
	{
		memset(this, 0, sizeof(T_GROOVE_INFOR));
	}
};

struct T_INFOR_WAVE_RAND
{
	int nLayerNo;
	std::vector<T_ROBOT_COORS> vtStartPoint;//焊接起点偏移信息
	std::vector<T_ROBOT_COORS> vtEndPoint;//焊接起点偏移信息
	T_ROBOT_COORS tStartPoint;//焊接起点偏移信息
	T_ROBOT_COORS tEndPoint;//焊接终点偏移信息
	double dAngleOffset;//焊接姿态跟标准姿态偏移
	double dStartWaveDis; // 起点摆动幅度 排道时计算 计算摆动轨迹时使用
	double dEndWaveDis; // 终点摆动幅度 排道时计算 计算摆动轨迹时使用
	double dStartSpeedRate; // 起点速度比率 排道时计算 计算摆动轨迹时使用
	double dEndSpeedRate; // 终点速度比率 排道时计算 计算摆动轨迹时使用
	double dStartWaveWaitTime; // 起点停留时间 排道时计算 计算摆动轨迹时使用
	double dEndWaveWaitTime; // 终点停留时间 排道时计算 计算摆动轨迹时使用
	T_INFOR_WAVE_RAND()
	{
		memset(this, 0, sizeof(T_INFOR_WAVE_RAND));
	}
};//多层多道焊接起终点信息

//2023/12/21 焊接加
typedef struct
{
	double dJointAngleTheta;
	double dTwistAngleAlpha;
	double dLinkLengthA;
	double dLinkOffsetD;
}T_LINK_PARAM;

typedef struct // 安川摆动参数中获取埃斯顿使用的摆动参数
{
	int nNo;
	int nType;
	double dFreq;
	double dAmpL;
	double dAmpR;
	double dTimeL;
	double dTimeC;
	double dTimeR;
	double dRotAngleX;
	double dRotAngleZ;
	int nDelateTypeL;
	int nDelateTypeC;
	int nDelateTypeR;
}T_WEAVE_PARA;

typedef struct
{
	CString strWorkPeace;
	CString strWeldType;
	int nWeldAngleSize; // 焊脚尺寸 仅支持整数尺寸
	int nLayerNo; // 多层多道层号
	double dStartArcCurrent;
	double dStartArcVoltage;
	double dStartWaitTime;
	double dTrackCurrent;
	double dTrackVoltage;
	double WeldVelocity;
	double dStopArcCurrent;
	double dStopArcVoltage;
	double dStopWaitTime;

	double dWrapCurrentt1;
	double dWrapVoltage1;
	double dWrapWaitTime1;
	double dWrapCurrentt2;
	double dWrapVoltage2;
	double dWrapWaitTime2;
	double dWrapCurrentt3;
	double dWrapVoltage3;
	double dWrapWaitTime3;

	double CrosswiseOffset;//横向补偿
	double verticalOffset;//竖直方向补偿
	int nWrapConditionNo; // 摆动条件号
	double dWeldAngle; // 焊接角度 焊丝和底板或立焊侧板夹角
	double dWeldDipAngle; // 焊接倾角 药芯焊丝(拉) 实芯焊丝(推) 倾角不同
	int nStandWeldDir; // 立焊焊接方向：0:立向下  1:立向上

	// CO2/混合气 实心/药芯 脉冲/直流 1.2/1.4 等不同组合
	int nWeldMethod; // 焊接方法 1脉冲分别/0直流分别 （0立焊 1平焊）

	T_WEAVE_PARA tWeaveParam; // 摆动参数(埃斯顿摆动需要动态写入使用)
	T_WAVE_PARA tGrooveWaveParam; // 国焊坡口摆动参数
}T_WELD_PARA;

typedef struct T_WELD_SEAM_TYPE_PARA
{
	T_WELD_PARA VerticalWeldPare;
	T_WELD_PARA SlopeFlatWeldPare;//斜坡平缝
	T_WELD_PARA LevelFlatWeldPare;//平对接
	T_WELD_PARA FlatSeamingWeldPare;//高度平缝
	T_WELD_PARA CornerLineWeldPare;
	T_WELD_PARA StrengthBoardWeldPare;
	T_WELD_PARA MarginStraightSlitWeldPare;
	T_WELD_PARA FrontPlateUnderFrameWeldPare;
}T_WELD_SEAM_TYPE_PARA;

//适用于全流程函数	2023/12/21 焊接加
typedef enum
{
	//通用类型
	RUNNING_STATUS_SUCCESS = 1,		//函数调用成功
	RUNNING_STATUS_EMG = -1,		//设备急停
	RUNNING_STATUS_STOP = -2,		//系统暂停
	RUNNING_STATUS_INVALID_COORDINATES = -3,		//无效的坐标位置
	RUNNING_STATUS_OUT_OF_RANGE_COORDINATES = -4,		//超限的坐标位置
	RUNNING_STATUS_TIME_OUT = -5,		//超时
	RUNNING_STATUS_INVALID_DATA = -6,		//读文件读到无效数据
	RUNNING_STATUS_TOO_MANY_SEGMENTS = -7,		//多次段过多警告
	RUNNING_STATUS_ABNORMAL_COORDINATES = -8,		//超出程序预测的坐标位置（异常坐标）
	RUNNING_STATUS_TEST = -9,		//测试用
	RUNNING_STATUS_EXTERNAL_AXLE_OUT_OF_RANGE = -10,		//外部轴超限
	RUNNING_STATUS_REPEAT_CALL = -11,		//重复调用函数
	RUNNING_STATUS_INVALID_ROBOT_NUM = -12,		//无效的机械臂数量
	RUNNING_STATUS_LOCATION_IS_NOT_REACHED = -13,		//未到达指定坐标位置
	RUNNING_STATUS_UNKNOWN_ERROR = -99,		//未知错误

	//CheckDone
	RUNNING_STATUS_PROXIMITY = -100,		//CheckDone函数，接近传感器反馈

	RUNNING_STATUS_CHECK_COORS_ERROR = 150,
	RUNNING_STATUS_CHECK_PULSE_ERROR = 151,

	//导入图纸
	RUNNING_STATUS_EMPTY_PATH = -200,		//空路径
	RUNNING_STATUS_WRONG_PATH = -201,		//错误路径

	//全景/灰度识别
	RUNNING_STATUS_SAVE_DEPTH_IMG_FAILED = -300,		//全景采图失败
	RUNNING_STATUS_NO_WORKPIECE = -301,		//上料台无可用工件
	RUNNING_STATUS_WRONG_TEMPLATE_NO = -302,		//错误的模板编号
	RUNNING_STATUS_RECOGNIZE_FAILED = -303,		//全景识别失败
	RUNNING_STATUS_SAVE_GRAY_IMG_FAILED = -350,		//灰度采图失败

	//上下料
	RUNNING_STATUS_MISSING_WORKPIECE = -400,		//工件遗失	
	RUNNING_STATUS_FIND_NO_WORKPIECE = -401,		//下降区域内未发现工件
	RUNNING_STATUS_DEMAGNETIZING_FAILED = -402,		//退磁失败	
	RUNNING_STATUS_MAGNETIZING_FAILED = -403,		//充磁失败

	//示教
	RUNNING_STATUS_ANTISPLASH_CLOSED = -500,		//防飞溅挡板关闭，无法打开
	RUNNING_STATUS_TEACH_ERROR_THRESHOLD = -501,		//示教误差超出阈值
	RUNNING_STATUS_CALC_TEACH_TRACK_FAILED = -502,		//计算示教轨迹错误
	RUNNING_STATUS_TEACH_GET_IMAGE_FAILED = -503,		//示教采图失败
	RUNNING_STATUS_FIND_CORNNER_FAILED = -504,		//FindCornner失败

	//匹配工艺
	RUNNING_STATUS_WRITE_PROCESS_ID_FAILED = -601,		//写入工艺ID失败,海宝专用
	RUNNING_STATUS_LOAD_PROCESS_PARA_FAILED = -602,		//匹配工艺失败

	//下枪
	RUNNING_STATUS_INVALID_TRACK = -700,		//无效的轨迹
	RUNNING_STATUS_INVALID_IGNITION_HEIGHT = -701,		//无效的引弧高度
	RUNNING_STATUS_INVALID_ANGLE_COMPEN = -702,		//无效的角度补偿

	//切割
	RUNNING_STATUS_POINTS_OVERFLOW = -800,		//轨迹点数超限
	RUNNING_STATUS_INVALID_CUTTING_HEIGHT = -801,		//无效的切割高度
	RUNNING_STATUS_POS_VAR_OVERFLOW = -802,		//P变量超限
	RUNNING_STATUS_CHECK_POS_VAR_FAILED = -803,		//P变量校验失败
	RUNNING_STATUS_CHECK_INT_VAR_FAILED = -804,		//I变量校验失败
	RUNNING_STATUS_ARCING_FAILED = -805,		//起弧失败
	RUNNING_STATUS_SMALL_ARCING_FAILED = -805,		//起小弧失败
	RUNNING_STATUS_ARC_BREAK = -807,		//断弧	
	RUNNING_STATUS_INVALID_INT_VAR = -808,		//无效的I变量
	RUNNING_STATUS_I_VAR_CHANGED_OVERTIME = -809,		//I变量变更超时
	RUNNING_STATUS_UNKNOW_INCISE_WAYS = -810,		//未知的切割方式

	//打磨
	RUNNING_STATUS_SET_POLISHER_ANGLE = -900,		//调整打磨头角度失败
	RUNNING_STATUS_WRONG_POLISHER_STATUS = -901,		//错误的打磨电机状态

	//其它
	RUNNING_STATUS_ARCING_FAILED_REMOVE_POINTS_OVERFLOW = -1900,	//自动续弧达到上限
}E_ERROR_STATE;

//2023/12/21 焊接加
typedef struct
{
	double m_dWrapdParallel;
	double m_dWrapdVertical;
	double m_dWrapdVelocity;
	double m_dWrapdRX;
	double m_dWrapdRY;
	double m_dWrapdRZ;
	double m_dWrapdWaitTime;
	double m_dWrapdParallel2;
	double m_dWrapdVertical2;
	double m_dWrapdVelocity2;
	double m_dWrapdRX2;
	double m_dWrapdRY2;
	double m_dWrapdRZ2;
	double m_dWrapdWaitTime2;
	double m_dWrapdParallel3;
	double m_dWrapdVertical3;
	double m_dWrapdVelocity3;
	double m_dWrapdRX3;
	double m_dWrapdRY3;
	double m_dWrapdRZ3;
	double m_dWrapdWaitTime3;
	double m_dWrapdParallel4;
	double m_dWrapdVertical4;
	double m_dWrapdVelocity4;
	double m_dWrapdRX4;
	double m_dWrapdRY4;
	double m_dWrapdRZ4;
	double m_dWrapdWaitTime4;
	double m_dWrapdParallel5;
	double m_dWrapdVertical5;
	double m_dWrapdVelocity5;
	double m_dWrapdRX5;
	double m_dWrapdRY5;
	double m_dWrapdRZ5;
	double m_dWrapdWaitTime5;
	double m_dWrapdParallel6;
	double m_dWrapdVertical6;
	double m_dWrapdVelocity6;
	double m_dWrapdRX6;
	double m_dWrapdRY6;
	double m_dWrapdRZ6;
	double m_dWrapdWaitTime6;
	double m_dWrapdParallel7;
	double m_dWrapdVertical7;
	double m_dWrapdVelocity7;
	double m_dWrapdRX7;
	double m_dWrapdRY7;
	double m_dWrapdRZ7;
	double m_dWrapdWaitTime7;
}WELD_WRAP_ANGLE;

typedef struct
{
	bool bWrapType;
	CString strWrapName;
	WELD_WRAP_ANGLE tWeldWrapAngle;
}T_WELD_WRAP_ANGLE;

//2023/12/21 焊接加
typedef struct
{
	IplImage* cvLineScanImg;
	int nImgNo;
	T_ROBOT_COORS tRobotCoord;
	T_ANGLE_PULSE tRobotPulse;
	double carlienve;
}T_LINESCAN_IMGTOCAR;

typedef struct
{
	bool bAutoOpen = true;						//打开程序是否自动打开(仅大恒相机有效)
	int nInstallPos = 5;						//相机安装位置
	E_FLIP_MODE eFlipMode = E_FLIP_NONE;		//图像翻转模式
	int nSameTypeNo = 0;						//同类型相机编号
	CString strCameraName;						//相机名称
	CString strCameraNameEN;					//相机英文名称
	E_CAMERA_TYPE eCameraType;					//相机类型
	T_ROBOT_COORS tCameraTool;					//相机工具
	T_DH_CAMREA_DRIVER_PARAM tDHCameraDriverPara;	//大恒相机参数
	T_DEPTH_CAMREA_DRIVER_PARAM tPANOCameraDriverPara;	//全景相机参数
	T_MECHEYE_CAMREA_DRIVER_PARAM tMecheyeCameraDriverPara;	//梅卡曼德相机参数
	T_HAND_EYE_CALI_PARAM tHandEyeCaliPara;		//手眼关系
	T_CAMERA_TRANSFORM_PARAM tCameraTransfomPara;	//相机转换关系
	T_CAMERA_DISTORTION tCameraDistortion;		//畸变参数
	T_LASER_PLAN_EQUATION tLaserPlanEquation;	//激光线平面方程系数
	T_CAMERA_IO tCameraIO;						//相机IO控制
	XI_POINT  tPointCloudCompen;				//点云补偿
	double ppdHandEyeCompen[4][4];				//手眼补偿矩阵（取代原来的点云补偿）
}T_CAMREA_PARAM;

typedef struct
{
	int nSoftAxisNo;
	int nToRobotDir;
}T_MOTOR_PARAM;


typedef struct
{
	CString strShowName;					//工作台名称
	T_TABLE_PARA tTablePara;				//算法用工作台参数
	std::vector<double> vdRotateAngle;		//相对各机械臂的角度
	T_ANGLE_PULSE tPlaceCompartPulse;		//上下料比对坐标
	T_ANGLE_PULSE tPanoRecogPulse;			//全景识别位置
	T_ANGLE_PULSE tLineScanRecogPulse;		//线扫识别初始位置
	T_ANGLE_PULSE tGrayCorrectionPulse;		//灰度修正位置
	T_ROBOT_COORS tPolishPointInPolishRobot;//打磨
	double dUserBaseRotationAngle;			//灰度角度
	double dTableBaseDepth;					//相机识别深度
	double dHeightCompen;					//上下料高度补偿
	double dSlagHeightCompen;				//同摞单件下料高度额外补偿
	double dRecoHeight;						//真实识别高度
	double dSafeHeight;						//搬运机械臂移动过去的安全高度
	double dTrayHeight;						//托盘高度
}T_ALL_TABLE_PARA;

typedef struct
{
	//机械臂输出信号
	int nRobotRunningIO;									//机械臂运行中IO
	int nRobotServoOnIO;									//机械臂伺服接通IO
	int nErrorsIO;											//机械臂错误/警告IO
	int nEmgStopIO;											//机械急停IO
	int nBatteryWarningIO;									//电池报警IO
	int nOperationalOriginIO;								//作业原点IO
	int nMidwayStartIO;										//中途启动IO
	int nChoiceLongRangeModeIO;								//选择远程模式IO
	int nChoiceManualModeIO;								//选择手动模式IO
	int nChoiceTeachModeIO;									//选择示教模式IO
}T_YASKAWA_INPUT_IO;

typedef struct
{
	//机械臂输入信号
	int nRobotInitIO;										//机械臂外部启动IO
	int nRobotPauseIO;										//机械臂外暂停IO
	int nCallJobIO;											//机械臂程序调出
	int nServoOnIO;											//机械臂外部伺服接通IO	
	int nResetErrorsIO;										//机械臂错误重置IO		
	int nRobotExEmgIO;										//机械臂外部急停IO	
	int nProhibitOperationIO;								//机械臂禁止作业IO
}T_YASKAWA_OUTPUT_IO;

typedef struct
{
	T_YASKAWA_INPUT_IO tRobotInputIO;
	T_YASKAWA_OUTPUT_IO tRobotOutputIO;
}T_YASKAWA_ROBOT_IO;
typedef struct
{
	//输出信号
	int nAirSolenoidIO;					//气路电磁阀开关

	//输入信号
	int nAirPressureSignalIO;			//气体压强信号
}T_GAS_IO;
typedef struct
{
	int nCautionLightGreenIO_OUT;//警示灯绿
	int nCautionLightYellowIO_OUT;//警示灯黄
	int nCautionLightRedIO_OUT;//警示灯红
}T_WARNING_LIGHT_IO;

typedef struct
{
	//输出信号
	int nAirSolenoidIO;					//气路电磁阀开关

	//输入信号
	int nAirPressureSignalIO;			//气体压强信号
}T_MAGNET_IO;

typedef struct
{
	int nAllEmgStopIO;//主急停
}T_EMG_STOP_IO;

typedef struct
{
	int nRobotExternalPowerControl;//机器人控制柜电源
}T_ROBOT_POWER_IO;

typedef struct
{
	T_YASKAWA_ROBOT_IO tYaskawaRobotIO;
	T_GAS_IO tGasIO;
	T_WARNING_LIGHT_IO tWarningLightIO;
	T_EMG_STOP_IO tEmgStopIO;
	T_ROBOT_POWER_IO tRobtPowerIO;
}T_COMMON_IO;

typedef enum
{
	E_LS_IO = 0,	// 雷赛IO
	E_HC_IO = 1,	// 汇川IO
	E_YASKAWA_IO = 2,	// 安川机器人IO
	E_ESTUN_IO = 3,	// 埃斯顿机器人IO
	E_CROBOTP_IO = 4,	// 卡诺普机器人IO
}E_IO_TYPE;

typedef struct
{
	CString sNameCN;	// IO名称
	CString sNameEN;	// 英文名
	bool bReadOnly;		// true：只读  false：读写
	int nCardNo;		// 一个卡卡号为0  有多个卡依次递增
	int nCardNodeNo;	// 0:本地IO  1:模块0   2:模块1 …… (机器人IO此参数无效)
	int nType;			// 0:雷赛 汇川 安川机器人 埃斯顿机器人 卡诺普机器人
	int nDefaultVal;	// 正常状态下的默认值（如激光关闭 无急停 碰撞 等情况）
	int nBit;			// IO号
}T_IO_PARAM;

typedef enum
{
	MOVE_CONTROL_CARD_2410,
	MOVE_CONTROL_CARD_2610,
	MOVE_CONTROL_CARD_2C80,
	MOVE_CONTROL_CARD_4400,
	MOVE_CONTROL_CARD_5800,
}E_MOVE_CONTROL_CARD_TYPE;

struct T_ROBOT_CAL_PARA
{
	T_KINEMATICS tKinematics;
	T_AXISUNIT tAxisUnit;
	T_AXISLIMITANGLE tAxisLimitAngle;
};

const WORD MOVE_DIR_POSITIVE = 1;
const WORD MOVE_DIR_NEGATIVE = 0;

// 运动模式，相对绝对
const WORD MOVE_RELATIVE = 0;
const WORD MOVE_ABSOLUTE = 1;

const WORD HOME_MOVE_DIR_POSITIVE = 1;
const WORD HOME_MOVE_DIR_NEGfATIVE = 2;

const WORD CONTROL_CARD_NO = 0;
const double SECTION_S_TIME = 0.05;

const WORD D5800_MAX_AXIS_NO = 7;
const WORD D5800_LOGIC = 0;
const WORD D5800_CRD = 0;

const WORD D2C80_OUTBIT_SERVO_ENABLE = 1;

const double ACC_SRATIO = 0.4;

const int DMC5800_CONTIMOV_INSTRUCT_NUM = 5000;

typedef enum
{
	//*****************************切断分拣相关*****************************//
	RTN_STATUS_SUCCEED = 0x0000,		//成功
	RTN_STATUS_CAMPLATE = 0x0001,		//完成
	RTN_STATUS_VOID = 0x0002,		//无状态
	RTN_STATUS_MANUAL_CANCEL = 0x0003,		//手动取消
	RTN_STATUS_CANCEL = 0x0004,		//取消
	RTN_STATUS_MANUAL_CONTINUE = 0x0005,		//手动继续

	////////////数据/////////////
	RTN_STATUS_FILE_VOID_PATH = 0x1001,//空路径
	RTN_STATUS_FILE_WRONG_PATH = 0x1002,//错误路径
	RTN_STATUS_FILE_FAILD = 0x1003,//读取数据失败
	//读取图纸信息
	RTN_STATUS_FILE_GRAB_COOR_ERROR = 0x1101,//读取抓取坐标失败
	RTN_STATUS_FILE_WORKPIECE_INFO_ERROR = 0x1102,//读取工件信息失败
	RTN_STATUS_FILE_STEEL_PLATE_INFO_ERROR = 0x1103,//读取钢板信息失败
	RTN_STATUS_FILE_WORKPIECE_INFO_VOID = 0x1104,//工件信息缺失
	//托盘数据
	RTN_STATUS_DATA_NO_TRAY = 0x1201,//没有托盘
	RTN_STATUS_DATA_ERROR = 0x1201,//没有托盘
	RTN_ONE_TRAY_INDEX_ERROR = 0x1202,//没有一次托盘序号


	////////////读写配置文件数据/////////////
	RTN_STATUS_INI_NO_FILE = 0x2001,//文件不存在
	RTN_STATUS_INI_NO_DATA = 0x2002,//数据不存在

	////////////数据处理/////////////
	RTN_STATUS_PROCESS_UNKNOWN_ERROR = 0x3001,//未知问题
	RTN_STATUS_PROCESS_DEFECT = 0x3002,//缺失处理过程
	RTN_STATUS_PROCESS_DEFECT_MAT = 0x3003,//缺失位置矩阵
	RTN_STATUS_PROCESS_DATA_ERROR = 0x3004,//数据错误
	//切断相关
	RTN_STATUS_PROCESS_CUTTING_HEIGHT_ERROR = 0x3101,//高度偏差大，无法切割

	////////////坐标转换/////////////
	RTN_STATUS_PROCESS_TRANSFOR_ERROR = 0x4001,//坐标转换失败，数据错误
	RTN_STATUS_MKMD_TO_ROBOT_ERROR = 0x4002,//相机点云转机器人点云失败


	////////////运动/////////////
	RTN_STATUS_MOVE_RUNNING = 0x5001,//正在运行中
	RTN_STATUS_MOVE_EMG = 0x5002,//设备急停
	RTN_STATUS_MOVE_ROBOT_EMG = 0x5002,//机器人急停
	RTN_STATUS_MOVE_NO_RUNNING = 0x5003,//没有运动
	RTN_STATUS_MOVE_PLACE_ERROR = 0x5004,//位置错误
	RTN_STATUS_MOVE_ENCODER_DISCONNECT = 0x5005,//编码器连接失败
	RTN_STATUS_MOVE_ERROR_AXIS_NO = 0x5006,//错误的轴号
	RTN_STATUS_MOVE_PULSE_LIMIT = 0x5007,//脉冲超限
	RTN_STATUS_MOVE_PERFORATE_FAILD = 0x5008,//巡边失败
	RTN_STATUS_MOVE_AXIS0_FAILD = 0x5009,//外部轴0运动失败
	RTN_STATUS_MOVE_AXIS1_FAILD = 0x500A,//外部轴1运动失败
	RTN_STATUS_MOVE_AXIS2_FAILD = 0x500B,//外部轴2运动失败
	RTN_STATUS_MOVE_VAR_ERROR = 0x500C,//变量设置错误
	RTN_STATUS_MOVE_NO_ERROR = 0x500D,//编号错误
	RTN_STATUS_MOVE_NO_SAFE_POS = 0x500E,//没有在安全位置
	RTN_STATUS_MOVE_DEVICE_ERROR = 0x500F,//设备错误
	RTN_STATUS_MOVE_FAILD = 0x5010,//外部轴运动失败

	////////////视觉/////////////
	RTN_STATUS_IMAGE_CAMERA_ERROR = 0x6001,//采图失败
	RTN_STATUS_IMAGE_TYPE_ERROR = 0x6002,//图像类型错误
	RTN_STATUS_IMAGE_RESULT_UNRELIABLE = 0x6003,//结果不可靠
	RTN_STATUS_IMAGE_RESULT_ERROR = 0x6004,//识别结果错误
	RTN_STATUS_IMAGE_LAREGE_ERROR = 0x6005,//识别结果偏差大
	RTN_STATUS_IMAGE_NO_RESULT = 0x6006,//没有返回结果
	RTN_STATUS_IMAGE_PROCESS_FAIL = 0x6007,//识别失败

	////////////设备/////////////
	RTN_STATUS_DERVICE_NOT_READY = 0x6001,//设备未准备好
	RTN_STATUS_DERVICE_UNKNOWN_STATE = 0x6002,//设备状态未知
	RTN_STATUS_DERVICE_CUTTING_GAS_OPEN_FAIL = 0x6003,//切割气打开失败
	RTN_STATUS_DERVICE_CUTTING_GAS_CLOSE_FAIL = 0x6004,//切割气关闭失败
	RTN_STATUS_DERVICE_LASER_OPEN_FAIL = 0x6005,//激光打开失败
	RTN_STATUS_DERVICE_LASER_CLOSE_FAIL = 0x6006,//激光关闭失败
	RTN_STATUS_DERVICE_MAGNET_OPEN_FAIL = 0x6007,//电磁铁充磁失败
	RTN_STATUS_DERVICE_MAGNET_CLOSE_FAIL = 0x6008,//电磁铁退磁失败
	RTN_STATUS_DERVICE_FAULT = 0x6009,//无次设备
	RTN_STATUS_DERVICE_ERROR = 0x600A,//设备异常


	////////////分拣相关/////////////
	RTN_STATUS_SORTING_MAGNET_NO_SHRINK = 0x7001,//抖动失败
	RTN_STATUS_SORTING_SEND_10003_FALSE = 0x7002,//分拣询问中控放置位置发送10003返回False
	RTN_STATUS_SORTING_SEND_10005_FALSE = 0x7003,//分拣放置板链完成发送10005告知中控False
	RTN_STATUS_SORTING_WAIT_CENTENT_TIME_LONG = 0x7004,//分拣放置等待中控时间太长


	//////////个人二次配盘坐标///////////////////
	RTN_STATUS_SECONDARRANGE_POSITION_FALSE= 0x8001,//配盘的坐标错误
	RTN_STATUS_SECONDARRANGE_POSITION_UNREACHABLE = 0x8002,//配盘的坐标不可达
	RTN_STATUS_SECONDARRANGE_EXTERNALAXIS_UNREACHABLE = 0x8003//外部軸未運動到位

}E_RUNNING_RTN_STATE;

//2023/12/21 焊接加
// 测量点类型 (轨迹点类型)
typedef enum E_MEASURE_POINT_TYPE
{
	E_TRANSITION_POINT		= 1 << 0,		// 收下枪过渡点
	E_TEACH_POINT			= 1 << 1,		// 计算轨迹属性：先测后焊示教点
	E_ADJUST_POINT			= 1 << 2,		// 计算轨迹属性：长焊缝修正测量点
	E_SEARCH_POINT			= 1 << 3,		// 计算轨迹属性：搜索测量点 成对出现 第一个是起点 第二个是终点
	E_SCAN_POINT			= 1 << 4,		// 计算轨迹属性：扫描测量点 通过扫描焊缝点云提取焊缝
	E_ARRIS_POINT			= 1 << 5,		// 计算轨迹属性：自由端棱角测量点 至少连续两个 确定直线向下平面投影
	E_DOUBLE_LONG_LINE		= 1 << 6,		// 图像处理属性：双长线测量点
	E_LS_RL_FLIP			= 1 << 7,		// 图像处理属性：左侧短线右侧长线 且 需要上下反转图像 测量点
	E_LL_RS_FLIP			= 1 << 8,		// 图像处理属性：檩托板用立板边缘测量端点的短边图像
	E_WELD_ARC_ON			= 1 << 9,		// 焊接轨迹属性：起弧轨迹 64
	E_WELD_WRAP				= 1 << 10,		// 焊接轨迹属性：包角轨迹 128
	E_WELD_TRACK			= 1 << 11,		// 焊接轨迹属性：焊接轨迹 256
	E_WELD_ARC_OFF			= 1 << 12,		// 焊接轨迹属性：停弧轨迹 512（一般停弧不需要运动，没有运动轨迹）
	E_L_LINE_POINT			= 1 << 13,		// 图像处理属性: 角点和左测激光线上的点
	E_R_LINE_POINT			= 1 << 14,		// 图像处理属性: 角点和右侧激光线上的点
	E_WELD_CORNER_WRAP				= 1 << 15,					// 焊接轨迹属性：包角转角轨迹
	E_WELD_OPEN_TRACKING			= 1 << 16,					// 焊接轨迹属性：再次开启跟踪
	E_WELD_WRAP_CHANGE				= 1 << 17,					// 焊接轨迹属性：包角前变换一次电流电压
	E_TRANSITION_ARCOFF_FAST_POINT	= 1 << 18,					// 过渡轨迹属性：焊接过程停弧快速过渡
	E_WELD_TRACK_CHANGE_POSTURE		= 1 << 19,					// 焊接轨迹属性：变姿态焊接轨迹
	E_SEARCH_POINT_FIX				= 1 << 20 				    // 计算轨迹属性：定长搜索测量点
}E_MEASURE_POINT_TYPE;

typedef struct
{
	std::vector<E_RUNNING_RTN_STATE> veRtnState;
	std::vector<CString> vsRtnExplain;
	std::vector<CString> vsFuntionName;
	std::vector<int> vnRtnDataType;
	std::vector<void*> vpRtnData;
}T_RUNNING_RTN_RESULT;

BOOL CheckRtnResult(T_RUNNING_RTN_RESULT tRtnResult);

BOOL AddRtnData(T_RUNNING_RTN_RESULT &tRtnResult, E_RUNNING_RTN_STATE eRtnState, CString strFunction, int nLine, CString strRtnExplain = "", int nDataType = 0, void *vData = NULL);
BOOL AddRtnData(T_RUNNING_RTN_RESULT &tRtnResult, T_RUNNING_RTN_RESULT tRtnData);
E_RUNNING_RTN_STATE GetRtnState(T_RUNNING_RTN_RESULT tRtnResult);
//extern E_WELDED_THREAD_STATUS g_eThreadStatus;

BOOL LoadFileToVector(CString sFileName, std::vector <double> &vdData, int nDataNum = 0);
BOOL LoadFileToVector(CString sFileName, std::vector <long> &vlData, int nDataNum = 0);
double VelDisToVelPulse(WORD wAxisNum, WORD wPositionMode, long alPulse[], float fGunLength);

int GetFileLineNum( CString fileName );

//取代原来的XiMessageBoxGroup，受到界面上的单选框控制
XUI::MesBox& GetNaturalPop();

//int XiMessageBox(CLog *cLog, CString str);
//int XiMessageBox(CString str);
//int XiMessageBoxPopup(CString str, BOOL bPopup = TRUE);
//int XiMessageBoxPopup(CLog *cLog, CString str, BOOL bPopup = TRUE);
//int XiMessageBoxPopup(CLog *cLog, CString str, bool bPopup = true);
//int XiMessageBoxGroup(bool bIfGroup, char* format, ...);
//int XiMessageBoxGroup(BOOL bIfGroup, char* format, ...);
//int XiMessageBox(CLog *cLog, char *format, ...);
int XiMessageBox(const char* format); // 有确定、取消按钮
//int XiMessageBoxOk(CLog *cLog, char *format, ...); // 只有确定按钮
//int XiMessageBoxOk(CLog *cLog, CString str); // 只有确定按钮
int XiMessageBoxOk(const char* format); // 只有确定按钮

void SetDlgItemData(int nItem, double data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, int data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, long data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, CString data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, CDialog* pDlg, BOOL bIfWindowOn, char* format, ...);
void SetDlgItemData(int nItem, CDialog* pDlg, char* format, ...);
void GetButtonState(CButton *btn, int &state, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void GetDlgItemData(int nItem, double &data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void GetDlgItemData(int nItem, int &data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);

//防止界面假死无响应
void DoEvent();


CString GetModuleFilePath();

CString SaveFileDlg(CDialog* pFatherDlg, CString sFileType, CString sDefaultName, CString sDefaultPath, CString sTitle = "另存为");
CString OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, CString sTitle = "打开");

bool OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, std::vector< CString> &vstrFileList);
bool OpenFileDlg(CDialog* pFatherDlg, std::vector < CString> vsFileType, CString sDefaultPath, std::vector< CString>& vstrFileList);
//Utf8转GBK
CString Utf8ToGBK(LPCTSTR strUtf8);

//编辑框内容垂直居中，需要设置编辑框为多行
void VerticalCenter(CEdit &cEdit);

//检查文件是否存在
//bool CheckFileExists(std::string sFileName, bool bCreate = false);
bool CheckFileExists(CString sFileName, bool bCreate = false);

//检查文件夹是否存在，不存在则创建一个
long CheckFolder(CString strPath);//1:文件夹存在，0:文件夹不存在但已创建，其它(负数):报错

//在指定路径下寻找指定扩展名的全部文件
bool FindFile(std::vector<CString> &vstrFileList, CString strPath, CString strFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);

//复制整个文件夹到指定路径下
long CopyFolder(CString strSrcFolderName, CString strDstPath, bool bCopySubfolder = true);

//修改原文件夹的名字
BOOL ReNameFolder(LPCTSTR lpszFromPath, LPCTSTR lpszToPath);

//四舍五入，保留dDECimalDigits位小数
double DecimalRound(double dNum, int dDECimalDigits = 2);				

//清理文件夹中的文件
void DelFiles(CString directory_path);	
void DelFiles(CString directory_path, CString sFormat);
//清理文件夹内容
void   DeleteFolder(CString sPath);

//写JOB函数
void WriteJOB(int nLoopCount, int nStartPos, int nEndPos, E_ROBOT_CONTROL_CABINET_MODEL eCabinetModel = YRC1000, int nExternalAxle = 0);

//加载图片
IplImage *LoadImage(CString strImgName);

//保存图片
int SaveImage(IplImage *src, CString strImgName);
int SaveImage(IplImage *src, char *format, ...);
int SaveImage(IplImage **src, char *format, ...);
void XI_ReleaseImage(IplImage** img);

//按照区域和比例改变图片大小
CvRect ResizeImage(IplImage* img, IplImage* &dst, double dRatio, CvPoint &tCenter);

//根据控件显示区域大小和图片大小计算图片实际最大显示区域（会放大或缩小）
void CalImgRect(CRect &rectShowWholeImage, int nImageWidth, int nImageHeight, int nItem, CDialog *pDlg);

//在控件上显示图片
void DrawImage(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross = false);
void DrawImage_new(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross = false);

//自动使用CalImgRect函数，在控件上显示图片
void DrawImage(IplImage *pImage, CDialog *pDialog, int nStaticId, bool bDrawCross = false);

//辅助显示连续的图片,nStepNo或nPicNo超出限制将自动改为限制大小
void ShowStepPic(int &nStepNo, int &nPicNo, CString strPicPath, int nItem, CDialog *pDlg);

//计算lDate天后的日期
void CalDeadLine(SYSTEMTIME &tDeadline, SYSTEMTIME tTime, long lDate);		

//安全的文件读取,默认使用fopen_s，并有报错功能，返回0为正常，nShFlag设为_SH_DENYNO相当于fopen
errno_t XI_fopen_s(FILE **file, CString strFileName, _In_z_ const char * _Mode, int nShFlag = _SH_SECURE);

//传入16进制数字符串，返回unsigned long类型数
bool CStringToULongLong(unsigned long long &ullNum, CString strNum);

bool ULongLongToCString(CString &strNum, unsigned long long ullNum);

//
CString SystemTimeToCString(SYSTEMTIME tTime);
SYSTEMTIME CStringToSystemTime(CString sTime);
__int64 TimeDiff(SYSTEMTIME t1, SYSTEMTIME t2);

//异号判定
bool CheckContrarySign(double dNum1, double dNum2);				
bool CheckContrarySign(long lNum1, long lNum2);					

//关节臂参数读取
void LoadRobotPara(CString strRobotName, T_KINEMATICS &tKinematics, T_AXISUNIT &tAxisUnit, T_AXISLIMITANGLE &tAxisLimitAngle);

//用于深度图像数据读写
void WriteAdjustMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo, bool bLineScan);
bool LoadMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart, bool bLineScan);
void WriteAdjustMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo);
bool LoadMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart);

//已知三角形两边长和夹角角度，求对边长
double CalcTriangleSideLength(double dSide1Length, double dSide2Length, double dAngle);

//已知三角形三边长，求对角
double CalcTriangleDiagonal(double dSide1Length, double dSide2Length, double dOppositeSideLength);

//生成CRC
CString CalCRCData(unsigned long lData);

//从路径中获取文件名
bool GetFileNameFromPath(CString sFilePath, CString &sFileName, bool bSuffix = true);

//从路径中获取文件路径
bool GetFilePathFromPath(CString sPath, CString &sFilePath);

//计算最小采样间隔
int CalMinSampleGap(double dSpeed);

//替代clock
long long XI_clock();

void PicMean(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

void PicMeanRemove0(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

void PicMax(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

//消畸变
void Undistort(T_CAMERA_DISTORTION tCameraDistortion, IplImage *pImgGray, IplImage *pResultImgGray);

//微秒级延时
void DelayUs(double uDelay);
//毫秒级延时
void DelayMs(double uDelay);

double XI_ContourRound(double r);

//得到字符串
CString GetStr(char *format, ...);
CString GetStr(CString str);
CString GetStr(T_ROBOT_COORS tCoor, CString strToken = ",");
CString GetStr(T_ANGLE_PULSE tPulse, CString strToken = ",");
std::vector<CString> TokenizeCString(CString sSrc, CString sToken);
char* UnicodeToUtf8(char* str);

//格式转换
void TransforArray(T_ROBOT_COORS tCoor, double* adArray);
void TransforArray(T_ANGLE_PULSE tPulse, long* alArray);
void TransforArray(T_ANGLE_PULSE tPulse, double* adArray);
T_ANGLE_PULSE TransforPulse(long* alArray);
T_ANGLE_PULSE TransforPulse(double* adArray);
T_ROBOT_COORS TransforPos(double* adArray);


bool JudgeArea(double dAimPos, double dPos1, double dPos2);
void SaveErrorData(CString strName);
void SaveGrooveData(int nLayerNo, int nGroupNo, CString sRobotName, bool bSuccess); // 保存坡口测量数据
//获取程序内存
//double GetWorkingSetSize();

CString GetCurTime();

//void RunInteractiveWindow();
void RunInteractiveWindow(CString pointFileName, CString saveFileName, CString openFileName);
void RunInteractiveWindow(CString sUnitName);
bool AutomaticGrouping(CString sUnitName);

// 获取内存占用情况 并 打印日志
void GetMemoryInfo(CString sTipInfo);

XI_POINT P2P(CvPoint tPtnSrc);
XI_POINT P2P(T_ROBOT_COORS tPtnSrc);
XI_POINT CreateXI_POINT(double x, double y, double z);

//从字符串中提取第一个double数值，提取完之后，该段字符串会被删除
double GetDouble(char** str, int maxLen);

// 供算法使用
void AlgoAxisToRobotAxis(double& x, double& y, double& z);
void RobotAxisToAlgoAxis(double& x, double& y, double& z);
void AlgoAxisToRobotAxis(long& x, long& y, long& z);
//返回值:在同一坐标系下从原始位置到目标位置的坐标变换矩阵
//BasePoints:原始位置四个对应点在坐标系下的坐标
//TargetPoints:目标位置四个对应点在坐标系下的坐标
/*若四个点共面,则矩阵BaseCoordinateMatrix不满秩,其逆矩阵不存在,在计算BaseCoordinateMatrix_1时将得到错误的结果*/
cv::Mat CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints);
//返回值:原始位置点变换到目标位置后的结果点在同一坐标系下的坐标
//BasePoints:原始位置点坐标
//TransformationMatrix:坐标变换矩阵
std::vector<CvPoint3D64f> CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);
std::vector<CvPoint3D64f> CoordinateTransformateNormal(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);	//根据转换矩转换单位法向量xyz
long CopyFileFolder(CString strSrcFolderName, CString strDstPath);
 // 拟合直线
bool		 CalcLineParamRansac(T_LINE_PARA& tBestLineParam, std::vector<XI_POINT> vtPoint, double dSelectedRatio);
static bool	 	 dCmpIncFunc(double dFirstValue, double dSecondValue);
T_SPACE_LINE_DIR CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint);
double			 CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint);
// 计算投影点
double dot_product(XI_POINT V1, XI_POINT V2);
bool PointtoLineProjection(T_LINE_PARA tPointDir, T_ALGORITHM_POINT tpoint, T_ALGORITHM_POINT& tpointProjection);
bool PointtoLineProjection(T_LINE_PARA tPointDir, XI_POINT tpoint, XI_POINT& tpointProjection);
// 数据记录
void SaveRobotCoor(FILE* RecordTheoryData, T_ROBOT_COORS tCoor);
void SaveRobotCoor(FILE* RecordTheoryData, vector<T_ROBOT_COORS> vtCoors);
void SaveRobotCoor(FILE* RecordTheoryData, T_ROBOT_COORS tCoor, int nTrackType);
void SaveRobotCoor(FILE* RecordTheoryData, vector<T_ROBOT_COORS> vtCoors, vector<int> vtCoorType);
void SavePointsData(vector<T_ALGORITHM_POINT> PointsData, CString str);
void SavePointsData(vector<XI_POINT> PointsData, CString str);
bool LoadPointsData(vector<XI_POINT>& PointsData, CString str);
bool LoadPointsData(vector<T_ALGORITHM_POINT>& PointsData, CString str);

UINT ThreadSaveImage(void* pParam);

void FlipImage(IplImage* pImg, E_FLIP_MODE eFlipMode);
void ResumeImagePoint2D(int nImgWidth, int nImgHeight, CvPoint& tPtn2D, E_FLIP_MODE eFlipMode);

bool OpenProductLog( FILE** RecordTheoryData, CString sProductDataFileName);
bool CloseProductLog(FILE* RecordTheoryData);
bool SaveProductData(FILE* RecordTheoryData, double dWeldLength, int dWeldTime, int nWorkpieceNo);

template<typename T1, typename T2>
inline T2 Xi_P2P(T1 t)
{
	T2 t2;
	t2.x = t.x;
	t2.y = t.y;
	t2.z = t.z;
	return t2;
}

//摆动跟踪焊接参数
typedef struct SwingTrackPoint {
	T_ROBOT_COORS tLeftPoint;  //前点
	T_ROBOT_COORS tMiddlePoint; //中点
	T_ROBOT_COORS tRightPoint;	//后点
}SwingTrackPoint;
//摆动跟踪焊接参数
typedef struct SwingTrackWeldParam {
	//运动信息
	std::vector<T_ROBOT_COORS> vtWorldCoors; //未处理焊道
	std::vector<T_ROBOT_COORS> vtWeldRealCoors;  //机器人的真实焊道
	std::vector<double> vdAxisCoors; //外部轴真实焊道

	double dRobotWeldSpeed; //机器人速度
	double dAxisWeldSpeed;  //外部轴速度

							//需要参数
	T_ROBOT_COORS startPtn; //焊接起点
	T_ROBOT_COORS endPtn; //焊接终点

						  //焊接信息
	T_WELD_PARA vtWeldPara; //焊接参数

	int SwingNumber;                     //摆动条件号
	int nLayerNo;                        //层号
	double dSwingFrequency;              //摆动频率
	double dSwingLeftAmplitude;          //摆动左振幅
	double dSwingRightAmplitude;         //摆动右振幅
	double dSwingtime;                   //摆动停留时间

	double dSwingOffsetLayerNo;          //摆动各层偏移量
	bool bSwingMode;                     //摆动方式
	
	SwingTrackPoint swingTrackPoint;     //摆动轨迹

									     //计算使用
	cv::Vec3d norm;                      //焊道的向量
	size_t nFlat;                        //记录焊接结构体中的更新长度



	void InitSwingTrackWeldParam() {
		vtWorldCoors.clear();
		vtWeldRealCoors.clear();
		vdAxisCoors.clear();
		dRobotWeldSpeed = 0;
		dAxisWeldSpeed = 0;
		startPtn = T_ROBOT_COORS(0, 0, 0, 0, 0, 0, 0, 0, 0);
		endPtn = T_ROBOT_COORS(0, 0, 0, 0, 0, 0, 0, 0, 0);
		dSwingFrequency = 0;
		dSwingLeftAmplitude = 0;
		dSwingRightAmplitude = 0;
	}

}SwingTrackWeldParam;

/**************************** 线程相关操作 Start ****************************/
bool WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName); // 阻塞等待线程退出 线程函数返回0成功 其他失败 超时时间小于0无超时检测
bool WaitAndCheckAllThreadExit(std::vector<CWinThread*> &vpWinThread); // 等待所有线程结束 并 检测所有线程的运行结果
void ClearWinThread(std::vector<CWinThread*>& vpWinThread);
/**************************** 线程相关操作 End ****************************/

template<typename T1, typename T2>
inline void Trans3D(T1 tSrc, T2& tDst)
{
	tDst.x = tSrc.x;
	tDst.y = tSrc.y;
	tDst.z = tSrc.z;
}

template<typename T1, typename T2>
T1 P2P(T2 value)
{
	T1 temp;
	temp.x = value.x;
	temp.y = value.y;
	temp.z = value.z;
	return temp;
}

template<typename T1, typename T2>
void P2P(T1 src, T2& dst)
{
	dst.x = src.x;
	dst.y = src.y;
	dst.z = src.z;
}

template<typename T>
void deletePointer(T*& pointer)
{
	if (pointer)
	{
		delete pointer;
		pointer = nullptr;
	}
}

void setMainDlg(CDialog* pMainDlg);
CDialog* getMainDlg();

std::vector<CString> findSubfolder(CString sFolder);

// 模型匹配
bool Matching(E_WORKPIECE_TYPE eWorkPieceType);

bool ModelMatching(double dCorasrMatrix[4][4], const char* ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[4][4], bool if_Fine_Match);

bool RemoveStringInFile(CString sFileName, std::string sRemovedString);

#endif // AFX_CONST_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_
