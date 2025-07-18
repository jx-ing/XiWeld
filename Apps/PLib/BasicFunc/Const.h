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
//#define SINGLE_ROBOT  //������������ ���ⲿ������ע�͵�,ȷ���ⲿ�����������ƽ�С�����ú���Y��ƽ�У���֮��X��ƽ��
#endif

//// ��ɨ����
#define CAPTURE_IMAGE_FRAME_RATE		    100     	// ��ɨ��ͼ֡��
#define MAX_PROCESS_THREAD_NUM				4			// ��ɨͼ�����߳�����
#define	LINE_SCAN_BINARY_THRESHOLD			30			// ��ɨͼ��������������ֵ
#define LINE_SCAN_STEP_2D_POINT				1			// ��ɨͼ�����ά��������
#define LINE_SCAN_LASER_WIDTH				10			// ��ɨͼ�������߿��
#define LINESCAN_CAMERA_NUM					1			//��ɨ�������	
#define SAVE_IMAGE_THREAD_MUN				4			//��ͼ�߳�����
//#define LINE_SCAN_SAVE_IMAGE

// ������Ϣͳ��
//�жϰװ��ҹ��
enum ShiftType {
	DayShift = 0,
	NightShift = 1,
	InvalidShift = 2,
	SecDay = 3
};

extern int m_nWorkState; //�жϵ�ǰ�����ǰװ໹��ҹ��
extern CString m_sFirstOpenTime; // �״δ�ʱ�� ��/��/�� ʱ:��:��
extern CString m_sLastCloseTime; // ���ر�ʱ�� ��/��/�� ʱ:��:��
extern long m_lTotalOpenTime;	// �ܴ�ʱ�� ms
extern long m_lTotalColseTime;	// �ܶϿ�ʱ�� ms

extern long m_lWorkTime;			// ɨ�衢������ʾ�̡����� ���ܹ���ʱ��
extern long m_lStandbyTime;			// ����ʱ�� = ������ʱ�� - ����ʱ��
extern long m_lFlatWeldTime;		// ƽ��ʱ��
extern long m_lStandWeldTime;		// ����ʱ��
extern long m_lScanTime;			// ɨ��ʱ�� ��ɨʱ�� + ʾ��ʱ��
extern long m_lScanWeldTime;		// ɨ��ʱ�� + ƽ��ʱ�� + ����ʱ��

extern double m_dFlatWeldLen;	// ƽ������
extern double m_dStandWeldLen;	// ��������
extern double m_dTotalWeldLen;	// �����ܳ��� ƽ������ + ��������
extern double m_dTotalScanLen;	// ����ɨɨ�賤��

extern long lCurOpenTimeStamp; // ���ش�ʱ���
extern double m_dCurWeldLenBeforeCleanGun;

template<typename T>
inline auto square(T value)
{
	return value * value;
}

//������ͼ��תģʽ
typedef enum E_FLIP_MODE {
	E_FLIP_NONE = 0,				//����ת
	E_FLIP_VERTICAL,				//�ش�ֱ�ᷭת�����ҷ�ת��
	E_FLIP_HORIZEN,					//��ˮƽ�ᷭת�����·�ת��
	E_FLIP_BOTH						//ˮƽ+��ֱ ��ת
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
	E_DIAPHRAGM		= 0,	// ������
	E_LINELLAE		= 1,	// ������ ���ҹ���(С��������)
	SMALL_PIECE		= 2,	// ��Сɢ��(�Ȳ��)
	STIFFEN_PLATE	= 3,	// ���Ӿ���
	E_PURLIN_HANGER = 4,	// ������
	E_END_PLATE		= 5,	// ���˰�
	E_CORBEL		= 6,	// ��ţ��
	E_CLOSEDARC		= 7,	// ��һ�պϻ�Ȧ
	E_CAKE			= 8,    // ��ľ˹�糧
	E_BEVEL         = 9,    // �¿�
	E_SINGLE_BOARD = 10,	// ���嵥��
	E_SCAN_WELD_LINE = 11,	// ɨ�����
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

//2024/01/03 ���Ӽ�
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

//2023/12/21 ���Ӽ�

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
        str = __FILE__ + strTemp + ": ����ֵΪFALSE"; \
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
//         str = __FILE__ + strTemp + ": ����ֵΪFALSE"; \
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
        str = __FILE__ + strTemp + ": �ļ��򿪴���"; \
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

//�õ�Ӧ�ó�������·����
void GetModulePath(char *cPosfix);
void GetModulePath(const char *cPosfix);
extern char gcModulePath[500];
extern char gcModuleWholePath[500];

//ʵʱ��¼��ǰ��������������
extern char gcLogModulePath[500];
extern char gcLogModuleWholePath[500];
void GetLogModulePath(char *cPosfix);
void GetLogModulePath(const char *cPosfix);

////////////////////////////        ����      ////////////////////////////////////
const CString FILE_ERROR = _T("�ļ��쳣��");

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

const long    SCREENX        =   1280                               ;   // ��Ļ��ʾ�������ֵ����
const long    SCREENY        =   1024                               ;   // ��Ļ��ʾ�������ֵ����

//const double  PI = 3.1415926535897932384626433832795;

const int cnToolRotate = 90;//һȦ�Ƕȵȷַ���

////////////////////////////      �ļ�·��     ///////////////////////////////////

//�����ļ�
const CString CONTRAL_UNIT_INI					=	_T("ConfigFiles\\ContralUnitInfo.ini");
const CString DEBUG_INI							=	_T("ConfigFiles\\Debug.ini");
const CString DEPTH_CAMERA_PARA_INI				=	_T("ConfigFiles\\DepthCameraParam.ini");
const CString INTERFACE_CTRL_INI				=	_T("ConfigFiles\\InterfaceCtrlPara.ini");//��Ҫ����
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
const CString PROCESS_PARA_EXCEL				=	_T("ConfigFiles\\���ձ�.xlsx");
const CString MAT_GANTRY_ROBOT					=	_T("ConfigFiles\\MatGantryRobot.ini");

//����̨���
const CString TABLE_PATH						=	_T(".\\ConfigFiles\\Table\\");
const CString TABLE_GROUP_INI					=	_T("ConfigFiles\\Table\\TableGroup.ini");
const CString TABLE_INI							=	_T("ConfigFiles\\Table.ini");//���޸�

/*********************** ���ļ��ṹ��� Start ***********************/
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

/*********************** ���ļ��ṹ��� End ***********************/

//�Ի�е�ۻ��ֵ������ļ�
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

//���Բ����ڵ���Ҫ�Զ����ɵ�·��
const CString DXF_FILES							=	_T("DXF\\GraphData\\");
const CString GRAPH_DATA_FILES					=	_T("GraphData\\");
const CString MONITOR_PATH						=	_T("Monitor\\");
const CString SYSTEM_LOG_PATH					=	_T("Monitor\\SystemLog\\");
const CString OPC_LOG_PATH						=	_T("Monitor\\OPCLog\\");
const CString PRODUCTION_DATA					=	_T("��������\\");
const CString LASER_SCAN_CAMERA_PICTURES		=	_T("ProPics\\LaserScanPics\\");

//��Ҫ���ڵ�·��
const CString CAD_PROCESS_PARA_PATH				=	_T("ProcessParaTxt\\");
const CString ROBOT_INSTRUCTION					=	_T("˵��\\��е��\\");
const CString DH_CAMERA_INSTRUCTION				=	_T("˵��\\������Ҷ����\\");

typedef enum
{
	INCISEHEAD_THREAD_STATUS_START			= 0,					//����
	INCISEHEAD_THREAD_STATUS_READY			= 1,					//׼��
	INCISEHEAD_THREAD_STATUS_ENTER			= 2,					//��ǹ
	INCISEHEAD_THREAD_STATUS_TEACH			= 3,					//ʾ��
	INCISEHEAD_THREAD_STATUS_INCISE			= 4,					//�и�
	INCISEHEAD_THREAD_STATUS_LEAVE			= 5,					//��ǹ
	INCISEHEAD_THREAD_STATUS_STOPPED		= 6,					//��ͣ
	INCISEHEAD_THREAD_STATUS_CONTINUE		= 7,					//����
	INCISEHEAD_THREAD_STATUS_COMPLETE		= 8,					//���
	INCISEHEAD_THREAD_STATUS_FEEDING		= 9,					//����
	INCISEHEAD_THREAD_STATUS_BLANKING		= 10,					//����
	INCISEHEAD_THREAD_STATUS_ANTIFEEDING	= 11,					//������
	INCISEHEAD_THREAD_STATUS_DEPTH_RECOG	= 12,					//���ʶ��
	INCISEHEAD_THREAD_STATUS_GRAY_RECOG		= 13,					//�Ҷ�ʶ��
	INCISEHEAD_THREAD_STATUS_BACK_HOME		= 14,					//���ع̶�λ��
	INCISEHEAD_THREAD_STATUS_PICK_UP		= 15,					//ץȡ
	INCISEHEAD_THREAD_STATUS_TURN_OVER		= 16,					//����
	INCISEHEAD_THREAD_STATUS_POLISH			= 17					//��ĥ
}E_INCISEHEAD_THREAD_STATUS;

////2023/12/21 ���Ӽ�
extern E_INCISEHEAD_THREAD_STATUS g_eThreadStatus;

const int INCISEHEAD_THREAD_STATUS_NUM = 18;
const CString INCISEHEAD_THREAD_STATUS_NAME[INCISEHEAD_THREAD_STATUS_NUM] = 
				{ "����","׼��","��ǹ","ʾ��","�и�","��ǹ","��ͣ","����","���","����",
				"����","������","���ʶ��","�Ҷ�ʶ��","���ع̶�λ��","ץȡ","����","��ĥ" };

//ֻ����IO���ص����ͣ���ͣ�����ļ��Ķ�д���������е�۵��˶�
typedef enum
{
	CUTTING_ROBOT = 0x01,
	HANDLING_ROBOT = 0x02,
	POLISHING_ROBOT = 0x04,
}E_ROBOT_TYPE;

//��������RobotDriverAdaptor
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

//��������RobotDriverAdaptor
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

//2023/12/21 ���Ӽ�
typedef enum
{
	ROBOT_CAR_SUCCESS = 0,
	ROBOT_CAR_STOP = 1,		//����
	ROBOT_CAR_RUN = 2,		//������
	Y_COOR_OVERRUN = 3,		//Y�ᳬ��
	X_COOR_OVERRUN = 4,		//X�ᳬ��
	Z_COOR_OVERRUN = 5,		//Z�ᳬ��
	WELD_ROBOT_LEFT = 6,		//��������X�����
	WELD_ROBOT_RIGHT = 7,		//��������X���Ҳ�
	ROBOT_MOVE_FAILE = 8,     //�������˶�ʧ��
	CAR_MOVE_FAILE = 9         //���˶�ʧ��
}E_ROBOT_CAR_COORS_ERROR;

//2023/12/21 ���Ӽ�
typedef enum
{
	E_WRAPANGLE_EMPTY_SINGLE = 0,	// ���溸�Ӳ�����
	E_WRAPANGLE_EMPTY_DOUBLE,		// ˫�溸�Ӳ�����	
	E_WRAPANGLE_TWO_SINGLE,			// ���溸�ӷ����ΰ���
	E_WRAPANGLE_TWO_DOUBLE,			// ˫�溸�ӷ����ΰ���
	E_WRAPANGLE_ONCE,				// ˫�溸һ�ΰ���
	E_WRAPANGLE_JUMP				// ��ǹһ�ΰ���
}E_WRAPANGLE_TYPE;

//2023/12/21 ���Ӽ�
struct T_BOARD_SIZE
{
	int			BoardNum;		// ���
	XI_POINT	tStartPoint;	// ���
	XI_POINT	tEndPoint;		// �յ�
	int			dArc;			// �Ƿ�ΪԲ�� 
	double		dBoardlength;	// ��������
	int			dStartInterf;	// ����Ƿ����
	double		dStartNormal;	// ��㷨��
	int			dEndInterf;		// �յ��Ƿ����
	double		dEndNormal;		// �յ㷨��
	double		dBoardHeight;	// �����߶�Z
	double		dBoardThick;	// �������
	int			dOrDouble;		// ˫�溸��
	int			dStartWrapType;	// ����������
	int			dEndWrapType;	// �յ��������
};

struct E_WRAPANGLE_PARAM
{
	double dEndpointOffsetDis = 0.0;			// �˵�����ƫ�Ƴ���
	double dStartVerOffset = 0.0;				// ��ʼ�˴�ֱ����ƫ�Ƴ���
	double dStartBackVerOffset = 0.0;			// ��ʼ�˷��洹ֱ����ƫ�Ƴ���

	double dWarpSpeed = 0.0;					// �����ٶ�
	double dWarpLength = 0.0;					// ��ʼ���ǳ���
	double dRotateSpeed = 0.0;					// ��ʼ��ת�ٶ�
	double dRotateAngle = 0.0;					// ��ʼ��ת�Ƕ�
	double dRotateStpTime = 0.0;				// ��ʼ��ת�ǶȺ�ͣ��ʱ��
	double dInputSpeed = 0.0;					// ��ʼ�ν����ٶ�
	double dInputRatoi = 0.0;					// ��ʼ�ν������
	double dInputMaxDis = 10.0;					// ��ʼ�ν��뼫����ֵ
	double dBackRotateSpeed = 0.0;				// ��ʼ������ת�ٶ�
	double dBackRotateAngle = 0.0;				// ��ʼ������ת�Ƕ�
	double dBackRotateStpTime = 0.0;			// ��ʼ��ת�ǶȺ�ͣ��ʱ��
	double dBackInputSpeed = 0.0;				// ��ʼ�η�������ٶ�
	double dBackInputRatoi = 0.0;				// ��ʼ�η���������
	double dBackInputMaxDis = 10.0;				// ��ʼ�η�����뼫����ֵ
	double dJumpSafeHight = 50.0;				// ��ǹ��ȫ�߶�
};

// ����������
typedef enum E_ATTRIBUTE
{
	E_BELONG_START = 0,			// ��������
	E_BELONG_MIDDLE = 1,			// �����м�����������
	E_BELONG_END = 2				// �յ������
}E_ATTRIBUTE;

// ����˵�����
typedef enum E_ENDPOINT_TYPE
{
	E_INTERFERE_POINT = 0,			// ����˵�
	E_FREE_POINT = 1				// ���ɶ˵�
}E_ENDPOINT_TYPE;

// ʾ�̽��
typedef struct T_TEACH_RESULT
{
	double	dExAxlePos;					// ��ͼʱ�ⲿ��λ��
	T_ROBOT_COORS tRobotCoors;			// ��ͼʱ������ֱ������
	T_ANGLE_PULSE tRobotPulse;			// ��ͼʱ�����˹ؽ�����
	CvPoint tKeyPtn2D;					// �����ά��
	std::vector<CvPoint> vtLeftPtns2D;		// ���߶�ά�㼯��
	std::vector<CvPoint> vtRightPtns2D;		// �Ҷ�ά�㼯��
	XI_POINT tKeyPtn3D;					// ������ά����
	std::vector<XI_POINT> vtLeftPtns3D;		// ������ά���꼯��
	std::vector<XI_POINT> vtRightPtns3D;		// ������ά���꼯��
	XI_POINT tEndPtn3D;					// �г���ɨ��˵�����
}T_TEACH_RESULT;

typedef enum E_WELD_SEAM_TYPE
{
	E_FLAT_SEAM = 0,			// ��ͨƽ��
	E_STAND_SEAM = 1,			// ��ͨ����
	E_PLAT_MULTIPLE = 2,			// ƽ�������
	E_STAND_MULTIPLE = 3,			// ���������
	E_PLAT_GROOVE = 4,			// ƽ���¿�
	E_STAND_GROOVE = 5				// �����¿�
}E_WELD_SEAM_TYPE;

// ʾ������
typedef struct T_TEACH_DATA
{
	int nWeldNo;						// ����������������
	int nMeasurePtnNo;					// ��������(ͬһ������������ڼ���������˵�)
	int nMeasureType;					// �������ͣ� �Ȳ��ʾ�� �������� �����˵� ����ͶӰ
	E_WELD_SEAM_TYPE eWeldSeamType;		// ��������������ĺ�������: ƽ�� ���� ����
	E_ATTRIBUTE eAttribute;				// ���������ԣ����ڲ������ �յ� �򺸷��м�����
	E_ENDPOINT_TYPE eEndpointType;		// �����������˵�Ķ˵�����: ���ɶ˵� ����˵�
	T_ROBOT_COORS tMeasureCoordGunTool;	// ����λ��ֱ������ ��ǹ����
	T_ROBOT_COORS tMeasureCoordCamTool;	// ����λ��ֱ������ �������
}T_TEACH_DATA;

//�¿ڵ�����
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


//���߳�����ģʽ 2023.2.26 ��Ӷ�������
typedef enum E_SCANMODE {
	E_SCANMODE_2DPOINT = 0x00,		//�Ѷ˵�ģʽ
	E_SCANMODE_KINEMATIC = 0x01,	//���˶�ģʽ
	E_SCANMODE_FIXEDSCAN = 0x02		//��������
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
	CString strModelName;						//����ͺ���
	CString strID;								//���ID
	CString strHardwareVersion;					//Ӳ���汾
	CString strFirmwareVersion;					//�̼��汾
	CString strDeviceAddress;					//IP��ַ
	int	nPort;									//�˿ں�
}T_MECHEYE_CAMREA_DRIVER_PARAM;//÷�������������

typedef struct
{
	int nCameraType;							//�������
	int nColorWidth;							//��ͼ��
	int nColorHeight;							//��ͼ��
	int nInfraredWidth;							//����ͼ��
	int nInfraredHeight;						//����ͼ��
	int nDepthWidth;							//��ȿ�
	int nDepthHeight;							//��ȸ�
	bool bEnableUndistort;
	cv::Mat cvCameraMatrix;
	cv::Mat cvDistCoeffs;
}T_DEPTH_CAMREA_DRIVER_PARAM;//ȫ������ͺŲ���

typedef struct
{
	CString  strDeviceAddress;					//IP��ַ
	int nCameraAcquisitionMode;					//����ģʽ
	int nCallBackMode;							//�ص�ģʽ
	double dExposureTime;						//�ع�ʱ��
	double dGainLevel;							//����
	int nRoiWidth;								//��
	int nRoiHeight;								//��
	int nMaxWidth;								//��
	int nMaxHeight;								//��
	int nRoiOffsetX;							//ROI��X����ƫ��
	int nRoiOffsetY;							//ROI��Y����ƫ��
	int nStrobeLine1;							//������������1
	int nStrobeLine2;							//������������2

}T_DH_CAMREA_DRIVER_PARAM;//����������

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
	//���ɽ�����
	int nAntisplashOpenedIO;								//���ɽ������״̬IO
	int nAntisplashClosedIO;								//���ɽ�����ر�״̬IO
}T_CAMERA_IO_INPUT;
typedef struct
{
	//������
	//����ź�
	std::vector<int> vnLeaseLineIO;							//ͬһ��������ж���������

	//�����
	int nSupLEDIO;											//������ܿ�IO
	int nSupLED1IO;											//�����1IO
	int nSupLED2IO;											//�����2IO

	//���ɽ�����
	int nAirSolenoidValueIO;								//��·��ŷ�IO
	int nAntisplashIO;										//���ɽ�����IO

	//����ź�
	int m_nCameraPowerIO;									//��Դ����
}T_CAMERA_IO_OUTPUT;
typedef struct
{
	std::vector<CString> vsLaserIOName;
	T_CAMERA_IO_INPUT tInputIO;
	T_CAMERA_IO_OUTPUT tOutputIO;
}T_CAMERA_IO;

//2023/12/21 ���Ӽ�
typedef struct
{
	double drYMaxCar;				//Y��������λ��
	double drYMinCar;				//Y�����
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
	double dStartWave; //���ڻ�����
	double dEndWave;//�յ�ڻ�����
	double dWaveDistance;//�ڶ����
	double dEndSpeedRate;//ĩ������
	double dLeftWaitTime;//���ȴ�ʱ��	 // �������5mm��ͣ��ʱ��
	double dRightWaitTime;//�Ҳ�ȴ�ʱ��
	double dMidWaitTime;//�м�ȴ�ʱ��   // �����յ�12mm��ͣ��ʱ��
	double dVerticalAngle;//�����ĺ�������
	int waveType;//�����İڶ���ʽ
	double dWaveHeight;//���ǰں����ΰڵĸ�
	double dWaveUpBase;//���ΰڵ��ϵ�
	double dNormalOffset;//����ƫ��
	double dHorizontalOffset;//����ƫ��
	T_WAVE_PARA()
	{
		memset(this, 0, sizeof(T_WAVE_PARA));
	}
};//���������ڻ�ר��

struct T_GROOVE_INFOR
{
	std::vector<T_ROBOT_COORS> vtStartPoint;//�¿������Ϣ
	std::vector<T_ROBOT_COORS> vEndPoint;//�¿��յ���Ϣ
	T_ROBOT_COORS tStartPoint;//�¿������Ϣ
	T_ROBOT_COORS tEndPoint;//�¿��յ���Ϣ
	double dStartThickness; // �����
	double dEndThickness; // �յ���
	double dPlateThickness;//����
	double dStartLowerFace;//��������
	double dStartUpperFace; //���������
	double dEndLowerFace;//�յ������
	double dEndUpperFace;//�յ�������
	double weldAngle;//�����¿ڷ���ƽ����Ч
	int nStartChangeType; // ��㴦��0���� 1���� 2���
	int nEndChangeType; // �յ㴦��0���� 1���� 2���
	T_GROOVE_INFOR()
	{
		memset(this, 0, sizeof(T_GROOVE_INFOR));
	}
};

struct T_INFOR_WAVE_RAND
{
	int nLayerNo;
	std::vector<T_ROBOT_COORS> vtStartPoint;//�������ƫ����Ϣ
	std::vector<T_ROBOT_COORS> vtEndPoint;//�������ƫ����Ϣ
	T_ROBOT_COORS tStartPoint;//�������ƫ����Ϣ
	T_ROBOT_COORS tEndPoint;//�����յ�ƫ����Ϣ
	double dAngleOffset;//������̬����׼��̬ƫ��
	double dStartWaveDis; // ���ڶ����� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	double dEndWaveDis; // �յ�ڶ����� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	double dStartSpeedRate; // ����ٶȱ��� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	double dEndSpeedRate; // �յ��ٶȱ��� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	double dStartWaveWaitTime; // ���ͣ��ʱ�� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	double dEndWaveWaitTime; // �յ�ͣ��ʱ�� �ŵ�ʱ���� ����ڶ��켣ʱʹ��
	T_INFOR_WAVE_RAND()
	{
		memset(this, 0, sizeof(T_INFOR_WAVE_RAND));
	}
};//������������յ���Ϣ

//2023/12/21 ���Ӽ�
typedef struct
{
	double dJointAngleTheta;
	double dTwistAngleAlpha;
	double dLinkLengthA;
	double dLinkOffsetD;
}T_LINK_PARAM;

typedef struct // �����ڶ������л�ȡ��˹��ʹ�õİڶ�����
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
	int nWeldAngleSize; // ���ųߴ� ��֧�������ߴ�
	int nLayerNo; // ��������
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

	double CrosswiseOffset;//���򲹳�
	double verticalOffset;//��ֱ���򲹳�
	int nWrapConditionNo; // �ڶ�������
	double dWeldAngle; // ���ӽǶ� ��˿�͵װ���������н�
	double dWeldDipAngle; // ������� ҩо��˿(��) ʵо��˿(��) ��ǲ�ͬ
	int nStandWeldDir; // �������ӷ���0:������  1:������

	// CO2/����� ʵ��/ҩо ����/ֱ�� 1.2/1.4 �Ȳ�ͬ���
	int nWeldMethod; // ���ӷ��� 1����ֱ�/0ֱ���ֱ� ��0���� 1ƽ����

	T_WEAVE_PARA tWeaveParam; // �ڶ�����(��˹�ٰڶ���Ҫ��̬д��ʹ��)
	T_WAVE_PARA tGrooveWaveParam; // �����¿ڰڶ�����
}T_WELD_PARA;

typedef struct T_WELD_SEAM_TYPE_PARA
{
	T_WELD_PARA VerticalWeldPare;
	T_WELD_PARA SlopeFlatWeldPare;//б��ƽ��
	T_WELD_PARA LevelFlatWeldPare;//ƽ�Խ�
	T_WELD_PARA FlatSeamingWeldPare;//�߶�ƽ��
	T_WELD_PARA CornerLineWeldPare;
	T_WELD_PARA StrengthBoardWeldPare;
	T_WELD_PARA MarginStraightSlitWeldPare;
	T_WELD_PARA FrontPlateUnderFrameWeldPare;
}T_WELD_SEAM_TYPE_PARA;

//������ȫ���̺���	2023/12/21 ���Ӽ�
typedef enum
{
	//ͨ������
	RUNNING_STATUS_SUCCESS = 1,		//�������óɹ�
	RUNNING_STATUS_EMG = -1,		//�豸��ͣ
	RUNNING_STATUS_STOP = -2,		//ϵͳ��ͣ
	RUNNING_STATUS_INVALID_COORDINATES = -3,		//��Ч������λ��
	RUNNING_STATUS_OUT_OF_RANGE_COORDINATES = -4,		//���޵�����λ��
	RUNNING_STATUS_TIME_OUT = -5,		//��ʱ
	RUNNING_STATUS_INVALID_DATA = -6,		//���ļ�������Ч����
	RUNNING_STATUS_TOO_MANY_SEGMENTS = -7,		//��ζι��ྯ��
	RUNNING_STATUS_ABNORMAL_COORDINATES = -8,		//��������Ԥ�������λ�ã��쳣���꣩
	RUNNING_STATUS_TEST = -9,		//������
	RUNNING_STATUS_EXTERNAL_AXLE_OUT_OF_RANGE = -10,		//�ⲿ�ᳬ��
	RUNNING_STATUS_REPEAT_CALL = -11,		//�ظ����ú���
	RUNNING_STATUS_INVALID_ROBOT_NUM = -12,		//��Ч�Ļ�е������
	RUNNING_STATUS_LOCATION_IS_NOT_REACHED = -13,		//δ����ָ������λ��
	RUNNING_STATUS_UNKNOWN_ERROR = -99,		//δ֪����

	//CheckDone
	RUNNING_STATUS_PROXIMITY = -100,		//CheckDone�������ӽ�����������

	RUNNING_STATUS_CHECK_COORS_ERROR = 150,
	RUNNING_STATUS_CHECK_PULSE_ERROR = 151,

	//����ͼֽ
	RUNNING_STATUS_EMPTY_PATH = -200,		//��·��
	RUNNING_STATUS_WRONG_PATH = -201,		//����·��

	//ȫ��/�Ҷ�ʶ��
	RUNNING_STATUS_SAVE_DEPTH_IMG_FAILED = -300,		//ȫ����ͼʧ��
	RUNNING_STATUS_NO_WORKPIECE = -301,		//����̨�޿��ù���
	RUNNING_STATUS_WRONG_TEMPLATE_NO = -302,		//�����ģ����
	RUNNING_STATUS_RECOGNIZE_FAILED = -303,		//ȫ��ʶ��ʧ��
	RUNNING_STATUS_SAVE_GRAY_IMG_FAILED = -350,		//�ҶȲ�ͼʧ��

	//������
	RUNNING_STATUS_MISSING_WORKPIECE = -400,		//������ʧ	
	RUNNING_STATUS_FIND_NO_WORKPIECE = -401,		//�½�������δ���ֹ���
	RUNNING_STATUS_DEMAGNETIZING_FAILED = -402,		//�˴�ʧ��	
	RUNNING_STATUS_MAGNETIZING_FAILED = -403,		//���ʧ��

	//ʾ��
	RUNNING_STATUS_ANTISPLASH_CLOSED = -500,		//���ɽ�����رգ��޷���
	RUNNING_STATUS_TEACH_ERROR_THRESHOLD = -501,		//ʾ��������ֵ
	RUNNING_STATUS_CALC_TEACH_TRACK_FAILED = -502,		//����ʾ�̹켣����
	RUNNING_STATUS_TEACH_GET_IMAGE_FAILED = -503,		//ʾ�̲�ͼʧ��
	RUNNING_STATUS_FIND_CORNNER_FAILED = -504,		//FindCornnerʧ��

	//ƥ�乤��
	RUNNING_STATUS_WRITE_PROCESS_ID_FAILED = -601,		//д�빤��IDʧ��,����ר��
	RUNNING_STATUS_LOAD_PROCESS_PARA_FAILED = -602,		//ƥ�乤��ʧ��

	//��ǹ
	RUNNING_STATUS_INVALID_TRACK = -700,		//��Ч�Ĺ켣
	RUNNING_STATUS_INVALID_IGNITION_HEIGHT = -701,		//��Ч�������߶�
	RUNNING_STATUS_INVALID_ANGLE_COMPEN = -702,		//��Ч�ĽǶȲ���

	//�и�
	RUNNING_STATUS_POINTS_OVERFLOW = -800,		//�켣��������
	RUNNING_STATUS_INVALID_CUTTING_HEIGHT = -801,		//��Ч���и�߶�
	RUNNING_STATUS_POS_VAR_OVERFLOW = -802,		//P��������
	RUNNING_STATUS_CHECK_POS_VAR_FAILED = -803,		//P����У��ʧ��
	RUNNING_STATUS_CHECK_INT_VAR_FAILED = -804,		//I����У��ʧ��
	RUNNING_STATUS_ARCING_FAILED = -805,		//��ʧ��
	RUNNING_STATUS_SMALL_ARCING_FAILED = -805,		//��С��ʧ��
	RUNNING_STATUS_ARC_BREAK = -807,		//�ϻ�	
	RUNNING_STATUS_INVALID_INT_VAR = -808,		//��Ч��I����
	RUNNING_STATUS_I_VAR_CHANGED_OVERTIME = -809,		//I���������ʱ
	RUNNING_STATUS_UNKNOW_INCISE_WAYS = -810,		//δ֪���иʽ

	//��ĥ
	RUNNING_STATUS_SET_POLISHER_ANGLE = -900,		//������ĥͷ�Ƕ�ʧ��
	RUNNING_STATUS_WRONG_POLISHER_STATUS = -901,		//����Ĵ�ĥ���״̬

	//����
	RUNNING_STATUS_ARCING_FAILED_REMOVE_POINTS_OVERFLOW = -1900,	//�Զ������ﵽ����
}E_ERROR_STATE;

//2023/12/21 ���Ӽ�
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

//2023/12/21 ���Ӽ�
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
	bool bAutoOpen = true;						//�򿪳����Ƿ��Զ���(����������Ч)
	int nInstallPos = 5;						//�����װλ��
	E_FLIP_MODE eFlipMode = E_FLIP_NONE;		//ͼ��תģʽ
	int nSameTypeNo = 0;						//ͬ����������
	CString strCameraName;						//�������
	CString strCameraNameEN;					//���Ӣ������
	E_CAMERA_TYPE eCameraType;					//�������
	T_ROBOT_COORS tCameraTool;					//�������
	T_DH_CAMREA_DRIVER_PARAM tDHCameraDriverPara;	//����������
	T_DEPTH_CAMREA_DRIVER_PARAM tPANOCameraDriverPara;	//ȫ���������
	T_MECHEYE_CAMREA_DRIVER_PARAM tMecheyeCameraDriverPara;	//÷�������������
	T_HAND_EYE_CALI_PARAM tHandEyeCaliPara;		//���۹�ϵ
	T_CAMERA_TRANSFORM_PARAM tCameraTransfomPara;	//���ת����ϵ
	T_CAMERA_DISTORTION tCameraDistortion;		//�������
	T_LASER_PLAN_EQUATION tLaserPlanEquation;	//������ƽ�淽��ϵ��
	T_CAMERA_IO tCameraIO;						//���IO����
	XI_POINT  tPointCloudCompen;				//���Ʋ���
	double ppdHandEyeCompen[4][4];				//���۲�������ȡ��ԭ���ĵ��Ʋ�����
}T_CAMREA_PARAM;

typedef struct
{
	int nSoftAxisNo;
	int nToRobotDir;
}T_MOTOR_PARAM;


typedef struct
{
	CString strShowName;					//����̨����
	T_TABLE_PARA tTablePara;				//�㷨�ù���̨����
	std::vector<double> vdRotateAngle;		//��Ը���е�۵ĽǶ�
	T_ANGLE_PULSE tPlaceCompartPulse;		//�����ϱȶ�����
	T_ANGLE_PULSE tPanoRecogPulse;			//ȫ��ʶ��λ��
	T_ANGLE_PULSE tLineScanRecogPulse;		//��ɨʶ���ʼλ��
	T_ANGLE_PULSE tGrayCorrectionPulse;		//�Ҷ�����λ��
	T_ROBOT_COORS tPolishPointInPolishRobot;//��ĥ
	double dUserBaseRotationAngle;			//�ҶȽǶ�
	double dTableBaseDepth;					//���ʶ�����
	double dHeightCompen;					//�����ϸ߶Ȳ���
	double dSlagHeightCompen;				//ͬ���������ϸ߶ȶ��ⲹ��
	double dRecoHeight;						//��ʵʶ��߶�
	double dSafeHeight;						//���˻�е���ƶ���ȥ�İ�ȫ�߶�
	double dTrayHeight;						//���̸߶�
}T_ALL_TABLE_PARA;

typedef struct
{
	//��е������ź�
	int nRobotRunningIO;									//��е��������IO
	int nRobotServoOnIO;									//��е���ŷ���ͨIO
	int nErrorsIO;											//��е�۴���/����IO
	int nEmgStopIO;											//��е��ͣIO
	int nBatteryWarningIO;									//��ر���IO
	int nOperationalOriginIO;								//��ҵԭ��IO
	int nMidwayStartIO;										//��;����IO
	int nChoiceLongRangeModeIO;								//ѡ��Զ��ģʽIO
	int nChoiceManualModeIO;								//ѡ���ֶ�ģʽIO
	int nChoiceTeachModeIO;									//ѡ��ʾ��ģʽIO
}T_YASKAWA_INPUT_IO;

typedef struct
{
	//��е�������ź�
	int nRobotInitIO;										//��е���ⲿ����IO
	int nRobotPauseIO;										//��е������ͣIO
	int nCallJobIO;											//��е�۳������
	int nServoOnIO;											//��е���ⲿ�ŷ���ͨIO	
	int nResetErrorsIO;										//��е�۴�������IO		
	int nRobotExEmgIO;										//��е���ⲿ��ͣIO	
	int nProhibitOperationIO;								//��е�۽�ֹ��ҵIO
}T_YASKAWA_OUTPUT_IO;

typedef struct
{
	T_YASKAWA_INPUT_IO tRobotInputIO;
	T_YASKAWA_OUTPUT_IO tRobotOutputIO;
}T_YASKAWA_ROBOT_IO;
typedef struct
{
	//����ź�
	int nAirSolenoidIO;					//��·��ŷ�����

	//�����ź�
	int nAirPressureSignalIO;			//����ѹǿ�ź�
}T_GAS_IO;
typedef struct
{
	int nCautionLightGreenIO_OUT;//��ʾ����
	int nCautionLightYellowIO_OUT;//��ʾ�ƻ�
	int nCautionLightRedIO_OUT;//��ʾ�ƺ�
}T_WARNING_LIGHT_IO;

typedef struct
{
	//����ź�
	int nAirSolenoidIO;					//��·��ŷ�����

	//�����ź�
	int nAirPressureSignalIO;			//����ѹǿ�ź�
}T_MAGNET_IO;

typedef struct
{
	int nAllEmgStopIO;//����ͣ
}T_EMG_STOP_IO;

typedef struct
{
	int nRobotExternalPowerControl;//�����˿��ƹ��Դ
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
	E_LS_IO = 0,	// ����IO
	E_HC_IO = 1,	// �㴨IO
	E_YASKAWA_IO = 2,	// ����������IO
	E_ESTUN_IO = 3,	// ��˹�ٻ�����IO
	E_CROBOTP_IO = 4,	// ��ŵ�ջ�����IO
}E_IO_TYPE;

typedef struct
{
	CString sNameCN;	// IO����
	CString sNameEN;	// Ӣ����
	bool bReadOnly;		// true��ֻ��  false����д
	int nCardNo;		// һ��������Ϊ0  �ж�������ε���
	int nCardNodeNo;	// 0:����IO  1:ģ��0   2:ģ��1 ���� (������IO�˲�����Ч)
	int nType;			// 0:���� �㴨 ���������� ��˹�ٻ����� ��ŵ�ջ�����
	int nDefaultVal;	// ����״̬�µ�Ĭ��ֵ���缤��ر� �޼�ͣ ��ײ �������
	int nBit;			// IO��
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

// �˶�ģʽ����Ծ���
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
	//*****************************�жϷּ����*****************************//
	RTN_STATUS_SUCCEED = 0x0000,		//�ɹ�
	RTN_STATUS_CAMPLATE = 0x0001,		//���
	RTN_STATUS_VOID = 0x0002,		//��״̬
	RTN_STATUS_MANUAL_CANCEL = 0x0003,		//�ֶ�ȡ��
	RTN_STATUS_CANCEL = 0x0004,		//ȡ��
	RTN_STATUS_MANUAL_CONTINUE = 0x0005,		//�ֶ�����

	////////////����/////////////
	RTN_STATUS_FILE_VOID_PATH = 0x1001,//��·��
	RTN_STATUS_FILE_WRONG_PATH = 0x1002,//����·��
	RTN_STATUS_FILE_FAILD = 0x1003,//��ȡ����ʧ��
	//��ȡͼֽ��Ϣ
	RTN_STATUS_FILE_GRAB_COOR_ERROR = 0x1101,//��ȡץȡ����ʧ��
	RTN_STATUS_FILE_WORKPIECE_INFO_ERROR = 0x1102,//��ȡ������Ϣʧ��
	RTN_STATUS_FILE_STEEL_PLATE_INFO_ERROR = 0x1103,//��ȡ�ְ���Ϣʧ��
	RTN_STATUS_FILE_WORKPIECE_INFO_VOID = 0x1104,//������Ϣȱʧ
	//��������
	RTN_STATUS_DATA_NO_TRAY = 0x1201,//û������
	RTN_STATUS_DATA_ERROR = 0x1201,//û������
	RTN_ONE_TRAY_INDEX_ERROR = 0x1202,//û��һ���������


	////////////��д�����ļ�����/////////////
	RTN_STATUS_INI_NO_FILE = 0x2001,//�ļ�������
	RTN_STATUS_INI_NO_DATA = 0x2002,//���ݲ�����

	////////////���ݴ���/////////////
	RTN_STATUS_PROCESS_UNKNOWN_ERROR = 0x3001,//δ֪����
	RTN_STATUS_PROCESS_DEFECT = 0x3002,//ȱʧ�������
	RTN_STATUS_PROCESS_DEFECT_MAT = 0x3003,//ȱʧλ�þ���
	RTN_STATUS_PROCESS_DATA_ERROR = 0x3004,//���ݴ���
	//�ж����
	RTN_STATUS_PROCESS_CUTTING_HEIGHT_ERROR = 0x3101,//�߶�ƫ����޷��и�

	////////////����ת��/////////////
	RTN_STATUS_PROCESS_TRANSFOR_ERROR = 0x4001,//����ת��ʧ�ܣ����ݴ���
	RTN_STATUS_MKMD_TO_ROBOT_ERROR = 0x4002,//�������ת�����˵���ʧ��


	////////////�˶�/////////////
	RTN_STATUS_MOVE_RUNNING = 0x5001,//����������
	RTN_STATUS_MOVE_EMG = 0x5002,//�豸��ͣ
	RTN_STATUS_MOVE_ROBOT_EMG = 0x5002,//�����˼�ͣ
	RTN_STATUS_MOVE_NO_RUNNING = 0x5003,//û���˶�
	RTN_STATUS_MOVE_PLACE_ERROR = 0x5004,//λ�ô���
	RTN_STATUS_MOVE_ENCODER_DISCONNECT = 0x5005,//����������ʧ��
	RTN_STATUS_MOVE_ERROR_AXIS_NO = 0x5006,//��������
	RTN_STATUS_MOVE_PULSE_LIMIT = 0x5007,//���峬��
	RTN_STATUS_MOVE_PERFORATE_FAILD = 0x5008,//Ѳ��ʧ��
	RTN_STATUS_MOVE_AXIS0_FAILD = 0x5009,//�ⲿ��0�˶�ʧ��
	RTN_STATUS_MOVE_AXIS1_FAILD = 0x500A,//�ⲿ��1�˶�ʧ��
	RTN_STATUS_MOVE_AXIS2_FAILD = 0x500B,//�ⲿ��2�˶�ʧ��
	RTN_STATUS_MOVE_VAR_ERROR = 0x500C,//�������ô���
	RTN_STATUS_MOVE_NO_ERROR = 0x500D,//��Ŵ���
	RTN_STATUS_MOVE_NO_SAFE_POS = 0x500E,//û���ڰ�ȫλ��
	RTN_STATUS_MOVE_DEVICE_ERROR = 0x500F,//�豸����
	RTN_STATUS_MOVE_FAILD = 0x5010,//�ⲿ���˶�ʧ��

	////////////�Ӿ�/////////////
	RTN_STATUS_IMAGE_CAMERA_ERROR = 0x6001,//��ͼʧ��
	RTN_STATUS_IMAGE_TYPE_ERROR = 0x6002,//ͼ�����ʹ���
	RTN_STATUS_IMAGE_RESULT_UNRELIABLE = 0x6003,//������ɿ�
	RTN_STATUS_IMAGE_RESULT_ERROR = 0x6004,//ʶ��������
	RTN_STATUS_IMAGE_LAREGE_ERROR = 0x6005,//ʶ����ƫ���
	RTN_STATUS_IMAGE_NO_RESULT = 0x6006,//û�з��ؽ��
	RTN_STATUS_IMAGE_PROCESS_FAIL = 0x6007,//ʶ��ʧ��

	////////////�豸/////////////
	RTN_STATUS_DERVICE_NOT_READY = 0x6001,//�豸δ׼����
	RTN_STATUS_DERVICE_UNKNOWN_STATE = 0x6002,//�豸״̬δ֪
	RTN_STATUS_DERVICE_CUTTING_GAS_OPEN_FAIL = 0x6003,//�и�����ʧ��
	RTN_STATUS_DERVICE_CUTTING_GAS_CLOSE_FAIL = 0x6004,//�и����ر�ʧ��
	RTN_STATUS_DERVICE_LASER_OPEN_FAIL = 0x6005,//�����ʧ��
	RTN_STATUS_DERVICE_LASER_CLOSE_FAIL = 0x6006,//����ر�ʧ��
	RTN_STATUS_DERVICE_MAGNET_OPEN_FAIL = 0x6007,//��������ʧ��
	RTN_STATUS_DERVICE_MAGNET_CLOSE_FAIL = 0x6008,//������˴�ʧ��
	RTN_STATUS_DERVICE_FAULT = 0x6009,//�޴��豸
	RTN_STATUS_DERVICE_ERROR = 0x600A,//�豸�쳣


	////////////�ּ����/////////////
	RTN_STATUS_SORTING_MAGNET_NO_SHRINK = 0x7001,//����ʧ��
	RTN_STATUS_SORTING_SEND_10003_FALSE = 0x7002,//�ּ�ѯ���пط���λ�÷���10003����False
	RTN_STATUS_SORTING_SEND_10005_FALSE = 0x7003,//�ּ���ð�����ɷ���10005��֪�п�False
	RTN_STATUS_SORTING_WAIT_CENTENT_TIME_LONG = 0x7004,//�ּ���õȴ��п�ʱ��̫��


	//////////���˶�����������///////////////////
	RTN_STATUS_SECONDARRANGE_POSITION_FALSE= 0x8001,//���̵��������
	RTN_STATUS_SECONDARRANGE_POSITION_UNREACHABLE = 0x8002,//���̵����겻�ɴ�
	RTN_STATUS_SECONDARRANGE_EXTERNALAXIS_UNREACHABLE = 0x8003//�ⲿ�Sδ�\�ӵ�λ

}E_RUNNING_RTN_STATE;

//2023/12/21 ���Ӽ�
// ���������� (�켣������)
typedef enum E_MEASURE_POINT_TYPE
{
	E_TRANSITION_POINT		= 1 << 0,		// ����ǹ���ɵ�
	E_TEACH_POINT			= 1 << 1,		// ����켣���ԣ��Ȳ��ʾ�̵�
	E_ADJUST_POINT			= 1 << 2,		// ����켣���ԣ�����������������
	E_SEARCH_POINT			= 1 << 3,		// ����켣���ԣ����������� �ɶԳ��� ��һ������� �ڶ������յ�
	E_SCAN_POINT			= 1 << 4,		// ����켣���ԣ�ɨ������� ͨ��ɨ�躸�������ȡ����
	E_ARRIS_POINT			= 1 << 5,		// ����켣���ԣ����ɶ���ǲ����� ������������ ȷ��ֱ������ƽ��ͶӰ
	E_DOUBLE_LONG_LINE		= 1 << 6,		// ͼ�������ԣ�˫���߲�����
	E_LS_RL_FLIP			= 1 << 7,		// ͼ�������ԣ��������Ҳ೤�� �� ��Ҫ���·�תͼ�� ������
	E_LL_RS_FLIP			= 1 << 8,		// ͼ�������ԣ����а��������Ե�����˵�Ķ̱�ͼ��
	E_WELD_ARC_ON			= 1 << 9,		// ���ӹ켣���ԣ��𻡹켣 64
	E_WELD_WRAP				= 1 << 10,		// ���ӹ켣���ԣ����ǹ켣 128
	E_WELD_TRACK			= 1 << 11,		// ���ӹ켣���ԣ����ӹ켣 256
	E_WELD_ARC_OFF			= 1 << 12,		// ���ӹ켣���ԣ�ͣ���켣 512��һ��ͣ������Ҫ�˶���û���˶��켣��
	E_L_LINE_POINT			= 1 << 13,		// ͼ��������: �ǵ����⼤�����ϵĵ�
	E_R_LINE_POINT			= 1 << 14,		// ͼ��������: �ǵ���Ҳ༤�����ϵĵ�
	E_WELD_CORNER_WRAP				= 1 << 15,					// ���ӹ켣���ԣ�����ת�ǹ켣
	E_WELD_OPEN_TRACKING			= 1 << 16,					// ���ӹ켣���ԣ��ٴο�������
	E_WELD_WRAP_CHANGE				= 1 << 17,					// ���ӹ켣���ԣ�����ǰ�任һ�ε�����ѹ
	E_TRANSITION_ARCOFF_FAST_POINT	= 1 << 18,					// ���ɹ켣���ԣ����ӹ���ͣ�����ٹ���
	E_WELD_TRACK_CHANGE_POSTURE		= 1 << 19,					// ���ӹ켣���ԣ�����̬���ӹ켣
	E_SEARCH_POINT_FIX				= 1 << 20 				    // ����켣���ԣ���������������
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

//ȡ��ԭ����XiMessageBoxGroup���ܵ������ϵĵ�ѡ�����
XUI::MesBox& GetNaturalPop();

//int XiMessageBox(CLog *cLog, CString str);
//int XiMessageBox(CString str);
//int XiMessageBoxPopup(CString str, BOOL bPopup = TRUE);
//int XiMessageBoxPopup(CLog *cLog, CString str, BOOL bPopup = TRUE);
//int XiMessageBoxPopup(CLog *cLog, CString str, bool bPopup = true);
//int XiMessageBoxGroup(bool bIfGroup, char* format, ...);
//int XiMessageBoxGroup(BOOL bIfGroup, char* format, ...);
//int XiMessageBox(CLog *cLog, char *format, ...);
int XiMessageBox(const char* format); // ��ȷ����ȡ����ť
//int XiMessageBoxOk(CLog *cLog, char *format, ...); // ֻ��ȷ����ť
//int XiMessageBoxOk(CLog *cLog, CString str); // ֻ��ȷ����ť
int XiMessageBoxOk(const char* format); // ֻ��ȷ����ť

void SetDlgItemData(int nItem, double data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, int data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, long data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, CString data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void SetDlgItemData(int nItem, CDialog* pDlg, BOOL bIfWindowOn, char* format, ...);
void SetDlgItemData(int nItem, CDialog* pDlg, char* format, ...);
void GetButtonState(CButton *btn, int &state, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void GetDlgItemData(int nItem, double &data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);
void GetDlgItemData(int nItem, int &data, CDialog *pDlg, BOOL bIfWindowOn = TRUE);

//��ֹ�����������Ӧ
void DoEvent();


CString GetModuleFilePath();

CString SaveFileDlg(CDialog* pFatherDlg, CString sFileType, CString sDefaultName, CString sDefaultPath, CString sTitle = "���Ϊ");
CString OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, CString sTitle = "��");

bool OpenFileDlg(CDialog *pFatherDlg, CString sFileType, CString sDefaultPath, std::vector< CString> &vstrFileList);
bool OpenFileDlg(CDialog* pFatherDlg, std::vector < CString> vsFileType, CString sDefaultPath, std::vector< CString>& vstrFileList);
//Utf8תGBK
CString Utf8ToGBK(LPCTSTR strUtf8);

//�༭�����ݴ�ֱ���У���Ҫ���ñ༭��Ϊ����
void VerticalCenter(CEdit &cEdit);

//����ļ��Ƿ����
//bool CheckFileExists(std::string sFileName, bool bCreate = false);
bool CheckFileExists(CString sFileName, bool bCreate = false);

//����ļ����Ƿ���ڣ��������򴴽�һ��
long CheckFolder(CString strPath);//1:�ļ��д��ڣ�0:�ļ��в����ڵ��Ѵ���������(����):����

//��ָ��·����Ѱ��ָ����չ����ȫ���ļ�
bool FindFile(std::vector<CString> &vstrFileList, CString strPath, CString strFileType, bool bIncludeSubfolder = true, bool bReturnPathOrName = true);

//���������ļ��е�ָ��·����
long CopyFolder(CString strSrcFolderName, CString strDstPath, bool bCopySubfolder = true);

//�޸�ԭ�ļ��е�����
BOOL ReNameFolder(LPCTSTR lpszFromPath, LPCTSTR lpszToPath);

//�������룬����dDECimalDigitsλС��
double DecimalRound(double dNum, int dDECimalDigits = 2);				

//�����ļ����е��ļ�
void DelFiles(CString directory_path);	
void DelFiles(CString directory_path, CString sFormat);
//�����ļ�������
void   DeleteFolder(CString sPath);

//дJOB����
void WriteJOB(int nLoopCount, int nStartPos, int nEndPos, E_ROBOT_CONTROL_CABINET_MODEL eCabinetModel = YRC1000, int nExternalAxle = 0);

//����ͼƬ
IplImage *LoadImage(CString strImgName);

//����ͼƬ
int SaveImage(IplImage *src, CString strImgName);
int SaveImage(IplImage *src, char *format, ...);
int SaveImage(IplImage **src, char *format, ...);
void XI_ReleaseImage(IplImage** img);

//��������ͱ����ı�ͼƬ��С
CvRect ResizeImage(IplImage* img, IplImage* &dst, double dRatio, CvPoint &tCenter);

//���ݿؼ���ʾ�����С��ͼƬ��С����ͼƬʵ�������ʾ���򣨻�Ŵ����С��
void CalImgRect(CRect &rectShowWholeImage, int nImageWidth, int nImageHeight, int nItem, CDialog *pDlg);

//�ڿؼ�����ʾͼƬ
void DrawImage(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross = false);
void DrawImage_new(IplImage* pImage, CDialog* pDialog, int nStaticId, CRect rectImage, bool bDrawCross = false);

//�Զ�ʹ��CalImgRect�������ڿؼ�����ʾͼƬ
void DrawImage(IplImage *pImage, CDialog *pDialog, int nStaticId, bool bDrawCross = false);

//������ʾ������ͼƬ,nStepNo��nPicNo�������ƽ��Զ���Ϊ���ƴ�С
void ShowStepPic(int &nStepNo, int &nPicNo, CString strPicPath, int nItem, CDialog *pDlg);

//����lDate��������
void CalDeadLine(SYSTEMTIME &tDeadline, SYSTEMTIME tTime, long lDate);		

//��ȫ���ļ���ȡ,Ĭ��ʹ��fopen_s�����б����ܣ�����0Ϊ������nShFlag��Ϊ_SH_DENYNO�൱��fopen
errno_t XI_fopen_s(FILE **file, CString strFileName, _In_z_ const char * _Mode, int nShFlag = _SH_SECURE);

//����16�������ַ���������unsigned long������
bool CStringToULongLong(unsigned long long &ullNum, CString strNum);

bool ULongLongToCString(CString &strNum, unsigned long long ullNum);

//
CString SystemTimeToCString(SYSTEMTIME tTime);
SYSTEMTIME CStringToSystemTime(CString sTime);
__int64 TimeDiff(SYSTEMTIME t1, SYSTEMTIME t2);

//����ж�
bool CheckContrarySign(double dNum1, double dNum2);				
bool CheckContrarySign(long lNum1, long lNum2);					

//�ؽڱ۲�����ȡ
void LoadRobotPara(CString strRobotName, T_KINEMATICS &tKinematics, T_AXISUNIT &tAxisUnit, T_AXISLIMITANGLE &tAxisLimitAngle);

//�������ͼ�����ݶ�д
void WriteAdjustMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo, bool bLineScan);
bool LoadMatchInfo(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart, bool bLineScan);
void WriteAdjustMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> vtMatchInfo);
bool LoadMatchInfoForLineScan(CString sDepthDataPath, double dCameraDistance, std::vector<T_RECOGNITION_PARA> &vtIdentifyPart);

//��֪���������߳��ͼнǽǶȣ���Ա߳�
double CalcTriangleSideLength(double dSide1Length, double dSide2Length, double dAngle);

//��֪���������߳�����Խ�
double CalcTriangleDiagonal(double dSide1Length, double dSide2Length, double dOppositeSideLength);

//����CRC
CString CalCRCData(unsigned long lData);

//��·���л�ȡ�ļ���
bool GetFileNameFromPath(CString sFilePath, CString &sFileName, bool bSuffix = true);

//��·���л�ȡ�ļ�·��
bool GetFilePathFromPath(CString sPath, CString &sFilePath);

//������С�������
int CalMinSampleGap(double dSpeed);

//���clock
long long XI_clock();

void PicMean(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

void PicMeanRemove0(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

void PicMax(std::vector<cv::Mat > vInPutDepth, cv::Mat &OutDepth);

//������
void Undistort(T_CAMERA_DISTORTION tCameraDistortion, IplImage *pImgGray, IplImage *pResultImgGray);

//΢�뼶��ʱ
void DelayUs(double uDelay);
//���뼶��ʱ
void DelayMs(double uDelay);

double XI_ContourRound(double r);

//�õ��ַ���
CString GetStr(char *format, ...);
CString GetStr(CString str);
CString GetStr(T_ROBOT_COORS tCoor, CString strToken = ",");
CString GetStr(T_ANGLE_PULSE tPulse, CString strToken = ",");
std::vector<CString> TokenizeCString(CString sSrc, CString sToken);
char* UnicodeToUtf8(char* str);

//��ʽת��
void TransforArray(T_ROBOT_COORS tCoor, double* adArray);
void TransforArray(T_ANGLE_PULSE tPulse, long* alArray);
void TransforArray(T_ANGLE_PULSE tPulse, double* adArray);
T_ANGLE_PULSE TransforPulse(long* alArray);
T_ANGLE_PULSE TransforPulse(double* adArray);
T_ROBOT_COORS TransforPos(double* adArray);


bool JudgeArea(double dAimPos, double dPos1, double dPos2);
void SaveErrorData(CString strName);
void SaveGrooveData(int nLayerNo, int nGroupNo, CString sRobotName, bool bSuccess); // �����¿ڲ�������
//��ȡ�����ڴ�
//double GetWorkingSetSize();

CString GetCurTime();

//void RunInteractiveWindow();
void RunInteractiveWindow(CString pointFileName, CString saveFileName, CString openFileName);
void RunInteractiveWindow(CString sUnitName);
bool AutomaticGrouping(CString sUnitName);

// ��ȡ�ڴ�ռ����� �� ��ӡ��־
void GetMemoryInfo(CString sTipInfo);

XI_POINT P2P(CvPoint tPtnSrc);
XI_POINT P2P(T_ROBOT_COORS tPtnSrc);
XI_POINT CreateXI_POINT(double x, double y, double z);

//���ַ�������ȡ��һ��double��ֵ����ȡ��֮�󣬸ö��ַ����ᱻɾ��
double GetDouble(char** str, int maxLen);

// ���㷨ʹ��
void AlgoAxisToRobotAxis(double& x, double& y, double& z);
void RobotAxisToAlgoAxis(double& x, double& y, double& z);
void AlgoAxisToRobotAxis(long& x, long& y, long& z);
//����ֵ:��ͬһ����ϵ�´�ԭʼλ�õ�Ŀ��λ�õ�����任����
//BasePoints:ԭʼλ���ĸ���Ӧ��������ϵ�µ�����
//TargetPoints:Ŀ��λ���ĸ���Ӧ��������ϵ�µ�����
/*���ĸ��㹲��,�����BaseCoordinateMatrix������,������󲻴���,�ڼ���BaseCoordinateMatrix_1ʱ���õ�����Ľ��*/
cv::Mat CalculateTransformationMatrix(std::vector<CvPoint3D64f>& BasePoints, std::vector<CvPoint3D64f>& TargetPoints);
//����ֵ:ԭʼλ�õ�任��Ŀ��λ�ú�Ľ������ͬһ����ϵ�µ�����
//BasePoints:ԭʼλ�õ�����
//TransformationMatrix:����任����
std::vector<CvPoint3D64f> CoordinateTransformate(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);
std::vector<CvPoint3D64f> CoordinateTransformateNormal(std::vector<CvPoint3D64f>& BasePoints, cv::Mat TransformationMatrix);	//����ת����ת����λ������xyz
long CopyFileFolder(CString strSrcFolderName, CString strDstPath);
 // ���ֱ��
bool		 CalcLineParamRansac(T_LINE_PARA& tBestLineParam, std::vector<XI_POINT> vtPoint, double dSelectedRatio);
static bool	 	 dCmpIncFunc(double dFirstValue, double dSecondValue);
T_SPACE_LINE_DIR CalLineDir(XI_POINT tStartPoint, XI_POINT tEndPoint);
double			 CalDisPointToLine(XI_POINT tPoint, XI_POINT tSegmentStartPoint, XI_POINT tSegmentEndPoint);
// ����ͶӰ��
double dot_product(XI_POINT V1, XI_POINT V2);
bool PointtoLineProjection(T_LINE_PARA tPointDir, T_ALGORITHM_POINT tpoint, T_ALGORITHM_POINT& tpointProjection);
bool PointtoLineProjection(T_LINE_PARA tPointDir, XI_POINT tpoint, XI_POINT& tpointProjection);
// ���ݼ�¼
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

//�ڶ����ٺ��Ӳ���
typedef struct SwingTrackPoint {
	T_ROBOT_COORS tLeftPoint;  //ǰ��
	T_ROBOT_COORS tMiddlePoint; //�е�
	T_ROBOT_COORS tRightPoint;	//���
}SwingTrackPoint;
//�ڶ����ٺ��Ӳ���
typedef struct SwingTrackWeldParam {
	//�˶���Ϣ
	std::vector<T_ROBOT_COORS> vtWorldCoors; //δ������
	std::vector<T_ROBOT_COORS> vtWeldRealCoors;  //�����˵���ʵ����
	std::vector<double> vdAxisCoors; //�ⲿ����ʵ����

	double dRobotWeldSpeed; //�������ٶ�
	double dAxisWeldSpeed;  //�ⲿ���ٶ�

							//��Ҫ����
	T_ROBOT_COORS startPtn; //�������
	T_ROBOT_COORS endPtn; //�����յ�

						  //������Ϣ
	T_WELD_PARA vtWeldPara; //���Ӳ���

	int SwingNumber;                     //�ڶ�������
	int nLayerNo;                        //���
	double dSwingFrequency;              //�ڶ�Ƶ��
	double dSwingLeftAmplitude;          //�ڶ������
	double dSwingRightAmplitude;         //�ڶ������
	double dSwingtime;                   //�ڶ�ͣ��ʱ��

	double dSwingOffsetLayerNo;          //�ڶ�����ƫ����
	bool bSwingMode;                     //�ڶ���ʽ
	
	SwingTrackPoint swingTrackPoint;     //�ڶ��켣

									     //����ʹ��
	cv::Vec3d norm;                      //����������
	size_t nFlat;                        //��¼���ӽṹ���еĸ��³���



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

/**************************** �߳���ز��� Start ****************************/
bool WaitAndCheckThreadExit(CWinThread* pWinThread, CString sThreadName); // �����ȴ��߳��˳� �̺߳�������0�ɹ� ����ʧ�� ��ʱʱ��С��0�޳�ʱ���
bool WaitAndCheckAllThreadExit(std::vector<CWinThread*> &vpWinThread); // �ȴ������߳̽��� �� ��������̵߳����н��
void ClearWinThread(std::vector<CWinThread*>& vpWinThread);
/**************************** �߳���ز��� End ****************************/

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

// ģ��ƥ��
bool Matching(E_WORKPIECE_TYPE eWorkPieceType);

bool ModelMatching(double dCorasrMatrix[4][4], const char* ModelFileName, int nWeldPartNo, std::string SaveRoute_weldline_point, double dFineMatrix[4][4], bool if_Fine_Match);

bool RemoveStringInFile(CString sFileName, std::string sRemovedString);

#endif // AFX_CONST_H__075B4081_4CAC_47E6_A30F_411__INCLUDED_
