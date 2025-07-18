#pragma once

#include "CContralUnit.h"
#include "CalPlaneVector.h"
#include "AnalyzeDxf.h"
#include "Dll_XiCV_Image_Processing.h"

#ifndef PAI
#define PAI 3.1415926535897932384626433832795
#endif // !PAI
#ifndef SQUARE
#define SQUARE(x)  (x)*(x)
#endif // !SQUARE
#ifndef DISTANCE2
#define DISTANCE2(x,y) sqrt(SQUARE(x)+SQUARE(y))
#endif // !DISTANCE2
#ifndef DISTANCE3
#define DISTANCE3(x,y,z) sqrt(SQUARE(x)+SQUARE(y)+SQUARE(z))
#endif // !DISTANCE3
#ifndef DISTANCE4
#define DISTANCE4(x,y,m,n) sqrt(SQUARE(x-y)+SQUARE(m-n))
#endif // !DISTANCE4

T_ROBOT_COORS operator-(T_ROBOT_COORS tRobotCoors);
//T_ROBOT_COORS operator-(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2);
bool operator==(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2);
bool operator!=(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2);
bool operator==(T_ANGLE_PULSE tRobotCoors1, T_ANGLE_PULSE tRobotCoors2);
bool operator<(T_ANGLE_PULSE tRobotCoors1, long lPulse);

class CUnitDataManagement
	: public CContralUnit
{
public:
	CUnitDataManagement(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver = NULL);
	~CUnitDataManagement();
	/***************************************************************************/
	/*                                通用                                     */
	/***************************************************************************/

	//坐标类型转换
	void TransforArray(T_ROBOT_COORS tCoor, double *adArray);
	void TransforArray(T_ANGLE_PULSE tPulse, long *alArray);
	void TransforArray(T_ANGLE_PULSE tPulse, double *adArray);
	T_ANGLE_PULSE TransforPulse(long *alArray);
	T_ANGLE_PULSE TransforPulse(double *adArray);
	T_ROBOT_COORS TransforPos(double *adArray);

	//得到两点距离
	double GetDistance(T_ROBOT_COORS tCoor1, T_ROBOT_COORS tCoor2);

	//得到脉冲差
	T_ANGLE_PULSE GetAxisDiff(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2);

	//得到最大脉冲数
	long GetMaxPulse(T_ANGLE_PULSE tPulse);

	//坐标绕Z轴旋转
	T_ROBOT_COORS Coor2DRotate(T_ROBOT_COORS tCoor, double dAngle);
	//2D坐标旋转


	/***************************************************************************/
	/*                              坐标转换                                   */
	/***************************************************************************/
	/*
	* TranCameraToBase
	* 梅卡曼德相机坐标转换
	* param		nCameraNo:电磁铁组	tPulse:充磁等级,1-6	tCoor:	tCameraCoor:
	* return	转换结果，机器人坐标
	*/
	T_ROBOT_COORS TranCameraToBase(int nCameraNo, T_ANGLE_PULSE tPulse, T_ROBOT_COORS tCoor, T_ROBOT_COORS tCameraCoor);
	T_ROBOT_COORS GetMechMindCameraTool(int nCameraNo);


	//线结构光相机坐标转换
	std::map<int, XiCV_Image_Processing*> m_mpXiLaserCvObj;
	XiCV_Image_Processing* m_pXiLaserCvObj = NULL;
	T_ROBOT_COORS TranImageToBase(int nCameraNo, CvPoint tImagePoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse, T_ABS_POS_IN_BASE* tPointAbsCoordInBase =NULL);
	std::vector<T_ROBOT_COORS> TranImageToBase(int nCameraNo, const std::vector<CvPoint> &vtImagePoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	XI_POINT CameraCoor2DTo3D(int nCameraNo, CvPoint tImagePoint);
	T_ROBOT_COORS CameraCoorToRobotCoor(int nCameraNo, XI_POINT tCameraCoor, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	T_ROBOT_COORS GetCameraTool(int nCameraNo);


	//结构光相机标定新方法坐标转换函数
	std::vector<CvPoint3D64f> CoordinateTransformation(std::vector<CvPoint2D64f>& PixelCoordinates, cv::Mat CameraMatrix, cv::Mat DistortionParameter, std::vector<double> LineLaserCalibrationResult, cv::Size ImageSize);
	void GetMeasureInWorldLeaser(CvPoint2D64f dPoint, T_ANGLE_PULSE tCurrentPulse, XI_POINT& tResult);
	T_ROBOT_COORS GetMeasureInWorldLeaser(int nCameraNo, CvPoint2D64f dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	T_ROBOT_COORS GetMeasureInWorldLeaser(int nCameraNo, CvPoint dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);
	std::vector<CvPoint2D64f> P2P(std::vector<CvPoint> vpnt);
	std::vector<T_ROBOT_COORS> GetMeasureInWorldLeaser(int nCameraNo, std::vector < CvPoint2D64f> dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse);

	inline void Rotate2D(double x1, double y1, double alpha, double& x2, double& y2)
	{
		x2 = x1 * cos(alpha) - y1 * sin(alpha);
		y2 = x1 * sin(alpha) + y1 * cos(alpha);
	}
	inline double GetTriangleHypotenuse(double side1, double side2)
	{
		return sqrt(SQUARE(side1) + SQUARE(side2));
	}
	inline double GetTriangleOtherSide(double hypotenuse, double side)
	{
		return sqrt(SQUARE(hypotenuse) - SQUARE(side));
	}





	/***************************************** 基础算法 ********************************************/
	double m_dExternalToBaseTransMatrix[4][4];
	double m_dBaseToExternalTransMatrix[4][4];	
	void InitialTransMatrix(double dBaseToExternalTransMatrix[][4]);
	void InitialTransMatrix(double dBaseToExternalTransMatrix[][4], double dExternalToBaseTransMatrix[][4]);
	void CoorTransform(std::vector<T_POINT_3D>& vtPoint, T_POINT_3D tCenter, double dRotAngle);

	void TransBasePointToWorldPoint(T_ROBOT_COORS tBaseRobotCoors, double dBaseToExternalTransMatrix[][4], T_ROBOT_COORS_IN_WORLD& tRobotCoorsInWorld);

	//转换关系
	void CaliCutRobotAndPolishRobotTransMatrixForTwoRobots(
		T_ROBOT_COORS tFirstPointInCutRobot, T_ROBOT_COORS tSecondPointInCutRobot, T_ROBOT_COORS tThirdPointInCutRobot, 
		T_ROBOT_COORS tFirstPointInPolishRobot, T_ROBOT_COORS tSecondPointInPolishRobot, T_ROBOT_COORS tThirdPointInPolishRobot, 
		double dMatrixCutRobotToPolishRobot[][4], double dMatrixPolishRobotToCutRobot[][4]);


};

