#include "stdafx.h"
#include "CUnitDataManagement.h"




bool operator==(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2)
{
	if (IsEqual(tRobotCoors1.dX , tRobotCoors2.dX, 0.1)
		&& IsEqual(tRobotCoors1.dY , tRobotCoors2.dY, 0.1)
		&& IsEqual(tRobotCoors1.dZ , tRobotCoors2.dZ, 0.1)
		&& IsEqual(tRobotCoors1.dRX , tRobotCoors2.dRX, 0.1)
		&& IsEqual(tRobotCoors1.dRY , tRobotCoors2.dRY, 0.1)
		&& IsEqual(tRobotCoors1.dRZ , tRobotCoors2.dRZ, 0.1))
	{
		return true;
	}
	return false;
}

bool operator!=(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2)
{
	if (IsEqual(tRobotCoors1.dX, tRobotCoors2.dX, 0.1)
		&& IsEqual(tRobotCoors1.dY, tRobotCoors2.dY, 0.1)
		&& IsEqual(tRobotCoors1.dZ, tRobotCoors2.dZ, 0.1)
		&& IsEqual(tRobotCoors1.dRX, tRobotCoors2.dRX, 0.1)
		&& IsEqual(tRobotCoors1.dRY, tRobotCoors2.dRY, 0.1)
		&& IsEqual(tRobotCoors1.dRZ, tRobotCoors2.dRZ, 0.1))
	{
		return false;
	}
	return true;
}

T_ROBOT_COORS operator-(T_ROBOT_COORS tRobotCoors1, T_ROBOT_COORS tRobotCoors2)
{
	T_ROBOT_COORS tTemp;
	tTemp.dX = tRobotCoors1.dX - tRobotCoors2.dX;
	tTemp.dY = tRobotCoors1.dY - tRobotCoors2.dY;
	tTemp.dZ = tRobotCoors1.dZ - tRobotCoors2.dZ;
	tTemp.dRX = tRobotCoors1.dRX - tRobotCoors2.dRX;
	tTemp.dRY = tRobotCoors1.dRY - tRobotCoors2.dRY;
	tTemp.dRZ = tRobotCoors1.dRZ - tRobotCoors2.dRZ;
	tTemp.dBX = tRobotCoors1.dBX - tRobotCoors2.dBX;
	tTemp.dBY = tRobotCoors1.dBY - tRobotCoors2.dBY;
	tTemp.dBZ = tRobotCoors1.dBZ - tRobotCoors2.dBZ;
	return tTemp;
}

T_ROBOT_COORS operator-(T_ROBOT_COORS tRobotCoors)
{
	T_ROBOT_COORS tTemp;
	tTemp.dX =  -tRobotCoors.dX;
	tTemp.dY =  -tRobotCoors.dY;
	tTemp.dZ =  -tRobotCoors.dZ;
	tTemp.dRX = -tRobotCoors.dRX;
	tTemp.dRY = -tRobotCoors.dRY;
	tTemp.dRZ = -tRobotCoors.dRZ;
	tTemp.dBX = -tRobotCoors.dBX;
	tTemp.dBY = -tRobotCoors.dBY;
	tTemp.dBZ = -tRobotCoors.dBZ;
	return tTemp;
}

bool operator==(T_ANGLE_PULSE tRobotCoors1, T_ANGLE_PULSE tRobotCoors2)
{
	if (tRobotCoors1.nSPulse == tRobotCoors2.nSPulse
		&& tRobotCoors1.nLPulse == tRobotCoors2.nLPulse
		&& tRobotCoors1.nUPulse == tRobotCoors2.nUPulse
		&& tRobotCoors1.nRPulse == tRobotCoors2.nRPulse
		&& tRobotCoors1.nBPulse == tRobotCoors2.nBPulse
		&& tRobotCoors1.nTPulse == tRobotCoors2.nTPulse
		&& tRobotCoors1.lBXPulse == tRobotCoors2.lBXPulse
		&& tRobotCoors1.lBYPulse == tRobotCoors2.lBYPulse
		&& tRobotCoors1.lBZPulse == tRobotCoors2.lBZPulse)
	{
		return true;
	}
	return false;
}

bool operator<(T_ANGLE_PULSE tRobotCoors1, long lPulse)
{
	if (abs(tRobotCoors1.nSPulse) < lPulse
		&& abs(tRobotCoors1.nLPulse) < lPulse
		&& abs(tRobotCoors1.nUPulse) < lPulse
		&& abs(tRobotCoors1.nRPulse) < lPulse
		&& abs(tRobotCoors1.nBPulse) < lPulse
		&& abs(tRobotCoors1.nTPulse) < lPulse
		&& abs(tRobotCoors1.lBXPulse) < lPulse
		&& abs(tRobotCoors1.lBYPulse) < lPulse
		&& abs(tRobotCoors1.lBZPulse) < lPulse)
	{
		return true;
	}
	return false;
}


CUnitDataManagement::CUnitDataManagement(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver)
	:CContralUnit(tCtrlUnitInfo, pServoMotorDriver)
{
	InitialTransMatrix(m_dBaseToExternalTransMatrix, m_dExternalToBaseTransMatrix);
}

CUnitDataManagement::~CUnitDataManagement()
{
	for (auto iter = m_mpXiLaserCvObj.begin(); iter != m_mpXiLaserCvObj.end(); iter++)
	{
		if (iter->second != NULL)
		{
			delete iter->second; 
			iter->second = NULL;
		}
	}
}

void CUnitDataManagement::TransforArray(T_ROBOT_COORS tCoor, double *adArray)
{
	adArray[0] = tCoor.dX;
	adArray[1] = tCoor.dY;
	adArray[2] = tCoor.dZ;
	adArray[3] = tCoor.dRX;
	adArray[4] = tCoor.dRY;
	adArray[5] = tCoor.dRZ;
	adArray[6] = tCoor.dBX;
	adArray[7] = tCoor.dBY;
	adArray[8] = tCoor.dBZ;
}

void CUnitDataManagement::TransforArray(T_ANGLE_PULSE tPulse, long *alArray)
{
	alArray[0] = tPulse.nSPulse;
	alArray[1] = tPulse.nLPulse;
	alArray[2] = tPulse.nUPulse;
	alArray[3] = tPulse.nRPulse;
	alArray[4] = tPulse.nBPulse;
	alArray[5] = tPulse.nTPulse;
	alArray[6] = tPulse.lBXPulse;
	alArray[7] = tPulse.lBYPulse;
	alArray[8] = tPulse.lBZPulse;
}

void CUnitDataManagement::TransforArray(T_ANGLE_PULSE tPulse, double *adArray)
{
	adArray[0] = tPulse.nSPulse;
	adArray[1] = tPulse.nLPulse;
	adArray[2] = tPulse.nUPulse;
	adArray[3] = tPulse.nRPulse;
	adArray[4] = tPulse.nBPulse;
	adArray[5] = tPulse.nTPulse;
	adArray[6] = tPulse.lBXPulse;
	adArray[7] = tPulse.lBYPulse;
	adArray[8] = tPulse.lBZPulse;
}

T_ANGLE_PULSE CUnitDataManagement::TransforPulse(long *alArray)
{
	T_ANGLE_PULSE tPulse;
	tPulse.nSPulse = alArray[0];
	tPulse.nLPulse = alArray[1];
	tPulse.nUPulse = alArray[2];
	tPulse.nRPulse = alArray[3];
	tPulse.nBPulse = alArray[4];
	tPulse.nTPulse = alArray[5];
	tPulse.lBXPulse = alArray[6];
	tPulse.lBYPulse = alArray[7];
	tPulse.lBZPulse = alArray[8];
	return tPulse;
}

T_ANGLE_PULSE CUnitDataManagement::TransforPulse(double *adArray)
{
	T_ANGLE_PULSE tPulse;
	tPulse.nSPulse = long(adArray[0] + 0.5);
	tPulse.nLPulse = long(adArray[1] + 0.5);
	tPulse.nUPulse = long(adArray[2] + 0.5);
	tPulse.nRPulse = long(adArray[3] + 0.5);
	tPulse.nBPulse = long(adArray[4] + 0.5);
	tPulse.nTPulse = long(adArray[5] + 0.5);
	tPulse.lBXPulse = long(adArray[6] + 0.5);
	tPulse.lBYPulse = long(adArray[7] + 0.5);
	tPulse.lBZPulse = long(adArray[8] + 0.5);
	return tPulse;
}

T_ROBOT_COORS CUnitDataManagement::TransforPos(double *adArray)
{
	T_ROBOT_COORS tCoor;
	tCoor.dX = adArray[0];
	tCoor.dY = adArray[1];
	tCoor.dZ = adArray[2];
	tCoor.dRX = adArray[3];
	tCoor.dRY = adArray[4];
	tCoor.dRZ = adArray[5];
	tCoor.dBX = adArray[6];
	tCoor.dBY = adArray[7];
	tCoor.dBZ = adArray[8];
	return tCoor;
}

double CUnitDataManagement::GetDistance(T_ROBOT_COORS tCoor1, T_ROBOT_COORS tCoor2)
{
	return DISTANCE3(tCoor1.dX + tCoor1.dBX - tCoor2.dX - tCoor2.dBX, tCoor1.dY + tCoor1.dBY - tCoor2.dY - tCoor2.dBY, tCoor1.dZ + tCoor1.dBZ - tCoor2.dZ - tCoor2.dBZ);
}

T_ANGLE_PULSE CUnitDataManagement::GetAxisDiff(T_ANGLE_PULSE tPulse1, T_ANGLE_PULSE tPulse2)
{
	return T_ANGLE_PULSE(tPulse2.nSPulse - tPulse1.nSPulse, tPulse2.nLPulse - tPulse1.nLPulse, tPulse2.nUPulse - tPulse1.nUPulse,
		tPulse2.nRPulse - tPulse1.nRPulse, tPulse2.nBPulse - tPulse1.nBPulse, tPulse2.nTPulse - tPulse1.nTPulse,
		tPulse2.lBXPulse - tPulse1.lBXPulse, tPulse2.lBYPulse - tPulse1.lBYPulse, tPulse2.lBZPulse - tPulse1.lBZPulse);
}

long CUnitDataManagement::GetMaxPulse(T_ANGLE_PULSE tPulse)
{
	long lPulse = fabs(tPulse.nSPulse) > fabs(tPulse.nLPulse) ? fabs(tPulse.nSPulse) : fabs(tPulse.nLPulse);
	lPulse = lPulse > fabs(tPulse.nUPulse) ? lPulse : fabs(tPulse.nUPulse);
	lPulse = lPulse > fabs(tPulse.nRPulse) ? lPulse : fabs(tPulse.nRPulse);
	lPulse = lPulse > fabs(tPulse.nBPulse) ? lPulse : fabs(tPulse.nBPulse);
	lPulse = lPulse > fabs(tPulse.nTPulse) ? lPulse : fabs(tPulse.nTPulse);
	lPulse = lPulse > fabs(tPulse.lBXPulse) ? lPulse : fabs(tPulse.lBXPulse);
	lPulse = lPulse > fabs(tPulse.lBYPulse) ? lPulse : fabs(tPulse.lBYPulse);
	lPulse = lPulse > fabs(tPulse.lBZPulse) ? lPulse : fabs(tPulse.lBZPulse);
	return lPulse;
}

T_ROBOT_COORS CUnitDataManagement::Coor2DRotate(T_ROBOT_COORS tCoor, double dAngle)
{
	T_ROBOT_COORS temp = tCoor;
	Rotate2D(tCoor.dX, tCoor.dY, dAngle * PI / 180.0, temp.dX, temp.dY);
	temp.dRZ += dAngle;
	return temp;
}

//PixelCoordinates:点集的像素坐标
//CameraMatrix:相机内参矩阵(3*3)
//DistortionParameter:相机畸变参数(5*1)
//LineLaserCalibrationResult:激光平面方程系数
std::vector<CvPoint3D64f> CUnitDataManagement::CoordinateTransformation(std::vector<CvPoint2D64f>& PixelCoordinates, cv::Mat CameraMatrix, cv::Mat DistortionParameter, std::vector<double> LineLaserCalibrationResult, cv::Size ImageSize)
{
	if ((PixelCoordinates.size() <= 0) || (CameraMatrix.rows != 3) || (CameraMatrix.cols != 3) || (LineLaserCalibrationResult.size() != 4))
		return std::vector<CvPoint3D64f>();
	cv::Mat pixel_coordinates = cv::Mat(int(PixelCoordinates.size()), 1, CV_64FC2);
	for (int i = 0; i < pixel_coordinates.rows; i++)
	{
		pixel_coordinates.ptr<cv::Vec2d>(i)[0][0] = PixelCoordinates[i].x;
		pixel_coordinates.ptr<cv::Vec2d>(i)[0][1] = PixelCoordinates[i].y;
	}
	cv::Mat real_pixel_coordinates = cv::Mat(pixel_coordinates.rows, 1, CV_64FC2);
	cv::Mat NewCameraMatrix = cv::getOptimalNewCameraMatrix(CameraMatrix, DistortionParameter, ImageSize, 1.0);
	cv::undistortPoints(pixel_coordinates, real_pixel_coordinates, CameraMatrix, DistortionParameter, cv::noArray(), NewCameraMatrix);
//	cv::undistortPoints(pixel_coordinates, real_pixel_coordinates, CameraMatrix, DistortionParameter, cv::noArray(), CameraMatrix);
	cv::Mat CameraMatrix_1;
	cv::invert(CameraMatrix, CameraMatrix_1, cv::DECOMP_LU);
	cv::Mat CameraCoordinate_Zc1 = CameraMatrix_1 * CameraMatrix;
	std::vector<CvPoint3D64f> CameraCoordinates(pixel_coordinates.rows);
	cv::Mat pixelCoordinate_mat = cv::Mat::zeros(3, 1, CV_64F);
	for (int i = 0; i < pixel_coordinates.rows; i++)
	{
		pixelCoordinate_mat.ptr<double>(0)[0] = real_pixel_coordinates.ptr<cv::Vec2d>(i)[0][0];
		pixelCoordinate_mat.ptr<double>(1)[0] = real_pixel_coordinates.ptr<cv::Vec2d>(i)[0][1];
		pixelCoordinate_mat.ptr<double>(2)[0] = 1;

		cv::Mat CameraCoordinate_Zc = CameraMatrix_1 * pixelCoordinate_mat;
		double Xc_Zc = CameraCoordinate_Zc.ptr<double>(0)[0];
		double Yc_Zc = CameraCoordinate_Zc.ptr<double>(1)[0];
		double Zc = LineLaserCalibrationResult[3] / (LineLaserCalibrationResult[0] * Xc_Zc + LineLaserCalibrationResult[1] * Yc_Zc + LineLaserCalibrationResult[2]);

		CameraCoordinates[i].x = Xc_Zc * Zc;
		CameraCoordinates[i].y = Yc_Zc * Zc;
		CameraCoordinates[i].z = Zc;
	}
	return CameraCoordinates;
}

void CUnitDataManagement::InitialTransMatrix(double dBaseToExternalTransMatrix[][4])
{
	dBaseToExternalTransMatrix[0][0] = 1.0; dBaseToExternalTransMatrix[0][1] = 0.0; dBaseToExternalTransMatrix[0][2] = 0.0; dBaseToExternalTransMatrix[0][3] = 0.0;
	dBaseToExternalTransMatrix[1][0] = 0.0; dBaseToExternalTransMatrix[1][1] = 1.0; dBaseToExternalTransMatrix[1][2] = 0.0; dBaseToExternalTransMatrix[1][3] = 0.0;
	dBaseToExternalTransMatrix[2][0] = 0.0; dBaseToExternalTransMatrix[2][1] = 0.0; dBaseToExternalTransMatrix[2][2] = 1.0; dBaseToExternalTransMatrix[2][3] = 0.0;
	dBaseToExternalTransMatrix[3][0] = 0.0; dBaseToExternalTransMatrix[3][1] = 0.0; dBaseToExternalTransMatrix[3][2] = 0.0; dBaseToExternalTransMatrix[3][3] = 1.0;
}

void CUnitDataManagement::GetMeasureInWorldLeaser(CvPoint2D64f dPoint, T_ANGLE_PULSE tCurrentPulse, XI_POINT& tResult)
{
	std::vector<CvPoint3D64f> v3dPoint;
	std::vector<CvPoint2D64f> PixelCoordinates;
	CvPoint2D64f d2d2 = { dPoint.x,dPoint.y };
	PixelCoordinates.push_back(d2d2);
	cv::Mat CameraMatrix = cv::Mat::zeros(3, 3, CV_64F);
	cv::Mat DistortionParameter = cv::Mat::zeros(5, 1, CV_64F);
	std::vector<double> LineLaserCalibrationResult;
	LineLaserCalibrationResult.push_back(-0.00969145);
	LineLaserCalibrationResult.push_back(0.946686);
	LineLaserCalibrationResult.push_back(0.322011);
	LineLaserCalibrationResult.push_back(473.666);
	cv::Size ImageSize;
	ImageSize.width = 3840;
	ImageSize.height = 2748;
	CameraMatrix.ptr<double>(0)[0] = 3047.5;
	CameraMatrix.ptr<double>(0)[1] = 0; 
	CameraMatrix.ptr<double>(0)[2] = 1964.33;
	CameraMatrix.ptr<double>(1)[0] = 0; 
	CameraMatrix.ptr<double>(1)[1] = 3047.18; 
	CameraMatrix.ptr<double>(1)[2] = 1336.34;
	CameraMatrix.ptr<double>(2)[0] = 0; 
	CameraMatrix.ptr<double>(2)[1] = 0;
	CameraMatrix.ptr<double>(2)[2] = 1;
	DistortionParameter.ptr<double>(0)[0] = -0.12403;
	DistortionParameter.ptr<double>(1)[0] = 0.180267; 
	DistortionParameter.ptr<double>(2)[0] = -0.00020665;
	DistortionParameter.ptr<double>(3)[0] = 0.000561353; 
	DistortionParameter.ptr<double>(4)[0] = -0.0567146;
	//PixelCoordinates:点集的像素坐标
	//CameraMatrix:相机内参矩阵(3*3)
	//DistortionParameter:相机畸变参数(5*1)
	//LineLaserCalibrationResult:激光平面方程系数
	long long tTime = XI_clock();
	v3dPoint = CoordinateTransformation(PixelCoordinates, CameraMatrix, DistortionParameter, LineLaserCalibrationResult, ImageSize);
	//XiMessageBox("CoordinateTransformation:%d", XI_clock() - tTime);
	double dMatA[4][4] = {
		1.01158,  0.000459168 , 0.0651979,  265.367,
0.000941586,  1.01561, - 0.0233692 ,- 400.775,
- 0.0672073 , 0.0256595  ,0.984147 , 426.347,
0 , 0 , 0 , 1 };
	double dMatB[4][4] = { 0 };
	double dMatC[4][4] = { 0 };
	double dMatD[4][4] = { 0 };
	double dMatE[4][4] = { 0 };
	double dMatF[4][4] = { 0 };
	double dMatG[4][4] = { 0 };
	double dMatH[4][4] = { 0 };
	T_ROBOT_COORS tCoor, tTool;
	InitialTransMatrix(dMatD);
	dMatD[0][3] = v3dPoint.at(0).x;
	dMatD[1][3] = v3dPoint.at(0).y;
	dMatD[2][3] = v3dPoint.at(0).z;

	//标定手眼，拍照位置工具0（法兰中心）到基座
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(259.401, -153.522, 347.080, 9.8087, 3.5252, 0.0042, dMatB);
	//标定手眼，拍照位置工具0（法兰中心）到基座逆矩阵
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->InverseTransMatrix(dMatB, dMatC);
	//手眼关系，相机坐标到基座
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatC, 4, 4, dMatA, 4, 4, dMatF, 4, 4);


	//当前拍照位置工具0（法兰中心）到基座
	GetRobotCtrl()->RobotKinematics(tCurrentPulse, tTool, tCoor);
//	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(259.390, 116.473, 558.861, 25.6364, 3.3833, 0.9601, dMatH);
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, dMatH);

	//手眼关系，相机到当前拍照位置的转换关系
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatH, 4, 4, dMatF, 4, 4, dMatG, 4, 4);

	//相机坐标到机器人坐标
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatG, 4, 4, dMatD, 4, 4, dMatE, 4, 4);
	tResult.x = dMatE[0][3];
	tResult.y = dMatE[1][3];
	tResult.z = dMatE[2][3];
}

T_ROBOT_COORS CUnitDataManagement::GetMeasureInWorldLeaser(int nCameraNo, CvPoint2D64f dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	long long ltime = XI_clock();
	T_CAMREA_PARAM tCameraPara = m_vtCameraPara[nCameraNo];
	std::vector<CvPoint3D64f> v3dPoint;
	std::vector<CvPoint2D64f> PixelCoordinates;
	CvPoint2D64f d2d2 = {
		dPoint.x + tCameraPara.tDHCameraDriverPara.nRoiOffsetX,
		dPoint.y + tCameraPara.tDHCameraDriverPara.nRoiOffsetY };
	PixelCoordinates.push_back(d2d2);

	cv::Size ImageSize;
	ImageSize.width = tCameraPara.tDHCameraDriverPara.nMaxWidth;
	ImageSize.height = tCameraPara.tDHCameraDriverPara.nMaxHeight;

	v3dPoint = CoordinateTransformation(
		PixelCoordinates, 
		tCameraPara.tCameraDistortion.cvCameraMatrix, 
		tCameraPara.tCameraDistortion.cvDistCoeffs, 
		tCameraPara.tLaserPlanEquation.vdLineLaserCalibrationResult, ImageSize);
//	WRITE_LOG(GetStr("1------------%d", XI_clock() - ltime));
	ltime = XI_clock();
	double dMatB[4][4] = { 0 };
	double dMatC[4][4] = { 0 };
	double dMatD[4][4] = { 0 };
	double dMatE[4][4] = { 0 };
	double dMatF[4][4] = { 0 };
	double dMatG[4][4] = { 0 };
	double dMatH[4][4] = { 0 };
	T_ROBOT_COORS tCoor, tTool;
	vector<T_ROBOT_COORS> vtCoor;
	InitialTransMatrix(dMatD);
	dMatD[0][3] = v3dPoint.at(0).x;
	dMatD[1][3] = v3dPoint.at(0).y;
	dMatD[2][3] = v3dPoint.at(0).z;

	//标定手眼，拍照位置工具0（法兰中心）到基座
	//GetRobotCtrl()->RobotKinematics(tCameraPara.tHandEyeCaliPara.tCameraReadyRobotPulse, tTool, tCoor);
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->RobotJointPostion(tCameraPara.tHandEyeCaliPara.tCameraReadyRobotPulse, 
		tCameraPara.tHandEyeCaliPara.tCameraReadyRobotTheta, tTool, vtCoor, GetRobotCtrl()->m_eManipulatorType);
	tCoor = vtCoor[tCameraPara.nInstallPos];
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, dMatB);

	//标定手眼，拍照位置工具0（法兰中心）到基座逆矩阵
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->InverseTransMatrix(dMatB, dMatC);

	//手眼关系，相机坐标到基座
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatC, 4, 4, tCameraPara.tCameraTransfomPara.dMatrix, 4, 4, dMatF, 4, 4);

	//当前拍照位置工具0（法兰中心）到基座
	//GetRobotCtrl()->RobotKinematics(tCaptureRobotPulse, tTool, tCoor);
	T_ANGLE_THETA tCaptureRobotTheta = GetRobotCtrl()->PulseToTheta(tCaptureRobotPulse);
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->RobotJointPostion(tCaptureRobotPulse,
		tCaptureRobotTheta, tTool, vtCoor, GetRobotCtrl()->m_eManipulatorType);
	tCoor = vtCoor[tCameraPara.nInstallPos];
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, dMatH);

	//手眼关系，相机到当前拍照位置的转换关系
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatH, 4, 4, dMatF, 4, 4, dMatG, 4, 4);

	//相机坐标到机器人坐标
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatG, 4, 4, dMatD, 4, 4, dMatE, 4, 4);

	T_ROBOT_COORS tResult = tCaptureRobotCoor;
	tResult.dX = dMatE[0][3];
	tResult.dY = dMatE[1][3];
	tResult.dZ = dMatE[2][3];
//	WRITE_LOG(GetStr("2------------%d", XI_clock() - ltime));
	ltime = XI_clock();
	return tResult;
}

T_ROBOT_COORS CUnitDataManagement::GetMeasureInWorldLeaser(int nCameraNo, CvPoint dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	CvPoint2D64f tPtn2D = cvPoint2D64f(dPoint.x, dPoint.y);
	return GetMeasureInWorldLeaser(nCameraNo, tPtn2D, tCaptureRobotCoor, tCaptureRobotPulse);
}

std::vector<CvPoint2D64f> CUnitDataManagement::P2P(std::vector<CvPoint> vpnt)
{
	std::vector<CvPoint2D64f> vtemp;
	for (int i = 0; i < vpnt.size(); i++)
	{
		CvPoint2D64f temp;
		temp.x = vpnt[i].x;
		temp.y = vpnt[i].y;
		vtemp.push_back(temp);
	}
	return vtemp;
}

std::vector<T_ROBOT_COORS> CUnitDataManagement::GetMeasureInWorldLeaser(int nCameraNo, std::vector<CvPoint2D64f> dPoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	T_CAMREA_PARAM tCameraPara = m_vtCameraPara[nCameraNo];
	std::vector<CvPoint3D64f> v3dPoint;

	// 恢复到最大分辨率二维点
	for (int i = 0; i < dPoint.size(); i++)
	{
		dPoint[i].x += tCameraPara.tDHCameraDriverPara.nRoiOffsetX;
		dPoint[i].y += tCameraPara.tDHCameraDriverPara.nRoiOffsetY;
	}

	cv::Size ImageSize;
	ImageSize.width = tCameraPara.tDHCameraDriverPara.nMaxWidth;
	ImageSize.height = tCameraPara.tDHCameraDriverPara.nMaxHeight;

	v3dPoint = CoordinateTransformation(
		dPoint,
		tCameraPara.tCameraDistortion.cvCameraMatrix,
		tCameraPara.tCameraDistortion.cvDistCoeffs,
		tCameraPara.tLaserPlanEquation.vdLineLaserCalibrationResult, ImageSize);

	double dMatB[4][4] = { 0 };
	double dMatC[4][4] = { 0 };
	double dMatD[4][4] = { 0 };
	double dMatE[4][4] = { 0 };
	double dMatF[4][4] = { 0 };
	double dMatG[4][4] = { 0 };
	double dMatH[4][4] = { 0 };
	T_ROBOT_COORS tCoor, tTool;
	vector<T_ROBOT_COORS> vtCoor;

	if (-1 != tCameraPara.nInstallPos)
	{
		//标定手眼，拍照位置工具0（法兰中心）到基座
	//GetRobotCtrl()->RobotKinematics(tCameraPara.tHandEyeCaliPara.tCameraReadyRobotPulse, tTool, tCoor);
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->RobotJointPostion(tCameraPara.tHandEyeCaliPara.tCameraReadyRobotPulse,
			tCameraPara.tHandEyeCaliPara.tCameraReadyRobotTheta, tTool, vtCoor, GetRobotCtrl()->m_eManipulatorType);
		tCoor = vtCoor[tCameraPara.nInstallPos];
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, dMatB);

		//标定手眼，拍照位置工具0（法兰中心）到基座逆矩阵
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->InverseTransMatrix(dMatB, dMatC);

		//手眼关系，相机坐标到基座
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatC, 4, 4, tCameraPara.tCameraTransfomPara.dMatrix, 4, 4, dMatF, 4, 4);

		//当前拍照位置工具0（法兰中心）到基座
		//GetRobotCtrl()->RobotKinematics(tCaptureRobotPulse, tTool, tCoor);
		T_ANGLE_THETA tCaptureRobotTheta = GetRobotCtrl()->PulseToTheta(tCaptureRobotPulse);
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->RobotJointPostion(tCaptureRobotPulse,
			tCaptureRobotTheta, tTool, vtCoor, GetRobotCtrl()->m_eManipulatorType);
		tCoor = vtCoor[tCameraPara.nInstallPos];
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tCoor.dX, tCoor.dY, tCoor.dZ, tCoor.dRX, tCoor.dRY, tCoor.dRZ, dMatH);

		//手眼关系，相机到当前拍照位置的转换关系
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatH, 4, 4, dMatF, 4, 4, dMatG, 4, 4);
	}
	else
	{
		// 相机安装在机械臂以外
		for (size_t i = 0; i < 4; i++)
		{
			for (size_t j = 0; j < 4; j++)
			{
				dMatG[i][j] = tCameraPara.tCameraTransfomPara.dMatrix[i][j];
			}
		}
	}
	
	std::vector<T_ROBOT_COORS> vtResult;
	for (size_t i = 0; i < v3dPoint.size(); i++)
	{
		//相机坐标到机器人坐标
		InitialTransMatrix(dMatD);
		dMatD[0][3] = v3dPoint.at(i).x;
		dMatD[1][3] = v3dPoint.at(i).y;
		dMatD[2][3] = v3dPoint.at(i).z;
		GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatG, 4, 4, dMatD, 4, 4, dMatE, 4, 4);
		T_ROBOT_COORS tResult = tCaptureRobotCoor;
		tResult.dX = dMatE[0][3];
		tResult.dY = dMatE[1][3];
		tResult.dZ = dMatE[2][3];
		vtResult.push_back(tResult);
	}

	return vtResult;
}

T_ROBOT_COORS CUnitDataManagement::TranCameraToBase(int nCameraNo, T_ANGLE_PULSE tPulse, T_ROBOT_COORS tCoor, T_ROBOT_COORS tCameraCoor)
{
	T_ROBOT_COORS tRobotCoor;
	if (m_vtCameraPara.size() <= nCameraNo)
	{
		return tRobotCoor;
	}
	T_ROBOT_COORS tTool;
	tTool.dX = 0;
	tTool.dY = 0;
	tTool.dZ = 0;
	tTool.dRX = 0;
	tTool.dRY = 0;
	tTool.dRZ = 0;
	T_ROBOT_COORS tTemp = tCoor;
	GetRobotCtrl()->RobotKinematics(tPulse, tTool, tTemp, 0);
	double dMatB[4][4] = { 0 };
	double dMatC[4][4] = { 0 };
	double dMatD[4][4] = { 0 };
	double dMatE[4][4] = { 0 };
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tTemp.dX, tTemp.dY, tTemp.dZ, tTemp.dRX, tTemp.dRY, tTemp.dRZ, dMatB);
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatB, 4, 4, m_vtCameraPara[nCameraNo].tCameraTransfomPara.dMatrix, 4, 4, dMatC, 4, 4);
	dMatD[0][3] = tCameraCoor.dX * 1000.0;
	dMatD[1][3] = tCameraCoor.dY * 1000.0;
	dMatD[2][3] = tCameraCoor.dZ * 1000.0;
	dMatD[3][3] = 1;
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dMatC, 4, 4, dMatD, 4, 4, dMatE, 4, 4);
	tRobotCoor = tCoor;
	tRobotCoor.dX = dMatE[0][3];
	tRobotCoor.dY = dMatE[1][3];
	tRobotCoor.dZ = dMatE[2][3];
	return tRobotCoor;
}

T_ROBOT_COORS CUnitDataManagement::TranImageToBase(int nCameraNo, CvPoint tImagePoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse, T_ABS_POS_IN_BASE* tPointAbsCoordInBase)
{
	T_CAMREA_PARAM tCameraPara = m_vtCameraPara[nCameraNo];
	tImagePoint.x += tCameraPara.tDHCameraDriverPara.nRoiOffsetX;
	tImagePoint.y += tCameraPara.tDHCameraDriverPara.nRoiOffsetY;
	T_ROBOT_COORS tRetCoors = CameraCoorToRobotCoor(nCameraNo, CameraCoor2DTo3D(nCameraNo, tImagePoint), tCaptureRobotCoor, tCaptureRobotPulse);
	if (tPointAbsCoordInBase != NULL) {
		tPointAbsCoordInBase->tWeldGunPos.x = tCaptureRobotCoor.dX;
		tPointAbsCoordInBase->tWeldGunPos.y = tCaptureRobotCoor.dY;
		tPointAbsCoordInBase->tWeldGunPos.z = tCaptureRobotCoor.dZ;
		tPointAbsCoordInBase->tWeldLinePos.x = tRetCoors.dX;
		tPointAbsCoordInBase->tWeldLinePos.y = tRetCoors.dY;
		tPointAbsCoordInBase->tWeldLinePos.z = tRetCoors.dZ;
	}
	return tRetCoors;
}

std::vector<T_ROBOT_COORS> CUnitDataManagement::TranImageToBase(int nCameraNo, const std::vector<CvPoint> &vtImagePoint, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	std::vector<T_ROBOT_COORS> vtRetCoors(vtImagePoint.size());
	T_CAMREA_PARAM tCameraPara = m_vtCameraPara[nCameraNo];
	CvPoint tImagePoint;
	for (int i = 0; i < vtImagePoint.size(); i++)
	{
		tImagePoint = vtImagePoint[i];
		tImagePoint.x += tCameraPara.tDHCameraDriverPara.nRoiOffsetX;
		tImagePoint.y += tCameraPara.tDHCameraDriverPara.nRoiOffsetY;
		vtRetCoors[i] = CameraCoorToRobotCoor(nCameraNo, CameraCoor2DTo3D(nCameraNo, tImagePoint), tCaptureRobotCoor, tCaptureRobotPulse);
	}
	return vtRetCoors;
}

XI_POINT CUnitDataManagement::CameraCoor2DTo3D(int nCameraNo, CvPoint tImagePoint)
{
	if (m_vtCameraPara.size() <= nCameraNo)
	{
		XI_POINT point;
		point.x = 0.0;
		point.y = 0.0;
		point.z = 0.0;
		return point;
	}
	if (m_mpXiLaserCvObj[nCameraNo] == NULL)
	{
		m_mpXiLaserCvObj[nCameraNo] = new XiCV_Image_Processing(m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nMaxWidth, m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nMaxHeight);
	}
	
	XiLaserLightParamNode tLaserLightParam;

	XI_POINT tKeyPoint3D;
	XI_POINT tCamCenter3D;
	
	tLaserLightParam.l_dx = m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelX;
	tLaserLightParam.l_dy = m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraInnerPara.dPixelY;
	tLaserLightParam.l_f = m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraInnerPara.dFocal;
	tLaserLightParam.l_len = m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraInnerPara.dBaseLineLength;
	tLaserLightParam.l_ctan = m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraInnerPara.dCtanBaseLineAngle;

	CvPoint3D64f cptKeyPnt3D = m_mpXiLaserCvObj[nCameraNo]->KeyPoint2DTo3D(tImagePoint, 1, tLaserLightParam);
	tKeyPoint3D.x = cptKeyPnt3D.x;
	tKeyPoint3D.y = cptKeyPnt3D.y;
	tKeyPoint3D.z = cptKeyPnt3D.z;


	CvPoint      tCamCentrePnt = cvPoint(m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nMaxWidth / 2, m_vtCameraPara[nCameraNo].tDHCameraDriverPara.nMaxHeight / 2);
	CvPoint3D64f cpCamCenterPnt3D = m_mpXiLaserCvObj[nCameraNo]->KeyPoint2DTo3D(tCamCentrePnt, 1, tLaserLightParam);
	tCamCenter3D.x = cpCamCenterPnt3D.x;
	tCamCenter3D.y = cpCamCenterPnt3D.y;
	tCamCenter3D.z = cpCamCenterPnt3D.z;
	XI_POINT tCornerPoint;

	tCornerPoint.x = (tKeyPoint3D.x - tCamCenter3D.x) * (1.0);
	tCornerPoint.y = (tKeyPoint3D.y - tCamCenter3D.y) * (1.0);
	tCornerPoint.z = (tKeyPoint3D.z - tCamCenter3D.z) * (1.0);
	return tCornerPoint;
}

T_ROBOT_COORS CUnitDataManagement::CameraCoorToRobotCoor(int nCameraNo, XI_POINT tCameraCoor, T_ROBOT_COORS tCaptureRobotCoor, T_ANGLE_PULSE tCaptureRobotPulse)
{
	T_ABS_POS_IN_BASE tAbsCoor = GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->MeasureWeldLinePointInBase(tCameraCoor,
		tCaptureRobotCoor, tCaptureRobotPulse, GetRobotCtrl()->m_eManipulatorType, m_vtCameraPara[nCameraNo].tHandEyeCaliPara);
	tCaptureRobotCoor.dX = tAbsCoor.tWeldLinePos.x;
	tCaptureRobotCoor.dY = tAbsCoor.tWeldLinePos.y;
	tCaptureRobotCoor.dZ = tAbsCoor.tWeldLinePos.z;
	return tCaptureRobotCoor;
}
T_ROBOT_COORS CUnitDataManagement::GetCameraTool(int nCameraNo)
{
	T_ROBOT_COORS tCameraTool;
	double dMatrixCameraToWaist[4][4];
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateCameraToWaistMatrix(
		m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraReadyRobotCoors,
		m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraReadyRobotPulse,
		m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dX - m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraReadyRobotCoors.dX,
		m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dY - m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraReadyRobotCoors.dY,
		m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tGunMoveToCameraCenterRobotCoors.dZ - m_vtCameraPara[nCameraNo].tHandEyeCaliPara.tCameraReadyRobotCoors.dZ,
		dMatrixCameraToWaist, GetRobotCtrl()->m_eManipulatorType, m_vtCameraPara[nCameraNo].tHandEyeCaliPara);
	tCameraTool.dX = dMatrixCameraToWaist[0][3];
	tCameraTool.dY = dMatrixCameraToWaist[1][3];
	tCameraTool.dZ = dMatrixCameraToWaist[2][3];
	tCameraTool.dRX = 0.0;
	tCameraTool.dRY = 0.0;
	tCameraTool.dRZ = 0.0;
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GetRotateDecompose(dMatrixCameraToWaist, tCameraTool.dRX, tCameraTool.dRY, tCameraTool.dRZ);
	return tCameraTool;
}

T_ROBOT_COORS CUnitDataManagement::GetMechMindCameraTool(int nCameraNo)
{
	T_ROBOT_COORS tCameraTool;
	tCameraTool.dX = m_vtCameraPara[nCameraNo].tCameraTransfomPara.dMatrix[0][3];
	tCameraTool.dY = m_vtCameraPara[nCameraNo].tCameraTransfomPara.dMatrix[1][3];
	tCameraTool.dZ = m_vtCameraPara[nCameraNo].tCameraTransfomPara.dMatrix[2][3];
	tCameraTool.dRX = 0.0;
	tCameraTool.dRY = 0.0;
	tCameraTool.dRZ = 0.0;
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GetRotateDecompose(
		m_vtCameraPara[nCameraNo].tCameraTransfomPara.dMatrix, tCameraTool.dRX, tCameraTool.dRY, tCameraTool.dRZ);
	return tCameraTool;
}

void CUnitDataManagement::TransBasePointToWorldPoint(T_ROBOT_COORS tBaseRobotCoors, double dBaseToExternalTransMatrix[][4], T_ROBOT_COORS_IN_WORLD& tRobotCoorsInWorld)
{
	double dToolToBaseMatrix[4][4];
	double dPointInExternalMatrix[4][4];
	int nRows = 4;
	int nCols = 4;

	T_ROBOT_COORS tExternalRobotCoors;
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GenerateToolToBaseMatrix(tBaseRobotCoors.dX, tBaseRobotCoors.dY, tBaseRobotCoors.dZ, tBaseRobotCoors.dRX, tBaseRobotCoors.dRY, tBaseRobotCoors.dRZ, dToolToBaseMatrix);
	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->Trmul(dBaseToExternalTransMatrix, nRows, nCols, dToolToBaseMatrix, nRows, nCols, dPointInExternalMatrix, nRows, nCols);

	tExternalRobotCoors.dX = dPointInExternalMatrix[0][3];
	tExternalRobotCoors.dY = dPointInExternalMatrix[1][3];
	tExternalRobotCoors.dZ = dPointInExternalMatrix[2][3];

	GetRobotCtrl()->m_cXiRobotAbsCoorsTrans->GetRotateDecompose(dPointInExternalMatrix, tExternalRobotCoors.dRX, tExternalRobotCoors.dRY, tExternalRobotCoors.dRZ);

	tRobotCoorsInWorld.dX = tExternalRobotCoors.dX + tBaseRobotCoors.dBX;
	tRobotCoorsInWorld.dY = tExternalRobotCoors.dY + tBaseRobotCoors.dBY;
	tRobotCoorsInWorld.dZ = tExternalRobotCoors.dZ + tBaseRobotCoors.dBZ;
	tRobotCoorsInWorld.dRX = tExternalRobotCoors.dRX;
	tRobotCoorsInWorld.dRY = tExternalRobotCoors.dRY;
	tRobotCoorsInWorld.dRZ = tExternalRobotCoors.dRZ;
}

void CUnitDataManagement::CaliCutRobotAndPolishRobotTransMatrixForTwoRobots(T_ROBOT_COORS tFirstPointInCutRobot, T_ROBOT_COORS tSecondPointInCutRobot, T_ROBOT_COORS tThirdPointInCutRobot,
	T_ROBOT_COORS tFirstPointInPolishRobot, T_ROBOT_COORS tSecondPointInPolishRobot, T_ROBOT_COORS tThirdPointInPolishRobot,
	double dMatrixCutRobotToPolishRobot[][4], double dMatrixPolishRobotToCutRobot[][4])
{
	T_ROBOT_COORS_IN_WORLD tFirstPointInCutRobotInWorld, tSecondPointInCutRobotInWorld, tThirdPointInCutRobotInWorld;
	T_ROBOT_COORS_IN_WORLD tFirstPointInPolishRobotInWorld, tSecondPointInPolishRobotInWorld, tThirdPointInPolishRobotInWorld;

	TransBasePointToWorldPoint(tFirstPointInCutRobot, m_dExternalToBaseTransMatrix, tFirstPointInCutRobotInWorld);
	TransBasePointToWorldPoint(tSecondPointInCutRobot, m_dExternalToBaseTransMatrix, tSecondPointInCutRobotInWorld);
	TransBasePointToWorldPoint(tThirdPointInCutRobot, m_dExternalToBaseTransMatrix, tThirdPointInCutRobotInWorld);

	TransBasePointToWorldPoint(tFirstPointInPolishRobot, m_dExternalToBaseTransMatrix, tFirstPointInPolishRobotInWorld);
	TransBasePointToWorldPoint(tSecondPointInPolishRobot, m_dExternalToBaseTransMatrix, tSecondPointInPolishRobotInWorld);
	TransBasePointToWorldPoint(tThirdPointInPolishRobot, m_dExternalToBaseTransMatrix, tThirdPointInPolishRobotInWorld);

	std::vector<T_POINT_3D> vtPolishPoints, vtCutPoints;
	vtPolishPoints.clear();
	vtCutPoints.clear();
	T_POINT_3D tFirstPolishPoint, tSecondPolishPoint, tThirdPolishPoint;
	T_POINT_3D tFirstCutPoint, tSecondCutPoint, tThirdCutPoint;

	tFirstPolishPoint.x = tFirstPointInPolishRobotInWorld.dX;
	tFirstPolishPoint.y = tFirstPointInPolishRobotInWorld.dY;
	tFirstPolishPoint.z = tFirstPointInPolishRobotInWorld.dZ;
	tSecondPolishPoint.x = tSecondPointInPolishRobotInWorld.dX;
	tSecondPolishPoint.y = tSecondPointInPolishRobotInWorld.dY;
	tSecondPolishPoint.z = tSecondPointInPolishRobotInWorld.dZ;
	tThirdPolishPoint.x = tThirdPointInPolishRobotInWorld.dX;
	tThirdPolishPoint.y = tThirdPointInPolishRobotInWorld.dY;
	tThirdPolishPoint.z = tThirdPointInPolishRobotInWorld.dZ;
	vtPolishPoints.push_back(tFirstPolishPoint);
	vtPolishPoints.push_back(tSecondPolishPoint);
	vtPolishPoints.push_back(tThirdPolishPoint);

	tFirstCutPoint.x = tFirstPointInCutRobotInWorld.dX;
	tFirstCutPoint.y = tFirstPointInCutRobotInWorld.dY;
	tFirstCutPoint.z = tFirstPointInCutRobotInWorld.dZ;
	tSecondCutPoint.x = tSecondPointInCutRobotInWorld.dX;
	tSecondCutPoint.y = tSecondPointInCutRobotInWorld.dY;
	tSecondCutPoint.z = tSecondPointInCutRobotInWorld.dZ;
	tThirdCutPoint.x = tThirdPointInCutRobotInWorld.dX;
	tThirdCutPoint.y = tThirdPointInCutRobotInWorld.dY;
	tThirdCutPoint.z = tThirdPointInCutRobotInWorld.dZ;
	vtCutPoints.push_back(tFirstCutPoint);
	vtCutPoints.push_back(tSecondCutPoint);
	vtCutPoints.push_back(tThirdCutPoint);

	CAnalyzeDxf cAnalyzeDxf;
	cAnalyzeDxf.GetTransMatrixByMatchPoints(vtCutPoints, vtPolishPoints, dMatrixCutRobotToPolishRobot);
	cAnalyzeDxf.GetTransMatrixByMatchPoints(vtPolishPoints, vtCutPoints, dMatrixPolishRobotToCutRobot);
}

void CUnitDataManagement::InitialTransMatrix(double dBaseToExternalTransMatrix[][4], double dExternalToBaseTransMatrix[][4])
{
	dBaseToExternalTransMatrix[0][0] = 1.0; dBaseToExternalTransMatrix[0][1] = 0.0; dBaseToExternalTransMatrix[0][2] = 0.0; dBaseToExternalTransMatrix[0][3] = 0.0;
	dBaseToExternalTransMatrix[1][0] = 0.0; dBaseToExternalTransMatrix[1][1] = 1.0; dBaseToExternalTransMatrix[1][2] = 0.0; dBaseToExternalTransMatrix[1][3] = 0.0;
	dBaseToExternalTransMatrix[2][0] = 0.0; dBaseToExternalTransMatrix[2][1] = 0.0; dBaseToExternalTransMatrix[2][2] = 1.0; dBaseToExternalTransMatrix[2][3] = 0.0;
	dBaseToExternalTransMatrix[3][0] = 0.0; dBaseToExternalTransMatrix[3][1] = 0.0; dBaseToExternalTransMatrix[3][2] = 0.0; dBaseToExternalTransMatrix[3][3] = 1.0;

	dExternalToBaseTransMatrix[0][0] = 1.0; dExternalToBaseTransMatrix[0][1] = 0.0; dExternalToBaseTransMatrix[0][2] = 0.0; dExternalToBaseTransMatrix[0][3] = 0.0;
	dExternalToBaseTransMatrix[1][0] = 0.0; dExternalToBaseTransMatrix[1][1] = 1.0; dExternalToBaseTransMatrix[1][2] = 0.0; dExternalToBaseTransMatrix[1][3] = 0.0;
	dExternalToBaseTransMatrix[2][0] = 0.0; dExternalToBaseTransMatrix[2][1] = 0.0; dExternalToBaseTransMatrix[2][2] = 1.0; dExternalToBaseTransMatrix[2][3] = 0.0;
	dExternalToBaseTransMatrix[3][0] = 0.0; dExternalToBaseTransMatrix[3][1] = 0.0; dExternalToBaseTransMatrix[3][2] = 0.0; dExternalToBaseTransMatrix[3][3] = 1.0;
}

void CUnitDataManagement::CoorTransform(std::vector<T_POINT_3D>& vtPoint, T_POINT_3D tCenter, double dRotAngle)
{
	int nIdx = 0;
	for (nIdx = 0; nIdx < vtPoint.size(); nIdx++)
	{
		vtPoint[nIdx].x += tCenter.x;
		vtPoint[nIdx].y += tCenter.y;
		vtPoint[nIdx].z += tCenter.z;
	}

	// Rotation
	for (nIdx = 0; nIdx < vtPoint.size(); nIdx++)
	{
		T_POINT_3D tPoint;
		tPoint.x = (vtPoint[nIdx].x * cos(dRotAngle * PI / 180.0)) -
			(vtPoint[nIdx].y * sin(dRotAngle * PI / 180.0));
		tPoint.y = (vtPoint[nIdx].x * sin(dRotAngle * PI / 180.0)) +
			(vtPoint[nIdx].y * cos(dRotAngle * PI / 180.0));
		tPoint.z = vtPoint[nIdx].z;

		vtPoint[nIdx] = tPoint;
	}
}
