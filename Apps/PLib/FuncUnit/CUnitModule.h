#pragma once

#include ".\Apps\PLib\BasicFunc\HeaderFile.h"
#include ".\Apps\PLib\CtrlUnit\CUnit_Debug.h"
//#include "LineScan.h"

class CUnitModule :
#if ENABLE_UNIT_DEBUG
	public CUnit_Debug
#else
	public CUnit
#endif // ENABLE_UNIT_DEBUG
{
public:
	CUnitModule(T_CONTRAL_UNIT tCtrlUnitInfo, CServoMotorDriver* pServoMotorDriver = NULL);
	~CUnitModule();

	CString CUnitModule::GetNode();
	bool m_bCanRotate = true;//�ж����������ܷ���ת���ϻ���ɨ
	/***************************************** �����㷨 ********************************************/
	double m_dExternalToBaseTransMatrix[4][4];
	double m_dBaseToExternalTransMatrix[4][4];
	void InitialTransMatrix(double dBaseToExternalTransMatrix[][4], double dExternalToBaseTransMatrix[][4]);
	void CoorTransform(std::vector<T_POINT_3D>& vtPoint, T_POINT_3D tCenter, double dRotAngle);

	/************************************************************************/
	/************************************************************************/
	/*************************		  ��ɨ���  	  ***********************/
	/************************************************************************/
	/************************************************************************/

	/// <summary>
	/// ����ɨ��ȡ����
	/// </summary>
	/// <param name="strTableName">����̨��</param>
	/// <returns></returns>
	bool LaserScanGetPointCouid(CString strTableName, CString &strPointCouldPath, std::vector<CvPoint3D64f>& vpntPointCould);
	bool LaserScanGetPointCouid(T_ROBOT_COORS tDiffCoor, CString strTableName, CString strTemplateNo, CString strPath, CString& strPointCouldPath, std::vector<CvPoint3D64f>& vpntPointCould, T_ROBOT_MOVE_SPEED tPulseMove = { 1000,100,100 });

	/*
	* LaserScanRuning
	* ��ɨ
	* param		strTableName:����̨��
	* return	true:�ɹ�	false:ʧ��
	*/
	bool LaserScanRuning(CString strTableName, int nCurRecognitionTableNo);

	//����ʶ����
	std::vector<T_RECOGNITION_PARA> m_vtRecognitionPara;
	std::vector<vector<T_RECOGNITION_PARA>> m_vvtRecognitionParaA;//A�����˵��߸�����������ɨ�����ʣ����ɨ�����
	bool LoadMatchInfo(std::vector<T_RECOGNITION_PARA>& vtMatchInfo, CString sRealResultFileName = ".\\panoRecognition\\������\\MatchInfo.txt");
	bool SaveMatchInfo(std::vector<T_RECOGNITION_PARA> vtMatchInfo, CString sRealResultFileName = ".\\panoRecognition\\������\\MatchInfo.txt");
	//��������ץȡλ��
	std::vector < T_GRASPANDPLACE_INFO> m_vtGraspAndPlaceInfo;
	bool LoadGraspAndPlaceInfo(std::vector<T_GRASPANDPLACE_INFO>& vtGraspAndPlaceInfo, CString sRealResultFileName = ".\\GraphData\\GraspAndPlaceInfo.txt");
	//������������λ��
	std::vector <T_Template_PARA> m_vtTheoryGraphCenter;
	std::vector<T_Template_PARA> LoadTheoryGraphCenter(CString sIdealResultFileName = ".\\panoRecognition\\������\\Ideal.txt");
	//��ȡģ���Ӧ����
	int GetPartNoFromTemplate(T_RECOGNITION_PARA& tRecognitionPara, int nTemplateNo, std::vector<T_RECOGNITION_PARA> vtRecognitionPara);
	//ɾ����Ӧ����
	bool DeletePartNoFromTemplate(T_RECOGNITION_PARA tRecognitionPara, std::vector<T_RECOGNITION_PARA> &vtRecognitionPara);
	
	//����ץȡλ��
	T_RUNNING_RTN_RESULT CalcGrabPos(int nPartNo, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tRobotTool, T_ROBOT_COORS& tGrabPos);
	T_RUNNING_RTN_RESULT CalcGrabPos(T_RECOGNITION_PARA tRecognitionPara, T_ANGLE_PULSE tReferencePulse, T_ROBOT_COORS tRobotTool, T_ROBOT_COORS& tGrabPos);

	/************************************************************************/
	/************************************************************************/
	/***************************		  �˶�  	  ***********************/
	/************************************************************************/
	/************************************************************************/


	//�˶�
	T_RUNNING_RTN_RESULT SetTrack(std::vector<T_ROBOT_MOVE_INFO>& tRobotMoveTrack, int nContinueStep = 0);
	T_RUNNING_RTN_RESULT SetTrack(CRobotMove cMoveInfo, int nContinueStep = 0);
	T_RUNNING_RTN_RESULT StartMove();
	T_RUNNING_RTN_RESULT CheckMoveTrack(T_ROBOT_COORS tRobotCoor);
	T_RUNNING_RTN_RESULT CheckMoveTrack(T_ANGLE_PULSE tRobotPulse);

	//�����˶�
	T_RUNNING_RTN_RESULT RobotPosMove(T_ROBOT_COORS tRobotCoor, T_ROBOT_MOVE_SPEED tRobotSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, bool bWait = true);

	T_RUNNING_RTN_RESULT RobotPosMove(T_ANGLE_PULSE tRobotPulse, T_ROBOT_MOVE_SPEED tRobotSpeed, int nMoveType = MOVJ, UINT unToolNum = 1, bool bWait = true);

	bool CheckSafePos();


};

