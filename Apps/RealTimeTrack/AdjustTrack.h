#pragma once
#include <atomic>
#include "Apps/PartData/WeldTrack.h"
#include "Apps/PLib/YaskawaRobot/RobotDriverAdaptor.h"
#include "LocalFiles/ExLib/Vision/include/GFPGJointLib.h"
#include "Apps/Welding/PolygonCollision.h"

class CUnit;

namespace RTT
{
	//ʵʱ���٣��켣����
	class AdjustTrack
	{
	public:
		AdjustTrack();
		~AdjustTrack();

		bool loadPara();

		/// @brief ���ÿ��Ƶ�Ԫ����ָ��
		/// @param cUnit ���Ƶ�Ԫ����ָ��
		void setUnit(CUnit& cUnit);

		/// @brief ���ó�ʼ���ӹ켣
		/// @param cWeldTrack ��ʼ���ӹ켣
		void initTrack(PartData::WeldTrack& cWeldTrack);

		/// @brief ���ٿ�ʼ֮ǰ��ʼ������
		/// @return �ɹ�/ʧ��
		bool initBeforeTrack();

		/// @brief �ж��Ƿ���Ҫ����ͼƬ
		/// @param tImgCoord �ɼ�ͼƬʱ������
		/// @return ��/��
		bool determineWhetherToProcessImg(T_ROBOT_COORS tImgCoord, CvPoint& tIdeal2DKeyPoint, CvPoint& tHorTrackingPointInImage, CvPoint& tVerTrackingPointInImage);

		/// @brief �����켣
		/// @param tImgCoord �ɼ�ͼƬʱ������
		/// @param tSeenCoord ���⽻�����ά����
		/// @return �ɹ�/ʧ��
		bool adjustTrack(T_ROBOT_COORS tImgCoord, T_ROBOT_COORS tSeenCoord);

		/// @brief �������һ�η��͵ĵ�����
		/// @param tLastSendPnt ���һ�η��͵ĵ�����
		/// @param nPntNo ���һ�η��͵ĵ����
		void setLastSendPnt(T_ROBOT_COORS tLastSendPnt, int nPntNo);

		/// @brief ��ȡ���һ�η��͵ĵ�����
		/// @return ���һ�η��͵ĵ�����
		T_POINT_3D getLastSendPnt();

		/// @brief ����������������켣��ķ����
		/// @param tSeenCoord �����
		/// @return �����
		double calcNearestPointNormalDegree(T_ROBOT_COORS tSeenCoord);

		/// @brief �Ƿ�֧�ָ����յ�
		/// @return ��/��
		bool supportUpdatingEndPntOrNot();

		/// @brief �����յ�
		/// @param tEndPnt ���յ�
		/// @return �ɹ�/ʧ��
		bool updateEndPnt(T_ROBOT_COORS tEndPnt);

		/// @brief ��ȡ���ݱ����ļ��ĸ�Ŀ¼
		/// @return ���ݱ����ļ��ĸ�Ŀ¼
		CString getDataFileRootFolder() const;

		//
		//
		bool AdjustMeasureCoors(CUnit& cUnit, std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints, std::vector<T_LINE_DIR_3D> vtWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, std::vector<T_ROBOT_COORS_IN_WORLD>& vtAdjustMeasureRobotPoints);


	private:
		/// @brief д��־
		/// @param format ��ʽ���ַ���
		/// @param  ����ʽ������
		void writeLog(const char* format, ...) const;

		/// @brief ����켣��ʵ��
		/// @return �ɹ�/ʧ��
		bool calcTrackEntity();

		/// @brief ʹ�켣ʵ��պϲ�����պ�ʵ�����ɢ��
		/// @return �ɹ�/ʧ��
		bool makeEntitiesClose();

		/// @brief ȷ��������㲻��
		/// @return �ɹ�/ʧ��
		bool makesureStratPnt();

		/// @brief ���رպ�ʵ��
		/// @return �ɹ�/ʧ��
		bool loadClosedEntities();

		/// @brief ����պ�ʵ���Zֵ
		/// @param vtOldCoord ����������ϵ�¾ɹ켣
		/// @param vtOld3DPnts ��������ϵ�¾ɹ켣
		/// @return �ɹ�/ʧ��
		bool calcClosedEntitiesZ(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts);

		/// @brief �ı�Բ�����丽���켣��RZ
		/// @param vnPntCount ÿ��ʵ��Ĺ켣��Ŀ
		/// @param vtInitWeldCoord ��������Ĺ켣
		/// @return �ɹ�/ʧ��
		bool changeRZForArc(const std::vector<int>& vnPntCount, std::vector<T_ROBOT_COORS>& vtInitWeldCoord);

		/// @brief ���رպϹ켣
		/// @param vtOldCoord ����������ϵ�¾ɹ켣
		/// @param vtOld3DPnts ��������ϵ�¾ɹ켣
		/// @return �ɹ�/ʧ��
		bool loadClosedTrack(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts);

		/// @brief �����޸ĺ�Ĺ켣���ļ���
		/// @param vtNewPoints �޸ĺ�Ĺ켣
		void saveAdjustTrack(const std::vector<T_POINT_3D>& vtNewPoints);

		/// @brief �жϹ켣�Ƿ�ȫ�ɴ�
		/// @param tStartPulse �������
		/// @param vtTrack �켣����
		/// @return ��/��
		bool isSafeTrack(T_ANGLE_PULSE tStartPulse, const std::vector<T_ROBOT_COORS>& vtTrack);

		double m_dMaxAdjustDisXY = 20.0;//XY��������
		double m_dMaxAdjustDisZ = 15.0;//Z��������
		CUnit* m_pUnit = nullptr;						//���Ƶ�Ԫָ��
		CRobotDriverAdaptor* m_pRobot = nullptr;		//�����˿���ָ��
		PartData::WeldTrack* m_pWeldTrack = nullptr;	//���ӹ켣
		bool m_bOnlyLineEntity = false;

		/**********************************************���¿���ֲ�Ĵ���**********************************************/
	public:
		/// @brief �켣��Ϣ
		struct T_DOTRACKING_PARA
		{
			bool bIfDoTracking = false;						//�Ƿ�ɸ��٣��Ƿ��ܿ����ǵ��˵�
			int nEntityNo = -1;								//������ʵ���
			int nEntityType = 0;							//������ʵ�����ͣ�1ֱ�� 2Բ��
			T_POINT_3D tIntersectionPoint;					//��������
			std::vector<T_ALGORITHM_POINT> vtFilterInput;	//�˲������
		};

		typedef enum
		{
			E_BEVEL_UP_TRACKING,
			E_BEVEL_DOWN_TRACKING
		}E_DOTRACKING_BEVEL_TYPE;

		typedef enum
		{
			E_OUTTERCONTOUR_TRACKING,
			E_INNERCONTOUR_TRACKING
		}E_DOTRACKING_CONTOUR_TYPE;

		typedef struct
		{
			XI_POINT tStartPoint;
			XI_POINT tEndPoint;
			XI_POINT tCenterPoint;
			T_XIGEO_LINE_DIR tVerDir;
			double dRadius;
		}T_XIGEO_ARC_PARA;

		typedef struct
		{
			double a;
			double b;
			double c;
			double d;
		}T_XIGEO_PLANE_PARAMTYPE;

		static bool PlaneContourInfo2T_DXF_ENTITY_PARA(const PlaneContourInfo& info, int nContourNo, T_DXF_ENTITY_PARA& tEntities);

	private:
		T_XIGEO_PLANE_PARA calcContourPlanePara();

		//��ȡ����������
		bool GetTrackingInciseData(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType, VT_GRAPH vtContourEntities, std::vector<T_POINT_3D> vtIncisePoints, std::vector<int> vnIncisePointEntityNo, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnBevelPointEntityNo,
			std::vector<vector<T_POINT_3D>>& vvtInciseEntityWithBevelPoints, std::vector<int>& vnInciseEntityWithBevelNo, std::vector<vector<T_LINE_DIR>>& vvtInciseEntityPointAdjustDir, std::vector<double>& vdInciseToBevelDis,
			std::vector<vector<T_POINT_3D>>& vvtBevelEntityPoints, std::vector<int>& vnBevelEntityNo);

		void CalAdjustDirAndDisForTracking(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType,
			T_DXF_ENTITY_PARA tEntity, std::vector<T_POINT_3D> vtIncisePoints, std::vector<T_POINT_3D> vtBevelPoints, std::vector<T_LINE_DIR>& vtAdjustDir, double& dAdjustDis);

		// ���٣������Ƿ���Ը��٣���ÿ��λ�ÿ�������ʲôʵ��
		void CalcIfDoTracking(T_XIGEO_PLANE_PARA tContourPlanePara, VT_GRAPH vtContourEntities, std::vector<int> vnBevelEntityNo, std::vector<T_DOTRACKING_PARA>& vtDoTrackPara);

		//void GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage);

		bool GetEntityAndPlaneInterseciton(T_DXF_ENTITY_PARA tEntity, T_XIGEO_PLANE_PARA tEntityPlanePara, T_POINT_3D tBasePoint, T_XIGEO_PLANE_PARA tPlanePara, T_POINT_3D& tIntersectionPoint);
		
		T_XIGEO_LINE_PARA GetXiLinePara(double dStartX, double dStartY, double dStartZ, double dEndX, double dEndY, double dEndZ);
		
		T_XIGEO_ARC_PARA GetXiArcPara(double dStartPointX, double dStartPointY, double dStartPointZ, double dEndPointX, double dEndPointY, double dEndPointZ, double dCenterX, double dCenterY, double dCenterZ, double dDirX, double dDirY, double dDirZ);

		BOOL LinePlateInterSection(T_XIGEO_LINE_PARA tLinePara, T_XIGEO_PLANE_PARA tPlanePara, T_POINT_3D& tIntersectionPoint);

		BOOL ArcPlateInterSection(T_XIGEO_ARC_PARA tArcPara, T_XIGEO_PLANE_PARA tPlanePara, std::vector<T_POINT_3D>& vtIntersectionPoint);

		bool GetTwoPlaneIntersectionLine(T_XIGEO_PLANE_PARA tFirstPlanePara, T_XIGEO_PLANE_PARA tSecondPlanePara, T_XIGEO_LINE_PARA& tIntersectionLine);

		double CalAngleEntityAndLine(T_POINT_3D tIntersectionPoint, T_DXF_ENTITY_PARA tEntity, T_POINT_3D tLineStartPoint, T_POINT_3D tLineEndPoint);
		
		double VectorsInnerAngle(double dFirstVecX, double dFirstVecY, double dFirstVecZ, double dSecondVecX, double dSecondVecY, double dSecondVecZ);

		void ConvertPlaneType(T_XIGEO_PLANE_PARA tPlanePara, T_XIGEO_PLANE_PARAMTYPE& tPlaneParamType);

		double CalDisPointToPlane(T_POINT_3D tPoint, T_XIGEO_PLANE_PARAMTYPE tPlaneParam);

		T_XIGEO_PLANE_PARA CalcLaserPlate(T_ROBOT_COORS_IN_WORLD tCutPoint, E_CAM_ID eCamId);

		void InitPointInCamerMatrix(T_POINT_3D tPoint, double dMatrix[][4]);

		T_XIGEO_PLANE_PARA CalcTrackCameraAndLaserParam(T_ROBOT_COORS_IN_WORLD tWeldPoint, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, T_POINT_3D& tLaserCenterPoint, T_POINT_3D& tCamCenterPoint, std::vector<T_POINT_3D>& vtLaserPlanePoints, std::vector<T_POINT_3D>& vtCameraPoints);

		void TransData(std::vector<T_POINT_3D> vtPoints, std::vector<coll::Point3D>& vtTransPoints);

		bool CheckCollision(std::vector<T_POINT_3D> vtLaserPolygon, std::vector<T_POINT_3D> vtCameraPolygon, std::vector<vector<T_POINT_3D>> vvtCheckBlockPoints);

		bool CalIfCollison(T_ROBOT_COORS_IN_WORLD tRobotCoors, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate);

		void CheckInputRobotRxRy(T_ROBOT_COORS_IN_WORLD& tRobotCoors);

		bool CalIfAdjust(T_ROBOT_COORS_IN_WORLD tRobotCoors, double dRxSearchRange, double dRySearchRange, double dSearchUnit, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, T_ROBOT_COORS_IN_WORLD& tAdjustRobotCoors);

		bool AdjustMeasureRobotCoors(std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints, std::vector<T_LINE_DIR_3D> vtWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, std::vector<T_ROBOT_COORS_IN_WORLD>& vtAdjustMeasureRobotPoints);

		bool CalIfAdjust(T_ROBOT_COORS_IN_WORLD tRobotCoors, double dWeldPostureMatrix[][4], double dRxSearchRange, double dRySearchRange, double dSearchUnit, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, T_ROBOT_COORS_IN_WORLD& tAdjustRobotCoors);

		bool AdjustMeasureRobotCoors(std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints, T_ROBOT_COORS_IN_WORLD tWeldRobotPoint, T_LINE_DIR_3D tWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, std::vector<T_ROBOT_COORS_IN_WORLD>& vtAdjustMeasureRobotPoints);

		T_LINE_DIR_3D CalLineDir(double dStartX, double dStartY, double dStartZ, double dEndX, double dEndY, double dEndZ);
		T_LINE_DIR CalLineDir(double dStartX, double dStartY, double dEndX, double dEndY);

		void CoorTransform(T_POINT_3D& tPoint, double dTransMatrix[][4]);

		int FindNearestPoint(T_POINT_3D tPoint, const std::vector<T_POINT_3D>& vtPoints);
		int FindNearestPointNew(T_POINT_3D tPoint, std::vector<T_POINT_3D> vtPoints, std::vector<int> vnEntityNo, int nEntityNo);

		void TransXiLineDirToLineDir(T_XIGEO_LINE_DIR tXiLineDir, T_LINE_DIR_3D& tLineDir);

		void TransLineDirToXiLineDir(T_LINE_DIR_3D tLineDir, T_XIGEO_LINE_DIR& tXiLineDir);
		
		void CalBevelAdjustDirForTracking(std::vector<T_POINT_3D> vtBevelPoints, std::vector<vector<T_POINT_3D>> vvtInciseEntityWithBevelPoints, std::vector<vector<T_LINE_DIR>> vvtInciseEntityPointAdjustDir, std::vector<T_LINE_DIR>& vtBevelAdjustDir);

		// ���٣����Ҳ�ͼλ�ÿ�����ʵ����Ϣ
		T_DOTRACKING_PARA* FindCurSeenEntities(T_ROBOT_COORS tImgCoord);

		// ���٣���ͼλ���� ������λ�� ������ʵ�� �ж��Ƿ���Ҫ����
		bool JudgeToDoAdjust(T_ALGORITHM_POINT tSeenPos, T_DOTRACKING_PARA tSeenEntity, T_ALGORITHM_POINT& tFilterOutPoint, bool& bOutValid);

		void AdjustBevelDataByTeachPointsForTrackingNew(std::vector<vector<T_POINT_3D>> vvtInciseEntityWithBevelPoints, std::vector<int> vnInciseEntityWithBevelNo, std::vector<vector<T_LINE_DIR>> vvtInciseEntityPointAdjustDir, std::vector<double> vdInciseToBevelDis,
			const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_3D> vtBevelPointsWithoutAdjust, std::vector<T_LINE_DIR> vtBevelAdjustDir, std::vector<T_POINT_3D> vtBevelPoints, T_POINT_3D tCurrentGunPoint,
			T_POINT_3D tCurrentBevelCutPoint, T_TEACH_PARA tTeachParaPoints, std::vector<T_POINT_3D>& vtAdjustPoints);
		
		void GetAdjustInfo(T_POINT_3D tCurrentGunPoint, T_POINT_3D tCurrentBevelCutPoint, T_POINT_3D tCurrentBevelMeasurePoint, const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_3D> vtBevelPoints, int nAdjustNumOnce, int& nCurrentCutNo, int& nCurrentMeasureNo, int& nAdjustEndNo);
		
		int GetAdjustEndNoNew(int nCurrentCutNo, int nCurrentMesureNo, int nAdjustNumOnce, const std::vector<T_DOTRACKING_PARA>& vtIfDoTracking);
		void GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage);
		T_POINT_2D GetIntersectionPointInImage(T_ROBOT_COORS_IN_WORLD tCutPoint, T_POINT_3D tIntersectionPoint, int nImageWidth, int nImageHeight, E_CAM_ID eCamId);
		void GetCameraIfDoTrackingNew(T_XIGEO_PLANE_PARA tContourPlanePara, std::vector<T_DXF_ENTITY_PARA> vtContourEntities, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnEntityNo, std::vector<T_ROBOT_COORS_IN_WORLD> vtCutRobotCoors, E_CAM_ID eCamId, std::vector<T_DOTRACKING_PARA>& vtIfDoTracking, std::vector<T_POINT_2D>& vtTrackingPointInImage, std::vector<T_POINT_2D>& vtHorTrackingPointInImage, std::vector<T_POINT_2D>& vtVerTrackingPointInImage);

		GROUP_ROBOT_ABS_COORS_TRANS::XiRobotAbsCoorsTrans* m_cXiRobotAbsCoorsTrans = nullptr;
		XiAlgorithm m_cXiAlgorithm;
		CAnalyzeDxf m_cAnalyzeObj;
		int m_nLastPointOfTracking = 0;							// ��¼���㵽�����һ���ɸ��ٵ������
		int m_nEndAdjustLimit = 65;								// �յ㴦������ֵ
		double m_dDisLimitForCalDoTracking = 60.0;		// �����Ƿ�ɸ��پ�����ֵ 40 60
		double m_dAngleLimitForCalDoTracking = 65.0;		// �����Ƿ�ɸ��ٽǶ���ֵ 45 65
		bool m_bEnableChangeRZ = 1;							//�Ƿ���Բ�����ٱ���̬
		const int m_nMinFilterInputPointNum = 10;				// �����˲�����켣���ٵ���
		const double m_dMinFilterInputPointSpacing = 1.0;		// ��������켣������С����
		const double m_dMaxFilterInputPointDis = 70.0;			// �����˲�����켣�����
		const int m_nPntCountBetweenEachModification = 5;		// ÿ�θ��ٹ켣�����ļ������

		int m_nFilterOutValidNum = 0;							// ��¼�˲���Ч����ܸ���

		std::vector<T_DXF_ENTITY_PARA> m_vtContourEntities;		//��ʼ������ʵ�壬���ں��ӣ�ͨ���켣��ϵõ�
		std::vector<T_POINT_3D> m_vtPartOutPoints;				//��ʼ�������켣
		std::vector<int> m_vnPartOutEntityNo;					//��ʼ�������켣��Ӧ��ʵ�����

		std::vector<T_POINT_3D> m_vtBevelPoints;				//��ʼ�¿ڹ켣�����ں��ӣ�ͬ������
		std::vector<int> m_vnBevelEntityNo;						//��ʼ�¿ڹ켣��Ӧ��ʵ����ţ����ں��ӣ�ͬ������
		std::vector<T_POINT_3D>			m_vtOrgRobotWeldPoints;		// ԭʼ���۹켣

		std::vector<vector<T_POINT_3D>> m_vvtInciseEntityWithBevelPoints;	// �㷨��������켣�ĳ�ʼ������
		std::vector<int>				m_vnInciseEntityWithBevelNo;
		std::vector<vector<T_LINE_DIR>> m_vvtInciseEntityPointAdjustDir;
		std::vector<double>				m_vdInciseToBevelDis;
		std::vector<vector<T_POINT_3D>> m_vvtBevelEntityPoints;				// δʹ�ò���
		std::vector<int>				m_vnOutBevelEntityNo;				// δʹ�ò���

		std::vector<T_DOTRACKING_PARA>  m_vtDoTrackPara;			//�Ƿ���Ը��٣��͸��ٵ�ʵ��

		std::vector<T_LINE_DIR>			m_vtBevelAdjustDir;

		std::map<int, std::vector<T_ALGORITHM_POINT>> m_mnvFilterInput; // ����ʵ����˲�����㼯�� (ʵ��ţ�ʵ���˲�����㼯��)
		//T_DOTRACKING_PARA m_tCurSeenEntity;//��ǰ������ʵ��

		CMyCriticalSection m_cLock;
		std::vector<T_POINT_3D>			m_vtTeachRobotWeldPoints;	// ��ʼʾ�����ɵ��и�켣��ÿ����������Ҫ����

		CMyCriticalSection m_cPntLock;
		T_POINT_3D m_tLastSendPnt;
		std::atomic<int> m_nLastSendPntNo;
		int m_nLastAdjustStartPntNo = 0;
		int m_nLastAdjustEndPntNo = 0;
		int m_nAdjustTimes = 0;//�޸Ĵ���

		bool savePlaneContourInfo();

		int m_nStartPntEntityNo = 0;//���ʵ��ʱ���������ʵ������
		PlaneContourInfo* m_ptPlaneContourInfo = NULL;//���ʵ��
		int m_nEntityCount = 0;//���ʵ�����Ŀ

		std::vector<T_POINT_2D> m_vtTrackingPointInImage;//���ۼ����ά���㣬������켣��ɢ���Ӧ
		std::vector<T_POINT_2D> m_vtHorTrackingPointInImage;//���۵װ巽�򼤹��ά�㣬������켣��ɢ���Ӧ
		std::vector<T_POINT_2D> m_vtVerTrackingPointInImage;//�������巽�򼤹��ά�㣬������켣��ɢ���Ӧ
	};
}

