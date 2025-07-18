#pragma once
#include <atomic>
#include "Apps/PartData/WeldTrack.h"
#include "Apps/PLib/YaskawaRobot/RobotDriverAdaptor.h"
#include "LocalFiles/ExLib/Vision/include/GFPGJointLib.h"
#include "Apps/Welding/PolygonCollision.h"

class CUnit;

namespace RTT
{
	//实时跟踪，轨迹调整
	class AdjustTrack
	{
	public:
		AdjustTrack();
		~AdjustTrack();

		bool loadPara();

		/// @brief 设置控制单元控制指针
		/// @param cUnit 控制单元控制指针
		void setUnit(CUnit& cUnit);

		/// @brief 设置初始焊接轨迹
		/// @param cWeldTrack 初始焊接轨迹
		void initTrack(PartData::WeldTrack& cWeldTrack);

		/// @brief 跟踪开始之前初始化数据
		/// @return 成功/失败
		bool initBeforeTrack();

		/// @brief 判断是否需要处理图片
		/// @param tImgCoord 采集图片时的坐标
		/// @return 是/否
		bool determineWhetherToProcessImg(T_ROBOT_COORS tImgCoord, CvPoint& tIdeal2DKeyPoint, CvPoint& tHorTrackingPointInImage, CvPoint& tVerTrackingPointInImage);

		/// @brief 调整轨迹
		/// @param tImgCoord 采集图片时的坐标
		/// @param tSeenCoord 激光交点的三维坐标
		/// @return 成功/失败
		bool adjustTrack(T_ROBOT_COORS tImgCoord, T_ROBOT_COORS tSeenCoord);

		/// @brief 设置最后一次发送的点坐标
		/// @param tLastSendPnt 最后一次发送的点坐标
		/// @param nPntNo 最后一次发送的点序号
		void setLastSendPnt(T_ROBOT_COORS tLastSendPnt, int nPntNo);

		/// @brief 获取最后一次发送的点坐标
		/// @return 最后一次发送的点坐标
		T_POINT_3D getLastSendPnt();

		/// @brief 计算距离输入点最近轨迹点的法向角
		/// @param tSeenCoord 输入点
		/// @return 法向角
		double calcNearestPointNormalDegree(T_ROBOT_COORS tSeenCoord);

		/// @brief 是否支持更新终点
		/// @return 是/否
		bool supportUpdatingEndPntOrNot();

		/// @brief 更新终点
		/// @param tEndPnt 新终点
		/// @return 成功/失败
		bool updateEndPnt(T_ROBOT_COORS tEndPnt);

		/// @brief 获取数据保存文件的根目录
		/// @return 数据保存文件的根目录
		CString getDataFileRootFolder() const;

		//
		//
		bool AdjustMeasureCoors(CUnit& cUnit, std::vector<T_ROBOT_COORS_IN_WORLD> vtMeasureRobotPoints, std::vector<T_LINE_DIR_3D> vtWeldLineDir, E_CAM_ID eCamId, std::vector<T_XIGEO_WELDLINE> vtCheckBlockPlate, std::vector<T_ROBOT_COORS_IN_WORLD>& vtAdjustMeasureRobotPoints);


	private:
		/// @brief 写日志
		/// @param format 格式化字符串
		/// @param  被格式化变量
		void writeLog(const char* format, ...) const;

		/// @brief 计算轨迹的实体
		/// @return 成功/失败
		bool calcTrackEntity();

		/// @brief 使轨迹实体闭合并输出闭合实体的离散点
		/// @return 成功/失败
		bool makeEntitiesClose();

		/// @brief 确保焊接起点不变
		/// @return 成功/失败
		bool makesureStratPnt();

		/// @brief 加载闭合实体
		/// @return 成功/失败
		bool loadClosedEntities();

		/// @brief 计算闭合实体的Z值
		/// @param vtOldCoord 机器人坐标系下旧轨迹
		/// @param vtOld3DPnts 世界坐标系下旧轨迹
		/// @return 成功/失败
		bool calcClosedEntitiesZ(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts);

		/// @brief 改变圆弧及其附近轨迹的RZ
		/// @param vnPntCount 每个实体的轨迹数目
		/// @param vtInitWeldCoord 输入输出的轨迹
		/// @return 成功/失败
		bool changeRZForArc(const std::vector<int>& vnPntCount, std::vector<T_ROBOT_COORS>& vtInitWeldCoord);

		/// @brief 加载闭合轨迹
		/// @param vtOldCoord 机器人坐标系下旧轨迹
		/// @param vtOld3DPnts 世界坐标系下旧轨迹
		/// @return 成功/失败
		bool loadClosedTrack(const std::vector<T_ROBOT_COORS>& vtOldCoord, const std::vector<CvPoint3D64f>& vtOld3DPnts);

		/// @brief 保存修改后的轨迹到文件里
		/// @param vtNewPoints 修改后的轨迹
		void saveAdjustTrack(const std::vector<T_POINT_3D>& vtNewPoints);

		/// @brief 判断轨迹是否安全可达
		/// @param tStartPulse 起点脉冲
		/// @param vtTrack 轨迹坐标
		/// @return 是/否
		bool isSafeTrack(T_ANGLE_PULSE tStartPulse, const std::vector<T_ROBOT_COORS>& vtTrack);

		double m_dMaxAdjustDisXY = 20.0;//XY最大调整量
		double m_dMaxAdjustDisZ = 15.0;//Z最大调整量
		CUnit* m_pUnit = nullptr;						//控制单元指针
		CRobotDriverAdaptor* m_pRobot = nullptr;		//机器人控制指针
		PartData::WeldTrack* m_pWeldTrack = nullptr;	//焊接轨迹
		bool m_bOnlyLineEntity = false;

		/**********************************************从坡口移植的代码**********************************************/
	public:
		/// @brief 轨迹信息
		struct T_DOTRACKING_PARA
		{
			bool bIfDoTracking = false;						//是否可跟踪：是否能看见角点或端点
			int nEntityNo = -1;								//看到的实体号
			int nEntityType = 0;							//看到的实体类型：1直线 2圆弧
			T_POINT_3D tIntersectionPoint;					//交点坐标
			std::vector<T_ALGORITHM_POINT> vtFilterInput;	//滤波输入点
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

		//获取跟踪用数据
		bool GetTrackingInciseData(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType, VT_GRAPH vtContourEntities, std::vector<T_POINT_3D> vtIncisePoints, std::vector<int> vnIncisePointEntityNo, std::vector<T_POINT_3D> vtBevelPoints, std::vector<int> vnBevelPointEntityNo,
			std::vector<vector<T_POINT_3D>>& vvtInciseEntityWithBevelPoints, std::vector<int>& vnInciseEntityWithBevelNo, std::vector<vector<T_LINE_DIR>>& vvtInciseEntityPointAdjustDir, std::vector<double>& vdInciseToBevelDis,
			std::vector<vector<T_POINT_3D>>& vvtBevelEntityPoints, std::vector<int>& vnBevelEntityNo);

		void CalAdjustDirAndDisForTracking(E_DOTRACKING_CONTOUR_TYPE eDoTrackingContourType, E_DOTRACKING_BEVEL_TYPE eDoTrackingBevelType,
			T_DXF_ENTITY_PARA tEntity, std::vector<T_POINT_3D> vtIncisePoints, std::vector<T_POINT_3D> vtBevelPoints, std::vector<T_LINE_DIR>& vtAdjustDir, double& dAdjustDis);

		// 跟踪：计算是否可以跟踪，和每个位置看到的是什么实体
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

		// 跟踪：查找采图位置看到的实体信息
		T_DOTRACKING_PARA* FindCurSeenEntities(T_ROBOT_COORS tImgCoord);

		// 跟踪：采图位置下 看到的位置 看到的实体 判断是否需要调整
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
		int m_nLastPointOfTracking = 0;							// 记录计算到的最后一个可跟踪点的索引
		int m_nEndAdjustLimit = 65;								// 终点处调整阈值
		double m_dDisLimitForCalDoTracking = 60.0;		// 计算是否可跟踪距离阈值 40 60
		double m_dAngleLimitForCalDoTracking = 65.0;		// 计算是否可跟踪角度阈值 45 65
		bool m_bEnableChangeRZ = 1;							//是否开启圆弧跟踪变姿态
		const int m_nMinFilterInputPointNum = 10;				// 跟踪滤波输入轨迹最少点数
		const double m_dMinFilterInputPointSpacing = 1.0;		// 跟踪输入轨迹点间隔最小距离
		const double m_dMaxFilterInputPointDis = 70.0;			// 跟踪滤波输入轨迹最长长度
		const int m_nPntCountBetweenEachModification = 5;		// 每次跟踪轨迹调整的间隔点数

		int m_nFilterOutValidNum = 0;							// 记录滤波有效输出总个数

		std::vector<T_DXF_ENTITY_PARA> m_vtContourEntities;		//初始外轮廓实体，对于焊接，通过轨迹拟合得到
		std::vector<T_POINT_3D> m_vtPartOutPoints;				//初始外轮廓轨迹
		std::vector<int> m_vnPartOutEntityNo;					//初始外轮廓轨迹对应的实体序号

		std::vector<T_POINT_3D> m_vtBevelPoints;				//初始坡口轨迹，对于焊接，同外轮廓
		std::vector<int> m_vnBevelEntityNo;						//初始坡口轨迹对应的实体序号，对于焊接，同外轮廓
		std::vector<T_POINT_3D>			m_vtOrgRobotWeldPoints;		// 原始理论轨迹

		std::vector<vector<T_POINT_3D>> m_vvtInciseEntityWithBevelPoints;	// 算法计算调整轨迹的初始化参数
		std::vector<int>				m_vnInciseEntityWithBevelNo;
		std::vector<vector<T_LINE_DIR>> m_vvtInciseEntityPointAdjustDir;
		std::vector<double>				m_vdInciseToBevelDis;
		std::vector<vector<T_POINT_3D>> m_vvtBevelEntityPoints;				// 未使用参数
		std::vector<int>				m_vnOutBevelEntityNo;				// 未使用参数

		std::vector<T_DOTRACKING_PARA>  m_vtDoTrackPara;			//是否可以跟踪，和跟踪的实体

		std::vector<T_LINE_DIR>			m_vtBevelAdjustDir;

		std::map<int, std::vector<T_ALGORITHM_POINT>> m_mnvFilterInput; // 所有实体的滤波输入点集合 (实体号，实体滤波输入点集合)
		//T_DOTRACKING_PARA m_tCurSeenEntity;//当前看见的实体

		CMyCriticalSection m_cLock;
		std::vector<T_POINT_3D>			m_vtTeachRobotWeldPoints;	// 初始示教生成的切割轨迹，每次修正后需要更新

		CMyCriticalSection m_cPntLock;
		T_POINT_3D m_tLastSendPnt;
		std::atomic<int> m_nLastSendPntNo;
		int m_nLastAdjustStartPntNo = 0;
		int m_nLastAdjustEndPntNo = 0;
		int m_nAdjustTimes = 0;//修改次数

		bool savePlaneContourInfo();

		int m_nStartPntEntityNo = 0;//拟合实体时，起点所在实体的序号
		PlaneContourInfo* m_ptPlaneContourInfo = NULL;//拟合实体
		int m_nEntityCount = 0;//拟合实体的数目

		std::vector<T_POINT_2D> m_vtTrackingPointInImage;//理论激光二维交点，与理想轨迹离散点对应
		std::vector<T_POINT_2D> m_vtHorTrackingPointInImage;//理论底板方向激光二维点，与理想轨迹离散点对应
		std::vector<T_POINT_2D> m_vtVerTrackingPointInImage;//理论立板方向激光二维点，与理想轨迹离散点对应
	};
}

