///jwq20250227 需要修改SendWeldData
#pragma once
#include "LocalFiles/ExLib/ALGO/include/AnalyzeDxf.h"
#include "LocalFiles/ExLib/ALGO/include/XiAlgorithm.h"
#include "LocalFiles/ExLib/ALGO/include/XiPlaneTypes.h"

namespace PartData
{
	class WeldTrack
	{
	public:
		WeldTrack();
		~WeldTrack();

		/// @brief 初始化焊接轨迹
		/// @param vtInitWeldCoord 初始焊接轨迹
		void initTrack(const std::vector<T_ROBOT_COORS>& vtInitWeldCoord);

		/// @brief 调整轨迹
		/// @param vtNewPoints 输入轨迹
		/// @param nBeginNo 调整起始点序号
		/// @param nEndNo 调整终点序号
		/// @return 成功/失败
		bool adjustTrack(const std::vector<T_POINT_3D>& vtNewPoints, int nBeginNo, int nEndNo);

		/// @brief 获取轨迹点的数目
		/// @return 轨迹点的数目
		size_t getCount();

		/// @brief 获取指定序号的点
		/// @param nNo 序号
		/// @return 指定序号的点
		T_ROBOT_COORS getPoint(int nNo);

		/// @brief 返回最后一个坐标
		/// @return 最后一个坐标
		T_ROBOT_COORS back();

		/// @brief 删除指定段轨迹
		/// @param nBeginNo 开始序号
		/// @param nEndNo 结束序号
		void erase(int nBeginNo, int nEndNo);

		/// @brief 获取指定段轨迹
		/// @param nBeginNo 开始序号
		/// @param nEndNo 结束序号
		/// @return 指定段轨迹
		std::vector<T_ROBOT_COORS> getTrack(int nBeginNo, int nEndNo);

		/// @brief 获取全部轨迹
		/// @return 全部轨迹
		std::vector<T_ROBOT_COORS> getTrack();

		/// @brief 获取最接近目标位置的坐标（快速）
		/// @param tNearestCoord 最接近目标位置的坐标
		/// @param tAimCoord 目标位置坐标
		/// @param dMaxDis 最大距离阈值
		/// @param nStartSearchNo 从某一序号开始搜索
		/// @return 成功/失败
		bool getNearestCoordFast(T_ROBOT_COORS& tNearestCoord, T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo = 0);

		/// @brief 获取最接近目标位置坐标的序号（快速）
		/// @param tAimCoord 目标位置坐标
		/// @param dMaxDis 最大距离阈值
		/// @param nStartSearchNo 从某一序号开始搜索
		/// @return 大于等于0：最接近目标位置坐标的序号
		/// @return 小于0：查找失败
		int getNearestCoordNoFast(T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo = 0);

		/// @brief 保存轨迹数据到文件
		/// @param sFilePath 文件路径
		/// @return 成功/失败
		bool saveTrack(CString sFilePath);

		/// @brief 从文件加载轨迹数据
		/// @param sFilePath 文件路径
		/// @return 成功/失败
		bool loadTrack(CString sFilePath);

		/// @brief 保存世界坐标系下轨迹数据到文件
		/// @param sFilePath 文件路径
		/// @return 成功/失败
		bool saveTrackWorld(CString sFilePath);

		/// @brief 转换到世界坐标系下
		/// @tparam T 单个坐标的数据类型
		/// @return 世界坐标系下轨迹
		template<typename T>
		inline std::vector<T> toWorld();

		/// @brief 转换到世界坐标系下
		/// @tparam T 单个坐标的数据类型
		/// @return 世界坐标系下轨迹
		template<typename T>
		inline std::vector<T> toWorldD();

		/// @brief 是否是闭合曲线
		/// @return 是/否
		bool isClosed() const;

	private:
		CMyCriticalSection m_cLock;						//轨迹读写保护锁
		std::vector<T_ROBOT_COORS> m_vtWeldCoord;		//真实焊接轨迹
		bool m_bIsClosedTrack = false;					//是否是闭合轨迹

		/// @brief 检查轨迹是否闭合
		void checkIsClosed();
	};
}

namespace PartData
{
	template<typename T>
	inline std::vector<T> WeldTrack::toWorld()
	{
		CRITICAL_AUTO_LOCK(m_cLock);

		std::vector<T> vtWorldCoord(m_vtWeldCoord.size());
		for (size_t i = 0; i < m_vtWeldCoord.size(); i++)
		{
			vtWorldCoord[i].x = m_vtWeldCoord[i].dX + m_vtWeldCoord[i].dBX;
			vtWorldCoord[i].y = m_vtWeldCoord[i].dY + m_vtWeldCoord[i].dBY;
			vtWorldCoord[i].z = m_vtWeldCoord[i].dZ + m_vtWeldCoord[i].dBZ;
		}
		return vtWorldCoord;
	}

	template<typename T>
	inline std::vector<T> WeldTrack::toWorldD()
	{
		CRITICAL_AUTO_LOCK(m_cLock);

		std::vector<T> vtWorldCoord(m_vtWeldCoord.size());
		for (size_t i = 0; i < m_vtWeldCoord.size(); i++)
		{
			vtWorldCoord[i].dX = m_vtWeldCoord[i].dX + m_vtWeldCoord[i].dBX;
			vtWorldCoord[i].dY = m_vtWeldCoord[i].dY + m_vtWeldCoord[i].dBY;
			vtWorldCoord[i].dZ = m_vtWeldCoord[i].dZ + m_vtWeldCoord[i].dBZ;
			vtWorldCoord[i].dRX = m_vtWeldCoord[i].dRX;
			vtWorldCoord[i].dRY = m_vtWeldCoord[i].dRY;
			vtWorldCoord[i].dRZ = m_vtWeldCoord[i].dRZ;
		}
		return vtWorldCoord;
	}
}

