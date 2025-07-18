///jwq20250227 ��Ҫ�޸�SendWeldData
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

		/// @brief ��ʼ�����ӹ켣
		/// @param vtInitWeldCoord ��ʼ���ӹ켣
		void initTrack(const std::vector<T_ROBOT_COORS>& vtInitWeldCoord);

		/// @brief �����켣
		/// @param vtNewPoints ����켣
		/// @param nBeginNo ������ʼ�����
		/// @param nEndNo �����յ����
		/// @return �ɹ�/ʧ��
		bool adjustTrack(const std::vector<T_POINT_3D>& vtNewPoints, int nBeginNo, int nEndNo);

		/// @brief ��ȡ�켣�����Ŀ
		/// @return �켣�����Ŀ
		size_t getCount();

		/// @brief ��ȡָ����ŵĵ�
		/// @param nNo ���
		/// @return ָ����ŵĵ�
		T_ROBOT_COORS getPoint(int nNo);

		/// @brief �������һ������
		/// @return ���һ������
		T_ROBOT_COORS back();

		/// @brief ɾ��ָ���ι켣
		/// @param nBeginNo ��ʼ���
		/// @param nEndNo �������
		void erase(int nBeginNo, int nEndNo);

		/// @brief ��ȡָ���ι켣
		/// @param nBeginNo ��ʼ���
		/// @param nEndNo �������
		/// @return ָ���ι켣
		std::vector<T_ROBOT_COORS> getTrack(int nBeginNo, int nEndNo);

		/// @brief ��ȡȫ���켣
		/// @return ȫ���켣
		std::vector<T_ROBOT_COORS> getTrack();

		/// @brief ��ȡ��ӽ�Ŀ��λ�õ����꣨���٣�
		/// @param tNearestCoord ��ӽ�Ŀ��λ�õ�����
		/// @param tAimCoord Ŀ��λ������
		/// @param dMaxDis ��������ֵ
		/// @param nStartSearchNo ��ĳһ��ſ�ʼ����
		/// @return �ɹ�/ʧ��
		bool getNearestCoordFast(T_ROBOT_COORS& tNearestCoord, T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo = 0);

		/// @brief ��ȡ��ӽ�Ŀ��λ���������ţ����٣�
		/// @param tAimCoord Ŀ��λ������
		/// @param dMaxDis ��������ֵ
		/// @param nStartSearchNo ��ĳһ��ſ�ʼ����
		/// @return ���ڵ���0����ӽ�Ŀ��λ����������
		/// @return С��0������ʧ��
		int getNearestCoordNoFast(T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo = 0);

		/// @brief ����켣���ݵ��ļ�
		/// @param sFilePath �ļ�·��
		/// @return �ɹ�/ʧ��
		bool saveTrack(CString sFilePath);

		/// @brief ���ļ����ع켣����
		/// @param sFilePath �ļ�·��
		/// @return �ɹ�/ʧ��
		bool loadTrack(CString sFilePath);

		/// @brief ������������ϵ�¹켣���ݵ��ļ�
		/// @param sFilePath �ļ�·��
		/// @return �ɹ�/ʧ��
		bool saveTrackWorld(CString sFilePath);

		/// @brief ת������������ϵ��
		/// @tparam T �����������������
		/// @return ��������ϵ�¹켣
		template<typename T>
		inline std::vector<T> toWorld();

		/// @brief ת������������ϵ��
		/// @tparam T �����������������
		/// @return ��������ϵ�¹켣
		template<typename T>
		inline std::vector<T> toWorldD();

		/// @brief �Ƿ��Ǳպ�����
		/// @return ��/��
		bool isClosed() const;

	private:
		CMyCriticalSection m_cLock;						//�켣��д������
		std::vector<T_ROBOT_COORS> m_vtWeldCoord;		//��ʵ���ӹ켣
		bool m_bIsClosedTrack = false;					//�Ƿ��ǱպϹ켣

		/// @brief ���켣�Ƿ�պ�
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

