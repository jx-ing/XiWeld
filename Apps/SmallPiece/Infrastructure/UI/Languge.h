#pragma once
#include "OpenClass/FileOP/Excel/OpExcel.h"
#include "Apps/SmallPiece/Infrastructure/UI/XiFmt.h"
#include "Apps/PLib/BasicFunc/VariableType.h"

namespace XUI
{
	/// @brief ����
	class Languge
	{
	public:
		/// @brief ��������
		struct T_SETTINGS
		{
			CString sName;//�������ƣ������
			bool bEnable = false;//�Ƿ�����
			UINT nLangugeID = MAKELANGID(LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED);//����ID��Ĭ������
		};

		static Languge& GetInstance()
		{
			return m_instance;
		}

		/// @brief ������������
		/// @param sLangugeFile �����ļ�
		/// @return �ɹ�/ʧ��
		bool loadAllLanguge(CString sLangugeFile = "SmallPieceConfig\\languge.xls");

		/// @brief ��ȡ��ѡ�����б�
		/// @return ��ѡ�����б�
		std::vector<CString> getLangugeList();

		/// @brief ѡ������
		/// @param sLanguge ����
		/// @return �ɹ�/ʧ��
		bool chioceLanguge(CString sLanguge);

		/// @brief ��ȡ��ǰ����ID
		/// @return ����ID
		UINT getCurLangugeID();

		/// @brief ����
		/// @param srtString ����ǰ�ַ���
		/// @return ������ַ���
		CString translate(CString srtString);

		/// @brief ���루����ʹ��C++20 std::format��ʽ���ַ�����
		/// @tparam ...T ��β�������
		/// @param format ��ʽ���ַ���
		/// @param ...args ����б�
		/// @return ������ַ���
		template <typename... T>
		CString translate(char* format, T&&... args)
		{
			CString sFormat(format);
			sFormat = translate(sFormat);
			auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
			return str.c_str();
		}

		/// @brief ����Ի���
		/// @param pDlg �Ի���ָ��
		void translateDialog(CDialog* pDlg);

	private:
		Languge();
		~Languge();
		Languge(const Languge&) = delete;
		Languge& operator=(const Languge&) = delete;

		static Languge m_instance;

	private:
		/// @brief �����������ò���
		/// @param mvsSettings �������ò���
		void loadSettings(const std::map<CString, std::vector<CString>>& mvsSettings);

		std::map<CString, std::vector<CString>> m_mvsLanguge;//�������Է���
		std::vector<T_SETTINGS> m_vsLangugeList;//��ѡ�����б�
		int m_nCurLangugeNo = -1;//��ǰѡ������ԣ�Ĭ������
	};
}

