#pragma once
#include "Apps/SmallPiece/Infrastructure/UI/Languge.h"
#include "Apps/SmallPiece/Infrastructure/UI/XiFmt.h"
#include "LocalFiles/ExLib/Others/include/XiBase.h"

namespace XUI
{
	/// @brief ���� ֧�ַ��� Ĭ��ʹ��ϵͳ��־
	class MesBox
	{
	public:
		MesBox();
		virtual ~MesBox();

		/// @brief �����Ƿ񵯳�
		/// @param bPopup �Ƿ񵯳�
		void setPopup(bool bPopup);

		/// @brief ������־
		/// @param cLog ��־
		void setLog(XiBase::CLog& cLog);

		/// @brief ��������Yes/Noѡ��ĵ�����ʹ���趨����־�͵���ѡ�
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		/// @return true��yes
		/// @return false��no
		template <typename... T>
		bool popYesNo(const char* format, T&&... args);

		/// @brief ��������Yes/Noѡ��ĵ�����ʹ��ϵͳ��־��
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		/// @return true��Yes
		/// @return false��No
		template <typename... T>
		static bool PopYesNo(const char* format, T&&... args);
				
		/// @brief ��������Ok/Cancelѡ��ĵ�����ʹ���趨����־�͵���ѡ�
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		/// @return true��Ok
		/// @return false��Cancel
		template <typename... T>
		bool popOkCancel(const char* format, T&&... args);

		/// @brief ��������Ok/Cancelѡ��ĵ�����ʹ��ϵͳ��־��
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		/// @return true��Ok
		/// @return false��Cancel
		template <typename... T>
		static bool PopOkCancel(const char* format, T&&... args);

		/// @brief ������ʾ������ʹ���趨����־�͵���ѡ�
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		void popInfo(const char* format, T&&... args);

		/// @brief ������ʾ������ʹ��ϵͳ��־��
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		static void PopInfo(const char* format, T&&... args);

		/// @brief �������浯����ʹ���趨����־�͵���ѡ�
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		void popWarning(const char* format, T&&... args);

		/// @brief �������浯����ʹ��ϵͳ��־��
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		static void PopWarning(const char* format, T&&... args);

		/// @brief �������󵯴���ʹ���趨����־�͵���ѡ�
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		void popError(const char* format, T&&... args);

		/// @brief �������󵯴���ʹ��ϵͳ��־��
		/// @param format ��ʽ���ַ���
		/// @param ...args ���
		template <typename... T>
		static void PopError(const char* format, T&&... args);

	private:
		XiBase::CLog* m_pLog = nullptr;//��־ָ��
		bool m_bPopup = true;//�Ƿ񵯳�

	};

	/// @brief ��鵱ǰ�Ƿ���ڵ���
	/// @return ��/��
	bool CheckIsPopup();

	/// @brief ��鵱ǰ�Ƿ���ڴ��󵯴�
	/// @return ��/��
	bool IsErrorPopup();

	/// @brief ��ȡ��ǰ����������
	/// @return ��ǰ����������
	std::vector<unsigned int> GetPopupType();

	/// @brief �������������룩
	/// @param cLog ��־
	/// @param nType ��������
	/// @param bPopup �Ƿ񵯳�
	/// @param sMessage ��������
	/// @return ����ֵ
	int XiMessageBoxPopup(XiBase::CLog& cLog, unsigned int nType, bool bPopup, std::string sMessage);

	/// @brief �������������룩
	/// @param nType ��������
	/// @param bPopup �Ƿ񵯳�
	/// @param sMessage ��������
	/// @return ����ֵ
	int XiMessageBoxPopup(unsigned int nType, bool bPopup, std::string sMessage);
}

namespace XUI
{
	template<typename ...T>
	bool MesBox::popYesNo(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		return IDYES == XUI::XiMessageBoxPopup(*m_pLog, MB_ICONQUESTION | MB_YESNO, m_bPopup, str);
	}

	template<typename ...T>
	bool MesBox::PopYesNo(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		return IDYES == XiMessageBoxPopup(MB_ICONQUESTION | MB_YESNO, TRUE, str);
	}

	template<typename ...T>
	inline bool MesBox::popOkCancel(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		return IDOK == XUI::XiMessageBoxPopup(*m_pLog, MB_ICONQUESTION | MB_OKCANCEL, m_bPopup, str);
	}

	template<typename ...T>
	bool MesBox::PopOkCancel(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		return IDOK == XUI::XiMessageBoxPopup(MB_ICONQUESTION | MB_OKCANCEL, TRUE, str);
	}

	template<typename ...T>
	inline void MesBox::popInfo(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(*m_pLog, MB_ICONINFORMATION | MB_OK, m_bPopup, str);
	}

	template<typename ...T>
	void MesBox::PopInfo(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(MB_ICONINFORMATION | MB_OK, TRUE, str);
	}

	template<typename ...T>
	inline void MesBox::popWarning(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(*m_pLog, MB_ICONEXCLAMATION | MB_OK, m_bPopup, str);
	}

	template<typename ...T>
	void MesBox::PopWarning(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(MB_ICONEXCLAMATION | MB_OK, TRUE, str);
	}

	template<typename ...T>
	inline void MesBox::popError(const char* format, T && ...args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(*m_pLog, MB_ICONSTOP | MB_OK, TRUE, m_bPopup, str);
	}

	template <typename... T>
	static void MesBox::PopError(const char* format, T&&... args)
	{
		CString sFormat(format);
		sFormat = Languge::GetInstance().translate(sFormat);
		auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
		XUI::XiMessageBoxPopup(MB_ICONSTOP | MB_OK, TRUE, str);
	}
}