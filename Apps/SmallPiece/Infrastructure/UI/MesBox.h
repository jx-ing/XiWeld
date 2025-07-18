#pragma once
#include "Apps/SmallPiece/Infrastructure/UI/Languge.h"
#include "Apps/SmallPiece/Infrastructure/UI/XiFmt.h"
#include "LocalFiles/ExLib/Others/include/XiBase.h"

namespace XUI
{
	/// @brief 弹窗 支持翻译 默认使用系统日志
	class MesBox
	{
	public:
		MesBox();
		virtual ~MesBox();

		/// @brief 设置是否弹出
		/// @param bPopup 是否弹出
		void setPopup(bool bPopup);

		/// @brief 设置日志
		/// @param cLog 日志
		void setLog(XiBase::CLog& cLog);

		/// @brief 弹出带有Yes/No选项的弹窗（使用设定的日志和弹出选项）
		/// @param format 格式化字符串
		/// @param ...args 变参
		/// @return true：yes
		/// @return false：no
		template <typename... T>
		bool popYesNo(const char* format, T&&... args);

		/// @brief 弹出带有Yes/No选项的弹窗（使用系统日志）
		/// @param format 格式化字符串
		/// @param ...args 变参
		/// @return true：Yes
		/// @return false：No
		template <typename... T>
		static bool PopYesNo(const char* format, T&&... args);
				
		/// @brief 弹出带有Ok/Cancel选项的弹窗（使用设定的日志和弹出选项）
		/// @param format 格式化字符串
		/// @param ...args 变参
		/// @return true：Ok
		/// @return false：Cancel
		template <typename... T>
		bool popOkCancel(const char* format, T&&... args);

		/// @brief 弹出带有Ok/Cancel选项的弹窗（使用系统日志）
		/// @param format 格式化字符串
		/// @param ...args 变参
		/// @return true：Ok
		/// @return false：Cancel
		template <typename... T>
		static bool PopOkCancel(const char* format, T&&... args);

		/// @brief 弹出提示弹窗（使用设定的日志和弹出选项）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		void popInfo(const char* format, T&&... args);

		/// @brief 弹出提示弹窗（使用系统日志）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		static void PopInfo(const char* format, T&&... args);

		/// @brief 弹出警告弹窗（使用设定的日志和弹出选项）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		void popWarning(const char* format, T&&... args);

		/// @brief 弹出警告弹窗（使用系统日志）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		static void PopWarning(const char* format, T&&... args);

		/// @brief 弹出错误弹窗（使用设定的日志和弹出选项）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		void popError(const char* format, T&&... args);

		/// @brief 弹出错误弹窗（使用系统日志）
		/// @param format 格式化字符串
		/// @param ...args 变参
		template <typename... T>
		static void PopError(const char* format, T&&... args);

	private:
		XiBase::CLog* m_pLog = nullptr;//日志指针
		bool m_bPopup = true;//是否弹出

	};

	/// @brief 检查当前是否存在弹窗
	/// @return 是/否
	bool CheckIsPopup();

	/// @brief 检查当前是否存在错误弹窗
	/// @return 是/否
	bool IsErrorPopup();

	/// @brief 获取当前弹窗的类型
	/// @return 当前弹窗的类型
	std::vector<unsigned int> GetPopupType();

	/// @brief 弹窗（不带翻译）
	/// @param cLog 日志
	/// @param nType 弹窗类型
	/// @param bPopup 是否弹出
	/// @param sMessage 弹窗内容
	/// @return 返回值
	int XiMessageBoxPopup(XiBase::CLog& cLog, unsigned int nType, bool bPopup, std::string sMessage);

	/// @brief 弹窗（不带翻译）
	/// @param nType 弹窗类型
	/// @param bPopup 是否弹出
	/// @param sMessage 弹窗内容
	/// @return 返回值
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