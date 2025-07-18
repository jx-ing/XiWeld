#pragma once
#include "OpenClass/FileOP/Excel/OpExcel.h"
#include "Apps/SmallPiece/Infrastructure/UI/XiFmt.h"
#include "Apps/PLib/BasicFunc/VariableType.h"

namespace XUI
{
	/// @brief 语言
	class Languge
	{
	public:
		/// @brief 语言设置
		struct T_SETTINGS
		{
			CString sName;//语言名称（翻译后）
			bool bEnable = false;//是否启用
			UINT nLangugeID = MAKELANGID(LANG_CHINESE, SUBLANG_CHINESE_SIMPLIFIED);//语言ID，默认中文
		};

		static Languge& GetInstance()
		{
			return m_instance;
		}

		/// @brief 加载所有语言
		/// @param sLangugeFile 语言文件
		/// @return 成功/失败
		bool loadAllLanguge(CString sLangugeFile = "SmallPieceConfig\\languge.xls");

		/// @brief 获取可选语言列表
		/// @return 可选语言列表
		std::vector<CString> getLangugeList();

		/// @brief 选择语言
		/// @param sLanguge 语言
		/// @return 成功/失败
		bool chioceLanguge(CString sLanguge);

		/// @brief 获取当前语言ID
		/// @return 语言ID
		UINT getCurLangugeID();

		/// @brief 翻译
		/// @param srtString 翻译前字符串
		/// @return 翻译后字符串
		CString translate(CString srtString);

		/// @brief 翻译（必须使用C++20 std::format格式的字符串）
		/// @tparam ...T 变参参数类型
		/// @param format 格式化字符串
		/// @param ...args 变参列表
		/// @return 翻译后字符串
		template <typename... T>
		CString translate(char* format, T&&... args)
		{
			CString sFormat(format);
			sFormat = translate(sFormat);
			auto str = fmt::vformat((LPCTSTR)sFormat, fmt::v11::vargs<T...>{{args...}});
			return str.c_str();
		}

		/// @brief 翻译对话框
		/// @param pDlg 对话框指针
		void translateDialog(CDialog* pDlg);

	private:
		Languge();
		~Languge();
		Languge(const Languge&) = delete;
		Languge& operator=(const Languge&) = delete;

		static Languge m_instance;

	private:
		/// @brief 加载语言设置参数
		/// @param mvsSettings 语言设置参数
		void loadSettings(const std::map<CString, std::vector<CString>>& mvsSettings);

		std::map<CString, std::vector<CString>> m_mvsLanguge;//所有语言翻译
		std::vector<T_SETTINGS> m_vsLangugeList;//可选语言列表
		int m_nCurLangugeNo = -1;//当前选择的语言，默认中文
	};
}

