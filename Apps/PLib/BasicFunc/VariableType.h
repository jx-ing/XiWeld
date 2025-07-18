#pragma once

namespace XiVariableType
{
	//以下是具体的相互转换函数
#if 1
	inline bool CString2bool(CString str, bool& value)
	{
		value = str == _T("是");
		if (!value
			&& str != _T("否"))
		{		
			return false;
		}
		return true;
	}

	inline bool bool2CString(bool value, CString& str)
	{
		value ? str = _T("是") : str = _T("否");
		return true;
	}

	inline bool CString2int(CString str, int& value)
	{
		value = atoi(str);
		return true;
	}

	inline bool int2CString(int value, CString& str)
	{
		str.Format(_T("%d"), value);
		return true;
	}

	inline bool CString2uint(CString str, unsigned int& value)
	{
		value = (unsigned int)strtoul(str, NULL, 0);
		return true;
	}

	inline bool uint2CString(unsigned int value, CString& str)
	{
		str.Format(_T("%u"), value);
		return true;
	}

	inline bool CString2long(CString str, long& value)
	{
		value = atol(str);
		return true;
	}

	inline bool long2CString(long value, CString& str)
	{
		str.Format(_T("%ld"), value);
		return true;
	}

	inline bool CString2ulong(CString str, unsigned long& value)
	{
		value = strtoul(str, NULL, 0);
		return true;
	}

	inline bool ulong2CString(unsigned long value, CString& str)
	{
		str.Format(_T("%lu"), value);
		return true;
	}

	inline bool CString2float(CString str, float& value)
	{
		value = (float)atof(str);
		return true;
	}

	inline bool float2CString(float value, CString& str)
	{
		str.Format(_T("%.6lf"), (double)value);
		return true;
	}

	inline bool CString2double(CString str, double& value)
	{
		value = atof(str);
		return true;
	}

	inline bool double2CString(double value, CString& str)
	{
		str.Format(_T("%.6lf"), value);
		return true;
	}

	inline bool CString2CString(CString src, CString& dst)
	{
		dst = src;
		return true;
	}

	static bool CString2RobotCoors(CString str, T_ROBOT_COORS& value)
	{
		std::vector<CString> vsCoors = XiBase::TokenizeCString(str, " \r\n,");
		if (vsCoors.size() != 9)
		{
			return false;
		}

		return CString2double(vsCoors[0], value.dX)
			&& CString2double(vsCoors[1], value.dY)
			&& CString2double(vsCoors[2], value.dZ)
			&& CString2double(vsCoors[3], value.dRX)
			&& CString2double(vsCoors[4], value.dRY)
			&& CString2double(vsCoors[5], value.dRZ)
			&& CString2double(vsCoors[6], value.dBX)
			&& CString2double(vsCoors[7], value.dBY)
			&& CString2double(vsCoors[8], value.dBZ);
	}

	inline bool RobotCoors2CString(T_ROBOT_COORS value, CString& str)
	{
		str.Format(_T("%11.6lf, %11.6lf, %11.6lf, %11.6lf, %11.6lf, %11.6lf, %11.6lf, %11.6lf, %11.6lf"), 
			value.dX, value.dY, value.dZ,
			value.dRX, value.dRY, value.dRZ,
			value.dBX, value.dBY, value.dBZ);
		return true;
	}

	static bool CString2AnglePulse(CString str, T_ANGLE_PULSE& value)
	{
		std::vector<CString> vsCoors = XiBase::TokenizeCString(str, " \r\n,");
		if (vsCoors.size() != 9)
		{
			return false;
		}

		return CString2long(vsCoors[0], value.nSPulse)
			&& CString2long(vsCoors[1], value.nLPulse)
			&& CString2long(vsCoors[2], value.nUPulse)
			&& CString2long(vsCoors[3], value.nRPulse)
			&& CString2long(vsCoors[4], value.nBPulse)
			&& CString2long(vsCoors[5], value.nTPulse)
			&& CString2long(vsCoors[6], value.lBXPulse)
			&& CString2long(vsCoors[7], value.lBYPulse)
			&& CString2long(vsCoors[8], value.lBZPulse);
	}

	inline bool AnglePulse2CString(T_ANGLE_PULSE value, CString& str)
	{
		str.Format(_T("%10ld, %10ld, %10ld, %10ld, %10ld, %10ld, %10ld, %10ld, %10ld"),
			value.nSPulse, value.nLPulse, value.nUPulse,
			value.nRPulse, value.nBPulse, value.nTPulse,
			value.lBXPulse, value.lBYPulse, value.lBZPulse);
		return true;
	}

#endif

	//X_MACRO的数据：
	// 1.第一个是枚举类型名
	// 2.第二个是数据类型
	// 3.第三个是对应的显示用数据类型名称，一般用于从文件读取
	// 4.第四个是CStringToAny转换函数
	// 5.第五个是AnyToCString转换函数
#define MACRO_TYPE\
    X_MACRO(E_bool			,bool			,"bool"				,CString2bool		,bool2CString		)\
    X_MACRO(E_int			,int			,"int"				,CString2int		,int2CString		)\
    X_MACRO(E_uint			,unsigned int	,"unsigned int"		,CString2uint		,uint2CString		)\
    X_MACRO(E_long			,long			,"long"				,CString2long		,long2CString		)\
    X_MACRO(E_ulong			,unsigned long	,"unsigned long"	,CString2ulong		,ulong2CString		)\
    X_MACRO(E_float			,float			,"float"			,CString2float		,float2CString		)\
    X_MACRO(E_double		,double			,"double"			,CString2double		,double2CString		)\
    X_MACRO(E_CString		,CString		,"CString"			,CString2CString	,CString2CString	)\
    X_MACRO(E_ROBOT_COORS	,T_ROBOT_COORS	,"T_ROBOT_COORS"	,CString2RobotCoors	,RobotCoors2CString	)\
    X_MACRO(E_ANGLE_PULSE	,T_ANGLE_PULSE	,"T_ANGLE_PULSE"	,CString2AnglePulse	,AnglePulse2CString	)\

	//数据类型的枚举
	enum E_VARIABLE_TYPE
	{
#define X_MACRO(a, b, c, d, e) a,
		MACRO_TYPE
#undef X_MACRO
	};

	//从显示用数据类型名称指向对应的数据类型枚举
	static std::map<CString, E_VARIABLE_TYPE> g_meVariableTypeFormShowName = 
	{ 
#define X_MACRO(a, b, c, d, e) {c, a},
		MACRO_TYPE
#undef X_MACRO
	};

	//从程序内部数据类型名称指向对应的数据类型枚举
	static std::map<CString, E_VARIABLE_TYPE> g_meVariableTypeFormTypeName =
	{
#define X_MACRO(a, b, c, d, e) {typeid(b).name(), a},
		MACRO_TYPE
#undef X_MACRO
	};

	//判断是否是相同的数据类型
	static bool IsSameVariableType(CString sShowName, CString sTypeName)
	{
		auto iter = g_meVariableTypeFormShowName.find(sShowName);
		if (iter == g_meVariableTypeFormShowName.end())
		{
			return false;
		}

		switch (iter->second)
		{
#define X_MACRO(a, b, c, d, e) case a: return typeid(b).name() == sTypeName;
			MACRO_TYPE
#undef X_MACRO			
		default:
			return false;
		}
		return false;
	};

	//从CString转为特定类型
	template<typename T>
	bool CStringToAny(CString sValue, T& value)
	{
		auto iter = g_meVariableTypeFormTypeName.find(typeid(T).name());
		if (iter == g_meVariableTypeFormTypeName.end())
		{
			return false;
		}

		switch (iter->second)
		{
#define X_MACRO(a, b, c, d, e) case a:  return d(sValue, *(b*)(void*)(&value));
			MACRO_TYPE
#undef X_MACRO			
		default:
			return false;
		}
	}

	//从CString转为特定类型，并检验数据类型和设定类型是否相同
	template<typename T>
	bool CStringToAny(CString sValue, CString sTypeShowName, T& value)
	{
		auto iter = g_meVariableTypeFormTypeName.find(typeid(T).name());
		if (iter == g_meVariableTypeFormTypeName.end())
		{
			return false;
		}

		switch (iter->second)
		{
#define X_MACRO(a, b, c, d, e) \
				case a: \
					if (c != sTypeShowName)\
					{\
						return false; \
					}\
					return d(sValue, *(b*)(void*)(&value));
			MACRO_TYPE
#undef X_MACRO			
		default:
			return false;
		}
	}

	//从特定类型转为CString
	template<typename T>
	bool AnyToCString(T value, CString& sValue)
	{
		auto iter = g_meVariableTypeFormTypeName.find(typeid(T).name());
		if (iter == g_meVariableTypeFormTypeName.end())
		{
			return false;
		}

		switch (iter->second)
		{
#define X_MACRO(a, b, c, d, e) case a: return e(*(b*)(void*)(&value), sValue);
			MACRO_TYPE
#undef X_MACRO			
		default:
			return false;
		}
	}

	//将指定行索引为sKey，列号为nColumnNo的数据转化为列号1的数据value
	//若输入参数和读取参数类型不同，则失败
	template<typename T>
	bool GetParaFromExcelData(const std::map<CString, std::vector<CString>>& vvsValue, CString sKey, size_t nColumnNo, T& value)
	{
		auto iter = vvsValue.find(sKey);
		if (iter == vvsValue.end())
		{
			return false;
		}
		if (iter->second.size() <= nColumnNo)
		{
			return false;
		}
		return XiVariableType::CStringToAny<T>(iter->second[nColumnNo], iter->second[1], value);
	}

	//将指定行索引为sKey，列号为2的数据转化为类型为列号1的数据value
	//若输入参数和读取参数类型不同，则失败
	template<typename T>
	bool GetParaFromExcelData(const std::map<CString, std::vector<CString>>& vvsValue, CString sKey, T& value)
	{
		auto iter = vvsValue.find(sKey);
		if (iter == vvsValue.end())
		{
			return false;
		}
		if (iter->second.size() < 3)
		{
			return false;
		}
		return XiVariableType::CStringToAny<T>(iter->second[2], iter->second[1], value);
	}

	//自动构造数组类，申请内存空间，析构时自动释放内存
	//注意如果使用结构体，结构体内部包含指针的话，这个指针的内存不会被释放
	//如果传入的指针本身已经指向一片内存，该类不会主动释放此内存
	template<typename T>
	class CNewArray
	{
	public:
		CNewArray(T*& arrayName, size_t unSize, bool bInit = true)
		{
			m_arrayName = &arrayName;
			*m_arrayName = new T[unSize];
			if (bInit)
			{
				for (size_t i = 0; i < unSize; i++)
				{
					(*m_arrayName)[i] = 0;
				}
			}
		}
		~CNewArray()
		{
			if (nullptr != *m_arrayName)
			{
				delete *m_arrayName;
			}
			*m_arrayName = nullptr;
			m_arrayName = nullptr;
		}

	private:
		T** m_arrayName = nullptr;
	};
}