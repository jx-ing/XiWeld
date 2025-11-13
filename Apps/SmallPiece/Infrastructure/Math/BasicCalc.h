/*************************************************************************************************
 * 文件： BasicCalc.h
 * 说明： 基础计算函数和常数
 * 作者： 江文奇
 * 日期： 2025-07-05
 * ToDo： 
 ************************************************************************************************/
#pragma once

namespace xi
{
namespace math
{
constexpr double _PI = 3.1415926535897932384626433832795;		//圆周率
constexpr double RAD_TO_REG = 180.0 / math::_PI;				//弧度转角度乘法系数
constexpr double MIN_1D_LIMIT = 0.0001;							//一维最小阈值
constexpr double MIN_2D_LIMIT = 0.00000001;						//二维最小阈值

/// @brief 弧度转角度
/// @param dRad 弧度
/// @return 角度
inline double rad2deg(double dRad)
{
	return dRad * RAD_TO_REG;
}

/// @brief 角度转弧度
/// @param dDeg 角度
/// @return 弧度
inline double deg2rad(double dDeg)
{
	return dDeg / RAD_TO_REG;
}

/// @brief 判断是否是闭合圆
/// @param dAngle 角度(0.0, 360.0]
/// @return 是/否
inline bool isClosedCircle(double dAngle)
{
	return  360.0 - dAngle <= MIN_1D_LIMIT;
}

/// @brief 判断是否是优弧
/// @param dAngle 角度(0.0, 360.0]
/// @return 是/否
inline bool isMajorArc(double dAngle)
{
	return dAngle - 180.0 >= MIN_1D_LIMIT;
}

/// @brief 判断是否是劣弧
/// @param dAngle 角度(0.0, 360.0]
/// @return 是/否
inline bool isMinorArc(double dAngle)
{
	return  180.0 - dAngle >= MIN_1D_LIMIT;
}

/// @brief 确保数值在周期范围内
/// @tparam nMin 周期最小值
/// @tparam nMax 周期最大值
/// @param value 原数值
/// @return 周期内数值
template<int nMin, int nMax>
inline double makesureInCycle(double value)
{
	constexpr double dRange = nMax - nMin;
	value = fmod(value, dRange);
	if (value >= nMax)
		return value - dRange;
	if (value < nMin)
		return value + dRange;
	return value;
}

/// @brief 确保角度在[0, 360.0)范围内
/// @param value 原角度
/// @return [0, 360.0)内角度
inline double makesureAngleRange(double value)
{
	return makesureInCycle<0, 360>(value);
}

/// @brief 求平方
/// @tparam T 数据类型
/// @param value 数值
/// @return 平方
template<typename T>
inline auto square(T value)
{
	return value * value;
}

/// @brief 四舍五入
/// @param dNum 原来的数字
/// @param nDecimalDigits 保留小数位数
/// @return 四舍五入后的数字
double DecimalRound(double dNum, int nDecimalDigits = 2);

}
}
