#include "stdafx.h"
#include "BasicCalc.h"

namespace xi
{
namespace math
{
double DecimalRound(double dNum, int nDecimalDigits)
{
	double dIntegerPart = floor(dNum);
	dNum -= dIntegerPart;
	for (int i = 0; i < nDecimalDigits; i++)
	{
		dNum *= 10;
	}
	dNum = floor(dNum + 0.5);
	for (int i = 0; i < nDecimalDigits; i++)
	{
		dNum /= 10;
	}
	return dIntegerPart + dNum;
}

}
}