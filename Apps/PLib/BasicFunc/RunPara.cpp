#include "stdafx.h"
#include "RunPara.h"
#include ".\OpenClass\FileOP\ini\opini.h"

RunPara RunPara::m_instance;

RunPara::RunPara()
{
}

RunPara::~RunPara()
{
}

bool RunPara::load()
{
	bool bRtn = true;
	COPini opini;
	bRtn = bRtn && opini.SetFileName(".\\ConfigFiles\\RunPara.ini");

	//平缝测量参数
	bRtn = bRtn && opini.SetSectionName("FlatMeasure");
#define X_MACRO(name, type, var) bRtn = bRtn && opini.ReadString(name, &m_tFlatMeasure.var);
	MACRO_FLAT_MEASURE_RUN_PARA
#undef X_MACRO

	//平缝跟踪参数
	bRtn = bRtn && opini.SetSectionName("FlatTrack");
#define X_MACRO(name, type, var) bRtn = bRtn && opini.ReadString(name, &m_tFlatTrack.var);
	MACRO_FLAT_TRACK_RUN_PARA
#undef X_MACRO

	//平缝焊接参数
	bRtn = bRtn && opini.SetSectionName("FlatWeld");
#define X_MACRO(name, type, var) bRtn = bRtn && opini.ReadString(name, &m_tFlatWeld.var);
	MACRO_FLAT_WELD_RUN_PARA
#undef X_MACRO

	//立缝测量参数
	bRtn = bRtn && opini.SetSectionName("VerMeasure");
#define X_MACRO(name, type, var) bRtn = bRtn && opini.ReadString(name, &m_tVerMeasure.var);
	MACRO_VER_MEASURE_RUN_PARA
#undef X_MACRO

	//立缝焊接参数
	bRtn = bRtn && opini.SetSectionName("VerWeld");
#define X_MACRO(name, type, var) bRtn = bRtn && opini.ReadString(name, &m_tVerWeld.var);
	MACRO_VER_WELD_RUN_PARA
#undef X_MACRO

	return bRtn;
}
