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

bool RunPara::loadPara(bool bCheck, CString sFileName)
{
	bool bRtn = true;
	COPini opini;
	bRtn = bRtn && opini.SetFileName(bCheck, sFileName);

	//系统参数
	bRtn = opini.SetSectionName("System") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tSystem.var) && bRtn;
	MACRO_SYSTEM_RUN_PARA;
#undef X_MACRO

	//埃斯顿参数
	bRtn = opini.SetSectionName("Estun") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tEstun.var) && bRtn;
	MACRO_ESTUN_RUN_PARA;
#undef X_MACRO

	//焊缝参数
	bRtn = opini.SetSectionName("WeldSeam") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tWeldSeam.var) && bRtn;
	MACRO_WELD_SEAM_RUN_PARA;
#undef X_MACRO

	//识别参数
	bRtn = opini.SetSectionName("Recognition") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tRecognition.var) && bRtn;
	MACRO_RECOGNITION_RUN_PARA;
#undef X_MACRO

	//平缝测量参数
	bRtn = opini.SetSectionName("FlatMeasure") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tFlatMeasure.var) && bRtn;
	MACRO_FLAT_MEASURE_RUN_PARA;
#undef X_MACRO

	//平缝跟踪参数
	bRtn = opini.SetSectionName("FlatTrack") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tFlatTrack.var) && bRtn;
	MACRO_FLAT_TRACK_RUN_PARA;
#undef X_MACRO

	//平缝焊接参数
	bRtn = opini.SetSectionName("FlatWeld") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tFlatWeld.var) && bRtn;
	MACRO_FLAT_WELD_RUN_PARA;
#undef X_MACRO

	//立缝测量参数
	bRtn = opini.SetSectionName("VerMeasure") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tVerMeasure.var) && bRtn;
	MACRO_VER_MEASURE_RUN_PARA;
#undef X_MACRO

	//立缝焊接参数
	bRtn = opini.SetSectionName("VerWeld") && bRtn;
#define X_MACRO(type, var) bRtn = opini.ReadString(bCheck, #var, &m_tVerWeld.var) && bRtn;
	MACRO_VER_WELD_RUN_PARA;
#undef X_MACRO

	return bRtn;
}

bool RunPara::loadAllPara()
{
	bool bRtn = loadPara(true, ".\\ConfigFiles\\RunPara.ini");
	if (CheckFileExists(m_sCustomParaFileName, false))
		loadPara(false, m_sCustomParaFileName);
	return bRtn;
}

bool SetCurLineScanFolder(int nTableNo)
{
	switch (nTableNo)
	{
	case 0:
		PARA_RECOGNITION(sLineScanFolder) = ".\\LineScan\\GantryLeft";
		break;
	case 1:
		PARA_RECOGNITION(sLineScanFolder) = ".\\LineScan\\GantryRight";
		break;
	default:
		return false;
	}

	bool bRtn = true;
	COPini opini;
	bRtn = bRtn && opini.SetFileName(".\\ConfigFiles\\RunPara.ini");
	bRtn = bRtn && opini.SetSectionName("Recognition");
	bRtn = bRtn && opini.WriteString("sLineScanFolder", PARA_RECOGNITION(sLineScanFolder));
	return bRtn;
}

CString GetCurLineScanPointCloudFile()
{
	return LINE_SCAN_POINT_CLOUD;
	return PARA_RECOGNITION(sLineScanFolder) + "\\PointCloud\\Scan5D.txt";
}

bool SetLineScanTimes(int nLineScanTimes)
{
	PARA_RECOGNITION(nLineScanTimes) = nLineScanTimes;
	bool bRtn = true;
	COPini opini;
	bRtn = bRtn && opini.SetFileName(".\\ConfigFiles\\RunPara.ini");
	bRtn = bRtn && opini.SetSectionName("Recognition");
	bRtn = bRtn && opini.WriteString("nLineScanTimes", PARA_RECOGNITION(nLineScanTimes));
	return bRtn;
}
