#include "stdafx.h"
#include "BaseParam.h"


//---------------------参数定义---------------------//
int g_nEquipmentNo;

//T_DEPTH_CAMERA_TYPE_PARA g_tDepthCameraTypePara;//深度相机参数
BOOL g_bAutoGroupingMark;//自动分组使能
BOOL g_bGantryEnableMark;//龙门使能
BOOL g_bLocalDebugMark;//本地调试标志
BOOL g_bEnableOPCComm;//使能OPC通讯
int g_nDeviceType;//设备类型

int g_nCMDID = 0;
bool g_bOPCConnectState = false;
bool g_bAutoGrooveMark = false;
//--------------------------------------------------//

//----------------------互斥锁----------------------//
CMyCriticalSection g_cBHinfoLock;


//--------------------------------------------------//



void LoadAllBaseParam()
{
	LoadLocalDebugMark();
	LoadGantryEnableMark();
}

CString GetSoftwareVersion()
{
	return SoftwareVersion;
}

int GetEquipmentNo()
{
	return g_nEquipmentNo;
}

void SetEquipmentNo(int nNo)
{
	g_nEquipmentNo = nNo;
}

void LoadLocalDebugMark()
{
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");

	opini.ReadAddString("LocalDebugMark", &g_bLocalDebugMark, 1);
	opini.ReadAddString("EnableOPCComm", &g_bEnableOPCComm, true);

}

BOOL GetLocalDebugMark()
{
	return g_bLocalDebugMark;
}

void LoadGantryEnableMark()
{
	COPini opini;
	opini.SetFileName(DEBUG_INI);
	opini.SetSectionName("Debug");

	opini.ReadAddString("AutoGroupingMart", &g_bAutoGroupingMark, 0);
	opini.ReadAddString("GantryEnableMark", &g_bGantryEnableMark, 0);
}

BOOL GetGantryEnableMark()
{
	return g_bGantryEnableMark;
}

void LoadDeviceType()
{
	COPini cIni;
	cIni.SetFileName(SYSTEM_PARA_INI);
	cIni.SetSectionName("DeviceType");
	cIni.ReadAddString("DeviceType", &g_nDeviceType, 0);
}

int GetDeviceType()
{
	return g_nDeviceType;
}

void LoadAutoGrooveMark()
{
	COPini opini;
	opini.SetFileName(SYSTEM_PARA_INI);
	opini.SetSectionName("AutoGrooveMark");
	opini.ReadAddString("AutoGrooveMark", &g_bAutoGrooveMark, false);
}

bool GetAutoGrooveMark()
{
	return g_bAutoGrooveMark;
}

bool GetOPCConnectState(int nServerNo)
{
	return g_bOPCConnectState;
}

void SetOPCConnectState(bool bOPCConnectState)
{
	g_bOPCConnectState = bOPCConnectState;
}
