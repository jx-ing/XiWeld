#include "stdafx.h"
#include "BaseParam.h"


//---------------------��������---------------------//
int g_nEquipmentNo;

//T_DEPTH_CAMERA_TYPE_PARA g_tDepthCameraTypePara;//����������
BOOL g_bAutoGroupingMark;//�Զ�����ʹ��
BOOL g_bGantryEnableMark;//����ʹ��
BOOL g_bLocalDebugMark;//���ص��Ա�־
BOOL g_bEnableOPCComm;//ʹ��OPCͨѶ
int g_nDeviceType;//�豸����

int g_nCMDID = 0;
bool g_bOPCConnectState = false;
bool g_bAutoGrooveMark = false;
//--------------------------------------------------//

//----------------------������----------------------//
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
