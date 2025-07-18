#pragma once
#include ".\Apps\PLib\ExLock\MyCriticalSection.h"
#include ".\OpenClass\COMM\OPC\OPCParam.h"

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////////     版本号     /////////////////////
const CString SoftwareVersion = "V2.1.0.0.23-Bate";
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//说明：
//1、本文件用于同一管理调用一些全局变量
//2、禁止直接以变量形式更改本文件参数，所有对变量操作需添加相应函数，并做好互斥锁
//3、所有变量初始化加载需添加在LoadAllBaseParam()函数中
//参数：
//1、深度相机相关参数
//2、本地调试标志
//3、设备类型
//4、OPCUA变量
//5、料口参数
//6、绝对编码器方向
//7、设备运行状态
//8、退出
//////////////////////////////////////////////////////////

//-------------基础 开始 -------------//
void LoadAllBaseParam();
CString GetSoftwareVersion();
extern int g_nEquipmentNo;
int GetEquipmentNo();
void SetEquipmentNo(int nNo);
//-------------基础 结束 -------------//

//-------------本地调试 开始 -------------//
extern BOOL g_bAutoGroupingMark;
extern BOOL g_bGantryEnableMark;
extern BOOL g_bLocalDebugMark;
extern BOOL g_bEnableOPCComm;
void LoadLocalDebugMark();
BOOL GetLocalDebugMark();
void LoadGantryEnableMark();
BOOL GetGantryEnableMark();
//-------------本地调试 结束 -------------//

//-------------设备型号 开始 -------------//
extern int g_nDeviceType;
void LoadDeviceType();
int GetDeviceType();
//-------------设备型号 结束 -------------//

//-------------OPC UA变量 开始 -------------//
bool GetOPCConnectState(int nServerNo = 0);
void SetOPCConnectState(bool bOPCConnectState);
extern int g_nCMDID;
//-------------OPC UA变量 结束 -------------//







