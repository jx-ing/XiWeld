#pragma once
#include ".\Apps\PLib\ExLock\MyCriticalSection.h"
#include ".\OpenClass\COMM\OPC\OPCParam.h"

//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////
/////////////////////     �汾��     /////////////////////
const CString SoftwareVersion = "V2.1.0.0.23-Bate";
//////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////
//˵����
//1�����ļ�����ͬһ�������һЩȫ�ֱ���
//2����ֱֹ���Ա�����ʽ���ı��ļ����������жԱ��������������Ӧ�����������û�����
//3�����б�����ʼ�������������LoadAllBaseParam()������
//������
//1����������ز���
//2�����ص��Ա�־
//3���豸����
//4��OPCUA����
//5���Ͽڲ���
//6�����Ա���������
//7���豸����״̬
//8���˳�
//////////////////////////////////////////////////////////

//-------------���� ��ʼ -------------//
void LoadAllBaseParam();
CString GetSoftwareVersion();
extern int g_nEquipmentNo;
int GetEquipmentNo();
void SetEquipmentNo(int nNo);
//-------------���� ���� -------------//

//-------------���ص��� ��ʼ -------------//
extern BOOL g_bAutoGroupingMark;
extern BOOL g_bGantryEnableMark;
extern BOOL g_bLocalDebugMark;
extern BOOL g_bEnableOPCComm;
void LoadLocalDebugMark();
BOOL GetLocalDebugMark();
void LoadGantryEnableMark();
BOOL GetGantryEnableMark();
//-------------���ص��� ���� -------------//

//-------------�豸�ͺ� ��ʼ -------------//
extern int g_nDeviceType;
void LoadDeviceType();
int GetDeviceType();
//-------------�豸�ͺ� ���� -------------//

//-------------OPC UA���� ��ʼ -------------//
bool GetOPCConnectState(int nServerNo = 0);
void SetOPCConnectState(bool bOPCConnectState);
extern int g_nCMDID;
//-------------OPC UA���� ���� -------------//







