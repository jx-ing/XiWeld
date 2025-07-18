#pragma once
#include "CalPlaneVector.h"
#include "Database_mysql.h"
#include <typeinfo>

#if ENABLE_MY_SQL

#define STATION_LINK "-"

#define VAL_NAME(val) (#val)
#define VAL_TYPE(val) (typeid(val).name())
#define VAL_INFO(val) val, VAL_NAME(val), VAL_TYPE(val)
#define VAL_ARRAY_NAME(val, index) (GetStr("%s[%d]", #val,index ))
#define VAL_ARRAY_INFO(val, index) val[index], VAL_ARRAY_NAME(val, index), VAL_TYPE(val)

//生成节点变量
#define GET_STATION_1(station) GetStr("%s", ToCString(station)[0])
#define GET_STATION_2(station1, station2) GetStr("%s-%s", ToCString(station1)[0], ToCString(station2)[0])
#define GET_STATION_3(station1, station2, station3) GetStr("%s-%s-%s", ToCString(station1)[0], ToCString(station2)[0], ToCString(station3)[0])

//按照节点设置变量值
#define SET_COMMON_STATION_PARA(val, station) g_CommonPara.SetCommonPara(ToCommonTable(station, VAL_INFO(val)))
#define GET_COMMON_STATION_PARA(val, station) GetCommonVal(station, VAL_INFO(val))
#define CHANGE_COMMON_STATION_PARA(val, newval,station) {val = newval;g_CommonPara.SetCommonPara(ToCommonTable(station, VAL_INFO(val)));}

//按照节点设置数组变量值
#define SET_COMMON_STATION_ARRAY_PARA(val, index, station) g_CommonPara.SetCommonPara(ToCommonTable(station, VAL_ARRAY_INFO(val, index)))
#define GET_COMMON_STATION_ARRAY_PARA(val, index, station) GetCommonVal(station, VAL_ARRAY_INFO(val, index))
#define CHANGE_COMMON_STATION_ARRAY_PARA(val, index, newval,station) {val[index] = newval;g_CommonPara.SetCommonPara(ToCommonTable(station, VAL_ARRAY_INFO(val, index)));}

//直接设置变量值
#define SET_COMMON_PARA(val) SET_COMMON_STATION_PARA(val, "null")
#define GET_COMMON_PARA(val) GET_COMMON_STATION_PARA(val, "null")
#define CHANGE_COMMON_PARA(val, newval) CHANGE_COMMON_STATION_PARA(val, newval, "null");

//直接设置数组变量值
#define SET_COMMON_ARRAY_PARA(val, index) SET_COMMON_STATION_ARRAY_PARA(val, index, "null")
#define GET_COMMON_ARRAY_PARA(val, index) GET_COMMON_STATION_ARRAY_PARA(val, index, "null")
#define CHANGE_COMMON_ARRAY_PARA(val, index, newval) CHANGE_COMMON_STATION_ARRAY_PARA(val, index, newval, "null");

//修改节点说明
#define CHANGE_STATION_PARA_DESCRIBE(station, describe) g_CommonPara.UpdateStationDescribe(station, describe)
#define CHANGE_PARA_DESCRIBE(station, val, describe) g_CommonPara.UpdateStationDescribe(station, VAL_NAME(val), describe)
#define CHANGE_ARRAY_PARA_DESCRIBE(station, val, index, describe) g_CommonPara.UpdateStationDescribe(station, VAL_ARRAY_NAME(val, index), describe)

//创建变量
#define CREATE_COMMON_PARA(val, describe) if(false == GET_COMMON_PARA(val))SET_COMMON_PARA(val);CHANGE_PARA_DESCRIBE("null", val, describe);
#define CREATE_COMMON_ARRAY_PARA(val, index, describe) if(false == GET_COMMON_ARRAY_PARA(val, index))GET_COMMON_ARRAY_PARA(val, index);CHANGE_ARRAY_PARA_DESCRIBE("null", val, index, describe);
#define CREATE_COMMON_STATION_PARA(val, station) if(false == GET_COMMON_STATION_PARA(val, station))SET_COMMON_STATION_PARA(val, station);
#define CREATE_COMMON_STATION_PARA_DESCRIBE(val, station, describe) if(false == GET_COMMON_STATION_PARA(val, station))SET_COMMON_STATION_PARA(val, station);CHANGE_PARA_DESCRIBE(station, val, describe);
#define CREATE_COMMON_STATION_ARRAY_PARA_DESCRIBE(val, index, station, describe) if(false == GET_COMMON_STATION_ARRAY_PARA(val, index, station))SET_COMMON_STATION_ARRAY_PARA(val, index, station);CHANGE_ARRAY_PARA_DESCRIBE(station, val, index, describe);


//通用参数表参数最大尺寸
#define COMMON_TABLE_DATA_SIZE 9
typedef struct
{
	int nID;
    CString strParaStation;//参数节点
    CString strParaName;//参数名
    CString strParaType;//参数类型
    CString strParaDescribe;//参数描述
    CString astrPara[COMMON_TABLE_DATA_SIZE];//参数
    CString astrParaBackups[COMMON_TABLE_DATA_SIZE];//参数备份
}T_COMMON_TABLE;//通用表

class CCommonParaByDatabase :
    public Database_mysql
{
public:
    CCommonParaByDatabase(CString strTableName = "common_para_table");
    ~CCommonParaByDatabase();

    //**********************************************//
    //*											   *//
    //***************** 通用参数表 *****************//
    //*											   *//
    //**********************************************//
    BOOL CreatCommonParaTable(); //创建通用参数表
    BOOL ReadCommonPara(CString strStation, std::vector<T_COMMON_TABLE>& vtCommonPara);//按节点读取
    BOOL ReadCommonPara(std::vector<T_COMMON_TABLE>& vtCommonPara);//读取所有数据
    BOOL ReadCommonPara(T_COMMON_TABLE& tCommonPara);//读取
    BOOL InsertCommonPara(T_COMMON_TABLE tCommonPara);//添加
    BOOL UpdateCommonPara(T_COMMON_TABLE tCommonPara);//更新
    BOOL SetCommonPara(T_COMMON_TABLE tCommonPara);//设置（没有添加，存在更新）
    BOOL UpdateStationDescribe(CString strStation, CString strDescribe);//更新节点说明
    BOOL UpdateStationDescribe(CString strStation, CString strName, CString strDescribe);//更新变量说明
    CString GetTableName();
private:
    CString m_strTableName = "common_para_table";//数据库表名
    CMyCriticalSection m_cDatabaseLock;
};

extern CCommonParaByDatabase g_CommonPara;



template <class T >
bool GetCommonVal(CString strStation, T &val, CString strName, CString strType)
{
    T_COMMON_TABLE tCommonPara; 
    tCommonPara.strParaName = strName;
    tCommonPara.strParaStation = strStation;
    tCommonPara.strParaType = strType;
    if (TRUE == g_CommonPara.ReadCommonPara(tCommonPara))
        return CStringTo(SetPara(tCommonPara), val);
    else
        return false;
}

template <class T >
T_COMMON_TABLE ToCommonTable(CString strStation, T val, CString strName, CString strType, CString strDescribe = "null")
{
    return GetPara(strStation, strName, strType, ToCString(val), strDescribe);
};

std::vector<CString> SetPara(T_COMMON_TABLE tCommonPara);
T_COMMON_TABLE GetPara(CString strStation, CString strName, CString strType, std::vector<CString> strPara, CString strDescribe = "null");

std::vector<CString> ToCString(int val);
std::vector<CString> ToCString(long val);
std::vector<CString> ToCString(long long val);
std::vector<CString> ToCString(bool val);
std::vector<CString> ToCString(float val);
std::vector<CString> ToCString(double val);
std::vector<CString> ToCString(char* val);
std::vector<CString> ToCString(std::string val);
std::vector<CString> ToCString(CString val);
std::vector<CString> ToCString(T_ROBOT_COORS val);
std::vector<CString> ToCString(T_ANGLE_PULSE val);
std::vector<CString> ToCString(T_ROBOT_MOVE_SPEED val);
std::vector<CString> ToCString(CvPoint3D64f val);
std::vector<CString> ToCString(T_ENTITY_RECT val);


bool CStringTo(std::vector<CString> str, int& val);
bool CStringTo(std::vector<CString> str, long& val);
bool CStringTo(std::vector<CString> str, long long& val);
bool CStringTo(std::vector<CString> str, bool& val);
bool CStringTo(std::vector<CString> str, float& val);
bool CStringTo(std::vector<CString> str, double& val);
bool CStringTo(std::vector<CString> str, char* val);
bool CStringTo(std::vector<CString> str, std::string& val);
bool CStringTo(std::vector<CString> str, CString& val);
bool CStringTo(std::vector<CString> str, T_ROBOT_COORS& val);
bool CStringTo(std::vector<CString> str, T_ANGLE_PULSE& val);
bool CStringTo(std::vector<CString> str, T_ROBOT_MOVE_SPEED& val);
bool CStringTo(std::vector<CString> str, CvPoint3D64f& val);
bool CStringTo(std::vector<CString> str, T_ENTITY_RECT& val);

#endif


