#pragma once
#include "Apps/PLib/BasicFunc/ChoiceResources.h"

#if ENABLE_RFID

#include <iostream>
#include <VECTOR>
//using namespace std; 
//#using "Sygole.HFReader.dll"
//using namespace Sygole::HFReader;
//using namespace Sygole;

//#define byte unsigned char


enum CStatus_enum {
	//SUCCESS				= 0x00,//执行成功
	//FAILURE				= 0x80,//执行失败
	//NO_TAG_ERROR		= 0x90, //无标签错误
	//BCC_ERROR			= 0xA0,//BCC校验错误
	//BLOCK_SIZE_ERROR	= 0XB0, //按块写入操作时，块大小参数错误
	//TIMEOUT_ERROR		= 0XC0,//超时错误
	//GPO_PORT_ERROR		= 0XD0,//输出端口错误
	//NO_RESPONSE			= 0XF1, //无响应
	//PARAM_ERR			= 0XF2,//参数错误
	//SERIAL_CLOSED		= 0XF3 //通信端口关闭
};

enum CBaudEnum {

};

struct CTagInfo
{
	byte AFI;
	byte BlockCnt;
	byte BlockSize;
	byte DSFID;
	byte ICrefcerence;
	byte InformationFlag;
	std::vector<byte> UID;
};

struct CUserCfg
{

}; 

struct CCommCfg
{

}; 

struct CAutoReadPara
{

};


class CSygoleRFID
{
public:
	CSygoleRFID();
	
	void Dispose();//释放资源

	/*用途： 打开连接端口。
		参数：
		addr：用来指明与读写器连接的地址，可以是 IP 地址，可以是串口端口。
		Baud：用串口通讯时，可指定波特率。波特率一般为 115200 或者 9600，请以具体型
		号读写器的说明书为准。
		Port : 用来指明 TCP / IP 通讯是的端口号，读写器默认的端口号为 3001。若采用
		的外接协议转换器，则以实际设置的端口号为准。
		返回值：bool：打开成功返回 true；失败返回 false。*/

	bool Connect(std::string addr,int baud);//建立连接
	bool Connect(std::string addr , short port);//建立连接

	void DisConnect();//中断链接

	/*用途： 获取当前标签的唯一识别号。
		参数：
		ReaderID：读写器 ID。
		UID：所读取的标签 UID。*/
	int Inventory(byte ReaderID,  byte* UID);//获取当前标签的唯一识别号

	/*用途：以字节形式读取标签内存
		参数：
		ReaderID：读写器 ID
		addr：读取的起始地址
		len : 读取的字节长度
		datas : 保存数据缓冲区*/
	int ReadBytes(byte ReaderID, short addr, byte len, std::vector<byte>& datas);//以字节形式读取标签内存
	int ReadBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte> &datas);//以字节形式读取标签内存

	//读标签多个块。部分标签可能不支持或只支持少量的块。
	/*参数：
		高频读写器 使用手册 v2.1 广东思谷智能技术有限公司
		4
		ReaderID：读写器 ID
		StartBlock：所需要操作的第一个块号
		BlockCnt：所需要操作的块数量
		Datas：读取的数据
		Len : 读取到的数据长度
		返回值：Status_enum。（见第 5 章节 返回值信息）
		注：一次最多可以操作 8 个块。*/
	int ReadMBlock(byte ReaderID, byte StartBlock,byte BlockCnt, std::vector<byte> &datas, byte &len);

	/*用途：以字节形式对标签进行写操作
		参数：
		ReaderID : 读写器 ID
		addr : 写入的起始地址
		len : 写入的字节长度
		datas : 写入的数据
		off : 已写入的字节*/
	int WriteBytes(byte ReaderID, short addr, byte len, std::vector<byte>& datas, int off);
	int WriteBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte> &datas, int off);

	/*用途： 写标签多个块。最大块数量为8，块数越多，操作需要的时间越长，稳定性
		相应减弱。
		参数：
		ReaderID：读写器 ID
		StartBlock：所需要操作的起始块号
		BlockCnt：块数量
		BlockSize：块大小，一般来说块大小有 4 字节和 8 字节两种
		BlockDatas：需要写入标签的数据
		返回值：Status_enum。
		注：在不清楚块结构大小的情况下，建议使用字节操作方式读写数据。一次最多可以操作 8 个
		块。*/
	int WriteMBlock(byte ReaderID, byte StartBlock,byte BlockCnt, int BlockSize, std::vector<byte> &BlockDatas);

/*
	用途：获取当前标签的结构信息
		参数：
		ReaderID：读写器 ID
		Info：标签的系统信息，详细定于如下，各项参数具体含义与上述参数一致：
	返回值：Status_enum。*/
	int GetTagInfo(byte ReaderID, CTagInfo* Info);
	int GetTagInfo(std::string addrs, byte ReaderID, CTagInfo &Info);

/*
	用途： 获取标签的块安全状态，部分标签不支持。
		参数：
		ReaderID：读写器 ID
		StartBlock：所有获取的第一个块号
		BlockCnt：块数量
		TagSecurity：读取的块安全状态，0x00 表示未锁定，0x01 表示锁定
		返回值：Status_enum。*/
	int GetTagSecurity(byte ReaderID, byte StartBlock, byte BlockCnt, std::vector<byte> &TagSecurity);

	/*用途： 获取读写器的配置信息。
		参数：
		ReaderID：读写器 ID。
		Cfg：获取读写器的配置信息
		返回值：Status_enum。*/
	int GetUserCfg(byte ReaderID, CUserCfg &cfg);

	/*用途： 设置读写器的配置信息。
		参数：
		ReaderID：读写器 ID。
		Cfg：读写器的配置信息
		返回值：Status_enum。*/
	int SetUserCfg(byte ReaderID, CUserCfg cfg);

	/*用途： 获取读写器的通讯信息。
		参数：
		ReaderID：读写器 ID。
		Cfg：存储读写器的通讯参数信息；
		返回值：Status_enum。*/
	int GetCommCfg(byte ReaderID, CCommCfg &cfg);

	/*用途： 获取读写器的通讯信息。
		参数：
		ReaderID：读写器 ID。
		Cfg：读写器的通讯参数信息，设置是 MAC 参数项无效。
		返回值：Status_enum。*/
	int SetCommCfg(byte ReaderID, CCommCfg cfg);

	/*用途：获取读写器的软件版本号。
		参数：
		ReaderID：读写器 ID。
		SoftVer：读写器的软件版本号，显示时转化为字符串。
		Len：版本号的长度
		返回值：Status_enum。*/
	int GetSWVer(byte ReaderID, std::vector<byte> &SoftVer , byte& len);

	/*用途：GPO输出控制。
		参数：
		ReaderID：读写器 ID
		GPO：GPO 端口号
		connect：GPO 端口状态
		返回值：Status_enum。*/
	int GPOCtrl(byte ReaderID, byte GPO, bool bconnect);

	/*用途： 获取GPI状态。
		参数：
		ReaderID：读写器 ID
		GpiStatus：GPI 状态
		cnt : 端口数量
		返回值：Status_enum。*/
	int GetGpiStatus(byte ReaderID, byte& GpiStatus, byte& cnt);

	/*用途：设置读写器自动读卡的功能以及参数。
		参数：
		ReaderID：读写器 ID
		Para：自动读卡的参数
		返回值：Status_enum。*/
	int GetAutoReadFunc(byte ReaderID, CAutoReadPara para);
	
	/*用途：获取读写器自动读卡的功能以及参数。
		参数：
		ReaderID：读写器 ID
		Para：自动读卡的参数
		返回值：Status_enum。*/
	int GetAutoReadFunc(byte ReaderID, CAutoReadPara &para);

/*
	用途：设置读写器串口波特率
		参数：
		ReaderID：读写器 ID
		baud：串口波特率
		返回值：Status_enum*/
	int SetBaud(byte ReaderID, CBaudEnum baud);

	/*用途：保存配置
		参数：ReaderID：读写器 ID
		返回值：Status_enum。*/
	int SaveCfg(byte ReaderID);

/*
	用途： 恢复默认配置
		参数：ReaderID：读写器 ID
		返回值：Status_enum*/
	int DefaultCfg(byte ReaderID);

	/*用途：自劢读卡读取到标签时产生。
		参数：
		AutoReadEventArgs Args：自动读取产生事件时携带的参数，定义如下：
		public class AutoReadEventArgs : EventArgs
	{
		public byte[] UID;
		public AutoReadEventArgs();
		public Antenna_enum ant{ get; set; }
		public CommArgs comm{ get; set; }
	}
}
其中CommArgs的定义如下：
public class CommArgs
{
	public CommArgs();
	public string addr{ get; set; }
	public byte ReaderID{ get; set; }
}*/
//	AutoReadEventArgs AutoReadHandler();


/*
	用途：当有GPI输入时产生。
		参数：
		GPITriggerEventArgs Args：GPI 输入产生事件时携带的参数，定义如下：
		public class GPITriggerEventArgs : EventArgs
	{
		public GPITriggerEventArgs();
		public GpiEnum Gpi{ get; set; }
		public CommArgs comm{ get; set; }
	}
其中GpiEnum的定义如下:
	public enum GpiEnum
	{
		GPI_1 = 1,
		GPI_2 = 2,
		GPI_3 = 3,
		GPI_4 = 4,
	}*/
	//GPITriggerEventArgs GPITriggerHandler();
	void test();


};

#endif // ENABLE_RFID

