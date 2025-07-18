#include "stdafx.h"
#include "CSygoleRFID.h"

#if ENABLE_RFID

#using "Sygole.HFReader.dll"

using namespace Sygole;
using namespace Sygole::HFReader;
using namespace System;


CSygoleRFID::CSygoleRFID()
{
	//(Sygole::HFReader::HFReader^)api1 = gcnew Sygole::HFReader::HFReader();
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	api->DisConnect();
}

void CSygoleRFID::Dispose()
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
//	api->Dispose();
}

bool CSygoleRFID::Connect(std::string addr, int baud)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	String^ str = gcnew String(addr.c_str());
	return api->Connect(str, baud);
}

bool CSygoleRFID::Connect(std::string addr, short port)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	String^ str = gcnew String(addr.c_str());
	return api->Connect(str, port);;
}

void CSygoleRFID::DisConnect()
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	api->DisConnect();
}

int CSygoleRFID::ReadBytes(byte ReaderID, short addr, byte len, std::vector<byte> &datas)
{
	cli::array<unsigned char>^ Arrdatas = gcnew cli::array<unsigned char>(len);
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	api->Connect("192.168.1.10", 3001);
	Status_enum eStatus = api->ReadBytes(ReaderID, addr, len, Arrdatas);
	api->DisConnect();
	datas.clear();
	for (int i = 0; i < len; i++)
	{
		datas.push_back(byte());
		datas[i] = Arrdatas[i];
	}
	api->DisConnect();
	return (int)eStatus;

}

int CSygoleRFID::ReadBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte>& datas)
{
	cli::array<unsigned char>^ Arrdatas = gcnew cli::array<unsigned char>(len);
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	String^ str = gcnew String(addrs.c_str());
	if (api->Connect(str, 3001) == false)
	{
		return -100;
	}
	Status_enum eStatus = api->ReadBytes(ReaderID, addr, len, Arrdatas);
	api->DisConnect();
	datas.clear();
	for (int i = 0; i < len; i++)
	{
		datas.push_back(byte());
		datas[i] = Arrdatas[i];
	}
	api->DisConnect();
	return (int)eStatus;
}

int CSygoleRFID::ReadMBlock(byte ReaderID, byte StartBlock, byte BlockCnt, std::vector<byte> &datas, byte & len)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = datas.size();
	cli::array<unsigned char>^ Arrdatas = gcnew cli::array<unsigned char>(nNum);
	for (int i = 0; i < datas.size(); i++)
	{
		Arrdatas[i] = datas[i];
	}
	Status_enum eStatus = api->ReadMBlock(ReaderID, StartBlock, BlockCnt, Arrdatas,len);
	return (int)eStatus;
}

int CSygoleRFID::WriteBytes(byte ReaderID, short addr, byte len, std::vector<byte> &datas,  int off)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = datas.size();
	cli::array<unsigned char>^ Arrdatas = gcnew cli::array<unsigned char>(nNum);
	api->Connect("192.168.1.10", 3001);
	for (int i = 0; i < datas.size(); i++)
	{
		Arrdatas[i] = datas[i];
	}
	Status_enum eStatus = api->WriteBytes(ReaderID, addr, len, Arrdatas, off);
	api->DisConnect();
	return (int)eStatus;
}

int CSygoleRFID::WriteBytes(std::string addrs, byte ReaderID, short addr, byte len, std::vector<byte>& datas, int off)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = datas.size();
	cli::array<unsigned char>^ Arrdatas = gcnew cli::array<unsigned char>(nNum);
	String^ str = gcnew String(addrs.c_str());
	api->Connect(str, 3001);
	for (int i = 0; i < datas.size(); i++)
	{
		Arrdatas[i] = datas[i];
	}
	Status_enum eStatus = api->WriteBytes(ReaderID, addr, len, Arrdatas, off);
	api->DisConnect();
	return (int)eStatus;
}

int CSygoleRFID::WriteMBlock(byte ReaderID, byte StartBlock, byte BlockCnt, int BlockSize, std::vector<byte> &BlockDatas)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = BlockDatas.size();
	cli::array<unsigned char>^ ArrBlockDatas = gcnew cli::array<unsigned char>(nNum);
	for (int i = 0; i < BlockDatas.size(); i++)
	{
		ArrBlockDatas[i] = BlockDatas[i];
	}
	Status_enum eStatus = api->WriteMBlock( ReaderID,  StartBlock,  BlockCnt,  BlockSize, ArrBlockDatas);
	return (int)eStatus;
}

int CSygoleRFID::GetTagInfo(byte ReaderID, CTagInfo * Info)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	TagInfo^ tagInfo = gcnew Sygole::HFReader::TagInfo();
	api->Connect("192.168.1.11", 3001);
	Status_enum eStatus = api->GetTagInfo(ReaderID, tagInfo);
	api->DisConnect();
	Info->AFI = TagInfo().AFI;
	Info->BlockCnt = TagInfo().BlockCnt;
	Info->BlockSize = TagInfo().BlockSize;
	Info->DSFID = TagInfo().DSFID;
	Info->ICrefcerence = TagInfo().ICrefcerence;
	Info->InformationFlag = TagInfo().InformationFlag;
	for (int i = 0; i < TagInfo().UID->Length-1; i++)
	{
		Info->UID.push_back(byte());
		Info->UID[i] = TagInfo().UID[i];
	}
	
	return (int)eStatus;
}
int CSygoleRFID::GetTagInfo(std::string addrs, byte ReaderID, CTagInfo& Info)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	TagInfo^ tagInfo = gcnew Sygole::HFReader::TagInfo();
	String^ str = gcnew String(addrs.c_str());
	api->Connect(str, 3001);
	Status_enum eStatus = api->GetTagInfo(ReaderID, tagInfo);
	api->DisConnect();
	Info.AFI = TagInfo().AFI;
	Info.BlockCnt = TagInfo().BlockCnt;
	Info.BlockSize = TagInfo().BlockSize;
	Info.DSFID = TagInfo().DSFID;
	Info.ICrefcerence = TagInfo().ICrefcerence;
	Info.InformationFlag = TagInfo().InformationFlag;
	byte b1;
	Info.UID.clear();
	for (int i = 0; i < TagInfo().UID->Length; i++)
	{
		b1 = (byte)TagInfo().UID[i];
		Info.UID.push_back(b1);
	}
	return (int)eStatus;
}
int CSygoleRFID::GetTagSecurity(byte ReaderID, byte StartBlock, byte BlockCnt, std::vector<byte> &TagSecurity)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = TagSecurity.size();
	cli::array<unsigned char>^ ArrTagSecurity = gcnew cli::array<unsigned char>(nNum);
	for (int i = 0; i < TagSecurity.size(); i++)
	{
		ArrTagSecurity[i] = TagSecurity[i];
	}
	Status_enum eStatus = api->GetTagSecurity(ReaderID, StartBlock, BlockCnt, ArrTagSecurity);
	return (int)eStatus;
}

int CSygoleRFID::GetUserCfg(byte ReaderID, CUserCfg &cfg)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	UserCfg ^userCfg = gcnew Sygole::HFReader::UserCfg();
	Status_enum eStatus = api->GetUserCfg(ReaderID, userCfg);
	return (int)eStatus;

}

int CSygoleRFID::SetUserCfg(byte ReaderID, CUserCfg cfg)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	UserCfg ^userCfg = gcnew Sygole::HFReader::UserCfg();
	Status_enum eStatus = api->SetUserCfg(ReaderID, userCfg);
	return (int)eStatus;
}

int CSygoleRFID::GetCommCfg(byte ReaderID, CCommCfg &cfg)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	CommCfg ^commCfg = gcnew Sygole::HFReader::CommCfg();
	Status_enum eStatus = api->GetCommCfg(ReaderID, commCfg);
	return (int)eStatus;
}

int CSygoleRFID::SetCommCfg(byte ReaderID, CCommCfg cfg)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	CommCfg ^commCfg = gcnew Sygole::HFReader::CommCfg();
	Status_enum eStatus = api->SetCommCfg(ReaderID, commCfg);
	return (int)eStatus;
}

int CSygoleRFID::GetSWVer(byte ReaderID, std::vector<byte> &SoftVer, byte &len)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	static int nNum = SoftVer.size();
	cli::array<unsigned char>^ ArrSoftVer = gcnew cli::array<unsigned char>(nNum);
	for (int i = 0; i < SoftVer.size(); i++)
	{
		ArrSoftVer[i] = SoftVer[i];
	}
	
	Status_enum eStatus = api->GetSWVer(ReaderID, ArrSoftVer, len);
	return (int)eStatus;
}

int CSygoleRFID::GPOCtrl(byte ReaderID, byte GPO, bool bconnect)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();

	Status_enum eStatus = api->GPOCtrl(ReaderID, GPO, bconnect);
	return (int)eStatus;
}

int CSygoleRFID::GetGpiStatus(byte ReaderID, byte &GpiStatus, byte &cnt)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	Status_enum eStatus = api->GetGpiStatus(ReaderID, GpiStatus, cnt);
	return (int)eStatus;
}

int CSygoleRFID::GetAutoReadFunc(byte ReaderID, CAutoReadPara para)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	AutoReadPara ^autoReadPara = gcnew Sygole::HFReader::AutoReadPara();
	Status_enum eStatus = api->GetAutoReadFunc(ReaderID, autoReadPara);
	return (int)eStatus;
}

int CSygoleRFID::GetAutoReadFunc(byte ReaderID, CAutoReadPara & para)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	AutoReadPara ^autoReadPara = gcnew Sygole::HFReader::AutoReadPara();
	Status_enum eStatus = api->GetAutoReadFunc(ReaderID, autoReadPara);
	return (int)eStatus;
}

int CSygoleRFID::SetBaud(byte ReaderID, CBaudEnum baud)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	BaudEnum baudEnum = (BaudEnum)((int)baud);
	Status_enum eStatus = api->SetBaud(ReaderID, baudEnum);
	return (int)eStatus;
}

int CSygoleRFID::SaveCfg(byte ReaderID)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	//Status_enum eStatus = api->SaveCfg(ReaderID);
	//return (int)eStatus;
	return 1;
}

int CSygoleRFID::DefaultCfg(byte ReaderID)
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	Status_enum eStatus = api->DefaultCfg(ReaderID);
	return (int)eStatus;
	return 0;
}







void CSygoleRFID::test()
{
	Sygole::HFReader::HFReader^ api = gcnew Sygole::HFReader::HFReader();
	api->Connect("195", 3001);
	
}








#endif // ENABLE_RFID