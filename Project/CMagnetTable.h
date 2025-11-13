#pragma once
#include ".\Apps\PLib\PanasonicServo\AbsoluteEncoder.h"
class CMagnetTable :
	public CAbsoluteEncoder
{
public:
	CMagnetTable();
	~CMagnetTable();

	bool CMagnetTable::SetMagnetTableUP(int iMagnetNo);
	bool CMagnetTable::SetMagnetTableDown(int iMagnetNo);
	bool CMagnetTable::SetSupportUP(int iSupportNo);
	bool CMagnetTable::SetSupportDown(int iSupportNo);
	bool CMagnetTable::CheckMagnetNo(int iSupportNo);

protected:
	unsigned int Crc_Count(unsigned char pbuf[], unsigned char num);
	void Crc_Count(std::vector<uint8_t1> &vcbuf);
	bool CheckCrc_Count(std::vector<uint8_t1> vcbuf);
	long GetDecimal(std::vector<uint8_t1> vData);
	long GetMultipleSquare(long lData, int m);

private:
	std::vector<uint8_t1> SendData(uint8_t1 Adr, uint8_t1 Fun, uint8_t1 StartAdrH, uint8_t1 StartAdrL, uint8_t1 DataH, uint8_t1 DataL);
	bool RcvData(std::vector<uint8_t1> vcSendData, std::vector<uint8_t1> &vcData);
};

