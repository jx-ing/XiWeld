#include "stdafx.h"
#include "WeldTrack.h"
#include "LocalFiles/ExLib/Vision/include/GFPGJointLib.h"

PartData::WeldTrack::WeldTrack()
{
}

PartData::WeldTrack::~WeldTrack()
{
}

void PartData::WeldTrack::initTrack(const std::vector<T_ROBOT_COORS>& vtInitWeldCoord)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	m_vtWeldCoord = vtInitWeldCoord;
	checkIsClosed();
}

bool PartData::WeldTrack::adjustTrack(const std::vector<T_POINT_3D>& vtNewPoints, int nBeginNo, int nEndNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	if (vtNewPoints.size() != m_vtWeldCoord.size())
	{
		return false;
	}
	if (nBeginNo > m_vtWeldCoord.size()
		|| nBeginNo > nEndNo)
	{
		return false;
	}
	if (nEndNo > m_vtWeldCoord.size())
	{
		nEndNo = m_vtWeldCoord.size() - 1;
	}
	for (size_t i = nBeginNo; i <= nEndNo; i++)
	{
		auto& tNewPnt = vtNewPoints[i];
		T_ROBOT_COORS& tOldCoord = m_vtWeldCoord[i];
		tOldCoord.dX = tNewPnt.x - tOldCoord.dBX;
		tOldCoord.dY = tNewPnt.y - tOldCoord.dBY;
		tOldCoord.dZ = tNewPnt.z - tOldCoord.dBZ;
	}
	return true;
}

size_t PartData::WeldTrack::getCount()
{
	CRITICAL_AUTO_LOCK(m_cLock);
	return m_vtWeldCoord.size();
}

T_ROBOT_COORS PartData::WeldTrack::getPoint(int nNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	return m_vtWeldCoord.at(nNo);
}

T_ROBOT_COORS PartData::WeldTrack::back()
{
	return m_vtWeldCoord.back();
}

void PartData::WeldTrack::erase(int nBeginNo, int nEndNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	std::vector<T_ROBOT_COORS> vtReturnTrack{};
	if (nBeginNo > m_vtWeldCoord.size()
		|| nBeginNo > nEndNo)
	{
		return;
	}
	if (nEndNo > m_vtWeldCoord.size())
	{
		nEndNo = m_vtWeldCoord.size() - 1;
	}
	m_vtWeldCoord.erase(m_vtWeldCoord.begin() + nBeginNo, m_vtWeldCoord.begin() + nEndNo);
	checkIsClosed();
	return;
}

std::vector<T_ROBOT_COORS> PartData::WeldTrack::getTrack(int nBeginNo, int nEndNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	std::vector<T_ROBOT_COORS> vtReturnTrack{};
	if (nBeginNo > m_vtWeldCoord.size()
		|| nBeginNo > nEndNo)
	{
		return vtReturnTrack;
	}
	if (nEndNo > m_vtWeldCoord.size())
	{
		nEndNo = m_vtWeldCoord.size() - 1;
	}
	vtReturnTrack.insert(vtReturnTrack.begin(), m_vtWeldCoord.begin() + nBeginNo, m_vtWeldCoord.begin() + nEndNo + 1);
	return vtReturnTrack;
}

std::vector<T_ROBOT_COORS> PartData::WeldTrack::getTrack()
{
	CRITICAL_AUTO_LOCK(m_cLock);
	return m_vtWeldCoord;
}

bool PartData::WeldTrack::getNearestCoordFast(T_ROBOT_COORS& tNearestCoord, T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);

	bool bRtn = false;
	double dThre = dMaxDis * dMaxDis;
	for (size_t i = nStartSearchNo; i < m_vtWeldCoord.size(); i++)
	{
		double dTemp = square(tAimCoord.dX + tAimCoord.dBX - m_vtWeldCoord[i].dX - m_vtWeldCoord[i].dBX)
			+ square(tAimCoord.dY + tAimCoord.dBY - m_vtWeldCoord[i].dY - m_vtWeldCoord[i].dBY)
			+ square(tAimCoord.dZ + tAimCoord.dBZ - m_vtWeldCoord[i].dZ - m_vtWeldCoord[i].dBZ);

		//小于等于阈值，说明当前点更近
		if (dTemp <= dThre)
		{
			dThre = dTemp;
			tNearestCoord = m_vtWeldCoord[i];
			bRtn = true;
		}
		//大于阈值并且已经找到过足够接近的点，认为之后的点不会更接近
		else if(bRtn)
		{
			return bRtn;
		}
	}

	return bRtn;
}

int PartData::WeldTrack::getNearestCoordNoFast(T_ROBOT_COORS tAimCoord, double dMaxDis, int nStartSearchNo)
{
	CRITICAL_AUTO_LOCK(m_cLock);

	int nNearestCoordNo = -1;
	bool bRtn = false;
	double dThre = dMaxDis * dMaxDis;
	for (size_t i = nStartSearchNo; i < m_vtWeldCoord.size(); i++)
	{
		double dTemp = square(tAimCoord.dX + tAimCoord.dBX - m_vtWeldCoord[i].dX - m_vtWeldCoord[i].dBX)
			+ square(tAimCoord.dY + tAimCoord.dBY - m_vtWeldCoord[i].dY - m_vtWeldCoord[i].dBY)
			+ square(tAimCoord.dZ + tAimCoord.dBZ - m_vtWeldCoord[i].dZ - m_vtWeldCoord[i].dBZ);

		//小于等于阈值，说明当前点更近
		if (dTemp <= dThre)
		{
			dThre = dTemp;
			nNearestCoordNo = i;
			bRtn = true;
		}
		//大于阈值并且已经找到过足够接近的点，认为之后的点不会更接近
		else if (bRtn)
		{
			return nNearestCoordNo;
		}
	}

	return nNearestCoordNo;
}

bool PartData::WeldTrack::saveTrack(CString sFilePath)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	FILE* file = nullptr;
	fopen_s(&file, sFilePath, "w");
	if (!file)
	{
		return false;
	}
	for (size_t i = 0; i < m_vtWeldCoord.size(); i++)
	{
		fprintf_s(file, 
			"%4zd %12.4lf %12.4lf %12.4lf "
			"%12.4lf %12.4lf %12.4lf "
			"%12.4lf %12.4lf %12.4lf\n", 
			i, m_vtWeldCoord[i].dX, m_vtWeldCoord[i].dY, m_vtWeldCoord[i].dZ, 
			m_vtWeldCoord[i].dRX, m_vtWeldCoord[i].dRY, m_vtWeldCoord[i].dRZ, 
			m_vtWeldCoord[i].dBX, m_vtWeldCoord[i].dBY, m_vtWeldCoord[i].dBZ);
	}
	fclose(file);
	return true;
}

bool PartData::WeldTrack::loadTrack(CString sFilePath)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	m_vtWeldCoord.clear();

	int nIndex = 0;
	T_ROBOT_COORS tCoord;
	FILE* file = nullptr;
	fopen_s(&file, sFilePath, "r");
	if (!file)
	{
		return false;
	}
	while (EOF != fscanf_s(file, "%d%lf%lf%lf%lf" "%lf%lf%lf%lf%lf", 
		&nIndex, &tCoord.dX, &tCoord.dY, &tCoord.dZ, &tCoord.dRX, 
		&tCoord.dRY, &tCoord.dRZ, &tCoord.dBX, &tCoord.dBY, &tCoord.dBZ))
	{
		m_vtWeldCoord.push_back(tCoord);
	}
	fclose(file);
	checkIsClosed();
	return true;
}

bool PartData::WeldTrack::saveTrackWorld(CString sFilePath)
{
	CRITICAL_AUTO_LOCK(m_cLock);
	FILE* file = nullptr;
	fopen_s(&file, sFilePath, "w");
	if (!file)
	{
		return false;
	}
	for (size_t i = 0; i < m_vtWeldCoord.size(); i++)
	{
		fprintf_s(file,
			"%4zd %12.4lf %12.4lf %12.4lf "
			"%12.4lf %12.4lf %12.4lf\n",
			i, m_vtWeldCoord[i].dX + m_vtWeldCoord[i].dBX,
			m_vtWeldCoord[i].dY + m_vtWeldCoord[i].dBY,
			m_vtWeldCoord[i].dZ + m_vtWeldCoord[i].dBZ,
			m_vtWeldCoord[i].dRX, m_vtWeldCoord[i].dRY, m_vtWeldCoord[i].dRZ);
	}
	fclose(file);
	return true;
}

bool PartData::WeldTrack::isClosed() const
{
	return m_bIsClosedTrack;
}

void PartData::WeldTrack::checkIsClosed()
{
	if (m_vtWeldCoord.size() < 2)
	{
		m_bIsClosedTrack = false;
		return;
	}
	auto dis = TwoPointDis(
		m_vtWeldCoord[0].dX + m_vtWeldCoord[0].dBX,
		m_vtWeldCoord[0].dY + m_vtWeldCoord[0].dBY,
		m_vtWeldCoord[0].dZ + m_vtWeldCoord[0].dBZ,
		m_vtWeldCoord.back().dX + m_vtWeldCoord.back().dBX,
		m_vtWeldCoord.back().dY + m_vtWeldCoord.back().dBY,
		m_vtWeldCoord.back().dZ + m_vtWeldCoord.back().dBZ);
	m_bIsClosedTrack = dis < 2.0;
}

