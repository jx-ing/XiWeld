#pragma once
#include <windows.h>

#define CRITICAL_AUTO_LOCK(cMyCriticalSection) CMyCriticalSectionLock cMyCriticalSectionLock(cMyCriticalSection.GetSection());

class CMyCriticalSection
{
public:
	CMyCriticalSection()
	{
		InitializeCriticalSection(&m_cSection);
	};
	virtual ~CMyCriticalSection()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			DeleteCriticalSection(&m_cSection);
		}
	};

	//功能：锁定
	//返回值：无
	void Lock()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			EnterCriticalSection(&m_cSection);
		}
	};

	//功能：解锁
	//返回值：无
	void UnLock()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			LeaveCriticalSection(&m_cSection);
		}
	};

	//功能：返回m_pSection
	//返回值：m_pSection
	CRITICAL_SECTION* GetSection()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			return &m_cSection;
		}
		return NULL;
	};

private:
	CRITICAL_SECTION m_cSection;
};

//class CMyCriticalSection
//{
//public:
//	CMyCriticalSection()
//	{
//		InitializeCriticalSection(&m_cSection);
//	}
//
//	void Lock()
//	{
//		EnterCriticalSection(&m_cSection);
//	}
//
//	void UnLock()
//	{
//		LeaveCriticalSection(&m_cSection);
//	}
//
//
//	//利用析构函数删除临界区对象
//	virtual ~CMyCriticalSection()
//	{
//			DeleteCriticalSection(&m_cSection);
//	}
//private:
//	CRITICAL_SECTION m_cSection;
//};

class CMyCriticalSectionLock
{
public:
	CMyCriticalSectionLock(CRITICAL_SECTION* pSection)
	{
		m_pSection = pSection;
		if (NULL != m_pSection && 0XCCCCCCCC != m_pSection->LockCount)
		{
			EnterCriticalSection(m_pSection);
		}
	};
	virtual ~CMyCriticalSectionLock()
	{
		if (NULL != m_pSection && 0XCCCCCCCC != m_pSection->LockCount)
		{
			LeaveCriticalSection(m_pSection);
		}
	};
private:
	CRITICAL_SECTION* m_pSection;
};
