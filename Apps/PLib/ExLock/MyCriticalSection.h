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

	//���ܣ�����
	//����ֵ����
	void Lock()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			EnterCriticalSection(&m_cSection);
		}
	};

	//���ܣ�����
	//����ֵ����
	void UnLock()
	{
		if (0XCCCCCCCC != m_cSection.LockCount)
		{
			LeaveCriticalSection(&m_cSection);
		}
	};

	//���ܣ�����m_pSection
	//����ֵ��m_pSection
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
//	//������������ɾ���ٽ�������
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
