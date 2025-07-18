#include "StdAfx.h"
#include "AsynchrousProc.h"
#include <vector>

void AsynchrousProc::MutexFunction(void (* Func)(LPVOID),LPVOID param, int index, const int& Cycletime)
{
    while ( m_vbMutexLock[index] == true)
    {
        Sleep(Cycletime);
    }
    m_vnResourceNum[index]++;   //���ܴ��ڻ���������ʱ����û�ͬʱ���ʣ���˳��ʹ�û�����Դ
    m_vbMutexLock[index] = true;
    while (m_vnResourceNum[index] != 0)
    {
        (*Func)(param);
        m_vnResourceNum[index]--;
    }
    m_vbMutexLock[index] = false;
    return;
}