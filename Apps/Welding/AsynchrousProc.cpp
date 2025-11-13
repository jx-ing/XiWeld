#include "StdAfx.h"
#include "AsynchrousProc.h"
#include <vector>

void AsynchrousProc::MutexFunction(void (* Func)(LPVOID),LPVOID param, int index, const int& Cycletime)
{
    while ( m_vbMutexLock[index] == true)
    {
        Sleep(Cycletime);
    }
    m_vnResourceNum[index]++;   //可能存在互斥锁放行时多个用户同时访问，则顺序使用互斥资源
    m_vbMutexLock[index] = true;
    while (m_vnResourceNum[index] != 0)
    {
        (*Func)(param);
        m_vnResourceNum[index]--;
    }
    m_vbMutexLock[index] = false;
    return;
}