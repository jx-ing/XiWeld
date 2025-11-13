#pragma once
#include "stdafx.h"
#include <afxwin.h>
#include <atlstr.h>
#include <iostream>

using namespace std;
class AsynchrousProc
{
private:
	unsigned int m_nMutexResourceNumber;
	vector<bool> m_vbMutexLock;
	vector<int> m_vnResourceNum;
public:
	//默认创建互斥资源数量为1
	AsynchrousProc() { 
		m_vbMutexLock.clear(); 
		m_vnResourceNum.clear(); 
		m_vbMutexLock.push_back(false);
		m_vnResourceNum.push_back(0);
		m_nMutexResourceNumber = 1;
	}
	//以nMutexResourceNum个互斥资源为基准初始化异步处理
	AsynchrousProc(const unsigned int& nMutexResourceNum) { m_vbMutexLock.resize(nMutexResourceNum); m_vnResourceNum.resize(nMutexResourceNum); this->m_nMutexResourceNumber = nMutexResourceNum; }
	/*
		函数说明：需要访问互斥资源的函数走这里
				  统一声明为void类型，参数通过 结构体 传递
		参数一:函数指针
		参数二:函数参数(默认无参)
		参数三:互斥资源编号
		参数四:再次请求互斥资源时长
	*/
	void MutexFunction(void (*func)(LPVOID), LPVOID param=0,int index=0, const int& cycletime = 10);
};