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
	//Ĭ�ϴ���������Դ����Ϊ1
	AsynchrousProc() { 
		m_vbMutexLock.clear(); 
		m_vnResourceNum.clear(); 
		m_vbMutexLock.push_back(false);
		m_vnResourceNum.push_back(0);
		m_nMutexResourceNumber = 1;
	}
	//��nMutexResourceNum��������ԴΪ��׼��ʼ���첽����
	AsynchrousProc(const unsigned int& nMutexResourceNum) { m_vbMutexLock.resize(nMutexResourceNum); m_vnResourceNum.resize(nMutexResourceNum); this->m_nMutexResourceNumber = nMutexResourceNum; }
	/*
		����˵������Ҫ���ʻ�����Դ�ĺ���������
				  ͳһ����Ϊvoid���ͣ�����ͨ�� �ṹ�� ����
		����һ:����ָ��
		������:��������(Ĭ���޲�)
		������:������Դ���
		������:�ٴ����󻥳���Դʱ��
	*/
	void MutexFunction(void (*func)(LPVOID), LPVOID param=0,int index=0, const int& cycletime = 10);
};