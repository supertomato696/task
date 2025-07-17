#pragma once
#define _CRT_SECURE_NO_WARNINGS
#include <stdlib.h>
#include <stdio.h>
#include <ctime>
#include <cstring>
#include "cpl_conv.h"
#include <thread>
#include <mutex>
using namespace std;

class logUtil {
public:
	logUtil();
	~logUtil();
private:
	static FILE* m_logFile;
	static mutex* m_mtx;
	 
public:
	static void initLog(mutex* mtx);

	static void log(const char* pszMsg);
};

