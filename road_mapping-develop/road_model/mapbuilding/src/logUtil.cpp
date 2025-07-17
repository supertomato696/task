#include "logUtil.h"
FILE* logUtil::m_logFile = NULL;
mutex* logUtil::m_mtx = NULL;

logUtil::logUtil() = default;

logUtil::~logUtil() = default;

void logUtil::initLog(mutex* mtx) {
	char szExePath[512] = { 0 };
	CPLGetExecPath(szExePath, 512);
	string basePath = CPLGetPath(szExePath);
	basePath += "/mapbuilding.log";
	m_logFile = fopen(basePath.c_str(), "a");
	m_mtx = mtx;
}

void logUtil::log(const char* pszMsg) {
	if (pszMsg == NULL || m_logFile == NULL || m_mtx == NULL) {
		return;
	}
	time_t timestamp = time(0);
	tm* p = localtime(&timestamp);

	m_mtx->lock();
	fprintf(m_logFile,"%04d-%02d-%02d %02d:%02d:%02d %s\n", p->tm_year + 1900, p->tm_mon + 1, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec,pszMsg);
	fflush(m_logFile);
	m_mtx->unlock();
}
