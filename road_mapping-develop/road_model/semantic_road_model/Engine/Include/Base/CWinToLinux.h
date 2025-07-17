/******************************************************************
作者: test
日期: 2021-8-18 11:19
******************************************************************/
#ifndef _CWINTOlINUX_H
#define _CWINTOlINUX_H

#include <stdio.h>
#include <stdlib.h>

#include <string>
using namespace std;

#ifdef _WIN32
#include <windows.h>
#include <io.h>

#define snprintf sprintf_s

#else
#include <sys/time.h>
#include <sys/io.h>
#include <time.h>
#include <string.h>
#endif

#ifndef _WIN32
#include <sys/stat.h>

typedef unsigned char BYTE;
typedef long long __int64;

inline int _mkdir(const char *_sDir)
{
	int status = ::mkdir(_sDir, 0777);
	if (status >= 0)
	{
		chmod(_sDir, 0777);
	}
	return status;
}
// ��ȡ��ǰ����ֱ��.exe�ľ���·�����ַ���
inline int GetModuleFileName(char *sFileName, int nSize)
{
	int ret = -1;
	char sLine[1024] = {0};
	void *pSymbol = (void *)"";
	FILE *fp;
	char *pPath;

	fp = fopen("/proc/self/maps", "r");
	if (fp != NULL)
	{
		while (!feof(fp))
		{
			unsigned long start, end;

			if (!fgets(sLine, sizeof(sLine), fp))
				continue;
			if (!strstr(sLine, " r-xp ") || !strchr(sLine, '/'))
				continue;

			sscanf(sLine, "%lx-%lx ", &start, &end);
			if (pSymbol >= (void *)start && pSymbol < (void *)end)
			{
				char *tmp;
				// size_t len;

				/* Extract the filename; it is always an absolute path */
				pPath = strchr(sLine, '/');

				/* Get rid of the newline */
				tmp = strrchr(pPath, '\n');
				if (tmp)
					*tmp = 0;

				ret = 0;
				// strcpy(sFileName, pPath);
				string strorigingps(pPath);
				string::size_type nPos = strorigingps.find_last_of("\\/");
				if (nPos != string::npos)
				{
					string sExePath = strorigingps.substr(0, nPos);
					strcpy(sFileName, sExePath.c_str());
					break;
				}
			}
		}
		fclose(fp);
	}

	printf("%s\n", sFileName);
	return ret;
}
//////////////////////////
inline void _split_whole_name(const char *whole_name, char *fname, char *ext)
{
	const char *p_ext;

	p_ext = rindex(whole_name, '.');
	if (NULL != p_ext)
	{
		strcpy(ext, p_ext);
		snprintf(fname, p_ext - whole_name + 1, "%s", whole_name);
	}
	else
	{
		ext[0] = '\0';
		strcpy(fname, whole_name);
	}
}
/////////////�ع�windows  _splitpath_s ����
inline void _splitpath_s(const char *path, char *drive, char *dir, char *fname, char *ext)
{
	const char *p_whole_name;

	drive[0] = '\0';
	if (NULL == path)
	{
		dir[0] = '\0';
		fname[0] = '\0';
		ext[0] = '\0';
		return;
	}

	if ('/' == path[strlen(path)])
	{
		strcpy(dir, path);
		fname[0] = '\0';
		ext[0] = '\0';
		return;
	}

	p_whole_name = rindex(path, '/');
	if (NULL != p_whole_name)
	{
		p_whole_name++;
		_split_whole_name(p_whole_name, fname, ext);

		snprintf(dir, p_whole_name - path, "%s", path);
	}
	else
	{
		_split_whole_name(path, fname, ext);
		dir[0] = '\0';
	}
}

#endif

//////////////���ļ�
inline int OpenFile(FILE *&_File, const char *_Filename, const char *_Mode)
{
	int errID = 0;
#ifdef _WIN32
	errID = fopen_s(&_File, _Filename, _Mode);

#else
	_File = fopen(_Filename, _Mode);
	if (_File == NULL)
		errID = 1;
#endif
	if (errID != 0)
	{
		printf("can't open file %s\n", _Filename);
	}
	return errID;
}
///////////////��ȡ��ǰʱ��
inline void GetCurrentDateTime(string &_szCurrentDateTime)
{
	char szCurrentDateTime[32];

#ifdef _WIN32
	SYSTEMTIME systm;
	GetLocalTime(&systm);
	sprintf_s(szCurrentDateTime, "%.2d-%.2d-%.2d %.2d:%.2d:%.2d",
			  systm.wYear, systm.wMonth, systm.wDay,
			  systm.wHour, systm.wMinute, systm.wSecond);
#else
	// struct timeval tv;
	// struct tm      tm;
	// gettimeofday(&tv, NULL);
	// localtime_r(&tv.tv_sec, &tm);
	// strftime(szCurrentDateTime, 32, "%.2d-%.2d-%.2d %.2d:%.2d:%.2d", &tm);

	time_t t;
	time(&t);
	strftime(szCurrentDateTime, 32, "%F %T", localtime(&t));

#endif

	_szCurrentDateTime = szCurrentDateTime;
}
////////////��ȡ��ǰ����Ŀ¼
inline void GetModuleFileName(string &_exeFullPath)
{
#ifdef _WIN32
	CHAR exeFullPath[MAX_PATH];
	::GetModuleFileNameA(NULL, exeFullPath, MAX_PATH);
	_exeFullPath = exeFullPath;
#else
	char exeFullPath[2048] = {0};
	GetModuleFileName(exeFullPath, 2048); // �˴�����������
	_exeFullPath = exeFullPath;
#endif
}

/// ��ȡ��ǰ����Ŀ¼
#ifdef _WIN32
inline std::string GetGetModuleFileDir()
{
	string exeFullPath;
	GetModuleFileName(exeFullPath);

	std::string strFilePath(exeFullPath);
	string::size_type pos = strFilePath.rfind("\\");

	if (pos != string::npos)
	{
		strFilePath = strFilePath.substr(0, pos + 1);
	}
	return strFilePath;
}
#endif
#endif
