/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:dllmain.cpp
简要描述:
******************************************************************/
#ifdef _WIN32
#include <Windows.h>

BOOL APIENTRY DllMain(HMODULE hModule,
					  DWORD ul_reason_for_call,
					  LPVOID lpReserved)
{
	switch (ul_reason_for_call)
	{
	case DLL_PROCESS_ATTACH:
	case DLL_THREAD_ATTACH:
	case DLL_THREAD_DETACH:
	case DLL_PROCESS_DETACH:
		break;
	}

	return TRUE;
}
#endif
