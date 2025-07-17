/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:TimeTracker.cpp
简要描述:
******************************************************************/

#include "Base/TimeTracker.h"

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif

using namespace Engine::Base;

TimeTracker::TimeTracker()
{
	m_u64Now = 0;
}

TimeTracker::~TimeTracker()
{
	m_u64Now = 0;
}

// 开始计时
// 返回值是毫秒数
UInt64 TimeTracker::Start()
{
	m_u64Now = Now();
	return m_u64Now;
}

// 结束计时
// 返回值是毫秒数
UInt64 TimeTracker::Stop()
{
	UInt64 u64Now = Now();

	UInt64 u64Result = u64Now - m_u64Now;

	m_u64Now = 0;

	return u64Result;
}

// 统计从计时开始到现在耗费的时间
// 返回值是毫秒数
UInt64 TimeTracker::Elapsed()
{
	UInt64 u64Now = Now();

	return (u64Now - m_u64Now);
}

#ifndef _WIN32
// 参照 http://www.cppblog.com/aaxron/archive/2011/03/05/141150.html
UInt64 GetTickCount64()
{
	struct timeval tv;
	if (gettimeofday(&tv, NULL) != 0)
	{
		return 0;
	}
	return (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
}
#endif

// 返回当前毫秒数
UInt64 TimeTracker::Now()
{
	return GetTickCount64();
}
