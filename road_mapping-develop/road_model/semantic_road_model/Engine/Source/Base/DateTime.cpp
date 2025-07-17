/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:DateTime.cpp
简要描述:
******************************************************************/

#include "Base/DateTime.h"

using namespace Engine::Base;

DateTime::DateTime(Int32 year, Int32 month, Int32 day)
{
	m_TimeInfo.tm_year = year - 1900;
	m_TimeInfo.tm_mon = month - 1;
	m_TimeInfo.tm_mday = day;
	m_TimeInfo.tm_hour = 0;
	m_TimeInfo.tm_min = 0;
	m_TimeInfo.tm_sec = 0;
	m_TimeInfo.tm_isdst = -1;

	MakeTime();
}

DateTime::DateTime(Int32 year, Int32 month, Int32 day, Int32 hour, Int32 minute, Int32 second)
{
	m_TimeInfo.tm_year = year - 1900;
	m_TimeInfo.tm_mon = month - 1;
	m_TimeInfo.tm_mday = day;
	m_TimeInfo.tm_hour = hour;
	m_TimeInfo.tm_min = minute;
	m_TimeInfo.tm_sec = second;
	m_TimeInfo.tm_isdst = -1;

	MakeTime();
}

DateTime::DateTime(struct tm *timeinfo)
{
	m_TimeInfo.tm_sec = timeinfo->tm_sec;
	m_TimeInfo.tm_min = timeinfo->tm_min;
	m_TimeInfo.tm_hour = timeinfo->tm_hour;
	m_TimeInfo.tm_mday = timeinfo->tm_mday;
	m_TimeInfo.tm_mon = timeinfo->tm_mon;
	m_TimeInfo.tm_year = timeinfo->tm_year;
	m_TimeInfo.tm_wday = timeinfo->tm_wday;
	m_TimeInfo.tm_yday = timeinfo->tm_yday;
	m_TimeInfo.tm_isdst = timeinfo->tm_isdst;

	MakeTime();
}

// 析构函数
DateTime::~DateTime()
{
}

// 复制构造函数
DateTime::DateTime(const DateTime &rhs)
{
	SetTM(rhs.GetTM());
}

// 赋值操作符
DateTime &DateTime::operator=(const DateTime &rhs)
{
	if (this != &rhs)
	{
		SetTM(rhs.GetTM());
	}

	return *this;
}

Void DateTime::MakeTime()
{
	m_Timestamp = mktime(&m_TimeInfo);
}

Bool DateTime::IsValidDate() const
{
	return m_Timestamp != -1LL;
}

Void DateTime::SetDate(Int32 year, Int32 month, Int32 day)
{
	m_TimeInfo.tm_year = year;
	m_TimeInfo.tm_mon = month;
	m_TimeInfo.tm_mday = day;

	MakeTime();
}

Void DateTime::SetTime(Int32 hour, Int32 minute, Int32 second)
{
	m_TimeInfo.tm_hour = hour;
	m_TimeInfo.tm_min = minute;
	m_TimeInfo.tm_sec = second;

	MakeTime();
}

Void DateTime::SetDST(DSTInfo dst)
{
	m_TimeInfo.tm_isdst = static_cast<Int32>(dst);
}

DSTInfo DateTime::GetDST() const
{
	if (m_TimeInfo.tm_isdst > 0)
		return DSTInfo::ENABLE;
	if (m_TimeInfo.tm_isdst == 0)
		return DSTInfo::DISABLE;
	return DSTInfo::NOT_AVAILABLE;
}

Int64 DateTime::GetTimestamp() const
{
	return static_cast<Int64>(m_Timestamp);
}

const tm &DateTime::GetTM() const
{
	return m_TimeInfo;
}

Void DateTime::SetTM(const tm &timeinfo)
{
	m_TimeInfo = timeinfo;
	MakeTime();
}

Bool DateTime::IsLeapYear(Int32 year)
{
	if ((year % 4 == 0) && (year % 100 != 0) || (year % 400 == 0))
		return true;
	return false;
}

DateTime DateTime::GetLocalTime()
{
	time_t now = time(0);
	struct tm timeinfo;
#ifdef _WIN32
	localtime_s(&timeinfo, &now);
#else
	localtime_r(&now, &timeinfo);
#endif
	return DateTime(&timeinfo);
}

DateTime DateTime::GetSystemTime()
{
	time_t now = time(0);
	struct tm timeinfo;
#ifdef _WIN32
	gmtime_s(&timeinfo, &now);
#else
	gmtime_r(&now, &timeinfo);
#endif
	return DateTime(&timeinfo);
}

DateTime DateTime::FromEpochToLocalTime(Int64 seconds)
{
	struct tm timeinfo;

#ifdef _WIN32
	localtime_s(&timeinfo, &seconds);
#else
	time_t now = seconds;
	localtime_r(&now, &timeinfo);
#endif
	return DateTime(&timeinfo);
}

DateTime DateTime::FromEpochToSystemTime(Int64 seconds)
{
	struct tm timeinfo;
#ifdef _WIN32
	gmtime_s(&timeinfo, &seconds);
#else
	time_t now = seconds;
	gmtime_r(&now, &timeinfo);
#endif

	return DateTime(&timeinfo);
}

String DateTime::ToString(const Char *format) const
{
	if (!IsValidDate())
	{
		return "";
	}

	char buf[128];
	if (format == nullptr)
		strftime(buf, sizeof(buf), "%Y-%m-%d %X", &m_TimeInfo);
	else
		strftime(buf, sizeof(buf), format, &m_TimeInfo);
	return buf;
}

Bool DateTime::operator!=(const DateTime &rhs) const
{
	return !(*this == rhs);
}

Bool DateTime::operator<(const DateTime &rhs) const
{
	if (!IsValidDate() || !rhs.IsValidDate())
		return false;
	return GetTimestamp() < rhs.GetTimestamp();
}

Bool DateTime::operator<=(const DateTime &rhs) const
{
	return !(*this > rhs);
}

Bool DateTime::operator==(const DateTime &rhs) const
{
	if (!IsValidDate() || !rhs.IsValidDate())
		return false;
	return GetTimestamp() == rhs.GetTimestamp();
}

Bool DateTime::operator>(const DateTime &rhs) const
{
	if (!IsValidDate() || !rhs.IsValidDate())
		return false;
	return GetTimestamp() > rhs.GetTimestamp();
}

Bool DateTime::operator>=(const DateTime &rhs) const
{
	return !(*this < rhs);
}

Void DateTime::SetSecond(Int32 value)
{
	m_TimeInfo.tm_sec = value;
	MakeTime();
}

Int32 DateTime::GetSecond() const
{
	return m_TimeInfo.tm_sec;
}

Void DateTime::SetMinute(Int32 value)
{
	m_TimeInfo.tm_min = value;
	MakeTime();
}

Int32 DateTime::GetMinute() const
{
	return m_TimeInfo.tm_min;
}

Void DateTime::SetHour(Int32 value)
{
	m_TimeInfo.tm_hour = value;
	MakeTime();
}

Int32 DateTime::GetHour() const
{
	return m_TimeInfo.tm_hour;
}

Void DateTime::SetDay(Int32 value)
{
	m_TimeInfo.tm_mday = value;
	MakeTime();
}

Int32 DateTime::GetDay() const
{
	return m_TimeInfo.tm_mday;
}

Void DateTime::SetMonth(Int32 value)
{
	m_TimeInfo.tm_mon = value - 1;
	MakeTime();
}

Int32 DateTime::GetMonth() const
{
	return m_TimeInfo.tm_mon;
}

Void DateTime::SetYear(Int32 value)
{
	m_TimeInfo.tm_year = value - 1900;
	MakeTime();
}

Int32 DateTime::GetYear() const
{
	return m_TimeInfo.tm_year;
}
