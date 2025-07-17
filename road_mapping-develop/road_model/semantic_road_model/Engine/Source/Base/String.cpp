/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:String.cpp
简要描述:
******************************************************************/

#include "Base/String.h"

// #include <windows.h>

#include <sstream>
#include <string>
#include <string.h>

using namespace Engine::Base;

String::String()
	: m_str(new std::string())
{
	m_emCC = CharsetCode::UTF8;
}

String::String(const Char *str)
{
	m_emCC = CharsetCode::UTF8;

	if (str != NULL)
	{
		m_str = new std::string(str);
	}
	else
	{
		m_str = new std::string();
	}
}
// Del  llj 2018-10-22
// String::String(const UInt16* str)
//{
//	m_emCC = CharsetCode::UTF8;
//
//	if (str != NULL)
//	{
//		Int32 nLen = WideCharToMultiByte(CP_UTF8, 0, (LPWSTR)str, -1, NULL, 0, NULL, NULL);
//		Char* pUTF8 = new Char[nLen + 1];
//		memset(pUTF8, 0, nLen + 1);
//		WideCharToMultiByte(CP_UTF8, 0, (LPWSTR)str, -1, pUTF8, nLen, NULL, NULL);
//
//		m_str = new std::string(pUTF8);
//		delete[] pUTF8;
//		pUTF8 = NULL;
//	}
//	else
//	{
//		m_str = new std::string();
//	}
// }

String::String(const Char *str, Int32 nStart, Int32 nCount)
{
	m_emCC = CharsetCode::UTF8;

	m_str = new std::string;
	if (str != NULL)
	{
		m_str->append(str + nStart, nCount);
	}
}

String::String(const Int32 num)
{
	m_str = new std::string(std::to_string(num)); // by duanzhikang 优化自动化接边功能需要
	// 	std::stringstream ss;
	// 	ss << num;
	// 	*m_str = ss.str();
	// 	ss.clear();
}

String::String(const Int64 num)
{
	// m_str = new std::string(std::to_string(num));
	m_str = new std::string();
	std::stringstream ss;
	ss << num;
	*m_str = ss.str();
	ss.clear();
}

String::String(const Float num)
{
	// m_str = new std::string(std::to_string(num));
	m_str = new std::string();
	std::stringstream ss;
	ss << num;
	*m_str = ss.str();
	ss.clear();
}

String::String(const Double num, const Int32 nPre)
{
	m_str = new std::string();
	Char buffer[20];
	char szFormat[16];
#ifdef _WIN32
	sprintf_s(szFormat, "%%.%dlf", nPre);
	sprintf_s(buffer, szFormat, num);
#else
	sprintf(szFormat, "%%.%dlf", nPre);
	sprintf(buffer, szFormat, num);
#endif
	*m_str = buffer;
}

String::String(const String &str)
{
	m_str = new std::string(str.GetCStr());
	m_emCC = str.m_emCC;
}

// 赋值操作符
String &String::operator=(const String &str)
{
	// 检查自赋值
	if (this != &str)
	{
		// 先释放当前资源
		if (m_str != NULL)
		{
			delete m_str;

			m_str = NULL;
		}

		// 然后进行深拷贝操作
		m_str = new std::string(str.GetCStr());

		m_emCC = str.m_emCC;
	}

	return *this;
}

Bool String::IsEmpty() const
{
	return m_str->empty();
}

SizeT String::Length() const
{
	return m_str->length();
}

Void String::Append(const Char *str)
{
	m_str->append(str);
}

Void String::Append(const Char *str, int n)
{
	m_str->append(str, n);
}

const Char *String::GetCStr() const
{
	return m_str->c_str();
}

Bool String::operator<(const String &rhs) const
{
	return *m_str < *rhs.m_str;
}

String String::operator+(const String &rhs)
{
	String abc;
	abc = *this;
	abc += rhs;

	return abc;
}

String &String::operator+=(const String &rhs)
{
	*this->m_str += *rhs.m_str;

	return *this;
}

String::~String()
{
	if (m_str != NULL)
	{
		delete m_str;

		m_str = NULL;
	}
}
// Del  llj 2018-10-22
// String String::ToANSI()
//{
//	if (m_emCC == CharsetCode::ANSI)
//	{
//		return *this;
//	}
//
//	Int32 nWideCharLen = MultiByteToWideChar(CP_UTF8, 0, m_str->c_str(), -1, NULL, 0);
//	UInt16*pWideChars = new UInt16[nWideCharLen + 1];
//	memset(pWideChars, 0, sizeof(UInt16)*(nWideCharLen + 1));
//
//	MultiByteToWideChar(CP_UTF8, 0, m_str->c_str(), -1, (LPWSTR)pWideChars, nWideCharLen);
//
//	Int32 nANSICharLen = WideCharToMultiByte(CP_ACP, 0, (LPWSTR)pWideChars, nWideCharLen, NULL, 0, NULL, NULL);
//	Char* pANSI = new Char[nANSICharLen + 1];
//	memset(pANSI, 0, nANSICharLen + 1);
//	WideCharToMultiByte(CP_ACP, 0, (LPWSTR)pWideChars, nWideCharLen, pANSI, nANSICharLen, NULL, NULL);
//
//	String result(pANSI);
//	result.m_emCC = CharsetCode::ANSI;
//
//	delete[] pANSI;
//	delete[] pWideChars;
//
//	return result;
// }

Array<String> String::Splite(String &delim)
{
	Array<String> result;

	size_t last = 0;
	size_t index = m_str->find_first_of(delim.GetCStr(), last);
	while (index != std::string::npos)
	{
		result.Add(m_str->substr(last, index - last).c_str());
		last = index + delim.m_str->size();
		index = m_str->find_first_of(delim.GetCStr(), last);
	}
	if (index - last > 0)
	{
		result.Add(m_str->substr(last, index - last).c_str());
	}
	return result;
}

String &String::ReplaceString(const String &src, const String &des)
{
	std::string::size_type pos = 0;
	SizeT lsrc = src.Length();
	SizeT ldes = des.Length();

	while ((pos = m_str->find(src.GetCStr(), pos)) != std::string::npos)
	{
		m_str->replace(pos, lsrc, des.GetCStr());
		pos = pos + ldes;
	}
	return *this;
}

SizeT String::Pos(const String &src, SizeT pos)
{
	return m_str->find(src.GetCStr(), pos);
}
void String::Erase(SizeT pos, SizeT cnt)
{
	if (m_str != NULL)
	{
		m_str->erase(pos, cnt);
	}
}

Bool String::StartWith(const String &src)
{
	return m_str->find(src.GetCStr(), 0) == 0;
}

Bool String::EndWith(const String &src)
{
	return m_str->find_last_of(src.GetCStr()) == m_str->size() - 1;
}

namespace Navinfo
{
	namespace Engine
	{
		namespace Base
		{
			Bool operator==(const String &a, const String &b)
			{
				return strcmp(a.GetCStr(), b.GetCStr()) == 0;
			}

			Bool operator!=(const String &a, const String &b)
			{
				return strcmp(a.GetCStr(), b.GetCStr()) != 0;
			}

		}
	}
}
