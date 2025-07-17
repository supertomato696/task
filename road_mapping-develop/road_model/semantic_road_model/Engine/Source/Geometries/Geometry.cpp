/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Geometry.cpp
简要描述:
******************************************************************/

#include "Geometries/Geometry.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

// 默认构造函数
Geometry::Geometry()
{
}

// 析构函数
Geometry::~Geometry()
{
}

// 复制构造函数
Geometry::Geometry(const Geometry &rhs)
{
}

// 赋值操作符
Geometry &Geometry::operator=(const Geometry &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
	}

	return *this;
}

std::string &trim(std::string &s)
{
	if (s.empty())
		return s;

	s.erase(0, s.find_first_not_of(" "));
	s.erase(s.find_last_not_of(" ") + 1);
	return s;
}

const int GetGeometryHeadType(const string _sType)
{
	string sType = _sType;
	trim(sType);
	if (sType == "POLYGON")
		return 1;
	else if (sType == "LINESTRING")
		return 2;
	return 0;
}

const int Geometry::ReadType(const char *&_sGeo)
{
	string stype;
	const char *pstr = _sGeo;

	while (pstr != NULL && *pstr)
	{
		if (*pstr == '(')
		{ ////获取 geo 类型
			_sGeo = pstr;
			return GetGeometryHeadType(stype);
		}
		else
		{ // 保存类型
			stype += *pstr;
		}
		pstr++;
		//_sGeo++;
	}
	return 0;
}
