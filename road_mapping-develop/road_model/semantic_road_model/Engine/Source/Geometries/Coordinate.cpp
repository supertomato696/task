/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Coordinate.cpp
简要描述:
******************************************************************/

#include "Geometries/Coordinate.h"
#include "Base/Math.h"
#include <boost/format.hpp>

#include <stdio.h>

using namespace Engine::Base;
using namespace Engine::Geometries;
using namespace Engine;
// 默认构造函数
Coordinate::Coordinate()
	: x(0.0), y(0.0), z(0.0)
{
}

Coordinate::Coordinate(Double x, Double y, Double z)
	: x(x), y(y), z(z)
{
}

// 析构函数
Coordinate::~Coordinate()
{
}

// 复制构造函数
Coordinate::Coordinate(const Coordinate &rhs)
{
	x = rhs.x;
	y = rhs.y;
	z = rhs.z;
}

// 赋值操作符
Coordinate &Coordinate::operator=(const Coordinate &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
		x = rhs.x;
		y = rhs.y;
		z = rhs.z;
	}

	return *this;
}

/// 判断两个Coordinate对象是否相等
Bool Coordinate::Equals(const Coordinate &other) const
{
	if (!Math::Equal(x, other.x))
	{
		return false;
	}
	if (!Math::Equal(y, other.y))
	{
		return false;
	}
	if (!Math::Equal(z, other.z))
	{
		return false;
	}
	return true;
}

/// 返回x y z格式字符串
String Coordinate::ToString() const
{
	boost::format fmt = boost::format("%.15f %.15f %.15f") % x % y % z;
	std::string stdstr = fmt.str();
	return String(stdstr.c_str());
}

/// 计算两个坐标之间的距离
Double Coordinate::Distance(const Coordinate &p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	double dz = z - p.z;
	return Math::Sqrt(dx * dx + dy * dy + dz * dz);
}

/// 计算两个坐标之间的距离
Double Coordinate::DistanceSquare(const Coordinate &p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	double dz = z - p.z;

	return (dx * dx + dy * dy + dz * dz);
}

// 重载运算符-
Coordinate Coordinate::operator-(const Coordinate &rhs) const
{
	return Coordinate(x - rhs.x, y - rhs.y, z - rhs.z);
}

// 重载运算符+
Coordinate Coordinate::operator+(const Coordinate &rhs) const
{
	return Coordinate(x + rhs.x, y + rhs.y, z + rhs.z);
}

// 重载运算符*,将坐标看做向量运算
Double Coordinate::operator*(const Coordinate &rhs) const
{
	return x * rhs.x + y * rhs.y + z * rhs.z;
}

Double Coordinate::GetLength() const
{
	return Math::Sqrt(x * x + y * y + z * z);
}

Void Coordinate::Normalize()
{
	Double lenth = GetLength();

	if (lenth > 0.0)
	{
		x = x / lenth;
		y = y / lenth;
		z = z / lenth;
	}
}

Coordinate Coordinate::operator*(Double N) const
{
	return Coordinate(x * N, y * N, z * N);
}

Coordinate Coordinate::operator/(Double N) const
{
	return Coordinate(x / N, y / N, z / N);
}

Coordinate Coordinate::CrossProduct(const Coordinate &other) const
{
	double ax = y * other.z - z * other.y;
	double ay = z * other.x - x * other.z;
	double az = x * other.y - y * other.x;
	return Coordinate(ax, ay, az);
}

Double Coordinate::DotProduct(const Coordinate &pnt) const
{
	return x * pnt.x + y * pnt.y + z * pnt.z;
}

Double Coordinate::DistanceXY(const Coordinate &p) const
{
	double dx = x - p.x;
	double dy = y - p.y;
	return Math::Sqrt(dx * dx + dy * dy);
}

Bool Coordinate::IsZeroVector() const
{
	return GetLength() <= Geometries_EP;
}

Double Coordinate::AngleWith(const Coordinate &v) const
{
	if (IsZeroVector() || v.IsZeroVector())
		return 0;

	Double d = (*(this) * v) / (GetLength() * v.GetLength());
	return Math::ACos(d);
}

String Coordinate::ToString2D() const
{
	boost::format fmt = boost::format("X:%.6f,Y:%.6f") % x % y;
	std::string stdstr = fmt.str();
	return String(stdstr.c_str());
}

Coordinate &Coordinate::operator+=(const Coordinate &rhs)
{
	x += rhs.x;
	y += rhs.y;
	z += rhs.z;
	return *this;
}

Coordinate &Coordinate::operator*=(Double N)
{
	x *= N;
	y *= N;
	z *= N;
	return *this;
}

bool Coordinate::IsEqual(const Coordinate &t, Base::Double tolerance /*= Geometries_NEP*/) const
{
	if (fabs(this->x - t.x) < tolerance && fabs(this->y - t.y) < tolerance && fabs(this->z - t.z) < tolerance)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void Coordinate::FormatOne(string &_lineTxt, const char *_sStart) const
{
	/*lineTxt = "v ";
	sprintf_s(chValue, "%f", EntrePoints[i].x);
	strValue = chValue;
	lineTxt += strValue;
	lineTxt += " ";
	sprintf_s(chValue, "%f", EntrePoints[i].y);
	strValue = chValue;
	lineTxt += strValue;
	lineTxt += " ";
	sprintf_s(chValue, "%f", EntrePoints[i].z);
	strValue = chValue;
	lineTxt += strValue;
	lineTxt += "\n";*/

	char chValue[128];
	if (_sStart == NULL)
	{
#ifdef _WIN32
		sprintf_s(chValue, "%f %f %f\n", x, y, z);
#else
		sprintf(chValue, "%f %f %f\n", x, y, z);
#endif
	}
	else
	{
#ifdef _WIN32
		sprintf_s(chValue, "%s%f %f %f\n", _sStart, x, y, z);
#else
		sprintf(chValue, "%s%f %f %f\n", _sStart, x, y, z);
#endif
	}
	_lineTxt = chValue;
}

void Coordinate::FormatTwo(string &_lineTxt, const Coordinate &_other) const
{
	// lineTxt = "v ";
	// sprintf_s(chValue, "%f", Shape[i].x + ClassifyShapeTool::m_arrTransXYZ[ii].x);
	// strValue = chValue;
	// lineTxt += strValue;
	// lineTxt += " ";
	// sprintf_s(chValue, "%f", Shape[i].y + ClassifyShapeTool::m_arrTransXYZ[ii].y);
	// strValue = chValue;
	// lineTxt += strValue;
	// lineTxt += " ";
	// sprintf_s(chValue, "%f", Shape[i].z + ClassifyShapeTool::m_arrTransXYZ[ii].z);
	// strValue = chValue;
	// lineTxt += strValue;
	// lineTxt += "\n";

	char chValue[128];
#ifdef _WIN32
	sprintf_s(chValue, "v %f %f %f\n", x + _other.x, y + _other.y, z + _other.z);
#else
	sprintf(chValue, "v %f %f %f\n", x + _other.x, y + _other.y, z + _other.z);
#endif
	_lineTxt = chValue;
}

void Coordinate::Assing(const int _nIndex, const Base::Double _fNum)
{
	if (_nIndex == 0)
		x = _fNum;
	else if (_nIndex == 1)
		y = _fNum;
	else if (_nIndex == 2)
		z = _fNum;
}

bool Coordinate::Assing(const char *_sGeo)
{
	const char *pstr = _sGeo;
	int nNum = 0;
	string sNum;
	while (pstr != NULL && *pstr)
	{
		if (*pstr == '(')
			;
		else if (*pstr == ')')
			break;
		else if (*pstr != ' ')
			sNum += *pstr;
		else
		{
			Assing(nNum, atof(sNum.c_str()));
			nNum++;
			sNum.clear();
		}
		pstr++;
	}
	if (!sNum.empty())
	{
		Assing(nNum, atof(sNum.c_str()));
		nNum++;
	}
	return nNum > 0;
}