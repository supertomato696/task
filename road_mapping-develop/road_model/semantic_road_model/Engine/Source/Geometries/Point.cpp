/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point.cpp
简要描述:
******************************************************************/

#include "Geometries/Point.h"
#include "Geometries/GeometryCollection.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

// 默认构造函数
Point::Point()
	: m_coordinate(new Coordinate())
{
}

Point::Point(Double x, Double y, Double z)
	: m_coordinate(new Coordinate(x, y, z))
{
}

Point::Point(Coordinate *coord)
	: m_coordinate(coord)
{
}

Point::Point(Coordinate coord)
	: m_coordinate(new Coordinate(coord.x, coord.y, coord.z))
{
}

// 析构函数
Point::~Point()
{
	if (m_coordinate != NULL)
	{
		delete m_coordinate;

		m_coordinate = NULL;
	}
}

// 复制构造函数
Point::Point(const Point &rhs)
{
	m_coordinate = new Coordinate(rhs.GetX(), rhs.GetY(), rhs.GetZ());
}

// 赋值操作符
Point &Point::operator=(const Point &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		if (m_coordinate != NULL)
		{
			delete m_coordinate;

			m_coordinate = NULL;
		}

		// 然后进行深拷贝操作
		m_coordinate = new Coordinate(rhs.GetX(), rhs.GetY(), rhs.GetZ());
	}

	return *this;
}

GeometryType Point::GetGeometryType() const
{
	return GeometryType::POINT;
}

Envelope *const Point::GetEnvelope() const
{
	Envelope *evn = new Envelope(GetX(), GetY(), GetX(), GetY());
	return evn;
}

Double Point::GetX() const
{
	return m_coordinate->x;
}

Double Point::GetY() const
{
	return m_coordinate->y;
}

Double Point::GetZ() const
{
	return m_coordinate->z;
}

Void Point::SetX(Double d)
{
	m_coordinate->x = d;
}

Void Point::SetY(Double d)
{
	m_coordinate->y = d;
}

Void Point::SetZ(Double d)
{
	m_coordinate->z = d;
}

// 深拷贝对象
Geometry *const Point::Clone() const
{
	Point *point = new Point(*this);
	return point;
}

Coordinate Point::ToCoordinate() const
{
	return *m_coordinate;
}
