/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Envelope.cpp
简要描述:
******************************************************************/

#include "Geometries/Envelope.h"
#include "Geometries/LineString.h"
#include "Geometries/GeomAdapter.h"
#include "Base/Math.h"
#include "algorithm"

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

// 默认构造函数
Envelope::Envelope()
{
	SetToNull();
}

Envelope::Envelope(Double x1, Double y1, Double x2, Double y2)
{
	Init(x1, y1, x2, y2);
}

// 析构函数
Envelope::~Envelope()
{
}

// 复制构造函数
Envelope::Envelope(const Envelope &rhs)
{
	Init(rhs.m_minx, rhs.m_miny, rhs.m_maxx, rhs.m_maxy);
}

// 赋值操作符
Envelope &Envelope::operator=(const Envelope &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
		Init(rhs.m_minx, rhs.m_miny, rhs.m_maxx, rhs.m_maxy);
	}

	return *this;
}

Double Envelope::GetMaxY() const
{
	return m_maxy;
}

Double Envelope::GetMaxX() const
{
	return m_maxx;
}

Double Envelope::GetMinY() const
{
	return m_miny;
}

Double Envelope::GetMinX() const
{
	return m_minx;
}

Double Envelope::GetWidth() const
{
	return m_maxx - m_minx;
}

Double Envelope::GetHeight() const
{
	return m_maxy - m_miny;
}

Double Envelope::GetArea() const
{
	return GetWidth() * GetHeight();
}

Void Envelope::ExpandBy(Double deltaX, Double deltaY)
{
	m_minx -= deltaX;
	m_maxx += deltaX;
	m_miny -= deltaY;
	m_maxy += deltaY;
}

Void Envelope::ExpandBy(Double distance)
{
	ExpandBy(distance, distance);
}

Void Envelope::ExpandToInclude(const Envelope *other)
{
	if (IsNull())
	{
		m_minx = other->m_minx;
		m_maxx = other->m_maxx;
		m_miny = other->m_miny;
		m_maxy = other->m_maxy;
	}
	else
	{
		if (other->m_minx < m_minx)
		{
			m_minx = other->m_minx;
		}
		if (other->m_maxx > m_maxx)
		{
			m_maxx = other->m_maxx;
		}
		if (other->m_miny < m_miny)
		{
			m_miny = other->m_miny;
		}
		if (other->m_maxy > m_maxy)
		{
			m_maxy = other->m_maxy;
		}
	}
}

Bool Envelope::Intersects(const Coordinate &p) const
{
	return !(p.x > m_maxx || p.x < m_minx || p.y > m_maxy || p.y < m_miny);
}

Bool Envelope::Intersects(Double x, Double y) const
{
	return !(x > m_maxx || x < m_minx || y > m_maxy || y < m_miny);
}

Bool Envelope::Equals(const Envelope *other) const
{
	if (IsNull() && other->IsNull())
	{
		return true;
	}
	else if (!IsNull() && other->IsNull())
	{
		return false;
	}
	else if (IsNull() && !other->IsNull())
	{
		return false;
	}
	else
	{
		return Math::Equal(other->GetMinX(), m_minx) &&
			   Math::Equal(other->GetMaxX(), m_maxx) &&
			   Math::Equal(other->GetMinY(), m_miny) &&
			   Math::Equal(other->GetMaxY(), m_maxy);
	}
}

Bool Envelope::Intersects(const Envelope &other) const
{
	return Intersects(&other);
};

Bool Envelope::Intersects(const Envelope *other) const
{
	if (!Equals(other) && !Contains(other) && !Contained(other))
	{
		return (other->m_minx <= m_maxx &&
				other->m_maxx >= m_minx &&
				other->m_miny <= m_maxy &&
				other->m_maxy >= m_miny);
	}
	else
	{
		return false;
	}
};

Bool Envelope::Intersection(const Envelope &other, Envelope &Intersec) const
{
	return Intersection(&other, Intersec);
};

Bool Envelope::Intersection(const Envelope *other, Envelope &Intersec) const
{
	Double x[4] = {m_minx, other->m_minx, m_maxx, other->m_maxx};
	Double y[4] = {m_miny, other->m_miny, m_maxy, other->m_maxy};
	sort(x, x + 4);
	sort(y, y + 4);

	if (Equals(other) || Contains(other) || Contained(other))
	{
		Intersec.Reset(x[1], y[1], x[2], y[2]);
		return true;
	}

	if (other->m_minx <= m_maxx &&
		other->m_maxx >= m_minx &&
		other->m_miny <= m_maxy &&
		other->m_maxy >= m_miny)
	{
		Intersec.Reset(x[1], y[1], x[2], y[2]);
		return true;
	}

	return false;
};

Bool Envelope::Contains(const Envelope &other) const
{
	return Contains(&other);
};

Bool Envelope::Contains(const Envelope *other) const
{
	return (other->m_minx > m_minx &&
			other->m_maxx < m_maxx &&
			other->m_miny > m_miny &&
			other->m_maxy < m_maxy);
};

Bool Envelope::Contained(const Envelope &other) const
{
	return Contained(&other);
};

Bool Envelope::Contained(const Envelope *other) const
{
	return (other->m_minx < m_minx &&
			other->m_maxx > m_maxx &&
			other->m_miny < m_miny &&
			other->m_maxy > m_maxy);
};

Bool Envelope::PtInRect(Double x, Double y) const
{
	if (x < m_minx || x > m_maxx)
		return false;

	if (y < m_miny || y > m_maxy)
		return false;

	return true;
};

Bool Envelope::IsNull() const
{
	return m_maxx < m_minx || m_maxy < m_miny;
}

Void Envelope::SetToNull()
{
	m_minx = 0.0;
	m_maxx = -1.0;
	m_miny = 0.0;
	m_maxy = -1.0;
}

Void Envelope::Init(Double x1, Double y1, Double x2, Double y2)
{
	m_minx = x1;
	m_maxx = x2;

	m_miny = y1;
	m_maxy = y2;

	Normalize();
}

// 设置X值最大值
Void Envelope::SetMaxX(Double x)
{
	// x坐标最大值
	m_maxx = x;
}

// 设置X值最小值
Void Envelope::SetMinX(Double x)
{
	m_minx = x;
}

// 设置X值最大值
Void Envelope::SetMaxY(Double y)
{
	// y坐标最大值
	m_maxy = y;
}

// 设置Y值最小值
Void Envelope::SetMinY(Double y)
{
	// y坐标最小值
	m_miny = y;
}

// 使用给定的两个点(x1,y1),(x2,y2) 重置外包框
Void Envelope::Reset(Double x1, Double y1, Double x2, Double y2)
{
	Init(x1, y1, x2, y2);
}

// 合法化四至范围
Void Envelope::Normalize()
{
	if (m_maxx < m_minx)
	{
		Double dTmp = m_minx;
		m_minx = m_maxx;
		m_maxx = dTmp;
	}

	if (m_maxy < m_miny)
	{
		Double dTmp = m_miny;
		m_miny = m_maxy;
		m_maxy = dTmp;
	}
}

Envelope::Envelope(const Coordinate &p1, const Coordinate &p2)
{
	Init(p1.x, p1.y, p2.x, p2.y);
}

Coordinate Envelope::BottomLeft() const
{
	return Coordinate(m_minx, m_miny, 0);
}

Bool Envelope::IntersectOrContain(LineString *lineString)
{
	Array<Coordinate *> *coords = lineString->GetCoordinates();

	for (Array<Coordinate *>::_Iter itr = coords->Begin(); itr != coords->End(); itr++)
	{
		if (Intersects(**itr))
		{
			return true;
		}
	}
	return false;
}

Coordinate Envelope::TopRight() const
{
	return Coordinate(m_maxx, m_maxy, 0);
}

//{{qiuli
Void Envelope::Union(const Envelope &other)
{
	if (m_minx > other.GetMinX())
	{
		m_minx = other.GetMinX();
	}

	if (m_maxx < other.GetMaxX())
	{
		m_maxx = other.GetMaxX();
	}

	if (m_miny > other.GetMinY())
	{
		m_miny = other.GetMinY();
	}

	if (m_maxy < other.GetMaxY())
	{
		m_maxy = other.GetMaxY();
	}
}
//}}qiuli