/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Envelope3D.cpp
简要描述:
******************************************************************/

#include "Geometries/Envelope3D.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/LineString.h"
#include "Geometries/MultiPolygon.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"
#include "Base/Array.h"
#include "algorithm"

using namespace Engine::Base;
using namespace Engine::Geometries;
using namespace std;

// 默认构造函数
Envelope3D::Envelope3D()
{
	SetToNull();
}
Envelope3D::Envelope3D(Double x1, Double y1, Double x2, Double y2, Double z1, Double z2)
{
	Init(x1, y1, x2, y2, z1, z2);
}

// 析构函数
Envelope3D::~Envelope3D()
{
}

// 复制构造函数
Envelope3D::Envelope3D(const Envelope3D &rhs)
{
	Init(rhs.m_minx, rhs.m_miny, rhs.m_maxx, rhs.m_maxy, rhs.m_maxz, rhs.m_minz);
}

// 赋值操作符
Envelope3D &Envelope3D::operator=(const Envelope3D &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
		Init(rhs.m_minx, rhs.m_miny, rhs.m_maxx, rhs.m_maxy, rhs.m_minz, rhs.m_maxz);
	}

	return *this;
}

Void Envelope3D::SetMaxZ(Double z)
{
	// z坐标最大值
	m_maxz = z;
}

// 设置Y值最小值
Void Envelope3D::SetMinZ(Double z)
{
	// z坐标最小值
	m_minz = z;
}
Void Envelope3D::Reset(Double x1, Double y1, Double x2, Double y2, Double z1, Double z2)
{
	Init(x1, y1, x2, y2, z1, z2);
}
Double Envelope3D::GetMaxZ() const
{
	return m_maxz;
}

Double Envelope3D::GetMinZ() const
{
	return m_minz;
}
Double Envelope3D::GetDepth() const
{
	return m_maxz - m_minz;
}

Void Envelope3D::ExpandBy(Double deltaX, Double deltaY, Double deltaZ)
{
	m_minx -= deltaX;
	m_maxx += deltaX;
	m_miny -= deltaY;
	m_maxy += deltaY;
	m_minz -= deltaZ;
	m_maxz += deltaZ;
};

Void Envelope3D::ExpandBy(Double distance)
{
	ExpandBy(distance, distance, distance);
};

Void Envelope3D::ExpandToInclude(const Envelope3D *other)
{
	if (IsNull())
	{
		m_minx = other->m_minx;
		m_maxx = other->m_maxx;
		m_miny = other->m_miny;
		m_maxy = other->m_maxy;
		m_minz = other->m_minz;
		m_maxz = other->m_maxz;
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
		if (other->m_maxz > m_maxz)
		{
			m_maxz = other->m_maxz;
		}
		if (other->m_minz < m_minz)
		{
			m_minz = other->m_minz;
		}
	}
};

Bool Envelope3D::Intersects(const Coordinate &p) const
{
	return !(p.x > m_maxx || p.x < m_minx || p.y > m_maxy || p.y < m_miny || p.z > m_maxz || p.z < m_minz);
};

Bool Envelope3D::Intersects(Double x, Double y, Double z) const
{
	return !(x > m_maxx || x < m_minx || y > m_maxy || y < m_miny || z > m_maxz || z < m_minz);
};

Bool Envelope3D::Intersects(const Envelope3D &other) const
{
	return Intersects(&other);
};

Bool Envelope3D::Intersects(const Envelope3D *other) const
{
	if (!Equals(other) && !Contains(other) && !Contained(other))
	{
		return (other->m_minx <= m_maxx &&
				other->m_maxx >= m_minx &&
				other->m_miny <= m_maxy &&
				other->m_maxy >= m_miny &&
				other->m_minz <= m_maxz &&
				other->m_maxz >= m_minz);
	}
	else
	{
		return false;
	}
};

Bool Envelope3D::Intersection(const Envelope3D &other, Envelope3D &Intersec) const
{
	return Intersection(&other, Intersec);
};

Bool Envelope3D::Intersection(const Envelope3D *other, Envelope3D &Intersec) const
{
	if (Intersects(other))
	{
		Double x[4] = {m_minx, other->m_minx, m_maxx, other->m_maxx};
		Double y[4] = {m_miny, other->m_miny, m_maxy, other->m_maxy};
		Double z[4] = {m_minz, other->m_minz, m_maxz, other->m_maxz};
		sort(x, x + 4);
		sort(y, y + 4);
		sort(z, z + 4);
		Intersec.Reset(x[1], y[1], x[2], y[2], z[1], z[2]);

		return true;
	}
	else
	{
		return false;
	}
};

Bool Envelope3D::Contains(const Envelope3D &other) const
{
	return Contains(&other);
};

Bool Envelope3D::Contains(const Envelope3D *other) const
{
	return (other->m_minx > m_minx &&
			other->m_maxx < m_maxx &&
			other->m_miny > m_miny &&
			other->m_maxy < m_maxy &&
			other->m_minz > m_minz &&
			other->m_maxz < m_maxz);
};

Bool Envelope3D::Contained(const Envelope3D &other) const
{
	return Contained(&other);
};

Bool Envelope3D::Contained(const Envelope3D *other) const
{
	return (other->m_minx < m_minx &&
			other->m_maxx > m_maxx &&
			other->m_miny < m_miny &&
			other->m_maxy > m_maxy &&
			other->m_minz < m_minz &&
			other->m_maxz > m_maxz);
};

Bool Envelope3D::PtInEnvelope3D(const Coordinate pt) const
{
	return false;
};

Double Envelope3D::GetVolum() const
{
	return GetDepth() * GetHeight() * GetWidth();
};

Bool Envelope3D::Equals(const Envelope3D *other) const
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
			   Math::Equal(other->GetMaxY(), m_maxy) &&
			   Math::Equal(other->GetMaxZ(), m_maxz) &&
			   Math::Equal(other->GetMinZ(), m_minz);
	}
	// throw std::logic_error("The method or operation is not implemented.");
}

Bool Engine::Geometries::Envelope3D::IsNull() const
{
	return m_maxx < m_minx || m_maxy < m_miny || m_maxz < m_minz;
	// throw std::logic_error("The method or operation is not implemented.");
}

Void Engine::Geometries::Envelope3D::SetToNull()
{
	m_minx = 0.0;
	m_maxx = -1.0;
	m_miny = 0.0;
	m_maxy = -1.0;
	m_minz = 0.0;
	m_maxz = -1.0;
	// throw std::logic_error("The method or operation is not implemented.");
}
Void Envelope3D::Init(Double x1, Double y1, Double x2, Double y2, Double z1, Double z2)
{
	m_minx = x1;
	m_maxx = x2;

	m_miny = y1;
	m_maxy = y2;

	m_maxz = z1;
	m_minz = z2;
	Normalize();
}
Void Engine::Geometries::Envelope3D::Normalize()
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

	if (m_maxz < m_minz)
	{
		Double dTmp = m_minz;
		m_minz = m_maxz;
		m_maxz = dTmp;
	}
	// throw std::logic_error("The method or operation is not implemented.");
}

Bool Envelope3D::Intersects(const Coordinate &v1, const Coordinate &v2) const
{
	Array<Coordinate *> *coordinates = new Array<Coordinate *>;
	coordinates->Add(new Coordinate(v1));
	coordinates->Add(new Coordinate(v2));
	LineString lineString(coordinates);

	Base::Bool res = false;
	// Intersect with Bottom
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_minz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		LinearRing *r1 = new LinearRing(coords);
		Polygon polyGon(r1);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	// Intersect with Top
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_minx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_minx, m_miny, m_maxz));
		LinearRing *r2 = new LinearRing(coords);
		Polygon polyGon(r2);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	// Intersect with Front
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_minz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_minx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		LinearRing *r3 = new LinearRing(coords);
		Polygon polyGon(r3);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	// Intersect with Back
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_minx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_minz));
		LinearRing *r4 = new LinearRing(coords);
		Polygon polyGon(r4);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	// Intersect with Left
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_minx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_minx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_minx, m_miny, m_minz));
		LinearRing *r5 = new LinearRing(coords);
		Polygon polyGon(r5);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	// Intersect with Right
	{
		Array<Coordinate *> *coords = new Array<Coordinate *>;
		coords->Add(new Coordinate(m_maxx, m_miny, m_minz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_minz));
		coords->Add(new Coordinate(m_maxx, m_maxy, m_maxz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_maxz));
		coords->Add(new Coordinate(m_maxx, m_miny, m_minz));
		LinearRing *r6 = new LinearRing(coords);
		Polygon polyGon(r6);
		res = GeometryAlgorithm::Intersects(&lineString, &polyGon);
		if (res)
			return true;
	}
	return false;
}
