/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Polygon.cpp
简要描述:
******************************************************************/

#include "Geometries/Polygon.h"
#include "Geometries/LinearRing.h"
#include "Base/Macros.h"
#include "Geometries/GeomAdapter.h"
#include "geos/geos.h"
#include "Geometries/GeometryAlgorithm.h"

#include <exception>
#include <stdexcept>

using namespace Engine;
using namespace Engine::Geometries;
using namespace Engine::Base;

using namespace std;

const Double Polygon_EP = 1.0e-5;

Geometries::Polygon::Polygon()
	: m_shell(NULL),
	  m_holes(new Array<LinearRing *>())
{
}

Geometries::Polygon::Polygon(Geometries::LinearRing *shell)
	: m_shell(shell),
	  m_holes(new Array<LinearRing *>())
{
}

Geometries::Polygon::Polygon(Geometries::LinearRing *shell, Array<Geometries::LinearRing *> *holes)
	: m_shell(shell),
	  m_holes(holes)
{
	if (m_shell->IsEmpty() && HasNoEmptyElement(m_holes))
	{
		// throw std::exception("无法构造Polygon对象:shell为空,但holes包含非空元素");
#ifdef _WIN32
		throw std::exception("无法构造Polygon对象:shell为空,但holes包含非空元素");
#else
		logic_error ex("无法构造Polygon对象:shell为空,但holes包含非空元素");
		throw std::exception(ex);
#endif
	}
}

// 析构函数
Geometries::Polygon::~Polygon()
{
	if (m_shell != nullptr)
	{
		delete m_shell;

		m_shell = nullptr;
	}

	if (m_holes != nullptr)
	{
		SizeT count = m_holes->GetCount();
		for (SizeT i = 0; i < count; ++i)
		{
			delete (*m_holes)[i];
		}

		delete m_holes;

		m_holes = nullptr;
	}
}

// 复制构造函数
Geometries::Polygon::Polygon(const Geometries::Polygon &rhs)
{
	m_shell = new Geometries::LinearRing(*rhs.m_shell);

	SizeT count = rhs.m_holes->GetCount();
	m_holes = new Array<Geometries::LinearRing *>(count);
	for (SizeT i = 0; i < count; ++i)
	{
		(*m_holes)[i] = new Geometries::LinearRing(*(*rhs.m_holes)[i]);
	}
}

// 赋值操作符
Geometries::Polygon &Geometries::Polygon::operator=(const Geometries::Polygon &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		if (m_shell != nullptr)
		{
			delete m_shell;

			m_shell = nullptr;
		}

		if (m_holes != nullptr)
		{
			SizeT count = m_holes->GetCount();
			for (SizeT i = 0; i < count; ++i)
			{
				delete (*m_holes)[i];
			}

			delete m_holes;

			m_holes = nullptr;
		}

		// 然后进行深拷贝操作
		m_shell = new Geometries::LinearRing(*rhs.m_shell);

		SizeT count = rhs.m_holes->GetCount();
		m_holes = new Array<Geometries::LinearRing *>(count);
		for (SizeT i = 0; i < count; ++i)
		{
			(*m_holes)[i] = new Geometries::LinearRing(*(*rhs.m_holes)[i]);
		}
	}

	return *this;
}

GeometryType Geometries::Polygon::GetGeometryType() const
{
	return GeometryType::POLYGON;
}

Geometries::Envelope *const Geometries::Polygon::GetEnvelope() const
{
	return m_shell->GetEnvelope();
}

Geometries::Envelope3D *const Geometries::Polygon::GetEnvelope3D() const
{
	return m_shell->GetEnvelope3D();
}

Geometries::Geometry *const Geometries::Polygon::Clone() const
{
	Geometries::Polygon *clonePolygon = new Geometries::Polygon(*this);

	return clonePolygon;
}

Geometries::LinearRing *const Geometries::Polygon::GetExteriorRing() const
{
	return m_shell;
}

SizeT Geometries::Polygon::GetNumInteriorRing() const
{
	return m_holes->GetCount();
}

Geometries::LinearRing *const Geometries::Polygon::GetInteriorRingN(SizeT index) const
{
	if (index >= m_holes->GetCount())
	{
		// throw std::exception("Polygon::GetInteriorRingN参数index越界");
#ifdef _WIN32
		throw std::exception("Polygon::GetInteriorRingN参数index越界");
#else
		logic_error ex("Polygon::GetInteriorRingN参数index越界");
		throw std::exception(ex);
#endif
	}
	return (*m_holes)[index];
}

Double Geometries::Polygon::GetLength() const
{
	Double len = 0.0;
	len += m_shell->GetLength();
	SizeT count = m_holes->GetCount();
	for (SizeT i = 0; i < count; ++i)
	{
		len += (*m_holes)[i]->GetLength();
	}
	return len;
}

Bool Geometries::Polygon::HasNoEmptyElement(Array<Geometries::LinearRing *> *linearRings) const
{
	SizeT count = linearRings->GetCount();
	for (SizeT i = 0; i < count; ++i)
	{
		if (!linearRings->GetAt(i)->IsEmpty())
		{
			return true;
		}
	}

	return false;
}

Bool Geometries::Polygon::ZoomInOut(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	/*四个边相邻两边，两两做点积操作，即计算夹角。如果夹角有一个不是90度，则不是符合规定的矩形多边形*/

	Geometries::Coordinate vector_02 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(0));
	Geometries::Coordinate vector_24 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(2));
	Geometries::Coordinate vector_46 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(4));
	Geometries::Coordinate vector_60 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(6));
	vector_02.Normalize();
	vector_24.Normalize();
	vector_46.Normalize();
	vector_60.Normalize();
	Double dot1 = vector_02.DotProduct(vector_24);
	Double dot2 = vector_24.DotProduct(vector_46);
	Double dot3 = vector_46.DotProduct(vector_60);
	Double dot4 = vector_60.DotProduct(vector_02);

	if (abs(dot1) > Polygon_EP || abs(dot2) > Polygon_EP || abs(dot3) > Polygon_EP || abs(dot4) > Polygon_EP)
	{
		return false;
	}

	/**/

	Geometries::Coordinate vector_0t = pt - *(m_shell->GetCoordinateN(0));
	Geometries::Coordinate vector_2t = pt - *(m_shell->GetCoordinateN(2));
	Geometries::Coordinate vector_4t = pt - *(m_shell->GetCoordinateN(4));
	Geometries::Coordinate vector_6t = pt - *(m_shell->GetCoordinateN(6));
	Double dot = -1.0;

	if (index == 1)
	{
		Geometries::Coordinate vector_06 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(0));
		vector_06.Normalize();
		dot = vector_06.DotProduct(vector_0t);
		*(m_shell->GetCoordinateN(0)) = *(m_shell->GetCoordinateN(0)) + vector_06 * dot;
		Geometries::Coordinate vector_24 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(2));
		vector_24.Normalize();
		dot = vector_24.DotProduct(vector_2t);
		*(m_shell->GetCoordinateN(2)) = *(m_shell->GetCoordinateN(2)) + vector_24 * dot;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
	}
	else if (index == 2)
	{
		Coordinate vector_06 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(0));
		vector_06.Normalize();
		dot = vector_06.DotProduct(vector_0t);
		*(m_shell->GetCoordinateN(0)) = *(m_shell->GetCoordinateN(0)) + vector_06 * dot;
		Coordinate vector_46 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(4));
		vector_46.Normalize();
		dot = vector_46.DotProduct(vector_4t);
		*(m_shell->GetCoordinateN(4)) = *(m_shell->GetCoordinateN(4)) + vector_46 * dot;
		Coordinate vector_64 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(6));
		Double length_64 = vector_64.GetLength();
		vector_64.Normalize();
		*(m_shell->GetCoordinateN(2)) = *(m_shell->GetCoordinateN(0)) + vector_64 * length_64;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
	}
	else if (index == 3)
	{
		Coordinate vector_20 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(2));
		Coordinate vector_46 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(4));
		vector_20.Normalize();
		dot = vector_20.DotProduct(vector_2t);
		*(m_shell->GetCoordinateN(2)) = *(m_shell->GetCoordinateN(2)) + vector_20 * dot;
		vector_46.Normalize();
		dot = vector_46.DotProduct(vector_4t);
		*(m_shell->GetCoordinateN(4)) = *(m_shell->GetCoordinateN(4)) + vector_46 * dot;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
	}
	else if (index == 4)
	{
		Coordinate vector_20 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(2));
		Coordinate vector_60 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(6));
		vector_20.Normalize();
		dot = vector_20.DotProduct(vector_2t);
		*(m_shell->GetCoordinateN(2)) = *(m_shell->GetCoordinateN(2)) + vector_20 * dot;
		vector_60.Normalize();
		dot = vector_60.DotProduct(vector_6t);
		*(m_shell->GetCoordinateN(6)) = *(m_shell->GetCoordinateN(6)) + vector_60 * dot;
		Coordinate vector_06 = *(m_shell->GetCoordinateN(6)) - *(m_shell->GetCoordinateN(0));
		Double length_06 = vector_06.GetLength();
		vector_06.Normalize();
		*(m_shell->GetCoordinateN(4)) = *(m_shell->GetCoordinateN(2)) + vector_06 * length_06;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
	}
	else if (index == 5)
	{
		Coordinate vector_42 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(4));
		Coordinate vector_60 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(6));
		vector_42.Normalize();
		dot = vector_42.DotProduct(vector_4t);
		*(m_shell->GetCoordinateN(4)) = *(m_shell->GetCoordinateN(4)) + vector_42 * dot;
		vector_60.Normalize();
		dot = vector_60.DotProduct(vector_6t);
		*(m_shell->GetCoordinateN(6)) = *(m_shell->GetCoordinateN(6)) + vector_60 * dot;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
	}
	else if (index == 6)
	{
		Coordinate vector_42 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(4));
		Coordinate vector_02 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(0));
		vector_42.Normalize();
		dot = vector_42.DotProduct(vector_4t);
		*(m_shell->GetCoordinateN(4)) = *(m_shell->GetCoordinateN(4)) + vector_42 * dot;
		vector_02.Normalize();
		dot = vector_02.DotProduct(vector_0t);
		*(m_shell->GetCoordinateN(0)) = *(m_shell->GetCoordinateN(0)) + vector_02 * dot;
		Coordinate vector_20 = *(m_shell->GetCoordinateN(0)) - *(m_shell->GetCoordinateN(2));
		Double length_20 = vector_20.GetLength();
		vector_20.Normalize();
		*(m_shell->GetCoordinateN(6)) = *(m_shell->GetCoordinateN(4)) + vector_20 * length_20;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
	}
	else if (index == 7)
	{
		Coordinate vector_64 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(6));
		Coordinate vector_02 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(0));
		vector_64.Normalize();
		dot = vector_64.DotProduct(vector_6t);
		*(m_shell->GetCoordinateN(6)) = *(m_shell->GetCoordinateN(6)) + vector_64 * dot;
		vector_02.Normalize();
		dot = vector_02.DotProduct(vector_0t);
		*(m_shell->GetCoordinateN(0)) = *(m_shell->GetCoordinateN(0)) + vector_02 * dot;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
	}
	else if (index == 0)
	{
		Coordinate vector_64 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(6));
		Coordinate vector_24 = *(m_shell->GetCoordinateN(4)) - *(m_shell->GetCoordinateN(2));
		vector_64.Normalize();
		dot = vector_64.DotProduct(vector_6t);
		*(m_shell->GetCoordinateN(6)) = *(m_shell->GetCoordinateN(6)) + vector_64 * dot;
		vector_24.Normalize();
		dot = vector_24.DotProduct(vector_2t);
		*(m_shell->GetCoordinateN(2)) = *(m_shell->GetCoordinateN(2)) + vector_24 * dot;
		Coordinate vector_42 = *(m_shell->GetCoordinateN(2)) - *(m_shell->GetCoordinateN(4));
		Double length_42 = vector_42.GetLength();
		vector_42.Normalize();
		*(m_shell->GetCoordinateN(0)) = *(m_shell->GetCoordinateN(6)) + vector_42 * length_42;
		*(m_shell->GetCoordinateN(7)) = (*(m_shell->GetCoordinateN(6)) + *(m_shell->GetCoordinateN(0))) / 2.0;
		*(m_shell->GetCoordinateN(1)) = (*(m_shell->GetCoordinateN(0)) + *(m_shell->GetCoordinateN(2))) / 2.0;
		*(m_shell->GetCoordinateN(3)) = (*(m_shell->GetCoordinateN(2)) + *(m_shell->GetCoordinateN(4))) / 2.0;
		*(m_shell->GetCoordinateN(5)) = (*(m_shell->GetCoordinateN(4)) + *(m_shell->GetCoordinateN(6))) / 2.0;
	}

	*(m_shell->GetCoordinateN(8)) = *(m_shell->GetCoordinateN(0));

	return true;
}

Double Geometries::Polygon::GetArea() const
{
	geos::geom::Geometry *pGeosGeom = GeomAdapter::ToGeosGeom(this);
	if (NULL == pGeosGeom)
		return 0;

	Double dArea = pGeosGeom->getArea();

	delete pGeosGeom;

	return dArea;
}

Base::Bool Geometries::Polygon::Distinct()
{
	return m_shell->Distinct();
}

////////////////////////////

void Geometries::Polygon::AddLinearRing(const string &slinegeo)
{
	if (!slinegeo.empty())
	{
		LinearRing *pLineRing = new LinearRing;
		if (pLineRing->Assing(slinegeo.c_str()))
		{
			if (m_shell == NULL)
				m_shell = pLineRing;
			else
				m_holes->Add(pLineRing);
		}
	}
}

bool Geometries::Polygon::Assing(const char *_sGeo)
{
	const char *pstr = _sGeo;

	string slinegeo;

	int nNum = 0;
	while (pstr != NULL && *pstr)
	{
		if (*pstr == '(')
		{
			nNum++;
			if (nNum > 1)
				slinegeo += *pstr;
		}
		else if (*pstr == ')')
		{
			nNum--;
			if (nNum == 0)
				break;
			slinegeo += *pstr;
		}
		else if (*pstr != ',' || nNum > 1)
			slinegeo += *pstr;
		else
		{
			AddLinearRing(slinegeo);
			slinegeo.c_str();
		}
		pstr++;
	}

	AddLinearRing(slinegeo);

	return true;
}
