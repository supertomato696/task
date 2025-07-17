/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Octagon.cpp
简要描述:八角形
******************************************************************/

#include "Geometries/Octagon.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine;

Octagon::Octagon()
{
	m_arrCoordinates.SetSize(8);
}

Octagon::Octagon(const Engine::Geometries::Coordinate &pntSegFrom,
				 const Engine::Geometries::Coordinate &pntSegTo,
				 const Engine::Geometries::Coordinate &pnt)
{
	ComputerNormal(pntSegFrom, pntSegTo, pnt);
	ComputerCenter(pntSegFrom, pntSegTo, pnt);
	for (int i = 0; i < 8; i++)
	{
		Coordinate v = pntSegFrom;
		double angle = i * PI / 4;
		BaseAlgorithm3D::RotatePoint(m_coordCenter, angle, m_vectorNormal, v);
		m_arrCoordinates.Add(v);
	}
}

Octagon::Octagon(const Engine::Base::Array<Engine::Geometries::Coordinate> &controlPoints)
{
	if (controlPoints.GetCount() < 8)
	{
		return;
	}

	*this = Octagon(controlPoints.GetAt(0), controlPoints.GetAt(1), controlPoints.GetAt(2));
}

Octagon::~Octagon()
{
}

Base::Array<Coordinate> Engine::Geometries::Octagon::GetControlPoints()
{
	return m_arrCoordinates;
}

Base::Array<Coordinate> Engine::Geometries::Octagon::GetCoordinates()
{
	return m_arrCoordinates;
}

Base::Void Engine::Geometries::Octagon::Offset(Base::Double dx, Base::Double dy, Base::Double dz)
{
	for (int i = 0; i < 8; i++)
	{
		m_arrCoordinates[i].x += dx;
		m_arrCoordinates[i].y += dy;
		m_arrCoordinates[i].z += dz;
	}
}

Base::Void Engine::Geometries::Octagon::Resize(Base::Double length)
{
	Coordinate dir = m_arrCoordinates[0] - m_coordCenter;
	dir.Normalize();
	Coordinate pntOrig = m_coordCenter + dir * length;
	m_arrCoordinates.Clear();

	for (int i = 0; i < 8; i++)
	{
		Coordinate v = pntOrig;
		double angle = i * PI / 4;
		BaseAlgorithm3D::RotatePoint(m_coordCenter, angle, m_vectorNormal, v);
		m_arrCoordinates.Add(v);
	}
}

Base::Bool Engine::Geometries::Octagon::Resize(Base::UInt16 index, const Engine::Geometries::Coordinate &pnt)
{

	Engine::Geometries::Coordinate pntProject;

	BaseAlgorithm3D::GetProjectpntToPlane(pnt, m_coordCenter, m_vectorNormal, pntProject);

	bool rst = BaseAlgorithm3D::GetProjectpntToLine(pntProject, m_coordCenter, m_arrCoordinates[index], pntProject);

	double r = pntProject.Distance(m_coordCenter);

	Resize(r);

	return rst;
}

Base::Void Engine::Geometries::Octagon::Rotate(Base::UInt16 index, Base::Double angle)
{
	if (index < 0 || index > 7)
	{
		return;
	}

	for (int i = 0; i < 8; i++)
	{
		Coordinate v = m_arrCoordinates[i];
		BaseAlgorithm3D::RotatePoint(m_coordCenter, angle, m_vectorNormal, v);
		m_arrCoordinates.SetAt(i, v);
	}
}

Octagon *Engine::Geometries::Octagon::FromPolygon(Geometries::Geometry *pGeometry)
{
	if (pGeometry == nullptr)
		return nullptr;

	if (GeometryType::POLYGON != pGeometry->GetGeometryType())
		return nullptr;

	Polygon *pPolygon = static_cast<Polygon *>(pGeometry);
	Base::Array<Coordinate *> *pArrCoords = pPolygon->GetExteriorRing()->GetCoordinates();

	if (pArrCoords->GetCount() != 4)
	{
		return nullptr;
	}
	else
	{
		return new Octagon(*(pArrCoords->GetAt(0)), *(pArrCoords->GetAt(1)), *(pArrCoords->GetAt(2)));
	}
	return nullptr;
}

Polygon *Engine::Geometries::Octagon::ToPolygon()
{
	Base::Array<Coordinate *> *pArrCoords = new Base::Array<Coordinate *>;
	for (int i = 0; i < 8; i++)
	{
		pArrCoords->Add(new Coordinate(m_arrCoordinates[i]));
	}
	LinearRing *lr = new LinearRing(pArrCoords);
	Polygon *pPolygon = new Polygon(lr, nullptr);
	return pPolygon;
}

Base::Void Geometries::Octagon::RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	auto arrContral = GetControlPoints();
	if (arrContral.GetCount() < 8)
	{
		return;
	}
	// if (index < 4)//角点旋转
	{
		auto &oldControlPoint = arrContral[index];
		auto &newControlPoint = pt;
		auto &centerPoint = /*GetCenter()*/ m_coordCenter;
		Base::Double angle = GeometryAlgorithm::ComputeAngle(oldControlPoint, centerPoint, newControlPoint);
		Base::UInt16 ptMatchLine = BaseAlgorithm3D::PntMatchLine(centerPoint, oldControlPoint, newControlPoint, m_vectorNormal /*GetNormalVector()*/);
		if (ptMatchLine == 0)
		{
			return;
		}

		if (ptMatchLine == 2)
		{
			angle *= -1;
		}

		Rotate(index, angle);
	}
}

Base::Void Geometries::Octagon::SetLength(Base::Double length)
{
	Base::Double angle = GeometryAlgorithm::DegreeToRadian(67.5);

	Base::Double radius = length / 2 / cos(angle); // 算外接圆半径

	Resize(radius);
}

GeometryType Octagon::GetGeometryType() const
{
	return GeometryType::OCTAGON;
}

Envelope *const Octagon::GetEnvelope() const
{
	Coordinate c = m_arrCoordinates.GetAt(0);
	double minx = c.x;
	double miny = c.y;
	double maxx = c.x;
	double maxy = c.y;
	Base::SizeT npts = m_arrCoordinates.GetCount();
	for (Base::SizeT i = 1; i < npts; ++i)
	{
		c = m_arrCoordinates.GetAt(i);
		minx = minx < c.x ? minx : c.x;
		maxx = maxx > c.x ? maxx : c.x;
		miny = miny < c.y ? miny : c.y;
		maxy = maxy > c.y ? maxy : c.y;
	}
	return new Envelope(minx, miny, maxx, maxy);
}

Geometry *const Octagon::Clone() const
{
	Octagon *octagon = new Octagon(*this);
	return octagon;
}

void Octagon::ComputerCenter(const Engine::Geometries::Coordinate &pntSegFrom,
							 const Engine::Geometries::Coordinate &pntSegTo,
							 const Engine::Geometries::Coordinate &pnt)
{
	ComputerNormal(pntSegFrom, pntSegTo, pnt);
	Coordinate center = pntSegTo; // 先将中心设为B点
	Base::Double angle = GeometryAlgorithm::DegreeToRadian(67.5);
	BaseAlgorithm3D::RotatePoint(pntSegFrom, angle, m_vectorNormal, center); // 绕着A旋转67.5度
	Coordinate dir = center - pntSegFrom;
	dir.Normalize();

	Base::Double len1 = pntSegFrom.Distance(pntSegTo) / 2.0;
	Base::Double radius = len1 / cos(angle); // 算外接圆半径
	m_coordCenter = pntSegFrom + dir * radius;
}

void Octagon::ComputerNormal(const Engine::Geometries::Coordinate &pntSegFrom,
							 const Engine::Geometries::Coordinate &pntSegTo,
							 const Engine::Geometries::Coordinate &pnt)
{
	Coordinate v1 = pntSegTo - pntSegFrom;
	Coordinate v2 = pntSegTo - pnt;
	Coordinate normal = v2.CrossProduct(v1);
	normal.Normalize();
	m_vectorNormal = normal;
}
