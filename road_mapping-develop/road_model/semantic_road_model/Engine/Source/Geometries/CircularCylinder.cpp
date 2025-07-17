/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:CircularCylinder.cpp
简要描述:斜圆柱体
******************************************************************/

#include "Geometries/CircularCylinder.h"
#include "Base/Types.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

CircularCylinder::CircularCylinder()
{
}

Void Engine::Geometries::CircularCylinder::ComputeCenters()
{
	ComputeBottomCenters();
	ComputeTopCenters();
}

Void CircularCylinder::ComputeBottomCenters()
{
	Double a = m_coordinates.GetAt(0).Distance(m_coordinates.GetAt(1));
	Double b = m_coordinates.GetAt(1).Distance(m_coordinates.GetAt(2));
	Double c = m_coordinates.GetAt(2).Distance(m_coordinates.GetAt(0));

	if (a == 0.0 || b == 0.0 || c == 0.0)
	{
		m_center1 = (m_coordinates.GetAt(0) + m_coordinates.GetAt(1) + m_coordinates.GetAt(2)) / 3.0;
		m_radius1 = (a + b + c) / 4;
	}
	else
	{
		Double p = (a + b + c) / 2;

		if (p < c)
		{
			p = c;
		}

		Double S = sqrt(p * (p - a) * (p - b) * (p - c));
		Double h_a = (2 * S) / a;
		Double h_b = (2 * S) / b;
		Double h_c = (2 * S) / c;

		if (h_a < 0.00000000001 || h_b < 0.00000000001 || h_c < 0.00000000001)
		{
		}
		else
		{
			m_radius1 = (a * b * c) / (4 * S);
			Coordinate vec21 = m_coordinates.GetAt(0) - m_coordinates.GetAt(1);
			Engine::Geometries::Coordinate vec_12 = m_coordinates.GetAt(1) - m_coordinates.GetAt(0);
			Engine::Geometries::Coordinate vec_13 = m_coordinates.GetAt(2) - m_coordinates.GetAt(0);
			Engine::Geometries::Coordinate vec_normal = vec_12.CrossProduct(vec_13);
			vec_normal.Normalize();
			Coordinate vec = vec21.CrossProduct(vec_normal);
			vec.Normalize();
			Engine::Geometries::Coordinate pntmid_12 = (m_coordinates.GetAt(0) + m_coordinates.GetAt(1)) / 2.0;
			Double disMid = m_coordinates.GetAt(0).Distance(pntmid_12);

			if (fabs(m_radius1 - disMid) < Geometries_EP)
			{
				m_center1 = pntmid_12;
			}
			else
			{
				Double dis = sqrt(pow(m_radius1, 2) - pow(disMid, 2));
				m_center1 = pntmid_12 + vec * dis;
			}
		}
	}
}

Void CircularCylinder::ComputeTopCenters()
{
	Double a = m_coordinates.GetAt(3).Distance(m_coordinates.GetAt(4));
	Double b = m_coordinates.GetAt(4).Distance(m_coordinates.GetAt(5));
	Double c = m_coordinates.GetAt(5).Distance(m_coordinates.GetAt(3));

	if (a == 0.0 || b == 0.0 || c == 0.0)
	{
		m_center2 = (m_coordinates.GetAt(3) + m_coordinates.GetAt(4) + m_coordinates.GetAt(5)) / 3.0;
		m_radius2 = (a + b + c) / 4;
	}
	else
	{
		Double p = (a + b + c) / 2;

		if (p < c)
		{
			p = c;
		}

		Double S = sqrt(p * (p - a) * (p - b) * (p - c));
		Double h_a = (2 * S) / a;
		Double h_b = (2 * S) / b;
		Double h_c = (2 * S) / c;

		if (h_a < 0.00000000001 || h_b < 0.00000000001 || h_c < 0.00000000001)
		{
		}
		else
		{
			m_radius2 = (a * b * c) / (4 * S);
			Coordinate vec21 = m_coordinates.GetAt(3) - m_coordinates.GetAt(4);
			Engine::Geometries::Coordinate vec_12 = m_coordinates.GetAt(4) - m_coordinates.GetAt(3);
			Engine::Geometries::Coordinate vec_13 = m_coordinates.GetAt(5) - m_coordinates.GetAt(3);
			Engine::Geometries::Coordinate vec_normal = vec_12.CrossProduct(vec_13);
			vec_normal.Normalize();
			Coordinate vec = vec21.CrossProduct(vec_normal);
			vec.Normalize();
			Engine::Geometries::Coordinate pntmid_12 = (m_coordinates.GetAt(3) + m_coordinates.GetAt(4)) / 2.0;
			Double disMid = m_coordinates.GetAt(3).Distance(pntmid_12);

			if (fabs(m_radius2 - disMid) < Geometries_EP)
			{
				m_center2 = pntmid_12;
			}
			else
			{
				Double dis = sqrt(pow(m_radius2, 2) - pow(disMid, 2));
				m_center2 = pntmid_12 + vec * dis;
			}
		}
	}
}

Void CircularCylinder::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	Engine::Geometries::Coordinate vec_12 = m_coordinates[2] - m_coordinates[0];
	m_normalVector = vec_01.CrossProduct(vec_12);
	if (m_normalVector.GetLength() == 0.0)
	{
		m_normalVector = Coordinate(0.0, 0.0, 1.0);
	}
	m_normalVector.Normalize();
}

CircularCylinder::CircularCylinder(const Engine::Geometries::Coordinate &pnt1,
								   const Engine::Geometries::Coordinate &pnt2,
								   const Engine::Geometries::Coordinate &pnt3,
								   const Coordinate obliqueVector,
								   const Double height)
{
	m_coordinates.SetSize(6);
	m_coordinates.SetAt(0, pnt1);
	m_coordinates.SetAt(1, pnt2);
	m_coordinates.SetAt(2, pnt3);
	ComputeNormalVector();
	ComputeBottomCenters();

	m_obliqueVector = obliqueVector;
	m_obliqueVector.Normalize();
	m_height = height;
	m_obliqueVectorLength = height / m_obliqueVector.z;

	Coordinate vec_1center = m_center1 - m_coordinates[0];
	vec_1center.Normalize();
	m_coordinates[1] = m_coordinates[0] + vec_1center * (2 * m_radius1);
	Double dAngle = 90;
	dAngle = dAngle / 180.0 * PI;
	m_coordinates[2] = m_coordinates[1];
	BaseAlgorithm3D::RotatePoint(m_center1, dAngle, m_normalVector, m_coordinates[2]);
	m_coordinates.SetAt(3, m_coordinates.GetAt(0) + m_obliqueVector * m_obliqueVectorLength);
	m_coordinates.SetAt(4, m_coordinates.GetAt(1) + m_obliqueVector * m_obliqueVectorLength);
	m_coordinates.SetAt(5, m_coordinates.GetAt(2) + m_obliqueVector * m_obliqueVectorLength);
	ComputeTopCenters();
}

Engine::Geometries::CircularCylinder::CircularCylinder(const Engine::Geometries::Coordinate &top_center,
													   const Engine::Geometries::Coordinate &bottom_center,
													   const double &topDiameter,
													   const double &bottomDiameter)
{
	m_coordinates.SetSize(6);
	m_coordinates.SetAt(0, bottom_center + Geometries::Coordinate(0, bottomDiameter / 2, 0.0));
	m_coordinates.SetAt(1, bottom_center + Geometries::Coordinate(0, -bottomDiameter / 2, 0.0));
	m_coordinates.SetAt(2, bottom_center + Geometries::Coordinate(bottomDiameter / 2, 0, 0.0));
	m_coordinates.SetAt(3, top_center + Geometries::Coordinate(0, topDiameter / 2, 0.0));
	m_coordinates.SetAt(4, top_center + Geometries::Coordinate(0, -topDiameter / 2, 0.0));
	m_coordinates.SetAt(5, top_center + Geometries::Coordinate(topDiameter / 2, 0, 0.0));

	m_obliqueVector = top_center - bottom_center;
	m_obliqueVectorLength = m_obliqueVector.GetLength();
	m_obliqueVector.Normalize();
	m_height = abs(top_center.z - bottom_center.z);

	ComputeTopCenters();
	ComputeBottomCenters();
}

Bool CircularCylinder::Make(const Engine::Geometries::Coordinate &pnt1,
							const Engine::Geometries::Coordinate &pnt2,
							const Engine::Geometries::Coordinate &pnt3,
							const Coordinate obliqueVector,
							const Double height)
{
	Double disPtToLine = BaseAlgorithm3D::DisPtToLine(pnt1, pnt2, pnt3);

	if (disPtToLine < Geometries_EP)
	{
		return false;
	}

	m_coordinates.SetSize(6);
	m_coordinates.SetAt(0, pnt1);
	m_coordinates.SetAt(1, pnt2);
	m_coordinates.SetAt(2, pnt3);
	ComputeNormalVector();
	ComputeBottomCenters();

	m_obliqueVector = obliqueVector;
	m_obliqueVector.Normalize();
	m_height = height;
	m_obliqueVectorLength = height / m_obliqueVector.z;

	Coordinate vec_1center = m_center1 - m_coordinates[0];
	vec_1center.Normalize();
	m_coordinates[1] = m_coordinates[0] + vec_1center * (2 * m_radius1);
	Double dAngle = 90;
	dAngle = dAngle / 180.0 * PI;
	m_coordinates[2] = m_coordinates[1];
	BaseAlgorithm3D::RotatePoint(m_center1, dAngle, m_normalVector, m_coordinates[2]);
	m_coordinates.SetAt(3, m_coordinates.GetAt(0) + m_obliqueVector * m_obliqueVectorLength);
	m_coordinates.SetAt(4, m_coordinates.GetAt(1) + m_obliqueVector * m_obliqueVectorLength);
	m_coordinates.SetAt(5, m_coordinates.GetAt(2) + m_obliqueVector * m_obliqueVectorLength);

	ComputeTopCenters();
	return true;
}

CircularCylinder::~CircularCylinder()
{
}

CircularCylinder::CircularCylinder(const CircularCylinder &rhs)
{
}

CircularCylinder &CircularCylinder::operator=(const CircularCylinder &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
	}

	return *this;
}

GeometryType CircularCylinder::GetGeometryType() const
{
	return GeometryType::OBLIQUECIRCULARCYLINDER;
}

Envelope *const CircularCylinder::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> CircularCylinder::GetControlPoints()
{
	ComputeCenters();
	Array<Coordinate> arr_ControlPoints;
	{
		Coordinate vec_3center = m_center1 - m_coordinates[2];
		vec_3center.Normalize();
		Coordinate coordinate1 = m_coordinates[0] + vec_3center * m_radius1;
		arr_ControlPoints.Add(coordinate1);
		Coordinate coordinate2 = m_coordinates[1] + vec_3center * m_radius1;
		arr_ControlPoints.Add(coordinate2);
		Coordinate vec_center3 = m_coordinates[2] - m_center1;
		vec_center3.Normalize();
		Coordinate coordinate3 = m_coordinates[1] + vec_center3 * m_radius1;
		arr_ControlPoints.Add(coordinate3);
		Coordinate coordinate4 = m_coordinates[0] + vec_center3 * m_radius1;
		arr_ControlPoints.Add(coordinate4);
	}
	{

		Coordinate vec_3center = m_center2 - m_coordinates[5];
		vec_3center.Normalize();
		Coordinate coordinate1 = m_coordinates[3] + vec_3center * m_radius2;
		arr_ControlPoints.Add(coordinate1);
		Coordinate coordinate2 = m_coordinates[4] + vec_3center * m_radius2;
		arr_ControlPoints.Add(coordinate2);
		Coordinate vec_center3 = m_coordinates[5] - m_center2;
		vec_center3.Normalize();
		Coordinate coordinate3 = m_coordinates[4] + vec_center3 * m_radius2;
		arr_ControlPoints.Add(coordinate3);
		Coordinate coordinate4 = m_coordinates[3] + vec_center3 * m_radius2;
		arr_ControlPoints.Add(coordinate4);
	}
	return arr_ControlPoints;
}

Geometry *const CircularCylinder::Clone() const
{
	CircularCylinder *circularCylinder = new CircularCylinder(*this);

	return circularCylinder;
}

Array<Coordinate> CircularCylinder::GetBottomsurfaceCoords()
{
	Array<Coordinate> coords;
	coords.Add(m_coordinates[0]);
	coords.Add(m_coordinates[1]);
	coords.Add(m_coordinates[2]);

	return coords;
}

Coordinate CircularCylinder::GetObliqueVector()
{
	m_obliqueVector.Normalize();

	return m_obliqueVector;
}

Double CircularCylinder::GetHeight()
{
	return m_height;
}

Array<Coordinate> CircularCylinder::GetTopsurfaceCoords()
{
	Array<Coordinate> coords;
	coords.Add(m_coordinates[3]);
	coords.Add(m_coordinates[4]);
	coords.Add(m_coordinates[5]);

	return coords;
}

Coordinate CircularCylinder::GetBottomsurfaceCenter()
{
	ComputeCenters();

	return m_center1;
}

Coordinate CircularCylinder::GetTopsurfaceCenter()
{
	ComputeCenters();

	return m_center2;
}

Coordinate CircularCylinder::GetNormalVector()
{
	return m_normalVector;
}

Double CircularCylinder::GetBottomRadius()
{
	ComputeBottomCenters();

	return m_radius1;
}

Double CircularCylinder::GetTopRadius()
{
	ComputeTopCenters();

	return m_radius2;
}

Void CircularCylinder::Offset(Double dx, Double dy, Double dz)
{
	ComputeCenters();
	m_coordinates[0].x += dx;
	m_coordinates[0].y += dy;
	m_coordinates[0].z += dz;
	m_coordinates[1].x += dx;
	m_coordinates[1].y += dy;
	m_coordinates[1].z += dz;
	m_coordinates[2].x += dx;
	m_coordinates[2].y += dy;
	m_coordinates[2].z += dz;
	m_center1.x += dx;
	m_center1.y += dy;
	m_center1.z += dz;
	m_coordinates[3].x += dx;
	m_coordinates[3].y += dy;
	m_coordinates[3].z += dz;
	m_coordinates[4].x += dx;
	m_coordinates[4].y += dy;
	m_coordinates[4].z += dz;
	m_coordinates[5].x += dx;
	m_coordinates[5].y += dy;
	m_coordinates[5].z += dz;
	m_center2.x += dx;
	m_center2.y += dy;
	m_center2.z += dz;
}

Void CircularCylinder::ResizeBottom(Double radius)
{
	ComputeBottomCenters();
	Engine::Geometries::Coordinate vec_center0 = m_coordinates[0] - m_center1;
	vec_center0.Normalize();
	Engine::Geometries::Coordinate vec_center1 = m_coordinates[1] - m_center1;
	vec_center1.Normalize();
	Engine::Geometries::Coordinate vec_center2 = m_coordinates[2] - m_center1;
	vec_center2.Normalize();
	m_coordinates[0] = m_center1 + vec_center0 * radius;
	m_coordinates[1] = m_center1 + vec_center1 * radius;
	m_coordinates[2] = m_center1 + vec_center2 * radius;
	m_radius1 = radius;
}

Void CircularCylinder::ResizeTop(Double radius)
{
	ComputeTopCenters();
	Engine::Geometries::Coordinate vec_center3 = m_coordinates[3] - m_center2;
	vec_center3.Normalize();
	Engine::Geometries::Coordinate vec_center4 = m_coordinates[4] - m_center2;
	vec_center4.Normalize();
	Engine::Geometries::Coordinate vec_center5 = m_coordinates[5] - m_center2;
	vec_center5.Normalize();
	m_coordinates[3] = m_center2 + vec_center3 * radius;
	m_coordinates[4] = m_center2 + vec_center4 * radius;
	m_coordinates[5] = m_center2 + vec_center5 * radius;
	m_radius2 = radius;
}

Void CircularCylinder::SetTopsurfaceCenter(const Coordinate &pnt)
{
	ComputeTopCenters();
	Coordinate vec_Offset = pnt - m_center2;
	m_center2 = pnt;
	m_obliqueVector = m_center2 - m_center1;
	m_obliqueVectorLength = m_obliqueVector.GetLength();
	m_obliqueVector.Normalize();
	Double disHorizontal = sqrt(pow(m_center1.x - m_center2.x, 2) + pow(m_center1.y - m_center2.y, 2));
	m_height = sqrt(pow(m_obliqueVectorLength, 2) - pow(disHorizontal, 2));
	m_coordinates[3] = m_coordinates[3] + vec_Offset;
	m_coordinates[4] = m_coordinates[4] + vec_Offset;
	m_coordinates[5] = m_coordinates[5] + vec_Offset;
}

Void CircularCylinder::SetBottomsurfaceCenter(const Coordinate &pnt)
{
	ComputeBottomCenters();
	Coordinate vec_Offset = pnt - m_center1;
	m_center1 = pnt;
	m_obliqueVector = m_center2 - m_center1;
	m_obliqueVectorLength = m_obliqueVector.GetLength();
	m_obliqueVector.Normalize();
	Double disHorizontal = sqrt(pow(m_center1.x - m_center2.x, 2) + pow(m_center1.y - m_center2.y, 2));
	m_height = sqrt(pow(m_obliqueVectorLength, 2) - pow(disHorizontal, 2));
	m_coordinates[0] = m_coordinates[0] + vec_Offset;
	m_coordinates[1] = m_coordinates[1] + vec_Offset;
	m_coordinates[2] = m_coordinates[2] + vec_Offset;
}

Polygon *CircularCylinder::BottomsurfaceToPolygon(UInt16 subsectionnum)
{
	Array<Coordinate *> *shellCoords = new Array<Coordinate *>();
	Coordinate *shellCoord = new Coordinate();
	shellCoord->x = m_coordinates[0].x;
	shellCoord->y = m_coordinates[0].y;
	shellCoord->z = m_coordinates[0].z;
	shellCoords->Add(shellCoord);
	Double j = 360.0 / ((Double)subsectionnum);
	Coordinate pntResult;
	Double k = j;
	k = k / 180.0 * PI;

	for (Double i = j; i < 360.0; i += j)
	{
		pntResult = *shellCoord;
		BaseAlgorithm3D::RotatePoint(m_center1, k, m_normalVector, pntResult);
		shellCoord = new Coordinate();
		shellCoord->x = pntResult.x;
		shellCoord->y = pntResult.y;
		shellCoord->z = pntResult.z;
		shellCoords->Add(shellCoord);
	}

	shellCoord = new Coordinate();
	shellCoord->x = m_coordinates[0].x;
	shellCoord->y = m_coordinates[0].y;
	shellCoord->z = m_coordinates[0].z;
	shellCoords->Add(shellCoord);
	LinearRing *shell = new LinearRing(shellCoords);
	Polygon *py = new Polygon(shell);

	return py;
}

Polygon *CircularCylinder::TopsurfaceToPolygon(UInt16 subsectionnum)
{
	Array<Coordinate *> *shellCoords = new Array<Coordinate *>();
	Coordinate *shellCoord = new Coordinate();
	shellCoord->x = m_coordinates[3].x;
	shellCoord->y = m_coordinates[3].y;
	shellCoord->z = m_coordinates[3].z;
	shellCoords->Add(shellCoord);
	Double j = 360.0 / ((Double)subsectionnum);
	Coordinate pntResult;
	Double k = j;
	k = k / 180.0 * PI;

	for (Double i = j; i < 360.0; i += j)
	{
		pntResult = *shellCoord;
		BaseAlgorithm3D::RotatePoint(m_center2, k, m_normalVector, pntResult);
		shellCoord = new Coordinate();
		shellCoord->x = pntResult.x;
		shellCoord->y = pntResult.y;
		shellCoord->z = pntResult.z;
		shellCoords->Add(shellCoord);
	}

	shellCoord = new Coordinate();
	shellCoord->x = m_coordinates[3].x;
	shellCoord->y = m_coordinates[3].y;
	shellCoord->z = m_coordinates[3].z;
	shellCoords->Add(shellCoord);
	LinearRing *shell = new LinearRing(shellCoords);
	Polygon *py = new Polygon(shell);

	return py;
}
