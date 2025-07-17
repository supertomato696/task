#include "Geometries/Polygon.h"
/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Circle.cpp
简要描述:圆
******************************************************************/

#include "Geometries/Circle.h"
#include "Base/Types.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/Polygon.h"
#include "Geometries/LinearRing.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

Void Circle::ComputeCenter()
{
	/*Engine::Geometries::Coordinate vectemp = m_coordinate1;
	m_coordinate1 = m_coordinate1 - vectemp;
	m_coordinate2 = m_coordinate2 - vectemp;
	m_coordinate3 = m_coordinate3 - vectemp;
	Engine::Geometries::Coordinate vec_12 = m_coordinate2 - m_coordinate1;
	Engine::Geometries::Coordinate vec_13 = m_coordinate3 - m_coordinate1;
	Engine::Geometries::Coordinate vec_normal = vec_12.CrossProduct(vec_13);
	vec_normal.Normalize();
	Engine::Geometries::Coordinate pntmid_12 = (m_coordinate1 + m_coordinate2) / 2.0;
	Engine::Geometries::Coordinate pntmid_13 = (m_coordinate1 + m_coordinate3) / 2.0;
	Double AX = vec_normal.x * vec_12.z - vec_normal.z * vec_12.x;
	Double AY = vec_normal.y * vec_12.z - vec_normal.z * vec_12.y;
	auto a1 = pntmid_12.x * vec_normal.x * vec_12.z;
	auto a2 = pntmid_12.y * vec_normal.y * vec_12.z;
	auto a3 = pntmid_12.x * vec_normal.z * vec_12.x;
	auto a4 = pntmid_12.y * vec_normal.z * vec_12.y;
	Double A = pntmid_12.x * vec_normal.x * vec_12.z + pntmid_12.y * vec_normal.y * vec_12.z -
		pntmid_12.x * vec_normal.z * vec_12.x - pntmid_12.y * vec_normal.z * vec_12.y;
	Double BX = vec_normal.x * vec_13.z - vec_normal.z * vec_13.x;
	Double BY = vec_normal.y * vec_13.z - vec_normal.z * vec_13.y;
	Double B = pntmid_13.x * vec_normal.x * vec_13.z + pntmid_13.y * vec_normal.y * vec_13.z -
		pntmid_13.x * vec_normal.z * vec_13.x - pntmid_13.y * vec_normal.z * vec_13.y;
	m_center.x = (A * BY - B * AY) / (AX * BY - BX * AY);
	m_center.y = (A * BX - B * AX) / (AY * BX - BY * AX);
	m_center.z = ((pntmid_12.x - m_center.x) * vec_normal.x
		+ (pntmid_12.y - m_center.y) * vec_normal.y
		+ pntmid_12.z * vec_normal.z) / vec_normal.z;
	m_radius = sqrt(pow(m_coordinate1.x - m_center.x, 2) + pow(m_coordinate1.y - m_center.y, 2) + pow(m_coordinate1.z - m_center.z, 2));
	m_coordinate1 = m_coordinate1 + vectemp;
	m_coordinate2 = m_coordinate2 + vectemp;
	m_coordinate3 = m_coordinate3 + vectemp;
	m_center = m_center + vectemp;*/

	Double a = m_coordinate1.Distance(m_coordinate2);
	Double b = m_coordinate2.Distance(m_coordinate3);
	Double c = m_coordinate3.Distance(m_coordinate1);

	if (a == 0.0 || b == 0.0 || c == 0.0)
	{
		m_center = (m_coordinate1 + m_coordinate2 + m_coordinate3) / 3.0;
		m_radius = (a + b + c) / 2.0;
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
			m_radius = (a * b * c) / (4 * S);
			Coordinate vec21 = m_coordinate1 - m_coordinate2;
			Engine::Geometries::Coordinate vec_12 = m_coordinate2 - m_coordinate1;
			Engine::Geometries::Coordinate vec_13 = m_coordinate3 - m_coordinate1;
			Engine::Geometries::Coordinate vec_normal = vec_12.CrossProduct(vec_13);
			vec_normal.Normalize();
			Coordinate vec = vec21.CrossProduct(vec_normal);
			vec.Normalize();
			Engine::Geometries::Coordinate pntmid_12 = (m_coordinate1 + m_coordinate2) / 2.0;
			Double disMid = m_coordinate1.Distance(pntmid_12);

			if (fabs(m_radius - disMid) < Geometries_EP)
			{
				m_center = pntmid_12;
			}
			else
			{
				Double dis = sqrt(pow(m_radius, 2) - pow(disMid, 2));
				m_center = pntmid_12 + vec * dis;
			}
		}
	}
}

Void Circle::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_12 = m_coordinate2 - m_coordinate1;
	Engine::Geometries::Coordinate vec_13 = m_coordinate3 - m_coordinate1;
	vec_12.Normalize();
	vec_13.Normalize();

	if (vec_12.Distance(vec_13) < Geometries_EP) // 如果两者平行
	{
		m_normalVector = vec_12.CrossProduct(Engine::Geometries::Coordinate(0.0, 0.0, 1.0));
	}
	else
	{
		m_normalVector = vec_12.CrossProduct(vec_13);
		m_normalVector.Normalize();
	}
}

Circle::Circle(const Engine::Geometries::Coordinate &pnt1,
			   const Engine::Geometries::Coordinate &pnt2,
			   const Engine::Geometries::Coordinate &pnt3)
	: m_radius(0.0)
{
	m_coordinate1 = pnt1;
	m_coordinate2 = pnt2;
	m_coordinate3 = pnt3;
	ComputeCenter();
	ComputeNormalVector();
	Coordinate vec_1center = m_center - m_coordinate1;
	vec_1center.Normalize();
	m_coordinate2 = m_coordinate1 + vec_1center * (2 * m_radius);
	Double dAngle = 90;
	dAngle = dAngle / 180.0 * PI;
	m_coordinate3 = m_coordinate2;
	BaseAlgorithm3D::RotatePoint(m_center, dAngle, m_normalVector, m_coordinate3);
}

Circle::Circle(const Engine::Base::Array<Coordinate> &controlPoints) : m_radius(0.0)
{
	if (controlPoints.GetCount() >= 8)
	{
		Engine::Geometries::Coordinate vec_12 = controlPoints[0] - controlPoints[1];
		Engine::Geometries::Coordinate vec_13 = controlPoints[1] - controlPoints[2];
		auto normalVector = vec_12.CrossProduct(vec_13);
		normalVector.Normalize();

		if (controlPoints[0].Distance(controlPoints[1]) > controlPoints[1].Distance(controlPoints[2]))
		{
			Engine::Geometries::Coordinate tmpcoordinate = controlPoints[4];

			Engine::Geometries::Coordinate center = (controlPoints[4] + controlPoints[6]) / 2;

			Double dAngle = 90;
			dAngle = dAngle / 180.0 * PI;
			BaseAlgorithm3D::RotatePoint(center, dAngle, normalVector, tmpcoordinate);

			*this = Circle(tmpcoordinate, controlPoints[4], controlPoints[6]);
		}
		else
		{
			Engine::Geometries::Coordinate tmpcoordinate = controlPoints[5];

			Engine::Geometries::Coordinate center = (controlPoints[5] + controlPoints[7]) / 2;

			Double dAngle = 90;
			dAngle = dAngle / 180.0 * PI;
			BaseAlgorithm3D::RotatePoint(center, dAngle, normalVector, tmpcoordinate);

			*this = Circle(tmpcoordinate, controlPoints[5], controlPoints[7]);
		}
	}
}

Circle::~Circle()
{
}

// Circle::Circle(const Circle &rhs)
// {
// }
//
// Circle& Circle::operator =(const Circle &rhs)
// {
// 	//检查自赋值
// 	if (this != &rhs)
// 	{
// 	}
//
// 	return *this;
// }

GeometryType Circle::GetGeometryType() const
{
	return GeometryType::CIRCLE;
}

Envelope *const Circle::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> Circle::GetControlPoints()
{
	ComputeCenter();
	Array<Coordinate> arr_ControlPoints;
	Coordinate vec_3center = m_center - m_coordinate3;
	vec_3center.Normalize();
	Coordinate coordinate1 = m_coordinate1 + vec_3center * m_radius;
	arr_ControlPoints.Add(coordinate1);
	Coordinate coordinate2 = m_coordinate2 + vec_3center * m_radius;
	arr_ControlPoints.Add(coordinate2);
	Coordinate coordinate3 = m_coordinate2 + (vec_3center * -1.0) * m_radius;
	arr_ControlPoints.Add(coordinate3);
	Coordinate coordinate4 = m_coordinate1 + (vec_3center * -1.0) * m_radius;
	arr_ControlPoints.Add(coordinate4);
	arr_ControlPoints.Add((coordinate1 + coordinate2) / 2.0);
	arr_ControlPoints.Add((coordinate2 + coordinate3) / 2.0);
	arr_ControlPoints.Add((coordinate3 + coordinate4) / 2.0);
	arr_ControlPoints.Add((coordinate4 + coordinate1) / 2.0);

	return arr_ControlPoints;
}

Geometry *const Circle::Clone() const
{
	Circle *circle = new Circle(*this);

	return circle;
}

Array<Coordinate> Circle::GetCoordinates()
{
	Array<Coordinate> coordinates;
	coordinates.Add(m_coordinate1);
	coordinates.Add(m_coordinate2);
	coordinates.Add(m_coordinate3);

	return coordinates;
}

Coordinate Circle::GetCenter()
{
	ComputeCenter();

	return m_center;
}

Coordinate Circle::GetNormalVector()
{
	return m_normalVector;
}

Double Circle::GetRadius()
{
	ComputeCenter();

	return m_radius;
}

Void Circle::Offset(Double dx, Double dy, Double dz)
{
	ComputeCenter();
	m_coordinate1.x += dx;
	m_coordinate1.y += dy;
	m_coordinate1.z += dz;
	m_coordinate2.x += dx;
	m_coordinate2.y += dy;
	m_coordinate2.z += dz;
	m_coordinate3.x += dx;
	m_coordinate3.y += dy;
	m_coordinate3.z += dz;
	m_center.x += dx;
	m_center.y += dy;
	m_center.z += dz;
}

Void Circle::Resize(Double radius)
{
	if (radius <= 0)
	{
		return;
	}

	ComputeCenter();
	Engine::Geometries::Coordinate vec_center1 = m_coordinate1 - m_center;
	vec_center1.Normalize();
	Engine::Geometries::Coordinate vec_center2 = m_coordinate2 - m_center;
	vec_center2.Normalize();
	Engine::Geometries::Coordinate vec_center3 = m_coordinate3 - m_center;
	vec_center3.Normalize();
	m_coordinate1 = m_center + vec_center1 * radius;
	m_coordinate2 = m_center + vec_center2 * radius;
	m_coordinate3 = m_center + vec_center3 * radius;
	m_radius = radius;
}

Void Circle::Rotate(UInt16 index, Double angle)
{
	Array<Coordinate> arr_ControlPoints = GetControlPoints();

	if (index == 4)
	{
		Coordinate vec_23 = arr_ControlPoints[3] - arr_ControlPoints[2];
		vec_23.Normalize();
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[2], angle, vec_23, m_coordinate1);
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[2], angle, vec_23, m_coordinate2);
	}
	else if (index == 5)
	{
		Coordinate vec_30 = arr_ControlPoints[0] - arr_ControlPoints[3];
		vec_30.Normalize();
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[3], angle, vec_30, m_coordinate2);
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[3], angle, vec_30, m_coordinate3);
	}
	else if (index == 6)
	{
		Coordinate vec_01 = arr_ControlPoints[1] - arr_ControlPoints[0];
		vec_01.Normalize();
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[0], angle, vec_01, m_coordinate1);
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[0], angle, vec_01, m_coordinate2);
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[0], angle, vec_01, m_coordinate3);
	}
	else if (index == 7)
	{
		Coordinate vec_12 = arr_ControlPoints[2] - arr_ControlPoints[1];
		vec_12.Normalize();
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[1], angle, vec_12, m_coordinate1);
		BaseAlgorithm3D::RotatePoint(arr_ControlPoints[1], angle, vec_12, m_coordinate3);
	}
}

Polygon *Circle::ToPolygon(UInt16 subsectionnum)
{
	Array<Coordinate *> *shellCoords = new Array<Coordinate *>();
	Coordinate *shellCoord = new Coordinate();
	shellCoord->x = m_coordinate1.x;
	shellCoord->y = m_coordinate1.y;
	shellCoord->z = m_coordinate1.z;
	shellCoords->Add(shellCoord);
	Double j = 360.0 / ((Double)subsectionnum);
	Coordinate pntResult;
	Double k = j;
	k = k / 180.0 * PI;

	for (Double i = j; i < 360.0; i += j)
	{
		pntResult = *shellCoord;
		BaseAlgorithm3D::RotatePoint(m_center, k, m_normalVector, pntResult);
		shellCoord = new Coordinate();
		shellCoord->x = pntResult.x;
		shellCoord->y = pntResult.y;
		shellCoord->z = pntResult.z;
		shellCoords->Add(shellCoord);
	}

	shellCoord = new Coordinate();
	shellCoord->x = m_coordinate1.x;
	shellCoord->y = m_coordinate1.y;
	shellCoord->z = m_coordinate1.z;
	shellCoords->Add(shellCoord);
	LinearRing *shell = new LinearRing(shellCoords);
	Polygon *py = new Polygon(shell);

	return py;
}

Circle *Circle::FromPolygon(Geometry *pGeometry)
{
	if (pGeometry == nullptr)
		return nullptr;
	if (GeometryType::POLYGON != pGeometry->GetGeometryType())
		return nullptr;
	Polygon *pPolygon = static_cast<Polygon *>(pGeometry);
	Array<Coordinate *> *pArrCoords = pPolygon->GetExteriorRing()->GetCoordinates();

	if (pArrCoords->GetCount() != 4)
	{
		return nullptr;
	}
	else
	{
		return new Circle(*(pArrCoords->GetAt(0)), *(pArrCoords->GetAt(1)), *(pArrCoords->GetAt(2)));
	}
	return nullptr;
}

Int32 Engine::Geometries::Circle::GetOppositeControlPointIndex(Base::Int32 index)
{
	switch (index)
	{
	case 0:
		return 2;
	case 1:
		return 3;
	case 2:
		return 0;
	case 3:
		return 1;
	case 4:
		return 6;
	case 5:
		return 7;
	case 6:
		return 4;
	case 7:
		return 5;
	default:
		return -1;
	}
	return -1;
}

Void Engine::Geometries::Circle::RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	auto arrContral = GetControlPoints();
	if (arrContral.GetCount() < 8)
	{
		return;
	}
	if (index < 4) // 圆不用角点旋转
	{
		// 		auto& oldControlPoint = arrContral[index];
		// 		auto& newControlPoint = pt;
		// 		auto& centerPoint = GetCenter();
		// 		Double angle = GeometryAlgorithm::ComputeAngle(oldControlPoint, centerPoint, newControlPoint);
		// 		Base::UInt16 ptMatchLine = BaseAlgorithm3D::PntMatchPlane(newControlPoint, centerPoint, GetNormalVector());
		// 		//Base::UInt16 ptMatchLine = BaseAlgorithm3D::PntMatchLine(centerPoint, oldControlPoint, newControlPoint, GetControlPointFaceVector(index));
		// 		if (ptMatchLine == 1)
		// 		{
		// 			angle = -angle;
		// 		}
		//
		// 		if (ptMatchLine == 2)
		// 		{
		// 			angle = -angle;
		// 		}
		// 		Rotate(angle);
	}
	else // 边点旋转
	{
		auto &oldControlPoint = arrContral[index];
		auto &newControlPoint = pt;
		auto &oppositeControlPoint = arrContral[GetOppositeControlPointIndex(index)];
		Double angle = GeometryAlgorithm::ComputeAngle(oldControlPoint, oppositeControlPoint, newControlPoint);
		Base::UInt16 ptMatchLine = BaseAlgorithm3D::PntMatchPlane(newControlPoint, oppositeControlPoint, GetNormalVector());
		if (ptMatchLine == 1)
		{
			angle = -angle;
		}
		angle = -angle; // 实测再转一次才正常
		Rotate(index, angle);
	}
}

Engine::Geometries::Coordinate Engine::Geometries::Circle::GetControlPointFaceVector(Base::Int32 index)
{
	Coordinate restlt;

	auto arrContral = GetControlPoints();
	if (arrContral.GetCount() < 8)
	{
		return restlt;
	}
	switch (index)
	{
	case 4:
		restlt = arrContral[1] - arrContral[0];
		break;
	case 5:
		restlt = arrContral[2] - arrContral[1];

		break;
	case 6:
		restlt = arrContral[3] - arrContral[2];

		break;
	case 7:
		restlt = arrContral[0] - arrContral[3];

		break;
	default:
		break;
	}
	restlt.Normalize();
	return restlt;
}

Void Engine::Geometries::Circle::GetControlPointFaceCoords(Base::UInt16 index, Engine::Geometries::Coordinate &pt1, Engine::Geometries::Coordinate &pt2, Engine::Geometries::Coordinate &pt3)
{
	Coordinate vec1 = GetControlPointFaceVector(index);
	if (abs(vec1.z - 0.0) < 0.0001)
	{
		vec1.z = 0.0001; // 防止除0
	}
	auto arrContral = GetControlPoints();
	if (arrContral.GetCount() < 8)
	{
		return;
	}
	Coordinate indexCoord = arrContral[index];
	// 获取第三点z
	// 设 x=y=0
	double z = (indexCoord.x * vec1.x + indexCoord.y * vec1.y) / vec1.z + indexCoord.z;

	pt1 = arrContral[index];
	pt2 = arrContral[GetOppositeControlPointIndex(index)];
	pt3 = Coordinate(0, 0, z);
}
