/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Triangle.cpp
简要描述:三角形
******************************************************************/

#include "Geometries/Triangle.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Base/Types.h"
#include "Base/Macros.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

Triangle::Triangle(const Engine::Geometries::Coordinate &pnt1,
				   const Engine::Geometries::Coordinate &pnt2,
				   const Engine::Geometries::Coordinate &pnt3)
{
	Coordinate pnt3Project;
	BaseAlgorithm3D::GetProjectpntToLine(pnt3, pnt1, pnt2, pnt3Project);
	Coordinate vec_3Project_3 = pnt3 - pnt3Project;
	vec_3Project_3.Normalize();
	Double dis_12 = pnt1.Distance(pnt2);
	Double h = sqrt(pow(dis_12, 2) - pow(dis_12 / 2.0, 2));
	Coordinate pntMid_12 = (pnt1 + pnt2) / 2.0;
	m_coordinate1 = pnt1;
	m_coordinate2 = pnt2;
	m_coordinate3 = pntMid_12 + vec_3Project_3 * h;
	ComputeCenter();
	ComputeNormalVector();
}

Triangle::~Triangle()
{
}

Triangle::Triangle(const Triangle &rhs)
{
	m_coordinate1 = rhs.m_coordinate1;
	m_coordinate2 = rhs.m_coordinate2;
	m_coordinate3 = rhs.m_coordinate3;
}

Triangle::Triangle(const Engine::Base::Array<Coordinate> &controlPoints, bool isInv, bool fromInv)
{
	if (controlPoints.GetCount() < 8)
	{
		return;
	}
	if (fromInv)
	{
		*this = Triangle(controlPoints[2], controlPoints[3], controlPoints[4]);
		return;
	}

	if (isInv)
	{
		*this = Triangle(controlPoints[2], controlPoints[3], controlPoints[4]);
	}
	else
	{
		*this = Triangle(controlPoints[0], controlPoints[1], controlPoints[6]);
	}
}

Triangle &Triangle::operator=(const Triangle &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		m_coordinate1 = rhs.m_coordinate1;
		m_coordinate2 = rhs.m_coordinate2;
		m_coordinate3 = rhs.m_coordinate3;
	}

	return *this;
}

GeometryType Triangle::GetGeometryType() const
{
	return GeometryType::TRIANGLE;
}

Envelope *const Triangle::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> Triangle::GetControlPoints()
{
	Array<Coordinate> arr_ControlPoints;
	arr_ControlPoints.Add(m_coordinate1);
	arr_ControlPoints.Add(m_coordinate2);
	Coordinate pnt3Project;
	BaseAlgorithm3D::GetProjectpntToLine(m_coordinate3, m_coordinate1, m_coordinate2, pnt3Project);
	Coordinate vec_3Project_3 = m_coordinate3 - pnt3Project;
	vec_3Project_3.Normalize();
	Double dis_12 = m_coordinate1.Distance(m_coordinate2);
	Double h = sqrt(pow(dis_12, 2) - pow(dis_12 / 2.0, 2));
	Coordinate coordinate3 = m_coordinate2 + vec_3Project_3 * h;
	arr_ControlPoints.Add(coordinate3);
	Coordinate coordinate4 = m_coordinate1 + vec_3Project_3 * h;
	arr_ControlPoints.Add(coordinate4);
	arr_ControlPoints.Add((m_coordinate1 + m_coordinate2) / 2.0);
	arr_ControlPoints.Add((m_coordinate2 + coordinate3) / 2.0);
	arr_ControlPoints.Add((coordinate3 + coordinate4) / 2.0);
	arr_ControlPoints.Add((coordinate4 + m_coordinate1) / 2.0);

	return arr_ControlPoints;
}

Geometry *const Triangle::Clone() const
{
	Triangle *triangle = new Triangle(*this);

	return triangle;
}

Void Triangle::ComputeCenter()
{
	m_center = (m_coordinate1 + m_coordinate2 + m_coordinate3) / 3;
	/*
	Engine::Geometries::Coordinate vec_12 = m_coordinate2 - m_coordinate1;
	Engine::Geometries::Coordinate vec_13 = m_coordinate3 - m_coordinate1;
	Engine::Geometries::Coordinate vec_normal = vec_12.CrossProduct(vec_13);
	vec_normal.Normalize();
	Engine::Geometries::Coordinate pntmid_12 = (m_coordinate1 + m_coordinate2) / 2.0;
	Engine::Geometries::Coordinate pntmid_13 = (m_coordinate1 + m_coordinate3) / 2.0;
	Double AX = vec_normal.x * vec_12.z - vec_normal.z * vec_12.x;
	Double AY = vec_normal.y * vec_12.z - vec_normal.z * vec_12.y;
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
		+ pntmid_12.z * vec_normal.z) / vec_normal.z;*/
}

Void Triangle::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_12 = m_coordinate2 - m_coordinate1;
	Engine::Geometries::Coordinate vec_13 = m_coordinate3 - m_coordinate1;
	m_normalVector = vec_12.CrossProduct(vec_13);
	m_normalVector.Normalize();
}

Array<Coordinate> Triangle::GetCoordinates()
{
	Array<Coordinate> coordinates;
	coordinates.Add(m_coordinate1);
	coordinates.Add(m_coordinate2);
	coordinates.Add(m_coordinate3);

	return coordinates;
}

Coordinate Triangle::GetCenter()
{
	ComputeCenter();

	return m_center;
}

Coordinate Triangle::GetNormalVector()
{
	return m_normalVector;
}

Void Triangle::Offset(Double dx, Double dy, Double dz)
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

Void Triangle::Resize(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	/*等边三角形外包围矩形的四个角点的索引为0到3，索引号为0、1的角点与等边三角形底边的两端的点重合，
	等边三角形外包围矩形的四个边的终点的索引为4到7，等边三角形的中心是O*/

	ComputeCenter();
	Double dis_01 = m_coordinate1.Distance(m_coordinate2);
	Coordinate coordinate4 = (m_coordinate1 + m_coordinate2) / 2.0;
	Double dis_center4 = m_center.Distance(coordinate4);
	Double tan01 = dis_01 / 2.0 / dis_center4;
	Double dis_23 = dis_01;
	Coordinate vec_46 = m_center - coordinate4;
	vec_46.Normalize();
	Double h = sqrt(pow(dis_01, 2) - pow(dis_01 / 2.0, 2));
	Coordinate coordinate2 = m_coordinate2 + vec_46 * h;
	Coordinate coordinate3 = m_coordinate1 + vec_46 * h;
	Coordinate coordinate6 = (coordinate2 + coordinate3) / 2.0;
	Double dis_center6 = m_center.Distance(coordinate6);
	Double tan23 = dis_23 / 2.0 / dis_center6;
	Engine::Geometries::Coordinate pntProject;
	BaseAlgorithm3D::GetProjectpntToPlane(pt, m_center, m_normalVector, pntProject);
	Coordinate vec_center_t = pntProject - m_center;
	Coordinate vec_01 = m_coordinate2 - m_coordinate1;
	vec_01.Normalize();
	Coordinate vec_12 = coordinate2 - m_coordinate2;
	vec_12.Normalize();
	Double len_01 = fabs(vec_center_t.DotProduct(vec_01));
	Double len_12 = fabs(vec_center_t.DotProduct(vec_12));
	Double len = 0.0;

	if (index == 0 || index == 1)
	{
		if (len_01 / len_12 >= tan01)
		{
			Double angle = 60.0 / 180.0 * PI;
			len = len_12 / cos(angle);
		}
		else
		{
			Double angle = 60.0 / 180.0 * PI;
			len = len_01 / sin(angle);
		}
	}
	else if (index == 2 || index == 3)
	{
		if (len_01 / len_12 >= tan01)
		{
			len = len_12;
		}
		else
		{
			Double angle = 60.0 / 180.0 * PI;
			len = len_01 / sin(angle);
		}
	}

	Coordinate vec_center_1 = m_coordinate2 - m_center;
	vec_center_1.Normalize();
	m_coordinate2 = m_center + vec_center_1 * len;
	Coordinate vec_center_0 = m_coordinate1 - m_center;
	vec_center_0.Normalize();
	m_coordinate1 = m_center + vec_center_0 * len;
	m_coordinate3 = m_center + vec_46 * len;

	/**/
}

Void Triangle::Rotate(Double angle)
{
	ComputeCenter();
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinate1);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinate2);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinate3);
}

Void Triangle::Rotate(UInt16 index, Double angle)
{
	if (index == 4)
	{
		Coordinate vec_23 = m_coordinate1 - m_coordinate2;
		vec_23.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinate3, angle, vec_23, m_coordinate1);
		BaseAlgorithm3D::RotatePoint(m_coordinate3, angle, vec_23, m_coordinate2);
	}
	else if (index == 5)
	{
		Coordinate pnt_Mid12 = (m_coordinate1 + m_coordinate2) / 2.0;
		Coordinate vec_3_Mid12 = pnt_Mid12 - m_coordinate3;
		vec_3_Mid12.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinate1, angle, vec_3_Mid12, m_coordinate2);
		BaseAlgorithm3D::RotatePoint(m_coordinate1, angle, vec_3_Mid12, m_coordinate3);
	}
	else if (index == 6)
	{
		Coordinate vec_12 = m_coordinate2 - m_coordinate1;
		vec_12.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinate1, angle, vec_12, m_coordinate3);
	}
	else if (index == 7)
	{
		Coordinate pnt_Mid12 = (m_coordinate1 + m_coordinate2) / 2.0;
		Coordinate vec_Mid12_3 = m_coordinate3 - pnt_Mid12;
		vec_Mid12_3.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinate2, angle, vec_Mid12_3, m_coordinate1);
		BaseAlgorithm3D::RotatePoint(m_coordinate2, angle, vec_Mid12_3, m_coordinate3);
	}
}

Double Triangle::GetLength()
{
	Double length = m_coordinate1.Distance(m_coordinate2);

	return length;
}

Void Triangle::SetLength(Double length)
{
	ComputeCenter();
	Coordinate vec_center_1 = m_coordinate1 - m_center;
	vec_center_1.Normalize();
	Coordinate vec_center_2 = m_coordinate2 - m_center;
	vec_center_2.Normalize();
	Coordinate vec_center_3 = m_coordinate3 - m_center;
	vec_center_3.Normalize();
	Double angle = 30;
	angle = angle / 180.0 * PI;
	length = length / 2.0 / cos(angle);
	m_coordinate1 = m_center + vec_center_1 * length;
	m_coordinate2 = m_center + vec_center_2 * length;
	m_coordinate3 = m_center + vec_center_3 * length;
}

Int32 Engine::Geometries::Triangle::GetOppositeControlPointIndex(Base::Int32 index)
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

Void Engine::Geometries::Triangle::RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	auto arrContral = GetControlPoints();
	if (arrContral.GetCount() < 8)
	{
		return;
	}
	if (index < 4) // 角点旋转
	{
		auto &oldControlPoint = arrContral[index];
		auto &newControlPoint = pt;
		auto centerPoint = GetCenter();
		Double angle = GeometryAlgorithm::ComputeAngle(oldControlPoint, centerPoint, newControlPoint);
		Base::UInt16 ptMatchLine = BaseAlgorithm3D::PntMatchLine(centerPoint, oldControlPoint, newControlPoint, GetNormalVector());
		if (ptMatchLine == 0)
		{
			return;
		}

		if (ptMatchLine == 2)
		{
			angle *= -1;
		}

		Rotate(angle);
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

Engine::Geometries::Coordinate Engine::Geometries::Triangle::GetControlPointFaceVector(Base::Int32 index)
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

Void Engine::Geometries::Triangle::GetControlPointFaceCoords(Base::UInt16 index, Engine::Geometries::Coordinate &pt1, Engine::Geometries::Coordinate &pt2, Engine::Geometries::Coordinate &pt3)
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
