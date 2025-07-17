/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Square.cpp
简要描述:正方形
******************************************************************/

#include "Geometries/Square.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Base/Types.h"
#include "Base/Macros.h"
#include "math.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

const Double Square_EP = 1.0e-5;

Void Square::ComputeCenter()
{
	m_center.x = (m_coordinates[0].x + m_coordinates[2].x) / 2.0;
	m_center.y = (m_coordinates[0].y + m_coordinates[2].y) / 2.0;
	m_center.z = (m_coordinates[0].z + m_coordinates[2].z) / 2.0;
}

Void Square::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	Engine::Geometries::Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
	m_normalVector = vec_01.CrossProduct(vec_12);
	m_normalVector.Normalize();
}

Square::Square(const Engine::Geometries::Coordinate &pntSegFrom,
			   const Engine::Geometries::Coordinate &pntSegTo,
			   const Engine::Geometries::Coordinate &pnt) : Rectangle(pntSegFrom, pntSegTo, pnt)
{
	Engine::Geometries::Coordinate pntProject;
	Bool b = BaseAlgorithm3D::GetProjectpntToLine(pnt, pntSegFrom, pntSegTo, pntProject);
	Engine::Geometries::Coordinate vec_pntProject_pnt = pnt - pntProject;
	vec_pntProject_pnt.Normalize();
	Double len = sqrt(pow(pntSegFrom.x - pntSegTo.x, 2) + pow(pntSegFrom.y - pntSegTo.y, 2) + pow(pntSegFrom.z - pntSegTo.z, 2));
	Engine::Geometries::Coordinate pntSegFrom1 = pntSegFrom + vec_pntProject_pnt * len;
	Engine::Geometries::Coordinate pntSegTo1 = pntSegTo + vec_pntProject_pnt * len;
	m_coordinates.Clear();
	m_coordinates.Add(pntSegFrom);
	m_coordinates.Add(pntSegTo);
	m_coordinates.Add(pntSegTo1);
	m_coordinates.Add(pntSegFrom1);
	ComputeCenter();
	ComputeNormalVector();
}

Square::Square(const Engine::Base::Array<Engine::Geometries::Coordinate> &controlPoints)
{
	if (controlPoints.GetCount() >= 8)
	{
		*this = Square(controlPoints[0], controlPoints[1], controlPoints[6]);
	}
}

Square::~Square()
{
}

// Square::Square(const Square &rhs) :Rectangle(rhs)
// {
// }
//
// Square& Square::operator =(const Square &rhs)
// {
// 	//检查自赋值
// 	if (this != &rhs)
// 	{
// 	}
//
// 	return *this;
// }

GeometryType Square::GetGeometryType() const
{
	return GeometryType::SQUARE;
}

Envelope *const Square::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> Square::GetCoordinates()
{
	return m_coordinates;
}

Array<Coordinate> Square::GetControlPoints()
{
	Array<Coordinate> coordinates;
	;
	coordinates.Add(m_coordinates[0]);
	coordinates.Add(m_coordinates[1]);
	coordinates.Add(m_coordinates[2]);
	coordinates.Add(m_coordinates[3]);
	coordinates.Add((m_coordinates[0] + m_coordinates[1]) / 2.0);
	coordinates.Add((m_coordinates[1] + m_coordinates[2]) / 2.0);
	coordinates.Add((m_coordinates[2] + m_coordinates[3]) / 2.0);
	coordinates.Add((m_coordinates[3] + m_coordinates[0]) / 2.0);

	return coordinates;
}
//
// Geometry* const Square::Clone() const
//{
//	Square *square = new Square(*this);
//	return square;
//}

Coordinate Square::GetCenter()
{
	ComputeCenter();

	return m_center;
}

Void Square::Offset(Double dx, Double dy, Double dz)
{
	for (Int32 i = 0; i < m_coordinates.GetCount(); i++)
	{
		m_coordinates[i].x += dx;
		m_coordinates[i].y += dy;
		m_coordinates[i].z += dz;
	}
}

Bool Square::Resize(UInt16 index, const Engine::Geometries::Coordinate &pnt)
{
	/*四个边相邻两边，两两做点积操作，即计算夹角。如果夹角有一个不是90度，则不是符合规定的正方形*/

	Coordinate vector_01 = m_coordinates[1] - m_coordinates[0];
	Coordinate vector_12 = m_coordinates[2] - m_coordinates[1];
	Coordinate vector_23 = m_coordinates[3] - m_coordinates[2];
	Coordinate vector_30 = m_coordinates[0] - m_coordinates[3];
	vector_01.Normalize();
	vector_12.Normalize();
	vector_23.Normalize();
	vector_30.Normalize();
	Double dot1 = vector_01.DotProduct(vector_12);
	Double dot2 = vector_12.DotProduct(vector_23);
	Double dot3 = vector_23.DotProduct(vector_30);
	Double dot4 = vector_30.DotProduct(vector_01);

	if (abs(dot1) > Square_EP || abs(dot2) > Square_EP || abs(dot3) > Square_EP || abs(dot4) > Square_EP)
	{
		return false;
	}

	/**/

	Engine::Geometries::Coordinate pntProject;
	BaseAlgorithm3D::GetProjectpntToPlane(pnt, m_center, m_normalVector, pntProject);
	Coordinate vec_center_t = pntProject - m_center;
	Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	vec_01.Normalize();
	Double len_01 = fabs(vec_center_t.DotProduct(vec_01));
	Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
	vec_12.Normalize();
	Double len_12 = fabs(vec_center_t.DotProduct(vec_12));
	Double len = min(len_01, len_12);
	Double angle = 45.0 / 180.0 * PI;
	len = len / sin(angle);
	Coordinate vec = m_coordinates[0] - m_center;
	vec.Normalize();
	m_coordinates[0] = m_center + vec * len;
	vec = m_coordinates[1] - m_center;
	vec.Normalize();
	m_coordinates[1] = m_center + vec * len;
	vec = m_coordinates[2] - m_center;
	vec.Normalize();
	m_coordinates[2] = m_center + vec * len;
	vec = m_coordinates[3] - m_center;
	vec.Normalize();
	m_coordinates[3] = m_center + vec * len;

	return true;
}

Void Square::Rotate(Double angle)
{
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[0]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[1]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[2]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[3]);
}

Void Square::Rotate(UInt16 index, Double angle)
{
	if (index == 4)
	{
		Coordinate vec_23 = m_coordinates[3] - m_coordinates[2];
		vec_23.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinates[2], angle, vec_23, m_coordinates[0]);
		BaseAlgorithm3D::RotatePoint(m_coordinates[2], angle, vec_23, m_coordinates[1]);
	}
	else if (index == 5)
	{
		Coordinate vec_30 = m_coordinates[0] - m_coordinates[3];
		vec_30.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinates[3], angle, vec_30, m_coordinates[1]);
		BaseAlgorithm3D::RotatePoint(m_coordinates[3], angle, vec_30, m_coordinates[2]);
	}
	else if (index == 6)
	{
		Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
		vec_01.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinates[0], angle, vec_01, m_coordinates[2]);
		BaseAlgorithm3D::RotatePoint(m_coordinates[0], angle, vec_01, m_coordinates[3]);
	}
	else if (index == 7)
	{
		Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
		vec_12.Normalize();
		BaseAlgorithm3D::RotatePoint(m_coordinates[1], angle, vec_12, m_coordinates[3]);
		BaseAlgorithm3D::RotatePoint(m_coordinates[1], angle, vec_12, m_coordinates[0]);
	}
}

Double Square::GetLength()
{
	Double length = m_coordinates[0].Distance(m_coordinates[1]);

	return length;
}

Void Square::SetLength(Double length)
{
	ComputeCenter();
	Coordinate vec_center_0 = m_coordinates[0] - m_center;
	vec_center_0.Normalize();
	Coordinate vec_center_1 = m_coordinates[1] - m_center;
	vec_center_1.Normalize();
	Coordinate vec_center_2 = m_coordinates[2] - m_center;
	vec_center_2.Normalize();
	Coordinate vec_center_3 = m_coordinates[3] - m_center;
	vec_center_3.Normalize();
	Double angle = 45;
	angle = angle / 180.0 * PI;
	length = length / 2.0 / cos(angle);
	m_coordinates[0] = m_center + vec_center_0 * length;
	m_coordinates[1] = m_center + vec_center_1 * length;
	m_coordinates[2] = m_center + vec_center_2 * length;
	m_coordinates[3] = m_center + vec_center_3 * length;
}

Int32 Engine::Geometries::Square::GetOppositeControlPointIndex(Base::Int32 index)
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

Void Engine::Geometries::Square::RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt)
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

Engine::Geometries::Coordinate Engine::Geometries::Square::GetControlPointFaceVector(Base::Int32 index)
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

Void Engine::Geometries::Square::GetControlPointFaceCoords(Base::UInt16 index, Engine::Geometries::Coordinate &pt1, Engine::Geometries::Coordinate &pt2, Engine::Geometries::Coordinate &pt3)
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
