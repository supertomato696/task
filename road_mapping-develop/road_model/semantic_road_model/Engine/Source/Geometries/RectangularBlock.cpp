/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:RectangularBlock.cpp
简要描述:斜矩形块
******************************************************************/

#include "Geometries/RectangularBlock.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Base/Macros.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

RectangularBlock::RectangularBlock()
{
}

Void RectangularBlock::ComputeCenters()
{
	m_center1.x = (m_coordinates[0].x + m_coordinates[2].x) / 2.0;
	m_center1.y = (m_coordinates[0].y + m_coordinates[2].y) / 2.0;
	m_center1.z = (m_coordinates[0].z + m_coordinates[2].z) / 2.0;
	m_center2.x = (m_coordinates[4].x + m_coordinates[6].x) / 2.0;
	m_center2.y = (m_coordinates[4].y + m_coordinates[6].y) / 2.0;
	m_center2.z = (m_coordinates[4].z + m_coordinates[6].z) / 2.0;
}

Void RectangularBlock::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	Engine::Geometries::Coordinate vec_12 = m_coordinates[2] - m_coordinates[0];
	m_normalVector = vec_01.CrossProduct(vec_12);
	m_normalVector.Normalize();
}

RectangularBlock::RectangularBlock(const Engine::Geometries::Coordinate &pntSegFrom,
								   const Engine::Geometries::Coordinate &pntSegTo,
								   const Engine::Geometries::Coordinate &pnt,
								   const Coordinate obliqueVector,
								   const Double height)
{
	Engine::Geometries::Coordinate pntProject;
	Bool b = BaseAlgorithm3D::GetProjectpntToLine(pnt, pntSegFrom, pntSegTo, pntProject);
	Engine::Geometries::Coordinate vec_pntProject_pnt = pnt - pntProject;
	Engine::Geometries::Coordinate pntSegFrom1 = pntSegFrom + vec_pntProject_pnt;
	Engine::Geometries::Coordinate pntSegTo1 = pntSegTo + vec_pntProject_pnt;
	m_coordinates.SetSize(8);
	m_coordinates[0] = pntSegFrom;
	m_coordinates[1] = pntSegTo;
	m_coordinates[2] = pntSegTo1;
	m_coordinates[3] = pntSegFrom1;
	m_obliqueVector = obliqueVector;
	m_obliqueVector.Normalize();
	m_height = height;
	m_obliqueVectorLength = height / m_obliqueVector.z;
	m_coordinates[4] = m_coordinates[0] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[5] = m_coordinates[1] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[6] = m_coordinates[2] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[7] = m_coordinates[3] + m_obliqueVector * m_obliqueVectorLength;
	ComputeCenters();
	Double dis = m_center1.Distance(m_coordinates[0]);

	if (dis < 0.2)
	{
		Coordinate vec = m_coordinates[0] - m_center1;
		vec.Normalize();
		m_coordinates[0] = m_center1 + vec * 0.2;
		vec = m_coordinates[1] - m_center1;
		vec.Normalize();
		m_coordinates[1] = m_center1 + vec * 0.2;
		vec = m_coordinates[2] - m_center1;
		vec.Normalize();
		m_coordinates[2] = m_center1 + vec * 0.2;
		vec = m_coordinates[3] - m_center1;
		vec.Normalize();
		m_coordinates[3] = m_center1 + vec * 0.2;
		m_coordinates[4] = m_coordinates[0] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[5] = m_coordinates[1] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[6] = m_coordinates[2] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[7] = m_coordinates[3] + m_obliqueVector * m_obliqueVectorLength;
	}

	ComputeNormalVector();
}

Bool RectangularBlock::Make(const Engine::Geometries::Coordinate &pntSegFrom,
							const Engine::Geometries::Coordinate &pntSegTo,
							const Engine::Geometries::Coordinate &pnt,
							const Coordinate obliqueVector,
							const Double height)
{
	Double disPtToLine = BaseAlgorithm3D::DisPtToLine(pntSegFrom, pntSegTo, pnt);

	if (disPtToLine < Geometries_EP)
	{
		return false;
	}

	Engine::Geometries::Coordinate pntProject;
	Bool b = BaseAlgorithm3D::GetProjectpntToLine(pnt, pntSegFrom, pntSegTo, pntProject);
	Engine::Geometries::Coordinate vec_pntProject_pnt = pnt - pntProject;
	Engine::Geometries::Coordinate pntSegFrom1 = pntSegFrom + vec_pntProject_pnt;
	Engine::Geometries::Coordinate pntSegTo1 = pntSegTo + vec_pntProject_pnt;
	m_coordinates.SetSize(8);
	m_coordinates[0] = pntSegFrom;
	m_coordinates[1] = pntSegTo;
	m_coordinates[2] = pntSegTo1;
	m_coordinates[3] = pntSegFrom1;
	m_obliqueVector = obliqueVector;
	m_obliqueVector.Normalize();
	m_height = height;
	m_obliqueVectorLength = height / m_obliqueVector.z;
	m_coordinates[4] = m_coordinates[0] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[5] = m_coordinates[1] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[6] = m_coordinates[2] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[7] = m_coordinates[3] + m_obliqueVector * m_obliqueVectorLength;
	ComputeCenters();
	Double dis = m_center1.Distance(m_coordinates[0]);

	if (dis < 0.2)
	{
		Coordinate vec = m_coordinates[0] - m_center1;
		vec.Normalize();
		m_coordinates[0] = m_center1 + vec * 0.2;
		vec = m_coordinates[1] - m_center1;
		vec.Normalize();
		m_coordinates[1] = m_center1 + vec * 0.2;
		vec = m_coordinates[2] - m_center1;
		vec.Normalize();
		m_coordinates[2] = m_center1 + vec * 0.2;
		vec = m_coordinates[3] - m_center1;
		vec.Normalize();
		m_coordinates[3] = m_center1 + vec * 0.2;
		m_coordinates[4] = m_coordinates[0] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[5] = m_coordinates[1] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[6] = m_coordinates[2] + m_obliqueVector * m_obliqueVectorLength;
		m_coordinates[7] = m_coordinates[3] + m_obliqueVector * m_obliqueVectorLength;
	}

	ComputeNormalVector();

	return true;
}

RectangularBlock::~RectangularBlock()
{
}

RectangularBlock::RectangularBlock(const RectangularBlock &rhs)
{
}

RectangularBlock &RectangularBlock::operator=(const RectangularBlock &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
	}

	return *this;
}

GeometryType RectangularBlock::GetGeometryType() const
{
	return GeometryType::INCLINEDRECTANGULARBLOCK;
}

Envelope *const RectangularBlock::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> RectangularBlock::GetControlPoints()
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

Geometry *const RectangularBlock::Clone() const
{
	RectangularBlock *rectangularBlock = new RectangularBlock(*this);

	return rectangularBlock;
}

Array<Coordinate> RectangularBlock::GetBottomsurfaceCoords()
{
	Array<Coordinate> coords;
	coords.Add(m_coordinates[0]);
	coords.Add(m_coordinates[1]);
	coords.Add(m_coordinates[2]);
	coords.Add(m_coordinates[3]);

	return coords;
}

Coordinate RectangularBlock::GetObliqueVector()
{
	m_obliqueVector.Normalize();

	return m_obliqueVector;
}

Double RectangularBlock::GetHeight()
{
	return m_height;
}

Array<Coordinate> RectangularBlock::GetTopsurfaceCoords()
{
	Array<Coordinate> coords;
	coords.Add(m_coordinates[4]);
	coords.Add(m_coordinates[5]);
	coords.Add(m_coordinates[6]);
	coords.Add(m_coordinates[7]);

	return coords;
}

Coordinate RectangularBlock::GetBottomsurfaceCenter()
{
	ComputeCenters();

	return m_center1;
}

Coordinate RectangularBlock::GetTopsurfaceCenter()
{
	ComputeCenters();

	return m_center2;
}

Coordinate RectangularBlock::GetNormalVector()
{
	return m_normalVector;
}

Void RectangularBlock::Offset(Double dx, Double dy, Double dz)
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
	m_coordinates[3].x += dx;
	m_coordinates[3].y += dy;
	m_coordinates[3].z += dz;
	m_center1.x += dx;
	m_center1.y += dy;
	m_center1.z += dz;
	m_coordinates[4].x += dx;
	m_coordinates[4].y += dy;
	m_coordinates[4].z += dz;
	m_coordinates[5].x += dx;
	m_coordinates[5].y += dy;
	m_coordinates[5].z += dz;
	m_coordinates[6].x += dx;
	m_coordinates[6].y += dy;
	m_coordinates[6].z += dz;
	m_coordinates[7].x += dx;
	m_coordinates[7].y += dy;
	m_coordinates[7].z += dz;
	m_center2.x += dx;
	m_center2.y += dy;
	m_center2.z += dz;
}

Bool RectangularBlock::Resize(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	/*四个边相邻两边，两两做点积操作，即计算夹角。如果夹角有一个不是90度，则斜矩形块的底面不是符合规定的矩形*/

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

	if (abs(dot1) > Geometries_EP || abs(dot2) > Geometries_EP || abs(dot3) > Geometries_EP || abs(dot4) > Geometries_EP)
	{
		return false;
	}

	/**/

	ComputeCenters();
	Engine::Geometries::Coordinate pntProjectToPlane;
	BaseAlgorithm3D::GetProjectpntToPlane(pt, m_center1, m_normalVector, pntProjectToPlane);
	Coordinate vector_0t = pntProjectToPlane - m_coordinates[0];
	Coordinate vector_1t = pntProjectToPlane - m_coordinates[1];
	Coordinate vector_2t = pntProjectToPlane - m_coordinates[2];
	Coordinate vector_3t = pntProjectToPlane - m_coordinates[3];
	Double dot = -1.0;
	Coordinate vec_center_t = pntProjectToPlane - m_center1;
	Coordinate vec_02 = m_coordinates[2] - m_coordinates[0];
	vec_02.Normalize();
	Coordinate vec_13 = m_coordinates[3] - m_coordinates[1];
	vec_13.Normalize();
	Coordinate pntProject;
	Double len = 0.0;
	Coordinate vec_center_0 = m_coordinates[0] - m_center1;
	vec_center_0.Normalize();
	Coordinate vec_center_1 = m_coordinates[1] - m_center1;
	vec_center_1.Normalize();
	Coordinate vec_center_2 = m_coordinates[2] - m_center1;
	vec_center_2.Normalize();
	Coordinate vec_center_3 = m_coordinates[3] - m_center1;
	vec_center_3.Normalize();

	if (index == 4)
	{
		Coordinate vector_03 = m_coordinates[3] - m_coordinates[0];
		vector_03.Normalize();
		dot = vector_03.DotProduct(vector_0t);
		m_coordinates[0] = m_coordinates[0] + vector_03 * dot;
		Coordinate vector_12 = m_coordinates[2] - m_coordinates[1];
		vector_12.Normalize();
		dot = vector_12.DotProduct(vector_1t);
		m_coordinates[1] = m_coordinates[1] + vector_12 * dot;
	}
	else if (index == 1)
	{
		BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, m_coordinates[1], m_coordinates[3], pntProject);
		len = m_center1.Distance(pntProject);
		m_coordinates[0] = m_center1 + vec_center_0 * len;
		m_coordinates[1] = m_center1 + vec_center_1 * len;
		m_coordinates[2] = m_center1 + vec_center_2 * len;
		m_coordinates[3] = m_center1 + vec_center_3 * len;
	}
	else if (index == 5)
	{
		Coordinate vector_10 = m_coordinates[0] - m_coordinates[1];
		vector_10.Normalize();
		dot = vector_10.DotProduct(vector_1t);
		m_coordinates[1] = m_coordinates[1] + vector_10 * dot;
		Coordinate vector_23 = m_coordinates[3] - m_coordinates[2];
		vector_23.Normalize();
		dot = vector_23.DotProduct(vector_2t);
		m_coordinates[2] = m_coordinates[2] + vector_23 * dot;
	}
	else if (index == 2)
	{
		BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, m_coordinates[0], m_coordinates[2], pntProject);
		len = m_center1.Distance(pntProject);
		m_coordinates[0] = m_center1 + vec_center_0 * len;
		m_coordinates[1] = m_center1 + vec_center_1 * len;
		m_coordinates[2] = m_center1 + vec_center_2 * len;
		m_coordinates[3] = m_center1 + vec_center_3 * len;
	}
	else if (index == 6)
	{
		Coordinate vector_21 = m_coordinates[1] - m_coordinates[2];
		vector_21.Normalize();
		dot = vector_21.DotProduct(vector_2t);
		m_coordinates[2] = m_coordinates[2] + vector_21 * dot;
		Coordinate vector_30 = m_coordinates[0] - m_coordinates[3];
		vector_30.Normalize();
		dot = vector_30.DotProduct(vector_3t);
		m_coordinates[3] = m_coordinates[3] + vector_30 * dot;
	}
	else if (index == 3)
	{
		BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, m_coordinates[1], m_coordinates[3], pntProject);
		len = m_center1.Distance(pntProject);
		m_coordinates[0] = m_center1 + vec_center_0 * len;
		m_coordinates[1] = m_center1 + vec_center_1 * len;
		m_coordinates[2] = m_center1 + vec_center_2 * len;
		m_coordinates[3] = m_center1 + vec_center_3 * len;
	}
	else if (index == 7)
	{
		Coordinate vector_32 = m_coordinates[2] - m_coordinates[3];
		vector_32.Normalize();
		dot = vector_32.DotProduct(vector_3t);
		m_coordinates[3] = m_coordinates[3] + vector_32 * dot;
		Coordinate vector_01 = m_coordinates[1] - m_coordinates[0];
		vector_01.Normalize();
		dot = vector_01.DotProduct(vector_0t);
		m_coordinates[0] = m_coordinates[0] + vector_01 * dot;
	}
	else if (index == 0)
	{
		BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, m_coordinates[0], m_coordinates[2], pntProject);
		len = m_center1.Distance(pntProject);
		m_coordinates[0] = m_center1 + vec_center_0 * len;
		m_coordinates[1] = m_center1 + vec_center_1 * len;
		m_coordinates[2] = m_center1 + vec_center_2 * len;
		m_coordinates[3] = m_center1 + vec_center_3 * len;
	}

	Double dis = 0.0;

	if (index == 0 || index == 1 || index == 2 || index == 3)
	{
		dis = m_center1.Distance(m_coordinates[0]);

		if (dis < 0.2)
		{
			Coordinate vec = m_coordinates[0] - m_center1;
			vec.Normalize();
			m_coordinates[0] = m_center1 + vec * 0.2;
			vec = m_coordinates[1] - m_center1;
			vec.Normalize();
			m_coordinates[1] = m_center1 + vec * 0.2;
			vec = m_coordinates[2] - m_center1;
			vec.Normalize();
			m_coordinates[2] = m_center1 + vec * 0.2;
			vec = m_coordinates[3] - m_center1;
			vec.Normalize();
			m_coordinates[3] = m_center1 + vec * 0.2;
		}
	}
	else if (index == 4)
	{
		dis = m_coordinates[0].Distance(m_coordinates[2]);

		if (dis < 0.4)
		{
			Double dis_01 = m_coordinates[0].Distance(m_coordinates[1]);
			Double len = 2 * sqrt(pow(0.2, 2) - pow(dis_01 / 2.0, 2));
			m_coordinates[0] = m_coordinates[3] + vector_30 * len;
			m_coordinates[1] = m_coordinates[2] + vector_12 * -1.0 * len;
		}
	}
	else if (index == 5)
	{
		dis = m_coordinates[0].Distance(m_coordinates[2]);

		if (dis < 0.4)
		{
			Double dis_12 = m_coordinates[1].Distance(m_coordinates[2]);
			Double len = 2 * sqrt(pow(0.2, 2) - pow(dis_12 / 2.0, 2));
			m_coordinates[1] = m_coordinates[0] + vector_01 * len;
			m_coordinates[2] = m_coordinates[3] + vector_23 * -1.0 * len;
		}
	}
	else if (index == 6)
	{
		dis = m_coordinates[0].Distance(m_coordinates[2]);

		if (dis < 0.4)
		{
			Double dis_23 = m_coordinates[2].Distance(m_coordinates[3]);
			Double len = 2 * sqrt(pow(0.2, 2) - pow(dis_23 / 2.0, 2));
			m_coordinates[2] = m_coordinates[1] + vector_12 * len;
			m_coordinates[3] = m_coordinates[0] + vector_30 * -1.0 * len;
		}
	}
	else if (index == 7)
	{
		dis = m_coordinates[0].Distance(m_coordinates[2]);

		if (dis < 0.4)
		{
			Double dis_30 = m_coordinates[3].Distance(m_coordinates[0]);
			Double len = 2 * sqrt(pow(0.2, 2) - pow(dis_30 / 2.0, 2));
			m_coordinates[3] = m_coordinates[2] + vector_23 * len;
			m_coordinates[0] = m_coordinates[1] + vector_01 * -1.0 * len;
		}
	}

	m_coordinates[4] = m_coordinates[0] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[5] = m_coordinates[1] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[6] = m_coordinates[2] + m_obliqueVector * m_obliqueVectorLength;
	m_coordinates[7] = m_coordinates[3] + m_obliqueVector * m_obliqueVectorLength;

	return true;
}

Void RectangularBlock::Rotate(Double angle)
{
	Coordinate vec_Normal(0.0, 0.0, 1.0);
	BaseAlgorithm3D::RotatePoint(m_center1, angle, vec_Normal, m_coordinates[0]);
	BaseAlgorithm3D::RotatePoint(m_center1, angle, vec_Normal, m_coordinates[1]);
	BaseAlgorithm3D::RotatePoint(m_center1, angle, vec_Normal, m_coordinates[2]);
	BaseAlgorithm3D::RotatePoint(m_center1, angle, vec_Normal, m_coordinates[3]);
	BaseAlgorithm3D::RotatePoint(m_center2, angle, vec_Normal, m_coordinates[4]);
	BaseAlgorithm3D::RotatePoint(m_center2, angle, vec_Normal, m_coordinates[5]);
	BaseAlgorithm3D::RotatePoint(m_center2, angle, vec_Normal, m_coordinates[6]);
	BaseAlgorithm3D::RotatePoint(m_center2, angle, vec_Normal, m_coordinates[7]);
}

Void RectangularBlock::SetTopsurfaceCenter(const Coordinate &pnt)
{
	ComputeCenters();
	Coordinate vec_Offset = pnt - m_center2;
	m_center2 = pnt;
	m_obliqueVector = m_center2 - m_center1;
	m_obliqueVectorLength = m_obliqueVector.GetLength();
	m_obliqueVector.Normalize();
	Double disHorizontal = sqrt(pow(m_center1.x - m_center2.x, 2) + pow(m_center1.y - m_center2.y, 2));
	m_height = sqrt(pow(m_obliqueVectorLength, 2) - pow(disHorizontal, 2));
	m_coordinates[4] = m_coordinates[4] + vec_Offset;
	m_coordinates[5] = m_coordinates[5] + vec_Offset;
	m_coordinates[6] = m_coordinates[6] + vec_Offset;
	m_coordinates[7] = m_coordinates[7] + vec_Offset;
}

Void RectangularBlock::SetBottomsurfaceCenter(const Coordinate &pnt)
{
	ComputeCenters();
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
	m_coordinates[3] = m_coordinates[3] + vec_Offset;
}
