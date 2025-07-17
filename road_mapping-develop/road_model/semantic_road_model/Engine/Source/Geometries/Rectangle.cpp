/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Rectangle.cpp
简要描述:矩形
******************************************************************/

#include "Geometries/Rectangle.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Base/Types.h"
#include "Base/Macros.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

const Double Rectangle_EP = 1.0e-5;

Void Rectangle::ComputeCenter()
{
	m_center.x = (m_coordinates[0].x + m_coordinates[2].x) / 2.0;
	m_center.y = (m_coordinates[0].y + m_coordinates[2].y) / 2.0;
	m_center.z = (m_coordinates[0].z + m_coordinates[2].z) / 2.0;
}

Void Rectangle::ComputeNormalVector()
{
	Engine::Geometries::Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	Engine::Geometries::Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
	m_normalVector = vec_01.CrossProduct(vec_12);
	m_normalVector.Normalize();
}

Rectangle::Rectangle(const Engine::Geometries::Coordinate &pntSegFrom,
					 const Engine::Geometries::Coordinate &pntSegTo,
					 const Engine::Geometries::Coordinate &pnt)
{
	Engine::Geometries::Coordinate pntProject;
	Bool b = BaseAlgorithm3D::GetProjectpntToLine(pnt, pntSegFrom, pntSegTo, pntProject);
	Engine::Geometries::Coordinate vec_pntProject_pnt = pnt - pntProject;
	Engine::Geometries::Coordinate pntSegFrom1 = pntSegFrom + vec_pntProject_pnt;
	Engine::Geometries::Coordinate pntSegTo1 = pntSegTo + vec_pntProject_pnt;
	m_coordinates.Clear();
	m_coordinates.Add(pntSegFrom);
	m_coordinates.Add(pntSegTo);
	m_coordinates.Add(pntSegTo1);
	m_coordinates.Add(pntSegFrom1);
	ComputeCenter();
	ComputeNormalVector();
}

Rectangle::Rectangle()
{
}

Bool Rectangle::Make(const Engine::Geometries::Coordinate &pntSegFrom,
					 const Engine::Geometries::Coordinate &pntSegTo,
					 Double width, Double lendth)
{
	Double dHeight0 = pntSegFrom.Distance(pntSegTo);

	// 2点重合
	if (dHeight0 < Geometries_EP)
	{
		return false;
	}

	if ((width < Geometries_EP) || (lendth < Geometries_EP))
	{
		return false;
	}

	// 放大比例
	Double dRadio = lendth / dHeight0;

	// 一半宽度
	Double dDis = width / 2;

	// 根据高度重新计算新的边线中点
	Engine::Geometries::Coordinate pntCenter = (pntSegFrom + pntSegTo) / 2;
	Engine::Geometries::Coordinate pntSegFromNew = (pntSegFrom - pntCenter) * dRadio;
	Engine::Geometries::Coordinate pntSegToNew = (pntSegTo - pntCenter) * dRadio;

	Engine::Geometries::Coordinate e0 = pntSegFromNew - pntSegToNew;

	if ((fabs(e0.x) < Geometries_EP) && (fabs(e0.y) < Geometries_EP)) // 竖直状态 结果不唯一，返回false;
	{
		return false;
	}

	Double x1, x2, y1, y2;
	Double dTemp;
	Double R0 = pntSegToNew.DotProduct(e0);
	Double R1 = R0 - e0.z * pntSegToNew.z;

	if (fabs(e0.x) < Geometries_EP)
	{
		y1 = y2 = (R1 / e0.y);
		dTemp = dDis * dDis - (y1 - pntSegToNew.y) * (y1 - pntSegToNew.y);
		x1 = pntSegToNew.x + sqrt(dTemp);
		x2 = pntSegToNew.x - sqrt(dTemp);
	}
	else if (fabs(e0.y) < Geometries_EP)
	{
		x1 = x2 = (R1 / e0.x);
		dTemp = dDis * dDis - (x1 - pntSegToNew.x) * (x1 - pntSegToNew.x);
		y1 = pntSegToNew.y + sqrt(dTemp);
		y2 = pntSegToNew.y - sqrt(dTemp);
	}
	else
	{
		Double dA = ((e0.y * e0.y) / (e0.x * e0.x)) + 1;
		Double dB = (-2) * ((e0.y / e0.x) * ((R1 / e0.x) - pntSegToNew.x) + pntSegToNew.y);
		Double dC = ((R1 / e0.x) - pntSegToNew.x) * ((R1 / e0.x) - pntSegToNew.x) + pntSegToNew.y * pntSegToNew.y - dDis * dDis;
		dB /= dA;
		dC /= dA;
		dA = 1.0;
		dTemp = sqrt(dB * dB - 4 * dA * dC);
		y1 = (-dB + dTemp) / (2 * dA);
		y2 = (-dB - dTemp) / (2 * dA);
		x1 = (R1 - e0.y * y1) / e0.x;
		x2 = (R1 - e0.y * y2) / e0.x;
	}

	Engine::Geometries::Coordinate pntSegFrom0, pntSegTo0, pntSegFrom1, pntSegTo1; // 矩形4个角点

	pntSegTo0.x = x1;
	pntSegTo0.y = y1;
	pntSegTo0.z = pntSegToNew.z;
	pntSegTo1 = pntSegToNew * 2 - pntSegTo0;
	pntSegFrom0 = pntSegFromNew + pntSegTo0 - pntSegToNew;
	pntSegFrom1 = pntSegFromNew + pntSegTo1 - pntSegToNew;

	Coordinate vec_From0_To0 = pntSegTo0 - pntSegFrom0;
	vec_From0_To0.Normalize();
	Coordinate vec_To0_To1 = pntSegTo1 - pntSegTo0;
	vec_To0_To1.Normalize();
	Coordinate normalVector = vec_From0_To0.CrossProduct(vec_To0_To1);
	normalVector.Normalize();
	Coordinate zAxis(0.0, 0.0, 1.0);
	Double dotProduct = normalVector.DotProduct(zAxis);
	m_coordinates.Clear();

	pntSegFrom0 = (pntSegFrom0 + pntCenter);
	pntSegTo0 = (pntSegTo0 + pntCenter);
	pntSegFrom1 = (pntSegFrom1 + pntCenter);
	pntSegTo1 = (pntSegTo1 + pntCenter);

	if (dotProduct >= 0)
	{
		m_coordinates.Add(pntSegFrom1);
		m_coordinates.Add(pntSegFrom0);
		m_coordinates.Add(pntSegTo0);
		m_coordinates.Add(pntSegTo1);
	}
	else
	{
		m_coordinates.Add(pntSegFrom0);
		m_coordinates.Add(pntSegFrom1);
		m_coordinates.Add(pntSegTo1);
		m_coordinates.Add(pntSegTo0);
	}

	ComputeCenter();
	ComputeNormalVector();

	return true;
}

Rectangle::~Rectangle()
{
}

Rectangle::Rectangle(const Engine::Base::Array<Coordinate> &controlPoints)
{
	if (controlPoints.GetCount() >= 8)
	{
		*this = Rectangle(controlPoints[0], controlPoints[1], controlPoints[6]);
	}
}

// Rectangle::Rectangle(const Rectangle &rhs)
// {
// }
//
// Rectangle& Rectangle::operator =(const Rectangle &rhs)
// {
// 	//检查自赋值
// 	if (this != &rhs)
// 	{
// 	}
//
// 	return *this;
// }

GeometryType Rectangle::GetGeometryType() const
{
	return GeometryType::RECTANGLE;
}

Envelope *const Rectangle::GetEnvelope() const
{
	Envelope *evn = NULL;

	return evn;
}

Array<Coordinate> Rectangle::GetControlPoints()
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

Geometry *const Rectangle::Clone() const
{
	Rectangle *rectangle = new Rectangle(*this);

	return rectangle;
}

Array<Coordinate> Rectangle::GetCoordinates()
{
	return m_coordinates;
}

Coordinate Rectangle::GetCenter()
{
	ComputeCenter();

	return m_center;
}

Coordinate Rectangle::GetNormalVector()
{
	return m_normalVector;
}

Void Rectangle::Offset(Double dx, Double dy, Double dz)
{
	ComputeCenter();

	for (Int32 i = 0; i < m_coordinates.GetCount(); i++)
	{
		m_coordinates[i].x += dx;
		m_coordinates[i].y += dy;
		m_coordinates[i].z += dz;
	}

	m_center.x += dx;
	m_center.y += dy;
	m_center.z += dz;
}

/*Bool Rectangle::Resize(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	/*四个边相邻两边，两两做点积操作，即计算夹角。如果夹角有一个不是90度，则不是符合规定的矩形*/

/*Coordinate vector_01 = m_coordinates[1] - m_coordinates[0];
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

if (abs(dot1) > Rectangle_EP || abs(dot2) > Rectangle_EP || abs(dot3) > Rectangle_EP || abs(dot4) > Rectangle_EP)
{
	return false;
}

/**/

/*ComputeCenter();
Engine::Geometries::Coordinate pntProjectToPlane;
BaseAlgorithm3D::GetProjectpntToPlane(pt, m_center, m_normalVector, pntProjectToPlane);
Coordinate vector_0t = pntProjectToPlane - m_coordinates[0];
Coordinate vector_1t = pntProjectToPlane - m_coordinates[1];
Coordinate vector_2t = pntProjectToPlane - m_coordinates[2];
Coordinate vector_3t = pntProjectToPlane - m_coordinates[3];
Double dot = -1.0;
Coordinate coordinate4 = (m_coordinates[0] + m_coordinates[1]) / 2.0;
Coordinate coordinate5 = (m_coordinates[1] + m_coordinates[2]) / 2.0;
Coordinate coordinate6 = (m_coordinates[2] + m_coordinates[3]) / 2.0;
Coordinate coordinate7 = (m_coordinates[3] + m_coordinates[0]) / 2.0;
Coordinate vector_46 = coordinate6 - coordinate4;
vector_46.Normalize();
Coordinate vector_57 = coordinate7 - coordinate5;
vector_57.Normalize();
Coordinate pntProject;
BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, coordinate4, coordinate6, pntProject);
Double dis57 = 2 * pntProjectToPlane.Distance(pntProject);
BaseAlgorithm3D::GetProjectpntToLine(pntProjectToPlane, coordinate5, coordinate7, pntProject);
Double dis46 = 2 * pntProjectToPlane.Distance(pntProject);

if (index == 4)
{
	//计算向量0t在直线03上的投影线段的长度
	Coordinate vector_03 = m_coordinates[3] - m_coordinates[0];
	vector_03.Normalize();
	dot = vector_03.DotProduct(vector_0t);

	//拉取01的中点，移动后，变换后0点的坐标=变换前0点的坐标+单位化后的向量03*向量0t在向量03上的投影
	m_coordinates[0] = m_coordinates[0] + vector_03 * dot;

	//计算向量1t在向量12上的投影
	Coordinate vector_12 = m_coordinates[2] - m_coordinates[1];
	vector_12.Normalize();
	dot = vector_12.DotProduct(vector_1t);

	//拉取01的中点，移动后，变换后1点的坐标=变换前1点的坐标+单位化后的向量12*向量1t在向量12上的投影
	m_coordinates[1] = m_coordinates[1] + vector_12 * dot;
}
else if (index == 1)
{
	m_coordinates[1] = pntProjectToPlane;
	m_coordinates[0] = pntProjectToPlane + vector_57 * dis57;
	m_coordinates[2] = pntProjectToPlane + vector_46 * dis46;
	m_coordinates[3] = m_coordinates[2] + vector_57 * dis57;
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
	m_coordinates[2] = pntProjectToPlane;
	m_coordinates[1] = pntProjectToPlane + vector_46 * -1.0 * dis46;
	m_coordinates[3] = pntProjectToPlane + vector_57 * dis57;
	m_coordinates[0] = m_coordinates[3] + vector_46 * -1.0 * dis46;
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
	m_coordinates[3] = pntProjectToPlane;
	m_coordinates[0] = pntProjectToPlane + vector_46 * -1.0 * dis46;
	m_coordinates[2] = pntProjectToPlane + vector_57 * -1.0 * dis57;
	m_coordinates[1] = m_coordinates[0] + vector_57 * -1.0 * dis57;
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
	m_coordinates[0] = pntProjectToPlane;
	m_coordinates[1] = pntProjectToPlane + vector_57 * -1.0 * dis57;
	m_coordinates[3] = pntProjectToPlane + vector_46 * dis46;
	m_coordinates[2] = m_coordinates[1] + vector_46 * dis46;
}

return true;
}*/

/*Bool Rectangle::Resize(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	/*四个边相邻两边，两两做点积操作，即计算夹角。如果夹角有一个不是90度，则不是符合规定的矩形*/

/*Coordinate vector_01 = m_coordinates[1] - m_coordinates[0];
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

if (abs(dot1) > Rectangle_EP || abs(dot2) > Rectangle_EP || abs(dot3) > Rectangle_EP || abs(dot4) > Rectangle_EP)
{
	return false;
}

/**/

/*GetCenter();
Coordinate vector_0t = pt - m_coordinates[0];
Coordinate vector_1t = pt - m_coordinates[1];
Coordinate vector_2t = pt - m_coordinates[2];
Coordinate vector_3t = pt - m_coordinates[3];
Double dot = -1.0;
Engine::Geometries::Coordinate pntProjectToPlane;
BaseAlgorithm3D::GetProjectpntToPlane(pt, m_center, m_normalVector, pntProjectToPlane);
Coordinate vec_center_t = pntProjectToPlane - m_center;

if (index == 4)
{
	//计算向量0t在直线03上的投影线段的长度
	Coordinate vector_03 = m_coordinates[3] - m_coordinates[0];
	vector_03.Normalize();
	dot = vector_03.DotProduct(vector_0t);

	//拉取01的中点，移动后，变换后0点的坐标=变换前0点的坐标+单位化后的向量03*向量0t在向量03上的投影
	m_coordinates[0] = m_coordinates[0] + vector_03 * dot;

	//计算向量1t在向量12上的投影
	Coordinate vector_12 = m_coordinates[2] - m_coordinates[1];
	vector_12.Normalize();
	dot = vector_12.DotProduct(vector_1t);

	//拉取01的中点，移动后，变换后1点的坐标=变换前1点的坐标+单位化后的向量12*向量1t在向量12上的投影
	m_coordinates[1] = m_coordinates[1] + vector_12 * dot;
}
else if (index == 1)
{
	Coordinate vec_30 = m_coordinates[0] - m_coordinates[3];
	vec_30.Normalize();
	Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	vec_01.Normalize();
	Double dot = vec_center_t.DotProduct(vec_30);
	m_coordinates[0] = m_center + vec_30 * dot;
	dot = (vec_center_t * -1.0).DotProduct(vec_01);
	m_coordinates[0] = m_coordinates[0] + vec_01 * dot;
	Coordinate vec_center0 = m_coordinates[0] - m_center;
	m_coordinates[2] = m_center + vec_center0 * -1.0;
	m_coordinates[1] = pntProjectToPlane;
	m_coordinates[3] = m_center + vec_center_t * -1.0;
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
	Coordinate vec_30 = m_coordinates[0] - m_coordinates[3];
	vec_30.Normalize();
	Coordinate vec_23 = m_coordinates[3] - m_coordinates[2];
	vec_23.Normalize();
	Double dot = vec_center_t.DotProduct(vec_30);
	m_coordinates[3] = m_center + vec_30 * dot;
	dot = (vec_center_t * -1.0).DotProduct(vec_23);
	m_coordinates[3] = m_coordinates[3] + vec_23 * dot;
	Coordinate vec_center3 = m_coordinates[3] - m_center;
	m_coordinates[1] = m_center + vec_center3 * -1.0;
	m_coordinates[2] = pntProjectToPlane;
	m_coordinates[0] = m_center + vec_center_t * -1.0;
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
	Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
	vec_12.Normalize();
	Coordinate vec_23 = m_coordinates[3] - m_coordinates[2];
	vec_23.Normalize();
	Double dot = vec_center_t.DotProduct(vec_12);
	m_coordinates[2] = m_center + vec_12 * dot;
	dot = (vec_center_t * -1.0).DotProduct(vec_23);
	m_coordinates[2] = m_coordinates[2] + vec_23 * dot;
	Coordinate vec_center2 = m_coordinates[2] - m_center;
	m_coordinates[0] = m_center + vec_center2 * -1.0;
	m_coordinates[3] = pntProjectToPlane;
	m_coordinates[1] = m_center + vec_center_t * -1.0;
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
	Coordinate vec_30 = m_coordinates[0] - m_coordinates[3];
	vec_30.Normalize();
	Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	vec_01.Normalize();
	Double dot = vec_center_t.DotProduct(vec_30);
	m_coordinates[1] = m_center + vec_30 * dot;
	dot = (vec_center_t * -1.0).DotProduct(vec_01);
	m_coordinates[1] = m_coordinates[1] + vec_01 * dot;
	Coordinate vec_center1 = m_coordinates[1] - m_center;
	m_coordinates[3] = m_center + vec_center1 * -1.0;
	m_coordinates[0] = pntProjectToPlane;
	m_coordinates[2] = m_center + vec_center_t * -1.0;
}

return true;
}*/

Bool Rectangle::Resize(UInt16 index, const Engine::Geometries::Coordinate &pt)
{
	GetCenter();
	Engine::Geometries::Coordinate pntProjectToPlane;
	Coordinate pntTemp = pt;
	pntTemp.z -= 1.0;

	Coordinate eTemp = pt - pntTemp;
	if (eTemp.DotProduct(m_normalVector) < Geometries_EP)
	{
		BaseAlgorithm3D::GetProjectpntToPlane(pt, m_center, m_normalVector, pntProjectToPlane);
	}
	else
	{
		BaseAlgorithm3D::IntersectionLinePlane(pntTemp, pt, m_center, m_normalVector, pntProjectToPlane);
	}

	Coordinate vector_0t = pntProjectToPlane - m_coordinates[0];
	Coordinate vector_1t = pntProjectToPlane - m_coordinates[1];
	Coordinate vector_2t = pntProjectToPlane - m_coordinates[2];
	Coordinate vector_3t = pntProjectToPlane - m_coordinates[3];
	Double dot = -1.0;
	Coordinate vec_center_t = pntProjectToPlane - m_center;
	Coordinate vec_02 = m_coordinates[2] - m_coordinates[0];
	vec_02.Normalize();
	Coordinate vec_13 = m_coordinates[3] - m_coordinates[1];
	vec_13.Normalize();
	Coordinate pntProject;
	Double len = 0.0;
	Coordinate vec_center_0 = m_coordinates[0] - m_center;
	vec_center_0.Normalize();
	Coordinate vec_center_1 = m_coordinates[1] - m_center;
	vec_center_1.Normalize();
	Coordinate vec_center_2 = m_coordinates[2] - m_center;
	vec_center_2.Normalize();
	Coordinate vec_center_3 = m_coordinates[3] - m_center;
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
		len = m_center.Distance(pntProject);
		m_coordinates[0] = m_center + vec_center_0 * len;
		m_coordinates[1] = m_center + vec_center_1 * len;
		m_coordinates[2] = m_center + vec_center_2 * len;
		m_coordinates[3] = m_center + vec_center_3 * len;
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
		len = m_center.Distance(pntProject);
		m_coordinates[0] = m_center + vec_center_0 * len;
		m_coordinates[1] = m_center + vec_center_1 * len;
		m_coordinates[2] = m_center + vec_center_2 * len;
		m_coordinates[3] = m_center + vec_center_3 * len;
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
		len = m_center.Distance(pntProject);
		m_coordinates[0] = m_center + vec_center_0 * len;
		m_coordinates[1] = m_center + vec_center_1 * len;
		m_coordinates[2] = m_center + vec_center_2 * len;
		m_coordinates[3] = m_center + vec_center_3 * len;
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
		len = m_center.Distance(pntProject);
		m_coordinates[0] = m_center + vec_center_0 * len;
		m_coordinates[1] = m_center + vec_center_1 * len;
		m_coordinates[2] = m_center + vec_center_2 * len;
		m_coordinates[3] = m_center + vec_center_3 * len;
	}

	return true;
}

Void Rectangle::Rotate(Double angle)
{
	ComputeCenter();
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[0]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[1]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[2]);
	BaseAlgorithm3D::RotatePoint(m_center, angle, m_normalVector, m_coordinates[3]);
}

Void Rectangle::Rotate(UInt16 index, Double angle)
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

Double Rectangle::GetLength()
{
	Double length = m_coordinates[0].Distance(m_coordinates[1]);

	return length;
}

Void Rectangle::SetLength(Double length)
{
	Coordinate pntMid_01 = (m_coordinates[0] + m_coordinates[1]) / 2.0;
	Coordinate vec_01 = m_coordinates[1] - m_coordinates[0];
	vec_01.Normalize();
	m_coordinates[0] = pntMid_01 + vec_01 * -1.0 * length / 2.0;
	m_coordinates[1] = pntMid_01 + vec_01 * length / 2.0;
	Coordinate pntMid_23 = (m_coordinates[2] + m_coordinates[3]) / 2.0;
	Coordinate vec_23 = m_coordinates[3] - m_coordinates[2];
	vec_23.Normalize();
	m_coordinates[2] = pntMid_23 + vec_23 * -1.0 * length / 2.0;
	m_coordinates[3] = pntMid_23 + vec_23 * length / 2.0;
}

Double Rectangle::GetWidth()
{
	Double width = m_coordinates[1].Distance(m_coordinates[2]);

	return width;
}

Void Rectangle::SetWidth(Double width)
{
	Coordinate pntMid_12 = (m_coordinates[1] + m_coordinates[2]) / 2.0;
	Coordinate vec_12 = m_coordinates[2] - m_coordinates[1];
	vec_12.Normalize();
	m_coordinates[1] = pntMid_12 + vec_12 * -1.0 * width / 2.0;
	m_coordinates[2] = pntMid_12 + vec_12 * width / 2.0;
	Coordinate pntMid_30 = (m_coordinates[3] + m_coordinates[0]) / 2.0;
	Coordinate vec_30 = m_coordinates[0] - m_coordinates[3];
	vec_30.Normalize();
	m_coordinates[3] = pntMid_30 + vec_30 * -1.0 * width / 2.0;
	m_coordinates[0] = pntMid_30 + vec_30 * width / 2.0;
}

Int32 Engine::Geometries::Rectangle::GetOppositeControlPointIndex(Base::Int32 index)
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

Void Engine::Geometries::Rectangle::RotateByCoord(Base::UInt16 index, const Engine::Geometries::Coordinate &pt)
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

Engine::Geometries::Coordinate Engine::Geometries::Rectangle::GetControlPointFaceVector(Base::Int32 index)
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

Void Engine::Geometries::Rectangle::GetControlPointFaceCoords(Base::UInt16 index, Engine::Geometries::Coordinate &pt1, Engine::Geometries::Coordinate &pt2, Engine::Geometries::Coordinate &pt3)
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
