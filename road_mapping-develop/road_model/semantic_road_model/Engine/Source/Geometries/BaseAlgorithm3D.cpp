/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:BaseAlgorithm3D.cpp
简要描述:
******************************************************************/

#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/Coordinate.h"
#define _USE_MATH_DEFINES
#include <math.h>
#include "Geometries/Envelope3D.h"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include <complex>
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"
#include "Geometries/IRCreatLaneAlgorithm.h"
#include "Base/Set.h"
#include <float.h>

using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

Double BaseAlgorithm3D::DisPtToLine(
	const Engine::Geometries::Coordinate &pntS,
	const Engine::Geometries::Coordinate &pntE,
	const Engine::Geometries::Coordinate &pnt,
	Double tolerance)
{
	Double dDistance = pntS.Distance(pntE);

	// 直线的起点和终点重合
	if (dDistance < tolerance)
	{
		return pntS.Distance(pnt);
	}

	Coordinate e1 = pntE - pntS;
	Coordinate e2 = pnt - pntS;
	Coordinate n = e1.CrossProduct(e2);

	return ((n.GetLength()) / dDistance);
}

Double BaseAlgorithm3D::DisPtToLineSegment(
	const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	const Engine::Geometries::Coordinate &pnt,
	Double tolerance)
{
	Double disSegFromTo = pntSegFrom.Distance(pntSegTo);

	if (disSegFromTo < tolerance)
	{
		return pntSegFrom.Distance(pnt);
	}

	Double distance = -1.0;
	Coordinate e1 = pnt - pntSegFrom;
	Coordinate e2 = pntSegTo - pntSegFrom;
	e2.Normalize();
	Double dot = e1.DotProduct(e2);

	if ((dot < 0.0) || (dot > disSegFromTo))
	{
		Double dDisS = pnt.Distance(pntSegFrom);
		Double dDisE = pnt.Distance(pntSegTo);

		if (dDisS < dDisE)
		{
			distance = dDisS;
		}
		else
		{
			distance = dDisE;
		}
	}
	else
	{
		distance = DisPtToLine(pntSegFrom, pntSegTo, pnt);
	}

	return distance;
}

Bool BaseAlgorithm3D::GetNearestPntToRayOrigin(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate *pPnts,
	UInt32 nPointCount,
	Engine::Geometries::Coordinate &pntNearestToS,
	Double tolerance)
{
	if (pPnts == NULL || nPointCount == 0 ||
		(pntRayS.x == pntRayE.x &&
		 pntRayS.y == pntRayE.y &&
		 pntRayS.z == pntRayE.z))
	{
		return false;
	}

	Double dis = 0.0;
	Double disNearToS = 0.0;
	UInt32 indexMinNearToS = 0;
	Double disMinNearToS = sqrt(pow(pPnts[0].x - pntRayS.x, 2) + pow(pPnts[0].y - pntRayS.y, 2) + pow(pPnts[0].z - pntRayS.z, 2));

	for (UInt32 i = 0; i < nPointCount; i++)
	{
		dis = DisPtToLine(pntRayS, pntRayE, pPnts[i]);

		if (dis < tolerance)
		{
			disNearToS = sqrt(pow(pPnts[i].x - pntRayS.x, 2) +
							  pow(pPnts[i].y - pntRayS.y, 2) + pow(pPnts[i].z - pntRayS.z, 2));

			if (disNearToS < disMinNearToS)
			{
				indexMinNearToS = i;
				disMinNearToS = disNearToS;
			}
		}
	}

	pntNearestToS.x = pPnts[indexMinNearToS].x;
	pntNearestToS.y = pPnts[indexMinNearToS].y;
	pntNearestToS.z = pPnts[indexMinNearToS].z;

	return true;
}

Bool BaseAlgorithm3D::IsIntersectRayLineSegment(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	Double tolerance)
{
	if (Math::Equal(pntRayS.Distance(pntSegFrom), tolerance) || Math::Equal(pntRayS.Distance(pntSegTo), tolerance) || Math::Equal(pntRayE.Distance(pntSegFrom), tolerance) || Math::Equal(pntRayE.Distance(pntSegTo), tolerance))
		return true;

	Coordinate e1 = pntSegFrom - pntRayS;
	Coordinate e2 = pntRayE - pntRayS;
	Coordinate n = e1.CrossProduct(e2);
	Coordinate e3 = pntSegTo - pntRayS;
	n.Normalize();
	e3.Normalize();
	Double dot = n.DotProduct(e3);

	if (dot < tolerance)
	{
		Coordinate n1 = e2.CrossProduct(e1);
		Coordinate n2 = e2.CrossProduct(e3);
		n1.Normalize();
		n2.Normalize();
		dot = n1.DotProduct(n2);

		return (dot <= 0.0);
	}

	return false;
}

Bool BaseAlgorithm3D::IsIntersectRayLineSegments(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate *pPntSegments,
	UInt32 nPointcount,
	Double tolerance)
{
	if ((nPointcount < 2) || (pPntSegments == NULL))
	{
		return false;
	}

	for (UInt32 i = 0; i < (nPointcount - 1); i++)
	{
		if (IsIntersectRayLineSegment(pntRayS, pntRayE, pPntSegments[i], pPntSegments[i + 1], tolerance))
		{
			return true;
		}
	}

	return false;
}

Bool BaseAlgorithm3D::IntersectionRayLineSegment(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	Engine::Geometries::Coordinate &pntResult,
	Double tolerance)
{
	if (!IsIntersectRayLineSegment(pntRayS, pntRayE, pntSegFrom, pntSegTo, tolerance))
	{
		return false;
	}

	Coordinate e0 = pntSegFrom - pntRayS;
	Coordinate e1 = pntSegTo - pntSegFrom;
	Coordinate e2 = pntRayE - pntRayS;
	Double dT = -1.0;
	Double dDelta = e1.z * e2.x - e1.x * e2.z;

	if (dDelta != 0.0)
	{
		dT = (e0.x * e2.z - e0.z * e2.x) / dDelta;
		pntResult = pntSegFrom + (e1 * dT);

		return true;
	}

	dDelta = e1.z * e2.y - e1.y * e2.z;

	if (dDelta != 0.0)
	{
		dT = (e0.y * e2.z - e0.z * e2.y) / dDelta;
		pntResult = pntSegFrom + (e1 * dT);

		return true;
	}

	dDelta = e1.y * e2.x - e1.x * e2.y;

	if (dDelta != 0.0)
	{
		dT = (e0.x * e2.y - e0.y * e2.x) / dDelta;
		pntResult = pntSegFrom + (e1 * dT);

		return true;
	}

	return false;
}

Bool BaseAlgorithm3D::IntersectionRayLineSegments(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate *pPntSegments,
	UInt32 nPointcount,
	Engine::Geometries::Coordinate &pntResult,
	Double tolerance)
{
	if ((nPointcount < 2) || (pPntSegments == NULL))
	{
		return false;
	}

	for (UInt32 i = 0; i < (nPointcount - 1); i++)
	{
		if (IntersectionRayLineSegment(pntRayS, pntRayE, pPntSegments[i], pPntSegments[i + 1], pntResult, tolerance))
		{
			return true;
		}
	}

	return false;
}

Bool BaseAlgorithm3D::GetProjectpntToTriangle(
	const Engine::Geometries::Coordinate &pnt0Triangle,
	const Engine::Geometries::Coordinate &pnt1Triangle,
	const Engine::Geometries::Coordinate &pnt2Triangle,
	const Engine::Geometries::Coordinate &pnt,
	Engine::Geometries::Coordinate &pntProject)
{
	Coordinate e1 = pnt1Triangle - pnt0Triangle;
	Coordinate e2 = pnt2Triangle - pnt0Triangle;
	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();
	Coordinate e3 = pnt - pnt0Triangle;
	Double dot = n.DotProduct(e3);
	pntProject.x = pnt.x - dot * n.x;
	pntProject.y = pnt.y - dot * n.y;
	pntProject.z = pnt.z - dot * n.z;

	return true;
}

Double BaseAlgorithm3D::DisPtToTriangle(
	const Engine::Geometries::Coordinate &pnt0Triangle,
	const Engine::Geometries::Coordinate &pnt1Triangle,
	const Engine::Geometries::Coordinate &pnt2Triangle,
	const Engine::Geometries::Coordinate &pnt)
{
	Engine::Geometries::Coordinate pntProject;
	GetProjectpntToTriangle(pnt0Triangle, pnt1Triangle, pnt2Triangle, pnt, pntProject);

	return pnt.Distance(pntProject);
}

Bool BaseAlgorithm3D::IsIntersectRayTriangle(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate &pnt0Triangle,
	const Engine::Geometries::Coordinate &pnt1Triangle,
	const Engine::Geometries::Coordinate &pnt2Triangle)
{
	Coordinate vector_RayS_0Triangle = pnt0Triangle - pntRayS;
	Coordinate vector_RayS_1Triangle = pnt1Triangle - pntRayS;
	Coordinate vector_RayS_2Triangle = pnt2Triangle - pntRayS;
	Coordinate vector_RayS_RayE = pntRayE - pntRayS;
	Coordinate cross = vector_RayS_0Triangle.CrossProduct(vector_RayS_1Triangle);
	cross.Normalize();
	Double dot01 = cross.DotProduct(vector_RayS_RayE);
	cross = vector_RayS_1Triangle.CrossProduct(vector_RayS_2Triangle);
	cross.Normalize();
	Double dot12 = cross.DotProduct(vector_RayS_RayE);
	cross = vector_RayS_2Triangle.CrossProduct(vector_RayS_0Triangle);
	cross.Normalize();
	Double dot20 = cross.DotProduct(vector_RayS_RayE);

	if (dot01 > 0 || dot12 > 0 || dot20 > 0)
	{
		return false;
	}

	return true;
}

Bool BaseAlgorithm3D::IsVerticalRayTriangle(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate &pnt0Triangle,
	const Engine::Geometries::Coordinate &pnt1Triangle,
	const Engine::Geometries::Coordinate &pnt2Triangle,
	Double tolerance)
{
	Coordinate vector_0Triangle_1Triangle = pnt1Triangle - pnt0Triangle;
	Coordinate vector_0Triangle_2Triangle = pnt2Triangle - pnt0Triangle;
	Coordinate cross = vector_0Triangle_1Triangle.CrossProduct(vector_0Triangle_2Triangle);
	Coordinate vector_RayS_RayE = pntRayE - pntRayS;
	cross.Normalize();
	vector_RayS_RayE.Normalize();
	cross = cross.CrossProduct(vector_RayS_RayE);

	if (cross.GetLength() < tolerance)
	{
		return true;
	}

	return false;
}

Bool BaseAlgorithm3D::IntersectionRayTriangle(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate &pnt0Triangle,
	const Engine::Geometries::Coordinate &pnt1Triangle,
	const Engine::Geometries::Coordinate &pnt2Triangle,
	Engine::Geometries::Coordinate &pntResult)
{
	if (IsIntersectRayTriangle(pntRayS, pntRayE, pnt0Triangle, pnt1Triangle, pnt2Triangle) == false)
	{
		return false;
	}

	Coordinate vector_RayS_0Triangle = pnt0Triangle - pntRayS;
	Coordinate vector_0Triangle_1Triangle = pnt1Triangle - pnt0Triangle;
	Coordinate vector_0Triangle_2Triangle = pnt2Triangle - pnt0Triangle;
	Coordinate cross = vector_0Triangle_1Triangle.CrossProduct(vector_0Triangle_2Triangle);
	Coordinate vector_RayS_RayE = pntRayE - pntRayS;
	vector_RayS_RayE.Normalize();
	Double t = vector_RayS_0Triangle.DotProduct(cross) / vector_RayS_RayE.DotProduct(cross);
	pntResult = pntRayS + vector_RayS_RayE * t;

	return true;
}

Bool BaseAlgorithm3D::IsIntersectLineSegmentPlane(const Engine::Geometries::Coordinate &pntSegFrom,
												  const Engine::Geometries::Coordinate &pntSegTo,
												  const Engine::Geometries::Coordinate &pntOrg,
												  const Engine::Geometries::Coordinate &normalVector)
{
	Coordinate e1 = pntSegFrom - pntOrg;
	Coordinate e2 = pntSegTo - pntOrg;
	e1.Normalize();
	e2.Normalize();
	Double dot1 = e1.DotProduct(normalVector);
	Double dot2 = e2.DotProduct(normalVector);

	return !((dot1 * dot2) > 0.0);
}

Bool BaseAlgorithm3D::IntersectionLineSegmentPlane(const Engine::Geometries::Coordinate &pntSegFrom,
												   const Engine::Geometries::Coordinate &pntSegTo,
												   const Engine::Geometries::Coordinate &pntOrg,
												   const Engine::Geometries::Coordinate &normalVector,
												   Engine::Geometries::Coordinate &pntResult)
{
	Coordinate e1 = pntSegTo - pntSegFrom;
	Coordinate e2 = pntSegFrom - pntOrg;
	Double dot1 = e1.DotProduct(normalVector);
	if (fabs(dot1) < Geometries_EP) // 平行
	{
		return false;
	}

	Double dot2 = e2.DotProduct(normalVector);
	Double dT = (-dot2) / dot1;

	if ((dT >= Geometries_NEP) && (dT <= 1.0 + Geometries_EP))
	{
		pntResult = pntSegFrom + e1 * dT;

		return true;
	}

	return false;
}

Bool BaseAlgorithm3D::IntersectionLinePlane(const Engine::Geometries::Coordinate &pntFrom,
											const Engine::Geometries::Coordinate &pntTo,
											const Engine::Geometries::Coordinate &pntOrg,
											const Engine::Geometries::Coordinate &normalVector,
											Engine::Geometries::Coordinate &pntResult)
{
	Coordinate e1 = pntTo - pntFrom;
	Coordinate e2 = pntFrom - pntOrg;

	Coordinate normalVectorTemp = normalVector;
	normalVectorTemp.Normalize();

	Double dot1 = e1.DotProduct(normalVectorTemp);

	if (fabs(dot1) < Geometries_EP) // 平行
	{
		return false;
	}

	Double dot2 = e2.DotProduct(normalVectorTemp);
	Double dT = (-dot2) / dot1;

	pntResult = pntFrom + (pntTo - pntFrom) * dT;

	return true;
}
Bool BaseAlgorithm3D::CalcPlaneNormalVector(
	const Engine::Geometries::Coordinate &pnt1,
	const Engine::Geometries::Coordinate &pnt2,
	const Engine::Geometries::Coordinate &pnt3,
	Engine::Geometries::Coordinate &normalVector)
{
	// 平面内向量u
	Coordinate u1 = pnt1 - pnt2;
	Coordinate u2 = pnt1 - pnt3;

	// 平面法向量
	normalVector = u1.CrossProduct(u2);

	if (normalVector.IsZeroVector())
	{
		return false;
	}
	normalVector.Normalize();
	return true;
}

Bool BaseAlgorithm3D::CalcPlaneIntersectionLine(
	const Engine::Geometries::Coordinate &pnt1,
	const Engine::Geometries::Coordinate &normalVector1,
	const Engine::Geometries::Coordinate &pnt2,
	const Engine::Geometries::Coordinate &normalVector2,
	Engine::Geometries::Coordinate &pntFrom,
	Engine::Geometries::Coordinate &pntTo)
{
	// 计算一般式下的常量参数
	Base::Double d1 = pnt1.DotProduct(normalVector1);
	Base::Double d2 = pnt2.DotProduct(normalVector2);
	// 计算交线方向向量
	Engine::Geometries::Coordinate IntersectionVector = normalVector1.CrossProduct(normalVector2);
	if (IntersectionVector.IsZeroVector())
	{
		return false;
	}
	// 计算normalVector1、IntersectionVector的垂直方向
	Engine::Geometries::Coordinate crossVector = IntersectionVector.CrossProduct(normalVector1);

	// 确定点pnt3
	Engine::Geometries::Coordinate pnt1_1 = pnt1 + crossVector;

	// 计算交点
	if (!IntersectionLinePlane(pnt1, pnt1_1, pnt2, normalVector2, pntFrom))
	{
		return false;
	}
	//
	pntTo = pntFrom + IntersectionVector;
	return true;
}

Bool BaseAlgorithm3D::IsParallelCollinearLines(
	const Engine::Geometries::Coordinate &pntLine1From,
	const Engine::Geometries::Coordinate &pntLine1To,
	const Engine::Geometries::Coordinate &pntLine2From,
	const Engine::Geometries::Coordinate &pntLine2To,
	Double tolerance)
{
	Coordinate e1 = pntLine1To - pntLine1From;
	Coordinate e2 = pntLine2To - pntLine2From;
	e1.Normalize();
	e2.Normalize();
	Coordinate cross = e1.CrossProduct(e2);
	Double len = cross.GetLength();

	return (len < tolerance);
}

Bool BaseAlgorithm3D::GetProjectpntToPlane(const Engine::Geometries::Coordinate &pnt,
										   const Engine::Geometries::Coordinate &pntOrg,
										   Engine::Geometries::Coordinate &normalVector,
										   Engine::Geometries::Coordinate &pntProject)
{
	normalVector.Normalize();

	Coordinate e3 = pnt - pntOrg;
	Double dot = normalVector.DotProduct(e3);

	pntProject.x = pnt.x - dot * normalVector.x;
	pntProject.y = pnt.y - dot * normalVector.y;
	pntProject.z = pnt.z - dot * normalVector.z;

	return true;
}

Bool BaseAlgorithm3D::Complanation(Engine::Geometries::Coordinate *pPoints, Int32 nPointcount, Double tolerance)
{
	if ((pPoints == NULL) || (nPointcount < 3) || (tolerance < 0.0))
	{
		return false;
	}

	Double dMinZ, dMaxZ;
	Int32 nMinZ, nMaxz;
	Int32 i = 0;

	dMinZ = dMaxZ = pPoints[0].z;
	nMinZ = nMaxz = 0;

	for (i = 1; i < nPointcount; i++)
	{
		if (pPoints[i].z < dMinZ)
		{
			dMinZ = pPoints[i].z;
			nMinZ = i;
		}
		else if (pPoints[i].z > dMaxZ)
		{
			dMaxZ = pPoints[i].z;
			nMaxz = i;
		}
	}

	if ((dMaxZ - dMinZ) < tolerance)
	{
		Double dZ = (dMaxZ + dMinZ) / 2;

		for (i = 0; i < nPointcount; i++)
		{
			pPoints[i].z = dZ;
		}

		return true;
	}

	Coordinate V0 = pPoints[nMinZ];
	Coordinate V1 = pPoints[nMaxz];
	Double dLength = 0.0, dTempLength = 0.0;
	Int32 nMaxIndex = -1;

	for (i = 0; i < nPointcount; i++)
	{
		if ((i != nMinZ) && (i != nMaxz))
		{
			dTempLength = V0.Distance(pPoints[i]);
			dTempLength += V1.Distance(pPoints[i]);

			if (dLength < dTempLength)
			{
				dLength = dTempLength;
				nMaxIndex = i;
			}
		}
	}

	Coordinate V2 = pPoints[nMaxIndex];
	Coordinate e1 = V1 - V0;
	Coordinate e2 = V2 - V0;

	e1.Normalize();
	e2.Normalize();

	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();

	Engine::Geometries::Coordinate pntProject;

	for (i = 0; i < nPointcount; i++)
	{
		if (i == nMinZ)
		{
			continue;
		}

		GetProjectpntToPlane(pPoints[i], V0, n, pntProject);

		// if (sqrt(pow(pPoints[i].x - pntProject.x, 2) + pow(pPoints[i].y - pntProject.y, 2) + pow(pPoints[i].z - pntProject.z, 2)) > tolerance)
		if (pPoints[i].Distance(pntProject) > tolerance)
		{
			return false;
		}
	}

	for (i = 0; i < nPointcount; i++)
	{
		if (i == nMinZ)
		{
			continue;
		}

		GetProjectpntToPlane(pPoints[i], V0, n, pntProject);
		pPoints[i] = pntProject;
	}

	return true;
}

Bool BaseAlgorithm3D::Complanation(Array<Engine::Geometries::Coordinate *> *pPoints, Double tolerance)
{
	if ((pPoints == NULL) || pPoints->GetCount() < 3 || (tolerance < 0.0))
	{
		return false;
	}

	Double dMinZ, dMaxZ;
	Int32 nMinZ, nMaxz;
	Int32 i = 0;
	Int32 nPointcount = pPoints->GetCount();

	dMinZ = dMaxZ = pPoints->GetAt(0)->z;
	nMinZ = nMaxz = 0;

	for (i = 1; i < nPointcount; i++)
	{
		if (pPoints->GetAt(i)->z < dMinZ)
		{
			dMinZ = pPoints->GetAt(i)->z;
			nMinZ = i;
		}
		else if (pPoints->GetAt(i)->z > dMaxZ)
		{
			dMaxZ = pPoints->GetAt(i)->z;
			nMaxz = i;
		}
	}

	if ((dMaxZ - dMinZ) < tolerance)
	{
		Double dZ = (dMaxZ + dMinZ) / 2;

		for (i = 0; i < nPointcount; i++)
		{
			pPoints->GetAt(i)->z = dZ;
		}

		return true;
	}

	Coordinate V0 = *(pPoints->GetAt(nMinZ));
	Coordinate V1 = *(pPoints->GetAt(nMaxz));
	Double dLength = 0.0, dTempLength = 0.0;
	Int32 nMaxIndex = -1;

	for (i = 0; i < nPointcount; i++)
	{
		if ((i != nMinZ) && (i != nMaxz))
		{
			dTempLength = V0.Distance(*(pPoints->GetAt(i)));
			dTempLength += V1.Distance(*(pPoints->GetAt(i)));

			if (dLength < dTempLength)
			{
				dLength = dTempLength;
				nMaxIndex = i;
			}
		}
	}

	Coordinate V2 = *(pPoints->GetAt(nMaxIndex));
	Coordinate e1 = V1 - V0;
	Coordinate e2 = V2 - V0;

	e1.Normalize();
	e2.Normalize();

	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();

	Engine::Geometries::Coordinate pntProject;

	for (i = 0; i < nPointcount; i++)
	{
		if (i == nMinZ)
		{
			continue;
		}

		GetProjectpntToPlane(*(pPoints->GetAt(i)), V0, n, pntProject);

		// if (sqrt(pow(pPoints[i].x - pntProject.x, 2) + pow(pPoints[i].y - pntProject.y, 2) + pow(pPoints[i].z - pntProject.z, 2)) > tolerance)
		if (pPoints->GetAt(i)->Distance(pntProject) > tolerance)
		{
			return false;
		}
	}

	for (i = 0; i < nPointcount; i++)
	{
		if (i == nMinZ)
		{
			continue;
		}

		GetProjectpntToPlane(*(pPoints->GetAt(i)), V0, n, pntProject);
		*(pPoints->GetAt(i)) = pntProject;
	}

	return true;
}

Bool BaseAlgorithm3D::ComplanationFit(Engine::Geometries::Coordinate *pPoints, Int32 nPointcount, Double tolerance)
{
	if ((pPoints == NULL) || (nPointcount < 4) || (tolerance < 0.0))
	{
		return false;
	}
	/*
	 *	解XTX的特征值
	 */
	Eigen::MatrixXd X = Eigen::MatrixXd::Zero(nPointcount, 4);
	for (Int32 i = 0; i < nPointcount; i++)
	{
		Coordinate c = pPoints[i];
		X(i, 0) = c.x;
		X(i, 1) = c.y;
		X(i, 2) = c.z;
		X(i, 3) = 1.0;
	}
	Eigen::MatrixXd XTX = X.transpose() * X;

	Eigen::EigenSolver<Eigen::MatrixXd> es(XTX);

	std::complex<Double> d[4];
	d[0] = es.eigenvalues()[0];
	d[1] = es.eigenvalues()[1];
	d[2] = es.eigenvalues()[2];
	d[3] = es.eigenvalues()[3];

	int iMin = 0;
	double dMax = DBL_MAX;
	for (int i = 0; i < 4; i++)
	{
		if (d[i].real() < dMax)
		{
			iMin = i;
			dMax = d[i].real();
		}
	}
	Eigen::VectorXcd V = es.eigenvectors().col(iMin);
	// 平面方程
	double dA = V(0).real();
	double dB = V(1).real();
	double dC = V(2).real();
	double dD = V(3).real();

	// 投射到该平面
	for (int i = 0; i < nPointcount; i++)
	{
		double x = pPoints[i].x;
		double y = pPoints[i].y;
		double z = pPoints[i].z;
		double t = (dA * x + dB * y + dC * z + dD) / (dA * dA + dB * dB + dC * dC);

		pPoints[i].x = x - dA * t;
		pPoints[i].y = y - dB * t;
		pPoints[i].z = z - dC * t;
	}
	return true;
}

Bool BaseAlgorithm3D::ImposeComplanation(Engine::Geometries::Coordinate *pPoints, Int32 nPointcount)
{
	if ((pPoints == NULL) || (nPointcount < 0))
	{
		return false;
	}

	Double dMinZ, dMaxZ;
	Int32 nMinZ, nMaxz;
	Int32 i = 0;

	for (i = 0; i < nPointcount; i++)
	{
		if (i == 0)
		{
			dMinZ = dMaxZ = pPoints[0].z;
			nMinZ = nMaxz = 0;
		}
		else
		{
			if (pPoints[i].z < dMinZ)
			{
				dMinZ = pPoints[i].z;
				nMinZ = i;
			}
			else if (pPoints[i].z > dMaxZ)
			{
				dMaxZ = pPoints[i].z;
				nMaxz = i;
			}
		}
	}

	Coordinate V0 = pPoints[nMinZ];
	Coordinate V1 = pPoints[nMaxz];
	Double dLength = 0.0, dTempLength = 0.0;
	Int32 nMaxIndex = -1;

	for (i = 0; i < nPointcount; i++)
	{
		if ((i != nMinZ) && (i != nMaxz))
		{
			dTempLength = V0.Distance(pPoints[i]);
			dTempLength += V1.Distance(pPoints[i]);

			if (dLength < dTempLength)
			{
				dLength = dTempLength;
				nMaxIndex = i;
			}
		}
	}

	Coordinate V2 = pPoints[nMaxIndex];
	Coordinate e1 = V1 - V0;
	Coordinate e2 = V2 - V0;
	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();

	Coordinate pntTemp;
	for (i = 0; i < nPointcount; i++)
	{
		GetProjectpntToPlane(pPoints[i], V0, n, pntTemp);
		pPoints[i] = pntTemp;
	}

	return true;
}

Bool BaseAlgorithm3D::ImposeComplanation(Array<Engine::Geometries::Coordinate *> *pPoints)
{
	if ((pPoints == NULL) || (pPoints->GetCount() < 3))
	{
		return false;
	}

	Double dMinZ, dMaxZ;
	Int32 nMinZ, nMaxz;
	Int32 i = 0;
	Int32 nPointcount = pPoints->GetCount();

	dMinZ = dMaxZ = pPoints->GetAt(0)->z;
	nMinZ = nMaxz = 0;

	for (i = 1; i < nPointcount; i++)
	{
		if (pPoints->GetAt(i)->z < dMinZ)
		{
			dMinZ = pPoints->GetAt(i)->z;
			nMinZ = i;
		}
		else if (pPoints->GetAt(i)->z > dMaxZ)
		{
			dMaxZ = pPoints->GetAt(i)->z;
			nMaxz = i;
		}
	}

	Coordinate V0 = *(pPoints->GetAt(nMinZ));
	Coordinate V1 = *(pPoints->GetAt(nMaxz));
	Double dLength = 0.0, dTempLength = 0.0;
	Int32 nMaxIndex = -1;

	for (i = 0; i < nPointcount; i++)
	{
		if ((i != nMinZ) && (i != nMaxz))
		{
			dTempLength = V0.Distance(*(pPoints->GetAt(i)));
			dTempLength += V1.Distance(*(pPoints->GetAt(i)));

			if (dLength < dTempLength)
			{
				dLength = dTempLength;
				nMaxIndex = i;
			}
		}
	}

	Coordinate V2 = *(pPoints->GetAt(nMaxIndex));
	Coordinate e1 = V1 - V0;
	Coordinate e2 = V2 - V0;

	e1.Normalize();
	e2.Normalize();

	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();

	Engine::Geometries::Coordinate pntProject;
	for (i = 0; i < nPointcount; i++)
	{
		GetProjectpntToPlane(*(pPoints->GetAt(i)), V0, n, pntProject);
		*(pPoints->GetAt(i)) = pntProject;
	}

	return true;
}

Void BaseAlgorithm3D::RotatePointXY(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate &pntResult)
{
	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);
	Double xx = pntResult.x - pntOrg.x;
	Double yy = pntResult.y - pntOrg.y;
	pntResult.x = xx * dCosAngle - yy * dSinAngle + pntOrg.x;
	pntResult.y = xx * dSinAngle + yy * dCosAngle + pntOrg.y;
}

Void BaseAlgorithm3D::RotatePointXZ(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate &pntResult)
{
	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);
	Double xx = pntResult.x - pntOrg.x;
	Double zz = pntResult.z - pntOrg.z;
	pntResult.x = xx * dCosAngle + zz * dSinAngle + pntOrg.x;
	pntResult.z = -1 * xx * dSinAngle + zz * dCosAngle + pntOrg.z;
}

Void BaseAlgorithm3D::RotatePointYZ(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate &pntResult)
{
	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);
	Double yy = pntResult.y - pntOrg.y;
	Double zz = pntResult.z - pntOrg.z;
	pntResult.y = yy * dCosAngle - zz * dSinAngle + pntOrg.y;
	pntResult.z = yy * dSinAngle + zz * dCosAngle + pntOrg.z;
}

Void BaseAlgorithm3D::RotatePointXY(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate *pntsResult, Int32 nPointCount)
{
	if ((pntsResult == NULL) || (nPointCount < 0))
	{
		return;
	}

	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);

	for (Int32 i = 0; i < nPointCount; i++)
	{
		Double xx = pntsResult[i].x - pntOrg.x;
		Double yy = pntsResult[i].y - pntOrg.y;
		pntsResult[i].x = xx * dCosAngle - yy * dSinAngle + pntOrg.x;
		pntsResult[i].y = xx * dSinAngle + yy * dCosAngle + pntOrg.y;
	}
}

Void BaseAlgorithm3D::RotatePointXZ(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate *pntsResult, Int32 nPointCount)
{
	if ((pntsResult == NULL) || (nPointCount < 0))
	{
		return;
	}

	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);

	for (Int32 i = 0; i < nPointCount; i++)
	{
		Double xx = pntsResult[i].x - pntOrg.x;
		Double zz = pntsResult[i].z - pntOrg.z;
		pntsResult[i].x = xx * dCosAngle + zz * dSinAngle + pntOrg.x;
		pntsResult[i].z = -1 * xx * dSinAngle + zz * dCosAngle + pntOrg.z;
	}
}

Void BaseAlgorithm3D::RotatePointYZ(const Engine::Geometries::Coordinate &pntOrg, Double dAngle,
									Engine::Geometries::Coordinate *pntsResult, Int32 nPointCount)
{
	if ((pntsResult == NULL) || (nPointCount < 0))
	{
		return;
	}

	Double dCosAngle = cos(dAngle);
	Double dSinAngle = sin(dAngle);

	for (Int32 i = 0; i < nPointCount; i++)
	{
		Double yy = pntsResult[i].y - pntOrg.y;
		Double zz = pntsResult[i].z - pntOrg.z;
		pntsResult[i].y = yy * dCosAngle - zz * dSinAngle + pntOrg.y;
		pntsResult[i].z = yy * dSinAngle + zz * dCosAngle + pntOrg.z;
	}
}

// Void BaseAlgorithm3D::RotatePoint(const Engine::Geometries::Coordinate &pntOrg,
//	const Engine::Geometries::Coordinate &pntCenter, Double dAngle, Engine::Geometries::Coordinate &pntNormalVector,
//	Engine::Geometries::Coordinate &pntResult)
//{
//	Coordinate pntCenterProject;
//	Coordinate pntProject;
//	GetProjectpntToPlane(pntCenter, pntOrg, pntNormalVector, pntCenterProject);
//	Coordinate vec_CenterOrg = pntOrg - pntCenterProject;
//	dAngle = dAngle / 180.0 * M_PI;
//	pntNormalVector.Normalize();
//	Coordinate cross_CenterOrg_CenterResult = pntNormalVector * (vec_CenterOrg.GetLength() * vec_CenterOrg.GetLength() * sin(dAngle));
//	Double dot_CenterOrg_CenterResult = vec_CenterOrg.GetLength() * vec_CenterOrg.GetLength() * cos(dAngle);
//	Double R = cross_CenterOrg_CenterResult.x;
//	Double S = cross_CenterOrg_CenterResult.y;
//	Double T = cross_CenterOrg_CenterResult.z;
//	R += vec_CenterOrg.y * pntCenterProject.z - vec_CenterOrg.z * pntCenterProject.y;
//	S += vec_CenterOrg.z * pntCenterProject.x - vec_CenterOrg.x * pntCenterProject.z;
//	T += vec_CenterOrg.x * pntCenterProject.y - vec_CenterOrg.y * pntCenterProject.x;
//	Double Y = dot_CenterOrg_CenterResult;
//	Y += vec_CenterOrg.x * pntCenterProject.x + vec_CenterOrg.y * pntCenterProject.y + vec_CenterOrg.z * pntCenterProject.z;
//	pntResult.z = (Y * vec_CenterOrg.z - vec_CenterOrg.x * S + vec_CenterOrg.y * R) /
//		(pow(vec_CenterOrg.x, 2) + pow(vec_CenterOrg.y, 2) + pow(vec_CenterOrg.z, 2));
//	pntResult.x = (S + vec_CenterOrg.x * pntResult.z) / vec_CenterOrg.z;
//	pntResult.y = (vec_CenterOrg.y * pntResult.z - R) / vec_CenterOrg.z;
// }

Void BaseAlgorithm3D::RotatePoint(const Engine::Geometries::Coordinate &pntOrg,
								  Double dAngle, Engine::Geometries::Coordinate &oriVector,
								  Engine::Geometries::Coordinate &pntResult)
{
	Coordinate midPoint;
	midPoint = pntResult - pntOrg;
	Coordinate newPoint;
	oriVector.Normalize();
	Double c = cos(dAngle);

	if (dAngle == 90.0)
	{
		c = 0;
	}

	Double s = sin(dAngle);

	if (dAngle == 0)
	{
		s = 0;
	}

	Double x = oriVector.x;
	Double y = oriVector.y;
	Double z = oriVector.z;
	newPoint.x = (x * x * (1 - c) + c) * midPoint.x + (x * y * (1 - c) - z * s) * midPoint.y + (x * z * (1 - c) + y * s) * midPoint.z;
	newPoint.y = (y * x * (1 - c) + z * s) * midPoint.x + (y * y * (1 - c) + c) * midPoint.y + (y * z * (1 - c) - x * s) * midPoint.z;
	newPoint.z = (x * z * (1 - c) - y * s) * midPoint.x + (z * y * (1 - c) + x * s) * midPoint.y + (z * z * (1 - c) + c) * midPoint.z;
	pntResult = newPoint + pntOrg;
}

//
// Void BaseAlgorithm3D::RotatePoint2(const Engine::Geometries::Coordinate &pntOrg,
//	const Engine::Geometries::Coordinate &pntCenter, Double dAngle, Engine::Geometries::Coordinate &oriVector,
//	Engine::Geometries::Coordinate &pntResult)
//{
//	//Double angx, angy;
//	Coordinate midPoint;
//	midPoint = pntOrg - pntCenter;
//	Coordinate newPoint;
//	oriVector.Normalize();
//	Coordinate ori;
//	ori.x = 0; ori.y = 0; ori.z = 0;
//	double x = oriVector.x;
//	double y = oriVector.y;
//	double z = oriVector.z;
//	//dAngle = dAngle / 180.0 * M_PI;
//	Double angx, angy;
//	angx = asin(y /sqrt(x*x+y*y));
//	angy =asin(x / sqrt(x*x + y*y+z*z));
//
//
//
//	//RotatePointYZ(ori,  angx, &midPoint, 1);
//	//RotatePointXZ(ori, -1.0 * angy, &midPoint, 1);
//	//RotatePointXY(ori, dAngle, &midPoint, 1);
//	///*RotatePointXY(ori, -1*dAngle, &midPoint, 1);*/
//	//RotatePointXZ(ori, angy, &midPoint, 1);
//	//RotatePointYZ(ori, -1.0*angx, &midPoint, 1);
//	Coordinate P;
//	P.x = 10; P.y = 10; P.z = 10;
//	//RotatePointYZ(ori, M_PI / 180 * 45, &P, 1);
//	RotatePointXZ(ori, 1.0 *  M_PI / 180 * 45,&P, 1);
//	//RotatePointXY(ori, M_PI/180*120, &midPoint, 1);
//
//	//RotatePointXZ(ori, M_PI / 180 * 45, &P, 1);
//	//RotatePointYZ(ori, -1.0* M_PI / 180 * 45, &P, 1);
//
//
//	pntResult = midPoint + pntCenter;
//
//}

Bool BaseAlgorithm3D::GetEnvelope(const Engine::Geometries::Coordinate *pPoints, Int32 nPointCount, Envelope3D &envelope)
{
	if ((pPoints == NULL) || (nPointCount < 0))
	{
		return false;
	}

	Double dMinX, dMaxX, dMinY, dMaxY, dMinZ, dMaxZ;
	Int32 i = 0;

	for (i = 0; i < nPointCount; i++)
	{
		if (i == 0)
		{
			dMinX = dMaxX = pPoints[0].x;
			dMinY = dMaxY = pPoints[0].y;
			dMinZ = dMaxZ = pPoints[0].z;
		}
		else
		{
			if (pPoints[i].x < dMinX)
			{
				dMinX = pPoints[i].x;
			}
			else if (pPoints[i].x > dMaxX)
			{
				dMaxX = pPoints[i].x;
			}

			if (pPoints[i].y < dMinY)
			{
				dMinY = pPoints[i].y;
			}
			else if (pPoints[i].y > dMaxY)
			{
				dMaxY = pPoints[i].y;
			}

			if (pPoints[i].z < dMinZ)
			{
				dMinZ = pPoints[i].z;
			}
			else if (pPoints[i].z > dMaxZ)
			{
				dMaxZ = pPoints[i].z;
			}
		}
	}

	envelope = Envelope3D(dMinX, dMinY, dMaxX, dMaxY, dMaxZ, dMinZ);

	return true;
}

Double BaseAlgorithm3D::GetDistanceLineToLinesegments(
	const Engine::Geometries::Coordinate &pntLineS,
	const Engine::Geometries::Coordinate &pntLineE,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Int32 &nSegIndex,
	Double tolerance)
{
	if (coordinates == NULL)
	{
		return -1.0;
	}

	Int32 nPointCount = coordinates->GetCount();

	if (nPointCount < 2)
	{
		return -1.0;
	}

	Double dDisTemp = pntLineS.Distance(pntLineE);

	if (dDisTemp < tolerance)
	{
		return GetDistancePointToLinesegments(pntLineS, coordinates, pntProject, nSegIndex);
	}

	Double dMinDistance = -1.0, dDis = -1.0;
	Int32 i = 0;

	for (i = 0; i < nPointCount - 1; i++)
	{
		Coordinate e1 = *((*coordinates)[i]) - pntLineS;
		Coordinate e2 = *((*coordinates)[i + 1]) - *((*coordinates)[i]);
		Coordinate e3 = pntLineE - pntLineS;
		Coordinate n = e2.CrossProduct(e3);
		n.Normalize();

		// 直线与折线段平行或重合
		if (n.GetLength() < tolerance)
		{
			dDis = DisPtToLine(pntLineS, pntLineE, *((*coordinates)[i]));
			// dDis = DisPtToLineSegment(pntLineS, pntLineE, *((*coordinates)[i]));

			if ((dMinDistance < 0.0) || (dDis < dMinDistance))
			{
				pntProject = *((*coordinates)[i]);
				nSegIndex = i;
				dMinDistance = dDis;

				if (dMinDistance < tolerance)
				{
					return dMinDistance;
				}
			}
		}
		else
		{
			Coordinate n1 = e1.CrossProduct(n);
			Coordinate n2 = e2.CrossProduct(n);
			Coordinate n3 = e3.CrossProduct(n);
			Double dTemp1 = n3.x * n2.y - n3.y * n2.x;
			Double dTemp2 = n3.x * n2.z - n3.z * n2.x;
			Double dTemp3 = n3.y * n2.z - n3.z * n2.y;

			if ((fabs(dTemp1) < tolerance) && (fabs(dTemp2) < tolerance) && (fabs(dTemp3) < tolerance)) // 这是什么情况！
			{
				continue;
			}

			Double dT1 = -1.0;
			Double dT2 = -1.0;

			if (!(fabs(dTemp1) < tolerance))
			{
				dT1 = (n1.x * n3.y - n1.y * n3.x) / dTemp1;
				dT2 = (n1.x * n2.y - n1.y * n2.x) / dTemp1;
			}
			else if (!(fabs(dTemp2) < tolerance))
			{
				dT1 = (n1.x * n3.z - n1.z * n3.x) / dTemp2;
				dT2 = (n1.x * n2.z - n1.z * n2.x) / dTemp2;
			}
			else
			{
				dT1 = (n1.y * n3.z - n1.z * n3.y) / dTemp3;
				dT2 = (n1.y * n2.z - n1.z * n2.y) / dTemp3;
			}

			if ((dT1 < 0.0) || (dT1 > 1.0)) // 计算首末点距离
			{
				dDis = DisPtToLine(pntLineS, pntLineE, *((*coordinates)[i]));
				// dDis = DisPtToLineSegment(pntLineS, pntLineE, *((*coordinates)[i]));

				if ((dMinDistance < 0.0) || (dDis < dMinDistance))
				{
					pntProject = *((*coordinates)[i]);
					nSegIndex = i;
					dMinDistance = dDis;

					if (dMinDistance < tolerance)
					{
						return dMinDistance;
					}
				}

				dDis = DisPtToLine(pntLineS, pntLineE, *((*coordinates)[i + 1]));
				// dDis = DisPtToLineSegment(pntLineS, pntLineE, *((*coordinates)[i + 1]));

				if (dDis < dMinDistance)
				{
					pntProject = *((*coordinates)[i + 1]);
					nSegIndex = i;
					dMinDistance = dDis;

					if (dMinDistance < tolerance)
					{
						return dMinDistance;
					}
				}
			}
			else
			{
				Coordinate pntProject1 = *((*coordinates)[i]) + e2 * dT1;
				Coordinate pntProject2 = pntLineS + e3 * dT2;
				dDis = pntProject1.Distance(pntProject2);

				if (dMinDistance == -1.0 || dDis < dMinDistance)
				{
					pntProject = pntProject1;
					nSegIndex = i;
					dMinDistance = dDis;

					if (dMinDistance < tolerance)
					{
						return dMinDistance;
					}
				}
			}
		}
	}

	return dMinDistance;
}

Bool BaseAlgorithm3D::IsLinesOnPlane(
	const Engine::Geometries::Coordinate &pntLine0S,
	const Engine::Geometries::Coordinate &pntLine0E,
	const Engine::Geometries::Coordinate &pntLine1S,
	const Engine::Geometries::Coordinate &pntLine1E,
	Double tolerance)
{
	Coordinate e1 = pntLine0S - pntLine1S;
	Coordinate e2 = pntLine1E - pntLine1S;
	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();
	Coordinate e3 = pntLine0E - pntLine1S;
	e3.Normalize();
	Double dot = n.DotProduct(e3);

	return (dot < tolerance);
}

Bool BaseAlgorithm3D::GetProjectpntToLine(const Engine::Geometries::Coordinate &pnt,
										  const Engine::Geometries::Coordinate &pntLineS,
										  const Engine::Geometries::Coordinate &pntLineE,
										  Engine::Geometries::Coordinate &pntProject)
{
	Coordinate e1 = pnt - pntLineS;
	Coordinate e2 = pntLineE - pntLineS;
	e2.Normalize();
	Double dot = e1.DotProduct(e2);
	pntProject = pntLineS + e2 * dot;

	return true;
}

Double BaseAlgorithm3D::GetDistancePointToLinesegments(
	const Engine::Geometries::Coordinate &pnt,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Int32 &nSegIndex,
	Double tolerance)
{
	if (coordinates == NULL)
	{
		return -1.0;
	}

	int nPointCount = coordinates->GetCount();

	if (nPointCount < 2)
	{
		return -1.0;
	}

	Double dMinDistance = -1.0, dDis = -1.0;
	Int32 i = 0;

	for (i = 0; i < nPointCount - 1; i++)
	{
		Double dTemp = (*((*coordinates)[i])).Distance(*((*coordinates)[i + 1]));

		// 折线段的起点和终点重合
		if (dTemp < tolerance)
		{
			dDis = pnt.Distance(*((*coordinates)[i]));

			if ((dMinDistance < 0) || (dDis < dMinDistance))
			{
				dMinDistance = dDis;
				nSegIndex = i;
				pntProject = *((*coordinates)[i]);
			}
		}
		else
		{
			Coordinate e1 = pnt - *((*coordinates)[i]);
			Coordinate e2 = *((*coordinates)[i + 1]) - *((*coordinates)[i]);
			e2.Normalize();
			Double dot = e1.DotProduct(e2);

			// 投影点落在折线段外
			if ((dot < 0.0) || (dot > dTemp))
			{
				Double dDisS = pnt.Distance(*((*coordinates)[i]));
				Double dDisE = pnt.Distance(*((*coordinates)[i + 1]));

				if (dDisS < dDisE)
				{
					if ((dMinDistance < 0.0) || (dDisS < dMinDistance))
					{
						dMinDistance = dDisS;
						nSegIndex = i;
						pntProject = *((*coordinates)[i]);
					}
				}
				else
				{
					if ((dMinDistance < 0.0) || (dDisE < dMinDistance))
					{
						dMinDistance = dDisE;
						nSegIndex = i;
						pntProject = *((*coordinates)[i + 1]);
					}
				}
			}
			else
			{
				dDis = DisPtToLine(*((*coordinates)[i]), *((*coordinates)[i + 1]), pnt);

				if ((dMinDistance < 0) || (dDis < dMinDistance))
				{
					dMinDistance = dDis;
					nSegIndex = i;
					pntProject = *((*coordinates)[i]) + (e2 * dot);
				}
			}
		}
	}

	return dMinDistance;
}

Double BaseAlgorithm3D::GetDistancePointToLinesegments(
	const Engine::Geometries::Coordinate &pnt,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Int32 &nSegIndex,
	bool &bFindInLine,
	Double tolerance)
{
	if (coordinates == NULL)
	{
		return -1.0;
	}

	int nPointCount = coordinates->GetCount();

	if (nPointCount < 2)
	{
		return -1.0;
	}

	Double dMinDistance = -1.0, dDis = -1.0;
	Int32 i = 0;

	for (i = 0; i < nPointCount - 1; i++)
	{
		Double dTemp = (*((*coordinates)[i])).Distance(*((*coordinates)[i + 1]));

		// 折线段的起点和终点重合
		if (dTemp < tolerance)
		{
			dDis = pnt.Distance(*((*coordinates)[i]));

			if ((dMinDistance < 0) || (dDis < dMinDistance))
			{
				dMinDistance = dDis;
				nSegIndex = i;
				pntProject = *((*coordinates)[i]);
				bFindInLine = true;
			}
		}
		else
		{
			Coordinate e1 = pnt - *((*coordinates)[i]);
			Coordinate e2 = *((*coordinates)[i + 1]) - *((*coordinates)[i]);
			e2.Normalize();
			Double dot = e1.DotProduct(e2);

			// 投影点落在折线段外
			if ((dot < 0.0) || (dot > dTemp))
			{
				Double dDisS = pnt.Distance(*((*coordinates)[i]));
				Double dDisE = pnt.Distance(*((*coordinates)[i + 1]));

				if (dDisS < dDisE)
				{
					if ((dMinDistance < 0.0) || (dDisS < dMinDistance))
					{
						dMinDistance = dDisS;
						nSegIndex = i;
						pntProject = *((*coordinates)[i]);
						bFindInLine = false;
					}
				}
				else
				{
					if ((dMinDistance < 0.0) || (dDisE < dMinDistance))
					{
						dMinDistance = dDisE;
						nSegIndex = i;
						pntProject = *((*coordinates)[i + 1]);
						bFindInLine = false;
					}
				}
			}
			else
			{
				dDis = DisPtToLine(*((*coordinates)[i]), *((*coordinates)[i + 1]), pnt);

				if ((dMinDistance < 0) || (dDis < dMinDistance))
				{
					dMinDistance = dDis;
					nSegIndex = i;
					pntProject = *((*coordinates)[i]) + (e2 * dot);
					bFindInLine = true;
				}
			}
		}
	}

	return dMinDistance;
}

Double BaseAlgorithm3D::GetDistancePointToLinesegments(
	const Engine::Geometries::Coordinate &pnt,
	const Array<Engine::Geometries::Coordinate> coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Int32 &nSegIndex,
	Double tolerance)
{
	if (coordinates.IsEmpty() == true)
	{
		return -1.0;
	}

	int nPointCount = coordinates.GetCount();

	if (nPointCount < 2)
	{
		return -1.0;
	}

	Double dMinDistance = -1.0, dDis = -1.0;
	Int32 i = 0;

	for (i = 0; i < nPointCount - 1; i++)
	{
		Double dTemp = coordinates[i].Distance(coordinates[i + 1]);

		// 折线段的起点和终点重合
		if (dTemp < tolerance)
		{
			dDis = pnt.Distance(coordinates[i]);

			if ((dMinDistance < 0) || (dDis < dMinDistance))
			{
				dMinDistance = dDis;
				nSegIndex = i;
				pntProject = coordinates[i];
			}
		}
		else
		{
			Coordinate e1 = pnt - coordinates[i];
			Coordinate e2 = coordinates[i + 1] - coordinates[i];
			e2.Normalize();
			Double dot = e1.DotProduct(e2);

			// 投影点落在折线段外
			if ((dot < 0.0) || (dot > dTemp))
			{
				Double dDisS = pnt.Distance(coordinates[i]);
				Double dDisE = pnt.Distance(coordinates[i + 1]);

				if (dDisS < dDisE)
				{
					if ((dMinDistance < 0.0) || (dDisS < dMinDistance))
					{
						dMinDistance = dDisS;
						nSegIndex = i;
						pntProject = coordinates[i];
					}
				}
				else
				{
					if ((dMinDistance < 0.0) || (dDisE < dMinDistance))
					{
						dMinDistance = dDisE;
						nSegIndex = i;
						pntProject = coordinates[i + 1];
					}
				}
			}
			else
			{
				dDis = DisPtToLine(coordinates[i], coordinates[i + 1], pnt);

				if ((dMinDistance < 0) || (dDis < dMinDistance))
				{
					dMinDistance = dDis;
					nSegIndex = i;
					pntProject = coordinates[i] + (e2 * dot);
				}
			}
		}
	}

	return dMinDistance;
}
Base::Double BaseAlgorithm3D::GetDistanceXYPointToLinesegments(
	const Engine::Geometries::Coordinate &pnt,
	const Base::Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Base::Int32 &nSegIndex,
	bool &bFindInLine,
	Base::Double tolerance)
{
	if (coordinates == NULL)
	{
		return -1.0;
	}

	int nPointCount = coordinates->GetCount();

	if (nPointCount < 2)
	{
		return -1.0;
	}

	Double dMinDistance = -1.0, dDis = -1.0;
	Int32 i = 0;

	for (i = 0; i < nPointCount - 1; i++)
	{
		Double dTemp = (*((*coordinates)[i])).Distance(*((*coordinates)[i + 1]));

		// 折线段的起点和终点重合
		if (dTemp < tolerance)
		{
			dDis = pnt.DistanceXY(*((*coordinates)[i]));

			if ((dMinDistance < 0) || (dDis < dMinDistance))
			{
				dMinDistance = dDis;
				nSegIndex = i;
				pntProject = *((*coordinates)[i]);
				bFindInLine = true;
			}
		}
		else
		{
			Coordinate e1 = pnt - *((*coordinates)[i]);
			Coordinate e2 = *((*coordinates)[i + 1]) - *((*coordinates)[i]);
			e2.Normalize();
			Double dot = e1.DotProduct(e2);

			// 投影点落在折线段外
			if ((dot < 0.0) || (dot > dTemp))
			{
				Double dDisS = pnt.DistanceXY(*((*coordinates)[i]));
				Double dDisE = pnt.DistanceXY(*((*coordinates)[i + 1]));

				if (dDisS < dDisE)
				{
					if ((dMinDistance < 0.0) || (dDisS < dMinDistance))
					{
						dMinDistance = dDisS;
						nSegIndex = i;
						pntProject = *((*coordinates)[i]);
						bFindInLine = false;
					}
				}
				else
				{
					if ((dMinDistance < 0.0) || (dDisE < dMinDistance))
					{
						dMinDistance = dDisE;
						nSegIndex = i;
						pntProject = *((*coordinates)[i + 1]);
						bFindInLine = false;
					}
				}
			}
			else
			{
				dDis = BaseAlgorithm::DistancePtToLine2D(&pnt, (*coordinates)[i], (*coordinates)[i + 1]);
				if ((dMinDistance < 0) || (dDis < dMinDistance))
				{
					dMinDistance = dDis;
					nSegIndex = i;
					pntProject = *((*coordinates)[i]) + (e2 * dot);
					bFindInLine = true;
				}
			}
		}
	}

	return dMinDistance;
}
Bool BaseAlgorithm3D::GetDistanceLinesegmentsToLinesegments(
	const Array<Engine::Geometries::Coordinate *> *coords1,
	const Array<Engine::Geometries::Coordinate *> *coords2,
	Double &averageDis,
	Double tolerance /* = Geometries_EP*/)
{
	averageDis = 0.0;
	Double maxDis = 0.0;
	Int32 num = 0;
	Engine::Geometries::Coordinate pntProject;
	for (auto itor = coords1->Begin(); itor != coords1->End(); itor++)
	{
		Geometries::Coordinate pnt = *(*itor);
		int nSegIndex = -1;
		Double tmpDis = GetDistancePointToLinesegments(pnt, coords2, pntProject, nSegIndex);
		if (nSegIndex >= 0)
		{
			averageDis = averageDis + tmpDis;
			num++;
			if (tmpDis > maxDis)
			{
				maxDis = tmpDis;
			}
		}
	}
	if (0 == num)
	{
		return false;
	}
	if (num > 1)
	{
		averageDis = (averageDis - maxDis) / (num - 1);
	}
	else
	{
		averageDis = averageDis / num;
	}
	return true;
}

Bool BaseAlgorithm3D::GetAverageDistanceLinesegmentsToLinesegments(
	const Array<Engine::Geometries::Coordinate *> *coords1,
	const Array<Engine::Geometries::Coordinate *> *coords2,
	Double &averageDis,
	Double tolerance /* = Geometries_EP*/)
{
	averageDis = 0.0;
	Double maxDis = 0.0;
	Int32 num = 0;
	Engine::Geometries::Coordinate pntProject;
	for (auto itor = coords1->Begin(); itor != coords1->End(); itor++)
	{
		Geometries::Coordinate pnt = *(*itor);
		int nSegIndex = -1;
		bool isInline = true;
		Double tmpDis = GetDistancePointToLinesegments(pnt, coords2, pntProject, nSegIndex, isInline);
		if (isInline)
		{
			averageDis = averageDis + tmpDis;
			num++;
			if (tmpDis > maxDis)
			{
				maxDis = tmpDis;
			}
		}
	}
	if (0 == num)
	{
		return false;
	}

	averageDis = averageDis / num;

	return true;
}

Bool BaseAlgorithm3D::IsIntersectRayPolygon(
	const Engine::Geometries::Coordinate &pntRayS,
	const Engine::Geometries::Coordinate &pntRayE,
	const Engine::Geometries::Coordinate *pntsPolygon,
	UInt32 nPointcount)
{
	Coordinate vector_RayS_i;
	Coordinate vector_RayS_i1;

	// 射线起点到终点的向量
	Coordinate vector_RayS_RayE = pntRayE - pntRayS;

	Coordinate cross;
	Double dot = 0.0;

	for (UInt32 i = 0; i < nPointcount - 1; i++)
	{
		vector_RayS_i = pntsPolygon[i] - pntRayS;
		vector_RayS_i1 = pntsPolygon[i + 1] - pntRayS;
		cross = vector_RayS_i.CrossProduct(vector_RayS_i1);
		cross.Normalize();
		dot = cross.DotProduct(vector_RayS_RayE);

		if (dot > 0)
		{
			return false;
		}
	}

	return true;
}

/*Bool BaseAlgorithm3D::GenerateRectangle(const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	const Engine::Geometries::Coordinate &pnt,
	Array<Engine::Geometries::Coordinate> &arrRectangle)
{
	//计算线段起点到线段起点在xy平面投影点的向量
	Engine::Geometries::Coordinate vec_SegFromProject;
	vec_SegFromProject.x = 0.0;
	vec_SegFromProject.y = 0.0;
	vec_SegFromProject.z = -pntSegFrom.z;

	Engine::Geometries::Coordinate vec_SegFrom_SegTo = pntSegTo - pntSegFrom;

	//计算线段所在平面的法向量
	Engine::Geometries::Coordinate normalVector = vec_SegFrom_SegTo.CrossProduct(vec_SegFromProject);
	normalVector.Normalize();

	//计算三维点在线段所在平面的投影点C'
	Engine::Geometries::Coordinate pntProject;
	Bool b = GetProjectpntToPlane(pnt, pntSegFrom, normalVector, pntProject);

	//计算C'在线段上的投影点C''
	Engine::Geometries::Coordinate pntProject1;
	b = GetProjectpntToLine(pntProject, pntSegFrom, pntSegTo, pntProject1);

	//计算向量C''C'
	Engine::Geometries::Coordinate vec = pntProject - pntProject1;

	//计算线段和C'组成的新矩形的另外两个角点
	Engine::Geometries::Coordinate pntSegFrom1 = pntSegFrom + vec;
	Engine::Geometries::Coordinate pntSegTo1 = pntSegTo + vec;

	arrRectangle.Add(pntSegFrom);
	arrRectangle.Add(pntSegTo);
	arrRectangle.Add(pntSegTo1);
	arrRectangle.Add(pntSegFrom1);

	return true;
}*/

/*Bool BaseAlgorithm3D::GenerateCircle(const Engine::Geometries::Coordinate &pnt1,
	Engine::Geometries::Coordinate &pnt2,
	Engine::Geometries::Coordinate &pnt3,
	Engine::Geometries::Coordinate &pntCircleCenter)
{
	/*A：第一个三维点；B、C：后两个三维点；B'、C'：后两个三维点在A所在平面的投影点；01：AB'的中点；O2：AC'的中点；P：新的圆的圆心；.：点积*/

// 计算A所在平面的法向量
/*Engine::Geometries::Coordinate z;
z.x = 0.0;
z.y = 0.0;
z.z = 1.0;

//将B、C投影到A所在的平面，得到投影点B'、C'
Engine::Geometries::Coordinate pnt2Project;
Bool b = GetProjectpntToPlane(pnt2, pnt1, z, pnt2Project);
Engine::Geometries::Coordinate pnt3Project;
b = GetProjectpntToPlane(pnt3, pnt1, z, pnt3Project);

//计算O1、O2
Engine::Geometries::Coordinate pntMid_12Project = (pnt1 + pnt2Project) / 2;
Engine::Geometries::Coordinate pntMid_13Project = (pnt1 + pnt3Project) / 2;

//计算向量AB'、AC'
Engine::Geometries::Coordinate pnt_pnt1_12Project = pnt2Project - pnt1;
Engine::Geometries::Coordinate pnt_pnt1_13Project = pnt3Project - pnt1;

//(P-O1).(B'-A)=0,(P-O2).(C'-A)=0
//进一步得，(x-O1.x,y-O1.y).(AB'.x,AB'.y)=0，(x-O2.x,y-O2.y).(AC'.x,AC'.y)=0
//进一步得，(x-O1.x)*AB'.x+(y-O1.y)*AB'.y，(x-O2.x)*AC'.x+(y-O2.y)*AC'.y=0
//进一步得，x*AB'.x+y*AB'.y=O1.x*AB'.x+O1.y*AB'.y，x*AC'.x+y*AC'.y=O2.x*AC'.x+O2.y*AC'.y
//进一步得，(AB'.x*AC'.y-AB'.y*AC'.x)*x=O1.x*AB'.x*AC'.y+O1.y*AB'.y*AC'.y-O2.x*AC'.x*AB'.y-O2.y*AC'.y*AB'.y，
//(AB'.y*AC'.x-AB'.x*AC'.y)*y=O1.x*AB'.x*AC'.x+O1.y*AB'.y*AC'.x-O2.x*AC'.x*AB'.x-O2.y*AC'.y*AB'.x
//最后计算出P的x、y的值
pntCircleCenter.x = (pntMid_12Project.x*pnt_pnt1_12Project.x*pnt_pnt1_13Project.y + pntMid_12Project.y*pnt_pnt1_12Project.y*pnt_pnt1_13Project.y -
	pntMid_13Project.x*pnt_pnt1_13Project.x*pnt_pnt1_12Project.y - pntMid_13Project.y*pnt_pnt1_13Project.y*pnt_pnt1_12Project.y) /
	(pnt_pnt1_12Project.x*pnt_pnt1_13Project.y - pnt_pnt1_12Project.y*pnt_pnt1_13Project.x);
pntCircleCenter.y = (pntMid_12Project.x*pnt_pnt1_12Project.x*pnt_pnt1_13Project.x + pntMid_12Project.y*pnt_pnt1_12Project.y*pnt_pnt1_13Project.x -
	pntMid_13Project.x*pnt_pnt1_13Project.x*pnt_pnt1_12Project.x - pntMid_13Project.y*pnt_pnt1_13Project.y*pnt_pnt1_12Project.x) /
	(pnt_pnt1_12Project.y*pnt_pnt1_13Project.x - pnt_pnt1_12Project.x*pnt_pnt1_13Project.y);

/**/

/*pntCircleCenter.z = pnt1.z;
pnt2.z = pnt1.z;
pnt3.z = pnt1.z;

return true;
}*/

UInt16 BaseAlgorithm3D::PntMatchLine(
	const Engine::Geometries::Coordinate &pntLineFrom,
	const Engine::Geometries::Coordinate &pntLineTo,
	const Engine::Geometries::Coordinate &pnt,
	const Engine::Geometries::Coordinate &vec_Normal)
{
	Coordinate vec_From_To = pntLineTo - pntLineFrom;
	vec_From_To.Normalize();
	Coordinate vec_From_t = pnt - pntLineFrom;
	vec_From_t.Normalize();
	Coordinate crossProduct = vec_From_To.CrossProduct(vec_From_t);

	if (crossProduct.GetLength() < Geometries_EP)
	{
		return 0;
	}

	crossProduct.Normalize();
	Double dotProduct = crossProduct.DotProduct(vec_Normal);

	if (dotProduct > Geometries_EP)
	{
		return 1;
	}
	else if (dotProduct < Geometries_NEP)
	{
		return 2;
	}

	return 0;
}

UInt16 BaseAlgorithm3D::PntMatchPlane(
	const Engine::Geometries::Coordinate &pntTest,
	const Engine::Geometries::Coordinate &pnt,
	Engine::Geometries::Coordinate vec_Normal)
{
	Coordinate vec = pntTest - pnt;
	vec.Normalize();
	vec_Normal.Normalize();
	Double dotProduct = vec.DotProduct(vec_Normal);

	if (dotProduct > Geometries_EP)
	{
		return 1;
	}
	else if (dotProduct < Geometries_NEP)
	{
		return 2;
	}

	return 0;
}

Bool BaseAlgorithm3D::IsIntersectLineSegments(
	const Engine::Geometries::Coordinate &pntSegS,
	const Engine::Geometries::Coordinate &pntSegE,
	const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	Double tolerance)
{
	if (Math::Equal(pntSegS.Distance(pntSegFrom), tolerance) || Math::Equal(pntSegS.Distance(pntSegTo), tolerance) || Math::Equal(pntSegE.Distance(pntSegFrom), tolerance) || Math::Equal(pntSegE.Distance(pntSegTo), tolerance))
	{
		return true;
	}

	Coordinate e1 = pntSegFrom - pntSegS;
	Coordinate e2 = pntSegE - pntSegS;
	Coordinate e3 = pntSegTo - pntSegS;
	Coordinate n = e1.CrossProduct(e2);
	n.Normalize();
	if (n.GetLength() < Geometries_EP) // 平面失效
	{
		Coordinate eFS = pntSegFrom - pntSegS;
		Coordinate eFE = pntSegFrom - pntSegE;
		Coordinate eTS = pntSegTo - pntSegS;
		Coordinate eTE = pntSegTo - pntSegE;
		Coordinate eSE = pntSegE - pntSegS;

		Double dFS = eFS.GetLength();
		Double dFE = eFE.GetLength();
		Double dTS = eTS.GetLength();
		Double dTE = eTE.GetLength();
		Double dSE = eSE.GetLength();

		dFS += dFE;
		dTS += dTE;

		dFS -= dSE;
		dTS -= dSE;

		return ((dFS < tolerance) || (dTS < tolerance));
	}

	e3.Normalize();
	Double dot = n.DotProduct(e3);

	// 如果CA、CD的叉积和CB的点积接近于0，即CA、CD的叉积和CB垂直，那么两线段共面
	if (dot < tolerance)
	{
		Coordinate n1 = e2.CrossProduct(e1);
		n1.Normalize();
		Coordinate n2 = e2.CrossProduct(e3);
		n2.Normalize();
		Double dot1 = n1.DotProduct(n2);
		e1 = pntSegS - pntSegFrom;
		e2 = pntSegTo - pntSegFrom;
		e3 = pntSegE - pntSegFrom;
		n1 = e2.CrossProduct(e1);
		n1.Normalize();
		n2 = e2.CrossProduct(e3);
		n2.Normalize();
		Double dot2 = n1.DotProduct(n2);
		return ((dot1 <= tolerance) && (dot2 <= tolerance));
	}

	return false;

	/**/
}

Bool BaseAlgorithm3D::IntersectionLineSegmentPlanes(
	const Engine::Geometries::Coordinate &pntSegFrom,
	const Engine::Geometries::Coordinate &pntSegTo,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntResult,
	Double tolerance)
{
	if (coordinates == NULL)
	{
		return false;
	}

	Int32 nPointCount = coordinates->GetCount();

	if (nPointCount < 2)
	{
		return false;
	}

	Double dDisTemp = pntSegFrom.Distance(pntSegTo);

	if (dDisTemp < tolerance)
	{
		return false;
	}

	// 线段投影到xoy平面
	Engine::Geometries::Coordinate pntSegFromNew = pntSegFrom;
	Engine::Geometries::Coordinate pntSegToNew = pntSegTo;
	pntSegFromNew.z = 0.0;
	pntSegToNew.z = 0.0;

	Int32 i = 0;
	Engine::Geometries::Coordinate pnt1;
	Engine::Geometries::Coordinate pnt2;
	Coordinate e1;
	Coordinate e2;
	Coordinate n;

	for (i = 0; i < nPointCount - 1; i++)
	{
		pnt1 = *((*coordinates)[i]);
		pnt2 = *((*coordinates)[i + 1]);
		pnt1.z = pnt2.z = 0.0;

		if (IsIntersectLineSegments(pnt1, pnt2, pntSegFromNew, pntSegToNew, tolerance))
		{
			pnt1 = *((*coordinates)[i]);
			pnt2 = *((*coordinates)[i + 1]);
			Engine::Geometries::Coordinate pnt3 = pnt1;
			pnt3.z += 100.0;
			e1 = pnt2 - pnt1;
			e2 = pnt3 - pnt1;
			n = e1.CrossProduct(e2);
			n.Normalize();

			return IntersectionLineSegmentPlane(pntSegFrom, pntSegTo, pnt1, n, pntResult);
		}
	}

	return false;
}

Double BaseAlgorithm3D::StandardAngle(const Double &angle)
{
	Double standardAngle = angle;
	// 首先转到（0，2*pi）范围内
	while (standardAngle < 0)
	{
		standardAngle += 2 * PI;
	}
	while (standardAngle > 2 * PI)
	{
		standardAngle -= 2 * PI;
	}
	// 转到（0，pi）范围内
	if (standardAngle > PI)
	{
		standardAngle = 2 * PI - standardAngle;
	}
	return standardAngle;
}
void BaseAlgorithm3D::CalLineStringMinMaxAngle(const LineString *pLineString,
											   Double &maxAngel, Double &minAngle)
{
	// 角度变弧度
	minAngle = 999;
	maxAngel = -999;
	if (pLineString == nullptr || pLineString->GetNumPoints() < 3)
	{
		minAngle = PI;
		maxAngel = PI;
		return;
	}
	// 夹角计算
	for (auto itor = pLineString->GetCoordinates()->Begin() + 1;
		 itor != pLineString->GetCoordinates()->End() - 1; itor++)
	{
		Engine::Geometries::Coordinate *pntS = *(itor - 1);
		Engine::Geometries::Coordinate *pntMid = *(itor);
		Engine::Geometries::Coordinate *pntE = *(itor + 1);
		Base::Double tmpAngle = StandardAngle(BaseAlgorithm::ComputeAngle(*pntS, *pntMid, *pntE));
		if (tmpAngle < minAngle)
		{
			minAngle = tmpAngle;
		}
		if (tmpAngle > maxAngel)
		{
			maxAngel = tmpAngle;
		}
	}
}
Double BaseAlgorithm3D::CalAzimuth3D(const Coordinate *pPointFrom, const Coordinate *pPointTo)
{
	double dAngle = 0.0;
	double dDistx = pPointTo->x - pPointFrom->x;
	double dDisty = pPointTo->y - pPointFrom->y;
	double dDistz = pPointTo->z - pPointFrom->z;
	double dDisxy = sqrtf(dDistx * dDistx + dDisty * dDisty);

	dAngle = atan2(abs(dDistz), dDisxy);
	return dAngle;
}

bool BaseAlgorithm3D::Flexibility3D(Array<Coordinate> &arrPoints, Int32 nIndex, const Coordinate &pntNewPos)
{
	Int32 nCount = arrPoints.GetCount();
	if ((nIndex < 0) || (nIndex >= nCount)) // 调整点索引越界
	{
		return false;
	}

	if (nIndex == 0) // 移动首点
	{
		Coordinate pntOldPos = arrPoints[nIndex];
		Coordinate pntEndPos = arrPoints[nCount - 1];

		SizeT i = 0;
		Coordinate pntCurrentPos;

		double dEndToOld = pntEndPos.Distance(pntOldPos);
		double dEndToNew = pntEndPos.Distance(pntNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(pntOldPos, pntEndPos, pntNewPos);

		Coordinate e1 = pntOldPos - pntEndPos;
		Coordinate e2 = pntNewPos - pntEndPos;

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pntCurrentPos = arrPoints[i];

			pntCurrentPos.x = pntEndPos.x + (pntCurrentPos.x - pntEndPos.x) * dRatio;
			pntCurrentPos.y = pntEndPos.y + (pntCurrentPos.y - pntEndPos.y) * dRatio;
			pntCurrentPos.z = pntEndPos.z + (pntCurrentPos.z - pntEndPos.z) * dRatio;

			BaseAlgorithm3D::RotatePoint(pntEndPos, dRotateAngle, NormalVector, pntCurrentPos);
			arrPoints[i] = pntCurrentPos;
		}

		arrPoints[nIndex] = pntNewPos;
	}
	else if (nIndex == (nCount - 1)) // 移动尾点
	{
		Coordinate pntOldPos = arrPoints[nIndex];
		Coordinate pntOriginPos = arrPoints[0];

		double dOriginToOld = pntOriginPos.Distance(pntOldPos);
		double dOriginToNew = pntOriginPos.Distance(pntNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(pntOldPos, pntOriginPos, pntNewPos);

		Coordinate e1 = pntOldPos - pntOriginPos;
		Coordinate e2 = pntNewPos - pntOriginPos;

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		SizeT i = 0;
		Coordinate pntCurrentPos;

		for (i = 1; i < nIndex; ++i)
		{
			pntCurrentPos = arrPoints[i];

			pntCurrentPos.x = pntOriginPos.x + (pntCurrentPos.x - pntOriginPos.x) * dRatio;
			pntCurrentPos.y = pntOriginPos.y + (pntCurrentPos.y - pntOriginPos.y) * dRatio;
			pntCurrentPos.z = pntOriginPos.z + (pntCurrentPos.z - pntOriginPos.z) * dRatio;

			BaseAlgorithm3D::RotatePoint(pntOriginPos, dRotateAngle, NormalVector, pntCurrentPos);
			arrPoints[i] = pntCurrentPos;
		}

		arrPoints[nIndex] = pntNewPos;
	}
	else // 移动中间点
	{
		Coordinate pntOldPos = arrPoints[nIndex];
		Coordinate pntOriginPos = arrPoints[0];
		Coordinate pntEndPos = arrPoints[nCount - 1];

		double dOriginToOld = pntOriginPos.Distance(pntOldPos);
		double dOriginToNew = pntOriginPos.Distance(pntNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(pntOldPos, pntOriginPos, pntNewPos);

		Coordinate e1 = pntOldPos - pntOriginPos;
		Coordinate e2 = pntNewPos - pntOriginPos;

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		SizeT i = 0;
		Coordinate pntCurrentPos;

		for (i = 1; i < nIndex; ++i)
		{
			pntCurrentPos = arrPoints[i];

			pntCurrentPos.x = pntOriginPos.x + (pntCurrentPos.x - pntOriginPos.x) * dRatio;
			pntCurrentPos.y = pntOriginPos.y + (pntCurrentPos.y - pntOriginPos.y) * dRatio;
			pntCurrentPos.z = pntOriginPos.z + (pntCurrentPos.z - pntOriginPos.z) * dRatio;

			BaseAlgorithm3D::RotatePoint(pntOriginPos, dRotateAngle, NormalVector, pntCurrentPos);
			arrPoints[i] = pntCurrentPos;
		}

		double dEndToOld = pntEndPos.Distance(pntOldPos);
		double dEndToNew = pntEndPos.Distance(pntNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		dRotateAngle = BaseAlgorithm::ComputeAngle(pntOldPos, pntEndPos, pntNewPos);

		e1 = pntOldPos - pntEndPos;
		e2 = pntNewPos - pntEndPos;

		e1.Normalize();
		e2.Normalize();

		NormalVector = e1.CrossProduct(e2);

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pntCurrentPos = arrPoints[i];

			pntCurrentPos.x = pntEndPos.x + (pntCurrentPos.x - pntEndPos.x) * dRatio;
			pntCurrentPos.y = pntEndPos.y + (pntCurrentPos.y - pntEndPos.y) * dRatio;
			pntCurrentPos.z = pntEndPos.z + (pntCurrentPos.z - pntEndPos.z) * dRatio;

			BaseAlgorithm3D::RotatePoint(pntEndPos, dRotateAngle, NormalVector, pntCurrentPos);
			arrPoints[i] = pntCurrentPos;
		}

		arrPoints[nIndex] = pntNewPos;
	}

	return true;
}

Engine::Base::Bool Engine::Geometries::BaseAlgorithm3D::ExtendedLineString(
	Engine::Geometries::LineString &lineString,
	Base::Double length, Base::Double tolerance /*= Geometries_EP*/)
{
	// 参数检测
	if (lineString.GetNumPoints() < 2)
	{
		return false;
	}
	// 首尾重合
	Coordinate *pStartNode = lineString.GetCoordinateN(0);
	Coordinate *pEndNode = lineString.GetCoordinateN(lineString.GetNumPoints() - 1);
	if (pStartNode->Equals(*pEndNode))
	{
		return false;
	}
	// 延长起点
	Coordinate *pStartSecondNode = lineString.GetCoordinateN(1);
	Coordinate startVec = *pStartNode - *pStartSecondNode;
	if (pStartNode->Equals(*pStartSecondNode)) // 判断是否重合
	{
		return false;
	}
	Coordinate newStartVect = startVec * (length / startVec.GetLength());
	Coordinate newStartNode = *pStartNode + newStartVect;
	// 延长终点
	Coordinate *pEndSecondNode = lineString.GetCoordinateN(lineString.GetNumPoints() - 2);
	Coordinate endVec = *pEndNode - *pEndSecondNode;
	if (pEndNode->Equals(*pEndSecondNode)) // 判断是否重合
	{
		return false;
	}
	Coordinate newEndVect = endVec * (length / endVec.GetLength());
	Coordinate newEndNode = *pEndNode + newEndVect;
	// 更新起始、结束点
	*pStartNode = newStartNode;
	*pEndNode = newEndNode;
	return true;
}

Base::Bool Engine::Geometries::BaseAlgorithm3D::GetPolygonRectangle(
	Geometries::Polygon *pObject,
	Base::Array<Geometries::Coordinate *> &coordinates)
{
	if (nullptr == pObject)
	{
		return false;
	}
	Base::Array<Geometries::Coordinate *> *arrPolygonCoords = pObject->GetExteriorRing()->GetCoordinates();
	// 少于3点不能构成面
	if (nullptr == arrPolygonCoords || arrPolygonCoords->GetCount() < 3)
	{
		return false;
	}
	// 找出Ploygon中与前两点不共线的点
	Geometries::Coordinate coord1 = *arrPolygonCoords->GetAt(0);
	Geometries::Coordinate coord2 = *arrPolygonCoords->GetAt(1);
	Geometries::Coordinate coord3;
	Bool tmpIsExist = false;
	for (int i = 2; i < arrPolygonCoords->GetCount(); i++)
	{
		Double tmpLength = DisPtToLine(coord1, coord2, *arrPolygonCoords->GetAt(i));
		if (tmpLength > Geometries_EP * 100)
		{
			coord3 = *arrPolygonCoords->GetAt(i);
			tmpIsExist = true;
			break;
		}
	}
	if (!tmpIsExist)
	{
		return false;
	}
	// 计算Polygon的面法向量
	Geometries::Coordinate normalVector;
	if (!CalcPlaneNormalVector(coord1, coord2, coord3, normalVector))
	{
		return false;
	}
	// 判断是否为水平面
	if (!normalVector.CrossProduct(Geometries::Coordinate(0, 0, 1)).IsZeroVector())
	{
		// 获取Z值最大和最小的两个点
		Geometries::Coordinate minZValueCoord(0, 0, 9e9);
		Geometries::Coordinate maxZValueCoord(0, 0, -9e9);
		for (int i = 0; i < arrPolygonCoords->GetCount(); i++)
		{
			if (arrPolygonCoords->GetAt(i)->z < minZValueCoord.z)
			{
				minZValueCoord = *arrPolygonCoords->GetAt(i);
			}
			if (arrPolygonCoords->GetAt(i)->z > maxZValueCoord.z)
			{
				maxZValueCoord = *arrPolygonCoords->GetAt(i);
			}
		}
		// 计算方向
		Geometries::Coordinate dirVector = normalVector.CrossProduct(Geometries::Coordinate(0, 0, 1));
		Geometries::Coordinate minZValueCoord1 = minZValueCoord + dirVector;
		Geometries::Coordinate maxZValueCoord1 = maxZValueCoord + dirVector;

		// 计算Polygon上的形状点在直线（minZValueCoord，minZValueCoord1）上的投影
		Base::Array<Geometries::Coordinate> arrProjCoords;
		for (int i = 0; i < arrPolygonCoords->GetCount(); i++)
		{
			Engine::Geometries::Coordinate pntProject;
			if (GetProjectpntToLine(*arrPolygonCoords->GetAt(i),
									minZValueCoord, minZValueCoord1, pntProject))
			{
				arrProjCoords.Add(pntProject);
			}
		}
		if (arrProjCoords.GetCount() <= 0)
		{
			return false;
		}
		// 计算点集arrProjCoords最外侧的两个点
		Geometries::Coordinate outsideCoord1 = arrProjCoords.GetAt(0);
		Geometries::Coordinate outsideCoord2;
		Double maxDis = -1;
		for (int i = 0; i < arrProjCoords.GetCount(); i++)
		{
			if (outsideCoord1.Distance(arrProjCoords.GetAt(i)) > maxDis)
			{
				outsideCoord2 = arrProjCoords.GetAt(i);
				maxDis = outsideCoord1.Distance(arrProjCoords.GetAt(i));
			}
		}
		maxDis = -1;
		for (int i = 0; i < arrProjCoords.GetCount(); i++)
		{
			if (outsideCoord2.Distance(arrProjCoords.GetAt(i)) > maxDis)
			{
				outsideCoord1 = arrProjCoords.GetAt(i);
				maxDis = outsideCoord2.Distance(arrProjCoords.GetAt(i));
			}
		}
		// 计算outsideCoord1，outsideCoord2在直线（maxZValueCoord，maxZValueCoord1）上的投影
		Geometries::Coordinate outsideCoord1_1;
		Geometries::Coordinate outsideCoord2_1;
		if (!GetProjectpntToLine(outsideCoord1, maxZValueCoord, maxZValueCoord1, outsideCoord1_1) || !GetProjectpntToLine(outsideCoord2, maxZValueCoord, maxZValueCoord1, outsideCoord2_1))
		{
			return false;
		}
		coordinates.Add(new Geometries::Coordinate(outsideCoord1));
		coordinates.Add(new Geometries::Coordinate(outsideCoord1_1));
		coordinates.Add(new Geometries::Coordinate(outsideCoord2_1));
		coordinates.Add(new Geometries::Coordinate(outsideCoord2));
		return true;
	}
	else // 水平面
	{
		Double minX = 9e9;
		Double maxX = -9e9;
		Double minY = 9e9;
		Double maxY = -9e9;
		for (int i = 0; i < arrPolygonCoords->GetCount(); i++)
		{
			Coordinate tmpCoord = *arrPolygonCoords->GetAt(i);
			if (tmpCoord.x < minX)
			{
				minX = tmpCoord.x;
			}
			if (tmpCoord.y < minY)
			{
				minY = tmpCoord.y;
			}
			if (tmpCoord.x > maxX)
			{
				maxX = tmpCoord.x;
			}
			if (tmpCoord.y > maxY)
			{
				maxY = tmpCoord.y;
			}
		}
		Double zValue = arrPolygonCoords->GetAt(0)->z;

		coordinates.Add(new Geometries::Coordinate(minX, minY, zValue));
		coordinates.Add(new Geometries::Coordinate(minX, maxY, zValue));
		coordinates.Add(new Geometries::Coordinate(maxX, maxY, zValue));
		coordinates.Add(new Geometries::Coordinate(maxX, minY, zValue));
		return true;
	}
	return false;
}

Base::Array<Geometries::Coordinate> Engine::Geometries::BaseAlgorithm3D::CalcPointOfLinestrSelfIntersect(
	const Geometries::LineString *pLineString)
{
	Base::Array<Geometries::Coordinate> arrInterCoord;
	if (nullptr == pLineString)
	{
		return arrInterCoord;
	}
	struct LineSegment
	{
		Geometries::Coordinate coord1;
		Geometries::Coordinate coord2;
	};
	// 计算线段上一点pnt的z值
	auto CalcInterSectionZValue = [=](LineSegment lineSg, Geometries::Coordinate &pnt) -> bool
	{
		double xDlta1 = lineSg.coord1.x - lineSg.coord2.x;
		double yDlta1 = lineSg.coord1.y - lineSg.coord2.y;
		double xDlta2 = pnt.x - lineSg.coord2.x;
		double yDlta2 = pnt.y - lineSg.coord2.y;
		double k = 0.0;
		if (std::fabs(xDlta1) < Geometries_EP && std::fabs(yDlta1) < Geometries_EP)
		{
			return false;
		}
		if (std::fabs(yDlta1) > Geometries_EP)
		{
			k = yDlta2 / yDlta1;
		}
		else
		{
			k = xDlta2 / xDlta1;
		}
		double pntZ = k * (lineSg.coord1.z - lineSg.coord2.z) + lineSg.coord2.z;
		pnt.z = pntZ;
		return true;
	};
	// 划分线段
	Base::Array<LineSegment> arrLineSegment;
	for (int i = 0; i < pLineString->GetNumPoints() - 1; i++)
	{
		LineSegment tmpLineSegment;
		tmpLineSegment.coord1 = *pLineString->GetCoordinateN(i);
		tmpLineSegment.coord2 = *pLineString->GetCoordinateN(i + 1);
		arrLineSegment.Add(tmpLineSegment);
	}
	// 计算线段交点
	for (int iFirst = 0; iFirst < arrLineSegment.GetCount() - 1; iFirst++)
	{
		LineSegment lineSgFirst = arrLineSegment.GetAt(iFirst);
		for (int iSecond = iFirst + 1; iSecond < arrLineSegment.GetCount(); iSecond++)
		{
			LineSegment lineSgSecond = arrLineSegment.GetAt(iSecond);
			Geometries::Coordinate pntResult;
			if (IRCreatLaneAlgorithm::InterPtOfLineSegs(lineSgFirst.coord1, lineSgFirst.coord2,
														lineSgSecond.coord1, lineSgSecond.coord2, pntResult))
			{
				// 计算在线段lineSgFirst上的交点Z坐标
				if (CalcInterSectionZValue(lineSgFirst, pntResult))
				{
					arrInterCoord.Add(pntResult);
				}
				// 计算在线段lineSgSecond上的交点Z坐标
				if (CalcInterSectionZValue(lineSgSecond, pntResult))
				{
					arrInterCoord.Add(pntResult);
				}
			}
		}
	}
	return arrInterCoord;
}
Base::Array<Geometries::Coordinate> Engine::Geometries::BaseAlgorithm3D::calIntersectPointOfLines(
	Geometries::LineString *pLineStringF, const Geometries::LineString *pLineStringS)
{
	Base::Array<Geometries::Coordinate> arrInterCoord;
	Base::Set<Geometries::Coordinate *> temp;
	if (nullptr == pLineStringF || nullptr == pLineStringS)
	{
		return arrInterCoord;
	}
	struct LineSegment
	{
		Geometries::Coordinate coord1;
		Geometries::Coordinate coord2;
	};
	// 划分线段
	Base::Array<LineSegment> arrLineSegmentS;
	for (int i = 0; i < pLineStringS->GetNumPoints() - 1; i++)
	{
		LineSegment tmpLineSegment;
		tmpLineSegment.coord1 = *pLineStringS->GetCoordinateN(i);
		tmpLineSegment.coord2 = *pLineStringS->GetCoordinateN(i + 1);
		arrLineSegmentS.Add(tmpLineSegment);
	}

	if (arrLineSegmentS.GetCount() == 0)
	{
		return arrInterCoord;
	}
	for (int i = 0; i < arrLineSegmentS.GetCount(); i++)
	{
		LineSegment lineSgSecond = arrLineSegmentS.GetAt(i);
		Array<Geometries::Coordinate *> arrCoor;
		Array<Double> arrMeasure;
		IRCreatLaneAlgorithm::GetInterceptLineSegmentWithMultLine(pLineStringF, &lineSgSecond.coord1, &lineSgSecond.coord2,
																  arrCoor, arrMeasure);
		if (arrCoor.GetCount() != 0)
		{
			for (int t = 0; t < arrCoor.GetCount(); t++)
			{
				temp.Add(arrCoor.GetAt(t));
			}
		}
	}
	if (temp.GetCount() != 0)
	{
		Base::Set<Geometries::Coordinate *>::_Iter itr = temp.Begin();
		for (; itr != temp.End(); ++itr)
		{
			Engine::Geometries::Coordinate *p = *itr;
			arrInterCoord.Add(*p);
		}
	}

	return arrInterCoord;
}