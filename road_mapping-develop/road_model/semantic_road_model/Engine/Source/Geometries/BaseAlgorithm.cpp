/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:BaseAlgorithm.cpp
简要描述:
******************************************************************/

#include "Geometries/BaseAlgorithm.h"
#include <math.h>

using namespace Engine::Base;
using namespace Engine::Geometries;

// 计算方位角
Double BaseAlgorithm::CalAzimuth(const Coordinate *pPointFrom, const Coordinate *pPointTo)
{
	double dAngle = 0.0;
	double dDistx = pPointTo->x - pPointFrom->x;
	double dDisty = pPointTo->y - pPointFrom->y;
	dAngle = atan2(dDisty, dDistx);
	dAngle = PI / 2 - dAngle;

	if (dAngle < 0)
	{
		dAngle += 2 * PI;
	}

	return dAngle;
}

Double BaseAlgorithm::ComputeAngle(const Coordinate &coord1, const Coordinate &coord2, const Coordinate &coord3)
{
	Coordinate v21 = coord1 - coord2;
	Coordinate v23 = coord3 - coord2;

	v21.Normalize();
	v23.Normalize();

	Double cosa = v21 * v23;

	if (cosa > 1.0)
	{
		cosa = 1.0;
	}
	else if (cosa < -1.0)
	{
		cosa = -1.0;
	}

	Double arca = acos(cosa);
	return arca;
}

// 点的旋转
Void BaseAlgorithm::RotatePoint(Coordinate *pPointOrg, double dAngle, Coordinate *pPointResult)
{
	double dCosAngle = cos(dAngle);
	double dSinAngle = sin(dAngle);
	double xx = pPointResult->x - pPointOrg->x;
	double yy = pPointResult->y - pPointOrg->y;
	pPointResult->x = xx * dCosAngle - yy * dSinAngle + pPointOrg->x;
	pPointResult->y = xx * dSinAngle + yy * dCosAngle + pPointOrg->y;
}

// 点到直线距离
Double BaseAlgorithm::DistancePtToLine(
	const Coordinate *pt, const Coordinate *ptStart,
	const Coordinate *ptEnd)
{
	Double distPtToLine = 0.0;

	if (pt == NULL || ptStart == NULL || ptEnd == NULL)
	{
		return distPtToLine;
	}

	Double distSE = sqrt(pow(ptStart->x - ptEnd->x, 2) + pow(ptStart->y - ptEnd->y, 2) + pow(ptStart->z - ptEnd->z, 2));
	Double distSi = sqrt(pow(ptStart->x - pt->x, 2) + pow(ptStart->y - pt->y, 2) + pow(ptStart->z - pt->z, 2));
	Double distiE = sqrt(pow(pt->x - ptEnd->x, 2) + pow(pt->y - ptEnd->y, 2) + pow(pt->z - ptEnd->z, 2));
	Double p = (distSE + distSi + distiE) / 2;

	if (p <= distSE)
	{
		return distPtToLine;
	}

	Double S = sqrt(p * (p - distSE) * (p - distiE) * (p - distSi));
	distPtToLine = (S * 2) / distSE;

	return distPtToLine;
}

Double BaseAlgorithm::DistancePtToLine2D(
	const Coordinate *pt, const Coordinate *ptStart,
	const Coordinate *ptEnd)
{
	Double distPtToLine = 0.0;

	if (pt == NULL || ptStart == NULL || ptEnd == NULL)
	{
		return distPtToLine;
	}

	Double distSE = sqrt(pow(ptStart->x - ptEnd->x, 2) + pow(ptStart->y - ptEnd->y, 2));
	Double distSi = sqrt(pow(ptStart->x - pt->x, 2) + pow(ptStart->y - pt->y, 2));
	Double distiE = sqrt(pow(pt->x - ptEnd->x, 2) + pow(pt->y - ptEnd->y, 2));
	Double p = (distSE + distSi + distiE) / 2;

	if (p <= distSE)
	{
		return distPtToLine;
	}

	Double S = sqrt(p * (p - distSE) * (p - distiE) * (p - distSi));
	distPtToLine = (S * 2) / distSE;

	return distPtToLine;
}

Bool BaseAlgorithm::IsProjectToLineset(
	const Engine::Geometries::Coordinate *pntHitTest,
	const Engine::Geometries::Coordinate *pntLinsectStart,
	const Engine::Geometries::Coordinate *pntLinsectEnd)
{
	if (pntLinsectStart->x == pntLinsectEnd->x && pntLinsectStart->y == pntLinsectEnd->y)
		return true;

	double a2 = (pntHitTest->x - pntLinsectStart->x) * (pntHitTest->x - pntLinsectStart->x) + (pntHitTest->y - pntLinsectStart->y) * (pntHitTest->y - pntLinsectStart->y);
	double b2 = (pntHitTest->x - pntLinsectEnd->x) * (pntHitTest->x - pntLinsectEnd->x) + (pntHitTest->y - pntLinsectEnd->y) * (pntHitTest->y - pntLinsectEnd->y);
	double c2 = (pntLinsectEnd->x - pntLinsectStart->x) * (pntLinsectEnd->x - pntLinsectStart->x) + (pntLinsectEnd->y - pntLinsectStart->y) * (pntLinsectEnd->y - pntLinsectStart->y);

	if (b2 + c2 < a2 || a2 + c2 < b2)
	{
		return false;
	}
	else
	{
		return true;
	}
}

Engine::Geometries::Coordinate BaseAlgorithm::GetPtToLine(
	const Engine::Geometries::Coordinate *pntStart,
	const Engine::Geometries::Coordinate *pntEnd,
	const Engine::Geometries::Coordinate *pntHitTest)
{
	if (pntStart->x == pntEnd->x && pntStart->y == pntEnd->y)
	{
		return *pntStart;
	}

	Engine::Geometries::Coordinate pntMiddle;
	double dDaltaX = pntEnd->x - pntStart->x;
	double dDaltaY = pntStart->y - pntEnd->y;
	double dDaltaX2 = dDaltaX * dDaltaX;
	double dDaltaY2 = dDaltaY * dDaltaY;
	double dDeltaXY = dDaltaX * dDaltaY;
	double dLineSectDist = dDaltaX * dDaltaX + dDaltaY * dDaltaY;
	pntMiddle.x = (dDeltaXY * (pntStart->y - pntHitTest->y) +
				   pntStart->x * dDaltaY2 + pntHitTest->x * dDaltaX2) /
				  dLineSectDist;
	pntMiddle.y = (dDeltaXY * (pntStart->x - pntHitTest->x) +
				   pntStart->y * dDaltaX2 + pntHitTest->y * dDaltaY2) /
				  dLineSectDist;

	return pntMiddle;
}

// 二维点和点的距离计算
Double BaseAlgorithm::DistancePtToPt(
	const Engine::Geometries::Coordinate *pntFrom,
	const Engine::Geometries::Coordinate *pntTo)
{
	return sqrt((pntFrom->x - pntTo->x) * (pntFrom->x - pntTo->x) +
				(pntFrom->y - pntTo->y) * (pntFrom->y - pntTo->y));
}

// 是否为0的判断方法
Bool BaseAlgorithm::Is0(const double &dValue)
{
	return ((dValue > Geometries_NEP) && (dValue < Geometries_EP));
}

// 判断点是否在直线上
Bool BaseAlgorithm::IsPointOnLine(
	const Engine::Geometries::Coordinate &pntFrom,
	const Engine::Geometries::Coordinate &pntTo,
	const Engine::Geometries::Coordinate &pntTest)
{
	if (pntFrom.x == pntTo.x && pntFrom.y == pntTo.y)
	{
		return false;
	}

	if (Is0(pntTo.x - pntFrom.x))
	{
		return Is0(pntTo.x - pntTest.x);
	}

	if (Is0(pntTo.y - pntFrom.y))
	{
		return Is0(pntTo.y - pntTest.y);
	}

	if ((pntFrom.x == pntTest.x && pntFrom.y == pntTest.y) ||
		(pntTo.x == pntTest.x && pntTo.y == pntTest.y))
	{
		return true;
	}

	if (Is0(pntTo.x - pntTest.x))
	{
		return false;
	}

	double dResult = ((pntTest.y - pntTo.y) * (pntTo.x - pntFrom.x) -
					  (pntTo.y - pntFrom.y) * (pntTest.x - pntTo.x));
	dResult /= ((pntTo.x - pntFrom.x) * (pntTest.x - pntTo.x));

	if (dResult < Geometries_EP && dResult > Geometries_NEP)
	{
		return true;
	}
	else
	{
		return false;
	}
}

// 返回0  在直线上
// 返回1  在直线左侧
// 返回2  在直线右侧
UInt16 BaseAlgorithm::PntMatchLine(
	const Engine::Geometries::Coordinate &pntFrom,
	const Engine::Geometries::Coordinate &pntTo,
	const Engine::Geometries::Coordinate &pntTest)
{
	if (IsPointOnLine(pntFrom, pntTo, pntTest))
	{
		return 0;
	}

	double dResult = ((pntTest.y - pntTo.y) * (pntTo.x - pntFrom.x) -
					  (pntTo.y - pntFrom.y) * (pntTest.x - pntTo.x));

	if (dResult < 0)
	{
		return 2;
	}
	else
	{
		return 1;
	}
}

// 获得线上距离线外最近的点
Bool BaseAlgorithm::GetNearestPntToLineset(const Engine::Geometries::Coordinate &pntHitTest,
										   const Array<Engine::Geometries::Coordinate> &coordinates,
										   Engine::Geometries::Coordinate &pntProject,
										   Int32 &nSegIndex)
{
	if (coordinates.GetCount() < 2)
	{
		return false;
	}

	double dMinDis = -1.0;
	double dDistanceTemp = -1.0;
	Engine::Geometries::Coordinate pntProjecttemp;
	long i;

	for (i = 0; i < coordinates.GetCount() - 1; i++)
	{
		if (BaseAlgorithm::IsProjectToLineset(&pntHitTest, &(coordinates[i]), &(coordinates[i + 1])))
		{
			pntProjecttemp = BaseAlgorithm::GetPtToLine(&(coordinates[i]), &(coordinates[i + 1]), &pntHitTest);
			dDistanceTemp = BaseAlgorithm::DistancePtToPt(&pntProjecttemp, &pntHitTest);
		}
		else
		{
			continue;
		}

		if (dMinDis < 0.0 || dDistanceTemp < dMinDis)
		{
			dMinDis = dDistanceTemp;
			pntProject.x = pntProjecttemp.x;
			pntProject.y = pntProjecttemp.y;
			nSegIndex = i;
		}
	}

	for (i = 0; i < coordinates.GetCount(); i++)
	{
		dDistanceTemp = coordinates[i].DistanceXY(pntHitTest);

		if (dMinDis < 0.0 || dDistanceTemp < dMinDis)
		{
			pntProject = coordinates[i];
			dMinDis = dDistanceTemp;
			nSegIndex = i - 1;
		}
	}

	nSegIndex = nSegIndex < 0 ? 0 : nSegIndex;

	return true;
}

Bool BaseAlgorithm::GetNearestPntToLineset(
	const Engine::Geometries::Coordinate *pntHitTest,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject,
	Int32 &nSegIndex)
{
	if (pntHitTest == NULL || coordinates == NULL)
	{
		return false;
	}

	if (coordinates->GetCount() < 2)
	{
		return false;
	}

	double dMinDis = -1.0;
	double dDistanceTemp = -1.0;
	Engine::Geometries::Coordinate pntProjecttemp;
	long i;

	for (i = 0; i < coordinates->GetCount() - 1; i++)
	{
		Engine::Geometries::Coordinate *pntFrom = (*coordinates)[i];
		Engine::Geometries::Coordinate *pntTo = (*coordinates)[i + 1];

		if (BaseAlgorithm::IsProjectToLineset(pntHitTest, pntFrom, pntTo))
		{
			pntProjecttemp = BaseAlgorithm::GetPtToLine(pntFrom, pntTo, pntHitTest);
			dDistanceTemp = BaseAlgorithm::DistancePtToPt(&pntProjecttemp, pntHitTest);
		}
		else
		{
			continue;
		}

		if (dMinDis < 0.0 || dDistanceTemp < dMinDis)
		{
			dMinDis = dDistanceTemp;
			pntProject.x = pntProjecttemp.x;
			pntProject.y = pntProjecttemp.y;
			nSegIndex = i;
		}
	}

	for (i = 0; i < coordinates->GetCount(); i++)
	{
		dDistanceTemp = BaseAlgorithm::DistancePtToPt((*coordinates)[i], pntHitTest);

		if (dMinDis < 0.0 || dDistanceTemp < dMinDis)
		{
			pntProject.x = (*coordinates)[i]->x;
			pntProject.y = (*coordinates)[i]->y;
			dMinDis = dDistanceTemp;
			nSegIndex = i - 1;
		}
	}

	/*if (nSegIndex == coordinates->GetCount() - 1)
	{
		nSegIndex = coordinates->GetCount() - 2;
	}*/

	nSegIndex = nSegIndex < 0 ? 0 : nSegIndex;

	return true;
}
Bool BaseAlgorithm::GetNearestPntToLineset2D(
	const Engine::Geometries::Coordinate *pntHitTest,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::Coordinate &pntProject)
{
	if (pntHitTest == NULL || coordinates == NULL)
	{
		return false;
	}

	if (coordinates->GetCount() < 2)
	{
		return false;
	}

	double dMinDis = 9e9;

	for (Base::Int32 i = 0; i < coordinates->GetCount() - 1; i++)
	{
		double dDistanceTemp = coordinates->GetAt(i)->DistanceXY(*pntHitTest);
		if (dDistanceTemp < dMinDis)
		{
			dMinDis = dDistanceTemp;
			pntProject = *coordinates->GetAt(i);
		}
	}
	return true;
}
Void BaseAlgorithm::PointsPackage(
	const Array<Coordinate> pntsSource,
	Array<Coordinate> &pntsRezult)
{
	pntsRezult.Clear();
	SizeT nSourceCount = pntsSource.GetCount();

	if (nSourceCount == 0)
	{
		return;
	}

	Coordinate pntMaxY = pntsSource[0];
	Coordinate pnt1;
	Coordinate pnt2;
	SizeT i = 0;
	SizeT maxIndex = 0;
	for (i = 1; i < nSourceCount; i++)
	{
		if (pntsSource[i].y > pntMaxY.y)
		{
			pntMaxY = pntsSource[i];
			maxIndex = i;
		}
		else if ((pntsSource[i].y == pntMaxY.y) && (pntsSource[i].x > pntMaxY.x))
		{
			pntMaxY = pntsSource[i];
			maxIndex = i;
		}
	}

	pnt1 = pntMaxY;
	pntsRezult.Add(pnt1);
	Double dAngleFrom = 0.0;
	Double dAngleFirst = -1.0;
	std::vector<int> resIndex;
	bool isfind = false;
	while (1)
	{
		Double dMinAngle = 2 * PI;
		Int32 nMinIndex = -1;

		for (i = 0; i < nSourceCount; i++)
		{
			pnt2 = pntsSource[i];

			if (pnt2.x == pnt1.x &&
				pnt2.y == pnt1.y)
			{
				continue;
			}

			Double dAngle = CalAzimuth(&pnt1, &pnt2);

			// if (dAngle < dMinAngle && dAngle >= dAngleFrom)
			if (((dAngle + Geometries_EP) < dMinAngle) && (dAngle >= dAngleFrom))
			{
				dMinAngle = dAngle;
				nMinIndex = i;
			}
			else if (Is0(dAngle - dMinAngle) && dAngle >= dAngleFrom && nMinIndex >= 0)
			{
				Double dDistance1 = DistancePtToPt(&pnt1, &pnt2);
				Double dDistance2 = DistancePtToPt(&pnt1, &pntsSource[nMinIndex]);

				if (dDistance1 > dDistance2)
				{
					dMinAngle = dAngle;
					nMinIndex = i;
				}
			}
		}

		if (nMinIndex < 0)
		{
			dMinAngle = 2 * PI;
			dAngleFrom = 0.0;

			for (i = 0; i < nSourceCount; i++)
			{
				pnt2 = pntsSource[i];

				if (pnt2.x == pnt1.x &&
					pnt2.y == pnt1.y)
				{
					continue;
				}

				Double dAngle = CalAzimuth(&pnt1, &pnt2);

				if (dAngle < dMinAngle && dAngle >= dAngleFrom)
				{
					dMinAngle = dAngle;
					nMinIndex = i;
				}
				else if (Is0(dAngle - dMinAngle) && dAngle >= dAngleFrom && nMinIndex >= 0)
				{
					Double dDistance1 = DistancePtToPt(&pnt1, &pnt2);
					Double dDistance2 = DistancePtToPt(&pnt1, &pntsSource[nMinIndex]);

					if (dDistance1 > dDistance2)
					{
						dMinAngle = dAngle;
						nMinIndex = i;
					}
				}
			}
		}

		if (nMinIndex < 0)
		{
			break;
		}

		pnt2 = pntsSource[nMinIndex];
		Double dAngleFrom1 = CalAzimuth(&pnt1, &pnt2);
		dAngleFrom = dAngleFrom1;
		pnt1 = pnt2;
		pntsRezult.Add(pnt1);
		auto isfindindex = find(resIndex.begin(), resIndex.end(), nMinIndex);
		if (isfindindex == resIndex.end())
			resIndex.push_back(nMinIndex);
		else
			break;
		if (dAngleFirst < 0.0)
		{
			dAngleFirst = dAngleFrom;
		}
		else if (Is0(dAngleFirst - dAngleFrom))
		{
			break;
		}

		if (pnt1.x == pntMaxY.x &&
			pnt1.y == pntMaxY.y)
		{
			break;
		}
	}

	return;
}

Void BaseAlgorithm::PointsPackage(
	const Array<Coordinate *> *pnt2sSource,
	Array<Coordinate *> *pnt2sRezult)
{
	pnt2sRezult->Clear();
	SizeT nSourceCount = pnt2sSource->GetCount();

	if (nSourceCount == 0)
	{
		return;
	}

	Coordinate *pntMaxY = (*pnt2sSource)[0];
	Coordinate *pnt1 = NULL;
	Coordinate *pnt2 = NULL;
	SizeT i = 0;

	for (i = 1; i < nSourceCount; i++)
	{
		if ((*pnt2sSource)[i]->y > pntMaxY->y)
		{
			pntMaxY = (*pnt2sSource)[i];
		}
		else if (((*pnt2sSource)[i]->y == pntMaxY->y) && ((*pnt2sSource)[i]->x > pntMaxY->x))
		{
			pntMaxY = (*pnt2sSource)[i];
		}
	}

	pnt1 = new Coordinate(pntMaxY->x, pntMaxY->y, pntMaxY->z);
	pnt2sRezult->Add(pnt1);
	Double dAngleFrom = 0.0;

	Double dAngleFirst = -1.0;

	while (true)
	{
		Double dMinAngle = 2 * PI;
		Int32 nMinIndex = -1;

		for (i = 0; i < nSourceCount; i++)
		{
			pnt2 = (*pnt2sSource)[i];

			if (pnt2->x == pnt1->x &&
				pnt2->y == pnt1->y)
			{
				continue;
			}

			Double dAngle = CalAzimuth(pnt1, pnt2);

			if (((dAngle + Geometries_EP) < dMinAngle) && (dAngle >= dAngleFrom))
			{
				dMinAngle = dAngle;
				nMinIndex = i;
			}
			else if (Is0(dAngle - dMinAngle) && dAngle >= dAngleFrom && nMinIndex >= 0)
			{
				Double dDistance1 = DistancePtToPt(pnt1, pnt2);
				Double dDistance2 = DistancePtToPt(pnt1, (*pnt2sSource)[nMinIndex]);

				if (dDistance1 > dDistance2)
				{
					dMinAngle = dAngle;
					nMinIndex = i;
				}
			}
		}

		if (nMinIndex < 0) // 可能是临界造成的原因
		{
			dMinAngle = 2 * PI;
			dAngleFrom = 0.0;

			for (i = 0; i < nSourceCount; i++)
			{
				pnt2 = (*pnt2sSource)[i];

				if (pnt2->x == pnt1->x &&
					pnt2->y == pnt1->y)
				{
					continue;
				}

				Double dAngle = CalAzimuth(pnt1, pnt2);

				if (dAngle < dMinAngle && dAngle >= dAngleFrom)
				{
					dMinAngle = dAngle;
					nMinIndex = i;
				}
				else if (Is0(dAngle - dMinAngle) && dAngle >= dAngleFrom && nMinIndex >= 0)
				{
					Double dDistance1 = DistancePtToPt(pnt1, pnt2);
					Double dDistance2 = DistancePtToPt(pnt1, (*pnt2sSource)[nMinIndex]);

					if (dDistance1 > dDistance2)
					{
						dMinAngle = dAngle;
						nMinIndex = i;
					}
				}
			}
		}

		if (nMinIndex < 0)
		{
			break;
		}

		pnt2 = (*pnt2sSource)[nMinIndex];
		Double dAngleFrom1 = CalAzimuth(pnt1, pnt2);
		dAngleFrom = dAngleFrom1;
		pnt1 = new Coordinate(pnt2->x, pnt2->y, pnt2->z);
		pnt2sRezult->Add(pnt1);

		if (dAngleFirst < 0.0)
		{
			dAngleFirst = dAngleFrom;
		}
		else if (Is0(dAngleFirst - dAngleFrom))
		{
			break;
		}

		if (pnt1->x == pntMaxY->x &&
			pnt1->y == pntMaxY->y)
		{
			break;
		}
	}

	return;
}

UInt16 BaseAlgorithm::PntMatchLineSegments(
	const Array<Engine::Geometries::Coordinate *> *pPoints,
	const Engine::Geometries::Coordinate &pntTest)
{
	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	GetNearestPntToLineset(&pntTest, pPoints, pntProject, nSegIndex);
	Engine::Geometries::Coordinate *pntFrom = (*pPoints)[nSegIndex];
	Engine::Geometries::Coordinate *pntTo = (*pPoints)[nSegIndex + 1];

	return PntMatchLine(*pntFrom, *pntTo, pntTest);
}

Bool BaseAlgorithm::PerpendicularLine(const Coordinate &hitPoint, const Array<Engine::Geometries::Coordinate *> *coordinates, Coordinate &p0, Coordinate &p1)
{
	int nSegIndex = -1;
	if (!GetNearestPntToLineset(&hitPoint, coordinates, p0, nSegIndex))
		return false;

	Vector3d v = *(coordinates->GetAt(nSegIndex + 1)) - *(coordinates->GetAt(nSegIndex));
	Vector3d vNormal;
	vNormal.x = -v.y;
	vNormal.y = v.x;
	vNormal.z = 0;

	p1 = p0 + vNormal;
	return true;
}
