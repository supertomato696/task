#include "Geometries/SCHAlgorithm/KPIAlgorithm.h"
#include "Geometries/Spline.h"
#include "Geometries/BaseAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace Engine;

Bool SplineInterpolateUtility::InterpolateFlatZSpline(Spline* pSpline, const Base::Array<Coordinate*>* pCoorArray, const Coordinate* pHeadPoint, const Coordinate* pTailPoint)
{
	/* CHECK DATA */
	if (NULL == pSpline)
		return false;

	if (NULL == pCoorArray)
		return false;

	/*
	* 暂时不支持 3点下的曲线内插 2016.6.13
	*/
	if (pCoorArray->GetCount() < 3)
		return false;

	Array<Coordinate*>* pArray = new Array<Coordinate*>(pCoorArray->GetCount());

	/**抹平Z值*/
	for (SizeT i = 0, n = pArray->GetCount(); i < n; i++)
	{
		const Coordinate* pCoor = pCoorArray->ElementAt(i);
		pArray->SetAt(i,new Coordinate(pCoor->x, pCoor->y, 0));
	}

	//清楚spline 系数
	pSpline->Clear();

	Vector3d vHeadTangent, vTailTangent;

	/*
	*	是否指定了首点
	*/

	if (NULL != pHeadPoint)
	{
		Coordinate pHeadPointFlat(pHeadPoint->x, pHeadPoint->y, 0);
		vHeadTangent = Spline::BesselDerivative(&pHeadPointFlat, pArray->GetAt(0), pArray->GetAt(1), 1);
	}
	else
		vHeadTangent = Spline::BesselDerivative(pArray->GetAt(0), pArray->GetAt(1), pArray->GetAt(2), 0);

	/*
	* 是否指定了尾点
	*/
	SizeT n = pArray->GetCount();
	if (NULL != pTailPoint)
	{
		Coordinate pTailPointFlat(pTailPoint->x, pTailPoint->y, 0);
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 2), pArray->GetAt(n - 1), &pTailPointFlat, 1);
	}
	else
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 3), pArray->GetAt(n - 2), pArray->GetAt(n - 1), 2);

	/*
	*	插值
	*/
	pSpline->Interpolate3Degree(pArray, vHeadTangent, vTailTangent);

	for (SizeT i = 0; i < pArray->GetCount();i++)
	{
		Coordinate* pCoor = pArray->ElementAt(i);
		delete pCoor;
	}
	delete pArray;
	pArray = NULL;
	

	return true;
}

Bool SplineInterpolateUtility::InterpolateSlopeSpline(Spline* pSpline, const Base::Array<Coordinate*>* pCoorArray, const Coordinate* pHeadPoint, const Coordinate* pTailPoint)
{
	/* CHECK DATA */
	if (NULL == pSpline)
		return false;

	if (NULL == pCoorArray)
		return false;

	/*
	* 暂时不支持 3点下的曲线内插，2016.6.13
	*/
	if (pCoorArray->GetCount() < 3)
		return false;

	//清楚spline 系数
	pSpline->Clear();

	/*
	*	Convert point to  Coordinate(sum distance(x,y) , z, 0)
	*/
	Array<Coordinate*> *pArray = new Array<Coordinate*>(pCoorArray->GetCount());

	(*pArray)[0] = (new Coordinate(0, pCoorArray->ElementAt(0)->z, 0)); //First Point

	Double xSum = 0;
	for (SizeT i = 1, n = pCoorArray->GetCount(); i < n; i++)
	{
		Double distance = BaseAlgorithm::DistancePtToPt(pCoorArray->ElementAt(i - 1), pCoorArray->ElementAt(i));
		xSum += distance;
		(*pArray)[i] = (new Coordinate(xSum, pCoorArray->ElementAt(i)->z, 0));
	}

	/*
	*	TANGENT
	*/

	Vector3d vHeadTangent, vTailTangent;

	if (pHeadPoint != NULL)
	{
		Double distance = BaseAlgorithm::DistancePtToPt(pHeadPoint, pArray->ElementAt(0));
		Coordinate c(-distance, pHeadPoint->z, 0);
		vHeadTangent = Spline::BesselDerivative(&c, pArray->GetAt(0), pArray->GetAt(1), 1);
	}
	else
		vHeadTangent = Spline::BesselDerivative(pArray->GetAt(0), pArray->GetAt(1), pArray->GetAt(2), 0);

	const SizeT n = pArray->GetCount();
	if (pTailPoint != NULL)
	{
		Double distance = BaseAlgorithm::DistancePtToPt(pTailPoint, pArray->ElementAt(n - 1));
		Coordinate c(pArray->ElementAt(n - 1)->x + distance, pTailPoint->z, 0);
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 2), pArray->GetAt(n - 1), &c, 1);
	}
	else
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 3), pArray->GetAt(n - 2), pArray->GetAt(n - 1), 2);

	/*
	*	插值
	*/
	pSpline->Interpolate3Degree(pArray, vHeadTangent, vTailTangent);
	return true;
}
