#include "Geometries/SCHAlgorithm/KPICaculator.h"
#include "Geometries/LineString.h"
#include "Geometries/Spline.h"
#include "Geometries/BaseAlgorithm.h"
#include "Base/Math.h"
#include "Geometries/SCHAlgorithm/KPIAlgorithm.h"
#include "Geometries/SCHAlgorithm/KPICaculatorByLeastSquareCicle.h"

#define BASE_ELAVATE_SCALE 0.1

using namespace Engine::Base;
using namespace Engine::Geometries;

KPICacualtor::KPICacualtor():
m_pLineString(NULL),
m_pHeadPoint(NULL),
m_pTailPoint(NULL),
m_dThreshold(NULL)
{

}

KPICacualtor::~KPICacualtor()
{
	if (NULL != m_dThreshold)
	{
		delete m_dThreshold;
		m_dThreshold = NULL;
	}
		
}

Void KPICacualtor::SetData(const LineString* pLineString, const Coordinate* pHeadPoint, const Coordinate* pTailPoint)
{
	m_pLineString = pLineString;
	m_pHeadPoint = pHeadPoint;
	m_pTailPoint = pTailPoint;	
}

Void KPICacualtor::SetThreshold(Double threshold)
{
	m_dThreshold = new Base::Double(threshold);
}

Void KPICacualtor::AdjointCurve(KPIType eType, Double maxOffset, Double& maxKpi, Array<KPINode>& vecKPINodes) const
{
	if (NULL == m_pLineString)
		return ;

	SizeT nPoints = m_pLineString->GetNumPoints();
	if (2 == nPoints)
	{
		Double dDummyScale = 0;
		AdjointCurveLineSegment(eType, maxOffset, maxKpi, false, dDummyScale,vecKPINodes);
		return;
	}

	switch (eType)
	{
	case Engine::Geometries::Azimuth:
		AdjointAzimuthCurve(maxOffset, false, NULL, &maxKpi, vecKPINodes);
		break;
	case Engine::Geometries::Curvature:
		AdjointCurvatureCurve(maxOffset, false, NULL, &maxKpi, vecKPINodes);
		break;
	case Engine::Geometries::CrossSlope:
		break;
	case Engine::Geometries::Slope:
		AdjointSlopeCurve(maxOffset, false, NULL, &maxKpi, vecKPINodes);
		break;
	default:
		break;
	}
}

Void KPICacualtor::AdjointCurveFixedScale(KPIType eType, Double maxOffset, Double scale, Array<KPINode>& vecKPINodes) const
{
	if (NULL == m_pLineString)
		return;
	
	SizeT nPoints = m_pLineString->GetNumPoints();
	if (2 == nPoints)
	{
		Double dDummy = 0;
		AdjointCurveLineSegment(eType, maxOffset,dDummy,true, scale, vecKPINodes);
		return;
	}
	

	switch (eType)
	{
	case Engine::Geometries::Azimuth:
		AdjointAzimuthCurve(maxOffset, true, &scale, NULL, vecKPINodes);
		break;
	case Engine::Geometries::Curvature:
		AdjointCurvatureCurve(maxOffset, true, &scale, NULL, vecKPINodes);
		break;
	case Engine::Geometries::CrossSlope:
		break;
	case Engine::Geometries::Slope:
		AdjointSlopeCurve(maxOffset, true, &scale, NULL, vecKPINodes);
		break;
	default:
		break;
	}
}

LineString* KPICacualtor::GetSplineTesselation(Double tolerance) const 
{
	if (NULL == m_pLineString || m_pLineString->GetNumPoints() < 3) //��ʱ��֧��2����
		return NULL; 

	Spline bs;

	Array<Coordinate*>* pArray = m_pLineString->GetCoordinates();
	
	Vector3d vHeadTangent, vTailTangent;

	/*�׽ڵ�*/
	if (NULL != m_pHeadPoint)
		vHeadTangent = Spline::BesselDerivative(m_pHeadPoint, pArray->GetAt(0), pArray->GetAt(1), 1);
	else
		vHeadTangent = Spline::BesselDerivative(pArray->GetAt(0), pArray->GetAt(1), pArray->GetAt(2), 0);

	/*
	* β��
	*/
	SizeT n = pArray->GetCount();
	if (NULL != m_pTailPoint)
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 2), pArray->GetAt(n - 1), m_pTailPoint, 1);
	else
		vTailTangent = Spline::BesselDerivative(pArray->GetAt(n - 3), pArray->GetAt(n - 2), pArray->GetAt(n - 1), 2);

	/*
	*	��ֵ
	*/
	bs.Interpolate3Degree(pArray, vHeadTangent, vTailTangent);

	return bs.Tesselation(tolerance);
}

Void KPICacualtor::AdjointCurvatureCurve(Double maxOffset, Bool fixedScale, Double* scale, 
	Double* maxCurvature, Array<KPINode>& vecKPINodes) const
{
	if (NULL == m_pLineString)
		return;
	
	/**��άƽ�����ڲ�����*/
	Spline bs;
	if (!SplineInterpolateUtility::InterpolateFlatZSpline(&bs,m_pLineString->GetCoordinates(),m_pHeadPoint,m_pTailPoint))
		return;

	/*һ�׵���*/
	Spline bsFirstDeriv = bs.GetFirstDeriv();
	/**���׵���*/
	Spline bsSecondDeriv = bsFirstDeriv.GetFirstDeriv();

	const UInt16 p = bs.Degree();
	const Array<Double> vecKnots = bs.GetKnotsVector();

	Array<Coordinate> vecLocationPoint(vecKnots.GetCount() - 2*p);
	Array<Double> vecCurvature(vecKnots.GetCount() - 2 * p);
	Array<Vector3d> vecNormal(vecKnots.GetCount() - 2 * p);

	Double dMaxC = 0; //������maxcurvature 

	CurvatureByLeastSquareCicle caculator;
	caculator.SetData(m_pLineString);
	std::vector<Double> vecCurvatureFromLestSquare;
	Bool bSuccess = caculator.SCHAtVertex(vecCurvatureFromLestSquare);

	for (SizeT i = p, n = vecKnots.GetCount() - p; i < n;i++)
	{
		const Double d = vecKnots[i];
		Coordinate ptBase;
		bs.Evaluate(d, ptBase);
		Vector3d vFirstDeriv;
		bsFirstDeriv.Evaluate(d, vFirstDeriv);
	
		double dCurvate = vecCurvatureFromLestSquare[i-p];
		if (Math::Fabs(dCurvate) > dMaxC)
			dMaxC = dCurvate;

		Vector3d v(-vFirstDeriv.y, vFirstDeriv.x, 0); //�� vFirstDeriv ��ֱ ���������ĵ�λ����
		v.Normalize();

		vecLocationPoint[i-p] = ptBase;
		vecCurvature[i-p] = dCurvate;
		vecNormal[i-p] = v;
	}

	/*
	*���� scale
	*/
	Double dScaleInternal = 0;
	if (fixedScale)
	{
		dScaleInternal = *scale;
	}else
	{
		dScaleInternal = Math::Equal(0,dMaxC) ? 0.0 : maxOffset / dMaxC;
		*maxCurvature = dMaxC;
	}

	/*
	*	��������Ϊ�ڻ����� �ط�����ƫ�� curvature * scale ����
	*/
	vecKPINodes.SetSize(vecLocationPoint.GetCount());
	for (SizeT i = 0; i < vecLocationPoint.GetCount(); i++)
	{
		KPINode node;
		node.flag = 0x00;
		node.index = i;
		node.value = vecCurvature[i];

		//������ֵ
		if (NULL != m_dThreshold && std::abs(node.value) > *m_dThreshold)
			node.flag |= 0x04;

		Coordinate pt = vecLocationPoint[i] + vecNormal[i] * (vecCurvature[i] * dScaleInternal + maxOffset*BASE_ELAVATE_SCALE);
		node.point = pt;

		vecKPINodes[i] = node;
	}
}

Void KPICacualtor::AdjointAzimuthCurve(Double maxOffset, Bool fixedScale, Double* scale, 
	Double* maxAzimuth, Array<KPINode>& vecKPINodes) const
{
	/**��άƽ�����ڲ�����*/
	Spline bs;
	if (!SplineInterpolateUtility::InterpolateFlatZSpline(&bs,m_pLineString->GetCoordinates(),m_pHeadPoint,m_pTailPoint))
		return;

	const UInt16 p = bs.Degree();

	/*һ�׵���*/
	Spline bsFirstDeriv = bs.GetFirstDeriv();

	const Array<Double> vecKnots = bs.GetKnotsVector();

	/*
	 *	make sure who is the bigest azimuth
	 */
	Double dScaleInternal = 0;
	const Coordinate cOrigin(0, 0, 0);

	if (fixedScale)
		dScaleInternal = *scale;
	else
	{
		Double dMaxAzimuth = 0;
				
		for (SizeT i = p, n = vecKnots.GetCount() - p; i < n; i++)
		{
			const Double d = vecKnots[i];
			Vector3d vFirstDeriv;
			bsFirstDeriv.Evaluate(d, vFirstDeriv);

			Double dAzimuth = BaseAlgorithm::CalAzimuth(&cOrigin, &vFirstDeriv);

			if (dAzimuth > dMaxAzimuth)
				dMaxAzimuth = dAzimuth;
		}

		dScaleInternal = Math::Equal(dMaxAzimuth,0) ? 0 : maxOffset / dMaxAzimuth;
		*maxAzimuth = dMaxAzimuth;
	}

	vecKPINodes.Clear();
	for (SizeT i = p, n = vecKnots.GetCount() - p-1; i < n; i++)
	{
		Double d = vecKnots[i];
		const Double dNext = vecKnots[i + 1];
		Bool bKnots = true;
		while (d < dNext)
		{
			Coordinate ptBase;
			bs.Evaluate(d, ptBase);

			Vector3d vFirstDeriv;
			bsFirstDeriv.Evaluate(d, vFirstDeriv);

			Double dAzimuth = BaseAlgorithm::CalAzimuth(&cOrigin, &vFirstDeriv);

			Vector3d v(-vFirstDeriv.y, vFirstDeriv.x, 0); //�� vFirstDeriv ��ֱ ���������ĵ�λ����
			v.Normalize();

			Coordinate pt = ptBase + v * (dAzimuth * dScaleInternal + maxOffset*BASE_ELAVATE_SCALE);

			KPINode node;
			node.flag = bKnots ? 0x00 : 0x01;
			node.index = bKnots ? i-p : 9999;
			node.value = dAzimuth;
			node.point = pt;
			//��ֵ
			if (m_dThreshold != NULL && node.value > *m_dThreshold)
				node.flag |= 0x04;

			vecKPINodes.Add(node); 
			//////////////////////////////////////////////////////////////////////////
			
			if (bKnots)
				bKnots = false;
			d += maxOffset / 20; //STEP
		}		 
	}

	//last point
	Double dLastKnot = vecKnots[vecKnots.GetCount() - p-1];
	Coordinate ptBase;
	bs.Evaluate(dLastKnot, ptBase);

	Vector3d vFirstDeriv;
	bsFirstDeriv.Evaluate(dLastKnot, vFirstDeriv);

	Double dAzimuth = BaseAlgorithm::CalAzimuth(&cOrigin, &vFirstDeriv);

	Vector3d v(-vFirstDeriv.y, vFirstDeriv.x, 0); //�� vFirstDeriv ��ֱ ���������ĵ�λ����
	v.Normalize();

	Coordinate pt = ptBase + v * dAzimuth * dScaleInternal;

	KPINode node;
	node.flag = 0x00; //knot
	node.value = dAzimuth;
	node.index = vecKnots.GetCount() - 2*p -1;
	node.point = pt;
	//��ֵ
	if (m_dThreshold != NULL && node.value > *m_dThreshold)
		node.flag |= 0x04;

	vecKPINodes.Add(node);

	//last point end

}

Void KPICacualtor::AdjointSlopeCurve(Double maxOffSet, Bool fixedScale, Double* scale, Double* maxSlope, Array<KPINode>& vecKPINodes) const
{
	Spline bs;
	
	/**�ڲ��¶�����*/
	if (!SplineInterpolateUtility::InterpolateSlopeSpline(&bs,m_pLineString->GetCoordinates(),m_pHeadPoint,m_pTailPoint))
		return;

	const UInt16 p = bs.Degree();

	/*һ�׵���*/
	Spline bsFirstDeriv = bs.GetFirstDeriv();

	const Array<Double> vecKnots = bs.GetKnotsVector();

	/*
	*	make sure who is the bigest azimuth
	*/
	Double dScaleInternal = 0;
	const Coordinate cOrigin(0, 0, 0);

	if (fixedScale)
		dScaleInternal = *scale;
	else
	{
		Double dMaxSlope = 0;

		for (SizeT i = p, n = vecKnots.GetCount() - p; i < n; i++)
		{
			const Double d = vecKnots[i];
			Vector3d vFirstDeriv;
			bsFirstDeriv.Evaluate(d, vFirstDeriv);
			vFirstDeriv.Normalize();
			
			Double dSlope = Math::ACos(vFirstDeriv.x);
			if (dSlope > dMaxSlope)
				dMaxSlope = dSlope;
		}

		if (Math::Equal(dMaxSlope,0))
			dScaleInternal = 1;
		else
			dScaleInternal = maxOffSet / dMaxSlope;
	
		*maxSlope = dMaxSlope;
	}


	/*
	 *	ƫ������
	 */
	Array<Coordinate*> *pArray = m_pLineString->GetCoordinates();

	vecKPINodes.Clear();
	for (SizeT i = p, n = vecKnots.GetCount() - p - 1; i < n; i++)
	{
		Double d = vecKnots[i];
		const Double dNext = vecKnots[i + 1];
		Coordinate start, end;
		bs.Evaluate(d, start);
		bs.Evaluate(dNext, end);

		const Coordinate* pStart = pArray->ElementAt(i - p);
		const Coordinate* pEnd = pArray->ElementAt(i + 1 - p);

		Vector3d v = *pEnd - *pStart;
		v.Normalize();
		v = Vector3d(-v.y, v.x, 0);

		Bool bKnots = true;
		while (d < dNext)
		{
			Coordinate c;
			bs.Evaluate(d, c);

			Double t = (c.x - start.x) / (end.x - start.x);
			Coordinate ptBase = (*pStart) * (1 - t) + (*pEnd) * t;

			Vector3d vFirstDeriv;
			bsFirstDeriv.Evaluate(d, vFirstDeriv);
			vFirstDeriv.Normalize();
			
			Double dSlope = Math::ACos(vFirstDeriv.x);
			if (vFirstDeriv.y < 0) //yֵ����������
				dSlope = -dSlope;
			
			Coordinate pt = ptBase + v * dSlope * dScaleInternal;

			KPINode node;
			node.flag = bKnots ? 0x00 : 0x01;
			node.index = bKnots ? i - p : 9999;
			node.value = dSlope;
			node.point = pt;
			//��ֵ
			if (m_dThreshold != NULL && node.value > *m_dThreshold)
				node.flag |= 0x04;

			vecKPINodes.Add(node);
			//////////////////////////////////////////////////////////////////////////

			if (bKnots)
				bKnots = false;
			d += maxOffSet / 10.0; //STEP
		}
	}

	//last point
	Double dLastKnot = vecKnots[vecKnots.GetCount() - p - 1];
	Coordinate ptBase = *(pArray->ElementAt(pArray->GetCount() - 1));
	Coordinate ptBasePrev = *(pArray->ElementAt(pArray->GetCount() - 2));

	Vector3d v = ptBase - ptBasePrev;
	v.Normalize();
	v = Vector3d(-v.y, v.x, 0);
	
	Vector3d vFirstDeriv;
	bsFirstDeriv.Evaluate(dLastKnot, vFirstDeriv);
	vFirstDeriv.Normalize();
	
	Double dSlope = Math::ACos(vFirstDeriv.x);
	if (vFirstDeriv.y < 0) //yֵ����������
		dSlope = -dSlope;

	Coordinate pt = ptBase + v * dSlope * dScaleInternal;

	KPINode node;
	node.flag = 0x00; //knot
	node.value = dSlope;
	node.index = vecKnots.GetCount() - 2 * p - 1;
	node.point = pt;
	//��ֵ
	if (m_dThreshold != NULL && node.value > *m_dThreshold)
		node.flag |= 0x04;

	vecKPINodes.Add(node);
	//last point end
}

//////////////////////////////////////////////////////////////////////////
/*
* PositionKPICaculator
*/
const Double PositionKPICacualtor::c_POSITION_INTERVEL = 1.0; //��ά�����ϣ�ÿ��һ��һ����ȡposition 

PositionKPICacualtor::PositionKPI::PositionKPI()
{
	Pos = Coordinate(0, 0, 0);
	dAzimuth = 0;
	dCurvature = 0;
	dSlope = 0;
	dCrossSlope = 0;
	nSeqNum = 0;
}

Double PositionKPICacualtor::GetParamByDistance(const Spline* pSpline,UInt32 startIndex, Double distance) const
{
	const Array<Double>& vecKnots = pSpline->GetKnotsVector();
	const UInt16 p = pSpline->Degree();

	const Coordinate ptStart = *(m_pLineString->GetCoordinateN(startIndex));
	const Coordinate ptEnd = *(m_pLineString->GetCoordinateN(startIndex + 1));
	Vector3d vDirection = ptEnd - ptStart;
	vDirection.Normalize();
	
	Coordinate ptOnLine = ptStart + vDirection*distance;
	
	Double dLowerParam = vecKnots.ElementAt(p + startIndex);
	Double dUpperParam = vecKnots.ElementAt(p + startIndex+1);// distance + dLowerParam;

	Double dTolerance = 1e-3;
	
	/*
	 *	��ʼ��ֵ
	 */
	Double dParam = dLowerParam + (dUpperParam - dLowerParam) / 2;//dUpperParam;
	Coordinate ptOnCurve;
	pSpline->Evaluate(dParam, ptOnCurve);

	while (dUpperParam - dLowerParam > 1e-6)
	{
		if (ptOnCurve.Distance(ptOnLine) <= dTolerance)
			break;
		
		if ((ptOnCurve - ptOnLine)*vDirection >= 0)
		{
			dUpperParam = dParam;
		}
		else
		{
			dLowerParam = dParam;
		}

		dParam = dLowerParam + (dUpperParam - dLowerParam) / 2.0;
		pSpline->Evaluate(dParam, ptOnCurve);
	}

	/*
	* �����Ѿ��õ��������
	*/
	return dParam;
}

PositionKPICacualtor::PositionKPICacualtor():
m_pLineString(NULL),
m_pHeadPoint(NULL),
m_pTailPoint(NULL)
{
	
}

PositionKPICacualtor::~PositionKPICacualtor()
{

}

void PositionKPICacualtor::SetData(const LineString* pLineString, const Coordinate* pHeadPoint, const Coordinate* pTailPoint)
{
	m_pLineString = pLineString;
	m_pHeadPoint = pHeadPoint;
	m_pTailPoint = pTailPoint;
}

Bool PositionKPICacualtor::CaculateKPI(Array<PositionKPI>& vecPosKPIs) const
{
	/* 1. ���� position �� */
	Array<InternalPositionNode> vecPositions;
	GeneratePositionNode(vecPositions);
	if (vecPositions.IsEmpty())
		return false;

	/*2.�� Internal node ת���� �ⲿKPI NODE*/
	CaculateSCHbyLeastSquareFit(vecPositions);

	/*3.�� Internal node ת���� �ⲿKPI NODE*/
	vecPosKPIs.SetSize(vecPositions.GetCount());
	for (SizeT i = 0, n = vecPositions.GetCount(); i < n; i++)
	{
		vecPosKPIs.SetAt(i, vecPositions.ElementAt(i));
	}

	return true;
}

//�����¶��ǲ��������ڲ巽������
//Bool PositionKPICacualtor:: CaculateKPI(Array<PositionKPI>& vecPosKPIs) const
//{
//	/* 1. ���� position �� */
//	Array<InternalPositionNode> vecPositions;
//	GeneratePositionNode(vecPositions);
//	if (m_pLineString->GetNumPoints() == 2)
//	{
//		Double slope, heading, curvature;
//		Double a = m_pLineString->GetCoordinateN(1)->x - m_pLineString->GetCoordinateN(0)->x;
//		Double b = m_pLineString->GetCoordinateN(1)->y - m_pLineString->GetCoordinateN(0)->y;
//		Double c = m_pLineString->GetCoordinateN(1)->z - m_pLineString->GetCoordinateN(0)->z;
//		slope = atan(c / Base::Math::Sqrt(a*a + b*b ));
//		slope = slope * 180 / PI;
//		heading = BaseAlgorithm::CalAzimuth(m_pLineString->GetCoordinateN(0), m_pLineString->GetCoordinateN(1));
//		heading = heading * 180 / PI;
//		curvature = 0;
//		for (size_t i = 0; i < vecPositions.GetCount(); i++)
//		{
//			vecPositions[i].dSlope = slope;
//			vecPositions[i].dAzimuth = heading;
//			vecPositions[i].dCurvature = curvature;
//		}
//		vecPosKPIs.SetSize(vecPositions.GetCount());
//		for (SizeT i = 0, n = vecPositions.GetCount(); i < n; i++)
//		{
//			vecPosKPIs.SetAt(i, vecPositions.ElementAt(i));
//
//		}
//		return true;
//	}
//	/*2.�ڲ�����ƽ�棨zֵ��Ϊ0������*/
//	/*5.�ڲ����� XY - Z ����*/
//	Spline* pSpLine = new Spline;
//	Spline* pSlopeSpline = new Spline;
//	Bool bXYInterpolateSuccess = SplineInterpolateUtility::InterpolateFlatZSpline(pSpLine, m_pLineString->GetCoordinates(), m_pHeadPoint, m_pTailPoint);
//	Bool bZInterpolateSucess = SplineInterpolateUtility::InterpolateSlopeSpline(pSlopeSpline, m_pLineString->GetCoordinates(), m_pHeadPoint, m_pTailPoint);
//	if (bXYInterpolateSuccess)
//	{
//		/*3. �ҵ�ƽ�棨z��Ϊ0����position ��Ӧ�� parameter*/
//		SolveParameterValue_XY(pSpLine,vecPositions);
//
//		/*4.���㺽������*/
//		CaculateAzimuth(pSpLine,vecPositions);
//	//	CaculateCurvature(pSpLine, vecPositions);
//		CaculateCurvature(vecPositions);
//
//		delete pSpLine;
//		pSpLine = NULL;
//	}
//
//	if (bZInterpolateSucess)
//	{
//		/*6.���� arclength - z �ռ��� position ��Ӧ��parameter*/
//		SolveParameterValue_Z(pSlopeSpline,vecPositions);
//
//		/*7.���� ����*/
//		CaculateSlope(pSlopeSpline,vecPositions);
//
//		delete pSlopeSpline;
//		pSlopeSpline = NULL;	
//	}
//
//	//TODO : �������
//	/*8.�� Internal node ת���� �ⲿKPI NODE*/
//	vecPosKPIs.SetSize(vecPositions.GetCount());
//	for (SizeT i = 0, n = vecPositions.GetCount(); i < n;i++)
//	{
//		vecPosKPIs.SetAt(i, vecPositions.ElementAt(i));
//	}
//
//	return bXYInterpolateSuccess&&bZInterpolateSucess;
//}

Void PositionKPICacualtor::GeneratePositionNode(Array<InternalPositionNode>& vecPositions) const
{
	Array<Coordinate*> *pCoorArray = m_pLineString->GetCoordinates();
	if (c_POSITION_INTERVEL <= 0 || pCoorArray->IsEmpty())
		return;

	SizeT nPoints = pCoorArray->GetCount();

	SizeT positionNumber;
	Double lineDistance; //���γ���
	Double distResidue = 0;  //��һ�ηֶκ�����µ�һ��
	Int32 nPointSeq = 0;

	//add first point
	{
		InternalPositionNode node;
		node.Pos = *(pCoorArray->GetAt(0));
		node.nIndex = 0;
		node.dDistance = 0;
		node.dXYDistance = 0;
		node.nSeqNum = nPointSeq++;
		vecPositions.Add(node);
	}

	for (SizeT i = 0; i < nPoints - 1; i++)
	{
		Coordinate& ptFirstPoint = *(pCoorArray->ElementAt(i));
		Vector3d vDelta = *(pCoorArray->ElementAt(i + 1)) - ptFirstPoint;
		Double dLength = vDelta.GetLength(); //�߶γ���

		if (Math::Equal(0, dLength))
			continue;

		lineDistance = dLength + distResidue;
		positionNumber = Base::Math::Floor(lineDistance / c_POSITION_INTERVEL);

		vDelta.Normalize();
		for (int j = 1; j <= positionNumber; j++)
		{
			Coordinate c = vDelta*(j *c_POSITION_INTERVEL - distResidue) + ptFirstPoint;

			InternalPositionNode node;
			node.Pos = c;
			node.nIndex = i;
			node.dDistance = c.Distance(ptFirstPoint);
			node.dXYDistance = c.DistanceXY(ptFirstPoint);
			node.nSeqNum = nPointSeq++;

			vecPositions.Add(node);
		}

		distResidue = lineDistance - positionNumber*c_POSITION_INTERVEL;
	}
}

//Void PositionKPICacualtor::GeneratePositionNode(Array<InternalPositionNode>& vecPositions) const
//{
//	SizeT nodNumber,positionNumber,totPosNumber;
//	Double lineDistance,distResidue;
//	Double offsetx,offsety,offsetz;
//	distResidue = 0;
//	totPosNumber = 0;
//	InternalPositionNode pnode;
//	nodNumber = m_pLineString->GetNumPoints();
//	for (SizeT i = 0; i < nodNumber-1;i++)
//	{
//		Double a, b, c, u,u0;   
//		a = m_pLineString->GetCoordinateN(i+1)->x - m_pLineString->GetCoordinateN(i)->x;
//		b = m_pLineString->GetCoordinateN(i+1)->y - m_pLineString->GetCoordinateN(i)->y;
//		c = m_pLineString->GetCoordinateN(i+1)->z - m_pLineString->GetCoordinateN(i)->z;
//		u = 0; u0 = 0;
//		lineDistance = Base::Math::Sqrt(a*a+b*b+c*c)+distResidue;
//		positionNumber = Base::Math::Floor(lineDistance / c_POSITION_INTERVEL);
//
//		if ((i==nodNumber-2)&&(positionNumber==lineDistance))
//		{
//			positionNumber = positionNumber - 1;
//		}
//		for (SizeT j = 0; j < positionNumber;j++)
//		{
//			if (j==0)
//			{
//				u0 = Base::Math::Sqrt( 1.0/ (a*a + b*b + c*c))*(c_POSITION_INTERVEL - distResidue);
//			}
//			else
//			{
//				u = Base::Math::Sqrt( 1.0/ (a*a + b*b + c*c))*c_POSITION_INTERVEL;
//			}
//			pnode.Pos.x = a*(u*j + u0) + m_pLineString->GetCoordinateN(i)->x;
//			pnode.Pos.y = b*(u*j + u0) + m_pLineString->GetCoordinateN(i)->y;
//			pnode.Pos.z = c*(u*j + u0) + m_pLineString->GetCoordinateN(i)->z;
//			pnode.nIndex = i;
//			offsetx = pnode.Pos.x - m_pLineString->GetCoordinateN(i)->x;
//			offsety = pnode.Pos.y - m_pLineString->GetCoordinateN(i)->y;
//			offsetz = pnode.Pos.z - m_pLineString->GetCoordinateN(i)->z;
//			pnode.dDistance = Base::Math::Sqrt(offsetx*offsetx + offsety*offsety + offsetz*offsetz);
//			pnode.dXYDistance = Base::Math::Sqrt(offsetx*offsetx + offsety*offsety);
//			pnode.nSeqNum = totPosNumber;
//			vecPositions.Add(pnode);
//			totPosNumber++;
//		}
//		distResidue = lineDistance - positionNumber*c_POSITION_INTERVEL;
//	}
//}

Void PositionKPICacualtor::SolveParameterValue_XY(const Spline* pSpline, Array<InternalPositionNode>& vecPositions) const
{
	for (int i = 0; i < vecPositions.GetCount();i++)
	{
		vecPositions[i].dXYParameter = GetParamByDistance(pSpline, vecPositions[i].nIndex, vecPositions[i].dXYDistance);
	}
}

Void PositionKPICacualtor::SolveParameterValue_Z(const Spline* pSpline,Array<InternalPositionNode>& vecPositions) const
{
	for (int i = 0; i < vecPositions.GetCount(); i++)
	{
		vecPositions[i].dZParameter = GetParamByDistance(pSpline, vecPositions[i].nIndex, vecPositions[i].dDistance);
	}
}

Void PositionKPICacualtor::CaculateAzimuth(const Spline* pSpline,Array<InternalPositionNode>& vecPositions) const
{
	Spline splineDeriv=pSpline->GetFirstDeriv();
	Coordinate nodeZero(0,0,0);
	Coordinate splDeriv ;
	Double heading;
	for (size_t i = 0; i < vecPositions.GetCount(); i++)
	{
		splineDeriv.Evaluate(vecPositions[i].dXYParameter,splDeriv);
		heading=BaseAlgorithm::CalAzimuth(&nodeZero, &splDeriv);
		heading = heading * 180 / PI;
		vecPositions[i].dAzimuth = heading;
	}
}

Void PositionKPICacualtor::CaculateSlope(const Spline* pSpline, Array<InternalPositionNode>& vecPositions) const
{
	Spline splineDeriv = pSpline->GetFirstDeriv();
	Double slope;
	for (size_t i = 0; i < vecPositions.GetCount(); i++)
	{
		Coordinate splDeriv;
		splineDeriv.Evaluate(vecPositions[i].dZParameter, splDeriv);
		slope = atan(splDeriv.y / splDeriv.x);
		slope = slope * 180 / PI;
		vecPositions[i].dSlope = slope;
	}
	
}

	Void PositionKPICacualtor::CaculateCurvature(const Spline* pSpline, Array<InternalPositionNode>& vecPositions) const
	{
		Spline firstDeriv = pSpline->GetFirstDeriv();
		Spline secDeriv = firstDeriv.GetFirstDeriv();
		Coordinate vFirst,vSec,cross;
		Double curvature;
		for (SizeT i = 0; i < vecPositions.GetCount();i++)
		{
			firstDeriv.Evaluate(vecPositions[i].dXYParameter, vFirst);
			secDeriv.Evaluate(vecPositions[i].dXYParameter, vSec);
			cross = vFirst.CrossProduct(vSec);
			curvature = cross.GetLength() / (vFirst.GetLength()*vFirst.GetLength()*vFirst.GetLength());
			if (cross.z<0)
			{
				curvature = curvature*-1;
			}
			vecPositions[i].dCurvature = curvature;
		}
	}

	Void PositionKPICacualtor::CaculateSCHbyLeastSquareFit(Array<InternalPositionNode>& vecPositions) const
	{
		SizeT nNodes = vecPositions.GetCount();

		if (nNodes <= 0)
			return;

		InternalPositionNode& frontNode = vecPositions[0];

		std::vector<NodeInfo> vecNodes(nNodes);
		for (Int32 i = 0; i < nNodes; i++)
		{
			InternalPositionNode& nodePosition = vecPositions[i];
			NodeInfo node;
			node.setOrigin_X(nodePosition.Pos.x - frontNode.Pos.x);
			node.setOrigin_Y(nodePosition.Pos.y - frontNode.Pos.y);
			node.setWgs_Z(nodePosition.Pos.z * 100.0);
			vecNodes[i] = node;
		}

		CurvatureByLeastSquareCicle caculator;
		caculator.SCHAtPosition(vecNodes);

		for (Int32 i = 0; i < nNodes; i++)
		{
			NodeInfo& node = vecNodes[i];
			InternalPositionNode& nodePosition = vecPositions[i];
			nodePosition.dCurvature = node.getCurvature();
			nodePosition.dAzimuth = node.getHeading();
			nodePosition.dSlope = node.getSlope();
		}
	}

	Void KPICacualtor::AdjointCurveLineSegment(KPIType eType, Double maxOffset, Double& maxKpi, Bool bFixedScale ,Double dScaleExternal,Array<KPINode>& vecKPINodes) const 
	{
		Coordinate pFirstCoor =  *(m_pLineString->GetCoordinateN(0));
		Coordinate pSecondCoor = *(m_pLineString->GetCoordinateN(1));

		Vector3d vDirect = pSecondCoor - pFirstCoor;
		Vector3d vNormal(-vDirect.y, vDirect.x, 0);
		vNormal.Normalize();

		//����KPI
		Double dValue0 = 0;
		Double dValue1 = 0;

		Double dScale = 0;

		if (eType == Curvature)
		{
			maxKpi = 0;
			dScale = 0;
			dValue0 = 0;
			dValue1 = 0;
		}
		else if (eType == Slope)
		{
			Double dDeltaXY = std::sqrt(vDirect.x*vDirect.x + vDirect.y*vDirect.y);
			Double dDeltZ = vDirect.z;

			dValue0 = std::atan2(dDeltZ, dDeltaXY);
			dValue1 = dValue0;

			maxKpi = dValue0;

			if (Math::Equal(0, dValue0))
				dScale = 0;
			else
				dScale = maxOffset / std::abs(dValue0);
		}

		if (bFixedScale)
			dScale = dScaleExternal;

		KPINode node0;
		node0.flag = 0x00;
		node0.index = 0;
		node0.point = pFirstCoor + vNormal*(dScale * dValue0 + maxOffset*BASE_ELAVATE_SCALE);
		node0.value = dValue0;
		vecKPINodes.Add(node0);

		KPINode node1;
		node1.flag = 0x00;
		node1.index = 1;
		node1.point = pSecondCoor + vNormal*(dScale*dValue1 + maxOffset*BASE_ELAVATE_SCALE);
		node1.value = dValue1;
		vecKPINodes.Add(node1);
	}
