#include "Geometries/Spline.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/GeometryAlgorithm.h"

#include "Eigen/Dense"

#pragma warning(disable:4996)

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

Spline::Spline(UInt16 degree, const Array<Double>& vecKnots, const Array<Coordinate>& vecControlVertexes):
m_nDegree(degree),
m_vecKnots(vecKnots),
m_vecControlVertexes(vecControlVertexes)
{

}

Spline::Spline(const Spline &rhs):
m_nDegree(rhs.m_nDegree),
m_vecKnots(rhs.m_vecKnots),
m_vecControlVertexes(rhs.m_vecControlVertexes)
{

}


Spline::Spline():
m_nDegree(0)
{

}

Spline::~Spline()
{
	Clear();
}

GeometryType Spline::GetGeometryType() const
{
	return GeometryType::SPLINE;
}

Geometry* const Spline::Clone() const
{
	Spline* pSpline = new Spline(*this);
	return pSpline;
}

Envelope* const Spline::GetEnvelope() const
{
	if (m_vecControlVertexes.IsEmpty())
		return new Envelope();

	Coordinate c = m_vecControlVertexes.GetAt(0);
	Envelope* pEnv = new Envelope(c, c);

	typedef Array<Coordinate>::_Const_Iter const_iter;

	for (const_iter _it = m_vecControlVertexes.Begin();
		_it != m_vecControlVertexes.End();
		_it++)
	{
		c = *_it;
		Envelope envTemp(c, c);
		pEnv->ExpandToInclude(&envTemp);
	}
	return pEnv;
}

Void Spline::Clear()
{
	m_nDegree = 0;
	m_vecControlVertexes.Clear();
	m_vecKnots.Clear();
}

Bool Spline::GetFirstDeriv(Double param, Vector3d& firstDeriv) const
{
	Spline bs = this->GetFirstDeriv();
	return bs.Evaluate(param, firstDeriv);
}

Spline Spline::GetFirstDeriv() const
{
	const UInt16 p = m_nDegree;

	/*
	 *	b ������ ���� �� ���� ��һ��b��������
	 ��ڵ�ʸ����β��ɾ��һ��
	 */
	Spline bsDeriv;
	bsDeriv.m_nDegree = p - 1;
	bsDeriv.m_vecKnots = m_vecKnots;
	bsDeriv.m_vecKnots.Delete(bsDeriv.m_vecKnots.GetCount() - 1);
	bsDeriv.m_vecKnots.Delete(0);

	for (SizeT i = 0, n = m_vecControlVertexes.GetCount() - 1; i < n; i++)
	{
		Coordinate ptCV = (m_vecControlVertexes[i + 1] - m_vecControlVertexes[i]) * p / (m_vecKnots[i + p + 1] - m_vecKnots[i + 1]);
		bsDeriv.m_vecControlVertexes.Add(ptCV);
	}
	return bsDeriv;
}

Bool Spline::GetSecondDeriv(Double param, Vector3d& secondDeriv) const
{
	Spline bsFirstDeriv = this->GetFirstDeriv();
	return bsFirstDeriv.GetFirstDeriv(param,secondDeriv);
}

LineString* Spline::Tesselation(Double tolerance) const
{
	Array<Coordinate> vecCVs(m_vecControlVertexes);
	Array<Double> vecKnots(m_vecKnots);
	_Tesselation(0, vecCVs.GetCount() - 1, tolerance, vecCVs, vecKnots);
		
	Array<Coordinate*>* pCoorArray = new Array<Coordinate*>(vecCVs.GetCount());
	for (SizeT i = 0, n = vecCVs.GetCount(); i<n; i++)
	{
		pCoorArray->SetAt(i, new Coordinate(vecCVs[i]));
	}

	LineString* pLineString = new LineString(pCoorArray);
	return pLineString;
}

UInt32 Spline::_Tesselation(SizeT begin, SizeT end, Double tolerance,
	Array<Coordinate>& vecControlVertexes, Array<Double>& vecKnots) const
{
	/*
	 * �Ҹߵ����ݲ�ֹͣ��ɢ��
	 */
	if (ChordHeightLessThan(vecControlVertexes, begin, end, tolerance))
		return 0;

	const Base::UInt16 p = m_nDegree;

	//ȡparameter ���м�ֵ
	Base::Double dParam = (vecKnots[begin + 1] + vecKnots[end + 1]) / 2;

	Base::Int32 k = std::lower_bound(vecKnots.Begin() + begin, vecKnots.Begin() + end + 1, dParam) - vecKnots.Begin();
	Base::Int32 s = 0; //�ظ���
	while (vecKnots[k] == dParam)
	{
		k++;	s++;
	}
	k--;
	
	//////////////////////////////////////////////////////////////////////////
	// ��2*(p-s) +1 ����ԭ p-s�� ���Ƶ�
	const Base::Int32 nReplaceCVLength = 2 * (p - s) + 1;
	Coordinate* pReplaceCV = new Coordinate[nReplaceCVLength];
	for (Int32 i = k - p,j =0; i <= k - s;i++,j++)
	{
		pReplaceCV[j] = vecControlVertexes[i];
	}

	for (Int32 r = 1; r <= p - s; r++)
	{
		pReplaceCV[nReplaceCVLength - r] = pReplaceCV[p - s];

		Coordinate* pTemp = new Coordinate[p - s + 1 - r];
		for (Int32 i = k - p + r, j = r; i <= k - s; i++, j++)
		{
			Base::Double dA = (dParam - vecKnots[i]) / (vecKnots[i + p - r + 1] - vecKnots[i]);
			Coordinate pt = pReplaceCV[j] * dA + pReplaceCV[j - 1] * (1 - dA);
			pTemp[j - r] = pt;
		}

		for (Base::UInt32 i = 0; i < p - s + 1 - r; i++)
		{
			pReplaceCV[i + r] = pTemp[i];
		}

		delete[]pTemp;
	}
	//////////////////////////////////////////////////////////////////////////
	SizeT nOldSize = vecControlVertexes.GetCount();
	vecControlVertexes.SetSize(nOldSize + p - s); //����p-s��
	for (SizeT i = nOldSize - 1; i > k - s; i--) //�����ƶ�p-s
	{
		vecControlVertexes[i + p - s] = vecControlVertexes[i];
	}
	std::copy(pReplaceCV, pReplaceCV + nReplaceCVLength, vecControlVertexes.Begin() + k - p); //�м��Ƕ��滻��
	vecKnots.InsertAt(k + 1, dParam,p - s);//����p-s���ڵ�ʹ�ظ��ȵ�p

	delete[] pReplaceCV;
	//////////////////////////////////////////////////////////////////////////
	//�ʷ�����
	Base::UInt32 nOffsetLeft = _Tesselation(begin, k - s, tolerance, vecControlVertexes, vecKnots);
	//�ʷ��Ұ��
	Base::UInt32 nOffsetRight = _Tesselation(k - s + nOffsetLeft, nOffsetLeft + end + p - s, tolerance, vecControlVertexes, vecKnots);

	return nOffsetLeft + nOffsetRight + p - s;
}

Bool Spline::ChordHeightLessThan(const Array<Coordinate>& vecCVs, UInt32 begin, UInt32 end, Double limit) const
{
	const Coordinate* pHeadPt = &vecCVs[begin];
	const Coordinate* pTailPt = &vecCVs[end];
	for (UInt32  i = begin + 1; i < end; i++)
	{
		const Coordinate* pt = &vecCVs[i];
		double dDis = BaseAlgorithm::DistancePtToLine(pt, pHeadPt, pTailPt);
		if (dDis > limit)
			return false;
	}
	return true;
}

Void Spline::Interpolate3Degree(const Array<Coordinate*>* pPoints, const Vector3d& headTangent, const Vector3d& tailTangent)
{
	const SizeT nPoints = pPoints->GetCount();
	const SizeT n = nPoints + 2; //��ֵ���ṩ nPoints ������ ��������β�����ʸ�ṩ��������(Endpoint Condition)
	const UInt16 p = 3; //�״�
	m_nDegree = p;

	/*
	*	knots ʸ�� 
	*/
	Double dParam = 0;
	m_vecKnots.Clear();

	for (UInt16 i = 0; i < p;i++)
	{
		m_vecKnots.Add(0); //ǰ p�� 0
	}

	for (UInt32 i = 0; i<nPoints; i++)
	{
		if (i>0)
			dParam += BaseAlgorithm::DistancePtToPt((*pPoints)[i], (*pPoints)[i - 1]);
		m_vecKnots.Add(dParam);
	}

	for (UInt16 i = 0; i < p; i++)
	{
		m_vecKnots.Add(dParam); //ǰ p�� 0
	}
	
	/*
	 *	��ֵ�㷽��
	 */
	Eigen::MatrixXd N = Eigen::MatrixXd::Zero(n, n);

	//b�������ߵĶ��� C(u) = Sum(Basis(i,p,u)*Pi ); i ����[0,n)
	Array<Double> vecBasis(p);
	for (UInt32 i = 1; i < nPoints - 1; i++)
	{
		BasisFuns(m_vecKnots[i + p], i + p, vecBasis);
		for (int j = 0; j < p; j++)
			N(i, i + j) = vecBasis[j];
	}

	N(0, 0) = 1.0; //��ĩ�������
	N(nPoints - 1, n - 1) = 1.0;

	/*
	*	�˵㵼��
	*/
	N(nPoints, 0) = -1;
	N(nPoints, 1) = 1;

	N(nPoints + 1, n - 2) = -1;
	N(nPoints + 1, n - 1) = 1;

	/*
	 *	N*P = D;
	 */
	Eigen::MatrixXd D(n, 3); //��֪��(��ֵ��+�˵㵼��)
	Eigen::MatrixXd P(n, 3); //���Ƶ�
	for (UInt32 i = 0; i < nPoints; i++)
	{
		const Coordinate* pt = (*pPoints)[i];
		D(i, 0) = pt->x;
		D(i, 1) = pt->y;
		D(i, 2) = pt->z;
	}

	/*
	*	����
	*/
	Coordinate p0 = headTangent * ((m_vecKnots[p + 1] - m_vecKnots[1]) / p);

	D(nPoints, 0) = p0.x;
	D(nPoints, 1) = p0.y;
	D(nPoints, 2) = p0.z;

	Coordinate p1 = tailTangent * ((m_vecKnots[n + p - 1] - m_vecKnots[n - 1]) / p);
	D(nPoints + 1, 0) = p1.x;
	D(nPoints + 1, 1) = p1.y;
	D(nPoints + 1, 2) = p1.z;

	//��P
	P = N.lu().solve(D);
	/*
	 *	���Ƶ�
	 */
	m_vecControlVertexes.SetSize(n);
	for (UInt32 i = 0; i < n; i++)
	{
		Coordinate pt;
		pt.x = P(i, 0);
		pt.y = P(i, 1);
		pt.z = P(i, 2);
		m_vecControlVertexes[i] = pt;
	}
}

Void Spline::BasisFuns(Double u, UInt32 nSpan, Array<Double>& vecBasis) const
{
	vecBasis.SetSize(m_nDegree + 1);

	Double* pLeft = new Double[2 * (m_nDegree + 1)];
	Double* pRight = pLeft + m_nDegree + 1;

	Double temp, saved;

	vecBasis[0] = 1.0;
	for (UInt32 j = 1; j <= m_nDegree; j++)
	{
		pLeft[j] = u - m_vecKnots[nSpan + 1 - j];
		pRight[j] = m_vecKnots[nSpan + j] - u;
		saved = 0.0;
		for (UInt32 r = 0; r < j; r++)
		{
			temp = vecBasis[r] / (pRight[r + 1] + pLeft[j - r]);
			vecBasis[r] = saved + pRight[r + 1] * temp;
			saved = pLeft[j - r] * temp;
		}
		vecBasis[j] = saved;
	}

	delete[]pLeft;
	pLeft = NULL;
	pRight = NULL;
}

Void Spline::Interpolate3Degree(const Array<Coordinate*>* pPoints)
{
	const SizeT nSize = pPoints->GetCount();
	if (nSize < 2)
		return;

	if (2 == nSize)
	{
		Vector3d vTangent = *(*pPoints)[1] - *(*pPoints)[0];
		Interpolate3Degree(pPoints, vTangent, vTangent);
	}	else
	{
		Vector3d vHeadTangent = BesselDerivative((*pPoints)[0], (*pPoints)[1], (*pPoints)[2], 0);
		Vector3d vTailTangent = BesselDerivative((*pPoints)[nSize - 3], (*pPoints)[nSize - 2], (*pPoints)[nSize - 1], 2);
		Interpolate3Degree(pPoints, vHeadTangent, vTailTangent);
	}
	
	return;
}

Vector3d Spline::BesselDerivative(const Coordinate* p0, const Coordinate* p1,const Coordinate* p2, UInt16 index)
{
	Vector3d vDerivative;
	Double det1 = Bessel_detT(p0, p1, p2, 1);
	Double det0 = Bessel_detT(p0, p1, p2, 0);

	Vector3d vTanMiddle = Bessel_detP(p0, p1, p2, 0) * ( det1 / (det1 + det0)) + Bessel_detP(p0, p1, p2, 1) * (det0 / (det1 + det0));

	switch (index)
	{
	case 0:
		vDerivative = Bessel_detP(p0, p1, p2, 0)*2.0 - vTanMiddle;
		break;
	case 1:
		vDerivative = vTanMiddle; 
		break;
	case 2:
		vDerivative = Bessel_detP(p0, p1, p2, 1)*2.0 - vTanMiddle;
		break;

	default:
		break;
	}

	return vDerivative;
}

Vector3d Spline::Bessel_detP(const Coordinate* p0, const Coordinate* p1, const Coordinate* p2, int nIndex)
{
	const Coordinate* pNext = NULL;
	const Coordinate* pCur = NULL;
	switch (nIndex)
	{
	case 0:
		pNext = p1;
		pCur = p0;
		break;

	case 1:
		pNext = p2;
		pCur = p1;
		break;

	default:
		break;
	}

	return (*pNext - *pCur) / Bessel_detT(p0, p1, p2, nIndex);
}

Double Spline::Bessel_detT(const Coordinate* p0, const Coordinate* p1, const Coordinate* p2, int nIndex)
{
	const Coordinate* pNext = NULL;
	const Coordinate* pCur = NULL;
	switch (nIndex)
	{
	case 0:
		pNext = p1;
		pCur = p0;
		break;

	case 1:
		pNext = p2;
		pCur = p1;
		break;

	default:
		break;
	}

	return BaseAlgorithm::DistancePtToPt(pNext ,pCur);
}

Bool Spline::Evaluate(Double param, Coordinate& point) const
{
	if (!IsParameterValid(param))
		return false;

	if (param == m_vecKnots.ElementAt(0))
	{
		point = m_vecControlVertexes.ElementAt(0);
		return true;
	}

	if (param == m_vecKnots.ElementAt(m_vecKnots.GetCount()-1))
	{
		point = m_vecControlVertexes.ElementAt(m_vecControlVertexes.GetCount() - 1);
		return true;
	}

	Int32 k = std::lower_bound(m_vecKnots.Begin(), m_vecKnots.End(), param) - m_vecKnots.Begin();
	UInt32 s = 0; //�ظ���
	const UInt16 p = m_nDegree;

	while (m_vecKnots[k] == param)
	{
		k++;	s++;
	}

	k--;
	//////////////////////////////////////////////////////////////////////////
	//������Ӱ��� [k-p,k-s+1) �� p-s+1 �����Ƶ�
	Coordinate* pCVArray = new Coordinate[p - s + 1];

	for (UInt32 i = k - p,j =0; i <= k - s; i++,j++)
	{
		pCVArray[j] = m_vecControlVertexes[i];
	}

	//һ������ p-s ��ʹ �ڵ��ظ��ȴﵽ p
	for (UInt32 r = 1; r <= p - s; r++)
	{
		Coordinate* pCVArrayTemp = new Coordinate[p - s + 1 - r];
		for (UInt32 i = k - p + r, j = r;
			i <= k - s;
			i++, j++)
		{
			double dA = (param - m_vecKnots[i]) / (m_vecKnots[i + p - r + 1] - m_vecKnots[i]);
			Coordinate ptNew = pCVArray[j] * dA + pCVArray[j - 1] * (1.0 - dA);
			pCVArrayTemp[j - r] = ptNew;
		}

		for (UInt32 i = 0; i < p - s + 1 - r; i++)
		{
			pCVArray[i + r] = pCVArrayTemp[i];
		}

		delete[] pCVArrayTemp;
	}

	point = pCVArray[p - s];
	delete[] pCVArray;

	return true;
}

Bool Spline::IsParameterValid(Double param) const
{
	Double d0 = m_vecKnots[0];
	Double d1 = m_vecKnots[m_vecKnots.GetCount() - 1];
	return  !(param > d1 || param < d0);
}

Void Spline::Curvature(const Array<Double>& params, Array<Double>& curvatures) const
{
	Spline bsFirstDeriv = GetFirstDeriv(); //һ�׵���
	Spline bsSecondDeriv = bsFirstDeriv.GetFirstDeriv(); //���׵���
	for (SizeT i = 0; i<params.GetCount(); i++)
	{
		const double d = params[i];
		Vector3d vFirstDeriv;
		bsFirstDeriv.Evaluate(d, vFirstDeriv);
		Vector3d vSecondDeriv;
		bsSecondDeriv.Evaluate(d, vSecondDeriv);

		Vector3d vCrossProduct = vFirstDeriv.CrossProduct(vSecondDeriv);
		Double dCurvate = vCrossProduct.GetLength() / std::pow(vFirstDeriv.GetLength(), 3);
		/*
		*	FOR 2D ONLY
		*/
		dCurvate = vCrossProduct.z > 0 ? dCurvate : -dCurvate;
		curvatures.Add(dCurvate);
	}
}

LineString* Spline::AdjointCurvatureCurve(Double maxOffset, Bool fixedScale, Double& maxCurvature) const
{
	Array<Coordinate*> *coordinates = new Array<Coordinate*>();
	const UInt16 p = m_nDegree;
	/*
	*	һ�� ���׵���
	*/
	Spline bsFirstDeriv = GetFirstDeriv();
	Spline bsSecondDeriv = bsFirstDeriv.GetFirstDeriv();

	Array<Double> vecCurvatures;
	Array<Coordinate> vecBasePoints;
	Array<Vector3d> vecNormalVector;
	Double dMaxAbsCurvature = 0;

	for (SizeT i = p, n = m_vecKnots.GetCount() - p; i < n; i++)
	{
		const Double d = m_vecKnots[i];
		Coordinate ptBase;
		Evaluate(d, ptBase);
		Vector3d vFirstDeriv;
		bsFirstDeriv.Evaluate(d, vFirstDeriv);
		Vector3d vSecondDeriv;
		bsSecondDeriv.Evaluate(d, vSecondDeriv);

		Vector3d vCrossProduct = vFirstDeriv.CrossProduct(vSecondDeriv);
		double dCurvate = (vCrossProduct.GetLength() / std::pow(vFirstDeriv.GetLength(), 3));

		if (dCurvate > dMaxAbsCurvature)
			dMaxAbsCurvature = dCurvate;

		/*
		*	FOR 2D ONLY
		*/
		dCurvate = vCrossProduct.z > 0 ? dCurvate : -dCurvate;

		Vector3d v(-vFirstDeriv.y, vFirstDeriv.x, 0); //�� vFirstDeriv ��ֱ ���������ĵ�λ����
		v.Normalize();

		vecNormalVector.Add(v);
		vecBasePoints.Add(ptBase);
		vecCurvatures.Add(dCurvate);
	}
	/*
	 *	����̶���������scale Ϊ����� maxOffset / abs(maxCurvature)
	 ���� scale �� maxoffset / dMaxAbsCurvature,���Ҹ�ֵ maxCurvature����
	 */
	
	if (!fixedScale)
	{
		maxCurvature = dMaxAbsCurvature;
	}
	Double dScale = maxOffset / maxCurvature;
	
	/*
	 *	��������Ϊ�ڻ����� �ط�����ƫ�� curvature * scale ����
	 */
	for (UInt32 i = 0; i < vecBasePoints.GetCount();i++)
	{
		Coordinate* pt = new Coordinate();
		*pt = vecBasePoints[i] + vecNormalVector[i] * vecCurvatures[i] * dScale;
		coordinates->Add(pt);
	}

	LineString *pLineString = new LineString(coordinates);
	return pLineString;
}

const Array<Double>& Spline::GetKnotsVector() const
{
	return m_vecKnots;
}

Double Spline::GetStartParam() const
{
	if (m_vecKnots.IsEmpty())
		return -1;
	return m_vecKnots[0];
}

Double Spline::GetEndParam() const
{
	if (m_vecKnots.IsEmpty())
		return -1;

	return m_vecKnots[m_vecKnots.GetCount() - 1];
}

const Array<Coordinate>& Spline::GetControlVertexes() const
{
	return m_vecControlVertexes;
}

UInt16 Spline::Degree() const
{
	return m_nDegree;
}