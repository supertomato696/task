#include "Algorithm/Line.h"
#include "Base/Math.h"
#include "Algorithm/Angle.h"
#include <Eigen/Dense>
#include "Geometries/Coordinate.h"

#include "geos/geos.h"
#include "geos/operation/buffer/BufferParameters.h"
#include "geos/simplify/DouglasPeuckerSimplifier.h"

using namespace std;
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Algorithm;

#ifndef PI 
define PI 3.14159265359
#endif

Bool LineAlgorithm::LineEquation(const Geometries::Coordinate& c0, const Geometries::Coordinate& c1, Base::Double& A, Base::Double& B, Base::Double& C)
{
	if (Math::Equal(c0.DistanceXY(c1), 0))
		return false;

	Eigen::Matrix3d K;
	Eigen::Vector3d D;

	K << c0.x, c0.y, 1, c1.x, c1.y, 1, 1, 1, 0;
	D << 0, 0, 1;

	Eigen::ColPivHouseholderQR<Eigen::Matrix3d> dec(K);
	Eigen::Vector3d x = dec.solve(D);

	A = x[0];
	B = x[1];
	C = x[2];

	return true;
}

geos::geom::Geometry* LineAlgorithm::CreateGeosLineString(const Base::Array<Geometries::Coordinate>& vecCoords)
{
	if (vecCoords.IsEmpty())
		return nullptr;

	try
	{
		const geos::geom::GeometryFactory *gf = geos::geom::GeometryFactory::getDefaultInstance();
		vector<geos::geom::Coordinate>* geosCoords0 = new vector<geos::geom::Coordinate>(vecCoords.GetCount());

		for (Int32 i = 0, n = vecCoords.GetCount(); i < n; i++)
		{
			const Geometries::Coordinate & c = vecCoords[i];
			(*geosCoords0)[i] = (geos::geom::Coordinate(c.x, c.y, c.z));
		}

		geos::geom::CoordinateSequence* geoscs0 = gf->getCoordinateSequenceFactory()->create(geosCoords0).release();
		return gf->createLineString(geoscs0);
	}
	catch (geos::util::GEOSException e)
	{
		return nullptr;
	}
	
	return nullptr;
}

geos::geom::Geometry* LineAlgorithm::CreateLineStringBuffer(Base::Array<Geometries::Coordinate>& vecCoords, Base::Double dBufferDis, Base::Bool roundCap /* = false */)
{
	auto_ptr<geos::geom::Geometry> pLineString(CreateGeosLineString(vecCoords));
	if (pLineString.get() == nullptr)
		return nullptr;

	geos::operation::buffer::BufferParameters::EndCapStyle bufferPara = roundCap ? geos::operation::buffer::BufferParameters::CAP_ROUND : geos::operation::buffer::BufferParameters::CAP_FLAT;

	try{
		return pLineString->buffer(dBufferDis, 6, bufferPara).release();
	}
	catch (geos::util::GEOSException e)
	{
		return nullptr;
	}

	return nullptr;
	
}

Void LineAlgorithm::DouglasPeuckerSimplify(const Base::Array<Geometries::Coordinate>& vecInput, const Base::Double dTolerance, Base::Array<Geometries::Coordinate>& vecOutput)
{
	geos::geom::Geometry* pGeometry = CreateGeosLineString(vecInput);
	if (nullptr == pGeometry)
		return;

#ifdef _WIN32 
	auto_ptr<geos::geom::Geometry> pResult = geos::simplify::DouglasPeuckerSimplifier::simplify(pGeometry, dTolerance);
#else
	unique_ptr<geos::geom::Geometry> pResult = geos::simplify::DouglasPeuckerSimplifier::simplify(pGeometry, dTolerance);
#endif

	if (pResult->getGeometryTypeId() != GEOS_LINESTRING)
		return;

	geos::geom::CoordinateSequence* cs = pResult->getCoordinates().release();
	SizeT count = cs->getSize();
	vecOutput.SetSize(count);
	for (SizeT i = 0; i < count; ++i)
	{
		geos::geom::Coordinate coord = cs->getAt(i);
		vecOutput[i] = Geometries::Coordinate(coord.x, coord.y, coord.z);
	}
}

void LineAlgorithm::RemoveDuplicatePoints(Base::Array<Geometries::Coordinate>& vecInput, const Base::Double dTolerance)
{
	auto num = vecInput.GetCount();
	if (num < 2 || dTolerance < 0)
		return;

	for (int i = 0; i < num - 1;)
	{
		Double distance = vecInput[i].Distance(vecInput[i + 1]);
		if (distance < dTolerance)
		{
			//��������̫�������󱻳�ϡ��1����
			if (vecInput.GetCount() > 2)
			{
				vecInput.Delete(i + 1);
				num--;
			}
			else
				break;
		}
		else
		{
			++i;
		}
	}
}

Void LineAlgorithm::RemoveDuplicatePoints(Base::Array<Geometries::Coordinate*>& vecInput, const Base::Double dTolerance)
{
	auto num = vecInput.GetCount();
	if (num < 2 || dTolerance < 0)
		return;

	for (int i = 0; i < num - 1;)
	{
		Double distance = vecInput[i]->Distance(*vecInput[i + 1]);
		if (distance < dTolerance)
		{
			//��������̫�������󱻳�ϡ��1����
			if (vecInput.GetCount() > 2)
			{
				vecInput.Delete(i + 1);
				num--;
			}
			else
				break;
		}
		else
		{
			++i;
		}
	}
}

Void LineAlgorithm::SmoothSTurnSegments(Base::Array<Geometries::Coordinate>& vecInput, const Base::Double s_turn_segment_max_length,
	const Base::Double s_turn_angle_degree, const Base::Double chord_height)
{
	Int32 nPoints = vecInput.GetCount();
	if (nPoints < 3)
		return;

	const Double complete_angle = PI - Angle::Degree2Radian(s_turn_angle_degree);

	//���� i-1,i,i+1����
	for (Int32 i = 1; i < nPoints - 2;i++)
	{
		const Geometries::Vector3d vCurrent = vecInput[i + 1] - vecInput[i];
		
		//�������۽��߶ε���󳤶�
		if (vCurrent.GetLength()>=s_turn_segment_max_length)
			continue;

		const Geometries::Vector3d vPrev = vecInput[i] - vecInput[i - 1];
		const Geometries::Vector3d vNext = vecInput[i + 2] - vecInput[i + 1];

		Double angle0 = vPrev.AngleWith(vCurrent);
		Double angle1 = vCurrent.AngleWith(vNext);

		//��������
		/*if(angle0< complete_angle && angle1<complete_angle)
			continue;*/

		Double turn0 = vPrev.CrossProduct(vCurrent).z;
		Double turn1 = vCurrent.CrossProduct(vNext).z;

		//����s��ת�乹��
		if (turn0 < 0 && turn1<0 || turn0>0&&turn1>0)
			continue;

		Double sinbeta = 2 * chord_height / vCurrent.GetLength();
		Double cosbeta = sqrt(1.0 - sinbeta*sinbeta);

		//I-1,I
		{
			Double sinalpha = sin(angle0);

			//�����ŽǱȽϴ� //����һ��
			if (sinbeta >= sinalpha)
			{
				vecInput[i] = (vecInput[i - 1] + vecInput[i]) / 2.0;
			}
			else
			{
				//�����Ҫʹ�ö೤���룬�����������ʹ�������
				Double cosalpha = sqrt(1.0 - sinalpha*sinalpha);
				Double sin_alpha_beta = sinalpha*cosbeta - cosalpha*sinbeta;
				Double length = chord_height / sin_alpha_beta;

				//�����㹻
				if (length < vPrev.GetLength())
				{
					vecInput[i] = vecInput[i] + (-vPrev)*length / vPrev.GetLength();
				}
				else
				{
					vecInput[i] = (vecInput[i + 1] + vecInput[i]) / 2.0;
				}
			}
		}//prev half
		//i,i+1
		{
			Double sinalpha = sin(angle1);
			Double cosalpha = sqrt(1.0 - sinalpha*sinalpha);
			Double sin_alpha_beta = sinalpha*cosbeta - cosalpha*sinbeta;
			Double length = chord_height / sin_alpha_beta;

			//�����ŽǱȽϴ� //����һ��
			if (sinbeta >= sinalpha)
			{
				vecInput[i+1] = (vecInput[i + 1] + vecInput[i+2]) / 2.0;
			}
			else
			{
				//�����㹻
				if (length < vPrev.GetLength())
				{
					vecInput[i + 1] = vecInput[i + 1] + (vNext)*length / vPrev.GetLength();
				}
				else
				{
					//delete i+1
					vecInput.Delete(i + 1);
					--i;
					--nPoints;//ɾ����� ����Ŀ�б仯
				}
			}
		} //next half
	}
}