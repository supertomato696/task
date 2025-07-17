/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryAlgorithm.cpp
简要描述:
******************************************************************/

#include <sstream>
#include <geos/geos.h>
#include <geos/operation/buffer/OffsetCurveBuilder.h>
#include <geos/simplify/DouglasPeuckerLineSimplifier.h>
#include <geos/opDistance.h>
#include <geos/algorithm/CGAlgorithmsDD.h>
#include <geos/algorithm/Distance.h>
#include <geos/algorithm/PointLocation.h>
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/Geometry.h"
#include "Geometries/GeomAdapter.h"
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Point.h"
#include "Base/String.h"
#include "Base/Macros.h"
#include "Base/Math.h"
#include "Geometries/BaseAlgorithm.h"
#include <math.h>
#include "Geometries/Polygon.h"
#include "Geometries/LinearRing.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/MultiLineString.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "geos/operation/buffer/BufferOp.h"

#include <limits.h>

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace Engine;
using namespace std;

const int offset = 1000;
const double eps = 1e-8;

String *GeometryAlgorithm::GeometryToWKT(const Engine::Geometries::Geometry *geom)
{
	geos::geom::Geometry *geosGeom = GeomAdapter::ToGeosGeom(geom);
	geos::io::WKTWriter writer;
	writer.setOutputDimension(3);
	std::string wktStr = writer.write(geosGeom);
	delete geosGeom;
	return new String(wktStr.c_str());
}

Engine::Geometries::Geometry *GeometryAlgorithm::WKTToGeometry(const String *wkt)
{
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	geos::io::WKTReader reader(gf);
	geos::geom::Geometry *geosGeom = reader.read(wkt->GetCStr()).release();
	Engine::Geometries::Geometry *navGeom = GeomAdapter::ToGeometry(geosGeom);
	delete geosGeom;
	return navGeom;
}

String *GeometryAlgorithm::GeometryToWKB(const Engine::Geometries::Geometry *geom)
{
	geos::geom::Geometry *geosGeom = GeomAdapter::ToGeosGeom(geom);
	geos::io::WKBWriter writer;
	writer.setOutputDimension(3);
	std::stringstream stream;
	writer.writeHEX(*geosGeom, stream);
	std::string wkbStr = stream.str();
	return new String(wkbStr.c_str());
}

Engine::Geometries::Geometry *GeometryAlgorithm::WKBToGeometry(const String *wkb)
{
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	geos::io::WKBReader reader(*gf);
	std::stringstream stream(wkb->GetCStr());
	geos::geom::Geometry *geosGeom = reader.readHEX(stream).release();
	Engine::Geometries::Geometry *navGeom = GeomAdapter::ToGeometry(geosGeom);
	delete geosGeom;
	return navGeom;
}

Bool GeometryAlgorithm::GetCentroid(const Engine::Geometries::Geometry *geom, Engine::Geometries::Coordinate *coord)
{
	if (coord == nullptr)
	{
		return false;
	}

	geos::geom::Geometry *geosGeom = GeomAdapter::ToGeosGeom(geom);
	geos::geom::Coordinate geosCoord;
	Bool result = geosGeom->getCentroid(geosCoord);

	coord->x = geosCoord.x;
	coord->y = geosCoord.y;
	Double minZ = LONG_MAX;
	Double maxZ = LONG_MIN;
	// 由于geo_s计算时不支持Z值，所以这里对Z值先做一个简单的处理，yanchl
	GeometryType gt = geom->GetGeometryType();
	switch (gt)
	{
	case Engine::Geometries::GeometryType::POINT:
	{
		const Engine::Geometries::Point *navPt = static_cast<const Engine::Geometries::Point *>(geom);
		coord->z = navPt->GetZ();
	}
	break;
	case Engine::Geometries::GeometryType::LINESTRING:
	case Engine::Geometries::GeometryType::LINEARRING:
	{
		const Engine::Geometries::LineString *navls = static_cast<const Engine::Geometries::LineString *>(geom);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetCoordinates();
		SizeT count = navCoords->GetCount();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			if (minZ > navCoord->z)
			{
				minZ = navCoord->z;
			}

			if (maxZ < navCoord->z)
			{
				maxZ = navCoord->z;
			}
		}

		coord->z = minZ + 0.5 * (maxZ - minZ);
	}
	break;
	case Engine::Geometries::GeometryType::POLYGON:
	{
		const Engine::Geometries::Polygon *navls = static_cast<const Engine::Geometries::Polygon *>(geom);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetExteriorRing()->GetCoordinates();
		SizeT count = navCoords->GetCount();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			if (minZ > navCoord->z)
			{
				minZ = navCoord->z;
			}

			if (maxZ < navCoord->z)
			{
				maxZ = navCoord->z;
			}
		}

		coord->z = minZ + 0.5 * (maxZ - minZ);
	}
	break;
	case Engine::Geometries::GeometryType::MULTIPOINT:
	{
	}
	break;
	case Engine::Geometries::GeometryType::MULTILINESTRING:
	{
		const Engine::Geometries::MultiLineString *navls = static_cast<const Engine::Geometries::MultiLineString *>(geom);
		const Array<Engine::Geometries::Coordinate *> *navCoords = ((LineString *)(navls->GetGeometryN(0)))->GetCoordinates();
		SizeT count = navCoords->GetCount();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			if (minZ > navCoord->z)
			{
				minZ = navCoord->z;
			}

			if (maxZ < navCoord->z)
			{
				maxZ = navCoord->z;
			}
		}

		coord->z = minZ + 0.5 * (maxZ - minZ);
	}
	break;
	case Engine::Geometries::GeometryType::MULTIPOLYGON:
	{
	}
	break;
	case Engine::Geometries::GeometryType::GEOMETRYCOLLECTION:
	{
	}
	break;
	default:
	{
		throw std::exception();
	}
	break;
	}

	// coord->z = geosCoord.z;

	delete geosGeom;

	return result;
}

Double GeometryAlgorithm::GetMinDistance(const Engine::Geometries::Point *pt, const Engine::Geometries::LineString *ls, Engine::Geometries::Coordinate *coord)
{
	geos::geom::Geometry *geospt = GeomAdapter::ToGeosGeom(pt);
	geos::geom::Geometry *geosls = GeomAdapter::ToGeosGeom(ls);

	geos::operation::distance::DistanceOp disOp(*geospt, *geosls);
	geos::geom::CoordinateSequence *cs = disOp.nearestPoints().release();
	Double dis = disOp.distance();

	delete geospt;
	delete geosls;

	if (coord != NULL)
	{
		coord->x = cs->getAt(1).x;
		coord->y = cs->getAt(1).y;
		coord->z = cs->getAt(1).z;
	}

	return dis;
}

SizeT GeometryAlgorithm::IndexAtLineString(
	const Engine::Geometries::Coordinate *coord,
	const Engine::Geometries::LineString *ls,
	Bool *isOnVertex, Bool *isOnLine)
{
	geos::geom::Coordinate geosCoord(coord->x, coord->y, coord->z);
	geos::geom::Geometry *geosls = GeomAdapter::ToGeosGeom(ls);
	geos::geom::CoordinateSequence *geoslsCs = geosls->getCoordinates().release();
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	geos::geom::CoordinateSequence *lineCs = gf->getCoordinateSequenceFactory()->create(2, geoslsCs->getDimension()).release();
	SizeT index = -1;
	*isOnLine = false;
	*isOnVertex = false;

	for (SizeT i = 0; i < geoslsCs->getSize() - 1; ++i)
	{
		if (geosCoord.distance(geoslsCs->getAt(i)) < 2.0)
		{
			index = i;
			*isOnLine = true;
			*isOnVertex = true;

			break;
		}

		if (geosCoord.distance(geoslsCs->getAt(i + 1)) < 2.0)
		{
			index = i + 1;
			*isOnLine = true;
			*isOnVertex = true;

			break;
		}

		if (i < geoslsCs->getSize() - 1)
		{
			lineCs->setAt(geoslsCs->getAt(i), 0);
			lineCs->setAt(geoslsCs->getAt(i + 1), 1);
			Bool result = (geos::algorithm::Distance::pointToSegment(geosCoord, geoslsCs->getAt(i), geoslsCs->getAt(i + 1)) < 0.1);

			if (result)
			{
				index = i;
				*isOnLine = true;
				*isOnVertex = false;

				break;
			}
		}
	}

	delete lineCs;
	delete geosls;

	return index;
}

Bool GeometryAlgorithm::Intersects(const Engine::Geometries::Geometry *geom1, const Engine::Geometries::Geometry *geom2)
{
	geos::geom::Geometry *geosgeom1 = GeomAdapter::ToGeosGeom(geom1);
	geos::geom::Geometry *geosgeom2 = GeomAdapter::ToGeosGeom(geom2);

	Bool intersect = geosgeom1->intersects(geosgeom2);

	delete geosgeom1;
	delete geosgeom2;

	return intersect;
}

Bool GeometryAlgorithm::IntersectsLineSegmentPolygon(const Engine::Geometries::Geometry *geom1, const Engine::Geometries::Geometry *geom2)
{
	geos::geom::Geometry *geosgeom = nullptr;
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	GeometryType gt = geom1->GetGeometryType();
	geos::geom::Geometry *geosgeom1 = nullptr;
	geos::geom::Geometry *geosgeom2 = nullptr;

	if (gt == Engine::Geometries::GeometryType::LINESTRING)
	{
		const Engine::Geometries::LineString *navls = static_cast<const Engine::Geometries::LineString *>(geom1);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetCoordinates();
		SizeT count = navCoords->GetCount();
		vector<geos::geom::Coordinate> *geosCoords = new vector<geos::geom::Coordinate>();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			geos::geom::Coordinate geosCoord(navCoord->x, navCoord->y, 0.0);
			geosCoords->push_back(geosCoord);
		}

		geos::geom::CoordinateSequence *geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords).release();
		geosgeom1 = gf->createLineString(geoscs);
	}

	gt = geom2->GetGeometryType();
	gf = GeometryFactory::getDefaultInstance();

	if (gt == Engine::Geometries::GeometryType::POLYGON)
	{
		const Engine::Geometries::Polygon *navpy = static_cast<const Engine::Geometries::Polygon *>(geom2);
		const Engine::Geometries::LinearRing *navpyShell = navpy->GetExteriorRing();
		const Engine::Geometries::LineString *navls = static_cast<const Engine::Geometries::LineString *>(navpyShell);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetCoordinates();
		SizeT count = navCoords->GetCount();
		vector<geos::geom::Coordinate> *geosCoords = new vector<geos::geom::Coordinate>();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			geos::geom::Coordinate geosCoord(navCoord->x, navCoord->y, 0.0);
			geosCoords->push_back(geosCoord);
		}

		const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
		geos::geom::CoordinateSequence *geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords).release();
		geosgeom = gf->createLinearRing(geoscs);
		const geos::geom::LinearRing *geospyShell = dynamic_cast<const geos::geom::LinearRing *>(geosgeom);
		vector<geos::geom::LinearRing *> *geosHoles = new vector<geos::geom::LinearRing *>();
		count = navpy->GetNumInteriorRing();

		for (SizeT i = 0; i < count; ++i)
		{
			const Engine::Geometries::LinearRing *navpyHole = navpy->GetInteriorRingN(i);

			const Engine::Geometries::LineString *navls = static_cast<const Engine::Geometries::LineString *>(navpyHole);
			const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetCoordinates();
			SizeT count = navCoords->GetCount();
			vector<geos::geom::Coordinate> *geosCoords = new vector<geos::geom::Coordinate>();

			for (SizeT i = 0; i < count; ++i)
			{
				Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
				geos::geom::Coordinate geosCoord(navCoord->x, navCoord->y, 0.0);
				geosCoords->push_back(geosCoord);
			}

			const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
			geos::geom::CoordinateSequence *geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords).release();
			geos::geom::LinearRing *geospyHole = gf->createLinearRing(geoscs);
			geosHoles->push_back(geospyHole);
		}

		geos::geom::Polygon *geospy = gf->createPolygon(const_cast<geos::geom::LinearRing *>(geospyShell), geosHoles);
		geosgeom2 = geospy;
	}

	Bool intersect = geosgeom1->intersects(geosgeom2);
	delete geosgeom1;
	delete geosgeom2;

	return intersect;
}

Double GeometryAlgorithm::ComputeAngle(
	Engine::Geometries::Coordinate coord1,
	Engine::Geometries::Coordinate coord2,
	Engine::Geometries::Coordinate coord3)
{
	Engine::Geometries::Coordinate v21 = coord1 - coord2;
	Engine::Geometries::Coordinate v23 = coord3 - coord2;
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

	Double arca = Math::ACos(cosa);
	return arca;
}

// 将度转换为弧度
Double GeometryAlgorithm::DegreeToRadian(Double a)
{
	Double result = (PI / 180) * a;
	return result;
}

// 将弧度转换度
Double GeometryAlgorithm::RadianToDegree(Double a)
{
	Double result = (180 / PI) * a;
	return result;
}

Bool GeometryAlgorithm::PtBreakLine(
	const Engine::Geometries::Coordinate *pntHitTest,
	Engine::Geometries::LineString *ls,
	Engine::Geometries::LineString *&lsS,
	Engine::Geometries::LineString *&lsE)
{
	if (pntHitTest == NULL || ls == NULL)
	{
		return false;
	}

	Base::SizeT nCount = ls->GetNumPoints();

	if (nCount < 2)
	{
		return false;
	}

	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	Bool b = BaseAlgorithm::GetNearestPntToLineset(pntHitTest, ls->GetCoordinates(), pntProject, nSegIndex);
	int i = 0;

	if (nSegIndex >= 0 && nSegIndex < nCount)
	{
		Engine::Base::Array<Engine::Geometries::Coordinate *> *pPoints =
			ls->GetCoordinates();
		Engine::Base::Array<Engine::Geometries::Coordinate *> *pPtsS =
			new Engine::Base::Array<Engine::Geometries::Coordinate *>();

		for (i = 0; i <= nSegIndex; i++)
		{
			Engine::Geometries::Coordinate *pPtS =
				new Engine::Geometries::Coordinate();
			pPtS->x = (*pPoints)[i]->x;
			pPtS->y = (*pPoints)[i]->y;
			pPtsS->Add(pPtS);
		}

		Engine::Geometries::Coordinate *pPtS_Project =
			new Engine::Geometries::Coordinate();
		pPtS_Project->x = pntProject.x;
		pPtS_Project->y = pntProject.y;
		pPtsS->Add(pPtS_Project);
		Engine::Base::Array<Engine::Geometries::Coordinate *> *pPtsE =
			new Engine::Base::Array<Engine::Geometries::Coordinate *>();
		Engine::Geometries::Coordinate *pPtE_Project =
			new Engine::Geometries::Coordinate();
		pPtE_Project->x = pntProject.x;
		pPtE_Project->y = pntProject.y;
		pPtsE->Add(pPtE_Project);

		for (i = nSegIndex + 1; i < nCount; i++)
		{
			Engine::Geometries::Coordinate *pPtE = new Engine::Geometries::Coordinate();
			pPtE->x = (*pPoints)[i]->x;
			pPtE->y = (*pPoints)[i]->y;
			pPtsE->Add(pPtE);
		}

		if ((*pPtsS)[pPtsS->GetCount() - 2]->x == (*pPtsS)[pPtsS->GetCount() - 1]->x &&
			(*pPtsS)[pPtsS->GetCount() - 2]->y == (*pPtsS)[pPtsS->GetCount() - 1]->y)
		{
			delete (*pPtsS)[pPtsS->GetCount() - 1];
			pPtsS->Delete(pPtsS->GetCount() - 1);
		}

		if ((*pPtsE)[0]->x == (*pPtsE)[1]->x && (*pPtsE)[0]->y == (*pPtsE)[1]->y)
		{
			delete (*pPtsE)[0];
			pPtsE->Delete(0);
		}

		lsS = new Engine::Geometries::LineString(pPtsS);
		lsE = new Engine::Geometries::LineString(pPtsE);

		if (pPtsS->GetCount() < 2 || pPtsE->GetCount() < 2)
		{
			delete lsS;
			delete lsE;
			lsS = NULL;
			lsE = NULL;

			return false;
		}

		return true;
	}
	else
	{
		return false;
	}
}

Engine::Geometries::LineString *GeometryAlgorithm::InterceptPartLine(
	const Engine::Geometries::Coordinate *pntS,
	const Engine::Geometries::Coordinate *pntE,
	Engine::Geometries::LineString *ls)
{
	Engine::Geometries::Coordinate pntSProject;
	Engine::Geometries::Coordinate pntEProject;
	Int32 nSSegIndex;
	Int32 nESegIndex;

	BaseAlgorithm::GetNearestPntToLineset(pntS, ls->GetCoordinates(), pntSProject, nSSegIndex);
	BaseAlgorithm::GetNearestPntToLineset(pntE, ls->GetCoordinates(), pntEProject, nESegIndex);

	if (nSSegIndex > nESegIndex)
	{
		return NULL;
	}

	// 计算投影点Z值
	Double dLengthSP = ls->GetCoordinates()->GetAt(nSSegIndex)->DistanceXY(pntSProject);
	Double dLengthSPL = ls->GetCoordinates()->GetAt(nSSegIndex)->DistanceXY(*(ls->GetCoordinates()->GetAt(nSSegIndex + 1)));
	Double dLengthEP = ls->GetCoordinates()->GetAt(nESegIndex)->DistanceXY(pntEProject);
	Double dLengthEPL = ls->GetCoordinates()->GetAt(nESegIndex)->DistanceXY(*(ls->GetCoordinates()->GetAt(nESegIndex + 1)));
	Double dZtmp = ls->GetCoordinates()->GetAt(nSSegIndex)->z;
	Double dZMtmp = ls->GetCoordinates()->GetAt(nSSegIndex + 1)->z - ls->GetCoordinates()->GetAt(nSSegIndex)->z;
	pntSProject.z = dZtmp + dZMtmp * dLengthSP / dLengthSPL;
	dZtmp = ls->GetCoordinates()->GetAt(nESegIndex)->z;
	dZMtmp = ls->GetCoordinates()->GetAt(nESegIndex + 1)->z - ls->GetCoordinates()->GetAt(nESegIndex)->z;
	if (dLengthEPL == 0.0) // fix by duanzhikang 避免除以0
	{
		pntEProject.z = dZtmp + dZMtmp;
	}
	else
	{
		pntEProject.z = dZtmp + dZMtmp * dLengthEP / dLengthEPL;
	}

	if (nSSegIndex == nESegIndex)
	{
		Coordinate *coorSeg = ls->GetCoordinates()->GetAt(nSSegIndex);
		Double dDstSP2Seg = coorSeg->Distance(pntSProject);
		Double dDstEP2Seg = coorSeg->Distance(pntEProject);
		if (dDstSP2Seg < dDstEP2Seg)
		{
			Engine::Geometries::LineString *lsPart = new LineString();
			Array<Engine::Geometries::Coordinate *> *coordinates = lsPart->GetCoordinates();
			coordinates->Add(new Coordinate(pntSProject));
			coordinates->Add(new Coordinate(pntEProject));
			return lsPart;
		}
		else
		{
			return NULL;
		}
	}

	Engine::Geometries::LineString *lsPart = NULL;
	Array<Engine::Geometries::Coordinate *> *coordinates = ls->GetCoordinates();
	Int32 ptNum = coordinates->GetCount();
	Array<Engine::Geometries::Coordinate *> *coordinatesPart = new Array<Engine::Geometries::Coordinate *>();
	Int32 i = 0;

	if (pntSProject.Distance(*((*coordinates)[0])) < Geometries_EP && pntEProject.Distance(*((*coordinates)[ptNum - 1])) < Geometries_EP)
	{
		lsPart = new LineString(*ls);

		return lsPart;
	}
	else if (pntSProject.Distance(*((*coordinates)[0])) < Geometries_EP)
	{
		coordinatesPart->SetSize(nESegIndex + 2);

		for (Int32 j = 0; j <= nESegIndex; j++)
		{
			Coordinate *coordinate = new Coordinate();
			coordinate->x = coordinates->GetAt(j)->x;
			coordinate->y = coordinates->GetAt(j)->y;
			coordinate->z = coordinates->GetAt(j)->z;
			coordinatesPart->SetAt(i++, coordinate);
		}

		Coordinate *coordinate = new Coordinate();
		coordinate->x = pntEProject.x;
		coordinate->y = pntEProject.y;
		coordinate->z = pntEProject.z;
		coordinatesPart->SetAt(i++, coordinate);
		// lsPart = new LineString(coordinatesPart);

		// return lsPart;
	}
	else if (pntEProject.Distance(*((*coordinates)[ptNum - 1])) < Geometries_EP)
	{
		coordinatesPart->SetSize(ptNum - nSSegIndex);

		for (Int32 j = nSSegIndex + 1; j < ptNum; j++)
		{
			Coordinate *coordinate = new Coordinate();
			coordinate->x = coordinates->GetAt(j)->x;
			coordinate->y = coordinates->GetAt(j)->y;
			coordinate->z = coordinates->GetAt(j)->z;
			coordinatesPart->SetAt(++i, coordinate);
		}

		Coordinate *coordinate = new Coordinate();
		coordinate->x = pntSProject.x;
		coordinate->y = pntSProject.y;
		coordinate->z = pntSProject.z;
		coordinatesPart->SetAt(0, coordinate);
		// lsPart = new LineString(coordinatesPart);

		// return lsPart;
	}
	else
	{
		coordinatesPart->SetSize(nESegIndex - nSSegIndex + 2);
		i = 0;

		Coordinate *coordinate = new Coordinate();
		coordinate->x = pntSProject.x;
		coordinate->y = pntSProject.y;
		coordinate->z = pntSProject.z;
		coordinatesPart->SetAt(i++, coordinate);

		for (Int32 j = nSSegIndex + 1; j <= nESegIndex; j++)
		{
			coordinate = new Coordinate();
			coordinate->x = coordinates->GetAt(j)->x;
			coordinate->y = coordinates->GetAt(j)->y;
			coordinate->z = coordinates->GetAt(j)->z;
			coordinatesPart->SetAt(i++, coordinate);
		}

		coordinate = new Coordinate();
		coordinate->x = pntEProject.x;
		coordinate->y = pntEProject.y;
		coordinate->z = pntEProject.z;
		coordinatesPart->SetAt(i, coordinate);
	}

	if (coordinatesPart->GetAt(0)->Distance(*((*coordinatesPart)[1])) < Geometries_EP)
	{
		DELETE_PTR((*coordinatesPart)[0]);
		coordinatesPart->Delete(0);
	}

	ptNum = coordinatesPart->GetCount();
	if (coordinatesPart->GetAt(ptNum - 1)->Distance(*((*coordinatesPart)[ptNum - 2])) < Geometries_EP)
	{
		DELETE_PTR((*coordinatesPart)[ptNum - 1]);
		coordinatesPart->Delete(ptNum - 1);
	}

	lsPart = new LineString(coordinatesPart);

	return lsPart;
}

Bool GeometryAlgorithm::PtBreakLineSegments(
	const Engine::Geometries::Coordinate *pntHitTest,
	const Array<Engine::Geometries::Coordinate *> *coordinates,
	Engine::Geometries::LineString *&lsS,
	Engine::Geometries::LineString *&lsE)
{
	if (pntHitTest == NULL || coordinates == NULL)
	{
		return false;
	}

	Base::SizeT nCount = coordinates->GetCount();

	if (nCount < 2)
	{
		return false;
	}

	DELETE_PTR(lsS);
	DELETE_PTR(lsE);

	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	Double dis = BaseAlgorithm3D::GetDistancePointToLinesegments(*pntHitTest, coordinates, pntProject, nSegIndex);
	int i = 0;

	if (nSegIndex >= 0 && nSegIndex < nCount)
	{
		Engine::Base::Array<Engine::Geometries::Coordinate *> *pPtsS =
			new Engine::Base::Array<Engine::Geometries::Coordinate *>();

		for (i = 0; i <= nSegIndex; i++)
		{
			Engine::Geometries::Coordinate *pPtS =
				new Engine::Geometries::Coordinate();
			pPtS->x = (*coordinates)[i]->x;
			pPtS->y = (*coordinates)[i]->y;
			pPtS->z = (*coordinates)[i]->z;
			pPtsS->Add(pPtS);
		}

		Engine::Geometries::Coordinate *pPtS_Project =
			new Engine::Geometries::Coordinate();
		pPtS_Project->x = pntProject.x;
		pPtS_Project->y = pntProject.y;
		pPtS_Project->z = pntProject.z;
		pPtsS->Add(pPtS_Project);

		Engine::Base::Array<Engine::Geometries::Coordinate *> *pPtsE =
			new Engine::Base::Array<Engine::Geometries::Coordinate *>();
		Engine::Geometries::Coordinate *pPtE_Project =
			new Engine::Geometries::Coordinate();
		pPtE_Project->x = pntProject.x;
		pPtE_Project->y = pntProject.y;
		pPtE_Project->z = pntProject.z;
		pPtsE->Add(pPtE_Project);

		for (i = nSegIndex + 1; i < nCount; i++)
		{
			Engine::Geometries::Coordinate *pPtE = new Engine::Geometries::Coordinate();
			pPtE->x = (*coordinates)[i]->x;
			pPtE->y = (*coordinates)[i]->y;
			pPtE->z = (*coordinates)[i]->z;
			pPtsE->Add(pPtE);
		}

		if ((*pPtsS)[pPtsS->GetCount() - 2]->x == (*pPtsS)[pPtsS->GetCount() - 1]->x &&
			(*pPtsS)[pPtsS->GetCount() - 2]->y == (*pPtsS)[pPtsS->GetCount() - 1]->y &&
			(*pPtsS)[pPtsS->GetCount() - 2]->z == (*pPtsS)[pPtsS->GetCount() - 1]->z)
		{
			delete (*pPtsS)[pPtsS->GetCount() - 1];
			pPtsS->Delete(pPtsS->GetCount() - 1);
		}

		if ((*pPtsE)[0]->x == (*pPtsE)[1]->x && (*pPtsE)[0]->y == (*pPtsE)[1]->y && (*pPtsE)[0]->z == (*pPtsE)[1]->z)
		{
			delete (*pPtsE)[0];
			pPtsE->Delete(0);
		}

		lsS = new Engine::Geometries::LineString(pPtsS);
		lsE = new Engine::Geometries::LineString(pPtsE);

		if (pPtsS->GetCount() < 2)
		{
			delete lsS;
			lsS = NULL;
		}

		if (pPtsE->GetCount() < 2)
		{
			delete lsE;
			lsE = NULL;
		}

		return true;
	}

	return false;
}

Engine::Geometries::LineString *GeometryAlgorithm::GenerateOffsetLine(const Engine::Geometries::LineString *pLineString, Double distance)
{
	if (NULL == pLineString)
		return NULL;

	if (0 == distance)
		return new LineString(*pLineString);

	Int32 nPoints = pLineString->GetNumPoints();
	if (nPoints <= 0 || pLineString->GetLengthXY() < Geometries_EP)
		return NULL;

	LineString *pLineStringTmp = new LineString(*pLineString);
	Coordinate pFirstCoor = *(pLineStringTmp->GetCoordinateN(0));

	Array<Coordinate *> *pCoorsSrc = pLineStringTmp->GetCoordinates();
	for (int i = 0; i < nPoints; i++)
	{
		Coordinate *coor = pCoorsSrc->ElementAt(i);
		*coor = *coor - pFirstCoor;
	}

	//////////////////////////////////////////////////////////////////////////
	// 转出geos 线
	geos::geom::Geometry *pGeometry = GeomAdapter::ToGeosGeom(pLineStringTmp);
	if (NULL == pGeometry || pGeometry->getGeometryTypeId() != GEOS_LINESTRING)
	{
		DELETE_PTR(pLineStringTmp);
		return NULL;
	}
	DELETE_PTR(pLineStringTmp);

	geos::geom::LineString *pLineStringG = dynamic_cast<geos::geom::LineString *>(pGeometry);

	// 偏移参数
	typedef geos::operation::buffer::BufferParameters BufferParameters;
	BufferParameters bufParam(BufferParameters::DEFAULT_QUADRANT_SEGMENTS, BufferParameters::CAP_FLAT);
	bufParam.setJoinStyle(BufferParameters::JOIN_MITRE);
	geos::geom::PrecisionModel pm;
	// 偏移builder
	geos::operation::buffer::OffsetCurveBuilder offsetBuilder(&pm, bufParam);
	// 偏移后坐标
	std::vector<geos::geom::CoordinateSequence *> vecOffsetCoors;

	offsetBuilder.getSingleSidedLineCurve(pLineStringG->getCoordinatesRO() /*pSimplifiedCoor*/, std::abs(distance), vecOffsetCoors, (distance > 0), (distance < 0));

	if (vecOffsetCoors.empty())
	{
		// delete pSimplifiedCoor;
		geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pLineStringG);
		return NULL;
	}

	geos::geom::CoordinateSequence *pOffsetGeometryG = vecOffsetCoors.front();

	Array<Coordinate *> *pCoordinates = new Array<Coordinate *>(pOffsetGeometryG->size() - 1);
	for (SizeT i = 0, n = pOffsetGeometryG->size() - 1; i < n; i++)
	{
		geos::geom::Coordinate c = pOffsetGeometryG->getAt(i);
		pCoordinates->SetAt(i, new Coordinate(c.x + pFirstCoor.x, c.y + pFirstCoor.y, 0));
	}

	// 测试 GEOS右侧偏移线的方向与线方向相反，需要矫正
	if (distance < 0)
		pCoordinates->Reverse();

	LineString *pOffsetLineString = new LineString(pCoordinates);

	//////////////////////////////////////////////////////////////////////////
	// GARBAGE COLLECTION
	for (SizeT i = 0; i < vecOffsetCoors.size(); i++)
	{
		delete vecOffsetCoors[i];
	}
	vecOffsetCoors.clear();

	// delete pSimplifiedCoor;
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pLineStringG);

	return pOffsetLineString;
}

Engine::Geometries::Polygon *GeometryAlgorithm::GenerateOffsetPolygon(const Engine::Geometries::Polygon *pPolygon, Base::Double distance)
{
	if (distance <= 0)
		return NULL;

	Base::Array<Geometries::Coordinate *> *coords = pPolygon->GetExteriorRing()->GetCoordinates();
	if (coords->GetCount() < 3)
	{
		return NULL;
	}

	geos::geom::Geometry *pGeometry = GeomAdapter::ToGeosGeom(pPolygon);
	geos::geom::Geometry *pGeosPoly = geos::operation::buffer::BufferOp::bufferOp(pGeometry, distance);
	Engine::Geometries::Polygon *polygon = static_cast<Engine::Geometries::Polygon *>(GeomAdapter::ToGeometry(pGeosPoly));
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosPoly);

	Engine::Geometries::LinearRing *lr = polygon->GetExteriorRing();
	Base::Array<Engine::Geometries::Coordinate *> *NCoords = lr->GetCoordinates();
	for (auto itr = NCoords->Begin(); itr != NCoords->End(); itr++)
	{
		// 将z值赋值为第一个坐标的z
		(*itr)->z = coords->GetAt(0)->z;
	}
	return polygon;

	/*Geometries::LineString* origLine = static_cast<Geometries::LineString*>(pPolygon->GetExteriorRing()->Clone());
	Engine::Geometries::LineString* offsetLine = GenerateOffsetLine(origLine, distance * (-1));
	Base::Array<Geometries::Coordinate *>*  offCoords = offsetLine->GetCoordinates();
	DELETE_PTR(origLine);

	Base::Array<Geometries::Coordinate *>* newCoords = new Base::Array<Geometries::Coordinate *>;
	for (auto itr = offCoords->Begin(); itr != offCoords->End(); itr++)
	{
		newCoords->Add(new Geometries::Coordinate(**itr));
	}
	newCoords->Add(new Geometries::Coordinate(**offCoords->Begin()));
	DELETE_PTR(offsetLine);

	Engine::Geometries::LinearRing* lr = new Engine::Geometries::LinearRing(newCoords);
	Engine::Geometries::Polygon* polygon = new Engine::Geometries::Polygon(lr);
	return polygon;*/
}

Void GeometryAlgorithm::GenerateOffsetLines(const Engine::Geometries::LineString *pLineString, const Array<Double> &vecDistance, Array<Engine::Geometries::LineString *> &vecOffsetLines)
{
	vecOffsetLines.Clear();
	for (SizeT i = 0; i < vecDistance.GetCount(); i++)
	{
		LineString *pLineStringOffset = GenerateOffsetLine(pLineString, vecDistance.GetAt(i));
		vecOffsetLines.Add(pLineStringOffset);
	}
}

// 利用Delauney三角形提取中心线算法
/*Engine::Geometries::LineString* GeometryAlgorithm::GenerateCenterLine(
	const Engine::Geometries::LineString* pLineStringL,
	const Engine::Geometries::LineString* pLineStringR)
{
	Array<Engine::Geometries::Coordinate*> *pCoordinatesL = pLineStringL->GetCoordinates();
	Array<Engine::Geometries::Coordinate*> *pCoordinatesR = pLineStringR->GetCoordinates();

	if ((pCoordinatesL == NULL) || (pCoordinatesR == NULL))
	{
		return NULL;
	}

	SizeT ncoordsL = pCoordinatesL->GetCount();
	SizeT ncoordsR = pCoordinatesR->GetCount();

	if ((ncoordsL == 0) || (ncoordsR == 0))
	{
		return NULL;
	}

	//中线点坐标串数组
	Array<Engine::Geometries::Coordinate*>* arrCentercoords = new Array<Engine::Geometries::Coordinate*>();
	Engine::Geometries::Coordinate* centercoord = NULL;

	Engine::Geometries::Coordinate* coordLFrom = NULL;
	Engine::Geometries::Coordinate* coordLTo = NULL;
	Engine::Geometries::Coordinate* coordRFrom = NULL;
	Engine::Geometries::Coordinate* coordRTo = NULL;

	SizeT i = 0, j = 0;

	//加入第一个中心点
	centercoord = new Engine::Geometries::Coordinate();
	centercoord->x = (pCoordinatesL->GetAt(0)->x + pCoordinatesR->GetAt(0)->x) / 2;
	centercoord->y = (pCoordinatesL->GetAt(0)->y + pCoordinatesR->GetAt(0)->y) / 2;
	centercoord->z = (pCoordinatesL->GetAt(0)->z + pCoordinatesR->GetAt(0)->z) / 2;
	arrCentercoords->Add(centercoord);

	//生成中间部分的中心点
	while ((i < ncoordsL) || (j < ncoordsR))
	{
		coordLFrom = pCoordinatesL->GetAt(i);
		coordRFrom = pCoordinatesR->GetAt(j);

		if (i < ncoordsL - 1)
		{
			coordLTo = pCoordinatesL->GetAt(i + 1);
		}

		if (j < ncoordsR - 1)
		{
			coordRTo = pCoordinatesR->GetAt(j + 1);
		}

		//都到了尾巴，结束吧
		if ((coordLTo == NULL) && (coordRTo == NULL))
		{
			break;
		}

		if (coordLTo == NULL)//左侧已到头
		{
			centercoord = new Engine::Geometries::Coordinate();
			centercoord->x = (coordLFrom->x + coordRFrom->x + coordRTo->x) / 3;
			centercoord->y = (coordLFrom->y + coordRFrom->y + coordRTo->y) / 3;
			centercoord->z = (coordLFrom->z + coordRFrom->z + coordRTo->z) / 3;

			arrCentercoords->Add(centercoord);
			j++;
		}
		else if (coordRTo == NULL)//右侧到头
		{
			centercoord = new Engine::Geometries::Coordinate();
			centercoord->x = (coordLFrom->x + coordRFrom->x + coordLTo->x) / 3;
			centercoord->y = (coordLFrom->y + coordRFrom->y + coordLTo->y) / 3;
			centercoord->z = (coordLFrom->z + coordRFrom->z + coordLTo->z) / 3;

			arrCentercoords->Add(centercoord);
			i++;
		}
		else//两侧都还没到头
		{
			double dLDis = coordLFrom->Distance(*coordRTo);
			double dRDis = coordRFrom->Distance(*coordLTo);

			if (dLDis > dRDis)
			{
				centercoord = new Engine::Geometries::Coordinate();
				centercoord->x = (coordLFrom->x + coordRFrom->x + coordLTo->x) / 3;
				centercoord->y = (coordLFrom->y + coordRFrom->y + coordLTo->y) / 3;
				centercoord->z = (coordLFrom->z + coordRFrom->z + coordLTo->z) / 3;

				arrCentercoords->Add(centercoord);
				i++;
			}
			else
			{
				centercoord = new Engine::Geometries::Coordinate();
				centercoord->x = (coordLFrom->x + coordRFrom->x + coordRTo->x) / 3;
				centercoord->y = (coordLFrom->y + coordRFrom->y + coordRTo->y) / 3;
				centercoord->z = (coordLFrom->z + coordRFrom->z + coordRTo->z) / 3;

				arrCentercoords->Add(centercoord);
				j++;
			}
		}

		coordLFrom = NULL;
		coordLTo = NULL;
		coordRFrom = NULL;
		coordRTo = NULL;
	}

	//加入最后一个中心点
	centercoord = new Engine::Geometries::Coordinate();
	centercoord->x = (pCoordinatesL->GetAt(ncoordsL - 1)->x + pCoordinatesR->GetAt(ncoordsR - 1)->x) / 2;
	centercoord->y = (pCoordinatesL->GetAt(ncoordsL - 1)->y + pCoordinatesR->GetAt(ncoordsR - 1)->y) / 2;
	centercoord->z = (pCoordinatesL->GetAt(ncoordsL - 1)->z + pCoordinatesR->GetAt(ncoordsR - 1)->z) / 2;
	arrCentercoords->Add(centercoord);

	Engine::Geometries::LineString* pOffsetLineString =
		new Engine::Geometries::LineString(arrCentercoords);

	return pOffsetLineString;
}*/

Engine::Geometries::LineString *GeometryAlgorithm::GenerateCenterLine(
	const Engine::Geometries::LineString *pLineStringL,
	const Engine::Geometries::LineString *pLineStringR)
{
	Array<Engine::Geometries::Coordinate *> *pCoordinatesL =
		pLineStringL->GetCoordinates();
	Array<Engine::Geometries::Coordinate *> *pCoordinatesR =
		pLineStringR->GetCoordinates();

	if ((pCoordinatesL == NULL) || (pCoordinatesR == NULL))
	{
		return NULL;
	}

	Int32 ncoordsL = pCoordinatesL->GetCount();
	Int32 ncoordsR = pCoordinatesR->GetCount();

	if ((ncoordsL < 2) || (ncoordsR < 2))
	{
		return NULL;
	}

	Array<Engine::Geometries::Coordinate *> *arrCentercoords =
		new Array<Engine::Geometries::Coordinate *>();
	Engine::Geometries::Coordinate *centercoord = NULL;
	centercoord = new Engine::Geometries::Coordinate();
	centercoord->x = (pCoordinatesL->GetAt(0)->x + pCoordinatesR->GetAt(0)->x) / 2;
	centercoord->y = (pCoordinatesL->GetAt(0)->y + pCoordinatesR->GetAt(0)->y) / 2;
	centercoord->z = (pCoordinatesL->GetAt(0)->z + pCoordinatesR->GetAt(0)->z) / 2;
	arrCentercoords->Add(centercoord);
	int i = 1;
	int j = 1;
	double disL = -1.0;
	double disR = -1.0;
	Engine::Geometries::LineString *lsS = NULL;
	Engine::Geometries::LineString *lsE = NULL;
	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	Engine::Geometries::Coordinate *middleCoordL =
		new Engine::Geometries::Coordinate();
	Engine::Geometries::Coordinate *middleCoordR =
		new Engine::Geometries::Coordinate();

	while (i < pCoordinatesL->GetCount() - 1 && j < pCoordinatesR->GetCount() - 1)
	{
		if (disL < 0.0)
		{
			Bool bL = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesL->GetAt(i),
															pLineStringR->GetCoordinates(), pntProject, nSegIndex);
			middleCoordL->x = (pCoordinatesL->GetAt(i)->x + pntProject.x) / 2;
			middleCoordL->y = (pCoordinatesL->GetAt(i)->y + pntProject.y) / 2;
			middleCoordL->z = 0.0;
			disL = BaseAlgorithm::DistancePtToPt(centercoord, middleCoordL);
		}

		if (disR < 0.0)
		{
			Bool bR = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesR->GetAt(j),
															pLineStringL->GetCoordinates(), pntProject, nSegIndex);
			middleCoordR->x = (pCoordinatesR->GetAt(j)->x + pntProject.x) / 2;
			middleCoordR->y = (pCoordinatesR->GetAt(j)->y + pntProject.y) / 2;
			middleCoordR->z = 0.0;
			disR = BaseAlgorithm::DistancePtToPt(centercoord, middleCoordR);
		}

		centercoord = new Engine::Geometries::Coordinate();

		if (disL <= disR)
		{
			i++;
			centercoord->x = middleCoordL->x;
			centercoord->y = middleCoordL->y;
			centercoord->z = middleCoordL->z;
			disL = -1.0;
			disR = BaseAlgorithm::DistancePtToPt(centercoord, middleCoordR);
		}
		else
		{
			j++;
			centercoord->x = middleCoordR->x;
			centercoord->y = middleCoordR->y;
			centercoord->z = middleCoordR->z;
			disR = -1.0;
			disL = BaseAlgorithm::DistancePtToPt(centercoord, middleCoordL);
		}

		arrCentercoords->Add(centercoord);
	}

	if (i == pCoordinatesL->GetCount() - 1)
	{
		while (j < pCoordinatesR->GetCount() - 1)
		{
			Engine::Geometries::LineString *pLineStringLCopy =
				new Engine::Geometries::LineString(*pLineStringL);
			Bool bR = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesR->GetAt(j),
															pLineStringL->GetCoordinates(), pntProject, nSegIndex);
			middleCoordR->x = (pCoordinatesR->GetAt(j)->x + pntProject.x) / 2;
			middleCoordR->y = (pCoordinatesR->GetAt(j)->y + pntProject.y) / 2;
			middleCoordR->z = 0.0;
			centercoord = new Engine::Geometries::Coordinate();
			j++;
			centercoord->x = middleCoordR->x;
			centercoord->y = middleCoordR->y;
			centercoord->z = middleCoordR->z;
			arrCentercoords->Add(centercoord);
		}
	}
	else
	{
		while (i < pCoordinatesL->GetCount() - 1)
		{
			Engine::Geometries::LineString *pLineStringRCopy =
				new Engine::Geometries::LineString(*pLineStringR);
			Bool bL = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesL->GetAt(i),
															pLineStringR->GetCoordinates(), pntProject, nSegIndex);
			middleCoordL->x = (pCoordinatesL->GetAt(i)->x + pntProject.x) / 2;
			middleCoordL->y = (pCoordinatesL->GetAt(i)->y + pntProject.y) / 2;
			middleCoordL->z = 0.0;
			centercoord = new Engine::Geometries::Coordinate();
			i++;
			centercoord->x = middleCoordL->x;
			centercoord->y = middleCoordL->y;
			centercoord->z = middleCoordL->z;
			arrCentercoords->Add(centercoord);
		}
	}

	if (middleCoordL != NULL)
	{
		delete middleCoordL;
	}

	if (middleCoordR != NULL)
	{
		delete middleCoordR;
	}

	centercoord = new Engine::Geometries::Coordinate();
	centercoord->x = (pCoordinatesL->GetAt(pCoordinatesL->GetCount() - 1)->x + pCoordinatesR->GetAt(pCoordinatesR->GetCount() - 1)->x) / 2;
	centercoord->y = (pCoordinatesL->GetAt(pCoordinatesL->GetCount() - 1)->y + pCoordinatesR->GetAt(pCoordinatesR->GetCount() - 1)->y) / 2;
	centercoord->z = (pCoordinatesL->GetAt(pCoordinatesL->GetCount() - 1)->z + pCoordinatesR->GetAt(pCoordinatesR->GetCount() - 1)->z) / 2;
	arrCentercoords->Add(centercoord);
	Engine::Geometries::LineString *pOffsetLineString =
		new Engine::Geometries::LineString(arrCentercoords);

	return pOffsetLineString;
}

Bool GeometryAlgorithm::PtOnLineRing(const Engine::Geometries::LinearRing *lring, Engine::Geometries::Coordinate *pt, Engine::Base::Double tolerance /* = 1.0 */)
{
	if (lring == NULL || pt == NULL)
	{
		return false;
	}

	Base::Array<Coordinate *> *coords = lring->GetCoordinates();

	if (lring->GetNumPoints() < 3)
	{
		return false;
	}

	for (Base::SizeT i = 0; i < coords->GetCount() - 1; i++)
	{
		Coordinate *coord1 = coords->GetAt(i);
		Coordinate *coord2 = coords->GetAt(i + 1);
		if (!BaseAlgorithm::IsProjectToLineset(pt, coord1, coord2))
		{
			continue;
		}
		Base::Double dis = BaseAlgorithm::DistancePtToLine2D(pt, coord1, coord2);

		if (dis < tolerance) // 1米就认为是撞边了
		{
			*pt = BaseAlgorithm::GetPtToLine(coord1, coord2, pt);
			return true;
		}
	}
	return false;
}

Bool GeometryAlgorithm::PtInPolygon(const Engine::Geometries::Polygon *polygon,
									const Engine::Geometries::Coordinate *pt)
{
	if (polygon == NULL || pt == NULL)
	{
		return false;
	}

	Engine::Geometries::LinearRing *lr = polygon->GetExteriorRing();

	if (lr->GetNumPoints() < 3)
	{
		return false;
	}

	geos::geom::Coordinate geosCoord(pt->x, pt->y, pt->z);
	geos::geom::Geometry *geosls = GeomAdapter::ToGeosGeom(polygon);
	CoordinateSequence *pCoords = geosls->getCoordinates().release();
	Bool bResult = geos::algorithm::PointLocation::isInRing(geosCoord, pCoords);

	delete pCoords;
	delete geosls;

	return bResult;
}

Bool GeometryAlgorithm::OutOnMapProjection(
	const Engine::Geometries::Polygon *polygon,
	Engine::Geometries::Coordinate *pt,
	Engine::Geometries::Coordinate &pntProject)
{
	Bool bOutOnMapProjection = false;

	if (polygon == NULL || pt == NULL)
	{
		return bOutOnMapProjection;
	}

	Engine::Geometries::LinearRing *lr = polygon->GetExteriorRing();

	if (lr->GetNumPoints() < 3)
	{
		return bOutOnMapProjection;
	}

	Bool b = PtInPolygon(polygon, pt);

	if (b == false)
	{
		bOutOnMapProjection = true;
	}
	else
	{
		bOutOnMapProjection = false;
	}

	Int32 nSegIndex = -1;
	BaseAlgorithm::GetNearestPntToLineset(pt, lr->GetCoordinates(), pntProject, nSegIndex);
	return bOutOnMapProjection;
}

Bool GeometryAlgorithm::RoadWidth(
	const Engine::Geometries::LineString *pLineStringL,
	const Engine::Geometries::LineString *pLineStringR,
	Double &sWidth,
	Double &eWidth,
	Double &averageWidth)
{
	Array<Engine::Geometries::Coordinate *> *pCoordinatesL =
		pLineStringL->GetCoordinates();
	Array<Engine::Geometries::Coordinate *> *pCoordinatesR =
		pLineStringR->GetCoordinates();

	if ((pCoordinatesL == NULL) || (pCoordinatesR == NULL))
	{
		return false;
	}

	Int32 ncoordsL = pCoordinatesL->GetCount();
	Int32 ncoordsR = pCoordinatesR->GetCount();

	if ((ncoordsL < 2) || (ncoordsR < 2))
	{
		return false;
	}

	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	Double totalWidth = 0.0;
	Bool b = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesL->GetAt(0),
												   pLineStringR->GetCoordinates(), pntProject, nSegIndex);

	if (b)
	{
		sWidth = BaseAlgorithm::DistancePtToPt(pCoordinatesL->GetAt(0), &pntProject);
	}

	b = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesL->GetAt(ncoordsL - 1),
											  pLineStringR->GetCoordinates(), pntProject, nSegIndex);

	if (b)
	{
		eWidth = BaseAlgorithm::DistancePtToPt(
			pCoordinatesL->GetAt(ncoordsL - 1), &pntProject);
	}

	Int32 i = 1;

	Array<Double> arrLWidth;
	Array<Double> arrRWidth;

	arrLWidth.Add(sWidth);
	for (i = 1; i < ncoordsL - 1; i++)
	{
		b = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesL->GetAt(i),
												  pLineStringR->GetCoordinates(), pntProject, nSegIndex);

		if (b)
		{
			Double dwidth = BaseAlgorithm::DistancePtToPt(
				pCoordinatesL->GetAt(i), &pntProject);
			arrLWidth.Add(dwidth);
		}
	}
	arrLWidth.Add(eWidth);

	for (i = 0; i < ncoordsR; i++)
	{
		b = BaseAlgorithm::GetNearestPntToLineset(pCoordinatesR->GetAt(i),
												  pLineStringL->GetCoordinates(), pntProject, nSegIndex);

		if (b)
		{
			Double dwidth = BaseAlgorithm::DistancePtToPt(
				pCoordinatesR->GetAt(i), &pntProject);
			arrRWidth.Add(dwidth);
		}
	}
	// modify by douhang 19/6/2017 for requirement 1587
	const double dThreshold = 0.1;
	if (arrLWidth.GetCount() > 2)
	{
		for (auto itr = arrLWidth.Begin() + 1; itr != arrLWidth.End() - 1;)
		{
			if (((*(itr) - *(itr - 1)) >= dThreshold && (*(itr) - *(itr + 1)) >= dThreshold) ||
				((*(itr) - *(itr - 1)) <= -dThreshold && (*(itr) - *(itr + 1)) <= -dThreshold))
			{
				itr = arrLWidth.Erase(itr);
			}
			else
			{
				itr++;
			}
		}
	}
	if (arrRWidth.GetCount() > 2)
	{
		for (auto itr = arrRWidth.Begin() + 1; itr != arrRWidth.End() - 1;)
		{
			if (((*(itr) - *(itr - 1)) >= dThreshold && (*(itr) - *(itr + 1)) >= dThreshold) ||
				((*(itr) - *(itr - 1)) <= -dThreshold && (*(itr) - *(itr + 1)) <= -dThreshold))
			{
				itr = arrRWidth.Erase(itr);
			}
			else
			{
				itr++;
			}
		}
	}
	for (auto itr = arrRWidth.Begin(); itr != arrRWidth.End(); itr++)
	{
		totalWidth += (*itr);
	}
	for (auto itr = arrLWidth.Begin(); itr != arrLWidth.End(); itr++)
	{
		totalWidth += (*itr);
	}
	averageWidth = totalWidth / (Double(arrRWidth.GetCount() + arrLWidth.GetCount()));

	Int32 nMod = Int32(sWidth * 40);

	if ((nMod % 2) == 0)
	{
		sWidth = ((Double)nMod) / 40.0;
	}
	else
	{
		sWidth = ((Double)nMod + 1.0) / 40.0;
	}

	nMod = Int32(eWidth * 40);

	if ((nMod % 2) == 0)
	{
		eWidth = ((Double)nMod) / 40.0;
	}
	else
	{
		eWidth = ((Double)nMod + 1.0) / 40;
	}

	nMod = Int32(averageWidth * 40);

	if ((nMod % 2) == 0)
	{
		averageWidth = ((Double)nMod) / 40.0;
	}
	else
	{
		averageWidth = ((Double)nMod + 1.0) / 40.0;
	}

	return true;
}

// SizeT GeometryAlgorithm::DistanceLineToLine(
//	const Engine::Geometries::Point* ptStart,
//	const Engine::Geometries::Point* ptEnd,
//	const Engine::Geometries::LineString* line)
//{
//	return 0.0;
// }

Bool GeometryAlgorithm::MinTurnAngleLessThan(const Engine::Geometries::LineString *pLineString, const Double dLimitAngle)
{
	if (NULL == pLineString)
		return false;

	const Array<Coordinate *> *pCoordinates = pLineString->GetCoordinates();
	if (NULL == pCoordinates)
		return false;

	SizeT nPoints = pCoordinates->GetCount();
	for (SizeT i = 1; i < nPoints - 1; i++)
	{
		const Coordinate *sCoor = pCoordinates->ElementAt(i - 1);
		const Coordinate *mCoor = pCoordinates->ElementAt(i);
		const Coordinate *eCoor = pCoordinates->ElementAt(i + 1);
		Double dAngle = RadianToDegree(ComputeAngle(*sCoor, *mCoor, *eCoor));

		if (dAngle < dLimitAngle)
			return true;
	}

	return false;
}

Bool GeometryAlgorithm::GetIndexInLines(const Array<Engine::Geometries::LineString *> &arrLines, Array<int> &arrIndexs)
{
	Int32 nLineCount = arrLines.GetCount();
	if (nLineCount < 1)
	{
		return false;
	}

	arrIndexs.SetSize(nLineCount);

	for (Int32 i = 0; i < nLineCount; i++)
	{
		Int32 nIndex = GetIndexInLines(i, arrLines);

		Bool bRepeated = true;
		while (bRepeated)
		{
			bRepeated = false;

			for (Int32 j = 0; j < i; j++)
			{
				if (arrIndexs[j] == nIndex)
				{
					nIndex += 1;

					bRepeated = true;
					break;
				}
			}
		}

		arrIndexs[i] = nIndex;
	}

	return true;
}

Int32 GeometryAlgorithm::GetIndexInLines(Int32 nIndex, const Array<Engine::Geometries::LineString *> &arrLines)
{
	Int32 nLineCount = arrLines.GetCount();

	if ((nLineCount == 1) && (nIndex == 0)) // 序号正确
	{
		return 1;
	}
	else if ((nLineCount == 1) && (nIndex != 0)) // 序号乱了
	{
		return -1;
	}

	if ((nIndex >= nLineCount) || (nIndex < 0))
	{
		return -1;
	}

	Engine::Geometries::LineString *pLine = arrLines.GetAt(nIndex);
	if (pLine == NULL)
	{
		return -1;
	}

	// 被计算线的中间点
	Array<Engine::Geometries::Coordinate *> *pPoints = pLine->GetCoordinates();
	Int32 PointCount = pPoints->GetCount();
	Int32 nCenterIndex = Int32(PointCount / 2);

	Engine::Geometries::Coordinate *pPoint = new Engine::Geometries::Coordinate();
	if ((nCenterIndex > 0) && (nCenterIndex < (PointCount - 1))) // 非边界点
	{
		pPoint->x = (*pPoints)[nCenterIndex]->x;
		pPoint->y = (*pPoints)[nCenterIndex]->y;
	}
	else if (nCenterIndex == 0)
	{
		pPoint->x = ((*pPoints)[nCenterIndex]->x + (*pPoints)[nCenterIndex + 1]->x) / 2;
		pPoint->y = ((*pPoints)[nCenterIndex]->y + (*pPoints)[nCenterIndex + 1]->y) / 2;
	}
	else
	{
		pPoint->x = ((*pPoints)[nCenterIndex]->x + (*pPoints)[nCenterIndex - 1]->x) / 2;
		pPoint->y = ((*pPoints)[nCenterIndex]->y + (*pPoints)[nCenterIndex - 1]->y) / 2;
	}

	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;

	Int32 nResultIndex = 1;
	Int32 i = 0;
	for (i = 0; i < nLineCount; i++)
	{
		if (i != nIndex)
		{
			Engine::Geometries::LineString *ls = arrLines.GetAt(i);
			pPoints = ls->GetCoordinates();
			BaseAlgorithm::GetNearestPntToLineset(pPoint, pPoints, pntProject, nSegIndex);
			Engine::Geometries::Coordinate *pntFrom = (*pPoints)[nSegIndex];
			Engine::Geometries::Coordinate *pntTo = (*pPoints)[nSegIndex + 1];

			// 在其他标线的右侧，序号加1
			if (BaseAlgorithm::PntMatchLine(*pntFrom, *pntTo, *pPoint) == 2)
			{
				nResultIndex++;
			}
		}
	}

	delete pPoint;
	pPoint = NULL;
	return nResultIndex;
}

Int32 GeometryAlgorithm::GetIndexInLinesByPrj(Int32 nIndex, const Array<Engine::Geometries::LineString *> &arrLines)
{
	Int32 nLineCount = arrLines.GetCount();

	if ((nLineCount == 1) && (nIndex == 0)) // 序号正确
	{
		return 1;
	}
	else if ((nLineCount == 1) && (nIndex != 0)) // 序号乱了
	{
		return -1;
	}

	if ((nIndex >= nLineCount) || (nIndex < 0))
	{
		return -1;
	}

	Engine::Geometries::LineString *pLine = arrLines.GetAt(nIndex);
	if (pLine == NULL)
	{
		return -1;
	}

	// 被计算线的中间点
	Array<Engine::Geometries::Coordinate *> *pPoints = pLine->GetCoordinates();
	Int32 PointCount = pPoints->GetCount();

	Engine::Geometries::Coordinate *pPoint = new Engine::Geometries::Coordinate();

	Base::Double dDis = 9999.0;
	Geometries::Coordinate pntProject;
	Base::Int32 index;

	for (int j = 0; j < nLineCount; j++)
	{
		if (j != nIndex)
		{
			for (int i = 0; i < PointCount; i++)
			{
				double dis = Engine::Geometries::BaseAlgorithm3D::GetDistancePointToLinesegments(*pPoints->GetAt(i), arrLines.GetAt(j)->GetCoordinates(), pntProject, index);
				if (dis < dDis)
				{
					dDis = dis;
					pPoint->x = pPoints->GetAt(i)->x;
					pPoint->y = pPoints->GetAt(i)->y;
					pPoint->z = pPoints->GetAt(i)->z;
				}
			}
			if (dDis != 9999.0)
			{
				break;
			}
		}
	}

	Int32 nSegIndex = -1;

	Int32 nResultIndex = 1;
	Int32 i = 0;
	for (i = 0; i < nLineCount; i++)
	{
		if (i != nIndex)
		{
			Engine::Geometries::LineString *ls = arrLines.GetAt(i);
			pPoints = ls->GetCoordinates();
			BaseAlgorithm::GetNearestPntToLineset(pPoint, pPoints, pntProject, nSegIndex);
			Engine::Geometries::Coordinate *pntFrom = (*pPoints)[nSegIndex];
			Engine::Geometries::Coordinate *pntTo = (*pPoints)[nSegIndex + 1];

			// 在其他标线的右侧，序号加1
			if (BaseAlgorithm::PntMatchLine(*pntFrom, *pntTo, *pPoint) == 2)
			{
				nResultIndex++;
			}
		}
	}

	delete pPoint;
	pPoint = NULL;
	return nResultIndex;
}

Bool GeometryAlgorithm::EdgeMatch(
	Engine::Geometries::Geometry *&geometry1,
	Engine::Geometries::Geometry *&geometry2,
	Double dTolerance, Double &dMinDis, Bool bImposeMatch)
{
	if ((geometry1 == NULL) || (geometry2 == NULL) || (dTolerance <= 0.0))
	{
		return false;
	}

	Bool bIsEdgeMatch = false;

	if (geometry1->GetGeometryType() == GeometryType::POLYGON && geometry2->GetGeometryType() == GeometryType::POLYGON)
	{
		Engine::Geometries::Polygon *polygon1 = dynamic_cast<Engine::Geometries::Polygon *>(geometry1);
		Engine::Geometries::Polygon *polygon2 = dynamic_cast<Engine::Geometries::Polygon *>(geometry2);

		if (polygon1 == NULL || polygon2 == NULL)
		{
			return bIsEdgeMatch;
		}

		LinearRing *lr1 = polygon1->GetExteriorRing();
		LinearRing *lr2 = polygon2->GetExteriorRing();

		if (lr1 == NULL || lr2 == NULL)
		{
			return bIsEdgeMatch;
		}

		if (lr1->GetNumPoints() < 4 || lr2->GetNumPoints() < 4)
		{
			return bIsEdgeMatch;
		}

		dMinDis = -1.0;
		Int32 nIndex1 = -1;
		Int32 nIndex2 = -1;
		Int32 i = 0, j = 0;

		for (i = 0; i < lr1->GetNumPoints() - 1; i++)
		{
			Coordinate *coordinate1 = lr1->GetCoordinateN(i);

			for (j = 0; j < lr2->GetNumPoints() - 1; j++)
			{
				Coordinate *coordinate2 = lr2->GetCoordinateN(j);
				Double dDis = coordinate1->Distance(*coordinate2);

				if ((dMinDis < 0.0) || (dDis < dMinDis))
				{
					dMinDis = dDis;
					nIndex1 = i;
					nIndex2 = j;
				}
			}
		}

		if ((dMinDis > dTolerance) && (!bImposeMatch))
		{
			return false;
		}

		Coordinate *coordinate1Prev = NULL;
		Coordinate *coordinate1Next = NULL;
		Coordinate *coordinate2Prev = NULL;
		Coordinate *coordinate2Next = NULL;

		if (nIndex1 == 0)
		{
			coordinate1Prev = lr1->GetCoordinateN(lr1->GetNumPoints() - 2);
		}
		else
		{
			coordinate1Prev = lr1->GetCoordinateN(nIndex1 - 1);
		}

		if (nIndex1 + 1 == lr1->GetNumPoints() - 1)
		{
			coordinate1Next = lr1->GetCoordinateN(0);
		}
		else
		{
			coordinate1Next = lr1->GetCoordinateN(nIndex1 + 1);
		}

		if (nIndex2 == 0)
		{
			coordinate2Prev = lr2->GetCoordinateN(lr2->GetNumPoints() - 2);
		}
		else
		{
			coordinate2Prev = lr2->GetCoordinateN(nIndex2 - 1);
		}

		if (nIndex2 + 1 == lr2->GetNumPoints() - 1)
		{
			coordinate2Next = lr2->GetCoordinateN(0);
		}
		else
		{
			coordinate2Next = lr2->GetCoordinateN(nIndex2 + 1);
		}

		Double dPrev1Prev2 = coordinate1Prev->Distance(*coordinate2Prev);
		Double dPrev1Next2 = coordinate1Prev->Distance(*coordinate2Next);
		Double dNext1Prev2 = coordinate1Next->Distance(*coordinate2Prev);
		Double dNext1Next2 = coordinate1Next->Distance(*coordinate2Next);
		Double dMinDis1 = dPrev1Prev2;
		Coordinate *coordinate1N = coordinate1Prev;
		Coordinate *coordinate2N = coordinate2Prev;

		if (dMinDis1 > dPrev1Next2)
		{
			coordinate1N = coordinate1Prev;
			coordinate2N = coordinate2Next;
			dMinDis1 = dPrev1Next2;
		}

		if (dMinDis1 > dNext1Prev2)
		{
			coordinate1N = coordinate1Next;
			coordinate2N = coordinate2Prev;
			dMinDis1 = dNext1Prev2;
		}

		if (dMinDis1 > dNext1Next2)
		{
			coordinate1N = coordinate1Next;
			coordinate2N = coordinate2Next;
			dMinDis1 = dNext1Next2;
		}

		if ((dMinDis1 > dTolerance) && (!bImposeMatch))
		{
			return false;
		}

		Coordinate *coordinate1 = lr1->GetCoordinateN(nIndex1);
		Coordinate *coordinate2 = lr2->GetCoordinateN(nIndex2);
		Coordinate coordTemp = ((*coordinate1) + (*coordinate2)) / 2;
		(*coordinate1) = coordTemp;
		(*coordinate2) = coordTemp;
		coordTemp = ((*coordinate1N) + (*coordinate2N)) / 2;
		(*coordinate1N) = coordTemp;
		(*coordinate2N) = coordTemp;
		Coordinate *coordinateEnd1 = lr1->GetCoordinateN(lr1->GetNumPoints() - 1);
		Coordinate *coordinateStart1 = lr1->GetCoordinateN(0);
		(*coordinateEnd1) = *coordinateStart1;
		Coordinate *coordinateEnd2 = lr2->GetCoordinateN(lr2->GetNumPoints() - 1);
		Coordinate *coordinateStart2 = lr2->GetCoordinateN(0);
		(*coordinateEnd2) = *coordinateStart2;
		return true;
	}
	else if (geometry1->GetGeometryType() == GeometryType::MULTILINESTRING && geometry2->GetGeometryType() == GeometryType::MULTILINESTRING)
	{
		Engine::Geometries::MultiLineString *multiLineString1 = dynamic_cast<Engine::Geometries::MultiLineString *>(geometry1);
		Engine::Geometries::MultiLineString *multiLineString2 = dynamic_cast<Engine::Geometries::MultiLineString *>(geometry2);

		if (multiLineString1 == NULL || multiLineString2 == NULL)
		{
			return bIsEdgeMatch;
		}

		LineString *ls = NULL;
		Coordinate *pCoordinate = NULL;
		Array<Coordinate *> coordinates1;
		Array<Coordinate *> coordinates2;

		Int32 nLineCount1 = multiLineString1->GetNumGeometries();
		Int32 nLineCount2 = multiLineString2->GetNumGeometries();
		Int32 i = 0, j = 0;

		for (i = 0; i < nLineCount1; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(i));

			for (j = 0; j < ls->GetNumPoints() - 1; j++)
			{
				pCoordinate = ls->GetCoordinateN(j);
				coordinates1.Add(pCoordinate);
			}
		}

		for (i = 0; i < nLineCount2; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(i));

			for (j = 0; j < ls->GetNumPoints() - 1; j++)
			{
				pCoordinate = ls->GetCoordinateN(j);
				coordinates2.Add(pCoordinate);
			}
		}

		dMinDis = -1.0;
		Int32 nIndex1 = -1;
		Int32 nIndex2 = -1;

		Coordinate *pCoordinate1;
		Coordinate *pCoordinate2;

		for (i = 0; i < coordinates1.GetCount(); i++)
		{
			pCoordinate1 = coordinates1.GetAt(i);

			for (j = 0; j < coordinates2.GetCount(); j++)
			{
				pCoordinate2 = coordinates2.GetAt(j);
				Double dDis = pCoordinate1->Distance(*pCoordinate2);

				if ((dMinDis < 0.0) || (dDis < dMinDis))
				{
					dMinDis = dDis;
					nIndex1 = i;
					nIndex2 = j;
				}
			}
		}

		if ((dMinDis > dTolerance) && (!bImposeMatch))
		{
			return false;
		}

		Coordinate *coordinate1Prev = NULL;
		Coordinate *coordinate1Next = NULL;
		Coordinate *coordinate2Prev = NULL;
		Coordinate *coordinate2Next = NULL;

		if (nIndex1 == 0)
		{
			coordinate1Prev = coordinates1.GetAt(coordinates1.GetCount() - 1);
		}
		else
		{
			coordinate1Prev = coordinates1.GetAt(nIndex1 - 1);
		}

		if (nIndex1 == coordinates1.GetCount() - 1)
		{
			coordinate1Next = coordinates1.GetAt(0);
		}
		else
		{
			coordinate1Next = coordinates1.GetAt(nIndex1 + 1);
		}

		if (nIndex2 == 0)
		{
			coordinate2Prev = coordinates2.GetAt(coordinates2.GetCount() - 1);
		}
		else
		{
			coordinate2Prev = coordinates2.GetAt(nIndex2 - 1);
		}

		if (nIndex2 == coordinates2.GetCount() - 1)
		{
			coordinate2Next = coordinates2.GetAt(0);
		}
		else
		{
			coordinate2Next = coordinates2.GetAt(nIndex2 + 1);
		}

		Double dPrev1Prev2 = coordinate1Prev->Distance(*coordinate2Prev);
		Double dPrev1Next2 = coordinate1Prev->Distance(*coordinate2Next);
		Double dNext1Prev2 = coordinate1Next->Distance(*coordinate2Prev);
		Double dNext1Next2 = coordinate1Next->Distance(*coordinate2Next);
		Double dMinDis1 = dPrev1Prev2;
		Coordinate *coordinate1N = coordinate1Prev;
		Coordinate *coordinate2N = coordinate2Prev;

		if (dMinDis1 > dPrev1Next2)
		{
			coordinate1N = coordinate1Prev;
			coordinate2N = coordinate2Next;
			dMinDis1 = dPrev1Next2;
		}

		if (dMinDis1 > dNext1Prev2)
		{
			coordinate1N = coordinate1Next;
			coordinate2N = coordinate2Prev;
			dMinDis1 = dNext1Prev2;
		}

		if (dMinDis1 > dNext1Next2)
		{
			coordinate1N = coordinate1Next;
			coordinate2N = coordinate2Next;
			dMinDis1 = dNext1Next2;
		}

		if ((dMinDis1 > dTolerance) && (!bImposeMatch))
		{
			return false;
		}

		Coordinate *coordinate1 = coordinates1.GetAt(nIndex1);
		Coordinate *coordinate2 = coordinates2.GetAt(nIndex2);
		Coordinate coordTemp = ((*coordinate1) + (*coordinate2)) / 2;
		(*coordinate1) = coordTemp;
		(*coordinate2) = coordTemp;
		coordTemp = ((*coordinate1N) + (*coordinate2N)) / 2;
		(*coordinate1N) = coordTemp;
		(*coordinate2N) = coordTemp;
		Coordinate *coordinateEnd = NULL;
		Coordinate *coordinateStart = NULL;

		for (i = 0; i < nLineCount1; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(i));
			coordinateEnd = ls->GetCoordinateN(ls->GetNumPoints() - 1);

			if (i == (nLineCount1 - 1))
			{
				ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(0));
			}
			else
			{
				ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(i + 1));
			}

			coordinateStart = ls->GetCoordinateN(0);
			(*coordinateEnd) = *coordinateStart;
		}

		for (i = 0; i < nLineCount2; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(i));
			coordinateEnd = ls->GetCoordinateN(ls->GetNumPoints() - 1);

			if (i == (nLineCount2 - 1))
			{
				ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(0));
			}
			else
			{
				ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(i + 1));
			}

			coordinateStart = ls->GetCoordinateN(0);
			(*coordinateEnd) = *coordinateStart;
		}

		return true;
	}

	return bIsEdgeMatch;
}

// 0 成功 其他失败 1 对象几何数据为空 2 点顺序错误
Int32 GeometryAlgorithm::EdgeUnion(Engine::Geometries::Geometry *geometry1,
								   Engine::Geometries::Geometry *geometry2)
{
	if ((geometry1 == NULL) || (geometry2 == NULL))
	{
		return 1;
	}

	if (geometry1->GetGeometryType() == GeometryType::MULTILINESTRING && geometry2->GetGeometryType() == GeometryType::MULTILINESTRING)
	{
		Engine::Geometries::MultiLineString *multiLineString1 = dynamic_cast<Engine::Geometries::MultiLineString *>(geometry1);
		Engine::Geometries::MultiLineString *multiLineString2 = dynamic_cast<Engine::Geometries::MultiLineString *>(geometry2);

		if (multiLineString1 == NULL || multiLineString2 == NULL)
		{
			return 1;
		}

		LineString *ls = NULL;
		Coordinate *pCoordinate = NULL;
		Array<Coordinate *> coordinates1;
		Array<Coordinate *> coordinates2;

		Int32 nLineCount1 = multiLineString1->GetNumGeometries();
		Int32 nLineCount2 = multiLineString2->GetNumGeometries();
		Int32 i = 0, j = 0;

		for (i = 0; i < nLineCount1; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(i));

			pCoordinate = ls->GetCoordinateN(0);
			coordinates1.Add(pCoordinate);
		}

		for (i = 0; i < nLineCount2; i++)
		{
			ls = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(i));

			pCoordinate = ls->GetCoordinateN(0);
			coordinates2.Add(pCoordinate);
		}

		Double dMinDis = -1.0;
		Int32 nIndex1From = -1;
		Int32 nIndex2From = -1;

		Coordinate *pCoordinate1 = NULL;
		Coordinate *pCoordinate2 = NULL;

		for (i = 0; i < coordinates1.GetCount(); i++)
		{
			pCoordinate1 = coordinates1.GetAt(i);

			for (j = 0; j < coordinates2.GetCount(); j++)
			{
				pCoordinate2 = coordinates2.GetAt(j);
				Double dDis = pCoordinate1->Distance(*pCoordinate2);

				if ((dMinDis < 0.0) || (dDis < dMinDis))
				{
					dMinDis = dDis;
					nIndex1From = i;
					nIndex2From = j;
				}
			}
		}

		Int32 nIndex1To = -1;
		Int32 nIndex2To = -1;
		dMinDis = -1.0;

		for (i = 0; i < coordinates1.GetCount(); i++)
		{
			if (i == nIndex1From)
			{
				continue;
			}

			pCoordinate1 = coordinates1.GetAt(i);

			for (j = 0; j < coordinates2.GetCount(); j++)
			{
				if (j == nIndex2From) // 过滤
				{
					continue;
				}

				pCoordinate2 = coordinates2.GetAt(j);
				Double dDis = pCoordinate1->Distance(*pCoordinate2);

				if ((dMinDis < 0.0) || (dDis < dMinDis))
				{
					dMinDis = dDis;
					nIndex1To = i;
					nIndex2To = j;
				}
			}
		}

		// 返回错误的情况
		if ((abs(nIndex1To - nIndex1From) != 1) && (abs(nIndex1To - nIndex1From) != 3))
		{
			return 2;
		}

		if ((abs(nIndex2To - nIndex2From) != 1) && (abs(nIndex2To - nIndex2From) != 3))
		{
			return 2;
		}

		Int32 nDeleteEdgeIndex1 = nIndex1From;
		Int32 nDeleteEdgeIndex2 = nIndex2From;

		if (((nIndex1From > nIndex1To) && (nIndex1From != (nIndex1To + 3))) || ((nIndex1From + 3) == nIndex1To))
		{
			nDeleteEdgeIndex1 = nIndex1To;
		}

		if (((nIndex2From > nIndex2To) && (nIndex2From != (nIndex2To + 3))) || ((nIndex2From + 3) == nIndex2To))
		{
			nDeleteEdgeIndex2 = nIndex2To;
		}

		Int32 nModifyEdgeIndex1 = nDeleteEdgeIndex1 - 1;
		Int32 nModifyEdgeIndex2 = nDeleteEdgeIndex1 + 1;

		if (nModifyEdgeIndex1 < 0)
		{
			nModifyEdgeIndex1 = 3;
		}

		if (nModifyEdgeIndex2 > 3)
		{
			nModifyEdgeIndex2 = 0;
		}

		Int32 nAddEdgeIndex1 = nDeleteEdgeIndex2 + 1;
		Int32 nAddEdgeIndex2 = nDeleteEdgeIndex2 - 1;

		if (nAddEdgeIndex1 > 3)
		{
			nAddEdgeIndex1 = 0;
		}

		if (nAddEdgeIndex2 < 0)
		{
			nAddEdgeIndex2 = 3;
		}

		Int32 nSaveEdgeIndex = nAddEdgeIndex2 - 1;
		if (nSaveEdgeIndex < 0)
		{
			nSaveEdgeIndex = 3;
		}

		// 修改nModifyEdgeIndex1
		LineString *ls1 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(nModifyEdgeIndex1));
		LineString *ls2 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(nAddEdgeIndex1));

		Array<Engine::Geometries::Coordinate *> *pPoints1 = ls1->GetCoordinates();
		Array<Engine::Geometries::Coordinate *> *pPoints2 = ls2->GetCoordinates();

		Int32 nPointCount1 = pPoints1->GetCount();
		Int32 nPointCount2 = pPoints2->GetCount();

		pCoordinate1 = pPoints1->GetAt(nPointCount1 - 2);
		pCoordinate2 = pPoints1->GetAt(nPointCount1 - 1);

		Double dDis = pCoordinate1->Distance(*pCoordinate2);

		Bool bAdd = false;
		for (i = 0; i < nPointCount2; i++)
		{
			pCoordinate2 = pPoints2->GetAt(i);
			if (pCoordinate1->Distance(*pCoordinate2) > dDis)
			{
				bAdd = true;
			}

			if (bAdd)
			{
				Coordinate *coordinate = new Coordinate(*pCoordinate2);
				pPoints1->Add(coordinate);
			}
		}
		ls1->Distinct();

		// 修改nModifyEdgeIndex2
		ls1 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(nModifyEdgeIndex2));
		ls2 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(nAddEdgeIndex2));

		pPoints1 = ls1->GetCoordinates();
		pPoints2 = ls2->GetCoordinates();

		nPointCount1 = pPoints1->GetCount();
		nPointCount2 = pPoints2->GetCount();

		pCoordinate1 = pPoints1->GetAt(1);
		pCoordinate2 = pPoints1->GetAt(0);

		dDis = pCoordinate1->Distance(*pCoordinate2);

		bAdd = false;
		for (i = nPointCount2 - 1; i >= 0; i--)
		{
			pCoordinate2 = pPoints2->GetAt(i);
			if (pCoordinate1->Distance(*pCoordinate2) > dDis)
			{
				bAdd = true;
			}

			if (bAdd)
			{
				Coordinate *coordinate = new Coordinate(*pCoordinate2);
				pPoints1->InsertAt(0, coordinate);
			}
		}
		ls1->Distinct();

		ls1 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString1->GetGeometryN(nDeleteEdgeIndex1));
		ls2 = dynamic_cast<Engine::Geometries::LineString *>(multiLineString2->GetGeometryN(nSaveEdgeIndex));

		*ls1 = *ls2;
	}

	return 0;
}

Bool GeometryAlgorithm::PointsMatch(Engine::Base::Array<Engine::Geometries::Coordinate *> *pCoordinates, const Engine::Base::Array<Engine::Geometries::LineString *> &arrMarkLines)
{
	if (pCoordinates == NULL)
	{
		return false;
	}

	int nLineCount = arrMarkLines.GetCount();

	if (nLineCount == 0)
	{
		return false;
	}

	SizeT nPointCount = pCoordinates->GetCount();
	Int32 i = 0, j = 0;
	Coordinate *pPoint = NULL;
	LineString *ls = NULL;

	Coordinate pntProject;
	Int32 nSegIndex;
	Double dDstZ = 0.0;
	Bool bHavePrj = false;

	for (i = 0; i < nPointCount; i++)
	{
		pPoint = pCoordinates->ElementAt(i);
		Double dMinDis = -1.0;

		for (j = 0; j < nLineCount; j++)
		{
			ls = arrMarkLines.GetAt(j);
			if (ls == NULL || ls->GetNumPoints() < 2) // by lizhao 2017.3.6
				continue;

			Double dDis = BaseAlgorithm3D::GetDistancePointToLinesegments(*pPoint, ls->GetCoordinates(), pntProject, nSegIndex);

			// 避免出问题	zhangliqun 2020.1.5
			if (dDis < 0)
				continue;

			if ((dMinDis < 0.0) || (dMinDis > dDis))
			{
				// 有时杆底部的点确实比抓地后的点要低，抓地后的z值减去2米需要比杆底部z值低，从根本上避免了抓地时抓到头上路面的情况发生
				if (pntProject.z - 2 < pPoint->z)
				{
					// pPoint->z = pntProject.z;
					bHavePrj = true;
					dDstZ = pntProject.z;
					dMinDis = dDis;
				}
			}
		}

		if (bHavePrj)
		{
			pPoint->z = dDstZ;
			dDstZ = 0.0;
			bHavePrj = false;
		}
	}

	return true;
}

Void GeometryAlgorithm::ClipLineStringByPolygon(Geometries::LineString &lineString, Geometries::Polygon &polygon,
												Base::Array<Geometries::LineString *> &vecClipResult)
{
	geos::geom::Geometry *pGeosPolygon = GeomAdapter::ToGeosGeom(&polygon);
	geos::geom::Geometry *pGeosLineString = GeomAdapter::ToGeosGeom(&lineString);

	if (NULL == pGeosPolygon || NULL == pGeosLineString)
	{
		//////////add llj 2018-12-6
		if (pGeosPolygon)
			geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosPolygon);
		if (pGeosLineString)
			geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosLineString);
		//////////
		return;
	}

	vecClipResult.Clear();

	// 求交集
	geos::geom::Geometry *pIntersectionGeos = pGeosLineString->intersection(pGeosPolygon).release();

	switch (pIntersectionGeos->getGeometryTypeId())
	{
	case geos::geom::GEOS_MULTILINESTRING:
	case geos::geom::GEOS_GEOMETRYCOLLECTION:
	{
		geos::geom::GeometryCollection *pMultiLineString = dynamic_cast<geos::geom::GeometryCollection *>(pIntersectionGeos);
		for (Int32 i = 0, n = pMultiLineString->getNumGeometries(); i < n; i++)
		{
			const geos::geom::Geometry *pSubGeometry = pMultiLineString->getGeometryN(i);
			if (geos::geom::GEOS_LINESTRING != pSubGeometry->getGeometryTypeId())
				continue;

			Geometries::Geometry *pNIGeometry = GeomAdapter::ToGeometry(pSubGeometry);
			if (pNIGeometry->GetGeometryType() != Geometries::GeometryType::LINESTRING)
				continue;

			Geometries::LineString *pNILineString = dynamic_cast<Geometries::LineString *>(pNIGeometry);
			vecClipResult.Add(pNILineString);
		}
	}
	break;

	case geos::geom::GEOS_LINESTRING:
	{
		geos::geom::LineString *pLineString = dynamic_cast<geos::geom::LineString *>(pIntersectionGeos);
		Geometries::Geometry *pNIGeometry = GeomAdapter::ToGeometry(pLineString);
		Geometries::LineString *pNILineString = dynamic_cast<Geometries::LineString *>(pNIGeometry);
		vecClipResult.Add(pNILineString);
	}
	break;

	default:
		break;
	}

	// destroy
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosPolygon);
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosLineString);
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pIntersectionGeos);

	// 还原Z值
	for (Int32 i = 0, n = vecClipResult.GetCount(); i < n; i++)
	{
		Geometries::LineString *pLineString = vecClipResult[i];
		auto m = pLineString->GetNumPoints();
		if (m > 1)
		{
			Geometries::Coordinate *pFrontCoordinate = pLineString->GetCoordinateN(0);
			Geometries::Coordinate *pBackCoordinate = pLineString->GetCoordinateN(m - 1);

			double dDistanceTemp = -1.0;
			Engine::Geometries::Coordinate pntProjecttemp;

			Array<Double> front_z_candiates, back_z_candidates;

			for (int k = 0; k < lineString.GetNumPoints() - 1; k++)
			{
				Engine::Geometries::Coordinate *pntFrom = lineString.GetCoordinateN(k);
				Engine::Geometries::Coordinate *pntTo = lineString.GetCoordinateN(k + 1);

				// front point
				if (BaseAlgorithm::IsProjectToLineset(pFrontCoordinate, pntFrom, pntTo))
				{
					double distanceXy = pFrontCoordinate->DistanceXY(*pntFrom);
					if (distanceXy < Geometries_EP)
					{
						front_z_candiates.Add(pntFrom->z);
					}
					else
					{
						Vector3d vDirect = (*pntTo) - (*pntFrom);
						pntProjecttemp = (*pntFrom) + vDirect * distanceXy / (pntFrom->DistanceXY(*pntTo));
						dDistanceTemp = BaseAlgorithm::DistancePtToPt(&pntProjecttemp, pFrontCoordinate);

						if (dDistanceTemp < Geometries_EP)
						{
							front_z_candiates.Add(pntProjecttemp.z);
						}
					}
				}

				// back point
				if (BaseAlgorithm::IsProjectToLineset(pBackCoordinate, pntFrom, pntTo))
				{
					double distanceXy = pBackCoordinate->DistanceXY(*pntFrom);
					if (distanceXy < Geometries_EP)
					{
						back_z_candidates.Add(pntFrom->z);
					}
					else
					{
						Vector3d vDirect = (*pntTo) - (*pntFrom);
						pntProjecttemp = (*pntFrom) + vDirect * distanceXy / (pntFrom->DistanceXY(*pntTo));
						dDistanceTemp = BaseAlgorithm::DistancePtToPt(&pntProjecttemp, pBackCoordinate);

						if (dDistanceTemp < Geometries_EP)
						{
							back_z_candidates.Add(pntProjecttemp.z);
						}
					}
				}
			} // all linestring's vertexs

			// FRONT Z
			Double min_z_diff = std::numeric_limits<Double>::max();
			Int32 index = -1;

			for (auto j = 0; j < front_z_candiates.GetCount(); j++)
			{
				Double z_diff = fabs(front_z_candiates[j] - pLineString->GetCoordinateN(1)->z);
				if (z_diff < min_z_diff)
				{
					min_z_diff = z_diff;
					index = j;
				}
			}

			if ((!front_z_candiates.IsEmpty()) && (index > -1))
			{
				pFrontCoordinate->z = front_z_candiates[index];
			}

			// BACK Z
			min_z_diff = std::numeric_limits<Double>::max();
			index = -1;

			for (auto j = 0; j < back_z_candidates.GetCount(); j++)
			{
				Double z_diff = fabs(back_z_candidates[j] - pLineString->GetCoordinateN(m - 2)->z);
				if (z_diff < min_z_diff)
				{
					min_z_diff = z_diff;
					index = j;
				}
			}

			if ((!back_z_candidates.IsEmpty()) && (index > -1))
			{
				pBackCoordinate->z = back_z_candidates[index];
			}
		}
	}
}

Base::Int32 GeometryAlgorithm::LineOrientatedWithLine(Geometries::LineString *lhs, Geometries::LineString *rhs)
{
	Base::Array<Geometries::LineString *> lineStrings;
	lineStrings.Add(lhs);
	lineStrings.Add(rhs);

	return GetIndexInLines(0, lineStrings);
}

Base::Int32 GeometryAlgorithm::LineOrientatedWithLineByProj(Geometries::LineString *lhs, Geometries::LineString *rhs)
{
	if (rhs == nullptr || lhs == nullptr)
	{
		return 0;
	}
	Array<Int32> arrDir;

	Array<Engine::Geometries::Coordinate *> *lCoords = lhs->GetCoordinates();
	Int32 PointCount = lCoords->GetCount();
	Array<Engine::Geometries::Coordinate *> *rCoords = rhs->GetCoordinates();
	Base::Double dDis = 9999.0;
	Geometries::Coordinate pntProject;
	Base::Int32 index = 0;

	// 1、计算rhs上两点在lhs的投影点及形状点序号

	for (auto itr = rCoords->Begin(); itr != rCoords->End(); itr++)
	{
		Coordinate *coord = (*itr);
		if (coord == nullptr)
		{
			continue;
		}
		BaseAlgorithm::GetNearestPntToLineset(coord, lCoords, pntProject, index);
		Engine::Geometries::Coordinate *pntFrom = lCoords->GetAt(index); //(*pPoints)[index];
		Engine::Geometries::Coordinate *pntTo = lCoords->GetAt(index + 1);
		;

		// 在其他标线的右侧，序号加1
		if (BaseAlgorithm::PntMatchLine(*pntFrom, *pntTo, *coord) == 2)
		{
			arrDir.Add(1);
		}
		else
		{
			arrDir.Add(2);
		}
	}
	if (arrDir.GetCount() == 0)
	{
		return 0;
	}
	if (arrDir.GetCount() == 1)
	{
		return arrDir.GetAt(0);
	}
	for (int i = 0; i < arrDir.GetCount() - 1; i++)
	{
		if (arrDir.GetAt(i) != arrDir.GetAt(i + 1))
		{
			return 0;
		}
	}
	return arrDir.GetAt(0);
}

Engine::Base::Bool Engine::Geometries::GeometryAlgorithm::InsertLineStringPoint(
	Geometries::LineString *pLineStr, Base::Double dDis, Base::Bool sNode)
{
	if (nullptr == pLineStr || dDis < 0)
	{
		return false;
	}
	Base::Array<Geometries::Coordinate *> *m_coordinates = pLineStr->GetCoordinates();

	Int32 npts = m_coordinates->GetCount();
	if ((npts < 2) || (dDis <= Geometries_EP))
	{
		return false;
	}

	Geometries::Coordinate *coord = NULL;
	Geometries::Coordinate *pFrom = NULL;
	Geometries::Coordinate *pTo = NULL;
	Double len = 0.0;
	Int32 insertIdx = 0;
	if (sNode)
	{
		pFrom = m_coordinates->GetAt(0);
		pTo = m_coordinates->GetAt(1);
		insertIdx = 1;
	}
	else
	{
		pFrom = m_coordinates->GetAt(npts - 2);
		pTo = m_coordinates->GetAt(npts - 1);
		insertIdx = npts - 1;
	}
	len = pFrom->Distance(*pTo);
	// 小于插点距离就无需插点处理，直接返回成功
	if (len <= dDis)
	{
		return true;
	}
	coord = new Geometries::Coordinate(*pFrom);
	coord->x = pFrom->x + (pTo->x - pFrom->x) * dDis / len;
	coord->y = pFrom->y + (pTo->y - pFrom->y) * dDis / len;
	coord->z = pFrom->z + (pTo->z - pFrom->z) * dDis / len;
	m_coordinates->InsertAt(insertIdx, coord);
	return true;
}

Geometries::Polygon *Engine::Geometries::GeometryAlgorithm::GenerateOffsetPolygon(
	Geometries::Polygon *pPolygon, Geometries::Coordinate normalVector, Base::Double offsetDis)
{
	if (nullptr == pPolygon)
	{
		return nullptr;
	}
	// 获取几何
	Geometries::Polygon *pNewPolygon = dynamic_cast<Geometries::Polygon *>(pPolygon->Clone());
	Geometries::LinearRing *pLinearRing = const_cast<Geometries::LinearRing *>(pNewPolygon->GetExteriorRing());
	Geometries::LineString *pLineString = dynamic_cast<Geometries::LineString *>(pLinearRing);
	if (nullptr == pLineString)
	{
		return nullptr;
	}
	// 计算平移向量
	normalVector.Normalize();
	Geometries::Coordinate movedVector = normalVector * offsetDis;
	// 平移
	for (auto itor = pLineString->GetCoordinates()->Begin(); itor != pLineString->GetCoordinates()->End(); itor++)
	{
		*(*itor) = *(*itor) + movedVector;
	}
	return pNewPolygon;
}

Base::Bool Engine::Geometries::GeometryAlgorithm::CalcCenterOfPolygon(
	Geometries::Polygon *pPolygon, Geometries::Coordinate &coordCenter)
{
	if (nullptr == pPolygon)
	{
		return false;
	}
	// 获取几何
	Geometries::LinearRing *pLinearRing = const_cast<Geometries::LinearRing *>(pPolygon->GetExteriorRing());
	Geometries::LineString *pLineString = dynamic_cast<Geometries::LineString *>(pLinearRing);
	if (nullptr == pLineString)
	{
		return false;
	}
	// 计算点和，然后取平均
	Geometries::Coordinate movedVector(0, 0, 0);
	for (auto itor = pLineString->GetCoordinates()->Begin(); itor != pLineString->GetCoordinates()->End(); itor++)
	{
		movedVector = *(*itor) + movedVector;
	}
	coordCenter = movedVector / pLineString->GetNumPoints();
	return true;
}

Base::Bool Engine::Geometries::GeometryAlgorithm::CalcPolyhedronWithTwoPolygon(
	Geometries::Polygon *pPolygon1, Geometries::Polygon *pPolygon2, Base::Array<PointNormalPlane> &arrPlane)
{
	if (nullptr == pPolygon1 || nullptr == pPolygon2)
	{
		return false;
	}
	// 获取几何1
	Geometries::LinearRing *pLinearRing1 = const_cast<Geometries::LinearRing *>(pPolygon1->GetExteriorRing());
	Geometries::LineString *pLineString1 = dynamic_cast<Geometries::LineString *>(pLinearRing1);
	if (nullptr == pLineString1)
	{
		return false;
	}
	// 获取几何2
	Geometries::LinearRing *pLinearRing2 = const_cast<Geometries::LinearRing *>(pPolygon2->GetExteriorRing());
	Geometries::LineString *pLineString2 = dynamic_cast<Geometries::LineString *>(pLinearRing2);
	if (nullptr == pLinearRing2)
	{
		return false;
	}
	// 有效性检查
	if (pLineString1->GetNumPoints() != pLineString2->GetNumPoints())
	{
		return false;
	}
	// 计算中心点
	Geometries::Coordinate centerCoord;
	if (!CalcCenterOfPolygon(pPolygon1, centerCoord))
	{
		return false;
	}
	// 计算各个面的点法式表达
	for (int index = 0; index < pLineString1->GetNumPoints() - 1; index++)
	{
		Geometries::Coordinate coord1_1 = *pLineString1->GetCoordinateN(index);
		Geometries::Coordinate coord1_2 = *pLineString1->GetCoordinateN(index + 1);
		Geometries::Coordinate coord2_1 = *pLineString2->GetCoordinateN(index);
		Geometries::Coordinate coord2_2 = *pLineString2->GetCoordinateN(index + 1);
		// 计算法向量
		Geometries::Coordinate normalVector1;
		Geometries::Coordinate normalVector2;
		if (!BaseAlgorithm3D::CalcPlaneNormalVector(coord1_1, coord1_2, coord2_1, normalVector1) || !BaseAlgorithm3D::CalcPlaneNormalVector(coord1_1, coord1_2, coord2_2, normalVector2))
		{
			return false;
		}
		// 判断四点是否共面
		double angle = normalVector1.AngleWith(normalVector2);
		if (std::fabs(angle) > Geometries_EP && std::fabs(angle - PI) > Geometries_EP)
		{
			return false;
		}
		// 矫正法向量方向，使其指向空间多面体内部
		Geometries::Coordinate standerdVector = centerCoord - coord1_1;
		if (standerdVector.DotProduct(normalVector1) < 0)
		{
			normalVector1 = -normalVector1;
		}
		PointNormalPlane tmpPointNormalPlane;
		tmpPointNormalPlane.coord = coord1_1;
		tmpPointNormalPlane.normalVector = normalVector1;
		arrPlane.Add(tmpPointNormalPlane);
	}
	return true;
}
Base::Bool Engine::Geometries::GeometryAlgorithm::CalcOuterRectOfCircle(
	Geometries::Coordinate centerCoord, Base::Double radius, Base::Array<Geometries::Coordinate *> &arrCoords)
{
	if (radius < 0)
	{
		return false;
	}
	// 左下点
	Geometries::Coordinate *leftDownCoord = new Geometries::Coordinate(centerCoord.x - radius,
																	   centerCoord.y - radius, centerCoord.z);
	arrCoords.Add(leftDownCoord);
	// 右下点
	Geometries::Coordinate *rightDownCoord = new Geometries::Coordinate(centerCoord.x + radius,
																		centerCoord.y - radius, centerCoord.z);
	arrCoords.Add(rightDownCoord);
	// 右上点
	Geometries::Coordinate *rightUpCoord = new Geometries::Coordinate(centerCoord.x + radius,
																	  centerCoord.y + radius, centerCoord.z);
	arrCoords.Add(rightUpCoord);
	// 左上点
	Geometries::Coordinate *leftUpCoord = new Geometries::Coordinate(centerCoord.x - radius,
																	 centerCoord.y + radius, centerCoord.z);
	arrCoords.Add(leftUpCoord);
	return true;
}
Base::Bool Engine::Geometries::GeometryAlgorithm::IsPointInPolyhedron(
	Geometries::Coordinate checkCoord, Base::Array<PointNormalPlane> arrPlane)
{
	for (auto itor = arrPlane.Begin(); itor != arrPlane.End(); itor++)
	{
		Geometries::Coordinate standerdVector = checkCoord - itor->coord;
		double dotPro = standerdVector.DotProduct(itor->normalVector);
		if (dotPro < Geometries_NEP)
		{
			return false;
		}
	}
	return true;
}
