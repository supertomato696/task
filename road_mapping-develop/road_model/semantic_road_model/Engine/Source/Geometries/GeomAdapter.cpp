/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeomAdapter.cpp
简要描述:
******************************************************************/

#include "Geometries/GeomAdapter.h"
#include "Geometries/Geometry.h"
#include "Geometries/Point.h"
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Polygon.h"
#include "Geometries/MultiPoint.h"
#include "Geometries/MultiLineString.h"
#include "Geometries/MultiPolygon.h"
#include "geos/geos.h"
#include <vector>

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

geos::geom::Geometry *GeomAdapter::ToGeosGeom(const Engine::Geometries::Geometry *geom)
{
	geos::geom::Geometry *geosgeom = nullptr;
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	GeometryType gt = geom->GetGeometryType();
	switch (gt)
	{
	case Engine::Geometries::GeometryType::POINT:
	{
		const Engine::Geometries::Point *navPt = static_cast<const Engine::Geometries::Point *>(geom);
		geos::geom::Coordinate geosCoord(navPt->GetX(), navPt->GetY(), navPt->GetZ());
		geosgeom = gf->createPoint(geosCoord);
	}
	break;
	case Engine::Geometries::GeometryType::LINESTRING:
	{
		const Engine::Geometries::LineString *navls = static_cast<const Engine::Geometries::LineString *>(geom);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navls->GetCoordinates();
		SizeT count = navCoords->GetCount();
		vector<geos::geom::Coordinate> *geosCoords = new vector<geos::geom::Coordinate>();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			geos::geom::Coordinate geosCoord(navCoord->x, navCoord->y, navCoord->z);
			geosCoords->push_back(geosCoord);
		}
		std::unique_ptr<geos::geom::CoordinateSequence> geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords);
		geos::geom::CoordinateSequence *newgeoscs = geoscs.release();
		geosgeom = gf->createLineString(newgeoscs);
	}
	break;
	case Engine::Geometries::GeometryType::LINEARRING:
	{
		const Engine::Geometries::LineString *navlr = static_cast<const Engine::Geometries::LineString *>(geom);
		const Array<Engine::Geometries::Coordinate *> *navCoords = navlr->GetCoordinates();
		SizeT count = navCoords->GetCount();
		vector<geos::geom::Coordinate> *geosCoords = new vector<geos::geom::Coordinate>();

		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Coordinate *navCoord = navCoords->GetAt(i);
			geos::geom::Coordinate geosCoord(navCoord->x, navCoord->y, navCoord->z);

			geosCoords->push_back(geosCoord);
		}
		std::unique_ptr<geos::geom::CoordinateSequence> geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords);
		geosgeom = gf->createLinearRing(geoscs.release());
	}
	break;
	case Engine::Geometries::GeometryType::POLYGON:
	{
		const Engine::Geometries::Polygon *navpy = static_cast<const Engine::Geometries::Polygon *>(geom);
		const Engine::Geometries::LinearRing *navpyShell = navpy->GetExteriorRing();
		const geos::geom::LinearRing *geospyShell = dynamic_cast<const geos::geom::LinearRing *>(ToGeosGeom(navpyShell));

		vector<geos::geom::LinearRing *> *geosHoles = new vector<geos::geom::LinearRing *>();
		SizeT count = navpy->GetNumInteriorRing();
		for (SizeT i = 0; i < count; ++i)
		{
			const Engine::Geometries::LinearRing *navpyHole = navpy->GetInteriorRingN(i);
			geos::geom::LinearRing *geospyHole = dynamic_cast<geos::geom::LinearRing *>(ToGeosGeom(navpyHole));
			geosHoles->push_back(geospyHole);
		}

		geos::geom::Polygon *geospy = gf->createPolygon(const_cast<geos::geom::LinearRing *>(geospyShell), geosHoles);
		geosgeom = geospy;
	}
	break;
	case Engine::Geometries::GeometryType::MULTIPOINT:
	{
		vector<geos::geom::Geometry *> *geosgeoms = new vector<geos::geom::Geometry *>();

		const Engine::Geometries::GeometryCollection *navgc = static_cast<const Engine::Geometries::GeometryCollection *>(geom);
		SizeT count = navgc->GetNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Geometry *geosSubGeom = ToGeosGeom(navgc->GetGeometryN(i));

			geosgeoms->push_back(geosSubGeom);
		}

		geos::geom::MultiPoint *geosmpt = gf->createMultiPoint(geosgeoms);

		geosgeom = geosmpt;
	}
	break;
	case Engine::Geometries::GeometryType::MULTILINESTRING:
	{
		vector<geos::geom::Geometry *> *geosgeoms = new vector<geos::geom::Geometry *>();

		const Engine::Geometries::GeometryCollection *navgc = static_cast<const Engine::Geometries::GeometryCollection *>(geom);
		SizeT count = navgc->GetNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Geometry *geosSubGeom = ToGeosGeom(navgc->GetGeometryN(i));

			geosgeoms->push_back(geosSubGeom);
		}

		geos::geom::MultiLineString *geosmls = gf->createMultiLineString(geosgeoms);

		geosgeom = geosmls;
	}
	break;
	case Engine::Geometries::GeometryType::MULTIPOLYGON:
	{
		vector<geos::geom::Geometry *> *geosgeoms = new vector<geos::geom::Geometry *>();

		const Engine::Geometries::MultiPolygon *navmpy = static_cast<const Engine::Geometries::MultiPolygon *>(geom);
		SizeT count = navmpy->GetNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Geometry *geosSubGeom = ToGeosGeom(navmpy->GetGeometryN(i));

			geosgeoms->push_back(geosSubGeom);
		}

		geos::geom::MultiPolygon *geosmpy = gf->createMultiPolygon(geosgeoms);

		geosgeom = geosmpy;
	}
	break;
	case Engine::Geometries::GeometryType::GEOMETRYCOLLECTION:
	{
		vector<geos::geom::Geometry *> *geosgeoms = new vector<geos::geom::Geometry *>();

		const Engine::Geometries::GeometryCollection *navgc = static_cast<const Engine::Geometries::GeometryCollection *>(geom);
		SizeT count = navgc->GetNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Geometry *geosSubGeom = ToGeosGeom(navgc->GetGeometryN(i));

			geosgeoms->push_back(geosSubGeom);
		}

		geos::geom::GeometryCollection *geosgc = gf->createGeometryCollection(geosgeoms);

		geosgeom = geosgc;
	}
	break;
	default:
	{
		throw std::exception();
	}
	break;
	}

	return geosgeom;
}

Engine::Geometries::Geometry *GeomAdapter::ToGeometry(const geos::geom::Geometry *geom)
{
	Engine::Geometries::Geometry *navGeom = nullptr;
	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	geos::geom::GeometryTypeId gt = geom->getGeometryTypeId();
	switch (gt)
	{
	case geos::geom::GeometryTypeId::GEOS_POINT:
	{
		const geos::geom::Coordinate *coord = geom->getCoordinate();
		navGeom = new Engine::Geometries::Point(coord->x, coord->y, coord->z);
		DELETE_SAFE(coord); // add llj 2018-12-11
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_LINESTRING:
	{
		std::unique_ptr<geos::geom::CoordinateSequence> cs = geom->getCoordinates();
		SizeT count = cs->getSize();
		Array<Engine::Geometries::Coordinate *> *navCoords = new Array<Engine::Geometries::Coordinate *>();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Coordinate coord = cs->getAt(i);
			Engine::Geometries::Coordinate *navCoord = new Engine::Geometries::Coordinate(coord.x, coord.y, coord.z);
			navCoords->Add(navCoord);
		}
		navGeom = new Engine::Geometries::LineString(navCoords);
		//		DELETE_SAFE(cs);//add llj 2018-12-11
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_LINEARRING:
	{
		std::unique_ptr<geos::geom::CoordinateSequence> cs = geom->getCoordinates();
		SizeT count = cs->getSize();
		Array<Engine::Geometries::Coordinate *> *navCoords = new Array<Engine::Geometries::Coordinate *>();
		for (SizeT i = 0; i < count; ++i)
		{
			geos::geom::Coordinate coord = cs->getAt(i);
			Engine::Geometries::Coordinate *navCoord = new Engine::Geometries::Coordinate(coord.x, coord.y, coord.z);
			navCoords->Add(navCoord);
		}
		navGeom = new Engine::Geometries::LinearRing(navCoords);
		//		DELETE_SAFE(cs);//add llj 2018-12-11
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_POLYGON:
	{
		const geos::geom::Polygon *geospy = dynamic_cast<const geos::geom::Polygon *>(geom);
		const geos::geom::LineString *geospyShell = geospy->getExteriorRing();
		Engine::Geometries::LinearRing *navpyShell = static_cast<Engine::Geometries::LinearRing *>(ToGeometry(geospyShell));

		Array<Engine::Geometries::LinearRing *> *navpyHoles = new Array<Engine::Geometries::LinearRing *>();
		SizeT count = geospy->getNumInteriorRing();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::LinearRing *navpyHole = static_cast<Engine::Geometries::LinearRing *>(ToGeometry(geospy->getInteriorRingN(i)));
			navpyHoles->Add(navpyHole);
		}

		navGeom = new Engine::Geometries::Polygon(navpyShell, navpyHoles);
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_MULTIPOINT:
	{
		Array<Engine::Geometries::Geometry *> *navPoints = new Array<Engine::Geometries::Geometry *>();
		SizeT count = geom->getNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Geometry *navPoint = ToGeometry(geom->getGeometryN(i));
			navPoints->Add(navPoint);
		}

		navGeom = new Engine::Geometries::MultiPoint(navPoints);
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_MULTILINESTRING:
	{
		Array<Engine::Geometries::Geometry *> *navlss = new Array<Engine::Geometries::Geometry *>();
		SizeT count = geom->getNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Geometry *navPoint = ToGeometry(geom->getGeometryN(i));
			navlss->Add(navPoint);
		}

		navGeom = new Engine::Geometries::MultiLineString(navlss);
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_MULTIPOLYGON:
	{
		Array<Engine::Geometries::Geometry *> *navpys = new Array<Engine::Geometries::Geometry *>();
		SizeT count = geom->getNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Geometry *navpy = ToGeometry(geom->getGeometryN(i));
			navpys->Add(navpy);
		}

		navGeom = new Engine::Geometries::MultiPolygon(navpys);
	}
	break;
	case geos::geom::GeometryTypeId::GEOS_GEOMETRYCOLLECTION:
	{
		Array<Engine::Geometries::Geometry *> *navgeoms = new Array<Engine::Geometries::Geometry *>();
		SizeT count = geom->getNumGeometries();
		for (SizeT i = 0; i < count; ++i)
		{
			Engine::Geometries::Geometry *navPoint = ToGeometry(geom->getGeometryN(i));
			navgeoms->Add(navPoint);
		}

		navGeom = new Engine::Geometries::GeometryCollection(navgeoms);
	}
	break;
	default:
	{
		throw std::exception();
	}
	break;
	}

	// DELETE_SAFE(gf);//add llj 2018-12-11 不能删除 否则会引起错误
	return navGeom;
}
