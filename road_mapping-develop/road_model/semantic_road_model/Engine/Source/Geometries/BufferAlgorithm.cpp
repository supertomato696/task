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
#include "Geometries/BufferAlgorithm.h"
#include "Geometries/Geometry.h"
#include "Geometries/GeomAdapter.h"
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"
#include "Geometries/Point.h"
#include "Base/String.h"
#include "Base/Macros.h"
#include "Base/Math.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/Polygon.h"
#include "Geometries/LinearRing.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/MultiLineString.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/BaseAlgorithm.h"
#include "geos/operation/buffer/BufferOp.h"
#include "geos/algorithm/ConvexHull.h"
#include "Geometries/GeometryAlgorithm.h"

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace Engine;

Engine::Geometries::Geometry *Engine::Geometries::BufferAlgorithm::BufferOp(const Engine::Geometries::Geometry *geometry, Base::Double distance)
{
	geos::geom::Geometry *pGeometry = GeomAdapter::ToGeosGeom(geometry);
	geos::geom::Geometry *pGeosPoly = geos::operation::buffer::BufferOp::bufferOp(pGeometry, distance, geos::operation::buffer::BufferParameters::DEFAULT_QUADRANT_SEGMENTS, geos::operation::buffer::BufferParameters::CAP_FLAT);
	Engine::Geometries::Polygon *polygon = static_cast<Engine::Geometries::Polygon *>(GeomAdapter::ToGeometry(pGeosPoly));
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeosPoly);

	Engine::Geometries::LinearRing *lr = polygon->GetExteriorRing();
	Base::Array<Engine::Geometries::Coordinate *> *NCoords = lr->GetCoordinates();
	for (auto itr = NCoords->Begin(); itr != NCoords->End(); itr++)
	{
		// 将z值赋值为0.0
		(*itr)->z = 0.0;
	}
	return polygon;
}

Engine::Base::Double Engine::Geometries::BufferAlgorithm::Distance(const Geometry *lhs, const Geometry *rhs)
{
	geos::geom::Geometry *pGeometry1 = GeomAdapter::ToGeosGeom(lhs);
	geos::geom::Geometry *pGeometry2 = GeomAdapter::ToGeosGeom(rhs);
	Base::Double distance = pGeometry1->distance(pGeometry2);
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeometry1);
	geos::geom::GeometryFactory::getDefaultInstance()->destroyGeometry(pGeometry2);
	return distance;
}

Base::Bool Engine::Geometries::BufferAlgorithm::BufferIntersects(const Geometry *lhs, const Geometry *rhs, bool isConsiderRatio, double tolerance)
{
	bool flag = false;
	Engine::Geometries::Geometry *pGeosPoly1 = BufferOp(lhs, tolerance);
	Engine::Geometries::Geometry *pGeosPoly2 = BufferOp(rhs, tolerance);

	if (isConsiderRatio)
	{
		geos::geom::Geometry *pGeometry1 = GeomAdapter::ToGeosGeom(pGeosPoly1);
		geos::geom::Geometry *pGeometry2 = GeomAdapter::ToGeosGeom(pGeosPoly2);
		geos::geom::Geometry *intersection = pGeometry1->intersection(pGeometry2).release();

		if (intersection->getLength() / pGeometry2->getLength() > 0.3)
		{
			flag = true;
		}
		DELETE_PTR(pGeometry1);
		DELETE_PTR(pGeometry2);
		DELETE_PTR(intersection);
	}
	else
	{
		flag = GeometryAlgorithm::Intersects(pGeosPoly1, pGeosPoly2);
	}
	DELETE_PTR(pGeosPoly1);
	DELETE_PTR(pGeosPoly2);
	return flag;
}

Geometries::Geometry *BufferAlgorithm::ConverxHull(Base::Array<Geometries::Coordinate> points)
{
	std::vector<geos::geom::Coordinate> *geosCoords = new std::vector<geos::geom::Coordinate>();
	for (auto itr = points.Begin(); itr != points.End(); itr++)
	{
		geosCoords->push_back(geos::geom::Coordinate(itr->x, itr->y, 0.0));
	}

	const GeometryFactory *gf = GeometryFactory::getDefaultInstance();
	geos::geom::CoordinateSequence *geoscs = gf->getCoordinateSequenceFactory()->create(geosCoords).release();
	geos::geom::MultiPoint *multiPoint = gf->createMultiPoint(*geoscs);
	geos::geom::Geometry *convexHull = multiPoint->convexHull().release();
	Geometries::Geometry *geom = GeomAdapter::ToGeometry(convexHull);
	gf->destroyGeometry(multiPoint);
	gf->destroyGeometry(convexHull);
	return geom;
}
