/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeomAdapter.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_GEOMADAPTER_H_
#define ENGINE_GEOMETRIES_GEOMADAPTER_H_

#include "Export.h"

// 前置声明
namespace geos
{
    namespace geom
    {
        class Geometry;
    }
}

namespace Engine
{
    namespace Geometries
    {
        class Geometry;
        class Geometries_API GeomAdapter
        {
        public:
            static geos::geom::Geometry *ToGeosGeom(const Geometry *geom);

            static Geometry *ToGeometry(const geos::geom::Geometry *geom);
        };
    }
}

#endif // ENGINE_GEOMETRIES_GEOMADAPTER_H_