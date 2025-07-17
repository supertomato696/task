/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryType.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_GEOMETRYTYPE_H_
#define ENGINE_GEOMETRIES_GEOMETRYTYPE_H_

namespace Engine
{
    namespace Geometries
    {
        enum class GeometryType
        {
            /// a point
            POINT,
            /// a linestring
            LINESTRING,
            /// a linear ring (linestring with 1st point == last point)
            LINEARRING,
            /// a polygon
            POLYGON,
            /// a collection of points
            MULTIPOINT,
            /// a collection of linestrings
            MULTILINESTRING,
            /// a collection of polygons
            MULTIPOLYGON,
            /// a collection of heterogeneus geometries
            GEOMETRYCOLLECTION,
            /// B样条曲线
            SPLINE,

            // 不规则
            IRREGULAR,

            // 三角形
            TRIANGLE,

            // 矩形
            RECTANGLE,

            // 正方形
            SQUARE,

            // 圆
            CIRCLE,

            // 菱形
            RHOMBUS,

            // 倒三角
            INV_TRIANGLE,

            // 八角形
            OCTAGON,

            // 斜圆柱
            OBLIQUECIRCULARCYLINDER,

            // 斜长方体
            INCLINEDRECTANGULARBLOCK
        };
    }
}

#endif // ENGINE_GEOMETRIES_GEOMETRYTYPE_H_
