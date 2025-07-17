/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryAlgorithm.h
简要描述:提供对Geometry的各种运算和操作方法
******************************************************************/

#ifndef ENGINE_GEOMETRIES_BUFFERALGORITHM_H_
#define ENGINE_GEOMETRIES_BUFFERALGORITHM_H_

#include "Export.h"
#include "Base/Types.h"
#include "Base/Array.h"

namespace Engine
{
    namespace Base
    {
        class String;
    }

    namespace Geometries
    {
        class Geometry;
        class Coordinate;
        class LineString;
        class Point;
        class Polygon;
        class LinearRing;
        class MultiLineString;

        // 提供对Geometry的各种运算和操作方法
        class Geometries_API BufferAlgorithm
        {
        public:
            // 生成偏移面 2donly, distance必须大于0，表示往外扩
            static Geometry *BufferOp(const Geometry *geometry, Base::Double distance);

            // 两个几何的最短距离
            static Base::Double Distance(const Geometry *lhs, const Geometry *rhs);

            // 两个几何是否相交
            static Base::Bool BufferIntersects(const Geometry *lhs, const Geometry *rhs, bool isConsiderRatio, double tolerance = 1e-2);

            // 得到离散点的二维凸包
            static Geometry *ConverxHull(Base::Array<Geometries::Coordinate> points);
        };
    }
}

#endif // ENGINE_GEOMETRIES_BUFFERALGORITHM_H_
