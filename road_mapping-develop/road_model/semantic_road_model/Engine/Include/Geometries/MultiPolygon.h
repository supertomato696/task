/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiPolygon.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_MULTIPOLYGON_H_
#define ENGINE_GEOMETRIES_MULTIPOLYGON_H_

#include "GeometryCollection.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API MultiPolygon : public GeometryCollection
        {
        public:
            // 默认构造函数,不包含任何子几何体
            MultiPolygon();

            // 使用给定的Polygon数组构造MultiPolygon对象
            // 注意:如果数组中包含非Polygon类型的对象,将会抛出异常
            MultiPolygon(Base::Array<Geometry *> *geoms);

            // 析构函数,负责释放内存
            ~MultiPolygon();

            // 复制构造函数,深拷贝
            MultiPolygon(const MultiPolygon &rhs);

            // 赋值操作符,深拷贝
            MultiPolygon &operator=(const MultiPolygon &rhs);

            // 返回GeometryType::MULTIPOLYGON
            GeometryType GetGeometryType() const;
        };
    }
}

#endif // ENGINE_GEOMETRIES_MULTIPOLYGON_H_