/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiPoint.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_MULTIPOINT_H_
#define ENGINE_GEOMETRIES_MULTIPOINT_H_

#include "GeometryCollection.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API MultiPoint : public GeometryCollection
        {
        public:
            // 默认构造函数,不包含任何点
            MultiPoint();

            // 使用给定的点几何数组构造MultiPoint对象
            // 注意:如果数组中包含非LineString类型的对象,将会抛出异常
            MultiPoint(Base::Array<Geometry *> *geoms);

            // 析构函数,负责释放内存
            ~MultiPoint();

            // 复制构造函数,深拷贝
            MultiPoint(const MultiPoint &rhs);

            // 赋值操作符,深拷贝
            MultiPoint &operator=(const MultiPoint &rhs);

            // 返回GeometryType::MULTIPOINT
            GeometryType GetGeometryType() const;

            Geometry *const Clone() const;
        };
    }
}

#endif // ENGINE_GEOMETRIES_MULTIPOINT_H_