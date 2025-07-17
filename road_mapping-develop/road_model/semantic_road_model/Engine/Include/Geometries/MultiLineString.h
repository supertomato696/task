/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiLineString.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_MULTILINESTRING_H_
#define ENGINE_GEOMETRIES_MULTILINESTRING_H_

#include "GeometryCollection.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API MultiLineString : public GeometryCollection
        {
        public:
            // 默认构造函数,不包含任何子几何体
            MultiLineString();

            // 使用给定的LineString数组构造MultiLineString对象
            // 注意:如果数组中包含非LineString类型的对象,将会抛出异常
            MultiLineString(Base::Array<Geometry *> *geoms);

            // 析构函数,负责释放内存
            ~MultiLineString();

            // 复制构造函数,深拷贝
            MultiLineString(const MultiLineString &rhs);

            // 赋值操作符,深拷贝
            MultiLineString &operator=(const MultiLineString &rhs);

            // 返回GeometryType::MULTILINESTRING
            GeometryType GetGeometryType() const;

            Geometry *const Clone() const;
        };
    }
}

#endif // ENGINE_GEOMETRIES_MULTILINESTRING_H_