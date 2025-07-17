/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:LinearRing.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_LINEARRING_H_
#define ENGINE_GEOMETRIES_LINEARRING_H_

#include "LineString.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API LinearRing : public LineString
        {
        public:
            // 默认构造函数,不包含任何坐标点
            LinearRing();

            // 使用给定的坐标数组构造LinearRing对象
            LinearRing(Base::Array<Coordinate *> *coordinates);

            // 析构函数,负责释放内存
            ~LinearRing();

            // 复制构造函数,深拷贝
            LinearRing(const LinearRing &rhs);

            // 赋值操作符,深拷贝
            LinearRing &operator=(const LinearRing &rhs);

            // 如果LinearRing对象是闭合的,返回true,否则false;不包含任何坐标认为是闭合的,返回true
            Base::Bool IsClosed() const;

            // 返回GeometryType::LINEARRING
            virtual GeometryType GetGeometryType() const;

            // 返回LinearRing对象的坐标顺序相反的LinearRing对象
            // 注意:需要由调用者释放所返回的指针
            Geometry *const Reverse() const;

            // 去重,Write By xueyufei
            virtual Base::Bool Distinct();
        };
    }
}

#endif // ENGINE_GEOMETRIES_LINEARRING_H_