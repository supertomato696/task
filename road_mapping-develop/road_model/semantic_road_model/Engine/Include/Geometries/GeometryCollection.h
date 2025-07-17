/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryCollection.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_GEOMETRYCOLLECTION_H_
#define ENGINE_GEOMETRIES_GEOMETRYCOLLECTION_H_

#include "Geometry.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API GeometryCollection : public Geometry
        {
        public:
            // 默认构造函数,构造不包含任何Geometry的GeometryCollection对象
            GeometryCollection();

            // 使用newGeoms构造GeometryCollection对象,newGeoms由GeometryCollection对象接管
            GeometryCollection(Base::Array<Geometry *> *newGeoms);

            // 析构函数
            virtual ~GeometryCollection();

            // 复制构造函数,深拷贝
            GeometryCollection(const GeometryCollection &rhs);

            // 赋值操作符,深拷贝
            GeometryCollection &operator=(const GeometryCollection &rhs);

            // 返回GeometryType::GEOMETRYCOLLECTION
            virtual GeometryType GetGeometryType() const;

            // 返回GeometryCollection所包含的所有Geometry的二维外包框
            virtual Envelope *const GetEnvelope() const;

            // 返回GeometryCollection对象的深拷贝
            // 注意:需要由调用者释放所返回的指针
            virtual Geometry *const Clone() const;

            // 返回GeometryCollection内部包含的Geometry数量
            virtual Base::SizeT GetNumGeometries() const;

            // 返回GeometryCollection内部包含的索引为index的Geometry对象指针
            virtual Geometry *const GetGeometryN(Base::SizeT index) const;

        protected:
            Base::Array<Geometry *> *m_geometries;
        };
    }
}

#endif // ENGINE_GEOMETRIES_GEOMETRYCOLLECTION_H_