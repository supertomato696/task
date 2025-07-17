/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Geometry.h
简要描述:Geometry基类,抽象类,不能直接实例化
******************************************************************/

#ifndef ENGINE_GEOMETRIES_GEOMETRY_H_
#define ENGINE_GEOMETRIES_GEOMETRY_H_

#include "Export.h"
#include "Base/Types.h"
#include "Base/String.h"
#include "Envelope.h"
#include "GeometryType.h"
#include "Coordinate.h"
#include "Base/Array.h"

namespace Engine
{
    namespace Geometries
    {
        class Geometries_API Geometry
        {
        public:
            // 默认构造函数
            Geometry();

            // 析构函数
            virtual ~Geometry();

            // 复制构造函数
            Geometry(const Geometry &rhs);

            // 赋值操作符
            Geometry &operator=(const Geometry &rhs);

            // 返回Geometry对象的派生类类型,纯虚函数,派生类实现
            virtual GeometryType GetGeometryType() const = 0;

            // 返回Geometry对象的二维外包框,纯虚函数,派生类实现
            virtual Envelope *const GetEnvelope() const = 0;

            // 返回Geometry对象的深拷贝对象,纯虚函数,派生类实现
            virtual Geometry *const Clone() const = 0;

            //////add 2018-11-15 llj 字符串转为相关的几何类型
            virtual bool Assing(const char *_sGeo) { return false; };

            virtual const int ReadType(const char *&_sGeo); // 读取类型
        };

    }
}

#endif // ENGINE_GEOMETRIES_GEOMETRY_H_