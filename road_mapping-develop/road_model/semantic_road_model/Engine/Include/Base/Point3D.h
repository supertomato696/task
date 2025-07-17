/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point3D.h
简要描述:
******************************************************************/

#ifndef ENGINE_BASE_POINT3D_H_
#define ENGINE_BASE_POINT3D_H_

#include "Base.h"
#include "Base/Types.h"

namespace Engine
{
    namespace Base
    {
        class Base_API Point3D
        {
        public:
            // 默认构造函数
            Point3D();

            // 析构函数
            ~Point3D();

            // 复制构造函数
            Point3D(const Point3D &rhs);

            // 赋值操作符
            Point3D &operator=(const Point3D &rhs);

        public:
            Base::Double X;
            Base::Double Y;
            Base::Double Z;
        };
    }
}

#endif // ENGINE_BASE_POINT3D_H_