/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point2D.h
简要描述:
******************************************************************/

#ifndef ENGINE_BASE_POINT2D_H_
#define ENGINE_BASE_POINT2D_H_

#include "Base.h"
#include "Base/Types.h"

namespace Engine
{
    namespace Base
    {
        class Base_API Point2D
        {
        public:
            // 默认构造函数
            Point2D();

            // 默认构造函数
            Point2D(Double x, Double y);

            // 析构函数
            ~Point2D();

            // 复制构造函数
            Point2D(const Point2D &rhs);

            // 赋值操作符
            Point2D &operator=(const Point2D &rhs);

        public:
            Base::Double X;
            Base::Double Y;
        };
    }
}

#endif // ENGINE_BASE_POINT2D_H_