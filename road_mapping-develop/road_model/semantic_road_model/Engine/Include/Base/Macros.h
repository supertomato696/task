/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Macros.h
简要描述:常用宏定义
******************************************************************/

#ifndef ENGINE_BASE_MACROS_H_
#define ENGINE_BASE_MACROS_H_

#include <limits>
#include "Types.h"

namespace Engine
{
        namespace Base
        {
                // 表示空的值
#ifndef NULL
#define NULL 0
#endif // NULL

                // 表示真的值
#ifndef TRUE
#define TRUE true
#endif // TRUE

                // 表示非真的值
#ifndef FALSE
#define FALSE false
#endif // FALSE

                // π
#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

                // 小数精度，暂定小数点后10位
#ifndef NUMERIC_ACCURACY
#define NUMERIC_ACCURACY (1e-10)
#endif

                // 是否是非数值
#ifndef ISNAN
#define ISNAN(x) std::isnan(x)
#endif

                // 是否有界
#ifndef ISFINITE
#define ISFINITE(x) std::isfinite(x)
#endif

                // 非数值
#ifndef DOUBLE_NAN
#define DOUBLE_NAN std::numeric_limits<Double>::quiet_NaN()
#endif

                // 无穷大
#ifndef DOUBLE_INFINITY
#define DOUBLE_INFINITY std::numeric_limits<Double>::infinity()
#endif

                // 指针释放
#ifndef DELETE_PTR
#define DELETE_PTR(X)          \
        if ((X) != nullptr)    \
        {                      \
                delete (X);    \
                (X) = nullptr; \
        }
#endif

                // 数组释放
#ifndef DELETE_ARRAY
#define DELETE_ARRAY(X)        \
        if ((X) != nullptr)    \
        {                      \
                delete[] (X);  \
                (X) = nullptr; \
        }
#endif

                ///////////add llj 2018-12-6
                // 指针释放
#ifndef DELETE_SAFE
#define DELETE_SAFE(X)      \
        if ((X) != NULL)    \
        {                   \
                delete (X); \
                (X) = NULL; \
        }
#endif

                // 数组释放
#ifndef DELETE_ARRAY_SAFE
#define DELETE_ARRAY_SAFE(X)  \
        if ((X) != NULL)      \
        {                     \
                delete[] (X); \
                (X) = NULL;   \
        }
#endif

#ifndef FREE_SAFE
#define FREE_SAFE(X)        \
        if ((X) != NULL)    \
        {                   \
                free(X);    \
                (X) = NULL; \
        }
#endif

#ifndef _DELETE_VECTOR_SAFE
#define _DELETE_VECTOR_SAFE(vecInfo)             \
        for (int i = 0; i < vecInfo.size(); i++) \
        DELETE_SAFE(vecInfo[i])
#endif

#ifndef _DELETE_VECTOR_SAFE_FREE
#define _DELETE_VECTOR_SAFE_FREE(vecInfo)        \
        for (int i = 0; i < vecInfo.size(); i++) \
        FREE_SAFE(vecInfo[i])
#endif

                //////////////

                // 函数说明中，表示一个参数是输入值还是输出值的相关宏
#ifndef IN
#define IN
#endif

#ifndef OUT
#define OUT
#endif

                // 禁止复制构造函数和赋值操作符
#ifndef DISALLOW_COPY_AND_ASSIGN
#define DISALLOW_COPY_AND_ASSIGN(TypeName)   \
        TypeName(const TypeName &) = delete; \
        void operator=(const TypeName &) = delete
#endif

                // 定义极限值
#ifndef INT32_MIN
#define INT32_MIN (-2147483647i32 - 1)
#endif // !INT32_MIN

                // Geometries使用的容限
#ifndef Geometries_EP
#define Geometries_EP (1.0E-5)
#endif

                // Geometries使用的容限
#ifndef Geometries_NEP
#define Geometries_NEP (-1.0e-5)
#endif
        }
}

#endif // ENGINE_BASE_MACROS_H_
