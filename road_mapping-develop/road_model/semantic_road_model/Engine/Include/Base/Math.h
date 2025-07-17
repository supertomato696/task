/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Math.h
简要描述:封装基本的数学运算
******************************************************************/

#ifndef ENGINE_BASE_MATH_H_
#define ENGINE_BASE_MATH_H_

#include <cmath>
#include "Base.h"
#include "Types.h"
#include "Macros.h"

namespace Engine
{
    namespace Base
    {
        // 封装基本的数学运算函数
        class Math
        {
        public:
            // 求余弦值
            static Double Cos(Double a)
            {
                return std::cos(a);
            };

            // 求反余弦
            static Double ACos(Double a)
            {
                return std::acos(a);
            };

            // 求正弦值
            static Double Sin(Double a)
            {
                return std::sin(a);
            };

            // 求反正弦
            static Double ASin(Double a)
            {
                return std::asin(a);
            };

            // 求绝对值
            static Double Fabs(Double a)
            {
                return std::fabs(a);
            };

            // 浮点数判等,精度NUMERICAL_ACCURACY
            static Bool Equal(Double a, Double b)
            {
                return Equal(a, b, NUMERIC_ACCURACY);
            };

            // 浮点数判等,自定义精度
            static Bool Equal(Double a, Double b, Double accuracy)
            {
                return Fabs(a - b) < accuracy;
            };

            // 开平方
            static Double Sqrt(Double a)
            {
                return std::sqrt(a);
            };

            // 获取模板类型的最大值
            template <typename T>
            static T GetMax()
            {
                return std::numeric_limits<T>::max();
            }

            // 获取模板类型的最小值
            template <typename T>
            static T GetMin()
            {
                return std::numeric_limits<T>::min();
            }

            // 求取地板，抹零取整
            static Double Floor(Double a)
            {
                return std::floor(a);
            }

            // 求取天花板，拔高取整
            static Double Ceil(Double a)
            {
                return std::ceil(a);
            }

            // 四舍五入取整
            static Double Round(Double a)
            {
                // return std::round(a);
                return (a > 0.0) ? std::floor(a + 0.5) : std::ceil(a - 0.5);
            }
        };
    }
}

#endif // ENGINE_BASE_MATH_H_