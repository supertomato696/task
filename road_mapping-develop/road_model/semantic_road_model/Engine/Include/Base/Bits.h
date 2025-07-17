/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Bits.h
简要描述:
******************************************************************/

#ifndef ENGINE_BASE_BITS_H_
#define ENGINE_BASE_BITS_H_

#include "Base.h"
#include "Types.h"

namespace Engine
{
    namespace Base
    {
        class Base_API Bits
        {
        public:
            // 判断base的二进制序列倒数第bit位是否为1
            template <typename T>
            static Base::Bool TestBit(T base, T bit)
            {
                return (base & (1 << bit)) != 0;
            }

            // 设置base的二进制序列倒数第bit位为1
            template <typename T>
            static T SetBit(T base, T bit)
            {
                base |= (1 << bit);
                return base;
            }

            // 设置base的二进制序列倒数第bit位为0
            template <typename T>
            static T UnsetBit(T base, T bit)
            {
                base &= ~(1 << bit);
                return base;
            }

            // 返回A,B按位或人结果
            template <typename T>
            static T SetUnion(T A, T B)
            {
                return A | B;
            }

            // 返回A,B按位与人结果
            template <typename T>
            static T SetIntersection(T A, T B)
            {
                return A & B;
            }

            // 返回A,B按位异或人结果
            template <typename T>
            static T SetSubstraction(T A, T B)
            {
                return A & ~B;
            }

            // 返回二进制位计数
            template <typename T>
            static T BitCount(T base)
            {
                T count = 0;
                while (base > 0)
                {
                    if ((base & 1))
                        ++count;
                    base >>= 1;
                }
                return count;
            }

        private:
            // 默认构造函数
            Bits();

            // 析构函数
            ~Bits();
        };

    }
}

#endif // ENGINE_BASE_BITS_H_