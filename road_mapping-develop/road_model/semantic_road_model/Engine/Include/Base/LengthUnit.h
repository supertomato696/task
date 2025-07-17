/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:LengthUnit.h
简要描述:长度单位枚举
******************************************************************/

#ifndef ENGINE_BASE_LENGTHUNIT_H_
#define ENGINE_BASE_LENGTHUNIT_H_

namespace Engine
{
    namespace Base
    {
        enum class LengthUnit
        {
            MILLIMETER = 10,  // 毫米, 0.1毫米的十倍
            CENTIMETER = 100, // 厘米

            INCH = 254,       // 英寸，一英寸等于254个0.1毫米单位
            DECIMETER = 1000, // 分米

            FOOT = 3048,          // 英尺，一英尺等于3048个0.1毫米单位
            YARD = 9144,          // 码，一码等于9144个0.1毫米单位
            METER = 10000,        // 米
            KILOMETER = 10000000, // 千米
            MILE = 16093440,      //  英里
        };
    }
}

#endif // ENGINE_BASE_LENGTHUNIT_H_