/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:AngleUnit.h
简要描述:角度单位枚举
******************************************************************/

#ifndef ENGINE_BASE_ANGLEUNIT_H_
#define ENGINE_BASE_ANGLEUNIT_H_

namespace Engine
{
    namespace Base
    {
        enum class AngleUnit
        {
            MILLISECOND = 1,  // 毫秒, 度数的基础单位
            SECOND = 1000,    // 秒, 等于1000毫秒
            MINUTE = 60000,   // 分, 等于60秒，等于6万毫秒
            DEGREE = 3600000, // 度，等于60分，3600秒，三百六十万毫秒
        };
    }
}

#endif // ENGINE_BASE_ANGLEUNIT_H_