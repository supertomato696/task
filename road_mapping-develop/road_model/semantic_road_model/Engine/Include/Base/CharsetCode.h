/******************************************************************
作者: test
日期: 2021-8-18 11:19
简要描述:字符编码枚举
******************************************************************/

#ifndef ENGINE_BASE_CHARSETCODE_H_
#define ENGINE_BASE_CHARSETCODE_H_

namespace Engine
{
    namespace Base
    {
        enum class CharsetCode
        {
            NONE = 0,  // 未知或不明确的字符集
            ANSI = 1,  // 当前操作系统字符集
            UTF8 = 2,  // UTF8字符集
            UTF16 = 3, // UTF16 字符集
            UTF32 = 4, // UTF32字符集
        };
    }
}

#endif // ENGINE_BASE_CHARSETCODE_H_