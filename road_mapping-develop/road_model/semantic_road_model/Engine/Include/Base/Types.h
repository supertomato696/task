/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Types.h
简要描述:基础类型定义
******************************************************************/

#ifndef ENGINE_BASE_TYPES_H_
#define ENGINE_BASE_TYPES_H_
#include <stdio.h>
#include <stdlib.h>

namespace Engine
{
    namespace Base
    {
        /// 基础类型定义
        typedef void Void;    /// 无类型
        typedef void *Handle; /// 句柄类型，和平台相关，32位下是32位，64位下是64位
        typedef bool Bool;    /// 占用1字节，布尔型。	值域为true和false。true表示肯定，false为否定。

        /// 字符型
        typedef char Char;           /// 占用1字节, 有符号字符类型，值域范围<-128 to 127>
        typedef unsigned char UChar; /// 占用1字节, 无符号字符类型，值域范围<0 to 255>

        /// 整型
        typedef short Int16;               /// 占用2字节，16位整型。<-32,768 to 32,767>
        typedef unsigned short UInt16;     /// 占用2字节，16位无符号整型。<0 to 65,535>
        typedef int Int32;                 /// 占用4字节，32位整型。<-2,147,483,648 to 2,147,483,647>
        typedef unsigned int UInt32;       /// 占用4字节，32位无符号整型。<0 to 4,294,967,295>
        typedef long long Int64;           /// 占用8字节，64位整型。window下这么用，linux下直接可以用long表示
        typedef unsigned long long UInt64; /// 占用8字节，64位无符号整型

        /// 浮点数
        typedef float Float;   /// 占用4字节，浮点型。<3.4E +/- 38 (7 digits)>
        typedef double Double; /// 占用8字节，双精度浮点型。<1.7E +/- 308 (15 digits)>
        typedef size_t SizeT;  /// 类型大小，和平台大小相关，32位下是32位，64位是64位
    }
}
#endif // ENGINE_BASE_TYPES_H_