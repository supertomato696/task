/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Base.h
简要描述:
******************************************************************/

#ifndef BASE_H_
#define BASE_H_

// 任何其他项目上不应定义此符号BASE_EXPORTS
#ifdef _WIN32
#ifdef BASE_EXPORTS
#define Base_API __declspec(dllexport)
#else
#define Base_API __declspec(dllimport)
#endif
#else
#define Base_API
#endif

#endif // BASE_H_