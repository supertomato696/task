/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称: Algorithm.h
简要描述:
******************************************************************/

#ifndef ENGINE_ALGORITHM_H_
#define ENGINE_ALGORITHM_H_

// 任何其他项目上不应定义此符号BASE_EXPORTS
#ifdef _WIN32
#ifdef ALGORITHM_EXPORTS
#define Algorithm_API __declspec(dllexport)
#else
#define Algorithm_API __declspec(dllimport)
#endif
#else
#define Algorithm_API
#endif

// include base

#endif // ALGORITHM_H_
