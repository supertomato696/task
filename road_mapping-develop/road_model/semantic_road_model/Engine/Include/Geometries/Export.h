/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Export.h
简要描述:
******************************************************************/

#ifndef ENGINE_GEOMETRIES_EXPORT_H_
#define ENGINE_GEOMETRIES_EXPORT_H_
#ifdef _WIN32
// 任何其他项目上不应定义此符号GEOMETRY_EXPORTS
#ifdef GEOMETRIES_EXPORTS
#define Geometries_API __declspec(dllexport)
#else
#define Geometries_API __declspec(dllimport)
#endif
#else
#define Geometries_API
#endif

#endif // ENGINE_GEOMETRIES_EXPORT_H_