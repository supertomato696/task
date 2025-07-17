/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point3D.cpp
简要描述:
******************************************************************/

#include "Base/Point3D.h"

using namespace Engine::Base;

// 默认构造函数
Point3D::Point3D()
{
	X = 0.0;
	Y = 0.0;
	Z = 0.0;
}

// 析构函数
Point3D::~Point3D()
{
}

// 复制构造函数
Point3D::Point3D(const Point3D &rhs)
{
	*this = rhs;
}

// 赋值操作符
Point3D &Point3D::operator=(const Point3D &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		X = rhs.X;
		Y = rhs.Y;
		Z = rhs.Z;
		// 然后进行深拷贝操作
	}

	return *this;
}
