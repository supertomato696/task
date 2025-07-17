/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:Point2D.cpp
简要描述:
******************************************************************/

#include "Base/Point2D.h"

using namespace Engine::Base;

// 默认构造函数
Point2D::Point2D()
{
	X = 0.0;
	Y = 0.0;
}

Point2D::Point2D(Double x, Double y)
{
	X = x;
	Y = y;
}

// 析构函数
Point2D::~Point2D()
{
}

// 复制构造函数
Point2D::Point2D(const Point2D &rhs)
{
	*this = rhs;
}

// 赋值操作符
Point2D &Point2D::operator=(const Point2D &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		X = rhs.X;
		Y = rhs.Y;
		// 然后进行深拷贝操作
	}

	return *this;
}
