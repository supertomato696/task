/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiPoint.cpp
简要描述:
******************************************************************/

#include "Geometries/MultiPoint.h"
#include "Geometries/Point.h"
#include <exception>
#include <stdexcept>

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

// 默认构造函数
MultiPoint::MultiPoint()
	: GeometryCollection()
{
}

MultiPoint::MultiPoint(Array<Geometry *> *geoms)
	: GeometryCollection(geoms)
{
	SizeT ngeoms = m_geometries->GetCount();
	for (SizeT i = 0; i < ngeoms; ++i)
	{
		if (GeometryType::POINT != m_geometries->GetAt(i)->GetGeometryType())
		{
#ifdef _WIN32
			throw std::exception("MultiPoint只能包含POINT类型的Geometry");
#else
			logic_error ex("MultiPoint只能包含POINT类型的Geometry");
			throw std::exception(ex);
#endif
		}
	}
}

// 析构函数
MultiPoint::~MultiPoint()
{
}

// 复制构造函数
MultiPoint::MultiPoint(const MultiPoint &rhs)
	: GeometryCollection(rhs)
{
}

// 赋值操作符
MultiPoint &MultiPoint::operator=(const MultiPoint &rhs)
{

	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
		GeometryCollection::operator=(rhs);
	}

	return *this;
}

GeometryType MultiPoint::GetGeometryType() const
{
	return GeometryType::MULTIPOINT;
}

Geometry *const MultiPoint::Clone() const
{
	return new MultiPoint(*this);
}
