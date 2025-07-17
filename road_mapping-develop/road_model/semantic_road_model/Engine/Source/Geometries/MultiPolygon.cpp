/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiPolygon.cpp
简要描述:
******************************************************************/

#include "Geometries/MultiPolygon.h"
#include <exception>
#include <stdexcept>

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

// 默认构造函数
MultiPolygon::MultiPolygon()
	: GeometryCollection()
{
}

MultiPolygon::MultiPolygon(Array<Geometry *> *geoms)
	: GeometryCollection(geoms)
{
	SizeT ngeoms = m_geometries->GetCount();
	for (SizeT i = 0; i < ngeoms; ++i)
	{
		if (GeometryType::POLYGON != m_geometries->GetAt(i)->GetGeometryType())
		{

#ifdef _WIN32
			throw std::exception("MultiPolygon只能包含Polygon类型的Geometry");
#else
			logic_error ex("MultiPolygon只能包含Polygon类型的Geometry");
			throw std::exception(ex);
#endif
		}
	}
}

// 析构函数
MultiPolygon::~MultiPolygon()
{
}

// 复制构造函数
MultiPolygon::MultiPolygon(const MultiPolygon &rhs)
	: GeometryCollection(rhs)
{
}

// 赋值操作符
MultiPolygon &MultiPolygon::operator=(const MultiPolygon &rhs)
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

GeometryType MultiPolygon::GetGeometryType() const
{
	return GeometryType::MULTIPOLYGON;
}
