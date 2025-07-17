/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:MultiLineString.cpp
简要描述:
******************************************************************/

#include "Geometries/MultiLineString.h"
#include <exception>
#include <stdexcept>
using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

// 默认构造函数
MultiLineString::MultiLineString()
	: GeometryCollection()
{
}

MultiLineString::MultiLineString(Array<Geometry *> *geoms)
	: GeometryCollection(geoms)
{
	SizeT ngeoms = m_geometries->GetCount();
	for (SizeT i = 0; i < ngeoms; ++i)
	{
		if (GeometryType::LINESTRING != m_geometries->GetAt(i)->GetGeometryType() &&
			GeometryType::LINEARRING != m_geometries->GetAt(i)->GetGeometryType())
		{

#ifdef _WIN32
			throw std::exception("MultiLineString只能包含LineString类型和Linear类型的Geometry");
#else
			logic_error ex("MultiLineString只能包含LineString类型和Linear类型的Geometry");
			throw std::exception(ex);
#endif
		}
	}
}

// 析构函数
MultiLineString::~MultiLineString()
{
}

// 复制构造函数
MultiLineString::MultiLineString(const MultiLineString &rhs)
	: GeometryCollection(rhs)
{
}

// 赋值操作符
MultiLineString &MultiLineString::operator=(const MultiLineString &rhs)
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

GeometryType MultiLineString::GetGeometryType() const
{
	return GeometryType::MULTILINESTRING;
}

Geometry *const MultiLineString::Clone() const
{
	return new MultiLineString(*this);
}
