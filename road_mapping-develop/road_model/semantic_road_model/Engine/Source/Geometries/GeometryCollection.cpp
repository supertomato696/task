/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:GeometryCollection.cpp
简要描述:
******************************************************************/

#include "Geometries/GeometryCollection.h"

using namespace Engine::Base;
using namespace Engine::Geometries;

// 默认构造函数
GeometryCollection::GeometryCollection()
	: m_geometries(new Array<Geometry *>())
{
}

GeometryCollection::GeometryCollection(Array<Geometry *> *newGeoms)
{
	m_geometries = newGeoms;
}

// 析构函数
GeometryCollection::~GeometryCollection()
{
	for (SizeT i = 0; i < m_geometries->GetCount(); ++i)
	{
		delete (*m_geometries)[i];
	}

	delete m_geometries;
}

// 复制构造函数
GeometryCollection::GeometryCollection(const GeometryCollection &rhs)
{
	SizeT ngeoms = rhs.m_geometries->GetCount();

	m_geometries = new Array<Geometry *>(ngeoms);
	for (SizeT i = 0; i < ngeoms; ++i)
	{
		(*m_geometries)[i] = (*rhs.m_geometries)[i]->Clone();
	}
}

// 赋值操作符
GeometryCollection &GeometryCollection::operator=(const GeometryCollection &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		for (SizeT i = 0; i < m_geometries->GetCount(); ++i)
		{
			delete (*m_geometries)[i];
		}

		delete m_geometries;

		// 然后进行深拷贝操作

		SizeT ngeoms = rhs.m_geometries->GetCount();

		m_geometries = new Array<Geometry *>(ngeoms);
		for (SizeT i = 0; i < ngeoms; ++i)
		{
			(*m_geometries)[i] = (*rhs.m_geometries)[i]->Clone();
		}
	}

	return *this;
}

SizeT GeometryCollection::GetNumGeometries() const
{
	return m_geometries->GetCount();
}

Geometry *const GeometryCollection::GetGeometryN(SizeT index) const
{
	return (*m_geometries)[index];
}

GeometryType GeometryCollection::GetGeometryType() const
{
	return GeometryType::GEOMETRYCOLLECTION;
}

Envelope *const GeometryCollection::GetEnvelope() const
{
	Envelope *envelope = new Envelope();
	for (SizeT i = 0; i < m_geometries->GetCount(); i++)
	{
		const Envelope *env = (*m_geometries)[i]->GetEnvelope();
		envelope->ExpandToInclude(env);
	}
	return envelope;
}

Geometry *const GeometryCollection::Clone() const
{
	return new GeometryCollection(*this);
}