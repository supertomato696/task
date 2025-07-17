/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:LinearRing.cpp
简要描述:
******************************************************************/

#include "Geometries/LinearRing.h"

using namespace Engine::Geometries;
using namespace Engine::Base;

// 默认构造函数
LinearRing::LinearRing()
	: LineString()
{
}

LinearRing::LinearRing(Array<Coordinate *> *coordinates)
	: LineString(coordinates)
{
}

// 析构函数
LinearRing::~LinearRing()
{
}

// 复制构造函数
LinearRing::LinearRing(const LinearRing &rhs)
	: LineString(rhs)
{
}

// 赋值操作符
LinearRing &LinearRing::operator=(const LinearRing &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源

		// 然后进行深拷贝操作
		LineString::operator=(rhs);
	}

	return *this;
}

Bool LinearRing::IsClosed() const
{
	if (LineString::IsEmpty())
	{
		// empty LinearRings are closed by definition
		return true;
	}
	return LineString::IsClosed();
}

GeometryType LinearRing::GetGeometryType() const
{
	return GeometryType::LINEARRING;
}

Geometry *const LinearRing::Reverse() const
{
	SizeT npts = m_coordinates->GetCount();

	Array<Coordinate *> *coords = new Array<Coordinate *>();

	for (SizeT i = 0; i < npts; ++i)
	{
		Coordinate *coord = new Coordinate(*(*m_coordinates)[i]);
		coords->Add(coord);
	}

	coords->Reverse();
	LinearRing *linearRing = new LinearRing(coords);

	return linearRing;
}

Bool LinearRing::Distinct()
{
	if (m_coordinates->GetCount() < 3)
	{
		return false;
	}

	Array<Int32> deletes;
	for (int i = 0; i < m_coordinates->GetCount() - 2; i++)
	{
		Coordinate coord1 = *m_coordinates->GetAt(i);
		for (int j = i + 1; j < m_coordinates->GetCount() - 1; j++)
		{
			Coordinate coord2 = *m_coordinates->GetAt(j);
			if (coord1.Distance(coord2) < Geometries_EP)
			{
				deletes.Add(i);
			}
		}
	}

	Array<Coordinate *> *coords = new Array<Coordinate *>();
	for (int i = 0; i < m_coordinates->GetCount(); i++)
	{
		if (deletes.Find(i) == deletes.End())
		{
			coords->Add(new Coordinate(*m_coordinates->GetAt(i)));
		}
	}

	// 先释放当前资源
	for (Int32 j = 0; j < m_coordinates->GetCount(); j++)
	{
		delete (*m_coordinates)[j];
	}

	delete m_coordinates;
	m_coordinates = NULL;

	m_coordinates = coords;
	return true;
}
