/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:LineString.cpp
简要描述:
******************************************************************/

#include "Geometries/LineString.h"
#include "Geometries/MultiPoint.h"
#include "Geometries/GeomAdapter.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include <list>
#include <exception>
#include <stdexcept>
#ifdef _WIN32
#include <windows.h>
#endif

#include <string.h>

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace std;

// 默认构造函数
LineString::LineString()
	: m_coordinates(new Array<Coordinate *>())
{
}

LineString::LineString(Array<Coordinate *> *coordinates)
{
	m_coordinates = coordinates;
}

LineString::LineString(Coordinate *coord1, Coordinate *coord2)
{
	m_coordinates = new Base::Array<Coordinate *>(2);
	(*m_coordinates)[0] = new Coordinate(*coord1);
	(*m_coordinates)[1] = new Coordinate(*coord2);
}

// 析构函数
LineString::~LineString()
{
	for (SizeT i = 0; i < m_coordinates->GetCount(); ++i)
	{
		delete (*m_coordinates)[i];
		(*m_coordinates)[i] = nullptr;
	}

	delete m_coordinates;
	m_coordinates = nullptr;
}

// 复制构造函数
LineString::LineString(const LineString &rhs)
{
	SizeT ncoords = rhs.GetCoordinates()->GetCount();

	m_coordinates = new Array<Coordinate *>(ncoords);
	for (SizeT i = 0; i < ncoords; ++i)
	{
		(*m_coordinates)[i] = new Coordinate((*rhs.m_coordinates)[i]->x, (*rhs.m_coordinates)[i]->y, (*rhs.m_coordinates)[i]->z);
	}
}

// 赋值操作符
LineString &LineString::operator=(const LineString &rhs)
{
	// 检查自赋值
	if (this != &rhs)
	{
		// 先释放当前资源
		for (SizeT i = 0; i < m_coordinates->GetCount(); ++i)
		{
			delete (*m_coordinates)[i];
		}

		delete m_coordinates;
		m_coordinates = NULL;

		// 然后进行深拷贝操作
		SizeT ncoords = rhs.m_coordinates->GetCount();

		m_coordinates = new Array<Coordinate *>(ncoords);

		for (SizeT i = 0; i < ncoords; ++i)
		{
			(*m_coordinates)[i] = new Coordinate((*rhs.m_coordinates)[i]->x, (*rhs.m_coordinates)[i]->y, (*rhs.m_coordinates)[i]->z);
		}
	}

	return *this;
}

GeometryType LineString::GetGeometryType() const
{
	return GeometryType::LINESTRING;
}

Envelope *const LineString::GetEnvelope() const
{
	if (IsEmpty())
	{
		// We don't return NULL here
		// as it would indicate "unknown"
		// envelope. In this case we
		// *know* the envelope is EMPTY.
		return new Envelope();
	}

	Coordinate *c = m_coordinates->GetAt(0);
	double minx = c->x;
	double miny = c->y;
	double maxx = c->x;
	double maxy = c->y;
	SizeT npts = m_coordinates->GetCount();
	for (SizeT i = 1; i < npts; ++i)
	{
		c = m_coordinates->GetAt(i);
		minx = minx < c->x ? minx : c->x;
		maxx = maxx > c->x ? maxx : c->x;
		miny = miny < c->y ? miny : c->y;
		maxy = maxy > c->y ? maxy : c->y;
	}

	return new Envelope(minx, miny, maxx, maxy);
}

//{{qiuli 2017.2.28
Envelope LineString::GetBound() const
{
	if (IsEmpty())
	{
		return Envelope();
	}

	Coordinate *c = m_coordinates->GetAt(0);
	double minx = c->x;
	double miny = c->y;
	double maxx = c->x;
	double maxy = c->y;

	SizeT npts = m_coordinates->GetCount();

	for (SizeT i = 1; i < npts; ++i)
	{
		c = m_coordinates->GetAt(i);
		minx = minx < c->x ? minx : c->x;
		maxx = maxx > c->x ? maxx : c->x;
		miny = miny < c->y ? miny : c->y;
		maxy = maxy > c->y ? maxy : c->y;
	}

	return Envelope(minx, miny, maxx, maxy);
}
//}}qiuli 2017.2.28

Envelope3D *const Engine::Geometries::LineString::GetEnvelope3D() const
{
	if (IsEmpty())
	{
		return new Envelope3D();
	}

	Coordinate *c = m_coordinates->GetAt(0);
	double minx = c->x;
	double miny = c->y;
	double minz = c->z;
	double maxx = c->x;
	double maxy = c->y;
	double maxz = c->z;
	SizeT npts = m_coordinates->GetCount();
	for (SizeT i = 1; i < npts; i++)
	{
		c = m_coordinates->GetAt(i);
		minx = minx < c->x ? minx : c->x;
		maxx = maxx > c->x ? maxx : c->x;
		miny = miny < c->y ? miny : c->y;
		maxy = maxy > c->y ? maxy : c->y;
		minz = minz < c->z ? minz : c->z;
		maxz = maxz > c->z ? maxz : c->z;
	}

	return new Envelope3D(minx, miny, maxx, maxy, minz, maxz);
}

Bool LineString::IsSameDirection(const LineString *rhs) const
{
	Geometries::Coordinate vec1 = (*m_coordinates->GetAt(0) - *m_coordinates->GetAt(m_coordinates->GetCount() - 1)); // AC
	Geometries::Coordinate vec2 = (*rhs->GetCoordinateN(0) - *rhs->GetCoordinateN(rhs->GetNumPoints() - 1));		 // AC
	if (vec1.DotProduct(vec2) > 0)
		return true;
	return false;
}

Bool LineString::IsSameDirection2(const LineString *rhs) const
{
	// 最近两端点
	Geometries::Coordinate mSCoordinate = *m_coordinates->GetAt(0);
	Geometries::Coordinate mECoordinate = *m_coordinates->GetAt(m_coordinates->GetCount() - 1);
	Geometries::Coordinate rhsSCoordinate = *rhs->GetCoordinateN(0);
	Geometries::Coordinate rhsECoordinate = *rhs->GetCoordinateN(rhs->GetNumPoints() - 1);

	// 最近两段线
	Double dist = mSCoordinate.Distance(rhsSCoordinate);
	Geometries::Coordinate vec1 = mSCoordinate - *m_coordinates->GetAt(1);
	Geometries::Coordinate vec2 = rhsSCoordinate - *rhs->GetCoordinateN(1);
	if (dist > mSCoordinate.Distance(rhsECoordinate))
	{
		dist = mSCoordinate.Distance(rhsECoordinate);
		vec1 = mSCoordinate - *m_coordinates->GetAt(1);
		vec2 = *rhs->GetCoordinateN(rhs->GetNumPoints() - 2) - rhsECoordinate;
	}
	if (dist > mECoordinate.Distance(rhsSCoordinate))
	{
		dist = mECoordinate.Distance(rhsSCoordinate);
		vec1 = *m_coordinates->GetAt(m_coordinates->GetCount() - 2) - mECoordinate;
		vec2 = rhsSCoordinate - *rhs->GetCoordinateN(1);
	}
	if (dist > mECoordinate.Distance(rhsECoordinate))
	{
		dist = mECoordinate.Distance(rhsECoordinate);
		vec1 = *m_coordinates->GetAt(m_coordinates->GetCount() - 2) - mECoordinate;
		vec2 = *rhs->GetCoordinateN(rhs->GetNumPoints() - 2) - rhsECoordinate;
	}
	if (vec1.DotProduct(vec2) > 0)
		return true;
	return false;
}

Bool LineString::IsSameDirectionEx(const LineString *rhs) const
{
	Coordinate pntProject;
	Array<Int32> arrIndex;
	for (auto itr = m_coordinates->Begin(); itr != m_coordinates->End(); itr++)
	{
		Int32 segIndex = -1;
		if (Geometries::BaseAlgorithm::GetNearestPntToLineset(*itr, rhs->GetCoordinates(), pntProject, segIndex))
		{
			arrIndex.Add(segIndex);
		}
	}
	if (arrIndex.IsEmpty())
	{
		return false;
	}
	if (arrIndex.GetAt(0) == arrIndex.GetAt(arrIndex.GetCount() - 1))
	{
		return IsSameDirection2(rhs);
	}
	return arrIndex.GetAt(0) < arrIndex.GetAt(arrIndex.GetCount() - 1);
}

Bool LineString::IsEmpty() const
{
	return m_coordinates->IsEmpty();
}

SizeT LineString::GetNumPoints() const
{
	return m_coordinates->GetCount();
}

Array<Coordinate *> *const LineString::GetCoordinates() const
{
	return m_coordinates;
}

Geometry *const LineString::Clone() const
{
	LineString *cloneLineString = new LineString(*this);

	return cloneLineString;
}

Coordinate *const LineString::GetCoordinateN(SizeT index) const
{
	if (index >= m_coordinates->GetCount())
	{
#ifdef _WIN32
		throw std::exception("LineString::GetCoordinateN参数index越界");
#else
		logic_error ex("LineString::GetCoordinateN参数index越界");
		throw std::exception(ex);
#endif
	}
	return (*m_coordinates)[index];
}

Point *const LineString::GetPointN(SizeT index) const
{
	if (index >= m_coordinates->GetCount())
	{
#ifdef _WIN32
		throw std::exception("LineString::GetCoordinateN参数index越界");
#else
		logic_error ex("LineString::GetCoordinateN参数index越界");
		throw std::exception(ex);
#endif
	}

	Coordinate *coord = (*m_coordinates)[index];
	Point *pt = new Point(coord->x, coord->y, coord->z);
	return pt;
}

Point *const LineString::GetStartPoint() const
{
	if (IsEmpty())
	{
#ifdef _WIN32
		throw std::exception("LineString::GetCoordinateN参数index越界");
#else
		logic_error ex("LineString::GetCoordinateN参数index越界");
		throw std::exception(ex);
#endif
	}

	Coordinate *coord = (*m_coordinates)[0];
	Point *pt = new Point(coord->x, coord->y, coord->z);
	return pt;
}

Point *const LineString::GetEndPoint() const
{
	if (IsEmpty())
	{
#ifdef _WIN32
		throw std::exception("LineString::GetCoordinateN参数index越界");
#else
		logic_error ex("LineString::GetCoordinateN参数index越界");
		throw std::exception(ex);
#endif
	}

	Coordinate *coord = (*m_coordinates)[m_coordinates->GetCount() - 1];
	Point *pt = new Point(coord->x, coord->y, coord->z);
	return pt;
}

Bool LineString::IsClosed() const
{
	if (IsEmpty())
	{
		return false;
	}
	return GetCoordinateN(0)->Equals((*GetCoordinateN(GetNumPoints() - 1)));
}

Double LineString::GetLength() const
{
	SizeT npts = m_coordinates->GetCount();
	if (npts <= 1)
	{
		return 0.0;
	}

	double len = 0.0;

	Coordinate *p0 = m_coordinates->GetAt(0);

	for (SizeT i = 1; i < npts; ++i)
	{
		Coordinate *p = m_coordinates->GetAt(i);

		len += p0->Distance(*p);

		p0 = p;
	}

	return len;
}
Double LineString::GetPartLength(Int32 from, Int32 to) const
{
	SizeT npts = m_coordinates->GetCount();
	if (from < 0)
	{
		from = 0;
	}
	if (from > m_coordinates->GetCount() - 1)
	{
		from = m_coordinates->GetCount() - 1;
	}
	if (to < 0)
	{
		to = 0;
	}
	if (to > m_coordinates->GetCount() - 1)
	{
		to = m_coordinates->GetCount() - 1;
	}
	if (from > to)
	{
		std::swap(from, to);
	}
	if (from == to)
	{
		return 0.0;
	}
	double len = 0.0;

	Coordinate *p0 = m_coordinates->GetAt(from);

	for (SizeT i = from + 1; i <= to; ++i)
	{
		Coordinate *p = m_coordinates->GetAt(i);

		len += p0->Distance(*p);

		p0 = p;
	}

	return len;
}

Double LineString::GetLengthXY() const
{
	SizeT npts = m_coordinates->GetCount();
	if (npts <= 1)
	{
		return 0.0;
	}

	double len = 0.0;

	Coordinate *p0 = m_coordinates->GetAt(0);

	for (SizeT i = 1; i < npts; ++i)
	{
		Coordinate *p = m_coordinates->GetAt(i);

		len += p0->DistanceXY(*p);

		p0 = p;
	}

	return len;
}

Geometry *const LineString::Reverse() const
{
	SizeT npts = m_coordinates->GetCount();

	Array<Coordinate *> *coords = new Array<Coordinate *>();

	for (SizeT i = 0; i < npts; ++i)
	{
		Coordinate *coord = new Coordinate(*(*m_coordinates)[i]);
		coords->Add(coord);
	}

	coords->Reverse();
	LineString *lineString = new LineString(coords);

	return lineString;
}

Geometry *LineString::CutByLength(double dLength)
{
	Int32 npts = m_coordinates->GetCount();
	if (npts < 2 || dLength <= Geometries_EP || (*this).GetLength() < dLength)
	{
		return nullptr;
	}
	if ((*this).GetLength() == dLength)
	{
		LineString *lineString = new LineString(*this);
		return lineString;
	}
	Array<Coordinate *> *coords = new Array<Coordinate *>();
	coords->Add(new Coordinate(*m_coordinates->GetAt(0)));
	double dRemainderLeth = dLength;
	for (Int32 i = 1; i < npts; i++)
	{
		double dis = m_coordinates->GetAt(i - 1)->Distance(*m_coordinates->GetAt(i));
		if (dis == Geometries_EP)
		{
			continue;
		}
		if (dis < dRemainderLeth)
		{
			coords->Add(new Coordinate(*m_coordinates->GetAt(i)));
			dRemainderLeth -= dis;
		}
		else if (dis == dRemainderLeth)
		{
			coords->Add(new Coordinate(*m_coordinates->GetAt(i)));
			break;
		}
		else
		{
			Coordinate deltCoord = (*m_coordinates->GetAt(i)) - (*m_coordinates->GetAt(i - 1));
			double dPercent = dRemainderLeth / dis;
			Coordinate coord;
			coord.x = m_coordinates->GetAt(i - 1)->x + deltCoord.x * dPercent;
			coord.y = m_coordinates->GetAt(i - 1)->y + deltCoord.y * dPercent;
			coord.z = m_coordinates->GetAt(i - 1)->z + deltCoord.z * dPercent;
			coords->Add(new Coordinate(coord));
			break;
		}
	}
	LineString *pls = new LineString(coords);
	return pls;
}
Bool LineString::Discretization(Double distance)
{
	Int32 npts = m_coordinates->GetCount();

	if ((npts < 2) || (distance <= 0.0))
	{
		return false;
	}

	Array<Coordinate *> *coords = new Array<Coordinate *>();
	Coordinate *coord = NULL;
	Coordinate *pFrom = NULL;
	Coordinate *pTo = NULL;
	Double len = 0.0;
	Int32 nInsertPoints = 0;
	Double value = 0.0;

	for (Int32 i = 0; i < npts - 1; i++)
	{
		pFrom = m_coordinates->GetAt(i);
		pTo = m_coordinates->GetAt(i + 1);
		len = BaseAlgorithm::DistancePtToPt(pFrom, pTo);

		// 小于插点距离就无需插点处理，直接返回成功
		if (len < 2 * distance)
		{
			coord = new Coordinate(*pFrom);
			coords->Add(coord);

			continue;
		}

		nInsertPoints = Int32(len / distance);
		value = len - nInsertPoints * distance;

		if (value < distance)
		{
			nInsertPoints--;
		}

		for (Int32 j = 0; j <= nInsertPoints; j++)
		{
			coord = new Coordinate(*pFrom);
			coord->x = pFrom->x + (pTo->x - pFrom->x) * (j * distance) / len;
			coord->y = pFrom->y + (pTo->y - pFrom->y) * (j * distance) / len;
			coord->z = pFrom->z + (pTo->z - pFrom->z) * (j * distance) / len;
			coords->Add(coord);
		}
	}

	coord = new Coordinate(*m_coordinates->GetAt(npts - 1));
	coords->Add(coord);

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

Bool LineString::Discretization3D(Double distance)
{
	Int32 npts = m_coordinates->GetCount();

	if ((npts < 2) || (distance <= 0.0))
	{
		return false;
	}

	Array<Coordinate *> *coords = new Array<Coordinate *>();
	Coordinate *coord = NULL;
	Coordinate *pFrom = NULL;
	Coordinate *pTo = NULL;
	Double len = 0.0;
	Int32 nInsertPoints = 0;
	Double value = 0.0;

	for (Int32 i = 0; i < npts - 1; i++)
	{
		pFrom = m_coordinates->GetAt(i);
		pTo = m_coordinates->GetAt(i + 1);
		len = pFrom->Distance(*pTo); // BaseAlgorithm::DistancePtToPt(pFrom, pTo);

		// 小于插点距离就无需插点处理，直接返回成功
		if (len < 2 * distance)
		{
			coord = new Coordinate(*pFrom);
			coords->Add(coord);

			continue;
		}

		nInsertPoints = Int32(len / distance);
		value = len - nInsertPoints * distance;

		if (value < distance)
		{
			nInsertPoints--;
		}

		for (Int32 j = 0; j <= nInsertPoints; j++)
		{
			coord = new Coordinate(*pFrom);
			coord->x = pFrom->x + (pTo->x - pFrom->x) * (j * distance) / len;
			coord->y = pFrom->y + (pTo->y - pFrom->y) * (j * distance) / len;
			coord->z = pFrom->z + (pTo->z - pFrom->z) * (j * distance) / len;
			coords->Add(coord);
		}
	}

	coord = new Coordinate(*m_coordinates->GetAt(npts - 1));
	coords->Add(coord);

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

Bool LineString::Discretization3D2(Double distance)
{
	Int32 npts = m_coordinates->GetCount();

	if ((npts < 2) || (distance <= 0.0))
	{
		return false;
	}
	Array<Coordinate *> *coords = new Array<Coordinate *>();
	Array<Coordinate *> *coordsCpy = new Array<Coordinate *>();
	for (int i = 0; i < npts; i++)
	{
		coordsCpy->Add(new Coordinate(*m_coordinates->GetAt(i)));
	}

	Coordinate *coord = NULL;
	Coordinate *pFrom = NULL;
	Coordinate *pTo = NULL;
	Double len = 0.0;
	Double dThresholld = distance;
	coords->Add(new Coordinate(*m_coordinates->GetAt(0)));
	for (auto itr = coordsCpy->Begin(); itr != coordsCpy->End() - 1;)
	{
		pFrom = (*itr);
		pTo = (*(itr + 1));
		len = pFrom->Distance(*pTo);
		if (len < dThresholld)
		{
			dThresholld -= len;
			itr = coordsCpy->Erase(itr);
			continue;
		}

		coord = new Coordinate(*pFrom);
		coord->x = pFrom->x + (pTo->x - pFrom->x) * (dThresholld) / len;
		coord->y = pFrom->y + (pTo->y - pFrom->y) * (dThresholld) / len;
		coord->z = pFrom->z + (pTo->z - pFrom->z) * (dThresholld) / len;
		coords->Add(coord);
		(*itr)->x = coord->x;
		(*itr)->y = coord->y;
		(*itr)->z = coord->z;
		dThresholld = distance;
	}
	coords->Add(new Coordinate(*m_coordinates->GetAt(m_coordinates->GetCount() - 1)));
	if (coords->GetAt(coords->GetCount() - 2)->Distance(*coords->GetAt(coords->GetCount() - 1)) == 0)
	{
		coords->Delete(coords->GetCount() - 1);
	}
	// 先释放当前资源
	for (Int32 j = 0; j < m_coordinates->GetCount(); j++)
	{
		delete (*m_coordinates)[j];
	}

	delete m_coordinates;
	m_coordinates = NULL;
	m_coordinates = coords;
	// 	for (Int32 i = 0; i < coords->GetCount() - 1; i++)
	// 	{
	// 		pFrom = coords->GetAt(i);
	// 		pTo = coords->GetAt(i + 1);
	// 		Double dis = pFrom->Distance(*pTo);
	// 		if (fabs(dis - distance) > Geometries_EP)
	// 		{
	// 			int i = 0;
	// 		}
	// 	}
	return true;
}

bool LineString::Resampling(double tolerance)
{
	if ((m_coordinates == NULL) || (m_coordinates->GetCount() <= 0) || (tolerance <= 0.0))
	{
		return false;
	}

	SizeT nCount = m_coordinates->GetCount();
	unsigned char *pID = new unsigned char[nCount];

	if (pID == NULL)
	{
		return false;
	}

	memset(pID, 0, nCount);
	pID[0] = 1;
	pID[nCount - 1] = 1;
	list<SizeT> lstActiveID;
	lstActiveID.push_back(0);
	lstActiveID.push_back(nCount - 1);
	int nFromID, nToID, nMaxDistID;
	double dMaxDist, dDistToLine;

	int i = 0;
	list<SizeT>::iterator iterSE;
	list<SizeT>::iterator iterInsert;

	while (lstActiveID.size() > 1)
	{
		nMaxDistID = 0;
		dMaxDist = 0.0;
		iterSE = lstActiveID.begin();
		nFromID = (int)*iterSE;
		iterSE++;
		nToID = (int)*iterSE;

		for (i = nFromID + 1; i < nToID; i++)
		{
			dDistToLine = BaseAlgorithm::DistancePtToLine((*m_coordinates)[i], (*m_coordinates)[nFromID], (*m_coordinates)[nToID]);

			if (dMaxDist < dDistToLine)
			{
				dMaxDist = dDistToLine;
				nMaxDistID = i;
			}
		}

		if (dMaxDist > tolerance)
		{
			pID[nMaxDistID] = 1;
			iterInsert = lstActiveID.begin();
			iterInsert++;
			lstActiveID.insert(iterInsert, nMaxDistID);
		}
		else
		{
			lstActiveID.pop_front();
		}
	}

	for (i = (int)nCount - 1; i >= 0; i--)
	{
		if (pID[i] == 0)
		{
			delete (*m_coordinates)[i];
			m_coordinates->Delete(i);
		}
	}

	if (pID != NULL)
	{
		delete[] pID;
		pID = NULL;
	}

	return true;
}

bool Engine::Geometries::LineString::Flexibility(Base::SizeT nIndex, const Coordinate *pNewPos)
{
	SizeT nCount = m_coordinates->GetCount();
	if ((nIndex < 0) || (nIndex >= nCount)) // 调整点索引越界
	{
		return false;
	}

	if (nIndex == 0) // 移动首点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pEndPos = m_coordinates->GetAt(nCount - 1);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		double dEndToOld = BaseAlgorithm::DistancePtToPt(pOldPos, pEndPos);
		double dEndToNew = BaseAlgorithm::DistancePtToPt(pNewPos, pEndPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		double dAngleOld = BaseAlgorithm::CalAzimuth(pEndPos, pOldPos);
		double dAngleNew = BaseAlgorithm::CalAzimuth(pEndPos, pNewPos);
		double dRotateAngle = dAngleOld - dAngleNew;

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pEndPos->x + (pCurrentPos->x - pEndPos->x) * dRatio;
			pCurrentPos->y = pEndPos->y + (pCurrentPos->y - pEndPos->y) * dRatio;

			BaseAlgorithm::RotatePoint(pEndPos, dRotateAngle, pCurrentPos);
		}

		pOldPos->x = pNewPos->x;
		pOldPos->y = pNewPos->y;
	}
	else if (nIndex == (nCount - 1)) // 移动尾点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pOriginPos = m_coordinates->GetAt(0);

		double dOriginToOld = BaseAlgorithm::DistancePtToPt(pOriginPos, pOldPos);
		double dOriginToNew = BaseAlgorithm::DistancePtToPt(pOriginPos, pNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dAngleOld = BaseAlgorithm::CalAzimuth(pOriginPos, pOldPos);
		double dAngleNew = BaseAlgorithm::CalAzimuth(pOriginPos, pNewPos);
		double dRotateAngle = dAngleOld - dAngleNew;

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		for (i = 1; i < nIndex; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pOriginPos->x + (pCurrentPos->x - pOriginPos->x) * dRatio;
			pCurrentPos->y = pOriginPos->y + (pCurrentPos->y - pOriginPos->y) * dRatio;

			BaseAlgorithm::RotatePoint(pOriginPos, dRotateAngle, pCurrentPos);
		}

		pOldPos->x = pNewPos->x;
		pOldPos->y = pNewPos->y;
	}
	else // 移动中间点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pOriginPos = m_coordinates->GetAt(0);
		Coordinate *pEndPos = m_coordinates->GetAt(nCount - 1);

		double dOriginToOld = BaseAlgorithm::DistancePtToPt(pOriginPos, pOldPos);
		double dOriginToNew = BaseAlgorithm::DistancePtToPt(pOriginPos, pNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dAngleOld = BaseAlgorithm::CalAzimuth(pOriginPos, pOldPos);
		double dAngleNew = BaseAlgorithm::CalAzimuth(pOriginPos, pNewPos);
		double dRotateAngle = dAngleOld - dAngleNew;

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		for (i = 1; i < nIndex; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pOriginPos->x + (pCurrentPos->x - pOriginPos->x) * dRatio;
			pCurrentPos->y = pOriginPos->y + (pCurrentPos->y - pOriginPos->y) * dRatio;

			BaseAlgorithm::RotatePoint(pOriginPos, dRotateAngle, pCurrentPos);
		}

		double dEndToOld = BaseAlgorithm::DistancePtToPt(pEndPos, pOldPos);
		double dEndToNew = BaseAlgorithm::DistancePtToPt(pEndPos, pNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		dAngleOld = BaseAlgorithm::CalAzimuth(pEndPos, pOldPos);
		dAngleNew = BaseAlgorithm::CalAzimuth(pEndPos, pNewPos);
		dRotateAngle = dAngleOld - dAngleNew;

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pEndPos->x + (pCurrentPos->x - pEndPos->x) * dRatio;
			pCurrentPos->y = pEndPos->y + (pCurrentPos->y - pEndPos->y) * dRatio;

			BaseAlgorithm::RotatePoint(pEndPos, dRotateAngle, pCurrentPos);
		}

		pOldPos->x = pNewPos->x;
		pOldPos->y = pNewPos->y;
	}

	return true;
}

//{{qiuli 2017.3.14
bool Engine::Geometries::LineString::Flexibility3D(Base::SizeT nIndex, const Coordinate *pNewPos)
{
	SizeT nCount = m_coordinates->GetCount();
	if ((nIndex < 0) || (nIndex >= nCount)) // 调整点索引越界
	{
		return false;
	}

	if (nIndex == 0) // 移动首点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pEndPos = m_coordinates->GetAt(nCount - 1);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		double dEndToOld = pEndPos->Distance(*pOldPos);
		double dEndToNew = pEndPos->Distance(*pNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pEndPos, *pNewPos);

		Coordinate e1 = (*pOldPos) - (*pEndPos);
		Coordinate e2 = (*pNewPos) - (*pEndPos);

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pEndPos->x + (pCurrentPos->x - pEndPos->x) * dRatio;
			pCurrentPos->y = pEndPos->y + (pCurrentPos->y - pEndPos->y) * dRatio;
			pCurrentPos->z = pEndPos->z + (pCurrentPos->z - pEndPos->z) * dRatio;

			BaseAlgorithm3D::RotatePoint(*pEndPos, dRotateAngle, NormalVector, *pCurrentPos);
		}

		*pOldPos = *pNewPos;
	}
	else if (nIndex == (nCount - 1)) // 移动尾点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pOriginPos = m_coordinates->GetAt(0);

		double dOriginToOld = pOriginPos->Distance(*pOldPos);
		double dOriginToNew = pOriginPos->Distance(*pNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pOriginPos, *pNewPos);

		Coordinate e1 = (*pOldPos) - (*pOriginPos);
		Coordinate e2 = (*pNewPos) - (*pOriginPos);

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		for (i = 1; i < nIndex; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pOriginPos->x + (pCurrentPos->x - pOriginPos->x) * dRatio;
			pCurrentPos->y = pOriginPos->y + (pCurrentPos->y - pOriginPos->y) * dRatio;
			pCurrentPos->z = pOriginPos->z + (pCurrentPos->z - pOriginPos->z) * dRatio;

			BaseAlgorithm3D::RotatePoint(*pOriginPos, dRotateAngle, NormalVector, *pCurrentPos);
		}

		*pOldPos = *pNewPos;
	}
	else // 移动中间点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nIndex);
		Coordinate *pOriginPos = m_coordinates->GetAt(0);
		Coordinate *pEndPos = m_coordinates->GetAt(nCount - 1);

		double dOriginToOld = pOriginPos->Distance(*pOldPos);
		double dOriginToNew = pOriginPos->Distance(*pNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pOriginPos, *pNewPos);

		Coordinate e1 = (*pOldPos) - (*pOriginPos);
		Coordinate e2 = (*pNewPos) - (*pOriginPos);

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		for (i = 1; i < nIndex; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pOriginPos->x + (pCurrentPos->x - pOriginPos->x) * dRatio;
			pCurrentPos->y = pOriginPos->y + (pCurrentPos->y - pOriginPos->y) * dRatio;
			pCurrentPos->z = pOriginPos->z + (pCurrentPos->z - pOriginPos->z) * dRatio;

			BaseAlgorithm3D::RotatePoint(*pOriginPos, dRotateAngle, NormalVector, *pCurrentPos);
		}

		double dEndToOld = pEndPos->Distance(*pOldPos); // BaseAlgorithm::DistancePtToPt(pEndPos, pOldPos);
		double dEndToNew = pEndPos->Distance(*pNewPos); // BaseAlgorithm::DistancePtToPt(pEndPos, pNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			return false;
		}

		dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pEndPos, *pNewPos);

		e1 = (*pOldPos) - (*pEndPos);
		e2 = (*pNewPos) - (*pEndPos);

		e1.Normalize();
		e2.Normalize();

		NormalVector = e1.CrossProduct(e2);

		for (i = nIndex + 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pEndPos->x + (pCurrentPos->x - pEndPos->x) * dRatio;
			pCurrentPos->y = pEndPos->y + (pCurrentPos->y - pEndPos->y) * dRatio;
			pCurrentPos->z = pEndPos->z + (pCurrentPos->z - pEndPos->z) * dRatio;

			BaseAlgorithm3D::RotatePoint(*pEndPos, dRotateAngle, NormalVector, *pCurrentPos);
		}

		*pOldPos = *pNewPos;
	}

	return true;
}
//}}qiuli 2017.3.14

Bool LineString::GetMAtPoint(const Coordinate *pntProject, Int32 nSegIndex, Double &dMeasure)
{
	SizeT nCount = m_coordinates->GetCount();

	if ((nSegIndex < 0) || (nSegIndex >= (nCount - 1)))
	{
		return false;
	}

	Double dDis = 0.0;
	UInt32 i = 0;

	for (i = 0; i < nSegIndex; i++)
	{
		dDis += BaseAlgorithm::DistancePtToPt(m_coordinates->GetAt(i), m_coordinates->GetAt(i + 1));
	}

	dDis += BaseAlgorithm::DistancePtToPt(m_coordinates->GetAt(nSegIndex), pntProject);
	dMeasure = dDis;

	return true;
}

Void LineString::Translation(Double dx, Double dy, Double dz)
{
	for (Int32 i = 0; i < m_coordinates->GetCount(); i++)
	{
		(*m_coordinates)[i]->x += dx;
		(*m_coordinates)[i]->y += dy;
		(*m_coordinates)[i]->z += dz;
	}
}

Bool LineString::GetPointAtM(const Double dMeasure, Coordinate &pntProject, Int32 &nSegIndex)
{
	SizeT nCount = m_coordinates->GetCount();

	if (nCount <= 1)
	{
		return false;
	}

	if (BaseAlgorithm::Is0(dMeasure))
	{
		pntProject = *(m_coordinates->GetAt(0)); // 首点
		nSegIndex = 0;
	}

	if (dMeasure < 0.0) // 非法
	{
		return false;
	}

	Double dDis = 0.0;
	UInt32 i = 0;

	for (i = 0; i < nCount - 1; i++) // 计算线的长度
	{
		double dDisSeg = BaseAlgorithm::DistancePtToPt(m_coordinates->GetAt(i), m_coordinates->GetAt(i + 1));
		if ((dDis + dDisSeg) >= dMeasure) // 到该段
		{
			double dDisTemp = dMeasure - dDis;
			pntProject = (*(m_coordinates->GetAt(i))) + ((*(m_coordinates->GetAt(i + 1))) - (*(m_coordinates->GetAt(i)))) * (dDisTemp / dDisSeg);
			nSegIndex = i;

			return true; // 到就返回吧
		}

		dDis += dDisSeg;
	}

	if (BaseAlgorithm::Is0(dMeasure - dDis)) // 尾点
	{
		pntProject = *(m_coordinates->GetAt(nCount - 1));
		nSegIndex = nCount - 2;

		return true;
	}

	return false;
}

bool LineString::DeclineRotate3D(bool bIsStart, const Coordinate *pNewPos, unsigned int iDecCoeff)
{
	SizeT nCount = m_coordinates->GetCount();
	if (nCount < 2) // 调整点索引越界
	{
		return false;
	}

	auto MathSqrt = [](Double d, SizeT pow) -> Double
	{
		Double rst = d;
		for (int i = 0; i < pow; i++)
		{
			rst = Math::Sqrt(rst);
		}
		return rst;
	};

	LineString *pLineTmp = (LineString *)this->Clone();
	Double dLength = pLineTmp->GetLength();

	if (bIsStart) // 移动首点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(0);
		Coordinate *pEndPos = m_coordinates->GetAt(nCount - 1);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		double dEndToOld = pEndPos->Distance(*pOldPos);
		double dEndToNew = pEndPos->Distance(*pNewPos);

		if ((dEndToNew < 1.0e-10) || (dEndToOld < 1.0e-10)) // 异常数据
		{
			DELETE_PTR(pLineTmp);
			return false;
		}

		double dRatio = dEndToNew / dEndToOld; // 后一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pEndPos, *pNewPos);

		Coordinate e1 = (*pOldPos) - (*pEndPos);
		Coordinate e2 = (*pNewPos) - (*pEndPos);

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		for (i = 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pEndPos->x + (pCurrentPos->x - pEndPos->x) * dRatio;
			pCurrentPos->y = pEndPos->y + (pCurrentPos->y - pEndPos->y) * dRatio;
			pCurrentPos->z = pEndPos->z + (pCurrentPos->z - pEndPos->z) * dRatio;

			Double dScale = pLineTmp->GetDistanceBetweenTowPoint3D(i, nCount - 1) / dLength;

			BaseAlgorithm3D::RotatePoint(*pEndPos, dRotateAngle * MathSqrt(dScale, iDecCoeff), NormalVector, *pCurrentPos);
		}

		*pOldPos = *pNewPos;
	}
	else // 移动尾点
	{
		Coordinate *pOldPos = m_coordinates->GetAt(nCount - 1);
		Coordinate *pOriginPos = m_coordinates->GetAt(0);

		double dOriginToOld = pOriginPos->Distance(*pOldPos);
		double dOriginToNew = pOriginPos->Distance(*pNewPos);

		if ((dOriginToNew < 1.0e-10) || (dOriginToOld < 1.0e-10)) // 异常数据
		{
			DELETE_PTR(pLineTmp);
			return false;
		}

		double dRatio = dOriginToNew / dOriginToOld; // 前一段缩放比例

		// 逆时针旋转角度
		double dRotateAngle = BaseAlgorithm::ComputeAngle(*pOldPos, *pOriginPos, *pNewPos);

		Coordinate e1 = (*pOldPos) - (*pOriginPos);
		Coordinate e2 = (*pNewPos) - (*pOriginPos);

		e1.Normalize();
		e2.Normalize();

		Coordinate NormalVector = e1.CrossProduct(e2);

		SizeT i = 0;
		Coordinate *pCurrentPos = NULL;

		for (i = 1; i < nCount - 1; ++i)
		{
			pCurrentPos = m_coordinates->GetAt(i);

			pCurrentPos->x = pOriginPos->x + (pCurrentPos->x - pOriginPos->x) * dRatio;
			pCurrentPos->y = pOriginPos->y + (pCurrentPos->y - pOriginPos->y) * dRatio;
			pCurrentPos->z = pOriginPos->z + (pCurrentPos->z - pOriginPos->z) * dRatio;

			Double dScale = pLineTmp->GetDistanceBetweenTowPoint3D(0, i) / dLength;

			BaseAlgorithm3D::RotatePoint(*pOriginPos, dRotateAngle * MathSqrt(dScale, iDecCoeff), NormalVector, *pCurrentPos);
		}

		*pOldPos = *pNewPos;
	}

	DELETE_PTR(pLineTmp);
	return true;
}

Double LineString::GetDistanceBetweenTowPoint(SizeT nIndexS, SizeT nIndexE)
{
	if (nIndexS > nIndexE || m_coordinates->GetCount() - 1 < nIndexE)
	{
		return -1.0;
	}
	Double dDis = 0.0;
	for (int i = nIndexS; i < nIndexE; i++)
	{
		dDis += m_coordinates->GetAt(i)->DistanceXY(*m_coordinates->GetAt(i + 1));
	}
	return dDis;
}

Double LineString::GetDistanceBetweenTowPoint3D(SizeT nIndexS, SizeT nIndexE)
{
	if (nIndexS > nIndexE || m_coordinates->GetCount() - 1 < nIndexE)
	{
		return -1.0;
	}
	Double dDis = 0.0;
	for (int i = nIndexS; i < nIndexE; i++)
	{
		dDis += m_coordinates->GetAt(i)->Distance(*m_coordinates->GetAt(i + 1));
	}
	return dDis;
}

Bool LineString::Distinct()
{
	if (m_coordinates->GetCount() < 3)
	{
		return false;
	}

	Array<Int32> deletes;
	for (int i = 0; i < m_coordinates->GetCount() - 1; i++)
	{
		Coordinate coord1 = *m_coordinates->GetAt(i);
		for (int j = i + 1; j < m_coordinates->GetCount(); j++)
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

Bool LineString::DistinctXY()
{
	if (m_coordinates == NULL || m_coordinates->GetCount() < 2)
	{
		return false;
	}

	for (int i = m_coordinates->GetCount() - 1; i > 0; i--)
	{
		if (m_coordinates->GetAt(i)->DistanceXY(*m_coordinates->GetAt(i - 1)) < Geometries_EP)
		{
			Geometries::Coordinate *pCoorTmp = m_coordinates->GetAt(i);
			m_coordinates->Delete(i);
			DELETE_PTR(pCoorTmp);
		}
	}
	return true;
}

Bool LineString::Smooth(Double tolerance)
{
	if (m_coordinates->IsEmpty())
	{
		return false;
	}

	auto smoothing = [=](Array<Coordinate *> *coords, Double tolerance) -> Bool
	{
		bool res = false;
		for (SizeT i = 1; i < coords->GetCount() - 1; /*i++*/)
		{
			Coordinate *prev = coords->GetAt(i - 1);
			Coordinate *ccur = coords->GetAt(i);
			Coordinate *next = coords->GetAt(i + 1);

			if (BaseAlgorithm3D::DisPtToLine(*prev, *next, *ccur) > tolerance)
			{
				// delete coords->GetAt(i);
				auto pDelete = coords->GetAt(i);

				coords->Delete(i);
				coords->InsertAt(i, new Coordinate((*prev + *ccur) / 2));
				coords->InsertAt(i + 1, new Coordinate((*next + *ccur) / 2));
				res = true;
				i += 2;
				delete pDelete;
			}
			else
			{
				i++;
			}
		}
		return res;
	};

	bool needCircule = true;
	do
	{
		needCircule = smoothing(m_coordinates, tolerance);

	} while (needCircule);
	return true;
}

bool LineString::Assing(const char *_sGeo)
{
	const char *pstr = _sGeo;
	int nNum = 0;
	string sCoordInfo;

	while (pstr != NULL && *pstr)
	{
		if (*pstr == '(')
			;
		else if (*pstr == ')')
			break;
		else if (*pstr == ',')
		{
			Coordinate *pCoord = new Coordinate;
			if (pCoord->Assing(sCoordInfo.c_str()))
				m_coordinates->Add(pCoord);
			sCoordInfo.clear();
		}
		else
			sCoordInfo += *pstr;

		pstr++;
	}
	if (!sCoordInfo.empty())
	{
		Coordinate *pCoord = new Coordinate;
		if (pCoord->Assing(sCoordInfo.c_str()))
			m_coordinates->Add(pCoord);
	}

	return m_coordinates->GetCount() > 0;
}
