#include "LinearObjDuplicateRemoval.h"
#include "Geometries/LineString.h"

#include <string.h>

using namespace hdmap_build;
using namespace std;
using namespace Engine::Geometries;
using namespace Engine::Base;
#define  MEMO_CLIP "RD"
//////////////////////////////////////////////////////////////////////////
// LinearObject
Bool LinearObject::CompareLength(const LinearObject* p0, const LinearObject* p1)
{
    return p0->Geometry()->GetLength() > p1->Geometry()->GetLength();
}

LineString* LinearObject::Geometry() const
{
	return m_geometry;
}

LineString* LinearObject::Geometry()
{
	return m_geometry;
}

Void LinearObject::Break(const Array<Int32>& indices_mark, Array<LinearObject*>& results)
{
	int size = m_geometry->GetNumPoints();
	int start = 0;
	int flag = indices_mark[0];

	for (int i = start + 1; i < size;)
	{
		while (i<size && indices_mark[i] == flag)
		{
			++i;
		}

		//[start,i)
		int len = i - start;

		//������
		if (flag == 0 && len >1)
		{
			//�������һ�㱻���ɾ������ôĬ��Ϊ��һ��Ϊ��������β������������µ�
			if (start == 1)
				--start;
			
			if (i == size - 1)
				++i;

			Array<Coordinate*>* pArray = new Array<Coordinate*>();
			for (int j = start; j < i;++j)
			{
				Coordinate* p = new Coordinate(*m_geometry->GetCoordinateN(j));
				pArray->Add(p);
			}

			LineString* pLineString = new LineString(pArray);
			LinearObject* pNewObj = CreateObject(pLineString,"");
			pNewObj->m_memo = m_memo;
			if (len < size)
				pNewObj->m_memo = MEMO_CLIP;

			SubStrAttributeCopy(start, i - 1, pNewObj);

			results.Add(pNewObj);
		}

		//update
		if (i < size)
		{
			start = i;
			flag = indices_mark[i];
		}
	}
}

Void LinearObject::SubStrAttributeCopy(Int32 begin, Int32 end, LinearObject* pSubStr)
{
	
}

LinearObject* LinearObject::CreateObject(LineString* pGeometry, String strMemo)
{
	return new LinearObject(pGeometry, strMemo);
}


LinearObject::LinearObject(LineString* pGeometry, String strMemo):
m_geometry(pGeometry),
m_memo(strMemo)
{}

LinearObject::LinearObject():
m_geometry(nullptr)
{}
LinearObject::~LinearObject()
{
	//DELETE_PTR(m_geometry);//add llj 2018-12-13 ����ֱ��ɾ ������쳣
}


//////////////////////////////////////////////////////////////////////////
//LinearObjDuplicateRemoval

Void  LinearObjDuplicateRemoval::RemoveDuplicate(Array<LinearObject*>& input, Double tolerance, Bool compare_opposite, Double mininum_len)
{
	//���Ȱ��Ӵ�С����
	std::sort(input.Begin(), input.End(), LinearObject::CompareLength);

	Array<LineString*> arrTargetLines;
	for (int i = 0, n = input.GetCount(); i < n;++i)
	{
		arrTargetLines.Add(input[i]->Geometry());
	}

	//��������
	RoadTopoGrid RoadCondition;
	RoadCondition.BuildTopoGrid(arrTargetLines);

	//��С����
	for (int i = input.GetCount() - 1; i >= 0;--i)
	{
		LineString* pGeoLine = arrTargetLines[i];
		Array<Coordinate *>* arrpntTrack = pGeoLine->GetCoordinates();

		Int32 nPointCount = arrpntTrack->GetCount();

		Array<Int32> arrPointMatch;
		arrPointMatch.SetSize(nPointCount);
		memset(arrPointMatch.Data(), 0, sizeof(Int32)*nPointCount);

		//arrPointMatch ��1 Ϊɾ�����
		RoadCondition.GetMatchInfos(*arrpntTrack, i, arrPointMatch, tolerance,!compare_opposite);

		Array<LinearObject*> breakLines;

		input[i]->Break(arrPointMatch, breakLines);

		//���ȹ���
		for (auto j = 0; j < breakLines.GetCount();)
		{
			LinearObject* pObj = breakLines[j];
			if (pObj->Geometry()->GetLength() < mininum_len)
			{
				DELETE_PTR(pObj->m_geometry);//add llj 2018-12-13
				delete pObj;
				breakLines.Delete(j);
			}
			else
				++j;
		}

		DELETE_PTR(input[i]->m_geometry);//add llj 2018-12-13
		delete input[i];	
		input.Delete(i);
		input.Add(breakLines);
	}
}

LinearObjDuplicateRemoval::LinearObjDuplicateRemoval()
{}