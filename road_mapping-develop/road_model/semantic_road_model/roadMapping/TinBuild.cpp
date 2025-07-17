#include "TinBuild.h"
#include "TriangleLib.h"

#include <set>
#include "Geometries/Coordinate.h"
//#include "PCProcess/LinkUtility.h"
#include "Base/Macros.h"
#include "geos/index/strtree/SIRtree.h"
#include "Geometries/BaseAlgorithm3D.h"

#include "./include/CommonUtil.h"
#include "./include/CloudAlgorithm.h"

using namespace std;

Bool TinBuild::PointToTin(const Array<Coordinate>& vecPoints, Array<Triangle>& vecTriangles, Array<Coordinate>& edgepoints)
{
	if (vecPoints.GetCount() < 3)
	{
		return false;
	}
	
	Coordinate pntOrg, pntTemp, normalVector;
	
	Array<Coordinate> Points;
	Points.Add(vecPoints);

    hdmap_build::CommonUtil::TransGet(Points, pntOrg);

    hdmap_build::CSpatialProjection::ComputePlaneFromPoints(Points, pntOrg, normalVector);

	Int32 nTotalPoint = Points.GetCount();

	Coordinate e1 = Coordinate(0,0,1.0);	//z��
	Coordinate e2 = Coordinate(0, 1.0, 0);	//y��
	Coordinate e3 = Coordinate(1.0, 0, 0);	//x��

	double dot1 = fabs(e1.DotProduct(normalVector));
	double dot2 = fabs(e2.DotProduct(normalVector));
	double dot3 = fabs(e3.DotProduct(normalVector));

	if ((dot2 >= dot1) && (dot2 >= dot3))
	{
		for (Int32 i = 0; i < nTotalPoint; i++)
		{
			pntTemp = Points[i];

			Points[i].x = pntTemp.z;
			Points[i].y = pntTemp.x;
			Points[i].z = pntTemp.y;
		}
	}
	else if ((dot3 >= dot1) && (dot3 >= dot2))
	{
		for (Int32 i = 0; i < nTotalPoint; i++)
		{
			pntTemp = Points[i];

			Points[i].x = pntTemp.y;
			Points[i].y = pntTemp.z;
			Points[i].z = pntTemp.x;
		}
	}

	Double * pdPointList = new Double[2*nTotalPoint];
	
	if (pdPointList==NULL)
	{	
		if (pdPointList != NULL)
		{
			delete[]pdPointList;
		}
		
		return false;
	}
	
	Int32 j = 0;
	for(Int32 i=0; i < nTotalPoint; i++)
	{
		pdPointList[j++] = Points[i].x;
		pdPointList[j++] = Points[i].y;
	}
	
	triangulateio *in = new triangulateio;
	triangulateio *out  = new triangulateio;

    TriangleLib mTriangle;
    if(in != NULL)
    {
        in->pointattributelist = (Double*)NULL;
        in->numberofpointattributes = 0;
        in->pointlist = pdPointList;
        in->numberofpoints = nTotalPoint;
        in->pointmarkerlist = NULL;
    }
    if(out != NULL)
    {
        out->trianglelist = NULL;
    }

    mTriangle.triangulate(in,out,"-NQ");

    delete []pdPointList;
    pdPointList = NULL;
    delete in;
    in = NULL;

    Int32 ja, jb, jc;
    Triangle triangle;

    for(Int32 i = 0; i < out->numberoftriangles; i++)
    {
        ja = out->trianglelist[i * 3] - 1;
        jb = out->trianglelist[i * 3 + 1] - 1;
        jc = out->trianglelist[i * 3 + 2] - 1;

        triangle.nIndex0 = ja;
        triangle.nIndex1 = jb;
        triangle.nIndex2 = jc;

        triangle.Point0 = vecPoints[ja];
        triangle.Point1 = vecPoints[jb];
        triangle.Point2 = vecPoints[jc];

        vecTriangles.Add(triangle);
        //if (ValidTriangle(triangle))
        //{
        //	vecTriangles.Add(triangle);
        //}
    }

    delete [](out->trianglelist);
    delete out;

    return true;
}

Bool TinBuild::PointToTin(const Array<Coordinate>& topLine, const Array<Coordinate>& lowerLine, const Array<Coordinate>& vecPoints, Array<Triangle>& vecTriangles)
{
	Array<Coordinate> vecAllPoints;
	
//	vecAllPoints.Add(topLine);
//	vecAllPoints.Add(vecPoints);
//	vecAllPoints.Add(lowerLine);
//
//	if (PointToTin(vecAllPoints, vecTriangles))
//	{
//		Coordinate pntCenter, pntProject;
//		Int32 nSegIndex;
//
//		for (Int32 i = vecTriangles.GetCount() - 1; i >= 0; i--)
//		{
//			pntCenter = (vecTriangles[i].Point0 + vecTriangles[i].Point1)/2;
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, topLine, pntProject, nSegIndex);
//			if ((pntCenter.z - pntProject.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, lowerLine, pntProject, nSegIndex);
//			if ((pntProject.z - pntCenter.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			pntCenter = (vecTriangles[i].Point0 + vecTriangles[i].Point2) / 2;
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, topLine, pntProject, nSegIndex);
//			if ((pntCenter.z - pntProject.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, lowerLine, pntProject, nSegIndex);
//			if ((pntProject.z - pntCenter.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			pntCenter = (vecTriangles[i].Point2 + vecTriangles[i].Point1) / 2;
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, topLine, pntProject, nSegIndex);
//			if ((pntCenter.z - pntProject.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			BaseAlgorithm3D::GetDistancePointToLinesegments(pntCenter, lowerLine, pntProject, nSegIndex);
//			if ((pntProject.z - pntCenter.z) > Geometries_EP)
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//		}
//
//		for (Int32 i = vecTriangles.GetCount() - 1; i >= 0; i--)
//		{
//			Int32 nTopCount = topLine.GetCount();
//			Int32 nIndex0 = vecTriangles[i].nIndex0;
//			Int32 nIndex1 = vecTriangles[i].nIndex1;
//			Int32 nIndex2 = vecTriangles[i].nIndex2;
//
//			//��ȥȫ�����ϱ��߹��ɵ�������
//			if ((nIndex0 < nTopCount) && (nIndex1 < nTopCount) && (nIndex2 < nTopCount))
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//			Int32 nOtherM1 = nTopCount + vecPoints.GetCount() - 1;
//			//��ȥȫ�����±��߹��ɵ�������
//			if ((nIndex0 > nOtherM1) && (nIndex1 > nOtherM1) && (nIndex2 > nOtherM1))
//			{
//				vecTriangles.Delete(i);
//				continue;
//			}
//
//		}
//
//		return true;
//	}
	
	return false;
}

Bool TinBuild::ExtractTINBondaries(const Array<Coordinate>& vecPoints, const Array<Triangle>& vecTriangles, Array<Array<Coordinate>>& edgepoints)
{
    edgepoints.Clear();
    if (vecTriangles.IsEmpty())
        return false;

    Array<Edge> arrEdge;
    for (int i = 0; i < vecTriangles.GetCount(); ++i)
    {
        Edge newEdge1;
        newEdge1.nP1 = vecTriangles[i].nIndex0;
        newEdge1.nP2 = vecTriangles[i].nIndex1;
        newEdge1.nUsedTimes = 0;

        SameEdge(newEdge1, arrEdge);

        Edge newEdge2;
        newEdge2.nP1 = vecTriangles[i].nIndex1;
        newEdge2.nP2 = vecTriangles[i].nIndex2;
        newEdge2.nUsedTimes = 0;

        SameEdge(newEdge2, arrEdge);

        Edge newEdge3;
        newEdge3.nP1 = vecTriangles[i].nIndex0;
        newEdge3.nP2 = vecTriangles[i].nIndex2;
        newEdge3.nUsedTimes = 0;

        SameEdge(newEdge3, arrEdge);
    }

    Array<Edge> boundaryEdges;
    Array<Coordinate> boundaryPts;
    for (int i = 0; i < arrEdge.GetCount(); ++i)
    {
        if (arrEdge[i].nUsedTimes < 1)
        {
            boundaryEdges.Add(arrEdge[i]);
            boundaryPts.Add(vecPoints[arrEdge[i].nP1]);
            boundaryPts.Add(vecPoints[arrEdge[i].nP2]);
        }

    }
    edgepoints.Add(boundaryPts);
}

Void TinBuild::SameEdge(Edge newEdge, Array<Edge>& arrEdge)
{
    int sameIndex = -1;
    for (int j = 0; j < arrEdge.GetCount(); ++j) {
        if (newEdge.nP1 == arrEdge[j].nP1 &&  newEdge.nP2 == arrEdge[j].nP2)
        {
            sameIndex = j;
            break;
        }
        if (newEdge.nP1 == arrEdge[j].nP2 &&  newEdge.nP2 == arrEdge[j].nP1)
        {
            sameIndex = j;
            break;
        }
    }

    //找到相同边的处理
    if (sameIndex > -1)
        arrEdge[sameIndex].nUsedTimes++;
    else
        arrEdge.Add(newEdge);
}
//���������������ù켣�߼���̲߳�����
Bool TinBuild::GenerateContourLine(Double contourValue, const Array<Triangle>& vecTriangles, Array<Array<Coordinate>>& ContourLinePoints)
{
	if (vecTriangles.IsEmpty())
	{
		return false;
	}

	//����ÿ���������еĵȸ���
	Array<LineTwoPts> arrContourLines;
	for (Int32 i = 0; i < vecTriangles.GetCount(); i++)
	{
		if (AdjustZ(vecTriangles[i], contourValue) == false)
		{
			continue; //�����жϲ��ڸ̷߳�Χ��
		}
		
		LineTwoPts resLine;
		if (ContourLineInTriangle(vecTriangles[i], contourValue, resLine))
		{
			arrContourLines.Add(resLine);
		}
	}

	if (arrContourLines.IsEmpty())
	{
		return false;
	}

	//�������ߴ�����һ��
	AppendContourLines(arrContourLines, contourValue, ContourLinePoints);

	return true;
}

Bool TinBuild::ValidTriangle(const Triangle& triangle)
{
	Coordinate e1 = triangle.Point0 - triangle.Point1;
	Coordinate e2 = triangle.Point1 - triangle.Point2;
	Coordinate e3 = triangle.Point2 - triangle.Point0;

	e1.Normalize();
	e2.Normalize();
	e3.Normalize();

	double dot1 = fabs(e1.DotProduct(e2));
	double dot2 = fabs(e2.DotProduct(e3));
	double dot3 = fabs(e3.DotProduct(e1));

	if ((dot1 > 0.99862) || (dot2 > 0.99862) || (dot3 > 0.99862))
	{
		return false;
	}
	
	/*
	Coordinate e0 = Coordinate(0.0, 0.0, 1.0);
	Coordinate e  = e1.CrossProduct(e2);
	e.Normalize();
	
	if (fabs(e.DotProduct(e0)) > 0.866)
	{
		return false;
	}*/

	return true;
}

Double TinBuild::GetAspect(const Coordinate &pnt0, const Coordinate &pnt1, const Coordinate &pnt2)
{
	if ((fabs(pnt0.z - pnt1.z) < Geometries_EP) && (fabs(pnt0.z - pnt2.z) < Geometries_EP))
	{
		return -1.0;//ˮƽ,û�з�λ��,����-1
	}
	
	Double dAspect=0.0;
	Double dx1,dy1;
	Double dx2,dy2;   
	Double dz1,dz2;
	Double dA,dB,dC;
	Double dtancle;
	
	Coordinate pPoint;
	
	dx1=pnt1.x-pnt0.x;
	dy1= pnt1.y-pnt0.y;
	dz1=pnt1.z-pnt0.z;

	dx2=pnt2.x-pnt0.x;
	dy2=pnt2.y-pnt0.y;
	dz2=pnt2.z-pnt0.z;
	
	dA=dy1*dz2-dy2*dz1;
	dB=dz1*dx2-dz2*dx1;
	dC=dx1*dy2-dx2*dy1;

	if (dC<Geometries_EP)
	{
		dA=-dA;
		dB=-dB;
		dC=-dC;
	}
                   
	if ((fabs(dA) < Geometries_EP) && (fabs(dB) < Geometries_EP))
	{
		dAspect = -1.0;	//ˮƽ
	}
	else
	{
		dtancle=atan2(dB,dA);
		dAspect = PI/2.0 - dtancle;
		if (dAspect < Geometries_NEP)
		{
			dAspect += 2* PI;
		}
	}
	
	return dAspect;
}

Double TinBuild::GetSlope(const Coordinate &pnt0, const Coordinate &pnt1, const Coordinate &pnt2)
{
	if ((pnt2.Distance(pnt0) < Geometries_EP) || (pnt2.Distance(pnt1) < Geometries_EP) || (pnt1.Distance(pnt0) < Geometries_EP))
	{
		return -1.0;
	}
		
	
	Double ddx2  = pnt2.x - pnt0.x;
	Double ddy2= pnt2.y - pnt0.y;
	Double ddz2 = pnt2.z - pnt0.z;

	Double ddx1 = pnt1.x - pnt0.x;
	Double ddy1 = pnt1.y - pnt0.y;
	Double ddz1 = pnt1.z - pnt0.z;

	Double ddx3 = ddy1*ddz2 - ddy2*ddz1;
	Double ddy3 = ddz1*ddx2 - ddx1*ddz2;
	Double ddz3 = ddx1*ddy2 - ddy1*ddx2;

	if(ddz3 < 0)
	{
		ddx3 = -ddx3;
		ddy3 = -ddy3;
		ddz3 = -ddz3;
	}
	
	Double ddxy = sqrt(ddx3*ddx3 + ddy3*ddy3);
	
	if (fabs(ddxy) < Geometries_EP)
	{
		return 0.0;
	}
	
	Double dSlope = atan(ddz3/ddxy);
	return PI/2 - dSlope;
}

Bool TinBuild::AdjustZ(Triangle curTriangle, Double contourValue)
{
	Array<double> arrZ;
	arrZ.Add(curTriangle.Point0.z);
	arrZ.Add(curTriangle.Point1.z);
	arrZ.Add(curTriangle.Point2.z);
	double minZ = *min_element(arrZ.Begin(), arrZ.End());
	double maxZ = *max_element(arrZ.Begin(), arrZ.End());

	if ((contourValue < minZ) || (contourValue >= maxZ))
	{
		return false;
	}
	
	return true;
}

Bool TinBuild::GetInsertPoint(const Geometries::Coordinate& pnt0, const Geometries::Coordinate& pnt1, Double contourValue, Geometries::Coordinate& resPnt)
{
	Double p0Z = pnt0.z;
	Double p1Z = pnt1.z;
	p0Z = (p0Z == contourValue) ? (p0Z + Geometries_EP) : p0Z;
	p1Z = (p1Z == contourValue) ? (p1Z + Geometries_EP) : p1Z;
	
	Double differ0 = (p0Z - contourValue) * (p1Z - contourValue);
	if (differ0 > 0)
	{
		return false;
	}
	else
	{
		resPnt.x = pnt0.x + (pnt1.x - pnt0.x) * (contourValue - p0Z) / (p1Z - p0Z);
		resPnt.y = pnt0.y + (pnt1.y - pnt0.y) * (contourValue - p0Z) / (p1Z - p0Z);
		resPnt.z = contourValue;
	}
	return true;
}

Bool TinBuild::ContourLineInTriangle(Triangle curTriangle, Double contourValue, LineTwoPts& resLine)
{
	Array<Coordinate> arrIntersectPts;
	Coordinate resIntersectPoint;
	if (GetInsertPoint(curTriangle.Point0, curTriangle.Point1, contourValue, resIntersectPoint))
	{
		arrIntersectPts.Add(resIntersectPoint);
	}
	if (GetInsertPoint(curTriangle.Point1, curTriangle.Point2, contourValue, resIntersectPoint))
	{
		arrIntersectPts.Add(resIntersectPoint);
	}
	if (GetInsertPoint(curTriangle.Point2, curTriangle.Point0, contourValue, resIntersectPoint))
	{
		arrIntersectPts.Add(resIntersectPoint);
	}

	if (arrIntersectPts.GetCount() != 2)
	{
		return false;
	}
	
	resLine.ptStart = arrIntersectPts[0];
	resLine.ptEnd = arrIntersectPts[1];

	return true;
}

Bool TinBuild::JointClosedLine(const Array<Coordinate>& contourPoints, const Array<Int32>& arrMatchIndexs, MapIndex& arrNotDealIndexs, Array<Array<Coordinate>>& ContourLinesPoints)
{
	if (contourPoints.IsEmpty())
	{
		return false;
	}

	if (arrMatchIndexs.IsEmpty())
	{
		return false;
	}

	if (arrNotDealIndexs.empty())
	{
		return false;
	}

	while (!(arrNotDealIndexs.empty()))
	{
		Array<Int32> arrDealIndexs; //��¼���д�����ĵ�
		Array<Coordinate> contourLine;
		
		Int32 originalIndex = arrNotDealIndexs.begin()->first;//ȡԭʼ�жϵ�
		Int32 startIndex = (originalIndex % 2 == 0) ? (originalIndex + 1) : (originalIndex - 1);
		Int32 nextIndex = arrMatchIndexs[startIndex];
		
		arrDealIndexs.Add(originalIndex);
		contourLine.Add(contourPoints[originalIndex]);
		
		while (nextIndex != originalIndex)
		{
			arrDealIndexs.Add(startIndex);
			arrDealIndexs.Add(nextIndex);
			contourLine.Add(contourPoints[startIndex]);
			startIndex = (nextIndex % 2 == 0) ? (nextIndex + 1) : (nextIndex - 1);
			nextIndex = arrMatchIndexs[startIndex];
		}
		contourLine.Add(contourPoints[startIndex]);
		contourLine.Add(contourPoints[originalIndex]);
		arrDealIndexs.Add(startIndex);

		//�����պ�����
		if (contourLine.GetCount() > 2)
		{
			ContourLinesPoints.Add(contourLine);
		}

		//ɾ������������е�
		for (Int32 i = 0; i < arrDealIndexs.GetCount(); i++)
		{
			auto index_map = arrNotDealIndexs.find(arrDealIndexs[i]);
			arrNotDealIndexs.erase(index_map);
		}
		arrDealIndexs.Clear();
	}
	
	return true;
}

Bool TinBuild::JointOpenLine(const Array<Coordinate>& contourPoints, const Array<Int32>& arrMatchIndexs, MapIndex& arrNotDealIndexs, MapIndex& mapNotMatchIndexs, Array<Array<Coordinate>>& ContourLinesPoints)
{
	if (arrMatchIndexs.IsEmpty())
	{
		return false;
	}

	if (arrNotDealIndexs.empty())
	{
		return false;
	}

	Array<Int32> arrDealIndexs;
	while ((!(mapNotMatchIndexs.empty())) && (!(arrNotDealIndexs.empty())))
	{
		Array<Coordinate> contourLine;
		Int32 startIndex = mapNotMatchIndexs.begin()->first;
		contourLine.Add(contourPoints[startIndex]);
		arrDealIndexs.Add(startIndex);
		
		//ѭ�����ҽ����㣬����¼���д�����ĵ�
		Int32 curIndex = (startIndex % 2 == 0) ? (startIndex + 1) : (startIndex - 1);
		Int32 nextIndex = arrMatchIndexs[curIndex];
		while (nextIndex != -1)
		{
			contourLine.Add(contourPoints[curIndex]);
			arrDealIndexs.Add(curIndex);
			arrDealIndexs.Add(nextIndex);
			curIndex = (nextIndex % 2 == 0) ? (nextIndex + 1) : (nextIndex - 1);
			nextIndex = arrMatchIndexs[curIndex];	
		}
		contourLine.Add(contourPoints[curIndex]);
		arrDealIndexs.Add(curIndex);
		
		//���һ�������ȸ�����Ϣ
		if (contourLine.GetCount() > 1)
		{
			ContourLinesPoints.Add(contourLine);
		}

		//ɾ������������е�
		for (Int32 i = 0; i < arrDealIndexs.GetCount(); i++)
		{
			auto index_map = arrNotDealIndexs.find(arrDealIndexs[i]);
			arrNotDealIndexs.erase(index_map);
		}
		arrDealIndexs.Clear();

		//ɾ������������or�յ�
		mapNotMatchIndexs.erase(mapNotMatchIndexs.begin());
		auto find_map = mapNotMatchIndexs.find(curIndex);
		mapNotMatchIndexs.erase(find_map);
	}
	 
	return true;
}

Bool TinBuild::AppendContourLines(Array<LineTwoPts>& arrContourLines, Double contourValue, Array<Array<Coordinate>>& ContourLinesPoints)
{
	if (arrContourLines.IsEmpty())
	{
		return false;
	}
//	Int32 i, j;
//	Array<Coordinate> arrLinePnts;
//	for (i = 0; i < arrContourLines.GetCount(); i++)
//	{
//		arrLinePnts.Add(arrContourLines[i].ptStart);
//		arrLinePnts.Add(arrContourLines[i].ptEnd);
//	}
//
//	////test data/////////////////////
//	//Int32 z = contourValue * 100;
//	//static Char szMsg[1024];
//	//Array<Array<Coordinate>> arrArrPnts;
//	//arrArrPnts.SetSize(arrContourLines.GetCount());
//	//for (i = 0; i < arrContourLines.GetCount(); i++)
//	//{
//	//	arrArrPnts[i].Add(arrContourLines[i].ptStart);
//	//	arrArrPnts[i].Add(arrContourLines[i].ptEnd);
//	//}
//	//sprintf_s(szMsg, 256, "E:\\TestOutPut\\beforeAppendContourLines_%d.obj", z);
//	//LinkUtility::DumpToObjFile(szMsg, arrArrPnts);
//	///////////////////////
//
//
//	hdmap_build::RoadTopoGrid topoGrid;
//	topoGrid.BuildTopoGrid(arrLinePnts);
//
//	//Ѱ����ͬ��
//	Array<Int32> arrMatchIndexs;                                     //��¼ƥ����Ӧ������
//	std::unordered_map<Base::Int32, Base::Int32> mapNotMatchIndexs; //��¼û��ƥ��ĵ�����
//	MapIndex arrNotDealIndexs;                                  //��¼��δ�����ߵ�����
//	Int32 resIndex = -1;
//	Int32 notMatchCount = 0;
//	for (i = 0; i < arrLinePnts.GetCount(); i++)
//	{
//		topoGrid.GetMatchInfos(arrLinePnts[i], arrLinePnts, i, resIndex);
//		if (resIndex == -1)
//		{
//			mapNotMatchIndexs.insert(make_pair(i, notMatchCount));
//			notMatchCount++;
//		}
//		arrMatchIndexs.Add(resIndex);
//		arrNotDealIndexs.insert(make_pair(i, i));
//	}
//
//	//���ȴ������еķǱպϵȸ���
//	JointOpenLine(arrLinePnts, arrMatchIndexs, arrNotDealIndexs, mapNotMatchIndexs, ContourLinesPoints);
//
//	////testdata
//	//static Char szOpen[1024];
//	//sprintf_s(szOpen, 256, "E:\\TestOutPut\\OpenContourLines_%d.obj", z);
//	//LinkUtility::DumpToObjFile(szOpen, ContourLinesPoints);
//
//	//�ٴ���պϵȸ���
//	JointClosedLine(arrLinePnts, arrMatchIndexs, arrNotDealIndexs, ContourLinesPoints);
//
//	////testdata
//	//static Char sz[1024];
//	//sprintf_s(sz, 256, "E:\\TestOutPut\\ContourLines_%d.obj", z);
//	//LinkUtility::DumpToObjFile(sz, ContourLinesPoints);
//
	return true;
}
