//
//
//

#include "RoadTopoBuild.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/LineString.h"
#include "CommonUtil.h"
#include <set>
#include <cstring>
#include "proj_api.h"

using namespace Engine::Geometries;
using namespace Engine::Base;
using namespace hdmap_build;
using namespace std;

#define MAXGRIDCOUNT 10000000 // ���������
#define NIMIN(a, b) ((a) < (b) ? (a) : (b))
#define NIMAX(a, b) ((a) > (b) ? (a) : (b))

Bool RoadTopoBuild::MatchPoints(Array<Coordinate> &PrecisePoints, Array<LineString *> &MatchLines, Array<Array<Int32>> &arrMatchInfos)
{
    arrMatchInfos.Clear();

    RoadTopoGrid RoadCondition;
    RoadCondition.BuildTopoGrid(PrecisePoints);

    for (Int32 i = 0; i < MatchLines.GetCount(); i++)
    {
        LineString *pGeoLine = MatchLines[i];
        Array<Coordinate *> *arrpntTrack = pGeoLine->GetCoordinates();

        Int32 nPointCount = arrpntTrack->GetCount();

        Array<Int32> arrPointMatch;
        arrPointMatch.SetSize(nPointCount);
        memset(arrPointMatch.Data(), 0, sizeof(Int32) * nPointCount);

        RoadCondition.GetMatchInfos(PrecisePoints, *arrpntTrack, arrPointMatch);

        arrMatchInfos.Add(arrPointMatch);
    }

    return true;
}

Bool RoadTopoBuild::MatchPoints(RoadTopoGrid &topoGridPoints, Array<Coordinate> &PrecisePoints, LineString *MatchLines, Array<Int32> &arrMatchInfos, Double dTolerance)
{
    arrMatchInfos.Clear();

    Array<Coordinate *> *arrpntTrack = MatchLines->GetCoordinates();
    Int32 nPointCount = arrpntTrack->GetCount();

    if (nPointCount < 1)
    {
        return false;
    }

    arrMatchInfos.Clear();
    arrMatchInfos.SetSize(nPointCount);
    memset(arrMatchInfos.Data(), -1, sizeof(Int32) * nPointCount);

    topoGridPoints.GetMatchInfos(PrecisePoints, *arrpntTrack, arrMatchInfos, dTolerance);
    return true;
}

Bool dCmp_dLength(const LineObject &p0, const LineObject &p1)
{
    return p0.pLine->GetLength() > p1.pLine->GetLength();
}

Bool dCmp_dAveLength(const GroupLineObject &p0, const GroupLineObject &p1)
{
    return p0.avgDis > p1.avgDis;
}

Bool dCmp_GroupID(const LineObject &p0, const LineObject &p1)
{
    return p0.groupID < p1.groupID;
}

Bool RoadTopoBuild::CleanDuplicateLines(Array<Array<LineObject>> &lineObjectArrArr, Array<LineObject> &arrLineObjects, Double dTolerance, Bool joint, Double dLength)
{
    if (lineObjectArrArr.IsEmpty())
    {
        return false;
    }

    // 本方法中按照groupId排序,保证一个组内数据平齐(先计算每组数据的长度平均值，按平均长度排序)
    Array<GroupLineObject> arrGroupLineObjects;
    for (int i = 0; i < lineObjectArrArr.GetCount(); i++)
    {
        GroupLineObject newGroupLineObject;
        double sumLen = 0.0;
        for (int j = 0; j < lineObjectArrArr[i].GetCount(); ++j)
        {
            newGroupLineObject.arrLineObject.Add(lineObjectArrArr[i][j]);
            sumLen += lineObjectArrArr[i][j].pLine->GetLength();
        }
        double aveLen = sumLen / lineObjectArrArr[i].GetCount();
        newGroupLineObject.avgDis = aveLen;
        arrGroupLineObjects.Add(newGroupLineObject);
    }

    // sort(arrLineObjects.Begin(), arrLineObjects.End(), dCmp_dLength);
    // sort(arrLineObjects.Begin(), arrLineObjects.End(), dCmp_GroupID);
    sort(arrGroupLineObjects.Begin(), arrGroupLineObjects.End(), dCmp_dAveLength);
    arrLineObjects.Clear();
    for (int i = 0; i < arrGroupLineObjects.GetCount(); i++)
    {
        // 每个组内按照长度进行排序,长度短的位于尾部，处理的时候会优先删除长度较短的数据
        sort(arrGroupLineObjects[i].arrLineObject.Begin(), arrGroupLineObjects[i].arrLineObject.End(), dCmp_dLength);

        for (int j = 0; j < arrGroupLineObjects[i].arrLineObject.GetCount(); ++j)
        {
            arrLineObjects.Add(arrGroupLineObjects[i].arrLineObject[j]);
        }
    }
    if (arrLineObjects.IsEmpty())
    {
        return false;
    }

    Array<LineString *> arrTargetLines;
    Int32 i = 0;
    Int32 nLineCount = arrLineObjects.GetCount();

    for (i = 0; i < nLineCount; i++)
    {
        arrTargetLines.Add(arrLineObjects[i].pLine);
    }

    // 输出所有去重前的线[yxx 2021-9-9]
    //    string name = "quchongLines";
    //    string m_BasePath = "/home/test/data/travel_weiduiqizhuanqueshiwentigongyiyanzheng_1638437784/318790751/GPS/";
    //    CommonUtil::WriteToOBJ(arrTargetLines, m_BasePath, name, true);

    RoadTopoGrid RoadCondition;
    RoadCondition.BuildTopoGrid(arrTargetLines);

    for (i = nLineCount - 1; i >= 0; i--)
    {
        LineString *pGeoLine = arrTargetLines[i];
        Array<Coordinate *> *arrpntTrack = pGeoLine->GetCoordinates();

        Int32 nPointCount = arrpntTrack->GetCount();

        Array<Int32> arrPointMatch;
        arrPointMatch.SetSize(nPointCount);
        memset(arrPointMatch.Data(), 0, sizeof(Int32) * nPointCount);
        RoadCondition.GetMatchInfos(*arrpntTrack, i, arrPointMatch, dTolerance);

        DelDuplicateSegs(arrLineObjects, i, arrPointMatch, dLength);
    }

    return joint ? Joints(arrLineObjects) : true;
}

Bool RoadTopoBuild::Joints(Array<LineObject> &arrLineObjects)
{
    Int32 nLineCount = arrLineObjects.GetCount();

    if (nLineCount < 2)
    {
        return false;
    }

    for (Int32 i = nLineCount - 1; i > 0; i--)
    {
        LineString *pGeoLine = arrLineObjects[i].pLine;
        Array<Coordinate *> *arrpntTrack = pGeoLine->GetCoordinates();

        Int32 nPointCount = arrpntTrack->GetCount();
        if (nPointCount < 2)
        {
            continue;
        }

        Coordinate pntStart = *(*arrpntTrack)[0];
        Coordinate pntEnd = *(*arrpntTrack)[nPointCount - 1];

        for (Int32 j = i - 1; j >= 0; j--)
        {
            LineString *pGeoLine1 = arrLineObjects[j].pLine;
            Array<Coordinate *> *arrpntTrack1 = pGeoLine1->GetCoordinates();

            Int32 nPointCount1 = arrpntTrack1->GetCount();
            if (nPointCount1 < 2)
            {
                continue;
            }

            Coordinate pntStart1 = *(*arrpntTrack1)[0];
            Coordinate pntEnd1 = *(*arrpntTrack1)[nPointCount1 - 1];

            if (pntStart.Distance(pntEnd1) < 5.0)
            {
                Coordinate e1 = pntEnd1 - *(*arrpntTrack1)[nPointCount1 - 2];
                Coordinate e2 = *(*arrpntTrack)[1] - pntStart;

                e1.Normalize();
                e2.Normalize();

                if (e1.DotProduct(e2) < 0.0)
                {
                    continue;
                }

                Array<Coordinate> arrCoordinates;
                arrCoordinates.SetSize(nPointCount);

                for (Int32 k = 0; k < nPointCount; k++)
                {
                    arrCoordinates.SetAt(k, *(*arrpntTrack)[k]);
                }

                for (Int32 k = 0; k < arrCoordinates.GetCount(); k++)
                {
                    Coordinate *pCoordinate = new Coordinate(arrCoordinates[k]);
                    arrpntTrack1->Add(pCoordinate);
                }

                delete pGeoLine;
                pGeoLine = NULL;

                arrLineObjects.Delete(i);
                break;
            }
            else if (pntEnd.Distance(pntStart1) < 5.0)
            {
                Coordinate e1 = pntEnd - *(*arrpntTrack)[nPointCount - 2];
                Coordinate e2 = *(*arrpntTrack1)[1] - pntStart1;

                e1.Normalize();
                e2.Normalize();

                if (e1.DotProduct(e2) < 0.0)
                {
                    continue;
                }

                Array<Coordinate> arrCoordinates;
                arrCoordinates.SetSize(nPointCount);

                for (Int32 k = 0; k < nPointCount; k++)
                {
                    arrCoordinates.SetAt(k, *(*arrpntTrack)[k]);
                }

                for (Int32 k = nPointCount - 1; k >= 0; k--)
                {
                    Coordinate *pCoordinate = new Coordinate(arrCoordinates[k]);
                    arrpntTrack1->InsertAt(0, pCoordinate);
                }

                delete pGeoLine;
                pGeoLine = NULL;

                arrLineObjects.Delete(i);
                break;
            }
        }
    }

    return true;
}

Bool RoadTopoBuild::DelDuplicateSegs(Array<LineObject> &arrLineObjects, Int32 nIndex, Array<Int32> &arrPointMatch, Double dLength)
{
    Int32 nLineCount = arrLineObjects.GetCount();

    if ((nIndex < 0) || (nIndex >= nLineCount))
    {
        return false;
    }

    LineString *pGeoLine = arrLineObjects[nIndex].pLine;
    Array<Coordinate *> *arrpntTrack = pGeoLine->GetCoordinates();
    Int32 nPointCount = arrPointMatch.GetCount();

    Int32 i = 0, j = 0;
    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] == 0)
        {
            break;
        }
    }

    if (i == nPointCount)
    {
        delete pGeoLine;
        pGeoLine = NULL;

        arrLineObjects.Delete(nIndex);

        return true;
    }

    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] == 1)
        {
            break;
        }
    }

    if (i == nPointCount)
    {
        return true;
    }

    Array<LineString *> arrLines;

    Int32 nValue = arrPointMatch[0];
    Int32 nSIndex = 0, nEIndex = -1;

    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] != nValue)
        {
            if (nValue == 0)
            {
                nEIndex = i - 1;
                // ����һ����
                Array<Coordinate *> *arrCoordinates = new Array<Coordinate *>();
                for (j = nSIndex; j <= nEIndex; j++)
                {
                    Coordinate *pCoordinate = new Coordinate(*(*arrpntTrack)[j]);
                    arrCoordinates->Add(pCoordinate);
                }

                LineString *pLine = new LineString(arrCoordinates);

                if (pLine->GetLength() >= dLength)
                {
                    arrLines.Add(pLine);
                }
                else
                {
                    delete pLine;
                    pLine = NULL;
                }

                nValue = 1;
                nSIndex = i;
            }
            else
            {
                nSIndex = i;
                nValue = 0;
            }
        }
    }

    if ((nValue == 0) && (nSIndex > 0))
    {
        nEIndex = nPointCount - 1;

        // ����һ����
        Array<Coordinate *> *arrCoordinates = new Array<Coordinate *>();

        for (j = nSIndex; j <= nEIndex; j++)
        {
            Coordinate *pCoordinate = new Coordinate(*(*arrpntTrack)[j]);
            arrCoordinates->Add(pCoordinate);
        }

        LineString *pLine = new LineString(arrCoordinates);

        if (pLine->GetLength() >= dLength)
        {
            arrLines.Add(pLine);
        }
        else
        {
            delete pLine;
            pLine = NULL;
        }
    }

    pGeoLine = arrLineObjects[nIndex].pLine;
    delete pGeoLine;
    pGeoLine = NULL;

    Int32 nSide = arrLineObjects[nIndex].nSide;
    Int32 nObjType = arrLineObjects[nIndex].nObjType;
    Int32 nLineType = arrLineObjects[nIndex].nLineType;
    Int32 groupID = arrLineObjects[nIndex].groupID;
    Int32 indexInGroup = arrLineObjects[nIndex].indexInGroup;
    arrLineObjects.Delete(nIndex);

    Bool bAdd = (arrLineObjects.GetCount() == nIndex);

    if (arrLines.GetCount() > 0)
    {
        LineObject obj;
        for (i = arrLines.GetCount() - 1; i >= 0; i--)
        {
            obj.nSide = nSide;
            obj.groupID = groupID;
            obj.indexInGroup = indexInGroup;
            obj.nObjType = nObjType;
            obj.nLineType = nLineType;
            obj.pLine = arrLines[i];

            if (bAdd)
            {
                arrLineObjects.Add(obj);
                bAdd = false;
            }
            else
            {
                arrLineObjects.InsertAt(nIndex, obj);
            }
        }
    }

    return true;
}

Bool RoadTopoBuild::DelDuplicateSegs(Array<Coordinate> &arrCroods, Array<Int32> &arrPointMatch, Array<Array<Coordinate>> &arrResCroods, Double dLength)
{
    Int32 nCount = arrCroods.GetCount();

    if (nCount == 0)
    {
        return false;
    }

    arrResCroods.Clear();
    Int32 nPointCount = arrPointMatch.GetCount();

    if (nCount != nPointCount)
        return false;

    Int32 i = 0, j = 0;
    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] == 0)
        {
            break;
        }
    }

    if (i == nPointCount)
    {
        arrResCroods.Clear(); // 被全部匹配
        return true;
    }

    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] == 1)
        {
            break;
        }
    }

    if (i == nPointCount)
    {
        arrResCroods.Add(arrCroods);
        return true; // 全部未匹配
    }

    Array<Coordinate> arrPts;

    Int32 nValue = arrPointMatch[0];
    Int32 nSIndex = 0, nEIndex = -1;

    for (i = 0; i < nPointCount; i++)
    {
        if (arrPointMatch[i] != nValue)
        {
            if (nValue == 0)
            {
                nEIndex = i - 1;
                for (j = nSIndex; j <= nEIndex; j++)
                {
                    arrPts.Add(arrCroods[j]);
                }

                if (CommonUtil::GetLength(arrPts) >= dLength)
                {
                    arrResCroods.Add(arrPts);
                }
                else
                {
                    arrPts.Clear();
                }

                nValue = 1;
                nSIndex = i;
            }
            else
            {
                nSIndex = i;
                nValue = 0;
            }
        }
    }

    if ((nValue == 0) && (nSIndex > 0))
    {
        nEIndex = nPointCount - 1;

        Array<Coordinate> arrPts;

        for (j = nSIndex; j <= nEIndex; j++)
        {
            arrPts.Add(arrCroods[j]);
        }

        if (CommonUtil::GetLength(arrPts) >= dLength)
        {
            arrResCroods.Add(arrPts);
        }
        else
        {
            arrPts.Clear();
        }
    }

    return true;
}

Void RoadTopoBuild::SmoothSTurnSegments(Base::Array<Geometries::Coordinate> &vecInput)
{
    Int32 nPoints = vecInput.GetCount();
    if (nPoints < 3)
        return;

    // ���� i-1,i,i+1����
    for (Int32 i = 1; i < nPoints - 1; i++)
    {
        Geometries::Vector3d vecCur = vecInput[i];

        Geometries::Vector3d vPrev = vecInput[i - 1] - vecInput[i];
        vPrev.z = 0.0;
        vPrev.Normalize();
        Geometries::Vector3d vNext = vecInput[i + 1] - vecInput[i];
        vNext.z = 0.0;
        vNext.Normalize();

        Double turn = vPrev.DotProduct(vNext);

        // trun 大于0 正常 小于0有折返
        if (turn < 0)
            continue;

        // 删除点
        vecInput.Delete(i + 1);
        --i;
        --nPoints;
    }
}

Void RoadTopoBuild::LineSmooth3(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints)
{
    if (inputPoints.IsEmpty())
    {
        return;
    }

    Int32 N = inputPoints.GetCount();
    outputPoints.SetSize(N);
    Int32 i = 0;
    if (N < 3)
    {
        for (i = 0; i <= N - 1; i++)
        {
            outputPoints[i] = inputPoints[i];
        }
    }
    else
    {
        outputPoints[0].x = (5.0 * inputPoints[0].x + 2.0 * inputPoints[1].x - inputPoints[2].x) / 6.0;
        outputPoints[0].y = (5.0 * inputPoints[0].y + 2.0 * inputPoints[1].y - inputPoints[2].y) / 6.0;
        outputPoints[0].z = (5.0 * inputPoints[0].z + 2.0 * inputPoints[1].z - inputPoints[2].z) / 6.0;

        for (i = 1; i <= N - 2; i++)
        {
            outputPoints[i].x = (inputPoints[i - 1].x + inputPoints[i].x + inputPoints[i + 1].x) / 3.0;
            outputPoints[i].y = (inputPoints[i - 1].y + inputPoints[i].y + inputPoints[i + 1].y) / 3.0;
            outputPoints[i].z = (inputPoints[i - 1].z + inputPoints[i].z + inputPoints[i + 1].z) / 3.0;
        }

        outputPoints[N - 1].x = (5.0 * inputPoints[N - 1].x + 2.0 * inputPoints[N - 2].x - inputPoints[N - 3].x) / 6.0;
        outputPoints[N - 1].y = (5.0 * inputPoints[N - 1].y + 2.0 * inputPoints[N - 2].y - inputPoints[N - 3].y) / 6.0;
        outputPoints[N - 1].z = (5.0 * inputPoints[N - 1].z + 2.0 * inputPoints[N - 2].z - inputPoints[N - 3].z) / 6.0;
    }
}

Void RoadTopoBuild::LineSmooth5(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints)
{
    if (inputPoints.IsEmpty())
    {
        return;
    }

    Int32 N = inputPoints.GetCount();
    outputPoints.SetSize(N);
    Int32 i;
    if (N < 5)
    {
        LineSmooth3(inputPoints, outputPoints);
    }
    else
    {
        outputPoints[0].x = (3.0 * inputPoints[0].x + 2.0 * inputPoints[1].x + inputPoints[2].x - inputPoints[4].x) / 5.0;
        outputPoints[0].y = (3.0 * inputPoints[0].y + 2.0 * inputPoints[1].y + inputPoints[2].y - inputPoints[4].y) / 5.0;
        outputPoints[0].z = (3.0 * inputPoints[0].z + 2.0 * inputPoints[1].z + inputPoints[2].z - inputPoints[4].z) / 5.0;

        outputPoints[1].x = (4.0 * inputPoints[0].x + 3.0 * inputPoints[1].x + 2 * inputPoints[2].x + inputPoints[3].x) / 10.0;
        outputPoints[1].y = (4.0 * inputPoints[0].y + 3.0 * inputPoints[1].y + 2 * inputPoints[2].y + inputPoints[3].y) / 10.0;
        outputPoints[1].z = (4.0 * inputPoints[0].z + 3.0 * inputPoints[1].z + 2 * inputPoints[2].z + inputPoints[3].z) / 10.0;

        for (i = 2; i <= N - 3; i++)
        {
            outputPoints[i].x = (inputPoints[i - 2].x + inputPoints[i - 1].x + inputPoints[i].x + inputPoints[i + 1].x + inputPoints[i + 2].x) / 5.0;
            outputPoints[i].y = (inputPoints[i - 2].y + inputPoints[i - 1].y + inputPoints[i].y + inputPoints[i + 1].y + inputPoints[i + 2].y) / 5.0;
            outputPoints[i].z = (inputPoints[i - 2].z + inputPoints[i - 1].z + inputPoints[i].z + inputPoints[i + 1].z + inputPoints[i + 2].z) / 5.0;
        }

        outputPoints[N - 2].x = (4.0 * inputPoints[N - 1].x + 3.0 * inputPoints[N - 2].x + 2 * inputPoints[N - 3].x + inputPoints[N - 4].x) / 10.0;
        outputPoints[N - 2].y = (4.0 * inputPoints[N - 1].y + 3.0 * inputPoints[N - 2].y + 2 * inputPoints[N - 3].y + inputPoints[N - 4].y) / 10.0;
        outputPoints[N - 2].z = (4.0 * inputPoints[N - 1].z + 3.0 * inputPoints[N - 2].z + 2 * inputPoints[N - 3].z + inputPoints[N - 4].z) / 10.0;

        outputPoints[N - 1].x = (3.0 * inputPoints[N - 1].x + 2.0 * inputPoints[N - 2].x + inputPoints[N - 3].x - inputPoints[N - 5].x) / 5.0;
        outputPoints[N - 1].y = (3.0 * inputPoints[N - 1].y + 2.0 * inputPoints[N - 2].y + inputPoints[N - 3].y - inputPoints[N - 5].y) / 5.0;
        outputPoints[N - 1].z = (3.0 * inputPoints[N - 1].z + 2.0 * inputPoints[N - 2].z + inputPoints[N - 3].z - inputPoints[N - 5].z) / 5.0;
    }
}

Void RoadTopoBuild::LineSmooth7(Array<Coordinate> &inputPoints, Array<Coordinate> &outputPoints)
{
    if (inputPoints.IsEmpty())
    {
        return;
    }

    Int32 N = inputPoints.GetCount();
    outputPoints.SetSize(N);
    Int32 i;
    if (N < 7)
    {
        LineSmooth5(inputPoints, outputPoints);
    }
    else
    {
        // out[0] = (13.0 * in[0] + 10.0 * in[1] + 7.0 * in[2] + 4.0 * in[3] + in[4] - 2.0 * in[5] - 5.0 * in[6]) / 28.0;
        outputPoints[0].x = (13.0 * inputPoints[0].x + 10.0 * inputPoints[1].x + 7.0 * inputPoints[2].x + 4.0 * inputPoints[3].x + inputPoints[4].x - 2.0 * inputPoints[5].x - 5.0 * inputPoints[6].x) / 28.0;
        outputPoints[0].y = (13.0 * inputPoints[0].y + 10.0 * inputPoints[1].y + 7.0 * inputPoints[2].y + 4.0 * inputPoints[3].y + inputPoints[4].y - 2.0 * inputPoints[5].y - 5.0 * inputPoints[6].y) / 28.0;
        outputPoints[0].z = (13.0 * inputPoints[0].z + 10.0 * inputPoints[1].z + 7.0 * inputPoints[2].z + 4.0 * inputPoints[3].z + inputPoints[4].z - 2.0 * inputPoints[5].z - 5.0 * inputPoints[6].z) / 28.0;

        // out[1] = (5.0 * in[0] + 4.0 * in[1] + 3 * in[2] + 2 * in[3] + in[4] - in[6]) / 14.0;
        outputPoints[1].x = (5.0 * inputPoints[0].x + 4.0 * inputPoints[1].x + 3 * inputPoints[2].x + 2 * inputPoints[3].x + inputPoints[4].x - inputPoints[6].x) / 14.0;
        outputPoints[1].y = (5.0 * inputPoints[0].y + 4.0 * inputPoints[1].y + 3 * inputPoints[2].y + 2 * inputPoints[3].y + inputPoints[4].y - inputPoints[6].y) / 14.0;
        outputPoints[1].z = (5.0 * inputPoints[0].z + 4.0 * inputPoints[1].z + 3 * inputPoints[2].z + 2 * inputPoints[3].z + inputPoints[4].z - inputPoints[6].z) / 14.0;

        // out[2] = (7.0 * in[0] + 6.0 * in[1] + 5.0 * in[2] + 4.0 * in[3] + 3.0 * in[4] + 2.0 * in[5] + in[6]) / 28.0;
        outputPoints[2].x = (7.0 * inputPoints[0].x + 6.0 * inputPoints[1].x + 5.0 * inputPoints[2].x + 4.0 * inputPoints[3].x + 3.0 * inputPoints[4].x + 2.0 * inputPoints[5].x + inputPoints[6].x) / 28.0;
        outputPoints[2].y = (7.0 * inputPoints[0].y + 6.0 * inputPoints[1].y + 5.0 * inputPoints[2].y + 4.0 * inputPoints[3].y + 3.0 * inputPoints[4].y + 2.0 * inputPoints[5].y + inputPoints[6].y) / 28.0;
        outputPoints[2].z = (7.0 * inputPoints[0].z + 6.0 * inputPoints[1].z + 5.0 * inputPoints[2].z + 4.0 * inputPoints[3].z + 3.0 * inputPoints[4].z + 2.0 * inputPoints[5].z + inputPoints[6].z) / 28.0;

        for (i = 3; i <= N - 4; i++)
        {
            // out[i] = (in[i - 3] + in[i - 2] + in[i - 1] + in[i] + in[i + 1] + in[i + 2] + in[i + 3]) / 7.0;
            outputPoints[i].x = (inputPoints[i - 3].x + inputPoints[i - 2].x + inputPoints[i - 1].x + inputPoints[i].x + inputPoints[i + 1].x + inputPoints[i + 2].x + inputPoints[i + 3].x) / 7.0;
            outputPoints[i].y = (inputPoints[i - 3].y + inputPoints[i - 2].y + inputPoints[i - 1].y + inputPoints[i].y + inputPoints[i + 1].y + inputPoints[i + 2].y + inputPoints[i + 3].y) / 7.0;
            outputPoints[i].z = (inputPoints[i - 3].z + inputPoints[i - 2].z + inputPoints[i - 1].z + inputPoints[i].z + inputPoints[i + 1].z + inputPoints[i + 2].z + inputPoints[i + 3].z) / 7.0;
        }

        // out[N - 3] = (7.0 * in[N - 1] + 6.0 * in[N - 2] + 5.0 * in[N - 3] + 4.0 * in[N - 4] + 3.0 * in[N - 5] + 2.0 * in[N - 6] + in[N - 7]) / 28.0;
        outputPoints[N - 3].x = (7.0 * inputPoints[N - 1].x + 6.0 * inputPoints[N - 2].x + 5.0 * inputPoints[N - 3].x + 4.0 * inputPoints[N - 4].x + 3.0 * inputPoints[N - 5].x + 2.0 * inputPoints[N - 6].x + inputPoints[N - 7].x) / 28.0;
        outputPoints[N - 3].y = (7.0 * inputPoints[N - 1].y + 6.0 * inputPoints[N - 2].y + 5.0 * inputPoints[N - 3].y + 4.0 * inputPoints[N - 4].y + 3.0 * inputPoints[N - 5].y + 2.0 * inputPoints[N - 6].y + inputPoints[N - 7].y) / 28.0;
        outputPoints[N - 3].z = (7.0 * inputPoints[N - 1].z + 6.0 * inputPoints[N - 2].z + 5.0 * inputPoints[N - 3].z + 4.0 * inputPoints[N - 4].z + 3.0 * inputPoints[N - 5].z + 2.0 * inputPoints[N - 6].z + inputPoints[N - 7].z) / 28.0;

        // out[N - 2] = (5.0 * in[N - 1] + 4.0 * in[N - 2] + 3.0 * in[N - 3] + 2.0 * in[N - 4] + in[N - 5] - in[N - 7]) / 14.0;
        outputPoints[N - 2].x = (5.0 * inputPoints[N - 1].x + 4.0 * inputPoints[N - 2].x + 3 * inputPoints[N - 3].x + 2 * inputPoints[N - 4].x + inputPoints[N - 5].x - inputPoints[N - 7].x) / 14.0;
        outputPoints[N - 2].y = (5.0 * inputPoints[N - 1].y + 4.0 * inputPoints[N - 2].y + 3 * inputPoints[N - 3].y + 2 * inputPoints[N - 4].y + inputPoints[N - 5].y - inputPoints[N - 7].y) / 14.0;
        outputPoints[N - 2].z = (5.0 * inputPoints[N - 1].z + 4.0 * inputPoints[N - 2].z + 3 * inputPoints[N - 3].z + 2 * inputPoints[N - 4].z + inputPoints[N - 5].z - inputPoints[N - 7].z) / 14.0;

        // out[N - 1] = (13.0 * in[N - 1] + 10.0 * in[N - 2] + 7.0 * in[N - 3] + 4 * in[N - 4] + in[N - 5] - 2 * in[N - 6] - 5 * in[N - 7]) / 28.0;
        outputPoints[N - 1].x = (13.0 * inputPoints[N - 1].x + 10.0 * inputPoints[N - 2].x + 7.0 * inputPoints[N - 3].x + 4.0 * inputPoints[N - 4].x + inputPoints[N - 5].x - 2.0 * inputPoints[N - 6].x - 5.0 * inputPoints[N - 7].x) / 28.0;
        outputPoints[N - 1].y = (13.0 * inputPoints[N - 1].y + 10.0 * inputPoints[N - 2].y + 7.0 * inputPoints[N - 3].y + 4.0 * inputPoints[N - 4].y + inputPoints[N - 5].y - 2.0 * inputPoints[N - 6].y - 5.0 * inputPoints[N - 7].y) / 28.0;
        outputPoints[N - 1].z = (13.0 * inputPoints[N - 1].z + 10.0 * inputPoints[N - 2].z + 7.0 * inputPoints[N - 3].z + 4.0 * inputPoints[N - 4].z + inputPoints[N - 5].z - 2.0 * inputPoints[N - 6].z - 5.0 * inputPoints[N - 7].z) / 28.0;
    }
}

Bool RoadTopoBuild::ConnectedComponent(Array<Coordinate> &arrCloudPoints, Array<Array<Int32>> &arrArrCloudPoints, Double dDistance)
{
    if (arrCloudPoints.GetCount() < 1)
    {
        return false;
    }
    Array<Int32> arrMarked;

    RoadTopoGrid roadTopoGrid;
    roadTopoGrid.ConnectedComponent(arrCloudPoints, arrMarked, dDistance);

    Int32 i, j;
    Int32 nPointCount = arrCloudPoints.GetCount();
    Array<PointIndex_Tag> arrPointIndex_NumberTags;
    arrPointIndex_NumberTags.SetSize(nPointCount);

    PointIndex_Tag TempPointIndex_NumberTag;
    for (i = 0; i < nPointCount; i++)
    {
        TempPointIndex_NumberTag.nPointIndex = i;
        TempPointIndex_NumberTag.nNumberTag = arrMarked[i];

        arrPointIndex_NumberTags[i] = TempPointIndex_NumberTag;
    }

    sort(arrPointIndex_NumberTags.Begin(), arrPointIndex_NumberTags.End(), [&](const PointIndex_Tag &p1, const PointIndex_Tag &p2)
         { return p1.nNumberTag < p2.nNumberTag; });

    Int32 nNumberTag = 1;
    Int32 nBeginIndex = 0;

    for (i = 1; i < nPointCount; i++)
    {
        if (arrPointIndex_NumberTags[i].nNumberTag != arrPointIndex_NumberTags[nBeginIndex].nNumberTag)
        {
            Array<Int32> arrTempCoordinates;
            for (j = nBeginIndex; j < i; j++)
            {
                arrTempCoordinates.Add(arrPointIndex_NumberTags[j].nPointIndex);
            }
            arrArrCloudPoints.Add(arrTempCoordinates);

            nBeginIndex = i;
        }
    }

    Array<Int32> arrTempCoordinates;
    for (j = nBeginIndex; j < i; j++)
    {
        arrTempCoordinates.Add(arrPointIndex_NumberTags[j].nPointIndex);
    }
    arrArrCloudPoints.Add(arrTempCoordinates);

    return true;
}

RoadTopoBuild::RoadTopoBuild()
{
}

RoadTopoBuild::~RoadTopoBuild()
{
}

TopoGridEnv::TopoGridEnv()
{
    pGridUnits = NULL;
    IndexCols = 0;
    IndexRows = 0;
    dGridInterval = 0.0;
}

TopoGridEnv::~TopoGridEnv()
{
    if (pGridUnits != NULL)
    {
        delete[] pGridUnits;
        pGridUnits = NULL;
    }
}

TopoGridUnit::TopoGridUnit()
{
    nSegmentsCount = 0;
    pSegments = NULL;
}

TopoGridUnit::~TopoGridUnit()
{
    free(pSegments);
    pSegments = NULL;
}

Bool TopoGridUnit::AddSegment(const TGridSegment &tSegment)
{
    if (nSegmentsCount < 0)
    {
        return false;
    }

    pSegments = (TGridSegment *)realloc(pSegments, (nSegmentsCount + 1) * sizeof(TGridSegment));
    pSegments[nSegmentsCount].nLinkIndex = tSegment.nLinkIndex;
    pSegments[nSegmentsCount].nLinkSegIndex = tSegment.nLinkSegIndex;
    pSegments[nSegmentsCount].nPntIndex = tSegment.nPntIndex;

    nSegmentsCount++;
    return true;
}

RoadTopoGrid::RoadTopoGrid()
{
    m_arrLinkPoints.SetSize(0);
}

RoadTopoGrid::~RoadTopoGrid()
{
    delete[] m_topoGridEnv.pGridUnits;
    m_topoGridEnv.pGridUnits = NULL;

    m_arrLinkPoints.Clear();
}

Double RoadTopoGrid::DistanceToSegment(const Coordinate &pntHitTest, const Coordinate &pntStart, const Coordinate &pntEnd, Coordinate &pntMatch)
{
    if (pntStart.DistanceXY(pntEnd) < Geometries_EP)
    {
        pntMatch = pntStart;

        Double dDistance2 = (pntStart.x - pntHitTest.x) * (pntStart.x - pntHitTest.x) + (pntStart.y - pntHitTest.y) * (pntStart.y - pntHitTest.y);
        Double dDistance = sqrt(dDistance2);
        return dDistance;
    }

    Double da2 = (pntStart.x - pntHitTest.x) * (pntStart.x - pntHitTest.x) + (pntStart.y - pntHitTest.y) * (pntStart.y - pntHitTest.y);
    Double db2 = (pntEnd.x - pntHitTest.x) * (pntEnd.x - pntHitTest.x) + (pntEnd.y - pntHitTest.y) * (pntEnd.y - pntHitTest.y);
    Double dc2 = (pntStart.x - pntEnd.x) * (pntStart.x - pntEnd.x) + (pntStart.y - pntEnd.y) * (pntStart.y - pntEnd.y);

    Double dtemp = 0.0;
    dtemp = (da2 + dc2 - db2) / (2 * dc2);

    if (dtemp < 0)
    {
        dtemp = 0.0;
    }
    else if (dtemp > 1.0)
    {
        dtemp = 1.0;
    }

    Double dx = (pntEnd.x - pntStart.x) * dtemp + pntStart.x;
    Double dy = (pntEnd.y - pntStart.y) * dtemp + pntStart.y;
    Double dz = (pntEnd.z - pntStart.z) * dtemp + pntStart.z;

    pntMatch.x = dx;
    pntMatch.y = dy;
    pntMatch.z = dz;

    Double dDistance2 = (dx - pntHitTest.x) * (dx - pntHitTest.x) + (dy - pntHitTest.y) * (dy - pntHitTest.y);
    Double dDistance = sqrt(dDistance2);
    return dDistance;
}

Double RoadTopoGrid::DistanceToSegment(const Coordinate &pntHitTest, const Coordinate &pntStart, const Coordinate &pntEnd)
{
    if (pntStart.DistanceXY(pntEnd) < Geometries_EP)
    {
        return pntStart.DistanceXY(pntHitTest);
    }

    Coordinate ptDir = pntEnd - pntStart;
    ptDir.z = 0;
    ptDir.Normalize();

    Coordinate ptDist = pntHitTest - pntStart;
    ptDist.z = 0;
    Double dLen = ptDist.GetLength();
    Double dDot = ptDir * ptDist;
    Double dDist = sqrt(dLen * dLen - dDot * dDot);

    return dDist;
}

Bool RoadTopoGrid::IsIntersectLineSect(const Coordinate &pntStart1, const Coordinate &pntEnd1, const Coordinate &pntStart2, const Coordinate &pntEnd2)
{
    Double dToler = Geometries_EP;

    Double dmax, dmin;
    dmax = NIMAX(pntStart1.x, pntEnd1.x);
    dmin = NIMIN(pntStart1.x, pntEnd1.x);

    if (((pntStart2.x - dmax) > dToler && (pntEnd2.x - dmax) > dToler) || ((pntStart2.x - dmin) < (-dToler) && (pntEnd2.x - dmin) < (-dToler)))
    {
        return false;
    }

    dmax = NIMAX(pntStart1.y, pntEnd1.y);
    dmin = NIMIN(pntStart1.y, pntEnd1.y);

    if (((pntStart2.y - dmax) > dToler && (pntEnd2.y - dmax) > dToler) || ((pntStart2.y - dmin) < (-dToler) && (pntEnd2.y - dmin) < (-dToler)))
    {
        return false;
    }

    Double delt, t0, t1,
        dOffsetX1 = pntEnd1.x - pntStart1.x,
        dOffsetY1 = pntEnd1.y - pntStart1.y,
        dOffsetX2 = pntEnd2.x - pntStart2.x,
        dOffsetY2 = pntEnd2.y - pntStart2.y,
        dOffsetX12 = pntStart1.x - pntStart2.x,
        dOffsetY12 = pntStart1.y - pntStart2.y;

    delt = dOffsetX1 * dOffsetY2 - dOffsetX2 * dOffsetY1;
    Double deltB = delt;
    if (!BaseAlgorithm::Is0(dOffsetX1) && !BaseAlgorithm::Is0(dOffsetX2))
    {
        deltB /= (dOffsetX1 * dOffsetX2);
    }

    if (BaseAlgorithm::Is0(deltB))
    {
        if ((pntStart1.DistanceXY(pntStart2) < Geometries_EP) || (pntStart1.DistanceXY(pntEnd2) < Geometries_EP))
        {
            return true;
        }

        if ((pntEnd1.DistanceXY(pntStart2) < Geometries_EP) || (pntEnd1.DistanceXY(pntEnd2) < Geometries_EP))
        {
            return true;
        }

        return false;
    }

    t0 = (dOffsetX1 * dOffsetY12 - dOffsetY1 * dOffsetX12) / delt;
    t1 = (dOffsetX2 * dOffsetY12 - dOffsetY2 * dOffsetX12) / delt;

    if (BaseAlgorithm::Is0(t0))
    {
        return true;
    }
    else if (BaseAlgorithm::Is0(t0 - 1))
    {
        return true;
    }
    if (BaseAlgorithm::Is0(t1))
    {
        return true;
    }
    else if (BaseAlgorithm::Is0(t1 - 1))
    {
        return true;
    }

    if ((t0 < 0) && ((fabs(t0 * dOffsetX2) > dToler) || (fabs(t0 * dOffsetY2) > dToler)))
    {
        return false;
    }
    if ((t0 > 1.0) && ((fabs((t0 - 1) * dOffsetX2) > dToler) || (fabs((t0 - 1) * dOffsetY2) > dToler)))
    {
        return false;
    }
    if ((t1 < 0) && ((fabs(t1 * dOffsetX1) > dToler) || (fabs(t1 * dOffsetY1) > dToler)))
    {
        return false;
    }
    if ((t1 > 1.0) && ((fabs((t1 - 1) * dOffsetX1) > dToler) || (fabs((t1 - 1) * dOffsetY1) > dToler)))
    {
        return false;
    }

    return true;
}

Bool RoadTopoGrid::BuildTopoGrid(const Array<Coordinate> &arrPoints, Double dGridInterval)
{
    Int32 i = 0;
    Int32 nRefGeoCount = arrPoints.GetCount();
    if ((nRefGeoCount == 0) || (dGridInterval < 0.0))
    {
        return false;
    }

    Envelope rcBounds;
    Double dMinX, dMaxX, dMinY, dMaxY;
    dMinX = arrPoints[0].x;
    dMaxX = arrPoints[0].x;
    dMinY = arrPoints[0].y;
    dMaxY = arrPoints[0].y;

    for (i = 1; i < arrPoints.GetCount(); i++)
    {
        if (arrPoints[i].x < dMinX)
        {
            dMinX = arrPoints[i].x;
        }
        else if (arrPoints[i].x > dMaxX)
        {
            dMaxX = arrPoints[i].x;
        }

        if (arrPoints[i].y < dMinY)
        {
            dMinY = arrPoints[i].y;
        }
        else if (arrPoints[i].y > dMaxY)
        {
            dMaxY = arrPoints[i].y;
        }
    }

    rcBounds.SetMinX(dMinX);
    rcBounds.SetMaxX(dMaxX);
    rcBounds.SetMinY(dMinY);
    rcBounds.SetMaxY(dMaxY);

    Double dTolerance = rcBounds.GetWidth() / 1000000;
    rcBounds.ExpandBy(dTolerance);

    if (!IniTopGrid(rcBounds, dGridInterval))
    {
        return false;
    }

    BuildTopoGridPoints(arrPoints);

    return true;
}

Bool RoadTopoGrid::BuildTopoGrid(Envelope &rcBounds, const Array<Coordinate> &arrPoints, Double dGridInterval)
{
    if (rcBounds.GetArea() < Geometries_EP)
    {
        return false;
    }

    Double dTolerance = rcBounds.GetWidth() / 1000000;
    rcBounds.ExpandBy(dTolerance);

    Base::Array<Coordinate *> *coordinates = new Base::Array<Coordinate *>;
    for (SizeT i = 0; i < arrPoints.GetCount(); i++)
    {
        Coordinate *temp = new Coordinate(arrPoints[i].x, arrPoints[i].y, arrPoints[i].z);
        coordinates->Add(temp);
    }

    LineString pLine(coordinates);
    LineString *pGeoLine = new LineString(pLine);

    if (!IniTopGrid(rcBounds, dGridInterval))
    {
        return false;
    }

    Int32 nTickCount = 0;
    BuildTopoGridSegment(pGeoLine, 0, nTickCount);

    DELETE_PTR(pGeoLine);
    return true;
}

Bool RoadTopoGrid::BuildTopoGrid(Envelope &rcBounds, const Array<LineString *> &arrpGeoLines, Double dGridInterval)
{
    Int32 nRefGeoCount = arrpGeoLines.GetCount();
    if ((nRefGeoCount == 0) || (dGridInterval < 0))
    {
        return false;
    }

    if (rcBounds.GetArea() < Geometries_EP)
    {
        return false;
    }

    if (!IniTopGrid(rcBounds, dGridInterval))
    {
        return false;
    }

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    Int32 nIndexRows = m_topoGridEnv.IndexRows;
    Int32 nIndexCols = m_topoGridEnv.IndexCols;

    Int32 nTickCount = 0;
    Int32 i = 0;
    LineString *pGeoLine = NULL;
    for (i = 0; i < nRefGeoCount; i++)
    {
        pGeoLine = arrpGeoLines[i];
        if (pGeoLine == NULL)
        {
            continue;
        }

        BuildTopoGridSegment(pGeoLine, i, nTickCount);
    }

    return true;
}

Bool RoadTopoGrid::BuildTopoGridPoints(const Array<Coordinate> &arrPoints)
{
    Int32 nPointCount = arrPoints.GetCount();

    SizeT J = 0;

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Int32 nGridCount = IndexRows * IndexCols;

    Int32 VertexRow = 0, VertexCol = 0;
    Int32 nGridIndex = 0;

    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    Int32 nPntIndex = 0;

    for (J = 0; J < nPointCount; J++)
    {
        VertexRow = (Int32)floor((arrPoints[J].y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((arrPoints[J].x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < 0) || (VertexRow >= IndexRows) || (VertexCol < 0) || (VertexCol >= IndexCols))
        {
            continue;
        }

        nGridIndex = VertexRow * IndexCols + VertexCol;
        if (nGridIndex < 0 || nGridIndex >= nGridCount)
        {
            continue;
        }

        TGridSegment tSegment;
        tSegment.nLinkIndex = -1;
        tSegment.nLinkSegIndex = -1;
        tSegment.nPntIndex = J;

        m_topoGridEnv.pGridUnits[nGridIndex].AddSegment(tSegment);
    }

    return true;
}

Bool RoadTopoGrid::GetNearPoints(Array<Coordinate> &arrPoints, Int32 nIndex, Int32 VertexRow, Int32 VertexCol, Array<Int32> &arrGridMarked,
                                 Array<Int32> &arrPointIndex, Double dMaxDistance)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Coordinate pnt = arrPoints[nIndex];
    arrPointIndex.Clear();

    Int32 j = 0, H = 0;
    TGridSegment *pItems = NULL;
    Int32 nSegCount;
    Double dSquare = dMaxDistance * dMaxDistance;

    for (H = 0; H < 9; H++)
    {
        Int32 nTempRow = VertexRow + (Int32(H / 3)) - 1;
        Int32 nTempCol = VertexCol + (Int32(H % 3)) - 1;

        if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
        {
            continue;
        }

        Int32 nGridIndex = nTempRow * IndexCols + nTempCol;
        if (arrGridMarked[nGridIndex] != 0)
        {
            continue;
        }

        pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
        nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

        for (j = 0; j < nSegCount; j++)
        {
            if (arrPoints[pItems[j].nPntIndex].DistanceSquare(pnt) <= dSquare)
            {
                arrPointIndex.Add(pItems[j].nPntIndex);
            }
        }
    }
    return true;
}

Bool RoadTopoGrid::Merge(Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Int32 nNumberTag, Array<Int32> &arrPointIndex)
{
    Int32 nMinNumberTag = nNumberTag;
    Int32 nPointCount = arrMarked.GetCount();

    Int32 j, nTempNumberTag;
    for (j = 0; j < arrPointIndex.GetCount(); j++) // �����е���С��tag
    {
        nTempNumberTag = arrMarked[arrPointIndex[j]];

        if (nTempNumberTag > 0)
        {
            if ((arrRealMarked[nTempNumberTag] == 0) && (nTempNumberTag < nMinNumberTag)) // �޶�Ӧֵ
            {
                nMinNumberTag = nTempNumberTag;
            }
            else if ((arrRealMarked[nTempNumberTag] != 0) && (arrRealMarked[nTempNumberTag] < nMinNumberTag)) // �ж�Ӧֵ
            {
                nMinNumberTag = arrRealMarked[nTempNumberTag];
            }
        }
    }

    // ��Tag�Ѿ��ı䣬��ȡ�ı���ֵ��Ϊ��ֵ
    if (arrRealMarked[nMinNumberTag] == 0)
    {
        arrRealMarked[nMinNumberTag] = nMinNumberTag;
    }
    else
    {
        // Ƕ�ײ�ѯ��һֱ��tag���ն�Ӧ��ֵֹͣ
        while (arrRealMarked[nMinNumberTag] != nMinNumberTag)
        {
            nMinNumberTag = arrRealMarked[nMinNumberTag];
        }
    }

    for (j = 0; j < arrPointIndex.GetCount(); j++)
    {
        nTempNumberTag = arrMarked[arrPointIndex[j]];
        if ((nTempNumberTag > nMinNumberTag))
        {
            if (arrRealMarked[nTempNumberTag] == 0)
            {
                arrRealMarked[nTempNumberTag] = nMinNumberTag;
            }
            else
            {
                Int32 nTempMark1, nTempMark2;
                nTempMark1 = nTempNumberTag;
                while (nTempMark1 > nMinNumberTag)
                {
                    nTempMark2 = arrRealMarked[nTempMark1];
                    arrRealMarked[nTempMark1] = nMinNumberTag;
                    nTempMark1 = nTempMark2;
                }
            }
        }
        arrMarked[arrPointIndex[j]] = nMinNumberTag;
    }
    return true;
}

Bool RoadTopoGrid::GetMindisBetweenSegmets(const Array<Coordinate> &arrPoints, const GridNumberTag &gridNumberTag,
                                           const GridNumberTag &neighborNumberTag, Double dMaxDistance)
{
    Double dDisSquare = -1.0;
    Double dMaxSquare = dMaxDistance * dMaxDistance;

    Int32 nCount = gridNumberTag.arrPointIndex.GetCount();
    Int32 nNeighborCount = neighborNumberTag.arrPointIndex.GetCount();

    for (Int32 m = 0; m < nCount; m++)
    {
        Coordinate temCoord1 = arrPoints[gridNumberTag.arrPointIndex[m]];

        for (Int32 n = 0; n < nNeighborCount; n++)
        {
            dDisSquare = arrPoints[neighborNumberTag.arrPointIndex[n]].DistanceSquare(temCoord1);

            if (dDisSquare <= dMaxSquare)
            {
                return true;
            }
        }
    }

    return false;
}

Bool RoadTopoGrid::GetGridSegments(const Array<Coordinate> &arrPoints, const Int32 nGridIndex, const Array<Int32> &arrMarked, Array<GridNumberTag> &differNumberTag)
{
    Int32 nSegCount;
    nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

    if (nSegCount == 0)
    {
        return false; // ������Ϊ��ʱ����false
    }

    TGridSegment *pItems = NULL;
    pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;

    set<Int32> setNumberTag;
    setNumberTag.insert(arrMarked[pItems[0].nPntIndex]);
    for (Int32 j = 1; j < nSegCount; j++)
    {
        if (arrMarked[pItems[j].nPntIndex] != arrMarked[pItems[0].nPntIndex])
        {
            setNumberTag.insert(arrMarked[pItems[j].nPntIndex]);
        }
    }

    // һ�������ڵ���ݾ���ֶ�
    for (set<Int32>::iterator it = setNumberTag.begin(); it != setNumberTag.end(); it++)
    {
        GridNumberTag temGridNumberTag;
        temGridNumberTag.NumberTag = *it;
        for (Int32 j = 0; j < nSegCount; j++)
        {
            Int32 pntIndex = pItems[j].nPntIndex;
            if (arrMarked[pntIndex] == (*it))
            {
                temGridNumberTag.arrPointIndex.Add(pntIndex);
            }
        }
        differNumberTag.Add(temGridNumberTag);
    }
    return true;
}

Bool RoadTopoGrid::MergeSameGrid(const Array<Coordinate> &arrPoints, Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Array<GridNumberTag> &differNumberTag, Double dMaxDistance)
{
    if (differNumberTag.GetCount() < 2)
    {
        return false;
    }
    for (Int32 i = 0; i < differNumberTag.GetCount(); i++)
    {
        GridNumberTag tempGridNumberTag = differNumberTag[i];
        for (Int32 j = i + 1; j < differNumberTag.GetCount(); j++)
        {
            GridNumberTag compareGridNumberTag = differNumberTag[j];

            if (GetMindisBetweenSegmets(arrPoints, tempGridNumberTag, compareGridNumberTag, dMaxDistance))
            {
                Array<Int32> arrMergeIndex;
                arrMergeIndex.Add(tempGridNumberTag.arrPointIndex);
                arrMergeIndex.Add(compareGridNumberTag.arrPointIndex);

                Merge(arrMarked, arrRealMarked, tempGridNumberTag.NumberTag, arrMergeIndex);
            }
        }
    }

    return true;
}

Void RoadTopoGrid::MergeGridTag(const Array<Coordinate> &arrPoints, Int32 VertexRow, Int32 VertexCol, Array<Int32> &arrMarked, Array<Int32> &arrRealMarked, Array<GridNumberTag> &curNumberTag, Array<Int32> &arrGridMarked, Double dMaxDistance)
{
    if (curNumberTag.GetCount() > 1)
    {
        MergeSameGrid(arrPoints, arrMarked, arrRealMarked, curNumberTag, dMaxDistance);
    }
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    for (Int32 H = 0; H < 9; H++)
    {
        Int32 nTempRow = VertexRow + (Int32(H / 3)) - 1;
        Int32 nTempCol = VertexCol + (Int32(H % 3)) - 1;

        if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
        {
            continue;
        }

        Int32 nGridIndex = nTempRow * IndexCols + nTempCol;
        if (arrGridMarked[nGridIndex] != 0)
        {
            continue;
        }

        Array<GridNumberTag> neighborNumberTag;
        if (!(GetGridSegments(arrPoints, nGridIndex, arrMarked, neighborNumberTag)))
        {
            continue;
        }

        for (Int32 i = 0; i < curNumberTag.GetCount(); i++)
        {
            Int32 temCurNumberTag = curNumberTag[i].NumberTag;
            for (Int32 j = 0; j < neighborNumberTag.GetCount(); j++)
            {
                if (neighborNumberTag[j].NumberTag == temCurNumberTag)
                {
                    continue;
                }

                if (GetMindisBetweenSegmets(arrPoints, curNumberTag[i], neighborNumberTag[j], dMaxDistance))
                {
                    Array<Int32> arrMergeIndex;
                    arrMergeIndex.Add(curNumberTag[i].arrPointIndex);
                    arrMergeIndex.Add(neighborNumberTag[j].arrPointIndex);

                    Merge(arrMarked, arrRealMarked, temCurNumberTag, arrMergeIndex);
                }
            }
        }
    }
}

Bool RoadTopoGrid::BuildTopoGrid(const Array<LineString *> &arrpGeoLines, Double dGridInterval)
{
    Int32 i = 0;
    Int32 nRefGeoCount = arrpGeoLines.GetCount();
    if ((nRefGeoCount == 0) || (dGridInterval < 0))
    {
        return false;
    }

    Envelope rcBounds = arrpGeoLines[0]->GetBound();
    for (i = 1; i < arrpGeoLines.GetCount(); i++)
    {
        if (arrpGeoLines[i] == NULL)
        {
            continue;
        }

        Envelope rcBounds1 = arrpGeoLines[i]->GetBound();
        rcBounds.Union(rcBounds1);
    }

    Double dTolerance = rcBounds.GetWidth() / 1000000;
    rcBounds.ExpandBy(dTolerance);

    if (!IniTopGrid(rcBounds, dGridInterval))
    {
        return false;
    }

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    Int32 nIndexRows = m_topoGridEnv.IndexRows;
    Int32 nIndexCols = m_topoGridEnv.IndexCols;

    Int32 nTickCount = 0;
    LineString *pGeoLine = NULL;
    for (i = 0; i < nRefGeoCount; i++)
    {
        pGeoLine = arrpGeoLines[i];
        if (pGeoLine == NULL)
        {
            continue;
        }

        BuildTopoGridSegment(pGeoLine, i, nTickCount);
    }

    return true;
}

Bool RoadTopoGrid::IniTopGrid(const Envelope &rcBound, Double dGridInterval)
{
    m_topoGridEnv.rcBounds = rcBound;
    m_topoGridEnv.dGridInterval = dGridInterval;

    Int64 nGridCount = 0;
    if (rcBound.GetWidth() < rcBound.GetHeight())
    {
        m_topoGridEnv.IndexCols = (Int32)(floor(rcBound.GetWidth() / m_topoGridEnv.dGridInterval) + 1);
        m_topoGridEnv.IndexRows = (Int32)(floor(rcBound.GetHeight() / m_topoGridEnv.dGridInterval) + 1);
        nGridCount = (Int64)m_topoGridEnv.IndexCols * m_topoGridEnv.IndexRows;

        if (nGridCount > MAXGRIDCOUNT)
        {
            Double dRatio = sqrt((Double)(nGridCount / MAXGRIDCOUNT));
            m_topoGridEnv.dGridInterval = m_topoGridEnv.dGridInterval * dRatio;
            m_topoGridEnv.IndexCols = (Int32)(floor(rcBound.GetWidth() / m_topoGridEnv.dGridInterval) + 1);
            m_topoGridEnv.IndexRows = (Int32)(floor(rcBound.GetHeight() / m_topoGridEnv.dGridInterval) + 1);
            nGridCount = m_topoGridEnv.IndexCols * m_topoGridEnv.IndexRows;
        }
    }
    else
    {
        m_topoGridEnv.IndexCols = (Int32)(floor(rcBound.GetWidth() / m_topoGridEnv.dGridInterval) + 1);
        m_topoGridEnv.IndexRows = (Int32)(floor(rcBound.GetHeight() / m_topoGridEnv.dGridInterval) + 1);
        nGridCount = (Int64)m_topoGridEnv.IndexCols * m_topoGridEnv.IndexRows;

        if (nGridCount > MAXGRIDCOUNT)
        {
            Double dRatio = sqrt((Double)(nGridCount / MAXGRIDCOUNT));
            m_topoGridEnv.dGridInterval = m_topoGridEnv.dGridInterval * dRatio;
            m_topoGridEnv.IndexCols = (Int32)(floor(rcBound.GetWidth() / m_topoGridEnv.dGridInterval) + 1);
            m_topoGridEnv.IndexRows = (Int32)(floor(rcBound.GetHeight() / m_topoGridEnv.dGridInterval) + 1);
            nGridCount = m_topoGridEnv.IndexCols * m_topoGridEnv.IndexRows;
        }
    }

    if (nGridCount < 0)
    {
        return false;
    }

    m_topoGridEnv.pGridUnits = (TopoGridUnit *)new TopoGridUnit[nGridCount];

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }
    return true;
}

Bool RoadTopoGrid::BuildTopoGridSegment(LineString *pGeoLine, Int32 nGeoIndex, Int32 &nTickCount)
{
    if (pGeoLine == NULL)
    {
        return false;
    }

    Array<Coordinate *> *ppPoints = pGeoLine->GetCoordinates();

    Int32 nPointCount = ppPoints->GetCount();

    SizeT I = 0, J = 0, K = 0;

    Coordinate *pVertex1 = NULL;
    Coordinate *pVertex2 = NULL;

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Int32 nGridCount = IndexRows * IndexCols;

    Int32 Vertex1Row = 0, Vertex1Col = 0, Vertex2Row = 0, Vertex2Col = 0;
    Int32 nGridIndex = 0;
    Int32 Row = 0, Col = 0, Row1 = 0, Col1 = 0, Row2 = 0, Col2 = 0;

    Coordinate GridPoints[5];
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    Int32 nPntIndex = 0;

    for (J = 0; J < nPointCount - 1; J++)
    {
        pVertex1 = (*ppPoints)[J];
        pVertex2 = (*ppPoints)[J + 1];

        if (nTickCount >= m_arrLinkPoints.GetCount())
        {
            m_arrLinkPoints.SetSize(nTickCount + 8192);
        }

        m_arrLinkPoints.SetAt(nTickCount++, *pVertex1);
        nPntIndex = nTickCount - 1;

        if (J == nPointCount - 2)
        {
            if (nTickCount >= m_arrLinkPoints.GetCount())
            {
                m_arrLinkPoints.SetSize(nTickCount + 8192);
            }

            m_arrLinkPoints.SetAt(nTickCount++, *pVertex2);
        }

        Vertex1Row = (Int32)floor((pVertex1->y - rcBound.GetMaxY()) / Dy);
        Vertex1Col = (Int32)floor((pVertex1->x - rcBound.GetMinX()) / Dx);
        Vertex2Row = (Int32)floor((pVertex2->y - rcBound.GetMaxY()) / Dy);
        Vertex2Col = (Int32)floor((pVertex2->x - rcBound.GetMinX()) / Dx);

        if (Vertex1Row < 0)
        {
            Vertex1Row = 0;
        }
        else if (Vertex1Row >= IndexRows)
        {
            Vertex1Row = IndexRows - 1;
        }
        if (Vertex1Col < 0)
        {
            Vertex1Col = 0;
        }
        else if (Vertex1Col >= IndexCols)
        {
            Vertex1Col = IndexCols - 1;
        }
        if (Vertex2Row < 0)
        {
            Vertex2Row = 0;
        }
        else if (Vertex2Row >= IndexRows)
        {
            Vertex2Row = IndexRows - 1;
        }
        if (Vertex2Col < 0)
        {
            Vertex2Col = 0;
        }
        else if (Vertex2Col >= IndexCols)
        {
            Vertex2Col = IndexCols - 1;
        }

        if ((Vertex1Row == Vertex2Row) && (Vertex1Col == Vertex2Col))
        {
            nGridIndex = Vertex1Row * IndexCols + Vertex1Col;
            if (nGridIndex < 0 || nGridIndex >= nGridCount)
            {
                continue;
            }

            TGridSegment tSegment;
            tSegment.nLinkIndex = nGeoIndex;
            tSegment.nLinkSegIndex = J;
            tSegment.nPntIndex = nPntIndex;
            m_topoGridEnv.pGridUnits[nGridIndex].AddSegment(tSegment);
        }
        else
        {
            Row1 = NIMIN(Vertex1Row, Vertex2Row);
            Row2 = NIMAX(Vertex1Row, Vertex2Row);
            Col1 = NIMIN(Vertex1Col, Vertex2Col);
            Col2 = NIMAX(Vertex1Col, Vertex2Col);

            for (Row = Row1; Row <= Row2; Row++)
            {
                for (Col = Col1; Col <= Col2; Col++)
                {
                    GridPoints[0].x = rcBound.GetMinX() + Col * Dx;
                    GridPoints[0].y = rcBound.GetMaxY() + Row * Dy;
                    GridPoints[2].x = rcBound.GetMinX() + (Col + 1) * Dx;
                    GridPoints[2].y = rcBound.GetMaxY() + (Row + 1) * Dy;
                    GridPoints[1].x = GridPoints[2].x;
                    GridPoints[1].y = GridPoints[0].y;
                    GridPoints[3].x = GridPoints[0].x;
                    GridPoints[3].y = GridPoints[2].y;
                    GridPoints[4] = GridPoints[0];

                    for (K = 1; K <= 4; K++)
                    {
                        if (IsIntersectLineSect(*pVertex1, *pVertex2, GridPoints[K - 1], GridPoints[K]))
                        {
                            nGridIndex = Row * IndexCols + Col;
                            if (nGridIndex < 0 || nGridIndex >= nGridCount)
                            {
                                continue;
                            }

                            TGridSegment tSegment;
                            tSegment.nLinkIndex = nGeoIndex;
                            tSegment.nLinkSegIndex = J;
                            tSegment.nPntIndex = nPntIndex;
                            m_topoGridEnv.pGridUnits[nGridIndex].AddSegment(tSegment);

                            break;
                        }
                    }
                }
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;
    Int32 nPntCount = arrpntTrack.GetCount();
    if (nPntCount <= 1)
    {
        return false;
    }

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;
    Coordinate pntMatch, pntFrom, pntTo;
    Coordinate e1, e2;
    Double dDistance = -1.0;
    Coordinate *pPoints = (Coordinate *)m_arrLinkPoints.Data();

    for (i = 0; i < nPntCount; i++)
    {
        Coordinate pntTrack = *(arrpntTrack[i]);
        VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
        {
            arrPointMatch[i] = 2;
            continue;
        }

        Int32 nMinPntIndex = -1;
        Double dMinDis = -1.0;

        if (i < (nPntCount - 1))
        {
            pntFrom = pntTrack;
            pntTo = *(arrpntTrack[i + 1]);
        }
        else
        {
            pntFrom = *(arrpntTrack[i - 1]);
            pntTo = pntTrack;
        }

        e1 = pntTo - pntFrom;
        e1.Normalize();

        for (H = 0; H < 9; H++)
        {
            nTempRow = VertexRow + (Int32(H / 3)) - 1;
            nTempCol = VertexCol + (Int32(H % 3)) - 1;

            if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
            {
                continue;
            }

            nGridIndex = nTempRow * IndexCols + nTempCol;
            pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
            nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

            for (j = 0; j < nSegCount; j++)
            {
                // �̹߳�ϵ����
                if (((pntTrack.z > pPoints[pItems[j].nPntIndex].z) && (pntTrack.z > pPoints[pItems[j].nPntIndex + 1].z)) || (((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex].z) && ((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex + 1].z)))
                {
                    continue;
                }

                if (((pntTrack.z < pPoints[pItems[j].nPntIndex].z) && (pntTrack.z < pPoints[pItems[j].nPntIndex + 1].z)) || (((pntTrack.z - 5.5) > pPoints[pItems[j].nPntIndex].z) && ((pntTrack.z - 5.5) > pPoints[pItems[j].nPntIndex + 1].z)))
                {
                    continue;
                }

                e2 = pPoints[pItems[j].nPntIndex + 1] - pPoints[pItems[j].nPntIndex];
                e2.Normalize();

                if ((e1.DotProduct(e2) < 0.0 && direction_type) || (e1.DotProduct(e2) >= 0 && !direction_type)) // ������Ҫ��Աȷ���һ��
                {
                    continue;
                }

                dDistance = DistanceToSegment(pntTrack, pPoints[pItems[j].nPntIndex], pPoints[pItems[j].nPntIndex + 1], pntMatch);

                if (dDistance < dTolerance)
                {
                    arrPointMatch[i] = 1;
                    break;
                }
            }

            if (arrPointMatch[i] == 1)
            {
                break;
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Array<Coordinate> &arrpntTrack, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;
    Int32 nPntCount = arrpntTrack.GetCount();
    if (nPntCount <= 1)
    {
        return false;
    }

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;
    Coordinate pntMatch, pntFrom, pntTo;
    Coordinate e1, e2;
    Double dDistance = -1.0;
    Coordinate *pPoints = (Coordinate *)m_arrLinkPoints.Data();

    for (i = 0; i < nPntCount; i++)
    {
        Coordinate pntTrack = arrpntTrack[i];
        VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
        {
            arrPointMatch[i] = 2;
            continue;
        }

        Int32 nMinPntIndex = -1;
        Double dMinDis = -1.0;

        if (i < (nPntCount - 1))
        {
            pntFrom = pntTrack;
            pntTo = arrpntTrack[i + 1];
        }
        else
        {
            pntFrom = arrpntTrack[i - 1];
            pntTo = pntTrack;
        }

        e1 = pntTo - pntFrom;
        e1.Normalize();

        for (H = 0; H < 9; H++)
        {
            nTempRow = VertexRow + (Int32(H / 3)) - 1;
            nTempCol = VertexCol + (Int32(H % 3)) - 1;

            if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
            {
                continue;
            }

            nGridIndex = nTempRow * IndexCols + nTempCol;
            pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
            nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

            for (j = 0; j < nSegCount; j++)
            {
                // 高程也作为约束条件之一
                if (((pntTrack.z > pPoints[pItems[j].nPntIndex].z) && (pntTrack.z > pPoints[pItems[j].nPntIndex + 1].z)) || (((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex].z) && ((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex + 1].z)))
                {
                    continue;
                }

                if (((pntTrack.z < pPoints[pItems[j].nPntIndex].z) && (pntTrack.z < pPoints[pItems[j].nPntIndex + 1].z)) || (((pntTrack.z - 5.5) > pPoints[pItems[j].nPntIndex].z) && ((pntTrack.z - 5.5) > pPoints[pItems[j].nPntIndex + 1].z)))
                {
                    continue;
                }

                e2 = pPoints[pItems[j].nPntIndex + 1] - pPoints[pItems[j].nPntIndex];
                e2.Normalize();

                if ((e1.DotProduct(e2) < 0.0 && direction_type) || (e1.DotProduct(e2) >= 0 && !direction_type)) // ������Ҫ��Աȷ���һ��
                {
                    continue;
                }

                dDistance = DistanceToSegment(pntTrack, pPoints[pItems[j].nPntIndex], pPoints[pItems[j].nPntIndex + 1], pntMatch);

                if (dDistance < dTolerance)
                {
                    arrPointMatch[i] = 1;
                    break;
                }
            }

            if (arrPointMatch[i] == 1)
            {
                break;
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Array<Coordinate *> &arrpntTrack, Int32 nIndex, Array<Int32> &arrPointMatch, double dTolerance, bool direction_type)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;
    Int32 nPntCount = arrpntTrack.GetCount();
    if (nPntCount <= 1)
    {
        return false;
    }

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;
    Coordinate pntMatch, pntFrom, pntTo;
    Coordinate e1, e2;
    Double dDistance = -1.0;
    Coordinate *pPoints = (Coordinate *)m_arrLinkPoints.Data();

    for (i = 0; i < nPntCount; i++)
    {
        Coordinate pntTrack = *(arrpntTrack[i]);
        VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
        {
            continue;
        }

        Int32 nMinPntIndex = -1;
        Double dMinDis = -1.0;

        if (i < (nPntCount - 1))
        {
            pntFrom = pntTrack;
            pntTo = *(arrpntTrack[i + 1]);
        }
        else
        {
            pntFrom = *(arrpntTrack[i - 1]);
            pntTo = pntTrack;
        }

        e1 = pntTo - pntFrom;
        e1.Normalize();
        for (H = 0; H < 9; H++)
        {
            nTempRow = VertexRow + (Int32(H / 3)) - 1;
            nTempCol = VertexCol + (Int32(H % 3)) - 1;

            if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
            {
                continue;
            }

            nGridIndex = nTempRow * IndexCols + nTempCol;
            pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
            nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

            for (j = 0; j < nSegCount; j++)
            {
                // if ((nIndex == pItems[j].nLinkIndex) && ((i == pItems[j].nLinkSegIndex) || (i == (pItems[j].nLinkSegIndex-1))))
                // if (nIndex <= pItems[j].nLinkIndex)
                if (nIndex < pItems[j].nLinkIndex)
                {
                    continue;
                }
                if ((nIndex == pItems[j].nLinkIndex) && (i <= (pItems[j].nLinkSegIndex + 2)))
                {
                    continue;
                }

                // �̹߳�ϵ����
                if ((fabs(pntTrack.z - pPoints[pItems[j].nPntIndex].z) > 3.0) || (fabs(pntTrack.z - pPoints[pItems[j].nPntIndex + 1].z) > 3.0))
                {
                    continue;
                }

                e2 = pPoints[pItems[j].nPntIndex + 1] - pPoints[pItems[j].nPntIndex];
                e2.Normalize();

                if ((e1.DotProduct(e2) < 0.0 && direction_type) || (e1.DotProduct(e2) >= 0 && !direction_type)) // ������Ҫ��Աȷ���һ��
                {
                    continue;
                }

                dDistance = DistanceToSegment(pntTrack, pPoints[pItems[j].nPntIndex], pPoints[pItems[j].nPntIndex + 1], pntMatch);

                if (dDistance < dTolerance)
                {
                    arrPointMatch[i] = 1;
                    break;
                }
            }

            if (arrPointMatch[i] == 1)
            {
                break;
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Array<Coordinate> &Points, Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch, Double dTolerance)
{
    if (Points.GetCount() == 0)
    {
        return false;
    }

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;
    Int32 nPntCount = arrpntTrack.GetCount();
    if (nPntCount <= 1)
    {
        return false;
    }

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;

    Coordinate *pPoints = (Coordinate *)Points.Data();

    for (i = 0; i < nPntCount; i++)
    {
        Coordinate pntTrack = *(arrpntTrack[i]);
        VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
        {
            continue;
        }

        for (H = 0; H < 9; H++)
        {
            nTempRow = VertexRow + (Int32(H / 3)) - 1;
            nTempCol = VertexCol + (Int32(H % 3)) - 1;

            if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
            {
                continue;
            }

            nGridIndex = nTempRow * IndexCols + nTempCol;
            pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
            nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

            for (j = 0; j < nSegCount; j++)
            {
                Int32 ptIndex = pItems[j].nPntIndex;
                if ((pntTrack.DistanceXY(pPoints[ptIndex]) <= dTolerance) && (fabs(pntTrack.z - pPoints[ptIndex].z) < 3.0))
                {
                    arrPointMatch[i] = ptIndex;
                    break;
                }
            }

            if (arrPointMatch[i] >= 0)
            {
                break;
            }
        }
    }
    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Array<Coordinate> &Points, Array<Coordinate *> &arrpntTrack, Array<Int32> &arrPointMatch)
{
    if (Points.GetCount() == 0)
    {
        return false;
    }

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;
    Int32 nPntCount = arrpntTrack.GetCount();
    if (nPntCount <= 1)
    {
        return false;
    }

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;

    Coordinate *pPoints = (Coordinate *)Points.Data();

    for (i = 0; i < nPntCount; i++)
    {
        Coordinate pntTrack = *(arrpntTrack[i]);
        VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
        VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

        if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
        {
            continue;
        }

        for (H = 0; H < 9; H++)
        {
            nTempRow = VertexRow + (Int32(H / 3)) - 1;
            nTempCol = VertexCol + (Int32(H % 3)) - 1;

            if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
            {
                continue;
            }

            nGridIndex = nTempRow * IndexCols + nTempCol;
            pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
            nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

            for (j = 0; j < nSegCount; j++)
            {
                if (pntTrack.Distance(pPoints[pItems[j].nPntIndex]) <= Geometries_EP)
                {
                    arrPointMatch[i] = 1;
                    break;
                }
            }

            if (arrPointMatch[i] == 1)
            {
                break;
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetMatchInfos(Coordinate pntTrack, Array<Coordinate> &arrPointMatch, Array<Int32> &arrMatchIndex, Double dMaxDistance)
{
    arrMatchIndex.Clear();
    if (arrPointMatch.GetCount() == 0)
    {
        return false;
    }

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;
    Coordinate *pPoints = (Coordinate *)arrPointMatch.Data();

    VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
    VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

    if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
    {
        return false;
    }

    for (H = 0; H < 9; H++)
    {
        nTempRow = VertexRow + (Int32(H / 3)) - 1;
        nTempCol = VertexCol + (Int32(H % 3)) - 1;

        if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
        {
            continue;
        }

        nGridIndex = nTempRow * IndexCols + nTempCol;
        pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
        nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

        for (j = 0; j < nSegCount; j++)
        {
            if (pntTrack.Distance(pPoints[pItems[j].nPntIndex]) <= dMaxDistance) // �ҵ������
            {
                arrMatchIndex.Add(pItems[j].nPntIndex);
            }
        }
    }

    return true;
}

Void RoadTopoGrid::ConnectedComponent(Array<Coordinate> &arrPoints, Array<Int32> &arrMarked, Double dMaxDistance)
{
    BuildTopoGrid(arrPoints, dMaxDistance);

    Int32 nPointCount = arrPoints.GetCount();
    arrMarked.SetSize(nPointCount);
    memset(arrMarked.Data(), 0, sizeof(Int32) * nPointCount);

    Array<Int32> arrRealMarked;
    arrRealMarked.SetSize(nPointCount + 1);
    memset(arrRealMarked.Data(), 0, sizeof(Int32) * (nPointCount + 1));

    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Array<Int32> arrGridMarked;
    arrGridMarked.SetSize(IndexRows * IndexCols);
    memset(arrGridMarked.Data(), 0, sizeof(Int32) * IndexRows * IndexCols);

    Int32 i = 0, j = 0, H = 0, k = 0;
    Int32 nGridIndex = 0, nSegCount = 0;
    TGridSegment *pItems = NULL;
    Array<Int32> arrPointIndex;
    Int32 nNumberTag = 1;

    while ((k < IndexRows) || (k < IndexCols))
    {
        if (k < IndexCols)
        {
            Int32 nTempRows = k;
            if (nTempRows >= IndexRows)
            {
                nTempRows = IndexRows - 1;
            }

            for (i = 0; i <= nTempRows; i++)
            {
                nGridIndex = i * IndexCols + k;
                pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
                nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

                for (j = 0; j < nSegCount; j++)
                {
                    if (arrMarked[pItems[j].nPntIndex] == 0)
                    {
                        GetNearPoints(arrPoints, pItems[j].nPntIndex, i, k, arrGridMarked, arrPointIndex, dMaxDistance);
                        Merge(arrMarked, arrRealMarked, nNumberTag, arrPointIndex);

                        nNumberTag++;
                    }
                }

                arrGridMarked[nGridIndex] = 1;
            }
        }

        if (k < IndexRows)
        {
            Int32 nTempCols = k;
            if (nTempCols >= IndexCols)
            {
                nTempCols = IndexCols;
            }

            for (i = nTempCols - 1; i >= 0; i--)
            {
                nGridIndex = k * IndexCols + i;
                pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
                nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

                for (j = 0; j < nSegCount; j++)
                {
                    if (arrMarked[pItems[j].nPntIndex] == 0)
                    {
                        GetNearPoints(arrPoints, pItems[j].nPntIndex, k, i, arrGridMarked, arrPointIndex, dMaxDistance);
                        Merge(arrMarked, arrRealMarked, nNumberTag, arrPointIndex);

                        nNumberTag++;
                    }
                }

                arrGridMarked[nGridIndex] = 1;
            }
        }

        k++;
    }

    memset(arrGridMarked.Data(), 0, sizeof(Int32) * IndexRows * IndexCols);

    for (i = 0; i < nPointCount; i++)
    {
        Int32 temTag = arrMarked[i];
        arrMarked[i] = arrRealMarked[temTag];
    }

    i = 0, k = 0;
    while ((k < IndexRows) || (k < IndexCols))
    {
        if (k < IndexCols)
        {
            Int32 nTempRows = k;
            if (nTempRows >= IndexRows)
            {
                nTempRows = IndexRows - 1;
            }

            for (i = 0; i <= nTempRows; i++)
            {
                nGridIndex = i * IndexCols + k;

                if (!IsSameValue(i, k, arrMarked, arrGridMarked))
                {
                    Array<GridNumberTag> gridPntMarked;
                    if (GetGridSegments(arrPoints, nGridIndex, arrMarked, gridPntMarked))
                    {
                        MergeGridTag(arrPoints, i, k, arrMarked, arrRealMarked, gridPntMarked, arrGridMarked, dMaxDistance);
                    }
                }

                arrGridMarked[nGridIndex] = 1;
            }
        }

        if (k < IndexRows)
        {
            Int32 nTempCols = k;
            if (nTempCols >= IndexCols)
            {
                nTempCols = IndexCols;
            }

            for (i = nTempCols - 1; i >= 0; i--)
            {
                nGridIndex = k * IndexCols + i;

                if (!IsSameValue(k, i, arrMarked, arrGridMarked))
                {
                    Array<GridNumberTag> gridPntMarked;
                    if (GetGridSegments(arrPoints, nGridIndex, arrMarked, gridPntMarked))
                    {
                        MergeGridTag(arrPoints, k, i, arrMarked, arrRealMarked, gridPntMarked, arrGridMarked, dMaxDistance);
                    }
                }

                arrGridMarked[nGridIndex] = 1;
            }
        }
        k++;
    }

    for (i = nPointCount; i > -1; i--)
    {
        Int32 temTag = arrRealMarked[i];
        while (arrRealMarked[temTag] != temTag)
        {
            temTag = arrRealMarked[temTag];
        }
        arrRealMarked[i] = temTag;
    }

    for (i = 0; i < nPointCount; i++)
    {
        Int32 temTag = arrMarked[i];
        arrMarked[i] = arrRealMarked[temTag];
    }
}

Bool RoadTopoGrid::IsSameValue(Int32 VertexRow, Int32 VertexCol, const Array<Int32> &arrMarked, Array<Int32> &arrGridMarked)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Int32 j = 0, H = 0;
    TGridSegment *pItems = NULL;
    Int32 nSegCount;
    Int32 nValue = 0;

    for (H = 0; H < 9; H++)
    {
        Int32 nTempRow = VertexRow + (Int32(H / 3)) - 1;
        Int32 nTempCol = VertexCol + (Int32(H % 3)) - 1;

        if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
        {
            continue;
        }

        Int32 nGridIndex = nTempRow * IndexCols + nTempCol;
        if (arrGridMarked[nGridIndex] != 0)
        {
            continue;
        }

        pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
        nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

        for (j = 0; j < nSegCount; j++)
        {
            if (nValue == 0)
            {
                nValue = arrMarked[pItems[j].nPntIndex];
            }
            else if (arrMarked[pItems[j].nPntIndex] != nValue)
            {
                return false;
            }
        }
    }

    return true;
}

Bool RoadTopoGrid::GetLinesInDis(Coordinate pntTrack, double dTolerance, Array<MatchLineInfo> &arrResultInfos)
{
    Int32 IndexRows = m_topoGridEnv.IndexRows;
    Int32 IndexCols = m_topoGridEnv.IndexCols;

    Envelope rcBound = m_topoGridEnv.rcBounds;
    Double Dx = m_topoGridEnv.dGridInterval;
    Double Dy = -Dx;

    if (m_topoGridEnv.pGridUnits == NULL)
    {
        return false;
    }

    Int32 VertexRow = 0, VertexCol = 0, nTempRow = 0, nTempCol = 0;

    SizeT i = 0, j = 0;

    Int32 H = 0;
    Int32 nGridIndex = 0;
    Int32 nSegCount = 0;
    TGridSegment *pItems = NULL;

    Double dDistance = -1.0;
    Coordinate *pPoints = (Coordinate *)m_arrLinkPoints.Data();

    VertexRow = (Int32)floor((pntTrack.y - rcBound.GetMaxY()) / Dy);
    VertexCol = (Int32)floor((pntTrack.x - rcBound.GetMinX()) / Dx);

    if ((VertexRow < (-1)) || (VertexRow > IndexRows) || (VertexCol < (-1)) || (VertexCol > IndexCols))
    {
        return false;
    }

    for (H = 0; H < 9; H++)
    {
        nTempRow = VertexRow + (Int32(H / 3)) - 1;
        nTempCol = VertexCol + (Int32(H % 3)) - 1;

        if ((nTempRow < 0) || (nTempRow >= IndexRows) || (nTempCol < 0) || (nTempCol >= IndexCols))
        {
            continue;
        }

        nGridIndex = nTempRow * IndexCols + nTempCol;
        pItems = m_topoGridEnv.pGridUnits[nGridIndex].pSegments;
        nSegCount = m_topoGridEnv.pGridUnits[nGridIndex].nSegmentsCount;

        for (j = 0; j < nSegCount; j++)
        {
            // �̹߳�ϵ����
            if (((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex].z) && ((pntTrack.z + 5.5) < pPoints[pItems[j].nPntIndex + 1].z))
            {
                continue;
            }

            if ((pntTrack.z > (pPoints[pItems[j].nPntIndex].z + 5.5)) && (pntTrack.z > (pPoints[pItems[j].nPntIndex + 1].z + 5.5)))
            {
                continue;
            }

            Coordinate pntMatch;
            dDistance = DistanceToSegment(pntTrack, pPoints[pItems[j].nPntIndex], pPoints[pItems[j].nPntIndex + 1], pntMatch);

            // ���ҵ��ھ����ڵ��߽���ȥ�ش�������ͬһ������̾�����Ϣ
            if (dDistance < dTolerance)
            {
                MatchLineInfo matchLineInfo;
                matchLineInfo.nLinkIndex = pItems[j].nLinkIndex;
                matchLineInfo.sPnt = pPoints[pItems[j].nPntIndex];
                matchLineInfo.ePnt = pPoints[pItems[j].nPntIndex + 1];
                matchLineInfo.testPnt = pntTrack;
                matchLineInfo.projPoint = pntMatch;
                matchLineInfo.projectDis = dDistance;
                matchLineInfo.nSide = BaseAlgorithm::PntMatchLine(pPoints[pItems[j].nPntIndex], pPoints[pItems[j].nPntIndex + 1], pntTrack);
                arrResultInfos.Add(matchLineInfo);
            }
        }
    }
    return true;
}
