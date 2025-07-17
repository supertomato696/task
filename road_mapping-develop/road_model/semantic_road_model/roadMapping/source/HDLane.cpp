//
//
//
#include "HDLane.h"
#include "LineObj.h"
#include "Log.h"
#include "CommonUtil.h"
#include <map>
#include "proj_api.h"
#include "Geometries/Coordinate.h"
#include "Geometries/LineString.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Algorithm/Line.h"

using namespace hdmap_build;
using namespace std;
using namespace Engine::Algorithm;
using namespace Engine::Geometries;
using namespace Engine::Base;

HDLane::HDLane()
{
}
HDLane::~HDLane()
{
}

void HDLane::ConnectLineByID(std::vector<LineObj> &oriLaneMarks, std::vector<LineObj> &resLanes)
{
    if (oriLaneMarks.empty())
    {
        return;
    }

    // 将lane_edge_id作为寻找主键
    map<int64_t, LineObj> mapCHDLane;
    for (int i = 0; i < oriLaneMarks.size(); ++i)
    {
        mapCHDLane.insert(pair<int64_t, LineObj>(oriLaneMarks[i].m_lane_edge_id, oriLaneMarks[i]));
    }

    // 从前序号为空的线段开始查找，进行线连接
    std::vector<LineObj> tempLaneMarks;
    std::vector<std::vector<LineObj>> groupLanes;
    for (int i = 0; i < oriLaneMarks.size(); ++i)
    {
        if (oriLaneMarks[i].m_prev_id > 0)
        {
            continue; // 从开始线段寻找
        }
        VULCAN_LOG_INFO("start link lane_edge_id <{}> ", oriLaneMarks[i].m_lane_edge_id);
        tempLaneMarks.push_back(oriLaneMarks[i]);
        LineObj curCHDLane = oriLaneMarks[i];
        while (curCHDLane.m_next_id > 0)
        {
            auto iter = mapCHDLane.find(curCHDLane.m_next_id);
            if (iter != mapCHDLane.end())
            {
                tempLaneMarks.push_back(iter->second);
                curCHDLane = iter->second;
                VULCAN_LOG_INFO("next <{}> ", curCHDLane.m_lane_edge_id);
            }
            else
            {
                break;
            }
        }
        groupLanes.push_back(tempLaneMarks);
        tempLaneMarks.clear();
    }
    // 重新组织线形
    for (int i = 0; i < groupLanes.size(); ++i)
    {
        LineObj newCHDLane;
        newCHDLane.m_lane_edge_id = groupLanes[i][0].m_lane_edge_id;
        double totalSequence = 0;
        for (int j = 0; j < groupLanes[i].size(); ++j)
        {
            totalSequence += groupLanes[i][j].m_lane_edge_sequence;
            newCHDLane.m_vecSeq.push_back(groupLanes[i][j].m_lane_edge_sequence);
            // 插入点重复修改 【test 2021-9-9】
            for (int k = 0; k < groupLanes[i][j].m_lineCroods.GetCount(); k++)
            {
                if ((j != 0) && (k == 0))
                {
                    continue;
                }
                newCHDLane.m_lineCroods.Add(groupLanes[i][j].m_lineCroods[k]);
            }
        }
        newCHDLane.m_id = i;
        newCHDLane.m_ave_sequence = totalSequence / groupLanes[i].size();
        resLanes.push_back(newCHDLane);
    }

    std::sort(resLanes.begin(), resLanes.end(), [](const LineObj &p1, const LineObj &p2)
              { return p1.m_ave_sequence < p2.m_ave_sequence; });
}

void HDLane::GroupLeftRight(std::vector<std::vector<LineObj>> &vecVecLanes, std::vector<std::vector<GroupLane>> &vecVecGroupLanes)
{
    if (vecVecLanes.empty())
    {
        return;
    }

    // 区分左右
    for (int i = 0; i < vecVecLanes.size(); i++)
    {
        std::vector<LineObj> vecLanes = vecVecLanes[i];
        if (vecLanes.size() < 2) // 删除只有一条线的lane
        {
            continue;
        }

        std::sort(vecLanes.begin(), vecLanes.end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        std::vector<GroupLane> vecGroupLanes;
        for (int j = 0; j < vecLanes.size() - 1; j++)
        {
            GroupLane newGroupLane;
            newGroupLane.leftPoints = CommonUtil::EngineCroodToVec3(vecLanes[j].m_lineCroods);
            newGroupLane.leftPoints = CommonUtil::Utm2Wgs84(newGroupLane.leftPoints);
            newGroupLane.rightPoints = CommonUtil::EngineCroodToVec3(vecLanes[j + 1].m_lineCroods);
            newGroupLane.rightPoints = CommonUtil::Utm2Wgs84(newGroupLane.rightPoints);
            vecGroupLanes.push_back(newGroupLane);
        }
        vecVecGroupLanes.push_back(vecGroupLanes);
    }
}

void HDLane::HDLaneToLineObject(LineObj _CHDLane, LineObject &_lineobject, int i, int j)
{
    _lineobject.groupID = i;
    _lineobject.indexInGroup = j;
    _lineobject.nSide = 2; // 全部按照左侧处理
    if (_CHDLane.m_lineCroods.GetCount() < 2)
    {
        return;
    }

    Array<Coordinate> laneCoords;
    for (int k = 0; k < _CHDLane.m_lineCroods.GetCount(); k++)
    {
        // 转换坐标
        Coordinate coor = (_CHDLane.m_lineCroods)[k];
        coor.x = coor.x * DEG_TO_RAD;
        coor.y = coor.y * DEG_TO_RAD;
        projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
        projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
        pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);

        Coordinate newCoord;
        newCoord.x = coor.x;
        newCoord.y = coor.y;
        newCoord.z = coor.z;
        laneCoords.Add(newCoord);
    }
    LineString *newLine = CommonUtil::CoordsToLineString(laneCoords);
    if (newLine == nullptr)
    {
        DELETE_PTR(newLine);
        return;
    }
    _lineobject.pLine = newLine;
    Double dis = newLine->GetLength();
    _lineobject.length = dis;
    VULCAN_LOG_INFO("<{}>", dis);
    // coords to linestring
}

void HDLane::LineObjectsToHDLaneArr(Array<LineObject> &lineObjectArr, vector<vector<LineObj>> &oriTileCHDLaneVecVec, vector<vector<LineObj>> &tileCHDLaneVecVec)
{
    if (lineObjectArr.IsEmpty())
    {
        return;
    }

    sort(lineObjectArr.Begin(), lineObjectArr.End(), [&](const LineObject &p0, const LineObject &p1)
         { return p0.groupID < p1.groupID; });

    int oriGroup = lineObjectArr[0].groupID;
    vector<LineObj> temCHDLaneVec;
    LineObj oriHDline;
    LineObj newCHDLane;
    for (int i = 0; i < lineObjectArr.GetCount(); ++i)
    {
        if (lineObjectArr[i].groupID == oriGroup)
        {
            oriHDline = oriTileCHDLaneVecVec[oriGroup][lineObjectArr[i].indexInGroup];
            newCHDLane = oriHDline;
            newCHDLane.m_lineCroods.Clear();
            CommonUtil::EngineLinestringToVec3Points(lineObjectArr[i].pLine, newCHDLane.m_lineCroods);
            newCHDLane.length = lineObjectArr[i].pLine->GetLength();
            VULCAN_LOG_INFO("new length <{}>", newCHDLane.length);
            temCHDLaneVec.push_back(newCHDLane);
        }
        else
        {
            tileCHDLaneVecVec.push_back(temCHDLaneVec);
            temCHDLaneVec.clear();
            oriGroup = lineObjectArr[i].groupID;
            oriHDline = oriTileCHDLaneVecVec[oriGroup][lineObjectArr[i].indexInGroup];
            newCHDLane = oriHDline;
            newCHDLane.m_lineCroods.Clear();
            CommonUtil::EngineLinestringToVec3Points(lineObjectArr[i].pLine, newCHDLane.m_lineCroods);
            newCHDLane.length = lineObjectArr[i].pLine->GetLength();
            VULCAN_LOG_INFO("new length <{}>", newCHDLane.length);
            temCHDLaneVec.push_back(newCHDLane);
        }
    }
    if (temCHDLaneVec.size() > 0)
    {
        tileCHDLaneVecVec.push_back(temCHDLaneVec);
    }
}

LineString *HDLane::LineObjToLineString(LineObj _hdlane, bool PROJ)
{
    Array<Coordinate> arrPnts = _hdlane.m_lineCroods;
    if (arrPnts.IsEmpty() || arrPnts.GetCount() < 2)
        return nullptr;

    if (!PROJ) // 需要进行坐标转换
    {
        CommonUtil::Wgs84toUtm(arrPnts);
    }
    LineString *resLine = new LineString();
    resLine = CommonUtil::CoordsToLineString(arrPnts);

    return resLine;
}

void HDLane::RemoveShortHDLanes(vector<vector<LineObj>> &tileCHDLaneVecVec)
{
    if (tileCHDLaneVecVec.empty())
    {
        return;
    }
    for (int i = tileCHDLaneVecVec.size() - 1; i >= 0; i--)
    {
        // 对一个组内数据按距离排序
        sort(tileCHDLaneVecVec[i].begin(), tileCHDLaneVecVec[i].end(), LineObj::dCmp_Length);
        double dmaxLenPer3 = tileCHDLaneVecVec[i][0].length / 3.0;
        // Vec3 firstPoint = tileCHDLaneVecVec[i][0].m_linePnts[0];
        // int nPoits = tileCHDLaneVecVec[i][0].m_linePnts.size();
        // Vec3 endPoint = tileCHDLaneVecVec[i][0].m_linePnts[nPoits - 1];
        for (int j = tileCHDLaneVecVec[i].size() - 1; j >= 0; j--)
        {
            if (tileCHDLaneVecVec[i][j].length < dmaxLenPer3) // 总长度小于总长的1/4删除
            {
                tileCHDLaneVecVec[i].erase(tileCHDLaneVecVec[i].begin() + j);
                continue;
            }
            //            Vec3 firstPoint1 = tileCHDLaneVecVec[i][j].m_linePnts[0];
            //            if (firstPoint.dist(firstPoint1) > dmaxLenPer3) //两点的距离大于最长线的1/4删除
            //            {
            //                tileCHDLaneVecVec[i].erase(tileCHDLaneVecVec[i].begin() + j);
            //                continue;
            //            }
            //            int nPoits1 = tileCHDLaneVecVec[i][j].m_linePnts.size();
            //            Vec3 endPoint1 = tileCHDLaneVecVec[i][j].m_linePnts[nPoits1 - 1];
            //            if(endPoint.dist(endPoint1) > dmaxLenPer3)
            //            {
            //                tileCHDLaneVecVec[i].erase(tileCHDLaneVecVec[i].begin() + j);
            //                continue;
            //            }
        }
        if (tileCHDLaneVecVec[i].size() < 2)
        {
            tileCHDLaneVecVec.erase(tileCHDLaneVecVec.begin() + i);
        }
    }
}

bool HDLane::GetOutstandLane(vector<LineObj> &CHDLaneVec, LineObj &resCHDLane)
{
    for (int i = CHDLaneVec.size() - 1; i >= 0; i--)
    {
        if (CHDLaneVec[i].m_lineCroods.GetCount() < 2)
        {
            CHDLaneVec.erase(CHDLaneVec.begin() + i);
        }
    }

    int nCount = CHDLaneVec.size();
    if (nCount < 2)
    {
        resCHDLane = CHDLaneVec[0];
        return true;
    }

    bool isFind = false;
    for (int i = 0; i < nCount; ++i)
    {
        if (i + 1 == nCount && !isFind) // 最后一根线最突出
        {
            resCHDLane = CHDLaneVec[i];
            isFind = true;
            break;
        }
        Coordinate e1 = CHDLaneVec[i].m_lineCroods[0] - CHDLaneVec[i].m_lineCroods[1];
        e1.Normalize();
        for (int j = i + 1; j < nCount; ++j)
        {
            Coordinate e2 = CHDLaneVec[j].m_lineCroods[0] - CHDLaneVec[i].m_lineCroods[0];
            e2.Normalize();
            double cross = e1.DotProduct(e2);
            if (cross > 0) // 夹角是锐角，说明不是突出的线，垂点在另一条线了
            {
                isFind = false;
                break;
            }
            isFind = true;
        }
        if (isFind)
        {
            resCHDLane = CHDLaneVec[i];
            break;
        }
    }
    return isFind;
}

void HDLane::NodeAlineLaneGroup(vector<LineObj> &vecLanes)
{
    // 突出长度最长的线，即在所有线的垂点都在延长线上
    LineObj outstandCHDLane;
    if (GetOutstandLane(vecLanes, outstandCHDLane))
    {
        // 先处理最左右侧线
        string outFile = m_tileID + "_NodeAlineLeft_Before";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);
        NodeAlineLeftRight(vecLanes, outstandCHDLane, true);
        outFile = m_tileID + "_NodeAlineLeft";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);

        NodeAlineLeftRight(vecLanes, outstandCHDLane, false);
        outFile = m_tileID + "_NodeAlineRight";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);

        // 然后处理中间线
        NodeAlineMiddle(vecLanes);
        outFile = m_tileID + "_NodeAlineMiddle";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);
    }
    // 处理结尾，将所有线反转
    ReverseLane(vecLanes);
    if (GetOutstandLane(vecLanes, outstandCHDLane))
    {
        // 先处理最左右侧线
        NodeAlineLeftRight(vecLanes, outstandCHDLane, true);
        string outFile = m_tileID + "_NodeAlineLeft2";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);

        NodeAlineLeftRight(vecLanes, outstandCHDLane, false);
        outFile = m_tileID + "_NodeAlineRight2";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);

        // 然后处理中间线
        NodeAlineMiddle(vecLanes);
        outFile = m_tileID + "_NodeAlineMiddle2";
        CommonUtil::WriteToOBJ(vecLanes, m_tilePath, outFile, true);
    }

    // 将所有线反转回来
    ReverseLane(vecLanes);
}

void HDLane::NodeAlineLeftRight(vector<LineObj> &CHDLaneVec, const LineObj &longestLane, bool isLeft)
{
    LineObj dealCHDLane, compareCHDLane;
    int nCount = CHDLaneVec.size();
    if (isLeft) // 最左侧和临近一条线
    {
        dealCHDLane = CHDLaneVec[0];
    }
    else // 最右侧和临近一条线
    {
        dealCHDLane = CHDLaneVec[nCount - 1];
    }
    if (dealCHDLane.m_lineCroods.GetCount() == 10)
    {
        int n = 0;
    }
    // 最左右为最长线，用其余线中最长线判断是截取还是保留
    if (dealCHDLane.m_lane_edge_id == longestLane.m_lane_edge_id)
    {
        // 寻找其余线中最突出的线
        vector<LineObj> newCHDLaneVec = CHDLaneVec;
        if (isLeft)
        {
            newCHDLaneVec.erase(newCHDLaneVec.begin());
        }
        else
        {
            newCHDLaneVec.erase(newCHDLaneVec.end());
        }
        GetOutstandLane(newCHDLaneVec, compareCHDLane);

        // 突出值大于原长的三分之二,截断
        LineObj clipCHDLaneVec = dealCHDLane;
        ClipLane(clipCHDLaneVec, compareCHDLane);
        double diffDis = clipCHDLaneVec.length;
        double threePer = dealCHDLane.length * 2.0 / 3.0;
        cout << "diffDis:" << diffDis << endl;
        cout << "threePer:" << threePer << endl;
        if (diffDis < threePer) // 剩余长度小于原来的1/3
        {
            // 将左右侧线进行截取
            ClipLane(dealCHDLane, compareCHDLane);
        }
    }
    else // 其他线比最左右侧线长，则将左右侧线使用橡皮筋功能拉长
    {
        // 如果目前处理的是左侧，最长线为右侧，需考虑右侧线过长需要裁剪
        if (isLeft && (longestLane.m_lane_edge_id == CHDLaneVec[nCount - 1].m_lane_edge_id))
        {
            LineObj clipCHDLaneVec2 = longestLane;
            ClipLane(clipCHDLaneVec2, dealCHDLane);
            double diffDis2 = clipCHDLaneVec2.length;
            double threePer2 = longestLane.length * 2.0 / 3.0;
            if (diffDis2 < threePer2) // 剩余部分小于原来的2/3
            {
                // 右侧太长不能按照右侧拉取
                vector<LineObj> newCHDLaneVec2 = CHDLaneVec;
                newCHDLaneVec2.erase(newCHDLaneVec2.end());
                if (newCHDLaneVec2.empty()) // 只有2条线
                    return;
                GetOutstandLane(newCHDLaneVec2, compareCHDLane);
                Flexibility(dealCHDLane, compareCHDLane);         // 左侧用其余线拉取
                ClipLane(CHDLaneVec[nCount - 1], compareCHDLane); // 最右侧线用其余线截取
            }
            else
            {
                Flexibility(dealCHDLane, longestLane);
            }
        }
        else
        {
            Flexibility(dealCHDLane, longestLane);
        }
    }
    if (isLeft)
    {
        if (dealCHDLane.m_lineCroods.GetCount() > 1) // 线全部被截取，则删除
        {
            CHDLaneVec[0] = dealCHDLane;
        }
        else
        {
            CHDLaneVec.erase(CHDLaneVec.begin());
        }
    }
    else
    {
        if (dealCHDLane.m_lineCroods.GetCount() > 1) // 线全部被截取，则删除
        {
            CHDLaneVec[nCount - 1] = dealCHDLane;
        }
        else
        {
            CHDLaneVec.erase(CHDLaneVec.end());
        }
    }
}

void HDLane::NodeAlineMiddle(vector<LineObj> &CHDLaneVec)
{
    // 处理中间线
    if (CHDLaneVec.size() < 3)
    {
        return;
    }
    LineObj compareCHDLane;
    GetOutstandLane(CHDLaneVec, compareCHDLane);

    for (int i = 1; i < CHDLaneVec.size() - 1; i++)
    {
        if (compareCHDLane.m_lane_edge_id == CHDLaneVec[i].m_lane_edge_id)
        {
            continue; // 本身不再进行任何操作
        }
        Flexibility(CHDLaneVec[i], compareCHDLane);
        //        //判断与相邻车道的位置关系
        //        Coordinate pntProject;
        //        Int32 nSegIndex;
        //        double dis1 = BaseAlgorithm3D::GetDistancePointToLinesegments(CHDLaneVec[i].m_lineCroods[0], CHDLaneVec[i - 1].m_lineCroods, pntProject, nSegIndex);
        //        double dis2 = BaseAlgorithm3D::GetDistancePointToLinesegments(CHDLaneVec[i].m_lineCroods[0], CHDLaneVec[i + 1].m_lineCroods, pntProject, nSegIndex);
        //
        //        if(dis1 < 3.0 || dis2 < 3.0) //存在小于一车道的情况即需要打断
        //        {
        //            continue;
        //        }
        //        else //需要将线拉长
        //        {
        //            Flexibility(CHDLaneVec[i], compareCHDLane);
        //        }
    }
}

void HDLane::NodeAlineBreak(vector<LineObj> &CHDLaneVec)
{
    return;
}

void HDLane::NodeAline(vector<vector<LineObj>> &tileCHDLaneVecVec)
{
    for (int i = tileCHDLaneVecVec.size() - 1; i >= 0; i--)
    {
        // 排序
        std::sort(tileCHDLaneVecVec[i].begin(), tileCHDLaneVecVec[i].end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        NodeAlineLaneGroup(tileCHDLaneVecVec[i]);
    }
}

void HDLane::ClipLane(LineObj &dealCHDLane, const LineObj &compareCHDLane)
{
    if (dealCHDLane.m_lane_edge_id == compareCHDLane.m_lane_edge_id)
    {
        return;
    }

    if (dealCHDLane.m_lineCroods.IsEmpty() || compareCHDLane.m_lineCroods.IsEmpty())
    {
        return;
    }
    Coordinate pntProject;
    Int32 nSegIndex = -1;
    BaseAlgorithm3D::GetDistancePointToLinesegments(compareCHDLane.m_lineCroods[0], dealCHDLane.m_lineCroods, pntProject, nSegIndex);

    for (int i = nSegIndex; i >= 0; i--)
    {
        dealCHDLane.m_lineCroods.Delete(i);
    }
    if (dealCHDLane.m_lineCroods[0].DistanceXY(pntProject) > 0.01)
    {
        dealCHDLane.m_lineCroods.InsertAt(0, pntProject);
    }
    dealCHDLane.length = CommonUtil::GetLength(dealCHDLane.m_lineCroods); // 更新长度
}

bool HDLane::Flexibility(LineObj &dealCHDLane, const LineObj &compareCHDLane)
{
    if (dealCHDLane.m_lane_edge_id == compareCHDLane.m_lane_edge_id)
    {
        return false;
    }
    Int32 nSegIndex = 0;
    Coordinate pntProject;
    Int32 i = 0;

    BaseAlgorithm3D::GetDistancePointToLinesegments(dealCHDLane.m_lineCroods[0], compareCHDLane.m_lineCroods, pntProject, nSegIndex);

    // 距离小于5cm不动
    double dis = compareCHDLane.m_lineCroods[0].DistanceXY(pntProject);
    if (dis < 0.05)
        return false;
    // 计算最终拉长的终点
    Coordinate ePnt, pntTempEnd, eRemove;
    Double eRemoveDis = dealCHDLane.m_lineCroods[0].DistanceXY(pntProject);

    ePnt = dealCHDLane.m_lineCroods[0] - pntProject;
    ePnt.z = 0;
    ePnt.Normalize();
    pntTempEnd = compareCHDLane.m_lineCroods[0] + ePnt * eRemoveDis;

    // 计算参加橡皮筋计算的点，并将形状移动到起点和终点
    Array<Coordinate> arrClipCoordinates;
    for (i = 0; i <= nSegIndex; i++)
    {
        arrClipCoordinates.Add(compareCHDLane.m_lineCroods[i]);
    }
    arrClipCoordinates.Add(pntProject);

    BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, 0, pntTempEnd);

    Int32 nEndIndex = arrClipCoordinates.GetCount() - 1;
    BaseAlgorithm3D::Flexibility3D(arrClipCoordinates, nEndIndex, dealCHDLane.m_lineCroods[0]);

    // 将移动后的点添加到原来的线中(终点与原始起点重复，不再添加)
    for (int i = nEndIndex - 1; i >= 0; i--)
    {
        dealCHDLane.m_lineCroods.InsertAt(0, arrClipCoordinates[i]);
    }
    dealCHDLane.length = CommonUtil::GetLength(dealCHDLane.m_lineCroods); // 更新长度
    return true;
}

void HDLane::ReverseLane(vector<LineObj> &CHDLaneVec)
{
    for (int i = 0; i < CHDLaneVec.size(); i++)
    {
        CHDLaneVec[i].m_lineCroods.Reverse();
    }
}

void HDLane::ResampleCHDLane(vector<vector<LineObj>> &tileHDLaneVecVec, double tolerance)
{
    for (int i = 0; i < tileHDLaneVecVec.size(); ++i)
    {
        for (int j = 0; j < tileHDLaneVecVec[i].size(); ++j)
        {
            CommonUtil::RemoveDuplicatePoints(tileHDLaneVecVec[i][j].m_lineCroods, 1.0); // 1米为去除重复点数据
            CommonUtil::Resample(tileHDLaneVecVec[i][j].m_lineCroods, tolerance);
        }
    }
}

void HDLane::SmoothSTurnCHDLane(vector<vector<LineObj>> &tileHDLaneVecVec)
{
    for (int i = 0; i < tileHDLaneVecVec.size(); ++i)
    {
        for (int j = 0; j < tileHDLaneVecVec[i].size(); ++j)
        {
            // LineAlgorithm::SmoothSTurnSegments(tileHDLaneVecVec[i][j].m_lineCroods, s_turn_segment_max_length, s_turn_angle_degree, chord_height);
            RoadTopoBuild::SmoothSTurnSegments(tileHDLaneVecVec[i][j].m_lineCroods);
        }
    }
}

void HDLane::ClipByYellow(Array<Coordinate> &yellowPnts, vector<vector<LineObj>> &tileHDLaneVecVec)
{
    if (yellowPnts.IsEmpty() || tileHDLaneVecVec.empty())
        return;

    // 对黄色点建立网格(网格长宽均为5米)
    RoadTopoGrid PntsGrid;
    PntsGrid.BuildTopoGrid(yellowPnts, 5.0);
    Array<Int32> arrMatchInfos;

    for (int i = tileHDLaneVecVec.size() - 1; i >= 0; i--)
    {
        if (tileHDLaneVecVec[i].empty())
        {
            tileHDLaneVecVec.erase(tileHDLaneVecVec.begin() + i);
        }

        if (tileHDLaneVecVec[i].size() < 3) // 只有最左右侧线不需处理
            continue;

        // 从左向右排序
        std::sort(tileHDLaneVecVec[i].begin(), tileHDLaneVecVec[i].end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        // 从右向左匹配 遇到第一根黄线的时候进行删除操作(最左右侧数据不参加计算)
        int yellowIndex = -1;
        for (int j = tileHDLaneVecVec[i].size() - 2; j > 0; j--)
        {
            LineObj curLineObj = tileHDLaneVecVec[i][j];
            int nPointCount = curLineObj.m_lineCroods.GetCount();
            Array<Coordinate *> linePntsArr;
            for (int k = 0; k < nPointCount; k++)
            {
                Coordinate *newCoord = new Coordinate(curLineObj.m_lineCroods[k]);
                linePntsArr.Add(newCoord);
            }

            // 进行匹配(匹配阈值1.0米，匹配成功返回匹配点序号，匹配失败返回-1)
            arrMatchInfos.Clear();
            arrMatchInfos.SetSize(nPointCount);
            memset(arrMatchInfos.Data(), -1, sizeof(Int32) * nPointCount);

            PntsGrid.GetMatchInfos(yellowPnts, linePntsArr, arrMatchInfos, 1.0);

            for (int k = linePntsArr.GetCount() - 1; k >= 0; k--) // 销毁点
            {
                linePntsArr[k] = nullptr;
                linePntsArr.Delete(k);
            }
            // 统计匹配信息
            double mathCount = 0;
            for (int k = 0; k < arrMatchInfos.GetCount(); k++)
            {
                if (arrMatchInfos[k] >= 0)
                    mathCount += 1.0;
            }

            // 计算匹配到的点比例 大于70%标记为黄线[通过测试改为60%更合理，大规模测试后改为40% yxx 2021-10-22]
            double matchPer = mathCount / nPointCount;
            if (matchPer > 0.4)
            {
                yellowIndex = j;
                VULCAN_LOG_INFO("黄色标线查找成功，第<{}>组，从左算第<{}>根.......", i, j);
                break;
            }
        }

        if (yellowIndex > 0) // 找到黄线，进行分割操作(删除黄线左侧的数据)
        {
            for (int k = yellowIndex - 1; k >= 0; k--)
            {
                tileHDLaneVecVec[i].erase(tileHDLaneVecVec[i].begin() + k);
            }
        }
    }
}

void HDLane::ConnectHDLanes(vector<vector<LineObj>> &tileHDLaneVecVec)
{
    if (tileHDLaneVecVec.empty())
        return;
    for (int i = tileHDLaneVecVec.size() - 1; i >= 0; i--)
    {
        if (tileHDLaneVecVec[i].size() < 2)
        {
            tileHDLaneVecVec.erase(tileHDLaneVecVec.begin() + i);
        }

        // 排序
        std::sort(tileHDLaneVecVec[i].begin(), tileHDLaneVecVec[i].end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        int nLinesCount = tileHDLaneVecVec[i].size();
        for (int j = 0; j < nLinesCount - 1; ++j)
        {
            // 判断是否进行连接
            bool isFirst = IsConnect(tileHDLaneVecVec[i][j].m_lineCroods, tileHDLaneVecVec[i][j + 1].m_lineCroods);
            bool isSecond = IsConnect(tileHDLaneVecVec[i][j + 1].m_lineCroods, tileHDLaneVecVec[i][j].m_lineCroods);
            if (!isFirst && !isSecond) // j 和 j+1 不能连接
                continue;
            // 进行连接操作
            if (isFirst) // 把j+1的点添加到j中
            {
                for (int k = 0; k < tileHDLaneVecVec[i][j + 1].m_lineCroods.GetCount(); k++)
                {
                    tileHDLaneVecVec[i][j].m_lineCroods.Add(tileHDLaneVecVec[i][j + 1].m_lineCroods[k]);
                }
                tileHDLaneVecVec[i].erase(tileHDLaneVecVec[i].begin() + j + 1);
                j--;
            }
            else
            {
                for (int k = 0; k < tileHDLaneVecVec[i][j].m_lineCroods.GetCount(); k++)
                {
                    tileHDLaneVecVec[i][j + 1].m_lineCroods.Add(tileHDLaneVecVec[i][j].m_lineCroods[k]);
                }
                tileHDLaneVecVec[i].erase(tileHDLaneVecVec[i].begin() + j);
                j--;
            }
        }

        if (tileHDLaneVecVec[i].size() < 2)
        {
            tileHDLaneVecVec.erase(tileHDLaneVecVec.begin() + i);
        }
    }
}

void HDLane::WriteToOBJ(GroupLane &vecLanes, std::string filePath, std::string frameid, bool utm)
{
    string dirPath = filePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    string objFullPath = dirPath + frameid + ".obj";

    ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        cout << "obj 保存不成功" << endl;
        return;
    }

    vector<vector<Vec3>> lineVec;
    vector<Vec3> curVec;

    for (auto j = 0; j < vecLanes.leftPoints.size(); j++)
    {
        Vec3 coor = (vecLanes.leftPoints)[j];
        // VULCAN_LOG_INFO("ori coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);

        if (!utm)
        {
            coor.x = coor.x * DEG_TO_RAD;
            coor.y = coor.y * DEG_TO_RAD;

            projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
            projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
            pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);
            // VULCAN_LOG_INFO("trans coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
        }
        ofs << std::fixed;
        ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
        curVec.push_back(coor);
    }
    lineVec.push_back(curVec);

    curVec.clear();
    for (auto j = 0; j < vecLanes.rightPoints.size(); j++)
    {
        Vec3 coor = (vecLanes.rightPoints)[j];
        // VULCAN_LOG_INFO("ori coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);

        if (!utm)
        {
            coor.x = coor.x * DEG_TO_RAD;
            coor.y = coor.y * DEG_TO_RAD;

            projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
            projPJ g_utm = pj_init_plus("+proj=utm +zone=50N +ellps=WGS84 +no_defs");
            pj_transform(g_pWGS84, g_utm, 1, 1, &coor.x, &coor.y, &coor.z);
            // VULCAN_LOG_INFO("trans coordinate: <{}> <{}> <{}>......", coor.x, coor.y, coor.z);
        }
        ofs << std::fixed;
        ofs << "v " << coor.x << " " << coor.y << " " << coor.z << endl;
        curVec.push_back(coor);
    }
    lineVec.push_back(curVec);

    int K = 0;
    for (auto i = 0; i < lineVec.size(); i++)
    {
        if (lineVec[i].size() <= 1)
        {
            continue;
        }
        ofs << "l ";
        for (auto j = 0; j < lineVec[i].size(); j++)
        {
            ofs << ++K << " ";
        }
        ofs << endl;
    }

    ofs.close();
    return;
}

void HDLane::WriteToOBJ(std::vector<GroupLane> &vecLanes, std::string strTilePath, std::string frameid, bool utm)
{
    string dirPath = strTilePath;
    if (dirPath.back() != '/' || dirPath.back() != '\\')
    {
        dirPath += "/";
    }

    for (int i = 0; i < vecLanes.size(); ++i)
    {

        string name = frameid + "_" + to_string(i);
        WriteToOBJ(vecLanes[i], strTilePath, name, utm);
    }
}

bool HDLane::IsConnect(const Array<Coordinate> &firstLine, const Array<Coordinate> &compareLine)
{
    if (firstLine.GetCount() < 2 || compareLine.GetCount() < 2)
        return false;

    // 用第一条线的最后2点 与比较线的起点进行计算
    Coordinate e1 = firstLine[firstLine.GetCount() - 1] - firstLine[firstLine.GetCount() - 2];
    e1.Normalize();
    e1.z = 0.0;

    Coordinate e2 = compareLine[0] - firstLine[firstLine.GetCount() - 1];
    e2.Normalize();
    e2.z = 0.0;

    double dot = e1.DotProduct(e2);
    if (dot > 0.99863) // 偏航角度设置为5度 cos(3°) = 0.99863
        return true;
    return false;
}

void HDLane::GroupPoints(std::vector<std::vector<LineObj>> &vecVecLanes, std::vector<GroupLane> &groupLanes)
{
    if (vecVecLanes.empty())
    {
        return;
    }

    // 区分左右
    for (int i = 0; i < vecVecLanes.size(); i++)
    {
        std::vector<LineObj> vecLanes = vecVecLanes[i];

        std::sort(vecLanes.begin(), vecLanes.end(), [](const LineObj &p1, const LineObj &p2)
                  { return p1.m_ave_sequence < p2.m_ave_sequence; });

        for (int j = 0; j < vecLanes.size(); j++)
        {
            GroupLane newGroupLane;
            newGroupLane.leftPoints = CommonUtil::EngineCroodToVec3(vecLanes[j].m_lineCroods);
            groupLanes.push_back(newGroupLane);
        }
    }
}
