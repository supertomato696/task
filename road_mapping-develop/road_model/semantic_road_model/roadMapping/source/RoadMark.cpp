//
//
//
#include "RoadMark.h"
#include "DataReader.h"
#include "CommonUtil.h"
#include "RoadLayer.h"
#include "CloudAlgorithm.h"
#include "DataManager.h"
#include "Log.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"

#include "../data-access-engine/proxy/lane_proxy.h"
#include "../data-access-engine/proxy/position_proxy.h"

using namespace std;
using namespace hdmap_build;

RoadMark::RoadMark()
{
}

RoadMark::~RoadMark()
{
}

void RoadMark::InitParameter(string strTilePath, string road_branch, int tile_id)
{
    m_BasePath = strTilePath;
    m_TileId = tile_id;
    m_UpLoadBranch = road_branch;
}

void RoadMark::ExtractPackages(Array<Array<Coordinate>> &arrArrPackPoints)
{
    // 1.读取hd_map文件下json文件中的HD_ROADMARK.json
    Int32 nSize = 0;
    Array<Coordinate> roadMarkPnts;
    DataReader::LoadAllRoadMarkJson(m_BasePath, to_string(m_TileId), roadMarkPnts, nSize);

    if (roadMarkPnts.IsEmpty())
        return;

    // 2.将所有点输出为UTM坐标
    CommonUtil::Wgs84toUtm(roadMarkPnts);

    // 方便计算，将所有点进行原点偏移
    CommonUtil::TransGet(roadMarkPnts, m_offset);
    string name = to_string(m_TileId) + "_RoadMark_pts_BeforeFilte";
    CommonUtil::WriteToTxt(roadMarkPnts, m_BasePath, name, true);

    // 3.针对输入点进行密度过滤，去除噪点
    CommonUtil::FilterPtByRadius(roadMarkPnts, 0.2, 50, true);

    name = to_string(m_TileId) + "_RoadMark_pts_AfterFilte";
    CommonUtil::WriteToTxt(roadMarkPnts, m_BasePath, name, true);

    // 4.将所有点进行网格聚类，聚类距离0.5米
    Array<Array<Int32>> arrArrIndexs;
    RoadTopoBuild::ConnectedComponent(roadMarkPnts, arrArrIndexs, 0.5);
    Array<Array<Coordinate>> arrArrResultSegments;
    arrArrResultSegments.SetSize(arrArrIndexs.GetCount());

    Int32 i, j;
    for (i = 0; i < arrArrIndexs.GetCount(); i++)
    {
        for (j = 0; j < arrArrIndexs[i].GetCount(); j++)
        {
            Int32 coordIndex = arrArrIndexs[i].GetAt(j);
            arrArrResultSegments[i].Add(roadMarkPnts[coordIndex]);
        }
        //        string name0 = "Segments" + to_string(i);
        //        CommonUtil::WriteToTxt(arrArrResultSegments[i], m_BasePath, name0, true);
    }

    // 5.重新求每个聚类的外包络点集
    for (int i = 0; i < arrArrResultSegments.GetCount(); i++)
    {
        double count = roadMarkPnts.GetCount() / nSize;
        double average = 0.7 * count;
        if (arrArrResultSegments[i].GetCount() < average)
            continue;

        Array<Coordinate> pntsResult;
        BaseAlgorithm::PointsPackage(arrArrResultSegments[i], pntsResult);

        Int32 nResultCount = pntsResult.GetCount();
        if (nResultCount > 3 && pntsResult[0].Equals(pntsResult[nResultCount - 1]))
        {
            arrArrPackPoints.Add(pntsResult);
        }
    }
    string namePack = to_string(m_TileId) + "_RoadMarkPackages";
    CommonUtil::WriteToOBJ(arrArrPackPoints, m_BasePath, namePack, true);

    // 将点坐标进行恢复
    CommonUtil::TransPlus(m_offset, arrArrPackPoints);
}

void RoadMark::BindRoadMarkToLane(Array<Array<Coordinate>> &arrArrPackPoints, data_access_engine::TileInfoList &corrtiles)
{
    //***思路：获取tile内所有lane，并根据外包络点集搜素最近的lane，求取外包框，
    // 包装positionproxy放入lane的第一个lanesection中**//

    if (arrArrPackPoints.IsEmpty())
        return;

    // 获取tile内所有laneSection以及LineStrings
    RoadLayer readRoadLayer;
    std::vector<std::shared_ptr<data_access_engine::LaneSectionProxy>> LaneSections;
    Array<LineString *> laneLines;
    readRoadLayer.GetLeftLineStringsInTiles(corrtiles, LaneSections, laneLines);

    if (laneLines.IsEmpty() || LaneSections.empty())
        return;

    // 根据外包络点集，先拟合原始轴线，然后根据最近的标线调整轴线方向，删除长小于3米，宽小于0.2米的RoadMark
    Array<RoadMarkObj> arrRoadMarkObj;
    GetRoadMarkObjs(laneLines, arrArrPackPoints, arrRoadMarkObj);
    if (arrRoadMarkObj.IsEmpty())
        return;

    Array<Array<Coordinate>> arrArrPolygonPoints;
    for (int i = 0; i < arrRoadMarkObj.GetCount(); i++)
    {
        arrArrPolygonPoints.Add(arrRoadMarkObj[i].plygonPnts);
    }
    string name = to_string(m_TileId) + "_RoadMarkPolygons";
    CommonUtil::WriteToOBJ(arrArrPolygonPoints, m_BasePath, name, true);

    std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> objs;
    for (int i = 0; i < arrRoadMarkObj.GetCount(); i++)
    {
        // 转换坐标为经纬度
        vector<Vec3> vecPolygonPts = CommonUtil::EngineCroodToVec3(arrRoadMarkObj[i].plygonPnts);
        vector<Vec3> blPolygonPts = CommonUtil::Utm2Wgs84(vecPolygonPts);
        data_access_engine::TileInfoPtr tile;
        std::shared_ptr<data_access_engine::PositionObjectProxy> newPositionProxy = createObject(blPolygonPts, tile);
        objs.push_back(newPositionProxy);

        // 根据LaneSection id获取lane
        if (arrRoadMarkObj[i].nearstLineIndex >= 0)
        {
            auto LaneSection = LaneSections[arrRoadMarkObj[i].nearstLineIndex];
            auto id = newPositionProxy->id();
            LaneSection->mutable_objects()->add()->set(id);
            LaneSection->mark_changed();
        }
        corrtiles.push_back(tile);
    }
}

void RoadMark::WriteRoadMarkToObj(const data_access_engine::TileInfoList &tiles, string path, string name)
{
    for (auto tile : tiles)
    {
        int k = 0;
        for (auto obj : tile->position_objects())
        {
            Array<Coordinate> plyPts;
            int n = 0;
            if (obj->type() == 8)
                n = obj->pole()->pts().size();
            else
                n = obj->border()->pts().size();
            for (int j = 0; j < n; ++j)
            {
                Coordinate curCrood;
                if (obj->type() == 8)
                {
                    curCrood.x = obj->pole()->pts().at(j)->x();
                    curCrood.y = obj->pole()->pts().at(j)->y();
                    curCrood.z = obj->pole()->pts().at(j)->z();
                }
                else
                {
                    curCrood.x = obj->border()->pts().at(j)->x();
                    curCrood.y = obj->border()->pts().at(j)->y();
                    curCrood.z = obj->border()->pts().at(j)->z();
                }

                plyPts.Add(curCrood);
            }
            CommonUtil::Wgs84toUtm(plyPts);
            Array<Array<Coordinate>> arrArrplyPts;
            arrArrplyPts.Add(plyPts);

            string objName = to_string(tile->tile_id()) + "_" + name + "_Position_" + to_string(k);
            CommonUtil::WriteToOBJ(arrArrplyPts, path, objName);
            k++;
        }
    }
}

void RoadMark::GetNearestLankMark(const Array<LineString *> &arrLaneMarks, Array<MatchLineInfo> &arrMatchLines, NearestLankMark &resLankMark)
{
    resLankMark.leftLineIndex = -1;
    resLankMark.leftDis = -1;

    resLankMark.rightLineIndex = -1;
    resLankMark.rightDis = -1;

    if (arrMatchLines.IsEmpty())
    {
        return;
    }

    std::sort(arrMatchLines.Begin(), arrMatchLines.End(), [](const MatchLineInfo &objA, const MatchLineInfo &objB)
              { return objA.nLinkIndex < objB.nLinkIndex; });

    Array<MatchLineInfo> tempLines;
    Array<Array<MatchLineInfo>> arrArrLines;
    Int32 i;

    Int32 oriIndex = arrMatchLines[0].nLinkIndex;
    for (i = 0; i < arrMatchLines.GetCount(); i++)
    {
        if (arrMatchLines[i].nLinkIndex == oriIndex)
        {
            tempLines.Add(arrMatchLines[i]);
        }
        else
        {
            if (tempLines.GetCount() > 0)
            {
                arrArrLines.Add(tempLines);
                tempLines.Clear();
            }
            tempLines.Add(arrMatchLines[i]);
            oriIndex = arrMatchLines[i].nLinkIndex;
        }
    }

    if (tempLines.GetCount() > 0)
    {
        arrArrLines.Add(tempLines);
        tempLines.Clear();
    }

    Array<MatchLineInfo> arrAdjustLines;
    for (i = 0; i < arrArrLines.GetCount(); i++)
    {
        Array<MatchLineInfo> &compareLines = arrArrLines[i];
        if (compareLines.IsEmpty())
        {
            continue;
        }

        std::sort(compareLines.Begin(), compareLines.End(), [](const MatchLineInfo &obj1, const MatchLineInfo &obj2)
                  { return obj1.projectDis < obj2.projectDis; });

        arrAdjustLines.Add(compareLines[0]);
    }

    std::sort(arrAdjustLines.Begin(), arrAdjustLines.End(), [](const MatchLineInfo &obj1, const MatchLineInfo &obj2)
              { return obj1.projectDis < obj2.projectDis; });

    for (i = 0; i < arrAdjustLines.GetCount(); i++)
    {
        MatchLineInfo curMatchLineInfo = arrAdjustLines[i];

        if ((resLankMark.leftLineIndex == -1) && (curMatchLineInfo.nSide == 2))
        {
            resLankMark.leftLineIndex = curMatchLineInfo.nLinkIndex;
            resLankMark.sPtLeft = curMatchLineInfo.sPnt;
            resLankMark.ePtLeft = curMatchLineInfo.ePnt;
            resLankMark.leftDis = curMatchLineInfo.projectDis;
        }
        else if ((resLankMark.rightLineIndex == -1) && (curMatchLineInfo.nSide == 1))
        {
            resLankMark.rightLineIndex = curMatchLineInfo.nLinkIndex;
            resLankMark.sPtRight = curMatchLineInfo.sPnt;
            resLankMark.ePtRight = curMatchLineInfo.ePnt;
            resLankMark.rightDis = curMatchLineInfo.projectDis;
        }
    }
}

bool RoadMark::CalOriAxis(const Array<Coordinate> &arrCloudPts, TLineTwoPoints &resAxis, double &_dAvageThinness)
{
    if (arrCloudPts.IsEmpty())
        return false;

    Array<Coordinate> projPnts = arrCloudPts;
    // 进行线性拟合得到轴线
    return CCloudToGeoLine::ExactDashLine(projPnts, resAxis, _dAvageThinness);
}

void RoadMark::GetRoadMarkObjs(const Array<LineString *> &arrLaneMarks, Array<Array<Coordinate>> &arrArrPackPoints, Array<RoadMarkObj> &arrRoadMarkObj)
{
    arrRoadMarkObj.Clear();
    int n = arrArrPackPoints.GetCount() - 1;

    for (int i = n; i >= 0; i--)
    {
        TLineTwoPoints resAxis;
        double dAvageThinness = 0.0;
        if (CalOriAxis(arrArrPackPoints[i], resAxis, dAvageThinness)) // 轴线计算成功
        {
            // 长度和宽度满足阈值
            double length = resAxis.ptStart.DistanceXY(resAxis.ptEnd);
            if (length > 2.0 && dAvageThinness > 0.1) // 长度大于3米，宽度大于10cm
            {
                RoadMarkObj newRoadMarkObj;
                newRoadMarkObj.arrPackPnts = arrArrPackPoints[i];
                newRoadMarkObj.centerPt = CommonUtil::GetAveragePt(arrArrPackPoints[i]);
                newRoadMarkObj.axisTwoPts = resAxis;
                arrRoadMarkObj.Add(newRoadMarkObj);
            }
            else
            {
                arrArrPackPoints.Delete(i);
            }
        }
        else
        {
            arrArrPackPoints.Delete(i);
        }
    }
    if (arrLaneMarks.IsEmpty() || arrRoadMarkObj.IsEmpty())
        return;

    // 查找最近的linestring，并计算矩形
    RoadTopoGrid lankMarkGrid;
    lankMarkGrid.BuildTopoGrid(arrLaneMarks, 5.0);

    for (int i = 0; i < arrRoadMarkObj.GetCount(); i++)
    {

        Array<Coordinate> arrPackagesPnts = arrRoadMarkObj[i].arrPackPnts;
        Coordinate centerPt = arrRoadMarkObj[i].centerPt;

        // 根据中心点查找距离7米范围内的线(留出2个车道的距离)
        Array<MatchLineInfo> arrMatchLines;
        lankMarkGrid.GetLinesInDis(centerPt, 7.0, arrMatchLines);

        // 比较得出距离最近的左右侧线
        NearestLankMark resLankMark;
        GetNearestLankMark(arrLaneMarks, arrMatchLines, resLankMark);

        // 计算轴线
        CalculaAxis(centerPt, arrPackagesPnts, arrLaneMarks, resLankMark, arrRoadMarkObj[i].axisTwoPts, arrRoadMarkObj[i].nearstLineIndex);

        // 计算矩形
        GetConerPnts(arrPackagesPnts, arrRoadMarkObj[i].axisTwoPts, arrRoadMarkObj[i].plygonPnts);

        if (arrRoadMarkObj[i].plygonPnts.IsEmpty())
            continue;

        arrRoadMarkObj[i].axisTwoPts.ptStart = (arrRoadMarkObj[i].plygonPnts[0] + arrRoadMarkObj[i].plygonPnts[1]) / 2.0;
        arrRoadMarkObj[i].axisTwoPts.ptEnd = (arrRoadMarkObj[i].plygonPnts[2] + arrRoadMarkObj[i].plygonPnts[3]) / 2.0;
    }
}

void RoadMark::CalculaAxis(Coordinate centerPt, Array<Coordinate> &arrCloudPts, const Array<LineString *> &arrLaneMarks, NearestLankMark resLankMark, TLineTwoPoints &resAxis, int &nearstLine)
{
    nearstLine = -1;
    if (arrCloudPts.IsEmpty())
    {
        return;
    }

    TLineTwoPoints temAxis;

    if (resLankMark.leftLineIndex != -1)
    {
        temAxis.ptStart = resLankMark.sPtLeft;
        temAxis.ptEnd = resLankMark.ePtLeft;
        nearstLine = resLankMark.leftLineIndex;
    }
    else if (resLankMark.rightLineIndex != -1)
    {
        temAxis.ptStart = resLankMark.sPtRight;
        temAxis.ptEnd = resLankMark.ePtRight;
        nearstLine = resLankMark.rightLineIndex;
    }
    else
    {
        temAxis = resAxis;
    }

    Coordinate projectPnt, sPnt;
    BaseAlgorithm3D::GetProjectpntToLine(centerPt, temAxis.ptStart, temAxis.ptEnd, projectPnt);

    Double removeDis = centerPt.Distance(projectPnt);
    sPnt = centerPt - projectPnt;
    sPnt.Normalize();

    temAxis.ptStart = temAxis.ptStart + sPnt * removeDis;
    temAxis.ptEnd = temAxis.ptEnd + sPnt * removeDis;

    Double dLength = temAxis.ptStart.Distance(temAxis.ptEnd);
    Double radio = 20.0 / dLength;
    Coordinate newEPnt = (temAxis.ptEnd - temAxis.ptStart) * radio + temAxis.ptEnd;
    Coordinate newSPnt = (temAxis.ptStart - temAxis.ptEnd) * radio + temAxis.ptStart;

    resAxis.ptStart = newSPnt;
    resAxis.ptEnd = newEPnt;
}

void RoadMark::GetConerPnts(const Array<Coordinate> &arrCloudPts, TLineTwoPoints resAxis, Array<Coordinate> &conerPnts)
{
    conerPnts.Clear();
    Int32 nCount = arrCloudPts.GetCount();
    if (nCount < 3)
    {
        return;
    }

    Array<PointToLineInfo> arrPointInfoSort;
    CalPtsToLineInfo(arrCloudPts, resAxis, arrPointInfoSort);
    std::sort(arrPointInfoSort.Begin(), arrPointInfoSort.End(), [](const PointToLineInfo &objA, const PointToLineInfo &objB)
              { return objA.dMeasure < objB.dMeasure; });

    Coordinate dMeasureMinPnt = arrPointInfoSort[0].projectPnt;
    Coordinate dMeasureMaxPnt = arrPointInfoSort[nCount - 1].projectPnt;

    Array<PointToLineInfo> arrLeftInfoSort;
    Array<PointToLineInfo> arrRightInfoSort;
    for (Int32 i = 0; i < nCount; i++)
    {
        Int32 nSide = arrPointInfoSort[i].nSide;
        if (nSide == 1)
        {
            arrLeftInfoSort.Add(arrPointInfoSort[i]);
        }
        else if (nSide == 2)
        {
            arrRightInfoSort.Add(arrPointInfoSort[i]);
        }
    }

    if (arrLeftInfoSort.IsEmpty() || arrRightInfoSort.IsEmpty())
    {
        return;
    }

    std::sort(arrLeftInfoSort.Begin(), arrLeftInfoSort.End(), [](const PointToLineInfo &objA, const PointToLineInfo &objB)
              { return objA.dDistance > objB.dDistance; });

    std::sort(arrRightInfoSort.Begin(), arrRightInfoSort.End(), [](const PointToLineInfo &objA, const PointToLineInfo &objB)
              { return objA.dDistance > objB.dDistance; });

    Coordinate dDistanceLeftPnt = arrCloudPts[(arrLeftInfoSort[0].index)];
    Coordinate dDistanceRightPnt = arrCloudPts[(arrRightInfoSort[0].index)];

    TLineTwoPoints lowerAxis;
    Coordinate *oriPnt = new Coordinate(resAxis.ptStart);
    Coordinate *pPointOrg = new Coordinate(dMeasureMinPnt);

    BaseAlgorithm::RotatePoint(pPointOrg, 1.57, oriPnt);
    lowerAxis.ptStart.x = oriPnt->x;
    lowerAxis.ptStart.y = oriPnt->y;
    lowerAxis.ptStart.z = resAxis.ptStart.z;

    oriPnt = new Coordinate(resAxis.ptEnd);

    BaseAlgorithm::RotatePoint(pPointOrg, 1.57, oriPnt);
    lowerAxis.ptEnd.x = oriPnt->x;
    lowerAxis.ptEnd.y = oriPnt->y;
    lowerAxis.ptEnd.z = resAxis.ptEnd.z;

    TLineTwoPoints higherAxis;
    oriPnt = new Coordinate(resAxis.ptStart);
    pPointOrg = new Coordinate(dMeasureMaxPnt);

    BaseAlgorithm::RotatePoint(pPointOrg, 1.57, oriPnt);
    higherAxis.ptStart.x = oriPnt->x;
    higherAxis.ptStart.y = oriPnt->y;
    higherAxis.ptStart.z = resAxis.ptStart.z;

    oriPnt = new Coordinate(resAxis.ptEnd);
    BaseAlgorithm::RotatePoint(pPointOrg, 1.57, oriPnt);
    higherAxis.ptEnd.x = oriPnt->x;
    higherAxis.ptEnd.y = oriPnt->y;
    higherAxis.ptEnd.z = resAxis.ptEnd.z;

    oriPnt = NULL;
    DELETE_PTR(oriPnt);

    pPointOrg = NULL;
    DELETE_PTR(pPointOrg);

    Coordinate conerPt0, conerPt1, conerPt2, conerPt3;
    BaseAlgorithm3D::GetProjectpntToLine(dDistanceLeftPnt, lowerAxis.ptStart, lowerAxis.ptEnd, conerPt0);
    BaseAlgorithm3D::GetProjectpntToLine(dDistanceRightPnt, lowerAxis.ptStart, lowerAxis.ptEnd, conerPt1);
    BaseAlgorithm3D::GetProjectpntToLine(dDistanceRightPnt, higherAxis.ptStart, higherAxis.ptEnd, conerPt2);
    BaseAlgorithm3D::GetProjectpntToLine(dDistanceLeftPnt, higherAxis.ptStart, higherAxis.ptEnd, conerPt3);

    conerPnts.Add(conerPt0);
    conerPnts.Add(conerPt1);
    conerPnts.Add(conerPt2);
    conerPnts.Add(conerPt3);
    conerPnts.Add(conerPt0);
}

void RoadMark::CalPtsToLineInfo(const Array<Coordinate> &arrCloudPts, const TLineTwoPoints &resAxis, Array<PointToLineInfo> &arrPointInfoSort)
{
    Int32 nPointCount = arrCloudPts.GetCount();
    if (nPointCount == 0)
    {
        return;
    }

    arrPointInfoSort.Clear();
    for (Int32 i = 0; i < nPointCount; i++)
    {
        PointToLineInfo PointInfoSort;
        Coordinate projectPnt;
        BaseAlgorithm3D::GetProjectpntToLine(arrCloudPts[i], resAxis.ptStart, resAxis.ptEnd, projectPnt);

        Double isSameDirection = (resAxis.ptStart - projectPnt) * (resAxis.ptStart - resAxis.ptEnd);
        if (isSameDirection < 0)
        {
            PointInfoSort.dMeasure = (-1.0) * (arrCloudPts[i].Distance(resAxis.ptStart));
        }
        else
        {
            PointInfoSort.dMeasure = arrCloudPts[i].Distance(resAxis.ptStart);
        }

        PointInfoSort.dDistance = arrCloudPts[i].Distance(projectPnt);

        PointInfoSort.index = i;
        PointInfoSort.projectPnt = projectPnt;
        PointInfoSort.nSide = BaseAlgorithm::PntMatchLine(resAxis.ptStart, resAxis.ptEnd, arrCloudPts[i]);

        arrPointInfoSort.Add(PointInfoSort);
    }
}

std::shared_ptr<data_access_engine::PositionObjectProxy> RoadMark::createObject(const vector<Vec3> &points, data_access_engine::TileInfoPtr &tile)
{
    if (points.size() < 5)
        return nullptr;

    auto basePt = RoadDataManager::getInstance()->getBasePoint();

    auto obj = std::make_shared<data_access_engine::PositionObjectProxy>();
    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < points.size(); ++i)
    {
        auto pt = obj->mutable_border()->mutable_pts()->add();
        pt->set_x(points[i].x);
        pt->set_y(points[i].y);
        pt->set_z(points[i].z);
        auto pp = points[i];
        pp += basePt;

        pos.X() += pp.x;
        pos.Y() += pp.y;
        pos.Z() += pp.z;
    }
    pos.X() /= points.size();
    pos.Y() /= points.size();
    pos.Z() /= points.size();

    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();
    pMgr->make_new_id(pos, obj, tile, true);
    tile->mutable_position_objects()->push_back(obj);
    auto version = obj->id()->version();
    obj->mutable_id()->set_version(version + 1);
    return obj;
}

void RoadMark::DealDuplicatePosition(Array<RoadMarkObj> &lineobjs, std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> &vecPositions)
{
    if (lineobjs.IsEmpty() || vecPositions.empty())
        return;

    // 利用中心点进行去重操作
    /*
     * 基本策略：
     * 1、计算所有feature的中心点
     * 2、网格filter距离在1米内的中心点
     * 3、删除新生成的数据，保留原始数据
     */

    Array<Coordinate> oriCenterPts;
    for (int i = 0; i < vecPositions.size(); ++i)
    {
        Array<Coordinate> vecPlyPts;
        // 解析所有的点
        for (int j = 0; j < vecPositions[i]->border()->pts().size(); ++j)
        {
            Coordinate curCrood;
            curCrood.x = vecPositions[i]->border()->pts().at(j)->x();
            curCrood.y = vecPositions[i]->border()->pts().at(j)->y();
            curCrood.z = vecPositions[i]->border()->pts().at(j)->z();
            vecPlyPts.Add(curCrood);
        }
        oriCenterPts.Add(CommonUtil::GetAveragePt(vecPlyPts));
    }

    // 针对中心点建立网格
    RoadTopoGrid roadTopoGrid;
    roadTopoGrid.BuildTopoGrid(oriCenterPts, 1.0);

    int nCount = lineobjs.GetCount();
    for (int i = nCount - 1; i >= 0; i--)
    {
        lineobjs[i].centerPt = CommonUtil::GetAveragePt(lineobjs[i].plygonPnts);
        Array<Int32> matchIndexArr;
        // 在1米范围内搜索最近的点
        roadTopoGrid.GetMatchInfos(lineobjs[i].centerPt, oriCenterPts, matchIndexArr, 1.0);
        if (matchIndexArr.IsEmpty()) // 没匹配到保留
            continue;
        lineobjs.Delete(i); // 匹配到删除
    }
}