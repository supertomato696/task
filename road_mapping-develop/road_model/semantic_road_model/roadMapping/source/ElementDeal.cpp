//
//
//
#include "ElementDeal.h"
#include "DataReader.h"
#include "Log.h"
#include "HDLane.h"
#include "RoadMark.h"
#include "LineObj.h"
#include "TrafficSign.h"
#include "RoadLayer.h"
#include "DataManager.h"
#include "CommonUtil.h"
#include <string>
#include <iostream>
#include <fstream>
#include <memory.h>
#include <memory>
#include <shared_mutex>
#include <sys/stat.h>
#include <unistd.h>
#include "Base/Types.h"
#include "Geometries/Coordinate.h"
#include "Geometries/Envelope.h"
#include "Geometries/BaseAlgorithm.h"
#include "Base/Array.h"

#include "../data-access-engine/manager/road_geometry_mgr.h"
#include "../data-access-engine/proxy/lane_proxy.h"
#include "../data-access-engine/proxy/position_proxy.h"
#include "../data-access-engine/proxy/common_proxy.h"

using namespace std;
using namespace hdmap_build;
using namespace Engine;
using namespace Engine::Base;
using namespace Engine::Geometries;

ElementDeal::ElementDeal()
{
}

ElementDeal::~ElementDeal()
{
}

void ElementDeal::InitParameter(string strTilePath, string down_road_branch, string up_road_branch, int tile_id, string editor, string address)
{
    m_BasePath = strTilePath;
    m_TileId = tile_id;
    m_DownLoadBranch = down_road_branch;
    m_UpLoadBranch = up_road_branch;
    m_editor = editor;
    m_address = address;
}

void ElementDeal::ExtractLaneGroups()
{
    // 修改处理单元，所有tile读取结束以后整体处理 [yxx 2021-8-27]
    vector<vector<LineObj>> tileHDLaneVecVec;
    Array<Coordinate> yellowPnts; // 输出黄色信息的点【yxx 2021-10-12】
    HDLane _HDLane;
    _HDLane.m_tilePath = m_BasePath;
    _HDLane.m_tileID = to_string(m_TileId);
    DataReader::LoadAllLanes(m_BasePath, tileHDLaneVecVec, yellowPnts);

    // 读取到tile内所有数据，进1行去重操作(将坐标转换为UTM 50N)
    if (tileHDLaneVecVec.empty())
    {
        VULCAN_LOG_INFO("<{}> lanes empty!!!!!", m_TileId); // 为空输出log
        return;
    }

    // 输出所有去重前的线[yxx 2021-9-9]
    string name = to_string(m_TileId) + "_BeforQuchong";
    CommonUtil::WriteToOBJ(tileHDLaneVecVec, m_BasePath, name, false);

    Array<Array<LineObject>> lineObjectArrArr;
    for (int i = 0; i < tileHDLaneVecVec.size(); ++i)
    {
        Array<LineObject> lineObjects;
        for (int j = 0; j < tileHDLaneVecVec[i].size(); ++j)
        {
            LineObject _lineobject;
            _HDLane.HDLaneToLineObject(tileHDLaneVecVec[i][j], _lineobject, i, j); // 将坐标进行转换
            lineObjects.Add(_lineobject);
        }
        lineObjectArrArr.Add(lineObjects);
    }
    Array<LineObject> lineObjectArr;
    RoadTopoBuild::CleanDuplicateLines(lineObjectArrArr, lineObjectArr, 1.0, false); // 去重阈值暂时设为100cm

    // 恢复去重后数据(此时数据格式为UTM，并更新长度, 按照车道序号进行排列)
    vector<vector<LineObj>> newHDLaneVecVec;
    _HDLane.LineObjectsToHDLaneArr(lineObjectArr, tileHDLaneVecVec, newHDLaneVecVec);
    if (newHDLaneVecVec.empty())
    {
        return;
    }

    // 输出去重后线
    string name1 = to_string(m_TileId) + "_AfterQuchong";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, name1, true);

    // 去除折回线
    _HDLane.SmoothSTurnCHDLane(newHDLaneVecVec);

    string outPut = to_string(m_TileId) + "_SmoothSTurn";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);

    // 删除短线
    _HDLane.RemoveShortHDLanes(newHDLaneVecVec);

    // 输出去重后线
    outPut = to_string(m_TileId) + "_AfterDelete";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);

    // 连接线
    _HDLane.ConnectHDLanes(newHDLaneVecVec);

    // 输出连接后的线
    outPut = to_string(m_TileId) + "_AfterConnect";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);

    // 进行节点对齐的操作
    _HDLane.NodeAline(newHDLaneVecVec);

    // 输出节点对齐后的线
    outPut = to_string(m_TileId) + "_AfterNodeAline";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);

    // 检测黄线标识，进行删除操作[yxx 2021-10-12]此步位于抽稀前，保留尽可能多的原始点，从而使黄色匹配信息更准确
    CommonUtil::Wgs84toUtm(yellowPnts); // 将点转换为utm50N
    outPut = to_string(m_TileId) + "_yellowPnts";
    CommonUtil::WriteToTxt(yellowPnts, m_BasePath, outPut, true);

    _HDLane.ClipByYellow(yellowPnts, newHDLaneVecVec);
    outPut = to_string(m_TileId) + "_AfterClipByYellow";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);

    // 进行点去除短距离，抽稀功能 阈值5cm
    _HDLane.ResampleCHDLane(newHDLaneVecVec, 0.05);
    outPut = to_string(m_TileId) + "_AfterResample";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, outPut, true);
    m_lineobjs = newHDLaneVecVec;

    //    //转换坐标
    //    for(int i = 0; i < newHDLaneVecVec.size(); i++)
    //    {
    //        for (int j = 0; j < newHDLaneVecVec[i].size(); ++j)
    //        {
    //            CommonUtil::Utm2Wgs84(newHDLaneVecVec[i][j].m_lineCroods);
    //        }
    //    }
}

void ElementDeal::CreatLaneGroupProxy()
{
    VULCAN_LOG_INFO("CreatLaneGroupProxy process start ...... ");
    // 区分左右边界
    HDLane _HDLane;
    _HDLane.m_tilePath = m_BasePath;
    _HDLane.m_tileID = to_string(m_TileId);

    VULCAN_LOG_INFO("GroupLeftRight process start ...... ");
    std::vector<std::vector<HDLane::GroupLane>> vecVecGroupLanes;
    _HDLane.GroupLeftRight(m_lineobjs, vecVecGroupLanes);
    VULCAN_LOG_INFO("GroupLeftRight process finish ...... ");

    // 组装proto
    VULCAN_LOG_INFO("createRoad process start ...... ");
    RoadLayer roadLayer;
    roadLayer.setOriTileID(m_TileId);
    roadLayer.setTilePath(m_BasePath);
    for (int i = 0; i < vecVecGroupLanes.size(); i++)
    {
        // 输出GroupLane
        string outPut = to_string(m_TileId) + "_Group" + to_string(i);
        _HDLane.WriteToOBJ(vecVecGroupLanes[i], m_BasePath, outPut, false);
        std::shared_ptr<data_access_engine::LaneGroupProxy> resLaneGroupProxy;
        data_access_engine::TileInfoPtr tile;
        VULCAN_LOG_INFO("lanegroup : <{}>.......", i);
        resLaneGroupProxy = roadLayer.createRoad(vecVecGroupLanes[i], tile);
        if (resLaneGroupProxy == nullptr)
        {
            VULCAN_LOG_INFO("LaneGroupProxy is null ...... ");
            continue;
        }
        int nLaneGroup = tile->lane_groups().size();
        int nLane = tile->lanes().size();
        VULCAN_LOG_INFO("<{}> has lanes count : <{}>", tile->tile_id(), nLane);
        VULCAN_LOG_INFO("<{}> has lane_groups count : <{}>", tile->tile_id(), nLaneGroup);
    }
    VULCAN_LOG_INFO("createRoad process finish ...... ");
}

void ElementDeal::ExtractRoadMark()
{
    RoadMark roadMark;
    roadMark.InitParameter(m_BasePath, m_UpLoadBranch, m_TileId);

    // 1 提取RoadMark的外包络点集
    Array<Array<Coordinate>> arrArrPackPoints;
    roadMark.ExtractPackages(arrArrPackPoints);

    // 2 获取tile内lanesection,确定矩形框的主轴
    // roadMark.BindRoadMarkToLane(arrArrPackPoints, corrtiles);

    // 2.根据车道线的lineobjs，确认主轴，没有主轴的用自身点进行拟合
    Array<LineString *> arrLines;
    HDLane _HDLane;
    for (auto lineobj : m_lineobjs)
    {
        for (auto &line : lineobj)
        {
            arrLines.Add(_HDLane.LineObjToLineString(line, true)); // 此时坐标为UTM不需要坐标变换
        }
    }
    // 删除长小于3米，宽小于0.1米的RoadMark
    roadMark.GetRoadMarkObjs(arrLines, arrArrPackPoints, m_roadmarks);
}

void ElementDeal::CreatRoadMarkProxy()
{
    RoadMark roadMark;
    Array<Array<Coordinate>> arrArrPolygonPoints;
    for (int i = 0; i < m_roadmarks.GetCount(); i++)
    {
        arrArrPolygonPoints.Add(m_roadmarks[i].plygonPnts);
    }
    string name = to_string(m_TileId) + "_RoadMarkPolygons";
    CommonUtil::WriteToOBJ(arrArrPolygonPoints, m_BasePath, name, true);

    std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> objs;
    for (int i = 0; i < m_roadmarks.GetCount(); i++)
    {
        // 转换坐标为经纬度
        vector<Vec3> vecPolygonPts = CommonUtil::EngineCroodToVec3(m_roadmarks[i].plygonPnts);
        vector<Vec3> blPolygonPts = CommonUtil::Utm2Wgs84(vecPolygonPts);
        data_access_engine::TileInfoPtr tile;
        std::shared_ptr<data_access_engine::PositionObjectProxy> newPositionProxy = roadMark.createObject(blPolygonPts, tile);
        objs.push_back(newPositionProxy);

        //        //根据LaneSection id获取lane
        //        if (m_roadmarks[i].nearstLineIndex >= 0)
        //        {
        //            auto LaneSection = LaneSections[arrRoadMarkObj[i].nearstLineIndex];
        //            auto id = newPositionProxy->id();
        //            LaneSection->mutable_objects()->add()->set(id);
        //            LaneSection->mark_changed();
        //        }
    }
}
void ElementDeal::ExtractTrafficSign()
{
    Array<TrafficSignObj> allTrafficObjs;
    DataReader::LoadAllTrafficSignJson(m_BasePath, to_string(m_TileId), allTrafficObjs);

    // 输出所有的牌
    Array<Array<Coordinate>> allPnts;
    Array<Coordinate> positions;
    for (int i = 0; i < allTrafficObjs.GetCount(); ++i)
    {
        allPnts.Add(allTrafficObjs[i].plygonPnts);
    }

    string name = to_string(m_TileId) + "_Traffic_PlY";
    CommonUtil::WriteToOBJ(allPnts, m_BasePath, name, false);
}

void ElementDeal::CreatTrafficSignProxy()
{

    for (int i = 0; i < m_trafficsign.GetCount(); i++)
    {
        TrafficSign trafficSign;
        data_access_engine::TileInfoPtr tile;
        std::shared_ptr<data_access_engine::PositionObjectProxy> newPositionProxy = trafficSign.createObject(CommonUtil::EngineCroodToVec3(m_trafficsign[i].plygonPnts), tile);
    }
}

void ElementDeal::MerageData()
{
    // 拉取范围tiles内的原始langroup数据
    RoadLayer roadLayer;
    vector<int> tile_ids = roadLayer.GetTileIDsByData(m_lineobjs, false); // 成员变量中的数据为utm
    for (auto &tile : tile_ids)
    {
        VULCAN_LOG_INFO("数据经过的id：{}......", tile);
    }

    RoadDataManager newRoadDataManager;

    // 目前策略：
    /*
     * 1、首先下载现在所有的laneboundary与现有线进行去重（完成）
     * 2、去重后的线与原有laneGroup进行融合，多出的车道线融合到原有数据中
     * 3、判断数据是否要进行对齐，若需要移动，记录移动参数，将车道线、道路边界、地面标识等按照一定的规则进行移动
     * 4、对移动后的数据地面标识、道路边界进行去重操作（）
     */
    // 根据tileid下载数据,解析所有的laneboundary
    data_access_engine::ID2TileMap tilesmap;

    // 测试
    // tile_ids.push_back(329569486);
    // tile_ids.push_back(329569498);
    // tile_ids.push_back(329569499);
    newRoadDataManager.download_all_tile_data(tile_ids, m_address, m_DownLoadBranch, tilesmap);

    // 通过laneboundary类型获取标线 type=1-车道线
    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> allTileLaneBoundarys = roadLayer.getLaneBoundary(tilesmap, 1);
    std::string lbsName = "all_download_lbs";
    roadLayer.WriteBoundarysToObj(allTileLaneBoundarys, m_BasePath, lbsName);

    // 将LaneGroup数据去重
    roadLayer.DealDuplicateLaneGroup(m_lineobjs, allTileLaneBoundarys);
    string name = "DealDuplicateLaneGroup";
    CommonUtil::WriteToOBJ(m_lineobjs, m_BasePath, name, true); // 输出与数据库中去重后的数据

    // 下载所有roadboundary，并去重
    std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> allTileRoadBoundarys = roadLayer.getRoadBoundary(tilesmap);
    roadLayer.DealDuplicateRoadBoundary(m_roadboundaryobjs, allTileRoadBoundarys);

    // 下载所有的定位标识，并去重
    std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> allTilePosition = roadLayer.getPosition(tilesmap);
    RoadMark roadMark;
    roadMark.DealDuplicatePosition(m_roadmarks, allTilePosition);
}

void ElementDeal::UpLoadTiles(data_access_engine::TileInfoList &corrtiles)
{
    RoadLayer roadLayer;
    RoadMark roadMark;
    // 上传服务
    VULCAN_LOG_INFO("UpLoad Tile ID : <{}> start ......", m_TileId);
    std::string address = "172.21.207.124:8081";
    auto conAdd = RoadDataManager::getConfigAddress();
    conAdd->set_road_server_address(address);
    if (m_UpLoadBranch.empty())
    {
        m_UpLoadBranch = "yxx_branch";
    }

    conAdd->set_road_server_upload_branch(m_UpLoadBranch);
    RoadDataManager newRoadDataManager;
    auto pMgr = newRoadDataManager.getRoadGeometryManager();
    std::string editor = "test";

    // tile 修正前输出
    data_access_engine::TileInfoList uptiles;
    pMgr->get_changed_tile_data(uptiles);
    roadLayer.WriteTileObj(uptiles, m_BasePath, "Before_Correct");
    roadMark.WriteRoadMarkToObj(uptiles, m_BasePath, "Before_Correct");

    pMgr->correct_tiles();
    // pMgr->correct_tile_refs();

    // tile 修正后输出
    pMgr->get_changed_tile_data(corrtiles);
    roadLayer.WriteTileObj(corrtiles, m_BasePath, "After_Correct");
    roadMark.WriteRoadMarkToObj(uptiles, m_BasePath, "After_Correct");

    pMgr->upload_tiles(editor, corrtiles, 0);
    VULCAN_LOG_INFO("UpLoad Tile ID : <{}> finish ......", m_TileId);
    VULCAN_LOG_INFO("road_branch Tile ID : <{}> finish ......", m_UpLoadBranch);
}

void ElementDeal::ExtractRoadBoundarys()
{
    // 修改处理单元，所有tile读取结束以后整体处理 [yxx 2021-8-27]
    vector<vector<LineObj>> tileHDLaneVecVec;
    HDLane _HDLane;
    _HDLane.m_tilePath = m_BasePath;
    _HDLane.m_tileID = to_string(m_TileId);
    DataReader::LoadRoadBoundarys(m_BasePath, tileHDLaneVecVec);

    if (tileHDLaneVecVec.empty())
    {
        VULCAN_LOG_INFO("<{}> lanes empty!!!!!", m_TileId); // 为空输出log
        return;
    }

    // 输出所有线[yxx 2021-9-9]
    //  cout << "------------>" << __LINE__ << ": "  <<  "tileHDLaneVecVec.size(): " << tileHDLaneVecVec.size() << endl;
    //  cout << "------------>" << __LINE__ << ": "  <<  "tileHDLaneVecVec.front.size(): " << tileHDLaneVecVec.front().size() << endl;
    //  cout << "------------>" << __LINE__ << ": "  <<  "m_lineCroods.GetCount() : " << tileHDLaneVecVec.front().front().m_lineCroods.GetCount() << endl;

    std::vector<HDLane::GroupLane> group;
    _HDLane.GroupPoints(tileHDLaneVecVec, group);

    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();

    std::vector<int> tile_15_ids;
    tile_15_ids.push_back(m_TileId);
    pMgr->init_tiles_by_id(tile_15_ids);
    RoadLayer roadLayer;
    for (auto lane : group)
    {
        auto b = roadLayer.CreateRoadBoundary(lane.leftPoints);
        if (!b)
        {
            std::cout << __FILE__ << " ------------>" << __LINE__ << std::endl;
        }

        auto pts = b->mutable_geom()->mutable_pts();
        // for (int i = 0; i < pts->size(); i++)
        // {
        //     std::cout << __FILE__ <<" ------------>" << __LINE__ <<  ":  " <<  pts->at(i)->x() <<  ":  " <<  pts->at(i)->y() <<  ":  " <<  pts->at(i)->z() << std::endl;
        // }

        data_access_engine::TileInfoPtr tile;
        auto pp = lane.leftPoints.front();
        data_access_engine::Vector3D pos;
        pos.X() = pp.x;
        pos.Y() = pp.y;
        pos.Z() = pp.z;
        // cout << "------------>" << __LINE__ << ": "  <<  "pos: " << pos.X() <<  " " << pos.Y() <<  " " << pos.Z() << endl;
        // b->make_id_index();
        pMgr->make_new_id(pos, b, tile, true);
        // cout << "------------>" << __LINE__ << ": "  <<  "pos: " << pos.X() <<  " " << pos.Y() <<  " " << pos.Z() << endl;
        tile->mutable_road_boundarys()->push_back(b);
    }
}

void ElementDeal::ExtractRoadBoundarys2()
{
    // 修改处理单元，所有tile读取结束以后整体处理 [yxx 2021-8-27]
    vector<vector<LineObj>> tileHDLaneVecVec;
    HDLane _HDLane;
    _HDLane.m_tilePath = m_BasePath;
    _HDLane.m_tileID = to_string(m_TileId);
    DataReader::LoadRoadBoundarys2(m_BasePath, tileHDLaneVecVec);

    if (tileHDLaneVecVec.empty())
    {
        VULCAN_LOG_INFO("<{}> lanes empty!!!!!", m_TileId); // 为空输出log
        return;
    }

    // 输出所有去重前的线[yxx 2021-9-9]
    string name = to_string(m_TileId) + "_Boundary_BeforQuchong";
    CommonUtil::WriteToOBJ(tileHDLaneVecVec, m_BasePath, name, false);

    Array<Array<LineObject>> lineObjectArrArr;
    for (int i = 0; i < tileHDLaneVecVec.size(); ++i)
    {
        Array<LineObject> lineObjects;
        for (int j = 0; j < tileHDLaneVecVec[i].size(); ++j)
        {
            LineObject _lineobject;
            _HDLane.HDLaneToLineObject(tileHDLaneVecVec[i][j], _lineobject, i, j); // 将坐标进行转换
            lineObjects.Add(_lineobject);
        }
        lineObjectArrArr.Add(lineObjects);
    }

    Array<LineObject> lineObjectArr;
    RoadTopoBuild::CleanDuplicateLines(lineObjectArrArr, lineObjectArr, 0.30, true, 1.0); // 去重阈值暂时设为100cm

    // 恢复去重后数据(此时数据格式为UTM，并更新长度, 按照车道序号进行排列)
    vector<vector<LineObj>> newHDLaneVecVec;
    _HDLane.LineObjectsToHDLaneArr(lineObjectArr, tileHDLaneVecVec, newHDLaneVecVec);
    if (newHDLaneVecVec.empty())
    {
        return;
    }

    // 输出去重后线
    string name1 = to_string(m_TileId) + "_Boundary_AfterQuchong";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, name1, true);

    // 对输出线进行平滑、抽稀
    for (int i = 0; i < newHDLaneVecVec.size(); ++i)
    {

        for (int j = 0; j < newHDLaneVecVec[i].size(); j++)
        {
            Array<Coordinate> resPnts;
            RoadTopoBuild::LineSmooth7(newHDLaneVecVec[i][j].m_lineCroods, resPnts);
            newHDLaneVecVec[i][j].m_lineCroods = resPnts;

            CommonUtil::RemoveDuplicatePoints(newHDLaneVecVec[i][j].m_lineCroods, 1.0); // 1米为去除重复点数据
            CommonUtil::Resample(newHDLaneVecVec[i][j].m_lineCroods, 0.05);
        }
    }

    // 输出去平滑后线
    string name2 = to_string(m_TileId) + "_Boundary_AfterSmooth";
    CommonUtil::WriteToOBJ(newHDLaneVecVec, m_BasePath, name2, true);

    // 转换坐标
    for (int i = 0; i < newHDLaneVecVec.size(); i++)
    {
        for (int j = 0; j < newHDLaneVecVec[i].size(); ++j)
        {
            CommonUtil::Utm2Wgs84(newHDLaneVecVec[i][j].m_lineCroods);
        }
    }

    m_roadboundaryobjs = newHDLaneVecVec;
}

void ElementDeal::CreatRoadBoundaryProxy()
{

    HDLane _HDLane;
    _HDLane.m_tilePath = m_BasePath;
    _HDLane.m_tileID = to_string(m_TileId);

    std::vector<HDLane::GroupLane> group;
    _HDLane.GroupPoints(m_roadboundaryobjs, group);

    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();

    std::vector<int> tile_15_ids;
    tile_15_ids.push_back(m_TileId);
    pMgr->init_tiles_by_id(tile_15_ids);
    RoadLayer roadLayer;
    for (auto lane : group)
    {
        auto b = roadLayer.CreateRoadBoundary(lane.leftPoints);
        if (!b)
        {
            std::cout << __FILE__ << " ------------>" << __LINE__ << std::endl;
        }

        auto pts = b->mutable_geom()->mutable_pts();
        // for (int i = 0; i < pts->size(); i++)
        // {
        //     std::cout << __FILE__ <<" ------------>" << __LINE__ <<  ":  " <<  pts->at(i)->x() <<  ":  " <<  pts->at(i)->y() <<  ":  " <<  pts->at(i)->z() << std::endl;
        // }

        data_access_engine::TileInfoPtr tile;
        auto pp = lane.leftPoints.front();
        data_access_engine::Vector3D pos;
        pos.X() = pp.x;
        pos.Y() = pp.y;
        pos.Z() = pp.z;
        // cout << "------------>" << __LINE__ << ": "  <<  "pos: " << pos.X() <<  " " << pos.Y() <<  " " << pos.Z() << endl;
        // b->make_id_index();
        pMgr->make_new_id(pos, b, tile, true);
        // cout << "------------>" << __LINE__ << ": "  <<  "pos: " << pos.X() <<  " " << pos.Y() <<  " " << pos.Z() << endl;
        tile->mutable_road_boundarys()->push_back(b);
    }
}