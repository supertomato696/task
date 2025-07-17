//
//
//
#include "RoadLayer.h"
#include "DataManager.h"
#include "HDLane.h"
#include "Vec.h"
#include "Log.h"
#include "CommonUtil.h"
#include "../data-access-engine/manager/road_geometry_mgr.h"
#include "../data-access-engine/proxy/lane_proxy.h"
#include "../data-access-engine/proxy/position_proxy.h"
#include "../data-access-engine/proxy/common_proxy.h"
#include "../data-access-engine/public/proj_helper.h"
#include "../data-access-engine/public/vector3.h"

using namespace hdmap_build;
using namespace std;
RoadLayer::RoadLayer()
{
    _strLayerName = "road";
    //    _pLayerRenderGroup->setName("road");
}

RoadLayer::~RoadLayer()
{
}

void RoadLayer::setOriTileID(int tile_id)
{
    m_oriTileID = tile_id;
}

void RoadLayer::setTilePath(string tile_path)
{
    m_strtileFile = tile_path;
}

void RoadLayer::createBoundaryRange(data_access_engine::LaneBoundaryRangeProxy *&boundaryRange,
                                    const std::shared_ptr<const data_access_engine::LaneBoundaryProxy> &b)
{
    boundaryRange->mutable_bound_id()->set(b->id());
    boundaryRange->mutable_start_pt()->set_x(b->geom()->pts().front()->x());
    boundaryRange->mutable_start_pt()->set_y(b->geom()->pts().front()->y());
    boundaryRange->mutable_start_pt()->set_z(b->geom()->pts().front()->z());
    boundaryRange->mutable_end_pt()->set_x(b->geom()->pts().back()->x());
    boundaryRange->mutable_end_pt()->set_y(b->geom()->pts().back()->y());
    boundaryRange->mutable_end_pt()->set_z(b->geom()->pts().back()->z());
}

std::shared_ptr<data_access_engine::LaneBoundaryProxy> RoadLayer::createBoundary(const std::vector<Vec3> &points, float dis)
{
    if (points.size() < 2)
        return nullptr;
    auto helper = RoadDataManager::getInstance()->getProjectionHelper();
    auto basePt = RoadDataManager::getInstance()->getBasePoint();
    auto lane_boundary = std::make_shared<data_access_engine::LaneBoundaryProxy>();
    lane_boundary->set_color(1);
    lane_boundary->set_ldm(false);
    lane_boundary->set_marking(1);
    std::vector<int> types;
    types.push_back(1);
    lane_boundary->set_types(types);

    data_access_engine::Vector3D pos;
    for (int i = 0; i < points.size(); ++i)
    {
        auto pt = lane_boundary->mutable_geom()->mutable_pts()->add();
        auto pp = points[i];
        pp += basePt;

        // helper->Gauss_Kruger_unprojection(pp.x, pp.y, pp.z);
        pt->set_x(pp.x);
        pt->set_y(pp.y);
        pt->set_z(pp.z);

        if (i == 0)
        {
            pos.X() = pp.x;
            pos.Y() = pp.y;
            pos.Z() = pp.z;
            VULCAN_LOG_INFO("pos: <{}> <{}> <{}>......", pos.X(), pos.Y(), pos.Z());
        }
    }
    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();
    data_access_engine::TileInfoPtr tile;
    pMgr->make_new_id(pos, lane_boundary, tile, true);
    tile->mutable_lane_boundarys()->push_back(lane_boundary);
    VULCAN_LOG_INFO("lane_boundary tile_id <{}> ......", tile->tile_id());
    // LOG(INFO) << "createBoundary" << lane_boundary->id()->to_string();
    return lane_boundary;
}

std::shared_ptr<data_access_engine::LaneGroupProxy> RoadLayer::createRoad(std::vector<HDLane::GroupLane> &vecGroupLanes, data_access_engine::TileInfoPtr &tile)
{
    if (vecGroupLanes.empty())
        return nullptr;

    auto lane_group = std::make_shared<data_access_engine::LaneGroupProxy>();

    int seq_no = -1;
    data_access_engine::Vector3D laneGroup_left_pos;
    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();

    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> boundarys;
    int k = 0;
    for (size_t i = 0; i < vecGroupLanes.size(); i++) // 第一个需创建左右
    {
        // VULCAN_LOG_INFO("left_boundary <{}> ......", i);
        std::shared_ptr<data_access_engine::LaneBoundaryProxy> left_lane_boundary, right_lane_boundary;
        if (i == 0)
        {
            left_lane_boundary = createBoundary(vecGroupLanes[i].leftPoints);
            boundarys.push_back(left_lane_boundary);

            // VULCAN_LOG_INFO("right_boundary <{}> ......", i);
            right_lane_boundary = createBoundary(vecGroupLanes[i].rightPoints);
            boundarys.push_back(right_lane_boundary);
        }
        else if (i > 0 && i < vecGroupLanes.size()) // 其余数据left为上个数据的right 因此只需要创建right
        {
            left_lane_boundary = boundarys[boundarys.size() - 1];
            right_lane_boundary = createBoundary(vecGroupLanes[i].rightPoints);
            boundarys.push_back(right_lane_boundary);
        }

        auto lane = std::make_shared<data_access_engine::LaneExtProxy>();
        lane->set_seq_no(seq_no);
        lane->set_type(2);
        lane->set_transition(1);

        seq_no--;
        auto lane_section = lane->mutable_lanes()->add();
        auto left_boundaryRange = lane_section->mutable_left_boundary();
        createBoundaryRange(left_boundaryRange, left_lane_boundary);
        auto right_boundaryRange = lane_section->mutable_right_boundary();
        createBoundaryRange(right_boundaryRange, right_lane_boundary);

        auto &p0 = left_lane_boundary->geom()->pts().front();

        if (i == 0)
        {
            laneGroup_left_pos.X() = p0->x();
            laneGroup_left_pos.Y() = p0->y();
            laneGroup_left_pos.Z() = p0->z();
            // VULCAN_LOG_INFO("left_pos: <{}> <{}> <{}>......", laneGroup_left_pos.X(), laneGroup_left_pos.Y(), laneGroup_left_pos.Z());
        }

        data_access_engine::Vector3D lane_left_pos;
        lane_left_pos.X() = p0->x();
        lane_left_pos.Y() = p0->y();
        lane_left_pos.Z() = p0->z();

        pMgr->make_new_id(lane_left_pos, lane, tile, true);
        // VULCAN_LOG_INFO("lane <{}> tile_id <{}> ......", i, tile->tile_id());

        // 向tile内添加数据
        tile->mutable_lanes()->push_back(lane);
        //        tile->mutable_lane_boundarys()->push_back(left_lane_boundary);
        //        tile->mutable_lane_boundarys()->push_back(right_lane_boundary);
        lane_group->mutable_lanes()->push_back(lane->id());
        auto version = lane->id()->version();
        lane->mutable_id()->set_version(version + 1);

        // 输出lane
        // string name = to_string(tile->tile_id()) + "_" + to_string(k);
        // WriteLaneToObj(lane, m_strtileFile, name);
        k++;
    }

    pMgr->make_new_id(laneGroup_left_pos, lane_group, tile, true);
    tile->mutable_lane_groups()->push_back(lane_group);
    // VULCAN_LOG_INFO("lane_group tile_id <{}> ......", tile->tile_id());
    return lane_group;
}

std::shared_ptr<data_access_engine::RoadBoundaryProxy> RoadLayer::CreateRoadBoundary(const std::vector<Vec3> &points)
{
    if (points.size() < 2)
        return nullptr;

    auto helper = RoadDataManager::getInstance()->getProjectionHelper();
    auto basePt = RoadDataManager::getInstance()->getBasePoint();

    auto b = std::make_shared<data_access_engine::RoadBoundaryProxy>();

    for (int i = 0; i < points.size(); ++i)
    {
        auto pt = b->mutable_geom()->mutable_pts()->add();
        auto pp = points[i];
        pt->set_x(pp.x);
        pt->set_y(pp.y);
        pt->set_z(pp.z);
    }
    return b;
}

void RoadLayer::getLaneBounderyPoints(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &lb_proxy, std::vector<Vec3> &pointList)
{
    if (lb_proxy == NULL)
    {
        return;
    }
    int ptSize = lb_proxy->geom()->pts().size();

    if (ptSize < 2)
        return;

    for (int m = 0; m < ptSize; ++m)
    {
        auto pt = lb_proxy->geom()->pts().at(m);
        auto ep = Vec3(pt->x(), pt->y(), pt->z());
        pointList.push_back(ep);
    }
}

void RoadLayer::WriteLaneToObj(std::shared_ptr<data_access_engine::LaneProxy> &lane, string filePath, string name)
{
    if (lane == NULL)
    {
        return;
    }

    HDLane::GroupLane newGroup;
    if (lane->mutable_lanes()->size() > 0)
    {
        std::shared_ptr<data_access_engine::LaneBoundaryProxy> lb_proxy = lane->mutable_lanes()->at(0)->mutable_left_boundary()->mutable_bound_id()->proxy();
        getLaneBounderyPoints(lb_proxy, newGroup.leftPoints);

        std::shared_ptr<data_access_engine::LaneBoundaryProxy> rb_proxy = lane->mutable_lanes()->at(0)->mutable_right_boundary()->mutable_bound_id()->proxy();
        getLaneBounderyPoints(rb_proxy, newGroup.rightPoints);
    }

    HDLane _HDlane;
    _HDlane.WriteToOBJ(newGroup, filePath, name, false);
}

void RoadLayer::WriteBoundaryToObj(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &lb_proxy, string filePath, string name)
{
    HDLane::GroupLane newGroup;
    getLaneBounderyPoints(lb_proxy, newGroup.leftPoints);

    HDLane _HDlane;
    _HDlane.WriteToOBJ(newGroup, filePath, name, false);
}

void RoadLayer::WriteBoundarysToObj(std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &tilelbs, string filePath, string name)
{
    Array<LineString *> arrArrlBs;
    for (auto lb : tilelbs)
    {
        arrArrlBs.Add(LaneBoundaryToLineString(lb, false)); // 转换为线的时候会转化为utm坐标
    }
    CommonUtil::WriteToOBJ(arrArrlBs, filePath, name, true);
}

void RoadLayer::WriteTileMapsObj(data_access_engine::ID2TileMap tiles_maps, string filePath, string name)
{
    for (const auto &tile_map : tiles_maps)
    {
        auto tile = tile_map.second;
        int k = 0;
        for (auto lane : tile->lanes())
        {
            string laneName = to_string(tile->tile_id()) + "_" + name + "_lanes_" + to_string(k);
            WriteLaneToObj(lane, filePath, laneName);
            k++;
        }

        int m = 0;
        for (auto lb : tile->lane_boundarys())
        {
            string laneName = to_string(tile->tile_id()) + "_" + name + "_boundary_" + to_string(m);
            WriteBoundaryToObj(lb, filePath, laneName);
            m++;
        }
    }
}

void RoadLayer::WriteTileObj(data_access_engine::TileInfoList tiles, string strTilePath, string name)
{
    for (auto tile : tiles)
    {
        int k = 0;
        for (auto lane : tile->lanes())
        {
            string laneName = to_string(tile->tile_id()) + "_" + name + "_lanes_" + to_string(k);
            WriteLaneToObj(lane, strTilePath, laneName);
            k++;
        }

        int m = 0;
        for (auto lb : tile->lane_boundarys())
        {
            string laneName = to_string(tile->tile_id()) + "_" + name + "_boundary_" + to_string(m);
            WriteBoundaryToObj(lb, strTilePath, laneName);
            m++;
        }
    }
}

bool RoadLayer::BreakUpRoad(std::shared_ptr<data_access_engine::LaneGroupProxy> &_laneGroup)
{
    if (_laneGroup->mutable_lanes()->size() == 0)
        return false;
    // 判断点最左侧线是否有跨tile的情况
    //    auto pt = _selectedLanePoint.front();
    //    if (!pt)
    //        return false;
    //    auto pt_boundary = pt->lane_boundary.lock();
    //    if (!pt_boundary)
    //        return false;
    //    auto pLaneSection = pt->lanesection.lock();
    //    if (!pLaneSection)
    //        return false;
    //    auto ls = pLaneSection->lanesection;
    //    if (!ls)
    //        return false;
    //    bool bRight = pt_boundary == ls->right_boundary()->bound_id().proxy() ? true : false;
    //    auto veclane0 = pLaneSection->road_section.lock()->lanes[-1];
    //    if (veclane0.empty())
    //        return false;
    //    LOG(INFO) << " CRoadLayer::BreakUpRoad";
    //    auto& points = veclane0.front()->left_points;
    //    if (pt->lane_id != -1 || (pt->lane_id == -1 && bRight))
    //    {
    //        //��ȡ�����ϵĵ�
    //        osg::Vec3d intersectPt;
    //        if (findVerticalPoint(points, pt->pos, intersectPt)) {
    //            pt->pos = intersectPt;
    //            pt->lanesection = veclane0.front();
    //            pt->lane_boundary = veclane0.front()->lanesection->mutable_left_boundary()->mutable_bound_id()->proxy();
    //            pt->lane_id = 0;
    //        }
    //        else {
    //            return false;
    //        }
    //    }
    //    if (((pt->pos - points.front()).length() < 1e-03)
    //        || ((pt->pos - points.back()).length() < 1e-03)) {
    //        return false;
    //    }
    //    auto rm = pLaneSection->road_section.lock();
    //    std::vector<std::pair<int, std::shared_ptr<data_access_engine::LaneBoundaryProxy>>> first_left_boundarys;
    //    std::vector<std::pair<int, std::shared_ptr<data_access_engine::LaneBoundaryProxy>>> two_left_boundarys;
    //
    //    std::shared_ptr<data_access_engine::LaneBoundaryProxy> first_right_boundary;
    //    std::shared_ptr<data_access_engine::LaneBoundaryProxy> two_right_boundary;
    //    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> boundarys;
    //    for (auto itr = rm->lanes.rbegin(); itr != rm->lanes.rend(); ++itr)
    //    {
    //        auto lane_id = itr->first;
    //        auto& points = itr->second.front()->left_points;
    //        std::vector<osg::Vec3d> first_points;
    //        std::vector<osg::Vec3d> two_points;
    //        auto pos = pt->pos;
    //        if (lane_id != -1)
    //        {
    //            if (!findVerticalPoint(points, pt->pos, pos))
    //                return false;
    //        }
    //        if (!getTwoLanePoints(pos, points, first_points, two_points))
    //            return false;
    //        auto first_left_boundary = createBoundary(first_points);
    //        auto two_left_boundary = createBoundary(two_points);
    //        first_left_boundarys.push_back(std::make_pair(lane_id, first_left_boundary));
    //        two_left_boundarys.push_back(std::make_pair(lane_id, two_left_boundary));
    //        boundarys.push_back(first_left_boundary);
    //        boundarys.push_back(two_left_boundary);
    //        if (lane_id == rm->lanes.begin()->first) {
    //            auto& points = itr->second.front()->right_points;
    //            std::vector<osg::Vec3d> first_points;
    //            std::vector<osg::Vec3d> two_points;
    //            auto pos = pt->pos;
    //            if (!findVerticalPoint(points, pt->pos, pos))
    //                return false;
    //            if (!getTwoLanePoints(pos, points, first_points, two_points))
    //                return false;
    //            first_right_boundary = createBoundary(first_points);
    //            two_right_boundary = createBoundary(two_points);
    //            boundarys.push_back(first_right_boundary);
    //            boundarys.push_back(two_right_boundary);
    //        }
    //    }
    //
    //    std::vector<std::shared_ptr<data_access_engine::LaneProxy>> lanes;
    //    auto first_road = createRoad(first_left_boundarys, first_right_boundary, lanes);
    //    auto two_road = createRoad(two_left_boundarys, two_right_boundary, lanes);
    //    auto road = pLaneSection->road_section.lock()->road;
    //    for (int i = 0; i < road->lanes().size(); ++i)
    //    {
    //        /*auto l = road->mutable_lanes()->at(i);
    //        l->set_is_deleted(true);
    //        lanes.push_back(l.proxy());*/
    //    }
    //    road->mutable_id()->set_is_deleted(true);
    //    std::vector<std::shared_ptr<data_access_engine::LaneGroupProxy>> lanegroups;
    //    lanegroups.push_back(first_road);
    //    lanegroups.push_back(two_road);
    //    lanegroups.push_back(road);
    //    LOG(INFO) << "road:" << road->id()->to_string() << ",first_road:" << first_road->id()->to_string() << ",two_road:" << two_road->id()->to_string();
    //    DataManager::RoadDataManager::getInstance()->SetLaneGeometry(boundarys, lanes, lanegroups);
    //    DataManager::RoadDataManager::getInstance()->MakeNewVersion();
    //    data_access_engine::SharedProxyVector<data_access_engine::LaneGroupProxy> roads;
    //    for (auto& road : lanegroups) {
    //        roads.push_back(road);
    //    }
    //    loadRoad(roads);
    //    _bResetDisplay = true;
    //    return true;
}

void RoadLayer::GetLeftLineStringsInTiles(data_access_engine::TileInfoList &corrtiles, std::vector<std::shared_ptr<data_access_engine::LaneSectionProxy>> &LaneSections, Array<LineString *> &laneLines)
{
    for (auto tile : corrtiles)
    {
        int k = 0;
        for (auto lane : tile->lanes()) // tile内lane
        {
            if (lane->mutable_lanes()->empty())
                continue;
            std::vector<Vec3> pointListLeft, pointListRight;
            std::shared_ptr<data_access_engine::LaneBoundaryProxy> lb_proxy = lane->mutable_lanes()->at(0)->mutable_left_boundary()->mutable_bound_id()->proxy();
            getLaneBounderyPoints(lb_proxy, pointListLeft);

            if (pointListLeft.empty()) // 要求lanesection的左边界存在
                continue;

            // 将点坐标进行转换，并组织为线
            Array<Coordinate> leftArrPts, rightArrPts;
            leftArrPts = CommonUtil::Vec3CroodToEngine(pointListLeft);
            CommonUtil::Wgs84toUtm(leftArrPts);
            LineString *leftLine = CommonUtil::CoordsToLineString(leftArrPts);

            // 将线和Lane存储，建立对应关系1-1
            LaneSections.push_back(lane->mutable_lanes()->at(0));
            laneLines.Add(leftLine);
        }
    }
}

std::vector<std::shared_ptr<data_access_engine::LaneGroupProxy>> RoadLayer::getLaneGroup(const data_access_engine::ID2TileMap &tiles_map)
{
    std::vector<std::shared_ptr<data_access_engine::LaneGroupProxy>> resLaneGroups;
    if (tiles_map.empty())
        return resLaneGroups;
    for (auto &tile : tiles_map)
    {
        for (auto &lanegroup : tile.second->lane_groups())
        {
            resLaneGroups.push_back(lanegroup);
        }
    }
    return resLaneGroups;
}

std::vector<std::shared_ptr<data_access_engine::LaneProxy>> RoadLayer::getLane(const data_access_engine::ID2TileMap &tiles_map)
{
    std::vector<std::shared_ptr<data_access_engine::LaneProxy>> resLaneProxys;
    for (auto &tile : tiles_map)
    {
        for (auto &lane : tile.second->lanes())
        {
            resLaneProxys.push_back(lane);
        }
    }
    return resLaneProxys;
}

std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> RoadLayer::getLanesLaneBoundary(const data_access_engine::ID2TileMap &tiles_map)
{
    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> resLaneBoundarys;
    //    std::vector<std::shared_ptr<data_access_engine::LaneProxy>> resLanes = getLane(tiles_map);
    //    for (int i = 0; i < resLanes.size(); i++)
    //    {
    //        if (!resLanes[i]->mutable_lanes())
    //            continue;
    //        int nsize = resLanes[i]->mutable_lanes()->size();
    //        for (int j = 0; j < nsize; j++)
    //        {
    //            if (i == 0)
    //            {
    //                std::shared_ptr<data_access_engine::LaneBoundaryProxy> llb_proxy = resLanes[i]->mutable_lanes()->at(j)->mutable_left_boundary()->mutable_bound_id()->proxy();
    //                resLaneBoundarys.push_back(llb_proxy);
    //            }
    //            std::shared_ptr<data_access_engine::LaneBoundaryProxy> rlb_proxy = resLanes[i]->mutable_lanes()->at(j)->mutable_right_boundary()->mutable_bound_id()->proxy();
    //            resLaneBoundarys.push_back(rlb_proxy);
    //        }
    //    }
    //
    //    return resLaneBoundarys;

    // 上面的方法经过测试发现仍有一些不是车道线的线引入，并且线的序号有不正常的情况存在，因此测试LaneGroup解析的正确性
    std::vector<std::shared_ptr<data_access_engine::LaneGroupProxy>> resLaneGroups = getLaneGroup(tiles_map);
    for (int i = 0; i < resLaneGroups.size(); i++)
    {
        if (!resLaneGroups[i]->mutable_lanes()) // 通过lanegroup获取所有的lanes
            continue;
        int nsize = resLaneGroups[i]->mutable_lanes()->size(); // 一个lanegroup内所有lanes的个数
        for (int j = 0; j < nsize; j++)
        {
            std::shared_ptr<data_access_engine::LaneProxy> laneProxy = resLaneGroups[i]->mutable_lanes()->at(j).proxy(); // 得到lane
            for (int k = 0; k < laneProxy->mutable_lanes()->size(); ++k)                                                 // 循环lanesection的个数
            {
                if (j == 0)
                {
                    std::shared_ptr<data_access_engine::LaneBoundaryProxy> llb_proxy = laneProxy->mutable_lanes()->at(k)->mutable_left_boundary()->mutable_bound_id()->proxy();
                    int type = llb_proxy->types().at(0);
                    if (type == 1)
                        resLaneBoundarys.push_back(llb_proxy);
                }
                std::shared_ptr<data_access_engine::LaneBoundaryProxy> rlb_proxy = laneProxy->mutable_lanes()->at(k)->mutable_right_boundary()->mutable_bound_id()->proxy();
                int type = rlb_proxy->types().at(0);
                if (type == 1)
                    resLaneBoundarys.push_back(rlb_proxy);
            }
        }
    }
    return resLaneBoundarys;
}

std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> RoadLayer::getLaneBoundary(const data_access_engine::ID2TileMap &tiles_map, int type)
{
    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> vecLaneBoundaryProxy;
    if (tiles_map.empty())
        return vecLaneBoundaryProxy;
    for (auto tile : tiles_map)
    {
        for (auto laneboundary : tile.second->lane_boundarys())
        {
            int type0 = laneboundary->types().at(0);
            if (type0 == type)
                vecLaneBoundaryProxy.push_back(laneboundary);
        }
    }
    return vecLaneBoundaryProxy;
}

std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> RoadLayer::getRoadBoundary(const data_access_engine::ID2TileMap &tiles_map)
{
    std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> vecRoadBoundary;
    if (tiles_map.empty())
        return vecRoadBoundary;
    for (auto tile : tiles_map)
    {
        for (auto roadboundary : tile.second->road_boundarys())
        {
            vecRoadBoundary.push_back(roadboundary);
        }
    }
    return vecRoadBoundary;
}

std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> RoadLayer::getPosition(const data_access_engine::ID2TileMap &tiles_map)
{
    std::vector<std::shared_ptr<data_access_engine::PositionObjectProxy>> vecPositions;
    if (tiles_map.empty())
        return vecPositions;
    for (auto tile : tiles_map)
    {
        for (auto position : tile.second->position_objects())
        {
            vecPositions.push_back(position);
        }
    }
    return vecPositions;
}

void RoadLayer::DealDuplicateLaneGroup(std::vector<std::vector<LineObj>> &lineobjs, std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &laneBoundarys)
{
    if (lineobjs.empty() || laneBoundarys.empty())
        return;

    // 根据所有的laneboundary构建网格，网格宽度为5米
    Array<LineString *> lbLines;
    for (auto &laneboundary : laneBoundarys)
    {
        lbLines.Add(LaneBoundaryToLineString(laneboundary, false)); // 将lb的线中点转换为50N的UTM
    }
    RoadTopoGrid roadTopoGrid;
    roadTopoGrid.BuildTopoGrid(lbLines, 5.0);
    int nLaneGroup = lineobjs.size();
    for (int i = nLaneGroup - 1; i >= 0; i--)
    {
        // 对单个lanegroup数据进行匹配
        int nLanes = lineobjs[i].size();
        for (int j = nLanes - 1; j >= 0; j--)
        {
            Array<Coordinate> arrPnts = lineobjs[i][j].m_lineCroods;
            Int32 nPointCount = arrPnts.GetCount();
            if (nPointCount == 0)
                continue;
            Array<Int32> arrPointMatch;
            arrPointMatch.SetSize(nPointCount);
            memset(arrPointMatch.Data(), 0, sizeof(Int32) * nPointCount);

            // 得到每条线的match信息
            roadTopoGrid.GetMatchInfos(arrPnts, arrPointMatch, 5.0, true);

            // 将重复点删除
            Array<Array<Coordinate>> arrArrPtsRes;
            RoadTopoBuild roadTopoBuild;
            roadTopoBuild.DelDuplicateSegs(arrPnts, arrPointMatch, arrArrPtsRes, 5.0); // 长度大于5米的删除
            if (arrArrPtsRes.IsEmpty())
            {
                lineobjs[i].erase(lineobjs[i].begin() + j); // 全部匹配重复，删除所有线
            }
            if (arrArrPtsRes.GetCount() > 1)
            {
                // 保留最长的，别的先删除
                Array<Coordinate> lineLongest;
                double lenLongest = CommonUtil::LongestLine(arrArrPtsRes, lineLongest);
                lineobjs[i][j].m_lineCroods = lineLongest;
            }
        }
    }
}

void RoadLayer::DealDuplicateRoadBoundary(std::vector<std::vector<LineObj>> &lineobjs, std::vector<std::shared_ptr<data_access_engine::RoadBoundaryProxy>> &roadBoundarys)
{
    if (lineobjs.empty() || roadBoundarys.empty())
        return;

    // 根据所有的roadBoundarys构建网格，网格宽度为5米
    Array<LineString *> rbLines;
    for (auto &laneboundary : roadBoundarys)
    {
        LineString *newLine = new LineString();
        newLine = RoadBoundaryToLineString(laneboundary, false);
        if (newLine)
            rbLines.Add(newLine); // 将lb的线中点转换为50N的UTM
    }
    RoadTopoGrid roadTopoGrid;
    roadTopoGrid.BuildTopoGrid(rbLines, 5.0);
    int nLaneGroup = lineobjs.size();
    for (int i = nLaneGroup - 1; i >= 0; i--)
    {
        // 对单个lanegroup数据进行匹配
        int nLanes = lineobjs[i].size();
        for (int j = nLanes - 1; j >= 0; j--)
        {
            Array<Coordinate> arrPnts = lineobjs[i][j].m_lineCroods;
            Int32 nPointCount = arrPnts.GetCount();
            if (nPointCount == 0)
                continue;
            Array<Int32> arrPointMatch;
            arrPointMatch.SetSize(nPointCount);
            memset(arrPointMatch.Data(), 0, sizeof(Int32) * nPointCount);

            // 得到每条线的match信息
            roadTopoGrid.GetMatchInfos(arrPnts, arrPointMatch, 5.0, true);

            // 将重复点删除
            Array<Array<Coordinate>> arrArrPtsRes;
            RoadTopoBuild roadTopoBuild;
            roadTopoBuild.DelDuplicateSegs(arrPnts, arrPointMatch, arrArrPtsRes, 5.0); // 长度小于5米的删除
            if (arrArrPtsRes.IsEmpty())
            {
                lineobjs[i].erase(lineobjs[i].begin() + j); // 全部匹配重复，删除整条线
            }
            if (arrArrPtsRes.GetCount() > 1)
            {
                // 删除重复的 其余线保留
                lineobjs[i].erase(lineobjs[i].begin() + j); // 先删除所有的线，然后将不重复的线插入容器中
                for (int k = 0; k < arrArrPtsRes.GetCount(); ++k)
                {
                    LineObj newLineObj;
                    newLineObj.m_lineCroods = arrArrPtsRes[k];
                    lineobjs[i].push_back(newLineObj);
                }
            }
        }
    }
}

LineString *RoadLayer::LaneBoundaryToLineString(std::shared_ptr<data_access_engine::LaneBoundaryProxy> &laneboundary, bool utm)
{
    vector<Vec3> vecPnts;
    getLaneBounderyPoints(laneboundary, vecPnts);
    Array<Coordinate> arrPnts = CommonUtil::Vec3CroodToEngine(vecPnts);
    if (arrPnts.IsEmpty() || arrPnts.GetCount() < 2)
        return nullptr;

    if (!utm) // 需要进行坐标转换
    {
        CommonUtil::Wgs84toUtm(arrPnts);
    }
    LineString *resLine = new LineString();
    resLine = CommonUtil::CoordsToLineString(arrPnts);

    return resLine;
}

LineString *RoadLayer::RoadBoundaryToLineString(std::shared_ptr<data_access_engine::RoadBoundaryProxy> &roadboundary, bool utm)
{
    vector<Vec3> vecPnts;
    int ptSize = roadboundary->geom()->pts().size();

    if (ptSize < 2)
        return nullptr;

    for (int m = 0; m < ptSize; ++m)
    {
        auto pt = roadboundary->geom()->pts().at(m);
        auto ep = Vec3(pt->x(), pt->y(), pt->z());
        vecPnts.push_back(ep);
    }
    Array<Coordinate> arrPnts = CommonUtil::Vec3CroodToEngine(vecPnts);
    if (arrPnts.IsEmpty() || arrPnts.GetCount() < 2)
        return nullptr;

    if (!utm) // 需要进行坐标转换
    {
        CommonUtil::Wgs84toUtm(arrPnts);
    }
    LineString *resLine = new LineString();
    resLine = CommonUtil::CoordsToLineString(arrPnts);

    return resLine;
}

std::vector<int> RoadLayer::GetTileIDsByData(std::vector<std::vector<LineObj>> &lineobjs, bool wgs84)
{
    std::set<int> tild_ids;
    auto pMgr = RoadDataManager::getInstance()->getRoadGeometryManager();
    for (int i = 0; i < lineobjs.size(); i++)
    {
        for (int j = 0; j < lineobjs[i].size(); ++j)
        {
            Array<Coordinate> arrPts = lineobjs[i][j].m_lineCroods;
            if (!wgs84)
                CommonUtil::Utm2Wgs84(arrPts);
            for (int k = 0; k < arrPts.GetCount(); k++)
            {
                data_access_engine::Vector3<double> pos;
                pos.X() = arrPts[k].x;
                pos.Y() = arrPts[k].y;
                pos.Z() = arrPts[k].z;
                int tile_id = pMgr->WGS84_to_tileID(pos);
                tild_ids.insert(tile_id);
            }
        }
    }
    std::vector<int> res_tile_ids;
    for (auto tile : tild_ids)
    {
        res_tile_ids.push_back(tile);
    }
    return res_tile_ids;
}