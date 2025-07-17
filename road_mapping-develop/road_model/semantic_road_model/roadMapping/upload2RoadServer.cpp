//
//
//

#include "upload2RoadServer.h"
#include <vector>
#include <fstream>

// 包装RoadBoundaryProxy
std::shared_ptr<data_access_engine::RoadBoundaryProxy> upload2RoadServer::CreatRoadBoundaryProxy(const SingleLine &rb, int type)
{
    if (rb.linePtsVec.size() < 2)
        return nullptr;

    auto b = std::make_shared<data_access_engine::RoadBoundaryProxy>();
    b->set_type(type); // 路缘石
    for (int i = 0; i < rb.linePtsVec.size(); ++i)
    {
        auto pt = b->mutable_geom()->mutable_pts()->add();
        auto pp = rb.linePtsVec[i];
        pt->set_x(pp.X());
        pt->set_y(pp.Y());
        pt->set_z(pp.Z());
    }

    if (!b)
        return nullptr;

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    data_access_engine::Vector3D pos = rb.linePtsVec[0];

    pMgr->make_new_id(pos, b, tile, true);
    tile->mutable_road_boundarys()->push_back(b);
    return b;
}

// 包装PositionObjectProxy
std::shared_ptr<data_access_engine::PositionObjectProxy> upload2RoadServer::CreatPositionProxy(const Position &ps)
{
    if (ps.psPtsVec.size() < 2)
        return nullptr;

    auto obj = std::make_shared<data_access_engine::PositionObjectProxy>();
    obj->set_type(ps.ObjectType);

    if (ps.ObjectType == 8) // 停止线需设置子类型
        obj->set_sub_type(1);

    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < ps.psPtsVec.size(); ++i)
    {

        auto pt = std::make_shared<data_access_engine::PointProxy>();
        if (ps.ObjectType != 8) // 停止线为线型特征
            pt = obj->mutable_border()->mutable_pts()->add();
        else if (ps.ObjectType == 8) // 停止线为线型特征
            pt = obj->mutable_pole()->mutable_pts()->add();

        pt->set_x(ps.psPtsVec[i].X());
        pt->set_y(ps.psPtsVec[i].Y());
        pt->set_z(ps.psPtsVec[i].Z());

        pos += ps.psPtsVec[i];
    }
    pos.X() /= ps.psPtsVec.size();
    pos.Y() /= ps.psPtsVec.size();
    pos.Z() /= ps.psPtsVec.size();

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    pMgr->make_new_id(pos, obj, tile, true);
    tile->mutable_position_objects()->push_back(obj);
    return obj;
}

// 包装TrafficInfoProxy
std::shared_ptr<data_access_engine::TrafficInfoProxy> upload2RoadServer::CreateTrafficInfoProxy(const Position &ps)
{
    if (ps.psPtsVec.size() < 2)
        return nullptr;

    std::shared_ptr<data_access_engine::PositionObjectProxy> PositionObject = CreatPositionProxy(ps);
    if (PositionObject == nullptr)
        return nullptr;

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    auto info = std::make_shared<data_access_engine::TrafficInfoProxy>();
    info->mutable_objs()->push_back(PositionObject->id());

    // type 4 in traffic is landmark 2默认直行
    info->set_type(4);
    int arrow_type = ps.ArrowType;
    info->mutable_marking()->set_type(arrow_type);

    data_access_engine::Vector3D pos = ps.psPtsVec[0];

    pMgr->make_new_id(pos, info, tile, true);
    tile->mutable_traffic_infos()->push_back(info);
    return info;
}

// 包装JunctionProxy vevLines 为一个路口内所有道路边界
std::shared_ptr<data_access_engine::JunctionProxy> upload2RoadServer::CreatJunctionProxy(RoadMapping::Junction junction_input)
{
    int n = junction_input.lukou_polygon_pts.GetCount();
    if (n < 3)
    {
        return nullptr;
    }

    SingleLine newLine;
    for (int j = 0; j < n; ++j)
    {
        newLine.linePtsVec.push_back(data_access_engine::Vector3D(junction_input.lukou_polygon_pts[j].x, junction_input.lukou_polygon_pts[j].y, junction_input.lukou_polygon_pts[j].z));
    }

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    auto junction = std::make_shared<data_access_engine::JunctionProxy>();
    std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRoadBoundary = upload2RoadServer::CreatRoadBoundaryProxy(newLine, 0);
    if (!newRoadBoundary)
    {
        return nullptr;
    }
    junction->mutable_boundarys()->push_back(newRoadBoundary->id());
    for (int i = 0; i < junction_input.areas_relation.GetCount(); i++)
    {
        std::shared_ptr<data_access_engine::ImpassableAreaProxy> newImpassableArea = CreatImpassableAreaProxy(junction_input.areas_relation[i]);
        if (newImpassableArea)
        {
            junction->mutable_areas()->push_back(newImpassableArea);
        }
    }

    if (junction->boundarys().size() > 0)
    {
        data_access_engine::Vector3D pos = newLine.linePtsVec[0];
        pMgr->make_new_id(pos, junction, tile, true);
        tile->mutable_junctions()->push_back(junction);
    }

    return junction;
}

// 包装路口内所有不可通行区域
std::shared_ptr<data_access_engine::ImpassableAreaProxy> upload2RoadServer::CreatImpassableAreaProxy(RoadMapping::ImpassableArea impassable_input)
{
    if (impassable_input.plygonPnts.GetCount() < 2)
        return nullptr;

    auto obj = std::make_shared<data_access_engine::ImpassableAreaProxy>();
    obj->set_kind(impassable_input.subtype);

    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < impassable_input.plygonPnts.GetCount(); ++i)
    {

        auto pt = std::make_shared<data_access_engine::PointProxy>();
        pt = obj->mutable_geom()->mutable_pts()->add();

        pt->set_x(impassable_input.plygonPnts[i].x);
        pt->set_y(impassable_input.plygonPnts[i].y);
        pt->set_z(impassable_input.plygonPnts[i].z);

        pos.X() += impassable_input.plygonPnts[i].x;
        pos.Y() += impassable_input.plygonPnts[i].y;
        pos.Z() += impassable_input.plygonPnts[i].z;
    }
    pos.X() /= impassable_input.plygonPnts.GetCount();
    pos.Y() /= impassable_input.plygonPnts.GetCount();
    pos.Z() /= impassable_input.plygonPnts.GetCount();

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    return obj;
}

/******LaneGroupProxy*****/
// 包装LaneBoundaryProxy
std::shared_ptr<data_access_engine::LaneBoundaryProxy> upload2RoadServer::CreateLaneBoundaryProxy(const SingleLine &lb)
{
    if (lb.linePtsVec.size() < 2)
        return nullptr;

    auto lane_boundary = std::make_shared<data_access_engine::LaneBoundaryProxy>();
    lane_boundary->set_color(1);
    lane_boundary->set_ldm(false);
    lane_boundary->set_marking(1);
    lane_boundary->set_types({1});

    for (int i = 0; i < lb.linePtsVec.size(); ++i)
    {
        auto pt = lane_boundary->mutable_geom()->mutable_pts()->add();
        auto pp = lb.linePtsVec[i];

        pt->set_x(pp.X());
        pt->set_y(pp.Y());
        pt->set_z(pp.Z());
    }
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    data_access_engine::Vector3D pos = lb.linePtsVec[0];
    pMgr->make_new_id(pos, lane_boundary, tile, true);
    tile->mutable_lane_boundarys()->push_back(lane_boundary);

    return lane_boundary;
}

// 包装LaneBoundaryRangeProxy
void upload2RoadServer::CreateBoundaryRange(data_access_engine::LaneBoundaryRangeProxy *&boundaryRange,
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

// 包装LaneGroupProxy
std::shared_ptr<data_access_engine::LaneGroupProxy> upload2RoadServer::CreateLaneGroupProxy(const LaneGroup &vecGroupLanes)
{
    if (vecGroupLanes.lbVec.size() < 2)
        return nullptr;

    auto lane_group = std::make_shared<data_access_engine::LaneGroupProxy>();

    int seq_no = -1;
    data_access_engine::Vector3D laneGroup_left_pos;
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> boundarys;

    for (size_t i = 0; i < vecGroupLanes.lbVec.size() - 1; i++) // 第一个需创建左右
    {
        std::shared_ptr<data_access_engine::LaneBoundaryProxy> left_lane_boundary, right_lane_boundary;
        if (i == 0)
        {
            left_lane_boundary = upload2RoadServer::CreateLaneBoundaryProxy(vecGroupLanes.lbVec[i]);
            boundarys.push_back(left_lane_boundary);

            right_lane_boundary = upload2RoadServer::CreateLaneBoundaryProxy(vecGroupLanes.lbVec[i + 1]);
            boundarys.push_back(right_lane_boundary);
        }
        else if (i > 0 && i < vecGroupLanes.lbVec.size() - 1) // 其余数据left为上个数据的right 因此只需要创建right
        {
            left_lane_boundary = boundarys[boundarys.size() - 1];
            right_lane_boundary = upload2RoadServer::CreateLaneBoundaryProxy(vecGroupLanes.lbVec[i + 1]);
            boundarys.push_back(right_lane_boundary);
        }

        auto lane = std::make_shared<data_access_engine::LaneExtProxy>();
        lane->set_seq_no(seq_no);
        lane->set_type(0x1);
        lane->set_transition(1);

        auto lane_direct = std::make_shared<data_access_engine::LaneDirectionProxy>();
        lane_direct->set_direction(1);
        lane->mutable_directions()->push_back(lane_direct);

        seq_no--;
        auto lane_section = lane->mutable_lanes()->add();
        auto left_boundaryRange = lane_section->mutable_left_boundary();
        upload2RoadServer::CreateBoundaryRange(left_boundaryRange, left_lane_boundary);
        auto right_boundaryRange = lane_section->mutable_right_boundary();
        upload2RoadServer::CreateBoundaryRange(right_boundaryRange, right_lane_boundary);

        auto &p0 = left_lane_boundary->geom()->pts().front();
        if (i == 0)
        {
            laneGroup_left_pos.X() = p0->x();
            laneGroup_left_pos.Y() = p0->y();
            laneGroup_left_pos.Z() = p0->z();
        }

        data_access_engine::Vector3D lane_left_pos;
        lane_left_pos.X() = p0->x();
        lane_left_pos.Y() = p0->y();
        lane_left_pos.Z() = p0->z();
        pMgr->make_new_id(lane_left_pos, lane, tile, true);

        // 向tile内添加数据
        tile->mutable_lanes()->push_back(lane);
        lane_group->mutable_lanes()->push_back(lane->id());
        auto version = lane->id()->version();
        lane->mutable_id()->set_version(version + 1);
    }
    pMgr->make_new_id(laneGroup_left_pos, lane_group, tile, true);
    tile->mutable_lane_groups()->push_back(lane_group);
    return lane_group;
}

// 任务内所有结果包装proxy
void upload2RoadServer::CreatLaneGroupProxyAll(const std::vector<LaneGroup> &vecLGs)
{
    for (int i = 0; i < vecLGs.size(); ++i)
    {
        std::shared_ptr<data_access_engine::LaneGroupProxy> newLG = upload2RoadServer::CreateLaneGroupProxy(vecLGs[i]);
    }
}

void upload2RoadServer::CreatRoadBoundaryProxyAll(const std::vector<SingleLine> &vevRBs)
{
    for (int i = 0; i < vevRBs.size(); ++i)
    {
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRB = CreatRoadBoundaryProxy(vevRBs[i], 2);
    }
}

void upload2RoadServer::CreatPositionProxyAll(const std::vector<Position> vecRMs)
{
    for (int i = 0; i < vecRMs.size(); ++i)
    {
        std::shared_ptr<data_access_engine::PositionObjectProxy> newps = CreatPositionProxy(vecRMs[i]);
    }
}

void upload2RoadServer::CreatTrafficInfoProxyAll(const std::vector<Position> vecRMs)
{
    for (int i = 0; i < vecRMs.size(); ++i)
    {
        if (vecRMs[i].ObjectType != 7)
            continue; // 目前只有地面标识才需要使用CreatPositionProxy
        std::shared_ptr<data_access_engine::TrafficInfoProxy> newTrafficInfo = CreateTrafficInfoProxy(vecRMs[i]);
    }
}

void upload2RoadServer::CreatJunctionProxyAll(Engine::Base::Array<RoadMapping::Junction> vecJunctions)
{
    for (int i = 0; i < vecJunctions.GetCount(); ++i)
    {
        std::shared_ptr<data_access_engine::JunctionProxy> newJunction = CreatJunctionProxy(vecJunctions[i]);
    }
}

/*******************************************数据上传**************************************************/
void upload2RoadServer::upLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir)
{
    // 上传服务
    auto server_upload = data_access_engine::ConfigAddress::get_instance();
    server_upload->set_road_server_address(address);
    server_upload->set_road_server_upload_branch(road_branch);

    auto talker = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    // tile 修正前输出
    data_access_engine::TileInfoList up_tiles;
    talker->get_changed_tile_data(up_tiles);
    talker->correct_tiles();
    // talker->correct_tile_refs();

    // tile 修正后输出
    data_access_engine::TileInfoList corr_tiles;
    talker->get_changed_tile_data(corr_tiles);

    // 测试输出结果
    WriteBoundarysToObj(corr_tiles, outputDir, "res");
    WriteRoadMarkToObj(corr_tiles, outputDir, "res");

    talker->upload_tiles(editor, corr_tiles, 0);
    std::cout << "upload_tiles finish .......road_branch: " << road_branch << std::endl;
}

void upload2RoadServer::WriteBoundarysToObj(const data_access_engine::TileInfoList &tiles, std::string filePath, std::string name)
{
    for (auto tile : tiles)
    {
        int m = 0;
        for (auto lb : tile->lane_boundarys())
        {
            std::string lbOutName = std::to_string(tile->tile_id()) + "_" + name + "_boundary_" + std::to_string(m);
            std::vector<data_access_engine::Vector3D> vecPts;
            for (auto pt : lb->geom()->pts())
            {
                vecPts.push_back(data_access_engine::Vector3D(pt->x(), pt->y(), pt->z()));
            }
            WriteToObj(vecPts, filePath, lbOutName);
            m++;
        }
    }
}

void upload2RoadServer::WriteRoadMarkToObj(const data_access_engine::TileInfoList &tiles, std::string path, std::string name)
{
    for (auto tile : tiles)
    {
        int k = 0;
        for (auto obj : tile->position_objects())
        {
            std::vector<data_access_engine::Vector3D> vecPts;
            int n = 0;
            if (obj->type() == 8)
                n = obj->pole()->pts().size();
            else
                n = obj->border()->pts().size();
            for (int j = 0; j < n; ++j)
            {
                data_access_engine::Vector3D curCrood;
                if (obj->type() == 8)
                {
                    curCrood.X() = obj->pole()->pts().at(j)->x();
                    curCrood.Y() = obj->pole()->pts().at(j)->y();
                    curCrood.Z() = obj->pole()->pts().at(j)->z();
                }
                else
                {
                    curCrood.X() = obj->border()->pts().at(j)->x();
                    curCrood.Y() = obj->border()->pts().at(j)->y();
                    curCrood.Z() = obj->border()->pts().at(j)->z();
                }

                vecPts.push_back(curCrood);
            }
            std::vector<std::vector<data_access_engine::Vector3D>> vecVecPts;
            vecVecPts.push_back(vecPts);
            std::string objName = std::to_string(tile->tile_id()) + "_" + name + "_Position_" + std::to_string(k);
            WriteToObj(vecVecPts, path, objName);
            k++;
        }
    }
}

void upload2RoadServer::WriteToObj(std::vector<std::vector<data_access_engine::Vector3D>> vecVecPts, std::string path, std::string name)
{
    if (vecVecPts.empty())
        return;

    if (path.back() != '/' || path.back() != '\\')
    {
        path += "/";
    }

    std::string objFullPath = path + name + ".obj";

    std::ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        std::cout << "obj 保存不成功" << std::endl;
        return;
    }

    for (int i = 0; i < vecVecPts.size(); i++)
    {
        for (auto j = 0; j < vecVecPts[i].size(); j++)
        {
            ofs << std::fixed;
            ofs << "v " << vecVecPts[i][j].X() << " " << vecVecPts[i][j].Y() << " " << vecVecPts[i][j].Z() << std::endl;
        }
    }

    int K = 0;
    for (int i = 0; i < vecVecPts.size(); i++)
    {
        if (vecVecPts[i].size() <= 1)
        {
            continue;
        }
        ofs << "l ";
        for (auto j = 0; j < vecVecPts[i].size(); j++)
        {
            ofs << ++K << " ";
        }
        ofs << std::endl;
    }

    ofs.close();
    return;
}

void upload2RoadServer::WriteToObj(std::vector<data_access_engine::Vector3D> vecPts, std::string path, std::string name)
{
    if (vecPts.size() < 2)
        return;

    if (path.back() != '/' || path.back() != '\\')
    {
        path += "/";
    }

    std::string objFullPath = path + name + ".obj";

    std::ofstream ofs(objFullPath);
    if (!ofs.is_open())
    {
        std::cout << "obj 保存不成功" << std::endl;
        return;
    }

    for (int i = 0; i < vecPts.size(); i++)
    {
        ofs << std::fixed;
        ofs << "v " << vecPts[i].X() << " " << vecPts[i].Y() << " " << vecPts[i].Z() << std::endl;
    }

    int K = 0;
    ofs << "l ";
    for (auto i = 0; i < vecPts.size(); i++)
    {
        ofs << ++K << " ";
    }
    ofs << std::endl;

    ofs.close();
    return;
}