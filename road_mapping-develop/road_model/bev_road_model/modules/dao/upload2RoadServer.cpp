
#include "upload2RoadServer.h"
#include "Eigen/src/Core/Matrix.h"
#include <vector>
#include <fstream>
#include "utils/log_util.h"


DEFINE_bool(upload_non_essential_elements, false, "upload_non_essential_elements");

//wq upload test

std::shared_ptr<data_access_engine::LaneBoundaryProxy> upload2RoadServer::testCreateLaneBoundaryProxy(const fsdmap::RoadLaneBoundaryInfo& lb, fsdmap::dao::DataProcessorBase* dp)
{   
    if (lb.line_point_info.size() < 2) {
        return nullptr;
    }
    Eigen::Vector3d wgs;

    auto lane_boundary = std::make_shared<data_access_engine::LaneBoundaryProxy>();
    lane_boundary->set_color(lb.color);
    lane_boundary->set_ldm(false);
    lane_boundary->set_marking(lb.geo);
    lane_boundary->set_types({1});

    for (int i = 0; i < lb.line_point_info.size(); ++i) {
        auto pt = lane_boundary->mutable_geom()->mutable_pts()->add();
        auto pp = lb.line_point_info[i];
        dp->local2wgs(lb.line_point_info[i]->pos, wgs, true);

        pt->set_x(wgs[0]);
        pt->set_y(wgs[1]);
        pt->set_z(wgs[2]);
    }
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    data_access_engine::Vector3D pos;
    
    dp->local2wgs(lb.line_point_info[0]->pos, wgs, true);
    
    pos[0] = wgs[0];
    pos[1] = wgs[1];
    pos[2] = wgs[2];
    pMgr->make_new_id(pos, lane_boundary, tile, true);
    tile->mutable_lane_boundarys()->push_back(lane_boundary);

    return lane_boundary;
}


std::shared_ptr<data_access_engine::LaneGroupProxy> upload2RoadServer::testCreateLaneGroupProxy(fsdmap::RoadLaneGroupInfo& vecGroupLanes, fsdmap::dao::DataProcessorBase* dp)
{
    if (vecGroupLanes.lane_boundary_info.size() < 2)
        return nullptr;

    auto lane_group = std::make_shared<data_access_engine::LaneGroupProxy>();

    int seq_no = -1;
    data_access_engine::Vector3D laneGroup_left_pos;
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> boundarys;

    for(size_t i = 0 ; i < vecGroupLanes.lane_boundary_info.size() - 1; i++) //第一个需创建左右
    {
        std::shared_ptr<data_access_engine::LaneBoundaryProxy> left_lane_boundary, right_lane_boundary;
        if(i == 0)
        {
            left_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecGroupLanes.lane_boundary_info[i], dp);
            boundarys.push_back(left_lane_boundary);

            right_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecGroupLanes.lane_boundary_info[i + 1], dp);
            boundarys.push_back(right_lane_boundary);
        }
        else if (i > 0 && i < vecGroupLanes.lane_boundary_info.size() - 1) //其余数据left为上个数据的right 因此只需要创建right
        {
            left_lane_boundary = boundarys[boundarys.size() - 1];
            right_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecGroupLanes.lane_boundary_info[i + 1], dp);
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

        auto& p0 = left_lane_boundary->geom()->pts().front();
        if(i == 0)
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

        //向tile内添加数据
        tile->mutable_lanes()->push_back(lane);
        lane_group->mutable_lanes()->push_back(lane->id());
        auto version = lane->id()->version();
        lane->mutable_id()->set_version(version + 1);
    }
    pMgr->make_new_id(laneGroup_left_pos, lane_group, tile, true);
    tile->mutable_lane_groups()->push_back(lane_group);
    return lane_group;
}

std::shared_ptr<data_access_engine::LaneExtProxy> upload2RoadServer::testCreateLaneProxy(fsdmap::RoadLaneInfo& vecLanes, fsdmap::dao::DataProcessorBase* dp, int& seq_no, std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> &leftboundarys)
{

    auto lane = std::make_shared<data_access_engine::LaneExtProxy>();
    auto left_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecLanes.left_lane_boundary_info, dp);
    if (seq_no != -1)
    {
        left_lane_boundary = leftboundarys[leftboundarys.size() - 1];
    }
    auto right_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecLanes.right_lane_boundary_info, dp);
    if(FLAGS_upload_non_essential_elements)
    {
        auto center_line = upload2RoadServer::testCreateLaneBoundaryProxy(*vecLanes.center_lane_boundary_info, dp);
        lane->mutable_drivelines()->push_back(center_line->id());
    }
    leftboundarys.push_back(right_lane_boundary);

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    
    lane->set_seq_no(seq_no);
    lane->set_type(0x1);
    lane->set_transition(1);
    
    auto lane_direct = std::make_shared<data_access_engine::LaneDirectionProxy>();
    lane_direct->set_direction(1);
    lane->mutable_directions()->push_back(lane_direct);

    seq_no--;
    auto lane_section = lane->mutable_lanes()->add();

    for(int j = 0; j <vecLanes.bind_arrow_list.size(); j++)
    {
        auto arrow_ptr = vecLanes.bind_arrow_list[j]->traffic_proxy_ptr;
        // lane_section->mutable_traffics()->push_back(arrow_ptr->id());
        lane_section->mutable_objects()->push_back(arrow_ptr->objs().front().id());
    }

    auto left_boundaryRange = lane_section->mutable_left_boundary();
    upload2RoadServer::CreateBoundaryRange(left_boundaryRange, left_lane_boundary);
    auto right_boundaryRange = lane_section->mutable_right_boundary();
    upload2RoadServer::CreateBoundaryRange(right_boundaryRange, right_lane_boundary);

    auto& p0 = left_lane_boundary->geom()->pts().front();
    data_access_engine::Vector3D lane_left_pos;
    lane_left_pos.X() = p0->x();
    lane_left_pos.Y() = p0->y();
    lane_left_pos.Z() = p0->z();
    pMgr->make_new_id(lane_left_pos, lane, tile, true);

    //向tile内添加数据
    tile->mutable_lanes()->push_back(lane);
    auto version = lane->id()->version();
    lane->mutable_id()->set_version(version + 1);

    return lane;

}


std::shared_ptr<data_access_engine::LaneGroupProxy> upload2RoadServer::newCreateLaneGroupProxy(fsdmap::RoadLaneGroupInfo& vecGroupLanes, fsdmap::dao::DataProcessorBase* dp)
{
    if (vecGroupLanes.lane_line_info.size() < 1) {
        return nullptr;
    }

    auto lane_group = std::make_shared<data_access_engine::LaneGroupProxy>();
    int seq_no = -1;
    data_access_engine::Vector3D laneGroup_left_pos;
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> leftboundarys;

    for(size_t i = 0 ; i < vecGroupLanes.lane_line_info.size(); i++) //第一个需创建左右
    {
        
        std::shared_ptr<data_access_engine::LaneExtProxy> lane = upload2RoadServer::testCreateLaneProxy(*vecGroupLanes.lane_line_info[i], dp, seq_no, leftboundarys);
        vecGroupLanes.lane_line_info[i]->lane_ptr = lane;
        if(i == 0)
        {
            // auto left_lane_boundary = upload2RoadServer::testCreateLaneBoundaryProxy(*vecGroupLanes.lane_line_info[0]->left_lane_boundary_info, dp);
            // auto left_lane_boundary = first_left_bdr;
            // auto& p0 = left_lane_boundary->geom()->pts().front();
            auto& p0 = vecGroupLanes.lane_line_info[0]->left_lane_boundary_info->line_point_info[0]->pos;

            laneGroup_left_pos.X() = p0[0];
            laneGroup_left_pos.Y() = p0[1];
            laneGroup_left_pos.Z() = p0[2];
        }
        //向tile内添加数据
        lane_group->mutable_lanes()->push_back(lane->id());
    }
    pMgr->make_new_id(laneGroup_left_pos, lane_group, tile, true);
    tile->mutable_lane_groups()->push_back(lane_group);
    return lane_group;
}

void upload2RoadServer::testCreatLaneGroupProxyAll(std::vector<fsdmap::RoadLaneGroupInfo*>& vecLGs, fsdmap::dao::DataProcessorBase* dp)
{
    CLOG_INFO("upload lanegroups size is %d", vecLGs.size());
    for (int i = 0; i < vecLGs.size(); ++i) {
        // std::shared_ptr<data_access_engine::LaneGroupProxy> newLG = upload2RoadServer::testCreateLaneGroupProxy(*vecLGs[i], dp);
        std::shared_ptr<data_access_engine::LaneGroupProxy> newLG = upload2RoadServer::newCreateLaneGroupProxy(*vecLGs[i], dp);
        vecLGs[i]->lane_group_ptr = newLG;
    }

    for (int i = 0; i < vecLGs.size(); ++i) {
        for (int j = 0; j < vecLGs[i]->lane_line_info.size(); j++)
        {
            auto lane = vecLGs[i]->lane_line_info[j];
            CLOG_DEBUG("lane %p, 前驱车道个数 %d：", lane, lane->context.all_prev.size());
            for(auto& per_prev_ptr : lane->context.all_prev)
            {
                auto &per_prev = per_prev_ptr.src;
                if (per_prev->lane_ptr == NULL) {
                    continue;
                }
                lane->lane_ptr->mutable_preds()->push_back(per_prev->lane_ptr->id());
            }
            CLOG_DEBUG("lane %p, 后继车道个数 %d：", lane, lane->context.all_next.size());
            for(auto& per_next_ptr : lane->context.all_next)
            {
                auto &per_next = per_next_ptr.src;
                if (per_next->lane_ptr == NULL) {
                    continue;
                }
                lane->lane_ptr->mutable_succs()->push_back(per_next->lane_ptr->id());
            }
        }

        //车道组前驱后继
        CLOG_DEBUG("lg %p, 前驱车道组个数 %d：", vecLGs[i], vecLGs[i]->context.all_prev.size());
        for (auto& lg_prev_ptr : vecLGs[i]->context.all_prev)
        {
           auto lg_prev = lg_prev_ptr.src;
           if (lg_prev->lane_group_ptr == NULL) {
                continue;
           }
           vecLGs[i]->lane_group_ptr->mutable_preds()->push_back(lg_prev->lane_group_ptr->id());
        }
        CLOG_DEBUG("lg %p, 后继车道组个数 %d：", vecLGs[i], vecLGs[i]->context.all_next.size());
        for(auto& lg_next_ptr : vecLGs[i]->context.all_next)
        {
            auto &lg_next = lg_next_ptr.src;
            if (lg_next->lane_group_ptr == NULL) {
                continue;
            }
            vecLGs[i]->lane_group_ptr->mutable_succs()->push_back(lg_next->lane_group_ptr->id());
        }
        // 绑定道路边界
        for(auto& left_barrier: vecLGs[i]->left_barrier_segment_info)
        {
            auto boundary_ptr = left_barrier->road_boundary_ptr;
            if (boundary_ptr != nullptr){
            vecLGs[i]->lane_group_ptr->mutable_left_boundarys()->push_back(left_barrier->road_boundary_ptr->id());}
        }
        for(auto& right_barrier: vecLGs[i]->right_barrier_segment_info)
        {
            auto boundary_ptr = right_barrier->road_boundary_ptr;
            if (boundary_ptr != nullptr){
            vecLGs[i]->lane_group_ptr->mutable_right_boundarys()->push_back(right_barrier->road_boundary_ptr->id());}
        }
    }
}


std::shared_ptr<data_access_engine::RoadBoundaryProxy> upload2RoadServer::testCreatRoadBoundaryProxy(const fsdmap::RoadBoundarySegmentInfo& rb, fsdmap::dao::DataProcessorBase* dp)
{
    
    if (rb.point_info.size() < 2)
        return nullptr;

    Eigen::Vector3d wgs;

    auto b = std::make_shared<data_access_engine::RoadBoundaryProxy>();
    b->set_type(2); //路缘石
    // b->set_type(rb.subtype); //路缘石
    for (int i = 0; i < rb.point_info.size(); ++i) {
        auto pt = b->mutable_geom()->mutable_pts()->add();
        auto pp = rb.point_info[i];
        dp->local2wgs(pp->pos, wgs, true);

        pt->set_x(wgs[0]);
        pt->set_y(wgs[1]);
        pt->set_z(wgs[2]);
    }
    
    if (!b)
        return nullptr;

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    data_access_engine::Vector3D pos ;
    Eigen::Vector3d temp_pos= rb.point_info[0]->pos;
    dp->local2wgs(temp_pos, wgs, true);
    pos[0] = wgs[0];
    pos[1] = wgs[1];
    pos[2] = wgs[2];
    
    pMgr->make_new_id(pos, b,  tile, true);
    tile->mutable_road_boundarys()->push_back(b);
    return b;
}

void upload2RoadServer::testCreatRoadBoundaryProxyAll(const std::vector<fsdmap::RoadBoundarySegmentInfo*> vevRBs,fsdmap::dao::DataProcessorBase* dp) 
{
    for (int i = 0; i < vevRBs.size(); ++i) {
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRB = testCreatRoadBoundaryProxy(*vevRBs[i], dp);
        vevRBs[i]->road_boundary_ptr = newRB;
    }
}

void upload2RoadServer::testupLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir)
{
    //上传服务
    auto server_upload = data_access_engine::ConfigAddress::get_instance();
    server_upload->set_road_server_address(address);
    server_upload->set_road_server_upload_branch(road_branch);

    auto talker = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    //tile 修正前输出
    data_access_engine::TileInfoList up_tiles;
    talker->get_changed_tile_data(up_tiles);
    talker->correct_tiles();
    //talker->correct_tile_refs();

    //tile 修正后输出
    data_access_engine::TileInfoList corr_tiles;
    talker->get_changed_tile_data(corr_tiles);

    //测试输出结果
    // WriteBoundarysToObj(corr_tiles, outputDir, "res");
    // WriteRoadMarkToObj(corr_tiles, outputDir, "res");

    talker->upload_tiles(editor, corr_tiles, 0);
    LOG_INFO("upload_road_server finish[branch={}]", road_branch); 
}




/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//包装RoadBoundaryProxy
std::shared_ptr<data_access_engine::RoadBoundaryProxy> upload2RoadServer::CreatRoadBoundaryProxy(const SingleLine& rb, int type)
{
    if (rb.linePtsVec.size() < 2)
        return nullptr;

    auto b = std::make_shared<data_access_engine::RoadBoundaryProxy>();
    b->set_type(type); //路缘石
    for (int i = 0; i < rb.linePtsVec.size(); ++i) {
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
    
    pMgr->make_new_id(pos, b,  tile, true);
    tile->mutable_road_boundarys()->push_back(b);
    return b;
}

//包装PositionObjectProxy
std::shared_ptr<data_access_engine::PositionObjectProxy> upload2RoadServer::CreatPositionProxy(const Position& ps)
{
    if (ps.psPtsVec.size() < 2)
        return nullptr;

    auto obj = std::make_shared<data_access_engine::PositionObjectProxy>();
    obj->set_type(ps.ObjectType);

    if (ps.ObjectType == 8)//停止线需设置子类型
        obj->set_sub_type(1);

    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < ps.psPtsVec.size(); ++i) {

        auto pt = std::make_shared<data_access_engine::PointProxy>();
        if (ps.ObjectType != 8) //停止线为线型特征
            pt = obj->mutable_border()->mutable_pts()->add();
        else if (ps.ObjectType == 8) //停止线为线型特征
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

std::shared_ptr<data_access_engine::PositionObjectProxy> upload2RoadServer::testCreatPositionProxy(const fsdmap::RoadObjectInfo& rb, fsdmap::dao::DataProcessorBase* dp)
{
    if (rb.list.size() < 2)
        return nullptr;

    //增加对非必要定位要素上传的限制
    if(!FLAGS_upload_non_essential_elements && rb.ele_type == 4){
        return nullptr;
    }

    auto obj = std::make_shared<data_access_engine::PositionObjectProxy>();
    if(rb.ele_type == 5) //人行横道
    {
        obj->set_type(10);
    }
    else if (rb.ele_type == 6) //停止线
    {
       obj->set_type(8);
       obj->set_sub_type(1);
    }
    else if(rb.ele_type == 3 || rb.ele_type == 4) // 箭头或者其他印刷块
    {
        obj->set_type(7);
    }
    else if(rb.ele_type == 10) // 箭头或者其他印刷块
    {
        obj->set_type(6);
    }
    else
    {
        return nullptr;
    }

    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < rb.list.size(); ++i) {
        auto pt = std::make_shared<data_access_engine::PointProxy>();
        if (obj->type()!= 8){
            pt = obj->mutable_border()->mutable_pts()->add();
        } 
        else{
            //停止线为线型特征
            pt = obj->mutable_pole()->mutable_pts()->add();
        } 
            
        
        Eigen::Vector3d wgs;
        auto pp = rb.list[i];
        dp->local2wgs(pp->pos, wgs, true);

        pt->set_x(wgs[0]);
        pt->set_y(wgs[1]);
        pt->set_z(wgs[2]);
        
        pos.X() += wgs[0];
        pos.Y() += wgs[1];
        pos.Z() += wgs[2];
    }
    pos.X() /= rb.list.size();
    pos.Y() /= rb.list.size();
    pos.Z() /= rb.list.size();

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    pMgr->make_new_id(pos, obj, tile, true);
    tile->mutable_position_objects()->push_back(obj);
    return obj;
}

//包装TrafficInfoProxy
std::shared_ptr<data_access_engine::TrafficInfoProxy> upload2RoadServer::CreateTrafficInfoProxy(const Position& ps)
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
    info->mutable_marking()->set_type(2);

    data_access_engine::Vector3D pos = ps.psPtsVec[0];

    pMgr->make_new_id(pos, info, tile, true);
    tile->mutable_traffic_infos()->push_back(info);
    return info;
}

//包装TrafficInfoProxy
std::shared_ptr<data_access_engine::TrafficInfoProxy> upload2RoadServer::testCreateTrafficInfoProxy(fsdmap::RoadObjectInfo& rb, fsdmap::dao::DataProcessorBase* dp) //bev_road_model上传定位要素代码
{
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    if (rb.list.size() < 2){
        return nullptr;
    }
    if(rb.ele_type != 3 && rb.ele_type != 4){
        return nullptr; //不是箭头不处理 并且 不是注意行人
    }
    std::shared_ptr<data_access_engine::PositionObjectProxy> PositionObject = testCreatPositionProxy(rb, dp);
    if (PositionObject == nullptr)
        return nullptr;

    auto info = std::make_shared<data_access_engine::TrafficInfoProxy>();
    info->mutable_objs()->push_back(PositionObject->id());

    // type 4 in traffic is landmark 
    info->set_type(4);
    if(rb.ele_type == 3)
    {   
        auto type_id = std::atoi((rb.type).c_str());
        info->mutable_marking()->set_type(type_id); //使用王乾结果
        rb.traffic_proxy_ptr = info;
    }
    else
    {
        info->mutable_marking()->set_type(12); //默认为注意行人
    }

    Eigen::Vector3d wgs; 
    auto pp = rb.list[0];
    dp->local2wgs(pp->pos, wgs, true);
    data_access_engine::Vector3D pos(wgs[0], wgs[1], wgs[2]);

    pMgr->make_new_id(pos, info, tile, true);
    tile->mutable_traffic_infos()->push_back(info);
    return info;
}

//包装TrafficLightProxy
std::shared_ptr<data_access_engine::TrafficInfoProxy> upload2RoadServer::testCreatTrafficLightProxy(const fsdmap::RoadObjectInfo& rb, fsdmap::dao::DataProcessorBase* dp) //bev_road_model上传交通灯代码
{
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    if(rb.ele_type != 10)
        return nullptr; //不是箭头不处理 并且 不是注意行人
    if (rb.list.size() != 4)
        return nullptr;
    
    //先将数据包装为定位要素
    std::shared_ptr<data_access_engine::PositionObjectProxy> PositionObject = testCreatPositionProxy(rb, dp);
    if (PositionObject == nullptr){
        return nullptr;
    }
    
    //记录pos
    data_access_engine::Vector3D pos(0.0, 0.0, 0.0);
    for (int i = 0; i < rb.list.size(); ++i) {
        Eigen::Vector3d wgs;
        auto pp = rb.list[i];
        dp->local2wgs(pp->pos, wgs, true);
        
        pos.X() += wgs[0];
        pos.Y() += wgs[1];
        pos.Z() += wgs[2];
    }
    pos.X() /= rb.list.size();
    pos.Y() /= rb.list.size();
    pos.Z() /= rb.list.size();

    auto trafficinfo_obj = std::make_shared<data_access_engine::TrafficInfoProxy>();
    trafficinfo_obj->mutable_objs()->push_back(PositionObject->id());
    
    auto light_obj = trafficinfo_obj->mutable_light();
    light_obj->set_type(std::atoi((rb.type).c_str()));
    light_obj->set_bulbs_num(1);
    light_obj->set_arrange(0);

    // 交通信息对应的定位要素类型：
    // 1-交通标志牌；
    // 2-交通灯；
    // 4-地面标志；
    // 5-减速带；
    trafficinfo_obj->set_type(2);
    
    pMgr->make_new_id(pos, trafficinfo_obj, tile, true);
    tile->mutable_traffic_infos()->push_back(trafficinfo_obj);
    return trafficinfo_obj;
}

//包装JunctionProxy vevLines为一个路口内所有道路边界
std::shared_ptr<data_access_engine::JunctionProxy> upload2RoadServer::CreatJunctionProxy(const std::vector<SingleLine>& vevLines)
{
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    auto junction = std::make_shared<data_access_engine::JunctionProxy>();
    for (int j = 0; j < vevLines.size(); ++j)
    {
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRoadBoundary = upload2RoadServer::CreatRoadBoundaryProxy(vevLines[j], 2);
        if (!newRoadBoundary)
            continue;

        junction->mutable_boundarys()->push_back(newRoadBoundary->id());
    }
    if (junction->boundarys().size() > 0)
    {
        data_access_engine::Vector3D pos = vevLines[0].linePtsVec[0];
        pMgr->make_new_id(pos, junction, tile, true);
        tile->mutable_junctions()->push_back(junction);
    }
    return junction;
}

//包装路口内所有不可通行区域
std::shared_ptr<data_access_engine::ImpassableAreaProxy> upload2RoadServer::testCreatImpassableAreaProxy(const fsdmap::ImpassableAreaInfo& impassable_input, fsdmap::dao::DataProcessorBase* dp)
{
    if (impassable_input.list.size() < 3){
        return nullptr;
    }
        
    auto obj = std::make_shared<data_access_engine::ImpassableAreaProxy>();
    obj->set_kind(impassable_input.type);

    for(auto& pt_area : impassable_input.list){
        auto pt = std::make_shared<data_access_engine::PointProxy>();
        pt = obj->mutable_geom()->mutable_pts()->add();

        Eigen::Vector3d wgs;
        dp->local2wgs(pt_area->pos, wgs, true);
        pt->set_x(wgs.x());
        pt->set_y(wgs.y());
        pt->set_z(wgs.z());
    }

    return obj;
}
std::shared_ptr<data_access_engine::JunctionProxy> upload2RoadServer::testCreatJunctionProxy(const fsdmap::JunctionInfo& junction, fsdmap::dao::DataProcessorBase* dp)
{
    int n = junction.lukou_poly_pts.size();
    if(n < 3 || junction.lukou_bd_list.empty())
    {
        return nullptr;
    }

    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    auto junction_proxy = std::make_shared<data_access_engine::JunctionProxy>();
    for(auto& rb : junction.lukou_bd_list)
    {
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRoadBoundary = upload2RoadServer::testCreatRoadBoundaryProxy(*rb, dp);
        if (!newRoadBoundary)
        {
            continue;
        }
        junction_proxy->mutable_boundarys()->push_back(newRoadBoundary->id());     
    }
    
    if(junction_proxy->boundarys().empty()){
        return nullptr;
    }

    for (auto& area : junction.areas)
    {
        std::shared_ptr<data_access_engine::ImpassableAreaProxy> newImpassableArea = testCreatImpassableAreaProxy(*area, dp);
        if(newImpassableArea)
        {
            junction_proxy->mutable_areas()->push_back(newImpassableArea); 
        }
    }
    
    if (junction_proxy->boundarys().size() > 0)
    {
        Eigen::Vector3d wgs;
        Eigen::Vector3d firstPt = junction.lukou_poly_pts[0];
        dp->local2wgs(firstPt, wgs, true);
        data_access_engine::Vector3D pos(wgs.x(), wgs.y(), wgs.z());
        pMgr->make_new_id(pos, junction_proxy, tile, true);
        tile->mutable_junctions()->push_back(junction_proxy);
    }

    return junction_proxy;
}

/******LaneGroupProxy*****/
//包装LaneBoundaryProxy
std::shared_ptr<data_access_engine::LaneBoundaryProxy> upload2RoadServer::CreateLaneBoundaryProxy(const SingleLine& lb)
{
    if (lb.linePtsVec.size() < 2)
        return nullptr;

    auto lane_boundary = std::make_shared<data_access_engine::LaneBoundaryProxy>();
    lane_boundary->set_color(1);
    lane_boundary->set_ldm(false);
    lane_boundary->set_marking(1);
    lane_boundary->set_types({1});

    for (int i = 0; i < lb.linePtsVec.size(); ++i) {
        auto pt = lane_boundary->mutable_geom()->mutable_pts()->add();
        auto pp = lb.linePtsVec[i];

        pt->set_x(pp.X());
        pt->set_y(pp.Y());
        pt->set_z(pp.Z());
    }
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;
    data_access_engine::Vector3D pos =  lb.linePtsVec[0];
    pMgr->make_new_id(pos, lane_boundary, tile, true);
    tile->mutable_lane_boundarys()->push_back(lane_boundary);

    return lane_boundary;
}

//包装LaneBoundaryRangeProxy
void upload2RoadServer::CreateBoundaryRange(data_access_engine::LaneBoundaryRangeProxy*& boundaryRange,
                                const std::shared_ptr<const data_access_engine::LaneBoundaryProxy>& b)
{
    boundaryRange->mutable_bound_id()->set(b->id());
    boundaryRange->mutable_start_pt()->set_x(b->geom()->pts().front()->x());
    boundaryRange->mutable_start_pt()->set_y(b->geom()->pts().front()->y());
    boundaryRange->mutable_start_pt()->set_z(b->geom()->pts().front()->z());
    boundaryRange->mutable_end_pt()->set_x(b->geom()->pts().back()->x());
    boundaryRange->mutable_end_pt()->set_y(b->geom()->pts().back()->y());
    boundaryRange->mutable_end_pt()->set_z(b->geom()->pts().back()->z());
}

//包装LaneGroupProxy
std::shared_ptr<data_access_engine::LaneGroupProxy> upload2RoadServer::CreateLaneGroupProxy(const LaneGroup& vecGroupLanes)
{
    if (vecGroupLanes.lbVec.size() < 2)
        return nullptr;

    auto lane_group = std::make_shared<data_access_engine::LaneGroupProxy>();

    int seq_no = -1;
    data_access_engine::Vector3D laneGroup_left_pos;
    auto pMgr = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    data_access_engine::TileInfoPtr tile;

    std::vector<std::shared_ptr<data_access_engine::LaneBoundaryProxy>> boundarys;

    for(size_t i = 0 ; i < vecGroupLanes.lbVec.size() - 1; i++) //第一个需创建左右
    {
        std::shared_ptr<data_access_engine::LaneBoundaryProxy> left_lane_boundary, right_lane_boundary;
        if(i == 0)
        {
            left_lane_boundary = upload2RoadServer::CreateLaneBoundaryProxy(vecGroupLanes.lbVec[i]);
            boundarys.push_back(left_lane_boundary);

            right_lane_boundary = upload2RoadServer::CreateLaneBoundaryProxy(vecGroupLanes.lbVec[i + 1]);
            boundarys.push_back(right_lane_boundary);
        }
        else if (i > 0 && i < vecGroupLanes.lbVec.size() - 1) //其余数据left为上个数据的right 因此只需要创建right
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

        auto& p0 = left_lane_boundary->geom()->pts().front();
        if(i == 0)
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

        //向tile内添加数据
        tile->mutable_lanes()->push_back(lane);
        lane_group->mutable_lanes()->push_back(lane->id());
        auto version = lane->id()->version();
        lane->mutable_id()->set_version(version + 1);
    }
    pMgr->make_new_id(laneGroup_left_pos, lane_group, tile, true);
    tile->mutable_lane_groups()->push_back(lane_group);
    return lane_group;
}

//任务内所有结果包装proxy
void upload2RoadServer::CreatLaneGroupProxyAll(const std::vector<LaneGroup>& vecLGs)
{
    for (int i = 0; i < vecLGs.size(); ++i) {
        std::shared_ptr<data_access_engine::LaneGroupProxy> newLG = upload2RoadServer::CreateLaneGroupProxy(vecLGs[i]);
    }
}

void upload2RoadServer::CreatRoadBoundaryProxyAll(const std::vector<SingleLine>& vevRBs)
{
    for (int i = 0; i < vevRBs.size(); ++i) {
        std::shared_ptr<data_access_engine::RoadBoundaryProxy> newRB = CreatRoadBoundaryProxy(vevRBs[i], 2);
    }
}

void upload2RoadServer::CreatPositionProxyAll(const std::vector<Position> vecRMs)
{
    for (int i = 0; i < vecRMs.size(); ++i) {
        std::shared_ptr<data_access_engine::PositionObjectProxy> newps = CreatPositionProxy(vecRMs[i]);
    }
}

void upload2RoadServer::CreatTrafficInfoProxyAll(const std::vector<Position> vecRMs)
{
    for (int i = 0; i < vecRMs.size(); ++i) {
        if (vecRMs[i].ObjectType != 7)
            continue; //目前只有地面标识才需要使用CreatPositionProxy
        std::shared_ptr<data_access_engine::TrafficInfoProxy> newTrafficInfo = CreateTrafficInfoProxy(vecRMs[i]);
    }
}

void upload2RoadServer::CreatJunctionProxyAll(const std::vector<std::vector<SingleLine>>& vecVecLines)
{
    for (int i = 0; i < vecVecLines.size(); ++i) {
        std::shared_ptr<data_access_engine::JunctionProxy> newJunction = CreatJunctionProxy(vecVecLines[i]);
    }
}

/*******************************************数据上传**************************************************/
void upload2RoadServer::upLoadTiles(std::string address, std::string road_branch, std::string editor, std::string outputDir)
{
    //上传服务
    auto server_upload = data_access_engine::ConfigAddress::get_instance();
    server_upload->set_road_server_address(address);
    server_upload->set_road_server_upload_branch(road_branch);

    auto talker = data_access_engine::DAEInterface::get_instance()->get_road_geometry_manager();
    //tile 修正前输出
    data_access_engine::TileInfoList up_tiles;
    talker->get_changed_tile_data(up_tiles);
    talker->correct_tiles();
    //talker->correct_tile_refs();

    //tile 修正后输出
    data_access_engine::TileInfoList corr_tiles;
    talker->get_changed_tile_data(corr_tiles);

    //测试输出结果
    WriteBoundarysToObj(corr_tiles, outputDir, "res");
    WriteRoadMarkToObj(corr_tiles, outputDir, "res");

    talker->upload_tiles(editor, corr_tiles, 0);
    CLOG_INFO("upload_tiles finish .......");
}

void upload2RoadServer::WriteBoundarysToObj(const data_access_engine::TileInfoList& tiles, std::string filePath, std::string name)
{
    for(auto tile : tiles)
    {
        int m = 0;
        for(auto lb: tile->lane_boundarys())
        {
            std::string lbOutName = std::to_string(tile->tile_id()) + "_" + name + "_boundary_" + std::to_string(m);
            std::vector<data_access_engine::Vector3D> vecPts;
            for (auto pt: lb->geom()->pts()) {
                vecPts.push_back(data_access_engine::Vector3D(pt->x(), pt->y(), pt->z()));
            }
            WriteToObj(vecPts, filePath, lbOutName);
            m++;
        }
    }
    // for(auto& tile : tiles)
    // {
    //     for (auto& lg : tile->lane_groups())
    //     {
    //         std::cout<<lg->id()->id()<<"前驱车道组：";
    //         for (auto& lg_pre : lg->preds())
    //         {
    //             std::cout<<lg_pre.id()->id();
    //             std::cout << " ";
    //         }
    //         std::cout<<lg->id()->id()<<"后继车道组：";
    //         for (auto& lg_next : lg->succs())
    //         {
    //             std::cout<<lg_next.id()->id();
    //             std::cout << " ";
    //         }
    //     }
    // }
}

void upload2RoadServer::WriteRoadMarkToObj(const data_access_engine::TileInfoList& tiles, std::string path, std::string name)
{
    for(auto tile : tiles)
    {
        int k = 0;
        for(auto obj: tile->position_objects())
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
        for (auto j = 0 ; j < vecVecPts[i].size(); j++)
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
    for (auto i = 0 ; i < vecPts.size(); i++)
    {
        ofs << ++K << " ";
    }
    ofs << std::endl;

    ofs.close();
    return;
}
