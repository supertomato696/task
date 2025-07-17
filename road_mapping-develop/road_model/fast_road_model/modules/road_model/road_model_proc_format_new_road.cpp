

#include "road_model_proc_format_new_road.h"
#include "utils/algorithm_util.h"
#include <core/geometry.h>
#include <core/lane_boundary.h>
#include <core/lane_center.h>
#include <core/lane.h>
#include <core/object.h>
#include <core/lane_group.h>
#include <core/road_boundary.h>
#include <core/generate_id.h>
#include "utils/polyfit.h"

DEFINE_bool(format_new_road_enable, true, "format_new_road_enable");
DEFINE_bool(format_new_road_debug_pos_enable, true, "format_new_road_debug_pos_enable");
DEFINE_bool(format_new_road_filter_green_light, true, "format_new_road_filter_green_light");
DEFINE_bool(format_new_road_save_data_enable, true, "format_new_road_save_data_enable");
DEFINE_bool(format_new_road_object_judgeDis_enable, false, "format_new_road_object_judgedis_enable");
DEFINE_bool(format_new_road_other_object_boundary, true, "format_new_road_other_object_boundary");
DEFINE_bool(refine_stop_line_by_lane_group, true, "refine_stop_line_by_lane_group");

DEFINE_double(format_new_road_max_same_dir_feature_search_radius, 20, "format_new_road_max_same_dir_feature_search_radius    ");
DEFINE_double(format_new_road_max_same_dir_feature_theta, 45, "format_new_road_max_same_dir_feature_theta");
DEFINE_double(format_new_road_max_same_dir_vertical_distance, 50, "format_new_road_max_same_dir_vertical_distance");
DEFINE_double(format_new_road_max_same_dir_length_lambda, 1, "format_new_road_max_same_dir_length_lambda");
DEFINE_double(format_new_road_tree_line_gap, 2, "format_new_road_tree_line_gap");
DEFINE_double(format_new_road_max_line_point_length, 50, "format_new_road_max_line_point_length");
DEFINE_string(format_new_road_traffic_sign_define_json_file, "./conf/traffic_sign_type_define.json", "traffic_sign_define_json_file");
DEFINE_string(format_new_road_obj_output_file, "../../data/", "obj_output_file");

DEFINE_double(format_new_road_high_curvature_gap, 3, "format_new_road_high_curvature_gap");
DEFINE_double(format_new_road_low_curvature_gap, 18, "format_new_road_low_curvature_gap");
DEFINE_double(format_new_road_match_groud_radius, 3, "format_new_road_match_groud_radius");
DEFINE_double(format_new_road_double_move_dis, 0.14, "format_new_road_double_move_dis");

DECLARE_double(format_lane_max_radius_threshold);
DECLARE_double(build_boundary_reline_boudary_by_pos_max_null_length);
DECLARE_double(build_boundary_same_lane_dis_threshold);
DECLARE_double(build_boundary_same_lane_theta_threshold);
// 
DEFINE_double(format_max_alignment_length, 4, "format_max_alignment_length");
DEFINE_double(poss_empty_score, 0.70, "poss_empty_score");

DEFINE_double(stop_line_length_change, 2, "stop_line_length_change"); //断点变化长度阈值
DEFINE_double(boundary_and_lane_gap, 0.5, "boundary_and_lane_gap"); //道路边界车和车道线之间的阈值
DEFINE_double(stop_line_search_boundary_radius, 20, "stop_line_search_boundary_radius"); 
// 

namespace fsdmap
{
    namespace road_model
    {

        fsdmap::process_frame::PROC_STATUS RoadModelProcFormatNewRoad::proc(
            RoadModelSessionData *session)
        {
            auto save_debug2 = [session](int stage) {
                std::string display_name = "format_new_road_" + std::to_string(stage);
                session->set_display_name(display_name.c_str());

                auto log = session->add_debug_log(utils::DisplayInfo::POINT, "center_point");
                int count = 0;
                for (auto rs : session->road_segments)
                {
                    srand48(count++);
                    int label = 1E3 * drand48();
                    for (auto poss : rs->pos_sample_list)
                    {
                        for (auto p : poss->centers)
                        {
                            auto &ele = log->add(p.pos);
                            ele.label.label = label;
                            ele.label.opt_label = 0;
                            const LaneCenterGroupLine *from_center_line = p.from_center_line;
                            if (!from_center_line) {
                                LOG_INFO("from_center_line_DEBUG");
                            }
                        }
                    }
                }

                if (stage ==2 ) {
                    auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "lb");
                    log1->color = {255, 0, 0};
                    for (auto rs : session->road_segments)
                    {
                        srand48(count++);
                        int label = 1E3 * drand48();
                        for (auto poss : rs->pos_sample_list)
                        {
                            for (auto p : poss->centers)
                            {
                                auto &ele = log1->add(p.left_lb.pos);
                                ele.label.label = label;
                                ele.label.opt_label = 0;
                            }
                        }
                    }


                    auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "rlb");
                    log2->color = {0, 255, 0};
                    for (auto rs : session->road_segments)
                    {
                        srand48(count++);
                        int label = 1E3 * drand48();
                        for (auto poss : rs->pos_sample_list)
                        {
                            for (auto p : poss->centers)
                            {
                                auto &ele = log2->add(p.right_lb.pos);
                                ele.label.label = label;
                                ele.label.opt_label = 0;
                            }
                        }
                    }
                }

                for (auto rs : session->road_segments)
                {
                    for (auto lane_group : rs->lane_group_list)
                    {
                        // lane boundary && lane center
                        int n = lane_group->lane_list.size();
                        std::set<fast_road_model::LaneBoundaryPtr> hash_table;
                        for (int i = 0; i < n; i++)
                        {
                            auto lane = lane_group->lane_list[i];
                            auto clog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "center_line_{}", count++);
                        
                            // auto rlog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "right_line_{}",count++);
                            const auto &center = lane->center;
                            const auto &left = lane->left;
                            const auto &right = lane->right;
                            clog->color = {0, 255, 0};
                            // rlog->color={255,255,255};
                            for (auto pt : center->points)
                            {
                                auto &ele = clog->add(pt->pos);
                                ele.label.opt_label = i;
                            }
                            auto lane_boundarys = {left, right};
                            for (auto lane_boundary : lane_boundarys)
                            {
                                if (!lane_boundary)
                                {
                                    continue;
                                }
                                if (hash_table.count(lane_boundary))
                                {
                                    continue;
                                }
                                hash_table.insert(lane_boundary);
                                auto llog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_{}", count++);
                                llog->color = {255, 255, 255};
                                for (auto pt : lane_boundary->points)
                                {
                                    auto &ele = llog->add(pt->pos);
                                    ele.label.opt_label = 0;
                                }
                                if(lane_boundary->points.empty())
                                {
                                    continue;
                                }
                                // 标记是否有前驱后继
                                auto & front=lane_boundary->points.front();
                                auto & back=lane_boundary->points.back();
                                if(lane_boundary->get_prev().empty())
                                {
                                auto plog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "polygon{}", count++);
                                plog->color={255,0,0};
                                for(double theta=0;theta<2*3.14;theta+=3.14/3)
                                {
                                    Eigen::Vector3d pos={0.5*cos(theta),0.5*sin(theta),0};
                                    pos=pos+front->pos;
                                    auto &ele= plog->add(pos);
                                    ele.color={255,0,0};
                                }

                                }
                                if(lane_boundary->get_next().empty())
                                {
                                    auto plog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "polygon{}", count++);
                                    plog->color={0,0,255};
                                    for(double theta=0;theta<2*3.14;theta+=3.14/3)
                                    {
                                    Eigen::Vector3d pos={0.5*cos(theta),0.5*sin(theta),0};
                                    pos=pos+back->pos;
                                    auto &ele= plog->add(pos);
                                    ele.color={0,0,255};
                                    }
                                }
                            }
                        }
                    }
                }
                session->save_debug_info(display_name.c_str());

            };

            // 可视化结果
            CHECK_FATAL_PROC(sample_lane_center_pos(session),"sample_lane_center_pos");
            // 
            CHECK_FATAL_PROC(fix_less_lane(session),"fix_less_lane");
            // save_debug2(2);
            // 
            CHECK_FATAL_PROC(gen_lane_center_line(session),"gen_lane_center_line");
            // save_debug2(3);
            // 
            CHECK_FATAL_PROC(gen_relation(session),"gen_relation");
            // save_debug2(4);
            // 
            CHECK_FATAL_PROC(temp_gen_relation(session),"temp_gen_relation");
            // save_debug2(5);

            // 
            CHECK_FATAL_PROC(delte_sigle_point(session),"delte_sigle_point");

            //
            CHECK_FATAL_PROC(generate_new_stop_line_by_lane_group(session),"generate_new_stop_line_by_lane_group");  //默认关闭
            // save_debug2(7);

            // 
            CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
            // 
            return fsdmap::process_frame::PROC_STATUS_SUCC;
        }
        int RoadModelProcFormatNewRoad::fix_less_lane(RoadModelSessionData *session)
        {
            // session->set_display_name("fix_less_lane");
            // auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_light_lb");
            // log_2->color= {0,255,0};

            for (auto &rs : session->road_segments)
            {
                double n=rs->pos_sample_list.size();
                double score=0;
                for (auto poss : rs->pos_sample_list)
                {
                    // if(poss->centers.empty()&&poss->junction_score>0)
                    if(poss->centers.empty())
                    {
                        score++;
                    }
                }
                score=score/n;
                if(score>FLAGS_poss_empty_score)
                {
                    LOG_INFO("DELTE LANE: {}",score)
                    for (auto &poss : rs->pos_sample_list)
                    {
                        poss->centers.clear();

                        // // debug
                        // auto &ele1=log_2->add(poss->pos);
                        // ele1.color={0,0,255};
                        // ele1.label.label = 1;
                    } 
                }
            }

            // session->save_debug_info("fix_less_lane");
            return fsdmap::SUCC;
        }
        
        int RoadModelProcFormatNewRoad::generate_new_stop_line_by_lane_group(RoadModelSessionData *session)
        {

            //生成车道线的kd-tree
            RTreeProxy<fast_road_model::LBPoint*, float, 2> lane_line_topo_pts_tree;
            // RTreeProxy<fast_road_model::LBPointPtr, float, 2> lane_line_topo_pts_tree;
            for (auto rs : session->road_segments)
            {
                for (auto lane_group : rs->lane_group_list)
                {
                    for (auto lane: lane_group->lane_list)
                    {                       
                        for(auto side : {lane->left, lane->right}){
                            for(auto pt : side->points){
                                lane_line_topo_pts_tree.insert(pt->pos, pt.get());
                            }
                        }
                    }
                }
            }

            auto get_lane_point=[](const std::vector<fast_road_model::LaneGroupPtr> &group_list, bool is_start, bool is_left, Eigen::Vector3d &pos)->bool 
            {
                if (group_list.empty())
                {
                    return false;
                }
                auto &group = group_list.front();
                if(!group)
                {
                    return false;
                }
                auto &lane_list=group->lane_list;
                if (lane_list.empty())
                {
                    return false;
                }
                // auto &lane=is_start?lane_list.front():lane_list.back();
                auto &lane = is_left ? lane_list.front() : lane_list.back();
                auto &line = is_left ? lane->left : lane->right;
                if (!line)
                {
                    LOG_WARN("缺失车道边界")
                    return false;
                }
                auto &points = line->points;
                if (points.empty())
                {
                    return false;
                }
                auto &point = is_start ? points.front() : points.back();
                if (!point)
                {
                    return false;
                }
                pos = point->pos;
                return true;
            };
            auto get_lane_endpoint = [get_lane_point](const fast_road_model::RoadSegment* rs, bool is_start, Eigen::Vector3d &left_pos, Eigen::Vector3d &right_pos)->bool 
            {
                if (!rs)
                {
                    return false;
                }
                bool left = get_lane_point(rs->lane_group_list, is_start, true, left_pos);
                bool right = get_lane_point(rs->lane_group_list, is_start, false, right_pos);
                return  left&&right;

            };

            auto search_closest_lane_line = [&lane_line_topo_pts_tree](Eigen::Vector3d &bd_pos, Eigen::Vector3d &lane_line_pt)->bool
            {
                std::vector<fast_road_model::LBPoint*> lane_pts;
           
                lane_line_topo_pts_tree.search(bd_pos, FLAGS_boundary_and_lane_gap, lane_pts);
                 
                double dis = std::numeric_limits<double>::max();
                if(!lane_pts.empty()){
                    for (const auto& lane_pt : lane_pts) {
                        double current_dis = alg::calc_dis(lane_pt->pos, bd_pos);
                        if (current_dis < dis) {
                            dis = current_dis;
                            lane_line_pt = lane_pt->pos;
                        }
                    }
                    return true;
                }else{
                    return false;
                }
       
            };

            auto correct_stop_line_endpoints = [search_closest_lane_line](RoadModelSessionData *session, RoadObjectInfo* obj, KeyPose* keypose, const Eigen::Vector3d& origin_left_pos, const Eigen::Vector3d& origin_right_pos, std::string flag) 
            {
                LOG_INFO("correct_stop_line_endpoints")
                double stop_line_extend = 7;
                Eigen::Vector3d left_pt_et = origin_left_pos - obj->dir * stop_line_extend;
                Eigen::Vector3d right_pt_et = origin_right_pos + obj->dir * stop_line_extend;
                Eigen::Vector3d left_bd{0, 0, 0}; 
                Eigen::Vector3d right_bd{0, 0, 0}; 


                //1. 找道路边界
                std::vector<BoundaryFeature*> bd_pts;
                session->boundary_line_sample_tree.search(obj->pos, FLAGS_stop_line_search_boundary_radius, bd_pts);

                std::unordered_set<BoundaryGroupLine*> bd_lines;
                for(auto& bd_pt : bd_pts){
                    bd_lines.insert(bd_pt->group_line);
                }

                //找和全部线的交点
                std::vector<Eigen::Vector3d> bd_cross_points;
                for(const auto& bd_line : bd_lines){
                    std::vector<UsefulPnt> res;
                    bool is_intersect = alg::get_cross_point_with_curve_segment(left_pt_et, right_pt_et, bd_line->list, res, false, 0, 0, true);
                    for(auto& pt : res){
                        bd_cross_points.push_back(pt.pos);
                    }
                }

                //以keypose为中心，分左右
                std::vector<Eigen::Vector3d> left_bd_cross_points, right_bd_cross_points;
                for(auto& pt : bd_cross_points){
                    if(alg::judge_left2(pt, obj->pos, keypose->dir) == 1){
                        left_bd_cross_points.push_back(pt);
                    }else{
                        right_bd_cross_points.push_back(pt);
                    }
                }

                //计算左右的距离keypose最近点的
                
                //2.检查附近是否有车道线
                if(flag == "left"){
                    //左
                    if(left_bd_cross_points.size() > 0){
                        left_bd = alg::find_closest_point(left_bd_cross_points, keypose->pos);
                    }
                    
                    Eigen::Vector3d fix_left_stop_line_pos{0, 0 ,0};
                    if(left_bd.norm() > 1e-6){
                        Eigen::Vector3d left_lane_pt;
                        fix_left_stop_line_pos = left_bd;
                        bool res = search_closest_lane_line(left_bd, left_lane_pt);
                        if(res == true){
                            fix_left_stop_line_pos = left_lane_pt;
                        }
                        
                    }else{
                        fix_left_stop_line_pos = origin_left_pos;
                    }
                    
                    return  fix_left_stop_line_pos;
                }else{
                    //右
                    if(right_bd_cross_points.size() > 0){
                        right_bd = alg::find_closest_point(right_bd_cross_points, keypose->pos);
                    }
                    Eigen::Vector3d fix_right_stop_line_pos{0, 0 ,0};
                    if(right_bd.norm() > 1e-6){
                        Eigen::Vector3d right_lane_pt;
                        fix_right_stop_line_pos = right_bd;
                        bool res = search_closest_lane_line(right_bd, right_lane_pt);
                        if(res == true){
                            fix_right_stop_line_pos = right_lane_pt;
                        }
                        
                    }else{
                        fix_right_stop_line_pos = origin_right_pos;
                    }

                    return  fix_right_stop_line_pos;
                }

            };

            auto refine_stop_line_end_point=[get_lane_endpoint,correct_stop_line_endpoints](RoadModelSessionData *session, fast_road_model::RoadSegment* rs,KeyPose * poss, bool is_start)
            {
               
                if (!poss)
                {
                    return;
                }
                auto &road_break = poss->road_break;
                if (road_break)
                {
                    if(road_break->objs.empty())
                    {
                        return ;
                    }
                    Eigen::Vector3d left_pos;
                    Eigen::Vector3d right_pos;
                    bool status = get_lane_endpoint(rs, is_start, left_pos, right_pos);
                    if (status)
                    {
                        for (auto &obj : road_break->objs)
                        {
                            if (obj->ele_type == 6)
                            {
                                // LOG_INFO("start refine stop line end point")
                                if(FLAGS_refine_stop_line_by_lane_group == true){
                                    //确定原始左右点
                                    auto& origin_left_pos = obj->list[0]->pos;  
                                    auto& origin_right_pos = obj->list[1]->pos;
                                    if(alg::judge_left2(obj->list[0]->pos, poss->pos, poss->dir) == 1){
                                    }else{
                                        auto origin_left_pos_tmp = obj->list[1]->pos;
                                        auto origin_right_pos_tmp = obj->list[0]->pos;
    
                                        origin_left_pos = origin_left_pos_tmp;
                                        origin_right_pos= origin_right_pos_tmp;
                                        obj->dir = alg::get_dir(origin_right_pos, origin_left_pos);
                                    }
                                 
    
                                    //1.车道数和link不一致，重新校验生成端点
                                    Eigen::Vector3d v_origin = origin_right_pos - origin_left_pos;
                                    Eigen::Vector3d v_new = right_pos - left_pos;
                                    double theta = alg::calc_theta1(v_origin, v_new, true, true);
                                    if(rs->lane_group_list.size() > 0){
                                        LOG_INFO("link.size() : {}, lane.size() : {}, theta:{}", poss->from_raw_link->lanenum_sum, rs->lane_group_list.back()->lane_list.size(), theta);
                                        // if(poss->from_raw_link->lanenum_sum != poss->lane_group->lane_line_info.size()){
                                        if(poss->from_raw_link->lanenum_sum != rs->lane_group_list.back()->lane_list.size() || theta > 10){
                                            if(alg::calc_dis(left_pos, origin_left_pos) > FLAGS_stop_line_length_change){
                                                // LOG_INFO("left");
                                                Eigen::Vector3d res = correct_stop_line_endpoints(session, obj, poss, left_pos, right_pos, "left");
                                                //2次校验
                                                if(alg::calc_dis(res, origin_left_pos) > FLAGS_stop_line_length_change){
                                                    left_pos = origin_left_pos; //用回最初始的
                                                    LOG_INFO("use left origin stop, left {} {} {} right {} {} {}", left_pos.x(), left_pos.y(), left_pos.z(), right_pos.x(), right_pos.y(), right_pos.z());
                                                }else{
                                                    left_pos = res;
                                                    LOG_INFO("use left calculate stop, left {} {} {} right {} {} {}", left_pos.x(), left_pos.y(), left_pos.z(), right_pos.x(), right_pos.y(), right_pos.z());
                                                }
                                            }
                                            
                                            if(alg::calc_dis(right_pos, origin_right_pos) > FLAGS_stop_line_length_change){
                                                // LOG_INFO("right");
                                                // LOG_INFO("1 left {} {} {} right {} {} {}", left_pos.x(), left_pos.y(), left_pos.z(), right_pos.x(), right_pos.y(), right_pos.z());
                                            
                                                Eigen::Vector3d res = correct_stop_line_endpoints(session, obj, poss, left_pos, right_pos, "right");
                                                //2次校验
                                                if(alg::calc_dis(res, origin_right_pos) > FLAGS_stop_line_length_change){
                                                    right_pos = origin_right_pos;  //用回最初始的
                                                    LOG_INFO("use right origin stop, left {} {} {} right {} {} {}", left_pos.x(), left_pos.y(), left_pos.z(), right_pos.x(), right_pos.y(), right_pos.z());
                                                }else{
                                                    right_pos = res;
                                                    LOG_INFO("use right calculate stop, left {} {} {} right {} {} {}", left_pos.x(), left_pos.y(), left_pos.z(), right_pos.x(), right_pos.y(), right_pos.z());
    
                                                }
                                            }
                                        }
                                    }
                                    origin_left_pos = left_pos;
                                    origin_right_pos = right_pos;
                                }else{
                                    auto &ps = obj->list[0]->pos;
                                    auto &pe = obj->list[1]->pos;
                                    ps = left_pos;
                                    pe = right_pos;
                                }


                            }
                                // for(auto pos:rs->pos_sample_list)
                                // {
                                //     auto nroad_break= pos->road_break;
                                //     // LOG_INFO("OBJS:nroad_break {}",uint64_t(nroad_break));
                                //     if(nroad_break)
                                //     LOG_INFO("OBJS:nroad_break {} {}",nroad_break->reasons,nroad_break->objs.size());
                                // }
                            
                        }
                    }
                }
            };


            for (auto &rs : session->road_segments)
            {
                if (rs->pos_sample_list.empty())
                {
                    continue;
                }
                //  TODO delet length to small lane group
                // auto &front = rs->pos_sample_list.front();
                auto &back = rs->pos_sample_list.back();
                // refine_stop_line_end_point(rs,front, true);
                refine_stop_line_end_point(session, rs,back, false);
            }
            return fsdmap::SUCC;
        }


        int RoadModelProcFormatNewRoad::delte_sigle_point(RoadModelSessionData *session)
        {
            auto delte_road_boundary = [](fast_road_model::RoadBoundaryPtr &rb)
            {
                if (!rb)
                {
                    return;
                }
                int count = 0;
                for (auto p : rb->points)
                {
                    if (p)
                    {
                        count++;
                    }
                }
                if (count < 2)
                {
                    // LOG_WARN("road_boundary one point:{} size:{}", uint64_t(rb.get()), count)
                    rb.reset();
                    rb = nullptr;
                }
            };
            auto delte_lane_boundary = [](fast_road_model::LaneBoundaryPtr &lb)
            {
                if (!lb)
                {
                    return;
                }
                int count = 0;
                for (auto p : lb->points)
                {
                    if (p)
                    {
                        count++;
                    }
                }
                if (count < 2)
                {
                    LOG_WARN("lane_boundary one point:{} size:{}", uint64_t(lb.get()), count)
                    lb.reset();
                    lb = nullptr;
                }
            };

            for (auto &rs : session->road_segments)
            {
                for (auto &lane_group : rs->lane_group_list)
                {
                    delte_road_boundary(lane_group->left_road_boundary);
                    delte_road_boundary(lane_group->right_road_boundary);
                    for (auto &lane : lane_group->lane_list)
                    {
                        if(!lane)
                        {
                            continue;
                        }
                        delte_lane_boundary(lane->left);
                        delte_lane_boundary(lane->right);
                    }
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::gen_lane_center_line(RoadModelSessionData *session)
        {
            auto alignment= [session](fast_road_model::RoadSegment *rs, std::vector<HitCenterPoint> &line, double max_dis,bool is_start)
            {
                // TODO ALIGNMENT LEFT RIGHT
                using namespace fast_road_model;
                const auto start = rs->pos_sample_list.front();
                const auto end = rs->pos_sample_list.back();
                
                auto rss_p1 = alg::get_hori_pos(start->pos, start->road_vertical_dir, +30);
                auto rss_p2 = alg::get_hori_pos(start->pos, start->road_vertical_dir, -30);

                auto rse_p1 = alg::get_hori_pos(end->pos, end->road_vertical_dir, +30);
                auto rse_p2 = alg::get_hori_pos(end->pos, end->road_vertical_dir, -30);

                // static int g_cnt = 0;
                // if (start->road_break) {
                //     rss_p1 = alg::get_hori_pos(start->road_break->pos, start->road_vertical_dir, +30);
                //     rss_p2 = alg::get_hori_pos(start->road_break->pos, start->road_vertical_dir, -30);
                //     if (g_cnt < 50) {
                //         std::cout << " 1 rss_p1 " << rs << " " << start->road_break->pos.transpose() << " " << start->pos.transpose() << std::endl;
                //     }
                // }
                
                // if(end->road_break) {
                //     rse_p1 = alg::get_hori_pos(end->road_break->pos, end->road_vertical_dir, +30);
                //     rse_p2 = alg::get_hori_pos(end->road_break->pos, end->road_vertical_dir, -30);
                //     if (g_cnt < 50) {
                //         std::cout << " 2 rse_p1 " << rs << " " << end->road_break->pos.transpose() << " " << end->pos.transpose() << std::endl;
                //     }
                // }
                // g_cnt ++;

                std::vector<Eigen::Vector3d > vertical_points;
                //  在末端点延长10m
                const int n = line.size();
                if (n < 2)
                {
                    return;
                }
                std::vector<std::vector<Eigen::Vector3d *>> points;
                if(is_start)
                {
                    points = {{&line[0].pos, &line[1].pos},
                    {&line[0].left_lb.pos, &line[1].left_lb.pos},
                    {&line[0].right_lb.pos, &line[1].right_lb.pos}}; //
                    vertical_points={rss_p1,rss_p2};
                }else
                {
                    points = {{&line[n-1].pos, &line[n-2].pos},
                    {&line[n-1].left_lb.pos, &line[n-2].left_lb.pos},
                    {&line[n-1].right_lb.pos, &line[n-2].right_lb.pos}}; //
                    vertical_points={rse_p1,rse_p2};
                }
                int cnt = 0;
                for(auto &point:points)
                {
                    Eigen::Vector3d dir_s = alg::get_dir(*point[0], *point[1]);

                    // if (start->road_break && start->road_break->reasons=="split_merge" 
                    //     && end->road_break && end->road_break->reasons=="split_merge") {
                    //     auto road_dir = alg::get_vertical_dir(start->road_vertical_dir);
                    //     if(alg::calc_theta1(road_dir, dir_s) > 90) {
                    //         road_dir = -road_dir;
                    //     }
                    //     dir_s = road_dir;
                    //     // return;
                    // }

                    Eigen::Vector3d psc1 =*point[1];
                    Eigen::Vector3d psc2 =*point[0] + max_dis * dir_s;
                    //
                    Eigen::Vector3d refine;
                    // TODO check why  fail so many point
                    if (alg::findIntersection(vertical_points[0], vertical_points[1], psc1, psc2, refine, 2))
                    {
                        *point[0] = refine;  // 更新拉长后的中心线、左右边界线到输入line中

                        // if(cnt == 0) {
                        //     std::vector<BreakInfo*> bk_points;
                        //     double search_radius = 0.5;
                        //     session->sm_break_candidate_tree.search(refine, search_radius, bk_points);
                        //     if (bk_points.size() > 0) {
                        //         *point[0] = bk_points[q0]->pos; // 分合流交叉点，强制赋值
                        //     }
                        // }
                    }
                    else {
                        // LOG_WARN("refine start pos faile at[{} {}]", ps1.x(), ps1.y())
                    }
                    cnt++;
                }
            };

            // 根据曲率简化线上的点; 曲率小时采样距离大，曲率大时采样距离小;(只是从原始的中心线中根据曲率大小筛选点)
            // TODO: 目前分别用的 18米 和 3米, 阈值是否满足下游生产要求
            auto sparse_points_by_curvature = [](std::vector<HitCenterPoint> &line) -> std::vector<HitCenterPoint>
            {
                std::vector<HitCenterPoint> ret;
                Eigen::Vector3d prev_pos;
                bool use_low_curvature = true;
                double res_dis = 0;
                double min_curvature = 0.03;
                for (auto p : line)
                {
                    double cur_curvature = p.max_curvature;
                    if (ret.empty())
                    {
                        ret.push_back(p);
                        prev_pos = p.pos;
                        use_low_curvature = cur_curvature < min_curvature ? true : false;
                        // low：18， high：3米
                        res_dis = use_low_curvature ? FLAGS_format_new_road_low_curvature_gap : FLAGS_format_new_road_high_curvature_gap;
                        continue;
                    }
                    // 最后一个一定放进去
                    double to_back = alg::calc_dis(p.pos, line.back().pos);
                    double d = alg::calc_dis(prev_pos, p.pos);
                    res_dis = res_dis - d;
                    // LOG_INFO("res_dis:{} d:{} to_back:{}| cur_curvature:{}  use_low_curvature:{}", res_dis, d, to_back, cur_curvature, use_low_curvature)
                    if (to_back <= 0 || res_dis <= 0)
                    {
                        ret.push_back(p);
                        prev_pos = p.pos;
                        use_low_curvature = cur_curvature < min_curvature ? true : false;
                        res_dis = use_low_curvature ? FLAGS_format_new_road_low_curvature_gap : FLAGS_format_new_road_high_curvature_gap;
                        continue;
                    }
                    if (use_low_curvature && cur_curvature >   min_curvature)
                    {
                        ret.push_back(p);
                        prev_pos = p.pos;
                        use_low_curvature = false;
                        res_dis = FLAGS_format_new_road_high_curvature_gap;
                        continue;
                    }
                    // TODO find center sparse point
                    prev_pos = p.pos;
                }
                return ret;
            };
            auto road_segment_add_lane_group = [](bool first, 
                fast_road_model::RoadSegment *rs,
                 const std::vector<HitCenterPoint> &line)->fast_road_model::LanePtr
            {
                auto &gid=fast_road_model::GenerateID::getInstance();
                using namespace fast_road_model;
                auto &lane_group_list = rs->lane_group_list;
                // 第一条或者空需要创建 （每個rs第一次進來都是為空的）
                LaneGroupPtr new_lane_group = nullptr;
                if (first || lane_group_list.empty())
                {
                    new_lane_group = std::make_shared<LaneGroup>();  
                    new_lane_group->id=gid.update(GenerateID::TYPE::LANE_GROUP);
                    // 
                    if(!lane_group_list.empty())
                    {
                        auto &back=lane_group_list.back();
                        // lane_group的前驱后继关系
                        back->add_next(new_lane_group.get());
                        new_lane_group->add_prev(back.get());
                    }
                    // 
                    lane_group_list.push_back(new_lane_group);
                    // 看代碼一個rs，只有一個 lane_group_list 
                    // TODO：同一个roadsegment会执行几次，log测试下
                }
                else
                {
                    new_lane_group = lane_group_list.back();
                }
                // 创建其中一条lane
                auto &lane_list = new_lane_group->lane_list;
                LanePtr one_lane = std::make_shared<Lane>();
                auto lane_id=gid.update(fast_road_model::GenerateID::TYPE::LANE);
                one_lane->id=lane_id;
                lane_list.push_back(one_lane);
                //
                auto &center = one_lane->center;
                center = std::make_shared<LaneCenter>();
                center->id=lane_id;
                auto &left = one_lane->left;
                left = std::make_shared<LaneBoundary>();
                left->id=gid.update(fast_road_model::GenerateID::TYPE::LANE_BOUNDARY);
                auto &right = one_lane->right;
                right = std::make_shared<LaneBoundary>();
                right->id=gid.update(fast_road_model::GenerateID::TYPE::LANE_BOUNDARY);
                auto &cpoints = center->points;
                auto &lpoints = left->points;
                auto &rpoints = right->points;
                const int n=line.size();
                cpoints.resize(n);
                lpoints.resize(n);
                rpoints.resize(n);
                if(n<2)
                {
                    LOG_WARN("SINGLE POINT");
                    return one_lane;
                }
                std::unordered_map<int, int> l_type_freq, r_type_freq, l_color_freq, r_color_freq;
                int l_max_count=0, l_max_element=0, r_max_count=0, r_max_element=0;
                int l_max_count_color=0, l_max_element_color=0, r_max_count_color=0, r_max_element_color=0;
                for(int i=0;i<n;i++)
                {
                    auto pt=line[i];
                    // center
                    cpoints[i] = std::make_shared<LCPoint>();
                    cpoints[i]->init(pt.pos, pt.dir);
                    cpoints[i]->from_lane_center=center.get();
                    cpoints[i]->from_lane_group=new_lane_group.get();
                    // left 
                    lpoints[i] = std::make_shared<LBPoint>();
                    lpoints[i]->init(pt.left_lb.pos, pt.left_lb.dir);
                    lpoints[i]->from_lane_boundary=left.get();
                    lpoints[i]->from_lane_group=new_lane_group.get();

                    int l_current_count = ++l_type_freq[pt.left_lb.type];
                    if (l_current_count > l_max_count) {
                        l_max_count = l_current_count;
                        l_max_element = pt.left_lb.type;
                    }
                    int l_current_count_color = ++r_color_freq[pt.left_lb.color];
                    if (l_current_count_color > l_max_count_color) {
                        l_max_count_color = l_current_count_color;
                        l_max_element_color = pt.left_lb.color;
                    }

                    // right
                    rpoints[i] = std::make_shared<LBPoint>();
                    rpoints[i]->init(pt.right_lb.pos, pt.right_lb.dir);
                    rpoints[i]->from_lane_boundary=right.get();
                    rpoints[i]->from_lane_group=new_lane_group.get();
                    
                    int r_current_count = ++r_type_freq[pt.right_lb.type];
                    if (r_current_count > r_max_count) {
                        r_max_count = r_current_count;
                        r_max_element = pt.right_lb.type;
                    }
                    int r_current_count_color = ++r_color_freq[pt.right_lb.color];
                    if (r_current_count_color > r_max_count_color) {
                        r_max_count_color = r_current_count_color;
                        r_max_element_color = pt.right_lb.color;
                    }
                }

                left->type = l_max_element;
                right->type = r_max_element;
                left->color = l_max_element_color;
                right->color = r_max_element_color;

                // std::cout<<"--**-- [color]  left: "<<left->color<<", right: "<<right->color<<std::endl;
                //
                // LOG_INFO("points:{}", n)
               return  one_lane;
            };

            auto sort_lane=[](fast_road_model::RoadSegment*rs)
            {   
                // 以随机第一条中心线的中心点为基准，根据道路垂直方向的距离进行排序
                 for(auto &lane_group:rs->lane_group_list)
                 {
                   auto front_lane=lane_group->lane_list.front();
                   auto front_center=front_lane->center;
                   int n=front_center->points.size();
                   auto base_point=front_center->points[n*0.5];
                   std::sort(lane_group->lane_list.begin(),lane_group->lane_list.end(),[base_point](fast_road_model::LanePtr a ,fast_road_model::LanePtr b){
                    auto ca=a->center;
                    auto cb=b->center;
                    int na=ca->points.size();
                    int nb=cb->points.size();
                    auto ba=ca->points[na*0.5];
                    auto bb=cb->points[nb*0.5];
                    return fsdmap::alg::calc_vertical_dis(ba->pos,base_point->pos,base_point->dir,true)< fsdmap::alg::calc_vertical_dis(bb->pos,base_point->pos,base_point->dir,true);
                });}

                 for (auto &lane_group : rs->lane_group_list)
                 {
                     auto front_lane = lane_group->lane_list.front();
                     auto front_center = front_lane->center;
                     int n = front_center->points.size();
                     if (n == 0)
                     {
                         LOG_WARN("front_center:{}", n);
                         continue;
                     }
                     auto base_point = front_center->points[n * 0.5];
                     const int lane_num=lane_group->lane_list.size();
                     for (int i = 0; i < lane_num ; i++)
                     {
                         auto ca = lane_group->lane_list[i]->center;
                         int na = ca->points.size();
                         int index=na*0.5;
                         auto ba = ca->points[index];
                         double dis = fsdmap::alg::calc_vertical_dis(ba->pos, base_point->pos, base_point->dir, true);
                        //  LOG_INFO("out DIS[{}/{}]:[{}]", i, lane_num, dis);
                     }
                 }

            };
            auto merge_lane_boundary = [](fast_road_model::LaneGroupPtr lane_group)
            {
                auto is_sample_line = [](fast_road_model::LaneBoundaryPtr a, fast_road_model::LaneBoundaryPtr b)->bool 
                {
                    if (!a || !b)
                    {
                        return false;
                    }
                     int m = b->points.size();
                     int n = a->points.size();
                     n=std::min(m,n);
                    if (n==0)
                    {
                        // LOG_WARN("lane boundary point size not equal[{} -- {}]",n,m);
                        return false;
                    }
                   
                    double h_dis = 0;
                    for (int i = 0; i < n; i++)
                    {
                        h_dis += alg::calc_vertical_dis(a->points[i]->pos, b->points[i]->pos, b->points[i]->dir);
                    }
                    h_dis /= static_cast<double>(n);
                    if (h_dis < 0.5)
                    {
                        // LOG_INFO("h_dis TRUE:{}",h_dis);
                        return true;
                    }
                    // LOG_INFO("h_dis FALSE:{}",h_dis);
                    return false;
                };
                
        

                auto &lane_list = lane_group->lane_list;
                const int n = lane_list.size();

                // for (int i = 0; i < n; i++)
                // {
                //     auto lane=lane_list[i ];
                //     LOG_INFO( "B [L:{}  R:{}]",uint64_t(lane->left.get()),uint64_t(lane->right.get()))
                // }

                for (int i = 1; i < n; i++)
                {
                    if (is_sample_line(lane_list[i - 1]->right, lane_list[i]->left))
                    {
                        // lane_list[i - 1]->right.reset();
                        // lane_list[i - 1]->right = lane_list[i ]->left;
                        lane_list[i]->left.reset();
                        lane_list[i]->left = lane_list[i-1]->right;
                        // TODO yx：保留更长的线
                    }else
                    {
                        // LOG_INFO("not is_sample_line");
                    }
                }
                // for (int i = 0; i < n; i++)
                // {
                //     auto lane=lane_list[i ];
                //     LOG_INFO( "A [L:{}  R:{}]",uint64_t(lane->left.get()),uint64_t(lane->right.get()))
                // }
            };
             auto gen_lane_boundary = [](fast_road_model::RoadSegment *rs, std::map<fast_road_model::LanePtr, std::vector<HitCenterPoint> *> &lane_to_line_map)
            {
                auto &gid=fast_road_model::GenerateID::getInstance();
                auto &lgs = rs->lane_group_list;
                for (auto &lg : lgs)
                {
                    auto &lane_list = lg->lane_list;
                    if (lane_list.empty())
                    {
                        continue;
                    }
                    auto &left_road_boundary = lg->left_road_boundary;
                    if (!left_road_boundary)
                    {
                        left_road_boundary = std::make_shared<fast_road_model::RoadBoundary>();
                        left_road_boundary->id=gid.update(fast_road_model::GenerateID::TYPE::ROAD_BOUNDARY);
                    }
                    auto &right_road_boundary = lg->right_road_boundary;
                    if (!right_road_boundary)
                    {
                        right_road_boundary = std::make_shared<fast_road_model::RoadBoundary>();
                        right_road_boundary->id=gid.update(fast_road_model::GenerateID::TYPE::ROAD_BOUNDARY);
                    }

                    std::unordered_map<int, int> l_frequency, r_frequency;
                    int l_max_count=0, l_max_element=0, r_max_count=0, r_max_element=0;
                    
                    // 针对排序后的lane_list，取最左边和最右边的lane信息来生成道路边界
                    auto &left_lane = lane_list.front();
                    if (lane_to_line_map.count(left_lane))
                    {
                        auto line = *lane_to_line_map.at(left_lane);
                        auto &points = left_road_boundary->points;
                        for (auto hitcenter : line)
                        {
                            auto from_raw_center_feature = hitcenter.from_raw_center_feature;
                            if (!from_raw_center_feature)
                            {
                                LOG_WARN("from_raw_center_feature");
                            }
                            auto rbl = from_raw_center_feature->left_rb;
                            // 取原始的道路边界线，若原来没有则没有补点
                            if (!rbl)
                            {
                                continue;
                            }
                            fast_road_model::RBPointPtr rbp = std::make_shared<fast_road_model::RBPoint>();
                            rbp->init(rbl->pos, rbl->dir);
                            points.push_back(rbp);

                            int l_current_count = ++l_frequency[rbl->type];
                            if (l_current_count > l_max_count) {
                                l_max_count = l_current_count;
                                l_max_element = rbl->type;
                            }
                        }
                        left_road_boundary->type = l_max_element;
                    }
                    auto &right_lane = lane_list.back();
                    if (lane_to_line_map.count(right_lane))
                    {
                        auto line = *lane_to_line_map.at(right_lane);
                        auto &points = right_road_boundary->points;
                        for (auto hitcenter : line)
                        {
                            auto from_raw_center_feature = hitcenter.from_raw_center_feature;
                            if (!from_raw_center_feature)
                            {
                                LOG_WARN("from_raw_center_feature");
                            }
                            auto rbr = from_raw_center_feature->right_rb;
                            if (!rbr)
                            {
                                continue;
                            }
                            fast_road_model::RBPointPtr rbp = std::make_shared<fast_road_model::RBPoint>();
                            rbp->init(rbr->pos, rbr->dir);
                            points.push_back(rbp);
                            int r_current_count = ++r_frequency[rbr->type];
                            if (r_current_count > r_max_count) {
                                r_max_count = r_current_count;
                                r_max_element = rbr->type;
                            }
                        }
                        right_road_boundary->type = r_max_element;
                    }
                }
            };
            auto gen_lane= [alignment,sparse_points_by_curvature,
                 road_segment_add_lane_group,sort_lane,gen_lane_boundary](fast_road_model::RoadSegment *rs)
            {
                using namespace fast_road_model;
                // TODO spild for mutil lane group
                std::map<LaneCenterGroupLine *, std::vector<HitCenterPoint>> line_list;
                
                // 建立RoadSegment中所有from_center_line和对应的center_point，保存到line_list
                for (const auto &poss : rs->pos_sample_list) 
                {
                    for (const auto &center : poss->centers)
                    {
                        LaneCenterGroupLine *from_center_line = center.from_center_line;
                        if (!from_center_line)
                        {
                            LOG_INFO("from_center_line NULL");
                        }
                        else
                        {
                            line_list[from_center_line].push_back(center);
                        }
                    }
                }

                // 剔除单点的中心线
                for (auto iter = line_list.begin(); iter != line_list.end(); )
                {
                    if (iter->second.size() < 2) //约
                    {
                        iter = line_list.erase(iter);
                    }else
                    {
                        iter++;
                    }

                }

                //  sparase 
                // 根据曲率简化线上的点; 曲率小时采样距离大，曲率大时采样距离小;(只是从原始的中心线中根据曲率大小筛选点)
                for(auto iter = line_list.begin(); iter != line_list.end(); iter++)
                {
                    iter->second=sparse_points_by_curvature (iter->second);
                }
                // 

                int count = 0;
                std::map<LanePtr,std::vector<HitCenterPoint>*> lane_to_line_map;
                for (auto &mline : line_list)
                {
                    bool is_first = (count == 0);
                    auto &line = mline.second;
                    alignment(rs, line, FLAGS_format_max_alignment_length*2,true); // 4*2 = 8
                    alignment(rs, line, FLAGS_format_max_alignment_length*2,false);
                    // 生成单个车道中心线及其車道边界
                    auto lane=road_segment_add_lane_group(is_first, rs, line);
                    lane_to_line_map[lane]=&line;
                    count++;
                }
                // 车道排序
                sort_lane(rs);
                // 生成道路边界
                gen_lane_boundary(rs,lane_to_line_map);
            };
            // TODO USE MORTPOINT 
            for (auto rs : session->road_segments)
            {
                // 
                gen_lane(rs);
                // 合并车道边界到单一对象
                for(auto &lane_group:rs->lane_group_list)
                {
                    merge_lane_boundary(lane_group);
                }
            }
            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::gen_relation(RoadModelSessionData *session)
        {
            auto connect_road_boundary = [](fast_road_model::RoadBoundary *prev, fast_road_model::RoadBoundary *cur)
            {
                if (!prev || !cur)
                {
                    return;
                }
                if (prev->points.empty() || cur->points.empty())
                {
                    return;
                }
                auto back_point = prev->points.back();
                auto front_point = cur->points.front();
                // TODO change point to same
                if (alg::calc_dis(back_point->pos, front_point->pos) < 0.5) // 25cm
                {
                    prev->add_next(cur);
                    cur->add_prev(prev);
                }
            };

            auto connect_lane = [](fast_road_model::Lane *prev, fast_road_model::Lane *cur)
            {
                auto set_lane_boundary_connect = [](fast_road_model::LaneBoundaryPtr &prev, fast_road_model::LaneBoundaryPtr &next)
                {
                    if (!prev || !next)
                    {
                        return false;
                    }
                    auto &prev_points = prev->points;
                    auto &next_points = next->points;
                    if (prev_points.empty() || next_points.empty())
                    {
                        return false;
                    }
                    auto &prev_back_point = prev_points.back();
                    auto &cur_front_point = next_points.front();
                    if(alg::calc_dis(cur_front_point->pos, prev_back_point->pos) > 0.5||
                    alg::calc_vertical_dis(cur_front_point->pos, prev_back_point->pos,prev_back_point->dir) > 0.25)
                    {
                      return false;
                    }
                    prev_back_point->pos = cur_front_point->pos;
                    prev->add_next(next.get());
                    next->add_prev(prev.get());
                    return true;
                };
                if (!prev || !cur)
                {
                    return;
                }
                auto &prev_center = prev->center;
                auto &cur_center = cur->center;
                if (!prev_center || !cur_center)
                {
                    return;
                }
                if (prev_center->points.empty() || cur_center->points.empty())
                {
                    return;
                }
                auto &back_point = prev_center->points.back();
                auto &front_point = cur_center->points.front();
                // TODO change point to same
                if (alg::calc_dis(back_point->pos, front_point->pos) < 0.5&&
                    alg::calc_vertical_dis(back_point->pos, front_point->pos,front_point->dir) < 0.25) // 25cm
                {
                    back_point->pos=front_point->pos;
                    prev->add_next(cur);
                    cur->add_prev(prev);
                    prev_center->add_next(cur_center.get());
                    cur_center->add_prev(prev_center.get());
                    // 校验边界线位置
                    set_lane_boundary_connect(prev->left,cur->left);
                    set_lane_boundary_connect(prev->right,cur->right);
                }
            };

            auto connect_lane_boundary = [](fast_road_model::Lane *prev, fast_road_model::Lane *cur)
            {
                if (!prev || !cur)
                {
                    return;
                }
                auto prev_lbs={prev->left,prev->right};
                auto cur_lbs={cur->left,cur->right};
                for(auto &plb:prev_lbs)
                {
                    for(auto &clb:cur_lbs)
                    {
                        if(!plb||!clb)
                        {
                            continue;
                        }
                        auto& back_point = plb->points.back();
                        auto&front_point = clb->points.front();
                        if (alg::calc_dis(back_point->pos, front_point->pos) < 0.5&&
                            alg::calc_vertical_dis(back_point->pos, front_point->pos,front_point->dir) < 0.5) // 25cm
                         {
                            back_point->pos=front_point->pos;
                            plb->add_next(clb.get());
                            clb->add_prev(plb.get());
                         }
                    }
                }

                // 添加成组挂接逻辑
                if(prev->left&&prev->right&&cur->left&&cur->right){
                    auto& left_back_point = prev->left->points.back();
                    auto& left_front_point = cur->left->points.front();
                    auto& right_back_point = prev->right->points.back();
                    auto& right_front_point = cur->right->points.front();

                    if ((alg::calc_dis(left_back_point->pos, left_front_point->pos) < 0.8&&
                            alg::calc_vertical_dis(left_back_point->pos, left_front_point->pos, left_front_point->dir) < 0.8) || 
                        (alg::calc_dis(right_back_point->pos, right_front_point->pos) < 0.8&&
                            alg::calc_vertical_dis(right_back_point->pos, right_front_point->pos, right_front_point->dir) < 0.8)) // 25cm
                    {
                        left_front_point->pos = left_back_point->pos;
                        prev->left->add_next(cur->left.get());
                        cur->left->add_prev(prev->left.get());

                        right_front_point->pos = right_back_point->pos;
                        prev->right->add_next(cur->right.get());
                        cur->right->add_prev(prev->right.get());

                        // LOG_INFO("---------  forward alignment   ---------");
                    }
                }
            };

            auto connect_lane_boundary_reverse = [](fast_road_model::Lane *prev, fast_road_model::Lane *cur)
            {
                if (!prev || !cur)
                {
                    return;
                }
                auto prev_lbs={prev->left,prev->right};
                auto cur_lbs={cur->left,cur->right};
                for(auto &plb:prev_lbs)
                {
                    for(auto &clb:cur_lbs)
                    {
                        if(!plb||!clb)
                        {
                            continue;
                        }
                        auto& back_point = plb->points.back();
                        auto&front_point = clb->points.front();
                        if (alg::calc_dis(back_point->pos, front_point->pos) < 0.5&&
                            alg::calc_vertical_dis(back_point->pos, front_point->pos,front_point->dir) < 0.5) // 25cm
                         {
                            front_point->pos=back_point->pos;
                            plb->add_next(clb.get());
                            clb->add_prev(plb.get());
                         }
                    }
                }

                // 添加成组挂接逻辑
                if(prev->left&&prev->right&&cur->left&&cur->right){
                    auto& left_back_point = prev->left->points.back();
                    auto& left_front_point = cur->left->points.front();
                    auto& right_back_point = prev->right->points.back();
                    auto& right_front_point = cur->right->points.front();

                    if ((alg::calc_dis(left_back_point->pos, left_front_point->pos) < 0.8&&
                            alg::calc_vertical_dis(left_back_point->pos, left_front_point->pos, left_front_point->dir) < 0.8) || 
                        (alg::calc_dis(right_back_point->pos, right_front_point->pos) < 0.8&&
                            alg::calc_vertical_dis(right_back_point->pos, right_front_point->pos, right_front_point->dir) < 0.8)) // 25cm
                    {
                        left_back_point->pos = left_front_point->pos;
                        prev->left->add_next(cur->left.get());
                        cur->left->add_prev(prev->left.get());

                        right_back_point->pos = right_front_point->pos;
                        prev->right->add_next(cur->right.get());
                        cur->right->add_prev(prev->right.get());

                        // LOG_INFO("---------  backward alignment   ---------");
                    }
                }
            };

            // TODO 
            auto connect_lane_group = [connect_road_boundary, connect_lane,connect_lane_boundary, connect_lane_boundary_reverse]
                                    (fast_road_model::LaneGroup *prev, fast_road_model::LaneGroup *cur)
            {
                connect_road_boundary(prev->left_road_boundary.get(), cur->left_road_boundary.get());
                connect_road_boundary(prev->right_road_boundary.get(), cur->right_road_boundary.get());
                for (auto &prev : prev->lane_list)
                {
                    for (auto &cur : cur->lane_list)
                    {
                        connect_lane(prev.get(), cur.get()); // 此处连接的是中心线、车道线
                        // fix merge splid
                        connect_lane_boundary(prev.get(), cur.get()); // 此处连接的是分合流的线
                    }
                }
                for (auto &prev : prev->lane_list)
                {
                    for (auto &cur : cur->lane_list)
                    {
                        // connect_lane(prev.get(), cur.get()); // 此处连接的是中心线、车道线
                        // fix merge splid
                        connect_lane_boundary_reverse(prev.get(), cur.get()); // 此处连接的是分合流的线
                    }
                }
            };

            for (auto rs : session->road_segments)
            {
                if (!rs || rs->lane_group_list.empty())
                {
                    continue;
                }
                auto &front_lane_group = rs->lane_group_list.front();
                auto &all_prev_rs = rs->get_prev();  // road_segment前驱后继关系是在split_road中 make_road_segment 中赋值
                // LOG_INFO("front_lane_group:{}  all_prev:{}",uint64_t(front_lane_group.get()),all_prev.size());
                for (auto &p : all_prev_rs)
                {
                    if (!p || p->lane_group_list.empty())
                    {
                        continue;
                    }
                    auto &prev_back_lane_group = p->lane_group_list.back();
                    prev_back_lane_group->add_next(front_lane_group.get());
                    front_lane_group->add_prev(prev_back_lane_group.get());
                    connect_lane_group(prev_back_lane_group.get(), front_lane_group.get());
                }
                // auto &back_lane_greoup =rs->lane_group_list.back();
            }

            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::temp_gen_relation(RoadModelSessionData *session)
        {
            RTreeProxy<fast_road_model::LBPoint *, float, 2> prev_lane_boundary_tree; // prev  null
            RTreeProxy<fast_road_model::LBPoint *, float, 2> next_lane_boundary_tree; //next null
            std::vector<fast_road_model::LBPoint*> next_turn_buffer; //next  null
            std::vector<fast_road_model::LBPoint*> prev_turn_buffer; //prev  null
            auto is_turn_right_rs=[](const fast_road_model::RoadSegment*rs)
            {
                if (!rs)
                {
                    return false;
                }
                auto pos_sample_list=rs->pos_sample_list;

                if(pos_sample_list.empty())
                {
                    return false;
                }
                auto front=pos_sample_list.front();
                if(!front)
                {
                    return false;
                }
                auto from_raw_link=front->from_raw_link;
                if(!from_raw_link)
                {
                    return false;
                }
                // int from = std::stoi(from_raw_link->form);
                // if(from!=25&&from!=26)
                if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
                {
                    return false;
                }
               return true;
            };
            auto make_tree=[&session,is_turn_right_rs,&prev_lane_boundary_tree,&next_lane_boundary_tree,&next_turn_buffer,&prev_turn_buffer]()
            {
                for (auto &rs : session->road_segments)
                {
                    if (!rs)
                    {
                        continue;
                    }
                    bool is_turn_rs=is_turn_right_rs(rs);
                    std::set<fast_road_model::LaneBoundary*> hash_tabel;
                    for (auto &lane_group : rs->lane_group_list)
                    {
                        if (!lane_group)
                        {
                            continue;
                        }
                        for (auto &lane : lane_group->lane_list)
                        {
                            if (!lane)
                            {
                                continue;
                            }
                            auto lbs = {lane->left, lane->right};
                            bool is_left = true;
                            for(auto& lb:lbs)
                            {
                                // if(hash_tabel.count(lb.get()))
                                // {
                                //     continue;
                                // }
                                // hash_tabel.insert(lb.get());
                                if(!lb)
                                {
                                    continue;
                                }
                                auto &points=lb->points;
                                if(points.size()<=1)
                                {
                                    continue;
                                }
                                {
                                    auto &front=points.front();
                                    if(is_left){
                                        front->left_or_right = front->left_or_right==2?3:1;
                                        if(lane->right){
                                            front->right_lb = lane->right->points.front().get();
                                        }
                                    }else{
                                        front->left_or_right = front->left_or_right==1?3:2;
                                        if(lane->left){
                                            front->left_lb = lane->left->points.front().get();
                                        }
                                    }
                                    if(!is_turn_rs)
                                    {
                                        prev_lane_boundary_tree.insert(front->pos,front.get());
                                    }else
                                    {   
                                        if (rs->prev_all.size() == 0) {
                                            // LOG_INFO("右转车道");
                                            prev_turn_buffer.push_back(front.get());
                                        }
                                    }
                                   
                                }
                                {
                                    auto &back=points.back();
                                    if(is_left){
                                        back->left_or_right = back->left_or_right==2?3:1;
                                        if(lane->right){
                                            back->right_lb = lane->right->points.back().get();
                                        }
                                    }else{
                                        back->left_or_right = back->left_or_right==1?3:2;
                                        if(lane->left){
                                            back->left_lb = lane->left->points.back().get();
                                        }
                                    }
                                    if(!is_turn_rs)
                                    {
                                        next_lane_boundary_tree.insert(back->pos,back.get());
                                    }else
                                    {
                                        if (rs->next_all.size() == 0) {
                                            // LOG_INFO("右转车道");
                                            next_turn_buffer.push_back(back.get());
                                        }
                                    }
                                   
                                }
                                is_left = false;
                            }
                           
                        }
                    }
                }
            };
            
            auto set_connection=[]( RTreeProxy<fast_road_model::LBPoint *, float, 2> &tree, std::vector<fast_road_model::LBPoint*> &buffer,double range,bool is_prev)
            {
                for(auto &seed:buffer)
                {
                    std::vector<fast_road_model::LBPoint*> lbs;
                    tree.search(seed->pos,range*1.414,lbs);
                    fast_road_model::LBPoint* next=NULL;
                    double min_d=100;
                    for(auto &lb:lbs)
                    {
                        if(seed->from_lane_boundary==lb->from_lane_boundary)
                        {
                            continue;
                        }
                       double d1= alg::calc_vertical_dis(lb->pos,seed->pos,lb->dir);
                       double d2= alg::calc_hori_dis(lb->pos,seed->pos,lb->dir,true);
                       bool in_range=is_prev?(d2>-4&&d2<3):(d2<4&&d2>-3);
                    //    LOG_INFO("get  next [{} {} ][d1:{} d2:{} in_range:{} |{}]",seed->pos.x(),seed->pos.y(),d1,d2,in_range,lbs.size() );
                       if(d1<1.5&& in_range)
                       {
                           if(std::abs(d2)<min_d)
                           {
                            min_d=std::abs(d2);
                            next=lb;
                           }
                       }
                    }
                    if(next)
                    {
                        // LOG_INFO("set next [{} {} ][min_d:{} is_prev:{}",next->pos.x(),next->pos.y(),min_d ,is_prev);
                        // 从prev 空找next 空的tree
                        if(is_prev)
                        {   
                            auto &next_lane_boundary=seed->from_lane_boundary;
                            auto &prev_lane_boundary=next->from_lane_boundary;
                            prev_lane_boundary->add_next(next_lane_boundary);
                            next_lane_boundary->add_prev(prev_lane_boundary);
                            seed->pos=next->pos;

                            if(seed->left_or_right==1 && seed->right_lb && next->right_lb){
                                auto &next_lane_boundary2=seed->right_lb->from_lane_boundary;
                                auto &prev_lane_boundary2=next->right_lb->from_lane_boundary;
                                prev_lane_boundary2->add_next(next_lane_boundary2);
                                next_lane_boundary2->add_prev(prev_lane_boundary2);
                                seed->right_lb->pos=next->right_lb->pos;
                            }else if(seed->left_or_right==2 && seed->left_lb && next->left_lb){
                                auto &next_lane_boundary2=seed->left_lb->from_lane_boundary;
                                auto &prev_lane_boundary2=next->left_lb->from_lane_boundary;
                                prev_lane_boundary2->add_next(next_lane_boundary2);
                                next_lane_boundary2->add_prev(prev_lane_boundary2);
                                seed->left_lb->pos=next->left_lb->pos;
                            }
                        }
                        else
                        {
                            // 从next 空seed 找prev 空的tree
                            auto &prev_lane_boundary=seed->from_lane_boundary;
                            auto &next_lane_boundary=next->from_lane_boundary;
                            prev_lane_boundary->add_next(next_lane_boundary);
                            next_lane_boundary->add_prev(prev_lane_boundary);
                            seed->pos=next->pos;

                            if(seed->left_or_right==1 && seed->right_lb && next->right_lb){
                                auto &prev_lane_boundary2=seed->right_lb->from_lane_boundary;
                                auto &next_lane_boundary2=next->right_lb->from_lane_boundary;
                                prev_lane_boundary2->add_next(next_lane_boundary2);
                                next_lane_boundary2->add_prev(prev_lane_boundary2);
                                seed->right_lb->pos=next->right_lb->pos;
                            }else if(seed->left_or_right==2 && seed->left_lb && next->left_lb){
                                auto &prev_lane_boundary2=seed->left_lb->from_lane_boundary;
                                auto &next_lane_boundary2=next->left_lb->from_lane_boundary;
                                prev_lane_boundary2->add_next(next_lane_boundary2);
                                next_lane_boundary2->add_prev(prev_lane_boundary2);
                                seed->left_lb->pos=next->left_lb->pos;
                            }
                        }
                    }
                }
            };
            auto smooth_laneline=[&session,is_turn_right_rs]()
            {
                for (auto &rs : session->road_segments)
                {
                    bool is_turn_rs=is_turn_right_rs(rs);
                    if (!rs || !is_turn_rs)
                    {
                        continue;
                    }
                    for (auto &lane_group : rs->lane_group_list)
                    {
                        if (!lane_group)
                        {
                            continue;
                        }
                        for (auto &lane : lane_group->lane_list)
                        {
                            if (!lane)
                            {
                                continue;
                            }
                            auto lbs = {lane->left, lane->right};
                            // auto lbs = {lane->left};
                            bool is_left = true;
                            for(auto& lb:lbs)
                            {
                                if(!lb)
                                {
                                    continue;
                                }
                                auto &points=lb->points;
                                if(points.size()<=1)
                                {
                                    continue;
                                }

                                std::vector<Eigen::Vector3d> points_noz;
                                for(auto pt: points)
                                {
                                    points_noz.push_back({pt->pos.x(), pt->pos.y(), 0});
                                }
                                int n=10;
                                while(n){
                                    points_noz.push_back({points.front()->pos.x(), points.front()->pos.y(), 0});
                                    points_noz.push_back({points.back()->pos.x(), points.back()->pos.y(), 0});
                                    
                                    n--;
                                }
                                
                                auto &all_prev= lb->get_prev();
                                if(all_prev.size()>0){
                                    auto prev_lb = *(all_prev.begin());
                                    auto pt = prev_lb->points.back();
                                    points_noz.push_back({pt->pos.x(), pt->pos.y(), 0});
                                    // LOG_INFO("prev smooth +1 ...");
                                    // for(auto pt: prev_lb->points)
                                    // {
                                        //     points_noz.push_back({pt->pos.x(), pt->pos.y(), 0});
                                        // }
                                }
                                auto &all_next= lb->get_next();
                                if(all_next.size()>0){
                                    auto next_lb = *(all_next.begin());
                                    auto pt = next_lb->points.front();
                                    points_noz.push_back({pt->pos.x(), pt->pos.y(), 0});
                                    // LOG_INFO("next smooth +1 ...");
                                    // for(auto pt: next_lb->points)
                                    // {
                                    //     points_noz.push_back({pt->pos.x(), pt->pos.y(), 0});
                                    // }
                                }
                                
                                PolyFit::Problem problem;
                                problem.degree=3;
                                problem.points=points_noz;
                                PolyFit::Options option;
                                option.use_calc_direction=true;
                                option.use_calc_center=true;
                                PolyFit fit = PolyFit(problem, option);
                                fit.solver();
                                
                                fast_road_model::LBPointPtr prev_pt=NULL;
                                Eigen::Vector3d dir;
                                alg::calc_dir(points.back()->pos, points.front()->pos, dir, true);
                                for(auto pt: points)
                                {   
                                    if(!(pt == points.front() || pt == points.back())){
                                        Eigen::Vector3d fit_pnt(pt->pos.x(), pt->pos.y(), 0);
                                        auto added_fit_pnt = fit.eval(fit_pnt);
                                        pt->pos = Eigen::Vector3d(added_fit_pnt.x(), added_fit_pnt.y(), pt->pos.z());
                                    }
                                    
                                    if(prev_pt){
                                        Eigen::Vector3d cur_dir;
                                        alg::calc_dir(pt->pos, prev_pt->pos, cur_dir, true);
                                        if(alg::calc_theta(cur_dir, dir)>90){
                                            prev_pt->pos = pt->pos;
                                            LOG_INFO("find abnormal_dir pt +1 ...");
                                        }
                                    }

                                    prev_pt = pt;
                                }
                            }
                        }
                    }
                }
            };
            // step 1
            make_tree();
            // step 2
            set_connection(next_lane_boundary_tree,prev_turn_buffer,5,true);
            // 
            set_connection(prev_lane_boundary_tree,next_turn_buffer,5,false);
            // step 3
            // TODO add lane group and lane 
            smooth_laneline();

            return fsdmap::SUCC;
        }

        int RoadModelProcFormatNewRoad::sample_lane_center_pos(RoadModelSessionData *session)
        {
            // session->set_display_name("sample_lane_center_pos");
            // auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "found_light_lb");
            // log_2->color= {0,255,0};

            RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
            for (auto line : session->merge_lane_center_list)
            {
                for (auto pt : line->list)
                {
                    lane_center_tree.insert(pt->pos, pt.get());
                    
                    // // debug
                    // auto &ele1=log_2->add(pt->pos);
                    // ele1.color={0,0,255};
                    // ele1.label.label = 1;
                }
            }
            // session->save_debug_info("sample_lane_center_pos");

            auto interpolation=[](HitCenterPoint&center,Eigen::Vector3d cross,LaneCenterFeature*lc,LaneCenterFeature*lc_next)
            {
               double factor=alg::calc_dis(cross,lc->pos);
               factor=factor/alg::calc_dis(lc->pos,lc_next->pos);
               Eigen::Vector3d left_pos=(1.0-factor)*lc->left_lb.pos+factor*lc_next->left_lb.pos;
               Eigen::Vector3d right_pos=(1.0-factor)*lc->right_lb.pos+factor*lc_next->right_lb.pos;
               Eigen::Vector3d left_dir=(1.0-factor)*lc->left_lb.dir+factor*lc_next->left_lb.dir;
               Eigen::Vector3d right_dir=(1.0-factor)*lc->right_lb.dir+factor*lc_next->right_lb.dir;
               center.left_lb.init(left_pos,left_dir);
               center.left_lb.type = lc->left_lb.type;
               center.left_lb.color = lc->left_lb.color;
               center.right_lb.init(right_pos,right_dir);
               center.right_lb.type = lc->right_lb.type;
               center.right_lb.color = lc->right_lb.color;
                // if(172.0<lc->pos.x() && lc->pos.x() < 172.8 && -1.81<lc->pos.y() && lc->pos.y() < -1.61) {
                //     std::cout << "catch anbomal: " << lc << " pos "  << lc->pos.transpose() << std::endl;
                //     std::cout << "catch anbomal: " << lc_next << " pos "  << lc_next->pos.transpose() << std::endl;
                //     std::cout << " 1 catch anbomal src line: " << lc->group_line << "   "  << lc_next->group_line << std::endl;
                //     LOG_ERROR("lc init_lb_status:{}, lc_next init_lb_status:{}",lc->init_lb_status, lc_next->init_lb_status);
                //     LOG_ERROR("wrong1 : {} l: {} {} {} r: {} {} {}", factor, left_pos.x(), left_pos.y(), left_pos.z(), 
                //                         right_pos.x(), right_pos.y(), right_pos.z())
                //     LOG_ERROR("wrong1.1 : {} lc next rb: {} {} {} lc next rb: {} {} {}", factor, lc_next->left_lb.pos.x(), lc_next->left_lb.pos.y(), lc_next->left_lb.pos.z(), 
                //                         lc_next->right_lb.pos.x(), lc_next->right_lb.pos.y(), lc_next->right_lb.pos.z())
                // }
            };
            auto sort_center=[](KeyPose *poss)
            {
               Eigen::Vector3d dir=alg::get_vertical_dir( poss->road_vertical_dir);
               std::sort(poss->centers.begin(),poss->centers.end(),[&dir,poss](HitCenterPoint &a,HitCenterPoint&b)
                {
                    return fsdmap::alg::calc_vertical_dis( a.pos,poss->pos,dir,true) < fsdmap::alg::calc_vertical_dis( b.pos,poss->pos,dir,true);
                });
            };
            auto insert_center_poss=[](KeyPose*poss,HitCenterPoint&new_center_point, int is_sm)
            {
                // if(is_sm != 1) {
                    // 避免重复
                    for(auto &p:poss->centers)
                    {
                        // TODO：分合流的地方都插值不了
                        // if(alg::calc_dis(p.pos,new_center_point.pos)<0.5)
                        if(alg::calc_dis(p.pos,new_center_point.pos)<0.01)
                        {
                            return ; //
                        }
                    }
                // }
                poss->centers.push_back(new_center_point);
            };
            auto set_cross = [&lane_center_tree,interpolation,insert_center_poss](KeyPose *poss,std::set<int>&hash)
            {
                std::vector<LaneCenterFeature *> secs;
                lane_center_tree.search(poss->pos, 20, secs);
                auto p1=alg::get_hori_pos(poss->pos, poss->road_vertical_dir,+20);
                auto p2=alg::get_hori_pos(poss->pos, poss->road_vertical_dir,-20);
                for (auto lc : secs)
                {
                    auto neighbors = {lc->next};
                    // auto neighbors = {lc->next, lc->prev};
                    for (auto neighbor : neighbors)
                    {
                        if (!neighbor)
                        {
                            continue;
                        }
                        if(lc->init_lb_status == 0 || neighbor->init_lb_status == 0) {
                            std::cout << " sample lc " << lc << " lc pos " << lc->pos.transpose() << std::endl;
                            LOG_ERROR("lc init_lb_status:{}, neighbor init_lb_status:{}",lc->init_lb_status,neighbor->init_lb_status);
                        }
                        Eigen::Vector3d cross;
                        // uint64_t id=uint64_t(lc)+uint64_t(neighbor);
                        // if(hash.count(id))
                        // {
                        //     continue;
                        // }
                        // hash.insert(id);
                        // if(alg::match_any_with_forms(poss->from_raw_link->forms, {25, 26}) && alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross, 2)){
                        //     LOG_INFO("-----------DEBUG FORMAT: {} --- {}, {}, {}-----------", alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross, 2), 
                        //         static_cast<void*>(lc->hit_link), static_cast<void*>(neighbor->hit_link), static_cast<void*>(poss->from_link));
                        // }

                        if (alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross, 2))
                        // if (alg::get_cross_point_by_point(p1, p2, lc->pos, neighbor->pos, cross, true, 0))
                        {

                            if(lc->hit_link!=poss->from_link || neighbor->hit_link!=poss->from_link)
                            // if(lc->hit_link!=poss->from_link)
                            {
                                continue;
                            }
                            // 
                            // LOG_INFO("set_from_center_line:{}",uint64_t(lc->group_line));
                            HitCenterPoint new_center_point;
                            new_center_point.pos=cross;
                            interpolation(new_center_point,cross,lc,neighbor);
                            // 反向车道中心线
                            new_center_point.dir=lc->dir;
                            new_center_point.from_link=poss->from_link;
                            new_center_point.from_center_line=lc->group_line;
                            // LOG_INFO("set_from_center_line:{}",uint64_t(lc->group_line));
                            if(alg::calc_dis(lc->pos,cross)<alg::calc_dis(neighbor->pos,cross))
                            {
                                new_center_point.from_raw_center_feature=lc;
                            }
                            else
                            {
                                new_center_point.from_raw_center_feature=neighbor;
                            }

                            
                            insert_center_poss(poss,new_center_point, lc->point_status);
                            // 
                        }
                    }
                }
            };
            auto set_max_curvature = [](KeyPose * poss)
            {
                auto raw_link=poss->from_raw_link;
                double scale=1.0;
                if(raw_link)
                {
                    int lane_num=raw_link->lanenum_sum;
                    scale=lane_num<=1?100:1;

                }
                double max_curvature = 0;
                for (auto center : poss->centers)
                {
                    auto from_raw_center_feature = center.from_raw_center_feature;
                    if (from_raw_center_feature)
                    {
                        max_curvature = std::max(max_curvature, from_raw_center_feature->curvature);
                    }
                }
                for (auto &center : poss->centers)
                {
                    center.max_curvature = max_curvature*scale;
                }
            };
            std::set<int> hash;
            for (auto rs : session->road_segments)
            {
                for (auto &poss : rs->pos_sample_list)
                {
                    set_cross(poss, hash);
                    sort_center(poss);
                    set_max_curvature(poss);
                }
            }
            return fsdmap::SUCC;
        }
        int RoadModelProcFormatNewRoad::save_debug_info(RoadModelSessionData *session)
        {
            session->set_display_name("format_new_road");
            auto log = session->add_debug_log(utils::DisplayInfo::POINT, "center_point");
            int count = 0;
            for (auto rs : session->road_segments)
            {
                srand48(count++);
                int label = 1E3 * drand48();
                for (auto poss : rs->pos_sample_list)
                {
                    for (auto p : poss->centers)
                    {
                        auto &ele = log->add(p.pos);
                        ele.label.label = label;
                        ele.label.opt_label = 0;
                        const LaneCenterGroupLine *from_center_line = p.from_center_line;
                        if (!from_center_line)
                        {
                            LOG_INFO("from_center_line_DEBUG");
                        }
                    }
                }
            }
            for (auto rs : session->road_segments)
            {
                for (auto lane_group : rs->lane_group_list)
                {
                    // lane boundary && lane center
                    int n = lane_group->lane_list.size();
                    std::set<fast_road_model::LaneBoundaryPtr> hash_table;
                    for (int i = 0; i < n; i++)
                    {
                        auto lane = lane_group->lane_list[i];
                        auto clog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "center_line_{}", count++);
                       
                        // auto rlog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "right_line_{}",count++);
                        const auto &center = lane->center;
                        const auto &left = lane->left;
                        const auto &right = lane->right;
                        clog->color = {0, 255, 0};
                        // rlog->color={255,255,255};
                        for (auto pt : center->points)
                        {
                            auto &ele = clog->add(pt->pos);
                            ele.label.opt_label = i;
                        }
                        auto lane_boundarys = {left, right};
                        for (auto lane_boundary : lane_boundarys)
                        {
                            if (!lane_boundary)
                            {
                                continue;
                            }
                            if (hash_table.count(lane_boundary))
                            {
                                continue;
                            }
                            hash_table.insert(lane_boundary);
                            auto llog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_{}", count++);
                            llog->color = {255, 255, 255};
                            for (auto pt : lane_boundary->points)
                            {
                                auto &ele = llog->add(pt->pos);
                                ele.label.opt_label = 0;
                            }
                            if(lane_boundary->points.empty())
                            {
                                continue;
                            }
                            // 标记是否有前驱后继
                            auto & front=lane_boundary->points.front();
                            auto & back=lane_boundary->points.back();
                            if(lane_boundary->get_prev().empty())
                            {
                               auto plog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "polygon{}", count++);
                               plog->color={255,0,0};
                               for(double theta=0;theta<2*3.14;theta+=3.14/3)
                               {
                                Eigen::Vector3d pos={0.5*cos(theta),0.5*sin(theta),0};
                                pos=pos+front->pos;
                                auto &ele= plog->add(pos);
                                ele.color={255,0,0};
                               }

                            }
                            if(lane_boundary->get_next().empty())
                            {
                                auto plog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "polygon{}", count++);
                                plog->color={0,0,255};
                                for(double theta=0;theta<2*3.14;theta+=3.14/3)
                                {
                                 Eigen::Vector3d pos={0.5*cos(theta),0.5*sin(theta),0};
                                 pos=pos+back->pos;
                                 auto &ele= plog->add(pos);
                                 ele.color={0,0,255};
                                }
                            }
                        }
                    }
                    // road boundary
                    auto road_boundary = {lane_group->left_road_boundary, lane_group->right_road_boundary};
                    for (auto rb : road_boundary)
                    {
                        if (!rb)
                        {
                            continue;
                        }
                        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "road_boundary_{}", count++);
                        log->color = {81, 89, 240};
                        if (rb->points.empty())
                        {
                            continue;
                        }

                        for (auto rb_pt : rb->points)
                        {
                            auto &ele = log->add(rb_pt->pos);
                            ele.label.label = count;
                        }
                    }
                }
            }
            
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 5) // 人行横道
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "new crosswalks");
                    blog->color = {165, 42, 42};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 6) // 停止线
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::LINE, "new stoplines");
                    blog->color = {255, 0, 255};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 3) // 箭头
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "new arrow");
                    blog->color = {255, 0, 0};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }
            for (auto &bs : session->raw_object_ret_list)
            {
                if (bs->ele_type == 4) // 注意行人
                {
                    auto blog = session->add_debug_log(utils::DisplayInfo::POLYGEN, "new other object");
                    blog->color = {139, 58, 58};
                    for (auto &bp : bs->list)
                    {
                        blog->add(bp->pos);
                    }
                }
            }

            session->save_debug_info("format_new_road");
            //
            return fsdmap::SUCC;
        }
    }
}
/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
