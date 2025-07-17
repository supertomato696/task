


#include "road_model_proc_bind_trail.h"
#include "utils/polyfit.h"
#include "utils/spline_interpolator.h"
#include "utils/algorithm_util.h"

DEFINE_bool(bind_trail_enable, true, "bind_trail_enable");
DEFINE_bool(bind_trail_debug_pos_enable, true, "bind_trail_debug_enable");
DEFINE_bool(bind_trail_save_data_enable, true, "bind_trail_save_data_enable");
DEFINE_bool(bind_trail_save_data_save_detail_enable, false, "bind_trail_save_data_save_detail_enable");
DEFINE_double(bind_trail_turn_theta_threshold, 20, "bind_trail_turn_theta_threshold");
DEFINE_double(bind_trail_pos_match_lane_radius, 30, "bind_trail_pos_match_lane_radius");
DEFINE_double(bind_trail_pos_match_lane_z_max, 6, "bind_trail_pos_match_lane_z_max");
DEFINE_double(bind_trail_pos_match_lane_z_min, -2, "bind_trail_pos_match_lane_z_min");
DEFINE_double(bind_trail_pos_match_lane_theta, 30, "bind_trail_pos_match_lane_theta");
DEFINE_double(bind_trail_pos_match_crosswalk_radius, 15.0, "bind_trail_pos_match_crosswalk_radius");
DEFINE_double(bind_trail_pos_match_crosswalk_h_scope, 0.5, "bind_trail_pos_match_crosswalk_h_scope");
DEFINE_double(bind_trail_pos_match_crosswalk_v_scope, 5, "bind_trail_pos_match_crosswalk_v_scope");
DEFINE_double(bind_trail_bind_lc_width_buff, 0.8, "bind_trail_bind_lc_width_buff");
DEFINE_double(bind_trail_pos_match_stopline_theta_thres, 60, "bind_trail_pos_match_stopline_theta_thres");
DEFINE_double(bind_trail_pos_match_crosswalk_theta_thres, 60, "bind_trail_pos_match_crosswalk_theta_thres");
DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

DEFINE_double(lane_boundary_width_diff_thres, 0.5, "lane_boundary_width_diff_thres");
DEFINE_double(standard_lane_width, 3.5, "standard_lane_width");

namespace fsdmap {
namespace road_model {
fsdmap::process_frame::PROC_STATUS RoadModelProcBindTrail::proc(
        RoadModelSessionData* session) {
    auto save_debug2 = [session](int stage) {
        std::string display_name = "bind_lane_boundary_" + std::to_string(stage);
        session->set_display_name(display_name.c_str());

        int count1=0;
        if (stage<=7) {
            for(auto line:session->merge_lane_center_list)
            {
                // auto log = session->add_debug_log(utils::DisplayInfo::LINE, "build_boundary");
                // log->color = {0, 255, 0};
                // LaneCenterFeature*prev=NULL;
                // for (int64_t j = 0; j < line->list.size(); ++j) {
                //     auto &pt = line->list[j];
                //     // auto &ele = log->add(pt->pos, 1, line->match_list[j].size());
                //     // ele.label.opt_label = i;
                //     // ele.label.score = pt->score;
                //     // if (pt->invalid()) {
                //     //     ele.color = {100, 100, 100};
                //     // }
                //     double d = 5;
                //     if (prev) {
                //         d = alg::calc_dis(pt->pos,prev->pos);
                //     } else {
                //         prev=pt.get();
                //     }
                //     if(j<2 || d>=4 || j==line->list.size()-1) {
                //     // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==line->list.size()-1) {
                //         auto &ele = log->add(pt->pos, 1);
                //         // ele.label.opt_label = line->cur_line_id;
                //         ele.label.opt_label = j;
                //         ele.label.intensity_opt = !line->id.empty() ? std::stoi(line->id): 0;
                //         ele.label.score = count1;
                //         ele.label.label = 3; // 临时弄的
                //         if (pt->invalid()) {
                //             ele.color = {100, 100, 100};
                //         }
                //         // ele.label.intensity_opt=cnt++;
                //         prev=pt.get();
                //     } 
                // }

                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count1++);
                log->color = {255, 255, 255};
                for (auto pt : line->list)
                {
                    srand48(time(NULL));
                    Eigen::Vector3d pos = pt->pos;
                    auto &ele = log->add(pos);
                    ele.label.intensity = pt->curvature;
                    if (pt->hit_link)
                    {
                        auto link = pt->hit_link;
                        srand48(link->same_id);
                        ele.label.label = 1E3 * drand48();
                        ele.color = {0, 255, 0};
                    }
                    else
                    {
                        ele.color = {255, 255, 255};
                        srand48(100);
                        ele.label.label = 1E3 * drand48();
                    }
                    // ele.label.score = alg::calc_theta(pt->dir)*57.3;
                    ele.label.score = count1;
                    ele.label.opt_label = pt->hit_poss?1:0;
                    // LOG_INFO(" search error: line.id -- {}", line->id.empty());
                    ele.label.intensity_opt = !line->id.empty() ? std::stoi(line->id): 0;
                }
                count1++;
            }
        } else {
            for(auto line:session->merge_lane_center_list)
            {
                auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count1++);
                line_center->color={0,255,0};
                for (auto lc : line->list)
                {
                    // auto cut_line = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
                    // line_center->color={255,0,0};
                    line_center->add(lc->left_lb.pos);
                    line_center->add(lc->right_lb.pos);
                }
            }
        }
        session->save_debug_info(display_name.c_str());

    };

    auto save_debug_3 = [session](int stage) {
            std::string display_name = "bind_lane_boundary_" + std::to_string(stage);
            session->set_display_name(display_name.c_str());
            int count=0;
            for (auto line : session->merge_lane_line_list)
            {
                auto lane_boundary = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
                lane_boundary->color = {255, 255, 255};
                for (auto pt : line->list)
                {
                    lane_boundary->add(pt->pos);
                }
            }

            for(auto line:session->merge_lane_center_list)
            {
                auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
                line_center->color={0,255,0};
                for (auto lc : line->list)
                {
                    if(stage == 14 || stage == 15) {
                        if(lc->point_status == 0) {
                            continue;
                        }
                    }

                    auto &center_ele=line_center->add(lc->pos);
                    if(!alg::is_bit_set(lc->init_lb_status, 1)&&!alg::is_bit_set(lc->init_lb_status, 3))
                    {
                        center_ele.color={255,0,0};
                    }
                    if(!alg::is_bit_set(lc->init_lb_status, 2)&&!alg::is_bit_set(lc->init_lb_status, 4))
                    {
                        center_ele.color={255,255,0};
                    }

                    auto log_left = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
                    auto log_right = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
                    auto cut_line = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);

                    bool add_left=false;
                    bool add_right=false;           
                    if (alg::is_bit_set(lc->init_lb_status, 1))//left 原来就有
                    {
                        add_left=true;
                        log_left->color = {127, 255, 212}; // 蓝晶
                    }
                    else if(alg::is_bit_set(lc->init_lb_status, 3))
                    {
                        add_left=true;
                        log_left->color = {0, 255, 0}; // 绿
                    }

                    if (alg::is_bit_set(lc->init_lb_status, 2)) //right //原来就有
                    {   add_right=true;
                        log_right->color = {224, 255, 255}; // 白色
                    }
                    else if(alg::is_bit_set(lc->init_lb_status, 4))
                    {
                        add_right=true;
                        log_right->color = {0, 0, 255}; // 蓝
                    }

                    if(alg::is_bit_set(lc->init_lb_status, 5))
                    {
                        // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                        LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                        cut_line->color={255,0,0};
                        cut_line->add(lc->pos);
                        cut_line->add(lc->left_lb.pos);
                    }
                    
                    if(alg::is_bit_set(lc->init_lb_status, 6))
                    {
                        // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                        LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                        cut_line->color={255,0,0};
                        cut_line->add(lc->pos);
                        cut_line->add(lc->right_lb.pos); 
                    }
                    
                    if(add_left)
                    {
                        log_left->add(lc->pos);
                        log_left->add(lc->left_lb.pos);
                    }
                    if(add_right)
                    {
                        log_right->add(lc->pos);
                        log_right->add(lc->right_lb.pos);
                    }
                }
            }

            session->save_debug_info(display_name.c_str());
    };
    
    // CHECK_FATAL_PROC(sample_and_smooth_line(session),"sample_and_smooth_line");

    CHECK_FATAL_PROC(link_mark_turn_right_candiadte(session),"link_mark_turn_right_candiadte");
    // 道路边界内侧击中
    CHECK_FATAL_PROC(calc_lane_curvature(session),"calc_lane_curvature");
    // 
    CHECK_FATAL_PROC(hit_lane_center_candidate(session), "hit_lane_center_cadidate");
    // 第一轮扩展
    CHECK_FATAL_PROC(hit_lane_center_fill_neighbor(session),"hit_lane_center_fill_neighbor");
    // 同向击中
    CHECK_FATAL_PROC(fix_loss_road_boundary(session),"fix_loss_road_boundary");
    // 第二轮扩展
    CHECK_FATAL_PROC(hit_lane_center_fill_neighbor(session),"hit_lane_center_fill_neighbor");
    // 
    CHECK_FATAL_PROC(lane_center_bind_pose(session),"lane_center_bind_pose");
    // 
    CHECK_FATAL_PROC(reverse_lane_center(session),"reverse_lane_center");
    // save_debug2(7);
    // save_debug_3(11);

    //  TODO FIX dir inverse
    CHECK_FATAL_PROC(fill_lane_boundary(session), "fill_lane_boundary");
    // save_debug2(8);
    // save_debug_3(12);

    // 
    CHECK_FATAL_PROC(expend_fill_lane_boundary(session),"expend_fill_lane_boundary");
    // save_debug2(9);
    // save_debug_3(13);
    // save_debug_3(14);
    CHECK_FATAL_PROC(change_sm_lane_boundary(session),"change_sm_lane_boundary");
    // save_debug_3(15);

    // 
    CHECK_FATAL_PROC(hit_road_boundary_candidate(session), "hit_road_boundary_candidate");
    // save_debug2(10);
    
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    // 
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

void RoadModelProcBindTrail::update_lane_center_sample_tree(RoadModelSessionData* session) {

    session->merge_lane_center_sample_tree.RemoveAll();
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            session->merge_lane_center_sample_tree.insert(pt->pos, pt.get());
        }
    }
}

void RoadModelProcBindTrail::update_lb_sample_tree(RoadModelSessionData* session) {

    session->boundary_line_sample_tree.RemoveAll();
    for (auto line : session->merge_boundary_line_list) {
        for (auto pt : line->list) {
            session->boundary_line_sample_tree.insert(pt->pos, pt.get());
        }
    }
}

void RoadModelProcBindTrail::update_lane_line_sample_tree(RoadModelSessionData* session) {

    session->lane_line_sample_tree.RemoveAll();
    for (auto line : session->merge_lane_line_list) {
        for (auto pt : line->list) {
            session->lane_line_sample_tree.insert(pt->pos, pt.get());
        }
    }
}


void get_point(LaneCenterGroupLine*line ,int index,int size,bool is_forward,double distance,std::deque<Eigen::Vector3d> &points)
{
    if(distance<=0)
    {
        return ;
    }
    int prev_index=is_forward?index-1:index+1;
    if(!line)
    {
        return ;
    }
    if(index>=size||index<0|| prev_index>=size||prev_index<0)
    {
        return ;
    }
    auto cur_pt=line->list[index];
    auto prev_pt=line->list[prev_index];
    if(!cur_pt|| !prev_pt)
    {
        return ;
    }
    if(is_forward)
    {
        points.push_back(cur_pt->pos);
    }
    else
    {
        points.push_front(cur_pt->pos);
    }
    int next_index=is_forward?index+1:index-1;
    double next_distance=distance-alg::calc_dis(cur_pt->pos,prev_pt->pos);
    get_point(line,next_index,size,is_forward,next_distance,points);
}

int RoadModelProcBindTrail::calc_lane_curvature(RoadModelSessionData* session)
{
    auto set_calc_curvature = [](LaneCenterGroupLine *line, int index, const std::deque<Eigen::Vector3d> &points)
    {
        if (!line)
        {
            return;
        }
        auto pt = line->list[index];
        std::vector<Eigen::Vector3d> temp;
        temp.insert(temp.end(), points.begin(), points.end());
        // 计算曲率
        PolyFit::Problem problem;
        problem.degree=3;
        problem.points=temp;
        problem.center=pt->pos;
        problem.dir=pt->dir;
        PolyFit::Options option;
        option.use_calc_direction=true;
        option.use_calc_center=true;
        PolyFit fit=PolyFit(problem,option); 
        fit.solver();
        double curvature= fit.curvature(pt->pos);
        pt->curvature=curvature;
        // pt->curvature=1.0/(curvature+1E-3);
        // 
        // double c_x = 0;
        // double c_y = 0;
        // double c_r = 1000;
        // if (alg::fit_circle(temp, c_x, c_y, c_r))
        // {
        //     c_r=std::min(1000.0,c_r);
        //     pt->curvature = c_r;
        // }
        // else
        // {
        //     pt->curvature = 1000;
        // }

    };
    for (auto line : session->merge_lane_center_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
            std::deque<Eigen::Vector3d> points;
            get_point(line, i, size, true, 15, points);
            get_point(line, i, size, false, 15, points);
            set_calc_curvature(line, i, points);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::dfs_link_mark_lane_center_candidate(KeyPoseLine *kpl, int index, bool is_forward,
                                        RTreeProxy<LaneCenterFeature *, float, 2> &lane_center_tree)
{
    if (!kpl)
    {
        return -1;
    }
    const int n = kpl->list.size();
    if (index <= 0 || index >= n-1)
    {
        return index;
    }
    auto poss = kpl->list[index];
    if (!poss)
    {
        return -1;
    }
    auto from_raw_link = poss->from_raw_link;
    if (!from_raw_link)
    {
        return -1;
    }
    else
    {
        // int from = std::stoi(from_raw_link->form);
        // if (from != 25 || from != 26) // 感觉此处有bug，应该为  if (from == 25 || from == 26)
        if (alg::match_any_with_forms(from_raw_link->forms, {25, 26}))
        {
            return -1;
        }
    }
    bool find = false;
    std::vector<LaneCenterFeature *> lcfs;
    lane_center_tree.search(poss->pos, 1, lcfs);
    for (auto lc : lcfs)
    {
        if (lc->hit_link)
        {
            continue;
        }
        lc->hit_link = poss->from_link;
        lc->hit_poss = poss;
        //
        lc->hit_status |= (1 << 3);
        find = true;
    }
    if (find)
    {
        return index;
    }
    int next = is_forward ? index + 1 : index - 1;
    return dfs_link_mark_lane_center_candidate(kpl, next, is_forward, lane_center_tree);
}

int RoadModelProcBindTrail::sample_lane_center(RoadModelSessionData *session)
{
    // LOG_INFO("before size: {}",  session->merge_lane_center_list.size());
    std::vector<std::shared_ptr<LaneCenterGroupLine>> tmp_lane_center_line_list;
    double min_gap = 0.5;
    for (auto &group_list : session->merge_lane_center_list) {
        // std::vector<LaneCenterFeature*> origin_list;
        // for(auto&p : group_list->list) {
        //     origin_list.push_back(p.get());
        // }
        // auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  
        // session->sample_line(min_gap, origin_list,
        //                     session->lane_center_feature_ptr, new_line->list);
        // tmp_lane_center_line_list.push_back(new_line);

        auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  
        session->sample_line_share_ptr(min_gap, group_list->list,
                            session->lane_center_feature_ptr, new_line->list);
        tmp_lane_center_line_list.push_back(new_line);
    }

    session->merge_lane_center_list.clear();
    for(auto&new_line : tmp_lane_center_line_list) {
        if(!new_line->list.empty()) {
            for(auto& lc : new_line->list) {
                lc->group_line = new_line.get();
            }
            session->merge_lane_center_list.push_back(new_line.get());
        }
    }

    update_lane_center_sample_tree(session); //kdtree

    // LOG_INFO("after size: {}",  session->merge_lane_center_list.size());

    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::sample_lane_boundary(RoadModelSessionData *session)
{   
    // LOG_INFO("before size: {}",  session->merge_lane_line_list.size());
    std::vector<std::shared_ptr<LaneLineSampleGroupLine>> tmp_lane_center_line_list;
    double min_gap = 0.5;
    for (auto &group_list : session->merge_lane_line_list) {
        // std::vector<LaneCenterFeature*> origin_list;
        // for(auto&p : group_list->list) {
        //     origin_list.push_back(p.get());
        // }
        // auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  
        // session->sample_line(min_gap, origin_list,
        //                     session->lane_center_feature_ptr, new_line->list);
        // tmp_lane_center_line_list.push_back(new_line);

        auto new_line = session->add_ptr(session->lane_line_group_line_ptr);  
        session->sample_line_share_ptr(min_gap, group_list->list,
                            session->lane_line_sample_ptr, new_line->list);
        tmp_lane_center_line_list.push_back(new_line);
    }

    session->merge_lane_line_list.clear();
    for(auto&new_line : tmp_lane_center_line_list) {
        if(!new_line->list.empty()) {
            for(auto& lc : new_line->list) {
                lc->group_line = new_line.get();
            }
            session->merge_lane_line_list.push_back(new_line.get());
        }
    }

    update_lane_center_sample_tree(session); //kdtree

    // LOG_INFO("after size: {}",  session->merge_lane_line_list.size());

    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::sample_road_boundary(RoadModelSessionData *session)
{   
    // LOG_INFO("before size: {}",  session->merge_boundary_line_list.size());
    std::vector<std::shared_ptr<BoundaryGroupLine>> tmp_lane_center_line_list;
    double min_gap = 0.5;
    for (auto &group_list : session->merge_boundary_line_list) {
        // std::vector<LaneCenterFeature*> origin_list;
        // for(auto&p : group_list->list) {
        //     origin_list.push_back(p.get());
        // }
        // auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  
        // session->sample_line(min_gap, origin_list,
        //                     session->lane_center_feature_ptr, new_line->list);
        // tmp_lane_center_line_list.push_back(new_line);

        auto new_line = session->add_ptr(session->lane_boundary_group_line_ptr);  
        session->sample_line_share_ptr(min_gap, group_list->list,
                            session->boundary_feature_ptr, new_line->list);
        tmp_lane_center_line_list.push_back(new_line);
    }

    session->merge_boundary_line_list.clear();
    for(auto&new_line : tmp_lane_center_line_list) {
        if(!new_line->list.empty()) {
            for(auto& lc : new_line->list) {
                lc->group_line = new_line.get();
            }
            session->merge_boundary_line_list.push_back(new_line.get());
        }
    }

    update_lb_sample_tree(session); //kdtree

    // LOG_INFO("after size: {}",  session->merge_boundary_line_list.size());

    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::sample_and_smooth_line(RoadModelSessionData* session) {

    double min_gap = 0.5; //FLAGS_sample_line_sample_lane_line_gap;
    // int skip_num = FLAGS_sample_line_bev_frame_skip_num;
    double radius1 = 3; //FLAGS_sample_line_dir_radius1;
    double radius2 = 5; //FLAGS_sample_line_dir_radius2;
    double dir_gap_threshold = 0.3; //FLAGS_sample_line_lc_dir_gap_threshold;
    std::vector<double> radii = {radius1, radius2};

    //  smooth lane_center
    sample_lane_center(session);
    smooth_line(session, session->merge_lane_center_list, dir_gap_threshold, radii);
    sample_lane_center(session);

    //  smooth lane_boundary
    sample_lane_boundary(session);
    smooth_line(session, session->merge_lane_line_list, dir_gap_threshold, radii);
    sample_lane_boundary(session);

    //  smooth road_boundary
    sample_road_boundary(session);
    smooth_line(session, session->merge_boundary_line_list, dir_gap_threshold, radii);
    sample_road_boundary(session);

    if(0){
        std::string total_log_name = utils::fmt("sample_and_smooth_line");
        session->set_display_name(total_log_name.c_str());
        for (int64_t i = 0; i < session->merge_lane_center_list.size(); ++i) {
            int oppo = 0;
            auto &group_list = session->merge_lane_center_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_lane_center");
            log->color = {0, 255, 0};
            LaneCenterFeature*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                // ele.label.opt_label = i;
                // ele.label.score = pt->score;
                // if (pt->invalid()) {
                //     ele.color = {100, 100, 100};
                // }
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);

                    auto dir = alg::get_dir(pt->pos,prev->pos);
                    if(alg::calc_theta1(dir,prev->dir,true)>120) {
                        oppo++;
                    } 
                } else {
                    prev=pt.get();
                }

                if(j<2 || d>=0 || j==group_list->list.size()-1) {
                // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    // ele.label.opt_label = i;
                    ele.label.opt_label = group_list->cur_line_id;
                    ele.label.intensity_opt=j;
                    ele.label.score = pt->score;
                    ele.label.label = 3; // 临时弄的
                    if (pt->invalid()) {
                        ele.color={255, 200, 255}; // 浅浅紫色
                    }

                    if (group_list->boundary_type == LaneType::DEBUG) {
                        ele.label.label = 10; // 临时弄的
                    }
                    // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    prev=pt.get();
                } 
            }
            if (oppo > 0) {
                LOG_ERROR("line {} has oppo dir, cnt : {}", i, oppo);
            }
            // trail_map[lit1->first].push_back(log);
        }
        for (int64_t i = 0; i < session->merge_boundary_line_list.size(); ++i) {
            int oppo = 0;
            auto &group_list = session->merge_boundary_line_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_lane_boundary");
            log->color = {0, 255, 0};
            BoundaryFeature *prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                // ele.label.opt_label = i;
                // ele.label.score = pt->score;
                // if (pt->invalid()) {
                //     ele.color = {100, 100, 100};
                // }
                double d = 5;
                if (prev) {
                    d = alg::calc_dis(pt->pos,prev->pos);

                    auto dir = alg::get_dir(pt->pos,prev->pos);
                    if(alg::calc_theta1(dir,prev->dir,true)>120) {
                        oppo++;
                    } 
                } else {
                    prev=pt.get();
                }

                if(j<2 || d>=0 || j==group_list->list.size()-1) {
                // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                    auto &ele = log->add(pt->pos, 1);
                    // ele.label.opt_label = i;
                    ele.label.opt_label = group_list->cur_line_id;
                    ele.label.intensity_opt=j;
                    ele.label.score = pt->score;
                    ele.label.label = 3; // 临时弄的
                    if (pt->invalid()) {
                        ele.color={255, 200, 255}; // 浅浅紫色
                    }

                    if (group_list->boundary_type == LaneType::DEBUG) {
                        ele.label.label = 10; // 临时弄的
                    }
                    // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    prev=pt.get();
                } 
            }
            if (oppo > 0) {
                LOG_ERROR("line {} has oppo dir, cnt : {}", i, oppo);
            }
            // trail_map[lit1->first].push_back(log);
        }
        session->save_debug_info(total_log_name.c_str());
    }

    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::link_mark_turn_right_candiadte(RoadModelSessionData *session)
{
    //TODO  简化代码到merge feature 
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
    for (auto line : session->merge_boundary_line_list)
    {
        for (auto rb : line->list)
        {
            road_boundary_tree.insert(rb->pos, rb.get());
        }
    }
    auto get_right_boundary_rb = [&road_boundary_tree](KeyPose *poss) -> BoundaryFeature *
    {
        auto from_raw_link = poss->from_raw_link;
        if (!from_raw_link)
        {
            return NULL;
        }
        else
        {
            // int from = std::stoi(from_raw_link->form);
            // LOG_INFO("from_raw_link:{} {}",from ,from_raw_link->form)
            // if(from!=25&&from!=26)
            if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
            {
                return NULL;
            }
        }

        std::vector<BoundaryFeature *> bfs;
        road_boundary_tree.search(poss->pos, 10, bfs);
        Eigen::Vector3d p1 = poss->pos;
        Eigen::Vector3d p2 = alg::get_vertical_pos(poss->pos, poss->dir, 10);
        BoundaryFeature *min_dis_bf = NULL;
        double min_dis = 100;
        // LOG_INFO("bfs:{}",bfs.size());
        for (auto bf : bfs)
        {
            if (!bf->next)
            {
                continue;
            }
            Eigen::Vector3d cross;
            if (alg::findIntersection(p1, p2, bf->pos, bf->next->pos, cross))
            {
                // TODO CHECK
                if (alg::judge_left(cross, poss->pos, poss->dir) < 0)
                {
                    continue;
                }
                double dis = alg::calc_dis(cross, poss->pos);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    min_dis_bf = bf;
                }
            }
        }
        return min_dis_bf;
    };

    auto connetct_lane_center = [&lane_center_tree](const fast_road_model::LBPointPtr lb, double &dis) -> LaneCenterFeature *
    {
        std::vector<LaneCenterFeature *> lcfs;
        Eigen::Vector3d p1 = lb->pos;
        Eigen::Vector3d p2 = alg::get_vertical_pos(lb->pos, lb->dir, -3.75*2);
        lane_center_tree.search(lb->pos, 3.75 * 1.414*2, lcfs);
        LaneCenterFeature *ret = NULL;
        double min_dis = 3.75*2;    // 增加lb-lc的宽度限制，防止找到错误点位
        for (auto lc : lcfs)
        {
            auto neighbors = {lc->next, lc->prev};
            for (auto neighbor : neighbors)
            {
                if (!neighbor)
                {
                    continue;
                }

                double theta = alg::calc_theta(lb->dir, lc->dir);
                bool cond2 = theta < 20;
                Eigen::Vector3d cross;
                bool cond1 = alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross);
                if (cond1 && cond2)
                {   
                    // LOG_INFO(" [turnright] lb-lc theta: {}, exist_cross: {} ... ", theta, cond1);
                    //  TODO add right check
                    //
                    double dis = alg::calc_dis(cross, lb->pos);
                    if (dis < min_dis)
                    {
                        min_dis = dis;
                        ret = lc;
                    }
                }
            }
        }
        // 跳变
        if(dis>0&&std::abs(dis-min_dis)>3.5/2)
        {
            dis=-1;
            return NULL;
        }

        dis = min_dis;
        return ret;
    };
    LOG_INFO("candidate_right_boundary start ");
    auto calc_candidate=[&,session]()
    {
        for (auto link : session->link_sample_list)
        {
              if(link->list.empty())
              {
                continue;
              }
              auto front=link->list.front();
              auto from_raw_link = front->from_raw_link;
              if (!from_raw_link)
              {
                 continue;
              }

            //   int from = std::stoi(from_raw_link->form);
            //   if(from!=25&&from!=26)
              if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
              {
                continue;
              }
              std::vector<KeyPose*> new_single_line;
              session->sample_line(0.5, link->list, 
              session->key_pose_ptr,new_single_line);
        
             for(auto &p:new_single_line){
              p->from_raw_link=front->from_raw_link;
              p->from_link=front->from_link;
            }

            BoundaryFeature *prev_rb=NULL;
            for (auto poss : new_single_line)
            {

                BoundaryFeature *rb = get_right_boundary_rb(poss);
                if (rb)
                {
                    Eigen::Vector3d dir={0,0,0};
                    if(prev_rb)
                    {
                         // 避免太近方向算出来误差太大
                        if(alg::calc_dis(prev_rb->pos,rb->pos)<0.25)
                        {
                            continue;
                        }
                        // 避免弯道交叉
                        if(!alg::judge_front(rb->pos,prev_rb->pos,prev_rb->dir))
                        {
                          continue;
                        }
                        dir=alg::get_dir(rb->pos,prev_rb->pos);
                    }
                   prev_rb=rb;
                   candidate_right_boundary.push_back(rb);
                   auto &trc=turn_right_candidate[link];
                   TurnRightPoint  new_turn_right_point;
                   new_turn_right_point.lb=std::make_shared<fast_road_model::LBPoint>();
                   new_turn_right_point.from_road_boundary = rb;
                   new_turn_right_point.lb->init(rb->pos,dir);
                   if(!trc.empty())
                   {
                       auto &back=trc.back();
                       back.lb->set_dir(dir);
                   }
                   trc.push_back(new_turn_right_point);
                }
                else  //lane bounary loss
                {

                }
            }
        }
    };

    auto calc_lc_ditance=[&,connetct_lane_center]()
    {
        for(auto &mtrc:turn_right_candidate)
        {
            auto &trc=mtrc.second;
            const int n=trc.size();
            double dis=-1;
            for(int i=0;i<n;i++)
            {
             
              auto lc= connetct_lane_center(trc[i].lb,dis);
              if(lc)
              {
                trc[i].lc=lc;
                trc[i].distance=dis;
              }else
              {
                dis=-1;
              }
            }
        }
    };
    // auto pruning_one_link_candidate=[](std::vector<TurnRightPoint>& trc, bool forward)-> std::vector<TurnRightPoint>
    // {
    //         const int n=trc.size();
    //         double init_distance=-1;
    //         for(int i=0;i<n;i++)
    //         {
    //          int j=forward?i:n-1-i;
    //         //  LOG_INFO("lc:{}  distance:{}   init_distance:{}",uint64_t(trc[j].lc),trc[j].distance,init_distance)
    //          if(trc[j].lc && trc[j].distance>0&& trc[j].distance<3.75&& init_distance<0)
    //          {
    //             // LOG_INFO("init_distance:{}",init_distance)
    //             init_distance=trc[j].distance;
    //          }
    //         }
    //        std::vector<TurnRightPoint> temp; 
    //        if(init_distance>0)
    //        {
    //         int start=-1;
    //         double last_distance=-1;
    //         LaneCenterFeature*last_lc=NULL;
    //         double avg_diff=0;
    //         double count=0;
    //         for(int i=0;i<n;i++)
    //         {
    //            const  int j=forward?i:n-1-i;
    //             if(trc[j].distance>0&&trc[j].distance<4*init_distance&&start<0)
    //             {
    //                 // LOG_INFO("continue lc:{}  distance:{}   init_distance:{} start:{}",uint64_t(trc[j].lc),trc[j].distance,init_distance,start)
    //                 if(last_distance>0)
    //                 {
    //                     avg_diff+=std::abs(last_distance-trc[j].distance);
    //                 }
    //                 count+=1.0;
    //                 last_lc=trc[j].lc;
    //                 last_distance=trc[j].distance;
    //                 continue;
    //             }
    //             if(start<0)
    //             {
    
                    
    //                 // LOG_INFO("insert forward:{} lc:{}  distance:{}   init_distance:{} start:{}",uint64_t(trc[j].lc),forward,trc[j].distance,init_distance,start)
    //                 trc[j].distance=last_distance;
    //                 trc[j].lc=last_lc;
    //                 trc[j].mark_status=forward?1<<1:1<<2;
    //                 start=j;
    //                 if(count>0&&last_distance>0)
    //                 {
    //                     avg_diff=avg_diff/count;
    //                     LOG_INFO("avg_diff:{}",avg_diff);
    //                     if(avg_diff>0.15)
    //                     {
    //                         // 增长太快不符合
    //                         // trc[j].distance=-1;
    //                         // trc[j].lc=NULL;
    //                     }
    //                 }

    //             }
    //             // LOG_INFO("insert lc:{}  distance:{}   init_distance:{} start:{}",uint64_t(trc[j].lc),trc[j].distance,init_distance,start)
    //             temp.push_back(trc[j]);
    //         }
    //        }
    //   return temp;
       
    // };

    // 输入link找到rb，建立TurnRightPoint；一条link对应vector<TurnRightPoint>
    auto pruning_one_link_candidates=[](std::vector<TurnRightPoint>& trc_s)-> std::vector<std::vector<TurnRightPoint>>
    {
        std::vector<std::vector<TurnRightPoint>> candidate_list;
        std::vector<TurnRightPoint> temp;
        std::vector<TurnRightPoint> trc = trc_s;
        const int n = trc.size();
        double init_distance = -1, prev_init_distance = -1;
        bool init_dis = true, start = false, end = false, save_flag = false;
        double last_distance = -1;
        LaneCenterFeature*last_lc = NULL;

        for(int i = 0; i < n ; i++)
        {
            if(init_dis)
            {
                for(int j = i; j < n; j ++)
                {
                    if(trc[j].lc && trc[j].distance > 0 && trc[j].distance < 3.75 && init_distance < 0)
                    {
                        init_distance = trc[j].distance;
                        init_dis = false;
                        break;
                    }

                }
            }
            if(init_distance > 0)
            {          
                // double avg_diff = 0, count = 0;
                if(trc[i].distance > 0 && trc[i].distance < 4 * init_distance)
                {
                    last_lc = trc[i].lc;
                    last_distance = trc[i].distance;
                    if(save_flag && !end)
                    {
                        end = true;
                    }
                }
                else
                {
                    if(!start && !save_flag)
                    {
                        start = true;
                    }
                }
                if(start)
                {
                    trc[i].distance = last_distance;
                    trc[i].lc = last_lc;
                    trc[i].mark_status = 1 << 1;
                    save_flag = true;

                }
                // if(end)
                // {
                //     trc[i].distance = last_distance;
                //     trc[i].lc = last_lc;
                //     trc[i].mark_status = 1 << 2;
                // }
                if(save_flag)
                {
                    start = false;
                    if(end)
                    {
                        temp.back().distance = last_distance;
                        temp.back().lc = last_lc;
                        temp.back().mark_status = 1 << 2;

                        candidate_list.push_back(temp);
                        temp.clear();
                        end = false;
                        save_flag = false;
                        init_dis = true;
                        // init_distance = -1;
                    }
                    else{
                        temp.push_back(trc[i]);
                        if(i == n - 1)
                        {
                            candidate_list.push_back(temp);
                            temp.clear();
                        }
                    }
                }
            }
        }

      return candidate_list;
    };

    auto pruning_candidate=[&,pruning_one_link_candidates]()
    {
        session->set_display_name("turn_right_candicate");
        for(auto &mtrc:turn_right_candidate)
        {
            auto &trc=mtrc.second;
            // // LOG_INFO("trc:{}",trc.size())
            // auto forward_temp=pruning_one_link_candidate(trc,true);
            // // LOG_INFO("forward_temp:{}",forward_temp.size())
            // auto back_temp=pruning_one_link_candidate(forward_temp,false);
            // // LOG_INFO("back_temp:{}",back_temp.size())

            /////////////////   log info  /////////////////
            bool is_log=true;
            if(is_log){
                auto log = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_lc");
                log->color= {255,0,0};
                for(auto trp : trc){
                    if(trp.lc){
                        auto &ele=log->add(trp.lc->pos);
                        ele.color={255,0,0};
                    }
                }

                auto log_3 = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_all_lb");
                log_3->color= {0,255,0};
                for(auto trp : trc){
                    auto &ele=log->add(trp.lb->pos);
                    ele.color={0,255,0};
                }
            }
            /////////////////   log info  /////////////////

            auto res = pruning_one_link_candidates(trc);
            if(res.size()>0){
                pruning_turn_right_candidate[mtrc.first]=res;

                if(is_log){
                    auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_nolc");
                    log_2->color= {0,0,255};
                    for(auto back_temp: res){
                        for(auto trp : back_temp){
                            if(trp.lc){
                                auto &ele=log_2->add(trp.lc->pos);
                                ele.color={0,255,0};

                                auto &ele1=log_2->add(trp.lb->pos);
                                ele1.color={255,0,0};
                            }
                        }
                    }
                }
            }

            // if(back_temp.size()>1)
            // {
            //     std::reverse(back_temp.begin(),back_temp.end());
            //     pruning_turn_right_candidate[mtrc.first]=back_temp;
    
            //     // /////////////////   log info  /////////////////
            //     // auto log = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_lc");
            //     // log->color= {255,0,0};
            //     // for(auto trp : trc){
            //     //     if(trp.lc){
            //     //         auto &ele=log->add(trp.lc->pos);
            //     //         ele.color={255,0,0};
            //     //     }
            //     // }

            //     auto log_2 = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_nolc");
            //     log_2->color= {0,0,255};
            //     for(auto trp : back_temp){
            //         if(trp.lc){
            //             auto &ele=log_2->add(trp.lc->pos);
            //             ele.color={0,255,0};

            //             auto &ele1=log_2->add(trp.lb->pos);
            //             ele1.color={255,0,0};
            //         }
            //     }

                // auto log_3 = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_all_lb");
                // log_3->color= {0,255,0};
                // for(auto trp : trc){
                //     auto &ele=log->add(trp.lb->pos);
                //     ele.color={0,255,0};
                // }
            // }
         
        }
        session->save_debug_info("turn_right_candicate");
    };

    auto add_lane_center = [&,session]()
    {
        auto merge_lane_center = [&,&session](std::vector<TurnRightPoint> &trc)
        {
            auto get_end=[](LaneCenterFeature*lc,bool is_end,double& length)
            {
                length=0;
                LaneCenterFeature*next_lc=lc;
                int count=0;
                while(next_lc)
                {
                    auto next=is_end?next_lc->next:next_lc->prev;
                    if(next)
                    {
                        length+=alg::calc_dis(next->pos,next_lc->pos);
                    }
                    if(!next)
                    {
                        break;
                    }
                    next_lc=next;
                }
                return next_lc;
            };
            for (auto &p : trc)
            {
                if(p.lc)
                {
                    double length_to_end,length_to_start;
                    get_end(p.lc,true,length_to_end);
                    get_end(p.lc,false,length_to_start);
                    // LOG_INFO("D:[{}] lc:[{}] [prev:{} next:{}] [to_end:{} to_start:{}]",p.distance,
                    //     uint64_t(p.lc),uint64_t(p.lc->prev),uint64_t(p.lc->next),length_to_end,length_to_start)
                }
                else
                {
                    // LOG_INFO("D:[{}] lc:[{}] ",p.distance,uint64_t(p.lc))
                }
            }
            auto &front = trc.front();
            auto &back = trc.back();
            double d1 = front.distance;
            double d2 = back.distance;
            LaneCenterFeature *next = NULL;
            LaneCenterFeature *prev = NULL;
            if (front.lc)
            {
                double to_end;
                LaneCenterFeature *lc = get_end(front.lc, true, to_end);
                // LOG_INFO("[turn-right] front-lc: ({}, {}, {}), end_lc: ({}, {}, {}), length: {}", front.lc->pos.x(), front.lc->pos.y(), front.lc->pos.z(), lc->pos.x(), lc->pos.y(), lc->pos.z(), to_end);
                if (to_end < 3)
                {
                    front.lc = lc;
                }
                if (front.lc)
                {
                    prev = front.lc->next;
                }
                else
                {
                    LOG_WARN("merge center front lc null");
                }

                // if (prev){
                //     LOG_INFO("[turn-right] front-lc: ({}, {}, {}), prev: ({}, {}, {}), length: {}", front.lc->pos.x(), front.lc->pos.y(), front.lc->pos.z(), prev->pos.x(), prev->pos.y(), prev->pos.z(), to_end);
                // }
            }
            if (back.lc)
            {
                double to_end;
                LaneCenterFeature *lc = get_end(back.lc, false, to_end);
                // LOG_INFO("[turn-right] back-lc: ({}, {}, {}), end_lc: ({}, {}, {}), length: {}", back.lc->pos.x(), back.lc->pos.y(), back.lc->pos.z(), lc->pos.x(), lc->pos.y(), lc->pos.z(), to_end);
                if (to_end < 3)
                {
                    back.lc = lc;
                }
                if (back.lc)
                {
                    next = back.lc->prev;
                }
                else
                {
                    LOG_WARN("merge center back lc null");
                }
                // if (next){
                //     LOG_INFO("[turn-right] back-lc: ({}, {}, {}), next: ({}, {}, {}), length: {}", back.lc->pos.x(), back.lc->pos.y(), back.lc->pos.z(), next->pos.x(), next->pos.y(), next->pos.z(), to_end);
                // }
            }
            // case 1
            if (!next && !prev && front.lc && back.lc)
            {
                LOG_INFO("ADD CASE 1")
                LOG_INFO("[turn-right] pos: ({}, {}), ({}, {})", 
                    front.lc->pos.x(), front.lc->pos.y(), back.lc->pos.x(), back.lc->pos.y())

                // TODO check pos at front
                auto &prev_lc = front.lc;
                auto &line = prev_lc->group_line;
                double distance = (d1>0&&d2>0)? (d1+d2)/2:
                                (d1>0)? d1: 
                                (d2>0)? d2: 2.6;
                for (auto &p : trc)
                {
                    auto new_lc =session->add_ptr(session->lane_center_feature_ptr);
                    new_lc->pos = alg::get_vertical_pos(p.lb->pos, p.lb->dir, -distance );
                    new_lc->dir = p.lb->dir;
                    new_lc->group_line = line;
                    new_lc->set_prev(prev_lc);
                    line->list.push_back(new_lc);
                    prev_lc = new_lc.get();
                }
                back.lc->set_prev(prev_lc);
                // for(auto p:back.lc->group_line->list)
                // {
                //    p->group_line=line;
                //    line->list.push_back(p);
                // }

                int cnt = 0;
                auto back_line=back.lc->group_line;
                int back_line_points_size =back_line->list.size();

                // LOG_INFO("[turn-right] prev-line: {}, next-line: {}, count: {}", uint64_t(line), uint64_t(back_line), line->list.size())
                for(int j = 0; j < back_line_points_size; j++)
                {
                    auto back_pt = back_line->list[j];
                    if(back_pt ==nullptr) {
                        LOG_WARN("ADD CASE 1: not normal, pls check! {} {}   [{}]", cnt, back_line->list.size(),uint64_t(back_line));
                        cnt++;
                        continue;
                    }
                    cnt++;
                    back_pt->group_line=line;
                    // back_pt->set_prev(prev_lc);
                    line->list.push_back(back_pt);
                    // prev_lc = back_pt.get();
                }
                // LOG_INFO("[turn-right] after adding, count: {}", line->list.size())

                // session->set_display_name("turn_right_candicate2");
                // /////////////////   log info  /////////////////
                // auto log = session->add_debug_log(utils::DisplayInfo::POINT, "turnright_lc-line");
                // log->color= {255,0,0};
                // int idx = 0;
                // for(auto pt : line->list){
                //     auto &ele=log->add(pt->pos);
                //     ele.color={255,0,0};
                //     ele.label.opt_label = idx++;
                // }
                // session->save_debug_info("turn_right_candicate2");

                //  删除合并后线段
                for(auto iter=session->merge_lane_center_list.begin();iter!=session->merge_lane_center_list.end();)
                {
                    if((*iter)== back_line)
                    {
                       iter= session->merge_lane_center_list.erase(iter);
                    }
                    else
                    {
                        iter++;
                    }
                }
            } // case 2
            else if ( (next||!back.lc) && (!prev&&front.lc))
            {
                LOG_INFO("ADD CASE 2")
                LOG_INFO("[turn-right] pos: ({}, {})", front.lc->pos.x(), front.lc->pos.y())

                auto &prev_lc = front.lc;
                auto &line = prev_lc->group_line;
                double distance = d1>0? d1: 2.6;
                for (auto &p : trc)
                {
                    auto new_lc =session->add_ptr(session->lane_center_feature_ptr);
                    new_lc->pos = alg::get_vertical_pos(p.lb->pos, p.lb->dir, -distance );
                    new_lc->dir = p.lb->dir;
                    new_lc->group_line=line;
                    new_lc->set_prev(prev_lc);
                    line->list.push_back(new_lc);
                    prev_lc = new_lc.get();
                }
            } // case 3
            else if ((!next&&back.lc) && (prev||!front.lc))
            {
                LOG_INFO("ADD CASE 3")
                LOG_INFO("[turn-right] pos: ({}, {})", back.lc->pos.x(), back.lc->pos.y())

                LaneCenterFeature* prev_lc = NULL;
                auto next_lc = back.lc;
                std::vector<std::shared_ptr<LaneCenterFeature>> new_line ;
                auto &line = next_lc->group_line;
                double distance = d2>0? d2: 2.6;
                for (auto &p : trc)
                {
                    auto new_lc =session->add_ptr(session->lane_center_feature_ptr);
                    new_lc->pos = alg::get_vertical_pos(p.lb->pos, p.lb->dir, -distance);
                    new_lc->dir = p.lb->dir;
                    new_lc->group_line =line;
                    if(prev_lc)
                    {
                        new_lc->set_prev(prev_lc);
                    }
                    new_line.push_back(new_lc);
                    prev_lc = new_lc.get();
                }
                if(prev_lc)
                {
                    next_lc->set_prev(prev_lc);
                }
                new_line.insert(new_line.end(),line->list.begin(),line->list.end());
                std::swap(new_line,line->list);

            }
            else // case 4
            {
                LOG_INFO("ADD CASE 4")
                LOG_INFO("[turn-right] pos: ({}, {}), ({}, {})", 
                    front.lb->pos.x(), front.lb->pos.y(), back.lb->pos.x(), back.lb->pos.y())

                LaneCenterFeature* prev_lc = NULL;
                // auto lit = session->key_pose_map.begin();
                // bool found = (lit != session->key_pose_map.end());
                // if(found) {
                    // auto  new_group=session->add_ptr(lit->second.lane_center_line_group_list);;
                    auto  new_group=session->add_ptr(session->lane_center_group_line_ptr);
                    auto &new_line=new_group->list;
                    double distance = (d1>0&&d2>0)? (d1+d2)/2:
                                (d1>0)? d1: 
                                (d2>0)? d2: 2.6;
                    for (auto &p : trc)
                    {
                        auto new_lc =session->add_ptr(session->lane_center_feature_ptr);
                        new_lc->pos = alg::get_vertical_pos(p.lb->pos, p.lb->dir, -distance );
                        new_lc->dir = p.lb->dir;
                        new_lc->group_line=new_group.get();
                        if(prev_lc)
                        {
                            new_lc->set_prev(prev_lc);
                        }
                        new_line.push_back(new_lc);
                        prev_lc = new_lc.get();
                    }
                    if(!new_line.empty())
                    {
                        session->merge_lane_center_list.push_back(new_group.get());
                    }
                // }
            }
        };
        for (auto &mtrc : pruning_turn_right_candidate)
        {
            auto &trcs = mtrc.second;
            if (trcs.empty())
            {
                continue;
            }
            for(auto &trc: trcs){
                merge_lane_center(trc);
            }
        }
    };
    //  step 1
    calc_candidate();
    //  step 2
    calc_lc_ditance();
    //  step 3
    pruning_candidate();
    //  step 4
    add_lane_center();
    // //  step 5
    // refine_endpose();
    // 
    LOG_INFO("candidate_right_boundary  sucessfully ");
    // 
    return fsdmap::SUCC;
}

void RoadModelProcBindTrail::dfs_fill_hit_status( LaneCenterFeature* prev,LaneCenterGroupLine* line,const int cur_index,const int size,const bool forward)
{
    if (cur_index > (size - 1) || cur_index < 0|| line==NULL|| size<=0 )
    {
        return;
    }
    auto cur_ptr = line->list[cur_index];
    if (cur_ptr)
    {  
        if(cur_ptr->hit_link == NULL && prev != NULL)
        {
            cur_ptr->hit_link = prev->hit_link;
            cur_ptr->hit_status |= (1 << 2);
        }else
        {
            prev = cur_ptr.get();
        }
    }
    // TODO FOR TURN RIGHT BACK
    int next_index = forward ? cur_index + 1 : cur_index - 1;
    dfs_fill_hit_status(prev, line, next_index,size, forward);
}

void RoadModelProcBindTrail::bfs_fill_hit_status(RoadModelSessionData*session,LaneCenterGroupLine *line)
{
    auto get_lane_num = [](const LaneCenterFeature *lc)
    {
        int min_lane_num = 100;
        if (!lc)
        {
            return min_lane_num;
        }
        const auto hit_pose = lc->hit_poss;
        if (hit_pose)
        {
            const auto from_raw_link = hit_pose->from_raw_link;
            if (from_raw_link)
            {
                return from_raw_link->lanenum_sum;
            }
        }
        return min_lane_num;
    };
    
    RTreeProxy<KeyPose*, float, 2> link_tree; 
    for(auto link: session->link_sample_list)
    {
         for(auto poss:link->list)
         {  
            link_tree.insert(poss->pos,poss);
        }
    }
    auto cross_link=[&link_tree](LaneCenterFeature*lc )
    {
        auto p1=alg::get_vertical_pos(lc->pos,lc->dir,+20);
        auto p2=alg::get_vertical_pos(lc->pos,lc->dir,-20);
        std::vector<KeyPose*> poses;
        link_tree.search(lc->pos,30,poses);
        for(auto next:poses)
        {

            if(!next->next)
            {
                continue;
            }
            // 同一条link
            if(lc->hit_link!=next->from_link)
            {
                continue;
            }
            Eigen::Vector3d cross;
            if(alg::findIntersection(p1,p2,next->pos,next->next->pos,cross))
            {
                
                return true;
            }
        }
        return false;
    };
    std::vector<LaneCenterFeature *> candidate_seeds;
    for (auto lc : line->list)
    {
        if (!lc)
        {
            continue;
        }
        if (lc->hit_link)
        {
            candidate_seeds.push_back(lc.get());
        }
    }
    std::sort(candidate_seeds.begin(), candidate_seeds.end(),
              [get_lane_num](const LaneCenterFeature *a, const LaneCenterFeature *b)
              {
                  return get_lane_num(a) < get_lane_num(b);
              });
    std::set<LaneCenterFeature *> hash_tabel;
    const int n = line->list.size();
    for (auto seed : candidate_seeds)
    {
        // LOG_INFO("lane_num:{}",get_lane_num(seed));
        if (hash_tabel.count(seed))
        {
            continue;
        }
        std::queue<LaneCenterFeature *> next_seed;
        next_seed.push(seed);
        hash_tabel.insert(seed);
        int count = 0;
        while (!next_seed.empty())
        {
            count++;
            auto front = next_seed.front();
            next_seed.pop();
            auto neighbor={front->next,front->prev};
            for (auto &next : neighbor)
            {
                if (next == NULL)
                {
                    continue;
                }
                if (hash_tabel.count(next))
                {
                    continue;
                }
                // 不在link 范围
                if(!cross_link(front))
                {
                    continue;
                }
                if (front->hit_link && next->hit_link == NULL)
                {
                    next->hit_link = front->hit_link;
                    next->hit_status |= (1 << 2);
                    next_seed.push(next);
                    hash_tabel.insert(next);
                }
            }
        }
        // LOG_INFO("label_count:{}", count);
    }
}

int RoadModelProcBindTrail::hit_lane_center_fill_neighbor(RoadModelSessionData* session)
{
    // 优先车道数量少的道
    for(auto line:session->merge_lane_center_list)
    {
        bfs_fill_hit_status(session,line);
        // test ok only for turn right maybe miss to foward 
        // const int n=line->list.size();
        // LaneCenterFeature* prev1=NULL;
        // dfs_fill_hit_status(prev1,line,0,n,true);
        // LaneCenterFeature* prev2=NULL;
        // dfs_fill_hit_status(prev2,line,n-1,n,false);
    }
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::lane_center_bind_pose(RoadModelSessionData* session)
{
    RTreeProxy<KeyPose*, float, 2> link_tree; 
    for(auto link: session->link_sample_list)
    {
         for(auto poss:link->list)
         {  
            link_tree.insert(poss->pos,poss);
        }
    }
    for(auto &line:session->merge_lane_center_list)
    {
        for(auto &lc:line->list)
        {
            if(!lc->hit_link)
            {
                continue;
            }
            if(!lc->hit_poss)
            {
                continue;
            }
            auto p1=alg::get_vertical_pos(lc->pos,lc->dir,+20);
            auto p2=alg::get_vertical_pos(lc->pos,lc->dir,-20);
            std::vector<KeyPose*> poses;
            link_tree.search(lc->pos,30,poses);
            for(auto next:poses)
            {
                if(next->from_link!=lc->hit_link)
                {
                    continue;
                }
                if(!next->next)
                {
                    continue;
                }
                Eigen::Vector3d cross;
                if(alg::findIntersection(p1,p2,next->pos,next->next->pos,cross))
                {
                    lc->hit_poss=next;
                    break;
                }
            }
        }
    }
    return fsdmap::SUCC;
}
int RoadModelProcBindTrail::reverse_lane_center(RoadModelSessionData* session)
{
    for(auto line:session->merge_lane_center_list)
    {
         int oppo=0;
         int n=line->list.size();
        for(auto lc:line->list)
        {
             auto hit_poss=lc->hit_poss;
             if(!hit_poss)
             {
                continue;
             }
             if(alg::calc_theta1(hit_poss->dir,lc->dir,true)>120)
             {
              oppo++;
             }     
        }
        double factor=static_cast<double>(oppo)/static_cast<double>(n);
        if(oppo>0)
        {

            // for(int i=0;i<n;i++)
            // {
            //     auto lc=line->list[i];
            //     LOG_INFO("bef [prev:{} cur:{} next:{}]",uint64_t(lc->prev),uint64_t(lc.get()),uint64_t(lc->next));
            // }
            LaneCenterFeature*next=NULL;
            auto cur=line->list.front().get();
            while(cur)
            {
                LaneCenterFeature*temp=cur->next;
                cur->next=next;
                cur->prev=temp;
                next=cur;
                cur=temp;
            }
            // for(int i=0;i<n;i++)
            // {
            //     auto lc=line->list[i];
            //     LOG_INFO("after [prev:{} cur:{} next:{}]",uint64_t(lc->prev),uint64_t(lc.get()),uint64_t(lc->next));
            // }
            std::reverse(line->list.begin(),line->list.end());
            for(int i=1;i<n;i++)
            {
                auto dir=alg::get_dir(line->list[i]->pos,line->list[i-1]->pos);
                line->list[i-1]->dir=dir;
                line->list[i]->dir=dir;
            }
            oppo_lane_center.push_back(line);
            LOG_WARN("LANE CENTER NEED INVERSE:[{}: {}/{}]  [{}]",factor,oppo,n,oppo_lane_center.size())
        }
    }
    return fsdmap::SUCC;
}
int RoadModelProcBindTrail::hit_lane_center_candidate(RoadModelSessionData* session)
{
    RTreeProxy<LaneCenterFeature*, float, 2> lane_center_tree; 
    for(auto line:session->merge_lane_center_list)
    {
        for(auto pt:line->list)
        {
            lane_center_tree.insert(pt->pos,pt.get());
        }
    }
    auto mark_hit_lane_center = [&lane_center_tree](KeyPose *poss)
    {
        // TODO CHECK why have two 
        // if(poss->boundary.road_center_list.size()>1)
        // {
        //     LOG_INFO("road_center_list:{}",poss->boundary.road_center_list.size());
        // }
        std::vector<CandidateSeed<BoundaryFeature>> candidate_seed;
        for (auto &rc : poss->boundary.road_center_list)
        {
            auto left = rc->left;
            auto right = rc->right;
            if (!left || !right)
            {
                return;
            }
 
            double radius = alg::calc_dis(left->pos, right->pos) * 1.414;
            CandidateSeed<BoundaryFeature> new_cadidate;
            new_cadidate.ps=left;
            new_cadidate.pe=right;
            new_cadidate.radius=radius;
            new_cadidate.type="boundary";
            candidate_seed.push_back(new_cadidate);
        }
        // CandidateSeed new_cadidate;
        // new_cadidate.ps=alg::get_vertical_pos(poss->pos,poss->dir,3.75/2);
        // new_cadidate.pe=alg::get_vertical_pos(poss->pos,poss->dir,-3.75/2);
        // new_cadidate.radius=(3.75+0.5)*1.414;
        // new_cadidate.type="link";
        // candidate_seed.push_back(new_cadidate);

        for(auto candiate:candidate_seed)
        {
            std::vector<LaneCenterFeature *> secs;
            lane_center_tree.search(poss->pos, candiate.radius, secs);
            // LOG_INFO("find_sec:{}",secs.size());
            for (auto lc : secs)
            {
                auto neighbors = {lc->next, lc->prev};
                for (auto neighbor : neighbors)
                {
                    if (!neighbor)
                    {
                        // LOG_INFO("find_sec:{}",secs.size());
                        continue;
                    }
                    Eigen::Vector3d cross;
                    if (alg::findIntersection(candiate.ps->pos, candiate.pe->pos, lc->pos, neighbor->pos, cross))
                    {
                        // LOG_INFO("hit ");
                        lc->hit_link = poss->from_link;
                        lc->hit_poss = poss;
                        // TODO MARK DIFFERENT TYPE 
                        lc->hit_status|=(1<<1);
                    }
                }
            }
        }
    };

    //  TODO 增加优先级  优先扩展提右
   for(auto link: session->link_sample_list)
   {
        for(auto poss:link->list)
        {  
            mark_hit_lane_center(poss);
        } 
   }
   return fsdmap::SUCC;
}

int RoadModelProcBindTrail::fix_loss_road_boundary(RoadModelSessionData *session)
{
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    auto mark_hit_lane_center = [&lane_center_tree](KeyPose *poss)
    {
        std::vector<LaneCenterFeature *> secs;
        lane_center_tree.search(poss->pos, 20, secs);
        auto p1 = alg::get_vertical_pos(poss->pos, poss->dir, +20);
        auto p2 = alg::get_vertical_pos(poss->pos, poss->dir, -20);
        for (auto lc : secs)
        {
            auto neighbors = {lc->next, lc->prev};
            for (auto neighbor : neighbors)
            {
                // 
                if (lc->hit_link)
                {
                    continue;
                }

                if (!neighbor)
                {
                    continue;
                }
                if (alg::calc_theta1(lc->dir, poss->dir, true) > 30)
                {
                    continue;
                }
                Eigen::Vector3d cross;
                if (alg::findIntersection(p1, p2, lc->pos, neighbor->pos, cross))
                {
                    // LOG_INFO("hit ");
                    lc->hit_link = poss->from_link;
                    lc->hit_poss = poss;
                    // TODO MARK DIFFERENT TYPE
                    lc->hit_status |= (1 << 4);
                }
            }
        }
    };
    for (auto link : session->link_sample_list)
    {
        if(link->list.empty())
        {
          continue;
        }
        auto front=link->list.front();
        auto from_raw_link = front->from_raw_link;
        if (!from_raw_link)
        {
           continue;
        }

        // int from = std::stoi(from_raw_link->form);
        // if(from==25||from==26)
        if(alg::match_any_with_forms(from_raw_link->forms, {25, 26}))
        {
          continue;
        }
        //   TODO online 
        for (auto poss : link->list)
        {
            mark_hit_lane_center(poss);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::fill_lane_boundary(RoadModelSessionData* session)
{
    RTreeProxy<LaneLineSample *, float, 2> lane_boundary_tree;
    for (auto line : session->merge_lane_line_list)
    {
        for (auto pt : line->list)
        {
            lane_boundary_tree.insert(pt->pos, pt.get());
        }
    }
    //
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    auto cross_range_lc = [&lane_center_tree](Eigen::Vector3d pos, LaneCenterFeature *lc, double range)
    {
        // bool open_debug = true;
        bool open_debug = false;
        std::vector<LaneCenterFeature *> lcfs;
        lane_center_tree.search(pos, range * 1.414 * 2, lcfs);
        double min_dis = 100;
        for (auto clc : lcfs)
        {
            if (!clc->next)
            {
                continue;
            }
            if (clc->group_line == lc->group_line)
            {
                continue;
            }
            Eigen::Vector3d cross_point;
            bool found = alg::findIntersection(pos, lc->pos, clc->pos, clc->next->pos, cross_point);
            double dis = alg::calc_dis(lc->pos, cross_point);
        
            if (found && dis > 0.01) // 为了计算中心线的绑定关系，1cm内的点不算相交
            {
                if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                    std::cout << lc->pos.transpose() << std::endl;
                    LOG_ERROR("qzc 0 :dis  {}", dis);
                }
                return true;
            }
        }
        return false;
    };
    //
    auto match_line = [&session, &lane_center_tree, &lane_boundary_tree, cross_range_lc]
                        (LaneCenterFeature *lc, bool is_left, double &lpf_dis, int is_init, double &last_dis, LaneCenterFeature** last_lc, LaneLineSampleGroupLine** last_group_line)
    {
        // bool open_debug = true;
        bool open_debug = false;
        if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
        // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
            std::cout << lc->pos.transpose() << " lc:" << lc << std::endl;
            LOG_ERROR("qzc 1 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{}, id:{}, lc->point_status:{}", 
                is_left, lpf_dis, is_init, lc->init_lb_status, lc->group_line->id, lc->point_status);
        }

        Eigen::Vector3d lane_dir = lc->dir;
        if(lc->point_status == 1 && lc->endpoint_status == 2) {
            auto lane_dir1 = alg::get_vertical_dir(lc->v_road_dir);
            double angle = alg::calc_theta1(lc->dir, lane_dir1, true);
            if (angle > 90.0) {
                lane_dir = -lane_dir1;
            } else {
                lane_dir = lane_dir1;
            }
            if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
            // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                LOG_ERROR("qzc 111 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{}, id:{}, angle:{} n xyz:{},{},{} lane_dir1:{},{},{} lc->dir:{},{},{} lane_dir:{},{},{}", 
                    is_left, lpf_dis, is_init, lc->init_lb_status, lc->group_line->id, angle, 
                    lc->pos.x(), lc->pos.y(), lc->pos.z(), 
                    lane_dir1.x(), lane_dir1.y(), lane_dir1.z(), 
                    lc->dir.x(), lc->dir.y(), lc->dir.z(),
                    lane_dir.x(), lane_dir.y(), lane_dir.z()
                    );
            }
        }
        // if (lc->point_status == 1) {
        //     // 1. 先初步用附近则中心线，追踪到道路方向 v1
        //     std::vector<Eigen::Vector3d> dirs;
        //     std::vector<LaneCenterFeature *> lcfs;
        //     lane_center_tree.search(lc->pos, 15, lcfs);
        //     if (!lcfs.empty()) {
        //         for (const auto lcf : lcfs) {
        //             if(lcf->curvature<0.01){
        //                 dirs.push_back(lcf->dir);
        //             }
        //         }
        //     }
        //     lane_dir = alg::get_direction(dirs);
        //     if (alg::calc_theta1(lc->dir, lane_dir, true) > 90) {
        //         lane_dir = -lane_dir;
        //     }
        // }

        double match_line_max_ange_diff = 30;
        double standard_lane_width = FLAGS_standard_lane_width; // 3.5m
        double half_standard_lane_width = standard_lane_width * 0.5;
        std::vector<LaneLineSample *> llss;
        lane_boundary_tree.search(lc->pos, standard_lane_width * 1.414 * 2, llss);
        

        double ratio_search_range = 0.85; // 普通情况：3.5*0.85 = 2.975米
        if(lc->point_status == 1) {
            ratio_search_range = 1.0; // 分合流：3.5米
        }
        std::vector<BreakInfo*> bk_points;
        double search_radius = 0.1;
        session->sm_break_candidate_tree.search(lc->pos, search_radius, bk_points);
        if (bk_points.size() > 0) {
            ratio_search_range = 1.0; // 分合流：3.5米
        }

        auto p1 = alg::get_vertical_pos(lc->pos, lane_dir, -standard_lane_width * ratio_search_range);
        auto p2 = alg::get_vertical_pos(lc->pos, lane_dir, +standard_lane_width * ratio_search_range);
        double min_dis = 100; // 中心线到车道边界线的距离
        double origin_dis = 100;
        fast_road_model::LBPoint lb;
        LaneLineSampleGroupLine* found_group_line;
        for (auto lls : llss)
        {
            double min_theta = alg::calc_theta1(lls->dir, lane_dir, true);
            min_theta = std::min(min_theta, 180 - min_theta);
            if (min_theta > match_line_max_ange_diff) // 30 度
            {
                continue;
            }
            if (!lls->next)
            {
                continue;
            }
            Eigen::Vector3d cross_point;
            if (!alg::findIntersection(p1, p2, lls->pos, lls->next->pos, cross_point))
            {
                continue;
            }
            if (is_left ^ (alg::judge_left(cross_point, lc->pos, lane_dir) < 0))
            { // left==1,则筛选出左侧的车道线点
                continue;
            }
            origin_dis = alg::calc_dis(cross_point, lc->pos); // 半个车道宽
            double dis = origin_dis; // 半个车道宽
            if (cross_range_lc(lls->pos, lc, dis)) // 如果车道线点与中心线之间还存在另外一条中心线，则不处理该点
            {
                continue;
            }

            // 规避右转中心线的左边线找到直行的右边线
            if(is_left && !lc->hit_poss){
                continue;
            }
            if(is_left && alg::match_any_with_forms(lc->hit_poss->from_raw_link->forms, {25, 26})){
                auto lc_end = lc->group_line->list.back();
                auto lls_end = lls->group_line->list.back();
                Eigen::Vector3d cross_point2;
                if (alg::findIntersection(lc->pos, lc_end->pos, lls->pos, lls_end->pos, cross_point2))
                {
                    continue;
                }
            }

            // 和标准宽度差距
            dis = std::abs(dis - half_standard_lane_width); // (dis - 1.75)
            // 和上一次宽度差值
            double diff = std::abs(lpf_dis - dis);
            //  等待初始化： 要找车道线宽度不跳变的0.5m的
            double ll_width_diff_thres = FLAGS_lane_boundary_width_diff_thres;

            // 分合流参数
            bool is_sm = false;
            if (lc->point_status == 1) {
                ll_width_diff_thres = 0.6;
                // is_sm = is_init \
                //             && fabs(origin_dis - last_dis) < 0.5 \
                //             && (*last_lc) != NULL \
                //             && (*last_lc)->next == lc \
                //             && (*last_group_line) != NULL \
                //             && lls->group_line == *last_group_line;
            }

            bool is_not_junp = !is_init || diff < ll_width_diff_thres; // 0.5 m
            if (dis < min_dis && is_not_junp || is_sm)
            {
                if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
                // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                    std::cout << lc->pos.transpose() << " lc:" << lc << " last_group_line " << *last_group_line << " cur gl :" << lls->group_line << std::endl;
                    LOG_ERROR("qzc 22 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{},  origin_dis:{}, last_dis {}, dis:{} min_dis:{}, is_sm:{}", 
                        is_left, lpf_dis, is_init, lc->init_lb_status,
                        origin_dis, last_dis, dis, min_dis, is_sm
                        );
                }

                if(is_init && fabs(origin_dis - last_dis) > 0.5
                    && (*last_lc) != NULL
                    && (*last_lc)->next == lc 
                    && (*last_group_line) != NULL 
                    && lls->group_line != *last_group_line) {

                    if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
                    // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                        std::cout << lc->pos.transpose() << " lc:" << lc << " last_group_line " << *last_group_line << " cur gl :" << lls->group_line << std::endl;
                        LOG_ERROR("qzc 23 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{},  origin_dis:{}, last_dis {}, is_sm:{}", 
                            is_left, lpf_dis, is_init, lc->init_lb_status,
                            origin_dis, last_dis, is_sm
                            );
                    }

                    continue;
                }
                
                min_dis = dis;
                Eigen::Vector3d dir = lls->dir; // 车道线方向
                if (alg::calc_theta1(lane_dir, dir) > 120)
                {
                    dir = -dir;
                }
                // TODO:qzc 为啥不用交点的位置，原始的更准确吗？
                lb.init(lls->pos, lls->dir);
                lb.type = lls->attr.type;
                lb.color = lls->attr.color;
                // std::cout<<"--**-- type: "<<lb.type<<", color: "<<lb.color<<std::endl;
                // LOG_INFO("--------******-------- type: {}, color: {}", lb.type, lb.color);
                
                found_group_line = lls->group_line;
            }
            else
            {
                // LOG_INFO("min_dis diff[{} {} ]", dis, diff)
                if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
                // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                    LOG_ERROR("qzc 24 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{},  origin_dis:{}, last_dis {}, dis:{} min_dis:{}, is_sm:{}", 
                        is_left, lpf_dis, is_init, lc->init_lb_status,
                        origin_dis, last_dis, dis, min_dis, is_sm
                        );
                }
            }
        }

        if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
        // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
            std::cout << lc->pos.transpose() << std::endl;
            LOG_ERROR("qzc 2 :is_left  {} lpf_dis : {} is_init: {}, origin_dis:{} min_dis:{} ratio_search_range:{}", is_left, lpf_dis, is_init, origin_dis, min_dis, ratio_search_range);
        }

        last_dis = 0;
        (*last_lc) = NULL;
        (*last_group_line) = NULL;

        // 超过1个车道+0.25半车道宽度 及如果前面匹配不到车道线点，则退出
        double ratio_half_lane_width = 1.25; // 普通情况：3.5/2 * 1.25 = 2.1875米
        if (min_dis > half_standard_lane_width * ratio_half_lane_width) {
            return false;
        }
        // 正常道路
        auto &boundary = is_left ? lc->left_lb : lc->right_lb;
        auto &ref_boundary = !is_left ? lc->left_lb : lc->right_lb;
        boundary = lb;
        if(boundary.type == 0 || boundary.type == 99){
            if(ref_boundary.type != 0){
                boundary.type = ref_boundary.type;
                boundary.color = ref_boundary.color;
            }
        }

        if(ref_boundary.type == 0 || ref_boundary.type == 99){
            ref_boundary.type = boundary.type;
            ref_boundary.color = boundary.color;
        }

        if (is_left)
        {
            lc->init_lb_status |= (1 << 1);
        }
        else
        {
            lc->init_lb_status |= (1 << 2);
        }
        // TODO cut lc-lane_boundar/lc
        if (!is_init)
        {
            lpf_dis = min_dis;
            last_dis = origin_dis;
            *last_group_line = found_group_line;
            *last_lc = lc;
        }
        else
        {
            lpf_dis = 0.75 * lpf_dis + 0.25 * min_dis;
            last_dis = origin_dis;
            *last_group_line = found_group_line;
            *last_lc = lc;
        }
        if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
        // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
            std::cout << lc->pos.transpose() << " last_group_line: " << *last_group_line << std::endl;
            LOG_ERROR("qzc 3 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{}", is_left, lpf_dis, is_init, lc->init_lb_status);
        }

        return true;
    };
    auto init_boundary_pos = [](LaneCenterFeature *lc, bool is_left, double dis)
    {
        auto &boundary = is_left ? lc->left_lb : lc->right_lb;
        if (is_left)
        {
            auto p1 = alg::get_vertical_pos(lc->pos, lc->dir, -dis);
            // auto p1 = alg::get_vertical_pos(lc->pos, lc->dir, 0/ 2);
            boundary.init(p1, lc->dir);
        }
        else
        {
            auto p1 = alg::get_vertical_pos(lc->pos, lc->dir, +dis);
            //    auto p1 = alg::get_vertical_pos(lc->pos, lc->dir, 0 / 2);
            boundary.init(p1, lc->dir);
        }
    };
    // bool open_debug = true;
    bool open_debug = false;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto is_left : {true, false})
        {
            int count = 0;
            bool is_init = false;
            double lpf_dis = 0; // 初始化和持续击中会进行低通 修改
            double last_dis = 0; // 绝对的差值
            LaneLineSampleGroupLine* last_group_line = NULL; // 记录上一次匹配上的线
            LaneCenterFeature* last_lc = NULL;


            bool debug_log_tmp = false;
            for (auto lc : line->list)
            {
                
                if (!match_line(lc.get(), is_left, lpf_dis, is_init, last_dis, &last_lc, &last_group_line))//设置lc->init_lb_status的状态
                {
                    if (count++ > 5)
                    {
                        is_init = false;
                        lpf_dis = 0;
                        last_dis = 0;
                        last_group_line =NULL;
                        last_lc=NULL;
                    }
                    init_boundary_pos(lc.get(), is_left, lpf_dis);
                }
                else
                {
                    is_init = true;

                }
                //    LOG_INFO("lpf_dis:{}",lpf_dis)
                if(open_debug && -169.06<lc->pos.x() && lc->pos.x() < -168.07 && 210.10<lc->pos.y() && lc->pos.y() < 210.25) {
                // if(open_debug && -84.41<lc->pos.x() && lc->pos.x() < -84.35 && 115.5<lc->pos.y() && lc->pos.y() < 115.6) {
                    std::cout << lc->pos.transpose() << " lc:" << lc  << " last_group_line: " << last_group_line << std::endl;
                    LOG_ERROR("qzc 8 :is_left  {} lpf_dis : {} is_init: {}, lc->init_lb_status:{}, id:{}", 
                        is_left, lpf_dis, is_init, lc->init_lb_status, lc->group_line->id);
                    debug_log_tmp = true;
                }

            }
            //    LOG_INFO("lpf SPLID ")
            if(open_debug && debug_log_tmp) {
                // std::cout << lc->pos.transpose() << " lc:" << lc << std::endl;
                LOG_ERROR("qzc 9 :is_left  {} lpf_dis : {} is_init: {}", 
                    is_left, lpf_dis, is_init);
            }
        }
   }
 return fsdmap::SUCC;
}

void dfs_to_next(LaneCenterFeature *lc, bool is_left, double &length, std::set<LaneCenterFeature *> hash_table, std::vector<LaneCenterFeature *> &path, bool forward)
{
    if (!lc)
    {
        return;
    }
    if (hash_table.count(lc))
    {
        return;
    }
    
    // auto &boundary = is_left ? lc->left_lb : lc->right_lb;
    bool set_left = alg::is_bit_set(lc->init_lb_status, 1) | alg::is_bit_set(lc->init_lb_status, 3);
    bool set_right = alg::is_bit_set(lc->init_lb_status, 2) | alg::is_bit_set(lc->init_lb_status, 4);
    bool is_next = is_left ? !set_left : !set_right;
    if (is_next)
    {
        hash_table.insert(lc);
        path.push_back(lc);
        auto &next = forward ? lc->next : lc->prev;
        if (next)
        {
            length += alg::calc_dis(lc->pos, next->pos);
            // LOG_INFO("length:{}",length);
        }
        dfs_to_next(next, is_left, length, hash_table, path, forward);
        // TODO 回溯

        //
    }
}



void dfs_to_next_sm(LaneCenterFeature *lc, bool is_left, double &invalid_length,
            double &length, std::set<LaneCenterFeature *> hash_table, 
            std::vector<LaneCenterFeature *> &path, bool forward)
{
    if (!lc)
    {
        return;
    }
    if (hash_table.count(lc))
    {
        return;
    }

    bool set_left = alg::is_bit_set(lc->init_lb_status, 1);
    bool set_right = alg::is_bit_set(lc->init_lb_status, 2);
    bool is_high_confidence = is_left ? !set_left : !set_right;

    if (lc->point_status==1)
    {
        hash_table.insert(lc);
        path.push_back(lc);
        auto &next = forward ? lc->next : lc->prev;
        if (next)
        {
            if(is_high_confidence == false) {
                invalid_length += alg::calc_dis(lc->pos, next->pos);
            }
            length += alg::calc_dis(lc->pos, next->pos);
            // LOG_INFO("length:{}",length);
        }
        dfs_to_next_sm(next, is_left, invalid_length, length, hash_table, path, forward);
        // TODO 回溯

        //
    }
}

int RoadModelProcBindTrail::expend_fill_lane_boundary(RoadModelSessionData *session)
{

    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    auto cross_range_lc = [&lane_center_tree](Eigen::Vector3d pos, LaneCenterFeature *lc, double range)
    {
        std::vector<LaneCenterFeature *> lcfs;
        lane_center_tree.search(pos, range * 1.414 * 2, lcfs);
        double min_dis = 100;
        for (auto clc : lcfs)
        {
            if (!clc->next)
            {
                continue;
            }
            if (clc->group_line == lc->group_line)
            {
                continue;
            }
            Eigen::Vector3d cross_point;
            bool found = alg::findIntersection(pos, lc->pos, clc->pos, clc->next->pos, cross_point);
            double dis = alg::calc_dis(lc->pos, cross_point);
            if (found && dis > 1e-4) // 为了计算中心线的绑定关系，0.5cm内的点不算相交
            {
                return true;
            }
        }
        return false;
    };

    auto set_lane_boundary = [cross_range_lc, &session](std::vector<LaneCenterFeature *> &path, double length, bool is_left, bool need_update)
    {
        auto get_dis = [](LaneCenterFeature *lc, bool is_left)->double
        {
            double ret = -1;
            if (!lc)
            {
                return ret;
            }
            bool set_left = alg::is_bit_set(lc->init_lb_status, 1) || alg::is_bit_set(lc->init_lb_status, 3);
            bool set_right = alg::is_bit_set(lc->init_lb_status, 2) || alg::is_bit_set(lc->init_lb_status, 4);
            bool is_set = is_left ? set_left : set_right;
            if (is_set)
            {
                double left_dis = alg::calc_dis(lc->pos, lc->left_lb.pos);
                double right_dis = alg::calc_dis(lc->pos, lc->right_lb.pos);
                ret = is_left ? left_dis : right_dis;
            }
            return ret;
        };

        auto get_dis_oppo = [](LaneCenterFeature *lc, bool is_left)->double
        {
            double ret = -1;
            if (!lc)
            {
                return ret;
            }
            bool set_left = alg::is_bit_set(lc->init_lb_status, 1) || alg::is_bit_set(lc->init_lb_status, 3);
            bool set_right = alg::is_bit_set(lc->init_lb_status, 2) || alg::is_bit_set(lc->init_lb_status, 4);
            bool is_set = is_left ? set_right : set_left;
            if (is_set)
            {
                double left_dis = alg::calc_dis(lc->pos, lc->right_lb.pos);
                double right_dis = alg::calc_dis(lc->pos, lc->left_lb.pos);
                ret = is_left ? left_dis : right_dis;
            }
            return ret;
        };


        if(path.empty())
        {
            return ;
        }
        auto prev = path.front();
        auto next = path.back();
        double d1 = get_dis(prev->prev,is_left);
        if(need_update && prev->point_status==1 && !prev->prev) {
            d1 = get_dis_oppo(prev,is_left);
            // LOG_WARN("d1:[{} ",d1);
        }

        double d2 = get_dis(next->next,is_left);
        if(need_update && next->point_status==1 && !next->next) {
            d2 = get_dis_oppo(next,is_left);
            // LOG_WARN("d2:[{} ",d2);
        }
        double cur_length = 0;
        for (auto &p : path)
        {
            if(!p)
            {
                continue;
            }
            double calc_width = std::max(d1, d2);
            if (calc_width < 0)
            {
                calc_width = FLAGS_standard_lane_width * 0.5;
            }
            if (d1 >= 0 && d2 >= 0)
            {   
                if(prev&&p)
                {
                    cur_length += alg::calc_dis(prev->pos, p->pos);
                }
                double score = cur_length / length;
                calc_width = (1 - score) * d1 + score * d2;
                prev = p;
            }
            // LOG_INFO("calc_width:{}",calc_width);
            calc_width = calc_width * (is_left ? -1 : +1);
            auto pos = alg::get_vertical_pos(p->pos, p->dir, calc_width);
            auto &lb = is_left ? p->left_lb : p->right_lb;
            auto &ref_lb = !is_left ? p->left_lb : p->right_lb;
            // if(cross_range_lc(pos,p,calc_width))
            // {
            //     continue;
            // }
            lb.init(pos, p->dir);
            int num_bit = is_left ? 3 : 4;
            p->init_lb_status |= 1 << num_bit;

            if(lb.type == 0 || lb.type == 99){
                if(ref_lb.type != 0){
                    lb.type = ref_lb.type;
                    lb.color = ref_lb.color;
                }
            }

            if(ref_lb.type == 0 || ref_lb.type == 99){
                ref_lb.type = lb.type;
                ref_lb.color = lb.color;
            }
        }
    };

    for (auto &line : session->merge_lane_center_list)
    {
        for (auto is_left : {true, false})
        {
            for(auto forward:{true})
            {
                for(auto &lc:line->list)
                {
                    std::set<LaneCenterFeature *> hash_table;
                    std::vector<LaneCenterFeature *> path;
                    double length=0;
                    // 找到没有初始化边界的区间
                    dfs_to_next(lc.get(),is_left,length,hash_table,path,forward);
                    if(!path.empty())
                    {
                        // LOG_INFO("expend_fill_lane_boundary:[{}   {} ] ",path.size(),length);
                        set_lane_boundary(path,length,is_left, false);
                    }

                }
            }

        }
    }

    // 分合流重新刷一遍车道线边界位置
    // TODL：后续还可以继续优化
    // for (auto &line : session->merge_lane_center_list)
    // {
    //     for (auto is_left : {true, false})
    //     {
    //         for(auto forward:{true})
    //         {
    //             for(auto &lc:line->list)
    //             {
    //                 std::set<LaneCenterFeature *> hash_table;
    //                 std::vector<LaneCenterFeature *> path;
    //                 double invalid_length=0;
    //                 double length=0;
    //                 dfs_to_next_sm(lc.get(),is_left, invalid_length, length,hash_table,path,forward);
    //                 if(!path.empty())
    //                 {
    //                     // LOG_INFO("expend_fill_lane_boundary:[{}   {} ] ",path.size(),length);
    //                     LOG_INFO("expend_fill_lane_boundary:[{}   {}  {}] ",path.size(),length, invalid_length / length);
    //                     if (length > 0 && invalid_length / length > 0.2) {
    //                         set_lane_boundary(path,length,is_left, true);
    //                     } else {
    //                         set_lane_boundary(path,length,is_left, false);
    //                     }
    //                 }
    //             }
    //         }
    //     }
    // }

    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::change_sm_lane_boundary(RoadModelSessionData* session)
{
    bool open_log = false;
    auto sm_fitting = [&](const std::vector<Eigen::Vector3d>& control_pnts, const Eigen::Vector3d& start_dir, const Eigen::Vector3d& end_dir,
                          std::vector<Eigen::Vector3d>& fitted_pnts){
        SplineParameters params;
        params.use_chord_length = true;
        params.use_all_pnt_constrain = false;
        params.lambda1 = 10.0;
        params.lambda2 = 0.0001;
        SplineInterpolator interpolator(params);
        std::cout << "\n开始拟合样条曲线..." << std::endl;
        if (!interpolator.fitSpline(control_pnts, start_dir, end_dir)) {
            std::cerr << "拟合失败!" << std::endl;
            // return -1;
            return;
        }

        // 显示系数矩阵
        std::cout << "\n系数矩阵:" << std::endl;
        const auto& coeffs = interpolator.getCoefficients();
        std::cout << std::fixed << std::setprecision(6);
        for (int i = 0; i < 3; ++i) {
            char coord = (i == 0) ? 'x' : (i == 1) ? 'y' : 'z';
            std::cout << "  " << coord << ": ["
                    << coeffs(i, 0) << ", " << coeffs(i, 1) << ", "
                    << coeffs(i, 2) << ", " << coeffs(i, 3) << "]" << std::endl;
        }
        // interpolator.visualize2D("smooth_line1", 1000, 1000);
        // cv::waitKey(0);

        fitted_pnts = interpolator.resample_pnts();
    };

    // 处理所有分合流
    for (int i = 0; i < session->sm_matchs.size(); i++) {
        // 处理每一处分合流
        auto& one_sm_match = session->sm_matchs[i];
        auto& sm_fit_lines = one_sm_match->sm_fit_lines;
        // 处理每一处分合流的每条中心线
        LOG_INFO("[change_sm_lane_boundary] reconnected size: {}", sm_fit_lines.center_lines_reconnected.size());
        for (int j = 0; j < sm_fit_lines.center_lines_reconnected.size(); j++) {
            if (sm_fit_lines.valid_status[j].is_valid == false) {
                continue;
            }

            // 每条中心线上的点
            auto& lc_pnts = sm_fit_lines.center_lines_reconnected[j];
            int lc_pnts_size = lc_pnts.size();
            auto& line_match = one_sm_match->line_matchs[j];
            if(lc_pnts_size >= 2 && lc_pnts[0]->endpoint_status == 2 && lc_pnts[lc_pnts_size-1]->endpoint_status == 2) {
                LOG_INFO("[change_sm_lane_boundary] 1 reconnected start {} {} -> end {} {} match_status: {}", 
                            lc_pnts[0]->point_status, lc_pnts[lc_pnts_size-1]->point_status,
                            lc_pnts[0]->endpoint_status, lc_pnts[lc_pnts_size-1]->endpoint_status,
                            std::bitset<8>(line_match.match_status).to_string());
            } else if(lc_pnts_size >= 2 && lc_pnts[0]->point_status == 1 && lc_pnts[lc_pnts_size-1]->point_status == 1) {
                LOG_INFO("[change_sm_lane_boundary] 2 reconnected start {} {} -> end {} {} match_status: {}", 
                            lc_pnts[0]->point_status, lc_pnts[lc_pnts_size-1]->point_status,
                            lc_pnts[0]->endpoint_status, lc_pnts[lc_pnts_size-1]->endpoint_status,
                            std::bitset<8>(line_match.match_status).to_string());
                
                if(0) 
                {
                    // 先拟合中心线看看
                    std::vector<Eigen::Vector3d> to_fit_pnts;
                    // to_fit_pnts = {
                    //     lc_pnts[0]->pos,
                    //     lc_pnts[lc_pnts.size()-1]->pos
                    // };

                    for (int k = 0; k < lc_pnts.size(); k++) {
                        to_fit_pnts.push_back(lc_pnts[k]->pos);
                    }
                    double scale = 50.0;// TODO：先根据距离调整一波，后面排查一下原因
                    Eigen::Vector3d start_dir = scale * lc_pnts[0]->dir;
                    Eigen::Vector3d end_dir = scale * lc_pnts[lc_pnts.size()-1]->dir;
                    
                    
                    std::vector<Eigen::Vector3d> fitted_pnts;
                    sm_fitting(to_fit_pnts, start_dir, end_dir, fitted_pnts);

                    std::cout << "pos[0]: " << lc_pnts[0]->pos.transpose() << " pos[n]: " << lc_pnts[lc_pnts.size()-1]->pos.transpose()
                            << "\n dir[0]: " << start_dir.transpose() << " dir[n]: " << end_dir.transpose()
                            << "\n fit size: " << fitted_pnts.size() << std::endl;
                }

                // 拟合左车道线
                if(1)
                {
                    std::vector<Eigen::Vector3d> to_fit_pnts;
                    // to_fit_pnts = {
                    //     lc_pnts[0]->left_lb.pos,
                    //     lc_pnts[lc_pnts_size-1]->left_lb.pos
                    // };

                    for (int k = 0; k < lc_pnts_size; k++) {
                        to_fit_pnts.push_back(lc_pnts[k]->left_lb.pos);
                    }

                    double scale = alg::calc_dis(lc_pnts[0]->left_lb.pos, lc_pnts[lc_pnts_size-1]->left_lb.pos);;// TODO：先根据距离调整一波，后面排查一下原因
                    scale = scale > 50 ? 50 : scale;
                    Eigen::Vector3d start_dir = scale * lc_pnts[0]->left_lb.dir;
                    Eigen::Vector3d end_dir = scale * lc_pnts[lc_pnts_size-1]->left_lb.dir;
                    // 此处临时解决双向路中间车道线朝向问题
                    double start_angle = alg::calc_theta(start_dir, sm_fit_lines.road_dir);
                    double end_angle = alg::calc_theta(end_dir, sm_fit_lines.road_dir);
                    // 此处临时解决双向路中间车道线朝向问题:暂时不处理
                    // TODO: 鄢薪做完双车道模型再放开2
                    if(end_angle > 90 && start_angle > 90) {
                        continue;
                    }
                    // 2 打开这里：
                    // start_dir = start_angle > 90 ? -start_dir: start_dir;
                    // end_dir = end_angle > 90 ? -end_dir: end_dir;

                    std::vector<Eigen::Vector3d> fitted_pnts;
                    sm_fitting(to_fit_pnts, start_dir, end_dir, fitted_pnts);

                    for (int k = 0; k < lc_pnts_size; k++) {
                        // 虚拟线
                        if(alg::is_bit_set(line_match.match_status, 4)) {
                            lc_pnts[k]->left_lb.type = 1;
                            lc_pnts[k]->left_lb.color = 1;
                        }
                        if(k == 0 || k == lc_pnts_size - 1) {
                            lc_pnts[k]->left_lb_debug.pos = lc_pnts[k]->left_lb.pos;
                            lc_pnts[k]->left_lb_debug.dir = lc_pnts[k]->left_lb.dir;
                            continue;
                        }
                        lc_pnts[k]->left_lb.pos = fitted_pnts[k];
                        lc_pnts[k]->left_lb.dir = alg::get_dir(lc_pnts[k]->left_lb.pos, lc_pnts[k-1]->left_lb.pos);

                        lc_pnts[k]->left_lb_debug.pos = fitted_pnts[k];
                        lc_pnts[k]->left_lb_debug.dir = alg::get_dir(lc_pnts[k]->left_lb_debug.pos, lc_pnts[k-1]->left_lb_debug.pos); 
                    }

                    if(open_log) {
                        std::cout << "before pos[0]: " << to_fit_pnts[0].transpose() << " pos[n/2]: " << to_fit_pnts[lc_pnts_size/2].transpose()<< " pos[n]: " << to_fit_pnts[lc_pnts_size-1].transpose()
                                << "\n dir[0]: " << start_dir.transpose() << " dir[n]: " << end_dir.transpose()
                                << "\n fit size: " << fitted_pnts.size() << " scale: " << scale << std::endl;
                        std::cout << "after pos[0]: " << fitted_pnts[0].transpose() << " pos[n/2]: " << fitted_pnts[lc_pnts_size/2].transpose()<< " pos[n]: " << fitted_pnts[lc_pnts_size-1].transpose()
                                << "\n pos[0]: " << lc_pnts[0]->left_lb_debug.pos.transpose() << " pos[n]: " << lc_pnts[lc_pnts_size-1]->left_lb_debug.pos.transpose()
                                << "\n dir[0]: " << lc_pnts[0]->left_lb_debug.dir.transpose() << " dir[n]: " << lc_pnts[lc_pnts_size-1]->left_lb_debug.dir.transpose()
                                << "\n fit size: " << fitted_pnts.size() << " lc_pnts_size:" << lc_pnts_size << " scale: " << scale << std::endl;
                    }
                }

                // 拟合右车道线
                if(1)
                {
                    std::vector<Eigen::Vector3d> to_fit_pnts;
                    // to_fit_pnts = {
                    //     lc_pnts[0]->right_lb.pos,
                    //     lc_pnts[lc_pnts_size-1]->right_lb.pos
                    // };

                    for (int k = 0; k < lc_pnts_size; k++) {
                        to_fit_pnts.push_back(lc_pnts[k]->right_lb.pos);
                    }

                    double scale = alg::calc_dis(lc_pnts[0]->right_lb.pos, lc_pnts[lc_pnts_size-1]->right_lb.pos);// TODO：先根据距离调整一波，后面排查一下原因
                    scale = scale > 50 ? 50 : scale;
                    Eigen::Vector3d start_dir = scale * lc_pnts[0]->right_lb.dir;
                    Eigen::Vector3d end_dir = scale * lc_pnts[lc_pnts_size-1]->right_lb.dir;

                    std::vector<Eigen::Vector3d> fitted_pnts;
                    sm_fitting(to_fit_pnts, start_dir, end_dir, fitted_pnts);

                    for (int k = 0; k < lc_pnts_size; k++) {
                        // 虚拟线
                        if(alg::is_bit_set(line_match.match_status, 2)) {
                            lc_pnts[k]->right_lb.type = 1;
                            lc_pnts[k]->right_lb.color = 1;
                        }

                        if(k == 0 || k == lc_pnts_size - 1) {
                            lc_pnts[k]->right_lb_debug.pos = lc_pnts[k]->right_lb.pos;
                            lc_pnts[k]->right_lb_debug.dir = lc_pnts[k]->right_lb.dir;

                            continue;
                        }
                        
                        lc_pnts[k]->right_lb.pos = fitted_pnts[k];
                        lc_pnts[k]->right_lb.dir = alg::get_dir(lc_pnts[k]->right_lb.pos, lc_pnts[k-1]->right_lb.pos); 

                        lc_pnts[k]->right_lb_debug.pos = fitted_pnts[k];
                        lc_pnts[k]->right_lb_debug.dir = alg::get_dir(lc_pnts[k]->right_lb_debug.pos, lc_pnts[k-1]->right_lb_debug.pos); 
                    }
                    if(open_log) {
                        std::cout << "pos[0]: " << to_fit_pnts[0].transpose() << " pos[n/2]: " << to_fit_pnts[lc_pnts_size/2].transpose()<< " pos[n]: " << to_fit_pnts[lc_pnts_size-1].transpose()
                                << "\n dir[0]: " << start_dir.transpose() << " dir[n]: " << end_dir.transpose()
                                << "\n fit size: " << fitted_pnts.size() << " scale: " << scale << std::endl;
                        std::cout << "after pos[0]: " << fitted_pnts[0].transpose() << " pos[n/2]: " << fitted_pnts[lc_pnts_size/2].transpose()<< " pos[n]: " << fitted_pnts[lc_pnts_size-1].transpose()
                                << "\n pos[0]: " << lc_pnts[0]->right_lb_debug.pos.transpose() << " dir[n]: " << lc_pnts[lc_pnts_size-1]->right_lb_debug.pos.transpose()
                                << "\n dir[0]: " << lc_pnts[0]->right_lb_debug.dir.transpose() << " dir[n]: " << lc_pnts[lc_pnts_size-1]->right_lb_debug.dir.transpose()
                                << "\n fit size: " << fitted_pnts.size() << " lc_pnts_size:" << lc_pnts_size << " scale: " << scale << std::endl;
                    }
                }


                static int g_count_1 = 0;
                if(0) {
                    {
                        // 原始
                        std::string disp_name = "qzc_1l_"+std::to_string(i)+"_"+std::to_string(j);
                        session->set_display_name(disp_name.c_str());
                        auto line_line_left = session->add_debug_log(utils::DisplayInfo::LINE, "line_line_left_{}", g_count_1++);
                        line_line_left->color={255,0, 0};

                        auto line_line_right = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_line_right_{}", g_count_1++);
                        line_line_right->color={0,0,255};


                        auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", g_count_1++);
                        line_center->color={0,255,0};

                        for (int k = 0; k < lc_pnts_size; k++) {
                            auto &center_ele=line_center->add(lc_pnts[k]->pos);
                            auto &ll_l_ele=line_line_left->add(lc_pnts[k]->left_lb.pos);
                            auto &ll_r_ele=line_line_right->add(lc_pnts[k]->right_lb.pos);
                            if(lc_pnts[k]->left_lb.pos.norm() < 1e-6) {
                                LOG_ERROR("!!!!!!!!!!!!!qzc!!!!!!!!!!!!!");
                            }
                        } 
                        session->save_debug_info(disp_name.c_str());
                    }

                    {

                        // 拟合后
                        std::string disp_name = "qzc_2l_"+std::to_string(i)+"_"+std::to_string(j);
                        session->set_display_name(disp_name.c_str());
                        auto line_line_left = session->add_debug_log(utils::DisplayInfo::LINE, "line_line_left_{}", g_count_1++);
                        line_line_left->color={255,0, 0};

                        auto line_line_right = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_line_right_{}", g_count_1++);
                        line_line_right->color={0,0,255};


                        auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", g_count_1++);
                        line_center->color={0,255,0};

                        for (int k = 0; k < lc_pnts_size; k++) {
                            auto &center_ele=line_center->add(lc_pnts[k]->pos);
                            auto &ll_l_ele=line_line_left->add(lc_pnts[k]->left_lb_debug.pos);
                            auto &ll_r_ele=line_line_right->add(lc_pnts[k]->right_lb_debug.pos);
                            if(lc_pnts[k]->left_lb_debug.pos.norm() < 1e-6) {
                                LOG_ERROR("!!!!!!!!!!!!!qzc!!!!!!!!!!!!!");
                            }
                        } 

                        session->save_debug_info(disp_name.c_str());
                    }
                }

            } else {
                LOG_ERROR("lc_pnts_size:{}", lc_pnts_size);
                if(lc_pnts_size > 0) {
                    LOG_ERROR("[change_sm_lane_boundary] 3 reconnected start {} {} -> end {} {} match_status: {}", 
                        lc_pnts[0]->point_status, lc_pnts[lc_pnts_size-1]->point_status,
                        lc_pnts[0]->endpoint_status, lc_pnts[lc_pnts_size-1]->endpoint_status,
                        std::bitset<8>(line_match.match_status).to_string());
                } else {
                    LOG_ERROR("[change_sm_lane_boundary] 4 reconnected empty!!!!!!");
                }
            }

            // 1. 利用起始和终点的位置+方向 进行曲线拟合
            // 2. 替换现有的距离 和 方向
            // 3. 虚实属性
        }
    }
            
    return fsdmap::SUCC;
}


int RoadModelProcBindTrail::hit_road_boundary_candidate(RoadModelSessionData* session)
{
    RTreeProxy<BoundaryFeature*, float, 2> road_boundary_tree; 
    for(auto line: session->merge_boundary_line_list)
    {
     for(auto rb:line->list)
     {
        road_boundary_tree.insert(rb->pos,rb.get());
     }
    }
    auto set_cross_rb=[&road_boundary_tree](LaneCenterFeature*lc,bool is_left){
          std::vector<BoundaryFeature*> bfs;
          Eigen::Vector3d pos=lc->pos;
          road_boundary_tree.search(pos,3.75,bfs);
          Eigen::Vector3d p=is_left?lc->left_lb.pos:lc->right_lb.pos;
          Eigen::Vector3d p1=alg::get_vertical_pos(p,lc->dir,+3.75);
          Eigen::Vector3d p2=alg::get_vertical_pos(p,lc->dir,-3.75);
          BoundaryFeature* min_dis_bf=NULL;
          double min_dis=100;
        //   LOG_INFO("find bfs:{}",bfs.size());
          for(auto bf:bfs)
          {
            if(!bf->next)
            {
                continue;
            }
            if (is_left ^ (alg::judge_left(bf->pos, lc->pos, lc->dir) < 0))
            { // left==1,则筛选出左侧的车道线点
                continue;
            }
            Eigen::Vector3d cross;
            if(alg::findIntersection(p1,p2,bf->pos,bf->next->pos,cross))
            {
                 double dis=alg::calc_dis(cross,p);
                 if(dis<min_dis)
                 {
                    min_dis=dis;
                    min_dis_bf=bf;
                 }
            }
          }
          if(min_dis_bf)
          {
            // LOG_INFO("set min_dis_bf:{}",uint64_t(min_dis_bf));
            auto &rb=is_left?lc->left_rb:lc->right_rb;
             rb=min_dis_bf;
             if(rb->pos.x() <180 &&rb->pos.x() > 170 &&rb->pos.y()>13 && rb->pos.y()<18){
                //  LOG_INFO("is_left:{},{},{},{}", is_left, rb->pos.x(), rb->pos.y(), rb->pos.z());
             }
          }
    };
 //    
 auto set_rb_hit=[set_cross_rb](LaneCenterFeature*lc){
    // left   
    // if(alg::is_bit_set(lc->init_lb_status, 1))
    //   {
    //     set_cross_rb(lc,true);
    //   }
    // //   right
    //   if(alg::is_bit_set(lc->init_lb_status, 2))
    //   {
    //     set_cross_rb(lc,false);
    //   }
    set_cross_rb(lc,true);
    set_cross_rb(lc,false);
 };

 for(auto &line:session->merge_lane_center_list)
 {
     for(auto &lc:line->list)
     {
        set_rb_hit(lc.get());
     }
 }
    return fsdmap::SUCC;
}

int RoadModelProcBindTrail::save_debug_info(RoadModelSessionData* session) {
    int count=0;
    LOG_INFO("start polt bind_lane_center")
    session->set_display_name("bind_lane_center");
    for(auto line:session->merge_lane_center_list)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_center_line");
        log->color = {255, 255, 255};
        for (auto pt : line->list)
        {
            srand48(time(NULL));
            Eigen::Vector3d pos = pt->pos;
            auto &ele = log->add(pos);
            ele.label.intensity = pt->curvature;
            if (pt->hit_link)
            {
                auto link = pt->hit_link;
                srand48(link->same_id);
                ele.label.label = 1E3 * drand48();
                ele.color = {0, 255, 0};
            }
            else
            {
                ele.color = {255, 255, 255};
                srand48(100);
                ele.label.label = 1E3 * drand48();
            }
            // ele.label.score = alg::calc_theta(pt->dir)*57.3;
            ele.label.score = count;
            ele.label.opt_label =pt->hit_poss?1:0;
            // LOG_INFO(" search error: line.id -- {}", line->id.empty());
            ele.label.intensity_opt = !line->id.empty() ? std::stoi(line->id): 0;
        }
        count++;
    }
    for(auto &line:turn_right_range)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_center_line");
        for(auto &pt:line.list)
        {
            auto link=pt->from_link;
            auto &ele=log->add(pt->pos);
            srand48(link->same_id);
            ele.label.label = 1E3*drand48();
            ele.color={0,255,255};
            ele.label.intensity=1000;
            ele.label.score =alg::calc_theta(pt->dir)*57.3;
            ele.label.opt_label =0;
        }
    }


    session->save_debug_info("bind_lane_center");
    LOG_INFO("start polt bind_lane_boundary")
    session->set_display_name("bind_lane_boundary");
    for (auto line : session->merge_lane_line_list)
    {
        auto lane_boundary = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
        lane_boundary->color = {255, 255, 255};
        for (auto pt : line->list)
        {
            lane_boundary->add(pt->pos);
        }
    }

    for(auto line:session->merge_lane_center_list)
    {
        auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
        line_center->color={0,255,0};
        for (auto lc : line->list)
        {

            auto &center_ele=line_center->add(lc->pos);
            if(!alg::is_bit_set(lc->init_lb_status, 1)&&!alg::is_bit_set(lc->init_lb_status, 3))
            {
                center_ele.color={255,0,0};
            }
            if(!alg::is_bit_set(lc->init_lb_status, 2)&&!alg::is_bit_set(lc->init_lb_status, 4))
            {
                center_ele.color={255,20,0};
            }
            center_ele.label.score = lc->curvature;

            auto log_left = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
            auto log_right = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
            auto cut_line = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);

            bool add_left=false;
            bool add_right=false;           
            if (alg::is_bit_set(lc->init_lb_status, 1))//left 原来就有
            {
                add_left=true;
                log_left->color = {127, 255, 212}; // 蓝晶
            }
            else if(alg::is_bit_set(lc->init_lb_status, 3))
            {
                add_left=true;
                log_left->color = {0, 255, 0}; // 绿
            }

            if (alg::is_bit_set(lc->init_lb_status, 2)) //right //原来就有
            {   add_right=true;
                log_right->color = {224, 255, 255}; // 白色
            }
            else if(alg::is_bit_set(lc->init_lb_status, 4))
            {
                add_right=true;
                log_right->color = {0, 0, 255}; // 蓝
            }

            if(alg::is_bit_set(lc->init_lb_status, 5))
            {
                // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                cut_line->color={255,0,0};
                cut_line->add(lc->pos);
                cut_line->add(lc->left_lb.pos);
            }
            
            if(alg::is_bit_set(lc->init_lb_status, 6))
            {
                // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                cut_line->color={255,0,0};
                cut_line->add(lc->pos);
                cut_line->add(lc->right_lb.pos); 
            }
            
            if(add_left)
            {
                log_left->add(lc->pos);
                log_left->add(lc->left_lb.pos);
            }
            if(add_right)
            {
                log_right->add(lc->pos);
                log_right->add(lc->right_lb.pos);
            }
        }
    }

    session->save_debug_info("bind_lane_boundary");
    
    if(1){
        LOG_INFO("start polt bind_lane_boundary2")
        session->set_display_name("bind_lane_boundary2");
        for (auto line : session->merge_lane_line_list) {
            auto lane_boundary = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
            lane_boundary->color = {255, 255, 255};
            for (auto pt : line->list)
            {
                lane_boundary->add(pt->pos);
            }
        }

        for(auto line:session->merge_lane_center_list) {
            auto line_line_left = session->add_debug_log(utils::DisplayInfo::LINE, "line_line_left_{}", count++);
            line_line_left->color={255,0, 0};

            auto line_line_right = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_line_right_{}", count++);
            line_line_right->color={0,0,255};


            auto line_center = session->add_debug_log(utils::DisplayInfo::LINE, "lane_center_{}", count++);
            line_center->color={0,255,0};
            for (auto lc : line->list) {
                if(lc->point_status == 0) {
                    continue;
                }

                auto &center_ele=line_center->add(lc->pos);
                auto &ll_l_ele=line_line_left->add(lc->left_lb_debug.pos);
                auto &ll_r_ele=line_line_right->add(lc->right_lb_debug.pos);
                // if(lc->left_lb_debug.pos.norm() < 1e-6) {
                //     LOG_ERROR("qzc!!!!!!!!!!!!!");
                // }

                // if(!alg::is_bit_set(lc->init_lb_status, 1)&&!alg::is_bit_set(lc->init_lb_status, 3)) {
                //     center_ele.color={255,0,0};
                // }
                // if(!alg::is_bit_set(lc->init_lb_status, 2)&&!alg::is_bit_set(lc->init_lb_status, 4)) {
                //     center_ele.color={255,20,0};
                // }
                // center_ele.label.score = lc->curvature;



                #if 0
                auto log_left = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
                auto log_right = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);
                auto cut_line = session->add_debug_log(utils::DisplayInfo::LINE, "lane_boundary_{}", count++);

                bool add_left=false;
                bool add_right=false;           
                if (alg::is_bit_set(lc->init_lb_status, 1))//left 原来就有
                {
                    add_left=true;
                    log_left->color = {127, 255, 212}; // 蓝晶
                }
                else if(alg::is_bit_set(lc->init_lb_status, 3)) {
                    add_left=true;
                    log_left->color = {0, 255, 0}; // 绿
                }

                if (alg::is_bit_set(lc->init_lb_status, 2)) //right //原来就有
                {   add_right=true;
                    log_right->color = {224, 255, 255}; // 白色
                }
                else if(alg::is_bit_set(lc->init_lb_status, 4)) {
                    add_right=true;
                    log_right->color = {0, 0, 255}; // 蓝
                }

                if(alg::is_bit_set(lc->init_lb_status, 5)) {
                    // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                    LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                    cut_line->color={255,0,0};
                    cut_line->add(lc->pos);
                    cut_line->add(lc->left_lb_debug.pos);
                }
                
                if(alg::is_bit_set(lc->init_lb_status, 6)) {
                    // LOG_INFO("init_lb_status:{}",lc->init_lb_status);
                    LOG_ERROR("init_lb_status:{}",lc->init_lb_status);
                    cut_line->color={255,0,0};
                    cut_line->add(lc->pos);
                    cut_line->add(lc->right_lb_debug.pos); 
                }
                
                if(add_left) {
                    log_left->add(lc->pos);
                    log_left->add(lc->left_lb_debug.pos);
                }
                if(add_right) {
                    log_right->add(lc->pos);
                    log_right->add(lc->right_lb_debug.pos);
                }
                #endif
            }
        }

        session->save_debug_info("bind_lane_boundary2");
    }

    LOG_INFO("start polt oppo_lane_center")
    session->set_display_name("oppo_lane_center");
    for(auto line:oppo_lane_center)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        for(auto lc:line->list)
        {
          auto &ele=  log->add(lc->pos);
          ele.label.score=alg::calc_theta(lc->dir)*57.3;
        }

    }
    session->save_debug_info("oppo_lane_center");
    LOG_INFO("start polt hit_boundary")
// TODO check what happy 
    session->set_display_name("hit_boundary");
    for(const auto line:session->merge_lane_center_list)
    {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "bind_center_line");
        log->color = {255, 255, 255};
        for (const auto &lc : line->list)
        {
            if(!lc)
            {
                LOG_WARN("lc null")
                continue;
            }
          if(lc->left_rb)
          {
           auto pos=lc->left_rb->pos;
           bool hasInf = pos.array().isInf().any();

           if(pos.hasNaN()||hasInf)
           {
            LOG_WARN("left hash nan info")
            continue;
           }
            // if(pos.x() <180 && pos.x() > 170 &&pos.y()>13 && pos.y()<18){
            //     std::cout << "2left:" << pos.x() << ", " << pos.y() <<", " <<pos.z() <<std::endl;
            // }   
            auto &ele=log->add(pos);
            ele.color={255,0,0}; //红色
          }
          if(lc->right_rb)
          {
            auto pos=lc->right_rb->pos;
            bool hasInf = pos.array().isInf().any();
           if(pos.hasNaN()||hasInf)
            {
                LOG_WARN("right hash nan info")
                continue;
            }
            // if(pos.x() <180 && pos.x() > 170 &&pos.y()>13 && pos.y()<18){
            //     std::cout << "2right:" << pos.x() << ", " << pos.y() <<", " <<pos.z() <<std::endl;
            // }
            auto &ele= log->add(pos);
            ele.color={0,0,255}; //蓝色
          }
        }
    }
    session->save_debug_info("hit_boundary");
    LOG_INFO("start polt right_boundary")
    session->set_display_name("right_boundary");
    auto log = session->add_debug_log(utils::DisplayInfo::POINT, "rb");
    log->color={255,255,255};
    for(const auto rb:candidate_right_boundary)
    {
        log->add(rb->pos);

    }
    for(auto &ptrc: pruning_turn_right_candidate)
    {
         auto log = session->add_debug_log(utils::DisplayInfo::LINE, "LB_{}",count++);
         log->color={0,255,0};
         for(auto &pts:  ptrc.second)
         {
            for(auto p: pts){
                log->add(p.lb->pos);
            }
         }
    }
    session->save_debug_info("right_boundary");

    return fsdmap::SUCC;
}

}
}
