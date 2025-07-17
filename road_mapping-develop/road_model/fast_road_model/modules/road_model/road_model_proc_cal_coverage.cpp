#include "road_model_proc_cal_coverage.h"
#include "utils/algorithm_util.h"
#include <algorithm> // 包含 std::all_of 和 std::equal
#include <string>
#include <iterator>

DEFINE_bool(cal_coverage_save_data_enable, true, "cal_coverage_save_data_enable");
DEFINE_bool(coverage_detial_img_enable, false, "coverage_detial_img_enable");
DEFINE_double(standard_lane_width2, 3.5, "standard_lane_width2"); 
DEFINE_double(gap_road_bd_to_lane, 0.4, "gap_road_bd_to_lane"); 
DEFINE_double(search_num, 16, "search_num"); // 实际前后各能连 (16-1)*2m, 
DEFINE_double(duplicate_pts_dis, 0.15, "duplicate_pts_dis"); 
DECLARE_double(sample_line_sample_pose_gap);
DEFINE_double(short_connnect_dis, 1.7, "short_connnect_dis");  //理论和keypose采样间距一样 2m
DEFINE_double(check_hit_dis, 0.4, "check_hit_dis"); 
DEFINE_double(merge_feature_expend_range, 4,"merge_feature_expend_range");

DEFINE_double(first_horizon_push_scale, 0.9, "first_horizon_push_scale"); 
DEFINE_double(filter_init_pt_threshold, 0.5, "filter_init_pt_threshold"); //最小车道宽度 + T
DEFINE_double(filter_init_pt_theta, 15, "filter_init_pt_theta"); //车道线和车道中心线点的角度差
DEFINE_double(near_point_exist_dis, 0.7, "near_point_exist_dis"); //横向0.7m有车道中心线点，则前后推不补点
DEFINE_double(sm_point_exist_dis, 2.5, "sm_point_exist_dis"); //2.5m内有分合流点，则前后推不补点
DEFINE_double(mask_dis, 50, "mask_intersection_dis"); //距离路口中心点的范围不处理


namespace fsdmap {
    namespace road_model {
        
fsdmap::process_frame::PROC_STATUS RoadModelProcCalCoverage::proc( RoadModelSessionData* session) {
      
        CHECK_FATAL_PROC(delte_lane_all_in_junction(session),"delte_lane_all_in_junction");

        CHECK_FATAL_PROC(classify_prev_and_next_lc(session),"classify_prev_and_next_lc");

        double dis_th = FLAGS_merge_feature_expend_range*1.414;
        double angle = 30;
        double vertcial_dis = 0.5;
        CHECK_FATAL_PROC(fix_dis_connect(session, dis_th, angle, vertcial_dis),"fix_dis_connect");

        CHECK_FATAL_PROC(fix_dis_connect_lb(session),"fix_dis_connect_lb");

        CHECK_FATAL_PROC(generate_ll_by_lb(session),"genrate_ll_by_lb");
        
        CHECK_FATAL_PROC(fix_dis_connect_ll(session),"fix_dis_connect_ll");

        // 只连接了前面的一段，后面的可能没有连接，这里再连接一次
        CHECK_FATAL_PROC(fix_dis_connect_lb(session),"fix_dis_connect_lb2");

        CHECK_FATAL_PROC(sample_and_smooth_line(session),"sample_and_smooth_line");

        CHECK_FATAL_PROC(find_link_entrance(session), "find_link_entrance");
        
        //=================第一次遍历起始点，生成表=======================
        CHECK_FATAL_PROC(stage_1_recovery_single_road_cross_points(session), "stage_1_recovery_single_road_cross_points");
        save_debug_info(session, -1);
        
        CHECK_FATAL_PROC(verify_elements(session), "verify_elements");
        save_debug_info(session, 0);
        
        dis_th = FLAGS_merge_feature_expend_range*1.414;
        angle = 30;
        vertcial_dis = 0.5;
        CHECK_FATAL_PROC(fix_dis_connect(session, dis_th, angle, vertcial_dis),"fix_dis_connect2");
        save_debug_info(session, 1);
        save_debug_info(session, 2);//打印数字

        //=================  进行第二次遍历，处理存在车道缺失的keypose================
        CHECK_FATAL_PROC(stage_2_connect_pre_next(session), "stage_2_connect_pre_next");
        CHECK_FATAL_PROC(save_debug_info(session, 3), "prev_next_connect_line_before");
        CHECK_FATAL_PROC(save_debug_info(session, 4), "save_debug_info_with_N_log");//打印数字

        CHECK_FATAL_PROC(prev_next_connect_line(session),"prev_next_connect_line");
        CHECK_FATAL_PROC(save_debug_info(session, 5), "prev_next_connect_line_finish");
        CHECK_FATAL_PROC(rm_short_center_lines(session), "rm_short_center_lines");

        //=================  前后推后，重新刷一遍stage_1的表，计算分合流================
        CHECK_FATAL_PROC(find_link_entrance(session), "find_link_entrance_2");
        CHECK_FATAL_PROC(stage_1_recovery_single_road_cross_points(session), "stage_1_recovery_single_road_cross_points_2");
 
        // 通过时序修正一些异常的跳变点： is_full_lc
        CHECK_FATAL_PROC(modify_is_full_lc_by_neighbor(session), "modify_is_full_lc_by_neighbor");
        CHECK_FATAL_PROC(modify_N_road_by_neighbor(session), "modify_N_road_by_neighbor");
        save_debug_info(session, 6);//打印数字

        CHECK_FATAL_PROC(gen_lane_directin(session),"gen_lane_directin");

        CHECK_FATAL_PROC(save_debug_info_all(session, 0), "save_debug_info_all");

        return fsdmap::process_frame::PROC_STATUS_SUCC;
}


int RoadModelProcCalCoverage::sample_lane_center(RoadModelSessionData *session)
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

int RoadModelProcCalCoverage::sample_lane_boundary(RoadModelSessionData *session)
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

    update_lane_line_sample_tree(session); //kdtree

    // LOG_INFO("after size: {}",  session->merge_lane_line_list.size());

    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::sample_road_boundary(RoadModelSessionData *session)
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


int RoadModelProcCalCoverage::sample_and_smooth_line(RoadModelSessionData* session) {

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


int RoadModelProcCalCoverage:: delte_lane_all_in_junction(RoadModelSessionData *session)
{
    auto is_all_in_junction=[](LaneCenterGroupLine*line,const  std::vector<Eigen::Vector3d>&junction)
    {
        if(!line) {
            return true;
        }
        if(line->boundary_type == LaneType::RIGHT_TURN_LC) {
            LOG_WARN("RIGHT_TURN_LC");
            return false;
        }

        int n=line->list.size();
        double score=0;
        for(auto p:line->list) {
            if(alg::is_in_convex_polygon(junction,p->pos)) {
                score++;
            }
        }
        score= score/static_cast<double>(n);
        // LOG_INFO("score:{}",score)
        return score>0.95;
    };
    std::vector<std::vector<Eigen::Vector3d> >junctions;
    for (auto &obj : session->raw_intersections) {
        std::vector<Eigen::Vector3d> one_junction;
        auto center = obj->pos;
        for (auto &pt : obj->point_info) {
            one_junction.push_back(pt->pos);
        }
        junctions.push_back(one_junction);
    }

    // 1 删除路口框内的线
    for(auto iter=session->merge_lane_center_list.begin();iter!=session->merge_lane_center_list.end();) {
        bool erase=false;
        for(auto &junction:junctions) {
            if(is_all_in_junction((*iter),junction)) {
                erase=true;
               break;
            }
        }
        if(erase) {
            iter= session->merge_lane_center_list.erase(iter);
        } else {
            iter++;
        }
    }

    return fsdmap::SUCC;
}

void RoadModelProcCalCoverage::find_no_prev_and_next_lc(RoadModelSessionData *session, 
                            RTreeProxy<LaneCenterFeature *, float, 2>& no_prev_lc_tree,
                                RTreeProxy<LaneCenterFeature *, float, 2>& no_next_lc_tree)
{
    for (auto line : session->merge_lane_center_list) {
        for (int i = 0; i < line->list.size(); i++) {
            auto& pt = line->list[i];
            if (!pt->prev) {
                no_prev_lc_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next) {
                no_next_lc_tree.insert(pt->pos, pt.get());
            }
        }
    }
}

void RoadModelProcCalCoverage::find_no_prev_and_next_lc_v2(RoadModelSessionData *session, 
                                RTreeProxy<LaneCenterFeature *, float, 2>& no_prev_lc_tree,
                                 RTreeProxy<LaneCenterFeature *, float, 2>& no_next_lc_tree,
                                 std::vector<LaneCenterFeature *>& no_prev_buff,
                                 std::vector<LaneCenterFeature *>& no_next_buff)
{
    for (auto line : session->merge_lane_center_list) {
        for (int i = 0; i < line->list.size(); i++) {
            auto& pt = line->list[i];
            if (!pt->prev) {
                no_prev_lc_tree.insert(pt->pos, pt.get());
                no_prev_buff.push_back(pt.get());
            }
            if (!pt->next) {
                no_next_lc_tree.insert(pt->pos, pt.get());
                no_next_buff.push_back(pt.get());
            }
        }
    }
}


int RoadModelProcCalCoverage::classify_prev_and_next_lc(RoadModelSessionData*session)
{
    auto find_sm_points = [&](RTreeProxy<LaneCenterFeature *, float, 2>& lc_tree_in,
                            std::vector<LaneCenterFeature *>& buf,
                            bool is_forward) {
        for(auto next : buf) {
            // TODO:qzc 剪枝
            std::vector<LaneCenterFeature*> lcs;
            lc_tree_in.search(next->pos, 10, lcs); // 10m
            std::vector<LaneCenterFeature *> sm_candidate;
            double max_distance = -1;
            for(auto lc : lcs) {
                if(lc->group_line == next->group_line) {
                    continue;
                }
                double d = alg::calc_vertical_dis(lc->pos,next->pos,next->dir);
                if(d > 1.75) {
                    continue;
                }
                double theta = alg::calc_theta1(lc->dir, next->dir, true);
                if(theta > 30) {
                    continue;
                }

                double hori_dist = alg::calc_hori_dis(lc->pos,next->pos,next->dir);
                if (hori_dist > max_distance) {
                    max_distance = hori_dist;
                }

                sm_candidate.push_back(lc);
            }

            // 判断车道线的端点是否落在 <搜索点，最远后继或前驱>之间，是的话，说明该线段存在断裂
            int valid_cnt = 0;
            for(int i = 0; i < sm_candidate.size(); i++) {
                auto lc = sm_candidate[i];
                auto front_or_back_lc = is_forward ? lc->group_line->list.back() : lc->group_line->list.front();
                double front_or_back_hori_dist = alg::calc_hori_dis(front_or_back_lc->pos,next->pos,next->dir);
                if (front_or_back_hori_dist > max_distance) {
                    valid_cnt++;
                    // classified_lc_points_for_visual_debug.push_back(lc);
                    // if(next->pos.x()>15.3 && next->pos.x()<15.5 && next->pos.y()>440.9 && next->pos.y()<441.2) {
                    //     LOG_ERROR("find one sm points: {}, {}, is_forward: {}, xy lc:{}, {}, xy back:{}, {}, valid_cnt:{}, \
                    //     front_or_back_hori_dist:{}, max_distance:{}",
                    //                 next->pos.x(), next->pos.y(), is_forward, 
                    //                 lc->pos.x(), lc->pos.y(),
                    //                 front_or_back_lc->pos.x(), front_or_back_lc->pos.y(),valid_cnt, front_or_back_hori_dist, max_distance);
                    // }
                }
            }

            if(valid_cnt >= 2) {
                LOG_ERROR("find one sm points: {}, {}, valid_cnt:{}", next->pos.x(), next->pos.y(), valid_cnt);
                // lc_tree_out.insert(next->pos, next);
                session->classified_lc_points.push_back(next);
            }
        }
    };

    // 1.  找没前驱后继的lc
    RTreeProxy<LaneCenterFeature *, float, 2> no_prev_lc_tree;
    RTreeProxy<LaneCenterFeature *, float, 2> no_next_lc_tree;
    std::vector<LaneCenterFeature *> no_prev_buff;
    std::vector<LaneCenterFeature *> no_next_buff;
    find_no_prev_and_next_lc_v2(session, no_prev_lc_tree, no_next_lc_tree, no_prev_buff, no_next_buff);

    // 2. 对前驱后继点进行分类
    // 2.1 根据无prev找最近的无next的点
    // 2.2 判断方向是否一致，在固定距离内（5米） 或 找同方向的最近两个点
    // 2.3 如果横向距离小于半个车道
    // 2.4 分合流点位合并（同向共线的伪分合流要去除）
    find_sm_points(no_prev_lc_tree, no_next_buff, true);
    find_sm_points(no_next_lc_tree, no_prev_buff, false);

    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

void extend_lane_center(RTreeProxy<LaneCenterFeature *, float, 2> &tree,
                        bool is_forward,
                        std::deque<LaneCenterFeature *> &buff,
                        LaneCenterFeature *lc,
                        std::set<LaneCenterFeature *> &hash_tabel)
{
    //TODO  简化函数实现
     if(!lc)
     {
        return ;
     }
     if(hash_tabel.count(lc))
     {
        return ;
     }
     hash_tabel.insert(lc);
     if(is_forward)
     {
        buff.push_back(lc);
        if(lc->next)
        {
            extend_lane_center(tree,is_forward,buff,lc->next,hash_tabel);
        }else
        {
            std::vector<LaneCenterFeature*> lcs;
            tree.search(lc->pos,FLAGS_merge_feature_expend_range*1.414,lcs); // 4 * 1.414
            double min_dis=DBL_MAX;
            LaneCenterFeature *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis>FLAGS_merge_feature_expend_range*1.414||h_dis<0) // 4 * 1.414, 在前方
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&& dis<8) // 此从后继里面找了一个距离最小的 且在 8米内
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }

            if(final_next)
            {
                // LOG_INFO("extend_lane_center");
                extend_lane_center(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
        
     }else
     {
        buff.push_front(lc);
        if(lc->prev)
        {
            extend_lane_center(tree,is_forward,buff,lc->prev,hash_tabel);
        }else
        {
            std::vector<LaneCenterFeature*> lcs;
            tree.search(lc->pos,10,lcs);
            double min_dis=DBL_MAX;
            LaneCenterFeature *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis<-FLAGS_merge_feature_expend_range||h_dis>0)
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&&dis<8)
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }
            if(final_next)
            {
                // LOG_INFO("extend_lane_center");
                extend_lane_center(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
       
    }
}

int RoadModelProcCalCoverage::fix_dis_connect(RoadModelSessionData*session, double dis_th, double angle_th, double vertical_dis_th)
{
    // RTreeProxy<LaneCenterFeature*, float, 2> lane_center_tree; 
    RTreeProxy<LaneCenterFeature *, float, 2> prev_lane_center_tree;
    std::vector<LaneCenterFeature *> next_buff;
    for (auto line : session->merge_lane_center_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
            auto pt = line->list[i];
            // lane_center_tree.insert(pt->pos,pt.get());
            if (!pt->prev)
            {
                prev_lane_center_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                next_buff.push_back(pt.get());
            }
        }
    }
    std::set<LaneCenterGroupLine*> clear_lane;
    for(auto next:next_buff)
    {
        std::vector<LaneCenterFeature*> lcs;
        prev_lane_center_tree.search(next->pos, dis_th, lcs); // 4m * 1.414
        for(auto lc:lcs)
        {
            if(lc->group_line==next->group_line)
            {
                continue;
            }
            if(alg::calc_vertical_dis(lc->pos,next->pos,next->dir)>vertical_dis_th)
            {
              continue;
            }
            if(alg::calc_theta1(lc->dir,next->dir,true)>angle_th)
            {
              continue;
            }
            int count=0;
            LaneCenterFeature*cur=lc;
            double length=0;
            while(cur)
            {
                count++;
                if(length>6)
                {
                    break;
                }
                if(alg::calc_vertical_dis(cur->pos,next->pos,next->dir)>vertical_dis_th)
                {
                    break;
                }
                double h_dis=alg::calc_hori_dis(cur->pos,next->pos,next->dir,true);
                if(h_dis>1) // 过滤太近的点，连起来后有大折角
                {
                //   LOG_INFO("expend break:{}",count)
                  break;
                }
                if(cur->next)
                {
                    length+=alg::calc_dis(cur->pos,cur->next->pos);
                }
                // LOG_INFO("expend:[{}  {}  {}]",count,length,h_dis)
                cur=cur->next;
            }
            LaneCenterFeature*prev=cur;
            while(prev && length<FLAGS_merge_feature_expend_range && cur!=lc) // 0-4m如果有跳点、近点则删除，  0-4m内无， 则不会删除   
            {
               prev->cross_mask=true; // 过滤太近的点，连起来后有大折角， 这些点后面是要删除的
               prev=prev->prev;
            }
        }
    }
    for (auto &line : session->merge_lane_center_list)
    {
        const int size = line->list.size();
        std::vector<std::shared_ptr<LaneCenterFeature>> temp;
        std::shared_ptr<LaneCenterFeature> prev=NULL;
        for(auto &p:line->list)
        {
          if(p->cross_mask && line->del_previous_pts)
          {
            prev=NULL;
            p=NULL;
            continue;
          }
          temp.push_back(p);
          //fix   
          if(temp.size()==1)
          {
            // 删除 cross_mask 的点（1m）
            temp.front()->prev=NULL;
          }
          if(prev)
          {
            p->set_prev(prev.get());
          }
          prev=p;
        }
        std::swap(line->list,temp);
    }
    
    RTreeProxy<LaneCenterFeature *, float, 2> fix_prev_lane_center_tree;
    RTreeProxy<LaneCenterFeature *, float, 2> fix_next_lane_center_tree;
    std::vector<LaneCenterFeature *> prev_seed;
    //  for info addr 
    std::map<LaneCenterFeature *,std::shared_ptr<LaneCenterFeature>> addr_map;
    for (auto line : session->merge_lane_center_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
          
            auto pt = line->list[i];
            addr_map[pt.get()]=pt;
            if (!pt->prev)
            {
                prev_seed.push_back(pt.get());
                fix_prev_lane_center_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                fix_next_lane_center_tree.insert(pt->pos, pt.get());
            }
        }
    }
    //
    //刷一遍cross_point的src_line
    std::map<LaneCenterGroupLine*, LaneCenterGroupLine*> new_group_line_m; //groupline更新后的映射表，用来更新crosspoint的src_line
    auto update_cross_point_src_line = [&new_group_line_m](std::unordered_map<KeyPose*, HorizonCrossLine*>& hor_corss_line_m_)
    {
        for(auto &info : hor_corss_line_m_){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                for(auto& lc : road_cross_pt->lane_center_pts)
                {
                    if(new_group_line_m.count(lc->src_line)){
                        lc->src_line = new_group_line_m[lc->src_line];
                    }
                }
            }
        }
    };

    auto make_new_lane_center=[&addr_map, &new_group_line_m]( std::deque<LaneCenterFeature *> buff,RoadModelSessionData* session)->LaneCenterGroupLine *
    {
        std::shared_ptr<LaneCenterGroupLine> new_lane_center=std::make_shared<LaneCenterGroupLine>();
        static int g_lc_id = 0;
        new_lane_center->cur_line_id = g_lc_id;
        new_lane_center->id = std::to_string(g_lc_id++);
        session->lane_center_group_line_ptr.push_back(new_lane_center);
        LaneCenterFeature*prev=NULL;
        for(auto p:buff)
        {
            if(prev)
            {
                p->set_prev(prev);
            }
            prev=p;

            if(!new_group_line_m.count(p->group_line)){
                new_group_line_m.insert({p->group_line, new_lane_center.get()});
            }

            p->group_line=new_lane_center.get();
            if(addr_map.count(p))
            {
                new_lane_center->list.push_back(addr_map[p]);
            }else
            {
                LOG_WARN("log addr map info");
            }
        }
        return new_lane_center.get(); 
    };
    std::set<LaneCenterFeature*> hash_tabel;
    std::vector<LaneCenterGroupLine*> temp_merge_center_list;
    for(auto seed:prev_seed)
    {
        if(hash_tabel.count(seed))
        {
            continue;
        }
        std::deque<LaneCenterFeature *> buff;
        if(seed->next) // 1. 正向一条路走到黑
        {
            extend_lane_center(fix_prev_lane_center_tree,true,buff,seed->next,hash_tabel);
        }
        // 2. 逆向一条路走到黑
        extend_lane_center(fix_next_lane_center_tree,false,buff,seed,hash_tabel);
        if(buff.size()>=2)
        {
            // TODO：qzc此处只有连线，距离过长也没有采样没有补点
            auto nlc=make_new_lane_center(buff,session); // 3. 走过的黑路，构造为一条完整的线
            temp_merge_center_list.push_back(nlc);
        }
    }

    LOG_INFO("origin line: {}, after connect, cur line {}", session->merge_lane_center_list.size(), temp_merge_center_list.size());
    std::swap(temp_merge_center_list,session->merge_lane_center_list);
    
    // 2刷一遍值
    update_lane_center_sample_tree(session); //kdtree
    update_cross_point_src_line(session->hor_corss_line_m); // cross_point的src_line

    return fsdmap::SUCC;
}


void extend_lb(RTreeProxy<BoundaryFeature *, float, 2> &tree,
                        bool is_forward,
                        std::deque<BoundaryFeature *> &buff,
                        BoundaryFeature *lc,
                        std::set<BoundaryFeature *> &hash_tabel)
{
    //TODO  简化函数实现
     if(!lc)
     {
        return ;
     }
     if(hash_tabel.count(lc))
     {
        return ;
     }
     hash_tabel.insert(lc);
     if(is_forward)
     {
        buff.push_back(lc);
        if(lc->next)
        {
            extend_lb(tree,is_forward,buff,lc->next,hash_tabel);
        }else
        {
            std::vector<BoundaryFeature*> lcs;
            tree.search(lc->pos,FLAGS_merge_feature_expend_range*1.414,lcs); // 4 * 1.414
            double min_dis=DBL_MAX;
            BoundaryFeature *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis>FLAGS_merge_feature_expend_range*1.414||h_dis<0) // 4 * 1.414, 在前方
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&& dis<8) // 此从后继里面找了一个距离最小的 且在 8米内
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }

            if(final_next)
            {
                // LOG_INFO("extend_lb");
                extend_lb(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
        
     }else
     {
        buff.push_front(lc);
        if(lc->prev)
        {
            extend_lb(tree,is_forward,buff,lc->prev,hash_tabel);
        }else
        {
            std::vector<BoundaryFeature*> lcs;
            tree.search(lc->pos,10,lcs);
            double min_dis=DBL_MAX;
            BoundaryFeature *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis<-FLAGS_merge_feature_expend_range||h_dis>0)
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&&dis<8)
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }
            if(final_next)
            {
                // LOG_INFO("extend_lb");
                extend_lb(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
       
    }
}

void RoadModelProcCalCoverage::create_new_lane_line(RoadModelSessionData*session)
{
    // auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
    // LaneLineSample* prev = NULL;
    // int line_index = 0;
    // for (int64_t j = 0; j < group_line_src->list.size(); ++j) {
    //     auto &pt = group_line_src->list[j];
    //     if(pt == nullptr) {
    //         continue;
    //     }
    //     if(pt->invalid() || (pt->pos-anchor_pos).norm()>50) {
    //         continue;
    //     }
    //     auto new_node = session->add_ptr(session->lane_line_sample_ptr);
        
    //     // auto feature = std::make_shared<LaneFeature>();
    //     auto feature = session->add_ptr(session->raw_client_lane_feature);
    //     new_node->src = feature.get();
    //     // new_node->src->src = feature.get();
    //     new_node->src->line_id = std::to_string(session->lane_boundary_id);
    //     line_index++;
    //     new_node->src->line_index = line_index;
    //     new_node->src->dir = pt->dir;
    //     new_node->src->pos = pt->pos + right_optimizedTs[i][i2];
    //     new_node->src->raw_pos = pt->pos + right_optimizedTs[i][i2];
    //     new_node->src->key_pose = pt->key_pose;
    //     new_node->src->trail_id = std::to_string(session->lane_boundary_id);

    //     new_node->src->score = 1e-6; // 注意：在mergefeature中必须要
    //     new_node->src->type = 2; // get_std_attr("mmt_to_std", "lane_boundary", "type", 2);
    //     new_node->src->color = 1; // get_std_attr("mmt_to_std", "lane_boundary", "color", 1);
    //     new_node->src->boundary_type = LaneType::RIGHT_TURN_LB_RIGHT; // 右转：右车道线
        
    //     new_node->src->attr.type = new_node->src->type;
    //     new_node->src->attr.color = new_node->src->color;

    //     // // TODO:qzc change it !
    //     // int data_type = 0;
    //     // if (this->_data_type == "MMT_RC") {
    //     //     data_type = 5;
    //     // } else if (this->_data_type == "BYD_LIDAR_B") {
    //     //     data_type = 9;
    //     // }
    //     // // feature->type = data_type;
    //     // new_node->src->ele_type = this->get_ele_type(version, data_type);

    //     new_node->pos = pt->pos + right_optimizedTs[i][i2];
    //     new_node->dir = pt->dir;
    //     new_node->filter_status = 1;

    //     if (prev != NULL) {
    //         prev->dir = alg::get_dir(new_node->pos, prev->pos);
    //         new_node->dir = prev->dir;
    //         new_node->src->dir = prev->dir;

    //         new_node->set_prev(prev);
    //     }
    //     prev = new_node.get();

    //     new_line->list.push_back(new_node.get());
    // }

    // // auto shift_cloud = convert_lane_line_to_pcd(new_line);
    // // std::string save_line_file(FLAGS_debug_file_dir + "/qzc_shift_lines_"+std::to_string(i)+".pcd");
    // // pcl::io::savePCDFile<pcl::PointXYZINormal>(save_line_file.c_str(), *shift_cloud);
    // new_line->id = session->lane_boundary_id++;
}

void RoadModelProcCalCoverage::append_lane_line(RoadModelSessionData*session, LaneLineSampleGroupLine* ll_line, BoundaryGroupLine* rb_line,
                                                bool is_left, int prev_index, double prev_dis, int next_index, double next_dis)
{
    auto create_node = [&session](int line_index, std::string line_id, Eigen::Vector3d new_dir, Eigen::Vector3d new_pos, int type, int color, KeyPose* key_pose) {
        auto new_node = session->add_ptr(session->lane_line_sample_ptr);
        
        // auto feature = std::make_shared<LaneFeature>();
        auto feature = session->add_ptr(session->raw_client_lane_feature);
        new_node->src = feature.get();
        // new_node->src->src = feature.get();
        new_node->src->line_id = line_id;
        new_node->src->line_index = line_index;
        new_node->src->dir = new_dir;
        new_node->src->pos = new_pos;
        new_node->src->raw_pos = new_pos;
        new_node->src->key_pose = key_pose;
        new_node->src->trail_id = line_id;

        new_node->src->score = 1e-6; // 注意：在mergefeature中必须要
        // TODO:qzc check
        new_node->src->type = type; // get_std_attr("mmt_to_std", "lane_boundary", "type", 2);
        new_node->src->color = color; // get_std_attr("mmt_to_std", "lane_boundary", "color", 1);
        new_node->src->attr.type = new_node->src->type;
        new_node->src->attr.color = new_node->src->color;
        new_node->attr.type = new_node->src->type;
        new_node->attr.color = new_node->src->color;

        new_node->src->boundary_type = LaneType::NODEF; // 右转：右车道线
        
        new_node->src->attr.type = new_node->src->type;
        new_node->src->attr.color = new_node->src->color;

        new_node->pos = new_pos;
        new_node->dir = new_dir;
        new_node->filter_status = 1;

        return new_node;
    };

    auto set_prev_next = [](LaneLineSampleGroupLine* line) {
        std::shared_ptr<LaneLineSample> prev = nullptr;
        for(int i = 0 ; i <line->list.size(); i++){
            auto& cur_pt = line->list[i];
        

            if (prev == nullptr) {
                prev = cur_pt;
                continue;
            }
            cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
            prev->dir = cur_pt->dir;
            cur_pt->set_prev(prev.get());
            prev = cur_pt;
        }
    };

    int line_index = ll_line->list.size();
    double range = next_index - prev_index;
    if(line_index == 0 || range == 0) {
        return;
    }
    int type = ll_line->list.back()->attr.type;
    int color = ll_line->list.back()->attr.color;
    for (int i = prev_index; i < next_index; i++) { 
        auto& pt = rb_line->list[i];

        double ratio = (i - prev_index) / range;
        double dis_offset =  (1-ratio) * prev_dis + ratio * next_dis;
        
        Eigen::Vector3d dir = alg::get_vertical_dir(pt->dir);
        Eigen::Vector3d new_pos = pt->pos + dis_offset * dir;
        bool is_left2 = alg::judge_left2(new_pos, pt->pos, pt->dir) >= 0;
        if(is_left != is_left2) {
            dir = -dir;
            new_pos = pt->pos + dis_offset * dir;
        }
        std::string line_id = ll_line->list.front()->src->line_id;
        // 暂时取未连线前的点的type和color

        auto node = create_node(line_index++, line_id, pt->dir, new_pos, type, color, pt->key_pose); // TODO： key_pose 不一定有值
        node->group_line = ll_line;
        ll_line->list.push_back(node);
    }

    set_prev_next(ll_line);
}

int RoadModelProcCalCoverage::generate_ll_by_lb(RoadModelSessionData*session)
{
    // 方式1：
    // 1. 查找所有道路边界线，从头到尾查找最近车道线缺失的部分，然后通过前后的距离进行插值
    // RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
    // for (auto line : session->merge_boundary_line_list)
    // {
    //     for (auto rb : line->list)
    //     {
    //         road_boundary_tree.insert(rb->pos, rb.get());
    //     }
    // }

    auto set_prev_next = [](LaneLineSampleGroupLine* line) {
        std::shared_ptr<LaneLineSample> prev = nullptr;
        for(int i = 0 ; i <line->list.size(); i++){
            auto& cur_pt = line->list[i];

            if (prev == nullptr) {
                prev = cur_pt;
                continue;
            }
            cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
            prev->dir = cur_pt->dir;
            cur_pt->set_prev(prev.get());
            prev = cur_pt;
        }
    };

    for (auto line : session->merge_lane_line_list) {
        set_prev_next(line);
    }

    RTreeProxy<LaneLineSample *, float, 2> lane_boundary_tree;
    for (auto line : session->merge_lane_line_list) {
        for (auto pt : line->list) {
            lane_boundary_tree.insert(pt->pos, pt.get());
        }
    }

    std::vector<std::vector<Eigen::Vector3d> >junctions;
    for (auto &obj : session->raw_intersections) {
        std::vector<Eigen::Vector3d> one_junction;
        auto center = obj->pos;
        for (auto &pt : obj->point_info) {
            one_junction.push_back(pt->pos);
        }
        junctions.push_back(one_junction);
    }

    double scope = 0.8; // 1
    double radius = scope+0.01; // 5
    double theta_thres = 20; // 20
    std::vector<LaneLineSample*> secs;

    // bool open_debug = true;
    bool open_debug = false;
    bool open_flag1 = false;
    Eigen::Vector3d target_pos(143.43, -267.81, 0);
    auto print_pos = [&target_pos, &open_debug, &open_flag1](Eigen::Vector3d pos, Eigen::Vector3d search_pos, int id, std::string log_index, std::string other_log) {
        double buf = 0.05;
        if(open_debug && (target_pos.x()-buf<pos.x() && pos.x() < target_pos.x()+buf && target_pos.y()-buf<pos.y() && pos.y() < target_pos.y()+buf || id == 2)) {
            std::cout << pos.transpose()<<  " sarched " << search_pos.transpose() << std::endl;
            LOG_ERROR("open_debug qzc {} : id:{}, xyz:{} {} other:{}",  log_index, id, pos.x(), pos.y(), other_log);
            open_flag1 = true;
        }
    };

    for (auto rb_line : session->merge_boundary_line_list) {
        
        // 1. 查找该道路边界线，匹配的车道线信息
        std::vector<MatchBoundaryInfo> match_rb_infos;
        for (int i = 0; i < rb_line->list.size(); i++) {
            auto &pt = rb_line->list[i];

            print_pos(pt->pos, pt->pos,pt->group_line->cur_line_id, "1", "");
        

            // 1.1 路口内的过滤掉
            bool is_in_junction = false;
            for(auto &junction:junctions) {
                if(alg::is_in_convex_polygon(junction, pt->pos)) {
                    is_in_junction=true;
                     break;
                }
            }
            if(is_in_junction) {
                continue;
            }
            

            // 1.2 找最近的车道线（0.5米内）
            secs.clear();
            lane_boundary_tree.search(pt->pos, radius, secs);
            Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 1);
            Eigen::Vector3d cross_point = {0, 0, 0};
            print_pos(pt->pos, pt->pos, pt->group_line->cur_line_id, "2", std::to_string(secs.size()));
            for (int k = 0; k < secs.size(); k++) {
                auto &src_pt = secs[k];

                double dis = 0;
                if (src_pt->next == NULL) {
                    // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "3", "");
                    continue;
                } else {
                    // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "4.1", "");
                    // print_pos(pt->pos, src_pt->next->pos, pt->group_line->cur_line_id, "4.2", "");
                    if (!alg::get_cross_point_by_point(src_pt->pos, src_pt->next->pos,
                                pt->pos, v_pt, cross_point, false, 2, 3)) {
                        continue;
                    }
                    dis = alg::calc_dis(pt->pos, cross_point);
                }

                // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "5", std::to_string(dis));
                if (dis > scope) { // tar的垂直方向向量 与 src方向向量的交点 与 当前 tar 点 在 0.5m范围内
                    continue;
                }
                double theta = alg::calc_theta(src_pt->dir, pt->dir);
                // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "6", std::to_string(theta));
                if (theta > theta_thres) { // 两条线上的匹配点，对应的朝向 在 20 度范围内
                    continue;
                }
                double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
                // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "7", std::to_string(s_dis));
                if (s_dis > scope) { // tar 到 src 垂足距离 在 0.5m范围内
                    continue;
                }

                // print_pos(pt->pos, src_pt->pos, pt->group_line->cur_line_id, "8", std::to_string(i));

                MatchBoundaryInfo match_rb_info;
                match_rb_info.rb_index = i;
                match_rb_info.rb_pt = pt;
                match_rb_info.ll_pt = src_pt;
                match_rb_info.ll_group_line = src_pt->group_line;
                match_rb_info.dis = dis;
                match_rb_info.is_left = alg::judge_left2(src_pt->pos, pt->pos, pt->dir) >= 0;
                
                match_rb_infos.push_back(match_rb_info);
                // if (open_flag1) {
                //     std::cout << " " << i << std::endl;
                // }
                break;
            }
        }

        open_flag1 = false;
        
        auto get_end=[](LineSample<LaneFeature>*ll,bool is_end,double& length) {
            length=0;
            LineSample<LaneFeature>*next_ll=ll;
            int count=0;
            while(next_ll)
            {
                auto next=is_end?next_ll->next:next_ll->prev;
                if(next)
                {
                    length+=alg::calc_dis(next->pos,next_ll->pos);
                }
                if(!next)
                {
                    break;
                }
                next_ll=next;
            }
            return next_ll;
        };

        // 2. 对每条边界线没有绑定成功的线，进行连线
        int match_size = match_rb_infos.size();
        // LOG_ERROR("qzc 2 id:{} size:{} {} {}", rb_line->cur_line_id, match_rb_infos.size(), match_size, match_rb_infos.size()-1);
        for (int i = 0; i < match_size - 1; i++) { // 最后一个点先不补偿，防止连到其他线上去了
            auto& prev_info = match_rb_infos[i];
            auto& next_info = match_rb_infos[i+1];
            int prev_index = prev_info.rb_index + 1;
            int next_index = next_info.rb_index - 1;
            if(prev_info.ll_group_line == next_info.ll_group_line) {
                continue;
            }
            // LOG_ERROR("qzc 2.0 prev_index : {}, next_index: {}", prev_index, next_index);
            if(prev_index < rb_line->list.size() && next_index > 0 && next_index - prev_index > 3) {
                auto& ll_pt = prev_info.ll_pt;
                auto& rb_pt = rb_line->list[prev_index];

                double length_to_end;
                auto end_ll_pt = get_end(ll_pt,true,length_to_end); // prev line 可能还有后继，超出一点没问题，超出太多可能不适合连线

                double s_dis = alg::calc_vertical_dis(end_ll_pt->pos, rb_pt->pos, rb_pt->dir);
                if(s_dis > scope) { // 0.6m
                    // 如果有后继，那么说明原来的车道线可能分叉了，需要重新创建一条线
                    // create_new_lane_line(session);
                    continue;
                } else { // 如果滿足距离，说明可以直接在这条线后直接补点 
                    if (prev_info.is_left != next_info.is_left) {
                        LOG_WARN("prev and next not on same side");
                        continue;
                    }

                    auto& ll_line = ll_pt->group_line;
                    append_lane_line(session, ll_line, rb_line, prev_info.is_left, prev_index, prev_info.dis, next_index, next_info.dis);   
                }
            }
        }
    }


    // 方式2：
    // RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
    // for (auto line : session->merge_boundary_line_list)
    // {
    //     for (auto rb : line->list)
    //     {
    //         road_boundary_tree.insert(rb->pos, rb.get());
    //     }
    // }

    // // 1 找到分合流区间的 link

    // // 2 找到分合流区间同向的道路边界线

    // // 3 通过道路边界线，找到最近的车道线

    // // 4 补线逻辑


    // 2刷一遍值
    update_lane_line_sample_tree(session); //kdtree

    return fsdmap::SUCC;
}


int RoadModelProcCalCoverage::fix_dis_connect_lb(RoadModelSessionData*session)
{
    RTreeProxy<BoundaryFeature *, float, 2> prev_lb_tree;
    std::vector<BoundaryFeature *> next_buff;
    for (auto line : session->merge_boundary_line_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
            auto pt = line->list[i];
            if (!pt->prev)
            {
                prev_lb_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                next_buff.push_back(pt.get());
            }
        }
    }
    for(auto next:next_buff)
    {
        std::vector<BoundaryFeature*> lcs;
        prev_lb_tree.search(next->pos,FLAGS_merge_feature_expend_range*1.414,lcs); // 4m * 1.414
        for(auto lc:lcs)
        {
            if(lc->group_line==next->group_line)
            {
                continue;
            }
            if(alg::calc_vertical_dis(lc->pos,next->pos,next->dir)>0.5)
            {
              continue;
            }
            if(alg::calc_theta1(lc->dir,next->dir,true)>30)
            {
              continue;
            }
            int count=0;
            BoundaryFeature*cur=lc;
            double length=0;
            while(cur)
            {
                count++;
                if(length>6)
                {
                    break;
                }
                if(alg::calc_vertical_dis(cur->pos,next->pos,next->dir)>0.5)
                {
                    break;
                }
                double h_dis=alg::calc_hori_dis(cur->pos,next->pos,next->dir,true);
                if(h_dis>1) // 过滤太近的点，连起来后有大折角
                {
                //   LOG_INFO("expend break:{}",count)
                  break;
                }
                if(cur->next)
                {
                    length+=alg::calc_dis(cur->pos,cur->next->pos);
                }
                // LOG_INFO("expend:[{}  {}  {}]",count,length,h_dis)
                cur=cur->next;
            }
            BoundaryFeature*prev=cur;
            while(prev && length<FLAGS_merge_feature_expend_range && cur!=lc) // 4m
            {
               prev->cross_mask=true; // 过滤太近的点，连起来后有大折角， 这些点后面是要删除的
               prev=prev->prev;
            }
        }
    }
    for (auto &line : session->merge_boundary_line_list)
    {
        const int size = line->list.size();
        std::vector<std::shared_ptr<BoundaryFeature>> temp;
        std::shared_ptr<BoundaryFeature> prev=NULL;
        for(auto &p:line->list)
        {
          if(p->cross_mask)
          {
            prev=NULL;
            p=NULL;
            continue;
          }
          temp.push_back(p);
          //fix   
          if(temp.size()==1)
          {
            // 删除 cross_mask 的点（1m）
            temp.front()->prev=NULL;
          }
          if(prev)
          {
            p->set_prev(prev.get());
          }
          prev=p;
        }
        std::swap(line->list,temp);
    }
    // 
    RTreeProxy<BoundaryFeature *, float, 2> fix_prev_lb_tree;
    RTreeProxy<BoundaryFeature *, float, 2> fix_next_lb_tree;
    std::vector<BoundaryFeature *> prev_seed;
    //  for info addr 
    std::map<BoundaryFeature *,std::shared_ptr<BoundaryFeature>> addr_map;
    for (auto line : session->merge_boundary_line_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
          
            auto pt = line->list[i];
            addr_map[pt.get()]=pt;
            if (!pt->prev)
            {
                prev_seed.push_back(pt.get());
                fix_prev_lb_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                fix_next_lb_tree.insert(pt->pos, pt.get());
            }
        }
    }

    //
    //刷一遍cross_point的src_line
    std::map<BoundaryGroupLine*, BoundaryGroupLine*> new_group_line_m; //groupline更新后的映射表，用来更新crosspoint的src_line
    auto update_cross_point_src_line = [&new_group_line_m](std::unordered_map<KeyPose*, HorizonCrossLine*>& hor_corss_line_m_)
    {
        for(auto &info : hor_corss_line_m_){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                for(auto& lc : road_cross_pt->road_boundary_pts)
                {
                    if(new_group_line_m.count(lc->src_line)){
                        lc->src_line = new_group_line_m[lc->src_line];
                    }
                }
            }
        }
    };

    auto make_new_lb=[&addr_map, &new_group_line_m]( std::deque<BoundaryFeature *> buff,RoadModelSessionData* session)->BoundaryGroupLine *
    {
        // std::shared_ptr<BoundaryGroupLine> new_lb=std::make_shared<BoundaryGroupLine>();
        // session->lane_boundary_group_line_ptr.push_back(new_lb);
        auto new_lb = session->add_ptr(session->lane_boundary_group_line_ptr);
        static int g_rb_id = 0;
        new_lb->cur_line_id = g_rb_id++;
        BoundaryFeature*prev=NULL;
        for(auto p:buff)
        {
            if(prev)
            {
                p->set_prev(prev);
            }
            prev=p;

            if(!new_group_line_m.count(p->group_line)){
                new_group_line_m.insert({p->group_line, new_lb.get()});
            }

            p->group_line=new_lb.get();
            if(addr_map.count(p))
            {
                new_lb->list.push_back(addr_map[p]);
            }else
            {
                LOG_WARN("log addr map info");
            }
        }
        return new_lb.get(); 
    };
    std::set<BoundaryFeature*> hash_tabel;
    std::vector<BoundaryGroupLine*> temp_merge_lb_list;
    for(auto seed:prev_seed)
    {
        if(hash_tabel.count(seed))
        {
            continue;
        }
        std::deque<BoundaryFeature *> buff;
        if(seed->next) // 1. 正向一条路走到黑
        {
            extend_lb(fix_prev_lb_tree,true,buff,seed->next,hash_tabel);
        }
        // 2. 逆向一条路走到黑
        extend_lb(fix_next_lb_tree,false,buff,seed,hash_tabel);
        if(buff.size()>=2)
        {
            // TODO：qzc此处只有连线，距离过长也没有采样没有补点
            auto nlc=make_new_lb(buff,session); // 3. 走过的黑路，构造为一条完整的线
            temp_merge_lb_list.push_back(nlc);
        }
    }

    LOG_INFO("origin line: {}, after connect, cur line {}", session->merge_boundary_line_list.size(), temp_merge_lb_list.size());
    std::swap(temp_merge_lb_list,session->merge_boundary_line_list);
    
    // 2刷一遍值
    update_lb_sample_tree(session); //kdtree
    update_cross_point_src_line(session->hor_corss_line_m);

    return fsdmap::SUCC;
}



void extend_ll(RTreeProxy<LaneLineSample*, float, 2> &tree,
                        bool is_forward,
                        std::deque<LaneLineSample*> &buff,
                        LaneLineSample*lc,
                        std::set<LaneLineSample*> &hash_tabel)
{
    //TODO  简化函数实现
     if(!lc)
     {
        return ;
     }
     if(hash_tabel.count(lc))
     {
        return ;
     }
     hash_tabel.insert(lc);
     if(is_forward)
     {
        buff.push_back(lc);
        auto ll_next_base = dynamic_cast<LaneLineSample*>(lc->next);
        if(ll_next_base)
        {
            extend_ll(tree,is_forward,buff,ll_next_base,hash_tabel);
        }else
        {
            std::vector<LaneLineSample*> lcs;
            tree.search(lc->pos,FLAGS_merge_feature_expend_range*1.414,lcs); // 4 * 1.414
            double min_dis=DBL_MAX;
            LaneLineSample *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis>FLAGS_merge_feature_expend_range*1.414||h_dis<0) // 4 * 1.414, 在前方
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&& dis<8) // 此从后继里面找了一个距离最小的 且在 8米内
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }

            if(final_next)
            {
                // LOG_INFO("extend_ll");
                extend_ll(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
        
     }else
     {
        buff.push_front(lc);
        auto ll_prev_base = dynamic_cast<LaneLineSample*>(lc->prev);
        if(ll_prev_base)
        {
            extend_ll(tree,is_forward,buff,ll_prev_base,hash_tabel);
        }else
        {
            std::vector<LaneLineSample*> lcs;
            tree.search(lc->pos,10,lcs);
            double min_dis=DBL_MAX;
            LaneLineSample *final_next=NULL;
            for(auto next_lc:lcs)
            {
                if(next_lc->group_line==lc->group_line)
                {
                    continue;
                }
                // if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>1.5)
                if(alg::calc_vertical_dis(next_lc->pos,lc->pos,lc->dir)>0.5)
                {
                    continue;
                }
                double h_dis=alg::calc_hori_dis(next_lc->pos,lc->pos,lc->dir,true);
                if(h_dis<-FLAGS_merge_feature_expend_range||h_dis>0)
                {
                    continue;
                }
                if(alg::calc_theta1(lc->dir,next_lc->dir,true)>30)
                {
                  continue;
                }
                double dis=alg::calc_dis(lc->pos,next_lc->pos);
                if(dis<min_dis&&dis<8)
                {
                    min_dis=dis;
                    final_next=next_lc;
                }
            }
            if(final_next)
            {
                // LOG_INFO("extend_ll");
                extend_ll(tree,is_forward,buff,final_next,hash_tabel);
            }
        }
       
    }
}


int RoadModelProcCalCoverage::fix_dis_connect_ll(RoadModelSessionData*session)
{
    RTreeProxy<LaneLineSample *, float, 2> prev_ll_tree;
    std::vector<LaneLineSample *> next_buff;
    for (auto line : session->merge_lane_line_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
            auto pt = line->list[i];
            if (!pt->prev)
            {
                prev_ll_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                next_buff.push_back(pt.get());
            }
        }
    }
    for(auto next:next_buff)
    {
        std::vector<LaneLineSample*> lcs;
        prev_ll_tree.search(next->pos,FLAGS_merge_feature_expend_range*1.414,lcs); // 4m * 1.414
        for(auto lc:lcs)
        {
            if(lc->group_line==next->group_line)
            {
                continue;
            }
            if(alg::calc_vertical_dis(lc->pos,next->pos,next->dir)>0.5)
            {
              continue;
            }
            if(alg::calc_theta1(lc->dir,next->dir,true)>30)
            {
              continue;
            }
            int count=0;
            // LaneLineSample*cur=lc;
            LineSample<LaneFeature>* cur = lc;

            double length=0;
            while(cur)
            {
                count++;
                if(length>6)
                {
                    break;
                }
                if(alg::calc_vertical_dis(cur->pos,next->pos,next->dir)>0.5)
                {
                    break;
                }
                double h_dis=alg::calc_hori_dis(cur->pos,next->pos,next->dir,true);
                if(h_dis>1) // 过滤太近的点，连起来后有大折角
                {
                //   LOG_INFO("expend break:{}",count)
                  break;
                }
                if(cur->next)
                {
                    length+=alg::calc_dis(cur->pos,cur->next->pos);
                }
                // LOG_INFO("expend:[{}  {}  {}]",count,length,h_dis)
                cur=cur->next;
            }
            // LaneLineSample*prev=cur;
            LineSample<LaneFeature>* prev = cur;
            while(prev && length<FLAGS_merge_feature_expend_range && cur!=lc) // 4m
            {
               prev->cross_mask=true; // 过滤太近的点，连起来后有大折角， 这些点后面是要删除的
               prev=prev->prev;
            }
        }
    }
    for (auto &line : session->merge_lane_line_list)
    {
        const int size = line->list.size();
        std::vector<std::shared_ptr<LaneLineSample>> temp;
        std::shared_ptr<LaneLineSample> prev=NULL;
        for(auto &p:line->list)
        {
          if(p->cross_mask)
          {
            prev=NULL;
            p=NULL;
            continue;
          }
          temp.push_back(p);
          //fix   
          if(temp.size()==1)
          {
            // 删除 cross_mask 的点（1m）
            temp.front()->prev=NULL;
          }
          if(prev)
          {
            p->set_prev(prev.get());
          }
          prev=p;
        }
        std::swap(line->list,temp);
    }
    // 
    RTreeProxy<LaneLineSample *, float, 2> fix_prev_ll_tree;
    RTreeProxy<LaneLineSample *, float, 2> fix_next_ll_tree;
    std::vector<LaneLineSample *> prev_seed;
    //  for info addr 
    std::map<LaneLineSample *,std::shared_ptr<LaneLineSample>> addr_map;
    for (auto line : session->merge_lane_line_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
          
            auto pt = line->list[i];
            addr_map[pt.get()]=pt;
            if (!pt->prev)
            {
                prev_seed.push_back(pt.get());
                fix_prev_ll_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                fix_next_ll_tree.insert(pt->pos, pt.get());
            }
        }
    }

    //
    //刷一遍cross_point的src_line
    std::map<LaneLineSampleGroupLine*, LaneLineSampleGroupLine*> new_group_line_m; //groupline更新后的映射表，用来更新crosspoint的src_line
    auto update_cross_point_src_line = [&new_group_line_m](std::unordered_map<KeyPose*, HorizonCrossLine*>& hor_corss_line_m_)
    {
        for(auto &info : hor_corss_line_m_){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                for(auto& lc : road_cross_pt->lane_pts)
                {
                    if(new_group_line_m.count(lc->src_line)){
                        lc->src_line = new_group_line_m[lc->src_line];
                    }
                }
            }
        }
    };

    auto make_new_ll=[&addr_map, &new_group_line_m](std::deque<LaneLineSample *> buff,RoadModelSessionData* session)->LaneLineSampleGroupLine *
    {
        // std::shared_ptr<LaneLineSampleGroupLine> new_lb=std::make_shared<LaneLineSampleGroupLine>();
        // session->lane_line_group_line_ptr.push_back(new_lb);
        auto new_ll = session->add_ptr(session->lane_line_group_line_ptr);
        static int g_ll_id = 0;
        new_ll->cur_line_id = g_ll_id++;
        LaneLineSample*prev=NULL;
        for(auto p:buff)
        {
            if(prev)
            {
                p->set_prev(prev);
            }
            prev=p;

            if(!new_group_line_m.count(p->group_line)){
                new_group_line_m.insert({p->group_line, new_ll.get()});
            }

            p->group_line=new_ll.get();
            if(addr_map.count(p))
            {
                new_ll->list.push_back(addr_map[p]);
            }else
            {
                LOG_WARN("log addr map info");
            }
        }
        return new_ll.get(); 
    };
    std::set<LaneLineSample*> hash_tabel;
    std::vector<LaneLineSampleGroupLine*> temp_merge_ll_list;
    for(auto seed:prev_seed)
    {
        if(hash_tabel.count(seed))
        {
            continue;
        }
        std::deque<LaneLineSample*> buff;
        auto seed_next_base = dynamic_cast<LaneLineSample*>(seed->next);
        if(seed_next_base) // 1. 正向一条路走到黑
        {
            extend_ll(fix_prev_ll_tree,true,buff,seed_next_base,hash_tabel);
        }
        // 2. 逆向一条路走到黑
        extend_ll(fix_next_ll_tree,false,buff,seed,hash_tabel);
        if(buff.size()>=2)
        {
            // TODO：qzc此处只有连线，距离过长也没有采样没有补点
            auto nlc=make_new_ll(buff,session); // 3. 走过的黑路，构造为一条完整的线
            temp_merge_ll_list.push_back(nlc);
        }
    }

    LOG_INFO("origin lane line: {}, after connect, cur line {}", session->merge_lane_line_list.size(), temp_merge_ll_list.size());
    std::swap(temp_merge_ll_list,session->merge_lane_line_list);
    
    // 2刷一遍值
    update_lane_line_sample_tree(session); //kdtree
    update_cross_point_src_line(session->hor_corss_line_m);

    return fsdmap::SUCC;
}


void RoadModelProcCalCoverage::update_lane_center_sample_tree(RoadModelSessionData* session) {

    session->merge_lane_center_sample_tree.RemoveAll();
    for (auto line : session->merge_lane_center_list) {
        for (auto pt : line->list) {
            session->merge_lane_center_sample_tree.insert(pt->pos, pt.get());
        }
    }
}
void RoadModelProcCalCoverage::update_split_merge_pts_tree(RoadModelSessionData* session) {

    // split_merge_pts_tree.RemoveAll();
    // for(auto& info : session->split_merge_pts)
    // {
    //     for(int i = 0 ; i < info.center_lines.size(); i++)
    //     {
    //         if (info.valid_status[i].is_valid == false) {
    //             continue;
    //         }

    //         auto& line = info.center_lines[i];
    //         for(auto& pt : line){
    //             split_merge_pts_tree.insert(pt->pos, pt.get());
    //         }
    //     }
    // }
}

void RoadModelProcCalCoverage::update_lb_sample_tree(RoadModelSessionData* session) {

    session->boundary_line_sample_tree.RemoveAll();
    for (auto line : session->merge_boundary_line_list) {
        for (auto pt : line->list) {
            session->boundary_line_sample_tree.insert(pt->pos, pt.get());
        }
    }
}

void RoadModelProcCalCoverage::update_lane_line_sample_tree(RoadModelSessionData* session) {

    session->lane_line_sample_tree.RemoveAll();
    for (auto line : session->merge_lane_line_list) {
        for (auto pt : line->list) {
            session->lane_line_sample_tree.insert(pt->pos, pt.get());
        }
    }
}

PolyFit RoadModelProcCalCoverage::init_polyfit(const std::vector<Eigen::Vector3d>& points, LaneCenterFeature* center){
    PolyFit::Problem problem;
    PolyFit::Options option;
    problem.degree = 3;
    problem.points = points;
    if(center){
        problem.center = center->pos;
        problem.dir = center->dir;
        option.use_calc_direction = false;
        option.use_calc_center = false;
    }else{
        option.use_calc_direction = true;
        option.use_calc_center = true;
    }

    PolyFit fit(problem, option);
    fit.solver();
    
    return fit;
}

std::vector<Eigen::Vector3d> RoadModelProcCalCoverage::polyfit_points(const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Vector3d>& points_without_z,  PolyFit& fit)
{
    std::vector<Eigen::Vector3d> fit_points;  // 用于存储结果
    Eigen::Vector3d last_fit_pnt;

    for (int j = 0; j < points_without_z.size(); j++) {
        auto fit_pnt = fit.eval(points_without_z[j]);
        
        if (j == 0) {
            // 第一条线段，直接插值z并添加到fit_points
            Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), points[j].z());
            fit_points.push_back(out_fit_pnt);
            last_fit_pnt = fit_pnt;
        } else {
            auto diff = alg::calc_dis(fit_pnt, last_fit_pnt);
            auto diff_bak = diff;
            Eigen::Vector3d dir = alg::get_dir(fit_pnt, last_fit_pnt, false);
            
            while (diff > 2.1) {  // 当差值较大时进行插值
                auto predict_fit_pnt = last_fit_pnt + dir * 2;
                auto added_fit_pnt = fit.eval(predict_fit_pnt);
                
                // 插值z
                double factor = diff_bak < 1e-6 ? 0 : diff / diff_bak;
                double interp_z = factor * points[j - 1].z() + (1.0 - factor) * points[j].z();
                Eigen::Vector3d out_fit_pnt(added_fit_pnt.x(), added_fit_pnt.y(), interp_z);
                fit_points.push_back(out_fit_pnt);

                last_fit_pnt = added_fit_pnt;
                diff = alg::calc_dis(fit_pnt, last_fit_pnt);
            }

            // 最后插值z
            double factor = diff_bak < 1e-6 ? 0 : diff / diff_bak;
            double interp_z = factor * points[j - 1].z() + (1.0 - factor) * points[j].z();
            Eigen::Vector3d out_fit_pnt(fit_pnt.x(), fit_pnt.y(), interp_z);
            fit_points.push_back(out_fit_pnt);

            last_fit_pnt = fit_pnt;
        }
    }

    return fit_points;

}

void RoadModelProcCalCoverage::merge_delete_line_by_straight(const CandidateSeed<LaneCenterFeature>& cs, RoadModelSessionData* session)
{
    // cs.is_used = true;
    // std::cout <<"5.1" <<std::endl;
    LaneCenterGroupLine* prev_line = cs.ps->group_line;
    LaneCenterGroupLine* next_line = cs.pe->group_line;
    if(prev_line == next_line || prev_line == nullptr || next_line == nullptr){
        return;
    }
    // TODO: 晓峰，如果只存在一个点，应该要考虑
    
    //1、删除prev_line 、next_line多余的点
    auto ps_it = std::find_if(prev_line->list.begin(), prev_line->list.end(),
        [&](const std::shared_ptr<LaneCenterFeature>& pt)
        {return pt.get() == cs.ps;});

    if(ps_it == prev_line->list.end()) {
        LOG_WARN("[straight 0.1]ps_it is null");
        return;
    }
    
    auto pe_it = std::find_if(next_line->list.begin(), next_line->list.end(),
        [&](const std::shared_ptr<LaneCenterFeature>& pt)
        {return pt.get() == cs.pe;});
    if(pe_it == next_line->list.end()) {
        LOG_WARN("[straight 0.2]pe_it is null");
        return;
    }

    prev_line->list.erase(std::next(ps_it), prev_line->list.end());
    // 置空
    ps_it = prev_line->list.end();
    
    next_line->list.erase(next_line->list.begin(), pe_it); //左闭  右开
    LOG_WARN("[straight]delete line id prev {}, next:{}", prev_line->id, next_line->id);
    // 置空
    pe_it = next_line->list.end();
    
    //2、prev 连接points
    // TODO new_node->type根据端点来赋值
    for(auto& pt : cs.points){
        auto new_node = session->add_ptr(session->lane_center_feature_ptr);
        new_node->pos = pt.pos;
        new_node->boundary_type = LaneType::MATURE_VLC; 
        new_node->type = pt.type;
        new_node->type = pt.color;
        new_node->group_line = prev_line;
        new_node->prev = NULL;
        new_node->next = NULL;
        prev_line->list.push_back(new_node);
    }
    LOG_WARN("[straight2]delete line id prev {}, next:{}", prev_line->id, next_line->id);
    
    //设新点的dir和前驱后继
    std::shared_ptr<LaneCenterFeature> prev = nullptr;
    for(int i = 0 ; i <prev_line->list.size(); i++){
        auto& cur_pt = prev_line->list[i];

        if (prev == nullptr) {
            prev = cur_pt;
            continue;
        }
        cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
        prev->dir = cur_pt->dir;
        cur_pt->set_prev(prev.get());
        prev = cur_pt;
    }
    
    //3、连接next
    //更改next_line线和点的属性
    // next_line->id = prev_line->id; 
    for(int i = 0 ; i <next_line->list.size(); i++){
        auto& cur_pt = next_line->list[i];
        cur_pt->group_line = prev_line; // 已经包含cs.pe的group_line
    }
    // cs.pe->group_line = prev_line; 
    prev_line->list.back()->dir = alg::get_dir(next_line->list.front()->pos, prev_line->list.back()->pos);  //设置dir
    next_line->list.front()->set_prev(prev_line->list.back().get()); //设置连接点前驱后继
    prev_line->list.insert(prev_line->list.end(), next_line->list.begin(), next_line->list.end());

    //4、删线
    session->merge_lane_center_list.erase(
        std::remove(session->merge_lane_center_list.begin(), session->merge_lane_center_list.end(), next_line), 
        session->merge_lane_center_list.end());

}


int RoadModelProcCalCoverage::prev_next_connect_line(RoadModelSessionData *session)
{
    //生成车道中心线补点的kdtree
    RTreeProxy<CrossPoint<LaneCenterGroupLine>*, float, 2> lane_center_cross_pt_tree;
    RTreeProxy<RoadObjectInfo*, float, 2> stopline_crosswalk_tree;

    auto cross_big_intersection = [&stopline_crosswalk_tree](const CandidateSeed<LaneCenterFeature>& cs)
    {
         double dis = (alg::calc_dis(cs.ps->pos, cs.pe->pos) / 2);
         if(dis < 30){
            return false;
         }else{
            std::vector<RoadObjectInfo*> objs;
            Eigen::Vector3d mid_pt = (cs.ps->pos + cs.pe->pos) / 2;  //平均点
            stopline_crosswalk_tree.search(mid_pt, dis/2, objs);
            if(objs.size() > 0){
                return true;
            }else{
                return false;
            }
         }
    };

    //构建补点tree
    for(auto &info : session->hor_corss_line_m){
        for(auto& road_cross_pt : info.second->cross_feature_list){
            for(auto& pt : road_cross_pt->lane_center_pts){
                // if(pt->status == PointStatus::MATURE && pt->is_create_line == false){
                if(pt->status == PointStatus::MATURE || pt->status == PointStatus::INIT && pt->is_create_line == false){
                    lane_center_cross_pt_tree.insert(pt->pos, pt.get());
                }
            }
        }
    }

    //构建停止线人行横道tree
    for (auto& obj : session->raw_object_ret_list) {
        if(obj->ele_type == 5 || obj->ele_type == 6){
            stopline_crosswalk_tree.insert(obj->pos, obj.get());
        }
    }

    //1.生成初始的直行车道中心线连接点对
    init_lane_center_connect_candidate(session);

    for(const auto& cs : init_connect_candidates){
        //2.用停止线和人行横道校验，剔除穿过长度30m以上大路口的候选点
        bool is_cross = cross_big_intersection(cs);
        if(is_cross) {
            continue;
        }

        //3.校验初始连接点对,用init和mature点命中
        std::vector<CrossPoint<LaneCenterGroupLine>*> virutal_points;
        Eigen::Vector3d mid_pt = (cs.ps->pos + cs.pe->pos) / 2;  //平均点
        double search_radius = (alg::calc_dis(cs.ps->pos, cs.pe->pos) / 2) * 1.3;  //*1.3防止浮点数搜不到
        lane_center_cross_pt_tree.search(mid_pt, search_radius, virutal_points);    

        int hit_cnt = 0;
        int hope_hit_cnt = alg::calc_dis(cs.ps->pos, cs.pe->pos) / FLAGS_sample_line_sample_pose_gap;  //2m一个小间隔， 这个间隔需有一个点命中
        for(auto& pt : virutal_points){
            Eigen::Vector3d dir = alg::get_dir(cs.pe->pos, cs.ps->pos);
            double dis = alg::calc_vertical_dis(pt->pos, mid_pt, dir); //pt距离ps，pe连线的距离
            if(dis < FLAGS_check_hit_dis){ //FLAGS_check_hit_dis:0.4
                hit_cnt++;
            }
        }
        
        int experience_th = 1; 
        hope_hit_cnt = hope_hit_cnt > 2 ? hope_hit_cnt - 1 : hope_hit_cnt;
        std::cout << "hit_cnt: " << hit_cnt << ", hope_hit_cnt: " << hope_hit_cnt << std::endl;

        if(hit_cnt >= hope_hit_cnt && hit_cnt > 0){
            fb_check_candidates.push_back(cs);
        }

    }

    //4.拟合补点, 并生成最终的候选点
    for(auto& cs : fb_check_candidates){
        std::vector<LaneCenterFeature *> lc_points;
        std::set<LaneCenterFeature *>hash;
        get_range_points(cs.ps,hash,15,lc_points,false);// prev
        std::reverse(lc_points.begin(), lc_points.end());//保证点顺序从前往后

        get_range_points(cs.pe,hash,15,lc_points,true);//next

        std::vector<Eigen::Vector3d> points_without_z;
        std::vector<Eigen::Vector3d> points;
        for(auto pt : lc_points){
            points_without_z.push_back({pt->pos.x(), pt->pos.y(), 0});
            points.push_back({pt->pos.x(), pt->pos.y(), pt->pos.z()});
        }

        PolyFit fit2 = init_polyfit(points_without_z);
        std::vector<Eigen::Vector3d> fit_pts= polyfit_points(points, points_without_z, fit2);

        std::vector<int> ret_type_color;
        int valid_num = alg::vote_type_color(lc_points, ret_type_color);

        //设置candidate状态
        CandidateSeed<LaneCenterFeature> candidate;
        candidate.ps=lc_points.front();
        candidate.pe=lc_points.back();
        candidate.radius=cs.radius;
        if(fit_pts.size() > 2){
            for(int i = 1 ; i < fit_pts.size() - 1 ; i++){
                UsefulPnt useful_pnt(fit_pts[i], Eigen::Vector3d::Zero(), ret_type_color[0], ret_type_color[1]);
                candidate.points.push_back(useful_pnt);//第一个点和最后一个点不存，保持和bind_trail一致
            }
        }
        fb_fit_candidates.push_back(candidate);
    }

    std::cout << "init_connect_candidates.size: " << init_connect_candidates.size() <<std::endl;
    std::cout << "fb_check_candidates.size: " << fb_check_candidates.size() <<std::endl;

    //5. 连接直行车道中心线
    for(auto& cs : fb_fit_candidates){
        merge_delete_line_by_straight(cs, session);
    }
    update_lane_center_sample_tree(session);
    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::rm_short_center_lines(RoadModelSessionData* session)
{
    // 1. 去除不合理的的短中心线
    int line_size = session->merge_lane_center_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = session->merge_lane_center_list[i];
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("no point");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    double short_line_th = 20;
    double vec_lines_th = 2;
    std::unordered_set<int> short_center_lines;
    for(int i = 0; i < line_size; i++) {
        auto& l1 = all_curve[i];
        double l1_len = alg::bg::length(l1);
        if (l1_len > short_line_th) { 
            continue;
        }
        if (short_center_lines.count(i) > 0) {
            continue;
        }
        
        for (int j = 0; j < line_size; j++) {
            if (short_center_lines.count(j) > 0 || i == j) {
                continue;
            }

            auto& l2 =  all_curve[j];
            double l2_len = alg::bg::length(l2);
            if (l2_len < l1_len) {
                continue;
            }

            int l1_size = l1.size();
            double d_start = alg::bg::distance(l1[0], l2);
            double d_end = alg::bg::distance(l1[l1_size-1], l2);
            if (d_start < vec_lines_th && d_end < vec_lines_th) {
                std::cout << "d_start: " << d_start << ", d_end: " << d_end <<std::endl;
                std::cout << "l1_len: " << l1_len << ", l2_len: " << l2_len <<std::endl;
                short_center_lines.insert(i);
                break;
            }
        }
    }

    std::vector<LaneCenterGroupLine*> merge_lane_center_list_temp;
    merge_lane_center_list_temp.clear();
    for(int i = 0; i < line_size; i++) {
        if(short_center_lines.count(i) == 0) {
            merge_lane_center_list_temp.push_back(session->merge_lane_center_list[i]);
        }else{
            del_short_lane_center_list.push_back(session->merge_lane_center_list[i]);
        }
    }
    LOG_INFO("cxf_remove_short_lines origin center line: {}, after {} ", line_size, merge_lane_center_list_temp.size());

    session->merge_lane_center_list.clear();
    session->merge_lane_center_list.swap(merge_lane_center_list_temp);

    update_lane_center_sample_tree(session); //kdtree
    return fsdmap::SUCC;
}




void RoadModelProcCalCoverage:: get_range_points(LaneCenterFeature *lc,std::set<LaneCenterFeature *>&hash, double range,  std::vector<LaneCenterFeature*> &lc_points,bool forward)
{
    if (!lc)
        return;
    if (range <= 0)
        return;
    if(hash.count(lc))
    {
        LOG_WARN("get_range_points have circle lc_points");
        // 存在环
        return;
    }
    hash.insert(lc);  
    lc_points.push_back(lc);

    if (lc->next&&forward)
    {
        double diff = alg::calc_dis(lc->pos, lc->next->pos);
        double next_range = range - diff;
        get_range_points(lc->next,hash, next_range, lc_points,forward);
    }
    if (lc->prev&&!forward)
    {
        double diff = alg::calc_dis(lc->pos, lc->prev->pos);
        double next_range = range - diff;
        get_range_points(lc->prev,hash, next_range, lc_points,forward);
    }
}

int RoadModelProcCalCoverage::init_lane_center_connect_candidate(RoadModelSessionData *session)
{
    RTreeProxy<LaneCenterFeature*, float, 2> lane_center_tree; 
    RTreeProxy<LaneCenterFeature *, float, 2> prev_lane_center_tree;
    RTreeProxy<LaneCenterFeature *, float, 2> next_lane_center_tree;
    std::vector<LaneCenterFeature *> next_buff;
    for (auto line : session->merge_lane_center_list)
    {
        const int size = line->list.size();
        for (int i = 0; i < size; i++)
        {
            auto pt = line->list[i];
            lane_center_tree.insert(pt->pos,pt.get());
            if(pt->cross_mask)
            {
                CandidateSeed<LaneCenterFeature> cs;
                cs.ps=pt.get();
                cs.type="cross_mask";
                init_candidates.push_back(cs);
            }
            if (!pt->prev)
            {
                CandidateSeed<LaneCenterFeature> cs;
                cs.ps=pt.get();
                cs.type="prev";
                init_candidates.push_back(cs);
                prev_lane_center_tree.insert(pt->pos, pt.get());
            }
            if (!pt->next)
            {
                CandidateSeed<LaneCenterFeature> cs;
                cs.ps=pt.get();
                cs.type="next";
                init_candidates.push_back(cs);
                next_buff.push_back(pt.get());
                next_lane_center_tree.insert(pt->pos, pt.get());
            }
        }
    }
    // LOG_INFO("next_buff:{}",next_buff.size())
    for (auto next : next_buff)
    {
        // auto pos = next->pos;
        // if(pos.x() >-67 && pos.x()<-64.5&& pos.y() >-38 && pos.y() <-36.5){
        //         LOG_ERROR("cxf_capture pos: x:{},y:{}", pos.x(), pos.y())
        //         std::cout << "1 next " <<  std::endl;
        // }
        std::vector<LaneCenterFeature *> lc_points;
        std::set<LaneCenterFeature *>hash;
        get_range_points(next,hash,30,lc_points,false); //prev的 , 之前是30 

        if(lc_points.size()<2) //5
        {
            LOG_INFO("point size less :{}",lc_points.size());
            continue;
        }
        // LOG_INFO("points:{}",points.size());
        std::vector<Eigen::Vector3d> points;
        for(auto pt : lc_points){
            points.push_back(pt->pos);
        }
        PolyFit fit = init_polyfit(points, next);
        // PolyFit fit = init_polyfit(points);
 
        std::vector<LaneCenterFeature*> lcs;
        prev_lane_center_tree.search(next->pos,40,lcs);
        for(auto lc:lcs)
        {
            // 删除z字形的情况， 和角度变化太大的情况
            if(alg::calc_theta1(lc->dir,next->dir,true)>60)
            {
              continue;
            }
        
            if(alg::calc_hori_dis(lc->pos,next->pos,next->dir,true)<1) //过滤太近的点，连起来后有大折角
            {
              continue;
            }
        
            if(alg::calc_vertical_dis(lc->pos,next->pos,next->dir)>(3.75/2) * 1.4) //半车道*1.4
            {
                continue;
            }

            // auto lc_pos = lc->pos;
            // if(lc_pos.x() >-90.5&& lc_pos.x()<-88&& lc_pos.y() >-54 && lc_pos.y() <-52){
            //     LOG_ERROR("cxf_capture lc_pos: x:{},y:{}", lc_pos.x(), lc_pos.y())
            //     std::cout << "5 " <<  std::endl;
            // }

             auto fit_ret=fit.eval(lc->pos);
             auto diff=alg::calc_dis(lc->pos,fit_ret);
             if(diff < 5.5) //用整体过滤分合流地方，避免末端翘起来过滤不掉，  过滤右转+直行这种
            //  if(diff<(3.75/2) * 1.4) //用整体过滤分合流地方，避免末端翘起来过滤不掉，  过滤右转+直行这种
             {
                //设置candidate状态
                CandidateSeed<LaneCenterFeature> candidate;
                candidate.ps = next;
                candidate.pe = lc;
                candidate.radius=diff;
                init_connect_candidates.push_back(candidate);
      
             }
        }
    }
    return fsdmap::SUCC;
}


void RoadModelProcCalCoverage::search_stopline_keypose(RoadModelSessionData* session, Eigen::Vector3d center,
                std::unordered_map<KeyPoseLine*, std::unordered_map<Eigen::Vector3d, std::shared_ptr<LinkInterInfo>,Vector3dHash>>& link_inter_info_){

    find_cross_link_points_by_site_center(session, center, in_link_pts);
    
    std::map<KeyPose*, RoadObjectInfo*> stop_line_list_m; //路口内的4条进入停止线
    std::vector<KeyPose*> in_keypose_list;
    std::vector<KeyPose*> out_keypose_list;
    
    for (auto& in_pt : in_link_pts) {

        KeyPose* in_keypose = find_in_start_keypose(session, in_pt, stop_line_list_m, center);
        KeyPose* out_keypose = find_out_start_keypose(session, in_pt, in_link_pts, stop_line_list_m);
        
        if(in_keypose) in_keypose_list.push_back(in_keypose);
        if(out_keypose) out_keypose_list.push_back(out_keypose);
    }

    for(auto& in_keypose: in_keypose_list){
        for(auto& out_keypose : out_keypose_list){
            if (in_keypose->from_link == out_keypose->from_link) {
                std::shared_ptr<LinkInterInfo> info = std::make_shared<LinkInterInfo>();
                info->_use_stopline = true; 
                info->in_keypose = in_keypose;
                info->out_keypose= out_keypose;
                link_inter_info_[in_keypose->from_link][center] = info;
            }
        }
    }
}

void RoadModelProcCalCoverage::search_lc_in_out_keypose(RoadModelSessionData* session, const Eigen::Vector3d& center,
    std::unordered_map<KeyPoseLine*, std::unordered_map<Eigen::Vector3d, std::shared_ptr<LinkInterInfo>,Vector3dHash>>& link_inter_info_)
{
    auto find_lane_center = [&session](KeyPose* cur, bool forward) {
        KeyPose* res = nullptr;
        while (cur) {
            std::vector<LaneCenterFeature *> lcfs;
            session->merge_lane_center_sample_tree.search(cur->pos, 3.5, lcfs); // 1.75太小会遇到，刚好这条中心线缺少，keypose太后的case
            //点数大于0，且平均方向和 keypose方向角度差异 小于90
            if (lcfs.size() > 0) {
                std::vector<Eigen::Vector3d> dirs;
                for (const auto lc : lcfs) {   
                    dirs.push_back(lc->dir);
                }

                Eigen::Vector3d lc_dir = alg::get_direction(dirs);
                double theta = alg::calc_theta(lc_dir, cur->dir, true, true); 
                if (theta < 90) {
                    res = cur;  // 找到符合条件的 cur
                    break;
                }
            }
            cur = forward ? cur->next : cur->prev;  // 移动到前一个节点
        }
        return res;
    };

    // 1. 找的每条link距离center范围内的50m的最近的一个点
    std::vector<KeyPose*> nearest_pos;
    for (auto& link : session->link_sample_list) {
        auto link_it = link_inter_info_.find(link);  
        if (link_it != link_inter_info_.end()) {
            auto center_it = link_it->second.find(center); 
            if (center_it != link_it->second.end()){
                continue;  //该link已用停止线的keypose，不用处理
            }
        }

        KeyPose* res = nullptr;
        double min_dis = DBL_MAX; 
        for(auto& keypose : link->list){
            double dis = alg::calc_dis(keypose->pos, center);
            if(dis < 50 && dis < min_dis){
                res = keypose;
                min_dis = dis;
            }
        }
        if(res){
            nearest_pos.push_back(res);
        }
    }
 
    // 2. 从该点往前往后检查车道中心线，判断是否为进入或退出
    for(auto& cur : nearest_pos){
        KeyPose* in = nullptr;
        KeyPose* out = nullptr;
        in = find_lane_center(cur, false);
        out = find_lane_center(cur, true);

        if(in && out){
            std::shared_ptr<LinkInterInfo> info = std::make_shared<LinkInterInfo>();
            info->_use_stopline = false; 
            info->in_keypose = in;
            info->out_keypose= out;
            link_inter_info_[in->from_link][center] = info;
        }
    }
}



int RoadModelProcCalCoverage::find_link_entrance(RoadModelSessionData* session) {

    in_link_pts.clear(); out_link_pts.clear(); session->link_inter_info.clear(); 
    session->hor_corss_line_m.clear();
    Eigen::Vector3d center = session->data_processer->_center_link_pos; 

    auto mask_intersection_keypose = [&](KeyPoseLine* link, Eigen::Vector3d& center)
    {
        for(auto keypose:link->list) {  
            if(alg::match_any_with_forms(keypose->from_raw_link->forms, {16,25,26,27})) {
                continue;
            }

            if(alg::calc_dis(center, keypose->pos)< FLAGS_mask_dis){//50
                keypose->is_mask = true;
            }
        }
    };

    auto mask_search_keypose = [&](KeyPose* in_keypose, KeyPose* out_keypose)
    {
        //TODO:cxf， 如果终止位置是错误的话？  需要兜底
        while(in_keypose && in_keypose != out_keypose){
            in_keypose->is_mask = true;
            in_keypose = in_keypose->next;
        }
    };

    search_stopline_keypose(session, center, session->link_inter_info);//有停止线，则起始终止点用停止线的
    search_lc_in_out_keypose(session, center, session->link_inter_info);//没有停止线，则用最近的车道中心线的kyepose替代

    for(auto& link: session->link_sample_list) {
        //剔除 左转26 右转25 掉头16，借道左转27 ，辅路21
        KeyPose* start_keypose = nullptr;
        for(auto& keypose : link->list)
        {
            if(!keypose->from_raw_link){
                continue;
            }

            if(alg::match_any_with_forms(keypose->from_raw_link->forms, {16,25,26,27})) {
                continue;
            }else{
                start_keypose = keypose;
                break;
            }
        }

        if(!start_keypose){
            continue;
        }

        bool used_search = false;
        if(session->link_inter_info.find(link) != session->link_inter_info.end()){
            if(session->link_inter_info[link].find(center) != session->link_inter_info[link].end())
            {
                auto info = session->link_inter_info[link][center];
                mask_search_keypose(info->in_keypose, info->out_keypose);
                used_search = true;
            }
        }
        
        if(used_search == false){
            mask_intersection_keypose(link, center); //用经验值50m
        }
        //初始化keypose起始点
        session->hor_corss_line_m[start_keypose] = {};
    }
    LOG_INFO("session->hor_corss_line_m.size:{}",session->hor_corss_line_m.size())


    // // 1. 找到进入退出的起始link->form=16的点
    // // find_cross_link_points(session, in_link_pts, out_link_pts);  //方法1
    // find_cross_link_points_by_score(session, in_link_pts, out_link_pts);  //方法2
    // LOG_INFO("in_link_pts.size:{} out_link_pts.size():{}",in_link_pts.size(),out_link_pts.size())
    
    // 2. 找到真正的起始点
    //   进入点：靠近停止线的keypose
    //   退出点：停止线交到的对向link
    // std::map<KeyPose*, RoadObjectInfo*> stop_line_list_m; //路口内的4条进入停止线
    // for (auto& in_pt : in_link_pts) {
    //     KeyPose* in_start_keypose = find_in_start_keypose(session, in_pt, stop_line_list_m);
    //     KeyPose* out_start_keypose = find_out_start_keypose(session, in_pt, in_link_pts, stop_line_list_m);
    //     if (in_start_keypose) {
    //         session->hor_corss_line_m[in_start_keypose] = {};
    //     }
    //     if (out_start_keypose) {
    //         session->hor_corss_line_m[out_start_keypose] = {};
    //     }
    // }

    return fsdmap::SUCC;
}


int RoadModelProcCalCoverage::stage_1_recovery_single_road_cross_points(RoadModelSessionData* session)
{   
    for(auto& info : session->hor_corss_line_m){
        HorizonCrossLine* hor_cross_line = new HorizonCrossLine();
        info.second = hor_cross_line;
        hor_cross_line->start_keypose = info.first;

        KeyPose* cur_keypose = info.first;  //当前遍历点
        KeyPoseLine* link = info.first->from_link;
        //路口10个keypose使用link的车道数
        std::vector<std::pair<int,int>> link_lanenum_ranges = use_link_lanenum_ranges(session, link,10); 

        // 4. 遍历每个KeyPose对象
        for(int i = 0; i < link->list.size(); i++)
        {
            KeyPose* cur_keypose = link->list[i];
            
            //初始化key_pose_info对象
            HorizonCrossFeature* new_hor_cross_feature = new HorizonCrossFeature();
            new_hor_cross_feature->keypose = cur_keypose;
            cur_keypose->hor_cross_feature = new_hor_cross_feature;
            
            if(cur_keypose->is_mask == true){
                hor_cross_line->cross_feature_list.push_back(new_hor_cross_feature);
                continue;
            }
    
            //初始化过程变量
            Eigen::Vector3d road_dir;  //修正link方向
            double W_road;   //道路宽度
            double W_lane_t;  //真实车道宽度
            
            new_hor_cross_feature->N_link = cur_keypose->from_raw_link->lanenum_sum;
            new_hor_cross_feature->W_lane_t = FLAGS_standard_lane_width2;   //真实车道宽度
            new_hor_cross_feature->W_lane_st = FLAGS_standard_lane_width2;  //标准车道宽度
            
            //5. 计算修正真实的道路方向
            road_dir = cal_road_true_dir(session, cur_keypose);
            new_hor_cross_feature->road_dir = road_dir;
            
            // 6. 通过道路边界计算车道数
            std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>> in_road_boundary_points(2, 
                std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::UNDEF, Eigen::Vector3d::Zero())); 
            int N_road = cal_actual_road_boundary_count(session, cur_keypose, road_dir, in_road_boundary_points);
            
            for(auto& pt : in_road_boundary_points){
                if (pt->pos.norm() < 1e-6) { // 不存在该边界点
                    new_hor_cross_feature->road_boundary_pts.push_back(pt);
                } else {
                    pt->status = PointStatus::RAW;
                    new_hor_cross_feature->road_boundary_pts.push_back(pt);
                }
            }
                
            auto left_bf = new_hor_cross_feature->road_boundary_pts[0];  //左边界
            auto right_bf = new_hor_cross_feature->road_boundary_pts[1];  //右边界
    
            //判断边界是否完整，不完整则continue;
            if (left_bf->status == PointStatus::RAW && right_bf->status == PointStatus::RAW) {
                // 左右边界都有 ，其他情况is_full_bd默认都是 false
                W_road = alg::calc_dis(left_bf->pos, right_bf->pos, true) - FLAGS_gap_road_bd_to_lane;
            } else if (left_bf->status == PointStatus::RAW && right_bf->status == PointStatus::UNDEF) {
                // 只有左边界
                hor_cross_line->cross_feature_list.push_back(new_hor_cross_feature);
                continue;
            } else if (left_bf->status == PointStatus::UNDEF && right_bf->status == PointStatus::RAW) {
                // 只有右边界
                hor_cross_line->cross_feature_list.push_back(new_hor_cross_feature);
                continue;
            } else {
                // 左右边界都没有
                hor_cross_line->cross_feature_list.push_back(new_hor_cross_feature);
                continue;
            }

            //辅路，没有左边界的，则mask不处理
            bool side_road = alg::match_any_with_forms(cur_keypose->from_raw_link->forms, {21});
            if(left_bf->status == PointStatus::UNDEF && side_road){
                cur_keypose->is_mask = true;
            }
            
            //=============只有左右道路边界都存在，才会往下走=============
            //7. 计算实际的车道中心线数， 实际平均宽度
            double search_radius = W_road;
            std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> in_road_center_points;
            int N_lane_center = cal_actual_lane_center_count(session, cur_keypose, road_dir, search_radius,\
                left_bf->pos, right_bf->pos, in_road_center_points);
            for(auto& pt : in_road_center_points){
                new_hor_cross_feature->lane_center_pts.push_back(pt);
            }

            //8.更新真实的车道宽度
            W_lane_t = cal_true_lane_width(in_road_center_points, 0.3);
            new_hor_cross_feature->W_lane_t = W_lane_t;
            double gap = FLAGS_gap_road_bd_to_lane;
            N_road = static_cast<int>(std::round((W_road - gap) / W_lane_t)); //更新N_road
            
            //9. 计算交到的车道线形成的数量   N_lane = in_road_lane_points.size() -1
            std::vector<std::shared_ptr<CrossPoint<LaneLineSampleGroupLine>>> in_road_lane_points;
            int N_lane = cal_actual_lane_count(session, cur_keypose ,road_dir, search_radius,\
                                                left_bf->pos, right_bf->pos, in_road_lane_points);
            for(auto& pt : in_road_lane_points){
                new_hor_cross_feature->lane_pts.push_back(pt);
            }

            //10. 计算通过道路宽度计算的车道数N_road， 和期望的车道数N_hope
            new_hor_cross_feature->N_road = N_road;
            new_hor_cross_feature->N_lane = N_lane;
            new_hor_cross_feature->N_lane_center = N_lane_center;
            new_hor_cross_feature->W_road = W_road;
            
            //11. 车道线存在， 则补一次点, 并修改对象的N状态
            recovery_single_road_cross_point(new_hor_cross_feature, i, link_lanenum_ranges);
    
            //赋值
            hor_cross_line->cross_feature_list.push_back(new_hor_cross_feature);
        }
        
        
    }

    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::stage_2_connect_pre_next(RoadModelSessionData* session)
{
    auto search_lines = [session](HorizonCrossFeature* cur_it, std::vector<alg::linestring_t>& _all_find_lines, double search_radius)
    {
        std::vector<LaneCenterFeature *> lcfs;
        session->merge_lane_center_sample_tree.search(cur_it->keypose->pos, search_radius, lcfs);

        std::unordered_set<LaneCenterGroupLine*> lines;
        for(auto& lcf : lcfs){
            lines.insert(lcf->group_line);
        }

        // std::vector<alg::linestring_t> all_curve;
        for(const auto& center_line : lines){
            //转化为vector
            alg::linestring_t one_line;
            for(auto& pt :center_line->list){
                one_line.push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            }
            _all_find_lines.push_back(one_line);
        }
    };

    auto check_raw_pt_exist = [](std::vector<alg::linestring_t>& _all_find_lines, alg::linestring_t tar_line)
    {
        double min_dist = DBL_MAX;
        for (const auto& line : _all_find_lines) {
            double d = fsdmap::alg::cal_line_min_distance(line, tar_line);
            if(d < min_dist) {
                min_dist = d;
            }
        }
        bool exist = min_dist < FLAGS_near_point_exist_dis; //0.7
        return exist;
    };

    auto check_lane_center_pt_exist = [](const std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& lane_center_pts, const Eigen::Vector3d& cross_pt) ->bool
    {
        double min_dis = DBL_MAX;
        for(auto pt: lane_center_pts){
            min_dis = alg::calc_dis(pt->pos, cross_pt);
            if(min_dis < FLAGS_near_point_exist_dis){
                return true;
            }
        }
        return false;
    };

    auto check_sm_pt_exist = [](Eigen::Vector3d& cross_pt, RTreeProxy<CrossPoint<LaneCenterGroupLine>*, float, 2>& split_merge_pts_tree) ->bool
    {
        std::vector<CrossPoint<LaneCenterGroupLine>*> sm_pts;
        split_merge_pts_tree.search(cross_pt, FLAGS_sm_point_exist_dis, sm_pts);  //2.5m
        if(sm_pts.empty()){
            return false;
        }else{
            return true; //周围存在点
        }
    };

    auto find_ref = [](auto& it, auto boundary, bool is_forward, int serach_num) -> bool 
    {
        int count = 0;
        while (count < serach_num) {
            if (it == boundary) break;  // 达到边界，退出

            if (!(*it)->is_full_bd) {
                is_forward ? it++ : it--; 
                count++;
                continue;
            }

            if ((*it)->is_full_lc) return true;   // 找到了，返回true

            is_forward ? it++ : it--;  
            count++;
        }
        return false;  
    };

    update_split_merge_pts_tree(session); 

    for(auto& info : session->hor_corss_line_m){
        auto feature_list = info.second->cross_feature_list;

        //1. 遍历cur
        // TODO 考虑存在环， 防止死循环
        auto cur_it = feature_list.begin();
        while(cur_it != feature_list.end()){
            //已经满的， 则跳过不处理，处理下一个
            if ((*cur_it)->is_full_lc) {
                cur_it++;
                continue;
            }

            if((*cur_it)->keypose->is_mask == true){
                cur_it++;
                continue;
            }

            std::vector<alg::linestring_t> all_find_lines;
            search_lines(*cur_it, all_find_lines, FLAGS_search_num * 3);
            // ====找两个状态是 full 的锚点，ref1是往前的 ， ref2是往后的 =======
            // 2. 找ref1  只相信前后 10m，  最远是前找5个keypose 
            auto ref1_it = cur_it;
            bool found_ref1 = find_ref(ref1_it, feature_list.begin(), false, FLAGS_search_num);
            //没找到ref1，跳过后续所有步骤，  TODO： ref1找不到，用横向推的方法
            if(!found_ref1){
                cur_it++;
                continue;
            }

            //3. 找ref2
            auto ref2_it = cur_it;
            bool found_ref2 = find_ref(ref2_it, feature_list.end(), true, FLAGS_search_num);
           
            //找不到ref2跳过后面所有步骤
            if(!found_ref2){
                cur_it++;
                continue;
            }

            
            //ref1 和 ref2->N_hope相等才往下走
            if((*ref1_it)->N_hope != (*ref2_it)->N_hope){
                cur_it++;
                continue;
            }

            // auto pos = (*cur_it)->keypose->pos;
            // if(pos.x() >67 && pos.x()<72 && pos.y() >-16 && pos.y() <-11){
            //     LOG_ERROR("cxf_capture pos: x:{},y:{}", pos.x(), pos.y())
            //     std::cout << "start " <<  std::endl;
            // }
            
            // 4.在ref1和ref2之间寻找有缺失的keypose补点
            auto start_search_it = ref1_it;
            start_search_it++;
            int count3 =0;
            while(start_search_it != ref2_it){
                KeyPose* start_searching = (*start_search_it)->keypose;

                //这个中间的keypose 车道数点全的，跳过不需要补点
                if((*start_search_it)->is_full_lc == true){
                    start_search_it++;
                    continue; 
                }

                if((*start_search_it)->keypose->is_mask == true){
                    start_search_it++;
                    continue;
                }
            
                // 5. 连线补点逻辑
                int N_hope = std::min((*ref1_it)->N_hope, (*ref2_it)->N_hope); // 选锚点车道数最小的数量，尽量补全
                if((*start_search_it)->is_full_bd == false){
                    (*start_search_it)->lane_center_pts.clear();
                }

                for(int i=0; i< N_hope; i++)
                {
                    auto& ref1 = (*ref1_it)->lane_center_pts[i];
                    auto& ref2 = (*ref2_it)->lane_center_pts[i];

                    Eigen::Vector3d cross_pt;
                    Eigen::Vector3d pt1 = ref1->pos;
                    Eigen::Vector3d pt2 = ref2->pos;
                    Eigen::Vector3d v_dir = alg::get_vertical_dir((*start_search_it)->road_dir, false);
                    Eigen::Vector3d pos_to = start_searching->pos + v_dir;

                    //TODO: cxf 排查横向lc顺序，  有时候错顺序原因
                    //过滤：两个参考点错位，或角度差别太大，  暂时先注释， 这会影响一些前后推的点推不出来
                    if(ref1->dir.norm() > 1e-6 && ref2->dir.norm() > 1e-6)
                    {
                        if(alg::calc_vertical_dis(ref2->pos, ref1->pos, ref1->dir)>3.75/2){
                            break;
                        }

                        // if(alg::calc_theta1(ref1->dir,ref2->dir,true)>45){
                        //     break;
                        // }
                    }else{
                        break;
                    }

                    //计算交点
                    bool found_cross_pt = alg::get_cross_point_by_point(start_searching->pos, pos_to, pt1, pt2, cross_pt, false, 1);
                    if(found_cross_pt)
                    {
                        //方法1： 理论快很多 ，弃用
                        // bool found = false;
                        // double W_lane_t = (*start_search_it)->W_lane_t;
                        // for(int j = 0; j < (*start_search_it)->N_lane_center; ++j) {
                        //     Eigen::Vector3d pt = (*start_search_it)->lane_center_pts[j]->pos;
                        //     //找到了，即原来该点已有，则不用添加虚拟点
                        //     if(alg::calc_dis(cross_pt, pt) < (W_lane_t * 0.5 )) {
                        //         found = true;
                        //         // std::cout<< "444444444444444444"<< std::endl;
                        //         break;
                        //     }
                        // }

                        //方法2
                        // 如果原来中心线不存在距离较近的点 且不存在较近的虚拟点，则补点添加虚拟点
                        alg::linestring_t one_line;
                        one_line.push_back(alg::point_t(cross_pt.x(), cross_pt.y()));
                        bool sm_pt_exist = check_sm_pt_exist(cross_pt, split_merge_pts_tree);
                        bool raw_pt_exist = check_raw_pt_exist(all_find_lines, one_line);
                        bool lane_center_pt_exist = check_lane_center_pt_exist((*start_search_it)->lane_center_pts, cross_pt);

                        // if(sm_pt_exist == true){
                        //     break; //整体不处理
                        // }

                        if(raw_pt_exist == false && lane_center_pt_exist == false) 
                        {
                            auto new_cross_pt = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::MATURE, cross_pt); 
                            new_cross_pt->dir = ref1->dir;
                            (*start_search_it)->lane_center_pts.push_back(new_cross_pt);

                            if((*start_search_it)->is_full_bd == true) {
                                (*start_search_it)->N_lane_center++;
                                (*start_search_it)->N_diff--;
                            }
                        }
                    }
                } 

                ///情况1: 道路边界线完整，但缺车道中心线点，根据上述流程补完点后，重新从左排到右
                if((*start_search_it)->is_full_bd == true){
                    // TODO:cxf 优化找第几个的方法，直接插入， 不用重新排,  提升速度
                    std::sort((*start_search_it)->lane_center_pts.begin(), (*start_search_it)->lane_center_pts.end(), 
                                [left_bf = (*start_search_it)->road_boundary_pts[0]->pos] (const std::shared_ptr<CrossPoint<LaneCenterGroupLine>> &lhs,
                                                                                                const std::shared_ptr<CrossPoint<LaneCenterGroupLine>> &rhs) -> bool {
                                    return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
                                });
                    
                    //如果满了，则更改状态为full
                    if((*start_search_it)->N_lane_center == (*start_search_it)->N_hope){
                        (*start_search_it)->is_full_lc = true;
                    }
                }else
                {
                    if(1) 
                    {
                        ///情况2:  之前未处理过的，赋值
                        // auto min_ref_it = (*ref1_it)->N_hope <= (*ref2_it)->N_hope ? ref1_it : ref2_it; //用N_hope小的 那个 作为未处理的参数
                        auto min_ref_it = ref1_it;
                        (*start_search_it)->W_lane_st = (*min_ref_it)->W_lane_st;
                        (*start_search_it)->W_lane_t = (*min_ref_it)->W_lane_t;
                        (*start_search_it)->N_hope = (*min_ref_it)->N_hope;
                        (*start_search_it)->N_lane_center = (*start_search_it)->lane_center_pts.size();
                        (*start_search_it)->N_diff = (*start_search_it)->lane_center_pts.size() - (*start_search_it)->lane_pts.size();
                        
                        
                        // TODO: 这里确定不了满不满 
                        // TODO: cxf， 按照中心线补点的方式补道路边界线（也可参考分合流）
                        //算出左右边界点补全
                        if(0){
                            // TODO：cxf，后续如果有 road_dir 问题，可以考虑用：(*start_search_it)->road_dir = (*min_ref_it)->road_dir;
                            Eigen::Vector3d road_dir = (*start_search_it)->road_dir;
                            Eigen::Vector3d v_dir = alg::get_vertical_dir(road_dir);
                            Eigen::Vector3d left_ref = (*min_ref_it)->road_boundary_pts[0]->pos;
                            Eigen::Vector3d right_ref = (*min_ref_it)->road_boundary_pts[1]->pos;
                            // 通过
                            if(left_ref.norm() > 1e-6 && right_ref.norm() > 1e-6 )
                            {
                                Eigen::Vector3d left_bf_cur, right_bf_cur;
                                bool res_left, res_right;
                                res_left = alg::get_cross_point(left_ref, road_dir, (*start_search_it)->keypose->pos, v_dir, left_bf_cur, false);
                                res_right = alg::get_cross_point(right_ref, road_dir, (*start_search_it)->keypose->pos, v_dir, right_bf_cur, false);
                                
                                //TODO:添加判断原始的 左或右有没有， 有的话则不加
                                if(res_left && res_right)
                                {
                                    (*start_search_it)->road_boundary_pts[0] = (std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::INIT, left_bf_cur));
                                    (*start_search_it)->road_boundary_pts[1] = (std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::INIT, right_bf_cur));
                                    if ((*ref1_it)->N_hope == (*ref2_it)->N_hope) {
                                        (*start_search_it)->is_full_bd = true;
                                        // (*start_search_it)->is_full_lc = true;
                                    }
                                }
                            }
                        }
                    }
                }
                count3++;
                start_search_it++;
            }
            cur_it++;
        }
    }
    return fsdmap::SUCC;
}


// 步骤1的函数
//
std::vector<std::pair<int,int>> RoadModelProcCalCoverage::use_link_lanenum_ranges(RoadModelSessionData *session, KeyPoseLine* link, int use_num)
{  
    std::vector<std::pair<int, int>> use_ranges;
    if(session->link_inter_info.find(link) != session->link_inter_info.end()){
        for (auto& inter_info : session->link_inter_info[link]) {
            KeyPose* in_keypose = inter_info.second->in_keypose;
            KeyPose* out_keypose = inter_info.second->out_keypose;
            int in_index = -1, out_index = -1;
            
            int lise_size = link->list.size();
            for(int i =0; i < lise_size; i++){
                if (link->list[i] == in_keypose) {
                    in_index = i;
                }
                
                if (link->list[i] == out_keypose) {
                    out_index = i;
                }
                
                if (in_index != -1 && out_index != -1) {
                    break;
                }
            }

            //找到有效的点对位
            if (in_index != -1 && out_index != -1) {

                if (in_index > out_index) {
                    LOG_ERROR("use_link_lanenum_ranges error, in_index: {}, out_index: {}",in_index, out_index);
                    std::swap(in_index, out_index);
                }

                int in_index1 = std::max(0, in_index- use_num);
                int out_index2 = std::min(lise_size - 1, out_index + use_num);
                use_ranges.push_back({in_index1, in_index});
                use_ranges.push_back({out_index, out_index2});
            }
        }
    }
    return use_ranges;
}


//找form16的进入 和退出点 
void RoadModelProcCalCoverage::find_cross_link_points(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts, std::vector<KeyPose*>& out_link_pts) {
    std::vector<KeyPose*> cross_links;
    // std::cout << "session->link_sample_list.size() : " << session->link_sample_list.size() << std::endl;
    for(auto& line : session->link_sample_list){
        KeyPose* first_16 = nullptr;
        KeyPose* last_16 = nullptr;
        for(auto& keypose : line->list){
            // if(keypose->from_raw_link->form == "16"){
            if(alg::match_any_with_forms(keypose->from_raw_link->forms, {16})) {
                if(!first_16){
                    first_16 = keypose;
                }
                last_16 = keypose;
            }
        }

        if(first_16){
            in_link_pts.push_back(first_16); 
        }
        if(last_16){
            out_link_pts.push_back(last_16); 
        }
    }

}

//通过score 找到路口一条link上得分最高的点
void RoadModelProcCalCoverage::find_cross_link_points_by_score(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts) {
    for(auto& line : session->link_sample_list){
        KeyPose* max_score_pose = nullptr;
        float max_score = 0;
        bool new_segment  = false;
        for(auto& keypose : line->list){
            if(keypose->junction_score > 0){
                new_segment = true;
                if (keypose->junction_score > max_score) {
                    max_score = keypose->junction_score;
                    max_score_pose = keypose;
                }

            }else{
                if(new_segment && max_score_pose != nullptr){
                    in_link_pts.push_back(max_score_pose);
                }
                new_segment = false;
                max_score = 0 ;
                max_score_pose = nullptr;
            }
        }
    }

}

void RoadModelProcCalCoverage::find_cross_link_points_by_site_center(RoadModelSessionData* session, const Eigen::Vector3d& center,             
                                         std::vector<KeyPose*>& in_link_pts) {
    for(auto& line : session->link_sample_list){
        if(alg::match_any_with_forms(line->list.front()->from_raw_link->forms, {25, 16})) {
            continue;
        }

        KeyPose* closest_poss = nullptr;
        double min_dis = DBL_MAX;
        for(auto& poss : line->list){
            double dis = alg::calc_dis(poss->pos,center);
            if(dis<50 && dis<min_dis)
            {
                min_dis=dis;
                closest_poss=poss;
            }
        }

        if(closest_poss != nullptr){
            in_link_pts.push_back(closest_poss);
        }
    }
}

// 找到靠近停止线的keypose
KeyPose* RoadModelProcCalCoverage::find_in_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, 
    std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m, const Eigen::Vector3d& center) {
    
    KeyPose* src_pt = in_pt;
    KeyPose* in_start_keypose = nullptr;
    bool first_find = false;

    //停止线要限制在50m范围内
    while(in_pt && alg::calc_dis(in_pt->pos,center) <= 50){
        for(auto& object : in_pt->object_list){
            if(object->ele_type == 6 && !in_pt->from_link->list.empty())
            {
                stop_line_list_m[src_pt]=object;
                in_start_keypose = in_pt;
                first_find =true;
                break;
            }else{
                LOG_WARN("link not bind stopline");
            }
        }
        
        if(first_find){
            break;
        }

        in_pt = in_pt->prev;
    }

    return in_start_keypose;
}


// 找退出车道的起始点
KeyPose* RoadModelProcCalCoverage::find_out_start_keypose(RoadModelSessionData* session, KeyPose* in_pt,        
        std::vector<KeyPose*> in_link_pts,std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m) 
{
    KeyPose* out_start_keypose = nullptr;
    std::vector<KeyPoseLine*> rm_self_links; //剔除自身in_pt对应的link

    for(auto& pt : in_link_pts){
        if(alg::match_any_with_forms(pt->from_raw_link->forms, {27})) {
            continue;
        }

        if(pt != in_pt){
            rm_self_links.push_back(pt->from_link);
        }
    }

    auto it = stop_line_list_m.find(in_pt);
    if(it == stop_line_list_m.end()){
        LOG_ERROR("in_pt not found in stop_line_list_m");
        return nullptr;  //异常情况， 这个in_pt 是掉头的link
    }

    RoadObjectInfo* object = it->second;
    std::vector<KeyPose*> stopline_cross_keypose;

    
    for(auto& link : rm_self_links){
        //转为vector
        std::vector<Eigen::Vector3d> link_pts;
        for(auto& keypose : link->list){
            link_pts.push_back(keypose->pos);
        }
        
        //计算单条link的交点
        std::vector<UsefulPnt> cross_points;
        bool is_intersect = alg::get_cross_point_with_curve_segment(object->list[0]->pos, object->list[1]->pos, 
                                                                    link_pts, cross_points, false, 3, 30.0, false);
        if(is_intersect){
            double min_dis = 1000000;
            KeyPose* tmp_keypose = nullptr;
            for(auto& keypose : link->list){
                double dis = alg::calc_dis(cross_points[0].pos, keypose->pos);
                if(dis < min_dis){
                    min_dis = dis;
                    tmp_keypose = keypose;
                }
            }
            stopline_cross_keypose.push_back(tmp_keypose);
        } else {
            // LOG_WARN("No intersection found with the stop line for link");
        }
    }

    //多条link中找最近的交点，即为对向的keypose
    if (!stopline_cross_keypose.empty()) {
        double min_dis2 = 100000;
        for(auto& keypose : stopline_cross_keypose){
            double dis2 = alg::calc_dis(keypose->pos, object->list[0]->pos); 
            if(dis2 < min_dis2){
                min_dis2 = dis2;
                out_start_keypose = keypose;
            }
        }
    } else {
        // LOG_WARN("No cross key poses found for the stop line");
    }

    return out_start_keypose;
}


Eigen::Vector3d RoadModelProcCalCoverage::cal_road_true_dir(RoadModelSessionData* session, KeyPose* kp){
    std::vector<LaneCenterFeature *> lcfs;
    std::vector<Eigen::Vector3d> dirs;
    Eigen::Vector3d road_dir;

    session->merge_lane_center_sample_tree.search(kp->pos, 5, lcfs);
    for (const auto lc : lcfs){   
        dirs.push_back(lc->dir);
    }

    if (!lcfs.empty()){
        road_dir = alg::get_direction(dirs);
        
    }else{
        road_dir = kp->dir;  // 防止报错
    }

    return road_dir;
}

//数学工具
template <class T>
std::shared_ptr<CrossPoint<T>> find_closest_point(const std::vector<std::shared_ptr<CrossPoint<T>>>& points, 
                    const Eigen::Vector3d& center_point) 
{
    auto closest_point = std::make_shared<CrossPoint<T>>(PointStatus::UNDEF, Eigen::Vector3d::Zero());
    double min_dis = std::numeric_limits<double>::max();

    for (const auto& cross_pt : points) {
        double dis = alg::calc_dis(cross_pt->pos, center_point);
        if (dis < min_dis) {
            min_dis = dis;
            closest_point = cross_pt;
        }
    }

    return closest_point;
}

template <class T>
void sort_points_by_distance(std::vector<std::shared_ptr<CrossPoint<T>>>& all_points, const Eigen::Vector3d& left_bf) {
    std::sort(all_points.begin(), all_points.end(), [&left_bf](const std::shared_ptr<CrossPoint<T>>& lhs, const std::shared_ptr<CrossPoint<T>>& rhs) {
        return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
    });
}

template <class T>
void rm_duplicate_pts(std::vector<std::shared_ptr<CrossPoint<T>>>& points, double dis_threshold){
    auto it =points.begin();
    while(it != points.end()){
        auto next_it = std::next(it);
        while(next_it != points.end()){
            if(alg::calc_dis((*it)->pos, (*next_it)->pos) < dis_threshold){
                next_it = points.erase(next_it);
            }else{
                next_it++;
            }
        }
        it++;
    }
}


int RoadModelProcCalCoverage::cal_actual_road_boundary_count(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir,
        std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>>& in_road_boundary_points) {
    //  初始化起点车道数
    double W_lane_st = FLAGS_standard_lane_width2;
    double gap = FLAGS_gap_road_bd_to_lane;

    auto v_dir = alg::get_vertical_dir(road_dir, false);
    double dis_extend = (keypose->from_raw_link->lanenum_sum + 1) * W_lane_st;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;

    //找全部的道路边界交点
    std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>> cross_points;
    // TODO：cxf，这里效率有点慢，应该用 kdtree
    for(const auto& bd_line : session->merge_boundary_line_list){        
        std::vector<UsefulPnt> res;
        bool is_intersect = alg::get_cross_point_with_curve_segment(pt1, pt2, bd_line->list, res, false, 0, 0, true); // 三角岛，或某一侧多边界，所以用true
        for(auto& pt : res){
            auto cross_pt = std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::UNDEF, pt.pos, pt.type, pt.color);
            cross_pt->src_line = bd_line;
            cross_points.push_back(cross_pt);
        }
    }

    //以keypose为中心，分左右
    std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>> left_cross_points, right_cross_points;
    for(auto& cross_pt : cross_points){
        if(alg::judge_left2(cross_pt->pos, keypose->pos, road_dir) == 1){
            left_cross_points.push_back(cross_pt);
        }else{
            right_cross_points.push_back(cross_pt);
        }
    }

    //计算左右的距离keypose最近点的
    auto left_bf = std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::UNDEF, Eigen::Vector3d::Zero());
    auto right_bf = std::make_shared<CrossPoint<BoundaryGroupLine>>(PointStatus::UNDEF, Eigen::Vector3d::Zero());
    if(left_cross_points.size() > 0){
        left_bf = find_closest_point(left_cross_points, keypose->pos);
        in_road_boundary_points[0] = left_bf;
    }

    if(right_cross_points.size() > 0){
        right_bf = find_closest_point(right_cross_points, keypose->pos);
        in_road_boundary_points[1] = right_bf;
    }

    if(left_bf->pos.norm() > 1e-6 && right_bf->pos.norm() > 1e-6 ) {
        double W_road = alg::calc_dis(left_bf->pos, right_bf->pos, true);
        int N_road = static_cast<int>(std::round((W_road - gap) / W_lane_st));
        return N_road;    
    } else {
        return -1;
    }
}



// 实现计算实际的车道中心线数
int RoadModelProcCalCoverage::cal_actual_lane_center_count(RoadModelSessionData* session, KeyPose* keypose, 
    Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
    std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& in_road_center_points) {
    
    std::vector<LaneCenterFeature *> lcfs;
    session->merge_lane_center_sample_tree.search(keypose->pos, search_radius, lcfs);

    std::unordered_set<LaneCenterGroupLine*> lines;
    for(auto& lcf : lcfs){
        lines.insert(lcf->group_line);
    }


    auto v_dir = alg::get_vertical_dir(road_dir, false);
    double dis_extend = 50;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;
        
    //找全部的中心线交点
    std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> cross_points;
    for(const auto& center_line : lines){
        std::vector<UsefulPnt> interp_infos;
        bool is_intersect = alg::get_cross_point_with_curve_segment(pt1, pt2, center_line->list, interp_infos, false, 0, 0.0, false);
        for(int i = 0; i < interp_infos.size(); i++){
            auto cross_pt = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::RAW,
                                                                            interp_infos[i].pos, interp_infos[i].type, interp_infos[i].color);
            cross_pt->src_line = center_line;
            cross_pt->dir = interp_infos[i].dir;
            cross_points.push_back(cross_pt);
        }
    }
    
    //保留道路边界内的中心点
    for(auto &  pt : cross_points){
        if((alg::judge_left2(pt->pos, left_bf, road_dir) < 1) && (alg::judge_left2(pt->pos, right_bf, road_dir) > -1)){
            in_road_center_points.push_back(pt);
        }
    }
    
    //去重: 防止 merge feature 重复的线，影响计算
    rm_duplicate_pts(in_road_center_points, 0.25);
    
    //排序
    sort_points_by_distance(in_road_center_points, left_bf);
    int N_lane_center = in_road_center_points.size();

    return N_lane_center;
}


// 实现计算实际的车道线数
int RoadModelProcCalCoverage::cal_actual_lane_count(RoadModelSessionData* session, KeyPose* keypose, 
    Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
    std::vector<std::shared_ptr<CrossPoint<LaneLineSampleGroupLine>>>& in_road_lane_points) {

    std::vector<LaneLineSample *> lfs; //车道线的点 lf  lane_feature
    session->lane_line_sample_tree.search(keypose->pos, search_radius, lfs);

    std::unordered_set<LaneLineSampleGroupLine*> lines;
    for(auto& lf : lfs){
        lines.insert(lf->group_line);
    }


    auto v_dir = alg::get_vertical_dir(road_dir, false);
    double dis_extend = 50;
    Eigen::Vector3d pt1 = keypose->pos + v_dir * dis_extend;
    Eigen::Vector3d pt2 = keypose->pos - v_dir * dis_extend;
        
    //找全部的车道线交点
    std::vector<std::shared_ptr<CrossPoint<LaneLineSampleGroupLine>>> cross_points;
    for(const auto& lane_line : lines){
        std::vector<UsefulPnt> interp_infos;
        bool is_intersect = alg::get_cross_point_with_curve_segment(pt1, pt2, lane_line->list, interp_infos, false, 0, 0.0, false);
        for(int i = 0; i < interp_infos.size(); i++){
            auto cross_pt = std::make_shared<CrossPoint<LaneLineSampleGroupLine>>(PointStatus::RAW,
                                                                                interp_infos[i].pos, interp_infos[i].type, interp_infos[i].color);
            cross_pt->src_line = lane_line;
            cross_pt->dir = interp_infos[i].dir;
            cross_points.push_back(cross_pt);
        }
    }
    
    //保留道路边界内的车道线点
    for(auto &  pt : cross_points){
        if((alg::judge_left2(pt->pos, left_bf, road_dir) < 1) && (alg::judge_left2(pt->pos, right_bf, road_dir) > -1)){
            in_road_lane_points.push_back(pt);
        }
    }

    //去重: 防止 merge feature 重复的线，影响计算
    rm_duplicate_pts(in_road_lane_points, 0.15);

    //排序
    sort_points_by_distance(in_road_lane_points, left_bf);
    int N_lane = in_road_lane_points.size() -1;

    return N_lane;
}


double RoadModelProcCalCoverage::cal_true_lane_width(const std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& pts, double err){
    auto classify_group_by_error = [&](const std::vector<double>& widths, double error)
    {
        std::vector<std::vector<double>> group_list;
        for(const auto& width : widths){
            bool added = false;

            if(group_list.size() > 0){
                for(auto& group: group_list){
                    if(abs(group[0] - width) < error){
                        group.push_back(width);
                        added = true;
                        break;
                    }
                }
            }else{
                group_list.push_back({width}); //第一次加入
                added = true;  
            }

            if(!added){
                group_list.push_back({width});
            }

        }

        return group_list;
    };

    double std_width = FLAGS_standard_lane_width2;
    std::vector<double> widths;
   
    if(pts.size() < 2){
        return std_width; //用经验值
    }

    //1.计算宽度
    // std::cout << "1: dis: " << std::endl;
    for(int i = 1; i < pts.size(); i++){
        double dis = alg::calc_dis(pts[i]->pos, pts[i-1]->pos);
        widths.push_back(dis);
        // std::cout << dis << ", ";
    }
    // std::cout << std::endl;


    //2.分组
    std::vector<std::vector<double>> group_list = classify_group_by_error(widths, err);
    // std::cout << "分组: " << std::endl;
    // for (const auto& group : group_list) {
    //     for (double width : group) {
    //         std::cout << width << " ";
    //     }
    //     std::cout << std::endl;  // 打印完一组后换行
    // }

    //3.计算平均
    std::map<double, int >avg_group_cnts; //该组平均车道宽度，在组车道出现的次数
    for(const auto& group : group_list){
        if(group.empty()){
            continue;
        }

        double sum = 0;
        for(const auto& width : group){
            sum += width;
        }
        sum /= group.size();
        avg_group_cnts[sum] = group.size();
    }

    // for (const auto& entry : avg_group_cnts) {
    //     std::cout << "(" << entry.first << ", " << entry.second << ") ";
    // }
    // std::cout << std::endl; 

    //4. 找次数最多的
    double max_cnt_width = DBL_MIN;
    double max_cnt = DBL_MIN;
    for(auto info : avg_group_cnts){
        if(info.second > max_cnt){
            max_cnt = info.second;
            max_cnt_width = info.first;
        }else if(info.second == max_cnt && info.first > max_cnt_width){ //如果次数一样多，则选车道宽度大的。（取小会有问题）
            max_cnt_width = info.first;
        }
    }

    double lane_st = (max_cnt_width > 2.7 && max_cnt_width < 3.8) ? max_cnt_width :  std_width;
    // std::cout << "lane_st:" << lane_st << std::endl;
    return lane_st;
}
//版本2
#if 0
void RoadModelProcCalCoverage::cal_horizontal_miss_positions(RoadModelSessionData* session, std::shared_ptr<HorizonCrossFeature> &hor_cross_feature, 
    KeyPose* origin_start, KeyPose* start_keypose, Eigen::Vector3d road_dir,
    int N_hope, int N_lane_center, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
    std::vector<Eigen::Vector3d>& in_road_center_points) {
        
    std::vector<Eigen::Vector3d> all_points; // 理论上的中心点
    std::vector<bool> found(N_hope, false); // 标记对应车道是否缺
    double scale = 1;
    Eigen::Vector3d search_dir = alg::get_dir(left_bf, right_bf);

    // 从 right_bf 开始计算理
    Eigen::Vector3d A = right_bf + search_dir * (W_lane_t / 2.0);
    all_points.push_back(A);

    int found_count = 0;

    for(const auto& pt : in_road_center_points) {
        if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
            // 找到第一个实际车道点，从该点开始继续计算后续的理论中心点
            all_points.back() = pt;
            found[0] = true;
            found_count++;
                break;
            }
        }

    for(int i = 1; i < N_hope; ++i) {
        if(found[i - 1]) {
            // 如果上一个点找到了，基于上一个点的位置继续计算
            A = all_points.back() + search_dir * W_lane_t;
            all_points.push_back(A);
            found[i] = false;

            // 检查新点位置是否找到
            for(const auto& pt : in_road_center_points) {
                if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
                    all_points.back() = pt;
                    found[i] = true;
                    found_count++;
                    break;
                }
            }
        }
        // 如果上一个点没有找到，则直接向 left_bf 方向计算理论点
        else {
            A = right_bf + search_dir * (i * W_lane_t + W_lane_t / 2.0);
            all_points.push_back(A);
            found[i] = false;

            for(const auto& pt : in_road_center_points) {
                if(alg::calc_dis(A, pt) < (W_lane_t / 2.0) * scale) {
                    all_points.back() = pt;
                    found[i] = true;
                    found_count++;
                    break;
                }
            }
        }
    }
    
    for(int i = 0; i < N_hope; ++i) {
        if(found[i]) {
            auto new_cross_pt = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::RAW, all_points[i]);
            hor_cross_feature->lane_center_pts.push_back(new_cross_pt);
        }
        else {
            auto new_cross_pt = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::INIT, all_points[i]);
            hor_cross_feature->lane_center_pts.push_back(new_cross_pt);
        }
    }
}
#endif



bool all_elements_equal(const std::vector<int>& vec) {
    if (vec.empty()) return true; // 空向量视为所有元素相等
    return std::all_of(vec.begin(), vec.end(), [first = vec[0]](int element) { return element == first; });
}

int RoadModelProcCalCoverage::recovery_single_road_cross_point(HorizonCrossFeature* hor_cross_feature, int index, 
        std::vector<std::pair<int,int>>& link_lanenum_ranges) 
{      
    auto use_link_lanenum = [](int i, const std::vector<std::pair<int, int>>& skip_ranges) -> bool 
    {
        for (const auto& range : skip_ranges) {
            if (i >= range.first && i <= range.second) {
                return true;
            }
        }
        return false;  
    };

    //1. 初始化变量
    int N_road = hor_cross_feature->N_road; // 道路边界算出的车道数
    int N_lane = hor_cross_feature->N_lane; // 车道边界算出的车道数
    int N_lane_center = hor_cross_feature->N_lane_center; // 车道中心线算出的车道数
    int N_link = hor_cross_feature->N_link; // link上自带的车道数

    std::vector<int> all_predict_lane_num = {N_road, N_lane, N_lane_center, N_link};
    std::vector<int> all_predict_lane_num_except_link = {N_road, N_lane, N_lane_center};
    int count_non_zero = std::count_if(all_predict_lane_num_except_link.begin(), all_predict_lane_num_except_link.end(), [](int value){
        return value > 0;
    });

    if (count_non_zero <= 1) {
        // std::cout << "not enough info" << std::endl;
        return -1;
    }

    bool used_link = use_link_lanenum(index, link_lanenum_ranges);
    // hor_cross_feature->N_hope = std::max(static_cast<int>(std::round(N_road)), N_link);
    hor_cross_feature->N_hope = static_cast<int>(used_link ? N_link : std::max(static_cast<int>(std::round(N_road)), N_link));

    //2.  如果车道线、中心线、边界线都相等，则认为是已经完整了（如果 link 数也相等，就更好了）
    if ((all_elements_equal(all_predict_lane_num_except_link) == true /*&& hor_cross_feature->N_road > 2*/)
        || all_elements_equal(all_predict_lane_num) == true) {
        hor_cross_feature->is_full_bd = true;
        hor_cross_feature->N_hope = N_road;
    } else if (N_road > 0) {
        //3. 道路边界完整，车道中心线完整， 缺车道线
        if (N_road == N_lane_center && N_road > N_lane) {
            // if(N_road == 1) {
            //     double lane_width = 0;
            //     for (int i = 0; i < N_road; i++) {
            //         lane_width += alg::calc_dis(hor_cross_feature->road_boundary_pts[i], hor_cross_feature->road_boundary_pts[i+1]);
            //     }
            //     lane_width /= N_lane;
            //     double lane_width_half = lane_width/2;
            //     Eigen::vector3d mid = (hor_cross_feature->lane_pts[i]+hor_cross_feature->lane_pts[i+1])/2;
            //     pred[i] = std::make_shared<CrossPoint>(PointStatus::INIT, mid);
            // }
            // auto& left_bf = hor_cross_feature->road_boundary_pts[0].pos;
            // // auto& right_bf = hor_cross_feature->road_boundary_pts[1].pos;
            // std::sort(hor_cross_feature->lane_center_pts.begin(), hor_cross_feature->lane_center_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
            //         });
            // std::sort(hor_cross_feature->lane_pts.begin(), hor_cross_feature->lane_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
            //         });
            // hor_cross_feature->N_hope = N_road;
            hor_cross_feature->is_full_bd = true;
        } else if (N_road == N_lane && N_road > N_lane_center) {
            //4. 道路左右边界完整， 车道线也完整， 但车道中心线缺失，则补点
            // auto& left_bf = hor_cross_feature->road_boundary_pts[0]->pos;
            // auto& right_bf = hor_cross_feature->road_boundary_pts[1].pos;
            // std::sort(hor_cross_feature->lane_pts.begin(), hor_cross_feature->lane_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
            //         });
            // std::sort(hor_cross_feature->lane_center_pts.begin(), hor_cross_feature->lane_center_pts.end(), 
            //         [left_bf] (const std::shared_ptr<CrossPoint> &lhs, const std::shared_ptr<CrossPoint> &rhs) -> bool {
            //             return alg::calc_dis(lhs->pos, left_bf) < alg::calc_dis(rhs->pos, left_bf);
            //         });

            //4.1 计算需补点的位置
            //计算车道宽度
            auto lane_list = hor_cross_feature->lane_pts;
            double lane_width = 0;
            for (int i = 0; i < N_lane; i++) {
                lane_width += alg::calc_dis(lane_list[i]->pos, lane_list[i+1]->pos);
            }
            lane_width /= N_lane;
            lane_width = std::max(std::min(lane_width, 4.0), 3.0); //TODO: CXF crash 优先查这里
            double lane_width_half = lane_width/2;

            //计算横向中最小车道宽度
            double min_lane_width = DBL_MAX;
            bool found_min_width = false;
            if(lane_list.size()>2){
                for(int i = 0; i < lane_list.size()-1; i++){
                    double tem_width = alg::calc_dis(lane_list[i]->pos, lane_list[i + 1]->pos);
                    if(tem_width > 3.0 && tem_width < min_lane_width){
                        // std::cout << "tem_width: " << tem_width << " min_lane_width:" << min_lane_width <<std::endl;
                        min_lane_width = tem_width;
                        found_min_width = true;
                    }
                }
            }

            //计算原有点的索引，找出哪个车道是缺失的； 缺失的车道 predict_lane_centers 对应的值为 nullptr
            const auto lane_pt_0 = hor_cross_feature->lane_pts[0]->pos;
            std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> predict_lane_centers(N_lane, nullptr);
            for (int i = 0; i < N_lane_center; i++) {
                double d = std::fabs(alg::calc_dis(hor_cross_feature->lane_center_pts[i]->pos, lane_pt_0) - lane_width_half) / lane_width;
                int target_index = static_cast<int>(std::round(d)); 
                if(target_index >= N_lane){
                    LOG_WARN("需要调大车道宽度阈值4.0, target_index:{}, N_lane:{}", target_index, N_lane);
                    continue;
                }else{
                    predict_lane_centers[target_index] = hor_cross_feature->lane_center_pts[i];
                }
            }
            
            //4.2 补点逻辑， 并标记错误的点
            std::unordered_set<int> erase_index; //校验后，需要剔除的补点
            for (int i = 0; i < N_lane; i++) {
                auto& pred = predict_lane_centers[i];
                if(pred == nullptr) {
                    Eigen::Vector3d mid = (lane_list[i]->pos + lane_list[i+1]->pos)/2;
                    pred = std::make_shared<CrossPoint<LaneCenterGroupLine>>(PointStatus::INIT, mid);
                    pred->dir = lane_list[i]->dir;  

                    //标记错误的补点： a、车道太窄
                    double dis = alg::calc_dis(lane_list[i]->pos, lane_list[i+1]->pos);
                    if(dis < lane_width * FLAGS_first_horizon_push_scale){ //*0.9
                        erase_index.insert(i);
                    }

                    //标记错误的补点： b、车道比其他的宽
                    if(found_min_width == true && dis > min_lane_width + FLAGS_filter_init_pt_threshold){//  大于最小车道宽度 + 0.5  , 最小车道宽度至少>3m
                        double filter_dis = min_lane_width + FLAGS_filter_init_pt_threshold;
                        // std::cout << "dis:" << dis << " min_lane_width: " << min_lane_width <<std::endl;
                        erase_index.insert(i);
                    }
                }
            }


            //4.4剔除错误的补点， 车道线太近的点 ，  TODO:cxf 用上车道线的dir等信息做校验
            std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> valid_predict_lane_centers;
            for(int i = 0; i < predict_lane_centers.size(); i++){
                if(erase_index.find(i) == erase_index.end()){
                    valid_predict_lane_centers.push_back(predict_lane_centers[i]);
                }
            }
            hor_cross_feature->lane_center_pts = valid_predict_lane_centers;
            hor_cross_feature->N_lane_center = valid_predict_lane_centers.size();  //对应修改N_lane_center， 不然会段错误
            // hor_cross_feature->N_hope = N_road;
            hor_cross_feature->is_full_bd = true;
        } else {
            //5.  都无法完全确定
            hor_cross_feature->is_full_bd = true;
            
        }
    } else {
        // 如果没有道路边界线，就先不处理，后续增强
        hor_cross_feature->is_full_bd = false;
    }

    //6. 计算车道数差异  
    int N_diff = hor_cross_feature->N_hope - hor_cross_feature->N_lane_center;
    if (N_diff == 0) {
        hor_cross_feature->is_full_lc  = true;
    }
    hor_cross_feature->N_diff = N_diff;

    return 0;
}



int RoadModelProcCalCoverage::verify_elements(RoadModelSessionData* session){
    transform_init_pts_to_mature(session);

    transform_mature_pts_to_line(session);



    return fsdmap::SUCC;
}

void RoadModelProcCalCoverage::transform_init_pts_to_mature(RoadModelSessionData* session){

    RTreeProxy<LaneCenterFeature *, float, 2> prev_lc_tree;
    RTreeProxy<LaneCenterFeature *, float, 2> next_lc_tree;

    auto check_lane_dir = [](HorizonCrossFeature*& road_cross_pt, int index) ->bool
    {
        //找平均车道中心线的方向
        Eigen::Vector3d lane_center_dir_avg = Eigen::Vector3d::Zero();
        int raw_cnt = 0;
        for(int i = 0 ;i < road_cross_pt->lane_center_pts.size(); i++){
            auto pt = road_cross_pt->lane_center_pts[i];
            if(pt->status == PointStatus::RAW){
                lane_center_dir_avg += pt->dir;
                raw_cnt++;
            }
        }
        
        if(raw_cnt > 0){
            lane_center_dir_avg /= raw_cnt;
        }else{
            return false;
        }
        
        //比较该点两条车道线和 平均车道中心线的方向
        double diff1 = alg::calc_theta1(road_cross_pt->lane_pts[index]->dir, lane_center_dir_avg, true, true);
        double diff2 = alg::calc_theta1(road_cross_pt->lane_pts[index + 1]->dir, lane_center_dir_avg, true, true);

        if(diff1 > FLAGS_filter_init_pt_theta || diff2 > FLAGS_filter_init_pt_theta){  //15度
            return false;
        }else{
            return true;
        }
    };


    auto check_pt_in_split_merge = [&prev_lc_tree, &next_lc_tree,&session](const std::shared_ptr<CrossPoint<LaneCenterGroupLine>>& pt) -> bool
    {
        std::vector<LaneCenterFeature*> prev_lcs;
        prev_lc_tree.search(pt->pos,4,prev_lcs);

        std::vector<LaneCenterFeature*> next_lcs;
        next_lc_tree.search(pt->pos,4,next_lcs);

        if(prev_lcs.size() >=2 || next_lcs.size() >=2){
            return true;
        }else{
            return false;
        }
    };
    //找没前驱后继的lc
    find_no_prev_and_next_lc(session, prev_lc_tree, next_lc_tree);
    

    //1. 点所在的车道宽度， 大于最小车道宽度+阈值T0.5   (生成点的时候已实现)
    
    //2.车道线上的方向与中心线角度 差异15度 
    for(auto &info : session->hor_corss_line_m){
        for(auto& road_cross_pt : info.second->cross_feature_list)
        {
            if(!road_cross_pt->is_full_lc){
                continue;
            }

            auto& lane_center_pts = road_cross_pt->lane_center_pts;
            for(int i = 0 ;i < lane_center_pts.size(); i++)
            {
                if(lane_center_pts[i]->status != PointStatus::INIT){
                    continue;
                }
           
                //角度
                bool is_samll_diff = check_lane_dir(road_cross_pt, i);
                if(is_samll_diff != true){
                    continue;
                }

                //3.分合流处附近，如果该补点位置有两个无prev 或next，的也过滤
                bool is_in_split_merge = check_pt_in_split_merge(lane_center_pts[i]);
                if(is_in_split_merge == false){
                    lane_center_pts[i]->status = PointStatus::MATURE;
                }
            }
        }
    }


    //TODO:cxf  待扩展
    //3.用上Keypose  前后  车道宽度变化变化戳 

    //刷一遍full_lc状态
    for(auto &info : session->hor_corss_line_m){
        for(auto& road_cross_pt : info.second->cross_feature_list)
        {
            for(auto& lc : road_cross_pt->lane_center_pts)
            {
                //不全是RAW和MATURE， 则full_lc=false;
                if (lc->status != PointStatus::RAW && lc->status != PointStatus::MATURE) {
                    road_cross_pt->is_full_lc = false;
                }
            }
        }
    }
}

void RoadModelProcCalCoverage::find_continuous_pt(std::vector<HorizonCrossFeature*>& cross_feature_list, 
                    std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& temp_line, 
                    std::set<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& hash_tabel, int i, int j) 
{
    if(i >= cross_feature_list.size()){
        return;
    }
    
    if(j >= cross_feature_list[i]->lane_center_pts.size()){
        return;
    }
    
    auto pt = cross_feature_list[i]->lane_center_pts[j];
    if(hash_tabel.count(pt)){
        return;
    }
    
    if (pt->status == PointStatus::MATURE && pt->is_create_line == false) {
        temp_line.push_back(pt);
        hash_tabel.insert(pt);
        find_continuous_pt(cross_feature_list, temp_line, hash_tabel, i + 1, j);
    }else{
        return;
    }
}

int RoadModelProcCalCoverage::transform_mature_pts_to_line(RoadModelSessionData* session) {

    auto create_new_line = [&session](KeyPose* keypose, std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& temp_line){
        auto new_line = session->add_ptr(session->lane_center_group_line_ptr);  
        new_line->del_previous_pts = false;
        new_line->id = std::to_string(session->lane_center_id++); 
        new_line->boundary_type = LaneType::MATURE_VLC;
        
        //加点
        // if(keypose->in_out_status == 2){
        //从前往后遍历
        // TODO new_node->type根据端点来赋值
        for(int i = 0 ; i < temp_line.size(); i++){
            auto& cp = temp_line[i];
            cp->src_line = new_line.get();
            cp->is_create_line = true;
            
            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
            new_node->pos = cp->pos;
            new_node->boundary_type = LaneType::MATURE_VLC; 
            new_node->type = cp->type;
            new_node->color = cp->color;
            new_node->group_line = new_line.get();
            new_node->prev = NULL;
            new_node->next = NULL;
            new_line->list.push_back(new_node);  
            
        }

        //设新点的dir和前驱后继
        std::shared_ptr<LaneCenterFeature> prev = nullptr;
        for(int i = 0; i < new_line->list.size() ; i++){ 
            auto& cur_pt = new_line->list[i];
            
            if (prev == nullptr) {
                prev = cur_pt;
                continue;
            }
            cur_pt->dir = alg::get_dir(cur_pt->pos, prev->pos);
            prev->dir = cur_pt->dir;
            cur_pt->set_prev(prev.get());
            prev = cur_pt;

        }
        
        if(!new_line->list.empty()){
            session->merge_lane_center_list.push_back(new_line.get());

        }
    };
    
    std::cout << "origin line num:" << session->merge_lane_center_list.size() << std::endl;
    for (auto &info : session->hor_corss_line_m) {
        std::set<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> hash_tabel;
        for (int i = 0; i < info.second->cross_feature_list.size(); i++) {
            for (int j = 0; j < info.second->cross_feature_list[i]->lane_center_pts.size(); j++) {
                auto& feature = info.second->cross_feature_list[i];
                std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> temp_line;
                if(feature->lane_center_pts[j]->status == PointStatus::MATURE && feature->lane_center_pts[j]->is_create_line == false)
                {
                    find_continuous_pt(info.second->cross_feature_list, temp_line, hash_tabel, i, j);
                    
                    if(temp_line.size() >=2){
                        create_new_line(info.first, temp_line);
                    }

                    temp_line.clear();
                }

                
            }
        }
        hash_tabel.clear();
    }

    //后面做了 时序刷full，这里暂时注释
    //mature为单点的，状态改回INIT  ： 防止离散的full出现
    for (auto &info : session->hor_corss_line_m) {
        std::set<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>> hash_tabel;
        for (int i = 0; i < info.second->cross_feature_list.size(); i++) {
            for (int j = 0; j < info.second->cross_feature_list[i]->lane_center_pts.size(); j++) 
            {
                auto& feature = info.second->cross_feature_list[i];
                if(feature->lane_center_pts[j]->status == PointStatus::MATURE && feature->lane_center_pts[j]->src_line == nullptr)
                {
                    feature->lane_center_pts[j]->status = PointStatus::INIT;
                    feature->N_lane_center--;
                    feature->N_diff++;
                    feature->is_full_lc = false;
                }
            }
        }
    }
    std::cout << "update line num:" << session->merge_lane_center_list.size() << std::endl;

    //刷一遍kd tree，以防新点没加进来
    update_lane_center_sample_tree(session);
    return fsdmap::SUCC;
}


//========步骤2的函数========

// 通过时序修正一些异常的跳变点： is_full_lc
int RoadModelProcCalCoverage:: modify_is_full_lc_by_neighbor(RoadModelSessionData* session)
{
    for(auto &info : session->hor_corss_line_m){
        int SKIP_KEYPOSE_NUM = 15; // 15 * 2 = 30m, 前面的15个keypose不用，因为右转处分合流和路中段的不一样
        HorizonCrossLine* hor_cross_line = info.second;
        auto& feature_list = hor_cross_line->cross_feature_list;
        int feature_size = feature_list.size();

        int window_size = 3;
        for (int i = SKIP_KEYPOSE_NUM; i < feature_size - window_size; i++) {
            if(feature_list[i]->is_full_lc == true) {
                continue;
            }

            bool neighbor_is_full = true;
            for (int j = 0; j < window_size; j++) {
                if (j == 0) {
                } else {
                    neighbor_is_full = neighbor_is_full && feature_list[i-j]->is_full_lc;
                    neighbor_is_full = neighbor_is_full && feature_list[i+j]->is_full_lc;
                }

                if(neighbor_is_full == false) {
                    break;
                }
            }

            if(neighbor_is_full) {
                feature_list[i]->is_full_lc = true;
            }
        }
    }

    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::modify_N_road_by_neighbor(RoadModelSessionData* session)
{
    for (auto &info : session->hor_corss_line_m) {
        int SKIP_KEYPOSE_NUM = 1; // 15 * 2 = 30m, 前面的15个keypose不用，因为右转处分合流和路中段的不一样
        HorizonCrossLine* hor_cross_line = info.second;
        auto& feature_list = hor_cross_line->cross_feature_list;
        int feature_size = feature_list.size();

        int window_size = 2; // 使用2个窗口来修正异常跳变
        for (int i = SKIP_KEYPOSE_NUM; i < feature_size - window_size; i++) {
            int current_N_road = feature_list[i]->N_road;

            // 跳过没有异常的点
            if (current_N_road == feature_list[i + 1]->N_road) {
                continue;
            }

            bool neighbor_consistent = true;

            // 检查当前点的前后几个点是否车道数一致
            for (int j = 1; j <= window_size; j++) {
                if (i - j >= 0 && i + j < feature_size) {
                    if (feature_list[i - j]->N_road != feature_list[i + j]->N_road) {
                        neighbor_consistent = false;
                        break;
                    }

                    if(j >=2){
                        if(feature_list[i - j]->N_road != feature_list[i - j + 1]->N_road || 
                                feature_list[i + j]->N_road != feature_list[i + j - 1]->N_road)
                        {
                            neighbor_consistent = false;
                             break;
                        }
                    }
                }
            }

            // 如果相邻点的车道数一致，修正当前点
            if (neighbor_consistent) {
                feature_list[i]->N_road = feature_list[i + 1]->N_road;
            }
        }
    }

    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::gen_lane_directin(RoadModelSessionData *session)
{
    RTreeProxy<LaneCenterFeature *, float, 2> lane_center_tree;
    for (auto line : session->merge_lane_center_list)
    {
        for (auto pt : line->list)
        {
            lane_center_tree.insert(pt->pos, pt.get());
        }
    }
    for (auto link : session->link_sample_list)
    {
        for (auto poss : link->list)
        {
            std::vector<Eigen::Vector3d> dirs;
            std::vector<LaneCenterFeature *> lcfs;
            dirs.push_back(alg::get_vertical_dir(poss->dir));
            auto pos=poss->pos;
            lane_center_tree.search(pos, 5, lcfs);
            if (!lcfs.empty())
            {
                for (const auto lc : lcfs)
                {
                    auto vdir = alg::get_vertical_dir(lc->dir);
                    dirs.push_back(vdir);
                }
            }
            // 垂向
            poss->road_vertical_dir = alg::get_direction(dirs);
            if(alg::calc_theta1(  poss->road_vertical_dir,poss->dir,true)>90)
            {
                poss->road_vertical_dir=- poss->road_vertical_dir;
            }
        }
    }
    return fsdmap::SUCC;
}

//=========================总流程结束 ====================================

//==========================结果可视化====================================
int RoadModelProcCalCoverage::save_debug_info(RoadModelSessionData* session, int stage) {
    if (!FLAGS_cal_coverage_save_data_enable) {
        return fsdmap::SUCC;
    }

    std::vector<int> close_stages = {0, 1, 2, 3, 4, 6};
    if (!FLAGS_coverage_detial_img_enable && std::find(close_stages.begin(), close_stages.end(), stage) != close_stages.end()) {
        return fsdmap::SUCC;
    }

    std::string display_name = "cal_coverage" + std::to_string(stage);
    session->set_display_name(display_name.c_str());

    //link: mask掉的
    for (auto &line : session->link_sample_list) {
        auto log_mask = session->add_debug_log(utils::DisplayInfo::LINE, "link");
        log_mask->color = {211, 211, 211};  //灰色
        for (auto &key_pose : line->list) {
            if(key_pose->is_mask == true){ 
                log_mask->add(key_pose->pos, 10);  
            }
        }
    }
    //link: 不mask掉的
    for (auto &line : session->link_sample_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "link");
        log->color = {255, 128, 255};
        for (auto &key_pose : line->list) {
                log->add(key_pose->pos, 10);  
        }
    }

    //任务的中心点
    auto center_log = session->add_debug_log(utils::DisplayInfo::POINT, "center");
    center_log->color = {235, 52, 235};
    center_log->add(session->data_processer->_center_link_pos, 10);  
    auto center_text_log= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", "site_center");
    center_text_log->color = {235, 52, 235}; 
    center_text_log->add(session->data_processer->_center_link_pos);

    //路口需跳过的起始和终止点
    for(auto &link_info : session->link_inter_info){
        for(auto& info : link_info.second){
            auto in_log = session->add_debug_log(utils::DisplayInfo::POINT, "in_log");
            in_log->color = {255, 255, 0};  //黄
            in_log->add(info.second->in_keypose->pos);
            auto out_log = session->add_debug_log(utils::DisplayInfo::POINT, "out_log");
            out_log->color = {0, 0, 255};  //黄
            out_log->add(info.second->out_keypose->pos);
        }
    }

    auto start_pt_log = session->add_debug_log(utils::DisplayInfo::POINT, "start_log");
    start_pt_log->color = {255, 255, 0};  //黄
    int start_cnt =0;
    for(auto& info : session->hor_corss_line_m){
        auto start_log= session->add_debug_log(utils::DisplayInfo::TEXT, "start: {}", start_cnt);
        start_log->color = {255, 255, 0};  //黄
        start_log->add(info.first->pos);
        start_pt_log->add(info.first->pos);
        start_cnt++;
    }

    //各种N的数字打印
    if(stage == 2 || stage == 4 || stage == 6){
        bool draw_from_top_to_bottom = false;
        bool draw_is_full = true;   //true 用full区分，   false用N_road变化地方区分
        for(auto &info : session->hor_corss_line_m){
            int pre_N_road = 0;
            for(auto& road_cross_pt : info.second->cross_feature_list){
                if(road_cross_pt->N_hope != 0){
                    //偏移画
                    if(draw_from_top_to_bottom == true){
                        std::vector<int> N_list = {road_cross_pt->N_hope,road_cross_pt->N_link, 
                            road_cross_pt->N_road, road_cross_pt->N_lane, road_cross_pt->N_lane_center};
    
                        double offset = 0;
                        for(const auto& num : N_list){
                            auto log_hope= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", num);
                            if(draw_is_full)
                            {
                                if (draw_is_full && road_cross_pt->is_full_lc) {
                                    log_hope->color = {255, 150, 100}; // 浅橘色
                                } else {
                                    log_hope->color = {255, 255, 41};  // 黄色
                                }
                            }

                            if(draw_is_full == false)
                            {
                                if(road_cross_pt->N_road > pre_N_road){
                                    log_hope->color = {201, 141, 180};  // 浅粉色
                                }else if(draw_is_full == false && road_cross_pt->N_road < pre_N_road){
                                    log_hope->color = {255, 255, 41};  // 黄色
                                }else{
                                    log_hope->color = {255, 150, 100}; // 浅橘色
                                }
                            }

                            double x = road_cross_pt->keypose->pos.x() + offset;
                            double y = road_cross_pt->keypose->pos.y() + offset;
                            Eigen::Vector3d pos = {x, y ,0};
                            log_hope->add(pos, 0.05);  
                            offset += 1; 
                        }
                    }else{
                        //cv库默认文本横着画
                        auto log_hope= session->add_debug_log(utils::DisplayInfo::TEXT, "{},{},{},{},{}", 
                            road_cross_pt->N_hope,road_cross_pt->N_link, road_cross_pt->N_road, road_cross_pt->N_lane, road_cross_pt->N_lane_center);
                        // auto log_hope= session->add_debug_log(utils::DisplayInfo::TEXT, "{} {},", road_cross_pt->N_hope, road_cross_pt->N_link);
                        if(draw_is_full)
                        {
                            if (draw_is_full && road_cross_pt->is_full_lc) {
                                log_hope->color = {255, 150, 100}; // 浅橘色
                            } else {
                                log_hope->color = {255, 255, 41};  // 黄色
                            }
                        }

                        if(draw_is_full == false)
                        {
                            if(road_cross_pt->N_road > pre_N_road){
                                log_hope->color = {201, 141, 180};  // 浅粉色
                            }else if(draw_is_full == false && road_cross_pt->N_road < pre_N_road){
                                log_hope->color = {255, 255, 41};  // 黄色
                            }else{
                                log_hope->color = {255, 150, 100}; // 浅橘色
                            }
                        }

                        log_hope->add(road_cross_pt->keypose->pos, 0.05);  
                    }
                }
                pre_N_road = road_cross_pt->N_road;
            }
        }
    }

    //link—16
    auto log1 = session->add_debug_log(utils::DisplayInfo::POINT, "in");
    log1->color = {0, 255, 0}; 
    for(auto &kp : in_link_pts){
        log1->add(kp->pos);
    }
   
    //停止线
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6)
        {
            auto log3 = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            log3->color = {255, 0, 255};
            for(auto& pt : object->list){
                log3->add(pt->pos, 10);
            }
            // //停止线对应的link点
            // for(auto& keypose : object->bind_links){
                //     log3_link->add(keypose->pos, 10);
                // }
        }
    }

    //道路边界 左右边界点
    // if (stage != 7) {
    {
        auto log_bd = session->add_debug_log(utils::DisplayInfo::POINT, "bd_point");
        log_bd->color = {74, 88, 240};  //深蓝 
        auto log_bd2 = session->add_debug_log(utils::DisplayInfo::POINT, "bd_point2");
        log_bd2->color = {255, 112, 224}; //粉
        auto log_bd_new = session->add_debug_log(utils::DisplayInfo::POINT, "bd_new");
        log_bd_new->color = {176, 0, 129}; //深粉，新添加的
        for(auto &info : session->hor_corss_line_m){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                if(!road_cross_pt->road_boundary_pts.empty()){
                    if(road_cross_pt->road_boundary_pts[0]->status == PointStatus::INIT){ 
                        log_bd_new->add(road_cross_pt->road_boundary_pts[0]->pos); //新增的
                    }else{
                        log_bd->add(road_cross_pt->road_boundary_pts[0]->pos);
                    }

                    if(road_cross_pt->road_boundary_pts[1]->status == PointStatus::INIT){
                        log_bd_new->add(road_cross_pt->road_boundary_pts[1]->pos); //新增的
                    }else{
                        log_bd2->add(road_cross_pt->road_boundary_pts[1]->pos);
                    }
                }
            }
            
        }

        //交到的车道中心线的点, 包含原有的， 和缺失的   
        auto log_raw_lc = session->add_debug_log(utils::DisplayInfo::POINT, "raw_lc");
        log_raw_lc->color = {214, 255, 216}; //浅绿色
        auto log_init_lc = session->add_debug_log(utils::DisplayInfo::POINT, "init_lc");
        log_init_lc->color = {255, 0, 0}; //红
        auto log_mature_lc = session->add_debug_log(utils::DisplayInfo::POINT, "mature_lc");
        log_mature_lc->color = {247, 8, 175}; //粉色

        bool print_index = false;
        for(auto &info : session->hor_corss_line_m){
            for(auto& road_cross_pt : info.second->cross_feature_list){
                for(int i = 0; i < road_cross_pt->lane_center_pts.size();i++){
                    auto coss_point= road_cross_pt->lane_center_pts[i];
                    if(coss_point->status == PointStatus::INIT){
                        if(print_index){
                            auto log_init_lc_index= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", i);
                            log_init_lc_index->color = {255, 0, 0}; //红色
                            log_init_lc_index->add(coss_point->pos);
                        }
                        log_init_lc->add(coss_point->pos);
                    }else if(coss_point->status == PointStatus::MATURE){
                        auto &mature_ele =log_mature_lc->add(coss_point->pos);
                        
                        mature_ele.label.distance_x_cm = coss_point->is_create_line ? 100 : 0;
                        if(print_index){
                            auto log_mature_lc_index= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", i);
                            log_mature_lc_index->color = {247, 8, 175}; //粉色
                            log_mature_lc_index->add(coss_point->pos);
                        }
                    }else if(coss_point->status == PointStatus::RAW){
                        if(print_index){
                            auto log_raw_lc_index= session->add_debug_log(utils::DisplayInfo::TEXT, "{}", i);
                            log_raw_lc_index->color = {0, 255, 0}; //绿色
                            log_raw_lc_index->add(coss_point->pos);
                        }
                        log_raw_lc->add(coss_point->pos);
                    }
                } 
            }
        }
    }
     
    // //调试  road的横向方向   
    // for(auto &info : session->hor_corss_line_m){
    //     for(auto& road_cross_pt : info.second->cross_feature_list){
    //         auto log_road_hor_dir = session->add_debug_log(utils::DisplayInfo::LINE, "road_hori_dir");
    //         log_road_hor_dir->color = {255, 0, 0}; //红
    //         auto v_dir = alg::get_vertical_dir(road_cross_pt->road_dir);
    //         double dis_extend = (road_cross_pt->keypose->from_raw_link->lanenum_sum + 1) * road_cross_pt->W_lane_st;
    //         Eigen::Vector3d pt1 = road_cross_pt->keypose->pos + v_dir * dis_extend;
    //         Eigen::Vector3d pt2 = road_cross_pt->keypose->pos - v_dir * dis_extend;
    //         log_road_hor_dir->add(pt1);
    //         log_road_hor_dir->add(pt2);
    //     }
    // }
    


    //=========merge_feature的线==============
    int cnt = 0;
    for (auto& group_list : session->merge_boundary_line_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
        if (stage == 0) {
            log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_boundary");
        }
        log->color = {81, 89, 240};
        BoundaryFeature* prev = NULL;
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            double d = 5;
            if (prev) {
                d = alg::calc_dis(pt->pos,prev->pos);
            } else {
                prev=pt.get();
            }

            // if(j < 2 || d>=4 || j==group_list->list.size()-1 || group_list->boundary_type == LaneType::ISLAND_RB) {
            if(1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score =cnt;
                if (pt->invalid()) {
                    ele.color = {255, 255, 100};
                }
                ele.label.label = 1; // 临时弄的
                prev=pt.get();
            }
        }
        cnt++;
    }
    #if 1
    cnt = 0;
    for (auto& group_list : session->merge_lane_line_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
        if (stage == 0) {
            log = session->add_debug_log(utils::DisplayInfo::POINT, "merge_lane");
        }
        log->color = {255, 255, 255};
        LaneLineSample*prev=NULL;
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            double d = 5;
            if (prev) {
                d = alg::calc_dis(pt->pos,prev->pos);
            } else {
                prev=pt.get();
            }

            // if(j<2 || d>=4 || j==group_list->list.size()-1) {
            if(1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score = cnt;
                ele.label.label = 2; // 临时弄的
                // ele.label.score = pt->score;
                if (pt->invalid()) {
                    ele.color = {255, 0, 0};
                }
                ele.label.cloud_pano_seg=pt->attr.type;
                ele.label.cloud_line_seg=pt->attr.color;
                // 
                // ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)
                prev=pt.get();
            }
        }
        cnt++;
    }

    cnt = 0;
    for (auto& group_list : session->merge_lane_center_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane_center");
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
            } else {
                prev=pt.get();
            }
            if(j<2 || d>=4 || j==group_list->list.size()-1) {
            // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                auto &ele = log->add(pt->pos, 1);
                ele.label.opt_label = j;
                ele.label.intensity_opt = group_list->cur_line_id;
                ele.label.score = cnt;
                ele.label.label = 3; // 临时弄的
                if (pt->invalid()) {
                    ele.color = {100, 100, 100};
                }
                // ele.label.intensity_opt=cnt++;
                prev=pt.get();
            } 
        }
        cnt++;
    }
    #endif

    session->save_debug_info(display_name.c_str());

    return fsdmap::SUCC;
}

int RoadModelProcCalCoverage::save_debug_info_all(RoadModelSessionData* session, int stage)
{
    if (!FLAGS_coverage_detial_img_enable ) {
        return fsdmap::SUCC;
    }

    // 可视化无前驱后继的线
    LOG_INFO("start polt init_candidates")
    session->set_display_name("init_candidates");
    int count=0;
    for(auto p:init_candidates) {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        if(p.type=="prev") {
            log->color={255,0,0};
        }
        if(p.type=="next") {
            log->color={0,255,0};
        }
        if(p.type=="cross_mask") {
            log->color={0,255,255};
        }
        log->add(p.ps->pos);
    }
    session->save_debug_info("init_candidates");

        
    LOG_INFO("start polt init_connect_candidates")
    session->set_display_name("init_connect_candidates");
    for(auto line : init_connect_candidates)
    {
      auto ps_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        ps_log->color={0,0,255};
        ps_log->add(line.ps->pos);
        auto pe_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        pe_log->color={255,125,0};
        pe_log->add(line.pe->pos);

        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        log->color={255,255,255};
        auto &ele=log->add(line.ps->pos);
        ele.label.score=line.radius;
        ele.label.label=1;
        for(auto p:line.points)
        {
            auto &ele2 = log->add(p.pos);
            ele2.label.score=0;
            ele2.label.label=100;
        }
        auto &ele3 = log->add(line.pe->pos);
        ele3.label.score=line.radius;
        ele3.label.label=1;

    }
    session->save_debug_info("init_connect_candidates");

 
    LOG_INFO("start polt fb_check_candidates")
    session->set_display_name("fb_check_candidates");
    for(auto line:fb_check_candidates)
    {
        auto ps_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        ps_log->color={0,0,255};
        ps_log->add(line.ps->pos);
        auto pe_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        pe_log->color={255,125,0};
        pe_log->add(line.pe->pos);

        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        log->color={255,255,255};
        auto &ele=log->add(line.ps->pos);
        ele.label.score=line.radius;
        ele.label.label=1;
        for(auto p:line.points)
        {
            auto &ele2 = log->add(p.pos);
            ele2.label.score=0;
            ele2.label.label=100;
        }
        auto &ele3 = log->add(line.pe->pos);
        ele3.label.score=line.radius;
        ele3.label.label=1;

     
    }
    session->save_debug_info("fb_check_candidates");
    
    LOG_INFO("start polt fb_fit_candidates")
    session->set_display_name("fb_fit_candidates");
    for(auto line:fb_fit_candidates)
    {
      auto ps_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        ps_log->color={0,0,255};
        ps_log->add(line.ps->pos);
        auto pe_log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
        pe_log->color={255,125,0};
        pe_log->add(line.pe->pos);

        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "line_{}",count++);
        log->color={255,255,255};
        auto &ele=log->add(line.ps->pos);
        ele.label.score=line.radius;
        ele.label.label=1;
        for(auto p:line.points)
        {
            log->add(p.pos);
            ele.label.score=0;
            ele.label.label=100;
        }
        log->add(line.pe->pos);
        ele.label.score=line.radius;
        ele.label.label=1;
    }
    session->save_debug_info("fb_fit_candidates");

    LOG_INFO("start polt del_short_lane_center_list")
    session->set_display_name("del_short_lane_center_list");
    for(auto line : del_short_lane_center_list)
    {
        for(auto& pt : line->list){
            auto log = session->add_debug_log(utils::DisplayInfo::POINT, "line_{}",count++);
            log->color={255,0,0};
            log->add(pt->pos);
        }
    }
    session->save_debug_info("del_short_lane_center_list");
    

    return fsdmap::SUCC;
}

}

}
