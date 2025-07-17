

#include "road_model_proc_build_boundary.h"

namespace fsdmap {
namespace road_model {

DEFINE_bool(build_boundary_enable, true, "build_boundary_enable");
DEFINE_bool(build_boundary_debug_pos_enable, true, "build_boundary_debug_enable");
DEFINE_bool(build_boundary_save_data_enable, true, "build_boundary_save_data_enable");

DEFINE_bool(build_boundary_use_sem_enable, false, "build_boundary_use_sem_enable");
// DEFINE_bool(build_boundary_use_trail_enable, true, "build_boundary_use_trail_enable");
DEFINE_bool(build_boundary_use_trail_enable, false, "build_boundary_use_trail_enable");
DEFINE_double(build_boundary_max_same_dir_feature_radius, 50, "build_boundary_max_same_dir_feature_radius");
DEFINE_double(build_boundary_opposite_pos_same_dir_theta, 30, "build_boundary_opposite_pos_same_dir_theta");
DEFINE_double(build_boundary_opposite_pos_scope_threshold, 3, "build_boundary_opposite_pos_scope_threshold");
DEFINE_double(build_boundary_max_same_pos_cb_theta, 60, "build_boundary_max_same_dir_theta");
DEFINE_double(build_boundary_mark_yellow_dis_threshold, 1, "build_boundary_mark_yellow_dis_threshold");
DEFINE_double(build_boudnary_yellow_spread_length, 20, "build_boudnary_yellow_spread_length");
DEFINE_double(build_boudnary_yellow_spread_gap_threshold, 1, "build_boudnary_yellow_spread_gap_threshold");
DEFINE_double(build_boundary_rc_scope_dis_threshold, 1, "build_boundary_rc_scope_dis_threshold");
DEFINE_double(build_boundary_smooth_variance_lambda, 0.3, "build_boundary_smooth_variance_lambda");
DEFINE_double(build_boundary_road_center_min_width, 2.7, "build_boundary_road_center_min_width");
DEFINE_double(build_boundary_smooth_road_center_scope, 25, "build_boundary_smooth_road_center_scope");
DEFINE_double(build_boundary_link_dir_rate_threshold, 0.7, "build_boundary_link_dir_rate_threshold");
DEFINE_double(build_boundary_match_lidar_rate_2, 0.3, "build_boundary_match_lidar_rate_2");
DEFINE_double(build_boundary_match_lidar_rate_1, 0.7, "build_boundary_match_lidar_rate_1");
DEFINE_double(build_boundary_search_poss_search_radius, 5, "build_boundary_search_poss_search_radius");
DEFINE_double(build_boundary_search_poss_search_scope_h_1, 5, "build_boundary_search_poss_search_scope_h_1");
DEFINE_double(build_boundary_search_poss_search_scope_v_1, 0.3, "build_boundary_search_poss_search_scope_v_1");
DEFINE_double(build_boundary_search_poss_search_scope_h_2, 5, "build_boundary_search_poss_search_scope_h_2");
DEFINE_double(build_boundary_search_poss_search_scope_v_2, 1.5, "build_boundary_search_poss_search_scope_v_2");
DEFINE_double(build_boundary_tracking_dis_threshold, 2, "build_boundary_tracking_dis_threshold");
DEFINE_double(build_boundary_tracking_max_theta, 15, "build_boundary_tracking_max_theta");

DEFINE_double(build_boundary_mark_yellow_road_width_threshold, 7, "build_boundary_mark_yellow_road_width_threshold");
DEFINE_double(build_boundary_mark_yellow_min_width_threshold, 3, "build_boundary_mark_yellow_min_width_threshold");
DEFINE_double(build_boundary_rebuild_min_dis_threshold, 4, "build_boundary_rebuild_min_dis_threshold");
DEFINE_double(build_boundary_pos_match_lane_radius, 30, "bind_trail_pos_match_lane_radius");
DEFINE_double(build_boundary_pos_match_lane_z_max, 6, "bind_trail_pos_match_lane_z_max");
DEFINE_double(build_boundary_pos_match_lane_z_min, -2, "bind_trail_pos_match_lane_z_min");
DEFINE_double(build_boundary_pos_match_lane_theta, 30, "bind_trail_pos_match_lane_theta");

fsdmap::process_frame::PROC_STATUS RoadModelProcBuildBoundary::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_build_boundary_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_build_boundary_debug_pos_enable;

    // 从轨迹点开始向前探路，生成每个轨迹点匹配到的符合方向的line集合
    CHECK_FATAL_PROC(valid_boundary_by_lidar(session), "valid_boundary_by_lidar");

    // 按照轨迹点搜索每条轨迹最近的左右边界
    CHECK_FATAL_PROC(search_boundary(session), "search_boundary");

    CHECK_FATAL_PROC(build_road_center(session), "build_road_center");

    CHECK_FATAL_PROC(filter_road_center_by_same(session), "filter_road_center_by_same");

    CHECK_FATAL_PROC(build_yellow_boundary(session), "build_yellow_boundary");

    CHECK_FATAL_PROC(filter_road_center_by_oppo(session), "filter_road_center_by_oppo");

    CHECK_FATAL_PROC(build_inter_boundary(session), "build_inter_boundary");

    // 结果可视化
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcBuildBoundary::build_inter_boundary(RoadModelSessionData* session) {
    for (auto &intersection : session->intersections) {
        for (auto &road_segment : session->road_segment_list) {
            for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
                auto &poss = road_segment->pos_sample_list[i];
                session->debug_pos(poss->pos);
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::valid_boundary_by_lidar(RoadModelSessionData* session) {
    // for (auto& line : session->boundary_line_sample_list_opt) {
    //     session->thread_pool->schedule([line, session, this](utils::ProcessBar *process_bar) {
    //             if (!match_lidar_by_line(session, line)) {
    //                 line->filter_status = 2;
    //             }
    //             });
    // }
    session->thread_pool->wait();
    for (auto& line : session->boundary_line_sample_list_opt) {
        if (line->invalid()) {
            continue;
        }
        if (line->src_status != 2) {
            continue;
        }
        for (auto &fls : line->list) {
            session->debug_pos(fls->pos);
            session->boundary_line_sample_tree.insert(fls->pos, fls);
        }
    }
    return fsdmap::SUCC;
}

bool RoadModelProcBuildBoundary::match_lidar_by_line(RoadModelSessionData* session, 
        BoundaryLine* line) {
    double rate_threshold_1 = FLAGS_build_boundary_match_lidar_rate_1;
    double rate_threshold_2 = FLAGS_build_boundary_match_lidar_rate_2;
    double rate_threshold = line->src_status == 2 ? rate_threshold_2 : rate_threshold_1;
    double h_scope_1 = FLAGS_build_boundary_search_poss_search_scope_h_1;
    double v_scope_1 = FLAGS_build_boundary_search_poss_search_scope_v_1;
    double h_scope_2 = FLAGS_build_boundary_search_poss_search_scope_h_2;
    double v_scope_2 = FLAGS_build_boundary_search_poss_search_scope_v_2;
    double h_scope = line->src_status == 2 ? h_scope_2 : h_scope_1;
    double v_scope = line->src_status == 2 ? v_scope_2 : v_scope_1;
    int64_t match_num = 0;
    for (auto &fls : line->list) {
        session->debug_pos(fls->pos);
        if (search_nearest_boundary_lidar(session, fls, v_scope, h_scope) == NULL) {
            continue;
        }
        ++match_num;
    }
    float rate = (float) match_num / line->list.size();
    if (rate > rate_threshold) {
        return true;
    }
    return false;
}

int RoadModelProcBuildBoundary::search_boundary(RoadModelSessionData* session) {
    get_poss_cross_fls(session);
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            session->thread_pool->schedule([poss, session, this](utils::ProcessBar *process_bar) {
                session->debug_pos(poss->pos);
                search_nearest_boundary(session, poss);
                search_opposite_poss(session, poss);    // link上搜索
                search_opposite_trail(session, poss);   // ins上搜索
            });
        }
    }
    session->thread_pool->wait();
    return fsdmap::SUCC;
}

// 遍历session->road_segment_list里面的每条road_segment，遍历road_segment里面的每个poss，根据轨迹左右两侧的Boundary位置来推算道路中心线，
// 当poss轨迹点左右两侧都有boundary时，车道中心点在当前轨迹点poss两侧左右围栏上两个点的平均值，
// 当只有左侧围栏点时，左侧围栏点在poss->dir垂线方向上向右延伸2m就是道路中心点，
// 当只有有的围栏点时，右侧围栏点在poss->dir垂线方向上向左延伸2m就是道路中心点。
int RoadModelProcBuildBoundary::build_road_center(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {    
            session->debug_pos(poss->pos);
            build_road_center_by_poss(session, poss);
            mark_curr_road_center(session, poss);
        }
    }
    // tracking by bev
    for (auto &road_segment : session->road_segment_list) {
        // smooth_road_center_single(session, road_segment);
        // tracking_road_center(session, road_segment, true);
        // tracking_road_center(session, road_segment, false);
    }
    for (auto &road_segment : session->road_segment_list) {
        complete_road_center(session, road_segment, true);
        complete_road_center(session, road_segment, false);
    }

    for (auto &road_segment : session->road_segment_list) {
        sync_road_center(session, road_segment);
    }

    // 对向去重
    for (auto &road_segment : session->road_segment_list) {
        smooth_road_center_width_oppo_poss(session, road_segment);
    }
    // for (auto &road_segment : session->road_segment_list) {
    //     predict_road_center(session, road_segment);
    // }
    // for (auto &road_segment : session->road_segment_list) {
    //     switch_road_center(session, road_segment);
    // }
    // 标记对向车道
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            set_road_index(session, poss);
        }
    }
    return fsdmap::SUCC;
}

// 遍历road_segment_list里面每段road_segment的每个位置poss，搜索离它30范围内的，朝向角度差在30度范围内的车道线点，
// 遍历这些筛选出的车道线点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
// 这些点放入lane_line_sample_ptr
int RoadModelProcBuildBoundary::get_poss_cross_fls(RoadModelSessionData* session) {
    // double turn_theta_threshold = FLAGS_build_boundary_turn_theta_threshold;
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            session->thread_pool->schedule([road_segment, poss, session, this]() {
                    session->debug_pos(poss->pos);
                    std::vector<LaneLineSample*> secs;
                    search_nearest_fls(session, poss, secs); // 在poss附近半径30m的范围内，筛选出与其朝向角度在30度范围内的车道线点
                    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, 2);
                    // auto &boundary = poss->boundary;
                    for (auto &fls : secs) {
                        session->debug_pos2(poss->pos, fls->pos);
                        DisDirPoint cross_point;
                        alg::calc_dir(fls->next->pos, fls->pos, cross_point.dir);// 计算fls->pos到fls->next->pos的方向向量
                        // 计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点
                        if (!alg::get_cross_point_for_segment(fls->pos, fls->next->pos,
                                    poss->pos, poss_v_pt, cross_point.pos, 1)) {
                            continue;
                        }
                        if (alg::calc_theta(cross_point.dir, poss->dir) > 90) {
                            cross_point.dir = -cross_point.dir;
                        }
                        // if (road_segment->type >= 2) {
                        //     double theta = alg::calc_theta(fls->dir, poss->dir, false, true);
                        //     if (theta > turn_theta_threshold) {
                        //         continue;
                        //     }
                        // }
                        auto new_fls = session->add_ptr(session->lane_line_sample_ptr, true);
                        new_fls->init(fls);
                        new_fls->src_fls = fls;
                        new_fls->pos = cross_point.pos;
                        new_fls->dir = cross_point.dir;
                        new_fls->key_pose = poss;
                        new_fls->bind_dis = alg::calc_dis(poss->pos, cross_point.pos); //与绑定轨迹点的距离
                        if (alg::judge_left(new_fls->pos, poss->pos, poss->dir) < 0) {
                            new_fls->bind_dis = -new_fls->bind_dis;
                        }
                        poss->lane_line_feature_sample_list.push_back(new_fls.get());
                    }
            });
        }
    }
    session->thread_pool->wait();
    return fsdmap::SUCC;
}

// 用反向行驶轨迹以及黄色车道线点来构建新的道路中心点，以及更新现有道路中心点位置
int RoadModelProcBuildBoundary::filter_road_center_by_oppo(RoadModelSessionData* session) {
    double rate_threshold = FLAGS_build_boundary_link_dir_rate_threshold;
    // 去除有oppo的-1
    for (auto &road_segment : session->road_segment_list) {
        for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            if (poss->invalid()) {
                continue;
            }
            auto &boundary = poss->boundary;
            if (boundary.same_road_status == 0) { // 有中间围栏，跳过
                continue;
            }
            if (boundary.left_oppo_pos.src == NULL) {
                continue;
            }
            auto &oppo_poss = boundary.left_oppo_pos.src;
            if (oppo_poss->road_segment->id < poss->road_segment->id) {
                continue;
            }
            if (!boundary.curr->in_scope(oppo_poss->boundary.curr->pos)) {
                continue;
            }

            if (boundary.curr->yellow_boundary.src != NULL) {// 如果道路中心有黄色车道线
                auto flsr = session->add_ptr(session->boundary_feature_ptr);
                flsr->pos = boundary.curr->yellow_boundary.src->pos;
                flsr->dir = poss->dir;
                flsr->key_pose = poss;
                flsr->src_status = 6;
                // 用黄色车道线点来构建新的道路中心点，以及更新现有道路中心点位置
                rebuild_road_center_by_fls(session, poss, flsr.get(), true);
                set_road_index(session, poss);
                rebuild_road_center_by_fls(session, oppo_poss, flsr.get(), true);
                set_road_index(session, oppo_poss);
                boundary.curr->yellow_boundary.src = NULL;
                oppo_poss->boundary.curr->yellow_boundary.src = NULL;
            }
            #if 0
             else if (boundary.left_oppo_trail.src != NULL) { // 如果道路中心没有黄色车道线，但是有对向行驶轨迹
                auto flsr = session->add_ptr(session->boundary_feature_ptr);
                flsr->pos = alg::get_vertical_pos(boundary.left_oppo_trail.src->pos,
                        poss->dir, 1.5);// 当前对向轨迹位置向当前对向轨迹位置的左侧延伸1.5m作为道路中心点
                flsr->dir = poss->dir;
                flsr->key_pose = poss;
                flsr->src_status = 6;

                if(!judge_road_center_with_lane_ratio(session, poss, flsr->pos))
                {
                    rebuild_road_center_by_fls(session, poss, flsr.get(), true);
                    set_road_index(session, poss);
                    rebuild_road_center_by_fls(session, oppo_poss, flsr.get(), true);
                    set_road_index(session, oppo_poss);
                }
            } else { 
                auto flsr = session->add_ptr(session->boundary_feature_ptr);
                flsr->pos = alg::get_vertical_pos(boundary.left_oppo_pos.src->pos,
                        poss->dir, 1.5);
                flsr->dir = poss->dir;
                flsr->key_pose = poss;
                flsr->src_status = 6;
                if(!judge_road_center_with_lane_ratio(session, poss, flsr->pos))
                {
                    rebuild_road_center_by_fls(session, poss, flsr.get(), true);
                    set_road_index(session, poss);
                    rebuild_road_center_by_fls(session, oppo_poss, flsr.get(), true);
                    set_road_index(session, oppo_poss);
                }
            }
            #endif
        }
    }
    return fsdmap::SUCC;
}

// 遍历session->road_segment_list里的road_segment，过滤road_segment->pos_sample_list中与其他轨迹pose对应的boundary重复的pose。
int RoadModelProcBuildBoundary::filter_road_center_by_same(RoadModelSessionData* session) {
    UMAP<RoadSegment*, std::pair<int, int>> same_map;
    for (auto &road_segment : session->road_segment_list) {
        for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            bool has_break = false;
            if (poss->invalid()) 
            {
                continue;
            }
            do {
                auto &boundary = poss->boundary;
                // 遍历与该boundary 50m范围内同向的pose点，
                for (auto &same_pos_p : boundary.same_dir_pos) {
                    auto &same_pos = same_pos_p.src;
                    // same_pos->pos是same_pos_p所对应的link的pos，ame_pos->boundary.curr->pos对应的道路中心线的中线点坐标
                    Eigen::Vector3d tar_pos = (same_pos->boundary.curr->pos + same_pos->pos) / 2;
                    for (auto &rc : boundary.road_center_list) {
                        if (rc == boundary.curr) {
                            continue;
                        }
                        if (rc->in_scope(tar_pos)) {// same_pos->boundary已经包含了rc，rc作为重复项滤除
                            rc->filter_status = 3;
                        }
                    }
                    // same_pos的长度更长，说明已经遍历过该road_segment了，不用再处理一遍，因为session->road_segment_list
                    // 已经是按id从小到大排序了
                    if (same_pos->road_segment->id < poss->road_segment->id) {
                        continue;
                    }
                    auto tar_road_segment = same_pos->road_segment;
                    auto tar_poss = same_pos;
                    int left = alg::judge_left(same_pos->pos, poss->pos, poss->dir); // 判断same_pos->pos是否在poss的左侧
                    if (left < 0) { // 更信任左侧的road_segment吗
                        tar_road_segment = road_segment;
                        tar_poss = poss;
                    }
                    if (boundary.curr->in_scope(tar_pos)) {
                        auto &get_poss = tar_road_segment->pos_sample_list[tar_poss->line_index];
                        session->debug_pos(get_poss->pos);
                        get_poss->filter_status = 2;
                    } 
                }
            } while (0);
        }
        // road_segment->link_direction = 3;
        // same_map.clear();
    }
    return fsdmap::SUCC;
}

// 遍历session->road_segment_list里面每个道路段road_segment，遍历road_segment->pos_sample_list里面的每个位置poss，
// 判断该poss位置处的boundary的道路中心点是否存在左右某侧没有边界点，把这些poss处的点选出来，用感知结果来补全缺失侧的点，然后新增车道中心点。
int RoadModelProcBuildBoundary::complete_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment, bool left) {
    // 用bev的数据补齐边界
    std::vector<int64_t> proc_index;
    // 筛选出左侧或者右侧还没有boundary的对应的位置点
    // left == 1,就筛选出不存在左侧围栏的位置
    // left == 0,就筛选出不存在右侧围栏的位置
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        if (left && boundary.curr->left != NULL) {
            continue;
        }
        if (!left && boundary.curr->right != NULL) {
            continue;
        }
        if (left && poss->left_boundary_status > 0) {
            continue;
        }
        if (!left && poss->right_boundary_status > 0) {
            continue;
        }
        proc_index.push_back(i);
    }
    for (int i = 0; i < proc_index.size(); ++i) {
        auto &index = proc_index[i];
        auto &poss = road_segment->pos_sample_list[index];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        double min_dis = DBL_MAX;
        BoundaryFeature* min_fls = NULL;
        for (int j = 0; j < boundary.boundary_feature_list.size(); ++j) {
            auto &fls = boundary.boundary_feature_list[j];
            if (left ^ fls->bind_dis < 0) { // 筛选出不同侧的bev boundary点
                continue;
            }
            if (min_dis > fabs(fls->bind_dis)) {
                min_dis = fabs(fls->bind_dis);
                min_fls = fls;
            }
        }
        if (min_fls == NULL) {
            continue;
        }
        rebuild_road_center_by_fls(session, poss, min_fls, left);
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::tracking_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment, bool left) {
    std::vector<int64_t> variance_index;
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        if (left && boundary.curr->left == NULL) {
            continue;
        }
        if (!left && boundary.curr->right == NULL) {
            continue;
        }
        variance_index.push_back(i);
    }
    SORT(variance_index,
            [road_segment](const int64_t &l,
                const int64_t &r)->bool {
            auto &l_poss = road_segment->pos_sample_list[l];
            auto &r_poss = road_segment->pos_sample_list[r];
            return l_poss->boundary.curr_bind_dis_var < r_poss->boundary.curr_bind_dis_var;
            });
    for (int i = 0; i < variance_index.size(); ++i) {
        auto &index = variance_index[i];
        auto &poss = road_segment->pos_sample_list[index];
        session->debug_pos(poss->pos);
        tracking_road_center_by_poss(session, road_segment, index, left, true);
        tracking_road_center_by_poss(session, road_segment, index, left, false);
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::tracking_road_center_by_poss(
        RoadModelSessionData* session, RoadSegment* road_segment,
        int64_t index, bool left, bool next) {
    double dis_threshold = FLAGS_build_boundary_tracking_dis_threshold;
    double theta_threshold = FLAGS_build_boundary_tracking_max_theta;
    auto &poss = road_segment->pos_sample_list[index];
    int64_t next_index = next ? index + 1 : index - 1;
    int64_t start_index = index;
    int64_t end_index = start_index;
    bool has_next_valid = false;
    while (true) {
        if (next_index < 0 || next_index >= road_segment->pos_sample_list.size()) {
            break;
        }
        auto next_poss = road_segment->pos_sample_list[next_index];
        session->debug_pos(next_poss->pos);
        // total_length = fabs(next_poss->line_length - poss->line_length);
        // if (total_length > dis_threshold) {
        //     break;
        // }
        auto &next_boundary = next_poss->boundary;
        end_index = next_index;
        if ((left && next_boundary.curr->left != NULL)
                || (!left && next_boundary.curr->right != NULL)) {
            has_next_valid = true;
            break;
        }
        next_index = next ? next_index + 1 : next_index - 1;
    }
    auto start_fls = left ? poss->boundary.curr->left : poss->boundary.curr->right;
    auto end_poss = road_segment->pos_sample_list[end_index];
    auto end_fls = left ? end_poss->boundary.curr->left : end_poss->boundary.curr->right;
    auto dir = start_fls->dir;
    if (fabs(end_index - start_index) <= 1) {
        return fsdmap::SUCC;
    }
    if (has_next_valid) {
        dir = alg::get_dir(end_fls->pos, start_fls->pos);
    }
    
    next_index = next ? start_index + 1 : start_index - 1;
    double total_length = 0;
    while (next_index != end_index) {
        if (next_index >= road_segment->pos_sample_list.size() || next_index < 0) {
            break;
        }
        auto next_poss = road_segment->pos_sample_list[next_index];
        session->debug_pos(next_poss->pos);
        total_length = fabs(next_poss->line_length - poss->line_length);
        auto &next_boundary = next_poss->boundary;
        double min_dis = DBL_MAX;
        BoundaryFeature* min_fls = NULL;
        for (auto &fls : next_boundary.boundary_feature_list) {
            double dis = alg::calc_vertical_dis(fls->pos, start_fls->pos, dir);
            if (dis <= dis_threshold && min_dis > dis) {
                min_dis = dis;
                min_fls = fls;
            }
        }
        if (min_fls != NULL) {
            rebuild_road_center_by_fls(session, next_poss, min_fls, left);
            double theta = alg::calc_theta(dir, min_fls->dir);
            if (theta < theta_threshold) {
                dir = min_fls->dir;
            }
            // TODO dir = get_tracking_dir(session, dir, fls, total_length);
            start_fls = min_fls;
        }
        if (left) {
            next_poss->left_boundary_status = 1;
        } else {
            next_poss->right_boundary_status = 1;
        }
        next_index = next ? next_index + 1 : next_index - 1;
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::rebuild_road_center_by_fls(
        RoadModelSessionData* session, KeyPose* poss, BoundaryFeature* fls, bool left) {
    double dis_threshold = FLAGS_build_boundary_road_center_min_width;
    auto rc = session->add_ptr(session->road_center_ptr);  // 增加新的道路中心点
    auto curr = poss->boundary.curr;
    if (fls == NULL) {
        return fsdmap::FAIL;
    }
    // if ((left && alg::judge_left(fls->pos, poss->pos, poss->dir) > 0)
    //       || (!left && alg::judge_left(fls->pos, poss->pos, poss->dir) < 0)) {
    //     int a = 1;
    // }
   

    auto poss_v_dir = alg::get_vertical_dir(poss->dir);

    if (left) {
        if (curr->right != NULL) {
            double dis = alg::calc_dis(fls->pos, curr->right->pos);
            if (dis < dis_threshold) {
                return fsdmap::FAIL;
            }
        }
        auto flsr = session->add_ptr(session->boundary_feature_ptr);
        flsr->init(fls);
        flsr->src_status = fls->src_status;
        flsr->bind_dis = alg::calc_vertical_dis(fls->pos, poss->pos, poss->dir, true);
        flsr->key_pose = poss;
        if (!alg::get_cross_point(fls->pos, fls->dir,
                    poss->pos, poss_v_dir, flsr->pos)) {
            return fsdmap::FAIL;
        }

        rc->right = flsr.get();
        rc->left = curr->left;
        make_road_center(session, poss, rc.get());
        curr->left = flsr.get();
        make_road_center(session, poss, curr);
    } else {
        if (curr->left != NULL) {
            double dis = alg::calc_dis(fls->pos, curr->left->pos);
            if (dis < dis_threshold) {
                return fsdmap::FAIL;
            }
        }
        auto flsr = session->add_ptr(session->boundary_feature_ptr);// 增加新的道路边界点
        flsr->init(fls);
        flsr->src_status = fls->src_status;
        flsr->bind_dis = alg::calc_vertical_dis(fls->pos, poss->pos, poss->dir, true);
        flsr->key_pose = poss;
        //计算 pose的方向垂线与右侧边界点方向向量的交点
        if (!alg::get_cross_point(fls->pos, fls->dir,
                    poss->pos, poss_v_dir, flsr->pos)) {
            return fsdmap::FAIL;
        }
        rc->left = flsr.get();
        rc->right = curr->right;
        make_road_center(session, poss, rc.get());
        curr->right = flsr.get();
        make_road_center(session, poss, curr);
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::smooth_road_center_single(
        RoadModelSessionData* session, RoadSegment* road_segment) {
    double scope = FLAGS_build_boundary_smooth_road_center_scope;
    UMAP<KeyPose*, std::vector<KeyPose*> > cache_context_list;
    std::vector<double> road_center_dis;
    std::vector<std::pair<double, KeyPose*>> variance_index;
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        road_center_dis.clear();
        auto &vec = cache_context_list[poss];
        poss->get_context_list(scope, vec);
        for (auto &tmp_poss : vec) {
            auto &t_b = tmp_poss->boundary;
            if (t_b.curr->left == NULL || t_b.curr->right == NULL) {
                continue;
            }
            road_center_dis.push_back(t_b.curr->bind_dis);
        }
        double var = alg::calc_line_variance(road_center_dis);
        if (road_center_dis.size() < (vec.size() / 2)) {
            var += 100;
        };
        poss->boundary.curr_bind_dis_var = var;
        variance_index.push_back(std::make_pair(var, poss));
    }
    SORT(variance_index,
            [](const std::pair<double, KeyPose*> &l,
               const std::pair<double, KeyPose*> &r)->bool{
                return l.first < r.first;
            });

    for (int i = 0; i < variance_index.size(); ++i) {
        auto &pit = variance_index[i];
        auto &poss = pit.second;
        session->debug_pos(poss->pos);
        road_center_dis.clear();
        auto &boundary = poss->boundary;
        auto &context_list = cache_context_list[poss];
        for (auto &tmp_poss : context_list) {
            auto &t_b = tmp_poss->boundary;
            if (t_b.curr->left == NULL || t_b.curr->right == NULL) {
                continue;
            }
            road_center_dis.push_back(t_b.curr->bind_dis);
        }
        double median = alg::calc_median(road_center_dis);
        auto tar_pos = alg::get_vertical_pos(poss->pos, poss->dir, median);
        if (boundary.curr->in_scope(tar_pos, 0)) {
            continue;
        }
        for (auto &rc : poss->boundary.road_center_list) {
            if (!rc->in_scope(tar_pos, 0)) {
                continue;
            }
            poss->boundary.curr = rc;
            break;
        }
    }
    return fsdmap::SUCC;
}

// 遍历road_segment->pos_sample_list里的每个poss（上图的pose1）对应对应的boundary，遍历boundary.same_dir_pos（分别在每条轨迹上，与该boundary对应的轨迹点pose同向，且与该轨迹点poss垂向距离最近的位姿点）里面的每个位置点，
// 一、假如该点在pose1的右侧，即图上的pose2。如果pose2不在pose1左右侧围栏范围内，并且pose1在pose2的左右围栏内，并且dist >2.7m 。
// - 需要增加pose2对应的左侧道路边界点（pose1的右侧道路边界点boundary.curr->right）到session->boundary_feature_ptr，虽然这个新的道路边界点之前已经添加，但是他 绑定的位置是pose1，现在这个新增的位置是绑定到pose2。
// - 因为trail2轨迹的出现，多了left_boundary2，需要根据left_boundary2增加车道中心点，因为left_boundary1与left_boundary2有一些差别，用left_boundary2与right_boundary1计算出来的车道中心线有一些差别。左侧道路边界是left_boundary2，右侧道路边界是right_boundary1。
// - 因为dist >2.7m，所有右边应该是增加了一个车道，需要更新下pose2原本的车道中心curr对应的left boundary为right_boundary1，同时重新计算curr的位置。
// 二、假如该点在pose1的左侧，同理
//遍历boundary.left_oppo_pos（与该RoadBoundary对应的轨迹点pose反向，在pose左侧，且离该点最近的轨迹点）中里面的每个位置点，更新和新增道路i中心点，同上
int RoadModelProcBuildBoundary::sync_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment) {
    double dft_road_width = 3;
    // Eigen::Vector3d cross_point = {0, 0, 0};
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        auto &rc = poss->boundary.curr;
        if ((rc->left != NULL && alg::judge_left(rc->left->pos, poss->pos, poss->dir) > 0)
                || (rc->right != NULL && alg::judge_left(rc->right->pos, poss->pos, poss->dir) < 0)) {
            int a = 1;
        }
        for (auto &same_poss : boundary.same_dir_pos) {
            auto &poss2 = same_poss.src;
            auto &other_boundary = poss2->boundary;
            Eigen::Vector3d pos1 = poss->pos;
            Eigen::Vector3d pos2 = poss2->pos;
            // if (boundary.curr->left == NULL && boundary.curr->right == NULL) {
            //     pos1 = poss->pos;
            // } else {
            //     if (boundary.curr->left != NULL && boundary.curr->right == NULL) {
            //         int is_left = alg::judge_left(poss->pos, boundary.curr->left->pos, poss->dir);
            //         if (is_left > 0) {
            //             pos1 = poss->pos;
            //         } else {
            //             pos1 = alg::get_vertical_pos(boundary.curr->left->pos, poss->dir, 
            //                     dft_road_width);
            //         }
            //     } else if (boundary.curr->left == NULL && boundary.curr->right != NULL) {
            //         int is_left = alg::judge_left(poss->pos, boundary.curr->right->pos, poss->dir);
            //         if (is_left < 0) {
            //             pos1 = poss->pos;
            //         } else {
            //             pos1 = alg::get_vertical_pos(boundary.curr->right->pos, poss->dir, 
            //                     -dft_road_width);
            //         }
            //     }
            // }
            // if (other_boundary.curr->left == NULL && other_boundary.curr->right == NULL) {
            //     pos2 = poss2->pos;
            // } else {
            //     if (other_boundary.curr->left != NULL && other_boundary.curr->right == NULL) {
            //         int is_left = alg::judge_left(poss2->pos, 
            //                 other_boundary.curr->left->pos, poss2->dir);
            //         if (is_left > 0) {
            //             pos2 = same_poss.src->pos;
            //         } else {
            //             pos2 = alg::get_vertical_pos(other_boundary.curr->left->pos, poss2->dir, 
            //                     dft_road_width);
            //         }
            //     } else if (other_boundary.curr->left == NULL && other_boundary.curr->right != NULL) {
            //         int is_left = alg::judge_left(poss2->pos, 
            //                 other_boundary.curr->right->pos, poss2->dir);
            //         if (is_left < 0) {
            //             pos2 = poss2->pos;
            //         } else {
            //             pos2 = alg::get_vertical_pos(other_boundary.curr->right->pos, poss2->dir, 
            //                     -dft_road_width);
            //         }
            //     }
            // }
#if 0
            auto v_pos = alg::get_vertical_pos(poss->pos, poss->dir, 50);
            if(other_boundary.curr->left != NULL && other_boundary.curr->right != NULL){
                if (!alg::get_cross_point_for_segment(other_boundary.curr->left->pos, other_boundary.curr->right->pos,
                    poss->pos, v_pos, cross_point, 1)) {
                    continue;
                }
            }
#endif           
            auto in_scope1 = boundary.curr->in_scope(pos2); // 判断pos2点是否在当前道路中心点左右侧边界点的范围内
            auto in_scope2 = other_boundary.curr->in_scope(pos1);
            bool left = same_poss.dis < 0;
            if (!in_scope1 && in_scope2) {
                auto &complete_fls = left ? boundary.curr->left : boundary.curr->right;
                rebuild_road_center_by_fls(session, poss2, complete_fls, !left);
            } else if (in_scope1 && !in_scope2) {
                auto &complete_fls = left ? other_boundary.curr->right : other_boundary.curr->left;
                rebuild_road_center_by_fls(session, poss, complete_fls, left);
            }
        }
        if (boundary.left_oppo_pos.src != NULL) { // 反向轨迹的boundary
            auto &poss2 = boundary.left_oppo_pos.src;
            auto other_boundary = boundary.left_oppo_pos.src->boundary;
            Eigen::Vector3d pos1 = poss->pos;
            Eigen::Vector3d pos2 = poss2->pos;
            // if (boundary.curr->left == NULL && boundary.curr->right == NULL) {
            //     pos1 = poss->pos;
            // } else {
            //     if (boundary.curr->left != NULL && boundary.curr->right == NULL) {
            //         int is_left = alg::judge_left(poss->pos, boundary.curr->left->pos, poss->dir);
            //         if (is_left > 0) {
            //             pos1 = poss->pos;
            //         } else {
            //             pos1 = alg::get_vertical_pos(boundary.curr->left->pos, poss->dir, 
            //                     dft_road_width);
            //         }
            //     } else if (boundary.curr->left == NULL && boundary.curr->right != NULL) {
            //         int is_left = alg::judge_left(poss->pos, boundary.curr->right->pos, poss->dir);
            //         if (is_left < 0) {
            //             pos1 = poss->pos;
            //         } else {
            //             pos1 = alg::get_vertical_pos(boundary.curr->right->pos, poss->dir, 
            //                     -dft_road_width);
            //         }
            //     }
            // }
            // if (other_boundary.curr->left == NULL && other_boundary.curr->right == NULL) {
            //     pos2 = poss2->pos;
            // } else {
            //     if (other_boundary.curr->left != NULL && other_boundary.curr->right == NULL) {
            //         int is_left = alg::judge_left(poss2->pos, other_boundary.curr->left->pos, poss2->dir);
            //         if (is_left > 0) {
            //             pos2 = poss2->pos;
            //         } else {
            //             pos2 = alg::get_vertical_pos(other_boundary.curr->left->pos, poss2->dir, 
            //                     dft_road_width);
            //         }
            //     } else if (other_boundary.curr->left == NULL && other_boundary.curr->right != NULL) {
            //         int is_left = alg::judge_left(poss2->pos, other_boundary.curr->right->pos, poss2->dir);
            //         if (is_left < 0) {
            //             pos2 = poss2->pos;
            //         } else {
            //             pos2 = alg::get_vertical_pos(other_boundary.curr->right->pos, poss2->dir, 
            //                     -dft_road_width);
            //         }
            //     }
            // }
            auto in_scope1 = boundary.curr->in_scope(pos2);
            auto in_scope2 = other_boundary.curr->in_scope(pos1);
            if (!in_scope1 && in_scope2) {
                auto &complete_fls = boundary.curr->left;
                rebuild_road_center_by_fls(session, poss2, complete_fls, true);
            } else if (in_scope1 && !in_scope2) {
                auto &complete_fls = other_boundary.curr->left;
                rebuild_road_center_by_fls(session, poss, complete_fls, true);
            }
        }
    }
    return fsdmap::SUCC;
}

// 判断road_segment里面每个pose处对应的boundary是否存在道路中间的围栏,根据pose处是否有反向轨迹，有中间围栏的话，
// 根据左右轨迹看到的围栏就不可能是只有一个道路中心。
int RoadModelProcBuildBoundary::smooth_road_center_width_oppo_poss(
        RoadModelSessionData* session, RoadSegment* road_segment) {
    double lambda = FLAGS_build_boundary_smooth_variance_lambda;
    double median_thres = 0.9;
    std::vector<double> same_road_status(road_segment->pos_sample_list.size()); // 判断每个pose处是否存在中间围栏，有中间围栏就不可能是只有一个道路中心
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        int status = judge_same_road_center(session, poss);
        same_road_status[i] = status;
        poss->boundary.same_road_status = status;
    }
    // std::vector<double> same_road_variance(same_road_status.size());
    int prev_status = 0;
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        // double median = alg::calc_median(same_road_status, 1, 0.8, i - 12, i + 12);
        // if (median < 0.9) {
        //     poss->boundary.same_road_status = 0;
        //     continue;
        // }
        if (i == 0) {
            prev_status = same_road_status[0];
            continue;
        }
        auto &status = same_road_status[i];
        if (status == prev_status) {
            continue;
        }
        double next = alg::calc_median(same_road_status, 1, 0.8, i, i + 24); //计算i位置之后24个road_status的加权中位数
        double prev = alg::calc_median(same_road_status, 1, 0.8, i - 24, i); //计算i位置之前24个road_status的加权中位数
        if (status == 1) {
            if (next < median_thres && prev < median_thres) { // 说明当前i处的status计算有问题，应该是0
                poss->boundary.same_road_status = 0;
            }
        } else {
            if (next >= median_thres && prev >= median_thres) {
                poss->boundary.same_road_status = 1;
            }
        }
        // double var = alg::calc_line_variance(same_road_status, 0, 0, i - 12, i + 13);
        // same_road_variance[i] = var;
    }
    // double total_score = 0;
    // double total_weight = 0;
    // for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
    //     double weight = lambda / (same_road_variance[i] + lambda);
    //     total_score += same_road_status[i] * weight;
    //     total_weight += weight;
    // }
    // total_score /= total_weight;
    // bool is_same_road = total_score > 0.5;
    // road_segment->same_road_status = is_same_road ? 1 : 0;
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::predict_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment) {
    std::vector<int> pose_index;
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        if ((road_segment->same_road_status == 0) ^ (boundary.same_road_status == 0)) {
            continue;
        }
        pose_index.push_back(i);
    }
    for (auto &i : pose_index) {
        spread_road_center(session, road_segment, i, true);
        spread_road_center(session, road_segment, i, false);
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::spread_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment, int64_t index, bool prev) {
    double scope = FLAGS_build_boundary_smooth_road_center_scope;
    auto poss = road_segment->pos_sample_list[index];
    std::vector<KeyPose*> context_list;
    std::vector<double> context_list_dis;
    UMAP<KeyPose*, double> cache_dis;
    int next_index = index;
    while (true) {
        next_index = prev ? next_index - 1 : next_index + 1;
        if (next_index < 0 || next_index >= road_segment->pos_sample_list.size()) {
            break;
        }
        auto next_poss = road_segment->pos_sample_list[next_index];
        session->debug_pos(next_poss->pos);
        auto &boundary = next_poss->boundary;
        if (boundary.same_road_status == poss->boundary.same_road_status) {
            break;
        }
        context_list.clear();
        context_list_dis.clear();
        next_poss->get_context_list(scope, context_list);
        for (auto &tmp_poss : context_list) {
            if (cache_dis.find(tmp_poss) != cache_dis.end()) {
                context_list_dis.push_back(cache_dis[tmp_poss]);
                continue;
            }
            if (tmp_poss->boundary.same_road_status != poss->boundary.same_road_status) {
                continue;
            }
            context_list_dis.push_back(tmp_poss->boundary.curr->bind_dis);
        }
        double median_dis = alg::calc_median(context_list_dis);
        boundary.curr_bind_dis_gap = median_dis - boundary.curr->bind_dis;
        auto tar_pos = alg::get_vertical_pos(next_poss->pos, next_poss->dir, median_dis);
        double min_dis = DBL_MAX;
        RoadCenterFeature* min_rc = NULL;
        for (auto &rc : boundary.road_center_list) {
            if (rc->in_scope(tar_pos, 0)) {
                min_dis = rc->bind_dis;
                min_rc = rc;
                break;
            }
            double d_dis = fabs(rc->bind_dis - median_dis);
            if (min_dis > d_dis) {
                min_dis = d_dis;
                min_rc = rc;
            }
        }
        cache_dis[next_poss] = min_rc->bind_dis;
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::switch_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment) {
    for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        if (road_segment->same_road_status == boundary.same_road_status) {
            continue;
        }
        auto left_poss = boundary.left_oppo_pos.src;
        if (left_poss == NULL) {
            continue;
        }
        auto opt_poss = poss;
        if (fabs(left_poss->boundary.curr_bind_dis_gap) > fabs(boundary.curr_bind_dis_gap)) {
            opt_poss = left_poss;
        }
        double min_dis = DBL_MAX;
        RoadCenterFeature* min_rc = NULL;
        double median_dis = opt_poss->boundary.curr_bind_dis_gap + opt_poss->boundary.curr->bind_dis;
        auto tar_pos = alg::get_vertical_pos(opt_poss->pos, opt_poss->dir, median_dis);
        for (auto &rc : opt_poss->boundary.road_center_list) {
            if (rc == opt_poss->boundary.curr) {
                continue;
            }
            if (rc->in_scope(tar_pos, 0)) {
                min_dis = rc->bind_dis;
                min_rc = rc;
                break;
            }
            double d_dis = fabs(rc->bind_dis - median_dis);
            if (min_dis > d_dis) {
                min_dis = d_dis;
                min_rc = rc;
            }
        }
        if (min_rc == NULL) {
            opt_poss->filter_status = 3;
        } else {
            opt_poss->boundary.curr = min_rc;
            opt_poss->boundary.same_road_status = opt_poss->road_segment->same_road_status;
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::judge_same_road_center(RoadModelSessionData* session, 
        KeyPose* poss) {
    auto &boundary = poss->boundary;

    // if(poss->from_raw_link->link_direction == 3 || poss->from_raw_link->link_direction == 4){
    //     return 1;
    // }
    if (boundary.left_oppo_pos.src == NULL) {
        return 0;
    }
    // if (poss->inter_status > 1) {
    //     return 0;
    // }
    if (boundary.left_oppo_pos.src != NULL) {
        auto &l_curr = boundary.left_oppo_pos.src->boundary.curr;
        if (boundary.curr->in_scope(l_curr->pos, 0)) {
            return 1;
        }
    }
    // if (boundary.right_oppo_pos.src != NULL) {
    //     auto &r_curr = boundary.right_oppo_pos.src->boundary.curr;
    //     double iou = calc_road_center_iou(session, boundary.curr, r_curr);
    //     if (iou > i_threshold) {
    //         return 2;
    //     }
    // }
    return 0;
}

// double RoadModelProcBuildBoundary::calc_road_center_iou(RoadModelSessionData* session,
//         RoadCenter* l, RoadCenter* r) {
//     double inter = 0;
//     int is_left = alg::judge_left(l->pos, r->pos, l->dir);
//     if (is_left > 0) {
//         tmp = l;
//         l = r;
//         r = tmp;
//     }
//     bool oppo = true;
//     if (alg::calc_theta(l->dir, r->dir) > 90) {
//         oppo = false;
//     }
//     auto r_left = oppo ? r->right : r->left;
//     auto r_right = oppo ? r->left : r->right;
//     if (r_left == NULL && l->right == NULL) {
//         return 1;
//     }
//     double l_l = -1000;
//     double l_r = 1000;
//     if (l->left != NULL && l->right != NULL) {
//         l_l = -l->width / 2;
//         l_r = -l->width / 2;
//     }
//     double r_l = -1000;
//     double r_r = 1000;
//     if (r_left != NULL) {
//         r_l = alg::calc_vertical_dis(r_left->pos, l->pos, l->dir, true);
//     }
//     if (r_right != NULL) {
//         r_r = alg::calc_vertical_dis(r_right->pos, l->pos, l->dir, true);
//     }
//     double l_dis = std::max(r_l, l_l);
//     double r_dis = std::min(r_r, l_r);
//     if (l_dis > r_dis) {
//         return 0;
//     }
//     double inter = r_dis - l_dis;
//     double iou = inter * 2 / ((l_r - l_l) + (r_r - r_l));
//     return iou;
// }

// 根据轨迹左右两侧的Boundary位置来推算车道中心线，
// 当poss轨迹点左右两侧都有boundary时，车道中心点在当前轨迹点poss两侧左右围栏上两个点的平均值，
// 当只有左侧围栏点时，左侧围栏点在poss->dir垂线方向上向右延伸2m就是车道中心点，
// 当只有有的围栏点时，右侧围栏点在poss->dir垂线方向上向左延伸2m就是车道中心点
int RoadModelProcBuildBoundary::build_road_center_by_poss(RoadModelSessionData* session, 
        KeyPose* poss) {
    double min_width = FLAGS_build_boundary_road_center_min_width; // 2.7，最窄的车道宽度
    BoundaryFeature* prev_fls = NULL;
    auto &boundary = poss->boundary;
    std::vector<BoundaryFeature*> valid_list;
    for (int i = 0; i < boundary.boundary_feature_list.size(); ++i) {
        auto &fls = boundary.boundary_feature_list[i];
        if (FLAGS_build_boundary_use_sem_enable 
                && fls->src_status != 2 
                && fls->src_status != 4) {
            continue;
        }
        double width = 0;
        if (prev_fls != NULL) {
            width = fls->bind_dis - prev_fls->bind_dis;
            if (width < min_width) {
                prev_fls = fls;
                if(i == boundary.boundary_feature_list.size() - 1) {
                    valid_list.push_back(fls);
                }
                continue;
            }
        }
        valid_list.push_back(fls);

        auto rc = session->add_ptr(session->road_center_ptr);
        rc->left = prev_fls; // 车道中心点的左侧围栏点
        rc->right = fls;
        rc->width = width;
        make_road_center(session, poss, rc.get()); // 计算车道中心点位置
        prev_fls = fls;
    }
    if (valid_list.size() > 0) {
        auto rc = session->add_ptr(session->road_center_ptr);
        rc->left = valid_list.back();
        rc->right = NULL;
        make_road_center(session, poss, rc.get());
    } else {
        auto rc = session->add_ptr(session->road_center_ptr);
        rc->left = NULL;
        rc->right = NULL;
        make_road_center(session, poss, rc.get());
    }
    return fsdmap::SUCC;
}

//int RoadModelProcBuildBoundary::mark_dumplicate_boundary(RoadModelSessionData* session, KeyPose* poss) {
//    
//    return fsdmap::SUCC;
//}
// 遍历poss->boundary中road_center_list中的车道中心点，选出左右boundary与自车位置间距最小的的车道中心点作为
// 该poss->boundary的车道中心点curr
int RoadModelProcBuildBoundary::mark_curr_road_center(RoadModelSessionData* session, KeyPose* poss) {
    double dis_threshold = FLAGS_build_boundary_rc_scope_dis_threshold;
    auto &boundary = poss->boundary;
    double min_dis = DBL_MAX;;
    RoadCenterFeature* min_rc = NULL;
    double offset = 0;
    for (auto &rc : boundary.road_center_list) {
        if (rc->left != NULL){
            double dis = alg::calc_dis(poss->pos, rc->left->pos);
            if (min_dis > dis) {
                min_rc = rc;
                min_dis = dis;
            }
        }
        if (rc->right != NULL){
            double dis = alg::calc_dis(poss->pos, rc->right->pos);
            if (min_dis > dis) {
                min_rc = rc;
                min_dis = dis;
            }
        }
    }
    // if (min_rc != NULL && min_dis < dis_threshold) {
    //     if (boundary.left_oppo_pos.src != NULL) {
    //         offset = 3;
    //     } else {
    //         offset = 2;
    //     }
    // }
    min_dis = DBL_MAX;
    auto right_pos = alg::get_vertical_pos(poss->pos, poss->dir, offset);
    for (auto &rc : boundary.road_center_list) {
        if (rc->in_scope(right_pos, 0)) {   // 如果poss->pos在rc的左右道路边界点的范围内，则直接使用rc作为curr，否则继续遍历，理由是：优先考虑link在道路范围内的道路作为当前道路，否则使用距离最近的道路作为当前道路，避免link靠近道路边界位置，如其左侧道路的没有left_pos,则会以其本身的right_pos垂线方向向右延伸2m作为当前道路中心点，导致对应的boundary.curr的设置有误
            boundary.curr = rc;
            break;
        }
        if (rc->left != NULL){
            double dis = alg::calc_dis(right_pos, rc->left->pos);
            if (min_dis > dis) {
                min_rc = rc;
                min_dis = dis;
            }
        }
        if (rc->right != NULL){
            double dis = alg::calc_dis(right_pos, rc->right->pos);
            if (min_dis > dis) {
                min_rc = rc;
                min_dis = dis;
            }
        }
    }
    if (boundary.curr == NULL) {
        if (min_rc != NULL) {
            boundary.curr = min_rc;
        } else {
            boundary.curr = boundary.road_center_list[0];   // 当前pos 对应的最左出的rc作为curr， boundary.road_center_list已经是从左到右的排序了
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::make_road_center(RoadModelSessionData* session, 
        KeyPose* poss, RoadCenterFeature* rc) {
    auto &boundary = poss->boundary;
    if (rc != boundary.curr) {
        boundary.road_center_list.push_back(rc);  // rc本身在存储过程中已经是从左到右的排序，所以road_center_list的存储顺序也是从左到右的
    }
    rc->key_pose = poss;
    if (rc->left != NULL && rc->right != NULL) {
        // rc->width = fabs(fls->bind_dis - prev_fls->bind_fls);
        rc->theta = alg::calc_theta(rc->left->dir, rc->right->dir);
        rc->pos = (rc->left->pos + rc->right->pos) / 2; // 当前轨迹点poss两侧左右围栏上两个点的平均值
        rc->dir = poss->dir;
    } else if (rc->left != NULL) { // 只有左侧围栏点
        rc->pos = alg::get_vertical_pos(rc->left->pos, poss->dir, 2); // 左侧围栏点在poss->dir垂线方向上向右延伸2m就是车道中心点
        rc->dir = poss->dir;
    } else if (rc->right != NULL) { // 只有右侧围栏点
        rc->pos = alg::get_vertical_pos(rc->right->pos, poss->dir, -2); // 右侧围栏点在poss->dir垂线方向上向左延伸2m就是车道中心点
        rc->dir = poss->dir;
    } else { // 左右两边都没有围栏点，就用当前poss的位置作为车道中心点
        rc->pos = poss->pos;
        rc->dir = poss->dir;
    }
    rc->bind_dis = alg::calc_vertical_dis(rc->pos, poss->pos, poss->dir, true);
    return fsdmap::SUCC;
}

// 在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它50范围内的，朝向角度差在60度范围内的boundary点，
// 遍历这些筛选出的boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
// 这些cross point会放入poss->boundary
int RoadModelProcBuildBoundary::search_nearest_boundary(RoadModelSessionData* session, KeyPose* poss) {
    double max_same_dir_theta = FLAGS_build_boundary_max_same_pos_cb_theta;
    float radius = FLAGS_build_boundary_max_same_dir_feature_radius;
    auto pos = poss->pos;
    std::vector<BoundaryFeature*> search_sec;
    session->boundary_line_sample_tree.search(pos, radius, search_sec);
    Eigen::Vector3d dir_get;
    auto &cb = poss->boundary;
    DisDirPoint cross_point;
    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, radius, true);
    for (auto& fls : search_sec) {
        // if (FLAGS_build_boundary_use_sem_enable ^ (fls->src_status == 2)) {
        //     continue;
        // }
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->next == NULL) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos);
        if (dis > radius) {
            continue;
        }

        // 无论角度如何，都是护栏
        double theta = alg::calc_theta(fls->dir, poss->dir);
        if (theta > max_same_dir_theta && 180 - theta > max_same_dir_theta) {
            continue;
        }

        if (!alg::get_cross_point_for_segment(fls->pos, fls->next->pos,
                    poss->pos, poss_v_pt, cross_point.pos, 1)) {
            continue;
        }
        double c_dis = alg::calc_dis(poss->pos, cross_point.pos);
        if (alg::judge_left(cross_point.pos, pos, poss->dir) < 0) { // 判断cross_point.pos是否在pos的左侧
            c_dis = -c_dis;
        }
        auto flsr = session->add_ptr(session->boundary_feature_ptr, true);
        flsr->init(fls);
        flsr->bind_dis = c_dis;
        flsr->pos = cross_point.pos;
        flsr->key_pose = poss;
        flsr->attr_feature = fls;
        if (theta > max_same_dir_theta) {
            flsr->dir = -flsr->dir;
        }
        cb.boundary_feature_list.push_back(flsr.get());
    }
    SORT(cb.boundary_feature_list, 
            [](const BoundaryFeature* l, const BoundaryFeature* r)->bool{
                return l->bind_dis < r->bind_dis;
            });
    return fsdmap::SUCC;
}

// 在poss附近半径30m的范围内，筛选出与其朝向角度在30度范围内的车道线点
int RoadModelProcBuildBoundary::search_nearest_fls(RoadModelSessionData* session,
        KeyPose* poss, std::vector<LaneLineSample*> &secs) {
    double radius = FLAGS_build_boundary_pos_match_lane_radius;  // 30
    double z_max = FLAGS_build_boundary_pos_match_lane_z_max;  // 6
    double z_min = FLAGS_build_boundary_pos_match_lane_z_min;  // -2
    double theta_threshold = FLAGS_build_boundary_pos_match_lane_theta; // 30
    Eigen::Vector3d pos = poss->pos;
    std::vector<LaneLineSample*> search_rets;
    session->lane_line_sample_tree.search(pos, radius, search_rets);
    std::vector<std::pair<LaneLineSample*, double>> tmp_ret;
    auto &boundary = poss->boundary;
    Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(poss->pos, poss->dir, 2, true);
    DisDirPoint cross_point;
    for (auto& fls : search_rets) {
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->invalid()) {
            continue;
        }
        if (fls->next == NULL) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos, true);
        if (dis > radius) {
            continue;
        }
        // double z_dis = pos.z() - fls->pos.z();
        // if (z_dis > z_max || z_dis < z_min) {
        //     continue;
        // }
        double theta = alg::calc_theta(fls->dir, poss->dir, true);
        if (theta >= theta_threshold) {
            continue;
        }
        secs.push_back(fls);
    }
    return fsdmap::SUCC;
}



BoundaryFeature* RoadModelProcBuildBoundary::search_nearest_boundary_lidar(
        RoadModelSessionData* session, BoundaryFeature* bf, double v_scope, double h_scope) {
    float radius = FLAGS_build_boundary_search_poss_search_radius;
    Eigen::Vector3d pos = bf->pos;
    std::vector<BoundaryFeature*> search_rets;
    session->boundary_lidar_tree.search(pos, radius, search_rets);
    for (auto& fls : search_rets) {
        session->debug_pos2(pos, fls->pos);
        if (fls->invalid()) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos, true);
        if (dis > radius) {
            continue;
        }
        double v_dis = alg::calc_vertical_dis(fls->pos, pos, bf->dir);
        if (v_dis > v_scope) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, pos, bf->dir);
        if (h_dis > h_scope) {
            continue;
        }
        return fls;
    }
    return NULL;
}

// 遍历road_segment->pos_sample_list里面每个poss，筛选出poss附近的黄线，同时根据该boundary的反向轨迹，按反向轨迹点往poss->dir的垂线方向平移1m构造新的黄色车道线点。
// 遍历poss的boundary里面的的道路中心点rc，选出rc处离道路中心距离最近的黄色车道线点。
// 遍历road_segment->pos_sample_list里面每个poss，把road_segment->pos_sample_list[index]处的boundary.curr->yellow_boundary，放入其前后20米范围内的pose的boundary.curr->yellow_score_list里面，相当于扩散黄线车道线点。
int RoadModelProcBuildBoundary::build_yellow_boundary(RoadModelSessionData* session) {
    // 计算置信分数，由高到低传播
    for (auto &road_segment : session->road_segment_list) {
        std::vector<int64_t> index_list(road_segment->pos_sample_list.size());
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            auto &boundary = poss->boundary;
            index_list[i] = i;
            match_yellow_boundary(session, poss); // 选出poss附近的离道路中心点最近的道路中间的黄色车道线点
        }
        // 对road_segment->pos_sample_list里面的pose处对应的boundary最窄处的道路中心
        // 对应的黄色车道中线按是否是黄线的概率按从大到小排序
        SORT(index_list, 
                [road_segment](const int64_t &l, const int64_t &r)->bool{
                    auto &l_poss = road_segment->pos_sample_list[l];
                    auto &r_poss = road_segment->pos_sample_list[r];
                    auto &l_score = l_poss->boundary.curr->yellow_boundary;
                    auto &r_score = r_poss->boundary.curr->yellow_boundary;
                    if (l_score.src != NULL && r_score.src != NULL) {
                        return l_score.score > r_score.score;
                    } else if (l_score.src != NULL) {
                        return true;
                    }
                    return false;
                });
        
        for (auto &index : index_list) {
            auto &poss =  road_segment->pos_sample_list[index];
            if (poss->boundary.curr->yellow_boundary.src == NULL) {
                continue;
            }
            poss->boundary.curr->yellow_score_list.push_back(
                    poss->boundary.curr->yellow_boundary);
            spread_yellow_boundary(session, road_segment, index, true);
            spread_yellow_boundary(session, road_segment, index, false);
        }
        // 加权计算
        for (auto &index : index_list) {
            auto &poss =  road_segment->pos_sample_list[index];
            session->debug_pos(poss->pos);
            mark_yellow_boundary(session, poss);
        }
    }
    return fsdmap::SUCC;
}

// 筛选出poss附近的黄线，同时根据该boundary的反向轨迹，按反向轨迹点往poss->dir的垂线方向平移1m构造新的黄色车道线点
int RoadModelProcBuildBoundary::match_yellow_boundary(RoadModelSessionData* session,
        KeyPose* poss) {
    double min_width_threshold = FLAGS_build_boundary_mark_yellow_min_width_threshold; // 3
    // 有对象link的，不用识别黄线
    auto &boundary = poss->boundary;
    // 有对向行驶路线，但是有中间围栏，所以道路中间没有黄线分隔，不用处理后续逻辑
    if (boundary.left_oppo_pos.src != NULL && boundary.same_road_status != 1) {
        return fsdmap::SUCC;
    }
    for (auto &fls : poss->lane_line_feature_sample_list) {
        double score = 1;
        if (fls->src->attr.is_double_line == 1) {
            if (fls->attr.color != 3) {  // 3表示黄色线
                score *= 0.9;  // 双线，但不是黄色，降低概率
            }
            boundary.yellow_feature_list.push_back({fls, fls->bind_dis, score});
        } else if (fls->attr.color == 3 && !fls->attr.is_bus) {
            score *= 0.6;
            boundary.yellow_feature_list.push_back({fls, fls->bind_dis, score});
        //  } else if (boundary.left_oppo_pos.src != NULL) {
        //      auto &oppo_rc = boundary.left_oppo_pos.src->boundary.curr;
        //      if (is_same_road_center(session, oppo_rc, ))
        }
    }
    // 根据该boundary的反向轨迹，按反向轨迹点往poss->dir的垂线方向平移1m构造新的黄色车道线点
    if (FLAGS_build_boundary_use_trail_enable && boundary.left_oppo_trail.src != NULL) {
        auto new_fls = session->add_ptr(session->lane_line_sample_ptr);
        auto &oppo_poss = boundary.left_oppo_trail.src;
        new_fls->pos = alg::get_vertical_pos(oppo_poss->pos, poss->dir, 1);
        new_fls->dir = -oppo_poss->dir;
        new_fls->key_pose = poss;
        new_fls->bind_dis = boundary.left_oppo_trail.dis + 1;
        boundary.yellow_feature_list.push_back(
                {new_fls.get(), new_fls->bind_dis, 0.8});
    }

    for (auto &rc : boundary.road_center_list) {
        match_yellow_boundary_by_rc(session, poss, rc); // 选出rc处离道路中心距离最近的黄色车道线点
    }
    return fsdmap::SUCC;
}

// 选出rc处离道路中心距离最近的黄色车道线点
int RoadModelProcBuildBoundary::match_yellow_boundary_by_rc(RoadModelSessionData* session,
        KeyPose* poss, RoadCenterFeature* rc) {
    double middle_dis = get_middle_dis(session, poss, rc);
    double road_width_threshold = FLAGS_build_boundary_mark_yellow_road_width_threshold; // 7
    double min_width_threshold = FLAGS_build_boundary_mark_yellow_min_width_threshold; // 3
    auto &boundary = poss->boundary;
    if (rc->left != NULL && rc->right != NULL) {
        // double width = fabs(rc->left->bind_dis - rc->right->bind_dis);
        if (rc->width < road_width_threshold) {  // 道路宽度不足两个车道宽度，不可能道路中间有黄线
            return fsdmap::SUCC;
        }
    }
    
    double min_dis = DBL_MAX;
    BoundaryParamPair<LaneLineSample> min_fls = {NULL, 0, 0};
    for (auto &flsp : boundary.yellow_feature_list) {
        if (!rc->in_scope(flsp.src->pos, 0)) { // 车道线的位置在道路边界的范围内
            continue;
        }
        // if (flsp.src->src != NULL && flsp.src->src->src_status != 2) {
        //     continue;
        // }
        // 计算左侧道路边界与道路中间黄线之间的距离
        if (rc->left != NULL) {
            double width = fabs(rc->left->bind_dis - flsp.src->bind_dis);
            if (width < min_width_threshold) { //少于一个车道宽度
                continue;
            }
        }
        // 计算右侧道路边界与道路中间黄线之间的距离
        if (rc->right != NULL) {
            double width = fabs(rc->right->bind_dis - flsp.src->bind_dis);
            if (width < min_width_threshold) {
                continue;
            }
        }
        double dis = fabs(flsp.src->bind_dis - middle_dis);
        if (dis < min_dis) {  // 选出离道路中心距离最近的黄色车道线点
            min_dis = dis;
            min_fls = flsp;
        }
    }
    // if (min_fls.src == NULL) {
    //     for (auto &flsp : boundary.yellow_feature_list) {
    //         if (!rc->in_scope(flsp.src->pos, 0)) {
    //             continue;
    //         }
    //         double dis = fabs(flsp.src->bind_dis - middle_dis);
    //         if (dis < min_dis) {
    //             min_dis = dis;
    //             min_fls = flsp;
    //         }
    //     }
    // }
    if (min_fls.src != NULL) {
        if(!judge_road_center_with_lane_ratio(session, poss, min_fls.src->pos))
        {
            rc->yellow_boundary = min_fls;
        }
    }
    return fsdmap::SUCC;
}

// 根据左右侧道路边界以及车道线边界计算道路中心点距离自车的位置
double RoadModelProcBuildBoundary::get_middle_dis(RoadModelSessionData* session,
        KeyPose* poss, RoadCenterFeature* rc) {
    double middle_dis = 0;
    if (rc->left != NULL && rc->right != NULL) {  // 左右侧边界都存在
        middle_dis = (rc->left->bind_dis + rc->right->bind_dis) / 2;
    } else {
        double left_dis = 0;
        double right_dis = 0;
        bool has_valid = false;
        // 选出道路边界范围内离当前位置最远的左右侧车道线距离
        for (auto &fls : poss->lane_line_feature_sample_list) {
            if (!rc->in_scope(fls->pos, 0.5)) { //判断车道线上的点是否在道路边界范围内
                continue;
            }
            if (!has_valid) {
                left_dis = fls->bind_dis;
                right_dis = fls->bind_dis;
                has_valid = true;
            } else {
                left_dis = std::min(left_dis, fls->bind_dis);
                right_dis = std::max(right_dis, fls->bind_dis);
            }
        }
        if (!has_valid) { // 没有在道路边界范围内的车道线点
            if (rc->left != NULL) {  // 左侧道路边界存在，加一个车道宽度，右平移一个车道的距离
                middle_dis = rc->left->bind_dis + 3.5;
            }
            if (rc->right != NULL) { // 右侧道路边界存在，减一个车道宽度，左移一个车道的距离
                middle_dis = rc->right->bind_dis - 3.5;
            }
        } else {  // 一侧道路边界与另一侧最边界车道线到自车距离的平均值
            if (rc->left != NULL) {
                left_dis = rc->left->bind_dis;
            }
            if (rc->right != NULL) {
                right_dis = rc->right->bind_dis;
            }
            middle_dis = (left_dis + right_dis) / 2;
        }
    }
    return middle_dis;
}

int RoadModelProcBuildBoundary::mark_yellow_boundary(RoadModelSessionData* session,
        KeyPose* poss) {
    double dis_threshold = FLAGS_build_boundary_mark_yellow_dis_threshold;  //1

    double total_bind_dis = 0;
    double total_score = 0;
    auto &rc = poss->boundary.curr;
    if (rc->yellow_score_list.size() == 0) {
        return fsdmap::SUCC;
    }
    
    // 计算加权平均距离
    for (auto &pair : rc->yellow_score_list) {
        total_score += pair.score;
        total_bind_dis += pair.dis * pair.score;
    }
    total_bind_dis /= total_score;
    
    rc->yellow_boundary.dis = total_bind_dis;
    double min_dis = DBL_MAX;
    LaneLineSample* min_fls = NULL;
    // 选出原始yellow_feature_list中距离加权平均距离最近的黄线点
    for (auto &flsp : poss->boundary.yellow_feature_list) {
        double dis = fabs(flsp.src->bind_dis - total_bind_dis);
        if (dis < min_dis && dis < dis_threshold) {
            min_dis = dis;
            min_fls = flsp.src;
        }
    }
    if (min_fls != NULL) {
        rc->yellow_boundary.src = min_fls;
        rc->yellow_boundary.dis = total_bind_dis;
    } else {  // poss->boundary.yellow_feature_list没有与加权平均距离在1m范围内的点，那就在普通的线里面找
        for (auto &fls : poss->lane_line_feature_sample_list) {
            double dis = fabs(fls->bind_dis - total_bind_dis);
            if (dis < min_dis && dis < dis_threshold) {
                min_dis = dis;
                min_fls = fls;
            }
        }
        if (min_fls != NULL) {
            rc->yellow_boundary.src = min_fls;
        } else {  // 在普通的线里面也没有找到，新建一个黄线点
            auto new_fls = session->add_ptr(session->lane_line_sample_ptr);
            new_fls->pos = alg::get_vertical_pos(poss->pos, poss->dir, total_bind_dis);
            new_fls->dir = poss->dir;
            new_fls->bind_dis = total_bind_dis;
            rc->yellow_boundary.src = new_fls.get();
        }
    }
    return fsdmap::SUCC;
}

// 把road_segment->pos_sample_list[index]处的boundary.curr->yellow_boundary.
// 放入其前后20米范围内的pose的boundary.curr->yellow_score_list里面
int RoadModelProcBuildBoundary::spread_yellow_boundary(RoadModelSessionData* session,
        RoadSegment* road_segment, int index, bool prev) {
    double spread_length = FLAGS_build_boudnary_yellow_spread_length; //20
    double spread_gap_threshold = FLAGS_build_boudnary_yellow_spread_gap_threshold; //1
    auto &start_poss = road_segment->pos_sample_list[index];
    int curr_index = index;
    double total_dis = 0;
    double curr_bind_dis = start_poss->boundary.curr->yellow_boundary.src->bind_dis;
    auto &start_score = start_poss->boundary.curr->yellow_boundary.score;
    double speed = 0;
    // 把自己的放进去
    while (true) {
        int64_t next_index = prev ? curr_index - 1 : curr_index + 1;
        if (next_index < 0 || next_index >= road_segment->pos_sample_list.size()) {
            break;
        }
        auto &curr_poss = road_segment->pos_sample_list[curr_index];
        auto &next_poss = road_segment->pos_sample_list[next_index];
        if (next_poss->boundary.curr->yellow_boundary.src == NULL) {
            break;
        }
        total_dis += alg::calc_dis(next_poss->pos, curr_poss->pos);
        if (total_dis > spread_length) {
            break;
        }

        auto &next_bind_dis = next_poss->boundary.curr->yellow_boundary.src->bind_dis;
        auto predict_dis = curr_bind_dis + speed;
        double score = (1 - total_dis / spread_length) * start_score;// 离start_poss越远分数越低
        if (fabs(next_bind_dis - predict_dis) < spread_gap_threshold) {
            predict_dis = next_bind_dis;
            speed = next_bind_dis - curr_bind_dis;
            curr_bind_dis = next_bind_dis;
        }
        next_poss->boundary.curr->yellow_score_list.push_back(
                {start_poss->boundary.curr->yellow_boundary.src, predict_dis, score});
        curr_index = next_index;
    }
    return fsdmap::SUCC;
}

// 在road_segment_link_tree半径50m范围内的点，与poss反向，在该点左侧，且离该点最近的轨迹点
// 在road_segment_link_tree半径50m范围内的点，与poss同向，在该点右侧，每条轨迹上离该点最近的点
int RoadModelProcBuildBoundary::search_opposite_poss(RoadModelSessionData* session, 
        KeyPose* poss) {
    double max_same_dir_theta = FLAGS_build_boundary_opposite_pos_same_dir_theta; // 30
    float radius = FLAGS_build_boundary_max_same_dir_feature_radius;  // 50
    float scope_dis_threshold = FLAGS_build_boundary_opposite_pos_scope_threshold;  // 3
    auto pos = poss->pos;
    std::vector<KeyPose*> secs;
    session->road_segment_link_tree.search(pos, radius, secs);
    auto &cb = poss->boundary;
    cb.left_oppo_pos.dis = DBL_MAX;
    UMAP<std::string, BoundaryParamPair<KeyPose>> right_map;
    UMAP<std::string, BoundaryParamPair<KeyPose>> oppo_map;
    for (auto& n_pos : secs) {
        if (n_pos->road_segment == NULL) {
            continue;
        }
        if (n_pos->road_segment == poss->road_segment) {
            continue;
        }
        double theta = alg::calc_theta(poss->dir, n_pos->dir);
        bool oppo = true;

        // if(n_pos->from_raw_link->form == "25" && (theta >=0 && theta < 90))
        // {
        //     oppo = false;
        // }
        // else{
        //     if (theta > max_same_dir_theta && theta < 180 - max_same_dir_theta) {
        //         continue;
        //     } else if (theta <= max_same_dir_theta) {
        //         oppo = false;
        //     }
        // }

        if (theta > max_same_dir_theta && theta < 180 - max_same_dir_theta) {
            continue;
        } else if (theta <= max_same_dir_theta) {
            oppo = false;
        }

        double scope_dis = alg::calc_hori_dis(n_pos->pos, pos, poss->dir); // 计算点n_pos->pos到直线poss->dir的垂线的距离
        if (scope_dis > scope_dis_threshold) {
            continue;
        }
        // double dis = alg::calc_vertical_dis(n_pos->pos, poss->pos, poss->dir, true);
        double fdis = fabs(scope_dis);
        if (oppo && alg::judge_left(n_pos->pos, pos, poss->dir) < 0) { // 反向，并且n_pos是否在pos的左侧
            // if (fabs(cb.left_oppo_pos.dis) > fdis) {
            //     cb.left_oppo_pos.dis = dis;
            //     cb.left_oppo_pos.src = n_pos;
            // }
            if (MAP_NOT_FIND(oppo_map, n_pos->line_id) 
                    || fabs(oppo_map[n_pos->line_id].dis) > fdis) { // 筛选出每条轨迹上离poss点纵向距离最近的点
                oppo_map[n_pos->line_id].dis = scope_dis;
                oppo_map[n_pos->line_id].src = n_pos;
            }
        } else if (!oppo) {
            if (MAP_NOT_FIND(right_map, n_pos->line_id) 
                    || fabs(right_map[n_pos->line_id].dis) > fdis) {
                right_map[n_pos->line_id].dis = scope_dis;
                right_map[n_pos->line_id].src = n_pos;
            }
        }
    }
    // if (cb.left_oppo_pos.src != NULL) {
    //     cb.left_oppo_pos.dis = alg::calc_vertical_dis(cb.left_oppo_pos.src->pos, 
    //             poss->pos, poss->dir);
    // }
    
    for (auto &lit : oppo_map) {
        double dis = alg::calc_vertical_dis(lit.second.src->pos, poss->pos, poss->dir, true);
        if (fabs(cb.left_oppo_pos.dis) > fabs(dis)) {
            cb.left_oppo_pos = lit.second;
            cb.left_oppo_pos.dis = dis;
        }
    }
    // if(oppo_map.size() == 0 && (poss->from_raw_link->link_direction == 3 || poss->from_raw_link->link_direction == 4)){
    //     cb.left_oppo_pos.src = poss;
    //     cb.left_oppo_pos.dis = 0;
    // }
    for (auto &rit : right_map) {
        rit.second.dis = alg::calc_vertical_dis(rit.second.src->pos, poss->pos, poss->dir, true);
        cb.same_dir_pos.emplace_back(rit.second);
    }
    return fsdmap::SUCC;
}

// 在session->key_pose_tree半径50m范围内的点，与poss反向，在该点左侧，且离该点最近的轨迹点
int RoadModelProcBuildBoundary::search_opposite_trail(RoadModelSessionData* session, 
        KeyPose* poss) {
    double max_same_dir_theta = FLAGS_build_boundary_opposite_pos_same_dir_theta;  // 30
    float radius = FLAGS_build_boundary_max_same_dir_feature_radius;  // 50
    float scope_dis_threshold = FLAGS_build_boundary_opposite_pos_scope_threshold;  // 3
    auto pos = poss->pos;
    std::vector<KeyPose*> secs;
    session->key_pose_tree.search(pos, radius, secs);
    auto &cb = poss->boundary;
    cb.left_oppo_trail.dis = DBL_MAX;
    UMAP<std::string, BoundaryParamPair<KeyPose>> right_map;
    UMAP<std::string, BoundaryParamPair<KeyPose>> oppo_map;
    for (auto& n_pos : secs) {
        double theta = alg::calc_theta(poss->dir, n_pos->dir);
        if (theta < 180 - max_same_dir_theta) {
            continue;
        }
        double scope_dis = alg::calc_hori_dis(n_pos->pos, pos, poss->dir);
        if (scope_dis > scope_dis_threshold) {
            continue;
        }
        // double dis = alg::calc_vertical_dis(n_pos->pos, poss->pos, poss->dir, true);
        double fdis = fabs(scope_dis);
        if (alg::judge_left(n_pos->pos, pos, poss->dir) < 0) {
            if (MAP_NOT_FIND(oppo_map, n_pos->line_id) 
                    || fabs(oppo_map[n_pos->line_id].dis) > fdis) {
                oppo_map[n_pos->line_id].dis = scope_dis;
                oppo_map[n_pos->line_id].src = n_pos;
            }
        } else {
            if (MAP_NOT_FIND(right_map, n_pos->line_id) 
                    || fabs(right_map[n_pos->line_id].dis) > fdis) {
                right_map[n_pos->line_id].dis = scope_dis;
                right_map[n_pos->line_id].src = n_pos;
            }
        }
    }
    
    for (auto &lit : oppo_map) {
        double dis = alg::calc_vertical_dis(lit.second.src->pos, poss->pos, poss->dir, true);
        if (fabs(cb.left_oppo_trail.dis) > fabs(dis)) {
            cb.left_oppo_trail = lit.second;
            cb.left_oppo_trail.dis = dis;
        }
    }
    // for (auto &rit : right_map) {
    //     rit.second.dis = alg::calc_vertical_dis(rit.second.src->pos, poss->pos, poss->dir, true);
    //     cb.same_dir_pos.emplace_back(rit.second);
    // }
    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::judge_road_center_with_lane_ratio(RoadModelSessionData* session, KeyPose* poss, const Eigen::Vector3d &pos){

    double width_threshold = FLAGS_build_boundary_mark_yellow_road_width_threshold / 2 ; // 3.5
    double ratio_width_threshold = FLAGS_build_boundary_mark_yellow_road_width_threshold / 2;
    double left_dis = 0, right_dis = 0;
    auto &boundary = poss->boundary;
    int current_lane_num = -1, oppo_lane_num = -1;
    bool ret = false;

    add_same_dir_lane_num(session, poss,  current_lane_num, oppo_lane_num);

    auto& rc = boundary.curr;

    if(rc->left != NULL && rc->right != NULL){
        // if(boundary.left_oppo_pos.src == NULL && poss->from_raw_link->link_direction != 4)
        // {
        //     int total_lane_num = static_cast<int>((rc->width + 0.5) / ratio_width_threshold);
        //     oppo_lane_num = total_lane_num - current_lane_num;
        // }
        ratio_width_threshold = rc->width / (oppo_lane_num + current_lane_num);
        left_dis = alg::calc_vertical_dis(pos, rc->left->pos, poss->dir);
        right_dis = alg::calc_vertical_dis(pos, rc->right->pos, poss->dir);
        
        if((left_dis - oppo_lane_num * ratio_width_threshold) > -width_threshold && (right_dis - current_lane_num * ratio_width_threshold) > -width_threshold){
            ret = true;
        }
    }
    else if(rc->left != NULL && rc->right == NULL && boundary.left_oppo_pos.src != NULL){
        // ratio_width_threshold = rc->left->bind_dis / oppo_lane_num;
        left_dis = alg::calc_vertical_dis(pos, rc->left->pos, poss->dir); 
        if((fabs(left_dis - oppo_lane_num * ratio_width_threshold) < width_threshold)){
            ret = true;
        }
    }
    else if(rc->left == NULL && rc->right != NULL){
        // ratio_width_threshold = rc->right->bind_dis / current_lane_num;
        right_dis = alg::calc_vertical_dis(pos, rc->right->pos, poss->dir);
        if(fabs(right_dis - current_lane_num * ratio_width_threshold) < width_threshold){
            ret = true;
        }
    }

    if(ret){
        return fsdmap::SUCC;
    }
    return fsdmap::FAIL;
}

int RoadModelProcBuildBoundary::set_road_index(RoadModelSessionData* session, KeyPose* poss) {

    auto &boundary = poss->boundary;
    session->debug_pos(poss->pos);
    int curr_index = 0;
    SORT(boundary.road_center_list,
            [](const RoadCenterFeature *l,
                const RoadCenterFeature *r)->bool {
                return l->bind_dis < r->bind_dis;
            });
    // 处理每个道路中心特征，建立左右关联
    for (int i = 0; i < boundary.road_center_list.size(); ++i) {
        auto &rc = boundary.road_center_list[i];
        if (rc == boundary.curr) {
            curr_index = i;
        }
        if (i == 0) {
            continue;
        }
        auto &prev_rc = boundary.road_center_list[i - 1];
        prev_rc->fill_right = rc;
        rc->fill_left = prev_rc;
    }
    // 为非当前道路中心特征分配索引
    for (int i = 0; i < boundary.road_center_list.size(); ++i) {
        auto &rc = boundary.road_center_list[i];
        if (rc != boundary.curr) {
            rc->index = i - curr_index;
        }
    }

    return fsdmap::SUCC;
}

int RoadModelProcBuildBoundary::add_same_dir_lane_num(RoadModelSessionData* session, KeyPose* poss,  int& current_num, int& oppo_num){
    double ratio_threshold = FLAGS_build_boundary_mark_yellow_road_width_threshold / 2;
    auto &boundary = poss->boundary;
    auto& rc = boundary.curr;

    int current_lane_num = 0, oppo_lane_num = 0;

    if(boundary.left_oppo_pos.src != NULL){
        for(auto& same_dir_link_poss : boundary.left_oppo_pos.src->boundary.same_dir_pos){
            if(boundary.curr->in_scope(same_dir_link_poss.src->pos, 0)){
                oppo_lane_num += same_dir_link_poss.src->from_raw_link->lanenum_sum;
            }
        }
    }

    for(auto& same_dir_link_poss : boundary.same_dir_pos){
        if(boundary.curr->in_scope(same_dir_link_poss.src->pos, 0)){
            current_lane_num += same_dir_link_poss.src->from_raw_link->lanenum_sum;
        }
    }
    current_lane_num += poss->from_raw_link->lanenum_sum;

    // if(poss->from_raw_link->link_direction == 3 || poss->from_raw_link->link_direction == 4){
    //     int total_lane_num = poss->from_raw_link->lanenum_sum;
    //     if (total_lane_num == 2){
    //         oppo_lane_num += 1;
    //         current_lane_num += 1;
    //     }
    //     else
    //     {
    //     //    oppo_lane_num += poss->from_raw_link->lanenum_e2s;
    //     //    current_lane_num += poss->from_raw_link->lanenum_s2e; //还不知道怎么处理，信息不够  lanenum_s2e和lanenum_e2s
    //     }
    // }
    // else
    // {
        current_lane_num += poss->from_raw_link->lanenum_sum;
        if(rc->left != NULL && rc->right != NULL){
            if(boundary.left_oppo_pos.src != NULL){
                auto& oppo_link_poss = boundary.left_oppo_pos.src;
                oppo_lane_num += oppo_link_poss->from_raw_link->lanenum_sum;
            }else{
                int total_lane_num = static_cast<int>((rc->width + 0.5) / ratio_threshold);
                oppo_lane_num = total_lane_num - current_lane_num;
            }
        // }
    }

    return fsdmap::SUCC;
}


int RoadModelProcBuildBoundary::save_debug_info(RoadModelSessionData* session) {
	if (!FLAGS_build_boundary_save_data_enable) {
		return fsdmap::SUCC;
	}
    session->set_display_name("build_boundary");

    // for (auto &line : session->boundary_line_sample_list_opt) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
    //     if (line->invalid()) {
    //         log->color = {150, 0, 150};
    //     } else if (line->src_status == 2 || line->src_status == 4) {
    //         log->color = {100, 100, 255};
    //     }
    //     for (auto &bf : line->list) {
    //         auto &ele = log->add(bf->pos);
    //         ele.label.label = line->src_status;
    //     }
    // }
    // for (auto &line : session->lane_line_sample_list_opt) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "yellow_vote");
    //     log->color = {150, 100, 100};
    //     for (auto &fls : line->list) {
    //         session->debug_pos(fls->pos);
    //         if (fls->attr.color == 3) {
    //             auto &ele = log->add(fls->pos);
    //         }
    //     }
    // }

    for (auto &road_segment : session->road_segment_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {223, 130, 154};
        for (auto &poss : road_segment->pos_sample_list) {
            session->debug_pos(poss->pos);
            auto &ele = log->add(poss->pos);
            ele.label.label = poss->filter_status;
            if (poss->invalid()) {
                ele.color = {255, 0, 0};
                continue;
            }
            auto &boundary = poss->boundary;
            for (auto &rc : boundary.road_center_list) {
                cv::Scalar color = {100, 100, 100};
                if (rc->invalid() || poss->invalid()) {
                    color = {255, 0, 0};
                } else if (rc == boundary.curr) {
                    color = {255, 255, 255};
                }
                if (rc->left != NULL) {
                    auto slog = session->add_debug_log(
                            utils::DisplayInfo::LINE_INDEX, "road_center_left");
                    slog->color = color;
                    slog->color[2] = 0;
                    if (rc->left->src_status == 1 && rc == boundary.curr) {
                        slog->color = {0, 255, 0};
                    }
                    slog->add(rc->pos);
                    slog->add(rc->left->pos);
                }
                if (rc->right != NULL) {
                    auto rlog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, 
                            "road_center_right");
                    rlog->color = color;
                    rlog->color[2] = 125;
                    if (rc->right->src_status == 1 && rc == boundary.curr) {
                        rlog->color = {0, 255, 255};
                    }
                    rlog->add(rc->pos);
                    rlog->add(rc->right->pos);
                }
                if (rc->yellow_boundary.src != NULL) {
                    auto ylog = session->add_debug_log(utils::DisplayInfo::POINT, 
                            "road_center_yellow");
                    ylog->color = {0, 255, 0};
                    ylog->add(rc->yellow_boundary.src->pos);
                    auto next_pos = alg::get_hori_pos(rc->yellow_boundary.src->pos, rc->yellow_boundary.src->dir, 1);
                    ylog->add(next_pos);
                }
            }
        }
    }

    session->save_debug_info("build_boundary");
	return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
