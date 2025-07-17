


#include "road_model_proc_match_lane.h"


DEFINE_bool(match_lane_enable, true, "match_lane_enable");
DEFINE_bool(match_lane_debug_pos_enable, true, "match_lane_debug_enable");
DEFINE_bool(match_lane_save_data_enable, true, "match_lane_save_data_enable");
DEFINE_bool(match_lane_save_data_save_detail_enable, false, "match_lane_save_data_save_detail_enable");
DEFINE_bool(match_lane_save_data_save_frame_enable, false, "match_lane_save_data_save_frame_enable");
DEFINE_bool(match_lane_use_sem_lane_enable, true, "match_lane_use_sem_lane_enable");
DEFINE_bool(match_lane_gen_oppo_enable, true, "match_lane_gen_oppo_enable");
DEFINE_bool(match_lane_filter_by_base_lane_enable, false, "match_lane_filter_by_base_lane_enable");
DEFINE_double(match_lane_scope_buff, 30, "match_lane_scope_buff");
DEFINE_double(match_lane_max_same_dir_feature_radius, 10, "match_lane_max_same_dir_feature_radius");
DEFINE_double(match_lane_max_same_dir_feature_z_radius, 1, "match_lane_max_same_dir_feature_z_radius");
DEFINE_double(match_lane_max_same_dir_feature_scope, 2, "match_lane_max_same_dir_feature_scope");
DEFINE_double(match_lane_max_same_dir_feature_theta, 30, "match_lane_max_same_dir_feature_theta");
DEFINE_double(match_lane_lane_center_min_width1, 2.5, "match_lane_lane_center_min_width1");
DEFINE_double(match_lane_lane_center_max_width1, 4.5, "match_lane_lane_center_max_width1");
DEFINE_double(match_lane_lane_center_min_width2, 2.4, "match_lane_lane_center_min_width2");
DEFINE_double(match_lane_lane_center_max_width2, 5, "match_lane_lane_center_max_width2");
DEFINE_double(match_lane_feature_line_sample_max_dis_theta_threshold, 10, "match_lane_feature_line_sample_max_dis_theta_threshold");
DEFINE_double(match_lane_feature_line_sample_min_theta_threshold, 5, "match_lane_feature_line_sample_min_theta_threshold");
DEFINE_double(match_lane_feature_line_sample_max_theta_threshold, 20, "match_lane_feature_line_sample_max_theta_threshold");
DEFINE_double(match_lane_feature_line_sample_min_trangle_dis, 0.1, "match_lane_feature_line_sample_min_trangle_dis");
DEFINE_double(match_lane_feature_line_sample_max_trangle_dis, 0.5, "match_lane_feature_line_sample_max_trangle_dis");
DEFINE_double(match_lane_raw_lane_center_width, 3.0, "match_lane_raw_lane_center_width");
DEFINE_double(match_lane_base_match_iou_threshold, 0.9, "match_lane_base_match_iou_threshold");

DEFINE_double(match_lane_max_base_match_radius, 8, "match_lane_max_base_match_radius");
DEFINE_double(match_lane_max_base_match_scope, 4, "match_lane_max_base_match_scope");
DEFINE_double(match_lane_raw_lc_match_radius, 8, "match_lane_raw_lc_match_radius");
DEFINE_double(match_lane_raw_lc_match_theta, 10, "match_lane_raw_lc_match_theta");

DEFINE_double(match_lane_raw_fls_match_radius, 4, "match_lane_raw_fls_match_radius");
DEFINE_double(match_lane_raw_fls_match_theta, 10, "match_lane_raw_fls_match_theta");
DEFINE_double(match_lane_raw_min_dis_threshold, 2, "match_lane_raw_min_dis_threshold");

DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcMatchLane::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_match_lane_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_match_lane_debug_pos_enable;

    // line进行两两匹配, 生成道路段
    CHECK_FATAL_PROC(make_feature_tree(session), "make_feature_tree"); // 构建车道线和车道中心线的树

    // line进行两两匹配, 生成道路段
    CHECK_FATAL_PROC(gen_lane_sample(session), "gen_lane_sample"); //遍根据车道线点，新增车道中心点

    // 处理bev识别的lanecenter
    CHECK_FATAL_PROC(make_raw_lane_sample(session), "make_raw_lane_sample");

    // CHECK_FATAL_PROC(merge_lane_center(session), "merge_lane_center");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

// int RoadModelProcMatchLane::merge_lane_center(RoadModelSessionData* session) {
//     session->lane_center_gen_tree.RemoveAll();
//     for (auto &lc : session->lane_center_feature_gen) {
//         session->lane_center_gen_tree.insert(lc->pos, lc);
//     }
//     int times = 0;
//     while (times++ < 2) {
//         for (auto lc : session->lane_center_feature_gen) {
//             session->thread_pool->schedule(
//                     [lc, session, this](utils::ProcessBar *process_bar) {
//                         session->debug_pos(lc->pos);
//                         merge_match_list(session, lc, match_list);
//                     });
//         }
//         session->thread_pool->wait(1, "merge_lane_center[times={}]", times);
//     }
//     return fsdmap::SUCC;
// }

int RoadModelProcMatchLane::make_feature_tree(RoadModelSessionData* session) {

    // 遍历车道线样本列表 lane_line_sample_list_opt
    for (auto &line : session->lane_line_sample_list_opt) {
        for (auto &node : line->list) {
            // if (FLAGS_match_lane_use_sem_lane_enable ^ (node->src->src_status == 2)) {
            //     continue;
            // }
            if (node->invalid()) {
                continue;
            }

            // 条件检查：确保节点的源状态为2
            if (node->src->src_status != 2) {
                continue;
            }

            // 将有效的节点插入到 lane_line_sample_tree 中
            session->lane_line_sample_tree.insert(node->pos, node);
        }
    }

    // 遍历合并车道中心线样本列表 merge_lane_center_list
    for (auto &line : session->merge_lane_center_list) {
        for (auto &lc : line->list) {

            // 将车道中心线节点插入到 raw_lane_center_sample_tree 中
            session->raw_lane_center_sample_tree.insert(lc->pos, lc.get());
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLane::gen_lane_sample(RoadModelSessionData* session) {
    // 遍历原始感知车道线融合降采样后的车道线点，新增车道中心点
    for (auto &line : session->lane_line_sample_list_opt) {
        for (auto &fls : line->list) {
            if (fls->invalid()) {
                continue;
            }
            if (fls->src->src_status != 2) {
                continue;
            }

            // 将处理任务添加到线程池，异步执行
            session->thread_pool->schedule(
                    [fls, session, this](utils::ProcessBar *process_bar) { 
                    session->debug_pos(fls->pos);

                    // 获取车道线点的方向
                    Eigen::Vector3d dir = fls->dir;
                    int size = search_right_fls_by_fls(session, fls, dir, true); // 根据fls右侧最近的车道线生成新的车道中心线点
                    process_bar->num_biz += size;
                    });
        }
    }
    session->thread_pool->wait(1, "process_match_lane base");
    LOG_WARN("lane_center_feature_gen:{}",session->lane_center_feature_gen.size());
    for (auto &lc : session->lane_center_feature_gen) {
        session->lane_center_gen_tree.insert(lc->pos, lc);
    }

    // 遍历原始感知车道线，新增车道中心点
    for (auto &group_line : session->merge_lane_line_list) {
        for (auto &fls_ptr : group_line->list) {
            auto fls = fls_ptr.get();
            if (fls->invalid()) {
                continue;
            }
            if (fls->src->src_status != 1) {
                continue;
            }
            session->thread_pool->schedule(
                    [fls, session, this](utils::ProcessBar *process_bar) { 
                    session->debug_pos(fls->pos);
                    Eigen::Vector3d dir = fls->dir;
                    int size = search_right_fls_by_fls(session, fls, dir, false);
                    process_bar->num_biz += size;
                    });
        }
    }
    session->thread_pool->wait(1, "process_match_lane");
    return fsdmap::SUCC;
}

// 挑选出除了base_fls所在车道线外的，右侧车道线上与base_fls在车道线朝向方向上的距离最小的车道线点lane_center_list；
// 根据base_fls与这些lane_center_list的每个点构建新的车道中心线点lc，判断lc对应的车道数，车道类型（汇入汇出口还是直道）等;
// 统计这些lc对应的车道数的最小值，筛选出车道数是这个最小值的车道中心点 保存；
// 对这些保存下来的新增车道中心点生成行驶方向反向的车道中心点，即方向向量反向，左右侧的车道线点也调换
int64_t RoadModelProcMatchLane::search_right_fls_by_fls(RoadModelSessionData* session,
         LaneLineSample *base_fls, Eigen::Vector3d &dir, bool base) {
     double radius = FLAGS_match_lane_max_same_dir_feature_radius;  //10
     double z_radius = FLAGS_match_lane_max_same_dir_feature_z_radius;  // 1
     double scope = FLAGS_match_lane_max_same_dir_feature_scope;  // 2
     double theta_threshold = FLAGS_match_lane_max_same_dir_feature_theta;  // 30
     auto &pos = base_fls->pos;
     std::vector<LaneLineSample*> search_sec;
     session->lane_line_sample_tree.search(pos, radius, search_sec); // 搜索10m范围内的车道线点
     std::map<std::string, std::pair<double, LaneLineSample*> > dis_map; // 挑选出除了base_fls所在车道线外的，右侧车道线上与base_fls在车道线朝向方向上的距离最小的车道线点
     Eigen::Vector3d center_dir;
     Eigen::Vector3d cross_point;;
     Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(base_fls->pos, base_fls->dir, 50, true); //计算base_fls->dir垂线方向上离pos距离是50m的点
     for (auto& fls : search_sec) {
         if (fls->next == NULL) {
             continue;
         }
         if (fls->src->src_status != 1 && fls->src->src_status != 2) {
             continue;
         }
         if (fls->src->src_status != base_fls->src->src_status && base_fls->src->src_status == 2) {
             continue;
         }
         // if (fls->line_id == base_fls->line_id && fls->src->src_status == 1) {
        //  if (fls->line_id == base_fls->line_id) { // 挑选不是在一条线上的点
        //      continue;
        //  }
        //  if (fls->src->trail_id != base_fls->src->trail_id) { // 挑选在一条轨迹上观测到的点
        //      continue;
        //  }
         Eigen::Vector3d next_pos = fls->next->pos;
         if (fls->next->next == NULL) {
             next_pos = alg::get_hori_pos(fls->pos, fls->dir, radius); //计算dir方向上离pos距离是radius的点
         }

         if (!alg::get_cross_point_for_segment(fls->pos, next_pos, // 计算两线段是否有交点
                      base_fls->pos, poss_v_pt, cross_point, 1)) {
             continue;
         }
         // if (fls->src->frame_id != base_fls->src->frame_id && fls->src->src_status == 1) {
         //     continue;
         // }
         double theta = alg::calc_theta(dir, fls->dir, false, true);
         // if (theta > theta_threshold && theta < 180 - theta_threshold) {
         if (theta > theta_threshold) {
             continue;
         }
         double dis = alg::calc_dis(base_fls->pos, fls->pos, true);
         if (dis > radius) {
             continue;
         }
         double z_dis = fabs(base_fls->pos.z() - fls->pos.z());
         if (z_dis > z_radius) {
             continue;
         }
         int is_left = alg::judge_left(fls->pos, base_fls->pos, dir);
         if (is_left <= 0) { // fls->pos在base_fls->pos左侧就跳过
             continue;
         }
         if (theta < 90) {
             center_dir = base_fls->dir + fls->dir;
         } else {
             center_dir = base_fls->dir - fls->dir;
         }
         double h_dis = alg::calc_hori_dis(fls->pos, pos, center_dir); // fls->pos与pos在车道线朝向方向上的距离
         // if (h_dis > scope) {
         //     continue;
         // }
         auto fit = dis_map.find(fls->line_id);
         if (fit == dis_map.end()) {
             dis_map[fls->line_id] = std::make_pair(h_dis, fls);
         } else if (fit->second.first > h_dis) { // 挑选出line_id这条线上与base_fls在车道线朝向方向上的距离最小的车道线点，也就是base_fls右侧最近的点
             dis_map[fls->line_id].first = h_dis;
             dis_map[fls->line_id].second = fls;
         }
    }
    int64_t ret_size = 0;
    int64_t filter_size = 0;
    std::vector<std::shared_ptr<LaneCenterFeature>> tmp_ptr_list;
    int min_lane_num = INT_MAX;
    for (auto &fit : dis_map) {
        auto lane_center_ptr = std::make_shared<LaneCenterFeature>();
        lane_center_ptr->init(base_fls, fit.second.second); // 根据左侧车道线base_fls，右侧车道线fit.second.second构建新的车道中心点
        if (lane_center_ptr->left->score <= 0.19 
                || lane_center_ptr->right->score <= 0.19) {
            int a = 1;
        }
        // lane_center_ptr->left = base_fls;
        // lane_center_ptr->right = fit.second.second;
        // lane_center_ptr->dir = dir;
        session->debug_pos(lane_center_ptr->pos);
        if (!base && match_base_lane(session, lane_center_ptr.get(), lane_center_ptr->width)) {
            ++filter_size;
            continue;
        }
        lane_center_ptr->src_status = base ? 2 : 1; // 2：新增
        int lane_num = make_lane_sample(session, lane_center_ptr.get());  // 判断lc对应的车道数，车道类型（汇入汇出口还是直道）等
        if (lane_num > 0) {
            session->add_vec(tmp_ptr_list, lane_center_ptr, false);
            min_lane_num = std::min(min_lane_num, lane_num); //选出新增的车道中心点里面最少的车道数
        }
    }
    for (auto &lc : tmp_ptr_list) {
        if (lc->match_level.lane_num > min_lane_num) { //删除车道数大于最少的车道中心点，最少肯定是更准确的
            continue;
        }
        // 对一个车道生成行驶方向反向的车道中心点，即方向向量反向，左右侧的车道线点也调换
        session->add_vec(session->lane_center_feature_ptr, lc, true);
        session->add_vec(session->lane_center_feature_gen, lc.get(), true);
        ++ret_size;
        if (FLAGS_match_lane_gen_oppo_enable) {
            auto oppo_ptr = session->add_ptr(session->lane_center_feature_ptr, true);
            session->add_vec(session->lane_center_feature_gen, oppo_ptr.get(), true);
            oppo_ptr->init(lc.get());
            oppo_ptr->oppo_status = 1;
            oppo_ptr->set_oppo();
        }
    }
    LOG_DEBUG("match pair[size={}, match={}, filter={}]", 
            dis_map.size(), ret_size, filter_size);
    return ret_size;
}

bool RoadModelProcMatchLane::match_base_lane(RoadModelSessionData* session,
        LaneCenterFeature* lc, double base_radius) {
     session->debug_pos(lc->pos);
     if (!FLAGS_match_lane_filter_by_base_lane_enable) {
         return false;
     }
     double radius = FLAGS_match_lane_max_base_match_radius;
     radius += base_radius;
     double scope = FLAGS_match_lane_max_base_match_scope;
     double theta_threshold = FLAGS_match_lane_max_same_dir_feature_theta;
     double iou_threshold = FLAGS_match_lane_base_match_iou_threshold;
     std::vector<LaneCenterFeature*> search_sec;
     session->lane_center_gen_tree.search(lc->pos, radius, search_sec);
     bool has_prev = false;
     bool has_next = false;
     for (auto &tar_lc : search_sec) {
         double h_dis = alg::calc_hori_dis(lc->pos, tar_lc->pos, tar_lc->dir, true);
         // if (fabs(h_dis) > scope) {
         //     continue;
         // }
         double theta = alg::calc_theta(tar_lc->dir, lc->dir);
         if (theta > theta_threshold) {
             continue;
         }
         if (alg::judge_left(lc->get_left(), tar_lc->get_right(), tar_lc->dir) > 0) {
             continue;
         }
         if (alg::judge_left(lc->get_right(), tar_lc->get_left(), tar_lc->dir) < 0) {
             continue;
         }
         // 以tar_lc left 为base
         double iou = alg::segment_iou(lc->get_left(), lc->get_right(), 
                 tar_lc->get_left(), tar_lc->get_right());
         if (iou < iou_threshold) {
             continue;
         }
         // if (fabs(h_dis) > scope) {
         //     continue;
         // }
         if (h_dis <= 0) {
             if (fabs(h_dis) > scope) {
                 has_prev = true;
             }
         } else {
             if (h_dis > scope) {
                 has_next = true;
             }
         }
         if (has_prev && has_next) {
             return true;
         }
     }
     return false;
}

// 判断lc对应的车道数，车道类型（汇入汇出口还是直道）等
int RoadModelProcMatchLane::make_lane_sample(RoadModelSessionData* session,
        LaneCenterFeature* lc) {
    double min_dis_threshold1 = FLAGS_match_lane_lane_center_min_width1;  // 2.5
    double max_dis_threshold1 = FLAGS_match_lane_lane_center_max_width1;  // 4，5
    double min_dis_threshold2 = FLAGS_match_lane_lane_center_min_width2;  // 2.4
    double max_dis_threshold2 = FLAGS_match_lane_lane_center_max_width2;  // 5
    double max_dis_theta_threshold = FLAGS_match_lane_feature_line_sample_max_dis_theta_threshold;  // 10
    double min_theta_threshold = FLAGS_match_lane_feature_line_sample_min_theta_threshold;  // 5
    double max_theta_threshold = FLAGS_match_lane_feature_line_sample_max_theta_threshold;  // 20
    double min_trangle_dis = FLAGS_match_lane_feature_line_sample_min_trangle_dis;  // 0.1
    double max_trangle_dis = FLAGS_match_lane_feature_line_sample_max_trangle_dis;   // 0.5
    double min_dis_threshold = lc->src_status == 2 ? min_dis_threshold2 : min_dis_threshold1;
    double max_dis_threshold = lc->src_status == 2 ? max_dis_threshold2 : max_dis_threshold1;

    int lane_type = 0;
    int join_out = 0;
    std::vector<int> lane_num_list = {1, 2};
    int valid_lane_num = 0;
    for (auto &lane_num : lane_num_list) {
        do {
            // 满足一个车道宽度
            if (lc->width >= lane_num * min_dis_threshold 
                    && lc->width <= lane_num * max_dis_threshold
                    && lc->theta < max_dis_theta_threshold) {
                // 同向等距情况
                if (lane_num == 1) { // 满足一个车道宽度，直接结束
                    lane_type = 1;
                    break;
                }
            }
            double min_dis = (lane_num - 1) * min_dis_threshold;
            min_dis = std::max(min_dis, min_trangle_dis);
            double max_dis = (lane_num + 1) * min_dis_threshold + max_trangle_dis;
            // 大于一个车道宽度，有可能是汇入汇出口，一车道；
            // 或者是大于一个车道宽度，有可能是汇入汇出口，2车道
            if (lc->theta >= min_theta_threshold
                    && lc->theta <= max_theta_threshold
                    && lc->width > min_dis
                    && lc->width < max_dis) {
                // 夹角情况
                lane_type = 2;
                // 判断开口方向是汇出还是汇入
                if (alg::judge_left(lc->left->dir, lc->dir, lc->dir) < 0) { // 这里应该是lc->pos吧？？？？？？
                    // 1 汇出，2 汇入
                    join_out = 1;
                } else {
                    join_out = 2;
                }
                break;
            }
        } while(0);
        if (lane_type > 0) {
            valid_lane_num = lane_num;
            break;
        }
    }

    if (lane_type <= 0) {
        DLOG_POINT2(lc->left->pos, lc->right->pos, "lane line sample not match["
                "theta={}, width={}]",
                lc->theta, lc->width);
        return valid_lane_num;
    }
    lc->match_level.lane_type = lane_type;
    lc->match_level.lane_num = valid_lane_num;
    lc->match_level.join_out = join_out;
    lc->score = sqrt(lc->left->score * lc->right->score);
    lc->frame_id = lc->left->src->frame_id;
    lc->trail_id = lc->left->src->trail_id;
    lc->key_pose = lc->left->src->key_pose;
    lc->line_id = utils::fmt("{}_{}", lc->left->line_id, lc->right->line_id);
    DLOG_POINT2(lc->left->pos, lc->right->pos, "line sample match[num={}, match_type={},"
            "theta={}, width={}, join_out={}]",
            valid_lane_num, lane_type, lc->theta, lc->width, join_out);
    return valid_lane_num;
}

// 由merge_lane_center_list构建反向车道中心线点；
// 把这些反向车道中心线点重新放入merge_lane_center_list
int RoadModelProcMatchLane::make_raw_lane_sample(RoadModelSessionData* session) {
    // 遍历合并后的车道中心线列表（merge_lane_center_list）
    LOG_WARN("merge_lane_center_list:{}",session->merge_lane_center_list.size());
    for (auto &line : session->merge_lane_center_list) {
        // 遍历每条车道线的中心线特征点
        for (auto &lc : line->list) {
            // 异步任务：处理每个车道线特征点
            session->thread_pool->schedule([lc, session, this](utils::ProcessBar *process_bar) { 
                // 调试输出当前车道点的坐标
                session->debug_pos(lc->pos);
                // 根据原始数据生成车道中心
                make_lane_center_by_raw(session, lc.get());
                // 如果与基础车道线匹配，则更新过滤状态
                if (match_base_lane(session, lc.get(), lc->width)) {
                    lc->filter_status = 2;
                }
            });
        }
    }
    // 等待线程池任务完成
    session->thread_pool->wait(1, "process_make_raw_lane_center");

    // 如果启用生成反向车道线（FLAGS_match_lane_gen_oppo_enable）
    if (FLAGS_match_lane_gen_oppo_enable) {
        int64_t line_index = 0;
        std::vector<LaneCenterGroupLine*> oppo_vec; 
        // 为反向车道线列表预留空间
        oppo_vec.reserve(session->merge_lane_center_list.size());
        
        // 遍历每条车道线生成反向车道线
        for (auto &line : session->merge_lane_center_list) {
            // 创建反向车道线
            auto oppo_line = session->add_ptr(session->lane_center_group_line_ptr);
            oppo_line->id = utils::fmt("{}_oppo", line->id); // 反向车道线ID
            oppo_line->list.reserve(line->list.size()); // 为反向车道线预留空间
            
            LaneCenterFeature* prev_lc = NULL;
            // 遍历车道线上的每个特征点
            for (auto &lc : line->list) {
                // 为反向车道线添加一个新的特征点
                auto new_lc = session->add_ptr(oppo_line->list);
                new_lc->init(lc.get()); // 初始化新的特征点
                new_lc->set_oppo(); // 标记为反向车道线点
                new_lc->oppo_status = 1; // 设置反向状态
                
                // 更新前后特征点的链接
                if (prev_lc != NULL) {
                    prev_lc->prev = new_lc.get();
                    new_lc->next = prev_lc;
                }
                // 将新的特征点插入到反向车道线列表中
                VEC_INSERT(oppo_line->list, new_lc);
                prev_lc = new_lc.get(); // 更新前一个特征点
            }
            // 将反向车道线添加到反向车道线列表
            oppo_vec.push_back(oppo_line.get());
        }
        // 将生成的反向车道线列表加入到原车道线列表中
        VEC_PUSH_ALL(session->merge_lane_center_list, oppo_vec);
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLane::make_lane_center_by_raw(RoadModelSessionData* session,
        LaneCenterFeature* lc) {
    double dft_lane_width = FLAGS_match_lane_raw_lane_center_width;
    // DisDirPoint left_lc;
    // DisDirPoint right_lc;
    // match_lane_center(session, lc, left_lc, right_lc);
    // Eigen::Vector3d left_pos = alg::get_vertical_pos(lc->pos, lc->dir, -dft_lane_width / 2);
    // Eigen::Vector3d right_pos = alg::get_vertical_pos(lc->pos, lc->dir, dft_lane_width / 2);
    // Eigen::Vector3d left_dir = lc->dir;
    // Eigen::Vector3d right_dir = lc->dir;
    // if (left_lc.is_valid) {
    //     left_pos = (lc->pos + left_lc.pos) / 2;
    //     left_dir = (lc->dir + left_lc.dir);
    //     left_dir.normalize();
    // } else {
    //     auto fls = match_fls_by_lc(session, lc, dft_lane_width / 2, true);
    //     if (fls.is_valid) {
    //         left_pos = fls.pos;
    //         left_dir = fls.dir;
    //     }
    // }

    // if (right_lc.is_valid) {
    //     right_pos = (lc->pos + right_lc.pos) / 2;
    //     right_dir = (lc->dir + right_lc.dir);
    //     right_dir.normalize();
    // } else {
    //     auto fls = match_fls_by_lc(session, lc, dft_lane_width / 2, false);
    //     if (fls.is_valid) {
    //         right_pos = fls.pos;
    //         right_dir = fls.dir;
    //     }
    // }
    // auto raw_pos = lc->pos;
    // lc->init(left_pos, left_dir, right_pos, right_dir);
    // session->debug_pos(lc->pos);
    // if (alg::calc_dis(lc->pos, raw_pos) > 0.5) {
    //     int a = 1;
    // }
    lc->match_level.lane_type = 1;
    lc->match_level.lane_num = 1;
    lc->match_level.join_out = 0;
    lc->src_status = 3;
    // if (lc->width < 1) {
    //     int a = 1;
    // }
    if (lc->next != NULL) {
        double theta = alg::calc_theta(lc->dir, lc->next->dir);
        if (theta > 90) {
            int a = 1;
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLane::match_lane_center(RoadModelSessionData* session,
         LaneCenterFeature *lc, DisDirPoint &left_lc, DisDirPoint &right_lc) {
    double radius = FLAGS_match_lane_raw_lc_match_radius;
    double theta_threshold = FLAGS_match_lane_raw_lc_match_theta;
    double min_dis_threshold = FLAGS_match_lane_raw_min_dis_threshold;
    std::vector<LaneCenterFeature*> search_sec;
    session->raw_lane_center_sample_tree.search(lc->pos, radius, search_sec);
    double min_dis_l = DBL_MAX;
    double min_dis_r = DBL_MAX;
    Eigen::Vector3d cross_point;
    left_lc.is_valid = false;
    right_lc.is_valid = false;
    auto v_pos = alg::get_vertical_pos(lc->pos, lc->dir, 50);
    for (auto& tar_lc : search_sec) {
        if (tar_lc->next == NULL) {
            continue;
        }
        if (tar_lc->frame_id != lc->frame_id) {
            continue;
        }
        if (tar_lc->line_id == lc->line_id) {
            continue;
        }

        double theta = alg::calc_theta(tar_lc->dir, lc->dir);
        if (theta > theta_threshold) {
            continue;
        }
        if (!alg::get_cross_point_for_segment(tar_lc->pos, tar_lc->next->pos,
                    lc->pos, v_pos, cross_point, 1)) {
            continue;
        }
        double dis = alg::calc_dis(cross_point, lc->pos);
        if (dis < min_dis_threshold) {
            continue;
        }
        if (alg::judge_left(cross_point, lc->pos, lc->dir) < 0) {
            if (min_dis_l > dis) {
                min_dis_l = dis;
                left_lc.pos = cross_point;
                left_lc.dir = lc->dir;
                left_lc.is_valid = true;
            }
        } else {
            if (min_dis_r > dis) {
                min_dis_r = dis;
                right_lc.pos = cross_point;
                right_lc.dir = lc->dir;
                right_lc.is_valid = true;
            }
        }
    }
    return fsdmap::SUCC;
}

DisDirPoint RoadModelProcMatchLane::match_fls_by_lc(RoadModelSessionData* session,
         LaneCenterFeature *lc, double tar_dis, bool left) {
    double radius = FLAGS_match_lane_raw_fls_match_radius;
    double theta_threshold = FLAGS_match_lane_raw_fls_match_theta;
    double min_dis_threshold = FLAGS_match_lane_raw_min_dis_threshold;
    std::vector<LaneLineSample*> search_sec;
    session->lane_line_sample_tree.search(lc->pos, radius, search_sec);
    double min_dis = DBL_MAX;
    Eigen::Vector3d cross_point;
    DisDirPoint ret;
    ret.is_valid = false;
    auto v_pos = alg::get_vertical_pos(lc->pos, lc->dir, 50);
    for (auto& tar_fls : search_sec) {
        if (tar_fls->next == NULL) {
            continue;
        }
        if (tar_fls->src->src_status != 1) {
            continue;
        }
        double theta = alg::calc_theta(tar_fls->dir, lc->dir);
        if (theta > theta_threshold) {
            continue;
        }
        if (!alg::get_cross_point_for_segment(tar_fls->pos, tar_fls->next->pos,
                    lc->pos, v_pos, cross_point, 1)) {
            continue;
        }
        if (left ^ (alg::judge_left(cross_point, lc->pos, lc->dir) < 0)) {
            continue;
        }
        double dis = alg::calc_dis(cross_point, lc->pos);
        double gap = fabs(dis - tar_dis);
        if (gap > min_dis_threshold) {
            continue;
        }
        if (min_dis > gap) {
            min_dis = gap;
            ret.pos = cross_point;
            ret.dir = tar_fls->dir;
            ret.is_valid = true;
        }
    }
    return ret;
}

// int RoadModelProcMatchLane::merge_match_list(
//         RoadModelSessionData* session, LaneCenterFeature* lc) {
//     float max_theta = FLAGS_gen_lane_line_search_lc_max_theta;
//     float radius = FLAGS_gen_lane_line_search_lc_max_radius;
//     float min_radius = FLAGS_gen_lane_line_search_lc_max_min_radius;
//     auto &pos = lc->pos;
//     std::vector<LaneCenterFeature*> search_sec;
//     session->lane_center_gen_tree.search(pos, radius, search_sec);
//     std::vector<std::shared_ptr<LaneCenterFeatureMatchPair>> match_list;
// 
//     for (auto &lc_get : search_sec) {
//         if (lc_get->invalid()) {
//             continue;
//         }
//         if (lc_get->src_status != 3 && lc->src_status == 3) { 
//             continue;
//         }
//         if (lc_get->src_status == 3 && lc->src_status != 3) { 
//             continue;
//         }
//         double dis = alg::calc_dis(lc->pos, lc_get->pos);
//         if (dis > radius) {
//             continue;
//         }
// 
//         double h_dis = alg::calc_hori_dis(lc_get->pos, lc->pos, lc->dir, true);
//         if (fabs(h_dis) <= scope) {
//             continue;
//         }
//         if (MAP_FIND(cache_map, lc_get->line_id)) {
//             if (cache_map[lc_get->line_id]->h_dis < h_dis) {
//                 continue;
//             }
//         }
// 
//         if (dis < min_radius) {
//             continue;
//         }
//         
//         double theta = alg::calc_theta(lc->dir, lc_get->dir);
//         if (theta > max_theta) {
//             continue;
//         }
//         auto pair = std::make_shared<LaneCenterFeatureMatchPair>();
//         pair->left = lc;
//         pair->right = lc_get;
//         pair->dis = dis;
//         pair->h_dis = h_dis;
//         pair->theta = theta;
// 
//         // if (!lc->is_same_base(lc_get)) {
//             // 过滤不同层级的,大类里的可以匹配
//             if (lc->match_level.lane_num != lc_get->match_level.lane_num) {
//                 continue;
//             }
//             
//             Eigen::Vector3d dir_gap = alg::get_dir(lc_get->pos, lc->pos);
//             double delta_l_theta = alg::calc_theta_with_dir(lc->dir, dir_gap);
//             if (fabs(delta_l_theta) > max_theta) {
//                 continue;
//             }
//             double delta_r_theta = alg::calc_theta_with_dir(lc_get->dir, dir_gap);
//             if (fabs(delta_r_theta) > max_theta) {
//                 continue;
//             }
//             pair->dir_gap = dir_gap;
//             pair->delta_l_theta = delta_l_theta;
//             pair->delta_r_theta = delta_r_theta;
//         // }
//         // if (!lc->is_same_base(lc_get)) {
//             calc_lane_center_line_match_score(session, pair.get());
//         // } else {
//         //     pair->match = true;
//         // }
//         recall_num++;
//         if (pair->match) {
//             session->add_vec(session->lane_center_line_match_pair_ptr, pair, true);
//             secs.push_back(pair.get());
//             cache_map[lc_get->line_id] = pair;
//         }
//     }
//     SORT(secs,
//             [](const LaneCenterFeatureMatchPair *l, const LaneCenterFeatureMatchPair *r)->bool {
//             return l->score > r->score;
//             });
// 
//     LOG_DEBUG("lane center match line pair[size={}, match={}]", recall_num, secs.size());
//     if (secs.size() > 0) {
//         return secs[0]->get_other(lc);
//     }
//     return NULL;
// }

int RoadModelProcMatchLane::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_match_lane_save_data_enable) {
        return fsdmap::SUCC;
    }
    double scope_buff = FLAGS_display_scope_buff;
    session->set_display_name("match_lane");
    // session->concate_display_name("sample_line");
    // auto log = session->add_debug_log(utils::DisplayInfo::LINE, "key_pose_sample");
    // log->color = {223, 130, 154};
    // for (auto &key_pose : session->pose_sample_list) {
    //     log->add(key_pose->pos);
    // }
    
    PTR_VEC<utils::DisplayInfo> log_ptr_list;
    UMAP<std::string, std::vector<utils::DisplayInfo*>> frame_map;
    std::vector<utils::DisplayInfo*> line_id_log;
    int64_t lc_id = 0;
    for (auto &lc : session->lane_center_feature_gen) {
        session->debug_pos(lc->pos);
        // if (lc->src_status != 2) {
        //     continue;
        // }
        lc_id++;
        line_id_log.clear();
        auto &frame_id = lc->frame_id;
        if (MAP_NOT_FIND(frame_map, frame_id)) {
            frame_map[frame_id] = std::vector<utils::DisplayInfo*>();
        }
        auto log_ptr = session->add_ptr(log_ptr_list);
        auto log_ptr1 = session->add_ptr(log_ptr_list);
        log_ptr->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", lc->line_id);
        log_ptr->color = {200, 200, 100};
        log_ptr->log_name = lc->left->line_id;
        log_ptr1->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", lc->line_id);
        log_ptr1->color = {100, 200, 200};
        log_ptr1->log_name = lc->right->line_id;
        if (lc->src_status == 2) {
            log_ptr->color = {255, 255, 255};
            log_ptr1->color = {0, 255, 0};
        }
        line_id_log.push_back(log_ptr.get());
        line_id_log.push_back(log_ptr1.get());
        frame_map[frame_id].push_back(log_ptr.get());
        frame_map[frame_id].push_back(log_ptr1.get());
        log_ptr->add(lc->pos, 1, lc->match_level.lane_type);
        log_ptr->add(lc->get_left(), 1, lc->match_level.lane_type);

        if (lc->left->next != NULL) {
            log_ptr->add(lc->left->next->pos, 1);
        } else if (lc->left->prev != NULL) {
            log_ptr->add(lc->left->prev->pos, 1);
        }

        log_ptr1->add(lc->pos, 1, lc->match_level.lane_type);
        log_ptr1->add(lc->get_right(), 1, lc->match_level.lane_type);
        if (lc->right->next != NULL) {
            log_ptr1->add(lc->right->next->pos, 1);
        } else if (lc->right->prev != NULL) {
            log_ptr1->add(lc->right->prev->pos, 1);
        }
        session->add_debug_log(log_ptr.get());
        session->add_debug_log(log_ptr1.get());

        if (!FLAGS_match_lane_save_data_save_detail_enable) {
            continue;
        }
        auto key_pose = session->data_processer->get_key_pose(frame_id);
        if (key_pose == NULL) {
            continue;
        }
        auto log_name = session->get_debug_dir("{}_{}_{}.png", lc->left->line_id, "match_lane", lc_id);
        utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
        box.set_resolution(FLAGS_display_scale_rate);
        // box.dir.z() = -key_pose->yaw;
        utils::save_display_image(log_name.c_str(), box, line_id_log);
        // auto data_name = session->get_debug_dir("{}_{}.csv", "lane_sample", lc_id);
        // utils::save_data_to_file(data_name.c_str(), line->param_list);
    }
    if (FLAGS_match_lane_save_data_save_frame_enable) {
        for (auto &it : frame_map) {
            auto key_pose = session->data_processer->get_key_pose(it.first);
            if (key_pose == NULL) {
                continue;
            }
            utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
            box.set_resolution(FLAGS_display_scale_rate);
            // box.dir.z() = -key_pose->yaw;
            auto log_name = session->get_debug_dir("{}_{}.png", "match_lane", it.first);
            utils::save_display_image(log_name.c_str(), box, it.second);
        }
    }

    session->save_debug_info("match_lane");
    return fsdmap::SUCC;
}

}
}
