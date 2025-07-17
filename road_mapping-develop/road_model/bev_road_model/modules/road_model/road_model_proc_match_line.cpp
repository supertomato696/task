


#include "road_model_proc_match_line.h"


DEFINE_bool(match_line_enable, true, "match_line_enable");
DEFINE_bool(match_line_debug_pos_enable, true, "match_line_debug_enable");
DEFINE_bool(match_line_save_data_enable, true, "match_line_save_data_enable");
DEFINE_bool(match_line_save_data_save_detail_enable, false, "match_line_save_data_save_detail_enable");
DEFINE_bool(match_line_save_data_save_frame_enable, false, "match_line_save_data_save_frame_enable");
DEFINE_bool(match_line_use_sem_lane_enable, true, "match_line_use_sem_lane_enable");
DEFINE_bool(match_line_gen_oppo_enable, true, "match_line_gen_oppo_enable");
DEFINE_bool(match_line_filter_by_base_lane_enable, false, "match_line_filter_by_base_lane_enable");
DEFINE_double(match_line_scope_buff, 30, "match_line_scope_buff");
DEFINE_double(match_line_max_same_dir_feature_radius, 10, "match_line_max_same_dir_feature_radius");
DEFINE_double(match_line_max_same_dir_feature_z_radius, 1, "match_line_max_same_dir_feature_z_radius");
DEFINE_double(match_line_max_same_dir_feature_scope, 2, "match_line_max_same_dir_feature_scope");
DEFINE_double(match_line_max_same_dir_feature_theta, 30, "match_line_max_same_dir_feature_theta");
DEFINE_double(match_line_lane_center_min_width1, 2.5, "match_line_lane_center_min_width1");
DEFINE_double(match_line_lane_center_max_width1, 4.5, "match_line_lane_center_max_width1");
DEFINE_double(match_line_lane_center_min_width2, 2.4, "match_line_lane_center_min_width2");
DEFINE_double(match_line_lane_center_max_width2, 5, "match_line_lane_center_max_width2");
DEFINE_double(match_line_feature_line_sample_max_dis_theta_threshold, 10, "match_line_feature_line_sample_max_dis_theta_threshold");
DEFINE_double(match_line_feature_line_sample_min_theta_threshold, 5, "match_line_feature_line_sample_min_theta_threshold");
DEFINE_double(match_line_feature_line_sample_max_theta_threshold, 20, "match_line_feature_line_sample_max_theta_threshold");
DEFINE_double(match_line_feature_line_sample_min_trangle_dis, 0.1, "match_line_feature_line_sample_min_trangle_dis");
DEFINE_double(match_line_feature_line_sample_max_trangle_dis, 0.5, "match_line_feature_line_sample_max_trangle_dis");
DEFINE_double(match_line_raw_lane_center_width, 3.0, "match_line_raw_lane_center_width");
DEFINE_double(match_line_base_match_iou_threshold, 0.9, "match_line_base_match_iou_threshold");

DEFINE_double(match_line_max_base_match_radius, 8, "match_line_max_base_match_radius");
DEFINE_double(match_line_max_base_match_scope, 4, "match_line_max_base_match_scope");
DEFINE_double(match_line_raw_lc_match_radius, 8, "match_line_raw_lc_match_radius");
DEFINE_double(match_line_raw_lc_match_theta, 10, "match_line_raw_lc_match_theta");

DEFINE_double(match_line_raw_fls_match_radius, 8, "match_line_raw_fls_match_radius");
DEFINE_double(match_line_raw_fls_match_theta, 10, "match_line_raw_fls_match_theta");
DEFINE_double(match_line_raw_min_dis_threshold, 1, "match_line_raw_min_dis_threshold");

DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcMatchLine::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_match_line_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_match_line_debug_pos_enable;

    // 处理bev识别的lanecenter
    CHECK_FATAL_PROC(make_raw_lane_sample(session), "make_raw_lane_sample");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

// int RoadModelProcMatchLine::merge_lane_center(RoadModelSessionData* session) {
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

int RoadModelProcMatchLine::make_feature_tree(RoadModelSessionData* session) {
    int i=0;
    for (auto &line : session->lane_line_sample_list_opt) {
        for (auto &node : line->list) {
            // if (FLAGS_match_line_use_sem_lane_enable ^ (node->src->src_status == 2)) {
            //     continue;
            // }
            if (node->invalid()) {
                continue;
            }
            if (node->src->src_status != 2) {
                continue;
            }
            i++;
            session->lane_line_sample_tree.insert(node->pos, node);
        }
    }
    LOG_INFO("lane_line_sample_tree insert:{}",i);
    // for (auto &line : session->lane_center_line_sample_list_opt) {
    for (auto &line : session->merge_lane_center_list) {
        for (auto &lc : line->list) {
            session->raw_lane_center_sample_tree.insert(lc->pos, lc.get());
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLine::gen_lane_sample(RoadModelSessionData* session) {
    for (auto &line : session->lane_line_sample_list_opt) {
        for (auto &fls : line->list) {
            if (fls->invalid()) {
                continue;
            }
            if (fls->src->src_status != 2) {
                continue;
            }
            session->thread_pool->schedule(
                    [fls, session, this](utils::ProcessBar *process_bar) { 
                    session->debug_pos(fls->pos);
                    Eigen::Vector3d dir = fls->dir;
                    int size = search_right_fls_by_fls(session, fls, dir, true);
                    process_bar->num_biz += size;
                    });
        }
    }
    session->thread_pool->wait(1, "process_match_line base");
    for (auto &lc : session->lane_center_feature_gen) {
        session->lane_center_gen_tree.insert(lc->pos, lc);
    }
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
    session->thread_pool->wait(1, "process_match_line");
    return fsdmap::SUCC;
}

int64_t RoadModelProcMatchLine::search_right_fls_by_fls(RoadModelSessionData* session,
         LaneLineSample *base_fls, Eigen::Vector3d &dir, bool base) {
     double radius = FLAGS_match_line_max_same_dir_feature_radius;
     double z_radius = FLAGS_match_line_max_same_dir_feature_z_radius;
     double scope = FLAGS_match_line_max_same_dir_feature_scope;
     double theta_threshold = FLAGS_match_line_max_same_dir_feature_theta;
     auto &pos = base_fls->pos;
     std::vector<LaneLineSample*> search_sec;
     session->lane_line_sample_tree.search(pos, radius, search_sec);
     std::map<std::string, std::pair<double, LaneLineSample*> > dis_map;
     Eigen::Vector3d center_dir;
     Eigen::Vector3d cross_point;;
     Eigen::Vector3d poss_v_pt = alg::get_vertical_pos(base_fls->pos, base_fls->dir, 50, true);
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
         if (fls->line_id == base_fls->line_id) {
             continue;
         }
         if (fls->src->trail_id != base_fls->src->trail_id) {
             continue;
         }
         Eigen::Vector3d next_pos = fls->next->pos;
         if (fls->next->next == NULL) {
             next_pos = alg::get_hori_pos(fls->pos, fls->dir, radius);
         }

         if (!alg::get_cross_point_for_segment(fls->pos, next_pos,
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
         if (is_left <= 0) {
             continue;
         }
         if (theta < 90) {
             center_dir = base_fls->dir + fls->dir;
         } else {
             center_dir = base_fls->dir - fls->dir;
         }
         double h_dis = alg::calc_hori_dis(fls->pos, pos, center_dir);
         // if (h_dis > scope) {
         //     continue;
         // }
         auto fit = dis_map.find(fls->line_id);
         if (fit == dis_map.end()) {
             dis_map[fls->line_id] = std::make_pair(h_dis, fls);
         } else if (fit->second.first > h_dis) {
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
        lane_center_ptr->init(base_fls, fit.second.second);
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
        lane_center_ptr->src_status = base ? 2 : 1;
        int lane_num = make_lane_sample(session, lane_center_ptr.get());
        if (lane_num > 0) {
            session->add_vec(tmp_ptr_list, lane_center_ptr, false);
            min_lane_num = std::min(min_lane_num, lane_num);
        }
    }
    for (auto &lc : tmp_ptr_list) {
        if (lc->match_level.lane_num > min_lane_num) {
            continue;
        }
        session->add_vec(session->lane_center_feature_ptr, lc, true);
        session->add_vec(session->lane_center_feature_gen, lc.get(), true);
        ++ret_size;
        if (FLAGS_match_line_gen_oppo_enable) {
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

bool RoadModelProcMatchLine::match_base_lane(RoadModelSessionData* session,
        LaneCenterFeature* lc, double base_radius) {
     session->debug_pos(lc->pos);
     if (!FLAGS_match_line_filter_by_base_lane_enable) {
         return false;
     }
     double radius = FLAGS_match_line_max_base_match_radius;
     radius += base_radius;
     double scope = FLAGS_match_line_max_base_match_scope;
     double theta_threshold = FLAGS_match_line_max_same_dir_feature_theta;
     double iou_threshold = FLAGS_match_line_base_match_iou_threshold;
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


int RoadModelProcMatchLine::make_lane_sample(RoadModelSessionData* session,
        LaneCenterFeature* lc) {
    double min_dis_threshold1 = FLAGS_match_line_lane_center_min_width1;
    double max_dis_threshold1 = FLAGS_match_line_lane_center_max_width1;
    double min_dis_threshold2 = FLAGS_match_line_lane_center_min_width2;
    double max_dis_threshold2 = FLAGS_match_line_lane_center_max_width2;
    double max_dis_theta_threshold = FLAGS_match_line_feature_line_sample_max_dis_theta_threshold;
    double min_theta_threshold = FLAGS_match_line_feature_line_sample_min_theta_threshold;
    double max_theta_threshold = FLAGS_match_line_feature_line_sample_max_theta_threshold;
    double min_trangle_dis = FLAGS_match_line_feature_line_sample_min_trangle_dis;
    double max_trangle_dis = FLAGS_match_line_feature_line_sample_max_trangle_dis;
    double min_dis_threshold = lc->src_status == 2 ? min_dis_threshold2 : min_dis_threshold1;
    double max_dis_threshold = lc->src_status == 2 ? max_dis_threshold2 : max_dis_threshold1;

    int lane_type = 0;
    int join_out = 0;
    std::vector<int> lane_num_list = {1, 2};
    int valid_lane_num = 0;
    for (auto &lane_num : lane_num_list) {
        do {
            if (lc->width >= lane_num * min_dis_threshold 
                    && lc->width <= lane_num * max_dis_threshold
                    && lc->theta < max_dis_theta_threshold) {
                // 同向等距情况
                if (lane_num == 1) {
                    lane_type = 1;
                    break;
                }
            }
            double min_dis = (lane_num - 1) * min_dis_threshold;
            min_dis = std::max(min_dis, min_trangle_dis);
            double max_dis = (lane_num + 1) * min_dis_threshold + max_trangle_dis;
            if (lc->theta >= min_theta_threshold
                    && lc->theta <= max_theta_threshold
                    && lc->width > min_dis
                    && lc->width < max_dis) {
                // 夹角情况
                lane_type = 2;
                // 判断开口方向是汇出还是汇入
                if (alg::judge_left(lc->left->dir, lc->dir, lc->dir) < 0) {
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

int RoadModelProcMatchLine::make_raw_lane_sample(RoadModelSessionData* session) {
    // for (auto &line : session->lane_center_line_sample_list_opt) {
    // lane_center_feature_merge_raw中的车道中心线点的位置毕竟按车道线中心点按左右1.5m偏移作为左右侧车道线点，
    // 根据左右侧车道线点取平均值作为车道中心点的位置，是虚构的车道线中心点，需要用真实的车道线点计算车道中心点位置
    for (auto lc : session->lane_center_feature_merge_raw) {
        // if (lc->invalid()) {
        //     continue;
        // }
        session->thread_pool->schedule([lc, session, this](utils::ProcessBar *process_bar) { 
            session->debug_pos(lc->pos);
        make_lane_center_by_raw(session, lc);
        });
    }
    session->thread_pool->wait(1, "process_make_raw_lane_center");
    return fsdmap::SUCC;
}

// 搜索lc左右车道线，用这些车道线点更新车道中心线lc的位置，车道宽度等信息
int RoadModelProcMatchLine::make_lane_center_by_raw(RoadModelSessionData* session,
        LaneCenterFeature* lc) {
    double dft_lane_width = FLAGS_match_line_raw_lane_center_width;  // 3.0
    DisDirPoint left_lc;
    DisDirPoint right_lc;
    // match_line_center(session, lc, left_lc, right_lc);
    Eigen::Vector3d &left_pos = lc->left_pos;
    Eigen::Vector3d &right_pos = lc->right_pos;
    Eigen::Vector3d left_dir = lc->left_dir;
    Eigen::Vector3d right_dir = lc->right_dir;
    {
        // 筛选出车道线中线点lc 8m范围内的车道线点中左侧的车道线点，选出其中与lc距离差与半车道宽度差距最小的车道线点
        auto fls = match_fls_by_lc(session, lc, dft_lane_width / 2, true);
        if (fls != NULL) {
            lc->left = fls;
            left_pos = fls->pos;
            lc->refine_pos_status=1;
        }
    }

    {
        // 筛选出车道线中线点lc 8m范围内的车道线点中右侧的车道线点，选出其中与lc距离差与半车道宽度差距最小的车道线点
        auto fls = match_fls_by_lc(session, lc, dft_lane_width / 2, false);
        if (fls != NULL) {
            lc->right = fls;
            right_pos = fls->pos;
            lc->refine_pos_status+=(1<<1);
        }
    }
        
    auto raw_pos = lc->pos;
    lc->raw_width = lc->width;
    lc->init(left_pos, left_dir, right_pos, right_dir);  // 更新车道中心点的位置等信息
    session->debug_pos(lc->pos);
    if (alg::calc_dis(lc->pos, raw_pos) > 0.5) {
        int a = 1;
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLine::match_line_center(RoadModelSessionData* session,
         LaneCenterFeature *lc, DisDirPoint &left_lc, DisDirPoint &right_lc) {
    double radius = FLAGS_match_line_raw_lc_match_radius;
    double theta_threshold = FLAGS_match_line_raw_lc_match_theta;
    double min_dis_threshold = FLAGS_match_line_raw_min_dis_threshold;
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



// 当left==1,,则筛选出车道线中线点lc 8m范围内的车道线点中左侧的车道线点，选出其中与lc距离差与半车道宽度tar_dis差距最小的车道线点
LaneLineSample* RoadModelProcMatchLine::match_fls_by_lc(RoadModelSessionData* session,
         LaneCenterFeature *lc, double tar_dis, bool left) {
    double radius = FLAGS_match_line_raw_fls_match_radius;  //8
    double theta_threshold = FLAGS_match_line_raw_fls_match_theta;  // 10
    double min_dis_threshold = FLAGS_match_line_raw_min_dis_threshold;  // 1
    std::vector<LaneLineSample*> search_sec;
    session->lane_line_sample_tree.search(lc->pos, radius, search_sec);  // 车道线中线点8m范围内的车道线点
    double min_dis = DBL_MAX;
    Eigen::Vector3d cross_point;
    LaneLineSample* ret = NULL;
    std::shared_ptr<LaneLineSample> flsr = std::make_shared<LaneLineSample>();
    auto v_pos = alg::get_vertical_pos(lc->pos, lc->dir, 50);  // lc车道线中线点方向垂线上50m处的点
    // auto is_print=[](Eigen::Vector3d p1,Eigen::Vector3d p2)
    // {
    //     return alg::calc_dis(p1,p2)<8;
    // };
    for (auto& tar_fls : search_sec) {
        // if(!is_print(tar_fls->pos,{54,59,0}))
        // {
        //     continue;
        // }
        // LOG_INFO("0 SETEP")
        double theta = alg::calc_theta1(tar_fls->dir,  lc->dir); 
        theta=fabs(theta);
        theta=std::min(theta,fabs(180-theta));
        if (theta > theta_threshold) {
            continue;
        }
        // LOG_INFO("1 SETEP")
        if (tar_fls->next == NULL && tar_fls->prev == NULL) {
            continue;
        }
        // LOG_INFO("2 SETEP")
        bool hasValidCrossPoint = false;
        if (tar_fls->next != NULL) {
            hasValidCrossPoint = alg::get_cross_point_for_segment(tar_fls->pos, tar_fls->next->pos, lc->pos, v_pos, cross_point, 1);
        }

        if (!hasValidCrossPoint && tar_fls->prev != NULL) {
            hasValidCrossPoint = alg::get_cross_point_for_segment(tar_fls->prev->pos, tar_fls->pos, lc->pos, v_pos, cross_point, 1);     
        }

        if (!hasValidCrossPoint) {
            continue;
        }
        // LOG_INFO("3 SETEP")
        if (left ^ (alg::judge_left(cross_point, lc->pos, lc->dir) < 0)) {  // left==1,则筛选出左侧的车道线点
            continue;
        }
        // LOG_INFO("4 SETEP")
        double dis = alg::calc_dis(cross_point, lc->pos);
        double gap = fabs(dis - tar_dis);
        if (gap > min_dis_threshold) {  // 在半车道宽度范围内
            continue;
        }
        // LOG_INFO("5 SETEP")
        if (min_dis > gap) { // 选出间距最小的车道线点
            min_dis = gap;
            flsr->init(tar_fls);
            flsr->src_fls = tar_fls;
            flsr->pos = cross_point;
            flsr->dir = lc->dir;
            // LOG_INFO("5 SETEP")
        }
    }
    if (flsr->src_fls == NULL) {
        return NULL;
    }
    session->add_vec(session->lane_line_sample_ptr, flsr, true);
    return flsr.get();
}

// int RoadModelProcMatchLine::merge_match_list(
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

int RoadModelProcMatchLine::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_match_line_save_data_enable) {
        return fsdmap::SUCC;
    }
    double scope_buff = FLAGS_display_scope_buff;
    session->set_display_name("match_line");
    for (auto &lc : session->lane_center_feature_merge_raw) {
        session->debug_pos(lc->pos);
        auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE, "match_line");
        log_ptr->color = {255, 255, 255};
        // log_ptr->add(lc->pos, 1);
        log_ptr->add(lc->get_left(), 1);
        auto &ele = log_ptr->add(lc->get_right(), 1);
        switch (lc->refine_pos_status)
        {
            case 1:
            {
                ele.color = {255, 228, 225}; //粉色
                break;
            }
            case 2:
            {
                ele.color = {0, 0, 255}; //蓝
                break;
            }
            case 3:
            {
                ele.color = {0, 255, 0}; //绿
                break;
            }
            default:
            {
                ele.color = {255,0 , 0}; //红
            }
        }
       
        if (lc->next != NULL) {
            auto &ele = log_ptr->add(lc->next->pos, 1);
        } else {
            auto next_pos = alg::get_hori_pos(lc->pos, lc->dir, 1);
            auto &ele = log_ptr->add(next_pos, 1);
        }
    }
    session->save_debug_info("match_line");


    session->set_display_name("match_line_center");
    auto log = session->add_debug_log(utils::DisplayInfo::POINT, "match_line");
    int i=0;
    for (auto &lc : session->lane_center_feature_merge_raw) {
        
        auto &ele=log->add(lc->pos);
        ele.label.intensity_opt=alg::calc_theta(lc->dir)*57.3;
        //
        auto v1 = alg::get_vertical_pos(lc->pos, lc->dir, 1); 
        auto v2 = alg::get_vertical_pos(lc->pos, lc->dir, -1); 
        //
        auto line_log = session->add_debug_log(utils::DisplayInfo::LINE, "match_line_{}",i++);
        auto &ele1=line_log->add(v1);
        auto &ele2=line_log->add(v2);
        ele1.color={255,0,0};
        ele2.color={255,0,0};
        ele1.label.label=1;
        ele2.label.label=1;
        // 
        auto line_log1 = session->add_debug_log(utils::DisplayInfo::LINE, "match_line1_{}",i++); 
        auto &lele1= line_log1->add(lc->left_pos);
        auto &lele2= line_log1->add(lc->right_pos);
        lele1.color={0,0,255};
        lele2.color={0,0,255};
        lele1.label.label=10;
        lele2.label.label=10;
    }
    session->save_debug_info("match_line_center");

    return fsdmap::SUCC;
}

}
}
