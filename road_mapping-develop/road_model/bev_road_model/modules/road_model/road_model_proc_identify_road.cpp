

#include "road_model_proc_identify_road.h"

namespace fsdmap {
namespace road_model {

DEFINE_bool(identify_road_enable, true, "identify_road_enable");
DEFINE_bool(identify_road_debug_pos_enable, true, "identify_road_debug_enable");
DEFINE_bool(identify_road_save_data_enable, true, "identify_road_save_data_enable");

DEFINE_double(identify_road_same_lane_dis_threshold, 4, "identify_road_same_lane_dis_threshold");
DEFINE_double(identify_road_same_lane_theta_threshold, 10, "identify_road_same_lane_theta_threshold");
DEFINE_double(identify_road_same_lane_max_width_threshold, 1, "identify_road_same_lane_max_width_threshold");
DEFINE_double(identify_road_same_lane_min_width_threshold, 0.3, "identify_road_same_lane_min_width_threshold");
DEFINE_double(identify_road_identify_null_lane_postion, 0.5, "identify_road_identify_null_lane_postion");
DEFINE_double(identify_road_identify_normal_lane_variance_threshold, 30, "identify_road_identify_normal_lane_variance_threshold");
DEFINE_double(identify_road_identify_triangle_lane_variance_threshold, 40, "identify_road_identify_triangle_lane_variance_threshold");
DEFINE_double(identify_road_identify_triangle_lane_a_threshold, 0.1, "identify_road_identify_triangle_lane_a_threshold");
DEFINE_double(identify_road_identify_pos_length, 20, "identify_road_identify_pos_length");
DEFINE_double(identify_road_identify_trangle_length, 50, "identify_road_identify_trangle_length");
DEFINE_double(identify_road_identify_inner_merge_barrier_max_length, 100, "identify_road_identify_inner_merge_barrier_max_length");
DEFINE_double(identify_road_identify_merge_width_threshold, 2, "identify_road_identify_merge_width_threshold");
DEFINE_double(identify_road_identify_merge_width_delta_factor, 1.5, "identify_road_identify_merge_width_delta_factor");
DEFINE_double(identify_road_poss_gap_threshold, 8, "identify_road_poss_gap_threshold");
DEFINE_double(identify_road_get_segment_max_distance, 50, "identify_road_get_segment_max_distance");
DEFINE_double(identify_road_identify_triangle_lane_expect_max_dis, 2, "identify_road_identify_triangle_lane_expect_max_dis");
DEFINE_double(identify_road_identify_triangle_lane_expect_max_value, 10, "identify_road_identify_triangle_lane_expect_max_value");
DEFINE_double(identify_road_get_break_point_max_length, 30, "identify_road_get_break_point_max_length");
DEFINE_double(identify_road_identify_poss_buff_length, 10, "identify_road_identify_poss_buff_length");
DEFINE_double(identify_road_identify_attr_max_length, 30, "identify_road_identify_attr_max_length");
DEFINE_double(identify_road_identify_attr_min_length, 10, "identify_road_identify_attr_min_length");
DEFINE_double(identify_road_cross_walk_gap, 2, "identify_road_cross_walk_gap");
DEFINE_double(identify_road_get_stop_poss_scope, 10, "identify_road_get_stop_poss_scope");
DEFINE_double(identify_road_stop_line_lc_gap, 4, "identify_road_stop_line_lc_gap");
DEFINE_double(identify_road_stop_line_inner_pos, true, "identify_road_stop_line_inner_pos");
DEFINE_double(identify_road_stop_line_last_gap, 20, "identify_road_stop_line_last_gap");
DEFINE_double(identify_road_nearest_fork_radius, 20, "identify_road_nearest_fork_radius");
DEFINE_double(identify_road_nearest_fork_v_radius, 4, "identify_road_nearest_fork_v_radius");
DEFINE_double(identify_road_nearest_fork_max_theta, 20, "identify_road_nearest_fork_max_theta");
DEFINE_double(identify_road_lc_rate_rate_threshold, 0.5, "identify_road_lc_rate_rate_threshold");
// DECLARE_double(filter_feature_cb_overflow_lambda);


fsdmap::process_frame::PROC_STATUS RoadModelProcIdentifyRoad::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_identify_road_debug_pos_enable;
    if (!FLAGS_identify_road_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }

    // 对于每个轨迹点，生成当前轨迹点的车道数据量，并找出变化点
    CHECK_FATAL_PROC(gen_lane_break(session), "gen_lane_break");

    // 识别车道数量变化点，并识别出真的打断点
    CHECK_FATAL_PROC(identify_road_break(session), "identify_attr_road_break");

    // 识别路口的打断
    CHECK_FATAL_PROC(identify_by_stop_line(session), "identify_by_stop_line");

    //路口进入退出断点的打断
    CHECK_FATAL_PROC(identify_by_inter_break_point(session), "identify_by_inter_break_point");

    // 同步周围打断带点
    CHECK_FATAL_PROC(sync_break_pos(session), "sync_break_pos");

    CHECK_FATAL_PROC(gen_sub_road_segment(session), "gen_sub_road_segment");

    // 结果可视化
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcIdentifyRoad::identify_by_stop_line(RoadModelSessionData* session) {
    //这里注释了， 主要想通过路口，去找道路段？ 
    // LOG_INFO("session->intersections 的个数：{}", session->intersections.size())
    // for (auto &inter : session->intersections) {
    //     for (auto &road_segment : inter->in_road_segment) {
    //         LOG_INFO("in_road_segment 的个数：{}", inter->in_road_segment.size())
    //         identify_stop_break(session, road_segment, true); //这个函数空的，
    //     }
    //     for (auto &road_segment : inter->out_road_segment) {
    //         identify_stop_break(session, road_segment, false);
    //     }
    //     // for (auto &road_segment : inter->turn_road_segment) {
    //     //     identify_stop_break(session, road_segment, true);
    //     // }
    // }
    //////////////yxx///////////////////////////////////
    for (auto& road_segment : session->road_segment_list) {
        //1、识别需要被停止线打断的打断点
        for (auto &poss : road_segment->pos_sample_list) {
            session->debug_pos(poss->pos);
            std::vector<RoadObjectInfo*> stopline_objs;
            for (auto &object : poss->object_list)
            {
                if(object->ele_type == 6)
                {
                    stopline_objs.push_back(object);
                }
            }
            if(stopline_objs.size() < 1)
            {
                continue;
            }

            poss->break_status = 3; //poss打断-停止线
            auto v_dir = alg::get_vertical_dir(stopline_objs[0]->dir);
            if (fabs(alg::calc_theta(v_dir, poss->dir, false, true)) > 90) {
                v_dir = -v_dir;
            }
            poss->break_pos.init(stopline_objs[0]->pos, v_dir);
            // LOG_INFO("stopline_pos : {}, {}",stopline_objs[0]->pos[0], stopline_objs[0]->pos[1]);
            // std::cout<<"road_segment:"<< (void *)road_segment << "poss:" << poss->pos[0] << "," << poss->pos[1] << std::endl;
            for (auto &lc : poss->filled_lane_sample) {
                session->debug_pos(lc->pos);
                if (lc->invalid()) {
                    continue;
                }
                // if (lc->break_status > 0) {
                //     continue;
                // }
                lc->break_status = 3; //break_status=3因为停止线打断
                lc->stop_status = 1;//将lane标记为已经被停止线打断
                lc->break_pos = poss->break_pos;
            }
            // poss->merge_next->filter_status = 3; //因为停止线被删除
        }
    }
    return fsdmap::SUCC;
}


int RoadModelProcIdentifyRoad::identify_by_inter_break_point(RoadModelSessionData* session) {
    for (auto &break_point : session->break_point_key_pose_list) {
        if (!break_point || !break_point->key_pose) {
            continue;
        }
        auto poss = break_point->key_pose;
        session->debug_pos(poss->pos);
        
        int status = (break_point->break_status == 0) ? 6 : 7;
        
        poss->break_status = status;
        poss->break_pos.init(poss->pos, poss->dir);

        for (auto &lc : poss->filled_lane_sample) {
            if (!lc || lc->invalid()) {
                continue;
            }
            session->debug_pos(lc->pos);
            lc->break_status = status;
            lc->stop_status = 1;
            lc->break_pos = poss->break_pos;
        }
    }
    
    return fsdmap::SUCC;
}


int RoadModelProcIdentifyRoad::sync_break_pos(RoadModelSessionData* session) {
    std::vector<std::tuple<int, int, bool, bool>> break_pro_list;
    // 1 优先级
    // 2 break_status 1 无前驱后继 2 fork 3 stopline 4 其他
    // 4 同类是否互斥, 1互斥，2不互斥, 
    // 5 是否跨road_index 1 跨，2不跨
    break_pro_list.push_back(std::make_tuple(3, 3, false, true));   // 高优先级，分叉点，跨 road_index
    break_pro_list.push_back(std::make_tuple(2, 2, false, false));  // 中优先级，分叉点，不跨 road_index
    break_pro_list.push_back(std::make_tuple(1, 1, false, false));  // 低优先级，无前驱后继，不跨 road_index
    break_pro_list.push_back(std::make_tuple(1, 4, false, false));  // 低优先级，其他类型，不跨 road_index
    UMAP<int, int> pro_map;

    for (int i = 0; i < break_pro_list.size(); ++i) {
        auto &ele = break_pro_list[i];
        pro_map[std::get<1>(ele)] = std::get<0>(ele);
    }
    std::vector<int> valid_road_index = {0, -1, 1};
    for (int i = 0; i < break_pro_list.size(); ++i) {
        auto &ele = break_pro_list[i];
        auto pro = std::get<0>(ele);
        auto status = std::get<1>(ele);
        auto m_f = std::get<2>(ele);
        auto m_s = std::get<3>(ele);
        for (auto& road_segment : session->road_segment_list) {
            //3、根据打断点将周围scope的所有断点拉齐
            for (auto &poss : road_segment->pos_sample_list) {
                session->debug_pos(poss->pos);
                if (poss->break_status != status) {
                    continue;
                }
                // road_index
                // side
                // prev_next
                for (auto &rc_index : valid_road_index) {
                    fresh_poss_break_status(session, poss, pro_map, rc_index, 1, true, m_f, m_s);
                    fresh_poss_break_status(session, poss, pro_map, rc_index, 2, true, m_f, m_s);
                    fresh_poss_break_status(session, poss, pro_map, rc_index, 1, false, m_f, m_s);
                    fresh_poss_break_status(session, poss, pro_map, rc_index, 2, false, m_f, m_s);
                }
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::fresh_poss_break_status(RoadModelSessionData* session,
        KeyPose* poss, UMAP<int, int> &pro_map, int rc_index, int side_status, 
        bool next, bool m_f, bool m_s) {

    double scope = FLAGS_identify_road_get_stop_poss_scope;
    std::vector<KeyPose*> context_list;
    bool has_lc = false;
    bool has_break_lc = false;
    int pro = 0;
    if (MAP_FIND(pro_map, poss->break_status)) {
        pro = pro_map[poss->break_status];
    }
    for (auto &lc : poss->filled_lane_sample) {
        session->debug_pos(lc->pos);
        if (lc->invalid()) {
            continue;
        }
        if (lc->road_index != rc_index) {
            continue;
        }
        if (lc->side_status != side_status) {
            continue;
        }
        has_lc = true;
        if (lc->break_status != poss->break_status) {
            continue;
        }
        has_break_lc = true;
        break;
    }
    if (has_lc && !has_break_lc && !m_s) {
        return fsdmap::SUCC;
    }

    bool has_next_lc = has_lc;
    if (next) {
        poss->get_context_list(scope, context_list, 1);
    } else {
        poss->get_context_list(scope, context_list, 2);
    }
    for (int j = 1; j < context_list.size(); ++j) {
        int index = next ? j : context_list.size() - 1 - j;
        auto &context_poss = context_list[index];
        session->debug_pos(context_poss->pos);
        if (has_next_lc && context_poss->break_status == 0) {
            continue;
        }
        int valid_lc_pro = 0;
        int valid_lc_break_status = 0;
        for (auto &lc : context_poss->filled_lane_sample) {
            session->debug_pos(lc->pos);
            if (lc->invalid()) {
                continue;
            }
            if (lc->road_index != rc_index) {
                continue;
            }
            if (lc->side_status != side_status) {
                continue;
            }
            if (lc->break_status == 0) {
                continue;
            }
            if (MAP_NOT_FIND(pro_map, lc->break_status)) {
                continue;
            }
            int lc_pro = pro_map[lc->break_status];
            if (valid_lc_pro < lc_pro) {
                valid_lc_pro = lc_pro;
                valid_lc_break_status = lc->break_status;
            }
        }
        if (pro < valid_lc_pro) {
                // 优先级高的
            break;
        } else if (pro == valid_lc_pro && !m_f) {
            // 互斥的
            break;
        }
        bool has_valid_lc = false;
        for (auto &lc : context_poss->filled_lane_sample) {
            session->debug_pos(lc->pos);
            if (lc->invalid()) {
                continue;
            }
            // if (!has_next_lc || !m_s) {
            // if (!has_next_lc) {
                if (lc->road_index != rc_index) {
                    continue;
                }
                if (lc->side_status != side_status) {
                    continue;
                }
            // }
            if (!has_next_lc) {
                lc->break_status = poss->break_status;
                lc->break_pos = poss->break_pos;
                if (poss->break_status == 3) {
                    lc->stop_status = 1;//将lane标记为已经被停止线打断
                }
                context_poss->break_pos = poss->break_pos;
                // context_poss->break_status = 0;
                // 复制boudnary
                // context_poss->boundary = poss->boundary;
                has_valid_lc = true;
            } else {
                // 置空
                lc->break_status = 0;
                // context_poss->break_status = 0;
            }
        }
        if (has_valid_lc) {
            has_next_lc = true;
        }
    }
    return fsdmap::SUCC;
}

// bool RoadModelProcIdentifyRoad::match_stop_and_lane(RoadModelSessionData* session,
//                                                      LaneCenterFeature* lc, 
//                                                      KeyPose* poss, 
//                                                      bool front) {
//     // 遍历 KeyPose 中的所有停车线（stop_line_feature_list）
//     for (auto &stop_line : poss->stop_line_feature_list) {
//         // 检查停车线是否无效，如果无效，则跳过当前循环，继续下一个停车线
//         if (stop_line->invalid()) {
//             continue;
//         }

//         // 如果当前停车线与车道中心特征（LaneCenterFeature）不匹配，则跳过当前停车线
//         if (!match_stop_line(lc, stop_line, front)) {
//             continue;
//         }

//         // 如果停车线与车道匹配，设置停车状态为 1（表示匹配成功）
//         lc->stop_status = 1;

//         // 获取停车线的垂直方向（perpendicular direction）
//         auto v_dir = alg::get_vertical_dir(stop_line->dir);

//         // 计算停车线方向与当前位置方向（poss->dir）的夹角，如果夹角大于 90 度，取反垂直方向
//         if (alg::calc_theta(v_dir, poss->dir) > 90) {
//             v_dir = -v_dir;
//         }

//         // 根据标志 FLAGS_identify_road_stop_line_inner_pos 判断是否要计算停车线的内位置
//         if (FLAGS_identify_road_stop_line_inner_pos) {
//             // 计算停车线相对于车道的“内位置”：
//             // 停车线的位置（stop_line->pos）根据车的方向（-poss->dir）和停车线宽度的一半调整位置，获取横向位置
//             Eigen::Vector3d inner_pos;
//             inner_pos = alg::get_hori_pos(stop_line->pos, -poss->dir, stop_line->width / 2);
//             // auto issue = session->add_debug_issue(inner_pos, "inner_pos");

//             // 将计算出的内位置和垂直方向传递给车道中心特征的 break_pos 变量
//             lc->break_pos.init(inner_pos, v_dir);
//         } else {
//             // 如果不需要计算内位置，直接使用停车线的位置（stop_line->pos）和垂直方向（v_dir）来初始化 break_pos
//             lc->break_pos.init(stop_line->pos, v_dir);
//         }

//         // 更新当前停车线的停车状态为 1（表示匹配到停车线）
//         stop_line->stop_status = 1;
//     }

//     // 如果车道中心特征的 stop_status 大于 0，表示找到了匹配的停车线，返回 true
//     // 否则，表示没有找到匹配的停车线，返回 false
//     return lc->stop_status > 0;
// }


bool RoadModelProcIdentifyRoad::match_walk_and_lane(RoadModelSessionData* session,
        LaneCenterFeature* lc, KeyPose* poss, bool front) {
    // double cross_gap = FLAGS_identify_road_cross_walk_gap;
    // if (!front) {
    //     cross_gap = -cross_gap;
    // }
    // for (auto &walk : poss->cross_walk_feature_list) {
    //     if (walk->invalid()) {
    //         continue;
    //     }
    //     if (!match_stop_line(lc, walk, front)) {
    //         continue;
    //     }
    //     lc->stop_status = 2;
    //     Eigen::Vector3d stop_pos = walk->pos;
    //     auto v_dir = walk->dir;
    //     if (alg::calc_theta(v_dir, poss->dir) > 90) {
    //         v_dir = -v_dir;
    //     }
    //     if (front) {
    //         stop_pos = alg::get_hori_pos(walk->next->pos, v_dir, cross_gap);
    //     } else {
    //         stop_pos = alg::get_hori_pos(walk->pos, v_dir, cross_gap);
    //     }
    //     lc->break_pos.init(stop_pos, walk->dir);
    //     walk->stop_status = 2;
    // }
    return lc->stop_status > 0;
}

bool RoadModelProcIdentifyRoad::match_stop_line(LaneCenterFeature* lc, Feature* fp, bool front) {
    // 获取上次计算的间隙距离，默认为某个常量值
    // double last_gap = FLAGS_identify_road_stop_line_last_gap;

    // 根据前后方向调整间隙的符号
    // if (!front) {
    //     last_gap = -last_gap;
    // }

    // 获取车道中心线的左边、右边和中心位置
    // Eigen::Vector3d left_pos = lc->get_left();
    // Eigen::Vector3d right_pos = lc->get_right();
    // Eigen::Vector3d center_pos = lc->pos;

    // 计算根据间隙推算出的下一个位置
    // Eigen::Vector3d next_left_pos = alg::get_hori_pos(left_pos, lc->dir, last_gap);
    // Eigen::Vector3d next_right_pos = alg::get_hori_pos(right_pos, lc->dir, last_gap);
    // Eigen::Vector3d next_center_pos = alg::get_hori_pos(center_pos, lc->dir, last_gap);

    // 如果是前方，使用下一个车道的位置信息；如果是后方，则使用上一个车道的位置信息
    // if (front && lc->fill_next != NULL) {
    //     next_left_pos = lc->fill_next->get_left();
    //     next_right_pos = lc->fill_next->get_right();
    //     next_center_pos = lc->fill_next->pos;
    // } else if (!front && lc->fill_prev != NULL) {
    //     next_left_pos = lc->fill_prev->get_left();
    //     next_right_pos = lc->fill_prev->get_right();
    //     next_center_pos = lc->fill_prev->pos;
    // }

    // 存储交点计算的结果
    // Eigen::Vector3d cross_ret;

    // 判断车道的左边、右边或中心线是否与停止线相交
    // bool match_left = alg::get_cross_point_for_segment(fp->info->start_pos, fp->info->end_pos, left_pos, next_left_pos, cross_ret, true);
    // bool match_right = alg::get_cross_point_for_segment(fp->info->start_pos, fp->info->end_pos, right_pos, next_right_pos, cross_ret, true);
    // bool match_center = alg::get_cross_point_for_segment(fp->info->start_pos, fp->info->end_pos, center_pos, next_center_pos, cross_ret, true);

    // 如果任意一条边与停止线相交，返回 true，表示匹配成功
    // if (match_left || match_right || match_center) {
    //     return true;
    // }

    // 如果没有找到匹配的交点，返回 false
    return false;
}


int RoadModelProcIdentifyRoad::identify_road_break(RoadModelSessionData* session) {
    // 识别渐变区，防止后续按照标准道路被优化
    for (auto& road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            session->debug_pos(poss->pos);
            for (auto &lc : poss->filled_lane_sample) {
                if (lc->invalid()) {
                    continue;
                }
                if (lc->identify_status != 1) {
                    continue;
                }
                if (lc->break_status > 0) {
                    continue;
                }
                DLOG_POINT(lc->pos, "break lc");
                // 识别右侧匝道区
                if (identify_fork_entrance(session, road_segment, poss, lc, true)) {
                    DLOG_POINT(lc->pos, "outside merge entrance lc");
                }

                if (identify_fork_entrance(session, road_segment, poss, lc, false)) {
                    DLOG_POINT(lc->pos, "outside merge entrance lc");
                }
            }
        }
        for (auto &poss : road_segment->pos_sample_list) {
            session->debug_pos(poss->pos);
            for (auto &lc : poss->filled_lane_sample) {
                session->debug_pos(lc->pos);
                if (lc->invalid()) {
                    continue;
                }
                if (lc->identify_status != 1) {
                    continue;
                }
                if (lc->break_status > 0) {
                    continue;
                }
                if (identify_lane_num_different(session, road_segment, poss, lc, true)) {
                    DLOG_POINT(lc->pos, "identify_lane_num_different");
                }

                if (identify_lane_num_different(session, road_segment, poss, lc, false)) {
                    DLOG_POINT(lc->pos, "identify_lane_num_different");
                }
            }
        }
        for (auto &poss : road_segment->pos_sample_list) {
            session->debug_pos(poss->pos);
            for (auto &lc : poss->filled_lane_sample) {
                session->debug_pos(lc->pos);
                if (lc->invalid()) {
                    continue;
                }
                if (lc->identify_status != 1) {
                    continue;
                }
                if (lc->break_status > 0) {
                    continue;
                }
                lc->break_status = 4;
                lc->break_pos.init(lc->pos, poss->dir);
                poss->break_status = 4;
                poss->break_pos = lc->break_pos;
            }
        }
    }
    return fsdmap::SUCC;
}

bool RoadModelProcIdentifyRoad::judge_has_break_around(RoadModelSessionData* session,
        KeyPose *poss, int road_index) {
    double poss_buff_length = FLAGS_identify_road_identify_poss_buff_length;
    double total_length = 0;
    auto prev_poss = poss;
    while (prev_poss != NULL && prev_poss->prev != NULL) {
        total_length += alg::calc_dis(prev_poss->pos, prev_poss->prev->pos);
        if (total_length > poss_buff_length) {
            break;
        }
        for (auto &lc : prev_poss->prev->filled_lane_sample) {
            if (lc->invalid()) {
                continue;
            }
            session->debug_pos(lc->pos);
            if (lc->road_index != road_index) {
                continue;
            }
            if (lc->break_status >= 1) {
                return true;
            }
        }
        prev_poss = prev_poss->prev;
    }
    total_length = 0;
    auto next_poss = poss;
    while (next_poss != NULL && next_poss->next != NULL) {
        total_length += alg::calc_dis(next_poss->pos, next_poss->next->pos);
        if (total_length > poss_buff_length) {
            break;
        }
        for (auto &lc : next_poss->next->filled_lane_sample) {
            if (lc->invalid()) {
                continue;
            }
            if (lc->road_index != road_index) {
                continue;
            }
            if (lc->break_status >= 1) {
                return true;
            }
        }
        next_poss = next_poss->next;
    }
    return false;
}

bool RoadModelProcIdentifyRoad::identify_lane_num_different(RoadModelSessionData* session,
		RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc, bool next) {
    auto fill_next = next ? lc->fill_next : lc->fill_prev;
    if (fill_next != NULL) {
        return false;
    }
    lc->break_status = 1;
    lc->break_pos.init(lc->pos, poss->dir);
    poss->break_status = 1;
    poss->break_pos = lc->break_pos;
    return false;
}

bool RoadModelProcIdentifyRoad::identify_inner_merge_entrance(RoadModelSessionData* session,
         RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc) {
    //特点，后面没有lane，前面三角区, 左右和反过来一样
    // 生成前后串联lane sample list
    double max_length = FLAGS_identify_road_identify_pos_length;
    double width_threshold = FLAGS_identify_road_identify_merge_width_threshold;
    double width_factor = FLAGS_identify_road_identify_merge_width_delta_factor;
    std::vector<LaneCenterFeature*> front_lc_list;
    std::vector<LaneCenterFeature*> back_lc_list;
    match_lane_sample_list(session, road_segment, poss, lc, true, max_length, front_lc_list);
    match_lane_sample_list(session, road_segment, poss, lc, false, max_length, back_lc_list);
    bool front_is_null = identify_null_lane(front_lc_list);
    bool back_is_null = identify_null_lane(back_lc_list);
    if (front_is_null && back_is_null) {
        // 前后都没有道路的不是
       return false;
    }
    // 判断宽度
    if (lc->width > width_threshold) {
        return false;
    }
    double front_expect_vertex = 0;
    double back_expect_vertex = 0;
    front_lc_list.insert(front_lc_list.begin(), lc);
    back_lc_list.insert(back_lc_list.begin(), lc);
    bool front_is_triangle = identify_triangle_lane(front_lc_list, front_expect_vertex);
    bool back_is_triangle = identify_triangle_lane(back_lc_list, back_expect_vertex);
    if (front_is_triangle && back_is_null) {
        // 需要判断最前面的lc
        for (auto &lc_next : back_lc_list) {
            if (lc_next != NULL 
                    && lc_next->match_level.lane_type > 1
                    && lc_next->width < lc->width) {
                return false;
            }
        }
        // 判断最大宽度
        double max_width = 0;
        for (auto &lc_next : front_lc_list) {
            if (lc_next != NULL) {
                max_width = std::max(max_width, lc_next->width);
            }
        }
        if (max_width < width_factor * lc->width) {
            return false;
        }

        // 标记三角区识别成功，防止后面过滤掉
        lc->identify_status = 3;
        // lc->scenes.divsion = 1;
        // lc->scenes.out_in = 1;
        for (auto &other_lc : front_lc_list) {
            if (other_lc != NULL) {
                other_lc->identify_status = 3;
            }
        }
        lc->break_status = 1;
        lc->break_pos.init(poss->pos, poss->dir);
        poss->break_pos = lc->break_pos;

        poss->break_status = 1;
        // auto break_poss = get_break_point(session, poss, lc, front_expect_vertex);
        // if (break_poss->prev != NULL) {
        //     // 汇出口在三角区的顶点前一个打断，打断点位于分段最后一个poss
        //     // 如果prev为NULL则不需要打断
        //     break_poss->prev->break_status = 1;
        //     break_poss->prev->lane_param.break_pos = break_poss->lane_param.break_pos;
        //     break_poss->prev->lane_param.has_break_pos = true;
        // }
        return true;
    }
    if (back_is_triangle && front_is_null) {
        for (auto &lc_next : front_lc_list) {
            if (lc_next != NULL 
                    && lc_next->match_level.lane_type > 1
                    && lc_next->width < lc->width) {
                return false;
            }
        }
        // 判断最大宽度
        double max_width = 0;
        for (auto &lc_next : back_lc_list) {
            if (lc_next != NULL) {
                max_width = std::max(max_width, lc_next->width);
            }
        }
        if (max_width < width_factor * lc->width) {
            return false;
        }

        // 标记三角区识别成功，防止后面过滤掉
        lc->identify_status = 3;
        // lc->scenes.divsion = 1;
        // lc->scenes.out_in = 2;
        for (auto &other_lc : back_lc_list) {
            if (other_lc != NULL) {
                other_lc->identify_status = 3;
            }
        }
        lc->break_status = 1;
        lc->break_pos.init(poss->pos, poss->dir);
        poss->break_pos = lc->break_pos;

        poss->break_status = 1;
        
        // auto break_poss = get_break_point(session, poss, lc, -back_expect_vertex);
        // break_poss->break_status = 1;
        // if (break_poss->next != NULL) {
        //     break_poss->next->lane_param.break_pos = break_poss->lane_param.break_pos;
        //     break_poss->next->lane_param.has_break_pos = true;
        // }
        return true;
    }
    return false;
}

bool RoadModelProcIdentifyRoad::identify_gradual_change(RoadModelSessionData* session,
         RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc) {
    // 特点，前后道路宽度不一致且能够拟合直线，并且方差在一定范围
    double max_length = FLAGS_identify_road_identify_trangle_length;
    std::vector<LaneCenterFeature*> front_lc_list;
    std::vector<LaneCenterFeature*> back_lc_list;
    front_lc_list.push_back(lc);
    back_lc_list.push_back(lc);
    match_lane_sample_list(session, road_segment, poss, lc, true, max_length, front_lc_list);
    match_lane_sample_list(session, road_segment, poss, lc, false, max_length, back_lc_list);
    
    double front_expect_vertex = 0;
    double back_expect_vertex = 0;
    bool front_is_triangle = identify_triangle_lane(front_lc_list, front_expect_vertex, false);
    bool back_is_triangle = identify_triangle_lane(back_lc_list, back_expect_vertex, false);
    if (front_is_triangle || back_is_triangle) {
        lc->match_level.lane_type = 4;
        return true;
    }
    return false;
}

bool RoadModelProcIdentifyRoad::identify_fork_entrance(RoadModelSessionData* session,
         RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc, bool next) {
    auto fill_next = next ? lc->fill_next : lc->fill_prev;
    if (fill_next != NULL) {
        return false;
    }
    auto &next_list = next ? lc->context.all_next : lc->context.all_prev;
    if (next_list.size() == 0) {
        return false;
    }
    lc->identify_status = 2;
    auto &next_lc = next_list.front().src;
    next_lc->break_status = 2;
    next_lc->break_pos.init(next_lc->pos, poss->dir);
    poss->break_status = 2;
    poss->break_pos = next_lc->break_pos;
    return true;
}

int RoadModelProcIdentifyRoad::match_lane_sample_list(RoadModelSessionData* session,
         RoadSegment *road_segment,
         KeyPose *poss,
         LaneCenterFeature *lc,
         bool is_front,
         double max_length,
         std::vector<LaneCenterFeature*> &lc_list) {
    // 生成前后串联lane sample 的宽度
    double total_length = 0;
    KeyPose* front_poss = NULL;
    if (is_front) {
        front_poss = poss->next;
    } else {
        front_poss = poss->prev;
    }
    LaneCenterFeature* prev_lc = lc;
    while (front_poss != NULL) {
        Eigen::Vector3d prev_pos;
        if (is_front) {
            prev_pos = front_poss->prev->pos;
        } else {
            prev_pos = front_poss->next->pos;
        }
        total_length += alg::calc_dis(front_poss->pos, prev_pos, true);
        if (total_length > max_length) {
            break;
        }
        LaneCenterFeature* matched_lc = NULL;
        for (auto& lc_get : front_poss->filled_lane_sample) {
            if (lc_get->invalid()) {
                continue;
            }
            if (is_front && judge_same_lane_line(session, prev_lc, lc_get)) {
                matched_lc = lc_get;
                prev_lc = lc_get;
                break;
            } else if (!is_front && judge_same_lane_line(session, lc_get, prev_lc)) {
                matched_lc = lc_get;
                prev_lc = lc_get;
                break;
            }
        }
        lc_list.push_back(matched_lc);
        if (is_front) {
            front_poss = front_poss->next;
        } else {
            front_poss = front_poss->prev;
        }
    }
    return fsdmap::SUCC;
}

bool RoadModelProcIdentifyRoad::judge_same_lane_line(RoadModelSessionData* session,
        LaneCenterFeature* lc_prev, LaneCenterFeature *lc) {
    if (lc_prev->fill_next == lc) {
        return true;
    }
    if (lc_prev->fill_next != NULL && lc_prev->fill_next != lc) {
        return false;
    }
    if (lc->fill_prev != NULL && lc->fill_prev != lc_prev) {
        return false;
    }
    // 中心点方向夹角，距离，宽度
    double dis_threshold = FLAGS_identify_road_same_lane_dis_threshold;
    double theta_threshold = FLAGS_identify_road_same_lane_theta_threshold;
    double max_width_threshold = FLAGS_identify_road_same_lane_max_width_threshold;
    double min_width_threshold = FLAGS_identify_road_same_lane_min_width_threshold;
    double dis = alg::calc_dis(lc_prev->pos, lc->pos);

    double gap_length = alg::calc_hori_dis(lc->pos, lc_prev->pos, lc_prev->dir);
    gap_length = gap_length > 2 ? gap_length - 2 : 0;
    dis_threshold += gap_length;
    Eigen::Vector3d calc_dir = lc_prev->dir;
    calc_dir.normalize();
    Eigen::Vector3d calc_pos = lc_prev->pos + gap_length * calc_dir;
    Eigen::Vector3d dir;
    alg::calc_dir(calc_pos, lc->pos, dir);
    double theta_1 = alg::calc_theta(lc->dir, dir, true);
    double theta_2 = alg::calc_theta(lc_prev->dir, dir, true);
    double theta_avg = (theta_1 + theta_2) / 2;
    double width_gap = lc->width - lc_prev->width;
    bool has_trangle = lc->match_level.lane_type != 1 ? true : false;
    has_trangle = has_trangle || lc_prev->match_level.lane_type != 1 ? true : false;
    int ret = 0;
    do {
        if (dis > dis_threshold) {
            break;
        }
        if (theta_avg > theta_threshold) {
            break;
		}
		if (has_trangle) {
            // 三角尖角对应的只能更窄
            if ((lc_prev->match_level.join_out == 2 || lc->match_level.join_out == 2) 
                    && width_gap > min_width_threshold) {
                break;
            } 
            if ((lc_prev->match_level.join_out == 1 || lc->match_level.join_out == 1) 
                    && width_gap < -min_width_threshold) {
                break;
            }
		} else if (fabs(width_gap) > max_width_threshold) {
			break;
		}
        ret = 1;
    } while (0);
    DLOG_POINT2(lc->pos, lc_prev->pos,
            "is samle line[ret={}, dis={}, theta_avg={}, width_gap={}]",
            ret, dis, theta_avg, width_gap);
    return ret == 1;
}

bool RoadModelProcIdentifyRoad::identify_normal_lane(std::vector<LaneCenterFeature*> &lc_list) {
    double variance_threshold = FLAGS_identify_road_identify_normal_lane_variance_threshold;
    std::vector<double> width_list;
    for (auto &lc : lc_list) {
        if (lc == NULL) {
             width_list.push_back(0);
             continue;
        }
        width_list.push_back(lc->width);
    }
    double variance = alg::calc_variance(width_list);
    if (variance > variance_threshold) {
        return false;
    }
    return true;
}

bool RoadModelProcIdentifyRoad::identify_null_lane(std::vector<LaneCenterFeature*> &lc_list) {
    double position = FLAGS_identify_road_identify_null_lane_postion;
    std::vector<double> width_list;
    for (auto &lc : lc_list) {
        if (lc == NULL) {
             width_list.push_back(0);
             continue;
        }
        width_list.push_back(lc->width);
    }
    double media = alg::calc_median(width_list, 1, position);
    if (media == 0) {
        return true;
    }
    return false;
}

bool RoadModelProcIdentifyRoad::identify_triangle_lane(std::vector<LaneCenterFeature*> &lc_list, 
        double &expect_vertex, bool need_asc) {
    double a_threshold = FLAGS_identify_road_identify_triangle_lane_a_threshold;
    double variance_threshold = FLAGS_identify_road_identify_triangle_lane_variance_threshold;
    double expect_max_dis = FLAGS_identify_road_identify_triangle_lane_expect_max_dis;
    double expect_max_value = FLAGS_identify_road_identify_triangle_lane_expect_max_value;
    std::vector<Eigen::Vector3d> width_list;
    double total_length = 0;
    int32_t near_dis = 0;
    LaneCenterFeature *prev_lc = NULL;
    for (auto &lc : lc_list) {
        if (lc == NULL) {
             continue;
        }
        if (prev_lc != NULL) {
            total_length += alg::calc_dis(lc->pos, prev_lc->pos);
        }
        width_list.resize(width_list.size() + 1);
        auto &back = width_list.back();
        back.x() = total_length;
        back.y() = lc->width;
        prev_lc = lc;
    }
    float a = 0;
    float b = 0;
    if (!alg::fit_line3d(width_list, a, b)) {
        return false;
    }
    a = need_asc ? a : fabs(a);
    if (a < a_threshold) {
        return false;
    }

    std::vector<double> gap_list;
    gap_list.reserve(width_list.size());
    for (int32_t i = 0; i < width_list.size(); ++i) {
        auto &pt = width_list[i];
        double y =  pt.x() * a + b;
        double gap = y - pt.y();
        gap_list.push_back(gap);
    }
    double variance = alg::calc_variance(gap_list);
    if (variance > variance_threshold) {
        return false;
    }
    float expect_a = 0;
    float expect_b = 0;
    if (expect_max_dis > 0 && alg::fit_line3d(width_list, expect_a, expect_b, 0, expect_max_dis)) {
        expect_vertex = - expect_b / expect_a;
    } else {
        expect_vertex = - b / a;
    }
    return true;
}

int RoadModelProcIdentifyRoad::gen_lane_break(RoadModelSessionData* session) {
    for (auto& road_segment : session->road_segment_list) {
        for (int32_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            auto &boundary = poss->boundary;
            if (poss->invalid()) {
                continue;
            }
            // 轨迹数量不一致
            for (auto &lc : poss->filled_lane_sample) {
                session->debug_pos(lc->pos);
                if (lc->invalid()) {
                    continue;
                }
                if (lc->fill_next == NULL || lc->fill_prev == NULL) {
                    lc->identify_status = 1;
                }
                if (lc->fill_next != NULL && lc->fill_next->road_index != lc->road_index) {
                    lc->identify_status = 1;
                }
                if (lc->fill_prev != NULL && lc->fill_prev->road_index != lc->road_index) {
                    lc->identify_status = 1;
                }
                if (lc->fill_next != NULL && lc->fill_next->side_status != lc->side_status) {
                    lc->identify_status = 1;
                }
                if (lc->fill_prev != NULL && lc->fill_prev->side_status != lc->side_status) {
                    lc->identify_status = 1;
                }
                if (lc->fill_next != NULL && lc->fill_next->invalid()) {
                    lc->identify_status = 1;
                }
                if (lc->fill_prev != NULL && lc->fill_prev->invalid()) {
                    lc->identify_status = 1;
                }
                DLOG_POINT(lc->pos, "lane break");
            }
        }
    }
    return fsdmap::SUCC;
}

Feature* RoadModelProcIdentifyRoad::seach_neares_fork(RoadModelSessionData* session,
		LaneCenterFeature* lc, double expect_dis) {
	// double radius = FLAGS_identify_road_nearest_fork_radius;
	// double v_radius = FLAGS_identify_road_nearest_fork_v_radius;
	// double max_theta = FLAGS_identify_road_nearest_fork_max_theta;
    // Eigen::Vector3d dir = lc->dir;
    // dir.normalize();
	// auto pos = lc->pos + dir * expect_dis;
	// // alg::trans_pos2local(pos);
	// float minb[3] = {pos.x() - radius, pos.y() - radius, pos.z() - 1};
	// float maxb[3] = {pos.x() + radius, pos.y() + radius, pos.z() + 1};
	// std::vector<Feature*> search_sec;
	// session->object_feature_tree.Search(minb, maxb, SearchResultCallback<Feature>, &search_sec);
	// double min_dis = 10000;
	// Feature* min_fork = NULL;
	// for (auto& ele : search_sec) {
	// 	if (ele->info->id.type() != fsdmap::TRT_LINE_LINE3D) {
	// 		continue;
	// 	}
	// 	if (ele->info->fork_type != 1) {
	// 		continue;
	// 	}
	// 	double dis = alg::calc_dis(ele->pos, pos);
	// 	if (dis > radius) {
	// 		continue;
	// 	}
	// 	double v_dis = alg::calc_vertical_dis(ele->pos, pos, lc->dir);
	// 	if (v_dis > v_radius) {
	// 		continue;
	// 	}
	// 	//double theta = alg::calc_theta(ele->dir, lc->dir);
	// 	//if (theta > max_theta) {
	// 	//	continue;
	// 	// }
	// 	if (dis < min_dis) {
	// 		min_fork = ele;
	// 		min_dis = dis;
	// 	}
	// }
	// return min_fork;
    return NULL;
}

int RoadModelProcIdentifyRoad::gen_sub_road_segment(RoadModelSessionData* session) {
    double rate_threshold = FLAGS_identify_road_lc_rate_rate_threshold;
    std::vector<int> valid_road_index = {-1, 0, 1};
    for (auto &rc_index : valid_road_index) {
        for (auto &road_segment : session->road_segment_list) {
            road_segment->road_type[rc_index] = 1;
            for (int i = 0; i < 2; ++i) {
                bool left = i == 1;
                int64_t start_index = 0;
                BreakPos start_break_pos;
                BreakPos end_break_pos;
                bool valid_start = true;
                SubRoadSegment* prev_srs = NULL;
                int64_t oppo_poss_num = 0;
                int64_t valid_poss_num = 0;
                int64_t oppo_pos_num = 0;
                for (int j = 0; j < road_segment->pos_sample_list.size(); ++j) {
                    auto &poss = road_segment->pos_sample_list[j];
                    session->debug_pos(poss->pos);
                    bool has_break = false; 
                    if (valid_start && poss->invalid()){
                        start_index = j + 1;
                        continue;
                    }
                    auto &boundary = poss->boundary;
                    if (valid_start 
                            && (rc_index == -1 || left)
                            && boundary.left_oppo_pos.src != NULL
                            && !boundary.left_oppo_pos.src->invalid()) {
                        start_index = j + 1;
                        continue;
                    }

                    if (valid_start && j == start_index) {
                        start_break_pos.init(poss->pos, poss->dir);
                    }
                    if (poss->invalid()) 
                    {
                        has_break = true;
                        valid_start = true;
                        prev_srs = NULL;
                        end_break_pos.init(poss->pos, poss->dir);
                    } else if ((rc_index == -1 || left)
                            && boundary.left_oppo_pos.src != NULL
                            && !boundary.left_oppo_pos.src->invalid()) {
                        has_break = true;
                        valid_start = true;
                        prev_srs = NULL;
                        end_break_pos.init(poss->pos, poss->dir);
                    } else {
                        bool has_lc = false;
                        bool has_oppo_lc = false;
                        for (auto &lc : poss->filled_lane_sample) {
                            session->debug_pos(lc->pos);
                            if (lc->invalid()) {
                                continue;
                            }
                            if (lc->road_index != rc_index) {
                                continue;
                            }
                            if ((left ^ (lc->side_status == 2)) && i == 0) {
                                has_oppo_lc = true; 
                                continue;
                            }
                            has_lc = true;
                            if (lc->break_status <= 0) {
                                continue;
                            }
                            if (j == start_index) {
                                start_break_pos = lc->break_pos;
                                // LOG_INFO("start_break_pos : {}, {}",start_break_pos.pos[0], start_break_pos.pos[1]);
                                continue;
                            }
                            
                            end_break_pos = lc->break_pos;
                            // LOG_INFO("end_break_pos : {}, {}",end_break_pos.pos[0], end_break_pos.pos[1]);

                            end_break_pos.src_lc = lc;
                            has_break = true;
                            break;
                        }
                        if (has_lc) {
                            poss->lc_num_map[rc_index] = 1;
                            ++valid_poss_num;
                            valid_start = false;
                            if (boundary.left_oppo_pos.src != NULL 
                                    && !boundary.left_oppo_pos.src->invalid()) {
                                oppo_pos_num++;
                            }
                        } else {
                            if (valid_start) {
                                start_index = j + 1;
                                continue;
                            }
                            has_break = true;
                            end_break_pos.init(poss->pos, poss->dir);
                        }
                        if (has_oppo_lc) {
                            ++oppo_poss_num;
                        }
                    }
                    if (!has_break && j == road_segment->pos_sample_list.size() - 1) {
                        end_break_pos.init(poss->pos, poss->dir);
                        has_break = true;
                    }
                    if (has_break) {
                        int64_t size = j - start_index;
                        if (size < 1) {
                            continue;
                        }
                        double length = alg::calc_hori_dis(start_break_pos.pos, end_break_pos.pos, end_break_pos.dir);
                        if (length >= 0.5) {
                            auto new_srs = session->add_ptr(session->sub_road_segment_ptr);
                            new_srs->init(road_segment);
                            new_srs->start_index = start_index;
                            new_srs->end_index = j;
                            new_srs->road_index = rc_index;
                            new_srs->start_break_pos = start_break_pos;
                            // LOG_INFO("new_srs->start_break_pos : {}, {}",start_break_pos.pos[0], start_break_pos.pos[1]);
                            new_srs->end_break_pos = end_break_pos;
                            session->debug_pos(end_break_pos.pos);
                            // LOG_INFO("new_srs->end_break_pos : {}, {}",end_break_pos.pos[0], end_break_pos.pos[1]);
                            new_srs->link_direction = left ? 2 : 1;
                            new_srs->need_left_boundary = true;
                            // double oppo_rate = (double) oppo_poss_num / size;
                            // if (oppo_rate > 0.3) {
                            //     new_srs->need_left_boundary = false;
                            // }
                            if (rc_index == -1) {
                                double oppo_pos_rate = (double) oppo_pos_num / size;
                                if (oppo_pos_rate < 0.2) {
                                    new_srs->need_left_road = true;
                                }
                            }
                            if (prev_srs != NULL) {
                                prev_srs->next = new_srs.get();
                                new_srs->prev = prev_srs;
                            }
                            prev_srs = new_srs.get();

                            session->sub_road_segment_list.push_back(new_srs.get());
                            road_segment->sub_road_segment_list.push_back(new_srs.get());
                        }

                        start_break_pos = end_break_pos;
                        start_index = j;
                    }

                }
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcIdentifyRoad::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_identify_road_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("identify_road");
    // for (auto &road_segment : session->road_segment_list) {
    //     if (road_segment->pos_sample_list.size() == 0) {
    //         continue;
    //     }
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "geo_break");
    //     for (auto &poss : road_segment->pos_sample_list) {
    //         if (poss->break_status == 0) {
    //             continue;
    //         }
    //         auto &elee = log->add(poss->pos);
    //         elee.color = {255, 0, 0};
    //     }
    // }
    int index = 0;
    for (auto &srs : session->sub_road_segment_list) {
        auto &road_segment = srs->src_road_segment;

        ++index;
        if (srs->road_index != 0) {
            continue;
        } 
        {
            auto start_poss = road_segment->pos_sample_list[srs->start_index];
            auto end_poss = road_segment->pos_sample_list[srs->end_index];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "geo_break");
            auto &ele_s = log->add(start_poss->pos);
            ele_s.label.label = index;
            if (start_poss->break_status == 2) {
                ele_s.color = {255, 0, 0};
            }
            auto &ele_e = log->add(end_poss->pos);
            ele_e.label.label = index;
            if (srs->road_index == -1) {
                log->color = {0, 255, 0};
            } else if (srs->road_index == 1) {
                log->color = {0, 255, 255};
            }
            if (end_poss->break_status == 2) {
                ele_s.color = {255, 0, 0};
            }
        }
        // {
        //     auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "geo_break");
        //     auto &ele_s = log->add(srs->start_break_pos.pos);
        //     ele_s.label.label = index;
        //     auto &ele_e = log->add(srs->end_break_pos.pos);
        //     ele_e.label.label = index;
        //     log->color = {255, 255, 0};
        //     if (srs->road_index == -1) {
        //         log->color = {255, 255, 0};
        //     } else if (srs->road_index == 1) {
        //         log->color = {255, 0, 255};
        //     }
        // }
        
    }
    session->save_debug_info("identify_road");

    return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
