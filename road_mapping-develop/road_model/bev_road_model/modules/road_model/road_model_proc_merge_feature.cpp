

#include "road_model_proc_merge_feature.h"

DECLARE_double(display_scope_buff);

DEFINE_bool(merge_feature_enable, true, "merge_feature_enable");
DEFINE_bool(merge_feature_debug_pos_enable, true, "merge_feature_debug_enable");
DEFINE_bool(merge_feature_save_data_enable, true, "merge_feature_save_data_enable");
DEFINE_bool(merge_feature_save_data_frame_log_enable, false, "merge_feature_save_data_frame_log_enable");
DEFINE_bool(merge_feature_save_data_multi_log_enable, true, "merge_feature_save_data_multi_log_enable");

DEFINE_double(merge_feature_vote_yellow_min_threshold, 1.5, "merge_feature_vote_yellow_min_threshold");
DEFINE_double(merge_feature_vote_yellow_max_threshold, 2.5, "merge_feature_vote_yellow_max_threshold");
DEFINE_double(merge_feature_vote_double_value_threshold, 0.7, "merge_feature_vote_double_value_threshold;");
DEFINE_double(merge_feature_vote_yellow_radius, 5, "merge_feature_vote_yellow_radius");
DEFINE_double(merge_feature_vote_yellow_dis_treshold, 1, "merge_feature_vote_yellow_dis_treshold");
DEFINE_double(merge_feature_vote_yellow_scope, 3, "merge_feature_vote_yellow_scope");
DEFINE_double(merge_feature_vote_yellow_dis_lambda, 0.5, "merge_feature_vote_yellow_dis_lambda");
DEFINE_double(merge_feature_vote_yellow_theta_treshold, 30, "merge_feature_vote_yellow_theta_treshold");
DEFINE_double(merge_feature_match_rate_threshold, 0.8, "merge_feature_match_rate_threshold");
DEFINE_double(merge_feature_scope, 1, "merge_feature_scope");
DEFINE_double(merge_feature_search_radius, 5, "merge_feature_search_radius");
DEFINE_double(merge_feature_theta, 20, "merge_feature_theta");
DEFINE_double(merge_feature_proc_min_size, 10, "merge_feature_proc_min_size");
DEFINE_double(merge_feature_proc_min_match_size, 1, "merge_feature_proc_min_match_size");
DEFINE_double(merge_feature_proc_score_threshold, 0.2, "merge_feature_proc_score_threshold");
DEFINE_double(merge_feature_match_ground_radius, 1, "merge_feature_match_ground_radius");
DEFINE_double(merge_feature_mutil_scope, 1, "merge_feature_mutil_scope");
// DEFINE_double(merge_feature_search_radius, 5, "merge_feature_search_radius");
DEFINE_double(merge_feature_mutil_theta, 20, "merge_feature_mutil_theta");
DEFINE_double(merge_feature_theta_thres, 360, "merge_feature_theta_thres");
DEFINE_double(merge_feature_single_h_dis, 30, "merge_feature_single_h_dis");
DEFINE_double(merge_feature_single_v_dis, 30, "merge_feature_single_v_dis");
// DEFINE_double(merge_feature_match_rate_threshold, 0.8, "merge_feature_match_rate_threshold");
// DEFINE_double(merge_feature_iou_thres_1, 0.85, "merge_feature_iou_thres_1");
// DEFINE_double(merge_feature_iou_thres_2, 0.5, "merge_feature_iou_thres_2");
DEFINE_double(merge_feature_valid_min_length, 20, "merge_feature_valid_min_length");
DEFINE_double(merge_feature_match_line_long_scope, 20, "merge_feature_match_line_long_scope");
DEFINE_double(merge_feature_match_line_long_max_dis_threshold, 2, 
        "merge_feature_match_line_long_max_dis_threshold");
DEFINE_double(merge_feature_single_boundary_length_thres, 5, 
        "merge_feature_single_boundary_length_thres");



namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcMergeFeature::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_merge_feature_debug_pos_enable;
    if (!FLAGS_merge_feature_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    // 将车道线、车道边界等的采样点和pose绑定，并根与自车的距离计算分数
    CHECK_FATAL_PROC(match_key_pose(session), "match_key_pose");

    // 单轨迹merge：边界线、车道线、车道中心线
    CHECK_FATAL_PROC(merge_single_trail(session), "merge_single_trail_boundary");

    CHECK_FATAL_PROC(make_tree(session), "make_tree");

    // CHECK_FATAL_PROC(merge_multi_lane_center(session), "merge_multi_lane_center");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info"); // 存储融合结果，用于可视化
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcMergeFeature::match_key_pose(RoadModelSessionData* session) {
    if (session->opt_ground_pcd) {
        for (auto &pt : session->opt_ground_pcd->points) {
            pt.intensity = pt.z;
            pt.z = 0;
        }
        session->ground_kdtree.setInputCloud(session->opt_ground_pcd);
    }
     
    KeyPoseTree poss_tree; // ins数据
    for (auto &it : session->key_pose_map) {
        for (auto &poss : it.second.list) {
            poss_tree.insert(poss->pos, poss);
        }
    }
    for (auto &[frame_id, list] : session->bev_frame_lane_line) { // bev感知车道线的采样结果
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        session->debug_pos(pt->pos);
                        if (!match_ground_by_point(session, pt->pos)) { // 没做什么处理，全部返回true
                            pt->filter_status = 5;
                            return;
                        }
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->src->trail_id, pt->src->key_pose);
                        pt->match_frame_num = ret.first;  // 离pt位置较近（轨迹上存在某点pose，pt到pose的方向向量和方向向量垂线距离都小于30m）的frame数量
                        pt->match_trail_num = ret.second; // 离pt位置较近（轨迹上存在某点pose，pt到pose的方向向量距离小于30m）的轨迹条数
                        // pt->src->score = 1e-6;
                        pt->src->score = 1;
                        // pt->src->score = get_feature_score_by_distance(session, pt->src);
                        // if (pt->src->score <= 0) {
                        //    pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来
                        });
            }
        }
    }
    for (auto &[frame_id, list] : session->bev_frame_lane_center_line) { // bev感知车道中心线的采样结果
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        if (!match_ground_by_point(session, pt->pos)) {
                            pt->filter_status = 5;
                            return;
                        }
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->trail_id, pt->key_pose);
                        pt->match_frame_num = ret.first;
                        pt->match_trail_num = ret.second;
                        // pt->score = 1e-6;
                        pt->score = 1;
                        // pt->score = get_feature_score_by_distance(session, pt);
                        // if (pt->score <= 0) {
                        //    pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来
                        });
            }
        }
    }
    //bev_frame_boundary_line： 集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)，对曲线做平滑后的结果
    for (auto &[frame_id, list] : session->bev_frame_boundary_line) {
        for (auto &line : list) {
            for (auto pt : line->list) {
                session->thread_pool->schedule([&, pt, session, this](
                            utils::ProcessBar *process_bar) {
                        if (!match_ground_by_point(session, pt->pos)) {
                            pt->filter_status = 5;
                            return;
                        }
                        auto ret = search_key_pose(session, poss_tree, pt->pos, pt->dir,
                                pt->trail_id, pt->key_pose);
                        pt->match_frame_num = ret.first;
                        pt->match_trail_num = ret.second;
                        //根据feature离自车位置远近，计算feature的可靠性，离自车越近，可靠性分数越高
                        // pt->score = 1e-6;
                        pt->score = 1;
                        // pt->score = get_feature_score_by_distance(session, pt);
                        // if (pt->score <= 0) {
                        //     pt->filter_status = 4;
                        // } // modified by ccj, 20241221, pt raw_pos目前是相对于地图起点的pos，不再是相对于自车坐标的pos, score计算不出来 
                        });
            }
        }
    }
    session->thread_pool->wait(5, "match_frame");
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::merge_single_trail(RoadModelSessionData* session) {
    // 遍历每条轨迹，单独融合每条轨迹里面的boundary信息
    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_boundary_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_boundary_merge[size={}]", 
    //                     trail_ptr->boundary_line_group_list.size());
    //     });
    // }

    auto lit1 = session->key_pose_map.begin();
    if (lit1 != session->key_pose_map.end()) {
        auto trail_ptr = &lit1->second;
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
            merge_single_boundary_by_trail(session, trail_ptr);
            LOG_INFO("trail_boundary_merge[size={}]",  trail_ptr->boundary_line_group_list.size());

            // 正序
            if(1) {
                // merge multi group line
                // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
                // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
                session->boundary_line_ptr_for_merge.clear();
                session->bev_frame_boundary_line.clear();
                
                for (int64_t i = 0; i < trail_ptr->boundary_line_group_list.size(); ++i) {
                    auto &group_line = trail_ptr->boundary_line_group_list[i];

                    // 1 构造单趟合并的 boundary
                    auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
                    new_line->boundary_type = group_line->boundary_type;

                    // std::shared_ptr<BoundaryGroupLine> group_line;
                    //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
                    for (int64_t j = 0; j < group_line->list.size(); ++j) {
                        auto &pt = group_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }

                        auto new_node = session->add_ptr(session->boundary_feature_ptr);
                        // new_node = pt;
                        // new_node->pose = pt->pos;
                        new_node->init(pt.get());

                        new_line->list.push_back(new_node.get());
                    }

                    if (new_line->list.size() == 0) {
                        continue;
                    }

                    session->bev_frame_boundary_line[""].push_back(new_line.get());


                }
                // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

                // 2. 重新调用 merge single 操作
                merge_single_boundary_by_trail(session, trail_ptr);
            }

            // 反序
            if(1) {
                // merge multi group line
                // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
                // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
                session->boundary_line_ptr_for_merge.clear();
                session->bev_frame_boundary_line.clear();
                
                int lane_boundary_size = trail_ptr->boundary_line_group_list.size();
                for (int64_t i = lane_boundary_size-1; i >= 0; --i) {
                    auto &group_line = trail_ptr->boundary_line_group_list[i];

                    // 1 构造单趟合并的 boundary
                    auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
                    new_line->boundary_type = group_line->boundary_type;

                    // std::shared_ptr<BoundaryGroupLine> group_line;
                    //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
                    for (int64_t j = 0; j < group_line->list.size(); ++j) {
                        auto &pt = group_line->list[j];
                        if(pt == nullptr) {
                            continue;
                        }

                        auto new_node = session->add_ptr(session->boundary_feature_ptr);
                        // new_node = pt;
                        // new_node->pose = pt->pos;
                        new_node->init(pt.get());

                        new_line->list.push_back(new_node.get());
                    }

                    if (new_line->list.size() == 0) {
                        continue;
                    }

                    session->bev_frame_boundary_line[""].push_back(new_line.get());


                }
                // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

                // 2. 重新调用 merge single 操作
                merge_single_boundary_by_trail(session, trail_ptr);
            }

            // save_debug_info_only_boundary(session, 1);
            merge_boundary_duplicate(session, trail_ptr, false);
            merge_boundary_duplicate(session, trail_ptr, true);
        });

    }    
    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_lane_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_lane_merge[size={}]", 
    //                     trail_ptr->lane_line_group_list.size());
    //     });
    // }

    auto lit2 = session->key_pose_map.begin();
    if (lit2 != session->key_pose_map.end()) {
        auto trail_ptr = &lit2->second;
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
                merge_single_lane_by_trail(session, trail_ptr);
                LOG_INFO("trail_lane_merge[size={}]", 
                        trail_ptr->lane_line_group_list.size());
                
                // 正序
                if(1) {
                    // merge multi group line
                    session->lane_line_sample_line_ptr_for_merge.clear();
                    session->bev_frame_lane_line.clear();
                    
                    for (int64_t i = 0; i < trail_ptr->lane_line_group_list.size(); ++i) {
                        auto &group_line = trail_ptr->lane_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);

                        // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
                        //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_by_trail(session, trail_ptr);
                }

                // 反序
                if(1) {
                    // merge multi group line
                    session->lane_line_sample_line_ptr_for_merge.clear();
                    session->bev_frame_lane_line.clear();
                    
                    int lane_line_size = trail_ptr->lane_line_group_list.size();
                    for (int64_t i = lane_line_size-1; i >= 0; --i) {
                        auto &group_line = trail_ptr->lane_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);

                        // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
                        //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_by_trail(session, trail_ptr);
                }

                // save_debug_info_only_lane(session, 1);
                merge_lane_duplicate(session, trail_ptr, false);
                merge_lane_duplicate(session, trail_ptr, true);
        });
    }

    // for (auto &lit : session->key_pose_map) {
    //     auto trail_ptr = &lit.second;
    //     session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
    //             merge_single_lane_center_by_trail(session, trail_ptr);
    //             LOG_INFO("trail_lane_center_merge[size={}]", 
    //                     trail_ptr->lane_center_line_group_list.size());
    //     });
    // }

    auto lit3 = session->key_pose_map.begin();
    if (lit3 != session->key_pose_map.end()) {
        auto trail_ptr = &lit3->second;
        session->thread_pool->schedule([trail_ptr, session, this](utils::ProcessBar *process_bar) {
                merge_single_lane_center_by_trail(session, trail_ptr);
                LOG_INFO("trail_lane_center_merge[size={}]", 
                        trail_ptr->lane_center_line_group_list.size());

                // 正序
                if(1) {
                    // merge multi group line
                    session->lane_center_line_ptr_for_merge.clear();
                    session->bev_frame_lane_center_line.clear();
                    
                    for (int64_t i = 0; i < trail_ptr->lane_center_line_group_list.size(); ++i) {
                        auto &group_line = trail_ptr->lane_center_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);

                        // std::shared_ptr<LaneCenterGroupLine> group_line;
                        //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                            // new_node = pt;
                            // new_node->pose = pt->pos;
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_center_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_center_by_trail(session, trail_ptr);
                }

                // 反序
                if(1) {
                    // merge multi group line
                    session->lane_center_line_ptr_for_merge.clear();
                    session->bev_frame_lane_center_line.clear();
                    
                    int lane_center_size = trail_ptr->lane_center_line_group_list.size();
                    for (int64_t i = lane_center_size-1; i >= 0; --i) {
                        auto &group_line = trail_ptr->lane_center_line_group_list[i];

                        // 1 构造单趟合并的 boundary
                        auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);

                        // std::shared_ptr<LaneCenterGroupLine> group_line;
                        //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
                        for (int64_t j = 0; j < group_line->list.size(); ++j) {
                            int valid_index = j;
                            // if (j == group_line->list.size() - 1) {
                            //     valid_index = j - 1;
                            // }
                            auto &pt = group_line->list[j];
                            if(pt == nullptr) {
                                continue;
                            }

                            auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                            // new_node = pt;
                            // new_node->pose = pt->pos;
                            new_node->init(pt.get());

                            new_line->list.push_back(new_node.get());
                        }

                        if (new_line->list.size() == 0) {
                            continue;
                        }

                        session->bev_frame_lane_center_line[""].push_back(new_line.get());
                    }

                    // 2. 重新调用 merge single 操作
                    merge_single_lane_center_by_trail(session, trail_ptr);
                }

                // save_debug_info_only_lane_center(session, 1);
                merge_lane_center_duplicate(session, trail_ptr, false);
                merge_lane_center_duplicate(session, trail_ptr, true);
        });
    }
    session->thread_pool->wait(5, "merge_single");
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::make_tree(RoadModelSessionData* session) {
    for (auto &lit : session->key_pose_map) {
        auto trail = &lit.second;
        for (int64_t i = 0; i < trail->boundary_line_group_list.size(); ++i) {
            auto &group_line = trail->boundary_line_group_list[i];
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if (pt->invalid()) {
                    prev = NULL;
                    continue;
                }
                pt->group_line = group_line.get();
                // pt->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                session->boundary_line_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
            }
            session->merge_boundary_line_list.push_back(group_line.get()); // 放入每条轨迹融合后的所有boundary曲线
        }
        for (int64_t i = 0; i < trail->lane_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_line_group_list[i];
            LaneLineSample* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                // if (pt->invalid()) {
                //     prev = NULL;
                //     continue;
                // }
                pt->group_line = group_line.get();
                // pt->src->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                session->lane_line_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
                // LOG_INFO("merge_lane_line_list:[{} {}]",pt->attr.color,pt->attr.type);
            }
            session->merge_lane_line_list.push_back(group_line.get());
        }
        for (int64_t i = 0; i < trail->lane_center_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_center_line_group_list[i];
            LaneCenterFeature* prev = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if (pt->invalid()) {
                    prev = NULL;
                    continue;
                }
                pt->group_line = group_line.get();
                // pt->src->src_status = 2;
                if (prev != NULL) {
                    pt->set_prev(prev);
                }
                session->merge_lane_center_sample_tree.insert(pt->pos, pt.get());
                prev = pt.get();
            }
            session->merge_lane_center_list.push_back(group_line.get());
        }
    }
    return fsdmap::SUCC;
}

void RoadModelProcMergeFeature::merge_lane_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->lane_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->lane_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.1);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->lane_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->lane_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[lane] 1 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 
                            << " l1 len: " << curve_overlap.l1_length << " l2 len: " << curve_overlap.l2_length 
                            << std::endl;
                }
                

                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->lane_line_group_list[i];
                // group_line_i.insert(j);
                // auto &group_line_j = trail_ptr->lane_line_group_list[j];
                // group_line_j.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[lane] 2 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
            }
            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->lane_line_group_list[id1]->list.begin(), trail_ptr->lane_line_group_list[id1]->list.end());
            std::shared_ptr<LaneLineSample> prev = NULL;
            for (auto &fls : trail_ptr->lane_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
            }
             if (debug_log && reverse_process == false) {
                std::cout << "[lane] 0#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << curve_overlap_0.l2_id << std::endl;
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                std::cout << "[lane] 1#need_reverse: " << need_reverse << " id1 : " << id1 << " id2: " << id2  << " matched_num: " << matched_num  << " has_processed[id2]:" << has_processed[id2] << std::endl; 
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_line_group_list[id2]->list.begin(), trail_ptr->lane_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneLineSample> prev = NULL;
                    for (auto &fls : trail_ptr->lane_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane] 2#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_line_group_list[id2]->list.begin(), trail_ptr->lane_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneLineSample> prev = NULL;
                    for (auto &fls : trail_ptr->lane_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane] 3#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }
    

    // merge multi group line
    session->lane_line_sample_line_ptr_for_merge.clear();
    session->bev_frame_lane_line.clear();
    
    int lane_size = trail_ptr->lane_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
            //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->lane_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_line_sample_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
            //      std::vector<std::shared_ptr<LaneLineSample>> list;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_line_sample_ptr);
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_line[""].push_back(new_line.get());
        }

    }

    // 2. 重新调用 merge single 操作
    merge_single_lane_by_trail(session, trail_ptr);
}


int RoadModelProcMergeFeature::merge_single_lane_by_trail(RoadModelSessionData* session,
        KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size;
    int min_match_size = FLAGS_merge_feature_proc_min_match_size;
    double score_thres = FLAGS_merge_feature_proc_score_threshold;
    if (trail->list.size() < min_size) {
        return fsdmap::FAIL;
    }
    trail->lane_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        LaneLineSampleTree trail_tree;
        trail->lane_line_group_list.clear();
        int64_t raw_line_size = 0;
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     // if (poss->frame_id == "6231672047094_1672018298180000") {
        //     //     int a = 1;
        //     // }
        //     if (MAP_NOT_FIND(session->bev_frame_lane_line, poss->frame_id)) {
        //         continue;
        //     }
        //     auto &ins_vec = session->bev_frame_lane_line[poss->frame_id];
        LOG_INFO("session->bev_frame_lane_line size: {}", session->bev_frame_lane_line.size());
        for (const auto &ins_pair : session->bev_frame_lane_line) {
            auto &ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) {
                ins->sub_line.clear();
                LaneLineSample* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins; // src表示sub_line的每段都是来自于父曲线ins
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    if (pt->invalid()) {
                        if (new_line->list.size() > 1) {
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->src = ins;
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                        } else {
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    match_frame_line_lane(session, trail->lane_line_group_list, trail_tree, sub_line.get());
                }
            }
            // save_frame_log_lane(session, trail, poss, times);
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_lane_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->lane_line_group_list.size(), raw_line_size);
        // 过滤
        #if 0
        std::vector<LaneLineSample*> match_fls_list;
        for (int64_t i = 0; i < trail->lane_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_line_group_list[i];
            LaneLineSample* prev_fls = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                // session->debug_pos(pt->pos);
                // int match_num = group_line->match_list[j].size();
                int match_num = group_line->match_list_point[valid_index].size();
                // UMAP<std::string, int> match_frame_map;
                // for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                //     match_frame_map[raw_pt->src->frame_id]++;
                // }
                match_fls_list.clear();
                VEC_PUSH_ALL(match_fls_list, group_line->match_list_point[valid_index]);
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map;
                    // match_frame_map[raw_pt->src->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        // match_frame_map[it.second->src->frame_id]++;
                        VEC_PUSH_ALL(match_fls_list, line_match_list[it.second->line_index]);
                    }
                }
                double total_num = match_num + other_match_num;
                pt->score = match_num / (total_num + 1e-6);//1e-6防止除数为0

                // double match_frame_num = match_frame_map.size() > pt->match_frame_num ? 
                //     pt->match_frame_num : match_frame_map.size();
                // pt->score = match_frame_num / pt->match_frame_num;
                // if (total_num < min_match_size) {
                //     pt->filter_status = 2;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 2;
                //     }
                // }
                // if (pt->score < score_thres) {
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                if (pt->invalid()) {
                    prev_fls = NULL;
                    continue;
                }
                if (prev_fls != NULL) {
                    prev_fls->next = pt.get();
                    pt->prev = prev_fls;
                    pt->dir = alg::get_dir(pt->pos, prev_fls->pos);
                    prev_fls->dir = pt->dir;
                    vote_yellow(session, prev_fls, match_fls_list);  // 投票计算prev_fls的颜色属性
                    vote_geo(session, prev_fls, match_fls_list); // 投票计算几何属性
                    vote_double(session, prev_fls, match_fls_list); // 投票计算是否是双线
                }
                prev_fls = pt.get();
                // DLOG_POINT(pt->pos, "merge_filter_lane[s={}, m1={}, m2={}, f={}]",
                //         pt->score, match_num, other_match_num, pt->filter_status);
            }
        }
        #endif
    }
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::match_frame_line_lane(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group, 
        LaneLineSampleTree &trail_tree,
        LaneLineSampleLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    std::vector<LaneLineSample*> secs;
    UMAP<LaneLineSampleGroupLine*, int> cache_map;
    UMAP<LaneLineSample*, int> used_map;
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        // session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        auto &match_map = pt->merge_match_map;
        trail_tree.search(pt->pos, radius, secs);
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true);
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
            } else {
                if (!alg::get_cross_point_for_segment(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, 1)) {
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // tar的垂直方向向量 与 src方向向量的交点 与 当前 tar 点 在 1m范围内
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir);
            if (theta > theta_thres) { // 两条线上的匹配点，对应的朝向 在 20 度范围内
                continue;
            }
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) { // tar 到 src 垂足距离 在 1m范围内
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1;
        }
    }
    using line_score = std::pair<LaneLineSampleGroupLine*, double>;
    std::vector<line_score> match_list;
    for (auto &it : cache_map) {
        double score = 0;
        if (!is_match_line_lane(session, it.first, tar_line, score)) {
            continue;
        }
        match_list.emplace_back(std::make_pair(it.first, score));
    }
    if (match_list.size() == 0) {
        gen_new_lane_group(session, trail_group, tar_line, trail_tree);
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        update_lane_group_line(session, top_line, tar_line, trail_tree);
    }
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::gen_new_lane_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group, 
        LaneLineSampleLine* tar_line,
        LaneLineSampleTree &trail_tree) {
    auto new_line = std::make_shared<LaneLineSampleGroupLine>();
    new_line->id = tar_line->id;
    LaneLineSample* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<LaneLineSample>();
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_list_point.resize(new_line->match_list.size());
        new_line->match_score.push_back(pt->src->score);
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }
    new_line->merge_lines.push_back(tar_line->src);
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end());
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::update_lane_group_line(RoadModelSessionData* session,
        LaneLineSampleGroupLine* src_line, 
        LaneLineSampleLine* tar_line,
        LaneLineSampleTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope;
    int64_t max_tar_valid = 0;
    int64_t max_src_valid = 0;
    int64_t min_tar_valid = INT_MAX;
    int64_t min_src_valid = INT_MAX;
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) {
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true);
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        double opt_dis = pt->src->score / (total_score + pt->src->score) * v_dis;// 1e-6 防止分母为0
        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt);
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->src->score;
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneLineSample>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_index = src_line->list.size();
                new_pt->line_id = src_line->id;
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->src->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }
    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            for (int i = min_tar_valid - 2; i >= 0; --i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneLineSample>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->src->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<LaneLineSample*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

bool RoadModelProcMergeFeature::is_match_line_lane(RoadModelSessionData* session,
        LaneLineSampleGroupLine* src, LaneLineSampleLine* tar, double &score) {
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        score = 1;
        return true;
    }

    double rate_thres = FLAGS_merge_feature_match_rate_threshold;

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) {
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }
    if (src_max_index == src->list.size() - 2) {
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }
    // if (score > rate_thres) {
    //     int a = 1;
    // }
    // LOG_DEBUG("merge_lane[score={:.2f}, match={}, total={}, "
    //         "tar_max={}, tar_min={}, src_max={}, src_min={}]",
    //         score, match_num, total_num, 
    //         tar_max_index, tar_min_index, src_max_index, src_min_index);
    if (score > rate_thres) {
        return true;
    }
    return false;
}

void RoadModelProcMergeFeature::merge_boundary_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->boundary_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->boundary_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.1);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->boundary_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->boundary_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[boundary] 1 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
                #if 0
                // case2: 有重叠 & 方式不一致，但是是180度, 则先进行反转再进行合并
                // TODO: qzc 需要考虑有多条线重叠的时候，应该以最长的方向为准
                if (curve_overlap.is_need_reverse_l1) {

                    // // for debug
                    // auto first_before = trail_ptr->boundary_line_group_list[i]->list[0]->pos;
                    // int p_size = trail_ptr->boundary_line_group_list[i]->list.size();
                    // auto last_before = trail_ptr->boundary_line_group_list[i]->list[p_size-1]->pos;

                    reverse(trail_ptr->boundary_line_group_list[i]->list.begin(), trail_ptr->boundary_line_group_list[i]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[i]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }

                    // // for debug
                    // auto frist_after = trail_ptr->boundary_line_group_list[i]->list[0]->pos;
                    // auto last_after = trail_ptr->boundary_line_group_list[i]->list[p_size-1]->pos;
                    // std::cout << "l1: first_before: " <<  first_before.transpose() << " last_before: " << last_before.transpose() << std::endl;
                    // std::cout << "l1: frist_after: " <<  frist_after.transpose() << " last_after " << last_after.transpose() << std::endl;
                } else if (curve_overlap.is_need_reverse_l2) {

                    // // for debug
                    // auto first_before = trail_ptr->boundary_line_group_list[j]->list[0]->pos;
                    // int p_size = trail_ptr->boundary_line_group_list[j]->list.size();
                    // auto last_before = trail_ptr->boundary_line_group_list[j]->list[p_size-1]->pos;

                    reverse(trail_ptr->boundary_line_group_list[j]->list.begin(), trail_ptr->boundary_line_group_list[j]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[j]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }

                    // // for debug
                    // auto frist_after = trail_ptr->boundary_line_group_list[j]->list[0]->pos;
                    // auto last_after = trail_ptr->boundary_line_group_list[j]->list[p_size-1]->pos;
                    // std::cout << "l2: first_before: " <<  first_before.transpose() << " last_before: " << last_before.transpose() << std::endl;
                    // std::cout << "l2: frist_after: " <<  frist_after.transpose() << " last_after " << last_after.transpose() << std::endl;
                }
                #endif
                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->boundary_line_group_list[i];
                // group_line_i.insert(j);
                // auto &group_line_j = trail_ptr->boundary_line_group_list[j];
                // group_line_j.insert(i);
                
                 if (debug_log && reverse_process == false) {
                    std::cout << "[boundary] 2 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }

                #if 0
                // case3: 无重叠 & 方式一致（首末尾）, 进行合并
                if (curve_overlap.is_need_reverse_l1) {
                    reverse(trail_ptr->boundary_line_group_list[i]->list.begin(), trail_ptr->boundary_line_group_list[i]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[i]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                } else if (curve_overlap.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[j]->list.begin(), trail_ptr->boundary_line_group_list[j]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[j]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                }
                #endif

            }
            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->boundary_line_group_list[id1]->list.begin(), trail_ptr->boundary_line_group_list[id1]->list.end());
            std::shared_ptr<BoundaryFeature> prev = NULL;
            for (auto &fls : trail_ptr->boundary_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
            }
             if (debug_log && reverse_process == false) {
                std::cout << "[boundary] 0#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << curve_overlap_0.l2_id << std::endl;
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                std::cout << "[boundary] 1#need_reverse: " << need_reverse << " id1 : " << id1 << " id2: " << id2  << " matched_num: " << matched_num  << " has_processed[id2]:" << has_processed[id2] << std::endl; 
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[id2]->list.begin(), trail_ptr->boundary_line_group_list[id2]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[boundary] 2#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->boundary_line_group_list[id2]->list.begin(), trail_ptr->boundary_line_group_list[id2]->list.end());
                    std::shared_ptr<BoundaryFeature> prev = NULL;
                    for (auto &fls : trail_ptr->boundary_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[boundary] 3#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }
    

    // merge multi group line
    // std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
    // UMAP<std::string, std::vector<BoundaryLine*>> tmp_bev_frame_boundary_line; 
    session->boundary_line_ptr_for_merge.clear();
    session->bev_frame_boundary_line.clear();
    
    int lane_size = trail_ptr->boundary_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<BoundaryGroupLine> group_line;
            //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_boundary_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->boundary_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->boundary_line_ptr_for_merge);
            new_line->boundary_type = group_line->boundary_type;
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<BoundaryGroupLine> group_line;
            //      std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->boundary_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_boundary_line[""].push_back(new_line.get());
        }
    }
    // session->bev_frame_boundary_line = tmp_bev_frame_boundary_line;

    // 2. 重新调用 merge single 操作
    merge_single_boundary_by_trail(session, trail_ptr);
}

int RoadModelProcMergeFeature::merge_single_boundary_by_trail(
        RoadModelSessionData* session, KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size;
    int min_match_size = FLAGS_merge_feature_proc_min_match_size;
    double score_thres = FLAGS_merge_feature_proc_score_threshold;
    double scope = FLAGS_merge_feature_scope;

    if (trail->list.size() < min_size) {
        return fsdmap::FAIL;
    }

    trail->boundary_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        BoundaryFeatureTree trail_tree;
        trail->boundary_line_group_list.clear();
        int64_t raw_line_size = 0;
        // 遍历这整条轨迹上的轨迹点
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     // if (poss->frame_id == "6231672047094_1672018299880000") {
        //     //     int a = 1;
        //     // }
        //     LOG_ERROR("session->bev_frame_boundary_line size:{} poss->frame_id :{}", session->bev_frame_boundary_line.size(),
        //      poss->frame_id);
        //     if (MAP_NOT_FIND(session->bev_frame_boundary_line, poss->frame_id)) {          
        //         continue;
        //     }
        
        int debug_index = 0;

        //     auto &ins_vec = session->bev_frame_boundary_line[poss->frame_id]; // 被当前帧pose看到的boundary
        for (const auto& ins_pair : session->bev_frame_boundary_line) { // 每一条边界线，有很多重复
            const auto& ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) { // 每一条线
                // if (ins->boundary_type != 1) {
                //     continue;
                // }
                
                ins->sub_line.clear();
                BoundaryFeature* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins;
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->boundary_type = ins->boundary_type;
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    // 三角岛不需要打断
                    if (new_line->boundary_type != 1 && pt->invalid()) { // 该点是异常点（比如是一个突变点，方向想来那个发生了突变），在前面步骤的sample_line()函数里把无效点filter_status置为2
                        if (new_line->list.size() > 1) { // pt这个点出现在了曲线的中间，那么将ins这条曲线从pt点这个位置切开，重启另外一条line放入ins->sub_line
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                            new_line->src = ins;
                        } else { // pt突变点出现在曲线开头，只需要清空下new_line，后面继续往这里面填
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    // 找到 sub_line 的匹配曲线，把他连接到这条曲线上
                    match_frame_line_boundary(session, trail->boundary_line_group_list, trail_tree, sub_line.get());
                }

                auto &poss = trail->list[0];
                debug_index += 1;
                std::string id_index(std::to_string(debug_index)+"_"+ins->id);
                save_frame_log_boundary(session, trail, poss, times, id_index);
            }
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_boundary_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->boundary_line_group_list.size(), raw_line_size);
        // 过滤
        #if 1
        double length_thres = FLAGS_merge_feature_single_boundary_length_thres;
        for (int64_t i = 0; i < trail->boundary_line_group_list.size(); ++i) {
            auto &group_line = trail->boundary_line_group_list[i];
            // add by qzc: 三角岛不处理
            if (group_line->boundary_type == 1) {
                continue;
            }
            
            double total_length = 0;
            BoundaryFeature* prev_pt = NULL;
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                // session->debug_pos(pt->pos);
                // int match_num = group_line->match_list[j].size();
                int match_num = group_line->match_list_point[valid_index].size();
                // UMAP<std::string, int> match_frame_map;

                double total_score = group_line->match_score[j];
                for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                    // match_frame_map[raw_pt->frame_id]++;
                    // 合并其他点
                    double v_dis = alg::calc_vertical_dis(raw_pt->pos, pt->pos, pt->dir, true); // 计算点raw_pt->pos到直线pt->dir的距离
                    if (v_dis > scope) { // 小于 scope 的匹配点才进行合并
                        continue;
                    }
                    double opt_dis = raw_pt->score / (total_score + raw_pt->score /*+ 1e-6*/) * v_dis;// 1e-6 防止分母为0
                    Eigen::Vector3d opt_pos = alg::get_vertical_pos(pt->pos, pt->dir, opt_dis);
                    pt->pos = opt_pos;
                    total_score += raw_pt->score;
                }
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map; // 与pt点匹配的点的对应其他的匹配关系
                    // match_frame_map[raw_pt->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        // match_frame_map[raw_pt->frame_id]++;
                    }
                }
                double total_num = match_num + other_match_num;
                // pt->score = match_num / (total_num + 1e-6); // 1e-6 防止分母为0

                // pt->score = (double)match_frame_map.size() / pt->match_frame_num;

                // if (total_num < min_match_size) { // 匹配点少于1个，滤除，与该点匹配上的其他曲线上的点也删除
                //     pt->filter_status = 2;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 2;
                //     }
                // }

                // if (pt->score < score_thres) { //可理解为共视率
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                if (pt->invalid()) {
                    continue;
                }
                if (prev_pt != NULL) {
                    total_length += alg::calc_dis(prev_pt->pos, pt->pos);
                }
                // 用group_line->match_list_point[valid_index]中出现频率最高的类别作为pt的类别

                vote_type(session, pt.get(), group_line->match_list_point[valid_index]);
                // DLOG_POINT(pt->pos, "merge_filter_boundary[s={}, m1={}, m2={}, f={}]",
                //         pt->score, match_num, other_match_num, pt->filter_status);
                prev_pt = pt.get();
            }
            // 长度小于5m的也滤除
            if (total_length < length_thres) {
                for (auto &pt : group_line->list) {
                    pt->filter_status = 4;
                }
            }
        }
        #endif

    }
    
    return fsdmap::SUCC;
}

// trail_tree: 存储了之前遍历到的单条轨迹上的轨迹点看到的每段曲线信息
// 在该趟路线中寻找与tar_line有80%的点都能匹配上的去遍历过的曲线，选出匹配度最高的那条曲线，
// 把tar_line上的点扩展到这条曲线上
int RoadModelProcMergeFeature::match_frame_line_boundary(RoadModelSessionData* session,
        std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group, 
        BoundaryFeatureTree &trail_tree,
        BoundaryLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    std::vector<BoundaryFeature*> secs;
    UMAP<BoundaryGroupLine*, int> cache_map; // 这趟路线中与tar_line有点匹配上的历史曲线
    UMAP<BoundaryFeature*, int> used_map;
    // 遍历tar_line上的每个点pt，在历史曲线树trail_tree中搜索最近邻的点，
    // 遍历这些最近邻点，判断是否与pt点满足距离要求、以及方向一致的匹配点，
    // 匹配上的话就更新pt点以及最近邻点的匹配关系
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        // add by qzc: 三角岛不处理
        if (tar_line->boundary_type == 1) {
            continue;
        }
        
        auto &pt = tar_line->list[j];
        // session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        // 在单轨迹车道线等特征融合过程中，与该点满足距离要求，以及方向一致的匹配点，以及该匹配点对应所属的曲线 BoundaryGroupLine，
        // <该匹配点对应所属的曲线，与该点满足距离要求、以及方向一致的匹配点>
        auto &match_map = pt->merge_match_map;
        trail_tree.search(pt->pos, radius, secs); // 搜索 5m 内的点
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true); //计算dir垂线方向上离pos距离是50m的点
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            // add by qzc: 三角岛不处理
            if (src_pt->boundary_type == 1) {
                continue;
            }
            
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
                // double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir, true);
                // if (h_dis < 0 && src_pt->next == NULL) {
                //     continue;
                // } 
                // if (h_dis > 0 && src_pt->prev == NULL) {
                //     continue;
                // }
                // dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            } else {
                if (!alg::get_cross_point_for_segment(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, 1)) { // 计算交点【通过pt的垂线与（src_pt, src_pt->next）线段的交点】
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // 1. 交点在 1 m 内 
                // match_map[src_pt->group_line] = NULL;
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir); //2.计算方向一致性
            if (theta > theta_thres) { // 20
                continue;
            }
            // 3.pt->pos 到直线 src_pt->dir 的垂直距离
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) {
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            // src_pt->group_line: src_pt点所属的曲线
            // src_pt->line_index 在所属曲线上的index
            // src_pt->group_line->match_list_point ：<src_pt在所属曲线上的Index，与src距离方向都匹配的其他曲线上的点（应该用于合并曲线）>
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1; // 记录 该src_pt 所属的 group_line 已经 cache ， 后面会用来和 tar_line 匹配，获取匹配度最高的线进行融合
        }
    }
    using line_score = std::pair<BoundaryGroupLine*, double>;
    std::vector<line_score> match_list; // 记录cache_map中与tar_line有80%的点都能匹配上的曲线
    for (auto &it : cache_map) {
        double score = 0;
        // 判断it.first与tar_line是否是同一条曲线
        if (!is_match_line_boundary(session, it.first, tar_line, score)) {
            continue;
        }
        // is_match_line_boundary(session, it.first, tar_line, score);

        match_list.emplace_back(std::make_pair(it.first, score));
    }
    if (match_list.size() == 0) { // 这趟路线中没有与tar_line有点匹配上的历史曲线
        // 通过tar_line产生新的线放入trail_group，并把tar_line中的点放入trail_tree
        gen_new_boundary_group(session, trail_group, tar_line, trail_tree);
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        //选出匹配度最高的一条曲线，把tar_line上的点扩展到这条曲线上，并更新top_line上匹配点的位置 
        update_boundary_group_line(session, top_line, tar_line, trail_tree);
    }
    return fsdmap::SUCC;
}

// 通过tar_line产生新的线放入trail_group，并把tar_line中的点放入trail_tree
int RoadModelProcMergeFeature::gen_new_boundary_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group, BoundaryLine* tar_line,
        BoundaryFeatureTree &trail_tree) {
    auto new_line = std::make_shared<BoundaryGroupLine>();
    new_line->id = tar_line->id;
    BoundaryFeature* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<BoundaryFeature>();
        new_pt->boundary_type = tar_line->boundary_type;
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_score.push_back(pt->score);
        new_line->match_list_point.resize(new_line->match_list.size());
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }     
    new_line->merge_lines.push_back(tar_line->src); // tar_line 在这里其实是 sub_line, src 其实就是 bev_frame_boundary_line 中的原始线段
    new_line->boundary_type = tar_line->boundary_type;
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end()); 
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

// 遍历tar_line中的每个点pt，在src_line寻找与他匹配上的点src_pt，如果计算点pt到直线src_pt->dir的距离dist小于1m，
// 那么更新src_pt的位置，更新原则按照pt->score（点的位置可信度）的占比，以及dist重新计算。
// 如果tar_line中尾部还剩下大于3个点没有在src_line找到匹配点，
// 同时满足tar_line中max_tar_valid + 1出的点在src_line最后一个点的前方，那么把
// tar_line索引从max_tar_valid + 2开始往后的点都归入src_line，拼接起来。
// 如果tar_line中头部还有多于2个点没有在src_line找到匹配点，
// 同时满足src_line第一个点在tar_line中min_tar_valid - 1处的点的前方，那么把
// tar_line索引从0开始到min_tar_valid - 2的点都归入src_line，拼接起来
int RoadModelProcMergeFeature::update_boundary_group_line(RoadModelSessionData* session,
        BoundaryGroupLine* src_line, BoundaryLine* tar_line,
        BoundaryFeatureTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope; // 1m
    int64_t max_tar_valid = 0;
    int64_t max_src_valid = 0;
    int64_t min_tar_valid = INT_MAX;
    int64_t min_src_valid = INT_MAX;
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) { // tar_line上的该点没有与src_line匹配上
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        // 计算点pt->pos到直线src_pt->dir的距离
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true);
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        // 按照pt->score（点的位置可信度）的占比，重新计算src_pt->pos的位置
        double opt_dis = pt->score / (total_score + pt->score) * v_dis;// 1e-6防止除数为0
        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt); //更新完位置的点放入tree
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->score; // 又新增了一个匹配点，加上匹配点的score
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    // 如果tar_line中尾部还剩下大于3个点没有在src_line找到匹配点，
    // 同时满足tar_line中max_tar_valid + 1出的点在src_line最后一个点的前方，那么把
    // tar_line索引从max_tar_valid + 2开始往后的点都归入src_line，拼接起来
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        // 判断next_tar_pt->pos是否在prev_pt->pos的前方
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            // for (int i = max_tar_valid + 1; i < tar_line->list.size(); ++i) { // TODO:qzc 为什么不是+1，而是+2？
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) { // TODO:qzc 为什么不是+1，而是+2？
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<BoundaryFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                new_pt->line_index = src_line->list.size();
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }

    // 如果tar_line中头部还有多于2个点没有在src_line找到匹配点，
    // 同时满足src_line第一个点在tar_line中min_tar_valid - 1处的点的前方，那么把
    // tar_line索引从0开始到min_tar_valid - 2的点都归入src_line，拼接起来
    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        // 判断next_pt->pos是否在prev_tar_pt->pos的前方
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            // for (int i = min_tar_valid - 1; i >= 0; --i) { // TODO:qzc 为什么不是-1，而是-2？
            for (int i = min_tar_valid - 2; i >= 0; --i) { // TODO:qzc 为什么不是-1，而是-2？
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<BoundaryFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<BoundaryFeature*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

// 根据曲线src与tar匹配上的点的个数，计算匹配度，如果tar中有80%的点都能在src上匹配到，那么认为他们是同一条曲线，可以合并
bool RoadModelProcMergeFeature::is_match_line_boundary(RoadModelSessionData* session,
        BoundaryGroupLine* src, BoundaryLine* tar, double &score) {
    // add by qzc: 如果在外面已经匹配好了，那么直接进行合并
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        score = 1;
        return true;
    }

    double rate_thres = FLAGS_merge_feature_match_rate_threshold; // 0.8

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) { // pt点与曲线src有匹配点
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }
    if (src_max_index == src->list.size() - 2) {
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }
    // if (score > rate_thres) {
    //     int a = 1;
    // }
    // LOG_DEBUG("merge_boundary[score={:.2f}, match={}, total={}, "
    //         "tar_max={}, tar_min={}, src_max={}, src_min={}]",
    //         score, match_num, total_num, 
    //         tar_max_index, tar_min_index, src_max_index, src_min_index);
    if (score > rate_thres) {
        return true;
    }
    return false;
}

void RoadModelProcMergeFeature::merge_lane_center_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process) {
    bool debug_log = false;
    // bool debug_log = true;
    int line_size = trail_ptr->lane_center_line_group_list.size();
    std::vector<alg::linestring_t> all_curve(line_size);
    std::vector<std::vector<Eigen::Vector3d>> all_curve_points_dir(line_size);
    for (int64_t i = 0; i < line_size; ++i) {
        auto &group_line = trail_ptr->lane_center_line_group_list[i];
        group_line->cur_line_id = i;
        
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if(pt == nullptr) {
                LOG_ERROR("!!!!!!!!!!!!not normal point!!!!!!!");
                continue;
            }
            all_curve[i].push_back(alg::point_t(pt->pos.x(), pt->pos.y()));
            all_curve_points_dir[i].push_back(pt->dir);
        }
    }

    std::vector<std::vector<alg::OverlapInfoLineString>> matched_list(line_size); // 每条线段与其他线段的匹配信息
    std::map<int, std::unordered_set<int>> common_prcessed_line; // 当前id，匹配上的line
    for(int i = 0; i < line_size; i++) {
        for (int j = i+1; j < line_size; j++) {
            alg::OverlapInfoLineString curve_overlap;
            curve_overlap.l1_id = i;
            curve_overlap.l2_id = j;
            // lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.05);
            lines_overlap_info(all_curve[i], all_curve[j], all_curve_points_dir[i], all_curve_points_dir[j], curve_overlap, 0.2);
            if (curve_overlap.is_overlap) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                auto &group_line_i = trail_ptr->lane_center_line_group_list[i];
                group_line_i->matched_line_ids.insert(j);
                auto &group_line_j = trail_ptr->lane_center_line_group_list[j];
                group_line_j->matched_line_ids.insert(i);

                if (debug_log && reverse_process == false) {
                    std::cout << "[lane_center] 1 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
                // case1: 有重叠 & 方式一致, 进行合并
            } else if(curve_overlap.is_nearby) {
                matched_list[i].push_back(curve_overlap);
                alg::OverlapInfoLineString curve_overlap1;
                alg::reverse_matched_info(curve_overlap, curve_overlap1);
                matched_list[j].push_back(curve_overlap1);

                if (common_prcessed_line.find(i) == common_prcessed_line.end()) {
                    common_prcessed_line[i] = {j};
                } else {
                    common_prcessed_line[i].insert(j);
                }
                if (common_prcessed_line.find(j) == common_prcessed_line.end()) {
                    common_prcessed_line[j] = {i};
                } else {
                    common_prcessed_line[j].insert(i);
                }

                // auto &group_line_i = trail_ptr->lane_center_line_group_list[i];
                // group_line_i.insert(j);
                // auto &group_line_j = trail_ptr->lane_center_line_group_list[j];
                // group_line_j.insert(i);

                 if (debug_log && reverse_process == false) {
                    std::cout << "[lane_center] 2 is_overlap : " << curve_overlap.is_overlap  << " is_nearby: " << curve_overlap.is_nearby << " revise: i " << i << " j: " << j  
                            << " revise l1: " << curve_overlap.is_need_reverse_l1 << " revise l2: " << curve_overlap.is_need_reverse_l2 << std::endl;
                }
            }
            // case4: 无重叠 & 方式不一致，不处理
        }
    }

    for (auto& matched_info : matched_list) {
        std::sort(matched_info.begin(), matched_info.end(), [](const alg::OverlapInfoLineString& a, const alg::OverlapInfoLineString& b) {
            return a.l2_length > b.l2_length;
        });
    }

    // 排序：匹配个数》长度
    std::sort(matched_list.begin(), matched_list.end(), [](const std::vector<alg::OverlapInfoLineString>& a, const std::vector<alg::OverlapInfoLineString>& b) {
        if (a.size() != b.size()) {
            return a.size() > b.size();
        } else if(a.size() == 0 && b.size() == 0) {
            return false;
        }
        
        return a[0].l1_length > b[0].l1_length;
    });

    std::vector<bool> has_processed(line_size, false);
    // 注意：matched_list 已经不是按照，line_size 的顺序了
    for (const auto& matched_info_i : matched_list) {
        int matched_num = matched_info_i.size();
        if (matched_num == 0) {
            continue;
        }

        // 当自己长度比较短，且需要反转，那么除了0以外的其他line，如果需要反转，则需要和以 need_reverse 为准
        auto const& curve_overlap_0 = matched_info_i[0];
        int id1 = curve_overlap_0.l1_id;
        bool need_reverse = curve_overlap_0.is_need_reverse_l1;

        // 必须要是还有未处理过的match点，再进行融合， TODO：qzc 可能还存在一种情况是，上次已经反转了一次，当前还有，那么可能会导致错误的反转，可能需要同步改掉matched_list中的状态才行
        int valid_match_size = 0;
        std::unordered_set<int> other_line_prcessed;
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            int id2 = curve_overlap_j.l2_id;
            if (has_processed[id2] == true) {
                // 如果已经处理过的id2，对应的匹配信息中，有id1，那么就不能重复处理，需要过滤掉
                if (common_prcessed_line.find(id2) != common_prcessed_line.end() && common_prcessed_line[id2].find(id1) != common_prcessed_line[id2].end()) {
                    other_line_prcessed.insert(common_prcessed_line[id2].begin(), common_prcessed_line[id2].end());
                }
                
                continue;
            }
            // 如果之前处理过，那么直接跳过
            if (other_line_prcessed.find(id2) != other_line_prcessed.end()) {
                continue;
            }

            valid_match_size++;
        }

        if (need_reverse && valid_match_size > 0) {
            reverse(trail_ptr->lane_center_line_group_list[id1]->list.begin(), trail_ptr->lane_center_line_group_list[id1]->list.end());
            std::shared_ptr<LaneCenterFeature> prev = NULL;
            for (auto &fls : trail_ptr->lane_center_line_group_list[id1]->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;
            }
             if (debug_log && reverse_process == false) {
                std::cout << "[lane_center] 0#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << curve_overlap_0.l2_id << std::endl;
            }
        }

        
        for (int j = 0; j < matched_num; j++) {
            auto const& curve_overlap_j = matched_info_i[j];
            
            int id2 = curve_overlap_j.l2_id;
             if (debug_log && reverse_process == false) {
                std::cout << "[lane_center] 1#need_reverse: " << need_reverse << " id1 : " << id1 << " id2: " << id2  << " matched_num: " << matched_num  << " has_processed[id2]:" << has_processed[id2] << std::endl; 
            }
            if (has_processed[id2] == true) {
                continue;
            }

            if (need_reverse == false) {
                // 如果自己不需要反转，其他和自己不相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 != curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_center_line_group_list[id2]->list.begin(), trail_ptr->lane_center_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneCenterFeature> prev = NULL;
                    for (auto &fls : trail_ptr->lane_center_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }

                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane_center] 2#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            } else {
                // 如果自己需要反转，其他和自己相同的，都反转的
                if (curve_overlap_j.is_need_reverse_l1 == curve_overlap_j.is_need_reverse_l2) {
                    reverse(trail_ptr->lane_center_line_group_list[id2]->list.begin(), trail_ptr->lane_center_line_group_list[id2]->list.end());
                    std::shared_ptr<LaneCenterFeature> prev = NULL;
                    for (auto &fls : trail_ptr->lane_center_line_group_list[id2]->list) {
                        if (prev == NULL) {
                            prev = fls;
                            continue;
                        }
                        fls->dir = alg::get_dir(fls->pos, prev->pos);
                        prev->dir = fls->dir;
                    }
                     if (debug_log && reverse_process == false) {
                        std::cout << "[lane_center] 3#need_reverse: " << need_reverse << " matched_num: " << matched_num << " l1: " << id1 << " l2: " << id2 << std::endl;
                    }
                }
            }

            // // case1: 有重叠 & 方向一致, 进行合并
            // // case2: 有重叠 & 方向不一致，但是是180度, 则先进行反转再进行合并
            // if (curve_overlap_j.is_overlap) {
            // } 
            // // case3: 无重叠 & 方向一致（首末尾）, 进行合并
            // if (curve_overlap_j.is_nearby) {
            // }
            // case4: 无重叠 & 方向不一致，不处理
        }

        has_processed[id1] = true;
    }

    // merge multi group line
    session->lane_center_line_ptr_for_merge.clear();
    session->bev_frame_lane_center_line.clear();
    
    int lane_size = trail_ptr->lane_center_line_group_list.size();
    if (reverse_process) {
        for (int64_t i = lane_size-1; i >= 0; --i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<LaneCenterGroupLine> group_line;
            //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_center_line[""].push_back(new_line.get());
        }
    } else {
        for (int64_t i = 0; i < lane_size; ++i) {
            auto &group_line = trail_ptr->lane_center_line_group_list[i];

            // 1 构造单趟合并的 boundary
            auto new_line = session->add_ptr(session->lane_center_line_ptr_for_merge);
            new_line->cur_line_id = group_line->cur_line_id;
            new_line->matched_line_ids = group_line->matched_line_ids;

            // std::shared_ptr<LaneCenterGroupLine> group_line;
            //      sstd::vector<std::shared_ptr<LaneCenterFeature>> list;;  // 该曲线上所有的点
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                // if (j == group_line->list.size() - 1) {
                //     valid_index = j - 1;
                // }
                auto &pt = group_line->list[j];
                if(pt == nullptr) {
                    continue;
                }

                auto new_node = session->add_ptr(session->lane_center_feature_ptr);
                // new_node = pt;
                // new_node->pose = pt->pos;
                new_node->init(pt.get());

                new_line->list.push_back(new_node.get());
            }

            if (new_line->list.size() == 0) {
                continue;
            }

            session->bev_frame_lane_center_line[""].push_back(new_line.get());
        }
    }

    // 2. 重新调用 merge single 操作
    merge_single_lane_center_by_trail(session, trail_ptr);
}


int RoadModelProcMergeFeature::merge_single_lane_center_by_trail(
        RoadModelSessionData* session, KeyPoseLine* trail) {
    int min_size = FLAGS_merge_feature_proc_min_size; // 10
    int min_match_size = FLAGS_merge_feature_proc_min_match_size; // 1
    double score_thres = FLAGS_merge_feature_proc_score_threshold; // 0.2
    if (trail->list.size() < min_size) { // 轨迹太短
        return fsdmap::FAIL;
    }
    trail->lane_center_line_group_list.reserve(trail->list.size());

    int times = 0;
    while (times < 2) {
        ++times;
        LaneCenterFeatureTree trail_tree;
        trail->lane_center_line_group_list.clear(); // 每次循环前清除
        int64_t raw_line_size = 0;
        // for (int64_t i = 0; i < trail->list.size(); ++i) {
        //     auto &poss = trail->list[i];
        //     if (poss->frame_id == "6181672392159_1672363358942000") {
        //         int a = 1;
        //     }
        //     if (MAP_NOT_FIND(session->bev_frame_lane_center_line, poss->frame_id)) { 
        //         continue;
        //     }
        //     auto &ins_vec = session->bev_frame_lane_center_line[poss->frame_id]; // 当前帧看到的所有车道中心线
        for (const auto &ins_pair : session->bev_frame_lane_center_line) {
            const auto &ins_vec = ins_pair.second;
            if (ins_vec.empty()) {
                continue;
            }
            for (auto &ins : ins_vec) {
                ins->sub_line.clear();
                LaneCenterFeature* prev = NULL;
                auto new_line = session->add_ptr(ins->sub_line);
                new_line->list.reserve(ins->list.size());
                new_line->src = ins;
                int64_t sub_line_index = 0;
                new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);

                // add by qzc
                new_line->cur_line_id = ins->cur_line_id;
                new_line->matched_line_ids = ins->matched_line_ids;

                for (auto &pt : ins->list) {
                    session->debug_pos(pt->pos);
                    if (pt->invalid()) {
                        if (new_line->list.size() > 1) {
                            new_line = session->add_ptr(ins->sub_line);
                            new_line->src = ins;
                            new_line->id = utils::fmt("{}_{}", ins->id, sub_line_index++);
                        } else {
                            new_line->list.clear();
                        }
                        prev = NULL;
                        continue;
                    }
                    new_line->list.push_back(pt);
                    pt->merge_match_map.clear();
                    if (prev != NULL) {
                        pt->set_prev(prev);
                    }
                    prev = pt;
                }
                for (auto &sub_line : ins->sub_line) {
                    if (sub_line->list.size() <= 1) {
                        continue;
                    }
                    match_frame_line_lane_center(session, trail->lane_center_line_group_list, 
                            trail_tree, sub_line.get());
                }
            }
            // save_frame_log_lane_center(session, trail, poss, times);
            raw_line_size += ins_vec.size();

        }
        LOG_INFO("merge_lane_center_final[id={}, p_size={}, times={}, size={}, raw_size={}]", 
                (int64_t)trail, trail->list.size(), times, 
                trail->lane_center_line_group_list.size(), raw_line_size);
        // 过滤
        for (int64_t i = 0; i < trail->lane_center_line_group_list.size(); ++i) {
            auto &group_line = trail->lane_center_line_group_list[i];
            for (int64_t j = 0; j < group_line->list.size(); ++j) {
                int valid_index = j;
                if (j == group_line->list.size() - 1) {
                    valid_index = j - 1;
                }
                auto &pt = group_line->list[j];
                session->debug_pos(pt->pos);
                int match_num = group_line->match_list[j].size();
                // int match_num = group_line->match_list_point[valid_index].size();
                UMAP<std::string, int> match_frame_map;
                for (auto &raw_pt : group_line->match_list_point[valid_index]) {
                    match_frame_map[raw_pt->frame_id]++;
                }
                int other_match_num = 0;
                for (auto &raw_pt : group_line->match_list[valid_index]) {
                    auto &match_map = raw_pt->merge_match_map;
                    match_frame_map[raw_pt->frame_id]++;
                    for (auto &it : match_map) {
                        if (it.first == group_line.get()) {
                            continue;
                        }
                        auto &line_match_list = it.first->match_list_point;
                        other_match_num += line_match_list[it.second->line_index].size();
                        match_frame_map[it.second->frame_id]++;
                    }
                }
                double total_num = match_num + other_match_num;
                pt->score = match_num / (total_num + 1e-6);
                // pt->score = (double)match_frame_map.size() / pt->match_frame_num;
                if (total_num < min_match_size) {
                    pt->filter_status = 2;
                    for (auto &raw_pt : group_line->match_list[valid_index]) {
                        // session->debug_pos(raw_pt->pos);
                        raw_pt->filter_status = 2;
                    }
                }
                // if (pt->score < score_thres) {
                //     pt->filter_status = 3;
                //     for (auto &raw_pt : group_line->match_list[valid_index]) {
                //         // session->debug_pos(raw_pt->pos);
                //         raw_pt->filter_status = 3;
                //     }
                // }
                DLOG_POINT(pt->pos, "merge_filter_lane_center[s={}, m1={}, m2={}, f={}]",
                        pt->score, match_num, other_match_num, pt->filter_status);
            }
        }
    }
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::match_frame_line_lane_center(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    double radius = FLAGS_merge_feature_search_radius; // 5
    double scope = FLAGS_merge_feature_scope; // 1
    double theta_thres = FLAGS_merge_feature_theta; // 20

    std::vector<LaneCenterFeature*> secs;
    UMAP<LaneCenterGroupLine*, int> cache_map; // 存储与tar_line的车道中心点存在同一车道的中心点的车道中心线
    UMAP<LaneCenterFeature*, int> used_map;
    // 遍历某条车道中心线里的每个点，选出附近5m半径内的车道中心点，在这些点中选出距离在一个车道范围内的点
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        auto &match_map = pt->merge_match_map;  // 该点附近5m半径内的车道中心点，，在这些点中选出距离在一个车道范围内的车道中心点，以及这个车道中心点所属的车道中心线上
        trail_tree.search(pt->pos, radius, secs);  // 筛选出pt点附近5m半径内的车道中心点
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true);
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            session->debug_pos2(pt->pos, src_pt->pos);
            double dis = 0;
            if (src_pt->next == NULL) {
                continue;
                // double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir, true);
                // if (h_dis < 0 && src_pt->next == NULL) {
                //     continue;
                // } 
                // if (h_dis > 0 && src_pt->prev == NULL) {
                //     continue;
                // }
                // dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            } else {
                if (!alg::get_cross_point_for_segment(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, 1)) {
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) { // 两个车道中心点不能是在不同车道里
                // match_map[src_pt->group_line] = NULL;
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir);
            if (theta > theta_thres) {
                continue;
            }
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) { // 两个车道中心点不能是在不同车道里
                continue;
            }
            match_map[src_pt->group_line] = src_pt;
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1;
        }
    }
    using line_score = std::pair<LaneCenterGroupLine*, double>;
    std::vector<line_score> match_list; // <与tar_line有匹配点的车道中心线并且匹配率达到80%, 匹配度得分>
    for (auto &it : cache_map) {
        double score = 0;
        if (!is_match_line_lane_center(session, it.first, tar_line, score)) {
            continue;
        }
        match_list.emplace_back(std::make_pair(it.first, score));
    }
    if (match_list.size() == 0) { // 没有与tar_line匹配的车道中心线，把与tar_line当成一条新的车道中心线处理
        gen_new_lane_center_group(session, trail_group, tar_line, trail_tree);
    } else {
        SORT(match_list, [](const line_score &l, const line_score &r) {
                return l.second > r.second;
                });
        auto &top_line = match_list[0].first;
        update_lane_center_group_line(session, top_line, tar_line, trail_tree);
    }
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::gen_new_lane_center_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, LaneCenterLine* tar_line,
        LaneCenterFeatureTree &trail_tree) {
    auto new_line = std::make_shared<LaneCenterGroupLine>();
    new_line->id = tar_line->id;
    LaneCenterFeature* prev_pt = NULL;
    for (auto &pt : tar_line->list) {
        auto new_pt = std::make_shared<LaneCenterFeature>();
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_score.push_back(pt->score);
        new_line->match_list_point.resize(new_line->match_list.size());
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }
    new_line->merge_lines.push_back(tar_line->src);
    new_line->matched_line_ids.insert(tar_line->matched_line_ids.begin(), tar_line->matched_line_ids.end());
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::update_lane_center_group_line(RoadModelSessionData* session,
        LaneCenterGroupLine* src_line, LaneCenterLine* tar_line,
        LaneCenterFeatureTree &trail_tree) {
    double scope = FLAGS_merge_feature_scope;
    int64_t max_tar_valid = 0;  // tar_line与src_line匹配的点中，最大的index
    int64_t max_src_valid = 0; // src_line与tar_line匹配的点中，最大的index
    int64_t min_tar_valid = INT_MAX;  // tar_line与src_line匹配的点中，最小的index
    int64_t min_src_valid = INT_MAX;  // src_line与tar_line匹配的点中，最小的index
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &pt = tar_line->list[i];
        auto &map = pt->merge_match_map;
        if (MAP_NOT_FIND(map, src_line)) {
            continue;
        }
        auto &src_pt = map[src_line];
        int64_t src_index = src_pt->line_index;
        double total_score = src_line->match_score[src_index];
        double v_dis_1 = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir, true); // pt与src_pt之间的垂向距离
        // double v_dis_2 = -alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir, true);
        double v_dis = v_dis_1;
        // if (fabs(v_dis_1) > fabs(v_dis_2)) {
        //     v_dis = v_dis_2;
        // }
        if (v_dis > scope) {
            continue;
        }
        double opt_dis = pt->score / (total_score + pt->score) * v_dis;// 1e-6防止除数为0
        Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt->pos, src_pt->dir, opt_dis);
        session->debug_pos(opt_pos);
        src_pt->pos = opt_pos;
        trail_tree.insert(src_pt->pos, src_pt);
        src_line->match_list[src_index].push_back(pt);
        src_line->match_score[src_index] += pt->score;
        max_tar_valid = std::max(max_tar_valid, i);
        min_tar_valid = std::min(min_tar_valid, i);
        max_src_valid = std::max(max_src_valid, src_index);
        min_src_valid = std::min(min_src_valid, src_index);
    }
    // 深坑，size 返回的是Uint 不能减
    int64_t end_index = (tar_line->list.size() >= 3) ? (tar_line->list.size() - 3) : 0;
    if (max_tar_valid <= end_index) {
        auto &next_tar_pt = tar_line->list[max_tar_valid + 1];
        auto prev_pt = src_line->list.back().get();
        if (alg::judge_front(next_tar_pt->pos, prev_pt->pos, prev_pt->dir)) {
            for (int i = max_tar_valid + 2; i < tar_line->list.size(); ++i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneCenterFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                new_pt->line_index = src_line->list.size();
                src_line->list.push_back(new_pt);
                src_line->match_list.push_back({pt});
                src_line->match_score.push_back(pt->score);
                src_line->match_list_point.resize(src_line->match_list.size());
                if (prev_pt != NULL) {
                    prev_pt->next = new_pt.get();
                    new_pt->prev = prev_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                prev_pt = new_pt.get();
            }
        }
    }

    if (min_tar_valid >= 2) {
        auto &prev_tar_pt = tar_line->list[min_tar_valid - 1];
        auto next_pt = src_line->list.front().get();
        if (!alg::judge_front(prev_tar_pt->pos, next_pt->pos, next_pt->dir)) {
            for (int i = min_tar_valid - 2; i >= 0; --i) {
                auto &pt = tar_line->list[i];
                auto new_pt = std::make_shared<LaneCenterFeature>();
                new_pt->init(pt);
                new_pt->group_line = src_line;
                new_pt->line_id = src_line->id;
                VEC_INSERT(src_line->list, new_pt);
                VEC_INSERT(src_line->match_list, {pt});
                VEC_INSERT(src_line->match_score, pt->score);
                src_line->match_list_point.insert(
                        src_line->match_list_point.begin(), std::vector<LaneCenterFeature*>());
                if (next_pt != NULL) {
                    next_pt->prev = new_pt.get();
                    new_pt->next = next_pt;
                }
                trail_tree.insert(new_pt->pos, new_pt.get());
                next_pt = new_pt.get();
            }
            for (int64_t i = 0; i < src_line->list.size(); ++i) {
                auto &src_pt = src_line->list[i];
                src_pt->line_index = i;
            }
        }
    }
    src_line->merge_lines.push_back(tar_line->src);
    return fsdmap::SUCC;
}

bool RoadModelProcMergeFeature::is_match_line_lane_center(RoadModelSessionData* session,
        LaneCenterGroupLine* src, LaneCenterLine* tar, double &score) {
    if(src->matched_line_ids.count(tar->cur_line_id) > 0) {
        score = 1;
        return true;
    }

    // double rate_thres = 0.3;
    double rate_thres = FLAGS_merge_feature_match_rate_threshold;

    int total_num = tar->list.size();
    int total_num_origin = tar->list.size();
    int match_num = 0;
    int64_t tar_max_index = 0;
    int64_t src_max_index = 0;
    int64_t tar_min_index = INT_MAX;
    int64_t src_min_index = INT_MAX;
    for (int64_t i = 0; i < total_num; ++i) {
        auto &pt = tar->list[i];
        auto &map = pt->merge_match_map;
        if (map.find(src) != map.end()) {
            if (map[src] != NULL) {
               ++match_num;
               auto &src_pt = map[src];
               src_max_index = src_pt->line_index;
               tar_max_index = i;
               tar_min_index = std::min(tar_min_index, i);
               src_min_index = std::min(src_min_index, src_pt->line_index);
            }
        }
    }
    if (src_min_index == 0) {
        total_num -= tar_min_index;
    }
    if (src_max_index == src->list.size() - 2) {
        total_num -= (tar->list.size() - tar_max_index - 1);
    }
    score = (double) match_num / total_num + 1e-6;
    if (match_num == 1 && total_num_origin > 1) {
        score = (double) match_num / total_num_origin + 1e-6;
    }
    // if (score > rate_thres) {
    //     int a = 1;
    // }
    // LOG_DEBUG("merge_lane_center[score={:.2f}, match={}, total={}, "
    //         "tar_max={}, tar_min={}, src_max={}, src_min={}]",
    //         score, match_num, total_num, 
    //         tar_max_index, tar_min_index, src_max_index, src_min_index);
    if (score > rate_thres) {
        return true;
    }
    return false;
}


std::pair<int, int> RoadModelProcMergeFeature::search_key_pose(RoadModelSessionData* session,
        KeyPoseTree &tree, Eigen::Vector3d pos, Eigen::Vector3d dir, std::string &trail_id,
        KeyPose* curr_poss) {
    double h_dis_thres = FLAGS_merge_feature_single_h_dis; // 30m
    double v_dis_thres = FLAGS_merge_feature_single_v_dis; // 30m
    double theta_thres = FLAGS_merge_feature_theta_thres;
    std::vector<KeyPose*> search_sec;
    tree.search(pos, h_dis_thres, search_sec);
    int poss_num = 1;
    // 在pos点30m范围内搜索所有最近邻ins位置，在这些位置中筛选出pos到这些位置方向向量距离小于30m的位置，
    // 对该位置点对应的轨迹id累加，表示该轨迹id离pos位置较近的轨迹点个数
    UMAP<std::string, int> trail_map; // <轨迹id，该轨迹id中离pos位置较近的轨迹点个数>
    for (auto &poss : search_sec) {
        // double theta =alg::calc_theta(poss->dir, dir, true);
        // if (theta > theta_thres) {
        //     continue;
        // }
        
        double v_dis = alg::calc_vertical_dis(pos, poss->pos, poss->dir); // 计算pos点到直线poss->dir的距离
        if (v_dis > v_dis_thres) {
            continue;
        }
        trail_map[poss->trail_id]++;
        // if (poss->trail_id != trail_id) {
        //     continue;
        // } // modified by ccj, 20241219, 多趟轨迹在semantic阶段已经被融合到一起了，不需要再判断
        if (curr_poss == poss) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(pos, poss->pos, poss->dir);
        if (h_dis > h_dis_thres) {
            continue;
        }
        
        ++poss_num;
    }
    if (poss_num == 1) {
        int a = 1;
    }
    return std::move(std::make_pair(poss_num, trail_map.size()));
}

int64_t RoadModelProcMergeFeature::vote_attr_by_fls(RoadModelSessionData* session,
        LaneLineSample *base_fls) {
    double radius = FLAGS_merge_feature_vote_yellow_radius;
    double scope = FLAGS_merge_feature_vote_yellow_scope;
    double theta_threshold = FLAGS_merge_feature_vote_yellow_theta_treshold;
    double dis_threshold = FLAGS_merge_feature_vote_yellow_dis_treshold;
    auto &pos = base_fls->pos;
    std::vector<LaneLineSample*> search_sec;
    session->lane_line_sample_tree.search(pos, radius, search_sec);
    std::vector<LaneLineSample*> attr_data_list;
    int64_t base_num = 0;
    UMAP<std::string, std::pair<LaneLineSample*, double>> data_map;
    for (auto &fls : search_sec) {
        if (fls->src->src_status != 1 && fls->src->src_status != 4 && fls->src->src_status != 5) {
            continue;
        }
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        if (v_dis > dis_threshold) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, base_fls->pos, base_fls->dir);
        if (h_dis > scope) {
            continue;
        }
        double theta = alg::calc_theta(fls->dir, base_fls->dir, true);
        if (theta > theta_threshold) {
            continue;
        }
        ++base_num;
        if (MAP_NOT_FIND(data_map, fls->src->line_id)) {
            data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        } else if (data_map[fls->src->line_id].second > h_dis) {
            data_map[fls->src->line_id] = std::make_pair(fls, h_dis);
        }
    }
    for (auto &tit : data_map) {
        attr_data_list.push_back(tit.second.first);
    }
    int64_t num_yellow = vote_yellow(session, base_fls, attr_data_list);
    int64_t num_double = vote_double(session, base_fls, attr_data_list);
    int64_t num_geo = vote_geo(session, base_fls, attr_data_list);
    return base_num;
}

int64_t RoadModelProcMergeFeature::vote_yellow(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    double min_thres = FLAGS_merge_feature_vote_yellow_min_threshold;
    double max_thres = FLAGS_merge_feature_vote_yellow_max_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 1) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        total_value += score * fls->src->attr.color;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    if (value > min_thres && value < max_thres) {
        // base_fls->attr.color = 2;
        base_fls->attr.color = 3;
    } else if (value <= min_thres){
        // base_fls->attr.color = 1;
        base_fls->attr.color = 2;
    } else {
        base_fls->attr.color = (int) (value + 0.5);
    }
    LOG_INFO("vote:{} :{}",value, base_fls->attr.color);
    return valid_num;
}

// 投票计算是否是双线
int64_t RoadModelProcMergeFeature::vote_double(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    double value_threshold = FLAGS_merge_feature_vote_double_value_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 4 && fls->src->src_status != 5) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        float is_double = fls->src->attr.is_double_line == 1 ? 1 : 0;
        total_value += score * is_double;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    if (value > value_threshold) {
        base_fls->attr.is_double_line = 1;
    } else {
        base_fls->attr.is_double_line = 0;
    }
    return valid_num;
}

// 投票计算几何属性
int64_t RoadModelProcMergeFeature::vote_geo(RoadModelSessionData* session,
        LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list) {
    double dis_lambda = FLAGS_merge_feature_vote_yellow_dis_lambda;
    // double value_threshold = FLAGS_merge_feature_vote_geo_value_threshold;
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    for (auto &fls : attr_data_list) {
        if (fls->src->src_status != 5 && fls->src->src_status != 4 && fls->src->src_status != 1) {
            continue;
        }
        ++valid_num;
        double v_dis = alg::calc_vertical_dis(fls->pos, base_fls->pos, base_fls->dir);
        double score = dis_lambda / (dis_lambda + v_dis); // v_dis越小，score得分越高
        total_value += score * fls->src->attr.geo;
        total_score += score;
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    double value = total_value / total_score;
    base_fls->attr.geo = (int)(value + 0.5);
    return valid_num;
}

// 用与base_fls匹配上的点集attr_data_list来统计base_fls属于的类别，
// 用attr_data_list中出现频率最高的类别作为base_fls的类别
int64_t RoadModelProcMergeFeature::vote_type(RoadModelSessionData* session,
        BoundaryFeature *base_fls, std::vector<BoundaryFeature*> &attr_data_list) {
    std::vector<std::pair<double, int>> data_list;

    double total_score = 0;
    double total_value = 0;
    int64_t valid_num = 0;
    UMAP<int, int> type_map;
    for (auto &fls : attr_data_list) {
        if (fls->src_status != 1) {
            continue;
        }
        ++valid_num;
        if(MAP_NOT_FIND(type_map,fls->sub_type)){
            type_map[fls->sub_type] = 0;
        }
        else{
            type_map[fls->sub_type]++;
        }
    }
    if (valid_num <= 0) {
        return valid_num;
    }
    std::vector<std::pair<int, int>> temp_type(type_map.begin(), type_map.end());
    std::sort(temp_type.begin(), temp_type.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    base_fls->sub_type = temp_type[0].first;
    return valid_num;
}

bool RoadModelProcMergeFeature::match_ground_by_point(RoadModelSessionData* session, 
        Eigen::Vector3d &pos) {

    return true;

    if (!session->opt_ground_pcd) {
        return false;
    }
    double radius = FLAGS_merge_feature_match_ground_radius;
    utils::CloudPoint searchPoint;
    searchPoint.x = pos.x();
    searchPoint.y = pos.y();
    searchPoint.z = 0;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    double min_dis = DBL_MAX;
    utils::CloudPoint* min_pt = NULL;
    if (session->ground_kdtree.radiusSearch(searchPoint, radius,
                pointIdxRadiusSearch, pointRadiusSquaredDistance) <= 0) {
        return false;
    }
    // for (int64_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
    //     auto &pt = session->opt_ground_pcd->points[pointIdxRadiusSearch[i]];
    //     double dis = pointRadiusSquaredDistance[i];
    //     if (dis < min_dis) {
    //         min_dis = dis;
    //         min_pt = &pt;
    //     }
    // }
    // if (min_pt == NULL) {
    //     return false;
    // }
    // pos.z() = min_pt->intensity;
    return true;
}

LaneCenterFeature* RoadModelProcMergeFeature::match_line_long_distance(
        RoadModelSessionData* session,
        LaneCenterFeature* src_pt, LaneCenterLine* tar_line, int64_t index) {
    // double scope = FLAGS_merge_feature_match_line_long_scope;
    // double dis_thres = FLAGS_merge_feature_match_line_long_max_dis_threshold;
    // auto &tar_pt = tar_line->list[index];
    // double total_dis = 0;
    // auto prev_lc = tar_pt;
    // std::vector<LaneCenterFeature*> line_vec;
    // for (int i = index + 1; i < tar_line->list.size(); ++i) {
    //     auto &next_lc = tar_line->list[i];
    //     total_dis += alg::calc_dis(prev_lc->pos, next_lc->pos);
    //     if (total_dis > scope) {
    //         break;
    //     }
    //     if (next_lc->group_pt == NULL) {
    //         continue;
    //     }
    //     line_vec.clear();
    //     if (!src_pt->on_same_line(scope, next_lc->group_pt, true, line_vec)) {
    //         continue;
    //     }
    //     double min_dis = DBL_MAX;
    //     LaneCenterFeature* min_lc = NULL;
    //     Eigen::Vector3d cross_point;
    //     for (auto &tmp_lc : line_vec) {
    //         for (auto &tmp_next_ptr : tmp_lc->context.all_next) {
    //             auto tmp_next = tmp_next_ptr.src;
    //             auto tmp_dir = alg::get_dir(tmp_next->pos, tmp_lc->pos);
    //             Eigen::Vector3d v_pt = alg::get_vertical_pos(tar_pt->pos, tmp_dir, 50, true);
    //             if (!alg::get_cross_point_for_segment(tmp_lc->pos, tmp_next->pos,
    //                         tar_pt->pos, v_pt, cross_point, 1)) {
    //                 continue;
    //             }
    //             double dis = alg::calc_dis(tar_pt->pos, cross_point);
    //             if (dis > min_dis) {
    //                 min_dis = dis;
    //                 min_lc = tmp_lc;
    //             }
    //         }
    //     }
    //     if (min_lc != NULL && min_dis > dis_thres) {
    //         return min_lc;
    //     }

    // }
    
    return NULL;
}

int RoadModelProcMergeFeature::gen_new_lane_center_group_merge(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, LaneCenterLine* tar_line,
        int64_t start_index, int64_t end_index, LaneCenterFeatureTree &trail_tree) {
    // auto new_line = std::make_shared<LaneCenterGroupLine>();
    // new_line->id = tar_line->id;
    // LaneCenterFeature* prev_pt = NULL;
    // for (int64_t i = start_index; i <= end_index; ++i) {
    //     auto &pt = tar_line->list[i];
    //     auto new_pt = std::make_shared<LaneCenterFeature>();
    //     new_pt->init(pt);
    //     new_pt->group_line = new_line.get();
    //     new_pt->line_index = new_line->list.size();
    //     new_pt->line_id = new_line->id;
    //     new_line->list.push_back(new_pt);
    //     new_line->match_list.push_back({pt});
    //     new_line->match_score.push_back(pt->score);
    //     new_line->match_list_point.resize(new_line->match_list.size());
    //     pt->group_pt = new_pt.get();
    //     if (prev_pt != NULL) {
    //         prev_pt->next = new_pt.get();
    //         new_pt->prev = prev_pt;

    //         auto &raw_prev_pt = tar_line->list[i - 1];
    //         double score = raw_prev_pt->context.all_next[0];
    //         prev_pt->context.set_next(new_pt.get());
    //         new_pt->context.set_prev(prev_pt);
    //         prev_pt->context.score_next.push_back(score);
    //         new_pt->context.score_prev.push_back(score);
    //     }
    //     trail_tree.insert(new_pt->pos, new_pt.get());
    //     prev_pt = new_pt.get();
    // }
    // new_line->merge_lines.push_back(tar_line->src);
    // trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::update_lane_center_group_line_merge(RoadModelSessionData* session,
        LaneCenterFeature* tar_lc, LaneCenterFeatureTree &trail_tree, int mode) {
    // double scope = FLAGS_merge_feature_mutil_scope;
    // int64_t max_tar_valid = 0;
    // int64_t max_src_valid = 0;
    // int64_t min_tar_valid = INT_MAX;
    // int64_t min_src_valid = INT_MAX;

    // auto &src_lc = tar_lc->group_pt;
    // auto &src_line = src_lc->group_line;
    // int64_t src_index = src_lc->line_index;

    // double total_score = src_line->match_score[src_index];
    // double v_dis = alg::calc_vertical_dis(tar_lc->pos, src_lc->pos, src_lc->dir, true);
    // if (fabs(v_dis) > scope) {
    //     return fsdmap::FAIL; 
    // }
    // double update_rate = tar_lc->score / (total_score + tar_lc->score);
    // double opt_dis = update_rate * v_dis;
    // Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_lc->pos, src_lc->dir, opt_dis);
    // if (isnan(opt_pos.x()) || isnan(opt_pos.y()) 
    //         || isnan(src_lc->dir.x()) || isnan(src_lc->dir.y())) {
    //     int a = 1;
    // }
    // session->debug_pos(opt_pos);
    // src_lc->pos = opt_pos;
    // src_lc->dir += update_rate * tar_lc->dir;
    // src_lc->dir.normalize();


    // session->debug_pos(src_lc->pos);
    // trail_tree.insert(src_lc->pos, src_lc);
    // src_line->match_list[src_index].push_back(tar_lc);
    // src_line->match_score[src_index] += tar_lc->score;
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::make_lane_line_raw(RoadModelSessionData* session) {
    // double dft_width = 3 / 2;
    // UMAP<LaneCenterFeature*, int> used_map;
    // for (auto &line : session->merge_lane_center_list) {
    //     for (auto &lc : line->list) {
    //         session->debug_pos(lc->pos);
    //         if (lc->invalid()) {
    //             continue;
    //         }
    //         if (MAP_FIND(used_map, lc.get())) {
    //             continue;
    //         }
    //         used_map[lc.get()] = 1;
    //         Eigen::Vector3d left_pos = alg::get_vertical_pos(lc->pos, lc->dir, -dft_width);
    //         Eigen::Vector3d right_pos = alg::get_vertical_pos(lc->pos, lc->dir, dft_width);
    //         lc->init(left_pos, lc->dir, right_pos, lc->dir);
    //         if (lc->next != NULL && !lc->next->invalid()) {
    //             double theta = alg::calc_theta(lc->dir, lc->next->dir);
    //             if (theta > 90) {
    //                 LOG_WARN("lc dir error, earse[dir1={},{}, dir2={},{}]",
    //                         lc->dir.x(), lc->dir.y(), lc->next->dir.x(), lc->next->dir.y());
    //                 lc->next->prev = NULL;
    //                 lc->next = NULL;
    //                 continue;
    //             }
    //             if (lc->context.set_next(lc->next)) {
    //                 lc->context.score_next.push_back(lc->next->score);
    //             }
    //             if (lc->next->context.set_prev(lc.get())) {
    //                 lc->next->context.score_prev.push_back(lc->score);
    //             }
    //         }

    //         session->lane_center_feature_raw.push_back(lc.get());
    //         // session->lane_center_gen_tree.insert(lc->pos, lc.get());
    //     }
    // }
    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::merge_multi_lane_center(RoadModelSessionData* session) {
    // // int min_size = FLAGS_merge_feature_proc_min_size;
    // // int min_match_size = FLAGS_merge_feature_proc_min_match_size;
    // std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_list;

    // int64_t raw_line_size = 0;
    // int64_t merge_line_size = 0;
    // int64_t raw_rela_size = 0;
    // int64_t merge_rela_size = 0;
    // LaneCenterFeatureTree trail_tree;
    // make_lane_line_raw(session);
    // // get_lc_max_length(session, true);
    // int64_t lc_num = get_lc_max_length(session, session->lane_center_feature_raw, false);
    // SORT(session->lane_center_feature_raw,
    //         [](const LaneCenterFeature* l, const LaneCenterFeature* r) {
    //             return l->context.next_max_length > r->context.next_max_length;
    //         });
    // int64_t times = 0;
    // int64_t patch = 0;
    // while (lc_num > 0) {
    //     LOG_INFO("merge_lane_center[times={}, lc_num={}", times++, lc_num);
    //     for (auto &lc : session->lane_center_feature_raw) {
    //         if (lc->invalid()) {
    //             continue;
    //         }
    //         if (lc->merge_status == 1) {
    //             continue;
    //         }
    //         bool has_valid_prev = false;
    //         for (auto &prev_lc : lc->context.all_prev) {
    //             if (!prev_lc->invalid() && prev_lc->merge_status == 0) {
    //                 has_valid_prev = true;
    //                 break;
    //             }
    //         }
    //         if (has_valid_prev) {
    //             continue;
    //         }
    //         auto tmp_line = session->add_ptr(lane_center_line_list);
    //         extract_lane_line(session, lc, tmp_line.get());
    //         // if (tmp_line->list.size() <= 1) {
    //         //     continue;
    //         // }
    //         for (auto &pt : tmp_line->list) {
    //             raw_rela_size += pt->context.all_next.size();
    //         }
    //         raw_line_size += tmp_line->list.size();
    //         match_lane_center_line_raw(session, session->lane_center_line_group_list_raw, 
    //                 trail_tree, tmp_line.get());

    //         lc_num -= tmp_line->list.size();
    //         save_log_multi_lane_center(session, session->lane_center_line_group_list_raw,
    //                 tmp_line.get(), patch++);
    //     }
    // }

    // for (auto &line : session->lane_center_line_group_list) {
    //     for (auto &lc : line->list) {
    //         std::fill(lc->context.score_next.begin(), lc->context.score_next.end(), 0);
    //         std::fill(lc->context.score_prev.begin(), lc->context.score_prev.end(), 0);
    //     }
    // }
    // // 边投票
    // double scope = 10;
    // std::vector<LaneCenterFeature*> line_vec;
    // for (auto &lc : session->lane_center_feature_raw) {
    //     if (lc->invalid()) {
    //         continue;
    //     }
    //     if (lc->group_pt == NULL) {
    //         continue;
    //     }
    //     for (int i = 0; i < lc->group_pt->context.all_next.size(); ++i) {
    //         auto &group_next = lc->group_pt->context.all_next[i];
    //         auto &group_next = lc->group_pt->context.all_next[i];
    //         for (auto &next : lc->context.all_next) {
    //             if (next->invalid()) {
    //                 continue;
    //             }
    //             auto &next_group = next->group_pt;
    //             if (next_group == NULL) {
    //                 continue;
    //             }
    //             line_vec.clear();
    //             if (!group_next->on_same_line(scope, next_group, true, line_vec)) {
    //                 continue;
    //             }
    //             lc->group_pt->context.score_next[i] += next->score;
    //             bool has_valid = false;
    //             for (int j = 0; j < group_next->context.all_prev.size(); ++j) {
    //                 auto tmp_group = group_next->context.all_prev[j];
    //                 if (tmp_group == lc->group_pt) {
    //                     group_next->context.score_prev[j] += next->score;
    //                     has_valid = true;
    //                     break;
    //                 }
    //             }
    //             if (!has_valid) {
    //                 int a = 1;
    //             }
    //         }
    //     }
    // }


    // // session->lane_center_gen_tree.RemoveAll();
    // for (auto &line : session->lane_center_line_group_list) {
    //     for (auto &lc : line->list) {
    //         // session->lane_center_gen_tree.insert(lc->pos, lc.get());
    //         merge_rela_size += lc->context.all_next.size();
    //         session->lane_center_feature_merge_raw.push_back(lc.get());
    //     }
    //     merge_line_size += line->list.size();
    // }

    // get_lc_max_length(session, session->lane_center_feature_merge_raw, true);
    // get_lc_max_length(session, session->lane_center_feature_merge_raw, false);
    // LOG_DEBUG("merge_lane_line[lsg={}, raw_lsg={}, rela={}, raw_rela={}]", 
    //         merge_line_size, raw_line_size, merge_rela_size, raw_rela_size);
    // // 过滤
    // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    //     auto &group_line = session->lane_center_line_group_list_raw[i];
    //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
    //         int valid_index = j;
    //         if (j > 0 && j == group_line->list.size() - 1) {
    //             valid_index = j - 1;
    //         }
    //         auto &pt = group_line->list[j];
    //         session->debug_pos(pt->pos);
    //         UMAP<std::string, double> trail_map;
    //         int total_num = 1;
    //         for (auto &raw_pt : group_line->match_list[valid_index]) {
    //             auto trail_id = raw_pt->trail_id;
    //             if (raw_pt->src_status == 2) {
    //                 trail_id = "sem";
    //             }
    //             total_num = std::max(raw_pt->match_trail_num, total_num);
    //             if (MAP_NOT_FIND(trail_map, trail_id)) {
    //                 trail_map[trail_id] = raw_pt->score;
    //                 continue;
    //             }
    //             if (trail_map[trail_id] < raw_pt->score) {
    //                 trail_map[trail_id] = raw_pt->score;
    //             }
    //         }
    //         double match_num = trail_map.size();
    //         if (MAP_FIND(trail_map, "sem")) {
    //             pt->score = 1;
    //         } else {
    //             pt->score = match_num / total_num;
    //         }

    //         DLOG_POINT(pt->pos, "merge_lane_line_d[src={}, s={:.2f}, m1={}, m2={}, "
    //                 "l1={:.2f}, l2={:.2f}]",
    //                 pt->src_status, pt->score, match_num, total_num, 
    //                 pt->context.prev_max_length, pt->context.next_max_length);
    //     }
    // }
    // for (auto &lc : session->lane_center_feature_merge_raw) {
    //     if (lc->context.all_next.size() == 0 && lc->context.all_prev.size() == 0) {
    //         lc->filter_status = 17;
    //         continue;
    //     }
    //     lc->context.max_length = lc->context.prev_max_length + lc->context.next_max_length;
    //     if (lc->context.max_length < FLAGS_merge_feature_valid_min_length) {
    //         lc->filter_status = 18;
    //         continue;
    //     }
    // }
    
    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::extract_lane_line(RoadModelSessionData* session,
        LaneCenterFeature* lc, LaneCenterLine* tar_line) {
    // auto next_lc = lc;
    // while(next_lc != NULL) {
    //     if (next_lc->invalid()) {
    //         break;
    //     }
    //     if (next_lc->merge_status == 1) {
    //         break;
    //     }
    //     tar_line->list.push_back(next_lc);
    //     next_lc->merge_status = 1;
    //     if (next_lc->context.all_next.size() == 0) {
    //         break;
    //     }
    //     if (next_lc->context.all_next.size() > 1) {
    //         SORT(next_lc->context.all_next,
    //                 [](const LaneCenterFeature* l, const LaneCenterFeature* r) {
    //                 return l->context.next_max_length > r->context.next_max_length;
    //                 });
    //     }
    //     next_lc = next_lc->context.all_next.front();
    // }
    return fsdmap::SUCC;
}

int64_t RoadModelProcMergeFeature::get_lc_max_length(RoadModelSessionData* session,
        std::vector<LaneCenterFeature*> &lc_list, bool next) {
    // int64_t lc_num = 0;
    // // std::vector<LaneCenterFeature*> new_lc_list;
    // for (auto &lc : lc_list) {
    //     if (lc->invalid()) {
    //         continue;
    //     }
    //     lc->length_status = 0;
    //     ++lc_num;
    //     // new_lc_list.push_back(lc);
    // }
    // int64_t valid_num = lc_num;
    // int64_t prev_lc_num = lc_num;
    // bool skip_calc = false;
    // int64_t times = 0;
    // while (lc_num > 0) {
    //     LOG_INFO("get_max_length[next={}, times={}, lc_num={}", next, times++, lc_num);
    //     for (auto &lc : lc_list) {
    //         auto next_lc = lc;
    //         while (next_lc != NULL) {
    //             session->debug_pos(next_lc->pos);
    //             if (next_lc->invalid()) {
    //                 break;
    //             }
    //             if (next_lc->length_status == 1) {
    //                 break;
    //             }
    //             double max_length = 0;
    //             bool has_valid_prev = false;
    //             auto &all_vec = next ? next_lc->context.all_prev : next_lc->context.all_next;
    //             for (auto &prev_lc_ptr : all_vec) {
    //                 auto &prev_lc = prev_lc_ptr.src;
    //                 if (!prev_lc->invalid() && prev_lc->length_status == 0) {
    //                     has_valid_prev = true;
    //                     break;
    //                 }
    //                 double prev_max = next ?
    //                     prev_lc->context.prev_max_length : prev_lc->context.next_max_length;
    //                 double dis = alg::calc_dis(next_lc->pos, prev_lc->pos);
    //                 prev_max += dis;
    //                 max_length = std::max(max_length, prev_max);
    //             }
    //             if (has_valid_prev && !skip_calc) {
    //                 break;
    //             }
    //             if (next) {
    //                 next_lc->context.prev_max_length = max_length;
    //             } else {
    //                 next_lc->context.next_max_length = max_length;
    //             }
    //             next_lc->length_status = 1;
    //             --lc_num;
    //             auto &all_next = next ? next_lc->context.all_next : next_lc->context.all_prev;
    //             if (all_next.size() != 1) {
    //                 break;
    //             }
    //             next_lc = all_next.front().src;
    //         }
    //     }
    //     skip_calc = prev_lc_num == lc_num;
    //     prev_lc_num = lc_num;
    // }
    // return valid_num;
}


int RoadModelProcMergeFeature::match_lane_center_line_raw(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    // double radius = FLAGS_merge_feature_search_radius;
    // double scope = FLAGS_merge_feature_mutil_scope;
    // double theta_thres = FLAGS_merge_feature_mutil_theta;

    // std::vector<LaneCenterFeature*> secs;
    // UMAP<LaneCenterGroupLine*, int> cache_map;
    // UMAP<LaneCenterFeature*, int> used_map;
    // for (int64_t j = 0; j < tar_line->list.size(); ++j) {
    //     auto &pt = tar_line->list[j];
    //     session->debug_pos(pt->pos);
    //     secs.clear();
    //     used_map.clear();
    //     LaneCenterFeature* min_from_lc = NULL;
    //     LaneCenterFeature* min_to_lc = NULL;
    //     double min_dis = DBL_MAX;
    //     trail_tree.search(pt->pos, radius, secs);
    //     Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true);
    //     Eigen::Vector3d cross_point = {0, 0, 0};
    //     for (int64_t k = 0; k < secs.size(); ++k) {
    //         auto &src_pt = secs[k];
    //         if (MAP_FIND(used_map, src_pt)) {
    //             continue;
    //         }
    //         used_map[src_pt] = 1;
    //         session->debug_pos2(pt->pos, src_pt->pos);
    //         bool front = alg::judge_front(pt->pos, src_pt->pos, src_pt->dir);
    //         auto &next_list = front ? src_pt->context.all_next : src_pt->context.all_prev;
    //         if (next_list.size() == 0) {
    //             // if (pt->next != NULL) {
    //             //     continue;
    //             // }
    //             // double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir);
    //             // if (h_dis > radius / 2) {
    //             //     continue;
    //             // } 
    //             // double dis = alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir);
    //             // if (dis > scope) {
    //             //     continue;
    //             // }
    //             // double theta = alg::calc_theta(src_pt->dir, pt->dir);
    //             // if (theta > theta_thres) {
    //             //     continue;
    //             // }
    //             // double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
    //             // if (s_dis > scope) {
    //             //     continue;
    //             // }
    //             // if (dis < min_dis) {
    //             //     min_dis = dis;
    //             //     min_from_lc = src_pt;
    //             //     min_to_lc = NULL;
    //             // }
    //         } else {
    //             double min_tmp_dis = DBL_MAX;
    //             LaneCenterFeature* min_tmp_from_lc = NULL;
    //             LaneCenterFeature* min_tmp_to_lc = NULL;
    //             for (auto &next_lc_ptr : next_list) {
    //                 auto next_lc = next_lc_ptr.src;
    //                 if (!alg::get_cross_point_for_segment(src_pt->pos, next_lc->pos,
    //                             pt->pos, v_pt, cross_point, 1)) {
    //                     continue;
    //                 }
    //                 double dis = alg::calc_dis(pt->pos, cross_point);
    //                 if (dis > scope) {
    //                     // match_map[src_pt->group_line] = NULL;
    //                     continue;
    //                 }
    //                 Eigen::Vector3d tmp_dir;
    //                 if (front) {
    //                     tmp_dir = alg::get_dir(next_lc->pos, src_pt->pos);
    //                 } else {
    //                     tmp_dir = alg::get_dir(src_pt->pos, next_lc->pos);
    //                 }
    //                 double theta = alg::calc_theta(tmp_dir, pt->dir);
    //                 if (theta > theta_thres) {
    //                     continue;
    //                 }
    //                 double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, tmp_dir);
    //                 if (s_dis > scope) {
    //                     continue;
    //                 }
    //                 if (dis < min_tmp_dis) {
    //                     min_tmp_dis = dis;
    //                     if (front) {
    //                         min_tmp_from_lc = src_pt;
    //                         min_tmp_to_lc = next_lc;
    //                     } else {
    //                         min_tmp_from_lc = next_lc;
    //                         min_tmp_to_lc = src_pt;
    //                     }
    //                 }
    //             }
    //             if (min_tmp_from_lc != NULL) {
    //                 if (min_tmp_dis < min_dis) {
    //                     min_dis = min_tmp_dis;
    //                     min_from_lc = min_tmp_from_lc;
    //                     min_to_lc = min_tmp_to_lc;
    //                 }
    //                 // src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
    //             }
    //         }
    //     }
    //     pt->group_pt = min_from_lc;
    //     pt->raw_to_lc = min_to_lc;
    // }
    // using line_score = std::pair<LaneCenterGroupLine*, double>;
    // std::vector<std::tuple<int64_t, int64_t, int>> sub_line_list;
    // int64_t start_index = 0;
    // int64_t end_index = 0;
    // std::vector<LaneCenterFeature*> line_vec;
    // LaneCenterFeature* prev_pt = NULL;
    // for (int64_t j = 0; j < tar_line->list.size(); ++j) {
    //     auto &pt = tar_line->list[j];
    //     if (prev_pt == NULL) {
    //         start_index = j;
    //         end_index = j;
    //         prev_pt = pt;
    //         if (j == tar_line->list.size() - 1) {
    //             sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
    //         }
    //         continue;
    //     }
    //     if (prev_pt->group_pt == NULL) {
    //         if (pt->group_pt != NULL) {
    //             end_index = j - 1;
    //             sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
    //             start_index = j;
    //         }
    //         if (j == tar_line->list.size() - 1) {
    //             end_index = j;
    //             sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
    //         }
    //     } else {
    //         line_vec.clear();
    //         int mode = 1;
    //         if (pt->group_pt != NULL && prev_pt->group_pt->on_same_line(20, 
    //                     pt->group_pt, true, line_vec)) {
    //         } else {
    //             auto match_pt = match_line_long_distance(session, prev_pt->group_pt, tar_line, j);
    //             if (match_pt != NULL) {
    //                 pt->group_pt = match_pt;
    //             } else {
    //                 end_index = j - 1;
    //                 sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 1));
    //                 start_index = j;
    //                 mode = 0;
    //             }
    //         }
    //         if (j == tar_line->list.size() - 1) {
    //             end_index = j;
    //             sub_line_list.emplace_back(std::make_tuple(start_index, end_index, mode));
    //         }
    //     }
    //     prev_pt = pt;

    // }

    // for (auto &tuple : sub_line_list) {
    //     int64_t start_index = std::get<0>(tuple);
    //     int64_t end_index = std::get<1>(tuple);
    //     int type = std::get<2>(tuple);
    //     if (type == 0) {
    //         gen_new_lane_center_group_merge(session, trail_group, 
    //                 tar_line, start_index, end_index, trail_tree);
    //         
    //     } else if (type == 1) {
    //         for (int64_t i = start_index; i <= end_index; ++i) {
    //             auto &tar_lc = tar_line->list[i];
    //             update_lane_center_group_line_merge(session, tar_lc, trail_tree, 2);
    //         }
    //     }
    // }
    // for (int64_t i = 0; i < tar_line->list.size(); ++i) {
    //     auto &tar_lc = tar_line->list[i];
    //     for (int j = 0; j < tar_lc->context.all_prev.size(); ++j) {
    //         auto &prev_lc = tar_lc->context.all_prev[j];
    //         if (prev_lc->group_pt == NULL) {
    //             continue;
    //         }
    //         if (prev_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
    //             continue;
    //         }
    //         auto valid_group_lc = prev_lc->group_pt;
    //         if (alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
    //                     tar_lc->group_pt->dir)) {
    //             continue;
    //         }
    //         double score = tar_lc->context.score_prev[j];
    //         if (prev_lc->group_pt->context.set_next(tar_lc->group_pt)) {
    //             prev_lc->group_pt->context.score_next.push_back(score);
    //         }
    //         if (tar_lc->group_pt->context.set_prev(prev_lc->group_pt)) {
    //             tar_lc->group_pt->context.score_prev.push_back(score);
    //         }
    //     }
    //     for (int j = 0; j < tar_lc->context.all_next.size(); ++j) {
    //         auto &next_lc_ptr = tar_lc->context.all_next[j];
    //         auto &next_lc = next_lc_ptr.src;
    //         if (next_lc->group_pt == NULL) {
    //             continue;
    //         }
    //         if (next_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
    //             continue;
    //         }
    //         auto valid_group_lc = next_lc->group_pt;
    //         if (!alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
    //                     tar_lc->group_pt->dir)) {
    //             valid_group_lc = next_lc->raw_to_lc;
    //             if (next_lc->raw_to_lc != NULL 
    //                     && !alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
    //                         tar_lc->group_pt->dir)) {
    //             } else {
    //                 continue;
    //             }
    //         }
    //         double score = tar_lc->context.score_next[j];
    //         if (valid_group_lc->context.set_prev(tar_lc->group_pt)) {
    //             valid_group_lc->context.score_prev.push_back(score);
    //         }
    //         if (tar_lc->group_pt->context.set_next(valid_group_lc)) {
    //             tar_lc->group_pt->context.score_next.push_back(score);
    //         }
    //     }
    // }
    return fsdmap::SUCC;
}




int RoadModelProcMergeFeature::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature");
    session->set_display_name(total_log_name.c_str());

//    for (auto &trail : session->key_pose_map) {
//         auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
//         auto trail_ptr = &trail;
//         // session->thread_pool->schedule(
//         //         [log, trail_ptr, session, this](utils::ProcessBar *process_bar) {
//         log->color = {223, 130, 154};
//         auto &first_poss = trail_ptr->second.list.front();
//         for (auto &key_pose : trail_ptr->second.list) {
//             auto &ele = log->add(key_pose->pos, 1, key_pose->raw_no);
//             ele.label.opt_label = key_pose->raw_file_no;
//             // ele.label.time = key_pose->timestamp;
//         }
//     }


    UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &it : session->key_pose_map) {
        for (int64_t i = 0; i < it.second.boundary_line_group_list.size(); ++i) {
            auto &group_list = it.second.boundary_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
            log->color = {81, 89, 240};
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                if(prev) {
                    if(j < 2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1 || group_list->boundary_type == 1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.intensity_opt = j;
                        ele.label.score = pt->score;
                        if (pt->invalid()) {
                            ele.color = {255, 255, 100};
                        }
                        ele.label.label = 1; // 临时弄的
                        prev=pt.get();
                    }

                } else {
                     prev=pt.get();
                }
            }

            trail_map[it.first].push_back(log);
        }
        #if 1
        for (int64_t i = 0; i < it.second.lane_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
            log->color = {255, 255, 255};
            LaneLineSample*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        // auto &ele = log->add(pt->pos);
                        ele.label.opt_label = i;
                        ele.label.intensity_opt = j;
                        ele.label.label = 2; // 临时弄的
                        ele.label.score = pt->score;
                        if (pt->invalid()) {
                            ele.color = {255, 0, 0};
                        }
                        ele.label.cloud_pano_seg=pt->attr.type;
                        ele.label.cloud_line_seg=pt->attr.color;
                        // 
                        ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                        // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)
                        prev=pt.get();
                    }
                } else {
                    prev=pt.get();
                }
            }

            trail_map[it.first].push_back(log);
        }

        for (int64_t i = 0; i < it.second.lane_center_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_center_line_group_list[i];
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
                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.score = pt->score;
                        ele.label.label = 3; // 临时弄的
                        if (pt->invalid()) {
                            ele.color = {100, 100, 100};
                        }
                        prev=pt.get();
                        ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                    } 
                } else {
                    prev=pt.get();
                }
              
            }
            trail_map[it.first].push_back(log);
        }
        #endif

        continue;
    }

    // for (auto &it : trail_map) {
    //     auto log_name = session->get_debug_dir("merge_detail/1_{}_{}.png", 
    //             "merge_single_trail", it.first);
    //     utils::save_display_image(log_name.c_str(), session->_scope, it.second);

    //     auto pcd_name = session->get_debug_dir("merge_detail/1_{}_{}.pcd", 
    //             "merge_single_trail", it.first);
    //     utils::save_display_pcd(pcd_name.c_str(), session->_scope, it.second);
    // }
    session->save_debug_info(total_log_name.c_str());
    // session->set_display_name("merge_feature_multi_lane_center");
    // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    //     auto &group_list = session->lane_center_line_group_list_raw[i];
    //     for (int64_t j = 0; j < group_list->list.size(); ++j) {
    //         auto &pt = group_list->list[j];
    //         session->debug_pos(pt->pos);
    //         for (int k = 0; k < pt->context.all_next.size(); ++k) {
    //             auto &next_ptr = pt->context.all_next[k];
    //             auto &next = next_ptr.src;
    //             // if (next->group_line == pt->group_line) {
    //             //     continue;
    //             // }

    //             auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "merge_raw_lane");
    //             tlog->color = {255, 255, 0};
    //             auto score = next_ptr.score;
    //             if (score < 2) {
    //                 tlog->color = {255, 0, 0};
    //             }
    //             auto &ele1 = tlog->add(pt->pos, 1, group_list->match_list[j].size());
    //             ele1.label.score = score;
    //             auto &ele2 = tlog->add(next->pos, 1, group_list->match_list[j].size());
    //             ele2.label.score = score;
    //         }
    //     }
    // }
    // session->save_debug_info("merge_feature_multi_lane_center");

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_debug_info_only_lane(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_lane_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &it : session->key_pose_map) {
        for (int64_t i = 0; i < it.second.lane_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_lane");
            log->color = {255, 255, 255};
            LaneLineSample*prev=NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];

                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        // auto &ele = log->add(pt->pos);
                        ele.label.opt_label = i;
                        ele.label.intensity_opt = j;
                        ele.label.label = 2; // 临时弄的
                        ele.label.score = pt->score;
                        if (pt->invalid()) {
                            ele.color = {255, 0, 0};
                        }
                        ele.label.cloud_pano_seg=pt->attr.type;
                        ele.label.cloud_line_seg=pt->attr.color;
                        // 
                        ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                        // LOG_INFO("merge color:{} type:{}",pt->attr.color,pt->attr.type)

                        prev=pt.get();
                    }
                } else {
                    prev=pt.get();
                }
            }
            // trail_map[it.first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_debug_info_only_lane_center(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_lane_center_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &it : session->key_pose_map) {
        for (int64_t i = 0; i < it.second.lane_center_line_group_list.size(); ++i) {
            auto &group_list = it.second.lane_center_line_group_list[i];
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
                if(prev) {
                    if(j<2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                    // if(alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.score = pt->score;
                        ele.label.label = 3; // 临时弄的
                        if (pt->invalid()) {
                            ele.color = {100, 100, 100};
                        }
                        ele.label.intensity_opt=alg::calc_theta(pt->dir)*57.3;
                        prev=pt.get();
                    } 
                } else {
                    prev=pt.get();
                }
              
            }
            // trail_map[it.first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_debug_info_only_boundary(RoadModelSessionData* session, int index) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    std::string total_log_name = utils::fmt("merge_feature_boundary_" + std::to_string(index));
    session->set_display_name(total_log_name.c_str());

    // UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &it : session->key_pose_map) {
        for (int64_t i = 0; i < it.second.boundary_line_group_list.size(); ++i) {
            auto &group_list = it.second.boundary_line_group_list[i];
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
            log->color = {81, 89, 240};
            BoundaryFeature* prev = NULL;
            for (int64_t j = 0; j < group_list->list.size(); ++j) {
                auto &pt = group_list->list[j];
                if(prev) {
                    if(j < 2 || alg::calc_dis(pt->pos,prev->pos)>=4 || j==group_list->list.size()-1 || group_list->boundary_type == 1) {
                        auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
                        ele.label.opt_label = i;
                        ele.label.intensity_opt = j;
                        ele.label.score = pt->score;
                        if (pt->invalid()) {
                            ele.color = {255, 255, 100};
                        }
                        ele.label.label = 1; // 临时弄的
                        prev=pt.get();
                    }
                } else {
                     prev=pt.get();
                }
            }

            // trail_map[it.first].push_back(log);
        }
    }

    session->save_debug_info(total_log_name.c_str());

    return fsdmap::SUCC;
}


int RoadModelProcMergeFeature::save_frame_log_lane(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }

    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->lane_line_group_list.size(); ++i) {
        auto &group_list = line->lane_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        log->color = {0, 255, 0};
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
          
        }
        log_vec.push_back(log.get());
    }
    auto &ins_vec = session->bev_frame_lane_line[poss->frame_id];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->type = utils::DisplayInfo::LINE;
        mlog->color = {100, 100, 125};
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }
    
    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_lane", 
            times, poss->frame_id);

    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_lane", 
            times, poss->frame_id);

    if (poss == line->list.back()) {
        log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_lane", 
                times, poss->frame_id);
        pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_lane", 
                times, poss->frame_id);
    }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_frame_log_boundary(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times, std::string line_id) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->boundary_line_group_list.size(); ++i) {
        auto &group_list = line->boundary_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        if (i == line->boundary_line_group_list.size()-1) {
            log->color = {255, 0, 255};
        } else {
            log->color = {0, 255, 0};
        }
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
            if(j == 0) {
                // auto line_index_log = session->add_ptr(ptr_vec);
                // log->type = utils::DisplayInfo::LINE_INDEX;
                // log_vec.push_back(line_index_log.get());
                // 添加车道组的索引日志，并设置为红色
                // auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}_{}", i, group_list->id);
                auto line_index_log = session->add_debug_log(utils::DisplayInfo::TEXT, "line_id:{}", group_list->id);
                line_index_log->color = {255, 0, 0}; // 红色
                line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
                log_vec.push_back(line_index_log);
            }
        }
        log_vec.push_back(log.get());

        // for (int64_t j = 0; j < group_list->list.size(); ++j) {
        // {
        //     if(j == 0) {
        //         // auto line_index_log = session->add_ptr(ptr_vec);
        //         // log->type = utils::DisplayInfo::LINE_INDEX;
        //         // log_vec.push_back(line_index_log.get());
        //         // 添加车道组的索引日志，并设置为红色
        //         // auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}_{}", i, group_list->id);
        //         auto line_index_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "line_id:{}", group_list->id);
        //         line_index_log->color = {255, 0, 0}; // 红色
        //         line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
        //         line_index_log->add(pt->pos); // 对第一个点添加到车道组索引日志
        //         // log_vec.push_back(line_index_log);
        //     }
        // }
    }

    // auto &ins_vec = session->bev_frame_boundary_line[poss->frame_id];
    auto &ins_vec = session->bev_frame_boundary_line[""];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->color = {100, 100, 125};
        mlog->type = utils::DisplayInfo::LINE;
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }
    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_boundary", 
            times, line_id);
    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_boundary", 
            times, line_id);

    // auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_boundary", 
    //         times, poss->frame_id);
    // auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_boundary", 
    //         times, poss->frame_id);
    // if (poss == line->list.back()) {
    //     log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_boundary", 
    //             times, poss->frame_id);
    //     pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_boundary", 
    //             times, poss->frame_id);
    // }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_frame_log_lane_center(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times) {
    if (!FLAGS_merge_feature_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (!FLAGS_merge_feature_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    for (int64_t i = 0; i < line->lane_center_line_group_list.size(); ++i) {
        auto &group_list = line->lane_center_line_group_list[i];
        auto log = session->add_ptr(ptr_vec);
        log->type = utils::DisplayInfo::LINE;
        log->color = {0, 255, 0};
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            // session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[i].size());
            ele.label.opt_label = i;
            if (pt->invalid()) {
                ele.color = {255, 255, 0};
            }
        }
        log_vec.push_back(log.get());
    }
    auto &ins_vec = session->bev_frame_lane_center_line[poss->frame_id];
    for (auto &ins : ins_vec) {
        auto mlog = session->add_ptr(ptr_vec);
        mlog->color = {100, 100, 125};
        mlog->type = utils::DisplayInfo::LINE;
        for (auto &pt : ins->list) {
            // session->debug_pos(pt->pos);
            auto &ele = mlog->add(pt->pos);
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
        }
        log_vec.push_back(mlog.get());
    }

    auto log_name = session->get_debug_dir("merge_detail/{}_{}_{}.png", "merge_frame_lane_center", 
            times, poss->frame_id);
    auto pcd_name = session->get_debug_dir("merge_detail/{}_{}_{}.pcd", "merge_frame_lane_center", 
            times, poss->frame_id);
    if (poss == line->list.back()) {
        log_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.png", "merge_frame_lane_center", 
                poss->frame_id, times);
        pcd_name = session->get_debug_dir("merge_detail/2_{}_{}_{}.pcd", "merge_frame_lane_center", 
                times, poss->frame_id, times);
    }
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);

    return fsdmap::SUCC;
}

int RoadModelProcMergeFeature::save_log_multi_lane_center(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &line_group,
        LaneCenterLine* tar_line, int times) {
    return fsdmap::SUCC;
}

}
}

/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
