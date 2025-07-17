


#include "road_model_proc_gen_lane_line.h"

DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

DEFINE_bool(gen_lane_line_enable, true, "gen_lane_line_enable");
DEFINE_bool(gen_lane_line_debug_pos_enable, true, "gen_lane_line_debug_enable");
DEFINE_bool(gen_lane_line_save_data_enable, true, "gen_lane_line_save_data_enable");
DEFINE_bool(gen_lane_line_save_data_save_detail_enable, true, "gen_lane_line_save_data_save_detail_enable");
DEFINE_bool(gen_lane_line_save_data_save_frame_enable, false, "gen_lane_line_save_data_save_frame_enable");
DEFINE_bool(gen_lane_line_save_data_frame_log_enable, false, "gen_lane_line_save_data_frame_log_enable");

DEFINE_bool(gen_lane_line_filter_line_by_length_enable, true, "gen_lane_line_filter_line_by_length_enable");
DEFINE_bool(gen_lane_line_vote_edge_all_enable, true, "gen_lane_line_vote_edge_all_enable");
DEFINE_double(gen_lane_line_search_lc_max_theta, 45, "gen_lane_line_search_lc_max_theta");
DEFINE_double(gen_lane_line_search_lc_max_radius, 10, "gen_lane_line_search_lc_max_radius");
DEFINE_double(gen_lane_line_search_lc_max_min_radius, 1.8, "gen_lane_line_search_lc_max_min_radius");
DEFINE_double(gen_lane_line_pair_score_dis_lambda, 20, "gen_lane_line_pair_score_dis_lambda");
DEFINE_double(gen_lane_line_pair_score_width_gap_lambda, 0.3, "gen_lane_line_pair_score_width_gap_lambda");
DEFINE_double(gen_lane_line_pair_score_theta_lambda, 5, "gen_lane_line_pair_score_theta_lambda");
DEFINE_double(gen_lane_line_pair_score_delta_theta_lambda, 5, "gen_lane_line_pair_score_delta_theta_lambda");
DEFINE_double(gen_lane_line_pair_score_threshold, 0.3, "gen_lane_line_pair_score_threshold");
DEFINE_double(gen_lane_line_valid_context_dir_threshold, 10, "gen_lane_line_valid_context_dir_threshold");
DEFINE_double(gen_lane_line_scope, 1, "gen_lane_line_scope");
DEFINE_double(gen_lane_line_search_radius, 5, "gen_lane_line_search_radius");
DEFINE_double(gen_lane_line_theta, 10, "gen_lane_line_theta");
DEFINE_double(gen_lane_line_match_rate_threshold, 0.8, "gen_lane_line_match_rate_threshold");
DEFINE_double(gen_lane_line_iou_thres_1, 0.85, "gen_lane_line_iou_thres_1");
DEFINE_double(gen_lane_line_iou_thres_2, 0.5, "gen_lane_line_iou_thres_2");
DEFINE_double(gen_lane_line_valid_min_length, 20, "gen_lane_line_valid_min_length");
DEFINE_double(gen_lane_line_match_line_long_scope, 20, "gen_lane_line_match_line_long_scope");
DEFINE_double(gen_lane_line_match_line_long_max_dis_threshold, 2, "gen_lane_line_match_line_long_max_dis_threshold");
DEFINE_double(gen_lane_line_match_line_long_max_theta_threshold, 10, "gen_lane_line_match_line_long_max_theta_threshold");
DEFINE_double(gen_lane_line_dft_lc_width, 3, "gen_lane_line_dft_lc_width");
DEFINE_double(gen_lane_line_edge_score_thres, 0.15, "gen_lane_line_edge_score_thres");
DEFINE_double(gen_lane_line_edge_score_scope, 20, "gen_lane_line_edge_score_scope");
DEFINE_double(gen_lane_line_edge_score_min_scope, 10, "gen_lane_line_edge_score_min_scope");
DEFINE_double(gen_lane_line_valid_edge_length, 10, "gen_lane_line_valid_edge_length");
DEFINE_double(gen_lane_line_valid_line_length, 20, "gen_lane_line_valid_line_length");

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcGenLaneLine::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_gen_lane_line_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_gen_lane_line_debug_pos_enable;

    CHECK_FATAL_PROC(make_gen_lane_center_tree(session), "make_gen_lane_center_tree");

    // CHECK_FATAL_PROC(match_context_set(session), "match_context_set");

    // CHECK_FATAL_PROC(make_lane_line_mutil(session), "make_lane_line_mutil");

    // CHECK_FATAL_PROC(make_lane_line_single(session), "make_lane_line_single");


    // CHECK_FATAL_PROC(merge_lane_line(session), "merge_lane_line");

    CHECK_FATAL_PROC(make_lane_line_raw(session), "make_lane_line_raw");

    CHECK_FATAL_PROC(merge_lane_center(session), "merge_lane_center");

    CHECK_FATAL_PROC(filter_lane_center(session), "filter_lane_center");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcGenLaneLine::make_gen_lane_center_tree(RoadModelSessionData* session) {
    session->lane_center_gen_tree.RemoveAll();
    for (auto &lc : session->lane_center_feature_gen) {
        session->lane_center_gen_tree.insert(lc->pos, lc);
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::match_context_set(RoadModelSessionData* session) {
    for (auto &lc : session->lane_center_feature_gen) {
        if (lc->invalid()) {
            continue;
        }
        session->thread_pool->schedule([lc, session, this](utils::ProcessBar *process_bar) {
                // if (!session->debug_pos(lc->pos, lc->left->src->frame_id)) {
                //     // return;
                // }
                // search_same_dir_lane_center_gen(session, lc, lc->next_pair_list);
                search_same_dir_lane_center_gen(session, lc);
                process_bar->num_biz += lc->next_pair_list.size();
                process_bar->num_biz += lc->prev_pair_list.size();
                });
    }

    session->thread_pool->wait(2, "process_gen_lane_line");
    // 补全后面的
    for (auto &lc : session->lane_center_feature_gen) {
        session->debug_pos(lc->pos, lc->left->src->frame_id);
        for (int i = 0; i < lc->next_pair_list.size(); ++i) {
            auto &next_lc = lc->next_pair_list[i];
            // 判断环
            bool is_cycle_lc = false;
            for (int j = 0; j < next_lc->right->next_pair_list.size(); ++j) {
                auto &tmp_pair = next_lc->right->next_pair_list[j];
                if (tmp_pair->get_other(next_lc->right) == lc) {
                    is_cycle_lc = true;
                    // VEC_ERASE(next_lc->right->next_pair_list, j);
                    break;
                }
            }
            if (is_cycle_lc) {
                VEC_ERASE(lc->next_pair_list, i);
                LOG_ERROR("lane center match cycle[{}]", next_lc->local());
                continue;
            }
            // next_lc->right->prev_pair_list.push_back(next_lc);
            // next_lc->right->prev_pair_map[lc] = next_lc;
            lc->next_pair_map[next_lc->right] = next_lc;
        }
    }

    return fsdmap::SUCC;
}

LaneCenterFeature* RoadModelProcGenLaneLine::search_same_dir_lane_center_gen(
        RoadModelSessionData* session,
         LaneCenterFeature* lc) {
    float max_theta = FLAGS_gen_lane_line_search_lc_max_theta;
    float radius = FLAGS_gen_lane_line_search_lc_max_radius;
    float min_radius = FLAGS_gen_lane_line_search_lc_max_min_radius;
    session->debug_pos(lc->pos);
    auto &pos = lc->pos;

    auto &next_list = lc->next_pair_list;
    auto &prev_list = lc->prev_pair_list;

    std::vector<LaneCenterFeature*> search_sec;
    session->lane_center_gen_tree.search(pos, radius, search_sec);
    
    UMAP<std::string, std::shared_ptr<LaneCenterFeatureMatchPair>> next_cache_map;
    UMAP<std::string, std::shared_ptr<LaneCenterFeatureMatchPair>> prev_cache_map;

    int64_t recall_num = 0;
    // std::vector<std::shared_ptr<LaneCenterFeatureMatchPair>> ptrs;
    for (auto &lc_get : search_sec) {
        if (lc_get->invalid()) {
            continue;
        }
        
        if (lc->trail_id != lc_get->trail_id) {
            continue;
        }
        if (!lc->is_same_base(lc_get)) {
            continue;
        }
        double theta = alg::calc_theta(lc->dir, lc_get->dir);
        if (theta > max_theta) {
            continue;
        }
        double dis = alg::calc_dis(lc->pos, lc_get->pos);
        double h_dis = alg::calc_hori_dis(lc_get->pos, lc->pos, lc->dir, true);
        if (lc->left != NULL) {
            if (lc_get->left != lc->left->prev 
                    && lc_get->left != lc->left->next) {
                continue;
            }
        } else {
            if (dis > radius) {
                continue;
            }
            
            if (fabs(h_dis) < min_radius) {
                continue;
            }
            if (h_dis > 0) {
                if (MAP_FIND(next_cache_map, lc_get->line_id)) {
                    if (fabs(next_cache_map[lc_get->line_id]->h_dis) < fabs(h_dis)) {
                        continue;
                    }
                }
            } else {
                if (MAP_FIND(prev_cache_map, lc_get->line_id)) {
                    if (fabs(prev_cache_map[lc_get->line_id]->h_dis) < fabs(h_dis)) {
                        continue;
                    }
                }
            }
        }
        
        auto pair = std::make_shared<LaneCenterFeatureMatchPair>();
        pair->left = lc;
        pair->right = lc_get;
        pair->dis = dis;
        pair->h_dis = h_dis;
        pair->theta = theta;

        Eigen::Vector3d dir_gap = alg::get_dir(lc_get->pos, lc->pos);
        double delta_l_theta = alg::calc_theta_with_dir(lc->dir, dir_gap);
        // if (fabs(delta_l_theta) > max_theta) {
        //     continue;
        // }
        double delta_r_theta = alg::calc_theta_with_dir(lc_get->dir, dir_gap);
        // if (fabs(delta_r_theta) > max_theta) {
        //     continue;
        // }
        pair->dir_gap = dir_gap;
        pair->delta_l_theta = delta_l_theta;
        pair->delta_r_theta = delta_r_theta;
        // }
        // if (!lc->is_same_base(lc_get)) {
        calc_lane_center_line_match_score(session, pair.get());
        // } else {
        //     pair->match = true;
        // }
        recall_num++;
        // if (pair->match) {
            session->add_vec(session->lane_center_line_match_pair_ptr, pair, true);
            if (h_dis > 0) {
                next_list.push_back(pair.get());
                next_cache_map[lc_get->line_id] = pair;
            } else {
                prev_list.push_back(pair.get());
                prev_cache_map[lc_get->line_id] = pair;
            }
        // }
    }
    SORT(next_list,
            [](const LaneCenterFeatureMatchPair *l, const LaneCenterFeatureMatchPair *r)->bool {
            return l->score > r->score;
            });
    SORT(prev_list,
            [](const LaneCenterFeatureMatchPair *l, const LaneCenterFeatureMatchPair *r)->bool {
            return l->score > r->score;
            });


    LOG_DEBUG("lane center match line pair[size={}, next={}, prev={}]", 
            recall_num, next_list.size(), prev_list.size());
    // if (secs.size() > 0) {
    //     return secs[0]->get_other(lc);
    // }
    return NULL;
}

bool RoadModelProcGenLaneLine::calc_lane_center_line_match_score(
        RoadModelSessionData* session, LaneCenterFeatureMatchPair *pair) {
    double edge_lambda = FLAGS_gen_lane_line_pair_score_dis_lambda;
    double width_gap_lambda = FLAGS_gen_lane_line_pair_score_width_gap_lambda;
    double theta_lambda = FLAGS_gen_lane_line_pair_score_theta_lambda;
    double delta_theta_lambda = FLAGS_gen_lane_line_pair_score_delta_theta_lambda;
    double score_threshold = FLAGS_gen_lane_line_pair_score_threshold;
    // 计算距离系数
    //debug_pos(left->ls->center.pos), true);
    double dis = pair->dis;
    double delta_l_theta = pair->delta_l_theta;
    double delta_r_theta = pair->delta_r_theta;
    double theta = pair->theta;

    double dis_factor = edge_lambda / (edge_lambda + dis);
    double avg_theta = fabs(delta_l_theta + delta_r_theta);
    double delta_theta_factor = delta_theta_lambda / (delta_theta_lambda + avg_theta);
    double width_dis = pair->left->width - pair->right->width;
    double width_gap = fabs(width_dis);
    double width_gap_factor = width_gap_lambda / (width_gap_lambda + width_gap);
    
    
    // 提取的其他feature
    // pair->left_center_theta = alg::calc_theta_with_dir(left->ls->left->dir, left->ls->right->dir, true, true);
    // pair->right_center_theta = alg::calc_theta_with_dir(right->ls->left->dir, right->ls->right->dir, true, true);
    // pair->left_theta = alg::calc_theta_with_dir(left->ls->left->dir, right->ls->left->dir);
    // pair->right_theta = alg::calc_theta_with_dir(left->ls->right->dir, right->ls->right->dir);
    // alg::calc_dir(left->ls->left->pos, right->ls->left->pos, dir);
    // pair->l_delta_theta_l = alg::calc_theta_with_dir(left->ls->left->dir, dir, true);
    // pair->l_delta_theta_r = alg::calc_theta_with_dir(right->ls->left->dir, dir, true);
    // alg::calc_dir(left->ls->right->pos, right->ls->right->pos, dir);
    // pair->r_delta_theta_l = alg::calc_theta_with_dir(left->ls->right->dir, dir, true);
    // pair->r_delta_theta_r = alg::calc_theta_with_dir(right->ls->right->dir, dir, true);

    // 计算group之间的夹角
    // 如果同向，则夹角和avg_theta应该相等，顾不再修正距离
    // if (pair->theta + 1 >= avg_theta) {
    //     delta_theta_factor = 1;
    // }
    double theta_factor = theta_lambda / (theta_lambda + theta);

    pair->score = sqrt(theta_factor * dis_factor * delta_theta_factor * width_gap_factor);
    pair->match = false;
    if (pair->score >= score_threshold) {
        pair->match = true;
    } else if (pair->left->left->is_same_line(pair->right->left) 
            && pair->left->right->is_same_line(pair->right->right)) {
        pair->match = true;
    }
    DLOG_POINT2(pair->left->pos, pair->right->pos,
            "lane center line match[match={}, score={}"
            ", theta_factor={}, dis_factor={}, delta_theta_factor={}, width_gap_factor={}"
            ", theta={}, avg_theta={}, dis={}, width_gap={}]",
            pair->match, pair->score,
            theta_factor, dis_factor, delta_theta_factor, width_gap_factor,
            theta, avg_theta, dis, width_gap);
    return pair->match;
}

// 将merge_lane_center_list中的车道中心线点，按左右1.5m偏移作为左右侧车道线点，根据左右侧车道线点取平均值作为车道中心点的位置，其实构建了新的虚拟车道线点
int RoadModelProcGenLaneLine::make_lane_line_raw(RoadModelSessionData* session) {
    double dft_width = FLAGS_gen_lane_line_dft_lc_width / 2;  // 1.5
    // for (auto &lc : session->lane_center_feature_merge_raw) {
    //     if (lc->invalid()) {
    //         continue;
    //     }
    //     session->lane_center_gen_tree.insert(lc->pos, lc);
    // }
    UMAP<LaneCenterFeature*, int> used_map;
    // LOG_INFO("merge_lane_center_list:{}", session->merge_lane_center_list.size());
    for (auto &line : session->merge_lane_center_list) {
        for (auto &lc : line->list) {
            session->debug_pos(lc->pos);
            if (lc->invalid()) {
                continue;
            }
            if (MAP_FIND(used_map, lc.get())) {
                continue;
            }
            used_map[lc.get()] = 1;
            Eigen::Vector3d left_pos = alg::get_vertical_pos(lc->pos, lc->dir, -dft_width);  //车道中心点左侧1.5m的点
            Eigen::Vector3d right_pos = alg::get_vertical_pos(lc->pos, lc->dir, dft_width);  //车道中心点右侧1.5m的点
            lc->init(left_pos, lc->dir, right_pos, lc->dir);
            // if (lc->prev != NULL && !lc->prev->invalid()) {
            //     if (lc->context.set_prev(lc->prev)) {
            //         lc->context.score_prev.push_back(1);
            //     }
            // }
            if (lc->next != NULL && !lc->next->invalid()) {
                double theta = alg::calc_theta(lc->dir, lc->next->dir);
                if (theta > 90) {
                    LOG_WARN("lc dir error, earse[dir1={},{}, dir2={},{}]", 
                            lc->dir.x(), lc->dir.y(), lc->next->dir.x(), lc->next->dir.y());
                    lc->next->prev = NULL;
                    lc->next = NULL;
                    continue;
                }
                double score = (lc->next->score + lc->score) / 2;
                lc->context.set_next(lc->next, score);
                lc->next->context.set_prev(lc.get(), score);
            }
            
            session->lane_center_feature_raw.push_back(lc.get());
            // session->lane_center_gen_tree.insert(lc->pos, lc.get());
        }
    }
    return fsdmap::SUCC;
}


int64_t RoadModelProcGenLaneLine::get_lc_max_length(RoadModelSessionData* session, 
        std::vector<LaneCenterFeature*> &lc_list, bool next) {
    int64_t lc_num = 0;
    // std::vector<LaneCenterFeature*> new_lc_list;
    for (auto &lc : lc_list) {
        if (lc->invalid()) {
            continue;
        }
        lc->length_status = 0;
        ++lc_num;
        // new_lc_list.push_back(lc);
    }
    int64_t valid_num = lc_num;
    int64_t prev_lc_num = lc_num;
    bool skip_calc = false;
    int64_t times = 0;
    while (lc_num > 0) {
        LOG_INFO("get_max_length[next={}, times={}, lc_num={}", next, times++, lc_num);
        for (auto &lc : lc_list) {
            auto next_lc = lc;
            while (next_lc != NULL) {
                session->debug_pos(next_lc->pos);
                if (next_lc->invalid()) {
                    break;
                }
                if (next_lc->length_status == 1) {
                    break;
                }
                double max_length = 0;
                bool has_valid_prev = false;
                auto &all_vec = next ? next_lc->context.all_prev : next_lc->context.all_next;
                for (auto &prev_lc_ptr : all_vec) {
                    auto &prev_lc = prev_lc_ptr.src;

                    if (prev_lc_ptr.invalid()) {
                        continue;
                    }
                    if (!prev_lc->invalid() && prev_lc->length_status == 0) {
                        has_valid_prev = true;
                        break;
                    }

                    double prev_max = next ? 
                        prev_lc->context.prev_max_length : prev_lc->context.next_max_length;
                    double dis = alg::calc_dis(next_lc->pos, prev_lc->pos);
                    prev_max += dis;
                    max_length = std::max(max_length, prev_max);
                }
                if (has_valid_prev && !skip_calc) {
                    break;
                }
                if (next) {
                    next_lc->context.prev_max_length = max_length;
                } else {
                    next_lc->context.next_max_length = max_length;
                }
                next_lc->length_status = 1;
                --lc_num;
                int64_t next_num = next ? 
                    next_lc->context.valid_next_size() : next_lc->context.valid_prev_size();
                if (next_num != 1) {
                    break;
                }
                next_lc = next ? next_lc->context.next() : next_lc->context.prev();
            }
        }
        skip_calc = prev_lc_num == lc_num;
        prev_lc_num = lc_num;
    }
    return valid_num;
}

int RoadModelProcGenLaneLine::merge_lane_line(RoadModelSessionData* session) {
    // int min_size = FLAGS_gen_lane_line_proc_min_size;
    // int min_match_size = FLAGS_gen_lane_line_proc_min_match_size;
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_list;

    int64_t raw_line_size = 0;
    int64_t merge_line_size = 0;
    int64_t raw_rela_size = 0;
    int64_t merge_rela_size = 0;
    LaneCenterFeatureTree trail_tree;
    // get_lc_max_length(session, true);
    int64_t lc_num = get_lc_max_length(session, session->lane_center_feature_gen, false);
    SORT(session->lane_center_feature_gen,
            [](const LaneCenterFeature* l, const LaneCenterFeature* r) {
                return l->context.next_max_length > r->context.next_max_length;
            });
    int64_t times = 0;
    while (lc_num > 0) {
        LOG_INFO("merge_lane_line[times={}, lc_num={}", times++, lc_num);
        for (auto &lc : session->lane_center_feature_gen) {
            if (lc->invalid()) {
                continue;
            }
            if (lc->merge_status == 1) {
                continue;
            }
            bool has_valid_prev = false;
            for (auto &prev_lc_ptr : lc->context.all_prev) {
                auto &prev_lc = prev_lc_ptr.src;
                if (!prev_lc->invalid() && prev_lc->merge_status == 0) {
                    has_valid_prev = true;
                    break;
                }
            }
            if (has_valid_prev) {
                continue;
            }
            auto tmp_line = session->add_ptr(lane_center_line_list);
            extract_lane_line(session, lc, tmp_line.get());
            // if (tmp_line->list.size() <= 1) {
            //     continue;
            // }
            for (auto &pt : tmp_line->list) {
                raw_rela_size += pt->context.all_next.size();
            }
            raw_line_size += tmp_line->list.size();
            match_lane_center_line(session, session->lane_center_line_group_list, 
                    trail_tree, tmp_line.get());
            lc_num -= tmp_line->list.size();
        }
    }

    session->lane_center_gen_tree.RemoveAll();
    for (auto &line : session->lane_center_line_group_list) {
        for (auto &lc : line->list) {
            session->lane_center_gen_tree.insert(lc->pos, lc.get());
            merge_rela_size += lc->context.all_next.size();
            session->lane_center_feature_merge.push_back(lc.get());
        }
        merge_line_size += line->list.size();
    }

    get_lc_max_length(session, session->lane_center_feature_merge, true);
    get_lc_max_length(session, session->lane_center_feature_merge, false);
    LOG_DEBUG("merge_lane_line[lsg={}, raw_lsg={}, rela={}, raw_rela={}]", 
            merge_line_size, raw_line_size, merge_rela_size, raw_rela_size);
    // 过滤
    for (int64_t i = 0; i < session->lane_center_line_group_list.size(); ++i) {
        auto &group_line = session->lane_center_line_group_list[i];
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            int valid_index = j;
            if (j > 0 && j == group_line->list.size() - 1) {
                valid_index = j - 1;
            }
            auto &pt = group_line->list[j];
            session->debug_pos(pt->pos);
            UMAP<std::string, double> trail_map;
            int total_num = 1;
            for (auto &raw_pt : group_line->match_list[valid_index]) {
                auto trail_id = raw_pt->trail_id;
                if (raw_pt->src_status == 2) {
                    trail_id = "sem";
                }
                total_num = std::max(raw_pt->left->match_trail_num, total_num);
                if (MAP_NOT_FIND(trail_map, trail_id)) {
                    trail_map[trail_id] = raw_pt->score;
                    continue;
                }
                if (trail_map[trail_id] < raw_pt->score) {
                    trail_map[trail_id] = raw_pt->score;
                }
            }
            double match_num = trail_map.size();
            if (MAP_FIND(trail_map, "sem")) {
                pt->score = 1;
            } else {
                pt->score = match_num / total_num;
            }

            DLOG_POINT(pt->pos, "merge_lane_line_d[src={}, s={:.2f}, m1={}, m2={}, "
                    "l1={:.2f}, l2={:.2f}]",
                    pt->src_status, pt->score, match_num, total_num, 
                    pt->context.prev_max_length, pt->context.next_max_length);
        }
    }
    for (auto &lc : session->lane_center_feature_merge) {
        if (lc->context.all_next.size() == 0 && lc->context.all_prev.size() == 0) {
            lc->filter_status = 17;
            continue;
        }
        lc->context.max_length = lc->context.prev_max_length + lc->context.next_max_length;
        if (lc->match_level.lane_num == 1 
                && lc->context.max_length < FLAGS_gen_lane_line_valid_min_length) {
            lc->filter_status = 18;
            continue;
        }
    }
    
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::extract_lane_line(RoadModelSessionData* session,
        LaneCenterFeature* lc, LaneCenterLine* tar_line) {
    auto next_lc = lc;
    while(next_lc != NULL) {
        if (next_lc->invalid()) {
            break;
        }
        if (next_lc->merge_status == 1) {
            break;
        }
        tar_line->list.push_back(next_lc);
        next_lc->merge_status = 1;
        if (next_lc->context.all_next.size() == 0) {
            break;
        }
        if (next_lc->context.all_next.size() > 1) {
            SORT(next_lc->context.all_next,
                    [](const LCP &l, const LCP &r) {
                    return l.src->context.next_max_length > r.src->context.next_max_length;
                    });
        }
        next_lc = next_lc->context.all_next.front().src;
    }
    return fsdmap::SUCC;
}


int RoadModelProcGenLaneLine::match_lane_center_line(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    double radius = FLAGS_gen_lane_line_search_radius;
    double scope = FLAGS_gen_lane_line_scope;
    double theta_thres = FLAGS_gen_lane_line_theta;
    double iou_thres_1 = FLAGS_gen_lane_line_iou_thres_1;
    double iou_thres_2 = FLAGS_gen_lane_line_iou_thres_2;

    std::vector<LaneCenterFeature*> secs;
    UMAP<LaneCenterGroupLine*, int> cache_map;
    UMAP<LaneCenterFeature*, int> used_map;
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        LaneCenterFeature* min_lc = NULL;
        double min_dis = DBL_MAX;
        trail_tree.search(pt->pos, radius, secs);
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
                if (pt->next != NULL) {
                    continue;

                }
                double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir, true);
                if (h_dis > radius / 2) {
                    continue;
                } 
                dis = alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir);
            } else {
                if (!alg::get_cross_point_for_segment(src_pt->pos, src_pt->next->pos,
                            pt->pos, v_pt, cross_point, 1)) {
                    continue;
                }
                dis = alg::calc_dis(pt->pos, cross_point);
            }
            if (dis > scope) {
                // match_map[src_pt->group_line] = NULL;
                continue;
            }
            double theta = alg::calc_theta(src_pt->dir, pt->dir);
            if (theta > theta_thres) {
                continue;
            }
            double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir);
            if (s_dis > scope) {
                continue;
            }
            double iou_thres = iou_thres_1;
            if (pt->match_level.lane_type == 2 && src_pt->match_level.lane_type == 2) {
                if (pt->match_level.join_out != src_pt->match_level.join_out) {
                    continue;
                }
                iou_thres = iou_thres_2;
            }
            double iou = alg::segment_iou(pt->get_left(), pt->get_right(),
                    src_pt->get_left(), src_pt->get_right());
            if (iou < iou_thres) {
                continue;
            }
            if (dis < min_dis) {
                min_dis = dis;
                min_lc = src_pt;
            }
            src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
            if (MAP_FIND(cache_map, src_pt->group_line)) {
                continue;
            }
            cache_map[src_pt->group_line] = 1;
        }
        pt->group_pt = min_lc;
    }
    using line_score = std::pair<LaneCenterGroupLine*, double>;
    std::vector<std::tuple<int64_t, int64_t, int>> sub_line_list;
    int status = 0;
    int64_t start_index = 0;
    int64_t end_index = 0;
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        if (pt->group_pt == NULL) {
            if (status == 1) {
                end_index = j - 1;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 1));
                start_index = j;
            } 
            status = 0;
            if (j == tar_line->list.size() - 1) {
                end_index = j;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
            }
            continue;
        } else {
            if (status == 0) {
                end_index = j - 1;
                if (end_index >= start_index) {
                    sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
                }
                start_index = j;
            } 
            status = 1;
            if (j == tar_line->list.size() - 1) {
                end_index = j;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 1));
            }
        }
    }
    for (auto &tuple : sub_line_list) {
        int64_t start_index = std::get<0>(tuple);
        int64_t end_index = std::get<1>(tuple);
        int type = std::get<2>(tuple);
        if (type == 0) {
            gen_new_lane_center_group(session, trail_group, 
                    tar_line, start_index, end_index, trail_tree);
            
        } else if (type == 1) {
            for (int64_t i = start_index; i <= end_index; ++i) {
                auto &tar_lc = tar_line->list[i];
                update_lane_center_group_line(session, tar_lc, trail_tree, 1);
            }
        }
    }
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &tar_lc = tar_line->list[i];
        for (int j = 0; j < tar_lc->context.all_prev.size(); ++j) {
            auto &prev_lc_ptr = tar_lc->context.all_prev[j];
            auto &prev_lc = prev_lc_ptr.src;
            if (prev_lc->group_pt == NULL) {
                continue;
            }
            if (prev_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
                continue;
            }
            if (!alg::judge_front(tar_lc->pos, prev_lc->pos, prev_lc->dir)) {
                continue;
            }
            double score = prev_lc_ptr.score;
            prev_lc->group_pt->context.set_next(tar_lc->group_pt, score);
            tar_lc->group_pt->context.set_prev(prev_lc->group_pt, score);
        }
        for (int j = 0; j < tar_lc->context.all_next.size(); ++j) {
            auto &next_lc_ptr = tar_lc->context.all_next[j];
            auto &next_lc = next_lc_ptr.src;;
            if (next_lc->group_pt == NULL) {
                continue;
            }
            if (next_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
                continue;
            }
            if (!alg::judge_front(next_lc->pos, tar_lc->pos, tar_lc->dir)) {
                continue;
            }
            double score = next_lc_ptr.score;
            next_lc->group_pt->context.set_prev(tar_lc->group_pt, score);
            tar_lc->group_pt->context.set_next(next_lc->group_pt, score);
        }
    }
    return fsdmap::SUCC;
}

// 寻找src_pt后继20m范围内的点，找出与tar_pt偏差最小的点
LineMatchParam<LaneCenterFeature> RoadModelProcGenLaneLine::match_line_long_distance(
        RoadModelSessionData* session,
        LaneCenterFeature* src_pt, LaneCenterLine* tar_line, int64_t index, bool next) {
    double scope = FLAGS_gen_lane_line_match_line_long_scope;  // 20
    double dis_thres = FLAGS_gen_lane_line_match_line_long_max_dis_threshold;  // 2
    auto &tar_pt = tar_line->list[index];

    double total_dis = 0;
    auto prev_lc = tar_pt;
    UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> line_vec;
    int start_index = index;
    int end_index = next ? tar_line->list.size() - 1 : 0;
    int lc_num = fabs(end_index - start_index);
    Eigen::Vector3d v_pt = alg::get_vertical_pos(tar_pt->pos, tar_pt->dir, 50, true);

    LineMatchParam<LaneCenterFeature> min_ret = {NULL, NULL, DBL_MAX};
    for (int i = 0; i < lc_num; ++i) {
        int real_index = next ? start_index + i : start_index - i;
        auto &next_lc = tar_line->list[real_index];
        total_dis += alg::calc_dis(prev_lc->pos, next_lc->pos);
        if (total_dis > scope) {
            break;
        }
        if (next_lc->group_pt == NULL) {
            continue;
        }
        line_vec.clear();
        if (!src_pt->context.on_same_line(scope, src_pt, next_lc->group_pt, next, &line_vec)) {
            continue;
        }
        Eigen::Vector3d cross_point;
        for (auto &tmp_lc_it : line_vec) {
            auto &tmp_lc = tmp_lc_it.first;
            for (auto &tmp_next : tmp_lc_it.second) {
                if (!alg::get_cross_point_for_segment(tmp_lc->pos, tmp_next->pos,
                            tar_pt->pos, v_pt, cross_point, 1)) {
                    continue;
                }
                double dis = alg::calc_dis(tar_pt->pos, cross_point);
                if (dis > min_ret.dis) {    //  小于号吧？
                    min_ret.dis = dis;
                    min_ret.from = tmp_lc;
                    min_ret.to = tmp_next;
                }
            }
        }
        if (min_ret.from != NULL && min_ret.dis > dis_thres) {
            break;
        }
    }
    
    return min_ret;
}

 // tar_line中 取[tar_line, start_index]的点构建新的车道中心线放入trail_group
int RoadModelProcGenLaneLine::gen_new_lane_center_group(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, LaneCenterLine* tar_line,
        int64_t start_index, int64_t end_index, LaneCenterFeatureTree &trail_tree) {
    auto new_line = std::make_shared<LaneCenterGroupLine>();
    new_line->id = tar_line->id;
    LaneCenterFeature* prev_pt = NULL;
    for (int64_t i = start_index; i <= end_index; ++i) {
        auto &pt = tar_line->list[i];
        auto new_pt = std::make_shared<LaneCenterFeature>();
        new_pt->init(pt);
        new_pt->group_line = new_line.get();
        new_pt->line_index = new_line->list.size();
        new_pt->line_id = new_line->id;
        new_line->list.push_back(new_pt);
        new_line->match_list.push_back({pt});
        new_line->match_score.push_back(pt->score);
        new_line->match_list_point.resize(new_line->match_list.size());
        pt->group_pt = new_pt.get();
        if (prev_pt != NULL) {
            prev_pt->next = new_pt.get();
            new_pt->prev = prev_pt;
            auto &raw_prev_pt = tar_line->list[i - 1];
            double score = raw_prev_pt->context.all_next[0].score;
            prev_pt->context.set_next(new_pt.get(), score);
            new_pt->context.set_prev(prev_pt, score);
        }
        trail_tree.insert(new_pt->pos, new_pt.get());
        prev_pt = new_pt.get();
    }
    new_line->merge_lines.push_back(tar_line->src);
    trail_group.push_back(new_line);
    return fsdmap::SUCC;
}

// 更新tar_pt->group_pt的位置
int RoadModelProcGenLaneLine::update_lane_center_group_line(RoadModelSessionData* session,
        LaneCenterFeature* tar_pt, LaneCenterFeatureTree &trail_tree, int mode) {
    double scope = FLAGS_gen_lane_line_scope;  // 1
    int64_t max_tar_valid = 0;
    int64_t max_src_valid = 0;
    int64_t min_tar_valid = INT_MAX;
    int64_t min_src_valid = INT_MAX;

    auto src_pt = tar_pt->group_pt;
    auto src_line = src_pt->group_line;
    int64_t src_index = src_pt->line_index;
    if (mode == 1) {
        update_lane_side(session, src_line, src_index, tar_pt, true);
        update_lane_side(session, src_line, src_index, tar_pt, false);
        Eigen::Vector3d l_pos = src_pt->left_pos;
        Eigen::Vector3d r_pos = src_pt->right_pos;
        Eigen::Vector3d l_dir = src_pt->left_dir;
        Eigen::Vector3d r_dir = src_pt->right_dir;
        src_pt->init(l_pos, l_dir, r_pos, r_dir);
    } else if (mode == 2) {
        if (tar_pt->raw_to_lc != NULL) {
            auto to_lc = tar_pt->raw_to_lc;
            double f_dis = alg::calc_hori_dis(tar_pt->pos, src_pt->pos, src_pt->dir);
            double t_dis = alg::calc_hori_dis(tar_pt->pos, to_lc->pos, to_lc->dir);
            if (t_dis < f_dis) {
                tar_pt->group_pt = to_lc;
                src_pt = to_lc;
                src_line = src_pt->group_line;
                src_index = src_pt->line_index;
            }
        }
        update_lane_center(session, src_line, src_index, tar_pt);
    }

    session->debug_pos(src_pt->pos);
    trail_tree.insert(src_pt->pos, src_pt);
    src_line->match_list[src_index].push_back(tar_pt);
    src_line->match_score[src_index] += tar_pt->score;
    return fsdmap::SUCC;
}

void RoadModelProcGenLaneLine::update_lane_side(RoadModelSessionData* session,
        LaneCenterGroupLine* src_line, int64_t src_index, LaneCenterFeature* tar_lc, bool left) {
    double scope = FLAGS_gen_lane_line_scope;
    auto &src_lc = src_line->list[src_index];
    double total_score = src_line->match_score[src_index];
    auto &tar_pt = left ? tar_lc->get_left() : tar_lc->get_right();
    auto &src_pt = left ? src_lc->get_left() : src_lc->get_right();
    double v_dis = alg::calc_vertical_dis(tar_pt, src_pt, src_lc->dir, true);
    if (fabs(v_dis) > scope) {
        return;
    }
    double update_rate = tar_lc->score / (total_score + tar_lc->score);
    double opt_dis = update_rate * v_dis;
    Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt, src_lc->dir, opt_dis);
    if (isnan(opt_pos.x()) || isnan(opt_pos.y()) 
            || isnan(src_lc->dir.x()) || isnan(src_lc->dir.y())) {
        int a = 1;
    }
    // session->debug_pos(opt_pos);
    if (left) {
        src_lc->left_pos = opt_pos;
        src_lc->left_dir += update_rate * tar_lc->left_dir;
        src_lc->left_dir.normalize();
    } else {
        src_lc->right_pos = opt_pos;
        src_lc->right_dir += update_rate * tar_lc->right_dir;
        src_lc->right_dir.normalize();
    }
    
}

// 根据src_line->list[src_index]的score以及tar_lc的score（离自车越近，分数越高）计算比例，
// 更新src_line->list[src_index]的位置以及方向向量
void RoadModelProcGenLaneLine::update_lane_center(RoadModelSessionData* session,
        LaneCenterGroupLine* src_line, int64_t src_index, LaneCenterFeature* tar_lc) {
    double scope = FLAGS_gen_lane_line_scope;  // 1
    auto &src_lc = src_line->list[src_index];
    double total_score = src_line->match_score[src_index];
    auto &tar_pt = tar_lc->pos;
    auto &src_pt = src_lc->pos;
    double v_dis = alg::calc_vertical_dis(tar_pt, src_pt, src_lc->dir, true);
    // if (fabs(v_dis) > scope) {
    //     return;
    // }
    double update_rate = tar_lc->score / (total_score + tar_lc->score);
    double opt_dis = update_rate * v_dis;
    Eigen::Vector3d opt_pos = alg::get_vertical_pos(src_pt, src_lc->dir, opt_dis);
    if (isnan(opt_pos.x()) || isnan(opt_pos.y()) 
            || isnan(src_lc->dir.x()) || isnan(src_lc->dir.y())) {
        int a = 1;
    }
    session->debug_pos(opt_pos);
    src_lc->pos = opt_pos;
    src_lc->dir += update_rate * tar_lc->dir;
    src_lc->dir.normalize();
}


int RoadModelProcGenLaneLine::merge_lane_center(RoadModelSessionData* session) {
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_list;

    int64_t raw_line_size = 0;
    int64_t merge_line_size = 0;
    int64_t raw_rela_size = 0;
    int64_t merge_rela_size = 0;
    LaneCenterFeatureTree trail_tree;
    // get_lc_max_length(session, true);
    int64_t lc_num = get_lc_max_length(session, session->lane_center_feature_raw, false);
    SORT(session->lane_center_feature_raw,
            [](const LaneCenterFeature* l, const LaneCenterFeature* r) {
                return l->context.next_max_length > r->context.next_max_length;
            });
    int64_t times = 0;
    int64_t patch = 0;
    while (lc_num > 0) {
        LOG_INFO("merge_lane_center[times={}, lc_num={}", times++, lc_num);
        for (auto &lc : session->lane_center_feature_raw) {
            if (lc->invalid()) {
                continue;
            }
            if (lc->merge_status == 1) {  // lc已经遍历过了，在extract_lane_line里会置为1
                continue;
            }
            bool has_valid_prev = false;
            for (auto &prev_lc_ptr : lc->context.all_prev) {
                auto &prev_lc = prev_lc_ptr.src;
                if (!prev_lc->invalid() && prev_lc->merge_status == 0) {
                    has_valid_prev = true;
                    break;
                }
            }
            if (has_valid_prev) {
                continue;
            }
            auto tmp_line = session->add_ptr(lane_center_line_list);
            extract_lane_line(session, lc, tmp_line.get());  //提取出从lc开始到第一个无效点或者已经merge过的后继的节点这段内的所有节点， 放入tmp_line
            // if (tmp_line->list.size() <= 1) {
            //     continue;
            // }
            for (auto &pt : tmp_line->list) {
                raw_rela_size += pt->context.all_next.size();
            }
            raw_line_size += tmp_line->list.size();
            match_lane_center_line_raw(session, session->lane_center_line_group_list_raw, 
                    trail_tree, tmp_line.get());

            lc_num -= tmp_line->list.size();
            save_frame_log_lane_center(session, session->lane_center_line_group_list_raw,
                    tmp_line.get(), patch++);
        }
    }

    for (auto &line : session->lane_center_line_group_list_raw) {
        for (auto &lc : line->list) {
            for (auto &ptr : lc->context.all_next) {
                ptr.score = 0;
            }
            for (auto &ptr : lc->context.all_prev) {
                ptr.score = 0;
            }
            session->lane_center_group_list_raw.push_back(lc.get());
        }
    }

    merge_lane_center_edge(session, session->lane_center_group_list_raw, true);
    merge_lane_center_edge(session, session->lane_center_group_list_raw, false);

    LOG_DEBUG("merge_lane_line[lsg={}, raw_lsg={}, rela={}, raw_rela={}]", 
            merge_line_size, raw_line_size, merge_rela_size, raw_rela_size);

    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::merge_lane_center_edge(RoadModelSessionData* session,
        std::vector<LaneCenterFeature*> &lane_center_list, bool next) {
    // std::vector<LaneCenterFeature*> context_list;
    // int mode = next ? 1 : 2;
    // for (int i = 0; i < lane_center_list.size(); ++i) {
    //     auto &lc = lane_center_list[i];
    //     double length = lc->context.get_context_list(scope, lc, context_list,
    //             [](LaneCenterFeature* curr, ParamPair<LaneCenterFeature> &pair){
    //                 return true;
    //             },
    //             mode);

    // }
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::gen_lane_center_score(RoadModelSessionData* session) {
    // 过滤
    for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
        auto &group_line = session->lane_center_line_group_list_raw[i];
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            int valid_index = j;
            if (j > 0 && j == group_line->list.size() - 1) {
                valid_index = j - 1;
            }
            auto &pt = group_line->list[j];
            session->debug_pos(pt->pos);
            // <轨迹id，该轨迹上属于group_line->match_list点的最大score>
            UMAP<std::string, double> trail_map;
            int total_num = 1;
            for (auto &raw_pt : group_line->match_list[valid_index]) {
                auto trail_id = raw_pt->trail_id;
                if (raw_pt->src_status == 2) {
                    trail_id = "sem";
                }
                total_num = std::max(raw_pt->match_trail_num, total_num);
                if (MAP_NOT_FIND(trail_map, trail_id)) {
                    trail_map[trail_id] = raw_pt->score;
                    continue;
                }
                if (trail_map[trail_id] < raw_pt->score) {
                    trail_map[trail_id] = raw_pt->score;
                }
            }
            double match_num = trail_map.size(); // 轨迹数量
            if (MAP_FIND(trail_map, "sem")) {
                pt->score = 1;
            } else {
                pt->score = match_num / total_num;
            }
            pt->match_trail_num = total_num;
            // DLOG_POINT(pt->pos, "merge_lane_line_d[src={}, s={:.2f}, m1={}, m2={}, "
            //         "l1={:.2f}, l2={:.2f}]",
            //         pt->src_status, pt->score, match_num, total_num, 
            //         pt->context.prev_max_length, pt->context.next_max_length);
        }
    }
    return fsdmap::SUCC;
}

// 更新score
int RoadModelProcGenLaneLine::vote_edge(RoadModelSessionData* session) {
    // 边投票
    double scope = 10;
    UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> line_map;
    for (auto &lc : session->lane_center_feature_raw) {
        session->debug_pos(lc->pos);
        if (lc->invalid()) {
            continue;
        }
        if (lc->group_pt == NULL) {
            continue;
        }
        session->debug_pos(lc->group_pt->pos);
        for (auto &next_ptr : lc->context.all_next) {
            auto &next = next_ptr.src;
            session->debug_pos(lc->pos);
            if (next->invalid()) {
                continue;
            }
            auto &next_group = next->group_pt;
            if (next_group == NULL || next_group->invalid()) {
                continue;
            }

            auto raw_dir = alg::get_dir(next->pos, lc->pos);

            line_map.clear();
            if (!lc->group_pt->context.on_same_line(
                        scope, lc->group_pt, next_group, true, &line_map)) {
                continue;
            }

            double score = (next->score + 0.1) / next->match_trail_num;
            // double score = (next->score) / next->match_trail_num;
            if (FLAGS_gen_lane_line_vote_edge_all_enable) {
                double valid_dis = FLAGS_gen_lane_line_scope;
                for (auto &it : line_map) {
                    auto &from_lc = it.first;
                    auto &line_vec = it.second;
                    for (auto &to_lc : line_vec) {
                        double f_dis = alg::calc_vertical_dis(from_lc->pos, lc->pos, raw_dir);
                        double t_dis = alg::calc_vertical_dis(to_lc->pos, lc->pos, raw_dir);
                        if (f_dis > valid_dis || t_dis > valid_dis) {
                            continue;
                        }
                        for (auto &tmp_lc_ptr : from_lc->context.all_next) {
                            auto &tmp_lc = tmp_lc_ptr.src;
                            if (tmp_lc == to_lc) {
                                tmp_lc_ptr.score += score;
                                break;
                            }
                        }
                        for (auto &tmp_lc_ptr : to_lc->context.all_prev) {
                            auto &tmp_lc = tmp_lc_ptr.src;
                            if (tmp_lc == from_lc) {
                                tmp_lc_ptr.score += score;
                                break;
                            }
                        }
                    }
                } 
            } else {
                auto proc_next = lc->group_pt;
                while (proc_next != NULL) {
                    if (MAP_NOT_FIND(line_map, proc_next)) {
                        break;
                    }
                    auto &line_vec = line_map[proc_next];
                    double min_dis = DBL_MAX;
                    LaneCenterFeature* valid_lc = NULL;
                    for (auto &tmp_lc : line_vec) {
                        if (tmp_lc->invalid()) {
                            continue;
                        }
                        // auto group_dir = alg::get_dir(tmp_lc->pos, it.first->pos);
                        // double theta = alg::calc_theta(raw_dir, group_dir);
                        double dis = alg::calc_vertical_dis(tmp_lc->pos, lc->pos, raw_dir);
                        if (dis < min_dis) {
                            min_dis = dis;
                            valid_lc = tmp_lc;
                        }
                    }

                    if (valid_lc == NULL) {
                        break;
                    }

                    bool has_valid = false;
                    for (auto &tmp_lc_ptr : proc_next->context.all_next) {
                        auto &tmp_lc = tmp_lc_ptr.src;
                        if (tmp_lc == valid_lc) {
                            tmp_lc_ptr.score += score;
                            has_valid = true;
                            break;
                        }
                    }
                    for (auto &tmp_lc_ptr : valid_lc->context.all_prev) {
                        auto &tmp_lc = tmp_lc_ptr.src;
                        if (tmp_lc == proc_next) {
                            tmp_lc_ptr.score += score;
                            has_valid = true;
                            break;
                        }
                    }
                    if (!has_valid) {
                        int a = 1;
                    }
                    if (proc_next == valid_lc) {
                        FLOG_POINT(valid_lc->pos, "dead_cycle");
                        break;
                    }
                    proc_next = valid_lc;
                }
            }
        }
    }
    return fsdmap::SUCC;
}

    
int RoadModelProcGenLaneLine::filter_lane_center(RoadModelSessionData* session) {
    gen_lane_center_score(session);  
    vote_edge(session);
    smooth_group_lane_center(session);
    int times = 2;
    while (times-- >= 1) {
        get_lc_max_length(session, session->lane_center_group_list_raw, true);
        get_lc_max_length(session, session->lane_center_group_list_raw, false);
        filter_lane_center_by_length(session, true);
        filter_lane_center_by_length(session, false);
    }

    for (auto &lc : session->lane_center_feature_merge_raw) {
        if (lc->context.all_next.size() == 0 && lc->context.all_prev.size() == 0) {
            lc->filter_status = 17;
            continue;
        }
        lc->context.max_length = lc->context.prev_max_length + lc->context.next_max_length;
        if (lc->match_level.lane_num == 1 
                && lc->context.max_length < FLAGS_gen_lane_line_valid_min_length) {
            lc->filter_status = 18;
            continue;
        }
    }

    session->lane_center_gen_tree.RemoveAll();
    for (auto &line : session->lane_center_line_group_list_raw) {
        for (auto &lc : line->list) {
            session->lane_center_feature_merge_raw.push_back(lc.get());
            session->lane_center_gen_tree.insert(lc->pos, lc.get());
        }
    }
    
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::filter_lane_center_by_length(RoadModelSessionData* session, bool next) {
    double scope = FLAGS_gen_lane_line_valid_edge_length; // 10
    double min_length = FLAGS_gen_lane_line_valid_line_length; //20

    for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
        auto &group_line = session->lane_center_line_group_list_raw[i];
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if (pt->invalid()) {
                continue;
            }
            session->debug_pos(pt->pos);
            double total_length = pt->context.next_max_length + pt->context.prev_max_length;
            // 滤除长度不满足要求的车道线点
            if (FLAGS_gen_lane_line_filter_line_by_length_enable && total_length < min_length) {
                pt->filter_status = 7; 
                continue;
            }
            double max_length = 0;
            auto &next_vec = next ? pt->context.all_next : pt->context.all_prev;
            // 选出next_vec中最长的前后继轨迹的最大长度
            for (int k = 0; k < next_vec.size(); ++k) {
                auto &next_ptr = next_vec[k];
                if (next_ptr.src->invalid()) {
                    continue;
                }
                auto next_length = next ?
                    next_ptr.src->context.next_max_length : next_ptr.src->context.prev_max_length;
                max_length = std::max(next_length, max_length);
            }
            for (int k = 0; k < next_vec.size(); ++k) {
                auto &next_ptr = next_vec[k];
                if (next_ptr.src->invalid()) {
                    continue;
                }
                auto next_length = next ?
                    next_ptr.src->context.next_max_length : next_ptr.src->context.prev_max_length;
                double gap = fabs(max_length - next_length);  // 当前节点的长度与最大长度的差距
                if (next_length > scope || gap < scope) {
                    continue;
                }
                next_ptr.status = 11;  // 无效点
                // auto &invalid_vec = next ? pt->context.invalid_next : pt->context.invalid_prev;
                // invalid_vec.push_back(next_ptr);
                // VEC_ERASE(next_vec, k);
                // next_ptr已经不满足长度等要求，next_ptr之后或之前的节点只要来自于一条大的车道中心线，同样被设置为无效点
                auto &prev_vec = next ? 
                    next_ptr.src->context.all_prev : next_ptr.src->context.all_next;
                for (int p = 0; p < prev_vec.size(); ++p) {
                    auto &tmp_ptr = prev_vec[p];
                    if (tmp_ptr.src != pt.get()) {
                        continue;
                    }
                    tmp_ptr.status = 11;
                    // auto &invalid_prev = next ? 
                    //     next_ptr.src->context.invalid_prev : next_ptr.src->context.invalid_next;
                    // invalid_prev.push_back(tmp_ptr);
                    // VEC_ERASE(prev_vec, p);
                }
            }
        }
    }
     
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::smooth_group_lane_center(RoadModelSessionData* session) {
    double edge_score_thres = FLAGS_gen_lane_line_edge_score_thres; // 0.15
    double scope = FLAGS_gen_lane_line_edge_score_scope;  // 20
    double min_scope = FLAGS_gen_lane_line_edge_score_min_scope; // 10

    for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
        auto &group_line = session->lane_center_line_group_list_raw[i];
        for (int64_t j = 0; j < group_line->list.size(); ++j) {
            auto &pt = group_line->list[j];
            if (pt->invalid()) {
                continue;
            }
            session->debug_pos(pt->pos);
            for (int k = 0; k < pt->context.all_next.size(); ++k) {
                auto &next_ptr = pt->context.all_next[k];
                if (next_ptr.src->invalid()) {
                    continue;
                }
                double score = next_ptr.score;
                next_ptr.status = 1;
                if (score < edge_score_thres) {
                    next_ptr.status = 10;
                } 
            }
            for (int k = 0; k < pt->context.all_prev.size(); ++k) {
                auto &next_ptr = pt->context.all_prev[k];
                if (next_ptr.src->invalid()) {
                    continue;
                }
                double score = next_ptr.score;
                next_ptr.status = 1;
                if (score < edge_score_thres) {
                    next_ptr.status = 10;
                }
            }
        }
    }

    // // 找联通区域
    // UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> line_map;
    // std::vector<LaneCenterFeature*> context_list;
    // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    //     auto &group_line = session->lane_center_line_group_list_raw[i];
    //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
    //         auto &lc = group_line->list[j];
    //         if (lc->invalid()) {
    //             continue;
    //         }
    //         context_list.clear();
    //         lc->context.get_context_list(scope, lc.get(), context_list,
    //                 [](LaneCenterFeature* curr, ParamPair<LaneCenterFeature> &pair)->bool{
    //                     return true;
    //                 }, 1);
    //         for (auto &next_lc : context_list) {
    //             if (next_lc->invalid()) {
    //                 continue;
    //             }
    //             double dis = alg::calc_dis(next_lc->pos, lc->pos);
    //             if (dis < min_scope) {
    //                 continue;
    //             }
    //             line_map.clear();
    //             if (lc->on_same_line(scope, next_lc, true, 
    //                         [](ParamPair<LaneCenterFeature> &ptr)->bool {
    //                             return ptr.status == 1;
    //                         }, 
    //                         line_map)) {
    //                 continue;
    //             }
    //             line_map.clear();
    //             if (!lc->on_same_line(scope, next_lc, true, line_map)) {
    //                 continue;
    //             }

    //             auto raw_dir = alg::get_dir(next_lc->pos, lc->pos);
    //             auto proc_next = lc.get();
    //             std::vector<ParamPair<LaneCenterFeature>*> pair_list;

    //             while (proc_next != NULL) {
    //                 if (MAP_NOT_FIND(line_map, proc_next)) {
    //                     break;
    //                 }
    //                 auto &line_vec = line_map[proc_next];
    //                 double min_dis = DBL_MAX;
    //                 LaneCenterFeature* valid_lc = NULL;
    //                 for (auto &tmp_lc : line_vec) {
    //                     if (tmp_lc == next_lc) {
    //                         valid_lc = tmp_lc;
    //                         break;
    //                     }
    //                     double dis = alg::calc_vertical_dis(tmp_lc->pos, lc->pos, raw_dir);
    //                     if (dis < min_dis) {
    //                         min_dis = dis;
    //                         valid_lc = tmp_lc;
    //                     }
    //                 }

    //                 if (valid_lc == NULL) {
    //                     break;
    //                 }
    //                 bool has_valid = false;
    //                 for (auto &tmp_lc_ptr : proc_next->context.all_next) {
    //                     auto &tmp_lc = tmp_lc_ptr.src;
    //                     if (tmp_lc == valid_lc) {
    //                         pair_list.push_back(&tmp_lc_ptr);
    //                         break;
    //                     }
    //                 }
    //                 if (proc_next == valid_lc) {
    //                     FLOG_POINT(valid_lc->pos, "dead_cycle");
    //                     break;
    //                 }
    //                 proc_next = valid_lc;
    //             }
    //             if (proc_next != next_lc) {
    //                 continue;
    //             }
    //             for (auto &pair : pair_list) {
    //                 if (pair->src->filter_status > 1) {
    //                     // pair->src->filter_status = 1;
    //                 }
    //                 if (pair->status != 1) {
    //                     pair->status = 2;
    //                 }
    //             }
    //         }
    //     }
    // }
    // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    //     auto &group_line = session->lane_center_line_group_list_raw[i];
    //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
    //         auto &pt = group_line->list[j];
    //         if (pt->invalid()) {
    //             continue;
    //         }
    //         session->debug_pos(pt->pos);
    //         for (int k = 0; k < pt->context.all_next.size(); ++k) {
    //             auto &next_ptr = pt->context.all_next[k];
    //             if (next_ptr.src->invalid()) {
    //                 continue;
    //             }
    //             if (next_ptr.status < 1 || next_ptr.status >= 10 ) {
    //                 pt->context.invalid_next.push_back(next_ptr);
    //                 VEC_ERASE(pt->context.all_next, k);
    //             }
    //         }
    //         for (int k = 0; k < pt->context.all_prev.size(); ++k) {
    //             auto &next_ptr = pt->context.all_prev[k];
    //             if (next_ptr.src->invalid()) {
    //                 continue;
    //             }
    //             if (next_ptr.status < 1 || next_ptr.status >= 10 ) {
    //                 pt->context.invalid_prev.push_back(next_ptr);
    //                 VEC_ERASE(pt->context.all_prev, k);
    //             }
    //         }
    //     }
    // }
     

    // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    //     auto &group_line = session->lane_center_line_group_list_raw[i];
    //     std::vector<double> status_vec_list(group_line->list.size());
    //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
    //         auto &pt = group_line->list[j];
    //         double max_score = 0;
    //         int max_score_index = 0;
    //         if (pt->context.all_prev.size() == 0) {
    //             continue;
    //         }
    //         if (j == group_line->list.size() - 1) {
    //             max_score = status_vec_list[j - 1];
    //         }
    //         for (int k = 0; k < pt->context.all_prev.size(); ++k) {
    //             auto &next_ptr = pt->context.all_prev[k];
    //             double score = next_ptr.score;
    //             if (score > max_score) {
    //                 max_score_index = k;
    //                 max_score = score;
    //             }
    //         }
    //         status_vec_list[j] = max_score;
    //         if (j == group_line->list.size() - 1) {
    //             auto &max_prev = pt->context.all_prev[max_score_index];
    //             max_prev.score = max_score;
    //             for (auto &tmp_ptr : max_prev.src->context.all_next) {
    //                 if (tmp_ptr.src == pt.get()) {
    //                     tmp_ptr.score = max_score;
    //                 }
    //             }
    //         }
    //     }
    // }
    // // for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
    // //     auto &group_line = session->lane_center_line_group_list_raw[i];
    // //     std::vector<LaneCenterFeature*> context_list;
    // //     for (int64_t j = 0; j < group_line->list.size(); ++j) {
    // //         auto &pt = group_line->list[j];
    // //         if (pt->context.all_prev.size() > 1 && pt->context.all_next.size() > 1)  {
    // //             continue;
    // //         }
    // //         context_list.clear();
    // //         pt->get_context_list(10);

    // //     }
    // // }

    return fsdmap::SUCC;
}


int RoadModelProcGenLaneLine::match_lane_center_line_raw(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    search_lane_center_group_line(session, trail_tree, tar_line);
    using line_score = std::pair<LaneCenterGroupLine*, double>;
    std::vector<std::tuple<int64_t, int64_t, int>> sub_line_list;
    int64_t start_index = 0;
    int64_t end_index = 0;
    UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> line_vec;
    LaneCenterFeature* prev_pt = NULL;
    // 根据tar_line中的点是否有group_pt，把tar_line分成几段
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        if (prev_pt == NULL) {
            start_index = j;
            end_index = j;
            prev_pt = pt;
            if (j == tar_line->list.size() - 1) {
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 0));
            }
            continue;
        }
        if (prev_pt->group_pt == NULL) {
            int mode = 0;
            if (pt->group_pt != NULL) {
                end_index = j - 1;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, mode));
                start_index = j;
                mode = 1;
            }
            if (j == tar_line->list.size() - 1) {
                end_index = j;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, mode));
            }
        } else {
            line_vec.clear();
            int mode = 1;
            if (pt->group_pt == NULL) {
                end_index = j - 1;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, 1));
                start_index = j;
                mode = 0;
            }
            if (j == tar_line->list.size() - 1) {
                end_index = j;
                sub_line_list.emplace_back(std::make_tuple(start_index, end_index, mode));
            }
        }
        prev_pt = pt;

    }

    for (auto &tuple : sub_line_list) {
        int64_t start_index = std::get<0>(tuple);
        int64_t end_index = std::get<1>(tuple);
        int type = std::get<2>(tuple);
        if (type == 0) {
            gen_new_lane_center_group(session, trail_group, 
                    tar_line, start_index, end_index, trail_tree);  // tar_line中 取[tar_line, start_index]的点构建新的车道中心线
            
        } else if (type == 1) {
            for (int64_t i = start_index; i <= end_index; ++i) {
                auto &tar_lc = tar_line->list[i];
                update_lane_center_group_line(session, tar_lc, trail_tree, 2);
            }
        }
    }
    // 更新前后继节点
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &tar_lc = tar_line->list[i];
        for (int j = 0; j < tar_lc->context.all_prev.size(); ++j) {
            auto &prev_lc_ptr = tar_lc->context.all_prev[j];
            auto &prev_lc = prev_lc_ptr.src;
            if (prev_lc->group_pt == NULL) {
                continue;
            }
            if (prev_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
                continue;
            }
            auto valid_group_lc = prev_lc->group_pt;
            if (alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
                        tar_lc->group_pt->dir)) {
                continue;
            }
            double score = prev_lc_ptr.score;
            prev_lc->group_pt->context.set_next(tar_lc->group_pt, score);
            tar_lc->group_pt->context.set_prev(prev_lc->group_pt, score);
        }
        // for (int j = 0; j < tar_lc->context.all_next.size(); ++j) {
        //     auto &next_lc_ptr = tar_lc->context.all_next[j];
        //     auto &next_lc = next_lc_ptr.src;
        //     if (next_lc->group_pt == NULL) {
        //         continue;
        //     }
        //     if (next_lc->group_pt->group_line == tar_lc->group_pt->group_line) {
        //         continue;
        //     }
        //     auto valid_group_lc = next_lc->group_pt;
        //     if (!alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
        //                 tar_lc->group_pt->dir)) {
        //         valid_group_lc = next_lc->raw_to_lc;
        //         if (next_lc->raw_to_lc != NULL 
        //                 && !alg::judge_front(valid_group_lc->pos, tar_lc->group_pt->pos, 
        //                     tar_lc->group_pt->dir)) {
        //         } else {
        //             continue;
        //         }
        //     }
        //     double score = next_lc_ptr.score;
        //     valid_group_lc->context.set_prev(tar_lc->group_pt, score);
        //     tar_lc->group_pt->context.set_next(valid_group_lc, score);
        // }
    }
    return fsdmap::SUCC;
}

// 计算tar_line里面每个点的最靠近的点（与该点5m范围内，方向一致，横向距离最短）
int RoadModelProcGenLaneLine::search_lane_center_group_line(RoadModelSessionData* session,
        LaneCenterFeatureTree &trail_tree,
        LaneCenterLine* tar_line) {
    double radius = FLAGS_gen_lane_line_search_radius;  //5
    double scope = FLAGS_gen_lane_line_scope;  //1
    double theta_thres = FLAGS_gen_lane_line_theta;  // 10
    double iou_thres_1 = FLAGS_gen_lane_line_iou_thres_1;  // 0.85
    double iou_thres_2 = FLAGS_gen_lane_line_iou_thres_2;  // 0.5

    std::vector<LaneCenterFeature*> secs;
    // <车道中心点pt，{pt 5m范围内的车道中心点src, src的下一个节点next(满足pt到src_next的方向向量的距离s小于1m)， 距离s}>
    UMAP<LaneCenterFeature*, std::vector<LineMatchParam<LaneCenterFeature>>> match_map;
    UMAP<LaneCenterFeature*, int> used_map;
    for (int64_t j = 0; j < tar_line->list.size(); ++j) {
        auto &pt = tar_line->list[j];
        session->debug_pos(pt->pos);
        secs.clear();
        used_map.clear();
        LaneCenterFeature* min_from_lc = NULL;
        LaneCenterFeature* min_to_lc = NULL;
        double min_dis = DBL_MAX;
        trail_tree.search(pt->pos, radius, secs);
        Eigen::Vector3d v_pt = alg::get_vertical_pos(pt->pos, pt->dir, 50, true);
        Eigen::Vector3d cross_point = {0, 0, 0};
        for (int64_t k = 0; k < secs.size(); ++k) {
            auto &src_pt = secs[k];
            if (MAP_FIND(used_map, src_pt)) {
                continue;
            }
            used_map[src_pt] = 1;
            session->debug_pos2(pt->pos, src_pt->pos);
            bool front = alg::judge_front(pt->pos, src_pt->pos, src_pt->dir);  // 判断pt->pos是否在src_pt->pos的前方
            auto &next_list = front ? src_pt->context.all_next : src_pt->context.all_prev;
            if (next_list.size() == 0) {  // src_pt没有后继点或前继点了
                double h_dis = alg::calc_hori_dis(pt->pos, src_pt->pos, src_pt->dir);  // 计算点pt->pos到直线 src_pt->dir的垂线的距离
                if (h_dis > radius / 2) {
                    continue;
                } 
                double dis = alg::calc_vertical_dis(src_pt->pos, pt->pos, pt->dir); // 计算向量src_pt->pos到方向pt->dir的距离
                if (dis > scope) {
                    continue;
                }
                double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, src_pt->dir); // 计算向量pt->pos到方向src_pt->dir的距离
                if (s_dis > scope) {
                    continue;
                }
                double theta = alg::calc_theta(src_pt->dir, pt->dir);
                if (theta > theta_thres) {
                    continue;
                }
                
                if (dis < min_dis) {
                    min_dis = dis;
                    min_from_lc = src_pt;
                    min_to_lc = NULL;
                }
                LineMatchParam<LaneCenterFeature> pair = {min_from_lc, min_to_lc, s_dis};
                match_map[pt].push_back(pair);
            } else {  // src_pt有后继点或前继点，在这些点中选出满足距离方向角度要求的点放入match_map，以及选出距离最小的点
                double min_tmp_dis = DBL_MAX;
                LaneCenterFeature* min_tmp_from_lc = NULL;
                LaneCenterFeature* min_tmp_to_lc = NULL;
                for (auto &next_lc_ptr : next_list) {
                    auto &next_lc = next_lc_ptr.src;
                    if (!alg::get_cross_point_for_segment(src_pt->pos, next_lc->pos,
                                pt->pos, v_pt, cross_point, 1)) {   // 计算交点
                        continue;
                    }
                    double dis = alg::calc_dis(pt->pos, cross_point);
                    if (dis > scope) {
                        continue;
                    }
                    Eigen::Vector3d tmp_dir;
                    if (front) {
                        tmp_dir = alg::get_dir(next_lc->pos, src_pt->pos);
                    } else {
                        tmp_dir = alg::get_dir(src_pt->pos, next_lc->pos);
                    }
                    double theta = alg::calc_theta(tmp_dir, pt->dir);
                    if (theta > theta_thres) {
                        continue;
                    }
                    double s_dis = alg::calc_vertical_dis(pt->pos, src_pt->pos, tmp_dir);  // pt->pos到tmp_dir的距离
                    if (s_dis > scope) {
                        continue;
                    }
                    if (front) {
                        LineMatchParam<LaneCenterFeature> pair = {src_pt, next_lc, s_dis};
                        match_map[pt].push_back(pair);
                    } else {
                        LineMatchParam<LaneCenterFeature> pair = {next_lc, src_pt, s_dis};
                        match_map[pt].push_back(pair);
                    }
                    if (dis < min_tmp_dis) {
                        min_tmp_dis = dis;
                        if (front) {
                            min_tmp_from_lc = src_pt;
                            min_tmp_to_lc = next_lc;
                        } else {
                            min_tmp_from_lc = next_lc;
                            min_tmp_to_lc = src_pt;
                        }
                    }
                }
                if (min_tmp_from_lc != NULL) {
                    if (min_tmp_dis < min_dis) {
                        min_dis = min_tmp_dis;
                        min_from_lc = min_tmp_from_lc;
                        min_to_lc = min_tmp_to_lc;
                    }
                    // src_pt->group_line->match_list_point[src_pt->line_index].push_back(pt);
                }
            }
        }
        pt->group_pt = min_from_lc;  //与该车道线中心点最靠近的车道线中心点，5m范围内，方向一致，横向距离最短
        pt->raw_to_lc = min_to_lc;
    }
    match_line_to_line(session, tar_line, match_map);
    return fsdmap::SUCC;
}

// 计算tar_line中每个点的：
// group_pt：与该车道线中心点最靠近的车道线中心点，5m范围内，方向一致，横向距离最短
// raw_to_lc：与group_pt是前后继节点的关系
int RoadModelProcGenLaneLine::match_line_to_line(RoadModelSessionData* session,
        LaneCenterLine* tar_line, 
        UMAP<LaneCenterFeature*, std::vector<LineMatchParam<LaneCenterFeature>>> &match_map) {
    double scope = FLAGS_gen_lane_line_match_line_long_scope;  // 20
    double theta_thres = FLAGS_gen_lane_line_theta;  // 10
    // for (auto &it : match_map) {
    //     SORT(it.second, 
    //             [](const LineMatchParam &l, const LineMatchParam &r)->bool {
    //                 return l.dis < r.dis;
    //             });
    // }

    std::vector<LaneCenterFeature*> prev_match;
    std::vector<LaneCenterFeature*> next_match;
    LaneCenterFeature* prev_lc = NULL;

    UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> line_vec;
    for (int64_t i = 0; i < tar_line->list.size(); ++i) {
        auto &lc = tar_line->list[i];

        do {
            if (prev_lc == NULL) {
                prev_lc = lc;
                break;
            }
            if (prev_lc->group_pt == NULL && lc->group_pt == NULL) {
                break;
            }
            if (prev_lc->group_pt == NULL) {
                auto match_pair = match_line_long_distance(
                        session, lc->group_pt, tar_line, i - 1, false);
                if (match_pair.from == NULL) {
                } else {
                    prev_lc->group_pt = match_pair.from;
                    prev_lc->raw_to_lc = match_pair.to;
                }
                break;
            }
            if (lc->group_pt == NULL) { // 为lc寻找group_pt和raw_to_lc
                auto match_pair = match_line_long_distance(
                        session, prev_lc->group_pt, tar_line, i, true);
                if (match_pair.from == NULL) {
                    
                } else {
                    lc->group_pt = match_pair.from;
                    lc->raw_to_lc = match_pair.to;
                }
                break;
            }
            line_vec.clear();
            //  prev_lc->group_pt后继20M范围内的点，遇到lc->group_pt停止,存入line_vec
            if (prev_lc->group_pt->context.on_same_line(
                        scope, prev_lc->group_pt, lc->group_pt, true, &line_vec)) {
                break;
            }
            auto match_pair = match_line_long_distance(
                    session, lc->group_pt, tar_line, i - 1, false);

            if (match_pair.from != prev_lc->group_pt) {
                prev_lc->group_pt = match_pair.from;
                prev_lc->raw_to_lc = match_pair.to;
                break;
            }
            match_pair = match_line_long_distance(
                    session, prev_lc->group_pt, tar_line, i, true);
            if (match_pair.from != lc->group_pt) {
                lc->group_pt = match_pair.from;
                lc->raw_to_lc = match_pair.to;
                break;
            }
        } while (0);

        LaneCenterFeature* need_base_lc = NULL;
        LaneCenterFeature* need_move_lc = NULL;  // 表示该车道中心点lc对应的group_pt与lc的前继节点的方向向量夹角差别较大，需要更新他的group_pt
        if (prev_lc->group_pt != NULL && lc->group_pt != NULL) {
            auto tmp_dir = alg::get_dir(lc->group_pt->pos, prev_lc->group_pt->pos);
            double theta = alg::calc_theta(tmp_dir, prev_lc->dir, true);  // 计算向量之间的夹角
            if (theta > theta_thres) {
                need_move_lc = lc;
                need_base_lc = prev_lc;
            }
        } else if (prev_lc->group_pt == NULL && lc->group_pt != NULL) {
            auto tmp_dir = alg::get_dir(lc->group_pt->pos, prev_lc->pos);
            double theta = alg::calc_theta(tmp_dir, prev_lc->dir, true);
            if (theta > theta_thres) {
                need_move_lc = lc;
                need_base_lc = prev_lc;
            }
        } else if (prev_lc->group_pt != NULL && lc->group_pt == NULL) {
            auto tmp_dir = alg::get_dir(lc->pos, prev_lc->group_pt->pos);
            double theta = alg::calc_theta(tmp_dir, prev_lc->dir, true);
            if (theta > theta_thres) {
                need_move_lc = prev_lc;
                need_base_lc = lc;
            }
        }
        if (need_move_lc != NULL) {
            auto &vec = match_map[need_move_lc]; // {need_move_lc 所有5m范围内的车道中心点src, src的下一个节点next(满足pt到src_next的方向向量的距离s小于1m)， 距离s}
            LaneCenterFeature* valid_group_from = NULL;
            LaneCenterFeature* valid_group_to = NULL;
            double min_theta = DBL_MAX;
            // 在候选点中重新找一个角度查最小的点作为group_pt，和raw_to_lc
            for (auto &tmp_group_pt : vec) {
                LaneCenterFeature *tar_group = tmp_group_pt.from;
                if (tar_group == need_move_lc->group_pt) {
                    continue;
                }
                auto tmp_dir = alg::get_dir(tar_group->pos, need_base_lc->pos);
                double theta = alg::calc_theta(tmp_dir, prev_lc->dir, true);
                if (theta > theta_thres) {
                    continue;
                }
                if (min_theta > theta) {
                    min_theta = theta;
                    valid_group_from = tar_group;
                    valid_group_to = tmp_group_pt.to;
                }
            }
            need_move_lc->group_pt = valid_group_from;
            need_move_lc->raw_to_lc = valid_group_to;
        }
        
        prev_lc = lc;
    }
     
    return fsdmap::SUCC;
}


int RoadModelProcGenLaneLine::make_lane_line_single(RoadModelSessionData* session) {
    for (auto &lc : session->lane_center_feature_gen) {
        session->debug_pos(lc->pos, lc->left->src->frame_id);
        if (lc->invalid()) {
            continue;
        }
        LaneCenterFeatureMatchPair* next_top = NULL;
        LaneCenterFeatureMatchPair* prev_top = NULL;

        if (lc->next_pair_list.size() > 0) {
            next_top = lc->next_pair_list.front();
        }
        if (lc->prev_pair_list.size() > 0) {
            prev_top = lc->prev_pair_list.front();
        }
        if (next_top != NULL) {
            auto next = next_top->get_other(lc);
            lc->context.set_next(next, next_top->score);
            next->context.set_prev(lc, next_top->score);
        }
        if (prev_top != NULL) {
            auto prev = prev_top->get_other(lc);
            lc->context.set_prev(prev, prev_top->score);
            prev->context.set_next(lc, prev_top->score);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::make_lane_line_mutil(RoadModelSessionData* session) {
    // for (auto &lc : session->lane_center_feature_gen) {
    //     session->debug_pos(lc->pos, lc->left->src->frame_id);
    //     if (lc->invalid()) {
    //         continue;
    //     }
    //     // get_valid_context1(session, lc, true);
    //     get_valid_context1(session, lc, false);
    // }
    // for (auto &lc : session->lane_center_feature_gen) {
    //     for (int i = 0; i < lc->context.all_next.size(); ++i) {
    //         auto &next_lc = lc->context.all_next[i];
    //         next_lc->context.set_prev(lc);
    //         next_lc->context.score_prev.push_back(lc->context.score_next[i]);
    //     }
    // }
    // for (auto &lc : session->lane_center_feature_gen) {
    //     std::vector<int64_t> index_vec(lc->context.all_prev.size());
    //     for (int i = 0; i < index_vec.size(); ++i) {
    //         index_vec[i] = i;
    //     }
    //     SORT(index_vec, [&](const int64_t &l, const int64_t &r){
    //                 return lc->context.score_prev[l] > lc->context.score_prev[r];
    //             });
    //     std::vector<LaneCenterFeature*> all_prev(index_vec.size());
    //     std::vector<double> score_prev(index_vec.size());
    //     for (int i = 0; i < index_vec.size(); ++i) {
    //         all_prev[i] = lc->context.all_prev[index_vec[i]];
    //         score_prev[i] = lc->context.score_prev[index_vec[i]];
    //     }
    //     std::swap(lc->context.all_prev, all_prev);
    //     std::swap(lc->context.score_prev, score_prev);
    // }
    // return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::get_valid_context1(
        RoadModelSessionData* session, LaneCenterFeature* lc, bool prev) {
    // double dir_threshold = FLAGS_gen_lane_line_valid_context_dir_threshold;
    // auto &vec = prev ? lc->prev_pair_list : lc->next_pair_list;
    // auto &next_vec = prev ? lc->context.all_prev : lc->context.all_next;
    // auto &next_vec_score = prev ? lc->context.score_prev : lc->context.score_next;
    // if (vec.size() == 0) {
    //     return fsdmap::SUCC;
    // }
    // using score_pair = std::tuple<double, bool, LaneCenterFeatureMatchPair*>;
    // std::vector<score_pair> dir_vec(vec.size());
    // for (int i = 0; i < vec.size(); ++i) {
    //     auto &src = vec[i];
    //     double dis = alg::calc_hori_dis(src->get_other(lc)->pos, lc->pos, lc->dir, true);
    //     dir_vec[i] = std::make_tuple(dis, true, src);
    // }
    // SORT(dir_vec, [](const score_pair &l, const score_pair &r) {
    //         return std::get<0>(l) < std::get<0>(r);
    //     });
    // std::vector<LaneCenterFeatureMatchPair*> score_vec;
    // score_vec.reserve(vec.size() / 5 + 1);
    // for (int i = 0; i < dir_vec.size(); ++i) {
    //     auto &curr = dir_vec[i];
    //     auto &curr_pair = std::get<2>(curr);
    //     auto curr_lc = curr_pair->get_other(lc);
    //     if (!std::get<1>(curr)) {
    //         continue;
    //     }
    //     for (int j = i + 1; j < dir_vec.size(); ++j) {
    //         auto &next = dir_vec[j];
    //         if (!std::get<1>(next)) {
    //             continue;
    //         }
    //         auto &next_pair = std::get<2>(next);
    //         double theta = alg::calc_theta(
    //                 std::get<2>(curr)->dir_gap, next_pair->dir_gap);
    //         if (theta <= dir_threshold) {
    //             auto next_lc = next_pair->get_other(lc);
    //             auto &next_pair_map = prev ? curr_lc->prev_pair_map : curr_lc->next_pair_map;
    //             if (MAP_NOT_FIND(next_pair_map, next_lc)) {
    //                 continue;
    //             }
    //             std::get<1>(next) = false;
    //         }
    //     }
    //     score_vec.push_back(curr_pair);
    // }
    // SORT(score_vec,
    //         [](const LaneCenterFeatureMatchPair *l, const LaneCenterFeatureMatchPair *r)->bool {
    //         return l->score > r->score;
    //         });
    // for (auto &pair : score_vec) {
    //     next_vec.push_back(pair->get_other(lc));
    //     next_vec_score.push_back(pair->score);
    // }
    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::get_valid_context(
        RoadModelSessionData* session, LaneCenterFeature* lc, bool prev) {
    // double dir_threshold = FLAGS_gen_lane_line_valid_context_dir_threshold;
    // auto &vec = prev ? lc->prev_pair_list : lc->next_pair_list;
    // auto &next_vec = prev ? lc->context.all_prev : lc->context.all_next;
    // auto &next_vec_score = prev ? lc->context.score_prev : lc->context.score_next;
    // if (vec.size() == 0) {
    //     return fsdmap::SUCC;
    // }
    // auto &first_pair = vec.front();
    // std::vector<std::pair<double, bool> > dir_vec(vec.size());
    // dir_vec[0] = std::make_pair(0, true);
    // Eigen::Vector3d dir1 = first_pair->dir_gap;

    // for (int i = 1; i < vec.size(); ++i) {
    //     auto &next_pair = vec[i];
    //     Eigen::Vector3d dir2 = next_pair->dir_gap;
    //     double theta = alg::calc_theta_with_dir(dir1, dir2);
    //     dir_vec[i] = std::make_pair(theta, false);
    // }
    // next_vec.push_back(first_pair->get_other(lc));
    // next_vec_score.push_back(first_pair->score);
    // // 按照最近距离最大排序
    // for (int tmp = 1; tmp < vec.size(); ++tmp) {
    //     double max_gap = 0;
    //     int max_index = 0;
    //     for (int i = 1; i < vec.size(); ++i) {
    //         if (dir_vec[i].second) {
    //             // 已经选中的
    //             continue;
    //         }
    //         double min_gap = 1000000;
    //         // 计算与池子中最近的
    //         for (int j = 0; j < vec.size(); ++j) {
    //             if (!dir_vec[j].second) {
    //                 continue;
    //             }
    //             double gap = fabs(dir_vec[i].first - dir_vec[j].first);
    //             if (gap < min_gap) {
    //                 min_gap = gap;
    //             }
    //         }
    //         if (min_gap > max_gap) {
    //             max_gap = min_gap;
    //             max_index = i;
    //         }
    //     }
    //     if (max_gap >= dir_threshold) {
    //         next_vec.push_back(vec[max_index]->get_other(lc));
    //         next_vec_score.push_back(vec[max_index]->score);
    //         dir_vec[max_index].second = true;
    //     } else {
    //         tmp = vec.size();
    //         break;
    //     }
    // }

    return fsdmap::SUCC;
}

int RoadModelProcGenLaneLine::save_frame_log_lane_center(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &line_group,
        LaneCenterLine* tar_line, int times) {
    if (!FLAGS_gen_lane_line_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (!FLAGS_gen_lane_line_save_data_frame_log_enable) {
        return fsdmap::SUCC;
    }
    std::vector<std::shared_ptr<utils::DisplayInfo>> ptr_vec;
    std::vector<utils::DisplayInfo*> log_vec;
    auto mlog = session->add_ptr(ptr_vec);
    mlog->init(utils::DisplayInfo::LINE, "raw_lane_center");
    mlog->color = {255, 255, 255};  // 白色
    log_vec.push_back(mlog.get());
    for (auto &lc : tar_line->list) {
        // session->debug_pos(pt->pos);
        
        auto tmp_pos = alg::get_vertical_pos(lc->pos, lc->dir, 1); // lc->pos垂线方向上延申1m
        auto &ele = mlog->add(tmp_pos);
        ele.label.score = lc->score;
        auto log_ptr = session->add_ptr(ptr_vec);
        log_ptr->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", lc->line_id);
        log_ptr->color = {255, 255, 0};  // 黄色
        auto &src_pt =lc->group_pt;
        if (src_pt->group_line->match_list[src_pt->line_index].size() == 1) {
            log_ptr->color = {255, 0, 0};
        }
        // tmp_pos指向lc->group_pt->pos,红色或者黄色
        log_ptr->add(tmp_pos);
        log_ptr->add(lc->group_pt->pos);
        log_vec.push_back(log_ptr.get());
    }
    for (int64_t i = 0; i < line_group.size(); ++i) {
        auto &group_list = line_group[i];
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &lc = group_list->list[j];
            for (int k = 0; k < lc->context.all_next.size(); ++k) {
                auto &next_lc_ptr = lc->context.all_next[k];
                auto &next_lc = next_lc_ptr.src;
                auto log_ptr = session->add_ptr(ptr_vec);
                log_ptr->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", lc->line_id);
                log_ptr->color = {0, 255, 0};
                
                // session->debug_pos(pt->pos);
                // lc->pos指向next_lc->pos， 绿色
                auto &ele1 = log_ptr->add(lc->pos, 1, group_list->match_list[i].size());
                auto &ele2 = log_ptr->add(next_lc->pos, 1, group_list->match_list[i].size());
                ele1.label.opt_label = i;
                ele1.label.score = group_list->match_score[j];
                ele2.label.score = group_list->match_score[j];
                log_vec.push_back(log_ptr.get());
            }
        }
    }

    auto log_name = session->get_debug_dir("gen_lane_raw_detail/{}_{}.png", 
            "merge_raw_lane_center", times);
    auto pcd_name = session->get_debug_dir("gen_lane_raw_detail/{}_{}.pcd", 
            "merge_raw_lane_center", times);
    utils::save_display_image(log_name.c_str(), session->_scope, log_vec);
    utils::save_display_pcd(pcd_name.c_str(), session->_scope, log_vec);
    return fsdmap::SUCC;
}



int RoadModelProcGenLaneLine::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_gen_lane_line_save_data_enable) {
        return fsdmap::SUCC;
    }
    double scope_buff = FLAGS_display_scope_buff;
    session->set_display_name("gen_lane_line");
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
        lc_id++;
        session->debug_pos(lc->pos);
        line_id_log.clear();
        auto &frame_id = lc->frame_id;
        if (MAP_NOT_FIND(frame_map, frame_id)) {
            frame_map[frame_id] = std::vector<utils::DisplayInfo*>();
        }
        for (auto &next_lc_ptr : lc->context.all_next) {
            auto next_lc = next_lc_ptr.src;
            auto log_ptr = session->add_ptr(log_ptr_list);
            log_ptr->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", lc->line_id);
            if (lc->src_status == 2) {
                log_ptr->color = {255, 255, 0};
            } else if (lc->src_status == 3) {
                log_ptr->color = {0, 255, 0};
            }
            log_ptr->log_name = lc->line_id;
            frame_map[frame_id].push_back(log_ptr.get());
            log_ptr->add(lc->pos, 1);
            auto &ele = log_ptr->add(next_lc->pos, 1);
            ele.label.score = next_lc->context.next_max_length;

            session->add_debug_log(log_ptr.get());
            
            line_id_log.push_back(log_ptr.get());
        }
        if (!FLAGS_gen_lane_line_save_data_save_detail_enable) {
            continue;
        }
        if (lc->context.all_next.size() == 0) {
            continue;
        }
        // auto key_pose = session->data_processer->get_key_pose(frame_id);
        // if (key_pose == NULL) {
        //     continue;
        // }
        // auto log_name = session->get_debug_dir("{}_{}_{}.png", lc->line_id, "gen_lane_line", lc_id);
        // utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
        // box.set_resolution(FLAGS_display_scale_rate);
        // utils::save_display_image(log_name.c_str(), box, line_id_log);
    }
    if (FLAGS_gen_lane_line_save_data_save_frame_enable) {
        for (auto &it : frame_map) {
            auto key_pose = session->data_processer->get_key_pose(it.first);
            if (key_pose == NULL) {
                continue;
            }
            utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
            box.set_resolution(FLAGS_display_scale_rate);
            // box.dir.z() = -key_pose->yaw;
            auto log_name = session->get_debug_dir("{}_{}.png", "gen_lane_line", it.first);
            utils::save_display_image(log_name.c_str(), box, it.second);
        }
    }

    session->save_debug_info("gen_lane_line");
    session->set_display_name("gen_lane_line_merge");
    for (int64_t i = 0; i < session->lane_center_line_group_list.size(); ++i) {
        auto &group_list = session->lane_center_line_group_list[i];
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
        log->color = {0, 255, 0};
        if (group_list->list.size() == 1) {
            log->type = utils::DisplayInfo::POINT;
        }
        for (int64_t j = 0; j < group_list->list.size(); ++j) {
            auto &pt = group_list->list[j];
            session->debug_pos(pt->pos);
            auto &ele = log->add(pt->pos, 1, group_list->match_list[j].size());
            ele.label.opt_label = i;
            // ele.label.score = pt->score;
            ele.label.score = pt->context.max_length;
            ele.label.intensity = pt->context.next_max_length;
            ele.label.intensity_opt = pt->context.prev_max_length;
            ele.label.cloud_pano_seg = pt->match_level.lane_num;
            ele.label.cloud_line_seg = pt->match_level.lane_type;
            if (pt->match_level.lane_num > 1) {
                ele.color = {100, 200, 200};
            }
            if (pt->invalid()) {
                ele.color = {255, 0, 0};
            }
            for (int k = 0; k < pt->context.all_next.size(); ++k) {
                auto &next_ptr = pt->context.all_next[k];
                auto &next = next_ptr.src;
                if (next->group_line == pt->group_line) {
                    continue;
                }

                auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "merge_boundary");
                tlog->color = {255, 255, 0};
                tlog->add(pt->pos);
                tlog->add(next->pos);
            }
        }
    }
    session->save_debug_info("gen_lane_line_merge");
    session->set_display_name("gen_lane_line_merge_raw");
    for (int64_t i = 0; i < session->lane_center_line_group_list_raw.size(); ++i) {
        auto &group_list = session->lane_center_line_group_list_raw[i];
        for (int64_t j = 0; j < group_list->list.size(); ++j) {

            auto &pt = group_list->list[j];
            session->debug_pos(pt->pos);
                  if(pt->left)
                    {
                        LOG_INFO("gen_lane color:{} type:{}",pt->left->attr.color,pt->left->attr.type)
                    }
                    if(pt->right)
                    {
                        LOG_INFO("gen_lane color:{} type:{}",pt->right->attr.color,pt->left->attr.type)
                    }
            double length = pt->context.next_max_length + pt->context.prev_max_length;
            for (int k = 0; k < pt->context.all_next.size(); ++k) {
                auto &next_ptr = pt->context.all_next[k];
                auto &next = next_ptr.src;
                auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "merge_raw_lane");
                tlog->color = {255, 255, 0};
                if (next_ptr.status == 2) {
                    tlog->color = {0, 255, 0};
                }
                if (pt->invalid() || next->invalid() 
                        || next_ptr.invalid()) {
                    tlog->color = {255, 0, 0};
                }
                auto score = next_ptr.score;
                auto &ele1 = tlog->add(pt->pos, 1, next_ptr.status);
                ele1.label.score = score;
                ele1.label.cloud_bev_label_score = pt->score;
                ele1.label.opt_label = pt->filter_status;
                ele1.label.intensity = pt->context.next_max_length;
                ele1.label.intensity_opt = pt->context.prev_max_length;
                auto &ele2 = tlog->add(next->pos, 1, next_ptr.status);
                ele2.label.score = score;
                ele2.label.cloud_bev_label_score = next->score;
                ele2.label.opt_label = pt->filter_status;
                ele2.label.intensity = pt->context.next_max_length;
                ele2.label.intensity_opt = pt->context.prev_max_length;
            }
            for (int k = 0; k < pt->context.invalid_next.size(); ++k) {
                auto &next_ptr = pt->context.invalid_next[k];
                auto &next = next_ptr.src;

                auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "merge_raw_lane");
                tlog->color = {255, 0, 0};
                if (next_ptr.status == 2) {
                    tlog->color = {0, 255, 0};
                }
                auto score = next_ptr.score;
                auto &ele1 = tlog->add(pt->pos, 1, next_ptr.status);
                ele1.label.score = score;
                ele1.label.cloud_bev_label_score = pt->score;
                ele1.label.opt_label = pt->filter_status;
                ele1.label.intensity = pt->context.next_max_length;
                ele1.label.intensity_opt = pt->context.prev_max_length;
                auto &ele2 = tlog->add(next->pos, 1, next_ptr.status);
                ele2.label.score = score;
                ele2.label.cloud_bev_label_score = next->score;
                ele2.label.opt_label = pt->filter_status;
                ele2.label.intensity = pt->context.next_max_length;
                ele2.label.intensity_opt = pt->context.prev_max_length;
            }
            if (pt->context.all_next.size() == 0) {
                auto tlog = session->add_debug_log(utils::DisplayInfo::POINT, "merge_raw_lane");
                tlog->color = {100, 100, 100};
                auto &ele = tlog->add(pt->pos);
                ele.label.opt_label = pt->filter_status;
                ele.label.intensity_opt = length;
            }
            if (pt->context.all_prev.size() == 0) {
                auto tlog = session->add_debug_log(utils::DisplayInfo::POINT, "merge_raw_lane");
                tlog->color = {0, 100, 100};
                auto &ele = tlog->add(pt->pos);
                ele.label.opt_label = pt->filter_status;
                ele.label.intensity_opt = length;
            }
        }
    }
    session->save_debug_info("gen_lane_line_merge_raw");

    return fsdmap::SUCC;
}

}
}
