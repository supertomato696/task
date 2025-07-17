


#include "road_model_proc_filter_lane.h"


DEFINE_bool(filter_lane_enable, true, "filter_lane_enable");
DEFINE_bool(filter_lane_debug_pos_enable, true, "filter_lane_debug_enable");
DEFINE_bool(filter_lane_save_data_enable, true, "filter_lane_save_data_enable");
DEFINE_double(filter_lane_lane_length_threshold, 10, "filter_lane_lane_length_threshold");
DEFINE_double(filter_lane_smooth_length_thres, 50, "filter_lane_smooth_length_thres");
DEFINE_double(filter_lane_search_trail_radius, 2, "filter_lane_search_trail_radius");
DEFINE_double(filter_lane_search_trail_theta, 30, "filter_lane_search_trail_theta");


namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcFilterLane::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_filter_lane_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_filter_lane_debug_pos_enable;

    CHECK_FATAL_PROC(match_trail(session), "match_trail");

    CHECK_FATAL_PROC(filter_lane_by_length(session), "filter_lane_by_length");

    CHECK_FATAL_PROC(smooth_filter_status(session), "smooth_filter_status");

    CHECK_FATAL_PROC(remark_road_index(session), "remark_road_index");

    CHECK_FATAL_PROC(smooth_road_index(session), "smooth_road_index");

    CHECK_FATAL_PROC(smooth_side_status(session), "smooth_side_status");

    CHECK_FATAL_PROC(mark_filter_status(session), "mark_filter_status");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcFilterLane::mark_filter_status(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                auto &lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                auto rc = poss->boundary.get_road_center(lc->road_index);
                if (rc != NULL && rc->invalid()) {
                    lc->filter_status = 2;
                }
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcFilterLane::match_trail(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                auto &lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                auto trail_pos = search_nearest_trail(session, lc);
                if (trail_pos != NULL) {
                    double theta = alg::calc_theta(poss->dir, trail_pos->dir);
                    if (theta < 90) {
                        lc->dir_status = 1;
                    } else {
                        lc->dir_status = 2;
                    }
                }
            }
        }
    }
    return fsdmap::SUCC;
}

KeyPose* RoadModelProcFilterLane::search_nearest_trail(RoadModelSessionData* session, 
        LaneCenterFeature* lc) {
    double max_same_dir_theta = FLAGS_filter_lane_search_trail_theta;
    double radius = FLAGS_filter_lane_search_trail_radius;
    auto pos = lc->pos;
    std::vector<KeyPose*> secs;
    session->key_pose_tree.search(pos, radius, secs);
    KeyPose* left_min_poss = NULL;
    double left_min_dis = DBL_MAX;
    KeyPose* right_min_poss = NULL;
    double right_min_dis = DBL_MAX;
    Eigen::Vector3d v_pt = alg::get_vertical_pos(lc->pos, lc->dir, 50, true);
    for (auto& n_pos : secs) {
        if (n_pos->next == NULL) {
            continue;
        }
        double theta = alg::calc_theta(n_pos->dir, lc->dir, true);
        if (theta > max_same_dir_theta) {
            continue;
        }
        Eigen::Vector3d cross_point;
        if (!alg::get_cross_point_for_segment(n_pos->pos, n_pos->next->pos,
                    lc->pos, v_pt, cross_point, 1)) {
            continue;
        }
        double dis = alg::calc_dis(cross_point, lc->pos);
        if (dis > radius) {
            continue;
        }
        if (alg::judge_left(n_pos->pos, lc->pos, lc->dir) < 0) {
            if (left_min_dis > dis) {
                left_min_dis = dis;
                left_min_poss = n_pos;
            }
        } else {
            if (right_min_dis > dis) {
                right_min_dis = dis;
                right_min_poss = n_pos;
            }
        }
    }
    if (left_min_poss != NULL && right_min_poss != NULL) {
        double theta = alg::calc_theta(left_min_poss->dir, right_min_poss->dir);
        if (theta > 90) {
            return NULL;
        } else if (left_min_dis < right_min_dis) {
            return left_min_poss;
        } else {
            return right_min_poss;
        }
    }
    if (left_min_poss != NULL) {
        return left_min_poss;
    } 
    if (right_min_poss != NULL) {
        return right_min_poss;
    }
    return NULL;
}

int RoadModelProcFilterLane::filter_lane_by_length(RoadModelSessionData* session) {
    double scope = FLAGS_filter_lane_lane_length_threshold;
    std::vector<LaneCenterFeature*> context_list;
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                context_list.clear();
                auto &lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                lc->get_context_list(2 * scope, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        return curr->invalid() ? NULL : curr->fill_prev;},
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        return curr->invalid() ? NULL : curr->fill_next;},
                        0);
                double total_length = 0;
                int curr_index = -1;
                for (int k = 1; k < context_list.size(); ++k) {
                    auto &prev_lc = context_list[k - 1];
                    auto &curr_lc = context_list[k];
                    total_length += alg::calc_dis(prev_lc->pos, curr_lc->pos);
                }
                if (total_length < scope) {
                    lc->filter_status = 15;
                }
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcFilterLane::smooth_filter_status(RoadModelSessionData* session) {
    double scope = FLAGS_filter_lane_lane_length_threshold;
    std::vector<LaneCenterFeature*> context_list;
    std::vector<int> side_vec;
    std::vector<int> road_vec;
    std::vector<int> prev_valid_vec;
    std::vector<int> next_valid_vec;
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                context_list.clear();
                side_vec.clear();
                road_vec.clear();
                prev_valid_vec.clear();
                next_valid_vec.clear();
                auto &lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                lc->get_context_list(2 * scope, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_prev;},
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_next;},
                        0);
                double total_length = 0;
                int curr_index = -1;
                for (int k = 0; k < context_list.size(); ++k) {
                    auto &curr_lc = context_list[k];
                    int valid = curr_lc->invalid() ? 1 : 0;
                    if (curr_lc == lc) {
                        curr_index = k;
                        prev_valid_vec.push_back(valid);
                    }
                    if (curr_index == -1) {
                        prev_valid_vec.push_back(valid);
                    }
                    if (curr_index >= 0 && k >= curr_index) {
                        next_valid_vec.push_back(valid);
                    }
                    if (curr_lc->invalid()) {
                        continue;
                    }
                    if (curr_lc->road_segment == lc->road_segment) {
                        side_vec.push_back(curr_lc->side_status);
                        road_vec.push_back(curr_lc->road_index);
                    }

                }
                int smooth_side = 1;
                double rate = 0;
                // if (side_vec.size() > 1) {
                //     alg::calc_max_count_int(side_vec, smooth_side, rate);
                //     lc->side_status = smooth_side;
                // }
                // int smooth_road_index = 1;
                // if (road_vec.size() > 1) {
                //     alg::calc_max_count_int(road_vec, smooth_road_index, rate);
                //     lc->road_index = smooth_road_index;
                // }

                int smooth_prev_valid = -1;
                int smooth_next_valid = -1;
                if (prev_valid_vec.size() > 1) {
                    alg::calc_max_count_int(prev_valid_vec, smooth_prev_valid, rate);
                }
                if (next_valid_vec.size() > 1) {
                    alg::calc_max_count_int(next_valid_vec, smooth_next_valid, rate);
                }

                if (smooth_prev_valid == smooth_next_valid) {
                    if (smooth_prev_valid == 0) {
                        lc->filter_status = 1;
                    } else if (smooth_prev_valid == 1 && lc->filter_status <= 1) {
                        lc->filter_status = 16;
                    }
                } else if (smooth_prev_valid == -1) {
                    if (smooth_next_valid == 1 && lc->filter_status <= 1) {
                        lc->filter_status = 16;
                    }
                } else if (smooth_next_valid == -1) {
                    if (smooth_prev_valid == 1 && lc->filter_status <= 1) {
                        lc->filter_status = 16;
                    }
                }
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcFilterLane::remark_road_index(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            auto &boundary = poss->boundary;
            int curr_index = 0;
            UMAP<RoadCenterFeature*, std::vector<LaneCenterFeature*>> rc_lc_map; 
            for (int rc_index = 0; rc_index < boundary.road_center_list.size(); ++rc_index) {
                auto &rc = boundary.road_center_list[rc_index];
                if (rc == boundary.curr) {
                    curr_index = rc_index;
                }
                int fill_num = 0;
                for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                    auto lc = poss->filled_lane_sample[j];
                    if (!rc->in_scope(lc->pos)) {
                        continue;
                    }
                    rc_lc_map[rc].push_back(lc);
                    ++fill_num;
                }
                if (fill_num == 0 && rc != boundary.curr) {
                    rc->filter_status = 3;
                }
            }
            int valid_index = 1;
            for (int rc_index = curr_index + 1; rc_index < boundary.road_center_list.size(); ++rc_index) {
                auto &rc = boundary.road_center_list[rc_index];
                if (rc->invalid()) {
                    continue;
                }
                rc->index = valid_index++;
            }
            valid_index = 1;
            for (int rc_index = curr_index - 1; rc_index >= 0; --rc_index) {
                auto &rc = boundary.road_center_list[rc_index];
                if (rc->invalid()) {
                    continue;
                }
                rc->index = -valid_index++;
            }
            for (int rc_index = 0; rc_index < boundary.road_center_list.size(); ++rc_index) {
                auto &rc = boundary.road_center_list[rc_index];
                if (rc->invalid()) {
                    continue;
                }
                auto &lc_vec = rc_lc_map[rc];
                for (auto &lc : lc_vec) {
                    lc->road_index = rc->index;
                }
            }
        }
    }
     
    return fsdmap::SUCC;
}

int RoadModelProcFilterLane::smooth_road_index(RoadModelSessionData* session) {
    double length_thres = FLAGS_filter_lane_smooth_length_thres;
    // using param = std::tuple<double, LaneCenterFeature*, LaneCenterFeature*, bool>;
    using param = std::tuple<double, LaneCenterFeature*, LaneCenterFeature*, bool, int, int>;
    std::vector<param> lc_list;
    std::vector<LaneCenterFeature*> context_list, next_lc_list;
    std::vector<int> road_index_vec;
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                auto lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                // for (auto &next_lc_ptr : lc->context.all_next) {
                auto next_lc = lc->fill_next;
                if (next_lc == NULL || next_lc->invalid()) {
                    continue;
                }
                if (next_lc->road_segment != lc->road_segment) {
                    continue;
                }
                if (next_lc->road_index == lc->road_index) {
                    continue;
                }
                auto &next_poss = next_lc->key_pose;
                auto oppo_poss = poss->boundary.left_oppo_pos.src;
                auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid()
                //     && poss->boundary.same_road_status == 1;
                // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid()
                //     && next_poss->boundary.same_road_status == 1;
                // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                bool has_valid_oppo = true;
                bool has_valid_oppo_next = true;
                if (poss->inter_status < 2 
                        && next_poss->inter_status < 2
                        && has_valid_oppo != has_valid_oppo_next) {
                    continue;
                }
                context_list.clear();
                double length1 = lc->get_context_list(1000, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_prev == NULL 
                                || curr->fill_prev->invalid()
                                || curr->fill_prev->road_segment != curr->road_segment
                                || curr->fill_prev->road_index != curr->road_index) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_prev;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;

                            if (poss->inter_status < 2
                                    && next_poss->inter_status < 2
                            && has_valid_oppo != has_valid_oppo_next) {
                            return NULL;
                        }
                        return curr->fill_prev;},

                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_next == NULL 
                                || curr->fill_next->invalid()
                                || curr->fill_next->road_segment != curr->road_segment
                                || curr->fill_next->road_index != curr->road_index) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_next;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();
                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                        if (poss->inter_status < 2
                            && next_poss->inter_status < 2
                            && has_valid_oppo != has_valid_oppo_next) {
                            return NULL;
                        }
                        return curr->fill_next;},
                        0);

                if (poss->inter_status > 1) {
                    length1 /= 2;
                }
                context_list.clear();
                double length2 = next_lc->get_context_list(1000, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_prev == NULL 
                                || curr->fill_prev->invalid()
                                || curr->fill_prev->road_segment != curr->road_segment
                                || curr->fill_prev->road_index != curr->road_index) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_prev;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();
                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                            if (poss->inter_status < 2
                                    && next_poss->inter_status < 2
                            && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_prev;},

                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_next == NULL 
                                || curr->fill_next->invalid()
                                || curr->fill_next->road_segment != curr->road_segment
                                || curr->fill_next->road_index != curr->road_index) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_next;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();
                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;

                            if (poss->inter_status < 2
                                    && next_poss->inter_status < 2
                            && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_next;},
                        
                        0);

                if (next_poss->inter_status > 1) {
                    length2 /= 2;
                }
                // if (length1 > length_thres && length2 > length_thres) {
                //     continue;
                // }

                // lc_list.push_back(std::make_tuple(length1, lc, next_lc, true));
                // lc_list.push_back(std::make_tuple(length2, next_lc, lc, false));

                lc_list.push_back(std::make_tuple(length1, lc, next_lc, true, j, poss->filled_lane_sample.size()));
                lc_list.push_back(std::make_tuple(length2, next_lc, lc, false, j, poss->filled_lane_sample.size()));
            }
        }
    }
    SORT(lc_list, [](const param &l, const param &r) {
                return std::get<0>(l) > std::get<0>(r);
            });
    for (auto &p : lc_list) {
        auto &lc = std::get<1>(p);
        auto next_lc = std::get<2>(p);
        int mode = 0;
        if (lc->road_index == next_lc->road_index) {
            continue;
        }
        auto &poss = lc->key_pose;
        bool next = std::get<3>(p);
        while (next_lc != NULL) {
            session->debug_pos(next_lc->pos);
            auto &next_poss = next_lc->key_pose;
            auto oppo_poss = poss->boundary.left_oppo_pos.src;
            auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
            // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid()
            //         && poss->boundary.same_road_status == 1;
            // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid()
            //         && next_poss->boundary.same_road_status == 1;
            // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
            // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;

            if (poss->inter_status < 2
                    && next_poss->inter_status < 2
                    && has_valid_oppo != has_valid_oppo_next) {
                break;
            }
            if (lc->road_segment != next_lc->road_segment) {
                break;
            }

            // next_lc->road_index = lc->road_index;  
            
            if(next_lc_list.size() == 0 || (next_lc_list.size() > 0 && lc != next_lc_list[0])){
                if(next_lc_list.size() > 0){
                    vote_road_index(session, next_lc_list, road_index_vec);
                }
                
                next_lc_list.clear();
                road_index_vec.clear();
                next_lc_list.push_back(lc);
                road_index_vec.push_back(lc->road_index);
            }
            
            next_lc_list.push_back(next_lc);
            road_index_vec.push_back(next_lc->road_index);
            next_lc = next ? next_lc->fill_next : next_lc->fill_prev;
        }
    }
    return fsdmap::SUCC;

}

int RoadModelProcFilterLane::smooth_side_status(RoadModelSessionData* session) {
    using param = std::tuple<double, LaneCenterFeature*, LaneCenterFeature*, bool>;
    std::vector<param> lc_list;
    std::vector<LaneCenterFeature*> context_list;
    for (auto &road_segment : session->road_segment_list) {
        for (int64_t i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int64_t j = 0; j < poss->filled_lane_sample.size(); ++j) {
                auto lc = poss->filled_lane_sample[j];
                session->debug_pos(lc->pos);
                // for (auto &next_lc_ptr : lc->context.all_next) {
                auto next_lc = lc->fill_next;
                if (next_lc == NULL || next_lc->invalid()) {
                    continue;
                }
                if (next_lc->road_segment != lc->road_segment) {
                    continue;
                }
                if (next_lc->side_status == lc->side_status) {
                    continue;
                }
                auto &next_poss = next_lc->key_pose;
                auto oppo_poss = poss->boundary.left_oppo_pos.src;
                auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid() 
                //     && poss->boundary.same_road_status == 1;
                // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid()
                //     && next_poss->boundary.same_road_status == 1;
                //
                // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                if (poss->inter_status < 2
                        && next_poss->inter_status < 2
                    && has_valid_oppo != has_valid_oppo_next) {
                    continue;
                }
                context_list.clear();
                double length1 = lc->get_context_list(1000, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_prev == NULL 
                                || curr->fill_prev->invalid()
                                || curr->fill_prev->road_segment != curr->road_segment
                                || curr->fill_prev->side_status != curr->side_status) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_prev;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();
                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                        if (poss->inter_status < 2
                                && next_poss->inter_status < 2
                                && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_prev;},

                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_next == NULL 
                                || curr->fill_next->invalid()
                                || curr->fill_next->road_segment != curr->road_segment
                                || curr->fill_next->side_status != curr->side_status) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_next;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;

                        if (poss->inter_status < 2
                                && next_poss->inter_status < 2
                                && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_next;},
                        
                        0);

                if (poss->inter_status > 1) {
                    length1 /= 2;
                }
                lc_list.push_back(std::make_tuple(length1, lc, next_lc, true));

                context_list.clear();
                double length2 = next_lc->get_context_list(1000, context_list,
                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_prev == NULL 
                                || curr->fill_prev->invalid()
                                || curr->fill_prev->road_segment != curr->road_segment
                                || curr->fill_prev->side_status != curr->side_status) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_prev;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                        if (poss->inter_status < 2
                                && next_poss->inter_status < 2
                                && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_prev;},

                        [](LaneCenterFeature* curr)->LaneCenterFeature* {
                        if ( curr->fill_next == NULL 
                                || curr->fill_next->invalid()
                                || curr->fill_next->road_segment != curr->road_segment
                                || curr->fill_next->side_status != curr->side_status) {
                        return NULL;
                        }
                        auto &next_lc = curr->fill_next;
                        auto &poss = curr->key_pose;
                        auto &next_poss = next_lc->key_pose;
                        auto oppo_poss = poss->boundary.left_oppo_pos.src;
                        auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
                        // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
                        // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;
                        if (poss->inter_status < 2
                                && next_poss->inter_status < 2
                                && has_valid_oppo != has_valid_oppo_next) {
                        return NULL;
                        }
                        return curr->fill_next;},
                        
                        0);

                if (next_poss->inter_status > 1) {
                    length2 /= 2;
                }
                lc_list.push_back(std::make_tuple(length2, next_lc, lc, false));
            }
        }
    }
    SORT(lc_list, [](const param &l, const param &r) {
                return std::get<0>(l) > std::get<0>(r);
            });
    for (auto &p : lc_list) {
        auto &lc = std::get<1>(p);
        auto next_lc = std::get<2>(p);
        int mode = 0;
        if (lc->side_status == next_lc->side_status) {
            continue;
        }
        auto &poss = lc->key_pose;
        bool next = std::get<3>(p);
        while (next_lc != NULL) {
            session->debug_pos(next_lc->pos);
            auto &next_poss = next_lc->key_pose;
            auto oppo_poss = poss->boundary.left_oppo_pos.src;
            auto oppo_poss_next = next_poss->boundary.left_oppo_pos.src;
            // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid()
            //         && poss->boundary.same_road_status == 1;
            // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid()
            //         && next_poss->boundary.same_road_status == 1;

            // bool has_valid_oppo = oppo_poss != NULL && !oppo_poss->invalid();
            // bool has_valid_oppo_next = oppo_poss_next != NULL && !oppo_poss_next->invalid();

                        bool has_valid_oppo = true;
                        bool has_valid_oppo_next = true;

            if (poss->inter_status < 2
                    && next_poss->inter_status < 2
                    && has_valid_oppo != has_valid_oppo_next) {
                break;
            }
            if (lc->road_segment != next_lc->road_segment) {
                break;
            }
            next_lc->side_status = lc->side_status;
            next_lc = next ? next_lc->fill_next : next_lc->fill_prev;
        }
    }
     
    return fsdmap::SUCC;

}

int RoadModelProcFilterLane::vote_road_index(RoadModelSessionData* session, std::vector<LaneCenterFeature*>& lc_list, std::vector<int>& road_index_map) {
    std::vector<int> changeIndices; // 用于存储发生变化的索引的vector
    std::map<int, int> frequencyMap;
    for (int number : road_index_map) {
        frequencyMap[number]++;
    }
    std::vector<std::pair<int, int>> vec(frequencyMap.begin(), frequencyMap.end());
    std::sort(vec.begin(), vec.end(), [](const std::pair<int, int>& p1, const std::pair<int, int>& p2) {
        return p1.second > p2.second; // 降序排序
    });

    int previousNumber = road_index_map[0];
    for(int i = 0 ; i < road_index_map.size(); i++){
        if(road_index_map[i] != previousNumber){
            changeIndices.push_back(i);
            previousNumber = road_index_map[i];
        }
    }

    int maybe_index = 0;
    bool zero_high_frequency = vec[0].first == 0 ? true : false;
    for (auto& p : vec) {
        if (p.first != 0) {
            maybe_index = p.first;
            break;
        }
    }
    auto iter = std::find_if(lc_list.begin(), lc_list.end(), [&](LaneCenterFeature* lc) {
        return lc->road_index == maybe_index;
    });

    auto most_lc = *iter;

    auto ite = std::find_if(lc_list.begin(), lc_list.end(), [&](LaneCenterFeature* lc) {
        return lc->road_index == 0;
    });

    auto new_lc = *ite;

    auto tmp_lc = lc_list[0];
    for(auto& lc : lc_list){
        if(lc->road_index == 0){
            if(judge_zero_road_index(session, lc) && tmp_lc->road_index != 0){     // 在其他road中，需要调整new_lc->road_index的值，真实非0
                lc->road_index = maybe_index;
            }
        }
        tmp_lc = lc;

    }
    

//     for (auto& lc : lc_list)
//     {
// #if 0
//         if (new_lc->road_index == lc->road_index) {
//             new_lc = lc;
//             continue;
//         }
//         if(lc->key_pose->filled_lane_sample.size() > 1 && lc == lc->key_pose->filled_lane_sample[lc->key_pose->filled_lane_sample.size() - 1]){
//             continue;
//         }
//         if(lc->key_pose->from_raw_link->lanenum_sum == new_lc->key_pose->from_raw_link->lanenum_sum){
//             auto& poss = new_lc->key_pose;
//             auto& rc = poss->boundary.curr;
//             if(!rc->in_scope(lc->pos, 0, 2)){
//                 lc->road_index = new_lc->road_index;
//                 // lc->side_status = rc->in_scope(lc->pos, 0, 2) ? 2 : 1;
//             }
            
//         }
// #endif
//         lc->road_index = new_lc->road_index;
//         lc->side_status = new_lc->side_status;
//     }
     
    return fsdmap::SUCC;
}

int RoadModelProcFilterLane::judge_zero_road_index(RoadModelSessionData* session, LaneCenterFeature* lc){

    auto& ll_pos = lc->key_pose->boundary.left_oppo_pos.src;
    auto& rt_pos = lc->key_pose->boundary.same_dir_pos;

    bool ret_1 = false, ret_2 = false;
    if(lc->road_index == 0){
        if(ll_pos != NULL && !ll_pos->invalid())
        {
            if(ll_pos->boundary.curr->in_scope(lc->pos, 0) ){
                ret_1 = true;
            }
        }  
        for(auto& rt : rt_pos)
        {
            if(rt.src != NULL && !rt.src->invalid()){
                if(rt.src->boundary.curr->in_scope(lc->pos, 0)){
                    ret_2 = true;
                    break;
                }
            }
        }      
    }
    if (ret_1 || ret_2) {   //road_index = 0有问题，需要处理
        return fsdmap::FAIL;
    }else{
        return fsdmap::SUCC;   // road_index = 0没有问题，可能是poss越界了，暂时不处理
    }

}

int RoadModelProcFilterLane::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_filter_lane_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("filter_lane");

    int64_t lc_id = 0;
    for (auto &road_segment : session->road_segment_list) {
        for (int i = 0; i < road_segment->pos_sample_list.size(); ++i) {
            auto &poss = road_segment->pos_sample_list[i];
            session->debug_pos(poss->pos);
            for (int j = 0; j < poss->filled_lane_sample.size(); ++j) {
                auto &lc = poss->filled_lane_sample[j];
                double score = lc->line_param.score;
                Eigen::Vector3d tar_pos = lc->pos + lc->dir * 0.5;
                auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                        "filter_lane");
                log->color = {100, 100, 0};
                log->add(poss->pos, 2);
                auto &ele = log->add(tar_pos);
                ele.label.label = lc->src_status;
                lc_id++;
                auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE_INDEX,
                        "filter_lane");
                // auto log_ptr1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, 
                //         "fill_lane_line");
                log_ptr->color = {255, 255, 255};
                // log_ptr1->color = {0, 255, 0};
                // if (lc->src_status == 2) {
                if (lc->fill_status == 2) {
                    log_ptr->color = {0, 255, 0};
                    // log_ptr1->color = {0, 100, 200};
                }
                if (lc->fill_status == 4) {
                    log_ptr->color = {0, 255, 255};
                    // log_ptr1->color = {0, 100, 200};
                }
                if (lc->road_index != 0) {
                    log_ptr->color[0] = 100;
                    // log_ptr1->color = {0, 100, 100};
                }

                // log_ptr->add(lc->pos, 1, lc->match_level.lane_type).label.intensity = score;
                log_ptr->add(lc->get_left(), 1, lc->match_level.lane_type).label.intensity = score;
                log_ptr->add(lc->get_right(), 1, lc->match_level.lane_type).label.intensity = score;
                // log_ptr1->add(lc->pos, 1, lc->match_level.lane_type).label.intensity = score;
                // log_ptr1->add(lc->get_right(), 1, lc->match_level.lane_type).label.intensity = score;
                bool has_next = false;
                if (lc->fill_next != NULL) {
                    log_ptr->add(lc->fill_next->pos, 1).label.intensity = score;
                    has_next = true;
                }
                if (lc->context.all_next.size() > 0) {
                    for (auto &next_ptr : lc->context.all_next) {
                        auto &next_lc = next_ptr.src;
                        if (next_lc == lc->fill_next) {
                            continue;
                        }
                        auto log1 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "");
                        log1->color = {255, 255, 0};
                        log1->add(lc->pos, 1).label.intensity = score;
                        log1->add(next_lc->pos, 1).label.intensity = score;
                    }
                    has_next = true;
                }
                if (!has_next) {
                    auto next_pos = alg::get_hori_pos(lc->pos, lc->dir, 0.5);
                    log_ptr->add(next_pos, 1).label.intensity = score;
                //     log_ptr1->add(lc->fill_prev->pos, 1);
                }
                if (lc->fill_status == 3) {
                    auto t_log = session->add_debug_log(utils::DisplayInfo::POINT,
                            "filter_lane");
                    t_log->color = {255, 0, 0};
                    t_log->add(lc->pos);
                }
            }
        }
    }

    session->save_debug_info("filter_lane");

    return fsdmap::SUCC;
}

}
}
