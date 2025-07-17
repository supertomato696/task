


#include "road_model_proc_match_lidar.h"

DECLARE_double(display_scope_buff);

DEFINE_bool(match_lidar_enable, false, "match_lidar_enable");
DEFINE_bool(match_lidar_debug_pos_enable, true, "match_lidar_debug_enable");
DEFINE_bool(match_lidar_save_data_enable, true, "match_lidar_save_data_enable");
DEFINE_double(match_lidar_search_poss_search_radius, 20, "match_lidar_search_poss_search_radius");
DEFINE_double(match_lidar_search_poss_search_scope, 4, "match_lidar_search_poss_search_scope");
DEFINE_double(match_lidar_match_boundary_scope, 2, "match_lidar_match_boundary_scope");
DEFINE_double(match_lidar_match_lane_scope, 1, "match_lidar_match_lane_scope");
DEFINE_double(match_lidar_match_ground_radius, 3, "match_lidar_match_ground_radius");

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcMatchLidar::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_match_lidar_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_match_lidar_debug_pos_enable;

    // CHECK_FATAL_PROC(bind_lidar_point(session), "bind_lidar_point");

    // CHECK_FATAL_PROC(match_ele_to_lidar(session), "match_ele_to_lidar");

    // CHECK_FATAL_PROC(match_ground(session), "match_ground");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}


int RoadModelProcMatchLidar::bind_lidar_point(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            search_nearest_lane_lidar(session, poss);
            search_nearest_boundary_lidar(session, poss);
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLidar::match_ele_to_lidar(RoadModelSessionData* session) {
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            match_lane_line_to_lidar(session, poss);
            match_boundary_to_lidar_by_side(session, poss, true, 0);
            match_boundary_to_lidar_by_side(session, poss, false, 0);
        }
    }
    // TODO 前后优化
    for (auto &road_segment : session->road_segment_list) {
        for (auto &poss : road_segment->pos_sample_list) {
            for (int i = 0; i < poss->filled_lane_sample.size(); ++i) {
                auto &lc = poss->filled_lane_sample[i];
                if (lc->invalid()) {
                    continue;
                }
                // lc->raw_pos = lc->pos;
                // lc->raw_dir = lc->dir;
                // lc->raw_width = lc->width;
                if (lc->left_match_lidar.src != NULL && lc->right_match_lidar.src != NULL) {
                    auto left_pos1 = lc->get_left();
                    auto right_pos1 = lc->get_right();
                    lc->init(lc->left_match_lidar.src->pos, lc->dir, 
                            lc->right_match_lidar.src->pos, lc->dir);
                    auto &left_pos = lc->get_left();
                    auto &right_pos = lc->get_right();
                    lc->left_opt_status = 1;
                    lc->right_opt_status = 1;
                    DLOG_POINT(lc->pos,
                            "match_lidar_lane1[raw_l={},{}, raw_r={},{}, tar_l={},{}, tar_r={},{}]",
                            left_pos1.x(), left_pos1.y(), left_pos.x(), left_pos.y(),
                            right_pos1.x(), right_pos1.y(), right_pos.x(), right_pos.y());
                } else if (lc->left_match_lidar.src != NULL) {
                    auto left_pos1 = lc->get_left();
                    auto right_pos1 = lc->get_right();
                    lc->init(lc->left_match_lidar.src->pos, lc->dir,
                            lc->get_right(), lc->dir);
                    auto &left_pos = lc->get_left();
                    auto &right_pos = lc->get_right();
                    lc->left_opt_status = 1;
                    DLOG_POINT(lc->pos,
                            "match_lidar_lane2[raw_l={},{}, raw_r={},{}, tar_l={},{}, tar_r={},{}]",
                            left_pos1.x(), left_pos1.y(), left_pos.x(), left_pos.y(),
                            right_pos1.x(), right_pos1.y(), right_pos.x(), right_pos.y());
                } else if (lc->right_match_lidar.src != NULL) {
                    auto left_pos1 = lc->get_left();
                    auto right_pos1 = lc->get_right();
                    lc->init(lc->get_left(), lc->dir, lc->right_match_lidar.src->pos, lc->dir);
                    auto &left_pos = lc->get_left();
                    auto &right_pos = lc->get_right();
                    lc->right_opt_status = 1;
                    DLOG_POINT(lc->pos,
                            "match_lidar_lane3[raw_l={},{}, raw_r={},{}, tar_l={},{}, tar_r={},{}]",
                            left_pos1.x(), left_pos1.y(), left_pos.x(), left_pos.y(),
                            right_pos1.x(), right_pos1.y(), right_pos.x(), right_pos.y());
                }
            }
            auto curr = poss->boundary.curr;
            curr->raw_left = curr->left;
            curr->raw_right = curr->right;
            if (curr->left_match_lidar.src != NULL) {
                curr->left = curr->left_match_lidar.src;
                curr->left->dir = curr->raw_left->dir;
                curr->left_opt_status = 1;
                DLOG_POINT(curr->left->pos,
                        "match_left_boundary[raw={},{}, tar={},{}",
                        curr->raw_left->pos.x(), curr->raw_left->pos.y(),
                        curr->left->pos.x(), curr->left->pos.y());
            }
            if (curr->right_match_lidar.src != NULL) {
                curr->right = curr->right_match_lidar.src;;
                curr->right->dir = curr->raw_right->dir;
                curr->right_opt_status = 1;
                DLOG_POINT(curr->right->pos,
                        "match_right_boundary[raw={},{}, tar={},{}",
                        curr->raw_right->pos.x(), curr->raw_right->pos.y(),
                        curr->right->pos.x(), curr->right->pos.y());
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcMatchLidar::match_lane_line_to_lidar(RoadModelSessionData* session,
        KeyPose* poss) {
    for (int i = 0; i < poss->filled_lane_sample.size(); ++i) {
        auto &lc = poss->filled_lane_sample[i];
        if (lc->invalid()) {
            continue;
        }
        match_lane_line_to_lidar_by_side(session, poss, lc, true, 0);
        match_lane_line_to_lidar_by_side(session, poss, lc, false, 0);
    }
    
    return fsdmap::SUCC;
}


bool RoadModelProcMatchLidar::match_lane_line_to_lidar_by_side(RoadModelSessionData* session,
        KeyPose* poss, LaneCenterFeature* lc, bool is_left, double offset) {
    double scope = FLAGS_match_lidar_match_lane_scope;
    auto &pos = is_left ? lc->get_left() : lc->get_right();
    double min_dis = DBL_MAX;
    BoundaryParamPair<LaneFeature> min_flsp = {NULL, 0, 0};
    for (int j = 0; j < poss->bind_lidar_lane_line.size(); ++j) {
        auto &flsp = poss->bind_lidar_lane_line[j];
        double lane_dis = alg::calc_vertical_dis(pos, poss->pos, poss->dir, true);
        double dis = fabs(lane_dis + offset - flsp.dis);
        if (dis > scope) {
            continue;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_flsp = flsp;
        }
    }
    if (min_flsp.src == NULL) {
        return false;
    }
    if (is_left) {
        lc->left_match_lidar = min_flsp;
    } else {
        lc->right_match_lidar = min_flsp;
    }
    return true;
}

bool RoadModelProcMatchLidar::match_boundary_to_lidar_by_side(RoadModelSessionData* session,
        KeyPose* poss, bool is_left, double offset) {
    double scope = FLAGS_match_lidar_match_boundary_scope;
    auto curr = poss->boundary.curr;
    auto rcf = is_left ? curr->left : curr->right;
    if (rcf == NULL) {
        return false;
    }
    double min_dis = DBL_MAX;
    BoundaryParamPair<BoundaryFeature> min_flsp = {NULL, 0, 0};
    for (int j = 0; j < poss->bind_lidar_boundary.size(); ++j) {
        auto &flsp = poss->bind_lidar_boundary[j];
        double lane_dis = alg::calc_vertical_dis(rcf->pos, poss->pos, poss->dir, true);
        double dis = fabs(lane_dis + offset - flsp.dis);
        if (dis > scope) {
            continue;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_flsp = flsp;
        }
    }
    if (min_flsp.src == NULL) {
        return false;
    }
    if (is_left) {
        curr->left_match_lidar = min_flsp;
    } else {
        curr->right_match_lidar = min_flsp;
    }
    return true;
}

int RoadModelProcMatchLidar::search_nearest_lane_lidar(RoadModelSessionData* session,
        KeyPose* poss) {
    float radius = FLAGS_match_lidar_search_poss_search_radius;
    float scope = FLAGS_match_lidar_search_poss_search_scope;
    Eigen::Vector3d pos = poss->pos;
    std::vector<LaneFeature*> search_rets;
    session->lane_line_lidar_tree.search(pos, radius, search_rets);
    for (auto& fls : search_rets) {
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->invalid()) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos, true);
        if (dis > radius) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, poss->pos, poss->dir);
        if (h_dis > scope) {
            continue;
        }
        BoundaryParamPair<LaneFeature> flsp;
        flsp.src = fls;
        flsp.dis = alg::calc_vertical_dis(fls->pos, poss->pos, poss->dir, true);
        poss->bind_lidar_lane_line.emplace_back(flsp);
    }
    SORT(poss->bind_lidar_lane_line,
            [] (const BoundaryParamPair<LaneFeature> &l, 
                const BoundaryParamPair<LaneFeature> &r)->bool {
                return l.dis < r.dis;
            });
    return fsdmap::SUCC;
}

int RoadModelProcMatchLidar::search_nearest_boundary_lidar(RoadModelSessionData* session,
        KeyPose* poss) {
    float radius = FLAGS_match_lidar_search_poss_search_radius;
    float scope = FLAGS_match_lidar_search_poss_search_scope;
    Eigen::Vector3d pos = poss->pos;
    std::vector<BoundaryFeature*> search_rets;
    session->boundary_lidar_tree.search(pos, radius, search_rets);
    for (auto& fls : search_rets) {
        session->debug_pos2(poss->pos, fls->pos);
        if (fls->invalid()) {
            continue;
        }
        double dis = alg::calc_dis(pos, fls->pos, true);
        if (dis > radius) {
            continue;
        }
        double h_dis = alg::calc_hori_dis(fls->pos, poss->pos, poss->dir);
        if (h_dis > scope) {
            continue;
        }
        BoundaryParamPair<BoundaryFeature> flsp;
        flsp.src = fls;
        flsp.dis = alg::calc_vertical_dis(fls->pos, poss->pos, poss->dir, true);
        poss->bind_lidar_boundary.push_back(flsp);
    }
    SORT(poss->bind_lidar_boundary,
            [] (const BoundaryParamPair<BoundaryFeature> &l, 
                const BoundaryParamPair<BoundaryFeature> &r)->bool {
                return l.dis < r.dis;
            });

    return fsdmap::SUCC;
}

int RoadModelProcMatchLidar::match_ground(RoadModelSessionData* session) {
    pcl::KdTreeFLANN<utils::CloudPoint> kdtree;
    if (session->opt_ground_pcd) {
        kdtree.setInputCloud(session->opt_ground_pcd);
        for (auto &road_segment : session->road_segment_list) {
            for (auto &poss : road_segment->pos_sample_list) {
                for (auto &lc : poss->filled_lane_sample) {
                    if (lc->invalid()) {
                        continue;
                    }
                    session->thread_pool->schedule([&, lc, session, this](
                                utils::ProcessBar *process_bar) {
                            auto left_pt = match_z_to_ground(session, kdtree, lc->get_left());
                            if (left_pt != NULL) {
                            lc->set_z(left_pt->z, true);
                            }
                            auto right_pt = match_z_to_ground(session, kdtree, lc->get_right());
                            if (right_pt != NULL) {
                            lc->set_z(right_pt->z, true);
                            }
                            });
                }
            }
        }
        for (auto &road_segment : session->road_segment_list) {
            for (auto &poss : road_segment->pos_sample_list) {
                auto &boundary = poss->boundary;
                for (auto &rc : boundary.road_center_list) {
                    session->thread_pool->schedule([&, rc, session, this](
                                utils::ProcessBar *process_bar) {
                            if (rc->left != NULL) {
                                auto left_pt = match_z_to_ground(session, kdtree, rc->left->pos);
                                if (left_pt != NULL) {
                                    rc->left->pos.z() = left_pt->z;
                                }
                            }
                            if (rc->right != NULL) {
                                auto right_pt = match_z_to_ground(session, kdtree, rc->right->pos);
                                if (right_pt != NULL) {
                                    rc->right->pos.z() = right_pt->z;
                                }
                            }
                            });
                }
            }
        }
    }
    session->thread_pool->wait(1, "match_to_ground");
    return fsdmap::SUCC;
}
                     

utils::CloudPoint* RoadModelProcMatchLidar::match_z_to_ground(RoadModelSessionData* session, 
        pcl::KdTreeFLANN<utils::CloudPoint> &kdtree, Eigen::Vector3d &pos) {
    double radius = FLAGS_match_lidar_match_ground_radius;
    std::vector<int> indexes;
    std::vector<float> radiuses;
    utils::CloudPoint center_pos;
    center_pos.x = pos.x();
    center_pos.y = pos.y();
    center_pos.z = pos.z();
    kdtree.radiusSearch(center_pos, radius, indexes, radiuses);
    if (indexes.size() == 0) {
        return NULL;
    }
    utils::CloudPoint* min_pt = NULL;
    double min_dis = DBL_MAX;
    for (int j = 0; j < indexes.size(); ++j) {
        auto &index = indexes[j];
        auto &pt = session->opt_ground_pcd->points[index];
        Eigen::Vector3d pt_pos = {pt.x, pt.y, pt.z};
        double dis = alg::calc_dis(pt_pos, pos);
        if (dis < min_dis) {
            min_pt = &pt;
            min_dis = dis;
        }
    }
    return min_pt;
}


int RoadModelProcMatchLidar::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_match_lidar_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("match_lidar");

    // for (auto &road_segment : session->road_segment_list) {
    //     for (auto &poss : road_segment->pos_sample_list) {
    //         for (auto &lc : poss->filled_lane_sample) {
    //             if (lc->invalid()) {
    //                 continue;
    //             }
    //             auto l_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "match_lane");
    //             l_log->color = {255, 255, 0};
    //             double dis = alg::calc_dis(lc->get_left(1), lc->get_left());
    //             if (dis > 2) {
    //                 int a = 1;
    //             }
    //             l_log->add(lc->get_left(1));
    //             l_log->add(lc->get_left());

    //             auto r_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "match_lane");
    //             r_log->color = {255, 0, 255};
    //             dis = alg::calc_dis(lc->get_right(1), lc->get_right());
    //             if (dis > 2) {
    //                 int a = 1;
    //             }
    //             r_log->add(lc->get_right(1));
    //             r_log->add(lc->get_right());
    //         }
    //     }
    // }

    session->save_debug_info("match_lidar");
    return fsdmap::SUCC;
}

}
}
