

#include "road_model_proc_format_boundary.h"

DEFINE_bool(format_boundary_enable, true, "format_boundary_enable");
DEFINE_bool(format_boundary_debug_pos_enable, true, "format_boundary_debug_pos_enable");
DEFINE_bool(format_boundary_save_data_enable, true, "format_boundary_save_data_enable");
DEFINE_double(format_boundary_max_null_length, 10, "format_boundary_max_null_length");
DEFINE_double(format_boundary_oppo_rate_threshold, 0.6, "format_lane_oppo_rate_threshold");
DEFINE_double(format_boundary_theta_threshold, 45, "format_boundary_theta_threshold");
DEFINE_double(format_boundary_min_dis_threshold, 0, "format_boundary_min_dis_threshold");
DECLARE_double(format_lane_calc_curvature_radius);

namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcFormatBoundary::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_format_boundary_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_format_boundary_debug_pos_enable;
    
    // CHECK_FATAL_PROC(rebuild_boundary(session), "make_boundary_tree");

    CHECK_FATAL_PROC(extract_boundary(session), "extract_boundary");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcFormatBoundary::extract_boundary(RoadModelSessionData* session) {
    double radius = FLAGS_format_lane_calc_curvature_radius;
    // 对新增道路部分按照轨迹进行连线
    // std::vector<int> valid_road_index = {-1, 0};
    for (auto srs : session->sub_road_segment_list) {
    // for (auto road_segment : session->road_segment_list) {
        if (srs->link_direction == 1) {
            reline_boundary_in_road(session, srs, srs->road_index, false);
            if (srs->need_left_boundary) {
                reline_boundary_in_road(session, srs, srs->road_index, true);
            }
        } else if (srs->link_direction == 2) {
            reline_boundary_in_road(session, srs, srs->road_index, true);
        }
    }
    for (int i = 0; i < session->new_boundary_line_list.size(); ++i) {
        auto &bl = session->new_boundary_line_list[i];
        BoundaryFeature* prev_bp = NULL;
        for (int j = 0; j < bl->list.size(); ++j) {
            auto &curr = bl->list[j];
            if (curr->invalid()) {
                continue;
            }
            if (prev_bp == NULL) {
                prev_bp = curr;
                continue;
            }
            prev_bp->next = curr;
            curr->prev = prev_bp;
            prev_bp = curr;
        }
    }
    std::vector<BoundaryFeature*> context_list;
    std::vector<Eigen::Vector3d> center_pos_list;
    for (int i = 0; i < session->new_boundary_line_list.size(); ++i) {
        auto &bl = session->new_boundary_line_list[i];
        for (int j = 0; j < bl->list.size(); ++j) {
            auto &bp = bl->list[j];
            session->debug_pos(bp->pos);
            context_list.clear();
            center_pos_list.clear();
            bp->get_context_list(radius, context_list, 0);
            for (auto &tmp_bp : context_list) {
                center_pos_list.push_back(tmp_bp->pos);
            }
            double c_x = 0;
            double c_y = 0;
            double c_r = 1000;
            if (alg::fit_circle(center_pos_list, c_x, c_y, c_r)) {
                bp->curvature = c_r;
            } else {
                bp->curvature = 1000;
            }
        }
    }

    return fsdmap::SUCC;
}

int RoadModelProcFormatBoundary::reline_boundary_in_road(RoadModelSessionData* session,
        SubRoadSegment* srs, int rc_index, bool left) {
    double max_null_length = FLAGS_format_boundary_max_null_length;
    double min_dis_threshold = FLAGS_format_boundary_min_dis_threshold;
    double theta_threshold = FLAGS_format_boundary_theta_threshold;
    double rate_threshold = FLAGS_format_boundary_oppo_rate_threshold;
    int oppo_num = 0;
    int valid_num = 0;
    auto boundary_line = session->add_ptr(session->boundary_line_ptr);
    BoundaryFeature* prev_fls = NULL;
    auto &road_segment = srs->src_road_segment;
    for (int i = srs->start_index; i <= srs->end_index; ++i) {
        auto &poss = road_segment->pos_sample_list[i];
        if (poss->invalid()) {
            continue;
        }
        session->debug_pos(poss->pos);
        auto &boundary = poss->boundary;
        RoadCenterFeature* curr = boundary.get_road_center(rc_index);
        BoundaryFeature* fls = NULL;
        bool has_break = false;
        bool valid_fls = false;
        do {
            if (i == srs->end_index) {
                has_break = true;
            }
            if (curr == NULL || curr->invalid()) {
                has_break = true;
                break;
            }
            fls = left ? curr->left : curr->right;
            if (fls == NULL || fls->src_status == 6) {
                // TODO 虚拟边界
                fls = NULL;
                break;
            }
            session->debug_pos(fls->pos);
            ++valid_num;
            if (left && curr->yellow_boundary.src != NULL) {
                ++oppo_num;
            }

            if (prev_fls == NULL) {
                break;
            }
            double dis = alg::calc_dis(prev_fls->pos, fls->pos);
            if (dis > max_null_length) {
                has_break = true;
                break;
            }
            if (dis < min_dis_threshold) {
                break;
            }
            fls->dir = alg::get_dir(fls->pos, prev_fls->pos);
            double theta = alg::calc_theta(prev_fls->dir, fls->dir);
            if (theta > theta_threshold) {
                has_break = true;
                break;
            }
            int prev_sub_type , fls_sub_type;
            if (prev_fls->attr_feature != nullptr){
                prev_sub_type = prev_fls->attr_feature->sub_type;
            }
            else{
                prev_sub_type = prev_fls->sub_type;
            }
            if (fls->attr_feature != nullptr){
                fls_sub_type = fls->attr_feature->sub_type;
            }
            else{
                fls_sub_type = fls->sub_type;
            }
            if (prev_sub_type != fls_sub_type) {
                has_break = true;
                break;
            }
            valid_fls = true;
            
        } while (0);
        if (has_break) {
            if (fls != NULL && valid_fls) {
                boundary_line->list.push_back(fls);
            }
            if (boundary_line->list.size() >= 2) {
                boundary_line->road_segment = road_segment;
                boundary_line->road_index = rc_index;
                boundary_line->road_type = 1;
                if (left && valid_num > 0) {
                    float rate = (float) oppo_num / valid_num;
                    if (rate > rate_threshold) {
                        boundary_line->road_type = 2;
                    }
                }
                do {
                    session->new_boundary_line_list.push_back(boundary_line.get());
                    srs->new_boundary_line_list.push_back(boundary_line.get());
                    boundary_line->side_status = left ? 2 : 1;
                } while (0);
            }
            boundary_line = session->add_ptr(session->boundary_line_ptr);
            prev_fls = NULL;
            oppo_num = 0;
            valid_num = 0;
            continue;
        }
        // TODO 边界类型
        if (fls == NULL) {
            continue;
        }
        // if (MAP_NOT_FIND(poss->lc_num_map, rc_index)) {
        //     continue;
        // }
                
        boundary_line->list.push_back(fls);
        prev_fls = fls;
        
    }
    // if (left && valid_num > 0) {
    //     float rate = (float) oppo_num / valid_num;
    //     if (rate > rate_threshold) {
    //         road_segment->road_type[rc_index] = 2;
    //     }
    // }
    return fsdmap::SUCC;
}

int RoadModelProcFormatBoundary::save_debug_info(RoadModelSessionData* session) {
	if (!FLAGS_format_boundary_save_data_enable) {
		return fsdmap::SUCC;
	}
    session->set_display_name("format_boundary");

    // for (auto &road_segment : session->road_segment_list) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
    //     log->color = {223, 130, 154};
    //     for (auto &poss : road_segment->pos_sample_list) {
    //         log->add(poss->pos);
    //     }
    // }
    for (auto srs : session->sub_road_segment_list) {
        for (int i = 0; i < srs->new_boundary_line_list.size(); ++i) {
            auto &bs = srs->new_boundary_line_list[i];
            auto blog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "road_boundary");
            blog->color = {81, 89, 240};
            if (srs->road_index != 0) {
                blog->color = {0, 89, 240};
            }
            for (auto &fls : bs->list) {
                auto &ele = blog->add(fls->pos, 1, i);
                ele.label.opt_label = bs->side_status;
            }
            { 
                auto clog = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "road_boundary");
                clog->add(bs->list[0]->pos);
                clog->add(srs->start_break_pos.pos);
            }

        }
    }

    session->save_debug_info("format_boundary");
	return fsdmap::SUCC;
}


}
}
/* vim: set expandtab ts=4 sw=4 sts=4 tw=100: */
