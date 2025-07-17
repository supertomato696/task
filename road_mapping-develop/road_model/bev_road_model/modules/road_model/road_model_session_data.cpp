

#include "road_model_session_data.h"
#include "road_model_proc_init_data.h"
#include "road_model_proc_sample_line.h"
#include "road_model_proc_match_lane.h"
#include "road_model_proc_match_line.h"
#include "road_model_proc_merge_feature.h"
#include "road_model_proc_gen_lane_line.h"
#include "road_model_proc_bind_trail.h"
#include "road_model_proc_fill_lane.h"
#include "road_model_proc_identify_road.h"
#include "road_model_proc_identify_attr.h"
#include "road_model_proc_format_lane.h"
#include "road_model_proc_format_new_road.h"
#include "road_model_proc_build_boundary.h"
#include "road_model_proc_format_boundary.h"
#include "road_model_proc_trans_interface.h"
#include "road_model_proc_split_road.h"
#include "road_model_proc_lidar_segmentation.h"
#include "road_model_proc_gen_key_point.h"
#include "road_model_proc_match_lidar.h"
#include "road_model_proc_filter_lane.h"
#include "road_model_proc_build_intersection.h"
#include "road_model_proc_bind_relation.h"
#include "road_model_proc_export_shp.h"

#include "road_model_proc_identify_break_point.h"
#include "road_model_proc_fix_object.h"
// #include "road_model_proc_bind_trail2.h"
// #include "road_model_proc_cal_coverage.h"


DEFINE_string(debug_file_dir, "./data_debug_log", "debug_file_dir");
DEFINE_string(shp_file_dir, "./export_to_shp", "shp_file_dir");
DEFINE_string(origin_shp_file_dir, "./export_to_origin_shp", "origin_shp_file_dir");
DEFINE_string(mid_shp_file_dir, "./export_to_mid_shp", "export_to_mid_shp");
DECLARE_double(display_scale_rate);
DECLARE_bool(format_lane_align_lane_local_enable);
DEFINE_bool(save_debug_use_global_coor, false, "save_debug_use_global_coor");
DEFINE_bool(use_image_debug, true, "use_image_debug");
namespace fsdmap {
namespace road_model {

int RoadModelSessionManager::declare_proc() {
    add_proc(new RoadModelProcInitData());  // 读取BEV感知、激光地面点云等数据
    
    int export_type = 0;
    add_proc(new RoadModelExportSHP(export_type));

    // add_proc(new RoadModelProcLidarSegmentation()); //没用到  ，接激光的pcd的 ， 预处理激光ground点云数据
    // add_proc(new RoadModelProcGenKeyPoint());  //没用到  ，接激光的pcd的
    add_proc(new RoadModelProcSampleLine());
    add_proc(new RoadModelProcMergeFeature());

    //cxf add
    // add_proc(new RoadModelProcBindTrail2());
    // add_proc(new RoadModelProcCalCoverage());
    

    // export_type = 1;
    // add_proc(new RoadModelExportSHP(export_type));

    add_proc(new RoadModelProcSplitRoad());
    add_proc(new RoadModelProcBuildBoundary());
    add_proc(new RoadModelProcMatchLane());
    add_proc(new RoadModelProcGenLaneLine());
    add_proc(new RoadModelProcMatchLine());
    add_proc(new RoadModelProcIdentifyAttr());
    add_proc(new RoadModelProcBindTrail());
    add_proc(new RoadModelProcFillLane());
    add_proc(new RoadModelProcFilterLane());

    //cxf add 
    add_proc(new RoadModelProcFixObject());
    // add_proc(new RoadModelProcIdentifyBreakPoint());

    add_proc(new RoadModelProcIdentifyRoad());
    // add_proc(new RoadModelProcMatchLidar());
    add_proc(new RoadModelProcFormatLane());
    add_proc(new RoadModelProcFormatBoundary());
    add_proc(new RoadModelProcFormatNewRoad());
    add_proc(new RoadModelProcBindRelation());
    add_proc(new RoadBuildIntersection());

    export_type = 2;
    add_proc(new RoadModelExportSHP(export_type));

    // TODO: qzc
    // add_proc(new RoadModelProcTransInterface());
    return fsdmap::SUCC;
}

void RoadModelSessionData::set_display_name(const char * log_name) {
    if (MAP_NOT_FIND(_log_map, log_name)) {
        _log_map[log_name] = std::vector<utils::DisplayInfo*>();
    }
    _curr_log = log_name;
}

void RoadModelSessionData::concate_display_name(const char * log_name) {
    if (MAP_NOT_FIND(_log_map, log_name)) {
        return;
    }
    VEC_PUSH_ALL(_log_map[_curr_log], _log_map[log_name]);
}

void RoadModelSessionData::save_debug_info(const char * log_name) {
    if (MAP_NOT_FIND(_log_map, log_name)) {
        return;
    }
    boost::filesystem::path base_debug_dir(FLAGS_debug_file_dir);
    if (!boost::filesystem::exists(base_debug_dir)) {
        boost::filesystem::create_directories(base_debug_dir);
    }
    std::string final_log_name = utils::fmt("0{}_{}", save_log_index++, log_name);
    auto &log_list = _log_map[log_name];
    auto box = _scope;
    if (FLAGS_save_debug_use_global_coor) {
        for (auto &log : log_list) {
            for (auto &pt : log->path) {
                pt.pos += data_processer->_center_pos;
            }
        }
        box.center_pos += data_processer->_center_pos;
    }

    // utm to lon lat
    // for (auto &log : log_list)
    // {
    //     for (auto &pt : log->path)
    //     {
    //         // pt.pos += data_processer->_center_pos;
    //         Eigen::Vector3d wgs;
    //         data_processer->local2wgs(pt.pos, wgs);
    //         double lng_gcj2;
    //         double lat_gcj2;
    //         alg::wgsTogcj2(wgs.x(), wgs.y(), lng_gcj2, lat_gcj2);
    //         pt.pos.x() = lng_gcj2;
    //         pt.pos.y() = lat_gcj2;
    //     }
    // }
    if(FLAGS_use_image_debug)
    {
      auto image_file = utils::fmt("{}.png", get_debug_dir(final_log_name));
      utils::save_display_image(image_file.c_str(), box, log_list);
    }


    auto pcd_file = utils::fmt("{}.pcd", get_debug_dir(final_log_name));
    utils::save_display_pcd(pcd_file.c_str(), box, log_list);
}


void RoadModelSessionData::init_scope(double min_x, double min_y, double max_x, double max_y, double z) {
    _scope.center_pos = {(min_x + max_x) / 2, (min_y + max_y) / 2, z};
    _scope.x_radius = (max_x - min_x) / 2;
    _scope.y_radius = (max_y - min_y) / 2;
    _scope.set_resolution(FLAGS_display_scale_rate);
}

// 根据start_lc的左右侧的车道线点构造新的车道中心点：
// 以start_lc左侧车道点为起点，end_lc左侧车道点与start_lc左侧车道点连线为方向向量，与break_pos为起点break_dir为
// 方向向量的交点作为新的左侧车道线点，右侧车道线点按同理构造，最后用这新生成的这两个左右侧车道线点构建新的车道中心点。
LaneCenterFeature* RoadModelSessionData::sample_new_lc(
        LaneCenterFeature* start_lc, LaneCenterFeature* end_lc,
        Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir, GenLaneLineParam* param) {
    // 左右分别进行交点计算，最后重组成新的lc
    auto &left_pos = start_lc->get_left();  // 车道中心线左侧车道线点
    auto &left_next_pos = end_lc->get_left(); // 车道中心线左侧车道线点
    DisDirPoint l_fls;
    DisDirPoint r_fls;
    l_fls.dir = alg::get_dir(left_next_pos, left_pos);
    if (!alg::get_cross_point(left_pos, l_fls.dir,
                break_pos, break_dir, l_fls.pos)) { // 计算左侧车道线点的方向向量与break_dir(pose点方向向量的垂向)的交点l_fls.pos，也就是新构造的左侧车道线点
        l_fls.pos = left_pos;
        l_fls.dir = start_lc->dir;
    }
    auto &right_pos = start_lc->get_right();
    auto &right_next_pos = end_lc->get_right();
    r_fls.dir = alg::get_dir(right_next_pos, right_pos);
    if (!alg::get_cross_point(right_pos, r_fls.dir,
                break_pos, break_dir, r_fls.pos)) {  // 计算右侧车道线点的方向向量与break_dir(pose点的垂向)的交点，也就是新构造的右侧车道线点
        r_fls.pos = right_pos;
        r_fls.dir = start_lc->dir;
    }

    auto lc_ptr_list = &this->lane_center_feature_ptr;
    auto fls_ptr_list = &this->lane_line_sample_ptr;
    bool need_lock = true;
    if (param != NULL) {
        lc_ptr_list = &param->lane_center_feature_ptr;
        fls_ptr_list = &param->lane_line_sample_ptr;
        need_lock = false;
    }
    auto new_lc = this->add_ptr(*lc_ptr_list, need_lock);
    auto new_left_fls = this->add_ptr(*fls_ptr_list, need_lock);
    auto new_right_fls = this->add_ptr(*fls_ptr_list, need_lock);
    new_lc->init(start_lc);
    new_lc->init(l_fls.pos, l_fls.dir, r_fls.pos, r_fls.dir);  // 根据新产生的左右车道线点，更新车道中心点
    new_lc->raw_from_lc = start_lc;
    new_lc->raw_to_lc = end_lc;
    new_lc->gen_status = 1;
    new_lc->left = new_left_fls.get();
    new_lc->right = new_right_fls.get();
    new_lc->filter_status == start_lc->filter_status;
    new_lc->dir_status == start_lc->dir_status;
    new_left_fls->init(start_lc->left);
    new_left_fls->pos = l_fls.pos; // 给左侧车道线点位置赋值
    new_left_fls->dir = l_fls.dir; // 给左侧车道线点方向赋值
    new_right_fls->init(start_lc->right);
    new_right_fls->pos = r_fls.pos;
    new_right_fls->dir = r_fls.dir;
    return new_lc.get();
}


LaneCenterFeature* RoadModelSessionData::sample_new_lc(LaneCenterFeature* start_lc,
        Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir, GenLaneLineParam* param) {
    // 左右分别进行交点计算，最后重组成新的lc
    DisDirPoint l_fls = this->sample_lane_center_side(
            start_lc, break_pos, break_dir, true, param);
    DisDirPoint r_fls = this->sample_lane_center_side(
            start_lc, break_pos, break_dir, false, param);

    auto lc_ptr_list = &this->lane_center_feature_ptr;
    auto fls_ptr_list = &this->lane_line_sample_ptr;
    bool need_lock = true;
    if (param != NULL) {
        lc_ptr_list = &param->lane_center_feature_ptr;
        fls_ptr_list = &param->lane_line_sample_ptr;
        need_lock = false;
    }
    auto new_lc = this->add_ptr(*lc_ptr_list, need_lock);
    auto new_left_fls = this->add_ptr(*fls_ptr_list, need_lock);
    auto new_right_fls = this->add_ptr(*fls_ptr_list, need_lock);
    new_lc->init(start_lc);
    new_lc->init(l_fls.pos, l_fls.dir, r_fls.pos, r_fls.dir);
    new_lc->raw_from_lc = start_lc;
    new_lc->raw_to_lc = start_lc->next;
    new_lc->gen_status = 1;
    new_lc->left = new_left_fls.get();
    new_lc->right = new_right_fls.get();
    new_lc->filter_status == start_lc->filter_status;
    new_lc->dir_status == start_lc->dir_status;
    new_left_fls->init(start_lc->left);
    new_left_fls->pos = l_fls.pos;
    new_left_fls->dir = l_fls.dir;
    new_right_fls->init(start_lc->right);
    new_right_fls->pos = r_fls.pos;
    new_right_fls->dir = r_fls.dir;
    return new_lc.get();
}

DisDirPoint RoadModelSessionData::sample_lane_center_side(LaneCenterFeature* start_lc,
        Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir, 
        bool left, GenLaneLineParam* param) {
    Eigen::Vector3d v_point = break_pos + break_dir * 10;
    DisDirPoint ret;
    ret.is_valid = false;
    std::vector<LaneCenterFeature*> context_list;
    start_lc->get_context_list(30.0, context_list, 
            [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_prev;},
            [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_next;}, 1);
    
    LaneCenterFeature* prev_lc = NULL;
    for (int i = 1; i < context_list.size(); ++i) {
        if (prev_lc == NULL) {
            prev_lc = context_list[i - 1];
        }
        auto &next = context_list[i];
        auto &curr_pos = left ? prev_lc->get_left() : prev_lc->get_right();
        auto &next_pos = left ? next->get_left() : next->get_right();
        if (alg::calc_dis(curr_pos, next_pos) < 0.1) {
            continue;
        }
        if (alg::get_cross_point_for_segment(curr_pos, next_pos,
                            break_pos, v_point, ret.pos, 1)) {
            ret.dir = alg::get_dir(next_pos, curr_pos);
            ret.is_valid = true;
            break;
        }
        prev_lc = next;
    }
    if (!ret.is_valid) {
        context_list.clear();
        start_lc->get_context_list(30, context_list, 
            [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_prev;},
            [](LaneCenterFeature* curr)->LaneCenterFeature* {return curr->fill_next;}, 2);
    
        LaneCenterFeature* next_lc = NULL;
        for (int i = context_list.size() - 2; i >= 0; --i) {
            if (next_lc == NULL) {
                next_lc = context_list[i + 1];
            }
            auto &prev = context_list[i];
            auto &curr_pos = left ? next_lc->get_left() : next_lc->get_right();
            auto &prev_pos = left ? prev->get_left() : prev->get_right();
            if (alg::calc_dis(curr_pos, prev_pos) < 0.1) {
                continue;
            }
            if (alg::get_cross_point_for_segment(curr_pos, prev_pos,
                        break_pos, v_point, ret.pos, 1)) {
                ret.dir = alg::get_dir(curr_pos, prev_pos);
                ret.is_valid = true;
                break;
            }
            next_lc = prev;
        }
    }
    if (!ret.is_valid) {
        auto &curr_pos = left ? start_lc->get_left() : start_lc->get_right();
        auto &dir = left ? start_lc->left_dir : start_lc->right_dir;
        if (alg::get_cross_point(curr_pos, dir, break_pos, break_dir, ret.pos)) {
            ret.dir = dir;
            ret.is_valid = true;
        }
    }
    if (!ret.is_valid) {
        auto &curr_pos = left ? start_lc->get_left() : start_lc->get_right();
        auto &dir = left ? start_lc->left_dir : start_lc->right_dir;
        ret.pos = curr_pos;
        ret.dir = dir;
        ret.is_valid = true;
    }
    return ret;
}

// LaneCenterFeature* RoadModelSessionData::sample_new_lc(LaneCenterFeature* lc, LaneCenterFeature* next, 
//         double dis, GenLaneLineParam* param, int mode) {
//     // 计算新宽度
//     DisDirPoint cross_point;
//     DisDirPoint left_cross_point;
//     DisDirPoint right_cross_point;
// 
//     auto &center_curr = *lc;
//     auto &center_next = *next;
//     double tmp_theta = alg::calc_theta(center_curr.dir, center_next.dir);
//     if (tmp_theta > 90) {
//         FLOG_POINT(lc->pos, "dir error[theta={}, d1={},{},{}, d2={},{},{}]", 
//                 tmp_theta, center_next.dir.x(), center_next.dir.y(), center_next.dir.z()
//                 , center_curr.dir.x(), center_curr.dir.y(), center_curr.dir.z());
//     }
//     left_cross_point.dir = get_lc_context_dir(lc, next, true, mode);
//     right_cross_point.dir = get_lc_context_dir(lc, next, false, mode);
//     cross_point.dir = alg::get_dir(center_next.pos, center_curr.pos);
//     Eigen::Vector3d cross_v_dir = alg::get_vertical_dir(cross_point.dir, true);
//     cross_point.pos = center_curr.pos + dis * cross_point.dir;
//     double left_width = center_curr.width / 2;
//     double right_width = center_curr.width / 2;
//     bool has_valid_left = false;
//     bool has_valid_right = false;
// 
//     if (alg::get_cross_point(center_curr.get_left(), left_cross_point.dir, 
//                 cross_point.pos, cross_v_dir, left_cross_point.pos, false)) {
//         left_width = alg::calc_dis(cross_point.pos, left_cross_point.pos, true);
//         has_valid_left = true;
//     }
//     if (alg::get_cross_point(center_curr.get_right(), right_cross_point.dir, 
//                 cross_point.pos, cross_v_dir, right_cross_point.pos, false)) {
//         right_width = alg::calc_dis(cross_point.pos, right_cross_point.pos, true);
//         has_valid_right = true;
//     }
//     double new_width = left_width + right_width;
//     // session->lock_run([&](void)->void {
//     auto lc_ptr_list = &this->lane_center_feature_ptr;
//     auto fls_ptr_list = &this->lane_line_sample_ptr;
//     bool need_lock = true;
//     if (param != NULL) {
//         lc_ptr_list = &param->lane_center_feature_ptr;
//         fls_ptr_list = &param->lane_line_sample_ptr;
//         need_lock = false;
//     }
//     auto new_lc = this->add_ptr(*lc_ptr_list, need_lock);
//     auto new_left_fls = this->add_ptr(*fls_ptr_list, need_lock);
//     auto new_right_fls = this->add_ptr(*fls_ptr_list, need_lock);
//     new_lc->raw_from_lc = lc;
//     new_lc->raw_to_lc = next;
//     new_lc->init(lc);
//     new_lc->gen_status = 1;
//     new_lc->pos = cross_point.pos;
//     new_lc->dir = cross_point.dir;
//     new_lc->width = new_width;
//     new_left_fls->init(lc->left);
//     new_lc->left = new_left_fls.get();
// 
//     new_left_fls->pos = left_cross_point.pos;
//     new_left_fls->dir = left_cross_point.dir;
//     if (!has_valid_left) {
//         new_left_fls->pos = new_lc->get_left();
//     }
//     new_right_fls->init(lc->right);
//     new_lc->right = new_right_fls.get();
//     new_right_fls->pos = right_cross_point.pos;
//     new_right_fls->dir = right_cross_point.dir;
//     if (!has_valid_right) {
//         new_right_fls->pos = new_lc->get_right();
//     }
//     new_lc->filter_status == lc->filter_status;
//     new_lc->dir_status == lc->dir_status;
//     return new_lc.get();
// }

double RoadModelSessionData::align_point(LaneCenterLine *ll, BreakPos &point, bool is_prev, double max_opt_dis) {
    double total_complete_dis = 0;
    if (ll->list.size() < 2) {
        return total_complete_dis;
    }
    // 需要判断lc的位置是不是需要删除多余的
    debug_pos(point.pos);
    auto base_break_pos = point;
    Eigen::Vector3d poss_v_pt1 = alg::get_vertical_pos(base_break_pos.pos, base_break_pos.dir, -20);
    Eigen::Vector3d poss_v_pt2 = alg::get_vertical_pos(base_break_pos.pos, base_break_pos.dir, 20);
    LaneCenterFeature* curr_lc = NULL;
    LaneCenterFeature* prev_lc = NULL;
    double offset = 0;
    Eigen::Vector3d cross_point = {0, 0, 0};
    Eigen::Vector3d base_cross_point = {0, 0, 0};
    bool has_cross = false;
    bool use_curr = false;
    double total_dis = 0;
    if (is_prev) {
        if (FLAGS_format_lane_align_lane_local_enable) {
            for (int i = 0; i < ll->list.size(); ++i) {
                auto &lc = ll->list[i];
                if (lc->break_pos.is_valid) {
                    base_break_pos.pos = lc->break_pos.pos;
                    base_break_pos.dir = lc->break_pos.dir;
                    break;
                }
            }
        }
        base_break_pos.dir.z() = 0;
        base_break_pos.dir.normalize();
        for (int i = 0; i < ll->list.size(); ++i) {
            auto &lc = ll->list[i];
            if (i == 0) {
                prev_lc = lc->fill_prev;
            }
            if (prev_lc == NULL) {
                prev_lc = lc;
                continue;
            }
            total_dis += alg::calc_dis(prev_lc->pos, lc->pos);
            if (total_dis > max_opt_dis) {
                break;
            }
            if (!alg::get_cross_point_for_segment(prev_lc->pos, lc->pos,
                        poss_v_pt1, poss_v_pt2, cross_point, 0)) {
                prev_lc = lc;
                if (i == 1) {
                    base_cross_point = cross_point;
                }
                continue;
            }
            offset = alg::calc_dis(prev_lc->pos, cross_point);
            for (int j = 0; j < i - 1; ++j) {
                ll->list[j]->filter_status = 11;
            }
            has_cross = true;
            curr_lc = lc;
            if (i == 0) {
                use_curr = true;
            }
            break;
        }
        if (!has_cross) {
            auto &first_lc = ll->list.front();
            prev_lc = first_lc;
            curr_lc = ll->list[1];
            alg::get_cross_point_by_point(prev_lc->pos, curr_lc->pos,
                    base_break_pos.pos, poss_v_pt2, cross_point, true, 3);
            offset = alg::calc_dis(prev_lc->pos, cross_point);
            if (alg::judge_front(first_lc->pos, base_break_pos.pos, base_break_pos.dir)) {
                offset = -offset;
            }
        }
    } else {
        if (FLAGS_format_lane_align_lane_local_enable) {
            for (int i = ll->list.size() - 1; i >= 0; --i) {
                auto &lc = ll->list[i];
                if (lc->break_pos.is_valid) {
                    base_break_pos.pos = lc->break_pos.pos;
                    base_break_pos.dir = lc->break_pos.dir;
                    break;
                }
            }
        }

        base_break_pos.dir.z() = 0;
        base_break_pos.dir.normalize();
        int end_index = ll->list.size() - 1;
        for (int i = end_index; i >= 0; --i) {
            auto &lc = ll->list[i];
            if (i == end_index) {
                prev_lc = lc->fill_next;
            }
            if (prev_lc == NULL) {
                prev_lc = lc;
                continue;
            }
            total_dis += alg::calc_dis(prev_lc->pos, lc->pos);
            if (total_dis > max_opt_dis) {
                break;
            }
            if (!alg::get_cross_point_for_segment(lc->pos, prev_lc->pos,
                        poss_v_pt1, poss_v_pt2, cross_point, 0)) {
                prev_lc = lc;
                if (i == end_index - 1) {
                    base_cross_point = cross_point;
                }
                continue;
            }
            offset = alg::calc_dis(lc->pos, cross_point);
            for (int j = i + 2; j < ll->list.size(); ++j) {
                ll->list[j]->filter_status = 12;
            }
            has_cross = true;
            curr_lc = lc;
            if (i == end_index) {
                use_curr = true;
            }
            break;
        }
        if (!has_cross) {
            auto &back_lc = ll->list.back();
            prev_lc = back_lc;
            curr_lc = ll->list[end_index - 1];
            alg::get_cross_point_by_point(prev_lc->pos, curr_lc->pos,
                    base_break_pos.pos, poss_v_pt2, cross_point, true, 3);
            double lc_gap = alg::calc_dis(prev_lc->pos, curr_lc->pos);
            offset = alg::calc_dis(prev_lc->pos, cross_point);
            if (!alg::judge_front(back_lc->pos, base_break_pos.pos, base_break_pos.dir)) {
                offset += lc_gap;
            } else {
                offset -= lc_gap;
            }
        }
    }
    debug_pos(prev_lc->pos);
    double opt_dis = alg::calc_hori_dis(prev_lc->pos, point.pos, point.dir);
    if (opt_dis > max_opt_dis) {
        FLOG_POINT2(prev_lc->pos, point.pos, "failed to alige point, break pos is to far[dis={}]",
                opt_dis);
        return total_complete_dis;
    }
    // if (near_center_pos(prev_lc->pos)) {
    //     DLOG_POINT(prev_lc->pos, "center is null");
    // }
    auto lc_start = prev_lc;
    auto lc_end = curr_lc;
    if (alg::judge_front(prev_lc->pos, curr_lc->pos, curr_lc->dir)) {
        lc_start = curr_lc;
        lc_end = prev_lc;
    }
    // auto new_lc = this->sample_new_lc(lc_start, lc_end, offset, NULL, 1);
    auto v_dir = alg::get_vertical_dir(base_break_pos.dir);
    auto new_lc = this->sample_new_lc(lc_start, base_break_pos.pos, v_dir, NULL);

    total_complete_dis = alg::calc_dis(new_lc->pos, prev_lc->pos);
    if (use_curr) {
        // DLOG_POINT2(curr_lc->pos, point.pos,
        //         "align_lane_line tar_pos[d_x={}, d_y={}, d_z={}]", 
        //         point.dir.x(), point.dir.y(), point.dir.z());
        // DLOG_POINT2(curr_lc->pos, new_lc->pos,
        //         "align_lane_line lc_pos[left_z={}, right_z={}]",
        //         curr_lc->left_fls->pos.z(), curr_lc->right_fls->pos.z());
        curr_lc->init(new_lc);
    } else {
        // DLOG_POINT2(prev_lc->pos, point.pos,
        //         "align_lane_line tar_pos[d_x={}, d_y={}, d_z={}]", 
        //         point.dir.x(), point.dir.y(), point.dir.z());
        // DLOG_POINT2(prev_lc->pos, new_lc->pos,
        //         "align_lane_line lc_pos[left_z={}, right_z={}]",
        //         prev_lc->left_fls->pos.z(), prev_lc->right_fls->pos.z());
        prev_lc->init(new_lc);
    }
    return total_complete_dis;
}
 
 
// Eigen::Vector3d RoadModelSessionData::get_lc_context_dir(LaneCenterFeature* lc, 
//         LaneCenterFeature* next, bool left, int mode) {
//     Eigen::Vector3d curr_pos;
//     Eigen::Vector3d next_pos;
//     curr_pos = left ? lc->get_left() : lc->get_right();
//     next_pos = left ? next->get_left() : next->get_right();
//     Eigen::Vector3d ret_dir;
//     ret_dir = alg::get_dir(next_pos, curr_pos);
// 
//     if (alg::calc_theta(ret_dir, lc->dir) >= 90) {
//         ret_dir = -ret_dir;
//     }
//     ret_dir.normalize();
//     return ret_dir;
// }

void RoadModelSessionData::display_pos(Eigen::Vector3d &start, Eigen::Vector3d &end, const char *file) {
    utils::DisplayInfo log;
    log.type = utils::DisplayInfo::LINE;
    log.add(start);
    log.add(end);
    utils::save_display_pcd(file, _scope, &log);
}

void RoadModelSessionData::display_pos(Eigen::Vector3d &pos, const char *file) {
    utils::DisplayInfo log;
    log.type = utils::DisplayInfo::POINT;
    log.add(pos);
    utils::save_display_pcd(file, _scope, &log);
}

}
}
