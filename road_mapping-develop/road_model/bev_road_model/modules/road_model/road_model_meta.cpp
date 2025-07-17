

#pragma once

#include "road_model/road_model_meta.h"


// DEFINE_double(debug_x, 0, "x for debug");
// DEFINE_double(debug_y, 0, "y for debug");
// DEFINE_double(debug_z, 0, "z for debug");
// DEFINE_double(debug_x2, 0, "x for debug");
// DEFINE_double(debug_y2, 0, "y for debug");
// DEFINE_double(debug_z2, 0, "z for debug");
// DEFINE_double(debug_dis, 10, "dis for debug");
// DEFINE_string(debug_frame_id, "", "frame id for debug");
DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

namespace fsdmap {

std::string BasePoint::wgs() {
    return wgs(",");
}

std::string BasePoint::wgs(const char * sep) {
    Eigen::Vector3d wgs = {0, 0 ,0};
    // utils::local2wgs(this->pos, wgs);
    // snprintf(wgs_text, sizeof(wgs_text), "%f%s%f%s%.2f", wgs.x(), sep, wgs.y(), sep, wgs.z());
    return "";
}

Eigen::Vector3d BasePoint::wgs_pos() {
    Eigen::Vector3d wgs;
    // utils::trans_local2wgs(pos, wgs);
    return wgs;
}

std::string BindKeyPosePoint::save(utils::DisplayScope &box, const char *file) {
    utils::DisplayInfo log;
    log.type = utils::DisplayInfo::POINT;
    log.add(pos);
    utils::save_display_pcd(file, box, &log);
}

std::string BasePoint::local() {
    return utils::fmt("{:.2f},{:.2f},{:.2f}", pos.x(), pos.y(), pos.z());
}

std::string BasePoint::local(utils::DisplayScope &box, bool need_scale) {
    Eigen::Vector3d tar = pos;
    utils::trans_display_local_pos(box, tar, need_scale);
    return utils::fmt("{:.2f},{:.2f},{:.2f}", tar.x(), tar.y(), tar.z());
}

bool MatchLevel::is_same(MatchLevel &other) {
    if (lane_type != other.lane_type) {
        return false;
    }
    if (lane_num != other.lane_num) {
        return false;
    }
    return true;
}

bool MatchLevel::is_same_num(MatchLevel &other) {
    if (lane_num != other.lane_num) {
        return false;
    }
    return true;
}

bool MatchLevel::is_same_type(MatchLevel &other) {
    if (lane_type != other.lane_type) {
        return false;
    }
    return true;
}

Eigen::Vector3d& KeyPose::get_base_line_dir(int mode) {
    // if (mode == 1) {
        for (auto &lc : filled_lane_sample) {
            if (lc->invalid()) {
                continue;
            }
            return lc->dir;
        }
    // } else {
    //     for (auto &lc : valid_lane_sample) {
    //         if (lc->invalid()) {
    //             continue;
    //         }
    //         return lc->dir;
    //     }
    // }
    return dir;
}


void LaneCenterFeature::init(LaneLineSample* l, LaneLineSample* r) { 
    // width 淡出用来度量，不作为坐标计算因素
    this->left = l;
    this->right = r;
    Eigen::Vector3d left_dir = left->dir;
    Eigen::Vector3d right_dir = right->dir;
    if (alg::calc_theta(left_dir, right_dir) > 90) {
        right_dir = -right_dir;
    }
    this->left_dir = left_dir;
    this->right_dir = right_dir;
    this->dir = left_dir + right_dir;
    this->dir.normalize();
    Eigen::Vector3d v_dir = alg::get_vertical_dir(this->dir);
    this->left_pos = l->pos;
    // this->right_pos = r->pos;
    alg::get_cross_point(l->pos, v_dir, r->pos, r->dir, this->right_pos);
    this->pos = (this->left_pos + this->right_pos) / 2;
    
    this->frame_id = left->src->frame_id;
    this->theta = alg::calc_theta(left_dir, right_dir);
    this->width = 2 * alg::calc_dis(this->left_pos, this->pos);
}

void LaneCenterFeature::init(Eigen::Vector3d &l_pos, Eigen::Vector3d &l_dir,
        Eigen::Vector3d &r_pos, Eigen::Vector3d &r_dir) {
    this->left_pos = l_pos;
    this->right_pos = r_pos;
    this->pos = (this->left_pos + this->right_pos) / 2; // 根据左右侧车道线重新计算车道中心点
    Eigen::Vector3d right_dir = r_dir;
    if (alg::calc_theta(l_dir, right_dir) > 90) {
        right_dir = -right_dir;
    }
    this->dir = l_dir + right_dir;
    this->dir.normalize();
    this->left_dir = l_dir;
    this->right_dir = right_dir;
    this->theta = alg::calc_theta(l_dir, right_dir);
    this->width = 2 * alg::calc_vertical_dis(this->left_pos, this->pos, this->dir);
}

// void LaneCenterFeature::init(Eigen::Vector3d &l, Eigen::Vector3d &r) {
//     this->left_pos = l;
//     this->right_pos = r;
//     this->pos = (l + r) / 2;
//     this->dir = alg::get_vertical_dir(r, l);
//     this->width = alg::calc_dis(l, r);
// }

bool LaneCenterFeature::is_filled() {
    if (fill_status >= 1) {
        return true;
    }
    // for (auto &pair : match_ls) {
    //     // 如果自己没选中，但是关联feature为主，则视为选中
    //     if (pair.first->fill_status >= 1) {
    //         return true;
    //     }
    // }
    return false;
}

bool LaneCenterFeature::has_same_edge(LaneCenterFeature *lc) {
    if (lc == NULL) {
        return false;
    }
    if (left->line == lc->right->line || right->line == lc->left->line) {
        return true;
    }
    return false;
}

bool LaneCenterFeature::is_same_base(LaneCenterFeature *lc, int mode) {
    if (lc == NULL) {
        return false;
    }
    if (left->group_line != NULL && right->group_line != NULL
            && left->group_line == lc->left->group_line 
            && right->group_line == lc->right->group_line) {
        return true;
    }
    if (left->line_id != "" && right->line_id != ""
            && left->line_id == lc->left->line_id
            && right->line_id == lc->right->line_id) {
        return true;
    }
    return false;
}

// bool LaneCenterFeature::on_same_line(double scope, LaneCenterFeature *lc, bool next,
//         UMAP<LaneCenterFeature*, std::vector<LaneCenterFeature*>> &line_map) {
//     if (lc == this) {
//         return true;
//     }
//     auto &next_vec = next ? this->context.all_next : this->context.all_prev;
//     for (auto &next_lc_ptr : next_vec) {
//         auto &next_lc = next_lc_ptr.src;
//         double dis = alg::calc_dis(next_lc->pos, this->pos);
//         if (dis > scope) {
//             continue;
//         }
//         if (next_lc->on_same_line(scope - dis, lc, next, line_map)) {
//             line_map[this].push_back(next_lc);
//             // return true;
//         }
//     }
//     if (line_map.size() > 0) {
//         return true;
//     }
//     return false;
// }

Eigen::Vector3d& LaneCenterFeature::get_left() {
    // if (mode == 0) {
    //     return this->left_pos;
    //     // Eigen::Vector3d v_dir = alg::get_vertical_dir(dir, true);
    //     // left_pos = pos - (width / 2) * v_dir;
    // } else if (mode == 1) {
    //     return this->left_pos;
    //     // Eigen::Vector3d v_dir = alg::get_vertical_dir(raw_dir, true);
    //     // left_pos = raw_pos - (raw_width / 2) * v_dir;
    // }
    return left_pos;

}

Eigen::Vector3d& LaneCenterFeature::get_right() {
    // if (mode == 0) {
    //     return this->right_pos;
    //     // Eigen::Vector3d v_dir = alg::get_vertical_dir(dir, true);
    //     // right_pos = pos + (width / 2) * v_dir;
    // } else if (mode == 1) {
    //     return this->right_pos;
    //     // Eigen::Vector3d v_dir = alg::get_vertical_dir(raw_dir, true);
    //     // right_pos = raw_pos + (raw_width / 2) * v_dir;
    // }
    return right_pos;
}

void LaneCenterFeature::set_z(double z, bool left) {
    if (left) {
        left_pos.z() = z;
    } else {
        right_pos.z() = z;
    }
}


// double LaneCenterFeature::get_fill_context_list(double scope, std::vector<LaneCenterFeature*> &ret, int mode) {
//     return LineNodeBase::get_context_list(scope, ret,
//             [](LaneCenterFeature* lc)->LaneCenterFeature* {return lc->fill_prev;},
//             [](LaneCenterFeature* lc)->LaneCenterFeature* {return lc->fill_next;},
//             mode);
// }

// 判断目标点tar_pos是否在车道的有效范围内
bool LaneCenterFeature::in_scope(Eigen::Vector3d &tar_pos, double buff, int mode) {
    // 计算目标点tar_pos与车道左边缘的垂直距离
    double left_dis = alg::calc_vertical_dis(tar_pos, left_pos, this->dir, true);
    // 如果目标点离左边缘的距离小于-缓冲区buff，说明目标点不在车道内
    if (left_dis + buff < 0) {
        return false;  // 不在范围内
    }
    
    // 计算目标点tar_pos与车道右边缘的垂直距离
    double right_dis = alg::calc_vertical_dis(tar_pos, right_pos, this->dir, true);
    // 如果目标点离右边缘的距离大于+缓冲区buff，说明目标点不在车道内
    if (right_dis - buff > 0) {
        return false;  // 不在范围内
    }
    
    return true;  // 目标点在车道范围内
}

bool LaneCenterFeature::in_scope_stop_line(Eigen::Vector3d &stop_line_pos, const float& length, double buff, int mode) {
    double vertical_dis = alg::calc_vertical_dis(stop_line_pos, this->pos, this->dir, true); //停止线中线点，中心线pos
    // x 方向的距离
    if (std::abs(vertical_dis) > length/2 + buff ) {
        return false;
    }
    //TODO:cxf  添加纵向距离的约束逻辑
    return true;
}



double LaneCenterLine::get_reline_length() {
    double total_dis = 0;
    for (int i = 0; i < list.size() - 1; ++i) {
        auto &lc = list[i];
        auto &lc_next = list[i + 1];
        if (lc->invalid()) {
            continue;
        }
        if (lc_next->invalid()) {
            continue;
        }
        // if (lc->reline_status > 0) {
        total_dis += alg::calc_dis(lc->pos, lc_next->pos);
        // }
    }
    return total_dis;
}

bool RoadCenterFeature::in_scope(Eigen::Vector3d &tar_pos, double buff, int mode) {
    if (left != NULL) {
        double left_dis = alg::calc_vertical_dis(tar_pos, left->pos, key_pose->dir, true);
        if (left_dis + buff < 0) {  // 表示tar_pos在左侧围栏点的左侧，不合理
            return false;
        }
    }
    if (right != NULL) {
        double right_dis = alg::calc_vertical_dis(tar_pos, right->pos, key_pose->dir, true);
        if (right_dis - buff > 0) { // 表示tar_pos在右侧围栏点的右侧，不合理
            return false;
        }
    }
    auto &fls = yellow_boundary.src;
    if (mode != 0) {
        if (fls != NULL) {
            double middle_dis = alg::calc_vertical_dis(tar_pos, fls->pos, key_pose->dir, true);
            buff = mode == 1 ? buff : -buff;
            return (middle_dis + buff < 0) ^ (mode == 1);
        }
        return mode == 1;
    }
    return true;
}

RoadCenterFeature* RoadBoundary::get_road_center(int rc_index) {
    RoadCenterFeature* ret = NULL;
    for (auto &rc : road_center_list) {
        if (rc->index == rc_index) {
            ret = rc;
            break;
        }
    }
    return ret;
}

// 选出左右边界包括了pos的道路中心点
RoadCenterFeature* RoadBoundary::get_road_center(Eigen::Vector3d &pos, double buff) {
    RoadCenterFeature* ret = NULL;
    for (auto &rc : road_center_list) {
        if (rc->in_scope(pos, buff)) {
            ret = rc;
            break;
        }
    }
    return ret;
}

void log_pcd(std::vector<Eigen::Vector3d> &data_list, const char *name) {
    utils::CloudPtr cloud(new pcl::PointCloud<utils::CloudPoint>);
    cloud->points.resize(data_list.size());
    cv::Scalar color = {255, 255, 255};
    for (int i = 0; i < data_list.size(); ++i) {
        auto &pt = data_list[i];
        auto &tar = cloud->points[i];
        tar.x = pt.x();
        tar.y = pt.y();
        tar.z = pt.z();
        tar.rgb = utils::rgb_2_float(color);
    }
    pcl::io::savePCDFileBinary(name, *cloud);
}

// bool RoadBoundary::has_dis(bool get_left, int mode) {
    // if (get_left) {
    //     if (left.src != NULL) {
    //         return true;
    //     }
    //     if (mode != 1) {
    //         if (left_yellow.src != NULL) {
    //             return true;
    //         }
    //     }
    //     if (left_oppo_pos.src != NULL) {
    //         return true;
    //     }
    // } else {
    //     if (right.src != NULL) {
    //         return true;
    //     }
    //     if (right_oppo_pos.src != NULL) {
    //         return true;
    //     }
    // }
    // return false;
// }

// double RoadBoundary::get_dis(bool get_left, int mode) {
//     double ret = 0;
    // double yellow_gap = 0.2;
    // do {
    //     if (get_left) {
    //         if (left.src != NULL) {
    //             ret = left.dis;
    //             // if (mode == 2) {
    //             //     if (left->fg->fp->group_type == GROUP_TYPE_LANE_LINE) {
    //             //         ret += yellow_gap;
    //             //     }
    //             // }
    //             break;
    //         }
    //         if (mode != 1) {
    //             if (left_yellow.src != NULL) {
    //                 ret = left_yellow.dis;
    //                 ret += yellow_gap;
    //                 break;
    //             }
    //         }
    //         if (left_oppo_pos.src != NULL) {
    //             ret = left_oppo_pos.dis;
    //         }
    //     } else {
    //         if (right.src != NULL) {
    //             ret = right.dis;
    //             // if (right->fg->fp->group_type == GROUP_TYPE_LANE_LINE) {
    //             //     ret += yellow_gap;
    //             // }
    //             break;
    //         }
    //         if (right_oppo_pos.src != NULL) {
    //             ret = right_oppo_pos.dis;
    //         }
    //     }
    // } while (0);
    // return ret;
// }
}
