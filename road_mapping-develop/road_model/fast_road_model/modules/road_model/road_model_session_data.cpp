

#include "road_model_session_data.h"
#include "utils/algorithm_util.h"
#include "road_model_proc_init_data.h"
#include "road_model_proc_sample_line.h"
#include "road_model_proc_merge_feature.h"
#include "road_model_proc_bind_trail.h"
#include "road_model_proc_identify_road.h"
#include "road_model_proc_format_new_road.h"
#include "road_model_proc_build_boundary.h"
#include "road_model_proc_split_road.h"
#include "road_model_proc_build_intersection.h"
#include "road_model_proc_export_shp.h"

#include "road_model_proc_bind_trail2.h"
#include "road_model_proc_cal_coverage.h"
#include "road_model_proc_split_merge.h"
#include "road_model_proc_batch_process.h"


DEFINE_string(crosspoint_file_dir, "../fsd_mapbuild_out_cloud_bev_label/crosspoint.pcd", "crosspoint_file_dir");
DEFINE_string(debug_file_dir, "./data_debug_log", "debug_file_dir");
DEFINE_string(shp_file_dir, "./export_to_shp", "shp_file_dir");
DEFINE_string(origin_shp_file_dir, "./export_to_origin_shp", "origin_shp_file_dir");
DEFINE_string(mid_shp_file_dir, "./export_to_mid_shp", "export_to_mid_shp");
DECLARE_double(display_scale_rate);
// DECLARE_bool(format_lane_align_lane_local_enable);
DEFINE_bool(save_debug_use_global_coor, false, "save_debug_use_global_coor");
DEFINE_bool(use_image_debug, true, "use_image_debug");
namespace fsdmap {
namespace road_model {
   
int RoadModelSessionManager::declare_proc() {

    add_proc(new RoadModelProcInitData());  // 读取BEV感知、激光地面点云等数据
    add_proc(new RoadModelProcSampleLine());
    add_proc(new RoadModelProcMergeFeature());
    // cxf add
    add_proc(new RoadModelProcBindTrail2());
    add_proc(new RoadModelProcCalCoverage());
    
    add_proc(new RoadModelProcSplitMerge());
    // 
    add_proc(new RoadModelProcBuildBoundary());
    add_proc(new RoadModelProcBindTrail());
    //cxf add 
    // add_proc(new RoadModelProcIdentifyBreakPoint());
    add_proc(new RoadModelProcIdentifyRoad());
    add_proc(new RoadModelProcSplitRoad());
    add_proc(new RoadModelProcFormatNewRoad());
    //
    add_proc(new RoadModelProcBatchProcess());
    //
    add_proc(new RoadModelExportSHP());
    return fsdmap::SUCC;

}

void RoadModelSessionData::set_display_name(const char * log_name) {
    _log_map.clear();
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
