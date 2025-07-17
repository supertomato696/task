#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

class RoadModelProcCalCoverage :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcCalCoverage() {};
    virtual ~RoadModelProcCalCoverage() {};

    const char * name() {
        return "proc_cal_coverage";
    }

    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int cal_coverage(RoadModelSessionData* session);

    Eigen::Vector3d cal_road_true_dir(RoadModelSessionData* session, KeyPose* kp);
    
    void find_cross_link_points(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts, 
        std::vector<KeyPose*>& out_link_pts);

    KeyPose* find_in_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m);

    KeyPose* find_out_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, 
        std::vector<KeyPose*> in_link_pts, std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m);

    bool cal_road_width(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir,
        Eigen::Vector3d& left_bf, Eigen::Vector3d& right_bf, double& W_road);

    void sort_points_by_distance(std::vector<Eigen::Vector3d>& all_points, const Eigen::Vector3d& left_bf);

    void rm_duplicate_pts(std::vector<Eigen::Vector3d>& points, double dis_threshold);

    int cal_actual_lane_center_count(RoadModelSessionData* session, KeyPose* keypose, 
        Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
        std::vector<Eigen::Vector3d>& in_road_center_points);

    int cal_actual_lane_count(RoadModelSessionData* session, KeyPose* keypose, 
        Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
        std::vector<Eigen::Vector3d>& in_road_lane_points);

    void cal_horizontal_miss_positions(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir, int N_hope,
         int Nt, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
         std::vector<Eigen::Vector3d>& in_road_center_points);

    void cal_horizontal_miss_positions2(RoadModelSessionData* session, std::shared_ptr<RoadCrossPoints> &road_cross_pts, 
        KeyPose* origin_start, KeyPose* start_keypose, Eigen::Vector3d road_dir, int N_hope,
         int Nt, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
         std::vector<Eigen::Vector3d>& in_road_center_points);

    void cal_triangle_island_miss_positions(KeyPose* keypose, int N_hope, int Nt);

    int recovery_single_road_cross_point(std::shared_ptr<RoadCrossPoints>& road_cross_pts, bool first_iteration);

    int save_debug_info(RoadModelSessionData* session);
};

}
}