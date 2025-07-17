

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcBuildBoundary :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcBuildBoundary() {};
    virtual ~RoadModelProcBuildBoundary() {};

    const char * name() {
        return "build_boundary";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int search_boundary(RoadModelSessionData* session);

    int build_road_center(RoadModelSessionData* session);

    int build_yellow_boundary(RoadModelSessionData* session);

    int filter_road_center_by_oppo(RoadModelSessionData* session);

    int filter_road_center_by_same(RoadModelSessionData* session);

    int build_inter_boundary(RoadModelSessionData* session);

    int get_poss_cross_fls(RoadModelSessionData* session);

    int search_nearest_boundary(RoadModelSessionData* session, KeyPose* poss);

    int search_opposite_poss(RoadModelSessionData* session, KeyPose* poss);

    int search_opposite_trail(RoadModelSessionData* session, KeyPose* poss);

    int build_road_center_by_poss(RoadModelSessionData* session, KeyPose* poss);

    int mark_curr_road_center(RoadModelSessionData* session, KeyPose* poss);

    int make_road_center(RoadModelSessionData* session, KeyPose* poss, RoadCenterFeature* rc);

    int smooth_road_center_single(RoadModelSessionData* session, RoadSegment* road_segment);

    int smooth_road_center_width_oppo_poss(RoadModelSessionData* session, RoadSegment* road_segment);

    int complete_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment, bool left);

    bool match_lidar_by_line(RoadModelSessionData* session, 
        BoundaryLine* line);

    int switch_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment);

    int predict_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment);

    int spread_road_center(RoadModelSessionData* session, 
        RoadSegment* road_segment, int64_t index, bool prev);

    int judge_same_road_center(RoadModelSessionData* session, 
        KeyPose* poss);

    int search_nearest_curb_barriar(RoadModelSessionData* session,
              KeyPose* poss);

    int tracking_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment, bool left);

    int tracking_road_center_by_poss(
        RoadModelSessionData* session, RoadSegment* road_segment,
        int64_t index, bool left, bool next);

    int rebuild_road_center_by_fls(
        RoadModelSessionData* session, KeyPose* poss, BoundaryFeature* fls, bool left);

    int sync_road_center(
        RoadModelSessionData* session, RoadSegment* road_segment);

    BoundaryFeature* search_nearest_boundary_lidar(
        RoadModelSessionData* session, BoundaryFeature* bf, double v_scope, double h_scope);

    int valid_boundary_by_lidar(RoadModelSessionData* session);

    int match_yellow_boundary(RoadModelSessionData* session, KeyPose* poss);

    int match_yellow_boundary_by_rc(RoadModelSessionData* session,
            KeyPose* poss, RoadCenterFeature* rc);

    double get_middle_dis(RoadModelSessionData* session, KeyPose* poss, RoadCenterFeature* rc);

    int mark_yellow_boundary(RoadModelSessionData* session,
            KeyPose* poss);

    int search_nearest_fls(RoadModelSessionData* session,
            KeyPose* poss, std::vector<LaneLineSample*> &secs);

    int spread_yellow_boundary(RoadModelSessionData* session,
            RoadSegment* road_segment, int index, bool prev);
    
    int judge_road_center_with_lane_ratio(RoadModelSessionData* session, KeyPose* poss, const Eigen::Vector3d &pos);

    int set_road_index(RoadModelSessionData* session, KeyPose* poss);

    int add_same_dir_lane_num(RoadModelSessionData* session, KeyPose* poss, int& current_num, int& oppo_num);
};

}
}

