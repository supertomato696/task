

#pragma once

#include "road_model_session_data.h"


namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcMatchLidar :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcMatchLidar() {};
    virtual ~RoadModelProcMatchLidar() {};

    const char * name() {
        return "proc_match_lidar";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int bind_lidar_point(RoadModelSessionData* session);

    int match_ele_to_lidar(RoadModelSessionData* session);

    int match_ground(RoadModelSessionData* session);

    int search_nearest_lane_lidar(RoadModelSessionData* session, 
            KeyPose* poss);

    int search_nearest_boundary_lidar(RoadModelSessionData* session, 
            KeyPose* poss);

    int match_lane_line_to_lidar(RoadModelSessionData* session,
            KeyPose* poss);

    bool match_lane_line_to_lidar_by_side(RoadModelSessionData* session,
            KeyPose* poss, LaneCenterFeature* lc, bool is_left, double offset);

    bool match_boundary_to_lidar_by_side(RoadModelSessionData* session,
            KeyPose* poss, bool is_left, double offset);

    utils::CloudPoint* match_z_to_ground(RoadModelSessionData* session, 
        pcl::KdTreeFLANN<utils::CloudPoint> &kdtree, Eigen::Vector3d &pos);

    int save_debug_info(RoadModelSessionData* session);

};

}
}
