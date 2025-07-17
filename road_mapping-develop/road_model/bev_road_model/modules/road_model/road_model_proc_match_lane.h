

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcMatchLane :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcMatchLane() {};
    virtual ~RoadModelProcMatchLane() {};

    const char * name() {
        return "proc_match_lane";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int make_feature_tree(RoadModelSessionData* session);

    int gen_lane_sample(RoadModelSessionData* session);

    int make_raw_lane_sample(RoadModelSessionData* session);

    bool match_base_lane(RoadModelSessionData* session,
            LaneCenterFeature* lc, double base_radius);
    
    int make_lane_center_by_raw(RoadModelSessionData* session,
            LaneCenterFeature* lc);

    int match_lane_center(RoadModelSessionData* session,
         LaneCenterFeature *lc, DisDirPoint &left_lc, DisDirPoint &right_lc);

    DisDirPoint match_fls_by_lc(RoadModelSessionData* session,
            LaneCenterFeature *lc, double tar_dis, bool left);

    int64_t search_right_fls_by_fls(RoadModelSessionData* session,
         LaneLineSample *base_fls, Eigen::Vector3d &dir, bool base);

    int make_lane_sample(RoadModelSessionData* session,
        LaneCenterFeature* lc);

};

}
}
