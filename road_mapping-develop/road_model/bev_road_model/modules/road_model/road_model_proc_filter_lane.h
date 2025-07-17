

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcFilterLane :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcFilterLane() {};
    virtual ~RoadModelProcFilterLane() {};

    const char * name() {
        return "proc_filter_lane";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int match_trail(RoadModelSessionData* session);

    int filter_lane_by_length(RoadModelSessionData* session);

    int smooth_filter_status(RoadModelSessionData* session);

    int remark_road_index(RoadModelSessionData* session);

    int smooth_road_index(RoadModelSessionData* session);

    int smooth_side_status(RoadModelSessionData* session);

    int vote_road_index(RoadModelSessionData* session, std::vector<LaneCenterFeature*>& lc_list, std::vector<int>& road_index_map);

    int mark_filter_status(RoadModelSessionData* session);
    
    int judge_zero_road_index(RoadModelSessionData* session, LaneCenterFeature* lc);

    KeyPose* search_nearest_trail(RoadModelSessionData* session, 
        LaneCenterFeature* lc);

};

}
}
