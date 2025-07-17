

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcFormatLane :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcFormatLane() {};
    virtual ~RoadModelProcFormatLane() {};

    const char * name() {
        return "proc_format_lane";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int init_param(RoadModelSessionData* session);

    int fill_lane_line(RoadModelSessionData* session);

    int filter_lane_line(RoadModelSessionData* session);

    int clear_lane_attr(RoadModelSessionData* session);

    int split_oppo_lane(RoadModelSessionData* session);

    int update_lc_opt_dir(RoadModelSessionData* session);

    int make_valid_lc_tree(RoadModelSessionData* session);

    int clear_invalid_lc(SubRoadSegment* road_segment);

    int align_end_point(RoadModelSessionData* session);

    int vote_valid_side(RoadModelSessionData* session);

    int vote_lane_attr(RoadModelSessionData* session, LaneCenterLine* ll);

    LaneCenterLine* vote_valid_side_by_laneline(RoadModelSessionData* session,
        LaneCenterLine* ll, bool left);

    int align_fork_point(SubRoadSegment* road_segment, bool front);

    int is_conflict(LaneCenterLine* ll1, LaneCenterLine* ll2);

    int vote_direction(RoadModelSessionData* session,
            LaneCenterLine* ll, SubRoadSegment* srs);

    bool get_context_poss_break(RoadModelSessionData* session,
            RoadSegment* road_segment, bool front);

    int merge_sub_road_segment(RoadModelSessionData* session);

    int filter_lane_line_by_length(
        RoadModelSessionData* session, LaneCenterLine* ll, double min_lane_length);

};

}
}

