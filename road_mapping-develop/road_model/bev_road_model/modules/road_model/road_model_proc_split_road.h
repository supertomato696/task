

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcSplitRoad :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcSplitRoad() {};
    virtual ~RoadModelProcSplitRoad() {};

    const char * name() {
        return "proc_split_road";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int split_intersection(RoadModelSessionData* session);

    int make_road_segment(RoadModelSessionData* session);

    int match_road_and_intersection(RoadModelSessionData* session);

    int extend_intersection_road_segment(RoadModelSessionData* session);

    int extend_road_between_inter(RoadModelSessionData* session);

    int sort_by_length(RoadModelSessionData* session);

    RoadSegment* get_extend_road_segment(
            RoadModelSessionData* session, KeyPose* poss, bool next);

    Intersection* get_extend_intersection(
            RoadModelSessionData* session, KeyPose* poss, bool next);

    int extand_pos_sample(RoadModelSessionData* session,
            Intersection* inter, RoadSegment* road_segment, bool in);

    RoadSegment* gen_between_inter_road_segment(
            RoadModelSessionData* session,
            Intersection* in_inter, Intersection* out_inter);

    int search_near_pg_by_intersecion(RoadModelSessionData* session,
            Intersection* inter, std::vector<KeyPose*> &secs);

    void mark_inter_poss(
        RoadModelSessionData* session, Intersection* inter, KeyPose* poss, bool next);
};

}
}
