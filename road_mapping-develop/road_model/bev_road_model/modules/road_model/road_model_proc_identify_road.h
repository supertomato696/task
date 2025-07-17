

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcIdentifyRoad :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcIdentifyRoad() {};
    virtual ~RoadModelProcIdentifyRoad() {};

    const char * name() {
        return "proc_identify_road";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int break_join_road_break(RoadModelSessionData* session); 

    int gen_lane_break(RoadModelSessionData* session);

    int identify_by_stop_line(RoadModelSessionData* session);
    
    int identify_by_inter_break_point(RoadModelSessionData* session);

    int identify_road_break(RoadModelSessionData* session);

    int sync_break_pos(RoadModelSessionData* session);

    int gen_sub_road_segment(RoadModelSessionData* session);

    int break_road_segment(RoadModelSessionData* session,
             RoadSegment *road_segment, std::vector<RoadSegment*> &ret);

    bool need_to_break(RoadModelSessionData* session,
            RoadSegment *road_segment, int32_t ps_index);

    int merge_road_segment(RoadModelSessionData* session);

    int make_poss_tree(RoadModelSessionData* session);

    bool judge_same_lane_line(RoadModelSessionData* session,
            LaneCenterFeature* lsg_prev, LaneCenterFeature *lsg);

    bool judge_has_break_around(RoadModelSessionData* session,
            KeyPose *poss, int road_index);

    RoadSegment* get_same_segment(RoadModelSessionData* session,
            KeyPose* poss);

    int get_segment_by_dir(RoadModelSessionData* session,
            RoadSegment* segment, bool front,
            std::vector<RoadSegment*> &merge_segment_vec);

    RoadSegment* get_same_segment(RoadModelSessionData* session,
            KeyPose* poss, bool front);

    bool identify_fork_entrance(RoadModelSessionData* session,
            RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc, bool next);

    bool identify_inner_merge_entrance(RoadModelSessionData* session,        
            RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lsg);

    bool identify_inner_merge_barrier(RoadModelSessionData* session,
            RoadSegment *road_segment);

    bool identify_lane_num_different(RoadModelSessionData* session,
            RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lc, bool next);

    bool identify_attr_different(RoadModelSessionData* session,
            RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lsg);

    bool identify_gradual_change(RoadModelSessionData* session,              
            RoadSegment *road_segment, KeyPose *poss, LaneCenterFeature *lsg);

    // bool is_over_boundary(KeyPose* pos, LaneCenterFeature* lsg);

    int match_lane_sample_list(RoadModelSessionData* session,
            RoadSegment *road_segment,
            KeyPose *poss,
            LaneCenterFeature *lsg,
            bool is_front,
            double max_length,
            std::vector<LaneCenterFeature*> &lsg_list);

    bool identify_normal_lane(std::vector<LaneCenterFeature*> &lsg_list);

    bool identify_null_lane(std::vector<LaneCenterFeature*> &lsg_list);

    bool identify_triangle_lane(std::vector<LaneCenterFeature*> &lsg_list, double &expect_vertex, bool need_asc=true);

    KeyPose* get_break_point(RoadModelSessionData* session, 
            KeyPose *poss, LaneCenterFeature *lsg, double break_offset);

    Feature* seach_neares_fork(RoadModelSessionData* session_data,
            LaneCenterFeature* lsg, double expect_dis);

    Eigen::Vector3d& get_base_line_dir(KeyPose *poss);

    int extend_by_stop_line(RoadModelSessionData* session);

    int get_stop_break_pos(RoadModelSessionData* session, RoadSegment* road_segment, KeyPose* poss);


    bool match_walk_and_lane(RoadModelSessionData* session,
             LaneCenterFeature* lsg, KeyPose* poss, bool front);

    // bool match_stop_and_lane(RoadModelSessionData* session,
    //          LaneCenterFeature* lsg, KeyPose* base_poss, bool front);
    int fresh_poss_break_status(RoadModelSessionData* session,
        KeyPose* poss, UMAP<int, int> &pro_map, int rc_index, int side_status, 
        bool next, bool m_f, bool m_s);

    bool match_stop_line(LaneCenterFeature* lsg, Feature* fp, bool front);

};
}
}

