

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcFillLane :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcFillLane() {};
    virtual ~RoadModelProcFillLane() {};

    const char * name() {
        return "proc_fill_lane";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int reline_road_segment(RoadModelSessionData* session);

    bool is_conflict(RoadModelSessionData* session,
            LaneCenterFeature* left, LaneCenterFeature* right);

    void fill_with_same_edge(RoadModelSessionData* session,
            LaneCenterFeature* first, KeyPose* poss);

    bool has_conflict(RoadModelSessionData* session, LaneCenterFeature* lc,
            KeyPose* poss, std::vector<LaneCenterFeature*> &filled_lane_sample);

    int fill_road_with_lane_line(RoadModelSessionData* session);

    int prev_fill_road(RoadModelSessionData* session);

    int sort_lane_line(RoadModelSessionData* session);

    int fresh_multi_relation(RoadModelSessionData* session);

    int process_fork_merge(RoadModelSessionData* session, bool next);

    int calc_lane_line_param(RoadModelSessionData* session, LaneCenterLine* lane_line, GenLaneLineParam &param);

    double calc_lane_line_score(RoadModelSessionData* session, 
            LaneCenterFeature *lc, LaneLineParam &param, int times);

    double format_center_gap(double center_gap);

    double calc_prev_next_pos_gap(LaneCenterFeature *lc, LaneCenterFeature *prev, LaneCenterFeature *next);

    double calc_prev_next_pos_theta(LaneCenterFeature *lc, LaneCenterFeature *prev, LaneCenterFeature *next);

    int fill_road_with_template(RoadModelSessionData* session);

    bool lane_sample_on_same_edge(RoadModelSessionData* session, LaneCenterFeature* left,
            LaneCenterFeature *right);

    bool is_valid_lane(LaneCenterFeature *lc);
    
    int reline_lane_sample(RoadModelSessionData* session);

    // int smooth_lc_attr(RoadModelSessionData* session);

    // int degrade_lc_num(KeyPose* poss, LaneCenterFeature* lc, LaneCenterFeature* target_lc, bool front);

    bool judge_same_lane_line(RoadModelSessionData* session, 
            LaneCenterFeature* lc_prev,
            LaneCenterFeature *lc, bool use_org, int mode=0);

    int64_t gen_lane_line_with_lc(RoadModelSessionData* session,
            KeyPose* poss, LaneCenterFeature* lc, GenLaneLineParam &param);

    int gen_lane_line(RoadModelSessionData* session, LaneCenterFeature* lc,
            double scope, bool front, GenLaneLineParam &param, std::vector<std::shared_ptr<LaneCenterLine>> &ret);

    bool gen_lane_line1(RoadModelSessionData* session, LaneCenterFeature* lc,
        double scope, bool front, GenLaneLineParam &param, LaneCenterLine* curr_line);

    int sample_lane_line(RoadModelSessionData* session,
            LaneCenterLine *lane_line, LaneCenterLine *sample_line, GenLaneLineParam &param);

    bool get_line_score(LaneCenterLine* ll, double &score, bool is_left);

    int valid_real_case(LaneCenterLine* ll);

    bool get_line_match(LaneCenterLine* ll);

    int filter_lane_without_boundary(RoadModelSessionData* session);

    void mark_valid_lc(RoadModelSessionData* session, KeyPose* poss, LaneCenterFeature* lc);

    // void filter_invalid_lc(RoadModelSessionData* session, KeyPose* poss, LaneCenterFeature* lc);

    int spread_valid_lc(RoadModelSessionData* session, KeyPose* poss, LaneCenterFeature* lc);

    void spread_valid_lc(RoadModelSessionData* session, LaneCenterFeature* lc, bool front, double total_dis);

    void spread_poss_boundary_status(RoadModelSessionData* session, KeyPose* poss, bool is_left);

    int fill_with_lc_list(RoadModelSessionData* session,
        RoadSegment* road_segment, int start_index, int end_index,
        std::vector<LaneCenterFeature*> &line_vec, bool next,
        LaneCenterFeature* from_lc, LaneCenterFeature* to_lc);
 

    LaneCenterFeature* get_next_valid_lc(RoadModelSessionData* session,
            RoadSegment* road_segment, int64_t poss_index, LaneCenterFeature* lc, bool next);

    LaneCenterFeature* get_next_reline_lc(RoadModelSessionData* session,
        RoadSegment* road_segment, int64_t poss_index, LaneCenterFeature* lc, bool next);

};

}
}
