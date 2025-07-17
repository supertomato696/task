

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {


/**
 * @brief ����ģ��
 */
class RoadModelProcGenLaneLine :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcGenLaneLine() {};
    virtual ~RoadModelProcGenLaneLine() {};

    const char * name() {
        return "proc_gen_lane_line";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int make_gen_lane_center_tree(RoadModelSessionData* session);

    int match_context_set(RoadModelSessionData* session);

    int make_lane_line_single(RoadModelSessionData* session);

    int make_lane_line_mutil(RoadModelSessionData* session);

    int make_lane_line_raw(RoadModelSessionData* session);

    int merge_lane_line(RoadModelSessionData* session);

    int merge_lane_center(RoadModelSessionData* session);

    int filter_lane_center(RoadModelSessionData* session);

    int get_valid_context(RoadModelSessionData* session, LaneCenterFeature* lc, bool prev);

    int get_valid_context1(RoadModelSessionData* session, LaneCenterFeature* lc, bool prev);

    int64_t get_lc_max_length(RoadModelSessionData* session, 
            std::vector<LaneCenterFeature*> &lc_list, bool next);

    LaneCenterFeature* search_same_dir_lane_center_gen(RoadModelSessionData* session,
            LaneCenterFeature* lc);

    bool calc_lane_center_line_match_score(RoadModelSessionData* session,
            LaneCenterFeatureMatchPair *pair);

    int gen_new_lane_center_group(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
            LaneCenterLine* tar_line,
            int64_t start_index, int64_t end_index, LaneCenterFeatureTree &trail_tree);

    int update_lane_center_group_line(RoadModelSessionData* session,
            LaneCenterFeature* tar_pt, LaneCenterFeatureTree &trail_tree, int mode);

    int match_lane_center_line(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterFeatureTree &trail_tree,
            LaneCenterLine* tar_line);

    int match_lane_center_line_raw(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterFeatureTree &trail_tree,
            LaneCenterLine* tar_line);

    int save_frame_log_lane_center(RoadModelSessionData* session,
        std::vector<std::shared_ptr<LaneCenterGroupLine> > &line_group,
        LaneCenterLine* tar_line, int times);


    int extract_lane_line(RoadModelSessionData* session,
            LaneCenterFeature* lc, LaneCenterLine* tar_line);

    void update_lane_side(RoadModelSessionData* session,
            LaneCenterGroupLine* src_line, int64_t src_index, LaneCenterFeature* tar_lc, bool left);

    void update_lane_center(RoadModelSessionData* session,
            LaneCenterGroupLine* src_line, int64_t src_index, LaneCenterFeature* tar_lc);

    LineMatchParam<LaneCenterFeature> match_line_long_distance(
        RoadModelSessionData* session,
        LaneCenterFeature* src_pt, LaneCenterLine* tar_line, int64_t index, bool next);

    int smooth_group_lane_center(RoadModelSessionData* session);

    int gen_lane_center_score(RoadModelSessionData* session);

    int vote_edge(RoadModelSessionData* session);

    int filter_lane_center_by_length(RoadModelSessionData* session, bool next);

    int search_lane_center_group_line(RoadModelSessionData* session,
        LaneCenterFeatureTree &trail_tree, LaneCenterLine* tar_line);

    int match_line_to_line(RoadModelSessionData* session,
        LaneCenterLine* tar_line,
        UMAP<LaneCenterFeature*, std::vector<LineMatchParam<LaneCenterFeature>>> &match_map);

    int merge_lane_center_edge(RoadModelSessionData* session,
        std::vector<LaneCenterFeature*> &lane_center_list, bool next);


    // int64_t get_lc_max_attr(RoadModelSessionData* session, 
    //         std::vector<LaneCenterFeature*> &lc_list, bool next) {
    //     int64_t lc_num = 0;
    //     // std::vector<LaneCenterFeature*> new_lc_list;
    //     for (auto &lc : lc_list) {
    //         if (lc->invalid()) {
    //             continue;
    //         }
    //         lc->length_status = 0;
    //         ++lc_num;
    //         // new_lc_list.push_back(lc);
    //     }
    //     int64_t valid_num = lc_num;
    //     int64_t prev_lc_num = lc_num;
    //     bool skip_calc = false;
    //     int64_t times = 0;
    //     while (lc_num > 0) {
    //         LOG_INFO("get_max_length[next={}, times={}, lc_num={}", next, times++, lc_num);
    //         for (auto &lc : lc_list) {
    //             auto next_lc = lc;
    //             while (next_lc != NULL) {
    //                 session->debug_pos(next_lc->pos);
    //                 if (next_lc->invalid()) {
    //                     break;
    //                 }
    //                 if (next_lc->length_status == 1) {
    //                     break;
    //                 }
    //                 double max_length = 0;
    //                 bool has_valid_prev = false;
    //                 auto &all_vec = next ? next_lc->context.all_prev : next_lc->context.all_next;
    //                 for (auto &prev_lc_ptr : all_vec) {
    //                     auto &prev_lc = prev_lc_ptr.src;
    //                     if (!prev_lc->invalid() && prev_lc->length_status == 0) {
    //                         has_valid_prev = true;
    //                         break;
    //                     }
    //                     double prev_max = next ? 
    //                         prev_lc->context.prev_max_length : prev_lc->context.next_max_length;
    //                     double dis = alg::calc_dis(next_lc->pos, prev_lc->pos);
    //                     prev_max += dis;
    //                     max_length = std::max(max_length, prev_max);
    //                 }
    //                 if (has_valid_prev && !skip_calc) {
    //                     break;
    //                 }
    //                 if (next) {
    //                     next_lc->context.prev_max_length = max_length;
    //                 } else {
    //                     next_lc->context.next_max_length = max_length;
    //                 }
    //                 next_lc->length_status = 1;
    //                 --lc_num;
    //                 auto &all_next = next ? next_lc->context.all_next : next_lc->context.all_prev;
    //                 if (all_next.size() != 1) {
    //                     break;
    //                 }
    //                 next_lc = all_next.front().src;
    //             }
    //         }
    //         skip_calc = prev_lc_num == lc_num;
    //         prev_lc_num = lc_num;
    //     }
    //     return valid_num;
    // }
 
};

}
}
