

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcMergeFeature :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcMergeFeature() {};
    virtual ~RoadModelProcMergeFeature() {};

    const char * name() {
        return "proc_merge_feature";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);
    int save_debug_info_only_lane(RoadModelSessionData* session, int index);
    int save_debug_info_only_lane_center(RoadModelSessionData* session, int index);
    int save_debug_info_only_boundary(RoadModelSessionData* session, int index);


    int make_tree(RoadModelSessionData* session);

    int match_key_pose(RoadModelSessionData* session);

    int merge_single_trail(RoadModelSessionData* session);

    int merge_multi_lane_center(RoadModelSessionData* session);

    int make_lane_line_raw(RoadModelSessionData* session);

    LaneCenterFeature* match_line_long_distance(
            RoadModelSessionData* session,
            LaneCenterFeature* src_pt, LaneCenterLine* tar_line, int64_t index);

    int gen_new_lane_center_group_merge(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterLine* tar_line,
            int64_t start_index, int64_t end_index, LaneCenterFeatureTree &trail_tree);

    int update_lane_center_group_line_merge(RoadModelSessionData* session,
            LaneCenterFeature* tar_pt, LaneCenterFeatureTree &trail_tree, int mode);

    int match_lane_center_line(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterFeatureTree &trail_tree,
            LaneCenterLine* tar_line);

    int match_lane_center_line_raw(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterFeatureTree &trail_tree,
            LaneCenterLine* tar_line);

    int64_t get_lc_max_length(RoadModelSessionData* session,
            std::vector<LaneCenterFeature*> &lc_list, bool next);

    int extract_lane_line(RoadModelSessionData* session,
            LaneCenterFeature* lc, LaneCenterLine* tar_line);

    int save_log_multi_lane_center(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &line_group,
            LaneCenterLine* tar_line, int times);

    bool match_ground_by_point(RoadModelSessionData* session, Eigen::Vector3d &pos);

    int save_frame_log_lane(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times);

    int merge_single_trail_lane(RoadModelSessionData* session);

    void merge_lane_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process = false);

    int merge_single_lane_by_trail(RoadModelSessionData* session, KeyPoseLine* line);

    int match_frame_line_lane(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group,
            LaneLineSampleTree &trail_tree,
            LaneLineSampleLine* ins_line);

    int gen_new_lane_group(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneLineSampleGroupLine> > &trail_group, 
            LaneLineSampleLine* ins_line, LaneLineSampleTree &trail_tree);

    int update_lane_group_line(RoadModelSessionData* session,
            LaneLineSampleGroupLine* src_line, LaneLineSampleLine* ins_line,
            LaneLineSampleTree &trail_tree);

    bool is_match_line_lane(RoadModelSessionData* session,
            LaneLineSampleGroupLine* src, LaneLineSampleLine* tar,  double &score);

    int save_frame_log_boundary(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times, std::string line_id);

    void merge_boundary_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process = false);

    int merge_single_boundary_by_trail(RoadModelSessionData* session, KeyPoseLine* line);

    int match_frame_line_boundary(RoadModelSessionData* session,
            std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group,
            BoundaryFeatureTree &trail_tree,
            BoundaryLine* ins_line);

    int gen_new_boundary_group(RoadModelSessionData* session,
            std::vector<std::shared_ptr<BoundaryGroupLine> > &trail_group, 
            BoundaryLine* ins_line, BoundaryFeatureTree &trail_tree);

    int update_boundary_group_line(RoadModelSessionData* session,
            BoundaryGroupLine* src_line, BoundaryLine* ins_line,
            BoundaryFeatureTree &trail_tree);

    bool is_match_line_boundary(RoadModelSessionData* session,
            BoundaryGroupLine* src, BoundaryLine* tar,  double &score);

    int save_frame_log_lane_center(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times);

    int merge_single_trail_lane_center(RoadModelSessionData* session);

    int merge_single_lane_center_by_trail(RoadModelSessionData* session, KeyPoseLine* line);

    void merge_lane_center_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process = false);

    int match_frame_line_lane_center(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group,
            LaneCenterFeatureTree &trail_tree,
            LaneCenterLine* ins_line);

    int gen_new_lane_center_group(RoadModelSessionData* session,
            std::vector<std::shared_ptr<LaneCenterGroupLine> > &trail_group, 
            LaneCenterLine* ins_line, LaneCenterFeatureTree &trail_tree);

    int update_lane_center_group_line(RoadModelSessionData* session,
            LaneCenterGroupLine* src_line, LaneCenterLine* ins_line,
            LaneCenterFeatureTree &trail_tree);

    bool is_match_line_lane_center(RoadModelSessionData* session,
            LaneCenterGroupLine* src, LaneCenterLine* tar,  double &score);

    std::pair<int, int> search_key_pose(RoadModelSessionData* session,
        KeyPoseTree &tree, Eigen::Vector3d pos, Eigen::Vector3d dir, std::string &trail_id,
        KeyPose* curr_poss);

    int64_t vote_attr_by_fls(RoadModelSessionData* session,
            LaneLineSample *base_fls);

    int64_t vote_yellow(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_double(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_geo(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_type(RoadModelSessionData* session,
            BoundaryFeature *base_fls, std::vector<BoundaryFeature*> &attr_data_list);

    //根据feature离自车位置远近，计算feature的可靠性，离自车越近，可靠性分数越高
    template<class T>
    double get_feature_score_by_distance(RoadModelSessionData* session, T* feature) {
        double d_f = feature->raw_pos.x();
        double d_s = feature->raw_pos.y();
        // 这个2m应该是感知最近能看到的离自车x方向上2m的目标，例子车越近，感知结果越可靠，所以均值设置为2，表示概率最高，可信度最高
        // 下面的0也是同理
        double f_score = alg::calc_score_by_gaussian(d_f, 20, 2);  // 正态分布，2的均值，计算x方向上的重要性
        double s_score = alg::calc_score_by_gaussian(fabs(d_s), 4, 0);
        double score = sqrt(f_score * s_score);
        return score;
    }
};
}
}

