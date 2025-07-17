

#pragma once

#include "road_model_session_data.h"
#include "utils/algorithm_util.h"

namespace fsdmap {
namespace road_model {
        
class RoadModelProcMergeFeature :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcMergeFeature() {};
    virtual ~RoadModelProcMergeFeature() {};

    const char * name() {
        return "proc_merge_feature";
    }

    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    struct TurnRightPoint
    {
        int mark_status{0}; // s:1<<1  e:1<<2
        double lc_distance{-1};
        double ll_distance{-1};
        double rl_distance{-1};
        LaneCenterFeature* lc{NULL};
        LaneLineSample* left_ll{NULL};
        LaneLineSample* right_ll{NULL};
        fast_road_model::LBPointPtr lb{nullptr};
        BoundaryFeature* from_road_boundary{NULL};
    };
    std::map<KeyPoseLine*,std::vector<TurnRightPoint>> turn_right_candidate;
    
    std::vector<std::shared_ptr<BoundaryGroupLine>> removed_boundary_line_group_list;
    std::vector<std::shared_ptr<LaneCenterGroupLine>> removed_lane_center_line_group_list;

    std::vector<alg::linestring_t> all_lc;
    std::vector<std::vector<Eigen::Vector3d>> all_lc_pnt_dir;

    int save_debug_info(RoadModelSessionData* session);
    int save_debug_info_only_lane(RoadModelSessionData* session, int index);
    int save_debug_info_only_lane_center(RoadModelSessionData* session, int index);
    int save_debug_info_only_lane_center_qzc(RoadModelSessionData* session, int index, int mode=0);
    int save_debug_info_only_boundary(RoadModelSessionData* session, int index);


    int make_tree(RoadModelSessionData* session);

    int match_key_pose(RoadModelSessionData* session);

    int make_right_turn(RoadModelSessionData* session);

    int remove_short_lines(RoadModelSessionData* session);

    int check_direction(RoadModelSessionData* session);
    
    int merge_single_trail(RoadModelSessionData* session);


    int save_frame_log_lane(RoadModelSessionData* session,
        KeyPoseLine* line, KeyPose* poss, int times);

    int merge_single_trail_lane(RoadModelSessionData* session);

        // stage: 0 merge_lane_duplicate 非右转的合并； 1 代表右转的去重
    void merge_lane_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process = false, int stage = 0);

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

    // stage: 0 代表 非右转的合并； 1 代表右转的去重
    void merge_lane_center_duplicate(RoadModelSessionData* session, KeyPoseLine* trail_ptr, bool reverse_process = false, int stage = 0);

    void convert_lane_center_to_bg(KeyPoseLine* trail_ptr);

    void merge_lane_center_split_line(RoadModelSessionData* session, KeyPoseLine* trail_ptr, std::shared_ptr<LaneCenterLine> new_line);

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

