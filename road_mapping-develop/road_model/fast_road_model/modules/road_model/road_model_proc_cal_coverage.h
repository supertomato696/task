#pragma once

#include "road_model_session_data.h"
#include "utils/polyfit.h"
#include "utils/algorithm_util.h"
namespace fsdmap {
namespace road_model {

class RoadModelProcCalCoverage :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcCalCoverage() {};
    virtual ~RoadModelProcCalCoverage() {};

    const char * name() {
        return "proc_cal_coverage";
    }

    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    struct MatchBoundaryInfo {
        int rb_index = -1;
        std::shared_ptr<BoundaryFeature> rb_pt {nullptr};
        LaneLineSample* ll_pt {NULL}; // lane line point
        LaneLineSampleGroupLine* ll_group_line {NULL};
        double dis= 0;
        bool is_left = false;
    };

    std::vector<KeyPose*> in_link_pts, out_link_pts; //4个 交点

    //前后推用的
    std::vector<CandidateSeed<LaneCenterFeature>> init_candidates;
    std::vector<CandidateSeed<LaneCenterFeature>> init_connect_candidates;
    std::vector<CandidateSeed<LaneCenterFeature>> fb_check_candidates;//前后推校验后的
    std::vector<CandidateSeed<LaneCenterFeature>> fb_fit_candidates;//前后推拟合后的
    RTreeProxy<CrossPoint<LaneCenterGroupLine>*, float, 2> split_merge_pts_tree;

    //调试用的
    std::vector<LaneCenterGroupLine*> del_short_lane_center_list;

    void update_lane_center_sample_tree(RoadModelSessionData* session);

    void update_split_merge_pts_tree(RoadModelSessionData* session);

    void update_lb_sample_tree(RoadModelSessionData* session);
    
    void update_lane_line_sample_tree(RoadModelSessionData* session);

    PolyFit init_polyfit(const std::vector<Eigen::Vector3d>& points, LaneCenterFeature* center = nullptr);

    std::vector<Eigen::Vector3d> polyfit_points(const std::vector<Eigen::Vector3d>& points,
        const std::vector<Eigen::Vector3d>& points_without_z,  PolyFit& fit);

    int delte_lane_all_in_junction(RoadModelSessionData *session);

    void find_no_prev_and_next_lc(RoadModelSessionData *session, 
                                RTreeProxy<LaneCenterFeature *, float, 2>& no_prev_lc_tree,
                                 RTreeProxy<LaneCenterFeature *, float, 2>& no_next_lc_tree);

    void find_no_prev_and_next_lc_v2(RoadModelSessionData *session, 
                                RTreeProxy<LaneCenterFeature *, float, 2>& no_prev_lc_tree,
                                 RTreeProxy<LaneCenterFeature *, float, 2>& no_next_lc_tree,
                                 std::vector<LaneCenterFeature *>& no_prev_buff,
                                 std::vector<LaneCenterFeature *>& no_next_buff);

    int classify_prev_and_next_lc(RoadModelSessionData*session);

    int fix_dis_connect(RoadModelSessionData*session, double dist = 8, double angle = 30, double vertical_dis = 0.5);

    int sample_lane_center(RoadModelSessionData *session);

    int sample_lane_boundary(RoadModelSessionData *session);

    int sample_road_boundary(RoadModelSessionData *session);

    int sample_and_smooth_line(RoadModelSessionData* session);

    void create_new_lane_line(RoadModelSessionData*session);

    void append_lane_line(RoadModelSessionData*session, LaneLineSampleGroupLine* ll_line, BoundaryGroupLine* rb_line, 
                        bool is_left, int prev_index, double prev_dis, int next_index, double next_dis);

    int generate_ll_by_lb(RoadModelSessionData*session);

    int fix_dis_connect_lb(RoadModelSessionData*session);

    int fix_dis_connect_ll(RoadModelSessionData*session);
    
    void get_range_points(LaneCenterFeature *lc,std::set<LaneCenterFeature *>&hash, double range,  std::vector<LaneCenterFeature*> &lc_points,bool forward);

    int init_lane_center_connect_candidate(RoadModelSessionData *session);

    void search_stopline_keypose(RoadModelSessionData* session, Eigen::Vector3d center,
          std::unordered_map<KeyPoseLine*, std::unordered_map<Eigen::Vector3d, std::shared_ptr<LinkInterInfo>,Vector3dHash>>& link_inter_info_);

    void search_lc_in_out_keypose(RoadModelSessionData* session, const Eigen::Vector3d& center,
            std::unordered_map<KeyPoseLine*, std::unordered_map<Eigen::Vector3d, std::shared_ptr<LinkInterInfo>,Vector3dHash>>& link_inter_info_);

    int find_link_entrance(RoadModelSessionData* session); //总入口

    int stage_1_recovery_single_road_cross_points(RoadModelSessionData* session);

    int verify_elements(RoadModelSessionData* session);

    void transform_init_pts_to_mature(RoadModelSessionData* session);

    int transform_mature_pts_to_line(RoadModelSessionData* session);

    void find_continuous_pt(std::vector<HorizonCrossFeature*>& cross_feature_list, std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& temp_line, 
                    std::set<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& hash_tabel, int i, int j);

    //前后推用的
    int stage_2_connect_pre_next(RoadModelSessionData* session);

    void merge_delete_line_by_straight(const CandidateSeed<LaneCenterFeature>& cs, RoadModelSessionData* session);
    
    int prev_next_connect_line(RoadModelSessionData *session);

    int rm_short_center_lines(RoadModelSessionData* session);

    std::vector<std::pair<int,int>> use_link_lanenum_ranges(RoadModelSessionData *session, KeyPoseLine* link, int use_num);

    Eigen::Vector3d cal_road_true_dir(RoadModelSessionData* session, KeyPose* kp);
    
    void find_cross_link_points(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts, 
        std::vector<KeyPose*>& out_link_pts);

    void find_cross_link_points_by_score(RoadModelSessionData* session, std::vector<KeyPose*>& in_link_pts);

    void find_cross_link_points_by_site_center(RoadModelSessionData* session, const Eigen::Vector3d& center,     
                        std::vector<KeyPose*>& in_link_pts);

    KeyPose* find_in_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m,
        const Eigen::Vector3d& center);

    KeyPose* find_out_start_keypose(RoadModelSessionData* session, KeyPose* in_pt, std::vector<KeyPose*> in_link_pts, 
        std::map<KeyPose*, RoadObjectInfo*>& stop_line_list_m);

    int cal_actual_road_boundary_count(RoadModelSessionData* session, KeyPose* keypose, Eigen::Vector3d road_dir,
        std::vector<std::shared_ptr<CrossPoint<BoundaryGroupLine>>>& in_road_boundary_points);

    // void sort_points_by_distance(std::vector<Eigen::Vector3d>& all_points, const Eigen::Vector3d& left_bf);

    // void rm_duplicate_pts(std::vector<Eigen::Vector3d>& points, double dis_threshold);

    int cal_actual_lane_center_count(RoadModelSessionData* session, KeyPose* keypose, 
        Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
        std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& in_road_center_points);

    int cal_actual_lane_count(RoadModelSessionData* session, KeyPose* keypose, 
        Eigen::Vector3d road_dir,double search_radius, Eigen::Vector3d left_bf, Eigen::Vector3d right_bf , 
        std::vector<std::shared_ptr<CrossPoint<LaneLineSampleGroupLine>>>& in_road_lane_points);

    
    double cal_true_lane_width(const std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& pts, double err);

    void cal_horizontal_miss_positions(RoadModelSessionData* session, std::shared_ptr<HorizonCrossFeature> &hor_cross_feature, 
        KeyPose* origin_start, KeyPose* start_keypose, Eigen::Vector3d road_dir, int N_hope,
         int Nt, double W_road, double W_lane_t, const Eigen::Vector3d& left_bf, const Eigen::Vector3d& right_bf,
         std::vector<Eigen::Vector3d>& in_road_center_points);

    int recovery_single_road_cross_point(HorizonCrossFeature* hor_cross_feature, int index, std::vector<std::pair<int,int>>& use_link_lanenum);

    int modify_is_full_lc_by_neighbor(RoadModelSessionData* session);

    int modify_N_road_by_neighbor(RoadModelSessionData* session);

    int gen_lane_directin(RoadModelSessionData *session);

    int save_debug_info(RoadModelSessionData* session, int stage);

    int save_debug_info_all(RoadModelSessionData* session, int stage = 0);


    template<class T>
    void smooth_line(RoadModelSessionData* session, std::vector<T*> &line_list,
            double dir_gap_threshold, std::vector<double> &radii) {
        using GapIndex = std::pair<double*, int64_t>;
        // ����dir
        for (auto line : line_list) {
            if(line->boundary_type == LaneType::ISLAND_RB) {
                continue;
            }

            session->thread_pool->schedule(
                    [&, line, session, this](utils::ProcessBar *process_bar) {
            for (auto &feature : line->list) {
                feature->calc_context_dir(radii);
            }
            line->param_list.resize(line->list.size());

            double prev_gap = 0;
            std::vector<GapIndex> pro_index(line->list.size());
            for (int i = 0; i < line->list.size(); ++i) {
                auto &feature = line->list[i];
                session->debug_pos(feature->pos);
                double dir_gap = (feature->dir - feature->context_dir).norm();
                if (alg::judge_left(feature->dir, feature->context_dir) < 0) {
                    dir_gap = -dir_gap;
                }
                line->param_list[i].dir_gap = dir_gap;
                line->param_list[i].dir_next_gap = fabs(dir_gap - prev_gap);
                line->param_list[i].dir_next_gap1 = line->param_list[i].dir_next_gap;
                pro_index[i] = {&line->param_list[i].dir_next_gap, i};
                if (i > 0) {
                    prev_gap = dir_gap;
                }
            }
            SORT(pro_index, [](const GapIndex &l, const GapIndex &r) {
                    return *l.first > *r.first;
                    });
            for (int i = 0; i < pro_index.size(); ++i) {
                auto &pro = pro_index[i];
                auto &feature = line->list[pro.second];
                if (*pro.first < dir_gap_threshold) {
                    continue;
                }
                feature->filter_status = 2;
                line->param_list[pro.second].dir_next_gap = 0;
                if (pro.second == 0 || pro.second == line->list.size() - 1) {
                    continue;
                }
                // ���¼�����һ���ڵ��dir
                auto &prev_feature = line->list[pro.second - 1];
                auto &next_feature = line->list[pro.second + 1];
                alg::calc_dir(next_feature->pos, prev_feature->pos, prev_feature->dir);
                double dir_gap = (prev_feature->dir - prev_feature->context_dir).norm();
                if (alg::judge_left(prev_feature->dir, prev_feature->context_dir) < 0) {
                    dir_gap = -dir_gap;
                }
                line->param_list[pro.second - 1].dir_gap = dir_gap;
                double prev_gap = pro.second < 2 ? 0 : line->param_list[pro.second - 2].dir_gap;
                line->param_list[pro.second - 1].dir_next_gap = fabs(dir_gap - prev_gap);
                line->param_list[pro.second + 1].dir_next_gap = fabs(line->param_list[pro.second + 1].dir_gap - dir_gap);
            }
            });
        }
        session->thread_pool->wait(1, "smooth_line");
    }
    
};

}
}