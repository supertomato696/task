
#pragma once

#include "road_model_session_data.h"
#include "utils/polyfit.h"

namespace fsdmap {
namespace road_model {
    class RoadModelProcSplitMerge : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
    {
    public:
        RoadModelProcSplitMerge() {};
        virtual ~RoadModelProcSplitMerge() {};
        const char *name()
        {
            return "proc_split_merge";
        }
        virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);

    private:
        // 前后搜索的信息
        struct SearchBoundaryInfo {
            int start_index = -1; // 分合流点的 index
            std::vector<int> search_index; // 满状态点(is_full_lc)的 index
            std::vector<int> continuous_search_index; // 连续的满状态点(is_full_lc)的 index，从突变点往左右开始搜索；针对prev：该值由大到小，针对next：该值由小到大
            std::vector<float> left_theta; // 往前搜索：左边界上，满状态点相对分合流点的角度，往后搜索：左边界上，分合流点相对满状态点的角度
            std::vector<float> right_theta; // 往前搜索：右边界上，满状态点相对分合流点的角度，往后搜索：左边界上，分合流点相对满状态点的角度
        };

        struct SortDistance {
            double dis;
            int type; // 0: line center prev, 1: line center next, 2: lane line prev, 3: lane line next
            int index;
            Eigen::Vector3d pt;
            SortDistance(double _dis, int _type, int _index, Eigen::Vector3d _pt):dis(_dis), type(_type), index(_index), pt(_pt) {};
        };

        struct InitSplitMergeIndex{
            int index;
            int status = 0; //0:未知， 1 分流变大， 2，合流变小
            // bool is_split = true;//车道数是否变大
        };

        //分合流用的
        std::unordered_map<KeyPose*, LaneCenterFeature*> m_keypose_2_sm_lc; // 分合流的中心点绑定到 keypose
        std::vector<LaneCenterFeature*> classified_lc_points_for_visual_debug; // 打印
        RTreeProxy<KeyPose*, float, 2> raw_link_fork_points; // 非主路口的叉路口
        std::vector<Eigen::Vector3d> raw_link_fork_points_for_visual; // 打印
        std::unordered_map<KeyPoseLine*, std::vector<KeyPose*>> turn_right_endpoint_m; 
        std::vector<OneSplitMergeMatch> sm_matchs;

        //分合流调试用的
        std::map<KeyPose*, std::vector<InitSplitMergeIndex>> init_sm_cs_indexs_before; 
        std::map<KeyPose*, std::vector<InitSplitMergeIndex>> init_sm_cs_indexs_update;
        std::map<KeyPose*, std::vector<SearchBoundaryInfo>> prev_search_bd_infos_before;
        std::map<KeyPose*, std::vector<SearchBoundaryInfo>> prev_search_bd_infos_update;
        std::vector<Eigen::Vector3d> add_pts;
        std::map<KeyPose*, std::vector<std::pair<int, int>>> road_steady_index;
        std::map<KeyPose*, std::vector<std::pair<int, int>>> road_steady_index_update;

        // 连线
        std::vector<BreakInfo*> sm_break_candidate;
        std::vector<CandidateSeed<LaneCenterFeature>> sm_cs_origin_list;//原始转化
        //string： split, merge ,straight
        std::vector<std::map<std::string, std::vector<std::vector<CandidateSeed<LaneCenterFeature>>>>> split_merge_connect_candidates_list; 

    private:    
        int stage_3_process_split_merge_scene(RoadModelSessionData* session);

        int bind_sm_to_keypose(RoadModelSessionData* session);

        void mask_turn_right_link(RoadModelSessionData* session);

        PolyFit init_polyfit(const std::vector<Eigen::Vector3d>& points, LaneCenterFeature* center = nullptr);

        std::vector<Eigen::Vector3d> polyfit_points(const std::vector<Eigen::Vector3d>& points,
            const std::vector<Eigen::Vector3d>& points_without_z,  PolyFit& fit);

        //步骤3 分合流的函数
        void find_link_fork_road(RoadModelSessionData* session);

        void find_link_fork_road_from_nodes(RoadModelSessionData* session);

        void calc_curvature_3pt(RoadModelSessionData* session); //3点法

        void calc_curvature_5pt(RoadModelSessionData* session); //5点法
        
        void convert_boundary_to_pcd(RoadModelSessionData* session); 
        
        std::vector<std::pair<int,int>> calc_skip_ranges(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, int skip_num);
        std::vector<std::pair<int, int>> calc_skip_ranges_for_turn_right(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, int skip_num);
        bool should_skip(int i, const std::vector<std::pair<int, int>>& skip_ranges);

        void move_candidate_to_lanes_change(HorizonCrossLine* hor_cross_line, const std::vector<std::pair<int,int>>& skip_ranges, 
                        std::vector<InitSplitMergeIndex>& init_sm_cs_indexs,const std::vector<std::pair<int,int>>& skip_ranges2);

        void add_point(RoadModelSessionData* session, std::vector<Eigen::Vector3d>& pts, std::vector<Eigen::Vector3d>& pts_no_z,
            Eigen::Vector3d prev_inside_pt, Eigen::Vector3d center_pt, bool is_prev,LaneCenterGroupLine* prev_line, LaneCenterGroupLine* next_line, 
            const std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>* >>>& prev_cp_m,
            const std::map<LaneCenterGroupLine*, std::vector<std::pair<Eigen::Vector3d, CrossPoint<LaneCenterGroupLine>* >>>& next_cp_m);

        void init_search_region(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, std::vector<InitSplitMergeIndex>& init_sm_cs_indexs);
        
        void refine_search_region(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line, const std::vector<InitSplitMergeIndex>& init_sm_cs_indexs,
                                std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                                std::vector<SearchBoundaryInfo>& next_search_bd_infos);
        LaneCenterFeature* find_candidate(RoadModelSessionData *session, 
                Eigen::Vector3d& pt, LaneCenterGroupLine* src_line, bool is_front, bool check_angle);

        void refine_search_region2(RoadModelSessionData* session, HorizonCrossLine* hor_cross_line,
                                                    const std::vector<InitSplitMergeIndex>& init_sm_cs_indexs,
                                                    std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                                                    std::vector<SearchBoundaryInfo>& next_search_bd_infos);

        void recovery_point(RoadModelSessionData* session, KeyPose* first_keypose, HorizonCrossLine* hor_cross_line, 
                            const std::vector<SearchBoundaryInfo>& prev_search_bd_infos,
                            const std::vector<SearchBoundaryInfo>& next_search_bd_infos);
        
        void match_prev_next(RoadModelSessionData* session,
                            const std::vector<HorizonCrossFeature*>& feature_list,
                            int left_min, int left_max, int mid_index, int right_min, int right_max,
                            Eigen::Vector3d center_point, Eigen::Vector3d mean_road_dir, Eigen::Vector3d mean_v_road_dir,
                            const SearchBoundaryInfo& prev_search_bd_info,
                            const SearchBoundaryInfo& next_search_bd_info
                            );

        bool check_fit_points(std::vector<std::shared_ptr<CrossPoint<LaneCenterGroupLine>>>& new_line);

        // 连线
        void update_lane_center_sample_tree(RoadModelSessionData *session);

        void update_link_pos_tree(RoadModelSessionData* session);

        void connect_line(const std::vector<CandidateSeed<LaneCenterFeature>>& cs_list, 
            RoadModelSessionData *session, std::string type);

        bool get_refined_cross_link(RoadModelSessionData* session, 
                                    RTreeProxy<KeyPose*, float, 2>& link_tree,
                                    RTreeProxy<LaneCenterFeature *, float, 2>& lane_center_tree,
                                    Eigen::Vector3d bk_pos,
                                    KeyPose*& nearst_pose);

        bool find_center_bk_cross(RoadModelSessionData* session, 
                                LaneCenterFeature * lc,
                                KeyPose* nearst_pose,
                                bool is_forward,
                                Eigen::Vector3d& cross);

        bool find_center_bk_cross_with_kdtree(RoadModelSessionData* session, 
                                            Eigen::Vector3d pt,
                                            LaneCenterGroupLine* src_line,
                                            KeyPose* nearst_pose,
                                            Eigen::Vector3d& cross);

        int connect_split_merge_center_line(RoadModelSessionData *session);

        int connect_split_merge_lane_line(RoadModelSessionData *session);

        int merge_sm_break_candidate(RoadModelSessionData *session);

        // 保存

        int save_debug_info(RoadModelSessionData* session, int stage);

        int save_debug_info_connect(RoadModelSessionData* session, int stage);

        int save_debug_info_all(RoadModelSessionData* session, int stage = 0);

    };
}
}
