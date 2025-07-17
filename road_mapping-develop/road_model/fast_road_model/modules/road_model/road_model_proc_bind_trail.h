

#pragma once

#include "road_model_session_data.h"
#include "utils/algorithm_util.h"
#include <core/lane_boundary.h>
//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {
    class RoadModelProcBindTrail : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
    {
    public:
        RoadModelProcBindTrail() {};
        virtual ~RoadModelProcBindTrail() {};

        const char *name()
        {
            return "proc_bind_trail";
        }

        virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);

    private:
        struct LinkLine
        {
            std::vector<KeyPose *> list;
        };

        struct TurnRightPoint
        {
            int mark_status{0}; // s:1<<1  e:1<<2
            double distance{-1};
            LaneCenterFeature* lc{NULL};
            fast_road_model::LBPointPtr lb{nullptr};
            BoundaryFeature* from_road_boundary{NULL};
        } ;

        //分合流新旧线映射 
        template <class T>
        struct NewOldGroupLine
        {
            T* new_line{NULL};
            T* old_line{NULL};
            int cnt = 0;
            bool need_deleate = false;
        };


        std::vector<LinkLine> turn_right_range;
        std::vector<LaneCenterGroupLine *> oppo_lane_center;
        std::vector<BoundaryFeature *> candidate_right_boundary;
        std::map<KeyPoseLine*,std::vector<TurnRightPoint>> turn_right_candidate;
        // std::map<KeyPoseLine*,std::vector<TurnRightPoint>>  pruning_turn_right_candidate;
        std::map<KeyPoseLine*,std::vector<std::vector<TurnRightPoint>>>  pruning_turn_right_candidate;

        LaneCenterFeature* debug_ps;


        void update_lane_center_sample_tree(RoadModelSessionData *session);

        void update_lb_sample_tree(RoadModelSessionData* session);
        
        void update_lane_line_sample_tree(RoadModelSessionData* session);

        int calc_lane_curvature(RoadModelSessionData *session);

        void bfs_fill_hit_status(RoadModelSessionData*session,LaneCenterGroupLine *line);

        void dfs_fill_hit_status(LaneCenterFeature *prev, LaneCenterGroupLine *line, const int cur_index, const int size, const bool forward);

        int dfs_link_mark_lane_center_candidate(KeyPoseLine *kpl, int index, bool is_forward,
                                                RTreeProxy<LaneCenterFeature *, float, 2> &lane_center_tree);

        int hit_lane_center_candidate(RoadModelSessionData *session);
        
        int fix_loss_road_boundary(RoadModelSessionData *session);
        
        int sample_lane_center(RoadModelSessionData *session);

        int sample_lane_boundary(RoadModelSessionData *session);

        int sample_road_boundary(RoadModelSessionData *session);

        int sample_and_smooth_line(RoadModelSessionData* session);

        int link_mark_turn_right_candiadte(RoadModelSessionData *session);

        int hit_lane_center_fill_neighbor(RoadModelSessionData *session);

        int fill_lane_boundary(RoadModelSessionData *session);

        int lane_center_bind_pose(RoadModelSessionData *session);

        int reverse_lane_center(RoadModelSessionData *session);

        int hit_road_boundary_candidate(RoadModelSessionData *session);

        int expend_fill_lane_boundary(RoadModelSessionData *session);

        int change_sm_lane_boundary(RoadModelSessionData *session);

        int save_debug_info(RoadModelSessionData *session);

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
