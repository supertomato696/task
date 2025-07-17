

#pragma once

#include "road_model_session_data.h"
#include "utils/algorithm_util.h"
#include "deque"
//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcSampleLine :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcSampleLine() {};
    virtual ~RoadModelProcSampleLine() {};

    const char * name() {
        return "proc_sample_line";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int sample_link(RoadModelSessionData* session);

    int sample_key_pose(RoadModelSessionData* session);

    int cut_link_from_crosspoint(RoadModelSessionData* session);

    int smooth_lane_line(RoadModelSessionData* session);

    int smooth_lane_center(RoadModelSessionData* session);

    int smooth_boundary(RoadModelSessionData* session);

    int get_link_pos_by_trail(RoadModelSessionData* session,KeyPoseLine*link,
        KeyPose* poss);
    std::string calc_trail_id(std::string str);

    void find_same_link(RTreeProxy<KeyPose*, float, 2> &link_pos_tree,
                   KeyPoseLine* link, std::deque<KeyPoseLine*> &one_link,
                   std::set<KeyPoseLine*>&hash_map ,bool is_next, bool current_link);

    std::vector< std::deque<KeyPoseLine*>> cluster_link(RoadModelSessionData* session);
    void  smooth_link_pose(  std::vector< KeyPose*> &link_list);

    void calc_smoth_score(KeyPoseLine&kl);
    void calc_junction_score(RoadModelSessionData* session,KeyPoseLine*kl);
 
    std::vector<Eigen::Vector3d> test_trjectory;

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

    std::vector<std::pair<Eigen::Vector3d,std::vector<Eigen::Vector3d>>> resize_junctions;

    std::vector<LaneCenterLine*> lane_center_line_sample_list_opt; // bev感知车道中心线的采样结果
    std::vector<LaneLineSampleLine*> lane_line_sample_list_opt;// 在整合了session->lane_line_instance_map、session->sem_lane_instance_map 中所有的车道线点后，在一起重新等间距采样
    
};

}
}
