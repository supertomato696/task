

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcInitData :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcInitData() {};
    virtual ~RoadModelProcInitData() {};

    const char * name() {
        return "proc_init_data";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int update_global_line_id(RoadModelSessionData* session);

    int init_param(RoadModelSessionData* session);

    int init_scope_data(RoadModelSessionData* session);

    int init_trail(RoadModelSessionData* session);

    int init_intersections(RoadModelSessionData* session);

    int init_feature_line(RoadModelSessionData* session);

    int init_semantic_line(RoadModelSessionData* session);

    template<class T, class L>
    T* feature_to_line(RoadModelSessionData* session, std::vector<std::shared_ptr<T>> &ptr_list, UMAP<std::string, L> &instance_map, Feature* feature) {
        if (MAP_NOT_FIND(instance_map, feature->line_id)) {
            instance_map[feature->line_id] = L();
            instance_map[feature->line_id].id = feature->line_id;
            instance_map[feature->line_id].frame_id = feature->frame_id;
            instance_map[feature->line_id].boundary_type = feature->boundary_type;
        }
        auto ele_feature = session->add_ptr(ptr_list); // 新建T类型的指针放入ptr_list的最后，并把新放入的指针返回给ele_feature
        ele_feature->init(feature);
        instance_map[feature->line_id].list.push_back(ele_feature.get());
        return ele_feature.get();
    }

    template<class T, class L>
    void feature_to_line(RoadModelSessionData* session, UMAP<std::string, L> &instance_map, T* feature) {
        if (MAP_NOT_FIND(instance_map, feature->line_id)) {
            instance_map[feature->line_id] = L();
            instance_map[feature->line_id].id = feature->line_id;
            instance_map[feature->line_id].frame_id = feature->frame_id;
        }
        instance_map[feature->line_id].list.push_back(feature);
    }
    uint8_t convert_arrow_type(const std::string& str);

    std::vector<std::shared_ptr<KeyPose>> raw_links_points; // link关系点， 所有的 link node 点， 通过 tar_pt->prev == nullptr 可以区分是哪条线的

    std::vector<std::shared_ptr<KeyPose>> raw_key_poses; // ins的数据
    std::vector<std::shared_ptr<Feature>> raw_lane_feature; // 车道边界线：BEV感知结果:车道边界线
    std::vector<std::shared_ptr<Feature>> raw_boundary_feature; // BEV感知结果:道路边界线
    std::vector<std::shared_ptr<Feature>> raw_features; // BEV感知结果:车道中心线、车道边界线、道路边界线
};

}
}
