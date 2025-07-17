

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcBindTrail2 :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcBindTrail2() {};
    virtual ~RoadModelProcBindTrail2() {};

    const char * name() {
        return "proc_bind_trail";
    }

    /**
     * @brief 
     *
     * @param session_data 
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int get_poss_cross_object2(RoadModelSessionData* session);
    
    //add 判断link是否穿过人行横道
    bool judge_keypose_cross_crosswalk(RoadModelSessionData* session, RoadObjectInfo* object, KeyPose* keypose);

    int find_more_crosspoint_and_cut_link(RoadModelSessionData* session);
};

}
}
