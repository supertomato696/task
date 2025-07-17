

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcBindTrail :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcBindTrail() {};
    virtual ~RoadModelProcBindTrail() {};

    const char * name() {
        return "proc_bind_trail";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int get_poss_cross_point_mutil(RoadModelSessionData* session);

    int get_poss_cross_fls(RoadModelSessionData* session);

    int get_poss_cross_object(RoadModelSessionData* session);

    int get_poss_cross_object2(RoadModelSessionData* session);
    
    //add 判断link是否穿过人行横道
    bool judge_keypose_cross_crosswalk(RoadModelSessionData* session, RoadObjectInfo* object, KeyPose* keypose);

    int sort_new_lc(RoadModelSessionData* session);

    int search_nearest_lc(RoadModelSessionData* session,
            KeyPose* poss, std::vector<LaneCenterFeature*> &secs, 
            std::vector<LaneCenterFeature*> &op_secs);

   int search_nearest_fls(RoadModelSessionData* session,
           KeyPose* poss, std::vector<LaneLineSample*> &secs);

   int search_nearest_crosswalk(RoadModelSessionData* session,
           KeyPose* poss, std::vector<RoadObjectInfo*> &secs);

};

}
}
