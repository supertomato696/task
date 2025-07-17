

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcTransInterface :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcTransInterface() {};
    virtual ~RoadModelProcTransInterface() {};

    const char * name() {
        return "proc_trans_interface";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

};

}
}
