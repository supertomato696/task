

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcIdentifyAttr :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcIdentifyAttr() {};
    virtual ~RoadModelProcIdentifyAttr() {};

    const char * name() {
        return "proc_identify_attr";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int sync_feature_attr(RoadModelSessionData* session);

    int64_t vote_attr_by_fls(RoadModelSessionData* session,
            LaneLineSample *base_fls);

    int64_t vote_yellow(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_double(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_geo(RoadModelSessionData* session,
            LaneLineSample *base_fls, std::vector<LaneLineSample*> &attr_data_list);

    int64_t vote_type_by_fls(RoadModelSessionData* session,
            BoundaryFeature *base_fls);

    int64_t vote_type(RoadModelSessionData* session,
            BoundaryFeature *base_fls, std::vector<BoundaryFeature*> &attr_data_list);       

};
}
}

