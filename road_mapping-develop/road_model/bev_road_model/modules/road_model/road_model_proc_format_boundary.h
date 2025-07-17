

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcFormatBoundary :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcFormatBoundary() {};
    virtual ~RoadModelProcFormatBoundary() {};

    const char * name() {
        return "format_boundary";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int extract_boundary(RoadModelSessionData* session);

    int reline_boundary_in_road(RoadModelSessionData* session,
            SubRoadSegment* road_segment, int rc_index, bool left);
};

}
}
