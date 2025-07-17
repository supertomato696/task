

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

class RoadModelProcBuildBoundary :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcBuildBoundary() {};
    virtual ~RoadModelProcBuildBoundary() {};

    const char * name() {
        return "build_boundary";
    }

    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int search_boundary(RoadModelSessionData* session);

    int build_road_center(RoadModelSessionData* session);

    int search_nearest_boundary(RoadModelSessionData* session, KeyPose* poss);

    int build_road_center_by_poss(RoadModelSessionData* session, KeyPose* poss);

    int mark_curr_road_center(RoadModelSessionData* session, KeyPose* poss);

    int make_road_center(RoadModelSessionData* session, KeyPose* poss, RoadCenterFeature* rc);

    int smooth_road_center_single(RoadModelSessionData* session, RoadSegment* road_segment);

};

}
}

