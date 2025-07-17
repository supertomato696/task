

#pragma once

#include "road_model_session_data.h"

//template<typename K, typename V>
// using UMAP = std::map<K, V>;

namespace fsdmap {
namespace road_model {


class RoadModelProcSplitRoad :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcSplitRoad() {};
    virtual ~RoadModelProcSplitRoad() {};

    const char * name() {
        return "proc_split_road";
    }


    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int make_road_segment(RoadModelSessionData* session);
    int save_debug_info(RoadModelSessionData* session);
};

}
}
