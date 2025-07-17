

#pragma once

#include "road_model_session_data.h"


namespace fsdmap {
namespace road_model {

class RoadModelProcFixObject :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcFixObject() {};
    virtual ~RoadModelProcFixObject() {};

    const char * name() {
        return "proc_fix_object";
    }


    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    
    int fix_stopline_geom(RoadModelSessionData* session);
    int fix_stopline_geom2(RoadModelSessionData* session);
    int save_debug_info(RoadModelSessionData* session);

 

};

}
}
