
#pragma once

#include "road_model_session_data.h"


namespace fsdmap {
namespace road_model {
    class RoadModelProcBatchProcess : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
    {
    public:
        RoadModelProcBatchProcess() {};
        virtual ~RoadModelProcBatchProcess() {};
        const char *name()
        {
            return "proc_batch_process";
        }
        virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);
    };
}
}