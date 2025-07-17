

#pragma once

#include "road_model_session_data.h"

namespace fsdmap
{
    namespace road_model
    {
        class RoadModelProcFormatNewRoad : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
        {
        public:
            RoadModelProcFormatNewRoad() {};
            virtual ~RoadModelProcFormatNewRoad() {};

            const char *name()
            {
                return "proc_format_new_road";
            }

      
            virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session_data);

        private:
            int delte_sigle_point(RoadModelSessionData *session);
            // 
            int gen_lane_center_line(RoadModelSessionData *session);
            // 
            int fix_less_lane(RoadModelSessionData *session);

            int gen_relation(RoadModelSessionData *session);

            int temp_gen_relation(RoadModelSessionData *session);

            int generate_new_stop_line_by_lane_group(RoadModelSessionData *session);
            
            int sample_lane_center_pos(RoadModelSessionData *session);

            int save_debug_info(RoadModelSessionData *session_data);

    };
}
}
