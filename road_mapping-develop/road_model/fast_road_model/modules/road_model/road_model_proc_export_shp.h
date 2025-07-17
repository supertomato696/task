#pragma once

#include "road_model_session_data.h"
#include <ogrsf_frmts.h>
#include <gdal_priv.h>

namespace fsdmap
{
    namespace road_model
    {

        class RoadModelExportSHP : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
        {
        public:
            RoadModelExportSHP() {};
            RoadModelExportSHP(int export_type) : export_type_(export_type) {};
            virtual ~RoadModelExportSHP() {};
            const char *name()
            {
                return "export_shp";
            }
            virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);
            int export_type_ = -1;

        private:

            bool create_field_defn(OGRLayer *layer, const std::string &key, OGRFieldType type, const std::string &element);
        
            // 1：车道线去重的矢量化
            int export_lane_boundary_to_shp_vectorize1(RoadModelSessionData *session, int flag = 0);
            int export_road_boundary_to_shp_vectorize1(RoadModelSessionData *session, int flag = 0);

            // 2： 车道线去重的矢量化+拓扑
            int export_to_shape_file_with_topo(RoadModelSessionData *session);
            int export_lane_boundary_to_shp(RoadModelSessionData *session);
            int export_lane_center_to_shp(RoadModelSessionData *session);
            int export_road_boundary_to_shp(RoadModelSessionData *session);
            int export_road_to_shp(RoadModelSessionData *session);
            int export_lane_to_shp(RoadModelSessionData *session);
            int export_lane_adas_to_shp(RoadModelSessionData *session);
            int export_road_centers_to_shp(RoadModelSessionData *session);
            int export_stop_line_to_shp(RoadModelSessionData *session);
            int export_cross_walk_to_shp(RoadModelSessionData *session);
            int export_road_mark_to_shp(RoadModelSessionData *session);
            int export_intersection_to_shp(RoadModelSessionData *session);
        };
    }
}
