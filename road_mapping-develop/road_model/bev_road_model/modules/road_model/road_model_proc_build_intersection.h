
#pragma once

#include "road_model_session_data.h"

namespace fsdmap
{
    namespace road_model
    {

        class RoadBuildIntersection : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
        {
        public:
            RoadBuildIntersection() {};
            virtual ~RoadBuildIntersection() {};
            const char *name()
            {
                return "build_intersection";
            }
            virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);

        private:
            void  build_inter(RoadModelSessionData *session);
            int save_debug_info(RoadModelSessionData *session);
            void build_one_inter( InterInfo& inter);
            InterInfo::VirtualLane create_one_connect(const InterInfo::KeyPoint& p1,const InterInfo::KeyPoint& p2);
            Eigen::Vector3d get_cross(const InterInfo::KeyPoint& p1,const InterInfo::KeyPoint& p2,bool &status);
            Eigen::Vector3d calc_center(const InterInfo::KeyPoint& p1,const InterInfo::KeyPoint& p2);
            bool in_middle(const InterInfo::KeyPoint& p1,const InterInfo::KeyPoint& p2,const Eigen::Vector3d &center);
            std::vector<Eigen::Vector3d> genarate_spline(const std::vector<Eigen::Vector3d> &ctrl,int max_num,double res);
            bool is_access(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2, const std::string &turn_type);
            InterInfo::LaneGroup get_sub_lane_group(const InterInfo::LaneGroup &in_groups, std::string &turn_type);
            Eigen::Matrix<int ,-1,-1> connect_rule(const InterInfo::LaneGroup &in_groups,const InterInfo::LaneGroup &out_groups,const std::string & turn_type);
            InterInfo::LaneGroup genarate_virture_lane_group(const InterInfo::LaneGroup &in_groups,const InterInfo::LaneGroup &out_groups,const std::string & turn_type);
            void  get_in_out_lane_groups(RoadModelSessionData *session);
            // 
            void add_virture_lane_group(RoadModelSessionData *session);
            std::vector<Eigen::Vector3d> get_point(RoadLaneInfo* lane_info,bool is_in);
            void  creat_new_virtual_group(RoadModelSessionData *session,const InterInfo::LaneGroup& lane_group);
            RoadLaneInfo* creat_lane_line_obj(RoadModelSessionData *session);
            RoadLaneInfo*  creat_new_lane_line(RoadModelSessionData *session,  RoadLaneGroupInfo* lane_group,const InterInfo::VirtualLane&vl);
            std::vector<Eigen::Vector3d >gnarate_lane_boundary_geo(const InterInfo::VirtualLane&vl,bool is_left);
            // 
        };
    }
}