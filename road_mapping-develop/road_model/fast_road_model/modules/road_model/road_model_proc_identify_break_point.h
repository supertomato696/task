

#pragma once

#include "road_model_session_data.h"


namespace fsdmap {
namespace road_model {

class RoadModelProcIdentifyBreakPoint :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcIdentifyBreakPoint() {};
    virtual ~RoadModelProcIdentifyBreakPoint() {};

    const char * name() {
        return "proc_identify_break_point";
    }


    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    #if 0
    int indentify_break_point(RoadModelSessionData* session);

    // bool cal_obj_cross_link_point(const std::vector<RoadObjectInfo*>& objs, 
    //         KeyPose* key_pose, Eigen::Vector3d& corss_pt);

    Eigen::Vector3d cal_weight_avg(const Eigen::Vector3d& stopline_cross_pt, 
        const std::vector<LaneCenterFeature*>& lc_points, 
        const Eigen::Vector3d& crosswalk_cross_pt, 
        const Eigen::Vector3d& inter_cross_pt);

    double cal_break_point(double point0_pr, 
            double sl_cross_pt_pr, 
            double lc_pts_avg_pr, 
            double lc_pts_avr, 
            double cw_cross_pt_pr, 
            double inter_cross_pt_pr);
    

    std::pair<double, double> gaussian_fusion_multiply(const double& mu1, 
        const double& sigma1, const double& mu2, const double& sigma2);

    

    int save_debug_info(RoadModelSessionData* session);
#endif
 

};

}
}
