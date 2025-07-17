#pragma once
#include <core/geometry.h>
namespace fast_road_model
{
    struct LaneBoundary;
    struct LaneGroup;
    struct LBPoint : public BasePoint<LBPoint>
    {
        // 
        LaneBoundary* from_lane_boundary{NULL};
        LaneGroup* from_lane_group{NULL};

        int type = 99;
        int color = 99;
        
        int left_or_right = -1;   // 标志该点归属左/右边界, 1 for left, 2 for right, 3 for both
        LBPoint *left_lb=NULL, *right_lb=NULL;

    };
    using LBPointPtr = std::shared_ptr<LBPoint>;
    struct LaneBoundary: public BasePoint<LaneBoundary>
    {
        int id;
        int type=99;
        int color=99;
        std::vector<LBPointPtr> points;
    };
    
    using LaneBoundaryPtr = std::shared_ptr<LaneBoundary>;
    
}