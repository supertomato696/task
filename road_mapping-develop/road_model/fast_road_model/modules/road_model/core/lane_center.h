#pragma once

#include <core/geometry.h>

namespace fast_road_model
{
    struct LaneCenter;
    struct LaneGroup;
    struct LCPoint : public BasePoint<LCPoint>
    {
        LaneCenter* from_lane_center{NULL};
        LaneGroup* from_lane_group{NULL};
    };
    using LCPointPtr = std::shared_ptr<LCPoint>;
    struct LaneCenter:public BasePoint<LaneCenter>
    {
        int id;
        std::string type;
        std::vector<LCPointPtr> points;
    };
    using LaneCenterPtr = std::shared_ptr<LaneCenter>;
}