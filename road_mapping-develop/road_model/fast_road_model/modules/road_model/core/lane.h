#pragma once
#include <core/geometry.h>
#include <core/lane_boundary.h>
#include <core/lane_center.h>

namespace fast_road_model
{
    struct Lane: BasePoint<Lane>
    {
        int id;
        LaneBoundaryPtr left;
        LaneBoundaryPtr right;
        LaneCenterPtr center;
        //
    };
    using LanePtr = std::shared_ptr<Lane>;
    
}