#pragma once
#include "core/geometry.h"
#include "core/road_boundary.h"
#include "core/lane.h"

namespace fast_road_model
{
    struct LaneGroup: public BasePoint<LaneGroup>
    {
        int id;
        RoadBoundaryPtr left_road_boundary{nullptr};
        RoadBoundaryPtr right_road_boundary{nullptr};
        std::vector<LanePtr > lane_list;
    };
    using LaneGroupPtr = std::shared_ptr<LaneGroup>;
}
