#pragma once
#include <core/geometry.h>
#include <map>
namespace fast_road_model
{
    struct RBPoint : public BasePoint<RBPoint>
    {
        // 
    };
    using RBPointPtr = std::shared_ptr<RBPoint>;
    struct RoadBoundary: public BasePoint<RoadBoundary>
    {
        int id;
        int type;
        std::vector<RBPointPtr > points;
    };
    using RoadBoundaryPtr = std::shared_ptr<RoadBoundary>;
}