#pragma once
#include <iostream>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <core/geometry.h>
#include <core/lane_group.h>
// #include <road_model/road_model.h>


namespace fast_road_model
{
    struct fsdmap::KeyPose;
    class RoadSegment : public BasePoint<RoadSegment>
    {
    public:
        int id;
        void gen_lane_center(void);
        std::vector<fsdmap::KeyPose *> pos_sample_list; // 这段RoadSegment包含的Link点
        std::vector<LaneGroupPtr> lane_group_list;

    private:
    
    };
    using RoadSegmentPtr = std::shared_ptr<RoadSegment>;
}