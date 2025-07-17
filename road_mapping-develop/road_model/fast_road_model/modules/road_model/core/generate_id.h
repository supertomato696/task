#pragma once
#include <iostream>
#include <set>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
namespace fast_road_model
{
    class GenerateID
    {
    public:
        enum TYPE
        {
            LANE_GROUP = 0,
            ROAD_BOUNDARY = 1,
            LANE_BOUNDARY = 2,
            LANE = 3,
            STOP_LINE = 4,
            ARROW = 5,
            CROSS_WALK = 6,
            JUNCTION = 7,
            TRAFFIC_LIGHT = 8
        };
        uint64_t  update(const TYPE &type)
        {
            auto &id = id_list[type];
            id++;
            return id;
        }
        uint64_t get_id(const TYPE &type) const
        {
            const auto &id = id_list[type];
            return id;
        }

        static GenerateID &getInstance()
        {
            return instance;
        }

    private:
        static GenerateID instance;
        std::vector<uint64_t> id_list = {1 * 1E8, 2 * 1E8, 3 * 1E8, 4 * 1E8, 5 * 1E8 + 0 * 1E7, 5 * 1E8 + 1 * 1E7, 5 * 1E8 + 2 * 1E7, 5 * 1E8 + 3 * 1E7, 6 * 1E8};
    };
    GenerateID GenerateID::instance;
}