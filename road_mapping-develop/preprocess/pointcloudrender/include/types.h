#ifndef __PTYPES_H__
#define __PTYPES_H__

#include <Eigen/Geometry>
#include <array>
#include <random>
#include <vector>
#include <thread>
#include <istream>
#include <unordered_set>
#include <map>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <malloc.h>

enum ObstacleType
{
    UNKNOWN = 0,
    CAR = 1,
    TRUCK = 2,
    MOTORCYCLE = 3,
    BICYCLE = 4,
    PEDESTRIAN = 5,
    CONE = 6,
    BARREL = 7,
    BARRIER = 8,
    SAFETY_TRIANGLE = 9,
    PARKING_LOCK = 10,
    SPACE_LIMITER = 11,
    TRICYCLE = 12,
    SPECIALVEHICLE = 13,
    BUS = 14,
    MINIVEHICLE = 15,
    CONSTRUCTION_VEHICLE = 16,
    UNKNOWN_MOVABLE = 17,
    UNKNOWN_UNMOVABLE = 18
};

enum SegmentationType
{
    CLASS_TYPE_0 = 0,     // road, pothole, parking_space
    CLASS_TYPE_10 = 10,   // sidewalk, building, vegetation, terrain, sky, parking_lock, column
    CLASS_TYPE_20 = 20,   // fence
    CLASS_TYPE_30 = 30,   // pole
    CLASS_TYPE_40 = 40,   // traffic_light, traffic_sign, traffic_guide_sign
    CLASS_TYPE_50 = 50,   // person, rider
    CLASS_TYPE_60 = 60,   // car, truck, bus, train
    CLASS_TYPE_70 = 70,   // motorcycle, bicycle, tricycle
    CLASS_TYPE_80 = 80,   // traffic_lane
    CLASS_TYPE_90 = 90,   // crosswalk_line
    CLASS_TYPE_100 = 100, // traffic_arrow
    CLASS_TYPE_110 = 110, // sign_line, slow_down_triangle, speed_sign, diamond, bicycle_sign, parking_line
    CLASS_TYPE_120 = 120, // guide_line
    CLASS_TYPE_130 = 130, // traffic_cone
    CLASS_TYPE_140 = 140, // stop_line
    CLASS_TYPE_150 = 150  // speed_bump
};

#endif