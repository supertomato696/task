#pragma once

#include <Eigen/Core>
#include <string>
#include "Base/Array.h"
#include "Geometries/Coordinate.h"

struct KeyPose
{
    double timestamp = 0.f; //in second
    int frame_id = 0; //img id
    std::string frameIdStr; //timestamp+id
    double lat = 0.f; //in rad
    double lon = 0.f; //in rad
    double alt = 0.f; //in meter
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); 
};


