#pragma once

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <string>
#include <Eigen/Dense>
#include "Utils.h"

namespace RoadMapping{

class ProcessRoadMark{
public:
    static void run(Eigen::Vector3d init_llh,
                    std::vector<KeyPose> &keyPoses,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_keyPose,
                    std::string inputDir, std::string outputDir);
};

} //RoadMapping