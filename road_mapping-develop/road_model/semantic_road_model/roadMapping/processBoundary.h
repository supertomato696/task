#pragma once
#include "cluster.h"

namespace RoadMapping{

class ProcessBoundary{
public:
    static void run(Eigen::Vector3d init_llh,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_keyPose,
                    std::string outputDir);

};

} //namespace RoadMapping

