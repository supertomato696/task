#ifndef INCLUDE_TYPES_
#define INCLUDE_TYPES_

#include <array>
#include <istream>
#include <map>
#include <random>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "pcl/ModelCoefficients.h"
#include "pcl/common/io.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/impl/conditional_removal.hpp"
#include "pcl/filters/impl/extract_indices.hpp"
#include "pcl/filters/impl/filter_indices.hpp"
#include "pcl/filters/impl/radius_outlier_removal.hpp"
#include "pcl/filters/impl/statistical_outlier_removal.hpp"
#include "pcl/filters/impl/voxel_grid.hpp"
#include "pcl/io/pcd_io.h"
#include "pcl/kdtree/impl/kdtree_flann.hpp"
#include "pcl/impl/pcl_base.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/registration/gicp.h"
#include "pcl/registration/icp.h"
#include "pcl/search/impl/kdtree.hpp"
#include "pcl/search/impl/organized.hpp"
#include "pcl/search/impl/search.hpp"

#include "util.h"
#include "json.h"

// clang-format off
struct EIGEN_ALIGN16 MyColorPointType
{
    PCL_ADD_POINT4D;
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY;
    uint16_t label;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyColorPointType,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint16_t, label, label))
// clang-format on

#endif