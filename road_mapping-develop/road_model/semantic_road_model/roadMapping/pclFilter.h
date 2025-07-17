//
//
//

#ifndef ROADMAPPING_PCLFILTER_H
#define ROADMAPPING_PCLFILTER_H
#include "pclPtType.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>

namespace pclFilter
{

    /////统计滤波
    void StatisticalOutlierRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, int K, float diff);
    void StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, int K, float diff);
    void StatisticalOutlierRemoval(pcl::PointCloud<ConRmPointType>::Ptr cloud_input, pcl::PointCloud<ConRmPointType>::Ptr &cloud_filtered, int K, float diff);

    /////均匀下采样
    void UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, float R);
    /////体素滤波
    void VoxelGridSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, const float &leaf_size);
    /////球半径滤波
    void RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, float R, int K);
    ////条件滤波
    void ConditionalRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, std::string ziduan, int value);
    void ConditionalRemoval(pcl::PointCloud<PointLabel>::Ptr cloud_input, pcl::PointCloud<PointLabel>::Ptr &cloud_filtered, std::string ziduan, int value);
    void ConditionalRemoval_GE_LE(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, std::string ziduan, int minvalue, int maxvalue);
    void ConditionalRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered, std::string ziduan, int value);
    void ConditionalRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<ConRmPointType>::Ptr &cloud_filtered, std::string ziduan, int value);
} // namespace CH
#endif // ROADMAPPING_PCLFILTER_H
