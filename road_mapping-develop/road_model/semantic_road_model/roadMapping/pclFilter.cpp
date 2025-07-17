//
//
//

#include "pclFilter.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>

void pclFilter::StatisticalOutlierRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, int K, float diff)
{
    std::cout << "->正在进行统计滤波..." << std::endl;
    pcl::StatisticalOutlierRemoval<MyColorPointType> sor; // 创建滤波器对象
    sor.setInputCloud(cloud_input);                       // 设置待滤波点云
    sor.setMeanK(K);                                      // 设置查询点近邻点的个数
    sor.setStddevMulThresh(diff);                         // 设置标准差乘数，来计算是否为离群点的阈值
    // sor.setNegative(true);							//默认false，保存内点；true，保存滤掉的离群点
    sor.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::StatisticalOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, int K, float diff)
{
    std::cout << "->正在进行统计滤波..." << std::endl;
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor; // 创建滤波器对象
    sor.setInputCloud(cloud_input);                    // 设置待滤波点云
    sor.setMeanK(K);                                   // 设置查询点近邻点的个数
    sor.setStddevMulThresh(diff);                      // 设置标准差乘数，来计算是否为离群点的阈值
    // sor.setNegative(true);							//默认false，保存内点；true，保存滤掉的离群点
    sor.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::RadiusOutlierRemoval(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, float R, int K)
{
    std::cout << "->正在进行半径滤波..." << std::endl;
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror; // 创建滤波器对象
    ror.setInputCloud(cloud_input);               // 设置待滤波点云
    ror.setRadiusSearch(R);                       // 设置查询点的半径范围
    ror.setMinNeighborsInRadius(K);               // 设置判断是否为离群点的阈值，即半径内至少包括的点数
    // ror.setNegative(true);						//默认false，保存内点；true，保存滤掉的外点
    ror.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::UniformSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, float R)
{
    std::cout << "->正在均匀采样..." << std::endl;
    pcl::UniformSampling<pcl::PointXYZ> us; // 创建滤波器对象
    us.setInputCloud(cloud_input);          // 设置待滤波点云
    us.setRadiusSearch(R);                  // 设置滤波球体半径
    us.filter(*cloud_filtered);             // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::VoxelGridSampling(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_input, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_filtered, const float &leaf_size)
{
    std::cout << "->正在体素滤波..." << std::endl;
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_input);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);
}

void pclFilter::ConditionalRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, std::string ziduan, int value)
{
    std::cout << "->正在进行条件滤波..." << std::endl;
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<MyColorPointType>::Ptr range_cond(new pcl::ConditionAnd<MyColorPointType>()); // 创建条件定义对象range_cond
    // 为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<MyColorPointType>::ConstPtr(new pcl::FieldComparison<MyColorPointType>(ziduan, pcl::ComparisonOps::EQ, value))); // 添加在x字段上大于 -0.1 的比较算子
    pcl::ConditionalRemoval<MyColorPointType> cr;                                                                                                                   // 创建滤波器对象
    cr.setCondition(range_cond);                                                                                                                                    // 用条件定义对象初始化
    cr.setInputCloud(cloud_input);                                                                                                                                  // 设置待滤波点云
    // cr.setKeepOrganized(true);				//设置保持点云的结构
    // cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::ConditionalRemoval(pcl::PointCloud<PointLabel>::Ptr cloud_input, pcl::PointCloud<PointLabel>::Ptr &cloud_filtered, std::string ziduan, int value)
{
    std::cout << "->正在进行条件滤波..." << std::endl;
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<PointLabel>::Ptr range_cond(new pcl::ConditionAnd<PointLabel>()); // 创建条件定义对象range_cond
    // 为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<PointLabel>::ConstPtr(new pcl::FieldComparison<PointLabel>(ziduan, pcl::ComparisonOps::EQ, value))); // 添加在x字段上大于 -0.1 的比较算子
    pcl::ConditionalRemoval<PointLabel> cr;                                                                                                                   // 创建滤波器对象
    cr.setCondition(range_cond);                                                                                                                                    // 用条件定义对象初始化
    cr.setInputCloud(cloud_input);                                                                                                                                  // 设置待滤波点云
    // cr.setKeepOrganized(true);				//设置保持点云的结构
    // cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::ConditionalRemoval_GE_LE(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<MyColorPointType>::Ptr &cloud_filtered, std::string ziduan, int minvalue, int maxvalue)
{
    std::cout << "->正在进行条件滤波..." << std::endl;
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<MyColorPointType>::Ptr range_cond(new pcl::ConditionAnd<MyColorPointType>()); // 创建条件定义对象range_cond
    // 为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<MyColorPointType>::ConstPtr(new pcl::FieldComparison<MyColorPointType>(ziduan, pcl::ComparisonOps::GE, minvalue))); // 添加大于或等于 value的比较算子
    range_cond->addComparison(pcl::FieldComparison<MyColorPointType>::ConstPtr(new pcl::FieldComparison<MyColorPointType>(ziduan, pcl::ComparisonOps::LE, maxvalue))); // 添加小于或等于value的比较算子
    pcl::ConditionalRemoval<MyColorPointType> cr;                                                                                                                      // 创建滤波器对象
    cr.setCondition(range_cond);                                                                                                                                       // 用条件定义对象初始化
    cr.setInputCloud(cloud_input);                                                                                                                                     // 设置待滤波点云
    // cr.setKeepOrganized(true);				//设置保持点云的结构
    // cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::ConditionalRemoval(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input, pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud_filtered, std::string ziduan, int value)
{
    std::cout << "->正在进行条件滤波..." << std::endl;
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<pcl::PointXYZI>::Ptr range_cond(new pcl::ConditionAnd<pcl::PointXYZI>()); // 创建条件定义对象range_cond
    // 为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<pcl::PointXYZI>::ConstPtr(new pcl::FieldComparison<pcl::PointXYZI>(ziduan, pcl::ComparisonOps::EQ, value))); // 添加在x字段上大于 -0.1 的比较算子
    pcl::ConditionalRemoval<pcl::PointXYZI> cr;                                                                                                                 // 创建滤波器对象
    cr.setCondition(range_cond);                                                                                                                                // 用条件定义对象初始化
    cr.setInputCloud(cloud_input);                                                                                                                              // 设置待滤波点云
    // cr.setKeepOrganized(true);				//设置保持点云的结构
    // cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::StatisticalOutlierRemoval(pcl::PointCloud<ConRmPointType>::Ptr cloud_input, pcl::PointCloud<ConRmPointType>::Ptr &cloud_filtered, int K, float diff)
{
    std::cout << "->正在进行统计滤波..." << std::endl;
    pcl::StatisticalOutlierRemoval<ConRmPointType> sor; // 创建滤波器对象
    sor.setInputCloud(cloud_input);                       // 设置待滤波点云
    sor.setMeanK(K);                                      // 设置查询点近邻点的个数
    sor.setStddevMulThresh(diff);                         // 设置标准差乘数，来计算是否为离群点的阈值
    // sor.setNegative(true);							//默认false，保存内点；true，保存滤掉的离群点
    sor.filter(*cloud_filtered); // 执行滤波，保存滤波结果于cloud_filtered
}

void pclFilter::ConditionalRemoval(pcl::PointCloud<MyColorPointType>::Ptr cloud_input, pcl::PointCloud<ConRmPointType>::Ptr &cloud_filtered, std::string ziduan, int value)
{
    std::cout << "->正在进行条件滤波, ziduan: " << ziduan << ", value: " << value << std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr cloud_middle(new pcl::PointCloud<MyColorPointType>);
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<MyColorPointType>::Ptr range_cond(new pcl::ConditionAnd<MyColorPointType>()); // 创建条件定义对象range_cond
    // 为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<MyColorPointType>::ConstPtr(new pcl::FieldComparison<MyColorPointType>(ziduan, pcl::ComparisonOps::EQ, value))); // 添加在x字段上大于 -0.1 的比较算子
    pcl::ConditionalRemoval<MyColorPointType> cr;                                                                                                                   // 创建滤波器对象
    cr.setCondition(range_cond);                                                                                                                                    // 用条件定义对象初始化
    cr.setInputCloud(cloud_input);                                                                                                                                  // 设置待滤波点云
    // cr.setKeepOrganized(true);				//设置保持点云的结构
    // cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_middle); // 执行滤波，保存滤波结果于cloud_middle

    for (size_t i = 0; i < cloud_middle->points.size(); i++)
    {
        ConRmPointType point;
        point.x = cloud_middle->points[i].x;
        point.y = cloud_middle->points[i].y;
        point.z = cloud_middle->points[i].z;
        point.intensity = cloud_middle->points[i].intensity;
        point.cloud_bev_label_score = cloud_middle->points[i].cloud_bev_label_score;
        point.cloud_bev_center_line_score = cloud_middle->points[i].cloud_bev_center_line_score;
        if (ziduan == "cloud_bev_label")
        {
            point.cloud_bev_label = cloud_middle->points[i].cloud_bev_label;
            point.cloud_bev_shape = cloud_middle->points[i].cloud_bev_label_shape;
            point.cloud_bev_color = cloud_middle->points[i].cloud_bev_label_color;
        } else if (ziduan == "cloud_bev_label_1") {
            point.cloud_bev_label = cloud_middle->points[i].cloud_bev_label_1;
            point.cloud_bev_shape = cloud_middle->points[i].cloud_bev_label_1_shape;
            point.cloud_bev_color = cloud_middle->points[i].cloud_bev_label_1_color;
        } else if (ziduan == "cloud_bev_label_2") {
            point.cloud_bev_label = cloud_middle->points[i].cloud_bev_label_2;
            point.cloud_bev_shape = cloud_middle->points[i].cloud_bev_label_2_shape;
            point.cloud_bev_color = cloud_middle->points[i].cloud_bev_label_2_color;
        } else if (ziduan == "cloud_bev_center_line") {
            point.cloud_bev_label = cloud_middle->points[i].cloud_bev_center_line;
            point.cloud_bev_shape = cloud_middle->points[i].cloud_bev_center_line_shape;
            point.cloud_bev_color = cloud_middle->points[i].cloud_bev_center_line_color;
        } else {
            // do nothing
        }
        cloud_filtered->points.push_back(point);
    }
}