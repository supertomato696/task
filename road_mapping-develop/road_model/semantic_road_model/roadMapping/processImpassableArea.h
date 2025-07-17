//
//
//

#ifndef ROADMAPPING_PROCESSIMPASSABLEAREA_H
#define ROADMAPPING_PROCESSIMPASSABLEAREA_H
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pclPtType.h"
#include "Base/Array.h"
#include "Geometries/Coordinate.h"

namespace RoadMapping
{

    class processImpassableArea
    {
    public:
        void run_impassable_area_process(std::string pcdPath, std::string outputDir);

        // 提取圆形的不可通行区域
        void run_circle_area(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, int type, std::string output_dir);
        void run_polygon_area(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, int type, std::string output_dir);

    private:
        void pcl_type_trans(const pcl::PointCloud<MyColorPointType>::Ptr &cloudPtr, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloudptr);
        void anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts); // 将多边形转换为逆时针
    };
} // namespace RoadMapping
#endif // ROADMAPPING_processImpassableArea_H