//
//
//

#ifndef ROADMAPPING_PROCESSPRECEPTIONLINE_H
#define ROADMAPPING_PROCESSPRECEPTIONLINE_H

#include <string>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <map>
#include <set>
#include <cmath>
#include <string>
#include <vector>
#include "pclPtType.h"
namespace RoadMapping
{
    struct Frame
    {
        std::map<std::string, pcl::PointCloud<MyColorPointType>> point_dict; // 容器内点以link方式组织
    };

    struct Trail
    {
        std::map<uint64_t, Eigen::Matrix4d, std::less<uint64_t>, Eigen::aligned_allocator<std::pair<const uint64_t, Eigen::Matrix4d>>> stamp_ms_T_world_veh; // 存储一个trail内所有时刻车体到utm局部坐标系转换参数
        std::map<uint64_t, std::string> stamp_ms_link;                                                                                                       // 存储每帧与link关系
    };
    struct LineType
    {
        // line types
        const uint64_t LineType_Line_Ramp = 0x100;   // 256  label = 1
        const uint64_t LineType_Line_Double = 0x200; // 512 label = 2
        const uint64_t LineType_Line_Dash = 0x400;   // 1024  label = 3
        const uint64_t LineType_Line_Solid = 0x800;  // 2048  label = 4
        // line color
        const uint64_t LineType_Line_White = 0x1000;  // 4096
        const uint64_t LineType_Line_Yellow = 0x2000; // 8192
        const uint64_t LineType_Line_Blue = 0x4000;   // 16384,reserved
        const uint64_t LineType_Line_Green = 0x8000;  // reserved

        const uint64_t LineType_Line_Fence = 0x10000; // 1 << 16, for fence
    };

    class processPPline
    {
    public:
        static void ReadPerceptionLinePts(std::string dataDir, std::string trailid, std::string outDir); // 获取感知结果的车道标线
        static Eigen::Matrix4d qab_tab_to_Tab(const Eigen::Quaterniond &q,
                                              const Eigen::Vector3d &t)
        {
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(q);
            T.pretranslate(t);
            return T.matrix();
        }

        static void GetPoints(Eigen::Vector4d coeffs, int type, Eigen::Matrix4d T_world_veh, pcl::PointCloud<MyColorPointType> &ptsCloud); // 从参数解析感知结果
    };

} // namespace RoadMapping

#endif // ROADMAPPING_PROCESSPRECEPTIONLINE_H
