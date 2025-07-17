#pragma once
#ifndef LOAM_FEATURE_EXTRATION_HPP_
#define LOAM_FEATURE_EXTRATION_HPP_

#include <algorithm>
#include <vector>
#include "types.h"

using LidarPointType = RawPointType;
using LidarPointCloud = pcl::PointCloud<LidarPointType>;
using LidarPointCloudPtr = typename LidarPointCloud::Ptr;

template <typename PointT>
class VoxelFilter
{
public:
    using PointCloud = typename pcl::PointCloud<PointT>;
    using PointCloudPtr = typename PointCloud::Ptr;

public:
    VoxelFilter() {}
    explicit VoxelFilter(float resolution) : inv_resolution_(1.0f / resolution) {}

    void setLeafSize(float resolution) { inv_resolution_ = 1.0f / resolution; }

    void filter(const PointCloudPtr &point_cloud, PointCloud &output)
    {
        const auto &points_used = RandomizedVoxelFilterIndices(point_cloud);
        if (point_cloud.get() == &output)
        { // cloud_in = cloud_out
            PointCloud output_temp;
            output_temp.reserve(size_);
            for (size_t i = 0; i < point_cloud->size(); i++)
            {
                if (points_used[i])
                {
                    output_temp.push_back(point_cloud->points[i]);
                }
            }
            pcl::copyPointCloud(output_temp, output);
        }
        else
        {
            output.clear();
            output.reserve(size_);
            for (size_t i = 0; i < point_cloud->size(); i++)
            {
                if (points_used[i])
                {
                    output.push_back(point_cloud->points[i]);
                }
            }
        }
    }

    PointCloudPtr filter(const PointCloudPtr &point_cloud)
    {
        PointCloudPtr results(new PointCloud());
        filter(point_cloud, *results);
        return results;
    }

private:
    uint64_t GetVoxelCellIndex(const Eigen::Vector3f &point)
    {
        const Eigen::Array3f index = point.array() * inv_resolution_;
        const uint64_t x = std::lround(index.x());
        const uint64_t y = std::lround(index.y());
        const uint64_t z = std::lround(index.z());
        return (x << 42) + (y << 21) + z;
    }

    std::vector<bool> RandomizedVoxelFilterIndices(const PointCloudPtr &point_cloud)
    {
        // According to https://en.wikipedia.org/wiki/Reservoir_sampling
        std::minstd_rand0 generator;
        std::unordered_map<uint64_t, std::pair<int, int>> voxel_count_and_point_index;
        for (size_t i = 0; i < point_cloud->size(); i++)
        {
            auto &voxel =
                voxel_count_and_point_index[GetVoxelCellIndex(point_cloud->points[i].getVector3fMap())];
            voxel.first++;
            if (voxel.first == 1)
            {
                voxel.second = i;
            }
            else
            {
                std::uniform_int_distribution<> distribution(1, voxel.first);
                if (distribution(generator) == voxel.first)
                {
                    voxel.second = i;
                }
            }
        }

        size_ = 0;
        std::vector<bool> points_used(point_cloud->size(), false);
        for (const auto &voxel_and_index : voxel_count_and_point_index)
        {
            points_used[voxel_and_index.second.second] = true;
            size_++;
        }
        return points_used;
    }

private:
    float inv_resolution_;
    size_t size_;
};

class LoamFeatureExtraction
{
public:
    int N_SCAN = 128; //
    int Horizon_SCAN = 1200;
    float lidarMinRange = 5.0;
    float lidarMaxRange = 180;
    int kEdgeNum = 5;
    float kNeighborSqrtDistance = 0.16;
    float edgeThreshold = 2000.0;
    float surfThreshold = 5;

    LoamFeatureExtraction()
    {

        raw_cloud.reset(new LidarPointCloud);
        points_feature_types.resize(2);
        points_feature_types[0] = FeatureType::SHARP;
        points_feature_types[1] = FeatureType::FLAT;
        feature_clouds.resize(2);
        for (auto &c : feature_clouds)
            c.reset(new LidarPointCloud);
    }

    std::vector<LidarPointCloudPtr> Extract(const LidarPointCloudPtr &lidar_scan)
    {
        SortByRingTime(lidar_scan);
        CalculateSmoothness();
        markOccludedPoints();
        ExtractFeatures();

        return feature_clouds;
    }

private:
    // 按照 Ring 排序, 同一个 Ring 中的点云按扫描顺序排序
    void SortByRingTime(const LidarPointCloudPtr &lidar_scan)
    {
        std::vector<LidarPointCloud> scan_rings(N_SCAN);
        for (auto &ring : scan_rings)
            ring.reserve(Horizon_SCAN);

        for (const auto &pt : lidar_scan->points)
        {
            scan_rings[pt.ring].push_back(pt);
            // 原始数据的时间不对, 这里用水平的角度来判断扫描顺序
            scan_rings[pt.ring].back().time = -atan2(pt.y, pt.x) * 180.0f / M_PI;
        }

        for (auto &ring : scan_rings)
        {
            std::sort(ring.points.begin(), ring.points.end(),
                      [](const auto &p1, const auto &p2)
                      { return p1.time < p2.time; });
        }

        raw_cloud->clear();
        ring_ranges_.assign(N_SCAN, std::make_pair(0, 0));
        for (int r = 0; r < N_SCAN; r++)
        {
            ring_ranges_[r].first = raw_cloud->size() + 5;
            *raw_cloud += scan_rings[r];
            ring_ranges_[r].second = raw_cloud->size() - 1 - 5;
        }
    }

    void CalculateSmoothness()
    {
        int cloud_sz = raw_cloud->size();
        points_feature_info.resize(cloud_sz);
        points_index_curvature.resize(cloud_sz);

        // calculate range
        for (int i = 0; i < cloud_sz; i++)
        {
            const Eigen::Vector3f &pt = raw_cloud->points[i].getVector3fMap();
            points_feature_info[i].range = pt.norm();

            if (points_feature_info[i].range < lidarMinRange ||
                points_feature_info[i].range > lidarMaxRange)
            {
                points_feature_info[i].picked = 1;
            }
            else
            {
                points_feature_info[i].picked = 0;
            }
        }

        // calculate curvature
        for (int i = 5; i < cloud_sz - 5; i++)
        {
            float diffRange =
                points_feature_info[i - 5].range + points_feature_info[i - 4].range + points_feature_info[i - 3].range +
                points_feature_info[i - 2].range + points_feature_info[i - 1].range - points_feature_info[i].range * 10 +
                points_feature_info[i + 1].range + points_feature_info[i + 2].range + points_feature_info[i + 3].range +
                points_feature_info[i + 4].range + points_feature_info[i + 5].range;

            points_feature_info[i].curvature = diffRange * diffRange;
            points_feature_info[i].type = 0;

            // cloud_smoothness_ for sorting
            points_index_curvature[i].value = points_feature_info[i].curvature;
            points_index_curvature[i].idx = i;
        }
    }

    void markOccludedPoints()
    {
        int cloud_sz = raw_cloud->points.size();
        for (int i = 5; i < cloud_sz - 5; ++i)
        {
            // parallel beam
            float diff1 = std::abs(points_feature_info[i - 1].range - points_feature_info[i].range);
            float diff2 = std::abs(points_feature_info[i + 1].range - points_feature_info[i].range);

            if (diff1 > 0.02 * points_feature_info[i].range || diff2 > 0.02 * points_feature_info[i].range)
            {
                points_feature_info[i].picked = 1;
            }
            float time_diff_1 = std::abs(raw_cloud->points[i - 1].time - raw_cloud->points[i].time);
            float time_diff_2 = std::abs(raw_cloud->points[i + 1].time - raw_cloud->points[i].time);
            if (time_diff_1 > 0.2 || time_diff_2 > 0.2)
            {
                points_feature_info[i].picked = 1;
            }
        }
    }

    void SuppressNeighbor(int idx)
    {
        for (int i = -5; i <= 5; i++)
        {
            if (i == 0)
                continue;

            const int offset = i > 0 ? 1 : -1;
            float diff_x = raw_cloud->points[idx + i].x - raw_cloud->points[idx + i - offset].x;
            float diff_y = raw_cloud->points[idx + i].y - raw_cloud->points[idx + i - offset].y;
            float diff_z = raw_cloud->points[idx + i].z - raw_cloud->points[idx + i - offset].z;
            if (SquaredDistance(diff_x, diff_y, diff_z) > kNeighborSqrtDistance)
                continue;
            points_feature_info[idx + i].picked = 1;
        }
    }

    float SquaredDistance(float dx, float dy, float dz)
    {
        return std::pow(dx, 2) + std::pow(dy, 2) + std::pow(dz, 2);
    }

    void ExtractFeatures()
    {
        for (auto &c : feature_clouds)
            c->clear();
        auto &corner_cloud = feature_clouds[0];
        auto &surface_cloud = feature_clouds[1];

        for (int i = 0; i < N_SCAN; i++)
        {
            const auto scan_start_index = ring_ranges_[i].first;
            const auto scan_end_index = ring_ranges_[i].second;
            if (scan_end_index - scan_start_index <= 6)
                continue;

            for (int j = 0; j < 6; j++)
            {
                const int sp = scan_start_index + (scan_end_index - scan_start_index) * j / 6;
                const int ep = scan_start_index + (scan_end_index - scan_start_index) * (j + 1) / 6 - 1;
                if (sp >= ep)
                    continue;

                std::sort(points_index_curvature.begin() + sp, points_index_curvature.begin() + ep + 1,
                          [](const auto &a, const auto &b)
                          { return a.value < b.value; });

                int largest_picked_num = 0;
                for (int k = ep; k >= sp; k--)
                {
                    const int idx = points_index_curvature[k].idx;
                    if (points_feature_info[idx].picked == 0 && points_feature_info[idx].curvature > edgeThreshold)
                    {
                        largest_picked_num++;
                        if (largest_picked_num <= kEdgeNum)
                        {
                            points_feature_info[idx].type = 1;
                            corner_cloud->push_back(raw_cloud->points[idx]);
                            // corner_cloud->back().debug = points_feature_info[idx].curvature;
                            points_feature_info[idx].picked = 1;
                            SuppressNeighbor(idx);
                        }
                        else
                        {
                            break;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    int idx = points_index_curvature[k].idx;
                    if (points_feature_info[idx].picked == 0 && points_feature_info[idx].curvature < surfThreshold)
                    {
                        points_feature_info[idx].type = -1;
                        surface_cloud->push_back(raw_cloud->points[k]);
                        // surface_cloud->back().debug = points_feature_info[idx].curvature;
                        points_feature_info[idx].picked = 1;
                        // SuppressNeighbor(idx);
                    }
                }
            }
        }
    }

private:
    struct IndexCurvature
    {
        size_t idx = 0;
        float value = 1e7; // 曲率
    };

    struct FeatureInfo
    {
        float range = 0.0f;
        float curvature = 0.0f; // 曲率
        int picked = 0;         // ==1 表示此点被排除
        int type = 0;
    };

    LidarPointCloudPtr raw_cloud;                   // 原始点云
    std::vector<LidarPointCloudPtr> feature_clouds; // 储存得到的特征点云

    std::vector<std::pair<int, int>> ring_ranges_;
    std::vector<FeatureInfo> points_feature_info;
    std::vector<IndexCurvature> points_index_curvature;
    std::vector<FeatureType> points_feature_types;
};

#endif