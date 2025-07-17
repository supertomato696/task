#pragma once
#ifndef PCA_FEATURE_EXTRATION_HPP_
#define PCA_FEATURE_EXTRATION_HPP_
#include <algorithm>
#include <vector>
#include "cfilter.hpp"

template <typename ScanPointT>
class PcaFeatureExtraction
{
public:
    PcaFeatureExtraction() {}

    void Extract(const typename pcl::PointCloud<ScanPointT>::Ptr &lidar)
    {
        ConvertToCblock(lidar); // convert to cloudblock_Ptr
        ExtractFeatures();
        UpdateFeatures();
    }

private:
    void ConvertToCblock(const typename pcl::PointCloud<ScanPointT>::Ptr &lidar)
    {
        static int cnt = 0;
        lo::cloudblock_Ptr cblock_in = lo::cloudblock_Ptr(new lo::cloudblock_t());
        cblock_in->filename = std::to_string(cnt);
        cblock_in->unique_id = cnt;
        cblock_in->is_single_scanline = false;
        cblock_in->pose_lo.setIdentity();
        cblock_in->pose_gt.setIdentity();
        pcl::copyPointCloud(*lidar, *cblock_in->pc_raw);

        cblock_source = cblock_in;
        cnt++;
    }

    void ExtractFeatures()
    {
        static float gf_max_h = 0.5;
        static float cloud_down_res = 0.0;
        static int ground_down_rate = 1;
        static int nonground_down_rate = 1;
        static int gf_down_down_rate_ground = 1;

        bool apply_dist_filter = false;
        float min_dist_used = 1.0;
        float max_dist_used = 120.0;
        float gf_grid_resolution = 3.0;
        float gf_max_grid_height_diff = 0.3;
        float gf_neighbor_height_diff = 1.5;
        float pca_neigh_r = 1;
        int pca_neigh_k = 50;
        float pca_linearity_thre = 0.6;
        float pca_planarity_thre = 0.6;
        float pca_curvature_thre = 0.12;
        float linearity_thre_down = 0.75;
        float planarity_thre_down = 0.75;

        static float beam_direction_sin = std::sin(25 / 180.0 * M_PI);
        static float pillar_direction_sin = std::sin(60 / 180.0 * M_PI);
        static float facade_normal_sin = std::sin(30 / 180.0 * M_PI);
        static float roof_normal_sin = std::sin(75 / 180.0 * M_PI);

        if (apply_dist_filter)
            cfilter.dist_filter(cblock_source->pc_raw, min_dist_used, max_dist_used);

        cfilter.extract_semantic_pts(cblock_source,
                                     cloud_down_res,
                                     gf_grid_resolution,
                                     gf_max_grid_height_diff,
                                     gf_neighbor_height_diff,
                                     gf_max_h,
                                     ground_down_rate,
                                     nonground_down_rate,
                                     pca_neigh_r,
                                     pca_neigh_k,
                                     pca_linearity_thre,
                                     pca_planarity_thre,
                                     pca_curvature_thre,
                                     linearity_thre_down,
                                     planarity_thre_down,
                                     true,                     // use_distance_adaptive_pca
                                     2,                        // distance_inverse_sampling_method
                                     15.0,                     // standard_distance
                                     0,                        // estimate_ground_normal_method
                                     2.0,                      // normal_estimation_radius
                                     false,                    // use_adpative_parameters
                                     false,                    // apply_scanner_filter
                                     false,                    // extract_curb_or_not
                                     2,                        // extract_vertex_points_method
                                     10,                       // gf_grid_pt_num_thre
                                     0,                        // gf_reliable_neighbor_grid_thre
                                     gf_down_down_rate_ground, // gf_down_down_rate_ground
                                     8,                        // pca_neighbor_k_min = 8
                                     2,                        // pca_down_rate
                                     FLT_MAX,                  // intensity_thre
                                     pillar_direction_sin,
                                     beam_direction_sin,
                                     roof_normal_sin,
                                     facade_normal_sin,
                                     false,   // sharpen_with_nms_on
                                     false,   // fixed_num_downsampling
                                     300,     // ground_down_fixed_num
                                     100,     // pillar_down_fixed_num
                                     400,     // facade_down_fixed_num
                                     100,     // beam_down_fixed_num
                                     0,       // roof_down_fixed_num
                                     10000,   // unground_down_fixed_num
                                     FLT_MAX, // beam_height_max
                                     2.0,     // roof_height_min
                                     1.5,     // approx_scanner_height
                                     -6.0,    // underground_thre
                                     0.3, 
                                     false, // semantic_assisted
                                     false, // apply_roi_filtering
                                     0.0,
                                     0.0);

        // LI_INFO(
        //     "Ground: [{}|{}] Pillar: [{}|{}] Beam: [{}|{}] Facade: [{}|{}] Roof: [{}|{}] Vertex: [{}]",
        //     cblock_source->pc_ground->points.size(), cblock_source->pc_ground_down->points.size(),
        //     cblock_source->pc_pillar->points.size(), cblock_source->pc_pillar_down->points.size(),
        //     cblock_source->pc_beam->points.size(), cblock_source->pc_beam_down->points.size(),
        //     cblock_source->pc_facade->points.size(), cblock_source->pc_facade_down->points.size(),
        //     cblock_source->pc_roof->points.size(), cblock_source->pc_roof_down->points.size(),
        //     cblock_source->pc_vertex->points.size());
    }
    void UpdateFeatures()
    {
        feature_clouds.resize(6);
        for (auto &c : feature_clouds)
        {
            c.reset(new pcl::PointCloud<pcl::PointXYZINormal>);
            c->clear();
        }

        auto &ground_cloud = feature_clouds[0];
        auto &facade_cloud = feature_clouds[1];
        auto &pillar_cloud = feature_clouds[2];
        auto &beam_cloud = feature_clouds[3];
        auto &roof_cloud = feature_clouds[4];
        auto &vertex_cloud = feature_clouds[5];

        for (auto &p : cblock_source->pc_ground_down->points)
            p.curvature = static_cast<int>(0); // 地面
        for (auto &p : cblock_source->pc_facade_down->points)
            p.curvature = static_cast<int>(1); // 立面
        for (auto &p : cblock_source->pc_pillar_down->points)
            p.curvature = static_cast<int>(2); // 立柱
        for (auto &p : cblock_source->pc_beam_down->points)
            p.curvature = static_cast<int>(3); // 横梁
        for (auto &p : cblock_source->pc_roof_down->points)
            p.curvature = static_cast<int>(4); // 屋顶
        for (auto &p : cblock_source->pc_vertex->points)
            p.curvature = static_cast<int>(5); // 顶点

        *ground_cloud += *cblock_source->pc_ground;
        *facade_cloud += *cblock_source->pc_facade;
        *pillar_cloud += *cblock_source->pc_pillar;
        *beam_cloud += *cblock_source->pc_beam;
        *roof_cloud += *cblock_source->pc_roof;
        *vertex_cloud += *cblock_source->pc_vertex;
    }

public:
    std::vector<pcl::PointCloud<pcl::PointXYZINormal>::Ptr> feature_clouds; // 储存得到的特征点云

private:
    lo::cloudblock_Ptr cblock_source;
    lo::CFilter<Point_T> cfilter;
};

#endif