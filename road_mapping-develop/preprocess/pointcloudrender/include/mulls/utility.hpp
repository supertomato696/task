//
// This file is a General Definition and Tool for Point Cloud Processing based
// on PCL. Dependent 3rd Libs: PCL (>1.7) By Yue Pan et al.
//
#ifndef _INCLUDE_UTILITY_HPP_
#define _INCLUDE_UTILITY_HPP_

// PCL
#include <pcl/ModelCoefficients.h>
#include <pcl/common/common.h>
#include <pcl/features/fpfh.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/point_types.h>
#include <pcl/registration/transformation_estimation_svd.h>

// Eigen
#include <omp.h>
#include <time.h>
#include <chrono>
#include <limits>
#include <list>
#include <vector>

#include <Eigen/Core>

#define max_(a, b) (((a) > (b)) ? (a) : (b))
#define min_(a, b) (((a) < (b)) ? (a) : (b))

// using namespace std;

// TypeDef
typedef pcl::PointXYZINormal Point_T; // mind that 'curvature' here is used as
                                      // ring number for spining scanner

/**
// pcl::PointXYZINormal member variables
// x,y,z: 3d coordinates
// intensity: relective intensity
// normal_x, normal_y, normal_z: used as normal or direction vector
// curvature: used as timestamp or descriptor
// data[3]: used as neighborhood curvature
// normal[3]: used as the height above ground
**/

typedef pcl::PointCloud<Point_T>::Ptr pcTPtr;
typedef pcl::PointCloud<Point_T> pcT;

typedef pcl::search::KdTree<Point_T>::Ptr pcTreePtr;
typedef pcl::search::KdTree<Point_T> pcTree;

typedef pcl::PointCloud<pcl::PointXYZI>::Ptr pcXYZIPtr;
typedef pcl::PointCloud<pcl::PointXYZI> pcXYZI;

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr pcXYZPtr;
typedef pcl::PointCloud<pcl::PointXYZ> pcXYZ;

typedef pcl::PointCloud<pcl::PointXY>::Ptr pcXYPtr;
typedef pcl::PointCloud<pcl::PointXY> pcXY;

typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcXYZRGBPtr;
typedef pcl::PointCloud<pcl::PointXYZRGB> pcXYZRGB;

typedef pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pcXYZRGBAPtr;
typedef pcl::PointCloud<pcl::PointXYZRGBA> pcXYZRGBA;

typedef pcl::PointCloud<pcl::PointNormal>::Ptr pcXYZNPtr;
typedef pcl::PointCloud<pcl::PointNormal> pcXYZN;

typedef pcl::PointCloud<pcl::PointXYZINormal>::Ptr pcXYZINPtr;
typedef pcl::PointCloud<pcl::PointXYZINormal> pcXYZIN;

typedef pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhPtr;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfh;

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

typedef std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> Matrix4ds;

namespace lo
{

    struct centerpoint_t
    {
        double x;
        double y;
        double z;
        centerpoint_t(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    };

    // regular bounding box whose edges are parallel to x,y,z axises
    struct bounds_t
    {
        double min_x;
        double min_y;
        double min_z;
        double max_x;
        double max_y;
        double max_z;
        int type;

        bounds_t() { min_x = min_y = min_z = max_x = max_y = max_z = 0.0; }
        void inf_x()
        {
            min_x = -DBL_MAX;
            max_x = DBL_MAX;
        }
        void inf_y()
        {
            min_y = -DBL_MAX;
            max_y = DBL_MAX;
        }
        void inf_z()
        {
            min_z = -DBL_MAX;
            max_z = DBL_MAX;
        }
        void inf_xyz()
        {
            inf_x();
            inf_y();
            inf_z();
        }
    };

    // the point cloud's collecting template (node's type)
    enum DataType
    {
        ALS,
        TLS,
        MLS,
        BPLS,
        RGBD,
        SLAM
    };

    // the edge (constraint)'s type
    enum ConstraintType
    {
        REGISTRATION,
        ADJACENT,
        HISTORY,
        SMOOTH,
        NONE
    };

    // Basic processing unit(node)
    struct cloudblock_t
    {
        // Strip (transaction) should be the container of the cloudblock while
        // cloudblock can act as either a frame or submap (local map)
        int unique_id;        // Unique ID
        int strip_id;         // Strip ID
        int id_in_strip;      // ID in the strip
        int last_frame_index; // last_frame_id is the frame index (not unique_id) of
                              // the last frame of the submap
        // ID means the number may not be continous and begining from 0 (like 3, 7,
        // 11, ...), but index should begin from 0 and its step (interval) should be 1
        // (like 0, 1, 2, 3, ...)

        DataType data_type; // Datatype

        bounds_t bound;               // Bounding Box in geo-coordinate system
        centerpoint_t cp;             // Center Point in geo-coordinate system
        centerpoint_t station;        // Station position in geo-coordinate system
        Eigen::Matrix4d station_pose; // Station pose in geo-coordinate system

        bounds_t local_bound;               // Bounding Box in local coordinate system
        centerpoint_t local_cp;             // Center Point in local coordinate system
        centerpoint_t local_station;        // Station position in local coordinate system
        Eigen::Matrix4d local_station_pose; // Station pose in local coordinate system

        bool station_position_available; // If the approximate position of the station
                                         // is provided
        bool station_pose_available;     // If the approximate pose of the station is
                                         // provided
        bool is_single_scanline;         // If the scanner is a single scanline sensor,
                                         // determining if adjacent cloud blocks in a strip
                                         // would have overlap

        bool pose_fixed = false;  // the pose is fixed or not
        bool pose_stable = false; // the pose is stable or not after the optimization

        // poses
        Eigen::Matrix4d pose_lo;        // used for lidar odometry
        Eigen::Matrix4d pose_gt;        // used for lidar odometry (ground turth)
        Eigen::Matrix4d pose_optimized; // optimized pose
        Eigen::Matrix4d pose_init;      // used for the init guess for pgo

        Matrix6d information_matrix_to_next;

        std::string filename;           // full path of the original point cloud file
        std::string filenmae_processed; // full path of the processed point cloud file

        // Raw point cloud
        pcTPtr pc_raw;

        // Downsampled point cloud
        pcTPtr pc_down;
        pcTPtr pc_sketch; // very sparse point cloud

        pcTPtr pc_raw_w; // in world coordinate system (for lidar odometry)

        // unground point cloud
        pcTPtr pc_unground;

        // All kinds of geometric feature points (in target scan)
        pcTPtr pc_ground;
        pcTPtr pc_facade;
        pcTPtr pc_roof;
        pcTPtr pc_pillar;
        pcTPtr pc_beam;
        pcTPtr pc_vertex;

        // downsampled feature points (in source scan)
        pcTPtr pc_ground_down;
        pcTPtr pc_facade_down;
        pcTPtr pc_roof_down;
        pcTPtr pc_pillar_down;
        pcTPtr pc_beam_down;

        // Kdtree of the feature points (denser ones)
        pcTreePtr tree_ground;
        pcTreePtr tree_pillar;
        pcTreePtr tree_beam;
        pcTreePtr tree_facade;
        pcTreePtr tree_roof;
        pcTreePtr tree_vertex;

        // actually, it's better to save the indices of feature_points_down instead of
        // saving another feature point cloud

        int down_feature_point_num;
        int feature_point_num;

        cloudblock_t()
        {
            init();
            // default value
            station_position_available = false;
            station_pose_available = false;
            is_single_scanline = true;
            pose_lo.setIdentity();
            pose_gt.setIdentity();
            pose_optimized.setIdentity();
            pose_init.setIdentity();
            information_matrix_to_next.setIdentity();
        }

        cloudblock_t(const cloudblock_t &in_block, bool clone_feature = false, bool clone_raw = false)
        {
            init();
            clone_metadata(in_block);

            if (clone_feature)
            {
                // clone point cloud (instead of pointer)
                *pc_ground = *(in_block.pc_ground);
                *pc_pillar = *(in_block.pc_pillar);
                *pc_facade = *(in_block.pc_facade);
                *pc_beam = *(in_block.pc_beam);
                *pc_roof = *(in_block.pc_roof);
                *pc_vertex = *(in_block.pc_vertex);
                // keypoint_bsc = in_block.keypoint_bsc;
            }
            if (clone_raw)
                *pc_raw = *(in_block.pc_raw);
        }

        void init()
        {
            pc_raw = std::make_shared<pcT>();
            pc_down = std::make_shared<pcT>();
            pc_raw_w = std::make_shared<pcT>();
            pc_sketch = std::make_shared<pcT>();
            pc_unground = std::make_shared<pcT>();

            pc_ground = std::make_shared<pcT>();
            pc_facade = std::make_shared<pcT>();
            pc_roof = std::make_shared<pcT>();
            pc_pillar = std::make_shared<pcT>();
            pc_beam = std::make_shared<pcT>();
            pc_vertex = std::make_shared<pcT>();

            pc_ground_down = std::make_shared<pcT>();
            pc_facade_down = std::make_shared<pcT>();
            pc_roof_down = std::make_shared<pcT>();
            pc_pillar_down = std::make_shared<pcT>();
            pc_beam_down = std::make_shared<pcT>();

            init_tree();

            // doubleVectorSBF().swap(keypoint_bsc);

            down_feature_point_num = 0;
            feature_point_num = 0;
        }

        void init_tree()
        {
            tree_ground = std::make_shared<pcTree>();
            tree_facade = std::make_shared<pcTree>();
            tree_pillar = std::make_shared<pcTree>();
            tree_beam = std::make_shared<pcTree>();
            tree_roof = std::make_shared<pcTree>();
            tree_vertex = std::make_shared<pcTree>();
        }

        void free_raw_cloud()
        {
            pc_raw.reset(new pcT());
            pc_down.reset(new pcT());
            pc_unground.reset(new pcT());
        }

        void free_tree()
        {
            tree_ground.reset(new pcTree());
            tree_facade.reset(new pcTree());
            tree_pillar.reset(new pcTree());
            tree_beam.reset(new pcTree());
            tree_roof.reset(new pcTree());
            tree_vertex.reset(new pcTree());
        }

        void free_all()
        {
            free_raw_cloud();
            free_tree();
            pc_ground.reset(new pcT());
            pc_facade.reset(new pcT());
            pc_pillar.reset(new pcT());
            pc_beam.reset(new pcT());
            pc_roof.reset(new pcT());
            pc_vertex.reset(new pcT());
            pc_ground_down.reset(new pcT());
            pc_facade_down.reset(new pcT());
            pc_pillar_down.reset(new pcT());
            pc_beam_down.reset(new pcT());
            pc_roof_down.reset(new pcT());
            pc_sketch.reset(new pcT());
            pc_raw_w.reset(new pcT());
            // doubleVectorSBF().swap(keypoint_bsc);
        }

        void clone_metadata(const cloudblock_t &in_cblock)
        {
            feature_point_num = in_cblock.feature_point_num;
            bound = in_cblock.bound;
            local_bound = in_cblock.local_bound;
            local_cp = in_cblock.local_cp;
            pose_lo = in_cblock.pose_lo;
            pose_gt = in_cblock.pose_gt;
            pose_init = in_cblock.pose_init;
            pose_optimized = in_cblock.pose_optimized;
            unique_id = in_cblock.unique_id;
            id_in_strip = in_cblock.id_in_strip;
            filename = in_cblock.filename;
        }

        void append_feature(const cloudblock_t &in_cblock, bool append_down,
                            std::string used_feature_type)
        {
            // pc_raw->points.insert(pc_raw->points.end(),
            // in_cblock.pc_raw->points.begin(), in_cblock.pc_raw->points.end());
            if (!append_down)
            {
                if (used_feature_type[0] == '1')
                    pc_ground->points.insert(pc_ground->points.end(), in_cblock.pc_ground->points.begin(),
                                             in_cblock.pc_ground->points.end());
                if (used_feature_type[1] == '1')
                    pc_pillar->points.insert(pc_pillar->points.end(), in_cblock.pc_pillar->points.begin(),
                                             in_cblock.pc_pillar->points.end());
                if (used_feature_type[2] == '1')
                    pc_facade->points.insert(pc_facade->points.end(), in_cblock.pc_facade->points.begin(),
                                             in_cblock.pc_facade->points.end());
                if (used_feature_type[3] == '1')
                    pc_beam->points.insert(pc_beam->points.end(), in_cblock.pc_beam->points.begin(),
                                           in_cblock.pc_beam->points.end());
                if (used_feature_type[4] == '1')
                    pc_roof->points.insert(pc_roof->points.end(), in_cblock.pc_roof->points.begin(),
                                           in_cblock.pc_roof->points.end());
                pc_vertex->points.insert(pc_vertex->points.end(), in_cblock.pc_vertex->points.begin(),
                                         in_cblock.pc_vertex->points.end());
            }
            else
            {
                if (used_feature_type[0] == '1')
                    pc_ground->points.insert(pc_ground->points.end(), in_cblock.pc_ground_down->points.begin(),
                                             in_cblock.pc_ground_down->points.end());
                if (used_feature_type[1] == '1')
                    pc_pillar->points.insert(pc_pillar->points.end(), in_cblock.pc_pillar_down->points.begin(),
                                             in_cblock.pc_pillar_down->points.end());
                if (used_feature_type[2] == '1')
                    pc_facade->points.insert(pc_facade->points.end(), in_cblock.pc_facade_down->points.begin(),
                                             in_cblock.pc_facade_down->points.end());
                if (used_feature_type[3] == '1')
                    pc_beam->points.insert(pc_beam->points.end(), in_cblock.pc_beam_down->points.begin(),
                                           in_cblock.pc_beam_down->points.end());
                if (used_feature_type[4] == '1')
                    pc_roof->points.insert(pc_roof->points.end(), in_cblock.pc_roof_down->points.begin(),
                                           in_cblock.pc_roof_down->points.end());
                pc_vertex->points.insert(pc_vertex->points.end(), in_cblock.pc_vertex->points.begin(),
                                         in_cblock.pc_vertex->points.end());
            }
        }

        void merge_feature_points(pcTPtr &pc_out, bool merge_down, bool with_out_ground = false)
        {
            if (!merge_down)
            {
                if (!with_out_ground)
                    pc_out->points.insert(pc_out->points.end(), pc_ground->points.begin(),
                                          pc_ground->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_facade->points.begin(),
                                      pc_facade->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_pillar->points.begin(),
                                      pc_pillar->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_beam->points.begin(), pc_beam->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_roof->points.begin(), pc_roof->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_vertex->points.begin(),
                                      pc_vertex->points.end());
            }
            else
            {
                if (!with_out_ground)
                    pc_out->points.insert(pc_out->points.end(), pc_ground_down->points.begin(),
                                          pc_ground_down->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_facade_down->points.begin(),
                                      pc_facade_down->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_pillar_down->points.begin(),
                                      pc_pillar_down->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_beam_down->points.begin(),
                                      pc_beam_down->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_roof_down->points.begin(),
                                      pc_roof_down->points.end());
                pc_out->points.insert(pc_out->points.end(), pc_vertex->points.begin(),
                                      pc_vertex->points.end());
            }
        }

        void transform_feature(const Eigen::Matrix4d &trans_mat, bool transform_down = true,
                               bool transform_undown = true)
        {
            if (transform_undown)
            {
                pcl::transformPointCloudWithNormals(*pc_ground, *pc_ground, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_pillar, *pc_pillar, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_beam, *pc_beam, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_facade, *pc_facade, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_roof, *pc_roof, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_vertex, *pc_vertex, trans_mat);
            }
            if (transform_down)
            {
                pcl::transformPointCloudWithNormals(*pc_ground_down, *pc_ground_down, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_pillar_down, *pc_pillar_down, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_beam_down, *pc_beam_down, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_facade_down, *pc_facade_down, trans_mat);
                pcl::transformPointCloudWithNormals(*pc_roof_down, *pc_roof_down, trans_mat);
            }
        }

        void clone_cloud(pcTPtr &pc_out, bool get_pc_done)
        {
            if (get_pc_done)
                pc_out->points.insert(pc_out->points.end(), pc_down->points.begin(), pc_down->points.end());
            else
                pc_out->points.insert(pc_out->points.end(), pc_raw->points.begin(), pc_raw->points.end());
        }

        void clone_feature(pcTPtr &pc_ground_out, pcTPtr &pc_pillar_out, pcTPtr &pc_beam_out,
                           pcTPtr &pc_facade_out, pcTPtr &pc_roof_out, pcTPtr &pc_vertex_out,
                           bool get_feature_down)
        {
            if (get_feature_down)
            {
                *pc_ground_out = *pc_ground_down;
                *pc_pillar_out = *pc_pillar_down;
                *pc_beam_out = *pc_beam_down;
                *pc_facade_out = *pc_facade_down;
                *pc_roof_out = *pc_roof_down;
                *pc_vertex_out = *pc_vertex;
            }
            else
            {
                *pc_ground_out = *pc_ground;
                *pc_pillar_out = *pc_pillar;
                *pc_beam_out = *pc_beam;
                *pc_facade_out = *pc_facade;
                *pc_roof_out = *pc_roof;
                *pc_vertex_out = *pc_vertex;
            }
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<cloudblock_t, Eigen::aligned_allocator<cloudblock_t>> strip;
    typedef std::vector<strip> strips;
    typedef std::shared_ptr<cloudblock_t> cloudblock_Ptr;
    typedef std::vector<cloudblock_Ptr> cloudblock_Ptrs;

    // the edge of pose(factor) graph
    struct constraint_t
    {
        int unique_id;                 // Unique ID
        cloudblock_Ptr block1, block2; // Two block  //Target: block1,  Source: block2
        ConstraintType con_type;       // ConstraintType
        Eigen::Matrix4d Trans1_2;      // transformation from 2 to 1 (in global shifted map
                                       // coordinate system)
        Matrix6d information_matrix;
        float overlapping_ratio; // overlapping ratio (not bbx IOU) of two cloud
                                 // blocks
        float confidence;
        float sigma;              // standard deviation of the edge
        bool cov_updated = false; // has the information_matrix already updated

        constraint_t()
        {
            block1 = cloudblock_Ptr(new cloudblock_t);
            block2 = cloudblock_Ptr(new cloudblock_t);
            Trans1_2.setIdentity();
            information_matrix.setIdentity();
            sigma = FLT_MAX;
            cov_updated = false;
        }

        void free_cloud()
        {
            block1->free_all();
            block2->free_all();
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    typedef std::vector<constraint_t, Eigen::aligned_allocator<constraint_t>> constraints;

    // basic common functions of point cloud
    template <typename PointT>
    class CloudUtility
    {
    public:
        // Get Center of a Point Cloud
        void get_cloud_cpt(const typename pcl::PointCloud<PointT>::Ptr &cloud, centerpoint_t &cp)
        {
            double cx = 0, cy = 0, cz = 0;
            int point_num = cloud->points.size();

            for (int i = 0; i < point_num; i++)
            {
                cx += cloud->points[i].x / point_num;
                cy += cloud->points[i].y / point_num;
                cz += cloud->points[i].z / point_num;
            }
            cp.x = cx;
            cp.y = cy;
            cp.z = cz;
        }

        // Get Bound of a Point Cloud
        void get_cloud_bbx(const typename pcl::PointCloud<PointT>::Ptr &cloud, bounds_t &bound)
        {
            double min_x = DBL_MAX;
            double min_y = DBL_MAX;
            double min_z = DBL_MAX;
            double max_x = -DBL_MAX;
            double max_y = -DBL_MAX;
            double max_z = -DBL_MAX;

            for (int i = 0; i < cloud->points.size(); i++)
            {
                if (min_x > cloud->points[i].x)
                    min_x = cloud->points[i].x;
                if (min_y > cloud->points[i].y)
                    min_y = cloud->points[i].y;
                if (min_z > cloud->points[i].z)
                    min_z = cloud->points[i].z;
                if (max_x < cloud->points[i].x)
                    max_x = cloud->points[i].x;
                if (max_y < cloud->points[i].y)
                    max_y = cloud->points[i].y;
                if (max_z < cloud->points[i].z)
                    max_z = cloud->points[i].z;
            }
            bound.min_x = min_x;
            bound.max_x = max_x;
            bound.min_y = min_y;
            bound.max_y = max_y;
            bound.min_z = min_z;
            bound.max_z = max_z;
        }

        // Get Bound and Center of a Point Cloud
        void get_cloud_bbx_cpt(const typename pcl::PointCloud<PointT>::Ptr &cloud, bounds_t &bound,
                               centerpoint_t &cp)
        {
            get_cloud_bbx(cloud, bound);
            cp.x = 0.5 * (bound.min_x + bound.max_x);
            cp.y = 0.5 * (bound.min_y + bound.max_y);
            cp.z = 0.5 * (bound.min_z + bound.max_z);
        }

        void get_intersection_bbx(bounds_t &bbx_1, bounds_t &bbx_2, bounds_t &bbx_intersection,
                                  float bbx_boundary_pad = 2.0)
        {
            bbx_intersection.min_x = max_(bbx_1.min_x, bbx_2.min_x) - bbx_boundary_pad;
            bbx_intersection.min_y = max_(bbx_1.min_y, bbx_2.min_y) - bbx_boundary_pad;
            bbx_intersection.min_z = max_(bbx_1.min_z, bbx_2.min_z) - bbx_boundary_pad;
            bbx_intersection.max_x = min_(bbx_1.max_x, bbx_2.max_x) + bbx_boundary_pad;
            bbx_intersection.max_y = min_(bbx_1.max_y, bbx_2.max_y) + bbx_boundary_pad;
            bbx_intersection.max_z = min_(bbx_1.max_z, bbx_2.max_z) + bbx_boundary_pad;
        }

        void merge_bbx(std::vector<bounds_t> &bbxs, bounds_t &bbx_merged)
        {
            bbx_merged.min_x = DBL_MAX;
            bbx_merged.min_y = DBL_MAX;
            bbx_merged.min_z = DBL_MAX;
            bbx_merged.max_x = -DBL_MAX;
            bbx_merged.max_y = -DBL_MAX;
            bbx_merged.max_z = -DBL_MAX;

            for (int i = 0; i < bbxs.size(); i++)
            {
                bbx_merged.min_x = min_(bbx_merged.min_x, bbxs[i].min_x);
                bbx_merged.min_y = min_(bbx_merged.min_y, bbxs[i].min_y);
                bbx_merged.min_z = min_(bbx_merged.min_z, bbxs[i].min_z);
                bbx_merged.max_x = max_(bbx_merged.max_x, bbxs[i].max_x);
                bbx_merged.max_y = max_(bbx_merged.max_y, bbxs[i].max_y);
                bbx_merged.max_z = max_(bbx_merged.max_z, bbxs[i].max_z);
            }
        }

        // Get Bound of Subsets of a Point Cloud
        void get_sub_bbx(typename pcl::PointCloud<PointT>::Ptr &cloud, std::vector<int> &index,
                         bounds_t &bound)
        {
            typename pcl::PointCloud<PointT>::Ptr temp_cloud(new pcl::PointCloud<PointT>);
            for (int i = 0; i < index.size(); i++)
            {
                temp_cloud->push_back(cloud->points[index[i]]);
            }
            get_cloud_bbx(temp_cloud, bound);
        }
    };
} // namespace lo

// TODO: reproduce the code with better data structure

#endif //_INCLUDE_UTILITY_HPP_