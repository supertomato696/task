#pragma once

#include <Eigen/Dense>
#include <mutex>
#include "public/sha1.hpp"
#include "cpp_code/services/tileserver.pb.h"
#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "opencv2/opencv.hpp"
#include "proj_helper.h"
#include "pcd_define.fbs.h"

//Abbreviation Warning
//Position => posn
//Pose => Pose
struct PointXYZIRGB  {
    float x;
    float y;
    float z;
    unsigned char intensity;
    unsigned char r;
    unsigned char g;
    unsigned char b;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRGB,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (unsigned char, r, r)
    (unsigned char, g, g)
    (unsigned char, b, b)
    (unsigned char, intensity, intensity)
    )

namespace pcl{
    struct PointXYZRGBILF
    {
        PCL_ADD_POINT4D;
        unsigned char r;
        unsigned char g;
        unsigned char b;
        unsigned char intensity;
        uint32_t label;
        uint32_t frameindex;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
    } EIGEN_ALIGN16;
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
    PointXYZRGBILF,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (unsigned char, r, r)
    (unsigned char, g, g)
    (unsigned char, b, b)
    (unsigned char, intensity, intensity)
    (uint32_t, label, label)
    (uint32_t, frameindex, frameindex)
    )

typedef pcl::PointCloud<pcl::PointXYZRGBILF> UltraCloud;
typedef UltraCloud::Ptr UltraCloudPtr;
typedef UltraCloud::ConstPtr UltraCloudConstPtr;

struct TilePosInfo {
    services::TileResourceID res_id;
    int64_t index;
    int tile_id;
    int trail_id;
    double timestamp;
    int64_t version;
    Eigen::Vector3d pos;
    Eigen::Quaterniond quat;
    
    TilePosInfo() {
        index = -1;
        tile_id = 0;
        trail_id = 0;
        timestamp = 0;
        version = 0;
    };
    std::string to_str() const {
        char buf[256];
#ifdef WIN32
        sprintf_s(buf, "%d_%d_%lld_%lld.pcd", res_id.tile_id(), res_id.trail_id(), res_id.index(), res_id.version());
#else
        snprintf(buf, sizeof(buf), "%d_%d_%lld_%lld.pcd", res_id.tile_id(), res_id.trail_id(), res_id.index(), res_id.version());
#endif
        std::string t(buf);
        return t;
    };

    Eigen::Affine3d affine_l_2_w;
    Eigen::Affine3d affine_w_2_l;

    void world2local(double& x, double& y, double& z) const;
    void local2world(double& x, double& y, double& z) const;
    Eigen::Vector3d get_heading() const;
    double get_head() const;
    double get_pitch() const;
    double get_roll() const;
};

struct TileImagePos {
    services::TileResourceID res_id;
    int64_t index;
    double mat[12];
    std::string path;
    TilePosInfo pos;
    
    TileImagePos() {
        memset(mat, 0, sizeof(mat));
    };
    std::string to_str() const {
        return path;
    };
    
    bool project_3d_2_2d(double x, double y, double z, int& px, int& py) const;
};

typedef std::shared_ptr<TileImagePos> ImagePosPtr;
typedef std::vector<ImagePosPtr> ImagePosPtrVect;
typedef std::shared_ptr<ImagePosPtrVect> ImagePosVectPtr;

struct TileCloudInfo {
    int64_t id;
    std::string path;
    TilePosInfo pos;
    TileCloudInfo() {
        id = 0;
    };
};

struct Feature3DInfo {
    services::TileResourceID id;
    int type;
    Eigen::Vector3d pos;
    Eigen::Vector3d dir;
    std::vector<Eigen::Vector3d> pts;
    float length;
    float width;
    float height;
    float prob;
    int pre_idx;
    int next_idx;
    int color_type;
    int geo_type;
        
    Feature3DInfo() {
        type = -1;
        length = -1;
        width = -1;
        height = -1;
        prob = -1;
        pre_idx = -1;
        next_idx = -1;
        color_type = -1;
        geo_type = -1;
    };    
};

typedef std::shared_ptr<Feature3DInfo> FeatInfoPtr;
typedef std::vector<FeatInfoPtr> FeatInfoPtrVect;
typedef std::shared_ptr<FeatInfoPtrVect> FeatInfoVectPtr;

class CTileServerDao;
typedef std::shared_ptr<CTileServerDao> ServerDaoPtr;
typedef std::shared_ptr<const CTileServerDao> ServerDaoConstPtr;

class CTileServerDao {
public:
    CTileServerDao(void) {
        _tile_id = 0;
        for (int i = 1; i < 12; ++i) {
            _tids.push_back(i);
        }
    }
    ~CTileServerDao(void) {
    }

    void set_tile_address(const std::string& addr) { _tile_server = addr; };
    void set_tile_id(int tile_id) { _tile_id = tile_id; _proj.SetDstTile(tile_id); };
    int get_tile_id() { return _tile_id; };
    void set_branch(const std::string& branch) { _branch = branch; };
    const std::string get_branch() { return _branch; };
    void set_path(const std::string& path) { _path = path; };
    const std::string get_server_info() const;

    //hadmap::common::TileInfo generate_tileinfo() const;

    //bool download_clouds(const std::string& path, std::vector<TileCloudInfo>& infos);
    //bool download_cloud_data(const TilePosInfo& pos, std::string& data);
    //pcl::PointCloud<PointXYZIRGB>::Ptr get_pointcloud(const TileCloudInfo& id);
    bool download_images(const std::string& path, std::vector<TileImagePos>& infos);
    //bool download_image_data(int64_t iid, std::string& data);
    cv::Mat get_image(const TileImagePos& iid);

    bool query_resources(const services::TileResourceID& id, std::vector<services::TileResource>& descs);
    bool download_data(const services::TileResourceID& id, std::map<std::string, std::string>& slots);
    bool query_pos_rect(int trail_id, std::vector<TilePosInfo>& infos);
    bool query_image_rect(int trail_id, std::vector<TileImagePos>& infos);
    bool query_feature_rect(int trail_id, std::deque<Feature3DInfo>& features);
    
    hadmap::ProjHelper<11> _proj;
private:
    std::string _tile_server;
    std::string _branch;
    std::string _path;
    int _tile_id;
    std::vector<int> _tids;
};
