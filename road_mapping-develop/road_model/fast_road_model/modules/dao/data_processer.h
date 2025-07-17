

#pragma once

#include "road_model/road_model_meta.h"
#include "hdmap_server/data-access-engine/public/proj_helper.h"
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace fsdmap {
namespace dao {

using ele_type = std::tuple<ELEMENT_TYPE, int, int, cv::Scalar>;
using GisPt        = boost::geometry::model::point<double, 2, boost::geometry::cs::cartesian>;
using GisPolyline  = boost::geometry::model::linestring<GisPt>;
using GisPolygon   = boost::geometry::model::polygon<GisPt>;
using GisMultiPolygon = boost::geometry::model::multi_polygon<GisPolygon>;
using ele_attr = std::tuple<ELEMENT_TYPE, std::string, uint16_t>;


class DataProcessorBase {
public:
    DataProcessorBase() {};
    virtual ~DataProcessorBase() {}

    virtual int init() = 0;

    virtual void clear() = 0;

    virtual int download_cross_point(std::vector<BreakInfo*>& split_merge_break_points) = 0;  

    virtual int download_key_pose(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
            bool need_pcd=false) = 0;

    virtual int download_key_pose_mnt(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
            bool need_pcd=false) = 0;        

    virtual int download_link(std::vector<std::shared_ptr<KeyPoseLine>> &link_list,
            std::vector<std::shared_ptr<KeyPose>> &link_point_list, std::vector<GisPolygon> &polygon_list, 
            std::vector<Eigen::Vector3d> &polygon_vec3d) = 0;     

    virtual int download_intersection(std::vector<std::shared_ptr<Intersection>> &inter_list) = 0;

    virtual int download_nodes(std::vector<std::shared_ptr<LinkNodes>> &node_list,
                            std::unordered_map<uint64_t, std::shared_ptr<LinkNodes>>& node_id_map) = 0;

    virtual int download_feature(std::vector<std::shared_ptr<Feature>> &feature_list) = 0;

    virtual int download_lane_line(std::vector<std::shared_ptr<Feature>> &feature_vec) = 0;

    virtual int download_lane_center(std::vector<std::shared_ptr<Feature>> &feature_vec)=0;

    virtual int download_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec) = 0;

    virtual int download_site_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec) = 0;

    virtual int download_feature_from_sem_client(std::vector<std::shared_ptr<LaneFeature>> &lane_feature,
            std::vector<std::shared_ptr<BoundaryFeature>> &boundary_feature) = 0;

    virtual int download_object(std::vector<std::shared_ptr<RoadObjectInfo>> &raw_object_list) = 0;

    virtual int download_junction(std::vector<std::shared_ptr<JunctionInfo>> &raw_junction_list, 
        std::vector<std::shared_ptr<ImpassableAreaInfo>> &raw_area_list) = 0;

    virtual bool local2wgs(Eigen::Vector3d &local_pos, Eigen::Vector3d &wgs, bool need_offset=true) = 0;

    virtual bool wgs2local(Eigen::Vector3d &wgs, Eigen::Vector3d &local_pos, bool need_offset=true) = 0;

    virtual int upload_road_info(std::string &data) {};

    // virtual int upload_log(feature_merge_pb::LogList &log_list) {return 0;};
    
    virtual ele_type get_ele_type(std::string &version, int type) = 0;

    virtual bool invalid_pos(Eigen::Vector3d &pos) {
        return false;
    }
    virtual bool is_in_processe_scope(Eigen::Vector3d &pos) {
        return false;
    }

    virtual KeyPose* get_key_pose(std::string frame_id) {
        if (MAP_NOT_FIND(_key_map, frame_id)) {
            return NULL;
        }
        return _key_map[frame_id].first.get();
    }

    // virtual bool get_local_pos(Eigen::Vector3d &base_pos, Eigen::Vector3d &tar_pos, double scale_size,
    //              double value_max) = 0;

    virtual utils::DisplayScope& get_display_scope(std::string frame_id) = 0;

    virtual bool debug_pos(Eigen::Vector3d &pos, std::string frame_id, bool enable=true) = 0;

    virtual bool debug_pos2(Eigen::Vector3d &pos1, Eigen::Vector3d &pos2, bool enable) = 0;

    virtual std::string get_log_dir() = 0;

    virtual std::string get_logid() = 0;

    virtual bool search_nearest_pose(pcl::PointXYZI& searchPt, std::vector<float>& squaredDistances) = 0;

public:
    // frame id : pose 信息 
    UMAP<std::string, std::pair<std::shared_ptr<KeyPose>, utils::DisplayScope>> _key_map;
    // add by qzc
    // KeyPoseTree origin_poss_tree_; // ins数据
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr origin_poss_tree_;
    UMAP<int, std::shared_ptr<KeyPose>> id_2_pose_map;

    std::shared_ptr<utils::ThreadPoolProxy> _thread_pool;
    UMAP<std::string, UMAP<int, ele_type>> ele_type_map;
    Eigen::Vector3d _center_pos;
    std::vector<GisPolygon> _valid_polygen;
    UMAP<std::string, UMAP<std::tuple<std::string, std::string, uint16_t>, uint16_t>> attr_map;
    std::string _data_type = "empty";
    Eigen::Vector3d _center_link_pos; // 路口内的主node点
    std::vector<std::vector<Eigen::Vector3d>> side_boundary_polygons;
    std::string frame_id = "";

    int lane_center_id = 0;
    int lane_boundary_id = 0; 
    int road_boundary_id = 0;
};


class BevDataProcessor : public DataProcessorBase {
public:
    BevDataProcessor() {}
    virtual ~BevDataProcessor() {}

    virtual int init();

    virtual int init_ele_type();

    virtual void clear() {};

    virtual int download_cross_point(std::vector<BreakInfo*>& split_merge_break_points);  

    virtual int download_key_pose(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
            bool need_pcd=false);

    virtual int download_key_pose_mnt(std::vector<std::shared_ptr<KeyPose>> &key_poses, 
            bool need_pcd=false);

    virtual int download_link(std::vector<std::shared_ptr<KeyPoseLine>> &link_list,
            std::vector<std::shared_ptr<KeyPose>> &link_point_list,
            std::vector<GisPolygon> &polygon_list, std::vector<Eigen::Vector3d> &polygon_vec3d);

    virtual int download_intersection(std::vector<std::shared_ptr<Intersection>> &inter_list);

    virtual int download_nodes(std::vector<std::shared_ptr<LinkNodes>> &node_list,
                            std::unordered_map<uint64_t, std::shared_ptr<LinkNodes>>& node_id_map);

    virtual int download_feature(std::vector<std::shared_ptr<Feature>> &feature_vec);

    virtual int download_lane_line(std::vector<std::shared_ptr<Feature>> &feature_vec);

    virtual int download_lane_center(std::vector<std::shared_ptr<Feature>> &feature_vec);

    virtual int download_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec);

    virtual int download_site_boundary_line(std::vector<std::shared_ptr<Feature>> &feature_vec);

    virtual int download_feature_from_sem_client(std::vector<std::shared_ptr<LaneFeature>> &lane_feature,
            std::vector<std::shared_ptr<BoundaryFeature>> &boundary_feature);

    virtual int download_object(std::vector<std::shared_ptr<RoadObjectInfo>> &raw_object_list);

    virtual int download_junction(std::vector<std::shared_ptr<JunctionInfo>> &raw_junction_list, 
        std::vector<std::shared_ptr<ImpassableAreaInfo>> &raw_area_list);

    virtual int upload_road_info(std::string &data) {};

    virtual bool search_nearest_pose(pcl::PointXYZI& searchPt, std::vector<float>& squaredDistances);

    virtual int convert_mnt_lane_center_type(int lane_type)
    {
        int line_type_new = 43;
        if (1 == lane_type)
        {
            line_type_new = 43;
        }
        else
        {
            line_type_new = 44;
        }
        return line_type_new;
    }

    virtual int convert_mnt_lane_boundaries_type(int lane_type)
    {
        int line_type_new = 0;
        if (2 == lane_type)
        {
            line_type_new = 0;
        }
        else
        {
            line_type_new = 1;
        }
        return line_type_new;
    }

    virtual int convert_mnt_road_boundaries_type(int lane_type)
    {
        int line_type_new = 24;
        if (2 == lane_type)
        {
            line_type_new = 24;
        }
        else if(4 == lane_type)
        {
            line_type_new = 26;
        }else{
            line_type_new = 27;
        }
        return line_type_new;
    }

    virtual bool invalid_pos(Eigen::Vector3d &pos) {
        return false;
    };

    virtual bool is_in_processe_scope(Eigen::Vector3d &pos);

    virtual std::string get_frame_id(const std::string &trail_id, const std::string &time_str) {
        return utils::fmt("{}_{}", trail_id, time_str);
    }

    std::string get_frame_id_by_timestamp(int64_t time_stamp) {

        std::string ret = "";

        //效率低下，建议后续优化调整轨迹和bev的目录结构以及时间搓的对应关系
        double min_gap = 1000;   //ms
        std::shared_ptr<KeyPose> tar_key_pose = nullptr;
        for(auto iter = _key_map.begin(); iter != _key_map.end(); ++iter){
            double key_pos_time_stamp = iter->second.first->timestamp;

            double gap = fabs(time_stamp - key_pos_time_stamp);
            if( gap < min_gap){
                min_gap = gap;
                ret = iter->second.first->frame_id;
            }
        }

        return ret;
    }

    virtual ele_type get_ele_type(std::string &version, int type) {
        auto type_map = &(this->ele_type_map["0"]);
        if (MAP_FIND(this->ele_type_map, version)) {
            type_map = &(this->ele_type_map[version]);
        }
        if (type_map == NULL || MAP_NOT_FIND((*type_map), type)) {
            return this->null_type;
        }
        return (*type_map)[type];
    }

    // virtual bool get_local_pos(Eigen::Vector3d &base_pos, Eigen::Vector3d &tar_pos, double scale_size,
    //              double value_max) {};

    virtual bool wgs2local(Eigen::Vector3d &wgs, Eigen::Vector3d &local_pos, bool need_offset=true);
    bool wgs2local(int32_t utm_zone, Eigen::Vector3d &wgs, Eigen::Vector3d &local_pos, bool need_offset = true);

    virtual bool local2wgs(Eigen::Vector3d &local_pos, Eigen::Vector3d &wgs, bool need_offset=true);

    virtual int read_feature_from_json(const std::string &file_name, std::string trail_id, std::string frame_id, 
        std::vector<std::shared_ptr<Feature>> &feature_vec);

    int read_client_feature_from_json(std::string &file_name, std::string trail_id, std::string frame_id,
        std::vector<std::shared_ptr<LaneFeature>> &lane_feature,
        std::vector<std::shared_ptr<BoundaryFeature>> &boundary_feature);

    virtual utils::DisplayScope& init_display_scope();

    virtual utils::DisplayScope& get_display_scope(std::string frame_id);

    virtual bool debug_pos(Eigen::Vector3d &pos, std::string frame_id, bool enable=true);

    virtual bool debug_pos2(Eigen::Vector3d &pos1, Eigen::Vector3d &pos2, bool enable);

    virtual std::string get_log_dir();

    virtual std::string get_logid() {
        return _task_id;
    };

    virtual void init_attr_map();
    virtual uint8_t get_std_attr(std::string version, std::string ele_type, std::string attr_name, uint8_t input_value);
    std::vector<uint8_t> get_std_attrs(std::string version, std::string ele_type, std::string attr_name, uint32_t raw_type);

public:
    std::string _branch;
    int64_t _tile_id;
    std::string _task_id;
    // nlohmann::json _middle_root;
    int32_t _utm_zone;
    std::string _tile_id_str;
    int _trail_id;
    data_access_engine::ProjectionHelper<15> _proj15;
    utils::DisplayScope _scope;
    ele_type null_type = {ELEMENT_NULL, 0, 0, {255, 255, 255}};

    // data_access_engine::DAEInterface* _dae_interface;
    // data_access_engine::RoadGeometryManager* _road_geometry_mgr;
    // data_access_engine::ConfigAddress* _road_server_config_address;
    // std::unordered_map<std::string, std::string> _map_pipeline_cache;
};

}
}



