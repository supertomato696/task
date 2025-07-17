

#pragma once

#include "utils/process_frame.h"
#include "road_model/road_model_meta.h"
#include "dao/data_processer.h"

#include "../version/version.h"
#include "core/road_segment.h"

DECLARE_string(crosspoint_file_dir);
DECLARE_string(debug_file_dir);
DECLARE_string(shp_file_dir);
DECLARE_string(origin_shp_file_dir);
DECLARE_string(mid_shp_file_dir);

namespace fsdmap {
namespace road_model {

struct RoadModelSessionData :
    public fsdmap::process_frame::SessionDataBase<RoadModelSessionData> {

public:
    RoadModelSessionData() {
    };
    virtual ~RoadModelSessionData() {};

    template<typename FormatString, typename... Args>
    utils::DisplayInfo* add_debug_log(utils::DisplayInfo::LOG_TYPE type,
            const FormatString &desc, const Args &... args) {
        auto log = add_ptr(_log_list);
        log->init(type, desc, args...);
        log->log_name = _curr_log;
        if (MAP_NOT_FIND(_log_map, _curr_log)) {
            _log_map[_curr_log] = std::vector<utils::DisplayInfo*>();
        }
        _log_map[_curr_log].push_back(log.get());
        return log.get();
    }
    
    void add_debug_log(utils::DisplayInfo* log) {
        if (MAP_NOT_FIND(_log_map, _curr_log)) {
            _log_map[_curr_log] = std::vector<utils::DisplayInfo*>();
        }
        _log_map[_curr_log].push_back(log);
    }
    
    template<typename FormatString, typename... Args>
    std::string get_debug_dir(const FormatString &file_name, const Args &... args) {
        auto image_file = utils::fmt(file_name, args...);
        auto abs_file = utils::fmt("{}/{}", FLAGS_debug_file_dir, image_file);
        return abs_file;
    }

    template<typename FormatString, typename... Args>
    void save_debug_image(cv::Mat &im, const FormatString &file_name, const Args &... args) {
        auto image_file = get_debug_dir(file_name, args...);
        cv::imwrite(image_file.c_str(), im);
    }

    template<class T>
    std::shared_ptr<T> add_ptr(std::vector<std::shared_ptr<T> > &ptr_vec, bool need_lock=false) {
        if (need_lock) {
            std::unique_lock<std::mutex> lk(_ptr_vec_mutex);
            ptr_vec.push_back(std::make_shared<T>());
            return ptr_vec.back();
        }
        ptr_vec.push_back(std::make_shared<T>());
        return ptr_vec.back();
    }

    template<class T>
    void add_vec(std::vector<T> &vec, std::vector<T> &vec1, bool need_lock=false) {
        if (need_lock) {
            std::unique_lock<std::mutex> lk(_ptr_vec_mutex);
            VEC_PUSH_ALL(vec, vec1);
            return;
        }
        VEC_PUSH_ALL(vec, vec1);
        return;
    }
    
    template<class T>
    void add_vec(T &vec, T &vec1, bool need_lock=false) {
        if (need_lock) {
            std::unique_lock<std::mutex> lk(_ptr_vec_mutex);
            VEC_PUSH_ALL(vec, vec1);
            return;
        }
        VEC_PUSH_ALL(vec, vec1);
        return;
    }

    template<class T>
    void add_vec(std::vector<T> &vec, T ele, bool need_lock=false) {
        if (need_lock) {
            std::unique_lock<std::mutex> lk(_ptr_vec_mutex);
            vec.push_back(ele);
            return;
        }
        vec.push_back(ele);
        return;
    }

    void lock_run(std::function<void()> fn) {
        std::unique_lock<std::mutex> lk(_ptr_vec_mutex);
        fn();
    }

    //遍历raw_list中的点，间隔min_gap取点放入ptr_list，和new_list
    template<class T, class S>
    void sample_line(double min_gap, std::vector<T*> &raw_list, 
            std::vector<std::shared_ptr<S>> &ptr_list, std::vector<S*> &new_list) {
        S* prev_node = NULL;
        for (int i = 0; i < raw_list.size(); ++i) {
            auto &raw_node = raw_list[i];
            this->debug_pos(raw_node->pos);
            if (raw_node->invalid()) { // filter_status>=2就是invalid,例如在数据读取的时候对ins的pose有稀疏化的操作，不满足距离间距的点filter_status=2
                continue;
            }
            auto new_node = this->add_ptr(ptr_list);
            new_node->init(raw_node);
            if (prev_node != NULL) {
                double pos_gap = calc_dis_tmp(prev_node->pos, raw_node->pos);
                if (pos_gap >= min_gap) {
                    double z_diff = raw_node->pos.z() - prev_node->pos.z();
                    double rate = min_gap / pos_gap;
                    calc_dir_tmp(raw_node->pos, prev_node->pos, prev_node->dir); //计算prev_node的朝向
                    new_node->pos = prev_node->pos + min_gap * prev_node->dir; // 按计算出来的朝向，按min_gap的距离递推new_node的位置
                    new_node->pos.z() += rate * z_diff;
                    i--;
                } else if (i == raw_list.size() - 1) {
                    if (pos_gap < min_gap / 2) {
                        prev_node->pos = raw_node->pos;
                        continue;
                    } else {
                        new_node->pos = raw_node->pos;
                    }
                } else {
                    continue;
                }
            } else {
                new_node->pos = raw_node->pos;
                new_node->line_length = 0;
            }
            new_list.push_back(new_node.get());
            prev_node = new_node.get();
        }
        // 重新计算前后节点的连接关系，朝向，以及line_length信息
        // 因为有些invalid的点被删除了
        prev_node = NULL;
        double total_length = 0;
        for (auto &node : new_list) {
            node->line_length = total_length;
            if (prev_node != NULL) {
                node->set_prev(prev_node);
                prev_node->dir = get_dir_tmp(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += calc_dis_tmp(node->pos, prev_node->pos);
                node->line_length = total_length;
            }
            prev_node = node;
        }
    }


    //遍历raw_list中的点，间隔min_gap取点放入ptr_list，和new_list
    template<class T, class S>
    void sample_line_share_ptr(double min_gap, std::vector<std::shared_ptr<T>> &raw_list, 
            std::vector<std::shared_ptr<S>> &ptr_list, std::vector<std::shared_ptr<S>> &new_list) {
        std::shared_ptr<S> prev_node = NULL;
        for (int i = 0; i < raw_list.size(); ++i) {
            auto &raw_node = raw_list[i];
            this->debug_pos(raw_node->pos);
            if (raw_node->invalid()) { // filter_status>=2就是invalid,例如在数据读取的时候对ins的pose有稀疏化的操作，不满足距离间距的点filter_status=2
                continue;
            }
            auto new_node = this->add_ptr(ptr_list);
            new_node->init(raw_node);
            if (prev_node != nullptr) {
                double pos_gap = calc_dis_tmp(prev_node->pos, raw_node->pos);
                if (pos_gap >= min_gap) {
                    double z_diff = raw_node->pos.z() - prev_node->pos.z();
                    double rate = min_gap / pos_gap;
                    calc_dir_tmp(raw_node->pos, prev_node->pos, prev_node->dir); //计算prev_node的朝向
                    new_node->pos = prev_node->pos + min_gap * prev_node->dir; // 按计算出来的朝向，按min_gap的距离递推new_node的位置
                    new_node->pos.z() += rate * z_diff;
                    i--;
                } else if (i == raw_list.size() - 1) {
                    if (pos_gap < min_gap / 2) {
                        prev_node->pos = raw_node->pos;
                        continue;
                    } else {
                        new_node->pos = raw_node->pos;
                    }
                } else {
                    continue;
                }
            } else {
                new_node->pos = raw_node->pos;
                new_node->line_length = 0;
            }
            new_list.push_back(new_node);
            prev_node = new_node;
        }
        // 重新计算前后节点的连接关系，朝向，以及line_length信息
        // 因为有些invalid的点被删除了
        prev_node = NULL;
        double total_length = 0;
        for (auto &node : new_list) {
            node->line_length = total_length;
            if (prev_node != NULL) {
                node->set_prev(prev_node.get());
                prev_node->dir = get_dir_tmp(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += calc_dis_tmp(node->pos, prev_node->pos);
                node->line_length = total_length;
            }
            prev_node = node;
        }
    }

    //遍历raw_list中的点，间隔min_gap取点放入ptr_list，和new_list
    template<class T, class S>
    void no_sample_line(double min_gap, std::vector<T*> &raw_list, 
            std::vector<std::shared_ptr<S>> &ptr_list, std::vector<S*> &new_list) {
        S* prev_node = NULL;
        for (int i = 0; i < raw_list.size(); ++i) {
            auto &raw_node = raw_list[i];
            this->debug_pos(raw_node->pos);
            if (raw_node->invalid()) { // filter_status>=2就是invalid,例如在数据读取的时候对ins的pose有稀疏化的操作，不满足距离间距的点filter_status=2
                continue;
            }
            auto new_node = this->add_ptr(ptr_list);
            new_node->init(raw_node);
            // if (prev_node != NULL) {
            //     double pos_gap = calc_dis_tmp(prev_node->pos, raw_node->pos);
            //     if (pos_gap >= min_gap) {
            //         double z_diff = raw_node->pos.z() - prev_node->pos.z();
            //         double rate = min_gap / pos_gap;
            //         calc_dir_tmp(raw_node->pos, prev_node->pos, prev_node->dir); //计算prev_node的朝向
            //         new_node->pos = prev_node->pos + min_gap * prev_node->dir; // 按计算出来的朝向，按min_gap的距离递推new_node的位置
            //         new_node->pos.z() += rate * z_diff;
            //         i--;
            //     } else if (i == raw_list.size() - 1) {
            //         if (pos_gap < min_gap / 2) {
            //             prev_node->pos = raw_node->pos;
            //             continue;
            //         } else {
            //             new_node->pos = raw_node->pos;
            //         }
            //     } else {
            //         continue;
            //     }
            // } else {
                new_node->pos = raw_node->pos;
                new_node->line_length = 0;
            // }
            new_list.push_back(new_node.get());
            prev_node = new_node.get();
        }
        // 重新计算前后节点的连接关系，朝向，以及line_length信息
        // 因为有些invalid的点被删除了
        prev_node = NULL;
        double total_length = 0;
        for (auto &node : new_list) {
            node->line_length = total_length;
            if (prev_node != NULL) {
                node->set_prev(prev_node);
                prev_node->dir = get_dir_tmp(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += calc_dis_tmp(node->pos, prev_node->pos);
                node->line_length = total_length;
            }
            prev_node = node;
        }
    }


    // version: { MMT, BYD }; scene: { turn_right }; obj: { boundary, laneline, centerline }; info: { type, color }
    int get_default_label_map(const std::string &scene, const std::string &info, const std::string &obj, const std::string version)
    {   
        int res = 99;  // 先给个异常值：其他类型
        if(scene == "turn_right"){

            if(info=="color"){
                if(obj=="laneline"){
                    res = 2;  // 白色
                }else{
                    LOG_ERROR("set default bev_label: ERROR, CHECK: [ obj:{}, info:{} ] !!!", obj, info);
                }
            }else if(info=="type"){  // type
                if (obj == "boundary"){
                    if(version == "MMT"){
                        res = 1;  // 普通-flat
                    }else{  // BYD
                        res = 11;  // 普通-flat
                    }
                } else if(obj == "laneline"){
                    res = 2;  // 实线
                } else if(obj == "centerline"){
                    res = 1;  // 普通车道
                } else {
                    LOG_ERROR("Set default bev_label: ERROR, CHECK: [ obj:{}, info:{} ] !!!", obj, info);
                }
            }
        } else{
            LOG_ERROR("Set default bev_label: ERROR, CHECK: [ scene:{} ] !!!", scene);
        }

        return res;
    }


    inline bool debug_pos(Eigen::Vector3d &pos) {
        return data_processer->debug_pos(pos, "", enable_debug_pos);
    }

    inline bool debug_pos(Eigen::Vector3d &pos, bool enable) {
        return data_processer->debug_pos(pos, "", enable);
    }

    inline bool debug_pos(Eigen::Vector3d &pos, std::string frame_id) {
        return data_processer->debug_pos(pos, frame_id, enable_debug_pos);
    }

    inline bool debug_pos(Eigen::Vector3d &pos, std::string frame_id, bool enable) {
        return data_processer->debug_pos(pos, frame_id, enable);
    }

    inline bool debug_pos2(Eigen::Vector3d &pos1, Eigen::Vector3d &pos2) {
        return data_processer->debug_pos2(pos1, pos2, enable_debug_pos);
    }

    inline bool debug_pos2(Eigen::Vector3d &pos1, Eigen::Vector3d &pos2, bool enable) {
        return data_processer->debug_pos2(pos1, pos2, enable);
    }

    void set_display_name(const char * log_name);

    void concate_display_name(const char * log_name);

    void save_debug_info(const char * log_name);

    void display_pos(Eigen::Vector3d &start_pos, const char *file);

    void display_pos(Eigen::Vector3d &start, Eigen::Vector3d &end, const char *file);

    void init_scope(double min_x, double min_y, double max_x, double max_y, double z);
public:
    int err_code;
    bool enable_debug_pos;
    dao::DataProcessorBase* data_processer;
    utils::ThreadPoolProxy* thread_pool;
    data_access_engine::ProjectionHelper<15> proj15;
    std::mutex _ptr_vec_mutex;
    UMAP<std::string, std::vector<utils::DisplayInfo*>> _log_map;
    utils::ProcessBar _thread_process_bar;
    int save_log_index = 0;

    int64_t tile_id;
    utils::DisplayScope _scope;
    std::string _curr_log;
    std::vector<std::shared_ptr<utils::DisplayInfo>> _log_list;

    int lane_center_id = 0; //由trail_ptr 初始化
    int lane_boundary_id = 0; 
    int road_boundary_id = 0; 

    
    // 临时使用
    std::vector<std::shared_ptr<LaneFeature>> raw_client_lane_feature; //人工标注的信息里面读取车道线信息

    /****************************start 暂未接入***************************************** */
    // std::vector<std::shared_ptr<BoundaryFeature>> raw_client_boundary_feature; //人工标注的信息里面读取道路边界
    // UMAP<std::string, BoundaryLine> label_boundary_instance_map;
    // std::vector<std::shared_ptr<JunctionInfo>> new_junction_list; // 路口边界
    // std::vector<std::shared_ptr<ImpassableAreaInfo>> new_area_list; // 不可通行区域
    /****************************end 暂未接入***************************************** */

    /********************start download**************************** */
    std::vector<std::shared_ptr<LaneFeature>> lane_line_feature_ptr; // 初始化时由raw_features中的车道线信息存入
    std::vector<std::shared_ptr<BoundaryFeature>> boundary_feature_ptr; // 初始化时由raw_features中的栅栏信息、路沿、十字交叉入口信息存入，以及raw_boundary_feature的道路边界，也包括道路中间的黄色车道线
    std::vector<std::shared_ptr<LaneCenterFeature>> lane_center_feature_ptr; // 初始化时由raw_features中的车道中心线信息存入，后面对其做了降采样，还有通过车道线生成的车道中心点（同时对这些车道生成行驶方向反向的车道中心点，即方向向量反向，左右侧的车道线点也调换）
    std::vector<std::shared_ptr<LaneCenterFeature>> virtual_lane_center_feature_ptr; // 初始化时由raw_features中的虚线信息存入
    std::vector<std::shared_ptr<RoadObjectInfo>> raw_object_ret_list; // 目标物体，目标包含点云以及类型等属性
    // std::vector<std::shared_ptr<ObjectPointFeature>> object_feature_ptr;
    std::vector<std::shared_ptr<ObjectFeature>> object_ptr;

    std::vector<std::shared_ptr<KeyPoseLine>> raw_links; // link关系，每条车道线上的每个link node点
    std::vector<std::shared_ptr<Intersection>> raw_intersections; // 交叉路口（十字路口）
    
    UMAP<std::string, LaneFeatureLine> sem_lane_instance_map; //初始化时由raw_lane_feature中的车道线信息存入
    UMAP<std::string, LaneCenterLine> lane_center_instance_map; // 中心线：初始化时由raw_features中的车道中心线信息存入
    UMAP<std::string, BoundaryLine> sem_curb_instance_map; // 道路边界线： 初始化时由 raw_boundary_feature 的道路边界存入
    std::vector<Intersection*> intersections;  // 初始化时直接从 raw_intersections 赋值过来
    UMAP<int, int> double_geo_map;  // 是否双线
    // 所有的数据（中心线、车道线、边界线）都暂存在这里
    UMAP<std::string, KeyPoseLine> key_pose_map; // 初始化的时候把raw_key_poses的结果放入key_pose_map，并且按一定的间距做了稀疏化

    std::unordered_map<uint64_t, std::shared_ptr<LinkNodes>> node_id_map;


    // download 暂未使用，持续一段时间，没用就删除
    // UMAP<std::string, BoundaryLine> junction_instance_map; // 初始化时由raw_features中的十字交叉入口信息存入
    // UMAP<std::string, BoundaryLine> curb_instance_map; // 初始化时由raw_features中的路沿信息存入
    // UMAP<std::string, BoundaryLine> barrier_instance_map; // 初始化时由raw_features中的栅栏信息信息存入
    // UMAP<std::string, LaneCenterLine> virtual_lane_center_instance_map; // 初始化时由raw_features中的虚线信息存入
    /********************end download**************************** */
    

    /********************start sample line**************************** */
    // sample line 输出
    std::vector<KeyPose*> key_pose_list; // key_pose_map.list中pose之间间距满足2m的ins数据
    std::vector<KeyPoseLine*> link_sample_list; // 与key_pose_list中位置匹配的raw_links点（把ins的pose映射到link每个点的pose），并且对其进行了降采样
 
    // 所有kdtree：sample line 输出
    RTreeProxy<KeyPose*, float, 2> key_pose_tree;  // key_pose_list 的树， 轨迹构成的树
    RTreeProxy<KeyPose*, float, 2> raw_link_pos_tree;  // 原始的 link 对应 pos 构建的tree
    RTreeProxy<KeyPose*, float, 2> link_pos_tree;  //link_sample_list生成的， 已经聚类后的 link（去除了很多分叉的link）对应 pos 构建的tree

    std::vector<std::shared_ptr<KeyPose>> key_pose_ptr; // key_pose_map.list中pose之间间距满足2m的ins数据，以及与key_pose_list中位置匹配的Link点（把ins的pose映射到link每个点的pose）
    std::vector<std::shared_ptr<KeyPoseLine>> key_pose_line_ptr; // 与key_pose_list中位置匹配的raw_links点（把ins的pose映射到link每个点的pose），并且对其进行了降采样
    std::vector<std::shared_ptr<LaneLineSample>> lane_line_sample_ptr;
    std::vector<std::shared_ptr<BoundaryLine>> boundary_line_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<LaneLineSampleLine>> lane_line_sample_line_ptr;
    /********************end sample line**************************** */


    /********************start merge feature**************************** */
    // 所有kdtree： merge feature 输出的信息
    RTreeProxy<LaneLineSample*, float, 2> lane_line_sample_tree;   // 所有路线中的车道线信息，每条路线中的车道线分别进行了曲线融合，由lane_line_sample_list_opt构建
    RTreeProxy<LaneCenterFeature*, float, 2> merge_lane_center_sample_tree;   // 由merge_lane_center_list 构建
    RTreeProxy<BoundaryFeature*, float, 2> boundary_line_sample_tree; // 所有路线中的boundary信息，每条路线中的boundary分别进行了曲线融合
    std::vector<LaneLineSampleGroupLine*> merge_lane_line_list; // 存放每条路线中融合后的所有车道线，从每条路线的lane_line_group_list读入
    std::vector<BoundaryGroupLine*> merge_boundary_line_list; // 存放每条路线中融合后的所有boundary曲线，从每条路线的boundary_line_group_list读入
    std::vector<LaneCenterGroupLine*> merge_lane_center_list; // 存放每条路线中融合后的所有车道中心线，从每条路线的lane_center_line_group_list读入；由merge_lane_center_list构建的反向车道中心线点会重新放入
    
    // merge feature 中间变量
    UMAP<std::string, std::vector<BoundaryLine*>> bev_frame_boundary_line;
    UMAP<std::string, std::vector<LaneLineSampleLine*>> bev_frame_lane_line; // bev感知车道线的采样结果
    UMAP<std::string, std::vector<LaneCenterLine*>> bev_frame_lane_center_line; // bev感知车道中心线的采样结果
    // add by qzc
    std::vector<std::shared_ptr<BoundaryLine>> boundary_line_ptr_for_merge;
    std::vector<std::shared_ptr<BoundaryGroupLine>> lane_boundary_group_line_ptr; //bev感知车道中心线采样结果
    std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_line_ptr;
    std::vector<std::shared_ptr<LaneLineSampleLine>> lane_line_sample_line_ptr_for_merge;
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_ptr; //bev感知车道中心线采样结果
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_ptr_for_merge; //bev感知车道中心线采样结果
    std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_group_line_ptr;// 由merge_lane_center_list构建的反向车道中心线点

    /********************end merge feature**************************** */


    /********************start build coverage**************************** */
    //<link, <路口中心点，路口起始结束信息>>
    std::unordered_map<KeyPoseLine*, std::unordered_map<Eigen::Vector3d, std::shared_ptr<LinkInterInfo>,Vector3dHash>> link_inter_info;
    std::unordered_map<KeyPose*, HorizonCrossLine*> hor_corss_line_m; //第一个KeyPose为路口起始点，第二位对应处理的这条线 
    // 分合流
    std::vector<LaneCenterFeature*> classified_lc_points; // 预处理的分合流点
    /********************end build coverage**************************** */

    /********************start split merge**************************** */
    std::vector<std::shared_ptr<OneSplitMergeMatch>> sm_matchs;
    /********************end split merge**************************** */


    /********************start bind trail**************************** */
    // 分合流断点
    std::vector<BreakInfo*> split_merge_break_points;
    RTreeProxy<BreakInfo*, float, 2> sm_break_candidate_tree;

    std::vector<SplitMergeRange> split_merge_ranges;
    /********************end bind trail**************************** */

    /********************start build boundary**************************** */
    std::vector<std::shared_ptr<RoadCenterFeature>> road_center_ptr; // 根据boundary计算出来的道路中心点, !!! 注意：不能删除，缓存数据 !!!
    /********************end build boundary**************************** */


    /********************start build intersection**************************** */
    std::vector<InterInfo*> inter_infos;
    std::vector<std::shared_ptr<InterInfo>> inter_info_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<RoadLaneInfo>> road_lane_line_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<RoadLaneGroupInfo>> lane_group_info_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<RoadLinePointInfo>> line_point_info_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<RoadLaneBoundaryInfo>> lane_boundary_info_ptr; // !!! 注意：不能删除，缓存数据 !!!
    /********************end build intersection**************************** */

    /********************start export shp**************************** */
    std::string frame_id = "";
    std::string version = ROAD_MAPPING_VERSION_STRING;
    std::vector<RoadLaneGroupInfo*> new_lane_groups;
    /********************end export shp**************************** */

    // 公共信息
    std::vector<fsdmap::dao::GisPolygon> task_polygon;
    std::vector<Eigen::Vector3d> task_polygon_vec3d;

    // cxf add
    // ！！！！！！！！！！！！！！！！！
    // TODO:cxf 整体挪到对应的类里面去
    std::vector<std::shared_ptr<BreakPointKeyPose>> break_point_key_pose_list;  //路口打断点
    //识别断点调试用，调试完注释关闭
    //进入
    std::vector<Eigen::Vector3d> lc_pts_eigen_in, point0_in;
    std::vector<Eigen::Vector3d>  sl_cross_pt_in, cw_cross_pt_in, inter_cross_pt_in, break_pt_in;
    //退出
    std::vector<Eigen::Vector3d> lc_pts_eigen_out, point0_out;
    std::vector<Eigen::Vector3d>  sl_cross_pt_out, cw_cross_pt_out, inter_cross_pt_out, break_pt_out;
    // TODO:cxf 整体挪到对应的类里面去
    // ！！！！！！！！！！！！！！！！！

    /********************start format new road**************************** */
    // 道路段
    std::vector<fast_road_model::RoadSegment*>road_segments;
    std::vector<std::shared_ptr<fast_road_model::RoadSegment>> road_segments_ptr; // !!! 注意：不能删除，缓存数据 !!!
    std::vector<std::shared_ptr<RoadBreak>> road_break_ptr; // !!! 注意：不能删除，缓存数据 !!!
    /********************end format new road**************************** */
};

class RoadModelSessionManager :
    public fsdmap::process_frame::SessionManager<RoadModelSessionData> {
public:
    RoadModelSessionManager() {};
    virtual ~RoadModelSessionManager() {};

    virtual const char * name() {
        return "road_model";
    }

    // ��ʼ��proc�Ľӿ�
    virtual int declare_proc();

    // data_access_engine::DAEInterface* dae_interface;
    // data_access_engine::RoadGeometryManager* road_geometry_mgr;
    // data_access_engine::ConfigAddress* road_server_config_address;
    std::shared_ptr<utils::ThreadPoolProxy> thread_pool;
};

}
}
