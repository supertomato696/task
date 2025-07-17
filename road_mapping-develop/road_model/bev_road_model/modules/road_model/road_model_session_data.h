

#pragma once

#include "utils/process_frame.h"
#include "road_model/road_model_meta.h"
#include "dao/data_processer.h"

#include "../version/version.h"

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
                double pos_gap = alg::calc_dis(prev_node->pos, raw_node->pos);
                if (pos_gap >= min_gap) {
                    double z_diff = raw_node->pos.z() - prev_node->pos.z();
                    double rate = min_gap / pos_gap;
                    alg::calc_dir(raw_node->pos, prev_node->pos, prev_node->dir); //计算prev_node的朝向
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
                prev_node->dir = alg::get_dir(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += alg::calc_dis(node->pos, prev_node->pos);
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
            //     double pos_gap = alg::calc_dis(prev_node->pos, raw_node->pos);
            //     if (pos_gap >= min_gap) {
            //         double z_diff = raw_node->pos.z() - prev_node->pos.z();
            //         double rate = min_gap / pos_gap;
            //         alg::calc_dir(raw_node->pos, prev_node->pos, prev_node->dir); //计算prev_node的朝向
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
                prev_node->dir = alg::get_dir(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += alg::calc_dis(node->pos, prev_node->pos);
                node->line_length = total_length;
            }
            prev_node = node;
        }
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

    LaneCenterFeature* sample_new_lc(LaneCenterFeature* lc, LaneCenterFeature* next, 
        Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir, GenLaneLineParam* param);

    LaneCenterFeature* sample_new_lc(LaneCenterFeature* lc, 
        Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir, GenLaneLineParam* param);

    DisDirPoint sample_lane_center_side(LaneCenterFeature* start_lc,
            Eigen::Vector3d &break_pos, Eigen::Vector3d &break_dir,
            bool left, GenLaneLineParam* param);

    // Eigen::Vector3d get_lc_context_dir(LaneCenterFeature* lc, LaneCenterFeature* next, bool left, int mode);

    double align_point(LaneCenterLine *ll, BreakPos &point, bool is_prev, double max_opt_dis);

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
    std::vector<std::shared_ptr<KeyPose>> raw_key_poses; // ins的数据
    std::vector<std::shared_ptr<KeyPoseLine>> raw_links; // link关系，每条车道线上的每个link node点
    std::vector<std::shared_ptr<KeyPose>> raw_links_points; // link关系点， 所有的 link node 点， 通过 tar_pt->prev == nullptr 可以区分是哪条线的
    std::vector<std::shared_ptr<Intersection>> raw_intersections; // 交叉路口（十字路口）
    std::vector<std::shared_ptr<Feature>> raw_features; // BEV感知结果:车道中心线、车道边界线、道路边界线
    std::vector<std::shared_ptr<Feature>> raw_site_boundary_feature; // add by qzc: BEV感知结果:路口道路边界线
    std::vector<std::shared_ptr<Feature>> raw_boundary_feature; // BEV感知结果:道路边界线
    std::vector<std::shared_ptr<Feature>> raw_lane_feature; // BEV感知结果:车道边界线
    std::vector<std::shared_ptr<ObjectFeature>> raw_object_feature;
    std::vector<std::shared_ptr<LaneFeature>> raw_client_lane_feature; //人工标注的信息里面读取车道线信息
    std::vector<std::shared_ptr<BoundaryFeature>> raw_client_boundary_feature; //人工标注的信息里面读取道路边界
    std::vector<std::shared_ptr<LaneFeature>> lane_line_feature_ptr; // 初始化时由raw_features中的车道线信息存入
    //集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)、人工标记的boundary信息(session->label_boundary_instance_map)，
    //对曲线做平滑.
    // 在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它50范围内的，朝向角度差在60度范围内的boundary点，
    // 遍历这些筛选出的boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
    // 这些cross point也会放入这里
    std::vector<std::shared_ptr<BoundaryFeature>> site_boundary_feature_ptr; // add by qzc: 路口道路边界线
    std::vector<std::shared_ptr<BoundaryFeature>> boundary_feature_ptr; // 初始化时由raw_features中的栅栏信息、路沿、十字交叉入口信息存入，以及raw_boundary_feature的道路边界，也包括道路中间的黄色车道线
    std::vector<std::shared_ptr<LaneCenterFeature>> lane_center_feature_ptr; // 初始化时由raw_features中的车道中心线信息存入，后面对其做了降采样，还有通过车道线生成的车道中心点（同时对这些车道生成行驶方向反向的车道中心点，即方向向量反向，左右侧的车道线点也调换）
    std::vector<std::shared_ptr<LaneCenterFeature>> virtual_lane_center_feature_ptr; // 初始化时由raw_features中的虚线信息存入
    std::vector<std::shared_ptr<RoadObjectInfo>> raw_object_ret_list; // 目标物体，目标包含点云以及类型等属性
    // std::vector<std::shared_ptr<ObjectPointFeature>> object_feature_ptr;
    std::vector<std::shared_ptr<ObjectFeature>> object_ptr;
    std::vector<std::shared_ptr<KeyPose>> key_pose_ptr; // key_pose_map.list中pose之间间距满足2m的ins数据，以及与key_pose_list中位置匹配的Link点（把ins的pose映射到link每个点的pose）
    // 在整合了session->lane_line_instance_map、session->sem_lane_instance_map、
    // session->label_lane_line_instance_map中所有的车道线点后，在一起重新等间距采样后的车道线.
    // 在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它30范围内的，朝向角度差在30度范围内的车道线点，
    // 遍历这些筛选出的车道线点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
    // 这些cross point也会放入这里，以及后面sample_new_lc根据车道线中心点新增的车道线点；
    // 包含RoadModelProcMatchLine::make_raw_lane_sample()中由session->lane_center_feature_merge_raw中的车道中心点在lane_line_sample_tree中搜索最相近的车道线点得到的车道线;
    std::vector<std::shared_ptr<LaneLineSample>> lane_line_sample_ptr;
    // 遍历每条link线，对他们判断link线里是不是有断点（相邻link点之间的距离大于10m），
    // 如果有，就在断点处分割成一段一段，存放在此处
    std::vector<std::shared_ptr<RoadSegment>> road_segment_ptr;
    std::vector<std::shared_ptr<SubRoadSegment>> sub_road_segment_ptr;
    std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_group_line_ptr;// 由merge_lane_center_list构建的反向车道中心线点

    std::vector<std::shared_ptr<KeyPoseLine>> key_pose_line_ptr; // 与key_pose_list中位置匹配的raw_links点（把ins的pose映射到link每个点的pose），并且对其进行了降采样
    std::vector<std::shared_ptr<LaneFeatureLine>> lane_line_ptr;
    std::vector<std::shared_ptr<LaneLineSampleLine>> lane_line_sample_line_ptr;
    // add by qzc
    std::vector<std::shared_ptr<LaneLineSampleLine>> lane_line_sample_line_ptr_for_merge;
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_ptr; //bev感知车道中心线采样结果
    // add by qzc
    std::vector<std::shared_ptr<LaneCenterLine>> lane_center_line_ptr_for_merge; //bev感知车道中心线采样结果
    std::vector<std::shared_ptr<RoadLaneInfo>> road_lane_line_ptr;
    std::vector<std::shared_ptr<LaneCenterFeatureMatchPair>> lane_center_line_match_pair_ptr;
    //集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)、人工标记的boundary信息(session->label_boundary_instance_map)，
    //对曲线做平滑
    std::vector<std::shared_ptr<BoundaryLine>> boundary_line_ptr;
    // add by qzc
    std::vector<std::shared_ptr<BoundaryLine>> boundary_line_ptr_for_merge;
    std::vector<std::shared_ptr<RoadCenterFeature>> road_center_ptr; // 根据boundary计算出来的道路中心点

    std::vector<std::shared_ptr<RoadLaneGroupInfo>> lane_group_info_ptr;
    std::vector<std::shared_ptr<RoadLaneBoundaryInfo>> lane_boundary_info_ptr;
    std::vector<std::shared_ptr<RoadLinePointInfo>> line_point_info_ptr;
    std::vector<std::shared_ptr<RoadBoundarySegmentInfo>> boundary_segment_info_ptr;
    std::vector<std::shared_ptr<RoadBoundaryPointInfo>> boundary_point_info_ptr;
    std::vector<std::shared_ptr<RoadObjectInfo>> road_object_info_ptr;

    std::vector<std::shared_ptr<JunctionInfo>> new_junction_list; // 路口边界
    std::vector<std::shared_ptr<ImpassableAreaInfo>> new_area_list; // 不可通行区域

    std::vector<KeyPose*> key_pose_list; // key_pose_map.list中pose之间间距满足2m的ins数据

    // std::vector<std::shared_ptr<Intersection>> intersection_ptr;

    std::vector<std::shared_ptr<InterInfo>> inter_info_ptr; 
    std::vector<InterInfo*> inter_infos;
    std::vector<std::shared_ptr<LinkInfo>> link_info_ptr; 
    std::vector<LinkInfo*> link_infos;


    std::vector<Intersection*> intersections;  // 初始化时直接从 raw_intersections 赋值过来
    //集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)，对曲线做平滑后的结果
    UMAP<std::string, std::vector<BoundaryLine*>> bev_frame_boundary_line;
    UMAP<std::string, std::vector<LaneLineSampleLine*>> bev_frame_lane_line; // bev感知车道线的采样结果
    UMAP<std::string, std::vector<LaneCenterLine*>> bev_frame_lane_center_line; // bev感知车道中心线的采样结果
    UMAP<std::string, LaneFeatureLine> lane_line_instance_map; //初始化时由raw_features中的车道线信息存入
    UMAP<std::string, LaneFeatureLine> sem_lane_instance_map; //初始化时由raw_lane_feature中的车道线信息存入
    UMAP<std::string, LaneFeatureLine> label_lane_line_instance_map;  // 初始化时由raw_client_lane_feature存入，人工标注的信息里面读取车道线信息
    UMAP<std::string, KeyPoseLine> key_pose_map; // 初始化的时候把raw_key_poses的结果放入key_pose_map，并且按一定的间距做了稀疏化
    // KeyPoseTree origin_poss_tree_; // ins数据

    UMAP<std::string, LaneCenterLine> lane_center_instance_map; // 初始化时由raw_features中的车道中心线信息存入
    UMAP<std::string, LaneCenterLine> virtual_lane_center_instance_map; // 初始化时由raw_features中的虚线信息存入
    UMAP<std::string, BoundaryLine> barrier_instance_map; // 初始化时由raw_features中的栅栏信息信息存入
    UMAP<std::string, BoundaryLine> curb_instance_map; // 初始化时由raw_features中的路沿信息存入
    UMAP<std::string, BoundaryLine> junction_instance_map; // 初始化时由raw_features中的十字交叉入口信息存入
    UMAP<std::string, BoundaryLine> sem_curb_instance_map; // 初始化时由 raw_boundary_feature 的道路边界存入
    UMAP<std::string, BoundaryLine> site_boundary_instance_map; // add by qzc: 初始化时由 site_boundary_feature_ptr 的道路边界存入
    UMAP<std::string, BoundaryLine> label_boundary_instance_map;
    UMAP<std::string, ObjectFeature> object_instance_map; // 初始化时由raw_features中的object信息存入
    UMAP<int, int> double_geo_map;

    //遍历每条link线，对他们判断link线是不是有断点（相邻link点之间的距离大于10m），
    // 如果有，就在断点处分割成一段一段，存放在此处，按照长度由长到短排序
    std::vector<RoadSegment*> road_segment_list;

    std::vector<SubRoadSegment*> sub_road_segment_list;
    std::vector<KeyPose*> pose_sample_list;

    std::vector<KeyPoseLine*> link_sample_list; // 与key_pose_list中位置匹配的raw_links点（把ins的pose映射到link每个点的pose），并且对其进行了降采样
    std::vector<LaneFeatureLine*> lane_line_sample_list;
    std::vector<LaneLineSampleLine*> lane_line_sample_list_opt;// 在整合了session->lane_line_instance_map、session->sem_lane_instance_map、session->label_lane_line_instance_map 中所有的车道线点后，在一起重新等间距采样
    std::vector<LaneCenterLine*> lane_center_line_sample_list;
    std::vector<LaneCenterLine*> lane_center_line_sample_list_opt; // bev感知车道中心线的采样结果
    std::vector<BoundaryLine*> boundary_line_sample_list;
    //集合BEV感知的栅栏信息(session->barrier_instance_map)、BEV感知的路沿信息(session->curb_instance_map)、
    //BEV感知的boundary信息(session->sem_curb_instance_map)、人工标记的boundary信息(session->label_boundary_instance_map)，
    //对曲线做平滑后的结果
    std::vector<BoundaryLine*> boundary_line_sample_list_opt;
    std::vector<BoundaryLine*> new_boundary_line_list;

    // 存放每条路线中融合后的所有车道线，从每条路线的lane_line_group_list读入
    std::vector<LaneLineSampleGroupLine*> merge_lane_line_list;
    // 存放每条路线中融合后的所有车道中心线，从每条路线的lane_center_line_group_list读入；由merge_lane_center_list构建的反向车道中心线点会重新放入
    std::vector<LaneCenterGroupLine*> merge_lane_center_list;
    // 存放每条路线中融合后的所有boundary曲线，从每条路线的boundary_line_group_list读入
    std::vector<BoundaryGroupLine*> merge_boundary_line_list;

    std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_line_group_list;
    // 由lane_center_feature_raw产生，去除了一些无效点，参考函数match_lane_center_line_raw
    std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_line_group_list_raw;

    RTreeProxy<KeyPose*, float, 2> link_pos_tree;  // key_pose_line_ptr 构建的树
    RTreeProxy<KeyPose*, float, 2> key_pose_tree;  // key_pose_list 的树
    RTreeProxy<KeyPose*, float, 2> road_segment_link_tree; // road_segment_list构建的树

    // match_lane
    std::vector<Eigen::Vector3d> line_infos;
    RTreeProxy<LaneLineSample*, float, 2> lane_line_sample_tree;   // 所有路线中的车道线信息，每条路线中的车道线分别进行了曲线融合，由lane_line_sample_list_opt构建
    RTreeProxy<LaneCenterFeature*, float, 2> merge_lane_center_sample_tree;   // 由merge_lane_center_list 构建
    RTreeProxy<LaneCenterFeature*, float, 2> raw_lane_center_sample_tree; // 由session->merge_lane_center_list构建
    // 将merge_lane_center_list中的车道中心线点的有效点，按左右1.5m偏移作为左右侧车道线点，根据左右侧车道线点取平均值作为车道中心点的位置。
    // 按next_max_length由大到小排序
    std::vector<LaneCenterFeature*> lane_center_feature_raw;
    std::vector<LaneCenterFeature*> lane_center_feature_gen; // 通过车道线生成的车道中心点（同时对这些车道生成行驶方向反向的车道中心点，即方向向量反向，左右侧的车道线点也调换）
    std::vector<LaneCenterFeature*> lane_center_feature_bind; // 这里面的车道中心点都用来成功构造过各自的cross_point
    std::vector<LaneCenterFeature*> lane_center_feature_merge;
    // 最开始由lane_center_line_group_list_raw传入，
    // 后面在RoadModelProcMatchLine里需要用真实的车道线点计算车道中心点位置
    std::vector<LaneCenterFeature*> lane_center_feature_merge_raw;
    // 最开始来自于lane_center_line_group_list_raw，并把里面每个点的context.all_next和context.all_prev的score都置为0
    std::vector<LaneCenterFeature*> lane_center_group_list_raw;

    //gen_lane_line
    RTreeProxy<LaneCenterFeature*, float, 2> lane_center_gen_tree;  // 由lane_center_feature_gen构建，到RoadModelProcGenLaneLine里被清除， 由lane_center_line_group_list_raw构建
    RTreeProxy<LaneCenterFeature*, float, 2> bind_lane_center_tree;  // 由 session->road_segment_list里每个road_segment->pos_sample_list里每个poss->lane_center_list构造

    // 来自于session->road_segment_list里面每个poss的lane_center_list，
    // 按照优先车道数量少的 -> 车道数量一样，车道类型一样，分数高的 -> 车道数量一样，车道类型不一样，一车道的正常车道排前面
    // 的原则排序
    std::vector<LaneCenterFeature*> lane_center_list_all;

    //gen_lane_line
    // 所有路线中的boundary信息，每条路线中的boundary分别进行了曲线融合
    RTreeProxy<BoundaryFeature*, float, 2> boundary_line_sample_tree;

    // identify_road
    std::vector<std::shared_ptr<RoadSegment> > identify_road_segment_ptr;
    // std::vector<RoadSegment*> join_road_segment;
    // std::vector<RoadSegment*> attr_road_segment;
    // std::vector<RoadSegment*> merge_road_segment;
    // std::vector<RoadSegment*> final_road_segment;

    RTreeProxy<KeyPose*, float, 2> final_poss_sample_tree;

    RTreeProxy<LaneCenterFeature*, float, 2> valid_lane_sample_group_tree;

    std::vector<RoadLaneGroupInfo*> new_lane_groups;
    std::vector<RoadBoundarySegmentInfo*> new_boundary_segment;
    std::vector<RoadObjectInfo*> new_object_list;
    
    utils::CloudPtr raw_pcd;

    std::vector<std::shared_ptr<KeyPose>> raw_ground_pcd;
    utils::CloudPtr opt_ground_pcd; // 激光地面点

    //gen_key_point
    pcl::PointCloud<pcl::PointXYZ>::Ptr opt_label_lane_pcd;
    pcl::PointCloud<pcl::PointXYZ>::Ptr roadedge_pcd;
    pcl::PointCloud<PointXYZ_OPT>::Ptr lane_line_lidar_pcd;
    pcl::PointCloud<PointXYZ_OPT>::Ptr boundary_lidar_pcd;

    RTreeProxy<LaneFeature*, float, 2> lane_line_lidar_tree; // 车道线采样点（通过样条曲线拟合后采样得到）
    RTreeProxy<BoundaryFeature*, float, 2> boundary_lidar_tree; // 道路边界采样点（通过样条曲线拟合后采样得到）
    RTreeProxy<RoadObjectInfo*, float, 2> object_tree;   

    //bind relation
    RTreeProxy<LaneCenterFeature*, float, 2> road_lane_center_tree;

    //wq test
    //test boundary tree
    RTreeProxy<BoundaryFeature*, float, 2> format_boundary_tree;
    std::vector<std::shared_ptr<RoadBoundarySegmentInfo>> tmp_boundary_segment_info_ptr;
    std::vector<std::shared_ptr<RoadBoundaryPointInfo>> tmp_boundary_point_info_ptr;

    pcl::KdTreeFLANN<utils::CloudPoint> ground_kdtree;

    //cxf add
    std::string version = ROAD_MAPPING_VERSION_STRING;
    std::vector<fsdmap::dao::GisPolygon> task_polygon;
    std::vector<Eigen::Vector3d> task_polygon_vec3d;
    std::vector<std::vector<Eigen::Vector3d>> side_boundary_polygons;
    std::vector<std::shared_ptr<BreakPointKeyPose>> break_point_key_pose_list;  //路口打断点

    //识别断点调试用，调试完注释关闭
    //进入
    std::vector<Eigen::Vector3d> lc_pts_eigen_in, point0_in;
    std::vector<Eigen::Vector3d>  sl_cross_pt_in, cw_cross_pt_in, inter_cross_pt_in, break_pt_in;

    //退出
    std::vector<Eigen::Vector3d> lc_pts_eigen_out, point0_out;
    std::vector<Eigen::Vector3d>  sl_cross_pt_out, cw_cross_pt_out, inter_cross_pt_out, break_pt_out;


    std::map<KeyPose*, LaneCoverage> lane_coverage_m; //Link点对应的车道覆盖度

    std::map<KeyPose*, std::vector<std::shared_ptr<RoadCrossPoints>>> key_pose_info_m;  //第一个KeyPose为起始点，第二个vector为对应处理的这条线

    // export shp type
    // int export_type_ = -1; // -1: 不合法， 0: 矢量化， 1：车道线去重后的矢量化，2：矢量化+拓扑
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
