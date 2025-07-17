

#pragma once

#include <atomic>
#include <iostream>
#include <unordered_map>
#include <functional>
#include <Eigen/Core>
#include <gflags/gflags.h>
#include <unordered_set>

#include "utils/RTree.h"
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include "utils/common_util.h"
#include "utils/algorithm_util.h"
#include "utils/visualization_util.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/traffic_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/lane_proxy.h"
#include "../thirdparty/hdmap_server/data-access-engine/proxy/position_proxy.h"

#include <cmath> 




template<typename K, typename V>
using UMAP = std::map<K, V>;
template<typename T>
using PTR_VEC = std::vector<std::shared_ptr<T>>;

namespace fsdmap {

// extern bool enable_debug_pos;

struct KeyPose;
struct LaneCenterFeatureMatchPair;
struct RoadBoundary;
struct RoadSegment;
struct SubRoadSegment;
struct LaneLineSample;
struct Intersection;
struct Feature;
struct LaneFeature;
struct BoundaryFeature;
struct LaneCenterFeature;
struct ObjectFeature;
struct LaneCenterLine;
struct LaneLineSampleLine;
struct RoadCenterFeature;

struct RoadLaneGroupInfo;
struct RoadLaneBoundaryInfo;
struct RoadLaneInfo;
struct RoadLinePointInfo;
struct RoadBoundarySegmentInfo;
struct RoadBoundaryPointInfo;
struct RoadObjectInfo;
struct JunctionInfo;
struct ImpassableAreaInfo;
struct BoundaryGroupLine;
struct LaneLineSampleGroupLine;
struct LaneCenterGroupLine;
struct BreakPos;
struct LinkInfo;
struct KeyPoseLine;
struct BreakPointKeyPose;

enum ELEMENT_TYPE {
    ELEMENT_NULL,
    ELEMENT_LANE_LINE,
    ELEMENT_LANE_CENTER,
    ELEMENT_ROAD_CENTER,
    ELEMENT_BARRIER,
    ELEMENT_CURB,
    ELEMENT_POSE,
    ELEMENT_OBJECT,
    ELEMENT_JUNCTION,
    ELEMENT_VIRTUAL_LANE_LINE,
    ELEMENT_TRAFFICLIGHT,
};

struct ContextParam{
    double score;
};

template<class T>
struct ParamPair : public std::enable_shared_from_this<ParamPair<T>> {
    ParamPair() {
    }
    ParamPair(T* src, double dis, double score) : src(src), dis(dis), score(score), status(0) {
    }
    T* src;
    double dis;
    double score;
    int status;
    bool invalid() {
        return status < 0 || status >= 10;
    }
};

template<class T>
struct LineContext {
    std::vector<ParamPair<T>> all_prev;
    std::vector<ParamPair<T>> all_next;
    std::vector<ParamPair<T>> invalid_prev;
    std::vector<ParamPair<T>> invalid_next;
    // std::vector<double> score_prev;
    // std::vector<double> score_next;
    std::map<T*, int> around_to_type;
    double prev_max_length;
    double next_max_length;
    double max_length;
    double prev_max_score;
    double next_max_score;
    double max_score;

    bool set_next(T* t, double score=0) {
        if (MAP_FIND(around_to_type, t)) {
            return false;
        }
        around_to_type[t] = 1;
        all_next.resize(all_next.size() + 1);
        all_next.back().src = t;
        all_next.back().score = score;
        all_next.back().status = 1;
        return true;
    }
    bool set_prev(T* t, double score=0) {
        if (MAP_FIND(around_to_type, t)) {
            return false;
        }
        around_to_type[t] = 2;
        all_prev.resize(all_prev.size() + 1);
        all_prev.back().src = t;
        all_prev.back().score = score;
        all_prev.back().status = 1;
        return true;
    }

    T* prev() {
        if (all_prev.size() == 0) {
            return NULL;
        }
        for (auto &lc : all_prev) {
            if (lc.invalid()) {
                continue;
            }
            return lc.src;
        }
        return NULL;
    }
    T* next() {
        if (all_next.size() == 0) {
            return NULL;
        }
        for (auto &lc : all_next) {
            if (lc.invalid()) {
                continue;
            }
            return lc.src;
        }
        return NULL;
    }
    int64_t valid_next_size() {
        int64_t valid_num = 0;
        for (auto &lc : all_next) {
            if (lc.invalid()) {
                continue;
            }
            ++valid_num;
        }
        return valid_num;
    }
    int64_t valid_prev_size() {
        int64_t valid_num = 0;
        for (auto &lc : all_prev) {
            if (lc.invalid()) {
                continue;
            }
            ++valid_num;
        }
        return valid_num;
    }

    template<typename Func>
    double get_context_list(double scope, T* curr, std::vector<T*> &ret, 
            Func valid_node, int mode = 0) {
        std::vector<T*> prev_vec;
        std::vector<T*> next_vec;
        double max_length = 0;
        if (mode != 1) {
            double prev_length = 0;
            for (auto &tmp_lc_ptr : all_prev) {
                auto tmp_lc = tmp_lc_ptr.src;
                if (!valid_node(curr, tmp_lc_ptr)) {
                    continue;
                }
                double dis = alg::calc_dis(tmp_lc->pos, curr->pos);
                if (dis > scope) {
                    continue;
                }
                prev_vec.push_back(tmp_lc);
                double length = tmp_lc->context.get_context_list(scope - dis, tmp_lc, prev_vec,
                        valid_node, 2);
                prev_length = std::max(length + dis, prev_length);
            }
            for (int i = 0; i < prev_vec.size(); ++i) {
                int index = prev_vec.size() - 1 - i;
                ret.push_back(prev_vec[index]);
            }
            max_length += prev_length;
        }
        if (mode != 2) {
            double next_length = 0;
            for (auto &tmp_lc_ptr : all_next) {
                auto tmp_lc = tmp_lc_ptr.src;
                if (!valid_node(curr, tmp_lc_ptr)) {
                    continue;
                }
                double dis = alg::calc_dis(tmp_lc->pos, curr->pos);
                if (dis > scope) {
                    continue;
                }
                next_vec.push_back(tmp_lc);
                double length = tmp_lc->context.get_context_list(scope - dis, tmp_lc, next_vec,
                        valid_node, 1);
                next_length = std::max(length + dis, next_length);
            }
            ret.insert(ret.end(), next_vec.begin(), next_vec.end());
            max_length += next_length;
        }
        return max_length;
    }
    // src后继scope范围内的点，遇到tar停止
    bool on_same_line(double scope, T* src, T* tar, bool next, 
            UMAP<T*, std::vector<T*>>* line_map) {
        return on_same_line(scope, src, tar, next, [](ParamPair<T> &c)->bool{
                return !c.invalid();}, line_map);
    }

    // 寻找src后继scope范围内的点，遇到tar停止
    template<typename Func>
    bool on_same_line(double scope, T* src, T* tar, bool next, 
            Func valid_node, UMAP<T*,  std::vector<T*>> *line_map) {
        if (tar == src) {
            return true;
        }
        bool has_valid = false;
        auto &next_vec = next ? src->context.all_next : src->context.all_prev;
        for (auto &next_lc_ptr : next_vec) {
            auto &next_lc = next_lc_ptr.src;
            if (!valid_node(next_lc_ptr)) {
                continue;
            }
            double dis = alg::calc_dis(next_lc->pos, src->pos);
            if (dis > scope) {
                continue;
            }
            if (next_lc->context.on_same_line(scope - dis, next_lc, tar, next, valid_node, line_map)) {
                if (line_map != NULL) {
                    (*line_map)[src].push_back(next_lc);
                }
                has_valid = true;
                // return true;
            }
        }
        return has_valid;
    }


};


struct BasePoint {
    Eigen::Vector3d pos; // utm 局部坐标，基本上用来存储在(0,0,0)坐标系下的坐标
    std::string wgs();
    std::string wgs(const char * sep);
    Eigen::Vector3d wgs_pos();
    virtual std::string local();
    virtual std::string local(utils::DisplayScope &box, bool need_scale=false);
};

struct DirPoint : public BasePoint{
    Eigen::Vector3d dir;
};

struct LabelPoint : public BasePoint{
    int64_t ins_id;
};

struct DisDirPoint : public DirPoint {
    bool is_valid = false;
    double dis;
    void set_pos(Eigen::Vector3d pos, Eigen::Vector3d dir) {
        this->pos = pos;
        this->dir = dir;
        this->is_valid = true;
    }
};

struct BindKeyPosePoint : public DirPoint {
    // 对于车道线、boundary等点，在遍历road_segment_list里面每段road_segment的每个位置poss，搜索离它30范围内的，朝向角度差在30度范围内的车道线、boundary点，
    // 遍历这些筛选出的车道线、boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线、boundary上的点）的方向向量的交点cross point，
    // 这些cross point的key_pose就是poss的位置(可以理解为对应的轨迹位置)
    KeyPose* key_pose;
    RoadCenterFeature* road_center;
    // 对于车道线、boundary，就是他们的cross point（车道线或者boundary的方向向量与poss方向的法向量的交点）的位置pos与poss的位置的距离，在左侧是负值，右侧是正值
    // 对于道路中心点，就是中心点与poss的位置的距离，在左侧是负值，右侧是正值
    double bind_dis;
    std::string save(utils::DisplayScope &box, const char * file);
};

template<class T>
struct MatchPairBase {
    T* left;
    T* right;
    double score;
    bool match;
    T* get_other(T* pos) {
        if (pos == NULL) {
            return left;
        }
        if (left == pos) {
            return right;
        }
        if (right == pos) {
            return left;
        }
        return NULL;
    }
    virtual std::string local() {
        return utils::fmt("{:.2f},{:.2f};{:.2f},{:.2f}", 
                left->pos.x(), left->pos.y(), right->pos.x(), right->pos.y());
    }
};

template<class T>
struct LineNodeBase : public BindKeyPosePoint,
    std::enable_shared_from_this<T> {
public:
    T* prev;
    T* next;
    std::string line_id; // 所属的曲线的id， 对于车道中心线是 utils::fmt("{}_{}", lc->left->line_id, lc->right->line_id)

    int64_t line_index; // 曲线上的点在曲线上的索引位置
    double line_length; // 到当前位置的总长度（对于轨迹就是走过的长度，对于车道线就是车道线的长度）
    double curvature;
    Eigen::Vector3d prev_dir;
    Eigen::Vector3d next_dir;
    Eigen::Vector3d context_dir; // 寻找当前节点前后大于一定距离的点，分别计算他们的方向向量，再做平均

    int filter_status;
    float smooth_score=0;
    float junction_score=0;
    float trajector_distribution=0;

    template<class S>
    void init(S* raw) {
        this->pos = raw->pos;
        this->dir = raw->dir;
        this->line_index = raw->line_index;
        this->line_length = raw->line_length;
        this->line_id= raw->line_id;
        this->key_pose= raw->key_pose;
        line_id = raw->line_id;
        line_index = raw->line_index;
        line_length = raw->line_length;
        smooth_score=raw->smooth_score;
        trajector_distribution=raw->trajector_distribution;
    }
    virtual bool invalid() {
        return filter_status > 1;
    }

    virtual T* get_this() {
        return reinterpret_cast<T*>(this);
    };

    virtual std::string& get_line_id() {
        return line_id;
    }

    virtual Eigen::Vector3d& get_pos() {
        return pos;
    }

    void set_prev(T* new_prev) {
        get_this()->prev = new_prev;
        new_prev->next = get_this();
    }

	virtual bool is_same_line(T *node, double max_dis=50, int mode = 0) {
		if (node == this) {
			return true;
		}
		if (node == NULL) {
			return false;
		}
		if (node->get_line_id() != this->get_line_id()) {
			return false;
		}

		double gap = node->line_length - this->line_length;
		if (fabs(gap) > max_dis) {
			return false;
		}
		if (mode == 1 && gap < 0) {
			// 1 往前找
			// check_prev = false;
			return false;
		} else if (mode == 2 && gap > 0) {
			// 1 往后找
			// check_next = false;
			return false;
		}
		return true;
    }

    void calc_context_dir(std::vector<double> &radii, int mode=0) {
        if (mode != 1) {
            calc_context_dir(radii, true);
        }
        if (mode != 2) {
            calc_context_dir(radii, false);
        }
        context_dir = prev_dir + next_dir;
        context_dir.normalize();
    }

    void calc_context_dir(std::vector<double> &radii, bool prev) {
        std::vector<int> status_list(radii.size(), 0);
        T* curr_node = get_this();
        auto &dir_ret = prev ? prev_dir : next_dir;
        dir_ret = {0, 0, 0};
        int curr_index = 0;
        while (SUM(status_list, 0) < status_list.size()) {
            auto next = prev ? curr_node->prev : curr_node->next;
            if (next == NULL) {
                break;
            }
            double dis = alg::calc_dis(next->pos, get_this()->pos);
            for (int i = curr_index; i < radii.size(); ++i) {
                auto &radius = radii[i];
                // 减少计算
                if (status_list[i] == 0 && dis < radius) {
                    break;
                }
                if (status_list[i] == 0 && dis >= radius) {
                    if (prev) {
                        dir_ret += alg::get_dir(get_this()->pos, next->pos);
                    } else {
                        dir_ret += alg::get_dir(next->pos, get_this()->pos);
                    }
                    status_list[i] = 1;
                    curr_index = i + 1;
                }
            }
            curr_node = next;
        }
        if (SUM(status_list, 0) == 0) {
            dir_ret = dir;
        }  else {
            dir_ret.normalize();
        }
    }

    virtual double get_context_list(double scope, std::vector<T*> &ret, int mode = 0) {
        return get_context_list(scope, ret, 
                [](T* curr)->T* {return curr->prev;},
                [](T* curr)->T* {return curr->next;},
                mode);
    }

    virtual double get_context_list(double scope, std::vector<T*> &ret, 
            T*(*get_prev)(T* curr), T*(*get_next)(T* curr), int mode = 0) {
		ret.clear();
        double get_length = 0;
		if (this == NULL) {
            return get_length;
		}
        std::unordered_map<T*, int> used_map;
		double total_length = 0;
		std::vector<T*> prev_vec;
		std::vector<T*> next_vec;
        if (mode != 1) {
            T* curr = get_this();
            prev_vec.reserve(int (scope + 1));
            while (get_prev(curr) != NULL) {
                double dis = alg::calc_dis(curr->get_pos(), get_prev(curr)->get_pos(), true);
                total_length += dis;
                if (total_length > scope) {
                    break;
                }
                curr = get_prev(curr);
                if (used_map.find(curr) != used_map.end()) {
                    break;
                }
                used_map[curr] = 1;
                prev_vec.push_back(curr);
                get_length += dis;
            }
        }
		total_length = 0;
        if (mode != 2) {
            T* curr = get_this();
            next_vec.reserve(int (scope + 1));
            while (get_next(curr) != NULL) {
                double dis = alg::calc_dis(curr->get_pos(), get_next(curr)->get_pos(), true);
                total_length += dis;
                if (total_length > scope) {
                    break;
                }
                curr = get_next(curr);
                if (used_map.find(curr) != used_map.end()) {
                    break;
                }
                used_map[curr] = 1;
                next_vec.push_back(curr);
                get_length += dis;
            }
        }
		int prev_size = prev_vec.size();
		int next_size = next_vec.size();
		if (mode == 3) {
			int min_size = std::min(prev_size, next_size);
			prev_size = min_size;
			next_size = min_size;
		}
		for (int i = 1; i <= prev_size; ++i) {
			int index = prev_size - i;
			ret.push_back(prev_vec[index]);
		}
		ret.push_back(get_this());
		for (int i = 0; i < next_size; ++i) {
			ret.push_back(next_vec[i]);
		}
		return get_length;
	}

    // template<typename Func>
    // double get_context_list1(double scope, std::vector<T*> &ret, Func get_prev, Func get_next, int mode = 0) {
	// 	ret.clear();
    //     double get_length = 0;
	// 	if (this == NULL) {
    //         return get_length;
	// 	}
    //     std::unordered_map<T*, int> used_map;
	// 	double total_length = 0;
	// 	std::vector<T*> prev_vec;
	// 	std::vector<T*> next_vec;
    //     if (mode != 1) {
    //         T* curr = get_this();
    //         prev_vec.reserve(int (scope + 1));
    //         while (get_prev(curr) != NULL) {
    //             double dis = alg::calc_dis(curr->get_pos(), get_prev(curr)->get_pos(), true);
    //             total_length += dis;
    //             if (total_length > scope) {
    //                 break;
    //             }
    //             curr = get_prev(curr);
    //             if (used_map.find(curr) != used_map.end()) {
    //                 break;
    //             }
    //             used_map[curr] = 1;
    //             prev_vec.push_back(curr);
    //             get_length += dis;
    //         }
    //     }
	// 	total_length = 0;
    //     if (mode != 2) {
    //         T* curr = get_this();
    //         next_vec.reserve(int (scope + 1));
    //         while (get_next(curr) != NULL) {
    //             double dis = alg::calc_dis(curr->get_pos(), get_next(curr)->get_pos(), true);
    //             total_length += dis;
    //             if (total_length > scope) {
    //                 break;
    //             }
    //             curr = get_next(curr);
    //             if (used_map.find(curr) != used_map.end()) {
    //                 break;
    //             }
    //             used_map[curr] = 1;
    //             next_vec.push_back(curr);
    //             get_length += dis;
    //         }
    //     }
	// 	int prev_size = prev_vec.size();
	// 	int next_size = next_vec.size();
	// 	if (mode == 3) {
	// 		int min_size = std::min(prev_size, next_size);
	// 		prev_size = min_size;
	// 		next_size = min_size;
	// 	}
	// 	for (int i = 1; i <= prev_size; ++i) {
	// 		int index = prev_size - i;
	// 		ret.push_back(prev_vec[index]);
	// 	}
	// 	ret.push_back(get_this());
	// 	for (int i = 0; i < next_size; ++i) {
	// 		ret.push_back(next_vec[i]);
	// 	}
	// 	return get_length;
	// }
};

struct BreakPos : public BindKeyPosePoint {
    bool is_valid;
    LaneCenterFeature* src_lc;
    void init(Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
        this->pos = pos;
        this->dir = dir;
        is_valid = true;
    }
};


template<class T>
struct LineSample : public LineNodeBase<LineSample<T>> {
    int real_status;
    T* src;
    void init(LineSample<T>* raw_node) {
        // LineNodeBase::init<T>(raw_node);
        src = raw_node->src;
        this->line_id = raw_node->line_id;
    }

    void init(T* raw_node) {
        src = raw_node;
        this->line_id = raw_node->line_id;
    }

};


template<class T>
struct BoundaryParamPair : public std::enable_shared_from_this<BoundaryParamPair<T>> {
    BoundaryParamPair() {
    }
    BoundaryParamPair(T* src, double dis, double score) :src(src), dis(dis), score(score) {
    }
    T* src;
    double dis;
    double score;
};

// 每个RoadBoundary都对应一个轨迹上的pose
struct RoadBoundary : public std::enable_shared_from_this<RoadBoundary> {
    RoadCenterFeature* curr;  // 用road_center_list中左右boundary与自车位置间距最小的boundary计算得到的道路中心点
    std::vector<RoadCenterFeature*> road_center_list;  // 根据boundary计算出来的道路中心点，按照bind_dis进行排序，在pose左侧离pose最远的在最前面

    // 在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它50范围内的，朝向角度差在60度范围内的boundary点，
    // 遍历这些筛选出的boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
    // 这些cross point也会放入这里，并且按bind_dis由小到大排序
    std::vector<BoundaryFeature*> boundary_feature_list;

    std::vector<BoundaryParamPair<LaneLineSample>> yellow_feature_list;  // 该RoadBoundary的pose附近是黄线的车道线点，不是黄线但是是双线的也会放进去
    BoundaryParamPair<KeyPose> left_oppo_pos; // 与该RoadBoundary对应的轨迹点pose反向，在pose左侧，且离该点最近的轨迹点（从link上拿轨迹点）
    BoundaryParamPair<KeyPose> left_oppo_trail;  // // 在session->key_pose_tree半径50m范围内的点，与poss反向，在该点左侧，且离该点最近的轨迹点
    std::vector<BoundaryParamPair<KeyPose>> same_dir_pos;  // 分别在每条轨迹上，选出与该RoadBoundary对应的轨迹点pose同向且50m范围内，与该轨迹点pose垂向距离最近的点（从link上拿轨迹点）
    double curr_bind_dis_gap;
    double curr_bind_dis_var;
    int same_road_status;  // 表示该RoadBoundary处的道路中间有没有围栏，没有中间围栏就是1

    void reset() {
    }

    RoadCenterFeature* get_road_center(Eigen::Vector3d &pos, double buff=0);

    RoadCenterFeature* get_road_center(int rc_index);
};

struct RoadLaneParam {
    int32_t lane_num;
    double predict_lane_num;
    double avg_lane_width;
    double radius;
    // BreakPos break_pos;
    RoadBoundary boundary;
    double update_matched_offset;
};

// struct RoadLaneSampleScope {
//     BoundaryParamPair<LaneLineSample> *left;
//     BoundaryParamPair<LaneLineSample> *right;
// };

struct KeyPose : public LineNodeBase<KeyPose> {
    double timestamp = 0.f; //in ms
    std::string id; // int frame_id
    std::string trail_id; //img id
    std::string frame_id; //img id
    utils::CloudPtr pcd;
    float yaw = 0;
    Eigen::Vector3d wgs;
    Eigen::Quaterniond quat; // 目前是存的全局坐标系下的姿态：当前帧车身坐标系到地理坐标系的四元数
    Eigen::Matrix3d r;
    KeyPose* merge_prev;
    KeyPose* merge_next;
    
    KeyPose* src_poss;

    RoadSegment* road_segment;  // 对于一条link线，可能会被打断成多段link，每段用RoadSegment表示，road_segment表示每条link上的link点所属的RoadSegment
    Intersection* intersection;  // 该位置35半径范围内的intersection
    RoadLaneGroupInfo* lane_group;

    // 该pose轨迹点附近的道路边界信息，在遍历road_segment_list里面每段road_segment的每个位置poss时，搜索离它50范围内的，朝向角度差在60度范围内的boundary点，
    // 遍历这些筛选出的boundary点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（boundary）的方向向量的交点cross point，
    // 这些cross point会放入这里的boundary_feature_list
    RoadBoundary boundary;
    RoadLaneParam lane_param;
    BreakPos break_pos;
    int raw_no;
    int raw_file_no;
    int stop_count;
    int break_group_num;
    int left_boundary_status;
    int right_boundary_status;
    int break_status;
    int stop_status;
    int road_status;
    int inter_status; // 2: 进入路口， 3： 退出路口
    // RoadLaneSampleScope lane_scope;
    int clip_index;
    int in_out_status = 0;  // 0未处理， 1进入，  -1退出
   
    
    UMAP<int, int> lc_num_map;
    
    std::vector<KeyPose*> other_pos;

    LineContext<KeyPose> context;

     //根据感知现有车道中心点，结合各个车道中心点的左右侧车道线点重新生成的车道中心点
    // 由 RoadModelProcBindTrail::get_poss_cross_point_mutil生产，
    // 后面会按照优先车道数量少的 -> 车道数量一样，车道类型一样，分数高的 -> 车道数量一样，车道类型不一样，一车道的正常车道排前面
    // 的原则排序
    std::vector<LaneCenterFeature*> lane_center_list;
    // 从该poss的lane_center_list里面选出每个车道的唯一车道中心点（直道应该只有一个点，对于弯道会有几个点） ，
    // 且这些点必须是is_valid_lane()点
    std::vector<LaneCenterFeature*> pre_filled_lane_sample;
    // 从该poss的lane_center_list里面选出每个车道的唯一车道中心点（直道应该只有一个点，对于弯道会有几个点）
    // 这些车道中心点按照从最左车道到最右车道排序
    std::vector<LaneCenterFeature*> score_filled_lane_sample;
    std::vector<LaneCenterFeature*> filled_lane_sample; // 由score_filled_lane_sample放入
    std::vector<LaneCenterFeature*> valid_lane_sample;

    // 遍历这些筛选出的车道线点，对于每个点fls，计算poss(轨迹点)处方向向量的垂线与fls（车道线上的点）的方向向量的交点cross point，
    std::vector<LaneLineSample*> lane_line_feature_sample_list;
    // std::vector<LaneLineSample*> other_feature_line_sample;
    std::vector<BoundaryParamPair<LaneFeature>> bind_lidar_lane_line;
    std::vector<BoundaryParamPair<BoundaryFeature>> bind_lidar_boundary;

    std::vector<Feature*> stop_line_feature_list;
    std::vector<RoadObjectInfo*> object_list;

    KeyPoseLine* from_link;
    KeyPoseLine* from_raw_link; //未分组前link信息

    // void set_break_pos(Eigen::Vector3d &pos, Eigen::Vector3d &dir) {
    //     lane_param.break_pos.pos = pos;
    //     lane_param.break_pos.dir = dir;
    //     lane_param.has_break_pos = true;
    // }

    // void set_break_pos(DirPoint &break_pos) {
    //     set_break_pos(break_pos.pos, break_pos.dir);
    // }

    // void set_break_pos(KeyPose* poss) {
    //     set_break_pos(poss->pos, poss->dir);
    // }

    // DisDirPoint& get_break_pos() {
    //     if (!lane_param.has_break_pos) {
    //         lane_param.break_pos.pos = this->pos;
    //         lane_param.break_pos.dir = this->dir;
    //     }
    //     return lane_param.break_pos;
    // }

    Eigen::Vector3d& get_base_line_dir(int mode=0);

    bool invalid() {
        return filter_status >= 2;
    }
};


struct LaneAttr {
    int32_t geo;
    // // # enum Color { // 颜色   修改了看下面
    // //                 #     UNKNOWN_COLOR = 0;//未确认
    // //                 #     WHITE = 1;//白色
    // //                 #     YELLOW = 2;//黄色
    // //                 #     ORANGE = 3;//橙色
    // //                 #     BLUE = 4;//蓝色
    // //                 #     GREEN = 5;//绿色
    // //                 #     GRAY = 6;//灰色
    // //                 #     LEFT_GRAY_RIGHT_YELLOW = 7;//左灰右黄
    // //                 #     LEFT_YELLOW_RIGHT_WHITE = 8;//左黄右白
    // //                 #     LEFT_WHITE_RIGHT_YELLOW = 9;//左白右黄
    // //                 # }

    // # enum Color { // 颜色
    //                 #     UNKNOWN_COLOR = 1;//未确认
    //                 #     WHITE = 2;//白色
    //                 #     YELLOW = 3;//黄色
    //                 #     LEFT_YELLOW_RIGHT_WHITE = 4;//左黄右白
    //                 #     LEFT_WHITE_RIGHT_YELLOW = 5;//左白右黄
    //                 #     ORANGE = 6;//橘色
    //                 #     BLUE = 7;//蓝色
    //                 #     OTHER = 99;//其他
    //                 # }
    int32_t color;
    int32_t is_double_line; // 是否是双线
    int32_t is_bus;
    int32_t is_bold;
    int32_t is_short;
    int32_t is_emergency;
    int32_t deceleration;
    int32_t diversion;
    int32_t type;
    void init() {
        geo = -1;
        color = -1;
        is_double_line = -1;
        is_bold = -1;
        is_short = -1;
        deceleration = -1;
        diversion = -1;
        is_emergency = -1;
        is_bus = -1;
    }
};


struct FeatureAttr {
    std::string frame_id; // 帧id
    std::string trail_id; // 轨迹id
    // KeyPose* key_pose;
    double score; //根据feature离自车位置远近，计算feature的可靠性，离自车越近，可靠性分数越高
    int type;  //虚线 实线 
    int color; // 颜色
    int src_status; // 1：原有，来自原始感知；2：新增， 4：人工标注，
    Eigen::Vector3d raw_pos;
    int boundary_type = 0; // 0: 默认普通， 1： island，路口三角岛
    // Eigen::Vector3d centriod_pos; // 质心点位置
    // int process_status;
    void init(FeatureAttr * raw_feature) {
        this->frame_id = raw_feature->frame_id;
        this->trail_id = raw_feature->trail_id;
        this->score= raw_feature->score;
        this->type= raw_feature->type;
        this->color= raw_feature->color;
        this->raw_pos= raw_feature->raw_pos;
        this->src_status= raw_feature->src_status;
        this->boundary_type= raw_feature->boundary_type;
        // this->centriod_pos= raw_feature->centriod_pos;
    }
};

struct Feature : public LineNodeBase<Feature>, FeatureAttr {
    int stop_status;
    double length;
    double width;
    double height;
    double yaw;
    std::tuple<ELEMENT_TYPE, int, int, cv::Scalar> ele_type;
};

struct LaneFeature : public LineNodeBase<LaneFeature>, FeatureAttr {
    LaneAttr attr;
    Feature* src_feature;
    LaneFeature* src;

    virtual void init(Feature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        src_feature = raw_feature;
       this-> type=raw_feature->type;
       this-> color=raw_feature->color;

    }

    virtual void init(LaneFeature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        attr = raw_feature->attr;
        src_feature = raw_feature->src_feature;
        src = raw_feature;
    }
};

// using LaneLineSample = LineSample<LaneFeature>;

struct LaneLineSample : public LineSample<LaneFeature> {
    LaneAttr attr;
    LaneLineSample* src_fls;
    LaneLineSampleLine* line;
    LaneLineSampleGroupLine* group_line; // 该点所属的曲线
    int fill_status;
    int match_frame_num; // 离该点位置较近（轨迹上存在某点pose，pt到pose的方向向量和方向向量垂线距离都小于30m）的frame数量
    int match_trail_num; // 离该点位置较近（轨迹上存在某点pose，pt到pose的方向向量距离小于30m）的轨迹条数
    double score;
    UMAP<LaneLineSampleGroupLine*, LaneLineSample*> merge_match_map; // 记录 tar group 线，匹配到的 src 线段

    //add
    bool is_intersection_point = false;
    void init(LaneLineSample* raw) {
        if (raw == NULL) {
            real_status = 2;
            return;
        }
        pos = raw->pos;
        dir = raw->dir;
        real_status = 1;
        LineSample::init(raw);
        attr = raw->attr;
        match_frame_num = raw->match_frame_num;
        match_trail_num = raw->match_trail_num;
        // 
        // attr.color=raw->color;
        // attr.type=raw->type;
    }
    
    void init(LaneFeature* raw) {
        LineSample::init(raw);
        pos = raw->pos;
        dir = raw->dir;
        attr = raw->attr;
        attr.color=raw->color;
        attr.type=raw->type;

    }
};

struct BoundaryFeature : public LineNodeBase<BoundaryFeature>, FeatureAttr {
    //  大类型下面的具体的子类型，例如对于RoadBoundary来说，又可以分为以下的子类型
    // # enum RoadBoundaryType { // 道路边界类型
    // #     UNKNOWN_BOUNDARY = 0; //未知类型
    // #     //LANELINE = 1;
    // #     CURB = 2;//路缘石
    // #     //CENTER = 3;
    // #     GUARDRAIL = 4;//防护栏
    // #     CONCRETE_BARRIER = 5;//混凝土防护栏（新泽西防护栏）
    // #     FENCE = 6;//栅栏
    // #     WALL = 7;//保护墙
    // #     CANOPY = 8;//遮棚
    // #     PAVE = 9;//自然边界,铺设未铺设边界
    // #     DITCH = 10;//沟渠
    // #     PUNCHEON = 11;//离散型障碍物（包括短柱:可表达石墩\短柱等无法穿越的障碍物）
    // # }
    int sub_type;
    RoadCenterFeature* road_center;
    BoundaryFeature* attr_feature;
    Feature* src_feature;
    BoundaryGroupLine* group_line; // 该点所属的曲线
    int match_frame_num;
    int match_trail_num;
    // 在单轨迹车道线等特征融合过程中，与该点满足距离要求，以及方向一致的匹配点，以及该匹配点对应所属的曲线BoundaryGroupLine，
    // <该匹配点对应所属的曲线，与该点满足距离要求、以及方向一致的匹配点>
    UMAP<BoundaryGroupLine*, BoundaryFeature*> merge_match_map;
    //add
    bool is_intersection_point = false;
    virtual void init(Feature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        src_feature = raw_feature;
    }

    virtual void init(BoundaryFeature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        sub_type = raw_feature->sub_type;
        src_feature = raw_feature->src_feature;
        match_frame_num = raw_feature->match_frame_num;
        match_trail_num = raw_feature->match_trail_num;
    }
};

struct ObjectFeature : public BindKeyPosePoint, FeatureAttr {
    std::vector<Eigen::Vector3d> list;
    ObjectFeature* src_feature;
    int type;
    double length;
    double width;
    double height;
    std::tuple<ELEMENT_TYPE, int, int, cv::Scalar> ele_type;
    void init(ObjectFeature* raw_feature) {
        FeatureAttr::init(raw_feature);
        src_feature = raw_feature;
        list.insert(list.end(), raw_feature->list.begin(), raw_feature->list.end());
        type = raw_feature->type;
        pos = raw_feature->pos;
        dir = raw_feature->dir;
        length = raw_feature->length;
        width = raw_feature->width;
        height = raw_feature->height;
        ele_type = raw_feature->ele_type;
    }
};

// struct ObjectPointFeature : public LineNodeBase<ObjectPointFeature>, FeatureAttr {
//     Feature* src_feature;
//     virtual void init(Feature *raw_feature) {
//         FeatureAttr::init(raw_feature);
//         LineNodeBase::init(raw_feature);
//         src_feature = raw_feature;
//     }
// };

struct MatchLevel {
    int lane_type;  // 1：正常一车道， 2：有夹角的路口，汇入汇出车道
    int lane_num;  // 该车道中心点对应的车道数，正常是1或者2
    int join_out;  // 汇入汇出口，1 汇出，2 汇入， 0 直道
    bool is_same(MatchLevel &other);
    bool is_same_num(MatchLevel &other);
    bool is_same_type(MatchLevel &other);
};

struct LaneLineParam {
    double width_variance; // 车道宽度方差
    // 在某条车道中心线上，在线段两头一定距离内寻找新的车道中心点，
    // 从线段的头部往前找到的车道中心线为pre_line, 从线段的尾部往后找到的车道中心线为next_line,
    // 计算pre_line的平均车道宽度
    double front_avg_width;
    double back_avg_width; // 计算next_line的平均车道宽度 
    double avg_width; // 车道平均宽度，被放大了100倍
    double lc_width;
    double radius;
    double line_theta;
    double center_gap;
    double center_gap_variance; // 计算某点到他前后节点组成的向量prev_next的距离,负数表示lc在prev左侧，计算这些的方差
    double score; //  sqrt(length_factor * width_factor * dir_factor)，反映了长度、宽度、dir的方差大小，方差越大分数越小
    double opt_score;
    double length;
    double complete_length;
    double length_factor;
    double width_variance_factor;
    double width_standard_factor;
    double width_factor;
    double theta_factor;
    double gap_factor;
    double dir_factor;
    double min_width;
    double max_width;
    double total_score;
    double filled_distance;
    int64_t lane_size;
    // std::shared_ptr<LaneCenterLine> sample_lane;
};


//车道中心的特征结构体 
struct LaneCenterFeature : public LineNodeBase<LaneCenterFeature>, FeatureAttr {
    LaneLineSample* left;  // 对应的左侧车道线点
    LaneLineSample* right; // 对应的右侧车道线点
    LaneCenterFeature* raw_from_lc; // 该新产生的车道中心点，从哪段原有的车道中心产生，这里是起始点
    LaneCenterFeature* raw_to_lc; // 该新产生的车道中心点，从哪段原有的车道中心产生，这里是终点， 与group_pt是前后继节点的关系
    LaneCenterFeature* group_pt;  //与该车道线中心点最靠近的车道线中心点，5m范围内，方向一致，横向距离最短

    LaneCenterFeature* fill_prev;
    LaneCenterFeature* fill_next;
    LaneCenterFeature* fill_left;
    LaneCenterFeature* fill_right;
    BoundaryParamPair<LaneFeature> left_match_lidar;
    BoundaryParamPair<LaneFeature> right_match_lidar;
    KeyPose* identify_poss;
    BreakPos break_pos;

    LaneCenterLine* fill_line;

    LaneAttr left_attr;
    LaneAttr right_attr;

    int road_index;  // 同RoadCenterFeature::index，与该road centre所属的RoadBoundary里面的curr的index之差
    double width;  // 车道宽度
    double raw_width;
    double theta;
    int break_group_num;
    int filled_context_count;
    MatchLevel match_level;
    int gen_status;
    int dir_status;
    int fill_status;
    int boundary_status;
    int identify_status;
    int break_status;
    int stop_status;  
    int road_lane_index;
    int road_lane_max_index;
    int left_valid_status;
    int right_valid_status;
    // int src_status;
    int left_opt_status;
    int right_opt_status;
    int side_status;  // 1：表示在道路中心右侧， 2：表示在道路中心左侧
    int match_frame_num;
    int match_trail_num;
    int merge_status;
    int oppo_status;
    int length_status;  // 该车道中心点是否计算过从他开始到后面第一个无效节点的长度
    int lane_index;
    std::string turn_type;// 车道转向类型    
    //
    int refine_pos_status=0;  //0 1:left 1  2:right<<1 3:all


    Eigen::Vector3d left_pos;  // 车道中心线左侧车道线点
    Eigen::Vector3d right_pos; // 车道中心线右侧车道线点
    Eigen::Vector3d left_dir; // 车道中心线左侧车道线点方向
    Eigen::Vector3d right_dir; // 车道中心线右侧车道线点方向

    Feature* src_feature;

    // Eigen::Vector3d raw_pos;
    // Eigen::Vector3d raw_dir;
    // double raw_width;

    //  该车道中心点lc，从lc->raw_from_lc往前继续寻找车道中心线prev_line， 
    //  lc->raw_to_lc往后继续寻找车道中心线next_line，
    // 把prev_line与next_line组合形成多条新的车道中心线，在这些车道中心线里选出score最高的车道中心线的line_param给他
    LaneLineParam line_param;
    // std::shared_ptr<LaneCenterLine>> line;
    RoadSegment* road_segment;  //属于哪段路段
    RoadSegment* identify_segment;
    RoadLaneInfo* road_lane_line;  //指回自身车道结构体
    // 当前车道中心点lc与他的context.all_next里面的每个点next构造出来的车道中心点。
    // 根据lc的左右侧的车道线点构造新的车道中心点：
    // 以lc左侧车道点为起点，next左侧车道点与lc左侧车道点连线为方向向量，与pos（lc是pos 30m方位内与该位置处方向向量夹角在30度以内）为起点pos方向的垂向为
    // 方向向量的交点作为新的左侧车道线点，右侧车道线点按同理构造，最后用这新生成的这两个左右侧车道线点构建新的车道中心点。
    // <next, 新生成的车道中心点>>,
    // std::vector<LaneCenterFeature*>里面的元素会按与lc的距离从小到大排序
    std::map<LaneCenterFeature*, std::vector<LaneCenterFeature*> > cross_point;

    LineContext<LaneCenterFeature> context;  // 车道中心点的前后连接点关系

    // std::vector<LaneCenterFeature*> all_prev;
    // std::vector<LaneCenterFeature*> all_next;
    std::vector<LaneCenterFeatureMatchPair*> prev_pair_list;
    std::vector<LaneCenterFeatureMatchPair*> next_pair_list;
    UMAP<LaneCenterFeature*, LaneCenterFeatureMatchPair*> prev_pair_map;
    UMAP<LaneCenterFeature*, LaneCenterFeatureMatchPair*> next_pair_map;

    std::vector<LaneCenterFeature*> all_fill_prev;
    std::vector<LaneCenterFeature*> all_fill_next;
    std::vector<LaneCenterFeature*> conflict_vec;;  // 存放与该车道中心点是一个车道内的车道中心点

    LaneCenterGroupLine* group_line;  // 该点所属的曲线
    UMAP<LaneCenterGroupLine*, LaneCenterFeature*> merge_match_map; // 该点附近5m半径内的车道中心点，在这些点中选出距离在一个车道范围内的车道中心点，以及这个车道中心点所属的车道中心线

    


    virtual void init(Feature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        src_feature = raw_feature;
    }

    virtual void init(LaneCenterFeature *raw_feature) {
        FeatureAttr::init(raw_feature);
        LineNodeBase::init(raw_feature);
        left = raw_feature->left;
        right = raw_feature->right;
        left_pos = raw_feature->left_pos;
        right_pos = raw_feature->right_pos;
        left_dir = raw_feature->left_dir;
        right_dir = raw_feature->right_dir;
        width = raw_feature->width;
        raw_width = raw_feature->raw_width;
        theta = raw_feature->theta;
        match_level = raw_feature->match_level;
        left_attr = raw_feature->left_attr;
        right_attr = raw_feature->right_attr;
        src_feature = raw_feature->src_feature;
        match_frame_num = raw_feature->match_frame_num;
        match_trail_num = raw_feature->match_trail_num;
    }
    void init(LaneLineSample* l, LaneLineSample* r);
    void init(Eigen::Vector3d &l_pos, Eigen::Vector3d &l_dir, 
            Eigen::Vector3d &r_pos, Eigen::Vector3d &r_dir);

    bool is_filled();
    Eigen::Vector3d &get_left();
    Eigen::Vector3d &get_right();
    LaneCenterFeature* get_next(int index=0) {
        if (context.all_next.size() > index) {
            return context.all_next[index].src;
        }
        return NULL;
    }
    LaneCenterFeature* get_prev(int index=0) {
        if (context.all_prev.size() > index) {
            return context.all_prev[index].src;
        }
        return NULL;
    }
    void set_oppo() {
        std::swap(this->left, this->right);
        std::swap(this->left_pos, this->right_pos);
        std::swap(this->left_dir, this->right_dir);
        this->dir = -this->dir;
        this->left_dir = -this->left_dir;
        this->right_dir = -this->right_dir;
    }
    void set_z(double z, bool left);
    bool has_same_edge(LaneCenterFeature *lc);
    bool is_same_base(LaneCenterFeature *lc, int mode=0);
     
    // double get_fill_context_list(double scope, 
    //         std::vector<LaneCenterFeature*> &ret, int mode);
    bool in_scope(Eigen::Vector3d &pos, double buff=0, int mode=0);
    bool in_scope_stop_line(Eigen::Vector3d &stop_line_pos, const float& length, double buff=0, int mode=0);
};

using LCP = ParamPair<LaneCenterFeature>;

struct RoadCenterFeature : public LineNodeBase<RoadCenterFeature>, FeatureAttr {
    BoundaryFeature* left; // 道路中心点的左侧围栏点
    BoundaryFeature* right; // 道路中心点的右侧围栏点
    BoundaryFeature* raw_left;
    BoundaryFeature* raw_right;
    RoadCenterFeature* fill_left; // 离自己最近的左侧道路中心点
    RoadCenterFeature* fill_right; // 离自己最近的右侧道路中心点
    BoundaryParamPair<BoundaryFeature> left_match_lidar;
    BoundaryParamPair<BoundaryFeature> right_match_lidar;
    int index; // 与该road centre所属的RoadBoundary里面的curr的index之差
    // BoundaryFeature* raw_from_lc;
    // BoundaryFeature* raw_to_lc;

    // BoundaryFeature* fill_prev;
    // BoundaryFeature* fill_next;
    // BoundaryFeature* fill_left;
    // BoundaryFeature* fill_right;
    // DisDirPoint break_pos;
    BoundaryParamPair<LaneLineSample> yellow_boundary; // 离道路中心距离最近的黄色车道线点
    std::vector<BoundaryParamPair<LaneLineSample>> yellow_score_list; // 该位置附近20范围内的离道路中心距离最近的黄色车道线点

    double width;
    double theta;
    MatchLevel match_level;

    int left_opt_status;
    int right_opt_status;

    std::vector<double> score_prev;
    std::vector<double> score_next;

    Eigen::Vector3d &get_left();
    Eigen::Vector3d &get_right();
    bool in_scope(Eigen::Vector3d &pos, double buff=0, int mode=0);
};

struct LaneCenterFeatureMatchPair : public MatchPairBase<LaneCenterFeature>,
    std::enable_shared_from_this<LaneCenterFeatureMatchPair> {
    double dis;
    double h_dis;
    double theta;
    double delta_l_theta;
    double delta_r_theta;
    Eigen::Vector3d dir_gap;
};

template<class T>
struct LineMatchParam {
    T* from;
    T* to;
    double dis;
};

struct LineNodeParam : std::enable_shared_from_this<LineNodeParam> {
    double dir_gap; // (feature->dir - feature->context_dir).norm()
    double dir_next_gap; // 当前node的dir_gap与前一个node的dir_gap的差值，用于判断曲线连续性
    double dir_next_gap1;

    std::string string() {
        return utils::fmt("{:.2f},{:.2f},{:.2f}", dir_gap, dir_next_gap, dir_next_gap1);
    }
    static std::string title() {
        return utils::fmt("{},{},{}", "dir_gap", "next_gap_op", "next_gap");
    }
    static std::string meta() {
        return utils::fmt("{},{},{}", "-2:2", "-2:2", "-2:2");
    }
};

template<class T, class L>
struct ElementLine : public std::enable_shared_from_this<L> {
    std::string id; // 线的id
    std::string frame_id;
    double length;
    KeyPose* key_pos;
    int boundary_type = 0; // 0: 默认普通， 1： island，路口三角岛
    std::vector<T> list;
    std::vector<T> opt_list;

    std::vector<LineNodeParam> param_list;
    int filter_status;
    int src_status;
    bool invalid() {
        return filter_status > 1;
    }
};

template <class T>
bool SearchResultCallback(T a_data, void* context) {
    std::vector<T>* pvec = (std::vector<T>*)context;
    pvec->push_back(a_data);
    return true;
}

template<class DATATYPE, class ELEMTYPE, int NUMDIMS>
struct RTreeProxy : RTree<DATATYPE, ELEMTYPE, NUMDIMS>,
    std::enable_shared_from_this<RTreeProxy<DATATYPE, ELEMTYPE, NUMDIMS>> {
    void insert(Eigen::Vector3d &pt, DATATYPE info) {
        if (NUMDIMS > 2) {
            float minp[] = {pt.x(), pt.y(), pt.z()};
            float maxp[] = {pt.x(), pt.y(), pt.z()};
            this->Insert(minp, maxp, info);
        } else {
            float minp[] = {pt.x(), pt.y()};
            float maxp[] = {pt.x(), pt.y()};
            this->Insert(minp, maxp, info);
        }
    }

    int search(Eigen::Vector3d &pos, ELEMTYPE radius, std::vector<DATATYPE> &search_sec) {
        if (NUMDIMS > 2) {
            ELEMTYPE minb[] = {pos.x() - radius, pos.y() - radius, pos.z() - 1};
            ELEMTYPE maxb[] = {pos.x() + radius, pos.y() + radius, pos.z() + 1};
            this->Search(minb, maxb, SearchResultCallback<DATATYPE>, &search_sec);
        } else {
            ELEMTYPE minb[] = {pos.x() - radius, pos.y() - radius};
            ELEMTYPE maxb[] = {pos.x() + radius, pos.y() + radius};
            this->Search(minb, maxb, SearchResultCallback<DATATYPE>, &search_sec);
        }
        return search_sec.size();
    }
};


struct LabelPointLine : public ElementLine<LabelPoint, LabelPointLine> {
};

struct KeyPoseLine : public ElementLine<KeyPose*, KeyPoseLine> {
    std::vector<std::string> bind_trail_id;
    std::vector<std::shared_ptr<BoundaryGroupLine>> boundary_line_group_list; // 该条轨迹里所有boundary，已经做了融合
    std::vector<std::shared_ptr<LaneLineSampleGroupLine>> lane_line_group_list; // 该条轨迹里所有lane line，已经做了融合
    std::vector<std::shared_ptr<LaneCenterGroupLine>> lane_center_line_group_list; // 该条轨迹里所有lane center line，已经做了融合
    int link_direction;
    int kind;
    std::string form;
    int lanenum_sum;
    uint64_t start_node_id; //仅在raw_link 有效分组后需要从每个point 的raw_link获取
    uint64_t end_node_id;   //
    double width;  // 暂时未加入
    int same_id;
    int seed_id=0;
    int link_index=0;
};

struct LaneFeatureLine : public ElementLine<LaneFeature*, LaneFeatureLine> {
};

struct LaneLineSampleLine : public ElementLine<LaneLineSample*, LaneLineSampleLine> {
    LaneLineSampleLine* src;  // 表示sub_line的每段都是来自于哪条父曲线
    std::vector<std::shared_ptr<LaneLineSampleLine>> sub_line;  // 一条line在某些位置可能存在突变点，以这些位置为切点，切成很多段存在这

    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};

struct LaneLineSampleGroupLine {
    std::string id = "-1";
    std::vector<std::shared_ptr<LaneLineSample>> list;
    std::vector<std::vector<LaneLineSample*>> match_list;
    std::vector<std::vector<LaneLineSample*>> match_list_point;
    std::vector<double> match_score;
    std::vector<LaneLineSampleLine*> merge_lines;
    int road_index;
    int road_type;
    int side_status;
    RoadSegment* road_segment;


    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};


struct LaneCenterLine : public ElementLine<LaneCenterFeature*, LaneCenterLine> {
    LaneLineParam line_param;
    // 表示该车道中心线是由start_lc所在中心线的基础上，在线段两头一定距离内寻找新的车道中心点，
    // 从线段的头部往前找到的车道中心线为pre_line, 从线段的尾部往后找到的车道中心线为next_line,
    LaneCenterFeature* start_lc;
    LaneCenterFeature* end_lc;
    LaneAttr left_attr;
    LaneAttr right_attr;
    double center_offset; // center_index点与start_lc之间的距离，其实就是start_lc与start_lc所在的车道线的起始点raw_from_lc的距离
    int center_index; // start_lc中解释的 pre_line的最后一个点
    int lane_index;
    int side_status;
    int road_index;
    LaneCenterLine* src;  // 从哪条大的车道中心线切分而来
    SubRoadSegment* sub_road_segment;
    std::vector<std::shared_ptr<LaneCenterLine>> sub_line; // 一条车道中心线根据无效点处做切分，分成很多子中心线
    double get_reline_length();

    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};

struct LaneCenterGroupLine {
    std::string id = "-1";
    std::vector<std::shared_ptr<LaneCenterFeature>> list;
    std::vector<std::vector<LaneCenterFeature*>> match_list;
    // 选出附近5m半径内的车道中心点，在这些点中选出距离在一个车道范围内的点
    // <该条车道线上某个点的index， 该点附近5m半径内的车道中心点在这些点中选出距离在一个车道范围内的点（并且方向一致）>
    std::vector<std::vector<LaneCenterFeature*>> match_list_point;
    std::vector<double> match_score;
    std::vector<LaneCenterLine*> merge_lines;
    int road_index;
    int road_type;
    int side_status;
    RoadSegment* road_segment;


    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};


struct BoundaryLine : public ElementLine<BoundaryFeature*, BoundaryLine> {
    BoundaryLine* src;
    int road_index;
    int road_type;
    int side_status;
    std::vector<std::shared_ptr<BoundaryLine>> sub_line; // 一条line在某些位置可能存在突变点，以这些位置为切点，切成很多段存在这
    RoadSegment* road_segment;

    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};

struct BoundaryGroupLine {
    std::string id = "-1";
    std::vector<std::shared_ptr<BoundaryFeature>> list;  // 该曲线上所有的点
    // 记录该条曲线src_line上每个点与其他曲线tar_line的匹配点，size为该条曲线上点的个数,
    // tar_line的在搜索历史曲线的匹配度是，与src_line的匹配度必须是最高的
    std::vector<std::vector<BoundaryFeature*>> match_list;
    // 记录该条曲线上每个点与其他曲线的匹配点
    //<点在该曲线上的Index，与该点距离方向都匹配的其他曲线上的点（应该用于合并曲线）>
    // match_list_point里每个点对应的匹配点比match_list的点更多一些，match_list是match_list_point里的一部分点，
    std::vector<std::vector<BoundaryFeature*>> match_list_point; 
    // 该条曲线上每个点的匹配score，先用list中的score初始化，在后面单轨迹曲线匹配时，
    // 如果该条曲线上某个点与另外一条曲线上某个点匹配上时，就会将另外那条曲线上的匹配点的score加进去，
    // 一般来说，该点与其他曲线上匹配上的点越多，对应的score就会越高
    int boundary_type = 0; // 0: 默认普通， 1： island，路口三角岛
    std::vector<double> match_score;
    std::vector<BoundaryLine*> merge_lines;
    int road_index;
    int road_type;
    int side_status;
    RoadSegment* road_segment;

    // add by qzc
    int cur_line_id = 0; // 合并过程中的中间值
    std::unordered_set<int> matched_line_ids; // 匹配上的线id
};


struct GenLaneLineParam {
    std::map<std::pair<LaneCenterFeature*, LaneCenterFeature*>, int> status_map;
    std::map<LaneCenterFeature*, int> single_status_map;
    std::vector<std::shared_ptr<LaneLineSample> > lane_line_sample_ptr;
    std::vector<std::shared_ptr<LaneCenterFeature> > lane_center_feature_ptr;
    std::vector<std::shared_ptr<LaneFeatureLine> > lane_line_list_ptr;
    std::vector<std::shared_ptr<LaneCenterLine> > lane_line_ret;
    int sample_mode;
    int lane_size;
    // int save_to_log(const char* file, int mode=0, int out_mode=0);
};


struct RoadObjectPointInfo : public LineNodeBase<RoadObjectPointInfo> {
    // RoadElementId id;
    Eigen::Vector3d raw_pos;
    RoadObjectInfo* object_info;
    int type;
    void init();
};


struct Intersection : public DirPoint {
    int id;
    int ele_type;
    int type;
    double radius; // 好像没有存入？
    // std::vector<Eigen::Vector3d> point_list;
    // 下面两行从 Junction 中复制过来的
    std::vector<std::shared_ptr<RoadObjectPointInfo>> point_info; //存储polygon上的每个点的特征信息，如邻接，矢量等
    std::vector<Eigen::Vector3d> lukou_poly_pts; //存储polygon上原始点
    
    std::vector<RoadSegment*> in_road_segment; //注释了，没用到,不用管
    std::vector<RoadSegment*> out_road_segment;//注释了，没用到
    std::vector<RoadSegment*> turn_road_segment;//注释了，没用到
    std::vector<RoadSegment*> inter_road_segment; //所有的

    // 在当前intersection半径35m范围内的所有link点，对这些link点根据不同的line id归类到不同的线上，
    // <该link line上距离当前intersection最近的点, link line>
    std::map<KeyPose*, KeyPoseLine> sub_trail_poses;  //  很多条Link 
    

    //路口中进入 退出的  Road段，
    //遍历sub_trail_poses中的每个点poss，poss->next处对应的RoadSegment，并且点poss的位置在该Intersection的后方，
    // <poss->next处对应的RoadSegment， poss->pos>
    std::map<RoadSegment*, std::vector<KeyPose*> > in_neighbour;
    //遍历sub_trail_poses中的每个点poss，poss->next处对应的RoadSegment，并且点poss的位置在该Intersection的前方，
    // <poss->next处对应的RoadSegment， poss->pos>
    std::map<RoadSegment*, std::vector<KeyPose*> > out_neighbour;

    //路口中，前面 和后面的 路口
    // 与当前intersection关联的LinK线上的Link点 20m范围内后方的intersection，与当前intersection关联的LinK
    // 最远距离可以是35m，所以当前intersection与后的intersection的最大距离可以是55m，
    // <与当前intersection关联的LinK线上的Link点 20m范围内后方的intersection， Link点>
    std::map<Intersection*, std::vector<KeyPose*> > in_intersection;
    // 与当前intersection关联的LinK线上的Link点 20m范围内前方的intersection，与当前intersection关联的LinK
    // 最远距离可以是35m，所以当前intersection与前方的intersection的最大距离可以是55m，
    // <与当前intersection关联的LinK线上的Link点 20m范围内前方的intersection， Link点>
    std::map<Intersection*, std::vector<KeyPose*> > out_intersection;


    //add 
    std::vector<RoadLaneInfo*> all_lanes;
    std::vector<RoadLaneInfo*> in_lanes;
    std::vector<RoadLaneInfo*> out_lanes;
    std::unordered_set<RoadLaneGroupInfo*> in_loads;
    std::unordered_set<RoadLaneGroupInfo*> out_loads;

    std::vector<RoadObjectInfo*> cross_walk_list;


    // std::map<RoadSegment*, std::vector<LaneFeatureLine*> > extend_lanes;
};

struct LinkInfo
{
    std:: string  trail_id;
    std:: string  link_id;
    std:: string  from_node;
    std:: string  to_node;
    double prj_lng;
    double prj_lat;
};

struct InterInfo {
    struct KeyPoint: public DirPoint{
        void * praw_lane_line; //原始车道指针
        int index; //
        int max_index_size;
        std::vector<std::string>  turn_type;
      bool type_match(const std::string& turn)
      {
        for(auto p:turn_type)
        {
            // LOG_INFO("turn type:[tar:{} cur:{}]",p,turn);
            if(p==turn)
            {
                return true;
            }
        }
        return false;
      }
    };
    struct VirtualLane
    {
        struct Point:public DirPoint
        {
          double width;
        };
        int id;
       std::vector<KeyPoint> start_end_point; //原始进入推出车道
       std::vector<Point> point_list;
    };
    struct LaneGroup
    {
        // void * praw_lane_group; //原始车道组指针
        std::vector<KeyPoint> key_points;
        std::vector<VirtualLane> lane_centers;
        int dir;
    };
    int inter_id;
    std::vector<LaneGroup> in_groups;
    std::vector<LaneGroup> out_groups;
    std::vector<LaneGroup> virtual_groups;         
};


struct RoadSegment {
    int64_t id; // id越小，说明这个RoadSegment的长度越大，代表的长度排序序号
    int type; // 只有这段RoadSegment点数超过2个的type才会被赋值为1
    double length;
    // 开始节点
    KeyPose* start_pos;
    // 结束节点
    KeyPose* end_pos;
    KeyPoseLine* link; // 表示该段RoadSegment从哪条link线分割而来
    int link_direction;
    int merge_status;
    int update_type;
    int issue_status;
    int pose_status;
    int prev_rela_status;
    int next_rela_status;
    int filter_status;
    int same_road_status;

    RoadSegment* src_road_segment;


    Intersection* intersection;

    std::vector<KeyPose*> pos_sample_list; // 这段RoadSegment包含的Link点

    std::vector<SubRoadSegment*> sub_road_segment_list;

    UMAP<int, int> road_type;

    void init(RoadSegment* src) {
        this->type = src->type;
        this->link_direction = src->link_direction;
        this->src_road_segment = src;
        this->link = src->link;
    }
};

struct SubRoadSegment {
    RoadSegment* src_road_segment;
    int road_index;
    int road_type;
    int start_index;
    int end_index;
    int link_direction;
    BreakPos start_break_pos;
    BreakPos end_break_pos;

    bool need_left_boundary;
    bool need_left_road;

    SubRoadSegment* prev;
    SubRoadSegment* next;

    std::vector<LaneCenterLine*> valid_lane_list;
    std::vector<LaneCenterLine*> filled_lane_list;
    std::vector<BoundaryLine*> new_boundary_line_list;
    std::vector<RoadLaneGroupInfo*> new_lane_groups;
    std::vector<RoadBoundarySegmentInfo*> new_boundary_segment;
    void init(RoadSegment* road_segment) {
        this->src_road_segment = road_segment;
        this->link_direction = road_segment->link_direction;
    }
};

// struct RoadElementId {
//     int32_t tile_id;
//     int32_t type;
//     uint64_t index;
//     int64_t version;
// };

enum class DataStatus {
  kNop = 0, // do nothing
  kNew = 1, // new
  kMod = 2, // modify
  kDel = 3 // delete
};

struct RoadLaneInfo {
    int side_status;
    int filter_status;
    int road_lane_id;
    RoadLaneGroupInfo* lane_group_info;
    RoadLaneBoundaryInfo* left_lane_boundary_info;
    RoadLaneBoundaryInfo* right_lane_boundary_info;
    RoadLaneBoundaryInfo* center_lane_boundary_info;
    //许多绑定列表
    std::vector<LaneCenterFeature*> lane_center_feature_list; // 3, 15, 8等 
    std::vector<RoadObjectInfo*> bind_arrow_list; //箭头
    std::vector<RoadObjectInfo*> bind_stop_line_list; //绑定停止线
    std::vector<RoadObjectInfo*> cross_walk_list; //绑定人行道
    std::vector<JunctionInfo*> junction_list; //绑定路口
    std::vector<Intersection*> intersection_list; //绑定路口
    
    LineContext<RoadLaneInfo> context;
    std::shared_ptr<data_access_engine::LaneExtProxy> lane_ptr;
};

struct RoadLaneGroupInfo {
    // RoadElementId id;
    int group_index = -1; // 用于joint lane group
    DataStatus joint_status = DataStatus::kNop;
    // const data_access_engine::RoadSectionProxy* info;
    // std::shared_ptr<const data_access_engine::RoadSectionProxy> lane_group_ptr;
    std::vector<RoadLaneBoundaryInfo*> lane_boundary_info;
    std::vector<RoadLaneInfo*> lane_line_info;
    std::vector<RoadBoundarySegmentInfo*> left_barrier_segment_info;
    std::vector<RoadBoundarySegmentInfo*> right_barrier_segment_info;
    RoadLaneBoundaryInfo* road_center_line_info;   // 道路内左侧第一车道的左边线
    std::shared_ptr<data_access_engine::LaneGroupProxy> lane_group_ptr;
    // std::vector<data_access_engine::BarrierSegmentProxy*> barrier_segment_proxy;
    // std::vector<data_access_engine::CurbSegmentProxy*> curb_segment_proxy;
    //
    SubRoadSegment* src_road_segment;

    // int64_t version;
    int update_status;
    int filter_status;
    int gen_pose_status;
    int prev_pose_status;
    int next_pose_status;
    // 1原有，2新增, 3分裂
    int src_status;
    int oppo_status;
    int64_t src_id_version;
    Eigen::Vector3d start_pos;
    double length;
    bool is_tollbooth;
    bool is_tunnel;

    // std::vector<RoadLaneBoundaryInfo*> src_lane_boundary;
    // std::vector<PosSection*> all_src_pose_vec;
    // 前驱后继lane_group
    LineContext<RoadLaneGroupInfo> context;
    // void init();
    // void init(const std::shared_ptr<const data_access_engine::RoadSectionProxy> &lane_group);
    bool get_road_property(std::string name, std::string* value=NULL);
    Eigen::Vector3d* get_pos(int lane_index, int line_point_index);
    const char* get_wgs(int lane_index, int segment_index);
};

struct RoadLaneBoundaryInfo {
    int lanes_index;
    RoadLaneGroupInfo* lane_group;
    std::vector<RoadLinePointInfo*> line_point_info;
    // int64_t version;
    int update_status;
    // 1原有，2新增
    int src_status;
    int issue_status;
    int use_status;
    // int64_t src_id_version;

    // Eigen::Vector3d start_pos;
    double length;
    double speed_min;
    double speed_max;
    int lane_id;
    int geo;
    int color;
    int deceleration;
    bool is_base_line;
    bool is_right_line;
    bool is_bold;
    bool is_emergency;
    bool is_nopass;
    bool in_built;
    bool is_juma;
    bool is_entrance;
    bool is_exit;
    bool pointcloud_not_clear;
    bool pointcloud_fusion_issue;
    bool emergency_parking_strip;
    int type ; //1 未知 2：实线 3：虚线

    LineContext<RoadLaneBoundaryInfo> context;
    RoadLaneBoundaryInfo* left;
    RoadLaneBoundaryInfo* right;
    // ScenesCase* start_break_point;
    // ScenesCase* end_break_point;
    void init();
    // void init(const std::shared_ptr<const data_access_engine::LaneSectionProxy> &lane_boundary);
    // bool get_lane_property(std::string name, std::string* value=NULL);
    bool is_same_line(RoadLaneBoundaryInfo* other, int mode=0, double scope=50);
    RoadLinePointInfo* get_nearest_ls(const Eigen::Vector3d &pos, const Eigen::Vector3d &v_dir);
    const char* get_wgs(int index=0);
};

struct RoadLinePointInfo : public LineNodeBase<RoadLinePointInfo> {
    RoadLaneBoundaryInfo* lane_boundary;
    RoadLaneGroupInfo* lane_group;
    // RoadLinePointInfo* src_line_info;
    // int64_t version;
    int32_t geo_type;
    int32_t color_type;
    int32_t is_bold;
    // Eigen::Vector3d pos_13;
    // Eigen::Vector3d start_pos;
    // Eigen::Vector3d end_pos;
    // Eigen::Vector3d raw_pos;
    // Eigen::Vector3d raw_start_pos;
    // Eigen::Vector3d raw_end_pos;
    double length;
    double width;
    double score;
    int64_t tile_id;
    // ScenesType scenes;
    // int identify_type;
    // 1，原有，2删除，3新增，4更新
    int update_status;
    int format_status;
    int freeze_status;
    int filter_status;
    int issue_status;
    int poss_status;
    int delete_status;
    int match_status;
    int src_status;
    int src_index;
    int real_status;
    int ground_status;
    LaneCenterFeature* format_lc;
    bool format_left;
    KeyPose* key_pose;
    int bind_lane_index;
    // int64_t src_id_version;

    // RoadMatchPair start_lc_info;
    // RoadMatchPair end_lc_info;
    // std::vector<RoadLineSegmentSample*> sample_list;
    std::vector<RoadBoundaryPointInfo*> match_barrier_list;
    std::vector<RoadBoundaryPointInfo*> match_curb_list;
    LineContext<RoadLinePointInfo> context;
    bool is_intersection_point = false;

    // AlignParam<RoadLineSegmentSample*> align_param;

    void init();
    // const char * get_start_wgs();
    // const char * get_end_wgs();
    // bool is_emergency();
};

struct RoadBoundaryPointInfo : public LineNodeBase<RoadBoundaryPointInfo> {
    // RoadElementId id;
    RoadBoundarySegmentInfo* barrier_segment;
    int index;
    int update_status;
    int src_status;
    int src_ele;

    int match_status;
    int bind_status;
    int ground_status;
    int64_t src_id_version;
    double curvature;

    // Eigen::Vector3d pos_13;
    // Eigen::Vector3d raw_pos;

    // std::vector<std::pair<RoadLinePointInfo*, double>> bind_line_points;

    int32_t tile_id;
    float height;
    float width;
    float length;

    double timestamp;
    bool is_intersection_point;

    // std::vector<RoadBoundaryPoint*> sample_list;

    // @尤西霞AlignParam<RoadBoundaryPointSample*> align_param;
    // std::vector<PosSection*> pose_candidate;

    LineContext<RoadBoundaryPointInfo> context;;

    void init();
};

struct RoadBoundarySegmentInfo {
    // RoadElementId id;
    // const data_access_engine::CurbSegmentProxy* info;
    // std::shared_ptr<const data_access_engine::CurbSegmentProxy> curb_segment_ptr;
    std::vector<RoadBoundaryPointInfo*> point_info;//道路边界点的信息
    RoadLaneGroupInfo* lane_group_info; // 指向道路车道组信息的指针，表示当前路段所属的车道组
    int boundary_id;
    int update_status;
    int filter_status;
    int src_status;
    int side_status; //边界段的侧面状态
    // int64_t src_id_version;
    std::shared_ptr<data_access_engine::RoadBoundaryProxy> road_boundary_ptr;

    // Eigen::Vector3d start_pos;
    float width;
    float height;
    int type;//道路边界段的分类
    int subtype; //道路边界段的更具体分类  左侧、右侧等?

    // ScenesCase* start_break_point;
    // ScenesCase* end_break_point;
    double timestamp;
    Eigen::Vector3d pos; //中点
    LineContext<RoadBoundarySegmentInfo> context;
    void init();

    Eigen::Vector3d* get_pos(int index);

    const char* get_wgs(int index=0);
};


struct RoadObjectInfo : public DirPoint {
    // RoadElementId id;
    // const data_access_engine::ObjectBoundingBoxProxy* info;
    // std::shared_ptr<const data_access_engine::ObjectBoundingBoxProxy> object_ptr;
    int obj_id;
    std::shared_ptr<data_access_engine::TrafficInfoProxy> traffic_proxy_ptr;
    std::shared_ptr<data_access_engine::PositionObjectProxy> position_obj_proxy_ptr;
    int update_status;
    int src_status;
    // int64_t src_id_version;
    std::vector<std::shared_ptr<RoadObjectPointInfo>> list;
    // int detailed_type;
    // int sign_shape;
    // int stop_line_is_valid;
    int ele_type;// 类型 如箭头、停止线等
    std::string type;//具体类型
    std::string subtype;
    // std::string max_speed;
    // std::string min_speed;
    // std::string pole_logic_subtype;
    // std::vector<TrafficSignInfo> speed_sign_infos;
    // std::vector<TrafficSignInfo> others_sign_infos;

    // PosSection* bind_pose; 

    // Eigen::Vector3d pos_13;
    // MatchInfo tar_pos;
    // int32_t tile_id;
    float heading;
    float width;
    float height;
    float length;
    float score;

    std::vector<KeyPose*> bind_links;//人行横道因为粘连可能绑定多个， 其他的元素都是对应一个keypose
    std::vector<RoadLaneInfo*> obj_bind_lanes;
    // std::vector<JunctionInfo*> obj_bind_junctions; //绑定路口
    std::vector<Intersection*> obj_bind_intersections; //绑定路口

    // bool is_speed;
    // bool is_roadmark_speed;
    // bool is_stop_line;
    // bool is_cross_walk;
    // bool is_ground_arrow;
    // bool is_no_parking_area;
    // bool is_traffic_light;
    // int32_t traffic_light_type;
    // int32_t traffic_light_subtype;
    // bool is_ground_speed_limit;
    // bool is_sign_box;
    // bool is_pole_box;
    // bool is_road_text;

    // double timestamp;
    void init();
    // void init(const std::shared_ptr<const data_access_engine::ObjectBoundingBoxProxy> &object);
};

//路口结构体
struct JunctionInfo : public DirPoint {
    std::vector<std::shared_ptr<RoadObjectPointInfo>> point_info; //存储polygon上的每个点的特征信息，如邻接，矢量等
    std::vector<Eigen::Vector3d> lukou_poly_pts; //存储polygon上原始点
    std::vector<std::shared_ptr<RoadBoundarySegmentInfo>> lukou_bd_list; //分好段的根据路口type的属性打断为多个道路边界，
    std::vector<ImpassableAreaInfo*> areas; //存储路口内关联的不可通行区域

    std::vector<RoadLaneInfo*> lanes; //关联车道 
    std::vector<RoadLaneInfo*> in_lanes; //关联进入车道
    std::vector<RoadLaneInfo*> out_lanes; //关联退出车道
   
    //TODO: 道路的结构体待确认
    std::vector<RoadLaneInfo*> in_roads; //关联进入道路    
    std::vector<RoadLaneInfo*> out_roads; //关联退出道路


    int ele_type;
    void init();
    // void init(const std::shared_ptr<const data_access_engine::ObjectBoundingBoxProxy> &object);
};

struct ImpassableAreaInfo : public DirPoint {
    JunctionInfo* junction_info;
    std::vector<std::shared_ptr<RoadObjectPointInfo>> list;
    int ele_type;
    int type;
    void init();
    // void init(const std::shared_ptr<const data_access_engine::ObjectBoundingBoxProxy> &object);
};

struct RoadFormatInfo {
    SubRoadSegment* road_segment;
    RoadLaneGroupInfo* lane_group;
    // int32_t lane_group_tile_id;
    // Wm5::Vector3d lane_group_local_pos_without_offset;

    LaneCenterLine* lane_line;
    int32_t lane_line_index;
    RoadLaneBoundaryInfo* lane_boundary;
    // int32_t lane_boundary_tile_id;
    // Wm5::Vector3d lane_boundary_local_pos_without_offset;

    LaneCenterFeature* lc;
    LaneCenterFeature* prev_lc;
    bool gen_left;
    bool format_oppo;
    double gen_length;
    int road_index;
    int double_left;
    RoadLinePointInfo* line_point;
    // int32_t lane_line_point_tile_id;
    // Wm5::Vector3d lane_line_point_local_pos_without_offset;

    RoadSegment* boundary_road_segment;
    RoadBoundarySegmentInfo* boundary_segment;
    // int32_t barrier_segment_tile_id;
    // FeatureLine* barrier_feature_line;
    // Wm5::Vector3d barrier_point_local_pos_without_offset;

    std::map<RoadLaneBoundaryInfo*, int> lane_boundary_used_map;

    void init() {
        road_segment = NULL;
        lane_group = NULL;
        // lane_group_tile_id = 0;

        lane_line = NULL;
        lane_line_index = 0;
        lane_boundary = NULL;
        // lane_boundary_tile_id = 0;

        lc = NULL;
        prev_lc = NULL;
        gen_left = false;
        gen_length = 0;
        line_point = NULL;
        // lane_line_point_tile_id = 0;

        boundary_road_segment = NULL;
        boundary_segment = NULL;
        // barrier_segment_tile_id = NULL;
        // barrier_feature_line = NULL;
    }
};


struct BreakPointKeyPose{
    KeyPose* key_pose;
    int break_status; //0进入，1退出
};

struct LaneCoverage{
    std::vector<Eigen::Vector3d> prev_exist_pts;
    std::vector<Eigen::Vector3d> missed_pts;
    // LaneCenterGroupLine* lane_center_group;
};



//横向交掉的点
struct CrossPoint{
    bool is_new  = false; //false: 原来就有， new: 新增的
    Eigen::Vector3d point;
    CrossPoint(bool is_new_val, const Eigen::Vector3d& pt)
    : is_new(is_new_val), point(pt) {}
};

struct RoadCrossPoints{
    KeyPose* keypose;
    bool is_deal = false;
    bool is_full = false;

    std::vector<std::shared_ptr<CrossPoint>> lane_center_pts;   //横向 与车道中心线的交点
    std::vector<std::shared_ptr<CrossPoint>> lane_pts;  //横向 与车道线的交点
    std::vector<std::shared_ptr<CrossPoint>> road_boundary_pts;  //横向 与道路边界的交点
    // std::vector<CrossPoint*> lane_center_pts;   //横向 与车道中心线的交点
    // std::vector<CrossPoint*> lane_pts;  //横向 与车道线的交点
    // std::vector<CrossPoint*> road_boundary_pts;  //横向 与道路边界的交点
    
    Eigen::Vector3d road_dir;  //修正link方向
    double W_lane_st; //理论车道宽度
    double W_lane_t;  //真实车道宽度
    int N_start;    //最原始起始位置的车道数
    int N_hope;   //期望车道数
    int N_diff;     //N_hope - Nt
    
    int N_road; // 道路边界算出的车道数
    int N_lane; // 车道边界算出的车道数
    int N_lane_center; // 车道中心线算出的车道数
    int N_link; // link上自带的车道数
};



void log_pcd(std::vector<Eigen::Vector3d> &data_list, const char *name);

using BoundaryFeatureTree = RTreeProxy<BoundaryFeature*, float, 2>;
using LaneLineSampleTree = RTreeProxy<LaneLineSample*, float, 2>;
using LaneCenterFeatureTree = RTreeProxy<LaneCenterFeature*, float, 2>;
using KeyPoseTree = RTreeProxy<KeyPose*, float, 2>;

}
