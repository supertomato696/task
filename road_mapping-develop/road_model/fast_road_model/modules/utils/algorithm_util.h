

#pragma once

#include <Eigen/Core>
#include <numeric>
#include "road_model_meta.h"
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <unordered_set>

namespace fsdmap {
namespace alg {
namespace bg = boost::geometry;
using bg_point = bg::model::d2::point_xy<double>;
using point_t = bg::model::d2::point_xy<double>;
using segment_t = bg::model::segment<point_t>;
using linestring_t = bg::model::linestring<point_t>;
using points_t = bg::model::multi_point<point_t>;

struct OverlapInfoLineString {
    int l1_id;
    int l2_id;
    double l1_length;
    double l2_length;
    linestring_t l1; // 重叠的点
    linestring_t l2; // 重叠的点
    std::vector<int> l1_index; // l1和l2重叠的点，对应l1上点的索引
    std::vector<int> l2_index; // l1和l2重叠的点，对应l2上点的索引
    bool is_need_reverse_l1 = false; // 是否需要反转l1
    bool is_need_reverse_l2 = false; // 是否需要反转l2
  
    int is_overlap_num = 0; // 重叠点数
    bool is_overlap = false; // 是否重叠
    bool is_nearby = false; // 是否临近，但不重叠
};

struct PointInPolygon {
        Eigen::Vector3d point;
        bool is_intersection_point = false;
        PointInPolygon(Eigen::Vector3d pt, bool is_intersect) : point(pt), is_intersection_point(is_intersect) {}
};
    

// struct OverlapMatchedInfo {
//     int id;
//     double length;
//     std::vector<int> matched_line;
//     std::vector<double> matched_line_length;
// };

// using overlap_matched_info = std::pair<OverlapMatchedInfo, double>;



// 定义常量
constexpr double PI = 3.1415926535897932384626;
constexpr double A = 6378245.0; // 长半轴
constexpr double EE = 0.00669342162296594323; // 第一偏心率平方
// constexpr double DEG_TO_RAD = 3.1415926535897932384626 / 180.0;

constexpr double pi = 3.1415926535897932384626;
constexpr double krasowski_a = 6378245.0;  // 克拉索夫斯基椭球参数长半轴a
constexpr double krasowski_e2 = 0.00669342162296594323;  // 克拉索夫斯基椭球参数第一偏心率平方

extern double calc_dis(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, bool ignore_z=true);

extern Eigen::Vector3d find_closest_point(const std::vector<Eigen::Vector3d>& points, 
        const Eigen::Vector3d& center_point);

extern Eigen::Vector3d get_dir(const Eigen::Vector3d &to, const Eigen::Vector3d &from, bool ignore_z=true);

//从dirs 获取最可靠的dir
extern Eigen::Vector3d get_direction (const std::vector<Eigen::Vector3d> &dirs);

extern Eigen::Vector3d get_vertical_dir(const Eigen::Vector3d &dir, bool ignore_z=true);

extern Eigen::Vector3d get_vertical_dir(const Eigen::Vector3d &from, const Eigen::Vector3d &to, bool ignore_z=true);

extern void calc_dir(const Eigen::Vector3d &to, const Eigen::Vector3d &from,
        Eigen::Vector3d &dir, bool ignore_z=true);

extern Eigen::Vector3d rotate_vector(const Eigen::Vector3d& vec, double theta);

extern double calc_theta(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute=false, bool ignore_z=true);

extern double calc_theta1(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute=false, bool ignore_z=true);

extern double calc_theta2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute=false, bool ignore_z=true);

extern double calc_theta(const Eigen::Vector3d &dir);
extern double calc_theta_with_dir(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute=false, bool ignore_z=true);

extern double calc_vertical_dis(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const Eigen::Vector3d &dir,
        bool need_dir=false, bool ignore_z=true);

extern double calc_hori_dis(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const Eigen::Vector3d &dir,
        bool need_dir=false, bool ignore_z=true);

extern Eigen::Vector3d& rotate_yaw(const Eigen::Vector3d &src, Eigen::Vector3d &tar, double yaw);

// !!!!!! TODO 有问题，慎用，左手系
// ！！！！！ 慎用  ！！！！！！，优先使用，judge_left2
extern int judge_left(const Eigen::Vector3d &dir1, const Eigen::Vector3d &dir2);
 
extern int judge_left(const Eigen::Vector3d &p, const Eigen::Vector3d &p1, const Eigen::Vector3d &dir);

// ！！！！！右手坐标系，符合人类直观感觉！！！！！！！
extern int judge_left2(const Eigen::Vector3d &dir1, const Eigen::Vector3d &dir2);
 
extern int judge_left2(const Eigen::Vector3d &p, const Eigen::Vector3d &p1, const Eigen::Vector3d &dir);

extern bool judge_front(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
        const Eigen::Vector3d &dir2);

extern Eigen::Vector3d get_vertical_pos(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir,
        double dis, bool ignore_z=true);

extern Eigen::Vector3d get_hori_pos(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir,
        double dis, bool ignore_z=true);

extern bool get_cross_point(const Eigen::Vector3d &from, const Eigen::Vector3d &from_dir,
        const Eigen::Vector3d &to, const Eigen::Vector3d &to_dir, Eigen::Vector3d &ret,
        bool ignore_z=true, int mode = 3, double len=100.0);

//cxf add
//求AB,CD的交点 ，注意AB为from1，from2，CD为to1，to2；
//默认都延长，延长长度为100
//mode 0： 不延长， 1:延长AB  2:延长CD  3:AB,CD都延长
extern bool get_cross_point_by_point(const Eigen::Vector3d &from1, const Eigen::Vector3d &from2,
        const Eigen::Vector3d &to1, const Eigen::Vector3d &to2, Eigen::Vector3d &ret, 
        bool ignore_z=true, int mode=3, double len=100.0);

//线段和 框（可闭合/不闭合）的交点, multi_points:true 算多个交点，false，交到第一个交点则返回
//线段默认不延长，可选择延长   
//mode: 0两边都延长len, 1延长from, 2延长to, 
extern bool get_cross_point_with_polygon(
        const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,
        const std::vector<Eigen::Vector3d> &polygon, std::vector<Eigen::Vector3d> &intersection_points, 
        bool ignore_z=true, int mode=0, double len=0.0, bool multi_points=false);


//线段和  曲线段的交点, 曲线段不闭合，  multi_points:true 算多个交点，false，交到第一个交点则返回
//线段默认不延长，可选择延长   
//mode: 0两边都延长len, 1延长from, 2延长to, 
extern bool get_cross_point_with_curve_segment(
        const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,
        const std::vector<Eigen::Vector3d> &polygon,
        std::vector<UsefulPnt> &interp_infos, 
        bool ignore_z=true, int mode=3, double len=0.0, bool multi_points=false);


//计算dir方向上最近或最远的点
extern Eigen::Vector3d project_points_to_dir(const Eigen::Vector3d &base_point, const Eigen::Vector3d &dir, 
        const std::vector<Eigen::Vector3d> &points, bool ignore_z=true, int mode=1);

//点投影到 基准线上的距离
extern double project_point_to_line(const Eigen::Vector3d& base_point, const Eigen::Vector3d& dir, const Eigen::Vector3d& point, bool ignore_z=true);

//算均值
extern Eigen::Vector3d cal_average(const std::vector<Eigen::Vector3d>& points);

//算标准差
extern Eigen::Vector3d cal_standard_deviation(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& mean);

extern double get_max_vertical_dis(const Eigen::Vector3d &pos1, const Eigen::Vector3d &dir1,
        const Eigen::Vector3d &pos2, const Eigen::Vector3d &dir2, bool ignore_z=true);

extern double calc_median(const std::vector<double> &vec, int num=1, 
        double position=0.5, int32_t start=0, int32_t end=0);

extern double calc_avg(const std::vector<double> &vec, int32_t start=0, int32_t end=0);

extern double calc_line_variance(const std::vector<double> &data, double miu=0, double sigma=0, 
        int64_t start=0, int64_t end=0);

extern double calc_variance(const std::vector<double> &data, double miu=0, double sigma=0, int64_t start=0, int64_t end=0);

extern double calc_variance_with_mean(const std::vector<double> &data, double mean, double miu=0, double sigma=0,
        int64_t start=0, int64_t end=0);

extern double calc_score_by_gaussian(double x, double sigma, double miu=0);

extern int fit_line2d(const std::vector<double>& data, float& coff_a, float& coff_b, int32_t start=0, int32_t end=0);

extern bool fit_line3d(const std::vector<Eigen::Vector3d>& data, float& coff_a, float& coff_b, int32_t start=0, int32_t end=0);

extern bool fit_circle(std::vector<Eigen::Vector3d> &point_list, 
        double &cir_x, double &cir_y, double &cir_r);

extern bool fit_circle_algebraic(std::vector<Eigen::Vector3d> &point_list, 
        double &cir_x, double &cir_y, double &cir_r);

extern double calc_area_rate(std::vector<Eigen::Vector3d> &polygon1,
        std::vector<Eigen::Vector3d> &polygon2, int mode);

extern double LogNormal(double x, double miu, double sigma);

extern double segment_iou(Eigen::Vector3d &start1, Eigen::Vector3d &end1, Eigen::Vector3d &start2, Eigen::Vector3d &end2);

extern bool point_in_polygon(Eigen::Vector3d &point, std::vector<Eigen::Vector3d> &polygon);

extern bool points_in_polygon(std::vector<Eigen::Vector3d> &polygon,
        std::vector<Eigen::Vector3d> &points, std::vector<int> &in_points_index);

extern bool any_point_in_polygon(std::vector<Eigen::Vector3d> &polygon, std::vector<Eigen::Vector3d> &points);

extern bool out_of_china(double lng, double lat);

extern double transform_lat(double lng, double lat);

extern double transform_lng(double lng, double lat);

extern void gcj2Towgs(double lng_gcj2, double lat_gcj2, double &lng_wgs, double &lat_wgs);

extern void wgsTogcj2(double lng_wgs, double lat_wgs, double &lng_gcj2, double &lat_gcj2);

extern void wgs84_to_gcj02(double &lng, double &lat);

template <typename TYPE>
bool calc_max_count_int(const std::vector<TYPE> &vec, TYPE &max_value, double &rate,
        int32_t start=0, int32_t end=0) {
    const TYPE* max_count_value = NULL;
    int64_t max_count = 0;
    if (end == 0 || end > vec.size()) {
        end = vec.size();
    }
    if (start < 0) {
        start = 0;
    }
    int32_t size = end - start;
    if (size < 1) {
        return false;
    }
    std::unordered_map<TYPE, int64_t> value_map;
    for (int i = start; i < end; ++i) {
        auto &value = vec[i];
        if (value_map.find(value) == value_map.end()) {
            value_map[value] = 1;
        } else {
            value_map[value] = value_map[value] + 1;
        }
    }
    for (auto &it : value_map) {
        if (it.second > max_count) {
            max_count = it.second;
            max_count_value = &it.first;
        }
    }
    rate = ((double)max_count) / size;
    if (max_count_value != NULL) {
        max_value = *max_count_value;
        return true;
    }
    return false;
}

template <typename TYPE>
TYPE calc_max_count_int_or(const std::vector<TYPE> &vec, TYPE max_value,
        int32_t start=0, int32_t end=0) {
    TYPE tmp_value;
    double rate = 0;
    if (calc_max_count_int(vec, tmp_value, rate, start, end)) {
        return tmp_value;
    }
    return max_value;
}


//算交点，并加上dir属性
template <class T>
bool get_cross_point_with_curve_segment(
    const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,const std::vector<std::shared_ptr<T>> &polygon, 
    std::vector<UsefulPnt> &interp_infos,
    bool ignore_z=true, int mode=0, double len=0.0, bool multi_points=false) {

    interp_infos.clear();
    int polygon_size = polygon.size(); 

    Eigen::Vector3d extended_from = line_from;
    Eigen::Vector3d extended_to = line_to;
    Eigen::Vector3d direction = alg::get_dir(line_to, line_from, ignore_z); 
    if (ignore_z) {
        direction.z() = 0;
    }

    if (mode == 0) {
        extended_from -= len * direction;
        extended_to += len * direction;
    } else if (mode == 1) {
        extended_from -= len * direction;
    } else if (mode == 2) {
        extended_to += len * direction;
    }

    // 遍历多边形的每一条边
    for (int i = 0; i < polygon_size - 1; ++i) {  // 处理从 0 到 n-2 个顶点
        Eigen::Vector3d poly_from = polygon[i]->pos;
        Eigen::Vector3d poly_to = polygon[i + 1]->pos;  // 下一顶点，形成一条边
        
        Eigen::Vector3d intersection_point;
        bool is_intersect = alg::get_cross_point_by_point(extended_from, extended_to, poly_from, poly_to, intersection_point, ignore_z, 0);
        if (is_intersect) {
            Eigen::Vector3d dir = (polygon[i]->dir + polygon[i + 1]->dir) / 2; // ！！！！ 注意这里的方向和另外一个的不一样
            intersection_point.z() = (polygon[i]->pos + polygon[i + 1]->pos).z()/2;

            int type, color;
            double d1 = (intersection_point - poly_from).norm();
            double d2 = (intersection_point - poly_to).norm();
            if(d1 < d2) {
                if constexpr (std::is_same_v<T, LaneLineSample>) {
                    type = polygon[i]->attr.type;
                    color = polygon[i]->attr.color;
                } else {
                    type = polygon[i]->type;
                    color = polygon[i]->color;
                }
            } else {
                if constexpr (std::is_same_v<T, LaneLineSample>) {
                    type = polygon[i+1]->attr.type;
                    color = polygon[i+1]->attr.color;
                } else {
                    type = polygon[i+1]->type;
                    color = polygon[i+1]->color;
                }
            }

            interp_infos.push_back(UsefulPnt(intersection_point, dir));
            if (!multi_points) {
                return true;
            }
        }
    }
    return !interp_infos.empty();
}

// mode: 
// 0: 忽视z值
// 1：用 p1s 和 p1e 加权
// 2：用 p2s 和 p2e 加权
extern bool findIntersection(const Eigen::Vector3d& p1s, const Eigen::Vector3d& p1e,
        const Eigen::Vector3d& p2s, const Eigen::Vector3d& p2e,
        Eigen::Vector3d& intersection, int mode = 0);

extern Eigen::Vector3d get_direction (const std::vector<Eigen::Vector3d> &dirs);

extern bool is_bit_set(int n, int k);


extern std::vector<Eigen::Vector3d> cut_line_in_polygon(std::vector<Eigen::Vector3d>& polygon_pts, const std::vector<Eigen::Vector3d>& input_points);  

extern bool is_in_convex_polygon( const std::vector<Eigen::Vector3d>& polygon,const Eigen::Vector3d& point);

extern double cal_line_min_distance(const linestring_t& l1, const linestring_t& l2);

extern void generate_virtual_center_line(const linestring_t& l1, const linestring_t& l2,
                                linestring_t& closest_points, linestring_t& center_line);

extern void generate_virtual_center_line2(const linestring_t& l1, const linestring_t& l2,
                                linestring_t& closest_points, linestring_t& center_line);

extern bool generate_virtual_center_line3(const linestring_t& l1, const linestring_t& l2,
                                double& distance, Eigen::Vector2d& optimize_T);

extern std::pair<point_t, point_t> find_closest_points(const linestring_t& curve1, const linestring_t& curve2);

extern std::pair<double, point_t> find_closest_points(const point_t& pt_in, const linestring_t& curve1);

extern void lines_overlap_info(const linestring_t& l1, const linestring_t& l2,
                        const std::vector<Eigen::Vector3d>& l1_dir, const std::vector<Eigen::Vector3d>& l2_dir,
                        OverlapInfoLineString& overlap_info, double thr);

extern void reverse_matched_info(const OverlapInfoLineString& in, OverlapInfoLineString& out);

extern bool match_any_with_forms(const std::unordered_set<uint64_t>& search_set, std::vector<uint64_t> check_form);

extern bool match_any_except_forms(const std::unordered_set<uint64_t>& search_set, std::vector<uint64_t> check_form);


// 业务算法相关
template <class T>
int64_t vote_type_color(std::vector<T*> &attr_data_list, T *base_fls)
{
    int64_t valid_num = 0;
    if(attr_data_list.size() == 0) {
        return valid_num;
    }

    UMAP<int, int> type_map;
    UMAP<int, int> color_map;
    int type, color;
    for (auto &fls : attr_data_list) {
        if constexpr (std::is_same_v<T, LaneLineSample>) {
            type  = fls->attr.type;
            color  = fls->attr.color;
        } else {
            type  = fls->type;
            color  = fls->color;
        }

        ++valid_num;
        if(MAP_NOT_FIND(type_map, type)){
            type_map[type] = 0;
        }
        else{
            type_map[type]++;
        }
        if(MAP_NOT_FIND(color_map, color)){
            color_map[color] = 0;
        }
        else{
            color_map[color]++;
        }
    }

    std::vector<std::pair<int, int>> temp_type(type_map.begin(), type_map.end());
    std::sort(temp_type.begin(), temp_type.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    std::vector<std::pair<int, int>> temp_color(color_map.begin(), color_map.end());
    std::sort(temp_color.begin(), temp_color.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    if constexpr (std::is_same_v<T, LaneLineSample>) {
        base_fls->attr.type = temp_type[0].first;
        base_fls->attr.color = temp_color[0].first;
    } else {
        base_fls->type = temp_type[0].first;
        base_fls->color = temp_color[0].first;
    }

    return valid_num;
}

template <class T>
int64_t vote_type_color(std::vector<T*> &attr_data_list, std::vector<int>& ret_type_color)
{
    int64_t valid_num = 0;
    if(attr_data_list.size() == 0) {
        ret_type_color.push_back(99);
        ret_type_color.push_back(99);
        return valid_num;
    }

    UMAP<int, int> type_map;
    UMAP<int, int> color_map;
    int type, color;
    for (auto &fls : attr_data_list) {
        if constexpr (std::is_same_v<T, LaneLineSample>) {
            type  = fls->attr.type;
            color  = fls->attr.color;
        } else {
            type  = fls->type;
            color  = fls->color;
        }

        ++valid_num;
        if(MAP_NOT_FIND(type_map, type)){
            type_map[type] = 0;
        }
        else{
            type_map[type]++;
        }
        if(MAP_NOT_FIND(color_map, color)){
            color_map[color] = 0;
        }
        else{
            color_map[color]++;
        }
    }

    std::vector<std::pair<int, int>> temp_type(type_map.begin(), type_map.end());
    std::sort(temp_type.begin(), temp_type.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    std::vector<std::pair<int, int>> temp_color(color_map.begin(), color_map.end());
    std::sort(temp_color.begin(), temp_color.end(), [](auto& a, auto& b) {return a.second >= b.second;});

    ret_type_color.push_back(temp_type[0].first);
    ret_type_color.push_back(temp_type[0].first);

    return valid_num;
}


}
}
