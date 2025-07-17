

#pragma once

#include <Eigen/Core>
#include <numeric>
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/multi_point.hpp>

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

extern bool get_cross_point_for_segment(const Eigen::Vector3d &from1, const Eigen::Vector3d &from2,
        const Eigen::Vector3d &to1, const Eigen::Vector3d &to2, Eigen::Vector3d &ret, int mode, bool ignore_z=true);

extern bool get_cross_point(const Eigen::Vector3d &from, const Eigen::Vector3d &from_dir,
        const Eigen::Vector3d &to, const Eigen::Vector3d &to_dir, Eigen::Vector3d &ret, bool ignore_z=true);

//cxf add
//求AB,CD的交点 ，注意AB为from1，from2，CD为to1，to2；
//默认都延长，延长长度为100
//mode 1:延长AB  2:延长CD  3:AB,CD都延长
extern bool get_cross_point_by_point(const Eigen::Vector3d &from1, const Eigen::Vector3d &from2,
        const Eigen::Vector3d &to1, const Eigen::Vector3d &to2, Eigen::Vector3d &ret, bool ignore_z=true, int mode=3, double len=100);

//AB,CD都不延长，确保交点确保在线段AB和CD上
extern bool get_cross_point_by_point2(const Eigen::Vector3d &from1, const Eigen::Vector3d &from2,
                               const Eigen::Vector3d &to1, const Eigen::Vector3d &to2,
                               Eigen::Vector3d &ret, bool ignore_z);

//传一个点和dir，和两个线段点,  选择延长哪条
extern bool get_cross_point_by_dir(const Eigen::Vector3d &from1, const Eigen::Vector3d &dir1,
        const Eigen::Vector3d &from2, const Eigen::Vector3d &to2, Eigen::Vector3d &ret, int mode, bool ignore_z=true);

//线段和 曲线框（可闭合/不闭合）的交点, multi_points:true 算多个交点，false，交到第一个交点则返回
//线段默认不延长，可选择延长
extern bool get_cross_point_with_polygon(
        const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,
        const std::vector<Eigen::Vector3d> &polygon, std::vector<Eigen::Vector3d> &intersection_points, bool ignore_z=true, bool multi_points=false, double len=0);

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
extern bool findIntersection(const Eigen::Vector3d& p1s, const Eigen::Vector3d& p1e,
        const Eigen::Vector3d& p2s, const Eigen::Vector3d& p2e,
        Eigen::Vector3d& intersection);

extern double cal_line_min_distance(const linestring_t& l1, const linestring_t& l2);

extern std::pair<point_t, point_t> find_closest_points(const linestring_t& curve1, const linestring_t& curve2);

extern void lines_overlap_info(const linestring_t& l1, const linestring_t& l2,
                        const std::vector<Eigen::Vector3d>& l1_dir, const std::vector<Eigen::Vector3d>& l2_dir,
                        OverlapInfoLineString& overlap_info, double thr);

extern void reverse_matched_info(const OverlapInfoLineString& in, OverlapInfoLineString& out);


}
}
