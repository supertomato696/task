

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include "utils/algorithm_util.h"
#include <boost/format.hpp>
#include <boost/assign.hpp>
#include <boost/geometry.hpp>
#include <boost/foreach.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/math/distributions/normal.hpp>
#include "pcl/sample_consensus/ransac.h"
#include "pcl/sample_consensus/sac_model_stick.h"
#include "pcl/sample_consensus/sac_model_line.h"
#include "pcl/sample_consensus/sac_model_plane.h"
#include "pcl/sample_consensus/sac_model_parallel_line.h"



namespace fsdmap {
namespace alg {

// BOOST_GEOMETRY_REGISTER_POINT_3D(Eigen::Vector3d, double, bg::cs::cartesian, x(), y(), z())
// using bg_point = bg::model::d2::point_xy<double>;
// using segment_t = bg::model::segment<bg_point>;
// using points_t = bg::model::multi_point<bg_point>;


double calc_dis(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2, bool ignore_z) {
    Eigen::Vector3d dis = pos2 - pos1;
    if (ignore_z) {
        dis.z() = 0;
    }
    return dis.norm();
}

Eigen::Vector3d find_closest_point(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& center_point) {
    Eigen::Vector3d closest_point;
    double min_dis = std::numeric_limits<double>::max();

    for (const auto& point : points) {
        double dis = calc_dis(point, center_point);
        if (dis < min_dis) {
            min_dis = dis;
            closest_point = point;
        }
    }

    return closest_point;
}


// 计算from到to的方向向量  // to ->.from
Eigen::Vector3d get_dir(const Eigen::Vector3d &to, const Eigen::Vector3d &from, bool ignore_z) {
    Eigen::Vector3d dir;
    calc_dir(to, from, dir, ignore_z);
    return dir;
}

// 计算dir的垂线，xy平面上垂直
Eigen::Vector3d get_vertical_dir(const Eigen::Vector3d &dir, bool ignore_z) {
    // 顺时针
    Eigen::Vector3d ret = {dir.y(), -dir.x(), dir.z()};
    if (ignore_z) {
        ret.z() = 0;
    }
    ret.normalize();
    return ret;
}


Eigen::Vector3d get_vertical_dir(const Eigen::Vector3d &from, const Eigen::Vector3d &to, bool ignore_z) {
    // 顺时针
    Eigen::Vector3d dir = get_dir(to, from);
    Eigen::Vector3d ret = {dir.y(), -dir.x(), dir.z()};
    if (ignore_z) {
        ret.z() = 0;
    }
    ret.normalize();
    return ret;
}

// 计算from到to的方向向量
void calc_dir(const Eigen::Vector3d &to, const Eigen::Vector3d &from,
        Eigen::Vector3d &dir, bool ignore_z) {
    dir = to - from;
    if (ignore_z) {
        dir.z() = 0;
    }
    dir.normalize();
}


// Eigen::Vector3d get_direction (const std::vector<Eigen::Vector3d> &dirs)
// {
//     const int n=dirs.size();
//     double min_sum_theta=DBL_MAX;
//     Eigen::Vector3d  dir;
//     for(int i=0;i<n;i++)
//     {
//         double sum_theta=0;
//         for(int j=0;j<n;j++)
//         {
//          sum_theta+=alg::calc_theta1(dirs[i],dirs[j],true);
//         }
//         if(sum_theta<min_sum_theta)
//         {
//             min_sum_theta=sum_theta;
//             dir=dirs[i];
//         }
//     }
//     return dir;
// }

Eigen::Vector3d rotate_vector(const Eigen::Vector3d& vec, double theta) {
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(theta * M_PI / 180.0, Eigen::Vector3d::UnitZ());
    return rotation_matrix * vec;
}

// [-2π, 2π]
double calc_theta2(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute, bool ignore_z) {
    Eigen::Vector3d v1_modified = v1;
    Eigen::Vector3d v2_modified = v2;

    if (ignore_z) {
        v1_modified.z() = 0;
        v2_modified.z() = 0;
    }
    double theta = atan2(v1_modified.y(), v1_modified.x()) - atan2(v2_modified.y(), v2_modified.x());

    if (need_acute) {
        theta = fabs(theta);
    }

    return theta * 180.0 / M_PI;
}

//[-π, π] ， 可以有正负  ，默认是二维
double calc_theta1(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute, bool ignore_z) {
    float theta = atan2(v1.y(), v1.x()) - atan2(v2.y(), v2.x()); //[-2π, 2π] 之间

    //修正后是在[-π, π]之间
	if (theta > M_PI)
		theta -= 2 * M_PI;
	if (theta < -M_PI)
		theta += 2 * M_PI;
    
	theta = theta * 180.0 / M_PI;
    if (need_acute) {
        theta = fabs(theta);
    }
	return theta;
}

// 范围值是【0,180】度
double calc_theta(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute, bool ignore_z) {
    Eigen::Vector3d v1_tmp = v1;
    Eigen::Vector3d v2_tmp = v2;
    if (ignore_z) {
        v1_tmp.z() = 0;
        v2_tmp.z() = 0;
    }
    double v1_norm = v1.squaredNorm();
    double v2_norm = v2.squaredNorm();
    if (v1_norm <= 0 || v2_norm <= 0) {
        return 0;
    }

    double rad = v1_tmp.dot(v2_tmp) / sqrt(v1_norm * v2_norm);  // [-1,1] 对应acos是 [0, pai]
    //点数计算的误差，余弦值可能会略微超出
    if (rad < -1.0) {
        rad = -1.0;
    } else if (rad > 1.0) {
        rad = 1.0;
    }
    if (need_acute) {
        rad = fabs(rad);
    }
    
    double theta = acos(rad) * 180 / M_PI;
    return theta;
}
double calc_theta(const Eigen::Vector3d &dir)
{
       auto cp_dir=dir;
       cp_dir.normalized();
       return std::atan2(cp_dir.y(),cp_dir.x());
}

double calc_theta_with_dir(const Eigen::Vector3d &v1, const Eigen::Vector3d &v2, bool need_acute, bool ignore_z) {
    double theta = calc_theta(v1, v2, need_acute, ignore_z);
    Eigen::Vector3d origin = {0, 0, 0};
    if (judge_left(v2, origin, v1) < 0) {
        return -theta;
    }
    return theta;
}

// from   在to的dir垂线方向的投影距离
double calc_vertical_dis(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const Eigen::Vector3d &dir,
        bool need_dir, bool ignore_z) {
    auto diff_sqr_norm = dir.squaredNorm();
    if (diff_sqr_norm <= 0.0) {
        return 1000;
    }

    double u = ((from - to).transpose() * dir)(0) / diff_sqr_norm;
    Eigen::Vector3d foot = to + u * dir;
    double dis = calc_dis(from, foot, ignore_z);
    
    if (need_dir) {
        int is_left = judge_left(from, to, dir);
        dis = is_left < 0 ? -dis : dis;
    }
    return dis;
}

// 计算frOmm  在to的dir方向的投影距离
double calc_hori_dis(const Eigen::Vector3d &from, const Eigen::Vector3d &to, const Eigen::Vector3d &to_dir, bool need_dir, bool ignore_z) {
    Eigen::Vector3d v_dir = get_vertical_dir(to_dir, ignore_z);
    double dis = calc_vertical_dis(from, to, v_dir, false, ignore_z);
    if (need_dir) {
        // 前正后负
        int is_left = judge_left(from, to, v_dir);
        dis = is_left < 0 ? dis : -dis;
    }
    return dis;
}

Eigen::Vector3d& rotate_yaw(const Eigen::Vector3d &src, Eigen::Vector3d &tar, double yaw) {
    // 正数为逆时针，负数为顺时针
    double radian = yaw * M_PI / 180;
    Eigen::AngleAxisd angle_axis(radian, Eigen::Vector3d(0, 0, 1));
    tar = angle_axis.matrix() * src;
    return tar;
}

// ！！！！！ 慎用  ！！！！！！，优先使用，judge_left2
//图像是左首系， 从电脑往人方向看
//从z-  往z+看， 1 :dir1在dir2 左侧  ； -1: dir1在dir2 右侧  0 :共线
int judge_left(const Eigen::Vector3d &dir1, const Eigen::Vector3d &dir2) {
    auto cross_v = dir1.cross(dir2);
    if (cross_v.z() > 0) {
        return 1;
    } else if (cross_v.z() < 0) {
        return -1;
    }
    return 0;
}

// ！！！！！ 慎用  ！！！！！！，优先使用，judge_left2
// 1: 表示p在p1的右侧
// -1: 表示p在p1的左侧
int judge_left(const Eigen::Vector3d &p, const Eigen::Vector3d &p1, const Eigen::Vector3d &dir) {
        // -1:left, 0:same, 1:right
        Eigen::Vector3d p_dir = p - p1;
        return judge_left(p_dir, dir);
}



// 右手系（前左上）： 1 :dir1在dir2 左侧  ； -1: dir1在dir2 右侧  0 :共线
int judge_left2(const Eigen::Vector3d &dir1, const Eigen::Vector3d &dir2) {
    auto cross_v = dir1.cross(dir2);
    if (cross_v.z() > 0) {
        return -1;
    } else if (cross_v.z() < 0) {
        return 1;
    }
    return 0;
}

// 右手系（前左上）： 
// 1: 表示p在p1的左侧
// 0：共线
// -1: 表示p在p1的右侧
int judge_left2(const Eigen::Vector3d &p, const Eigen::Vector3d &p1, const Eigen::Vector3d &dir) {
        // -1:left, 0:same, 1:right
        Eigen::Vector3d p_dir = p - p1;
        return judge_left2(p_dir, dir);
}

// 判断pos1是否在pos2的前方
bool judge_front(const Eigen::Vector3d &pos1, const Eigen::Vector3d &pos2,
        const Eigen::Vector3d &dir2) {
    // 判断pos1是否在pos2的前方
    if (pos1.x() == pos2.x() && pos1.y() == pos2.y() && pos1.z() == pos2.z()) {
        return false;
    }
    Eigen::Vector3d dir;
    calc_dir(pos1, pos2, dir, false);
    if (calc_theta(dir2, dir) >= 90) {
        return false;
    }
    return true;
}

//计算dir垂线方向上离pos距离是dis的点
Eigen::Vector3d get_vertical_pos(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir,
        double dis, bool ignore_z) {
    Eigen::Vector3d v_dir = get_vertical_dir(dir, ignore_z); //计算dir的垂线，xy平面上垂直
    return pos + v_dir * dis; //计算垂线方向上离pos距离是dis的点
}

//计算dir方向上离pos距离是dis的点
Eigen::Vector3d get_hori_pos(const Eigen::Vector3d &pos, const Eigen::Vector3d &dir,
        double dis, bool ignore_z) {
    Eigen::Vector3d v_dir = dir;
    if (ignore_z) {
        v_dir.z() = 0;
    }
    v_dir.normalize();
    return pos + v_dir * dis;
}

bool is_valid_pos(const Eigen::Vector3d &pos) {
    bool ret = false;
    do {
        if (isnan(pos.x()) || isnan(pos.y()) || isnan(pos.z())) {
            break;
        }
        ret = true;
    } while (0);
    return ret;
}

// 计算方向向量from_dir与to_dir的交点
bool get_cross_point(const Eigen::Vector3d &from, const Eigen::Vector3d &from_dir,
        const Eigen::Vector3d &to, const Eigen::Vector3d &to_dir, Eigen::Vector3d &ret,
        bool ignore_z, int mode, double len) {
    Eigen::Vector3d from2 = from + from_dir;
    Eigen::Vector3d to2 = to + to_dir;
    return get_cross_point_by_point(from, from2, to, to2, ret, ignore_z, mode);
}

bool get_cross_point_by_point(const Eigen::Vector3d &from1, const Eigen::Vector3d &from2,
        const Eigen::Vector3d &to1, const Eigen::Vector3d &to2, Eigen::Vector3d &ret, bool ignore_z, int mode, double len) {
    Eigen::Vector3d from_dir = get_dir(from2, from1);
    Eigen::Vector3d to_dir = get_dir(to2, to1);

    Eigen::Vector3d f1 = from1;
    Eigen::Vector3d f2 = from2;
    Eigen::Vector3d t1 = to1;
    Eigen::Vector3d t2 = to2;

    if (mode == 1 || mode == 3) {
        f1 = from1 - len * from_dir;
        f2 = from2 + len * from_dir;
    }

    if (mode == 2 || mode == 3) {
        t1 = to1 - len * to_dir;
        t2 = to2 + len * to_dir;
    }

    bg_point p_from1(f1.x(), f1.y());
    bg_point p_from2(f2.x(), f2.y());
    bg_point p_to1(t1.x(), t1.y());
    bg_point p_to2(t2.x(), t2.y());
    std::vector<bg_point> out;
    bool is_cross = bg::intersection(segment_t{p_from1, p_from2}, segment_t{p_to1, p_to2}, out);
    if (out.size() != 1) {
        return false;
    }

    if (mode == 0) {
        bg_point cross_point = out[0];
        double t = (cross_point.x() - p_from1.x()) / (p_from2.x() - p_from1.x());
        double u = (cross_point.x() - p_to1.x()) / (p_to2.x() - p_to1.x());

        // 确保交点在 AB 和 CD 两条线段内
        if (t < 0.0 || t > 1.0 || u < 0.0 || u > 1.0) {
            return false;  // 交点不在两条线段的有效区间内
        }
    }

    ret = {out[0].x(), out[0].y(), 0};
    if (ignore_z) {
        ret.z() = 0;
    } else {
        // Eigen::Vector3d dir = get_dir(ret, from1, true);
        // double dis = calc_dis(from1, ret, true);
        // dis = judge_left(ret, from, v_dir) < 0 ? dis : -dis;
        // ret.z() = from.z() + dis * dir.z();
        // TODO:当做同一高度，理论上不对
        auto diff_bak = alg::calc_dis(f1, f2, true);
        auto diff = alg::calc_dis(ret, f2, true);
        double factor= diff_bak < 1e-6 ? 0 : diff / diff_bak;
        ret.z() = factor*f1.z()+(1.0-factor)*f2.z();

        // ret.z() = (from1 + from2).z()/2;
        // 注意：有点人可能需要下面的
        // ret.z() = (to1 + to2).z()/2;
    }
    return true;
}

bool get_cross_point_with_polygon(
    const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,
    const std::vector<Eigen::Vector3d> &polygon, std::vector<Eigen::Vector3d> &intersection_points,
    bool ignore_z, int mode, double len, bool multi_points) {

    int polygon_size = polygon.size(); 
    if (polygon_size == 0) {
        return false;
    }

    Eigen::Vector3d extended_from = line_from;
    Eigen::Vector3d extended_to = line_to;
    Eigen::Vector3d direction = get_dir(line_to, line_from, ignore_z); 

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
        Eigen::Vector3d poly_from = polygon[i];
        Eigen::Vector3d poly_to = polygon[i + 1];  // 下一顶点，形成一条边
        
        Eigen::Vector3d intersection_point;
        bool is_intersect = get_cross_point_by_point(extended_from, extended_to, poly_from, poly_to, intersection_point, ignore_z, 0);
        if (is_intersect) {
            intersection_point.z() = (poly_from.z() + poly_to.z()) / 2;
            intersection_points.push_back(intersection_point);
            if (!multi_points) {
                return true;
            }
        }
    }

    // 如果不封闭的，多算一条
    bool is_polygon_closed = ((polygon[polygon_size - 1] - polygon[0]).norm() < 1e-6) ? true : false;
    if (!is_polygon_closed) {
        Eigen::Vector3d poly_from = polygon[polygon_size - 1];  
        Eigen::Vector3d poly_to = polygon[0]; 

        Eigen::Vector3d intersection_point;
        bool is_intersect = get_cross_point_by_point(extended_from, extended_to, poly_from, poly_to, intersection_point, ignore_z, 0);
        if (is_intersect) {
            intersection_point.z() = (poly_from.z() + poly_to.z()) / 2;
            intersection_points.push_back(intersection_point);
            if (!multi_points) {
                return true;
            }
        }
    }

    return !intersection_points.empty();
}


bool get_cross_point_with_curve_segment(
    const Eigen::Vector3d &line_from, const Eigen::Vector3d &line_to,
    const std::vector<Eigen::Vector3d> &polygon,
    std::vector<UsefulPnt> &interp_infos,
    bool ignore_z, int mode, double len, bool multi_points) {
    
    interp_infos.clear();
    int polygon_size = polygon.size(); 

    Eigen::Vector3d extended_from = line_from;
    Eigen::Vector3d extended_to = line_to;
    Eigen::Vector3d direction = get_dir(line_to, line_from, ignore_z); 

    if (ignore_z) {
        direction.z() = 0;
    }

    if (mode == 3) {
        extended_from -= len * direction;
        extended_to += len * direction;
    } else if (mode == 1) {
        extended_from -= len * direction;
    } else if (mode == 2) {
        extended_to += len * direction;
    }

    // 遍历多边形的每一条边
    for (int i = 0; i < polygon_size - 1; ++i) {  // 处理从 0 到 n-2 个顶点
        Eigen::Vector3d poly_from = polygon[i];
        Eigen::Vector3d poly_to = polygon[i + 1];  // 下一顶点，形成一条边
        
        Eigen::Vector3d intersection_point;
        bool is_intersect = get_cross_point_by_point(extended_from, extended_to, poly_from, poly_to, intersection_point, ignore_z, 0);
        if (is_intersect) {
            Eigen::Vector3d dir = direction; // ！！！！ 注意这里的方向和模板类中的不一样
            intersection_point.z() = (poly_from.z() + poly_to.z()) / 2;

            interp_infos.push_back(UsefulPnt(intersection_point));
            if (!multi_points) {
                return true;
            }
        }
    }
    return !interp_infos.empty();
}


Eigen::Vector3d project_points_to_dir(const Eigen::Vector3d &base_point, const Eigen::Vector3d &dir, const std::vector<Eigen::Vector3d> &points, bool ignore_z, int mode) {
    if (points.empty()) {
        LOG_ERROR("project_points_to_dir: points is empty");
    }

    Eigen::Vector3d normalized_dir = dir.normalized();
    if (ignore_z) {
        normalized_dir.z() = 0;
        normalized_dir.normalize();
    }

    double min_proj = std::numeric_limits<double>::max();
    double max_proj = std::numeric_limits<double>::lowest();
    Eigen::Vector3d min_point, max_point;

    for (const auto &point : points) {
        Eigen::Vector3d proj_point = point - base_point;
        if (ignore_z) {
            proj_point.z() = 0;
        }
        double projection = proj_point.dot(normalized_dir);
        if (projection < min_proj) {
            min_proj = projection;
            min_point = point;
        }
        if (projection > max_proj) {
            max_proj = projection;
            max_point = point;
        }
    }

    if (mode == 1) {
        return min_point;
    } else if (mode == 2) {
        return max_point;
    } else {
        LOG_ERROR("Invalid mode. Mode should be 1 or 2.");
        throw std::invalid_argument("Invalid mode. Mode should be 1 or 2.");
    }
}

double project_point_to_line(const Eigen::Vector3d& base_point, const Eigen::Vector3d& dir, const Eigen::Vector3d& point, bool ignore_z) {
    Eigen::Vector3d normalized_dir = dir.normalized();
    Eigen::Vector3d vec = point - base_point;
    if (ignore_z) {
        normalized_dir.z() = 0;
        vec.z() = 0;
    }
    return vec.dot(normalized_dir);
}

double get_max_vertical_dis(const Eigen::Vector3d &pos1, const Eigen::Vector3d &dir1,
        const Eigen::Vector3d &pos2, const Eigen::Vector3d &dir2, bool ignore_z) {
    double dis_1 = calc_vertical_dis(pos1, pos2, dir2, ignore_z);
    double dis_2 = calc_vertical_dis(pos2, pos1, dir1, ignore_z);
    return std::max(dis_1, dis_2);
}


Eigen::Vector3d cal_average(const std::vector<Eigen::Vector3d>& points) {
    Eigen::Vector3d sum(0, 0, 0);
    for (const auto& point : points) {
        sum += point;
    }
    return sum / points.size();
}


// 计算标准差
Eigen::Vector3d cal_standard_deviation(const std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& mean) {
    Eigen::Vector3d variance(0, 0, 0);
    for (const auto& point : points) {
        Eigen::Vector3d diff = point - mean;
        variance += diff.cwiseProduct(diff);  
    }
    variance /= points.size();

    return variance.cwiseSqrt();
}



double calc_avg(const std::vector<double> &vec, int32_t start, int32_t end) {
    return calc_median(vec, end - start, 0.5, start, end);
}

// 计算一个向量的某个子区间内的加权中位数
// vec：输入的向量。
// num：用于计算中位数的元素数量。
// position：中位数的位置偏移（0.0 到 1.0 之间）。
// start：子区间的起始索引。
// end：子区间的结束索引（不包含）
double calc_median(const std::vector<double> &vec, int num, double position, int32_t start, int32_t end) {
    if (end == 0 || end > vec.size()) {
        end = vec.size();
    }
    if (start < 0) {
        start = 0;
    }
    int32_t size = end - start;
    if (size < 1) {
        return 0;
    }
    if (num == 0) {
        num = 1;
    }
    if (num > size) {
        num = size;
    }
    std::vector<double> calc_vec;
    calc_vec.reserve(vec.size());
    calc_vec.insert(calc_vec.begin(), vec.begin() + start, vec.begin() + end);
    std::sort(calc_vec.begin(), calc_vec.end());
    int32_t middle = (size - 1) * position;
    double total_value = 0;
    for (int i = 0; i < num; ++i) {
        int32_t pos = middle - (num / 2) + i;
        if (pos < 0) {
            continue;
        }
        if (pos > size - 1) {
            break;
        }
        total_value += calc_vec[pos];
    }
    return total_value / num;
}

double calc_line_variance(const std::vector<double> &data, double miu, double sigma,
        int64_t start, int64_t end) {
    if (end == 0 || end > data.size()) {
        end = data.size();
    }
    if (start < 0) {
        start = 0;
    }
    int64_t size = end - start;
    float a = 0;
    float b = 0;
    if (fit_line2d(data, a, b, start, end) < 0) {
        return calc_variance(data, miu, sigma);
    }
    std::vector<double> gap_vec;
    gap_vec.reserve(size);
    for (int i = start; i < end; ++i) {
        double line_value = a * i + b;
        double gap = data[i] - line_value;
        gap_vec.push_back(gap);
    }
    return calc_variance(gap_vec, miu, sigma);
}

double calc_variance(const std::vector<double> &data, double miu, double sigma, 
        int64_t start, int64_t end) {
    if (end == 0 || end > data.size()) {
        end = data.size();
    }
    if (start < 0) {
        start = 0;
    }
    int64_t size = end - start;
    if (size < 2) {
        return 0.0;
    }
    double sum = std::accumulate(data.begin() + start, data.begin() + end, 0.0);
    double mean =  sum / data.size(); //均值
    return calc_variance_with_mean(data, mean, miu, sigma);
}

double calc_variance_with_mean(const std::vector<double> &data, 
        double mean, double miu, double sigma
        , int64_t start, int64_t end) {
    if (end == 0 || end > data.size()) {
        end = data.size();
    }
    if (start < 0) {
        start = 0;
    }
    int64_t size = end - start;
    if (size < 2) {
        return 0.0;
    }
    double sum = std::accumulate(data.begin(), data.end(), 0.0);
    double accum  = 0.0;
    for (int32_t i = start; i < end; ++i) {
        auto &d = data[i];
        double delta = d - mean;
        double factor = 1;
        if (sigma > 0) {
            factor += LogNormal(i, miu, sigma);
        }
        accum += pow(delta * factor, 2);
    }
    // std::for_each(data.begin(), data.end(), [&](const double d) {
    // });

    double stdev = accum/(end - start - 1); //方差
    // double stdev = sqrt(accum/(data.size() - 1)); //标准差
    return stdev;
}

// 归一化得分通过将观测值的概率密度除以分布的最大概率密度，使得我们可以比较不同观测值在分布中的相对重要性
double calc_score_by_gaussian(double x, double sigma, double miu) {
    double max_y = LogNormal(miu, miu, sigma);
    double value = LogNormal(x, miu, sigma);
    return value / max_y; //归一化
}

int fit_line2d(const std::vector<double>& data, float& coff_a, float& coff_b, int32_t start, int32_t end) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>());
    pts->reserve(data.size());
    if (end == 0) {
        end = data.size();
    }
    if (start < 0) {
        start = 0;
    }
    for (int32_t i = start; i < end; ++i) {
        pcl::PointXYZ p = {i, data[i], 0};
        pts->push_back(p);
    }
    if (pts->size() < 2) {
        coff_a = 0;
        coff_b = 0;
        return -1;
    }
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pts));
    pcl::RandomSampleConsensus<pcl::PointXYZ> rsc(line);
    rsc.setDistanceThreshold(1);
    int ret = 0;
    if (rsc.computeModel()) {
        Eigen::VectorXf coeff;
        rsc.getModelCoefficients(coeff);
        Eigen::Vector3d pt(coeff[0], coeff[1], coeff[2]);
        Eigen::Vector3d dir(coeff[3], coeff[4], coeff[5]);
        coff_a = dir.y() / dir.x();
        coff_b = pt.y() - coff_a * pt.x();
        ret = line->countWithinDistance(coeff, 1);
    } else {
        rsc.setDistanceThreshold(2);
        if (rsc.computeModel()) {
            Eigen::VectorXf coeff;
            rsc.getModelCoefficients(coeff);
            Eigen::Vector3d pt(coeff[0], coeff[1], coeff[2]);
            Eigen::Vector3d dir(coeff[3], coeff[4], coeff[5]);
            coff_a = dir.y() / dir.x();
            coff_b = pt.y() - coff_a * pt.x();
            ret = line->countWithinDistance(coeff, 2) * 2 / 3;
        } else {
            coff_a = 0;
            coff_b = 0;
            ret = -1;
        }
    }
    return ret;
}

bool fit_line3d(const std::vector<Eigen::Vector3d>& data, float& coff_a, float& coff_b, int32_t start, int32_t end) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pts(new pcl::PointCloud<pcl::PointXYZ>());
    pts->reserve(data.size());
    if (end == 0) {
        end = data.size();
    }
    if (start < 0) {
        start = 0;
    }
    for (int32_t i = start; i < end; ++i) {
        auto& ele = data[i];
        pcl::PointXYZ p = {ele.x(), ele.y(), ele.z()};
        pts->push_back(p);
    }
    if (pts->size() < 2) {
        coff_a = 0;
        coff_b = 0;
        return false;
    }
    pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line(new pcl::SampleConsensusModelLine<pcl::PointXYZ>(pts));
    pcl::RandomSampleConsensus<pcl::PointXYZ> rsc(line);
    rsc.setDistanceThreshold(1);
    int ret = 0;
    if (rsc.computeModel()) {
        Eigen::VectorXf coeff;
        rsc.getModelCoefficients(coeff);
        Eigen::Vector3d pt(coeff[0], coeff[1], coeff[2]);
        Eigen::Vector3d dir(coeff[3], coeff[4], coeff[5]);
        coff_a = dir.y() / dir.x();
        coff_b = pt.y() - coff_a * pt.x();
        ret = line->countWithinDistance(coeff, 1);
    } else {
        rsc.setDistanceThreshold(2);
        if (rsc.computeModel()) {
            Eigen::VectorXf coeff;
            rsc.getModelCoefficients(coeff);
            Eigen::Vector3d pt(coeff[0], coeff[1], coeff[2]);
            Eigen::Vector3d dir(coeff[3], coeff[4], coeff[5]);
            coff_a = dir.y() / dir.x();
            coff_b = pt.y() - coff_a * pt.x();
            ret = line->countWithinDistance(coeff, 2) * 2 / 3;
        } else {
            coff_a = 0;
            coff_b = 0;
            ret = -1;
        }
    }
    return ret >= 0;
}

double LogNormal(double x, double miu, double sigma) {
    //对数正态分布概率密度函数
    // return 1.0 / (sqrt(2 * M_PI) * sigma) * exp(-1 * pow(x - miu, 2) / pow(sigma, 2));
    // std::normal_distribution<> norm {miu, sigma};
    // static double v1 = 1 / (sqrt(2 * M_PI));
    boost::math::normal_distribution<> norm(miu, sigma);
    double v_pdf = boost::math::pdf(norm, x);
    return v_pdf;
}

double calc_area_rate(std::vector<Eigen::Vector3d> &polygon1, 
        std::vector<Eigen::Vector3d> &polygon2, int mode) {
    // 逆时针
    if (polygon1.size() <= 2 || polygon2.size() <= 2) {
        return 0;
    }
    typedef boost::geometry::model::d2::point_xy<double> DPoint;
    typedef boost::geometry::model::polygon<DPoint, false> DPolygon;

    auto lstOf1 = boost::assign::list_of(DPoint(polygon1[0].x(), polygon1[0].y()));
    for (int i = 1; i < polygon1.size(); i++) {
        auto &pt = polygon1[i];
        lstOf1(DPoint(pt.x(), pt.y()));
    }
    DPolygon poly1;
    poly1.outer().assign(lstOf1.begin(), lstOf1.end());

    auto lstOf2 = boost::assign::list_of(DPoint(polygon2[0].x(), polygon2[0].y()));
    for (int i = 1; i < polygon2.size(); i++) {
        auto &pt = polygon2[i];
        lstOf2(DPoint(pt.x(), pt.y()));
    }
    DPolygon poly2;
    poly2.outer().assign(lstOf2.begin(), lstOf2.end());
    boost::geometry::correct(poly1);
    boost::geometry::correct(poly2);
    double area_1 = boost::geometry::area(poly1);
    double area_2 = boost::geometry::area(poly2);
    if (area_1 + area_2 <= 0.1) {
        return 0;
    }
    std::vector<DPolygon> instersections;
    boost::geometry::intersection(poly1, poly2, instersections);
    if (instersections.empty()) {
        return 0;
    }
    double i_area = 0.0;
    BOOST_FOREACH(DPolygon const &p, instersections) {
        double tmp_ara = boost::geometry::area(p);
        if (tmp_ara < 0.0) {
            LOG_ERROR("bad area.");
        }
        i_area += boost::geometry::area(p);
    }
    double total_erea = area_1 + area_2;
    if (mode == 1) {
        total_erea = std::min(area_1, area_2);
    } else if (mode == 2) {
        total_erea = std::max(area_1, area_2);
    }
    return i_area / total_erea;
}

double segment_iou(Eigen::Vector3d &start1, Eigen::Vector3d &end1,
        Eigen::Vector3d &start2, Eigen::Vector3d &end2) {
    Eigen::Vector3d dir1 = get_vertical_dir(end1, start1);
    Eigen::Vector3d dir2 = get_vertical_dir(end2, start2);
    double inter = 0;
    double base = 0;
    { 
        double l_dis_1 = alg::calc_vertical_dis(start2, start1, dir1, true);
        double r_dis_1 = alg::calc_vertical_dis(end2, start1, dir1, true);
        double r_dis_2 = alg::calc_vertical_dis(end1, start1, dir1, true);
        double l1 = std::max(l_dis_1, 0.0);
        double r1 = std::min(r_dis_1, r_dis_2);
        double l2 = std::min(l_dis_1, 0.0);
        double r2 = std::max(r_dis_1, r_dis_2);
        inter += r1 - l1;
        base += r2 - l2;
    }
    { 
        double l_dis_1 = alg::calc_vertical_dis(start1, start2, dir2, true);
        double r_dis_1 = alg::calc_vertical_dis(end1, start2, dir2, true);
        double r_dis_2 = alg::calc_vertical_dis(end2, start2, dir2, true);
        double l1 = std::max(l_dis_1, 0.0);
        double r1 = std::min(r_dis_1, r_dis_2);
        double l2 = std::min(l_dis_1, 0.0);
        double r2 = std::max(r_dis_1, r_dis_2);
        inter += r1 - l1;
        base += r2 - l2;
    }
    return inter / base;
}

std::string cat_m(Eigen::MatrixXf m) {
    std::string ret;
    int rows = m.rows();
    int cols = m.cols();
    for (int i = 0; i < rows; ++i) {
        ret += "(";
        for (int j = 0; j < cols; ++j) {
            ret += utils::fmt(",{:.2f}", m(i, j));
        }
        ret += ")";
    }
    return ret;
}

bool fit_circle_algebraic(std::vector<Eigen::Vector3d> &vecPoints,
        double &cir_x, double &cir_y, double &cir_r) {
    if (vecPoints.size() == 0) {
        return false;
    }

    int i, iter, IterMAX = 99;

    double Xi, Yi, Zi;
    double Mz, Mxy, Mxx, Myy, Mxz, Myz, Mzz, Cov_xy, Var_z;
    double A0, A1, A2, A22, A3, A33;
    double Dy, xnew, x, ynew, y;
    double DET, Xcenter, Ycenter;

    int n = vecPoints.size();
    double avg_x = 0;
    double avg_y = 0;
    double avg_z = 0;
    for (size_t i = 0; i < vecPoints.size(); ++i) {
        auto &curr = vecPoints[i];
        avg_x += curr.x();
        avg_y += curr.y();
        avg_z += curr.z();
    }
    avg_x /= n;
    avg_y /= n;
    avg_z /= n;

    //     computing moments

    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.;

    for (i = 0; i <n; i++) {
        auto &curr = vecPoints[i];
        Xi = curr.x() - avg_x;   //  centered x-coordinates
        Yi = curr.y() - avg_y;   //  centered y-coordinates
        Zi = Xi*Xi + Yi*Yi;

        Mxy += Xi*Yi;
        Mxx += Xi*Xi;
        Myy += Yi*Yi;
        Mxz += Xi*Zi;
        Myz += Yi*Zi;
        Mzz += Zi*Zi;
    }
    Mxx /= n;
    Myy /= n;
    Mxy /= n;
    Mxz /= n;
    Myz /= n;
    Mzz /= n;

    //      computing coefficients of the characteristic polynomial

    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Var_z = Mzz - Mz*Mz;
    A3 = 4 * Mz;
    A2 = -3 * Mz*Mz - Mzz;
    A1 = Var_z*Mz + 4 * Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    A22 = A2 + A2;
    A33 = A3 + A3 + A3;

    //    finding the root of the characteristic polynomial
    //    using Newton's method starting at x=0
    //     (it is guaranteed to converge to the right root)

    for (x = 0., y = A0, iter = 0; iter < IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
        Dy = A1 + x*(A22 + A33*x);
        xnew = x - y / Dy;
        if ((xnew == x) || (!std::isfinite(xnew))) break;
        ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
        if (fabs(ynew) >= fabs(y))  break;
        x = xnew;  y = ynew;
    }

    //       computing paramters of the fitting circle

    DET = x*x - x*Mz + Cov_xy;
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy) / DET / 2;
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy) / DET / 2;

    //       assembling the output
    cir_x = Xcenter + avg_x;
    cir_y = Ycenter + avg_y;
    cir_r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);

    return true;
}

bool fit_circle(std::vector<Eigen::Vector3d> &point_list, 
        double &cir_x, double &cir_y, double &cir_r) {

    if (point_list.size() == 0) {
        return false;
    }

    //offset
    // std::vector<Geometries::Coordinate> vecPoints = vecPoints_Original;
    std::vector<Eigen::Vector3d> vecPoints = point_list;
    int n = vecPoints.size();
    auto ptFront = vecPoints[0];
    double avg_x = 0;
    double avg_y = 0;
    double avg_z = 0;
    for (size_t i = 0; i < vecPoints.size(); i++) {
        auto &curr = vecPoints[i];
        curr -= ptFront;
        avg_x += curr.x();
        avg_y += curr.y();
        avg_z += curr.z();
    }
    avg_x /= n;
    avg_y /= n;
    avg_z /= n;

    typedef struct {
        double x;
        double y;
        double r;
    } circle;
    circle circleIni = {0, 0, 0};
    if (!fit_circle_algebraic(vecPoints, circleIni.x, circleIni.y, circleIni.r)) {
        return false;
    }

    int code, i, iter, inner, IterMAX = 99;

    double factorUp = 10., factorDown = 0.04, lambda, ParLimit = 1.e+6;
    double dx, dy, ri, u, v;
    double Mu, Mv, Muu, Mvv, Muv, Mr, UUl, VVl, Nl, F1, F2, F3, dX, dY, dR;
    double epsilon = 3.e-8;
    double G11, G22, G33, G12, G13, G23, D1, D2, D3;

    circle Old, New;

    double oldCircleSd, newCircleSd;
    //       starting with the given initial circle (initial guess)

    New = circleIni;

    double LambdaIni = 0.01;

    //       compute the root-mean-square error via function Sigma; see Utilities.cpp
    // newCircleSd = DeviationAnalysis(vecPoints, New.x,New.y,New.r);
    {
        double sum = 0;
        for (auto i = 0; i < n; ++i) {
            auto &p = vecPoints[i];
            auto d = sqrt(pow((p.x() - New.x), 2) + pow((p.y() - New.x), 2)) - New.r;
            sum += d*d/ n;
        }

        newCircleSd = sqrt(sum);
    }

    //       initializing lambda, iteration counters, and the exit code
    lambda = LambdaIni;
    iter = inner = code = 0;

NextIteration:

    Old = New;
    oldCircleSd = newCircleSd;

    if (++iter > IterMAX) { code = 1;  goto enough; }

    //       computing moments
    Mu = Mv = Muu = Mvv = Muv = Mr = 0.;

    for (i = 0; i <n; i++)
    {
        dx = vecPoints[i].x() - Old.x;
        dy = vecPoints[i].y() - Old.y;
        ri = sqrt(dx*dx + dy*dy);
        u = dx / ri;
        v = dy / ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    Mu /= n;
    Mv /= n;
    Muu /= n;
    Mvv /= n;
    Muv /= n;
    Mr /= n;

    //       computing matrices
    F1 = Old.x + Old.r*Mu - avg_x;
    F2 = Old.y + Old.r*Mv - avg_y;
    F3 = Old.r - Mr;

try_again:

    UUl = Muu + lambda;
    VVl = Mvv + lambda;
    Nl = 1.0 + lambda;

    //  Cholesly decomposition
    G11 = sqrt(UUl);
    G12 = Muv / G11;
    G13 = Mu / G11;
    G22 = sqrt(VVl - G12*G12);
    G23 = (Mv - G12*G13) / G22;
    G33 = sqrt(Nl - G13*G13 - G23*G23);

    D1 = F1 / G11;
    D2 = (F2 - G12*D1) / G22;
    D3 = (F3 - G13*D1 - G23*D2) / G33;

    dR = D3 / G33;
    dY = (D2 - G23*dR) / G22;
    dX = (D1 - G12*dY - G13*dR) / G11;

    if ((fabs(dR) + fabs(dX) + fabs(dY)) / (1.0 + Old.r) < epsilon) goto enough;

    //       updating the parameters

    New.x = Old.x - dX;
    New.y = Old.y - dY;

    if (fabs(New.x) > ParLimit || fabs(New.y) > ParLimit) { code = 3; goto enough; }

    New.r = Old.r - dR;

    if (New.r <= 0.)
    {
        lambda *= factorUp;
        if (++inner > IterMAX) { code = 2;  goto enough; }
        goto try_again;
    }

    //       compute the root-mean-square error via function Sigma; see Utilities.cpp
    // newCircleSd = DeviationAnalysis(vecPoints, New.x, New.y, New.r);
    {
        double sum = 0;
        for (auto i = 0; i < n; ++i) {
            auto &p = vecPoints[i];
            auto d = sqrt(pow((p.x() - New.x), 2) + pow((p.y() - New.x), 2)) - New.r;
            sum += d*d/ n;
        }

        newCircleSd = sqrt(sum);
    }

    //       check if improvement is gained
    if (newCircleSd < oldCircleSd)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
        if (++inner > IterMAX) { code = 2;  goto enough; }
        lambda *= factorUp;
        goto try_again;
    }

    //       exit

enough:
    cir_x = Old.x + ptFront.x();
    cir_y = Old.y + ptFront.y();
    cir_r = Old.r;

    return (code ==0);
}

double fit_circle2(std::vector<Eigen::Vector3d> point_list) {
    double sumX = 0, sumY = 0;
    double sumXX = 0, sumYY = 0, sumXY = 0;
    double sumXXX = 0, sumXXY = 0, sumXYY = 0, sumYYY = 0;

    for (int i = 0; i < point_list.size(); ++i) {
        auto &p = point_list[i];

        sumX += p.x();
        sumY += p.y();
        sumXX += p.x() * p.x();
        sumYY += p.y() * p.y();
        sumXY += p.x() * p.y();
        sumXXX += p.x() * p.x() * p.x();
        sumXXY += p.x() * p.x() * p.y();
        sumXYY += p.x() * p.y() * p.y();
        sumYYY += p.y() * p.y() * p.y();
    }

    int pCount = point_list.size();
    double M1 = pCount * sumXY - sumX * sumY;
    double M2 = pCount * sumXX - sumX * sumX;
    double M3 = pCount * (sumXXX + sumXYY) - sumX * (sumXX + sumYY);
    double M4 = pCount * sumYY - sumY * sumY;
    double M5 = pCount * (sumYYY + sumXXY) - sumY * (sumXX + sumYY);

    double a = (M1 * M5 - M3 * M4) / (M2*M4 - M1 * M1);
    double b = (M1 * M3 - M2 * M5) / (M2*M4 - M1 * M1);
    double c = -(a * sumX + b * sumY + sumXX + sumYY) / pCount;

    //圆心XY 半径
    double xCenter = -0.5*a;
    double yCenter = -0.5*b;
    double radius = 0.5 * sqrt(a * a + b * b - 4 * c);
    // centerP[0] = xCenter;
    // centerP[1] = yCenter;
    return radius;
}

double fit_circle1(std::vector<Eigen::Vector3d> point_list) {
    std::vector<float> circle;
    int num = point_list.size();

    Eigen::MatrixXf M(num, 3);

    for (int i = 0; i < num; ++i) {
        auto &pt = point_list[i];
        M(i, 0) = pt.x();
        M(i, 1) = pt.y();
        M(i, 2) = pt.z();
    }

    Eigen::MatrixXf L1 = Eigen::MatrixXf::Ones(num, 1);
    Eigen::MatrixXf M1(3, num);

    M1 = M.transpose();
    auto d1 = cat_m(M);
    auto d2 = cat_m(M1);
    auto d3 = cat_m(M.transpose() * M);
    auto d4 = cat_m((M.transpose() * M).inverse());
    
    Eigen::Vector3f A = (M.transpose() * M).inverse() * M.transpose() * L1;

    auto A1 = A;
    A1.normalize();
    // printf("plane normal:%f,%f,%f\n", A1(0), A1(1), A1(2));

    Eigen::MatrixXf B = Eigen::MatrixXf::Zero(num - 1, 3);

    for (int i = 0; i < num - 1; i++) {
        B.row(i) = M.row(i + 1) - M.row(i);
    }

    Eigen::MatrixXf L2 = Eigen::MatrixXf::Zero(num - 1, 1);
    for (int i = 0; i < num - 1; i++)
    {
        L2(i) = (M(i + 1, 0) * M(i + 1, 0) + M(i + 1, 1) * M(i + 1, 1) + M(i + 1, 2) * M(i + 1, 2)
            - (M(i, 0) * M(i, 0) + M(i, 1) * M(i, 1) + M(i, 2) * M(i, 2))) / 2.0;
    }

    Eigen::Matrix4f D;
    D.setZero();
    D.block<3, 3>(0, 0) = B.transpose() * B;
    D.block<3, 1>(0, 3) = A;
    D.block<1, 3>(3, 0) = A.transpose();

    Eigen::Vector4f L3((B.transpose() * L2)(0), (B.transpose() * L2)(1), (B.transpose() * L2)(2), 1);
    Eigen::Vector4f C = D.inverse() * L3;

    float radius = 0;
    for (int i = 0; i < num; i++)
    {
        Eigen::Vector3f tmp(M.row(i)(0) - C(0), M.row(i)(1) - C(1), M.row(i)(2) - C(2));
        radius = radius + sqrt(tmp(0) * tmp(0) + tmp(1) * tmp(1) + tmp(2) * tmp(2));
    }

    radius = radius / num;
    return radius;
    // printf("radius:%f\n", radius);
    // printf("circle center:%f,%f,%f\n", C(0), C(1), C(2));
    // printf("lamda:%f\n", C(3));
}

bool point_in_polygon(Eigen::Vector3d &point, std::vector<Eigen::Vector3d> &polygon) {
    std::vector<Eigen::Vector3d> points;
    std::vector<int> indexs;
    points.push_back(point);
    return points_in_polygon(polygon, points, indexs);
}

bool points_in_polygon(std::vector<Eigen::Vector3d> &polygon,
        std::vector<Eigen::Vector3d> &points, std::vector<int> &in_points_index) {
    if (polygon.size() <= 2) {
        return false;
    }
    typedef boost::geometry::model::d2::point_xy<double> DPoint;
    typedef boost::geometry::model::polygon<DPoint, false> DPolygon;
    auto lstOf = boost::assign::list_of(DPoint(polygon[0].x(), polygon[0].y()));
    for (int i = 1; i < polygon.size(); i++) {
        auto &pt = polygon[i];
        lstOf(DPoint(pt.x(), pt.y()));
    }
    DPolygon poly1;    
    poly1.outer().assign(lstOf.begin(), lstOf.end());
    if (!bg::is_valid(poly1)) {
        bg::correct(poly1);
        if (!bg::is_valid(poly1)) {
            LOG_ERROR("1 polygon is not normal! please check !");
        }
    }
    for (int i = 0; i < points.size(); ++i) {
        auto &point = points[i];
        if (boost::geometry::within(DPoint(point.x(), point.y()), poly1)) {
            in_points_index.push_back(i);
        }
    }
    if (in_points_index.size() > 0) {
        return true;
    }
    return false;
}

bool any_point_in_polygon(std::vector<Eigen::Vector3d> &polygon, std::vector<Eigen::Vector3d> &points) {
    if (polygon.size() <= 2) {
        return false;
    }
    typedef boost::geometry::model::d2::point_xy<double> DPoint;
    typedef boost::geometry::model::polygon<DPoint, false> DPolygon;
    auto lstOf = boost::assign::list_of(DPoint(polygon[0].x(), polygon[0].y()));
    for (int i = 1; i < polygon.size(); i++) {
        auto &pt = polygon[i];
        lstOf(DPoint(pt.x(), pt.y()));
    }
    std::vector<int> in_points_index;
    DPolygon poly1;
    poly1.outer().assign(lstOf.begin(), lstOf.end());
    if (!bg::is_valid(poly1)) {
        bg::correct(poly1);
        if (!bg::is_valid(poly1)) {
            // LOG_ERROR("2 polygon is not normal! please check !");
        }
    }
    for (int i = 0; i < points.size(); ++i) {
        auto &point = points[i];
        if (boost::geometry::within(DPoint(point.x(), point.y()), poly1)) {
            in_points_index.push_back(i);
            break;
        }
    }
    if (in_points_index.size() > 0) {
        return true;
    }
    return false;
}

bool findIntersection(const Eigen::Vector3d& p1s, const Eigen::Vector3d& p1e,
    const Eigen::Vector3d& p2s, const Eigen::Vector3d& p2e,
    Eigen::Vector3d& intersection, int mode) {
        // 计算方向向量
        Eigen::Vector3d dir1 = p1e - p1s;
        Eigen::Vector3d dir2 = p2e - p2s;

        // 计算分母
        double denominator = dir1.x() * dir2.y() - dir1.y() * dir2.x();
        // 如果分母为零，说明两条线段平行或重合，没有交点
        if (fabs(denominator) < 1e-6) {
            return false; // 平行或重合，没有交点
        }

        // 计算 t 和 u
        double t = ((p2s.x() - p1s.x()) * dir2.y() - (p2s.y() - p1s.y()) * dir2.x()) / denominator;
        double u = ((p2s.x() - p1s.x()) * dir1.y() - (p2s.y() - p1s.y()) * dir1.x()) / denominator;

        // 检查 t 和 u 是否在 [0, 1] 范围内
        if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
            // 计算交点坐标
            intersection = p1s + t * dir1;

            if (mode == 1) {
                auto diff_bak = alg::calc_dis(p1s, p1e, true);
                auto diff = alg::calc_dis(intersection, p1e, true);
                double factor= diff_bak < 1e-6 ? 0 : diff / diff_bak;
                intersection.z() = factor*p1s.z()+(1.0-factor)*p1e.z();
            } else if (mode == 2) {
                auto diff_bak = alg::calc_dis(p2s, p2e, true);
                auto diff = alg::calc_dis(intersection, p2e, true);
                double factor= diff_bak < 1e-6 ? 0 : diff / diff_bak;
                intersection.z() = factor*p2s.z()+(1.0-factor)*p2e.z();
            }

            return true; // 有交点
        }

        return false; // 没有交点
}
Eigen::Vector3d get_direction (const std::vector<Eigen::Vector3d> &dirs)
{
    const int n=dirs.size();
    double min_sum_theta=DBL_MAX;
    Eigen::Vector3d  dir;
    for(int i=0;i<n;i++)
    {
        double sum_theta=0;
        for(int j=0;j<n;j++)
        {
         sum_theta+=alg::calc_theta1(dirs[i],dirs[j],true);
        }
        if(sum_theta<min_sum_theta)
        {
            min_sum_theta=sum_theta;
            dir=dirs[i];
        }
    }
    return dir;
}
 bool is_bit_set(int n, int k) {
    return (n & (1 << k)) != 0;
}



extern std::vector<Eigen::Vector3d> cut_line_in_polygon(std::vector<Eigen::Vector3d>& polygon_pts, const std::vector<Eigen::Vector3d>& input_points) {
    std::vector<PointInPolygon> points;
    for (const auto& pt : input_points) {
        points.emplace_back(pt, false);
    }

    if (points.size() > 1) {
        for (int i = 1; i < points.size(); ++i) {
            Eigen::Vector3d& current_point = points[i].point; // 当前点A
            Eigen::Vector3d& previous_point = points[i - 1].point; // 上一个点B

            bool current_flag = alg::point_in_polygon(current_point, polygon_pts);
            bool previous_flag = alg::point_in_polygon(previous_point, polygon_pts);

            // 如果当前点和前一个点的 flag 标志不同，则需要计算交点
            if (current_flag != previous_flag) {
                std::vector<Eigen::Vector3d> cross_points;
                // TODO:qzc 改变 ignore_z == false,试试
                bool is_intersect = alg::get_cross_point_with_polygon(previous_point, current_point, polygon_pts, cross_points, true);
                if (is_intersect) {
                    PointInPolygon new_point(cross_points[0], true); 
                    points.insert(points.begin() + i, new_point);
                    ++i; 
                }
            }
        }
    }

    //剔除不在多边形内的点，除非它是交点
    std::vector<Eigen::Vector3d> result_points;
    for (auto& lane_point : points) {
        if (lane_point.is_intersection_point || alg::point_in_polygon(lane_point.point, polygon_pts)) {
            result_points.push_back(lane_point.point);
        }
    }

    return result_points;
}

bool is_in_convex_polygon( const std::vector<Eigen::Vector3d>& polygon,const Eigen::Vector3d& point) {
    int n = polygon.size();
    if (n < 3) return false;  // 多边形至少需要3个顶点

    int intersection_count = 0;
    Eigen::Vector3d p1 = polygon[0];

    // 遍历多边形的每条边
    for (int i = 1; i <= n; ++i) {
        Eigen::Vector3d p2 = polygon[i % n];  // 处理最后一条边回到起点的情况

        // 判断射线是否与边相交
        if (point.y() > std::min(p1.y(), p2.y()) && point.y() <= std::max(p1.y(), p2.y())) {
            // 计算交点的 x 坐标
            double x_intersect = (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

            // 如果交点在点的右侧，则计数加1
            if (x_intersect > point.x()) {
                intersection_count++;
            }
        }

        p1 = p2;  // 移动到下一条边
    }

    // 如果交点数为奇数，则点在多边形内部
    return (intersection_count % 2 == 1);
}


bool out_of_china(double lng, double lat){
    if (lng < 72.004 || lng > 137.8347)
        return true;
    if (lat < 0.8293 || lat > 55.8271)
        return true;
    return false;
}

double transform_lat(double lng, double lat)
{
    double ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat +
                 0.1 * lng * lat + 0.2 * sqrt(fabs(lng));

    ret += (20.0 * sin(6.0 * lng * pi) + 20.0 *
                                             sin(2.0 * lng * pi)) *
           2.0 / 3.0;
    ret += (20.0 * sin(lat * pi) + 40.0 *
                                       sin(lat / 3.0 * pi)) *
           2.0 / 3.0;
    ret += (160.0 * sin(lat / 12.0 * pi) + 320 *
                                               sin(lat * pi / 30.0)) *
           2.0 / 3.0;
    return ret;
}

double transform_lng(double lng, double lat)
{
    double ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng +
                 0.1 * lng * lat + 0.1 * sqrt(fabs(lng));
    ret += (20.0 * sin(6.0 * lng * pi) + 20.0 *
                                             sin(2.0 * lng * pi)) *
           2.0 / 3.0;
    ret += (20.0 * sin(lng * pi) + 40.0 *
                                       sin(lng / 3.0 * pi)) *
           2.0 / 3.0;
    ret += (150.0 * sin(lng / 12.0 * pi) + 300.0 *
                                               sin(lng / 30.0 * pi)) *
           2.0 / 3.0;
    return ret;
}

void gcj2Towgs(double lng, double lat, double &lng_wgs, double &lat_wgs){
    if(out_of_china(lng, lat)){
        lng_wgs = lng;
        lat_wgs = lat;
        return;
    }

    double dlat = transform_lat(lng - 105.0, lat - 35.0);
    double dlng = transform_lng(lng - 105.0, lat - 35.0);
    double radlat = lat / 180.0 * pi;
    double magic = sin(radlat);
    magic = 1 - krasowski_e2 * magic * magic;
    double sqrtmagic = sqrt(magic);
    dlat = (dlat * 180.0) / ((krasowski_a * (1 - krasowski_e2)) / (magic * sqrtmagic) * pi);
    dlng = (dlng * 180.0) / (krasowski_a / sqrtmagic * cos(radlat) * pi);
    double mglat = lat + dlat;
    double mglng = lng + dlng;

    lng_wgs = lng * 2 - mglng;
    lat_wgs = lat * 2 - mglat;

    return;
}

void wgsTogcj2(double lon, double lat, double &lng_gcj2, double &lat_gcj2){
    double dLat = transform_lat(lon - 105.0, lat - 35.0);
    double dLon = transform_lng(lon - 105.0, lat - 35.0);
    double radLat = lat / 180.0 * pi;
    double magic = sin(radLat);
    magic = 1 - krasowski_e2 * magic * magic;
    double sqrtMagic = sqrt(magic);
    dLat = (dLat * 180.0) / ((krasowski_a * (1 - krasowski_e2)) / (magic * sqrtMagic) * pi);
    dLon = (dLon * 180.0) / (krasowski_a / sqrtMagic * cos(radLat) * pi);
    lat_gcj2 = lat + dLat;
    lng_gcj2 = lon + dLon;
    return;
}


// WGS-84 转换为 GCJ-02
 void wgs84_to_gcj02(double &lng, double &lat) {
    if (out_of_china( lng, lat)) {
        return; // 如果坐标不在中国境内，直接返回
    }

    double transformLatx = lng - 105.0;
    double transformLaty = lat - 35.0;

    double transformLat = -100.0 + 2.0 * transformLatx + 3.0 * transformLaty + 0.2 * transformLaty * transformLaty +
                          0.1 * transformLatx * transformLaty + 0.2 * std::sqrt(std::fabs(transformLatx));
    transformLat += (20.0 * std::sin(6.0 * transformLatx * PI) + 20.0 * std::sin(2.0 * transformLatx * PI)) * 2.0 / 3.0;
    transformLat += (20.0 * std::sin(transformLaty * PI) + 40.0 * std::sin(transformLaty / 3.0 * PI)) * 2.0 / 3.0;
    transformLat += (160.0 * std::sin(transformLaty / 12.0 * PI) + 320.0 * std::sin(transformLaty * PI / 30.0)) * 2.0 / 3.0;

    double transformLon = 300.0 + transformLatx + 2.0 * transformLaty + 0.1 * transformLatx * transformLatx +
                          0.1 * transformLatx * transformLaty + 0.1 * std::sqrt(std::fabs(transformLatx));
    transformLon += (20.0 * std::sin(6.0 * transformLatx * PI) + 20.0 * std::sin(2.0 * transformLatx * PI)) * 2.0 / 3.0;
    transformLon += (20.0 * std::sin(transformLatx * PI) + 40.0 * std::sin(transformLatx / 3.0 * PI)) * 2.0 / 3.0;
    transformLon += (150.0 * std::sin(transformLatx / 12.0 * PI) + 300.0 * std::sin(transformLatx / 30.0 * PI)) * 2.0 / 3.0;

    double radLat = lat / 180.0 * PI;
    double magic = std::sin(radLat);
    magic = 1 - EE * magic * magic;
    double sqrtMagic = std::sqrt(magic);

    double dLat = (transformLat * 180.0) / ((A * (1 - EE)) / (magic * sqrtMagic) * PI);
    double dLon = (transformLon * 180.0) / (A / sqrtMagic * std::cos(radLat) * PI);

    lat += dLat;
    lng += dLon;
}

// WGS-84 转换为 GCJ-02
// inline int wgs84_to_gcj02(Eigen::Vector3d &wgs, Eigen::Vector3d &gcj) {
//     double lng = wgs.x();
//     double lat = wgs.y();

//     wgs84_to_gcj02(lng, lat);

//     gcj.x() = lng;
//     gcj.y() = lat;
//     gcj.z() = wgs.z();

//     return 0;
// }


double cal_line_min_distance(const linestring_t& l1, const linestring_t& l2)
{
    return bg::distance(l1, l2);
}

// 计算两条曲线的最近点对
std::pair<point_t, point_t> find_closest_points(const linestring_t& curve1, const linestring_t& curve2) {
    double min_distance = std::numeric_limits<double>::max();
    point_t closest_point1, closest_point2;
    int curve1_index = -1, curve2_index = -1;

    for (size_t i = 0; i < curve1.size(); i++) {
        const auto& pt1 = curve1[i];
        for (size_t j = 0; j < curve2.size(); j++) {
            const auto& pt2 = curve2[j];
            double distance = bg::distance(pt1, pt2);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point1 = pt1;
                closest_point2 = pt2;

                curve1_index = i;
                curve2_index = j;
            }
        }
    }

    return {closest_point1, closest_point2};
}

std::pair<double, point_t> find_closest_points(const point_t& pt_in, const linestring_t& curve1) {
    double min_distance = std::numeric_limits<double>::max();
    point_t closest_point1;

    for (size_t i = 0; i < curve1.size(); i++) {
        const auto& pt1 = curve1[i];
        double distance = bg::distance(pt_in, pt1);
        if (distance < min_distance) {
            min_distance = distance;
            closest_point1 = pt1;
        }
    }

    return {min_distance, closest_point1};
}

std::pair<int, double> find_closest_points_index(const point_t& pt_in, const linestring_t& curve1) {
    double min_distance = std::numeric_limits<double>::max();
    int curve1_index = -1;

    for (size_t i = 0; i < curve1.size(); i++) {
        const auto& pt1 = curve1[i];
        double distance = bg::distance(pt_in, pt1);
        if (distance < min_distance) {
            min_distance = distance;
            curve1_index = i;
        }
    }

    return {min_distance, curve1_index};
}

std::pair<int, int> find_closest_points_index(const linestring_t& curve1, const linestring_t& curve2) {
    double min_distance = std::numeric_limits<double>::max();
    point_t closest_point1, closest_point2;
    int curve1_index = -1, curve2_index = -1;

    for (size_t i = 0; i < curve1.size(); i++) {
        const auto& pt1 = curve1[i];
        for (size_t j = 0; j < curve2.size(); j++) {
            const auto& pt2 = curve2[j];
            double distance = bg::distance(pt1, pt2);
            if (distance < min_distance) {
                min_distance = distance;
                closest_point1 = pt1;
                closest_point2 = pt2;

                curve1_index = i;
                curve2_index = j;
            }
        }
    }

    return {curve1_index, curve2_index};
}


void generate_virtual_center_line(const linestring_t& l1, const linestring_t& l2,
                                linestring_t& closest_points, linestring_t& center_line)
{

    // using ClosestPointStrategy = bg::strategy::distance::projected_point<point_t, linestring_t>;

    int l1_size = l1.size();
    for (int i = 0; i < l1_size; i++) {
        // 如果需要找到具体的最近点，可以使用 Boost.Geometry 的策略
        // point_t closest_point;
        // bg::distance(l1[i], l2, ClosestPointStrategy(), closest_point);
        // closest_points.push_back(closest_point);
        std::pair<double, point_t> distance_and_closest_point = find_closest_points(l1[i], l2);
        point_t closest_point = distance_and_closest_point.second;
        closest_points.push_back(closest_point);

        point_t mid_point((l1[i].x() + closest_point.x())/ 2.0, (l1[i].y() + closest_point.y())/ 2.0);
        center_line.push_back(mid_point); // 距离太远了是不是不增加了？
    }
}

void generate_virtual_center_line2(const linestring_t& l1, const linestring_t& l2,
                                linestring_t& closest_points, linestring_t& center_line)
{
    // 计算每个点的垂线方向
    std::vector<Eigen::Vector3d> l2_dirs(l2.size());
    for (int i = 1; i < l2.size(); ++i) {
        Eigen::Vector3d pre(l2[i-1].x(), l2[i-1].y(), 0);
        Eigen::Vector3d cur(l2[i].x(), l2[i].y(), 0);
        auto dir = alg::get_dir(cur, pre);
        l2_dirs.push_back(dir);
        if(i == 1) {
            l2_dirs[i-1] = dir;
        }
    }

    closest_points.clear();
    for (int i = 0; i < l2.size(); ++i) {
        Eigen::Vector3d extended_from(l2[i].x(), l2[i].y(), 0);
        Eigen::Vector3d extended_to = alg::get_vertical_pos(extended_from, l2_dirs[i], 20, true); //计算dir垂线方向上离pos距离是50m的点 
        Eigen::Vector3d extended_to2 = alg::get_vertical_pos(extended_from, -l2_dirs[i], 20, true); //计算dir垂线方向上离pos距离是50m的点 
        fsdmap::alg::linestring_t ray_line2{alg::point_t(extended_to2.x(), extended_to2.y()), alg::point_t(extended_to.x(), extended_to.y())};

        fsdmap::alg::points_t intersect_points2;
        bool is_intersect = fsdmap::alg::bg::intersection(l1, ray_line2, intersect_points2);
        if(intersect_points2.size() > 0){
            closest_points.push_back(l2[i]);

            point_t mid_point((l2[i].x() + intersect_points2[0].x())/ 2.0, (l2[i].y() + intersect_points2[0].y())/ 2.0);
            center_line.push_back(mid_point); // 距离太远了是不是不增加了？
        }
    }
}

bool generate_virtual_center_line3(const linestring_t& l1, const linestring_t& l2,
                                double& distance, Eigen::Vector2d& optimize_T)
{
    distance = FLT_MAX;
    // 计算每个点的垂线方向
    if(l2.size()<2) {
        return false;
    }

    std::vector<Eigen::Vector3d> l2_dirs(l2.size());
    for (int i = 1; i < l2.size(); ++i) {
        Eigen::Vector3d pre(l2[i-1].x(), l2[i-1].y(), 0);
        Eigen::Vector3d cur(l2[i].x(), l2[i].y(), 0);
        auto dir = alg::get_dir(cur, pre);
        l2_dirs.push_back(dir);
        if(i == 1) {
            l2_dirs[i-1] = dir;
        }
    }

    int valid_cnt = 0;
    for (int i = 0; i < l2.size(); ++i) {
        Eigen::Vector3d extended_from(l2[i].x(), l2[i].y(), 0);
        Eigen::Vector3d extended_to = alg::get_vertical_pos(extended_from, l2_dirs[i], 20, true); //计算dir垂线方向上离pos距离是50m的点 
        Eigen::Vector3d extended_to2 = alg::get_vertical_pos(extended_from, -l2_dirs[i], 20, true); //计算dir垂线方向上离pos距离是50m的点 
        fsdmap::alg::linestring_t ray_line2{alg::point_t(extended_to2.x(), extended_to2.y()), alg::point_t(extended_to.x(), extended_to.y())};

        fsdmap::alg::points_t intersect_points2;
        bool is_intersect = fsdmap::alg::bg::intersection(l1, ray_line2, intersect_points2);
        if(intersect_points2.size() > 0){
            auto diff = Eigen::Vector2d((intersect_points2[0].x()-l2[i].x())/2, (intersect_points2[0].y()-l2[i].y())/2);
            if(diff.norm()<10) {
                valid_cnt++;
                optimize_T += diff;
            }
        }
    }
    if (valid_cnt > 0) {
        optimize_T /= valid_cnt;
        distance = optimize_T.norm();
        return true;
    }

    return false;
}

void lines_overlap_info(const linestring_t& l1, const linestring_t& l2,
                        const std::vector<Eigen::Vector3d>& l1_dir, const std::vector<Eigen::Vector3d>& l2_dir,
                        OverlapInfoLineString& overlap_info, double thr)
{
    if (l1.size() <= 0 || l2.size() <= 0) {
        return;
    }
    
    double d = alg::cal_line_min_distance(l1, l2);  
    if (d > 0.5) { // 距离大于1m不处理
    // if (d > 4) { // 距离大于4m不处理 (连接相聚4米的线)
        return;
    }

    Eigen::Vector3d l1_avg_dir;
    l1_avg_dir.setZero();
    int l1_size = l1.size();
    for (int i = 0; i < l1_size; i++) {
        double d = bg::distance(l1[i], l2);
        if (d < thr) {
            // if(l1.size()==326) {
            //     LOG_ERROR("thr:{}, d1: {}, {} {}", thr, d, l1[i].x(), l1[i].y());
            // }
            overlap_info.l1.push_back(l1[i]);
            overlap_info.l1_index.push_back(i);
            l1_avg_dir += l1_dir[i];
        }
    }
    overlap_info.l1_length = bg::length(l1);

    Eigen::Vector3d l2_avg_dir;
    l2_avg_dir.setZero();
    int l2_size = l2.size();
    for (int i = 0; i < l2_size; i++) {
        double d = bg::distance(l2[i], l1);
        if (d < thr) {
            // if(l2.size()==166) {
            //     LOG_ERROR("thr:{}, d2: {}, {} {}", thr, d, l2[i].x(), l2[i].y());
            // }
            overlap_info.l2.push_back(l2[i]);
            overlap_info.l2_index.push_back(i);
            l2_avg_dir += l2_dir[i];
        }
    }
    overlap_info.l2_length = bg::length(l2);

    if (overlap_info.l1.size() > 0 && overlap_info.l2.size() > 0) {
        overlap_info.is_overlap_num = std::min(overlap_info.l1.size(), overlap_info.l2.size());
        l1_avg_dir /= overlap_info.l1.size();
        l2_avg_dir /= overlap_info.l2.size();
        double avg_angle_diff = calc_theta1(l1_avg_dir, l2_avg_dir, true, true); 

        // case1: 有重叠区域， 以下两种情况，说明是同向的
        // 情况1：l1线段的最后一个点在l2线段的里面 && l2线段的起点在l1线段的里面;
        // 情况2：l2线段的最后一个点在l1线段的里面 && l1线段的起点在l2线段的里面;
        // 情况3：l1线段全在在l2线段的里面;
        // 情况4：l2线段全在在l1线段的里面;
        if (avg_angle_diff < 10 && 
            ((overlap_info.l1_index.back() == l1_size - 1 && overlap_info.l2_index.front() == 0)
            || (overlap_info.l1_index.front() == 0 && overlap_info.l2_index.back() == l2_size - 1)
            || (overlap_info.l1_index.front() == 0 && overlap_info.l1_index.back() == l1_size - 1) 
            || (overlap_info.l2_index.front() == 0 && overlap_info.l2_index.back() == l2_size - 1)
            )) {
            overlap_info.is_overlap = true;
            // LOG_INFO("[lane_center] 1 is_overlap : {}, is_nearby: {}, avg_angle_diff:{}, l1_avg_dir:{} {}, l1.size:{}, l2.size:{}, l1.front:{}, l1.back:{}, l2.front:{}, l2.back:{}",
            //                 overlap_info.is_overlap, overlap_info.is_nearby, avg_angle_diff, l1_avg_dir.x(), l1_avg_dir.y(),
            //                 l1.size(), l2.size(),
            //                 overlap_info.l1_index.front(), overlap_info.l1_index.back(), 
            //                 overlap_info.l2_index.front(), overlap_info.l2_index.back());

        // case2: 有重叠区域， 以下两种情况，说明是反向的
        // 情况1：l1线段的最后一个点在l2线段的里面 && l2线段的最后一个也在l1线段的里面;
        // 情况2：l2线段的起点在l1线段的里面 && l1线段的起点在l2线段的里面;
        // 情况3：l1线段全在在l2线段的里面;
        // 情况4：l2线段全在在l1线段的里面;
        } else if ((180 - avg_angle_diff < 10) &&
            ((overlap_info.l1_index.back() == l1_size - 1 && overlap_info.l2_index.back() == l2_size - 1)
            || (overlap_info.l1_index.front() == 0 && overlap_info.l2_index.front() == 0)
            || (overlap_info.l1_index.front() == 0 && overlap_info.l1_index.back() == l1_size - 1) 
            || (overlap_info.l2_index.front() == 0 && overlap_info.l2_index.back() == l2_size - 1)
            ))
        {
            if (overlap_info.l1_length < overlap_info.l2_length) {
                overlap_info.is_need_reverse_l1= true;
            } else {
                overlap_info.is_need_reverse_l2= true;
            }
            overlap_info.is_overlap = true;
        // case3: 不同朝向的两条线部分重叠（例如直行和右转，或 分合流的地方）
        } else if (avg_angle_diff < 10 &&  overlap_info.is_overlap_num >= 5) {
            // // 情况1：l1线段的最后一个点不在l2线段的里面（但l1离末端点的点数小于3） && l2线段的起点在l1线段的里面;
            // std::pair<int, double> index_and_distance = find_closest_points_index(l1[l1_size - 1], l2);
            // Eigen::Vector3d dir1 = l1_dir[l1_size - 1];
            // Eigen::Vector3d dir2 = l2_dir[index_and_distance.first];
            // double angle = calc_theta1(dir1, dir2, true, true);
            // double d = index_and_distance.second;
            // bool check_case1 = (overlap_info.l1_index.back() >= l1_size - 3 && overlap_info.l2_index.front() == 0 && angle < 5 && d < 0.5);

            // // 情况2：l2线段的最后一个点在l1线段的里面 && l1线段的起点不在l2线段的里面（但l1离起点的点数小于3）;
            // index_and_distance = find_closest_points_index(l1[0], l2);
            // dir1 = l1_dir[0];
            // dir2 = l2_dir[index_and_distance.first];
            // angle = calc_theta1(dir1, dir2, true, true);
            // d = index_and_distance.second;
            // bool check_case2 = (overlap_info.l1_index.front() <= 2 && overlap_info.l2_index.back() == l2_size - 1 && angle < 5 && d < 0.5);

            // // 情况3：l2线段的最后一个点不在l1线段的里面（但l2离末端点的点数小于3） && l1线段的起点在l2线段的里面;
            // index_and_distance = find_closest_points_index(l2[l2_size - 1], l1);
            // dir1 = l1_dir[index_and_distance.first];
            // dir2 = l2_dir[l2_size - 1];
            // angle = calc_theta1(dir1, dir2, true, true);
            // d = index_and_distance.second;
            // bool check_case3 = (overlap_info.l1_index.front() == 0 && overlap_info.l2_index.back() >= l2_size - 3 && angle < 5 && d < 0.5);

            // // 情况4：l1线段的最后一个点在l2线段的里面 && l2线段的起点不在l1线段的里面（但l2离起点的点数小于3）;
            // index_and_distance = find_closest_points_index(l2[0], l1);
            // dir1 = l1_dir[index_and_distance.first];
            // dir2 = l2_dir[0];
            // angle = calc_theta1(dir1, dir2, true, true);
            // d = index_and_distance.second;
            // bool check_case4 = (overlap_info.l1_index.back() == l1_size - 1 && overlap_info.l2_index.front() <= 2 && angle < 5 && d < 0.5);

            // if (check_case1 || check_case2 || check_case3 || check_case4) {
            //     overlap_info.is_overlap = true;

            //     LOG_INFO("[lane_center] 3.1 check_case : {}, {}, {}, {}",
            //             check_case1, check_case2, check_case3, check_case4);
            // }
        }

    } else {
        // std::pair<point_t, point_t> closest_points = find_closest_points(l1, l2);
        std::pair<int, int> closest_points_index = find_closest_points_index(l1, l2);
        int l1_index = closest_points_index.first;
        int l2_index = closest_points_index.second;
        if ((l1_index == l1_size - 1 || l1_index == 0) && (l2_index == l2_size - 1 || l2_index == 0)) {
            const auto& l1_dir_tmp = l1_dir[l1_index];
            const auto& l2_dir_tmp = l2_dir[l2_index];
            double avg_angle_diff = calc_theta1(l1_avg_dir, l2_avg_dir, true, true); 
            // TODO：qzc 计算一个端点再另外一个端点+方向上的垂直距离 < 0.5 米
            if (avg_angle_diff < 5) {
                overlap_info.is_nearby = true;
            } else if (180 - avg_angle_diff < 5) {
                if (overlap_info.l1_length < overlap_info.l2_length) {
                    overlap_info.is_need_reverse_l1= true;
                } else {
                    overlap_info.is_need_reverse_l2= true;
                }
                
                overlap_info.is_nearby = true;
            }
        }
    }
}


void reverse_matched_info(const OverlapInfoLineString& in, OverlapInfoLineString& out)
{
    out.l1_id = in.l2_id;
    out.l2_id = in.l1_id;

    out.l1 = in.l2;
    out.l2 = in.l1;

    out.l1_length = in.l2_length;
    out.l2_length = in.l1_length;
    out.l1_index = in.l2_index;
    out.l2_index = in.l1_index;

    out.is_need_reverse_l1 = in.is_need_reverse_l2;
    out.is_need_reverse_l2 = in.is_need_reverse_l1;

    out.is_overlap_num = in.is_overlap_num;
    out.is_overlap = in.is_overlap;
    out.is_nearby = in.is_nearby;
}

bool match_any_with_forms(const std::unordered_set<uint64_t>& in, std::vector<uint64_t> check_form)
{
    for(auto& form : check_form) {
        if (in.count(form) > 0) {
           return true;
        }
    }

    return false;
}

bool match_any_except_forms(const std::unordered_set<uint64_t>& in, std::vector<uint64_t> check_form)
{
    for(auto& form : check_form) {
        if (in.count(form) > 0) {
           return false;
        }
    }

    return true;
}

}
}
