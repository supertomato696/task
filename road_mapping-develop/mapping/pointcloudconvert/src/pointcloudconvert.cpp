#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <cfloat>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <sstream>
#include <string>
#include <vector>
// #include "../wgs84_to_mars/include/Coord.h"
#include "Coord.h"
#include "util.h"
// #include <proj_api.h>
// #include <proj.h>


using Vec3d = Eigen::Vector3d;



struct EIGEN_ALIGN16 MyColorPointType
{
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        PCL_ADD_INTENSITY;
        uint8_t cloud_bev_label;
        uint32_t cloud_bev_label_shape;
        uint8_t cloud_bev_label_color;    
        uint8_t cloud_bev_label_1;
        uint32_t cloud_bev_label_1_shape;
        uint8_t cloud_bev_label_1_color;
        uint8_t cloud_bev_label_2;
        uint32_t cloud_bev_label_2_shape;
        uint8_t cloud_bev_label_2_color;
        uint8_t cloud_bev_center_line;
        uint32_t cloud_bev_center_line_shape;
        uint8_t cloud_bev_center_line_color;
        // uint8_t cloud_bev_traffic_light;
        // uint8_t cloud_bev_traffic_light_shape;
        // uint8_t cloud_bev_traffic_light_color;
        uint8_t cloud_bev_label_score;
        uint8_t cloud_bev_center_line_score;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
};


POINT_CLOUD_REGISTER_POINT_STRUCT(MyColorPointType, // 注册点类型宏 XYZRGBI + "GPSTime" (as fields)
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint32_t, cloud_bev_label_shape, cloud_bev_label_shape)
                                  (uint8_t, cloud_bev_label_color, cloud_bev_label_color)
                                  (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
                                  (uint32_t, cloud_bev_label_1_shape, cloud_bev_label_1_shape)
                                  (uint8_t, cloud_bev_label_1_color, cloud_bev_label_1_color)
                                  (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
                                  (uint32_t, cloud_bev_label_2_shape, cloud_bev_label_2_shape)
                                  (uint8_t, cloud_bev_label_2_color, cloud_bev_label_2_color)
                                  (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
                                  (uint32_t, cloud_bev_center_line_shape, cloud_bev_center_line_shape)
                                  (uint8_t, cloud_bev_center_line_color, cloud_bev_center_line_color)
                                //   (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
                                //   (uint8_t, cloud_bev_traffic_light_shape, cloud_bev_traffic_light_shape)
                                //   (uint8_t, cloud_bev_traffic_light_color, cloud_bev_traffic_light_color)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
                                  (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))



void ECEF2LonLatAlt(const Vec3d& xyz, Vec3d& lon_lat_alt) {
    double X = xyz[0];
    double Y = xyz[1];
    double Z = xyz[2];

    double a = 6378137.0;          // earth semimajor axis in meters
    double f = 1 / 298.257223563;  // reciprocal flattening
    double b = a * (1.0 - f);      // semi-minor axis

    double e2 = 2.0 * f - pow(f, 2.0);               // first eccentricity squared
    double ep2 = f * (2.0 - f) / pow((1.0 - f), 2);  // second eccentricity squared

    double r2 = pow(X, 2) + pow(Y, 2);
    double r = sqrt(r2);
    double E2 = pow(a, 2) - pow(b, 2);
    double F = 54 * pow(b, 2) * pow(Z, 2);
    double G = r2 + (1.0 - e2) * pow(Z, 2) - e2 * E2;
    double c = (e2 * e2 * F * r2) / (G * G * G);
    double s = pow((1.0 + c + sqrt(c * c + 2 * c)), 1.0 / 3.0);
    double P = F / (3.0 * pow((s + 1.0 / s + 1), 2) * G * G);
    double Q = sqrt(1 + 2 * e2 * e2 * P);
    double ro = -(e2 * P * r) / (1.0 + Q) +
                sqrt((a * a / 2.0) * (1 + 1.0 / Q) - ((1.0 - e2) * P * pow(Z, 2)) / (Q * (1.0 + Q)) - P * r2 / 2.0);
    double tmp = pow((r - e2 * ro), 2);
    double U = sqrt(tmp + pow(Z, 2));
    double V = sqrt(tmp + (1.0 - e2) * pow(Z, 2));
    double zo = (pow(b, 2) * Z) / (a * V);

    double h = U * (1.0 - pow(b, 2) / (a * V));
    double phi = atan((Z + ep2 * zo) / r);
    double lam = atan2(Y, X);

    lon_lat_alt[0] = 180.0 / M_PI * lam;
    lon_lat_alt[1] = 180.0 / M_PI * phi;
    lon_lat_alt[2] = h;

    return;
}


void LonLatAlt2ECEF(const Vec3d& lon_lat_alt, Vec3d& xyz) {
    double SEMI_MAJOR_AXIS = 6378137.0;
    double FIRST_ECCENTRICITY_SQUARED = 6.69437999014e-3;

    double lon = lon_lat_alt[0];
    double lat = lon_lat_alt[1];
    double alt = lon_lat_alt[2];

    if (!std::isnan(lon_lat_alt[2])) alt = lon_lat_alt[2];

    double rad_lon = lon / 180.0 * M_PI;
    double rad_lat = lat / 180.0 * M_PI;

    double sin_lon = sin(rad_lon);
    double cos_lon = cos(rad_lon);
    double sin_lat = sin(rad_lat);
    double cos_lat = cos(rad_lat);

    double chi = sqrt(1.0 - FIRST_ECCENTRICITY_SQUARED * sin_lat * sin_lat);
    double N = SEMI_MAJOR_AXIS / chi + alt;

    xyz[0] = N * cos_lat * cos_lon;
    xyz[1] = N * cos_lat * sin_lon;
    xyz[2] = (SEMI_MAJOR_AXIS * (1.0 - FIRST_ECCENTRICITY_SQUARED) / chi + alt) * sin_lat;
    return;
}


// 定义 WGS84 椭球体参数
const double a = 6378137.0;  // 长半轴
const double f = 1 / 298.257223563;  // 扁率
const double e2 = 2 * f - f * f;  // 第一偏心率的平方
const double e4 = e2 * e2;
const double e6 = e2 * e4;
const double k0 = 0.9996;  // 比例因子

// 计算子午线弧长
double meridian_arc_length(double lat_rad) {
    const double A0 = 1 - (e2 / 4) - (3 * e4 / 64) - (5 * e6 / 256);
    const double A2 = (3 / 8.0) * (e2 + (e4 / 4) + (15 * e6 / 128));
    const double A4 = (15 / 256.0) * (e4 + (3 * e6 / 4));
    const double A6 = 35 * e6 / 3072;
    return a * (A0 * lat_rad - A2 * std::sin(2 * lat_rad) + A4 * std::sin(4 * lat_rad) - A6 * std::sin(6 * lat_rad));
}

// 子午线弧长M的计算（需泰勒级数展开）
double calc_M(double lat_rad, double a, double e2) {
    // 复杂积分公式，此处简化为近似多项式
    return a * ((1 - e2/4 - 3*e2*e2/64) * lat_rad 
                - (3*e2/8 + 3*e2*e2/32) * sin(2*lat_rad) 
                + (15*e2*e2/256) * sin(4*lat_rad));
}

// WGS84 经纬高转 UTM 坐标
void wgs84_to_utm(double lat, double lon, double alt, double& easting, double& northing, int& zone, bool& is_northern) {
    // 1. 椭球参数
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double e2 = 2*f - f*f;
    const double k0 = 0.9996;

    // 2. 计算UTM带号和中央经线
    zone = static_cast<int>((lon + 180.0) / 6.0) + 1;
    double central_meridian = (zone * 6.0) - 183.0;

    // 3. 转换为弧度
    double lat_rad = lat * M_PI / 180.0;
    double delta_lon_rad = (lon - central_meridian) * M_PI / 180.0;

    // 4. 计算卯酉圈曲率半径N
    double sin_lat = sin(lat_rad);
    double N = a / sqrt(1 - e2 * sin_lat * sin_lat);

    // 5. 计算泰勒展开参数
    double T = pow(tan(lat_rad), 2);
    double C = e2 * pow(cos(lat_rad), 2);
    double A = delta_lon_rad * cos(lat_rad);

    // 6. 计算东距和北距（简化公式）
    easting = k0 * N * (A + (1 - T + C) * pow(A, 3)/6.0) + 500000.0;
    northing = k0 * (calc_M(lat_rad, a, e2) + N * tan(lat_rad) * 
                          (pow(A, 2)/2.0 + (5.0 - T + 9.0*C + 4.0*C*C)*pow(A, 4)/24.0));

    // 7. 处理南北半球
    bool is_north = (lat >= 0);
    if (!is_north) northing += 10000000.0;  // 南半球北距加10,000,000米
}


bool ConvertXYZ2LonLatAlt(const Vec3d& gps_origin_, const Vec3d& ecef_origin_, const Vec3d& xyz, Vec3d& lon_lat_alt) {

    if (std::isnan(xyz.x()) || std::isnan(xyz.y()) || std::isnan(xyz.z())) {
        std::cout << "XYZ value is NAN!" << std::endl;
        return false;
    }

    // // Vec3d gps_origin_;
    // // Vec3d ecef_origin_;
    // LonLatAlt2ECEF(gps_origin_, ecef_origin_);

    double rad_lon = gps_origin_[0] / 180.0 * M_PI;
    double rad_lat = gps_origin_[1] / 180.0 * M_PI;
    double sin_lon = sin(rad_lon);
    double cos_lon = cos(rad_lon);
    double sin_lat = sin(rad_lat);
    double cos_lat = cos(rad_lat);

    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();

    // clang-format off
    rot << -sin_lon, cos_lon, 0,
           -sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat,
            cos_lat * cos_lon, cos_lat * sin_lon, sin_lat;
    // clang-format on

    Vec3d diff_ecef = rot.inverse() * xyz;
    Vec3d gps_ecef = diff_ecef + ecef_origin_;
    ECEF2LonLatAlt(gps_ecef, lon_lat_alt);
    return true;
}


std::vector<double> parseDoubleArray(const std::string& input) {
    std::vector<double> result;
    std::stringstream ss(input);
    std::string token;

    // 按_分割字符串
    while (std::getline(ss, token, '_')) {
        try {
            // 将分割后的字符串转换为double类型
            double num = std::stod(token);
            result.push_back(num);
        } catch (const std::invalid_argument& e) {
            std::cerr << "无效的double值: " << token << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "double值超出范围: " << token << std::endl;
        }
    }
    return result;
}


int main(int argc, char* argv[]) {

    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 wgs84点云 文件路径", true, "");
    arg_parser.add<std::string>("output", '\0', "输出 utm点云 文件路径", true, "");
    arg_parser.add<std::string>("wgs84_lla", '\0', "输入 wgs84原点坐标", true, "");
    arg_parser.add<std::string>("utm_num", '\0', "输入 utm带号", true, "");
    arg_parser.add<std::string>("utm_lla", '\0', "输入 utm原点坐标", true, "");
    arg_parser.parse_check(argc, argv);

    std::string input_path = arg_parser.get<std::string>("input");
    std::string output_path = arg_parser.get<std::string>("output");
    std::string wgs84_lla = arg_parser.get<std::string>("wgs84_lla");
    int utm_num = std::stoi(arg_parser.get<std::string>("utm_num"));
    std::string utm_lla = arg_parser.get<std::string>("utm_lla");

    // std::string input_path = "/mnt/e/01_code/02_dataset/0315_Blidar/0315_result_centor_11m/layer_pointcloud_semantic.pcd";
    // std::string output_path;
    // std::string wgs84_lla;
    // int utm_num = 50;
    // std::string utm_lla;

    std::cout<<input_path<<' '<<output_path<<' '<<wgs84_lla<<' '<<utm_lla<<' '<<std::endl;

    std::vector<double> wgs84_lla_vec = parseDoubleArray(wgs84_lla);
    std::vector<double> utm_lla_vec = parseDoubleArray(utm_lla);
    std::cout<<wgs84_lla_vec[0]<<' '<<wgs84_lla_vec[1]<<' '<<utm_lla_vec[0]<<' '<<utm_lla_vec[1]<<' '<<std::endl;

    Vec3d gps_origin_ = {wgs84_lla_vec[0], wgs84_lla_vec[1], wgs84_lla_vec[2]};
    // Vec3d gps_origin_ = {114.30609336252927, 22.688912452751534, 0};
    Vec3d ecef_origin_ = {0, 0, 0};
    LonLatAlt2ECEF(gps_origin_, ecef_origin_);
    // std::cout<<"ecef - xyz: "<<ecef_origin_[0]<<", "<<ecef_origin_[1]<<", "<<ecef_origin_[2]<<std::endl;

    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    // if (std::filesystem::exists(input_path)) {
    //     // std::cout << "文件存在" << std::endl;
    // } else {
    //     std::cout << "文件不存在" << std::endl;
    //     return 0;
    // }

    if (pcl::io::loadPCDFile(input_path, *pc_ptr) == -1) {
        std::cerr << "[road_boundary] Error: Unable to load PCD file: " << input_path << std::endl;
        return 0;
    }

    pcl::PointCloud<MyColorPointType>::Ptr trans_cloud(new pcl::PointCloud<MyColorPointType>);
    pcl::copyPointCloud(*pc_ptr, *trans_cloud);

    for (size_t i = 0; i < pc_ptr->points.size(); i++) {
        // 1、将site下的相对坐标，转为WGS84下的绝对位置坐标。
        // std::cout<<"local - xyz: "<<pc_ptr->points[i].x<<", "<<pc_ptr->points[i].y<<", "<<pc_ptr->points[i].z<<std::endl;
        Vec3d local_p = {pc_ptr->points[i].x, pc_ptr->points[i].y, pc_ptr->points[i].z};
        Vec3d wgs84_lon_lat_alt;
        if (!ConvertXYZ2LonLatAlt(gps_origin_, ecef_origin_, local_p, wgs84_lon_lat_alt)) {
            continue;
        }
        // std::cout<<"wgs84 - xyz: " << std::fixed << std::setprecision(8)<<wgs84_lon_lat_alt[0]<<", "<<wgs84_lon_lat_alt[1]<<", "<<wgs84_lon_lat_alt[2]<<std::endl;

        // 将WGS84坐标，转化为GCJ02坐标(国策局的库)
        double lng = 0.0;
        double lat = 0.0;
        double* GCJ02_lng;
        double* GCJ02_lat;
        GCJ02_lng = &lng;
        GCJ02_lat = &lat;
        wgtochina_lb(0, wgs84_lon_lat_alt(0), wgs84_lon_lat_alt(1), 0, 0, 0, GCJ02_lng, GCJ02_lat);
        // std::cout<<"gcj02 - xyz: " << std::fixed << std::setprecision(8)<<*GCJ02_lng<<", "<<*GCJ02_lat<<std::endl;
        
        Eigen::Vector3d llh(*GCJ02_lng,*GCJ02_lat, wgs84_lon_lat_alt[2]);
        Eigen::Vector3d t_utm_world(utm_lla_vec[0], utm_lla_vec[1], 0);
        // Eigen::Vector3d t_utm_world(223223.615556846, 2511594.20419213, 0);
        
        double utm_x, utm_y;
        bool is_north;
        wgs84_to_utm(llh[1], llh[0], llh[2], utm_x, utm_y, utm_num, is_north);
        Eigen::Vector3d xyz = {utm_x, utm_y, 0};
        Eigen::Vector3d xyz_offset = xyz - t_utm_world;

        // std::cout<<"utm (global) - xyz: " << std::fixed << std::setprecision(8)<<xyz[0]<<", "<<xyz[1]<<", "<<xyz[2]<<std::endl;
        // std::cout<<"utm (local) - xyz: " << std::fixed << std::setprecision(8)<<xyz_offset[0]<<", "<<xyz_offset[1]<<", "<<xyz_offset[2]<<std::endl;

        MyColorPointType gcj02_p;
        trans_cloud->points[i].x = xyz_offset[0];
        trans_cloud->points[i].y = xyz_offset[1];
        trans_cloud->points[i].z = xyz_offset[2];
    }

    // pcl::io::savePCDFileBinary("/mnt/e/01_code/02_dataset/0315_Blidar/0315_result_centor_11m/global_cloud.pcd", *trans_cloud);
    pcl::io::savePCDFileBinary(output_path, *trans_cloud);
}