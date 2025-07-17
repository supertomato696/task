#include <Eigen/Geometry>
#include <array>
#include <random>
#include <vector>
#include <thread>
#include <istream>
#include <unordered_set>
#include <map>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <pcl/ModelCoefficients.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/common/impl/common.hpp>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/search/impl/organized.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/radius_outlier_removal.hpp>
#include <pcl/filters/impl/statistical_outlier_removal.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/conditional_removal.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <malloc.h>
#include "include/log.h"
#include "include/util.h"
#include "include/json.h"
#include "include/types.h"
#include <omp.h>

void generateGaussMask(cv::Mat &Mask, cv::Size wsize, double sigma)
{
    Mask.create(wsize, CV_64F);
    int h = wsize.height;
    int w = wsize.width;
    int center_h = (h - 1) / 2;
    int center_w = (w - 1) / 2;
    double sum = 0.0;
    double x, y;
    for (int i = 0; i < h; ++i)
    {
        y = pow(i - center_h, 2);
        for (int j = 0; j < w; ++j)
        {
            x = pow(j - center_w, 2);
            double g = exp(-(x + y) / (2 * sigma * sigma));
            Mask.at<double>(i, j) = g;
            sum += g;
        }
    }
    Mask = Mask / sum;
}

void GaussianFilter(cv::Mat &src, cv::Mat &dst, cv::Mat window, cv::Mat &cnt_mat)
{
    int border = (window.rows - 1) / 2;
    dst = cv::Mat::zeros(src.size(), src.type());
    cv::Mat Newsrc;
    cv::copyMakeBorder(src, Newsrc, border, border, border, border, cv::BORDER_REPLICATE);

    // 高斯滤波
    int finish_r = 0;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
    for (int i = border; i < src.rows + border; ++i)
    {
        finish_r += 1;
        if (finish_r % 1000 == 0)
        {
            LOG_INFO("高斯平滑进度{}/{}", finish_r, src.rows + border);
        }
        for (int j = border; j < src.cols + border; ++j)
        {
            double sum = 0;
            int zero_nums = 0;
            int z_nums = 0;
            float z_mean = 0;

            std::vector<float> z_vector;
            for (int r = -border; r <= border; ++r)
            {
                for (int c = -border; c <= border; ++c)
                {
                    if (Newsrc.at<float>(i + r, j + c) != 0.0)
                    {
                        z_vector.push_back(Newsrc.at<float>(i + r, j + c));
                    }
                }
            }
            std::sort(z_vector.begin(), z_vector.end());
            float z_mid = z_vector.size() > 0 ? z_vector[z_vector.size() / 2] : 10000.0;

            cv::Mat mask = cv::Mat::zeros(window.size(), CV_8UC1);
            for (int r = -border; r <= border; ++r)
            {
                for (int c = -border; c <= border; ++c)
                {
                    float z_Newsrc = Newsrc.at<float>(i + r, j + c);
                    if (z_Newsrc == 0 || std::fabs(z_mid - z_Newsrc) > 2.0)
                    {
                        zero_nums++;
                        mask.at<uchar>(border + r, border + c) = 255;
                    }
                    else
                    {
                        z_nums++;
                        z_mean += z_Newsrc;
                    }
                }
            }
            z_mean = z_mean / z_nums;

            if ((float)zero_nums / (z_nums + zero_nums) > 0.5)
            { // 不参与平滑
                dst.at<float>(i - border, j - border) = src.at<float>(i - border, j - border);
                continue;
            }
            // if ((float)zero_nums / (z_nums + zero_nums) > 0.7)
            // { //过滤路面外的噪点
            //     cnt_mat.at<int>(i - border, j - border) = 0;
            //     continue;
            // }

            for (int r = -border; r <= border; ++r)
            {
                for (int c = -border; c <= border; ++c)
                {
                    if (Newsrc.at<float>(i + r, j + c) == 0)
                    {
                        Newsrc.at<float>(i + r, j + c) = z_mean;
                    }
                }
            }

            // 高斯平滑
            for (int r = -border; r <= border; ++r)
            {
                for (int c = -border; c <= border; ++c)
                {
                    sum = sum + Newsrc.at<float>(i + r, j + c) * window.at<double>(r + border, c + border);
                }
            }

            for (int r = -border; r <= border; ++r)
            {
                for (int c = -border; c <= border; ++c)
                {
                    if (mask.at<uchar>(border + r, border + c) == 255)
                    {
                        Newsrc.at<float>(i + r, j + c) = 0;
                    }
                }
            }
            dst.at<float>(i - border, j - border) = sum;
        }
    }
}

struct Point2DHash
{
    std::size_t operator()(const std::pair<int, int> &p) const
    {
        return (p.first << 20) + p.second;
    }
};

void separate_2_thinner_layer(pcl::PointCloud<MyColorPointType> &input_cloud,
                              pcl::PointCloud<MyColorPointType> &output_cloud_1,
                              pcl::PointCloud<MyColorPointType> &output_cloud_2,
                              MyColorPointType min_p,
                              MyColorPointType max_p)
{
    std::unordered_map<std::pair<int, int>, MyColorPointType, Point2DHash> point_map_1;
    std::unordered_map<std::pair<int, int>, MyColorPointType, Point2DHash> point_map_2;
    std::unordered_map<std::pair<int, int>, MyColorPointType, Point2DHash> layer_1;

    std::sort(input_cloud.begin(), input_cloud.end(),
              [](MyColorPointType pt1, MyColorPointType pt2)
              { return pt1.z > pt2.z; });

    int finish_r = 0;
    for (MyColorPointType &p : input_cloud.points)
    {
        finish_r += 1;
        if (finish_r % 100000 == 0)
        {
            LOG_INFO("划分上下层进度{}/{}", finish_r, input_cloud.points.size());
        }

        int x = round((p.x - min_p.x) / 0.2);
        int y = round((p.y - min_p.y) / 0.2);

        if (layer_1.find(std::make_pair(x, y)) != layer_1.end() && std::fabs(layer_1[std::make_pair(x, y)].z - p.z) > 2.0) // 如果第一层存在，放到第二层
        {
            int _x = round((p.x - min_p.x) / 0.05);
            int _y = round((p.y - min_p.y) / 0.05);
            if (point_map_2.find(std::make_pair(_x, _y)) == point_map_2.end())
            {
                p.x = (_x * 0.05) + min_p.x;
                p.y = (_y * 0.05) + min_p.y;
                p.a = 1; // 用此字段计数
                point_map_2[std::make_pair(_x, _y)] = p;
            }
            else
            {
                auto &one_exist_p = point_map_2[std::make_pair(_x, _y)];
                one_exist_p.z = (one_exist_p.z * one_exist_p.a + p.z) / (one_exist_p.a + 1);
                one_exist_p.a = one_exist_p.a + 1;

                if ((p.intensity - int(p.intensity)) < (one_exist_p.intensity - int(one_exist_p.intensity)))
                {
                    one_exist_p.intensity = p.intensity;
                    one_exist_p.rgba = p.rgba;
                    one_exist_p.label = p.label;
                }
            }
        }
        else
        {
            layer_1[std::make_pair(x, y)] = p;
            int _x = round((p.x - min_p.x) / 0.05);
            int _y = round((p.y - min_p.y) / 0.05);
            if (point_map_1.find(std::make_pair(_x, _y)) == point_map_1.end())
            {
                p.x = (_x * 0.05) + min_p.x;
                p.y = (_y * 0.05) + min_p.y;
                p.a = 1; // 用此字段计数
                point_map_1[std::make_pair(_x, _y)] = p;
            }
            else if (std::fabs(point_map_1[std::make_pair(_x, _y)].z - p.z) < 2.0) // 如果第一层存在，且高差<2.0m，更新第一层
            {
                auto &one_exist_p = point_map_1[std::make_pair(_x, _y)];
                one_exist_p.z = (one_exist_p.z * one_exist_p.a + p.z) / (one_exist_p.a + 1);
                one_exist_p.a = one_exist_p.a + 1;

                if ((p.intensity - int(p.intensity)) < (one_exist_p.intensity - int(one_exist_p.intensity)))
                {
                    one_exist_p.intensity = p.intensity;
                    one_exist_p.rgba = p.rgba;
                    one_exist_p.label = p.label;
                }
            }
        }
    }

    for (auto &[k, p] : point_map_1)
    {
        output_cloud_1.push_back(p);
    }
    for (auto &[k, p] : point_map_2)
    {
        output_cloud_2.push_back(p);
    }
}

void smooth_ground(pcl::PointCloud<MyColorPointType> &input_cloud,
                   MyColorPointType min_p,
                   MyColorPointType max_p)
{
    int width = (max_p.x - min_p.x) / 0.05 + 1;
    int height = (max_p.y - min_p.y) / 0.05 + 1;
    cv::Mat cnt_mat = cv::Mat::zeros(height, width, CV_32SC1);
    cv::Mat z_mat = cv::Mat::zeros(height, width, CV_32FC1);
    for (MyColorPointType p : input_cloud.points)
    {
        int x = round((p.x - min_p.x) / 0.05);
        int y = round((p.y - min_p.y) / 0.05);
        cnt_mat.at<int>(y, x) = 1;
        z_mat.at<float>(y, x) = p.z;
    }

    // 自定义高斯滤波
    cv::Mat Mask;
    int ksize = 19;
    float sigma = 0.3 * ((ksize - 1) * 0.5 - 1) + 0.8;
    generateGaussMask(Mask, cv::Size(ksize, ksize), 4 * sigma); // 获取二维高斯滤波模板

    cv::Mat smooth_z_mat;
    LOG_INFO("do GaussianFilter");
    GaussianFilter(z_mat, smooth_z_mat, Mask, cnt_mat);
    LOG_INFO("finish GaussianFilter");

    for (MyColorPointType &p : input_cloud.points)
    {
        int x = round((p.x - min_p.x) / 0.05);
        int y = round((p.y - min_p.y) / 0.05);
        p.z = smooth_z_mat.at<float>(y, x);
    }
}

struct Point3DHash
{
    std::size_t operator()(const std::tuple<int, int, int> &p) const
    {
        return (std::get<2>(p) << 40) + (std::get<1>(p) << 20) + std::get<0>(p);
    }
};

// todo: z方向是不是可以设置成10cm，加快处理
void smooth_ground_by_neighbour(pcl::PointCloud<MyColorPointType> &input_cloud,
                                pcl::PointCloud<MyColorPointType> &output_cloud_1,
                                MyColorPointType min_p,
                                MyColorPointType max_p)
{
    std::unordered_map<std::tuple<int, int, int>, MyColorPointType, Point3DHash> point_map;
    std::unordered_map<std::tuple<int, int, int>, MyColorPointType, Point3DHash> point_map_2;

    int finish_r = 0;
    for (MyColorPointType &p : input_cloud.points)
    {
        finish_r += 1;
        if (finish_r % 100000 == 0)
        {
            LOG_INFO("计算点云索引对进度{}/{}", finish_r, input_cloud.points.size());
        }

        int x = round((p.x - min_p.x) / 0.05);
        int y = round((p.y - min_p.y) / 0.05);
        int z = round((p.z - min_p.z) / 0.05);

        if (point_map.find(std::make_tuple(x, y, z)) == point_map.end())
        {
            p.x = (x * 0.05) + min_p.x;
            p.y = (y * 0.05) + min_p.y;
            p.z = (z * 0.05) + min_p.z;
            p.a = 1; // 用此字段计数
            point_map[std::make_tuple(x, y, z)] = p;
        }
        else
        {
            auto &one_exist_p = point_map[std::make_tuple(x, y, z)];
            one_exist_p.z = (one_exist_p.z * one_exist_p.a + p.z) / (one_exist_p.a + 1);
            one_exist_p.a = one_exist_p.a + 1;

            if ((p.intensity - int(p.intensity)) < (one_exist_p.intensity - int(one_exist_p.intensity)))
            {
                one_exist_p.intensity = p.intensity;
                one_exist_p.rgba = p.rgba;
                one_exist_p.label = p.label;
            }
        }
    }
    point_map_2 = point_map;

    int finish_p = 0;
    std::mutex mtx;
#pragma omp parallel for num_threads(int(std::thread::hardware_concurrency() * 0.8))
    for (size_t b = 0; b < point_map.bucket_count(); b++)
    {
        for (auto iter = point_map.begin(b); iter != point_map.end(b); iter++)
        {
            finish_p += 1;
            if (finish_p % 100000 == 0)
            {
                LOG_INFO("点云平滑进度{}/{}", finish_p, point_map.size());
            }
            float mean_z = 0;
            int count_z = 0;
            std::tuple<int, int, int> c_key = iter->first;
            MyColorPointType p = iter->second;
            int c_x, c_y, c_z;
            c_x = std::get<0>(c_key);
            c_y = std::get<1>(c_key);
            c_z = std::get<2>(c_key);

            for (int _i = c_x - 5; _i <= c_x + 5; _i++)
            {
                for (int _j = c_y - 5; _j <= c_y + 5; _j++)
                {
                    for (int _k = c_z - 3; _k <= c_z + 3; _k++)
                    {
                        if (point_map_2.find(std::make_tuple(_i, _j, _k)) != point_map_2.end())
                        {
                            count_z++;
                            mean_z += point_map_2[std::make_tuple(_i, _j, _k)].z;
                        }
                    }
                }
            }
            mean_z /= count_z;
            p.z = mean_z;
            std::lock_guard<std::mutex> lck(mtx);
            output_cloud_1.push_back(p);
        }
    }
}

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 pcd path", true, "");
    arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    std::string input_pcd_path = arg_parser.get<std::string>("input");
    std::string output_folder = arg_parser.get<std::string>("output");

    // spdlog
    auto logger = Logger::instance();
    logger->setLogLevel(spdlog::level::level_enum::info,
                        spdlog::level::level_enum::trace);
    std::string logFile = output_folder + "/log/" + _time_str_s() + "_ground_thinner.log";
    logger->setup(logFile, "ground_thinner");

    LOG_INFO("============================================");
    LOG_INFO("  Hello ground thinner");
    LOG_INFO("  input_pcd_path:{}", input_pcd_path);
    LOG_INFO("  output_folder:{}", output_folder);
    LOG_INFO("============================================");

    pcl::PointCloud<MyColorPointType>::Ptr raw_ground_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(input_pcd_path, *raw_ground_ptr);
    MyColorPointType min_p, max_p;
    pcl::getMinMax3D(*raw_ground_ptr, min_p, max_p);
    min_p.x = int(min_p.x - 2);
    min_p.y = int(min_p.y - 2);
    max_p.x = int(max_p.x + 2);
    max_p.y = int(max_p.y + 2);

    //老方法
    pcl::PointCloud<MyColorPointType>::Ptr thinner_ground_1_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::PointCloud<MyColorPointType>::Ptr thinner_ground_2_ptr(new pcl::PointCloud<MyColorPointType>);
    log_info("do separate_2_thinner_layer");
    separate_2_thinner_layer(*raw_ground_ptr, *thinner_ground_1_ptr, *thinner_ground_2_ptr, min_p, max_p);

    if (thinner_ground_2_ptr->points.size() > 2000) // 如果2层点足够多，则对其变薄处理
    {
        log_info("do smooth_ground 2");
        smooth_ground(*thinner_ground_2_ptr, min_p, max_p);
    }
    log_info("do smooth_ground 1");
    smooth_ground(*thinner_ground_1_ptr, min_p, max_p);
    log_info("finish smooth_ground");
    pcl::PointCloud<MyColorPointType>::Ptr thinner_ground_ptr(new pcl::PointCloud<MyColorPointType>);
    *thinner_ground_ptr = *thinner_ground_1_ptr + *thinner_ground_2_ptr;
    //----------------------------
    
    
    //新方法
    // pcl::PointCloud<MyColorPointType>::Ptr thinner_ground_ptr(new pcl::PointCloud<MyColorPointType>);
    // smooth_ground_by_neighbour(*raw_ground_ptr, *thinner_ground_ptr, min_p, max_p);
    //-------------------------


    if (!thinner_ground_ptr->empty())
    {
        thinner_ground_ptr->points[0].intensity = 255.0;
        pcl::io::savePCDFileBinaryCompressed(path_join(output_folder, "thinner_ground.pcd"), *thinner_ground_ptr);
    }
    else
    {
        LOG_ERROR("thinner_ground_ptr empty()");
    }

    LOG_INFO("  Bye ground thinner");
    return 1;
}