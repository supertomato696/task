



#if 1
#include "util.h"
#include "pclFilter.h"

#include <iostream>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/segment.hpp>
#include <boost/version.hpp>

namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<double> point_type;
typedef bg::model::linestring<point_type> linestring_type;
typedef bg::model::multi_point<point_type> multi_point_type;
typedef bg::model::segment<point_type> segment_type;



int main(int argc, char **argv)
{

    std::cout << "boost version: " << BOOST_LIB_VERSION << std::endl;

    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 pcd path", false, "");
    // arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    std::string input_pcd_path = arg_parser.get<std::string>("input");

    std::string base_dir("/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0224/new_10lukou2/mmt_rc/batch-10013/model/auto_label/output/bev_mapbuild_out/debug_data/");
    std::string pcd_file("010_merge_feature.pcd");
    std::string file_path(base_dir + pcd_file);
    if(input_pcd_path != ""){
        file_path = input_pcd_path+".pcd";
    }
    std::cout << "process: " << file_path << std::endl;

    // 加载点云
    pcl::PointCloud<PointLabel>::Ptr origin_cloud(new pcl::PointCloud<PointLabel>());
    if (pcl::io::loadPCDFile<PointLabel>(file_path, *origin_cloud) == -1)
    {
        std::cerr << "Failed to load point cloud." << std::endl;
        return -1;
    }

    // 点云过滤
    pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary(new pcl::PointCloud<PointLabel>);
    pclFilter::ConditionalRemoval(origin_cloud, filtercloud_boundary, "label", 1);
    pcl::io::savePCDFileBinary(base_dir + "010_debug_boundary.pcd", *filtercloud_boundary);

    pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary1(new pcl::PointCloud<PointLabel>);
    pclFilter::ConditionalRemoval(filtercloud_boundary, filtercloud_boundary1, "opt_label", 1);
    pcl::io::savePCDFileBinary(base_dir + "010_debug_boundary1.pcd", *filtercloud_boundary1);

    pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary53(new pcl::PointCloud<PointLabel>);
    pclFilter::ConditionalRemoval(filtercloud_boundary, filtercloud_boundary53, "opt_label", 53);
    pcl::io::savePCDFileBinary(base_dir + "010_debug_boundary53.pcd", *filtercloud_boundary53);

    /************************************************************************************** */
    linestring_type curve1, curve2;
    for (int i = 0; i < filtercloud_boundary1->size(); i++) {
        auto& p = filtercloud_boundary1->points[i];
        curve1.push_back(point_type(p.x, p.y));
    }
    
    for (int i = 0; i < filtercloud_boundary53->size(); i++) {
        auto& p = filtercloud_boundary53->points[i];
        curve2.push_back(point_type(p.x, p.y));
    }

    linestring_type curve_overlap;
    for (int i = 0; i < curve1.size(); i++) {
        double d = bg::distance(curve1[i], curve2);
        if (d < 0.05) {
            curve_overlap.push_back(curve1[i]);
        }
    }

    if (curve_overlap.size() > 0) {
        std::cout  
                << " curve_overlap.size: " <<  curve_overlap.size()
                << " curve1.length: " <<  bg::length(curve1)
                << " curve2.length: " <<  bg::length(curve2)
                << " curve_overlap.length:" << bg::length(curve_overlap) << std::endl;
    }



    {
        pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary_intersections(new pcl::PointCloud<PointLabel>);
    
        for (const auto& pt : curve_overlap) {
            // std::cout << "Intersection point: (" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ")" << std::endl;
            PointLabel p;
            p.x = bg::get<0>(pt);
            p.y = bg::get<1>(pt);
            p.z = 0;
            p.label = 100;
            filtercloud_boundary_intersections->push_back(p);
        }
        pcl::io::savePCDFileASCII(base_dir + "010_debug_boundary_intersections1.pcd", *filtercloud_boundary_intersections);
    }

    /************************************************************************************** */
    linestring_type curve3, curve4;
    curve3.push_back(point_type(-81.8539, 10.51546));
    curve3.push_back(point_type(-82.8539, 11.51546));

    curve4.push_back(point_type(-81.8539, 3.51546));
    curve4.push_back(point_type(-83.8539, 3.51546));

    segment_type closest_segment;
    // bg::closest_points(curve3, curve4, closest_segment);
    
    // // 输出最近点对
    // std::cout << "Closest points between the two curves:" << std::endl;
    // std::cout << "Point 1: (" << bg::get<0>(closest_segment.first) << ", " << bg::get<1>(closest_segment.first) << ")" << std::endl;
    // std::cout << "Point 2: (" << bg::get<0>(closest_segment.second) << ", " << bg::get<1>(closest_segment.second) << ")" << std::endl;


    // 使用自定义策略判断两条曲线是否相交
    double intersects3 = bg::distance(curve1, curve3);
    std::cout << "The two curves intersect: intersects3: " << intersects3 << std::endl;
    if (intersects3 < 5) {
        pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary_intersections(new pcl::PointCloud<PointLabel>);
        for (const auto& pt : curve3) {
            std::cout << "Intersection point: (" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ")" << std::endl;
            PointLabel p;
            p.x = bg::get<0>(pt);
            p.y = bg::get<1>(pt);
            p.z = 0;
            p.label = 103;
            filtercloud_boundary_intersections->push_back(p);
        }
        pcl::io::savePCDFileASCII(base_dir + "010_debug_boundary_intersections3.pcd", *filtercloud_boundary_intersections);

    } else {
        std::cout << "The two curves do not intersect." << std::endl;
    }

    double intersects4 = bg::distance(curve1, curve4);
    std::cout << "The two curves intersect: intersects4: " << intersects4 << std::endl;
    if (intersects4 < 5) {
        pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary_intersections(new pcl::PointCloud<PointLabel>);
        for (const auto& pt : curve4) {
            std::cout << "Intersection point: (" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ")" << std::endl;
            PointLabel p;
            p.x = bg::get<0>(pt);
            p.y = bg::get<1>(pt);
            p.z = 0;
            p.label = 103;
            filtercloud_boundary_intersections->push_back(p);
        }
        pcl::io::savePCDFileASCII(base_dir + "010_debug_boundary_intersections4.pcd", *filtercloud_boundary_intersections);

    } else {
        std::cout << "The two curves do not intersect." << std::endl;
    }

    /************************************************************************************** */
    // 存储交点
    multi_point_type intersection_points;

    // 判断曲线段是否相交，并获取交点
    if (bg::intersection(curve1, curve2, intersection_points)) {
        std::cout << "The two curves intersect at the following points:" << std::endl;

        pcl::PointCloud<PointLabel>::Ptr filtercloud_boundary_intersections(new pcl::PointCloud<PointLabel>);
        for (const auto& pt : intersection_points) {
            std::cout << "Intersection point: (" << bg::get<0>(pt) << ", " << bg::get<1>(pt) << ")" << std::endl;
            PointLabel p;
            p.x = bg::get<0>(pt);
            p.y = bg::get<1>(pt);
            p.z = 0;
            p.label = 100;
            filtercloud_boundary_intersections->push_back(p);
        }
        pcl::io::savePCDFileASCII(base_dir + "010_debug_boundary_intersections2.pcd", *filtercloud_boundary_intersections);
    } else {
        std::cout << "The two curves do not intersect." << std::endl;

        // 计算端点之间的距离
        double startDistance = bg::distance(curve1.front(), curve2.front());
        double endDistance = bg::distance(curve1.back(), curve2.back());

        std::cout << "Distance between start points: " << startDistance << std::endl;
        std::cout << "Distance between end points: " << endDistance << std::endl;
    }


    return 0;
}


#else 

#include "skeleton_cluster.hpp"
#include "pclFilter.h"
#include "util.h"

using namespace RoadMapping;
double g_x_min = -1.0, g_x_max = 10.0; // X轴范围
double g_y_min = -5.0, g_y_max = 5.0;  // Y轴范围
double resolution = 0.05;              // 分辨率

// 定义BEV图像生成函数
pcl::PointCloud<pcl::PointXYZ>::Ptr BEV_GEN(const std::string &file_path, cv::Mat &BEV_IMG)
{
    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *point_cloud) == -1)
    {
        std::cerr << "Failed to load point cloud." << std::endl;
        return nullptr;
    }

    // 定义BEV图像的参数
    int x_bins = static_cast<int>((g_x_max - g_x_min) / resolution);
    int y_bins = static_cast<int>((g_y_max - g_y_min) / resolution);

    // 初始化BEV图像
    BEV_IMG = cv::Mat::zeros(y_bins, x_bins, CV_8UC1);

    // 遍历点云中的每个点，并将其投影到BEV图像上
    for (const auto &point : point_cloud->points)
    {
        int x_index = static_cast<int>((point.x - g_x_min) / resolution);
        int y_index = static_cast<int>((point.y - g_y_min) / resolution);

        if (x_index >= 0 && x_index < x_bins && y_index >= 0 && y_index < y_bins)
        {
            BEV_IMG.at<uint8_t>(y_bins - y_index - 1, x_index) += 1; // 翻转Y轴方向
        }
    }

    // 归一化BEV图像
    cv::normalize(BEV_IMG, BEV_IMG, 0, 255, cv::NORM_MINMAX, CV_8UC1);

    return point_cloud;
}

// 提取骨骼线
cv::Mat extractSkeleton(const cv::Mat &input)
{
    cv::Mat binary;
    cv::threshold(input, binary, 1, 255, cv::THRESH_BINARY);
    cv::imwrite("binary.png", binary);

    cv::Mat skeleton = binary.clone();
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(2, 2));

    bool done;
    do
    {
        cv::Mat eroded;
        cv::erode(binary, eroded, element); // 腐蚀
        cv::Mat temp;
        cv::dilate(eroded, temp, element); // 膨胀
        cv::subtract(binary, temp, temp);  // 噪声点
        cv::bitwise_or(skeleton, temp, skeleton);
        // cv::bitwise_and(skeleton, temp, skeleton);
        eroded.copyTo(binary);

        done = (cv::countNonZero(binary) == 0);
    } while (!done);

    return skeleton;
}

cv::Mat extractSkeleton2(const cv::Mat &input)
{
    cv::Mat binary;
    cv::threshold(input, binary, 1, 255, cv::THRESH_BINARY);
    cv::imwrite("binary.png", binary);

    // 形态学操作
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
    cv::medianBlur(binary, binary, 5);
    cv::dilate(binary, binary, kernel);
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);
    cv::imwrite("binary2.png", binary);

    // 提取骨骼线
    cv::Mat skeleton;
    cv::ximgproc::thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);

    return skeleton;
}

#if 0
// 提取端点和交叉点
void getPoints(const cv::Mat& skeleton, std::vector<cv::Point>& endPoints, std::vector<cv::Point>& junctionPoints) {
    cv::Mat padImg;
    cv::copyMakeBorder(skeleton, padImg, 1, 1, 1, 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    for (int y = 1; y < padImg.rows - 1; ++y) {
        for (int x = 1; x < padImg.cols - 1; ++x) {
            if (padImg.at<uint8_t>(y, x) == 255) {
                int neighbors = 0;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (padImg.at<uint8_t>(y + dy, x + dx) == 255) {
                            neighbors++;
                        }
                    }
                }
                if (neighbors == 2) { // 端点
                    // endPoints.push_back(cv::Point(x - 1, y - 1));
                    endPoints.push_back(cv::Point(y - 1, x - 1));
                } else if (neighbors > 3) { // 交叉点
                    // junctionPoints.push_back(cv::Point(x - 1, y - 1));
                    junctionPoints.push_back(cv::Point(y - 1, x - 1));
                }
            }
        }
    }
}

#endif

int SkeletonGetEndPt(const cv::Mat &skeletonImg, std::vector<cv::Point> &endPoints)
{
    cv::Mat zerOneImg, padImg;
    endPoints.clear();
    cv::copyMakeBorder(skeletonImg, padImg, 1, 1, 1, 1, CV_8UC1, cv::Scalar(0));
    cv::threshold(padImg, zerOneImg, 0, 1, cv::THRESH_BINARY);
#pragma region kernerl
    // http ://www.imagemagick.org/Usage/morphology/#linejunctions
    // 1表示图像中的（白色）前景结构，-1表示（黑色）背景结构，0表示忽略位置。
    cv::Mat kernel1 = (cv::Mat_<int>(3, 3) << -1, -1, 0, -1, 1, 1, -1, -1, 0);
    cv::Mat kernel2 = (cv::Mat_<int>(3, 3) << -1, -1, -1, -1, 1, -1, 0, 1, 0);
    cv::Mat kernel3 = (cv::Mat_<int>(3, 3) << 0, -1, -1, 1, 1, -1, 0, -1, -1);
    cv::Mat kernel4 = (cv::Mat_<int>(3, 3) << 0, 1, 0, -1, 1, -1, -1, -1, -1);
    cv::Mat kernel5 = (cv::Mat_<int>(3, 3) << -1, -1, -1, -1, 1, -1, -1, -1, 1);
    cv::Mat kernel6 = (cv::Mat_<int>(3, 3) << -1, -1, -1, -1, 1, -1, 1, -1, -1);
    cv::Mat kernel7 = (cv::Mat_<int>(3, 3) << 1, -1, -1, -1, 1, -1, -1, -1, -1);
    cv::Mat kernel8 = (cv::Mat_<int>(3, 3) << -1, -1, 1, -1, 1, -1, -1, -1, -1);
#pragma endregion
    cv::Mat endPointsImg, hitmisImg;
    endPointsImg.create(padImg.size(), CV_8UC1);
    endPointsImg.setTo(0);
    std::vector<cv::Mat> kerners{kernel1, kernel2, kernel3, kernel4, kernel5, kernel6, kernel7, kernel8};
    for (const cv::Mat &ker : kerners)
    {
        cv::morphologyEx(zerOneImg, hitmisImg, cv::MorphTypes::MORPH_HITMISS, ker);
        cv::bitwise_or(hitmisImg, endPointsImg, endPointsImg);
    }
    endPointsImg = endPointsImg * 255;
    for (int r = 0; r < endPointsImg.rows; r++)
    {
        for (int c = 0; c < endPointsImg.cols; c++)
        {
            if (endPointsImg.at<uchar>(r, c) > 0)
            {
                endPoints.emplace_back(c - 1, r - 1);
                // endPoints.emplace_back(r - 1, c - 1);
            }
        }
    }
    return 0;
}

int SkeletonGetJunctionPt(const cv::Mat &biImg, std::vector<cv::Point> &ptPairVec)
{
    cv::Mat zerOneImg, padImg;
    ptPairVec.clear();
    cv::copyMakeBorder(biImg, padImg, 1, 1, 1, 1, CV_8UC1, cv::Scalar(0));
    cv::threshold(padImg, zerOneImg, 0, 1, cv::THRESH_BINARY);
#pragma region kernerl
    cv::Mat kernel1 = (cv::Mat_<uchar>(3, 3) << 1, 0, 1, 0, 1, 0, 0, 1, 0);
    cv::Mat kernel2 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 0, 1, 1, 1, 0, 0);
    cv::Mat kernel3 = (cv::Mat_<uchar>(3, 3) << 0, 0, 1, 1, 1, 0, 0, 0, 1);
    cv::Mat kernel4 = (cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 1, 1, 0, 1, 0);
    cv::Mat kernel5 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 0, 1, 0, 1, 0, 1);
    cv::Mat kernel6 = (cv::Mat_<uchar>(3, 3) << 0, 0, 1, 1, 1, 0, 0, 1, 0);
    cv::Mat kernel7 = (cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 1, 1, 1, 0, 0);
    cv::Mat kernel8 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 0, 0, 0, 1);
    cv::Mat kernel9 = (cv::Mat_<uchar>(3, 3) << 1, 0, 0, 0, 1, 0, 1, 0, 1);
    cv::Mat kernel10 = (cv::Mat_<uchar>(3, 3) << 1, 0, 1, 0, 1, 0, 1, 0, 0);
    cv::Mat kernel11 = (cv::Mat_<uchar>(3, 3) << 1, 0, 1, 0, 1, 0, 0, 0, 1);
    cv::Mat kernel12 = (cv::Mat_<uchar>(3, 3) << 0, 0, 1, 0, 1, 0, 1, 0, 1);
#pragma endregion
    cv::Mat intersecImg, hitmisImg;
    intersecImg.create(padImg.size(), CV_8UC1);
    intersecImg.setTo(0);
    std::vector<cv::Mat> kerners{kernel1, kernel2, kernel3, kernel4, kernel5, kernel6,
                                 kernel7, kernel8, kernel9, kernel10, kernel11, kernel12};
    for (cv::Mat ker : kerners)
    {
        cv::morphologyEx(zerOneImg, hitmisImg, cv::MorphTypes::MORPH_HITMISS, ker);
        cv::bitwise_or(hitmisImg, intersecImg, intersecImg);
    }
    intersecImg = intersecImg * 255;

    for (int r = 0; r < intersecImg.rows; r++)
    {
        for (int c = 0; c < intersecImg.cols; c++)
        {
            if (intersecImg.at<uchar>(r, c) > 0)
            {
                ptPairVec.push_back(cv::Point(c - 1, r - 1));
                // ptPairVec.push_back(cv::Point(r - 1, c - 1));
            }
        }
    }
    return 0;
}

// 为cv::Point定义哈希函数
// namespace std
// {
//     template <>
//     struct hash<cv::Point>
//     {
//         std::size_t operator()(const cv::Point &pt) const
//         {
//             return std::hash<int>()(pt.x) ^ std::hash<int>()(pt.y);
//             // std::size_t hash_x = std::hash<int>()(pt.x);
//             // std::size_t hash_y = std::hash<int>()(pt.y);
//             // return hash_x ^ (hash_y << 1);
//         }
//     };
// } // namespace std

// 分割骨骼线
std::vector<std::vector<cv::Point>> splitSkeleton(const cv::Mat &skeleton,
                                                  const std::vector<cv::Point> &endPoints,
                                                  const std::vector<cv::Point> &junctionPoints)
{
    // 使用 insert 合并 交叉点 和 端点
    std::vector<cv::Point> merged_points;
    merged_points.insert(merged_points.end(), endPoints.begin(), endPoints.end());
    merged_points.insert(merged_points.end(), junctionPoints.begin(), junctionPoints.end());

    std::vector<std::vector<cv::Point>> segments;
    std::unordered_map<cv::Point, bool> visited;

    std::unordered_map<cv::Point, bool> merged;
    for (const auto &p : merged_points)
    {
        merged[p] = true;
    }

    auto isVisited = [&](const cv::Point &pt)
    { return visited.find(pt) != visited.end(); };

    auto markVisited = [&](const cv::Point &pt)
    { visited[pt] = true; };

    auto isMerged = [&](const cv::Point &pt)
    { return merged.find(pt) != merged.end(); };

    for (const auto &start : merged_points)
    {
        if (!isVisited(start))
        {
            std::vector<cv::Point> segment;
            std::queue<cv::Point> q;
            q.push(start);
            markVisited(start);

            while (!q.empty())
            {
                cv::Point current = q.front();
                q.pop();
                segment.push_back(current);

                std::vector<cv::Point> neighbors = {{current.x - 1, current.y - 1}, {current.x, current.y - 1}, {current.x + 1, current.y - 1}, {current.x - 1, current.y}, {current.x + 1, current.y}, {current.x - 1, current.y + 1}, {current.x, current.y + 1}, {current.x + 1, current.y + 1}};

                for (const auto &neighbor : neighbors)
                {
                    // std::cout << neighbor << " value: " << int(skeleton.at<uint8_t>(neighbor))
                    //           << ", is visisited: " << isVisited(neighbor) << std::endl;
                    // 1. 为车道线 2. 还未访问 3. 非端点或交叉点
                    if (skeleton.at<uint8_t>(neighbor) == 255 && !isVisited(neighbor) && !isMerged(neighbor))
                    {
                        markVisited(neighbor);
                        q.push(neighbor);
                    }
                }
            }
            segments.push_back(segment);
        }
    }

    return segments;
}

cv::Scalar GetCvColor(float size, float index)
{
    // 使用 HSV 颜色空间生成彩虹颜色
    float hue = (float)index / size * 180; // 180 代表 HSV 色调的范围
    float saturation = 1.0f;
    float value = 1.0f;

    // 转换 HSV 到 BGR
    cv::Scalar color = cv::Scalar::all(0); // 初始化 BGR 颜色
    cv::Mat hsv(1, 1, CV_8UC3, cv::Scalar(hue, saturation * 255, value * 255));
    cv::cvtColor(hsv, hsv, cv::COLOR_HSV2BGR); // 转换为 BGR 色彩空间
    color = hsv.at<cv::Vec3b>(0, 0);

    return color;
}

int main2()
{
    // {
    //     // 创建一个unordered_map来存储访问状态
    //     std::unordered_map<cv::Point, bool> visited;
    //     // 示例点
    //     cv::Point pt1(10, 20);
    //     cv::Point pt2(10, 20);
    //     cv::Point pt3(30, 40);
    //     // 标记点为已访问
    //     visited[pt1] = true;
    //     // 检查点是否已访问
    //     assert(visited[pt1] == true);   // pt1应该被访问
    //     assert(visited[pt2] == true);   // pt2应该被访问，因为pt2和pt1是相同的点
    //     assert(visited[pt3] == false);  // pt3应该未被访问
    //     std::cout << "All tests passed successfully!" << std::endl;
    // }

    // 定义点云文件路径
    std::string file_path = "/Users/aqiu/Documents/1_study/00_AllMyXX/AllMy4DLabel/lane_point_cloud.pcd";

    // 定义BEV图像
    cv::Mat BEV_IMAGE;

    // step1: 读取pcd 并 生成BEV图像
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud = BEV_GEN(file_path, BEV_IMAGE);

    if (point_cloud)
    {
        // 显示原始BEV图像
        cv::imshow("Original BEV Image", BEV_IMAGE);
        cv::imwrite("origin_bev_image.png", BEV_IMAGE);

        // step2: 进行基础的膨胀和腐蚀操作
        // 定义结构元素
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

        // 膨胀操作
        cv::Mat dilated_image;
        cv::dilate(BEV_IMAGE, dilated_image, kernel, cv::Point(-1, -1), 1);

        // 腐蚀操作
        cv::Mat eroded_image;
        cv::erode(dilated_image, eroded_image, kernel, cv::Point(-1, -1), 1);

        // 显示和保存处理后的BEV图像
        cv::imshow("Dilated BEV Image", dilated_image);
        cv::imshow("Eroded BEV Image", eroded_image);
        cv::imwrite("dilated_bev_image.png", dilated_image);
        cv::imwrite("eroded_bev_image.png", eroded_image);

        // step3: 提取骨骼线
        // cv::Mat skeleton = extractSkeleton(eroded_image);
        cv::Mat skeleton = extractSkeleton2(eroded_image);
        cv::imshow("Skeleton", skeleton);
        cv::imwrite("skeleton.png", skeleton);

        // step4: 提取端点和交叉点
        std::vector<cv::Point> junctionPoints;
        int ret1 = SkeletonGetJunctionPt(skeleton, junctionPoints);
        std::vector<cv::Point> endPoints;
        int ret2 = SkeletonGetEndPt(skeleton, endPoints);
        std::cout << "junction size: " << junctionPoints.size() << ", end size: " << endPoints.size() << std::endl;

        // 创建一个三通道的 RGB 图像，初始化为黑色
        cv::Mat rgbImage(skeleton.size(), CV_8UC3, cv::Scalar(0, 0, 0));

        // 遍历二值图像的每个像素
        for (int i = 0; i < skeleton.rows; i++)
        {
            for (int j = 0; j < skeleton.cols; j++)
            {
                // 获取二值图像中的当前像素值
                uchar pixelValue = skeleton.at<uchar>(i, j);

                // 如果像素值是 255 (白色)，将其转换为红色 (255, 0, 0)
                if (pixelValue == 255)
                {
                    // rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
                    rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255); // 白色
                }
                else
                {
                    rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // 黑色
                }
            }
        }

        for (auto &ptPair : junctionPoints)
        {
            // 修改像素为红色 (BGR模式：Blue, Green, Red)
            // int col = ptPair.x;
            // int row = ptPair.y;
            // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
            rgbImage.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255); // 红色 = (B, G, R)
        }
        cv::imshow("skeleton_junction", rgbImage);
        cv::imwrite("skeleton_junction.png", rgbImage);

        // step5: 根据端点或交叉点, 进行线段分割
        // 如果有一个 Y + I 形的两条线, 估计还是要从端点开始遍历
        // 分割骨骼线
        std::vector<std::vector<cv::Point>> segments = splitSkeleton(skeleton, endPoints, junctionPoints);
        std::cout << "segments size: " << segments.size() << std::endl;
        // 绘制结果
        cv::Mat result = cv::Mat::zeros(skeleton.size(), CV_8UC3);
        int cnt = 0;
        for (const auto &segment : segments)
        {
            std::cout << "segment: " << cnt << " size: " << segment.size() << std::endl;
            auto color = GetCvColor(segments.size(), cnt++);
            for (size_t i = 0; i < segment.size() - 1; ++i)
            {
                cv::line(result, segment[i], segment[i + 1], color, 1);
            }
        }
        cv::imshow("skeleton_segment", result);
        cv::imwrite("skeleton_segment.png", result);

        // step6: 计算交叉点处的线宽(或者用经验值),切割原始的车道线
        // 结果图像
        cv::Mat binary;
        cv::threshold(eroded_image, binary, 1, 255, cv::THRESH_BINARY);

        // 计算距离变换
        cv::Mat dist;
        cv::distanceTransform(binary, dist, cv::DIST_L2, 5);

        // 显示结果
        cv::imwrite("distance_transform.png", dist);
        // cv::normalize(dist, dist, 0, 1, cv::NORM_MINMAX);
        // cv::imshow("distance_transform", dist);

        // step7: 对原始车道线进行分段

        cv::waitKey(0);
    }

    return 0;
}

int main(int argc, char **argv)
{
    ArgParser arg_parser;
    arg_parser.add<std::string>("input", '\0', "输入 pcd path", false, "");
    // arg_parser.add<std::string>("output", '\0', "输出文件夹", true, "");
    arg_parser.parse_check(argc, argv);

    std::string input_pcd_path = arg_parser.get<std::string>("input");

    std::string base_dir("/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0125/new_7lukou2/batch-10013/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/");
    std::string seg_dir("11005307477522/laneBoundaryFromSeg/");
    std::string pcd_file("cluster_0");
    // std::string pcd_file("cluster_17");
    // {
    //     // 加载和保存车道线的点云
    //     std::string pcdPath(base_dir + "../model_pcd/global_cloud.pcd");
    //     pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    //     pcl::io::loadPCDFile(pcdPath, *pc_ptr);

    //     // 条件滤波
    //     pcl::PointCloud<ConRmPointType>::Ptr filtercloud_boundary(new pcl::PointCloud<ConRmPointType>);
    //     pclFilter::ConditionalRemoval(pc_ptr, filtercloud_boundary, "cloud_bev_label_1", 1);

    //     pcl::io::savePCDFileBinary(base_dir + "/model_pcd/global_cloud_lane.pcd", *filtercloud_boundary);
    // }

    // std::string file_path("/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross5/v1.00.25/batch-10010/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/9500026273967/laneBoundaryFromSeg/cluster_1.pcd");
    // std::string file_path("/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross5/v1.00.25/batch-10010/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/9500026273967/roadBoundaryFromSeg/cluster_6.pcd");
    // std::string file_path("/mnt/d/04_dataset/1_dilabel/crowd_source/raw_data_0125/new_7lukou2/batch-10010/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/11005267932214/laneBoundaryFromSeg/cluster_12.pcd");
    std::string file_path(base_dir + seg_dir + pcd_file + ".pcd");
    if(input_pcd_path != ""){
        file_path = input_pcd_path+".pcd";
    }
    std::cout << "process: " << file_path << std::endl;

    // 加载点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *clusterCloud) == -1)
    {
        std::cerr << "Failed to load point cloud." << std::endl;
        return -1;
    }

    std::string output_dir_("");
    std::string cluster_index("0");

    SkeletonCluster skeleton_cluster;
    // 如果需要分割，则进行 skeleton 分割
    std::map<int, int> indexMap;
    skeleton_cluster.SetInput(clusterCloud, output_dir_, cluster_index);
    // if (skeleton_cluster.Process())
    if (skeleton_cluster.ProcessBoundary())
    {
        auto &cluster_results = skeleton_cluster.GetResult();
        for (auto &one_cluster : cluster_results)
        {
            std::cout << " " << one_cluster.size()<< std::endl;
        }
    }

    return 0;
}

#endif