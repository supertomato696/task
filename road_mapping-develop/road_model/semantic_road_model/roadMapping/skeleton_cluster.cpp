#include "skeleton_cluster.hpp"
#include <filesystem>
#include <stack>
#include <cmath>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;
namespace RoadMapping
{

    SkeletonCluster::SkeletonCluster()
    {
        Clear();
    }

    SkeletonCluster::~SkeletonCluster()
    {
        Clear();
    }

    void SkeletonCluster::SetInput(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, std::string output_dir, std::string cluster_index) {
        output_dir_ = output_dir;
        cluster_index_ = cluster_index;
        // std::cout << "cluster_name: " << output_dir_ + "/cluster_" + cluster_index_ + ".pcd" << std::endl;
        Clear();
        clusterCloud_ptr_ = clusterCloud;

        // if (fs::exists(output_dir_+"/debug")) {
        // } else {
        //     if (fs::create_directory(output_dir_+"/debug")) {
        //     }
        // }
    };

    void SkeletonCluster::GenBEV()
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr
        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(*clusterCloud_ptr_, min_p1, max_p1);

        // 定义BEV图像的参数
        // float buffer_ = 0.5;
        // float resolution_ = 0.05;
        g_x_min_ = min_p1.x - buffer_;
        g_x_max_ = max_p1.x + buffer_;
        g_y_min_ = min_p1.y - buffer_;
        g_y_max_ = max_p1.y + buffer_;
        x_bins_ = static_cast<int>((g_x_max_ - g_x_min_) / resolution_);
        y_bins_ = static_cast<int>((g_y_max_ - g_y_min_) / resolution_);

        // 初始化BEV图像
        bev_img_ = cv::Mat::zeros(y_bins_, x_bins_, CV_8UC1);
        bev_img_z_ = cv::Mat::zeros(y_bins_, x_bins_, CV_32F);

        // 遍历点云中的每个点，并将其投影到BEV图像上
        int cloud_index = 0;
        for (const auto &point : clusterCloud_ptr_->points)
        {
            // TODO:qzc use position_xy_to_5cm_index_xy
            int x_index = static_cast<int>((point.x - g_x_min_) / resolution_);
            int y_index = static_cast<int>((point.y - g_y_min_) / resolution_);
            // 用于恢复原始的点云顺序
            cv::Point xy_index(y_index, x_index);
            m_xy_index_to_cloud_index_[xy_index] = cloud_index++;

            if (x_index >= 0 && x_index < x_bins_ && y_index >= 0 && y_index < y_bins_)
            {
                // bev_img_.at<uint8_t>(y_index, x_index) += 1;
                bev_img_.at<uint8_t>(y_bins_ - y_index - 1, x_index) += 1; // 翻转Y轴方向

                bev_img_z_.at<float>(y_bins_ - y_index - 1, x_index) = point.z;
            }
        }

        // 归一化BEV图像
        cv::normalize(bev_img_, bev_img_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }

    void SkeletonCluster::GenBEV_V2()
    {
        // pcl::PointCloud<pcl::PointXYZ>::Ptr
        pcl::PointXYZ min_p1, max_p1;
        pcl::getMinMax3D(*clusterCloud_ptr_, min_p1, max_p1);

        auto wlh = Inference::GetWLH(clusterCloud_ptr_);
        std::cout << "wlh: " << wlh.transpose() << std::endl;
        Eigen::Vector3f wlh2(max_p1.x - min_p1.x,  max_p1.y - min_p1.y, max_p1.z - min_p1.z);
        std::cout << "wlh: " <<  wlh2.transpose() << std::endl;

        // 定义BEV图像的参数
        // float buffer_ = 0.5;
        // float resolution_ = 0.05;
        g_x_min_ = min_p1.x - buffer_;
        g_x_max_ = max_p1.x + buffer_;
        g_y_min_ = min_p1.y - buffer_;
        g_y_max_ = max_p1.y + buffer_;
        // x_bins_ = static_cast<int>((g_x_max_ - g_x_min_) / resolution_);
        // y_bins_ = static_cast<int>((g_y_max_ - g_y_min_) / resolution_);

        min_index_xy_ = position_xy_to_5cm_index_xy(g_x_min_, g_y_min_, 50000, 50000);
        max_index_xy_ = position_xy_to_5cm_index_xy(g_x_max_, g_y_max_, 50000, 50000);
        x_bins_ = max_index_xy_.x() - min_index_xy_.x();
        y_bins_ = max_index_xy_.y() - min_index_xy_.y();

        // 初始化BEV图像
        bev_img_ = cv::Mat::zeros(y_bins_, x_bins_, CV_8UC1);
        bev_img_z_ = cv::Mat::zeros(y_bins_, x_bins_, CV_32F);

        // 遍历点云中的每个点，并将其投影到BEV图像上
        int cloud_index = 0;
        for (const auto &point : clusterCloud_ptr_->points)
        {
            // TODO:qzc use position_xy_to_5cm_index_xy
            // int x_index = static_cast<int>((point.x - g_x_min_) / resolution_);
            // int y_index = static_cast<int>((point.y - g_y_min_) / resolution_);
            auto index = position_xy_to_5cm_index_xy(point.x, point.y, 50000, 50000);
            int x_index = index.x() - min_index_xy_.x();
            int y_index = index.y() - min_index_xy_.y();
            // 用于恢复原始的点云顺序
            cv::Point xy_index(y_index, x_index);
            m_xy_index_to_cloud_index_[xy_index] = cloud_index++;

            if (x_index >= 0 && x_index < x_bins_ && y_index >= 0 && y_index < y_bins_)
            {
                // bev_img_.at<uint8_t>(y_index, x_index) += 1;
                bev_img_.at<uint8_t>(y_bins_ - y_index - 1, x_index) += 1; // 翻转Y轴方向

                bev_img_z_.at<float>(y_bins_ - y_index - 1, x_index) = point.z;
            }
        }

        // 归一化BEV图像
        cv::normalize(bev_img_, bev_img_, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    }

    int SkeletonCluster::SkeletonGetEndPt(const cv::Mat &skeletonImg, std::vector<cv::Point> &endPoints)
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

    int SkeletonCluster::SkeletonGetJunctionPt(const cv::Mat &biImg, std::vector<cv::Point> &junctionPoints, std::vector<cv::Point> &endPoints)
    {
        cv::Mat zerOneImg, padImg;
        junctionPoints.clear();
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
                    junctionPoints.push_back(cv::Point(c - 1, r - 1));
                    // junctionPoints.push_back(cv::Point(r - 1, c - 1));
                }
            }
        }
        
        auto F_distance = [&](const cv::Point &pt1, const cv::Point &pt2)
        { return std::max(fabs(pt1.x - pt2.x), fabs(pt1.y - pt2.y)); };

        int junction_size = junctionPoints.size();
        std::vector<bool> unique_one(junction_size, true);
        // case1: 如果存在两个相邻的交叉点，随机去掉一个
        for (int i = 0; i < junction_size; i++)
        {
            for (int j = i+1; j < junction_size; j++)
            {
                if (/*unique_one[j] == true && */F_distance(junctionPoints[i], junctionPoints[j]) == 1)
                {
                    unique_one[j] = false;
                }   
            }
        }
        
        std::vector<cv::Point> junctionPoints_modify;
        for (int i = 0; i < junction_size; i++)
        {
            if (unique_one[i] == true)
            {
                junctionPoints_modify.push_back(junctionPoints[i]);
            }
        }

        junctionPoints.clear();
        junctionPoints = junctionPoints_modify;

        // case2: 如果端点和交叉点相邻，则说明端点只有一个点，可以直接去掉端点
        std::vector<cv::Point> endPoints_modify;
        for (int i = 0; i < endPoints.size(); i++)
        {
            bool need_keep = true;
            for (int j = 0; j < junctionPoints.size(); j++)
            {
                if (F_distance(endPoints[i], junctionPoints[j]) <= 1) {
                    need_keep = false;
                    break;
                }
            }

            if (need_keep)
            {
                endPoints_modify.push_back(endPoints[i]);
            }
        }
        endPoints.clear();
        endPoints = endPoints_modify;


        return 0;
    }

    cv::Mat SkeletonCluster::ExtractSkeleton(const cv::Mat &binary)
    {
        // cv::Mat binary;
        // cv::threshold(input, binary, 1, 255, cv::THRESH_BINARY);
        // cv::imwrite("debug/3_1_binary.png", binary);

        // 形态学操作
        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // cv::medianBlur(binary, binary, 5);
        // cv::dilate(binary, binary, kernel);
        // cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

        // cv::imwrite("debug/3_2_binary2.png", binary);

        // 提取骨骼线
        cv::Mat skeleton;
        cv::ximgproc::thinning(binary, skeleton, cv::ximgproc::THINNING_ZHANGSUEN);

        return skeleton;
    }

    // 分割骨骼线
    std::vector<std::vector<cv::Point>> SkeletonCluster::SplitSkeleton(const cv::Mat &skeleton,
                                                                       const std::vector<cv::Point> &endPoints,
                                                                       const std::vector<cv::Point> &junctionPoints)
    {
        // 使用 insert 合并 交叉点 和 端点
        std::vector<cv::Point> merged_points;
        merged_points.insert(merged_points.end(), endPoints.begin(), endPoints.end());
        merged_points.insert(merged_points.end(), junctionPoints.begin(), junctionPoints.end());

        int end_point_size = endPoints.size();
        int junction_point_size = junctionPoints.size();

        std::vector<std::vector<cv::Point>> segments;
        std::unordered_map<cv::Point, bool> visited;
        for (int r = 0; r < skeleton.rows; r++)
        {
            for (int c = 0; c < skeleton.cols; c++)
            {
                if (skeleton.at<uchar>(r, c) > 0)
                {
                    cv::Point pt(r, c);
                    visited[pt] = false;
                }
            }
        }

        // std::unordered_map<cv::Point, bool> merged;
        // for (const auto &p : merged_points)
        // {
        //     merged[p] = true;
        // }


        std::unordered_map<cv::Point, bool> junctioned;
        for (const auto &p : junctionPoints)
        {
            junctioned[p] = true;
        }

        auto isVisited = [&](const cv::Point &pt)
        { return visited[pt]; };
        
        auto markVisited = [&](const cv::Point &pt)
        { visited[pt] = true; };

        auto markUnVisited = [&](const cv::Point &pt)
        { visited[pt] = false; };

        // auto isMerged = [&](const cv::Point &pt)
        // { return merged.find(pt) != merged.end(); };

        auto isJunction = [&](const cv::Point &pt)
        { return junctioned.find(pt) != junctioned.end(); };

        auto F_distance = [&](const cv::Point &pt1, const cv::Point &pt2)
        { return std::max(fabs(pt1.x - pt2.x), fabs(pt1.y - pt2.y)); };

        int merge_points_size = merged_points.size();
        for (int i = 0; i < merge_points_size; i++)
        // for (const auto &start : endPoints)
        {
            bool should_break = false;
            cv::Point start = merged_points[i];
            if (!isVisited(start))
            {
                std::vector<cv::Point> segment;
                // std::queue<cv::Point> q;
                std::stack<cv::Point> q;
                q.push(start);
                markVisited(start);

                while (!q.empty())
                {
                    // cv::Point current = q.front();
                    cv::Point current = q.top();
                    q.pop();
                    segment.push_back(current);

                    std::vector<cv::Point> neighbors = {{current.x - 1, current.y - 1}, {current.x, current.y - 1}, {current.x + 1, current.y - 1}, {current.x - 1, current.y}, {current.x + 1, current.y}, {current.x - 1, current.y + 1}, {current.x, current.y + 1}, {current.x + 1, current.y + 1}};
                    for (const auto &neighbor : neighbors)
                    {
                        // // 1. 如果本身是交叉点，且领域也是交叉点，则不退出,选择忽略该相邻的交叉点
                        // // 2. 如果本身不是交叉点，领域都是交叉点，和start与领域的F距离都是1， 则不退出,选择忽略该相邻的交叉点
                        // // 3. 
                        // if (!isVisited(neighbor) && 
                        //     isJunction(neighbor) && (
                        //         (isJunction(current) && F_distance(current, neighbor) == 1) || 
                        //         (!isJunction(current) && F_distance(current, neighbor) == 1 && F_distance(current, start) == 1))
                        //     )
                        // {
                        //     continue;
                        // }
                        

                        // 如果周边有交叉点 ，则直接退出
                        if (!isVisited(neighbor) && isJunction(neighbor))
                        {
                            should_break = true;
                            break; // 一级跳
                        }
                    }

                    if (should_break)
                    {
                        // 将stack中未处理的点都复位为未访问状态
                        markUnVisited(start);
                        i = 0; 

                        // 将stack中未处理的点都复位为未访问状态， 从该 start 起点继续遍历
                        while (!q.empty()){
                            // std::cout << "remain size: " << q.size() << std::endl;
                            cv::Point remained = q.top();
                            q.pop();
                            markUnVisited(remained);
                        }
                        
                        break; // 二级跳
                    }

                    for (const auto &neighbor : neighbors)
                    {
                        // std::cout << neighbor << " value: " << int(skeleton.at<uint8_t>(neighbor))
                        //           << ", is visisited: " << isVisited(neighbor) << std::endl;
                        // 1. 为车道线 2. 还未访问 3. 非端点或交叉点
                        // if (skeleton.at<uint8_t>(neighbor) == 255 && !isVisited(neighbor) && !isMerged(neighbor))
                        // {
                        //     markVisited(neighbor);
                        //     q.push(neighbor);
                        // }

                        // // 1. 如果本身是交叉点，且领域也是交叉点，则不退出,选择忽略该相邻的交叉点
                        // // 2. 如果本身不是交叉点，领域都是交叉点，和start与领域的F距离都是1， 则不退出,选择忽略该相邻的交叉点
                        // // 3. 
                        // if (!isVisited(neighbor) && 
                        //     isJunction(neighbor) && (
                        //         (isJunction(current) && F_distance(current, neighbor) == 1) || 
                        //         (!isJunction(current) && F_distance(current, neighbor) == 1 && F_distance(current, start) == 1))
                        //     )
                        // {
                        //     continue;
                        // }

                        if (skeleton.at<uint8_t>(neighbor) == 255 && !isVisited(neighbor))
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

    cv::Scalar SkeletonCluster::GetCvColor(float size, float index)
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

    static cv::Mat bin_img_remove_small_region(cv::Mat &bin_img, int min_area)
    {
        // 输入输出 单通道 前景白色 背景黑色
        cv::Mat out(bin_img.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat stats, centroids, labelImage, imshow_mat;
        int nLabels = cv::connectedComponentsWithStats(
            bin_img, labelImage, stats, centroids, 4);
        cv::Mat mask(labelImage.size(), CV_8UC1, cv::Scalar(0));
        cv::Mat surfSup = stats.col(4) >= min_area;
        int tmp_label;
        for (int i = 1; i < bin_img.rows; i++)
        {
            for (int j = 1; j < bin_img.cols; j++)
            {
                tmp_label = labelImage.at<int>(i, j);
                mask.at<char>(i, j) = (char)surfSup.at<char>(tmp_label, 0);
            }
        }
        bin_img.copyTo(out, mask);
        return out;
    }

    bool SkeletonCluster::Process2()
    {
        // return false;

        // step1: 读取pcd 并 生成BEV图像
        // GenBEV();
        GenBEV_V2();

        // step2: 进行基础的膨胀和腐蚀操作
        // 显示原始BEV图像
        if (save_image_)
        {
            cv::imshow("Original BEV Image", bev_img_);
            cv::imwrite("debug/1_origin_bev_image.png", bev_img_);
        }

        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(1, 1));
        cv::Mat kernel_2x2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
        cv::Mat kernel_3x3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::morphologyEx(bev_img_, bev_img_, cv::MORPH_OPEN, kernel_2x2);
        cv::morphologyEx(bev_img_, bev_img_, cv::MORPH_CLOSE, kernel_2x2);

        // 定义结构元素
        // cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // // 膨胀操作
        // cv::Mat dilated_image;
        // cv::dilate(bev_img_, dilated_image, kernel, cv::Point(-1, -1), 1);
        // // 腐蚀操作
        // cv::Mat eroded_image;
        // cv::erode(dilated_image, eroded_image, kernel, cv::Point(-1, -1), 1);
        // // 显示和保存处理后的BEV图像
        // if (save_image_)
        // {
        //     cv::imshow("Dilated BEV Image", dilated_image);
        //     cv::imshow("Eroded BEV Image", eroded_image);
        //     cv::imwrite("debug/2_1_dilated_bev_image.png", dilated_image);
        //     cv::imwrite("debug/2_2_eroded_bev_image.png", eroded_image);
        // }

        cv::Mat binary;
        cv::threshold(bev_img_, binary, 1, 255, cv::THRESH_BINARY);
        if (save_image_)
        {
            cv::imwrite("debug/2_1_binary.png", binary);
        }

        // cv::Mat invert_binary;
        // cv::bitwise_not(binary, invert_binary);

        // // cv::morphologyEx(invert_binary, invert_binary, cv::MORPH_CLOSE, kernel_3x3);
        // // cv::imwrite("debug/2_3_filter_image.png", invert_binary);

        // int min_area = 20;
        // cv::Mat filter_small_invert_binary = bin_img_remove_small_region(invert_binary, min_area);
        // cv::imwrite("debug/2_2_filter_image.png", filter_small_invert_binary);

        // cv::morphologyEx(filter_small_invert_binary, filter_small_invert_binary, cv::MORPH_CLOSE, kernel_2x2);
        // cv::imwrite("debug/2_3_filter_image.png", filter_small_invert_binary);

        // cv::bitwise_not(filter_small_invert_binary, binary);
        // filter_small_invert_binary = bin_img_remove_small_region(binary, min_area);
        // cv::imwrite("debug/2_4_filter_image.png", binary);

        // cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel_2x2);
        // cv::imwrite("debug/2_5_filter_image.png", binary);

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarcy;
        // cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::findContours(binary, contours, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        // std::cout << " contour.size: " << contours.size() << " hierarcy.size: " << hierarcy.size() << std::endl;

        cv::Mat result;
        if (save_image_)
        {
            cv::cvtColor(binary, result, cv::COLOR_GRAY2BGR);
        }
        for (int i = 0; i < contours.size(); i++)
        {
            // 检查是否有子轮廓（空洞）
            // int next = hierarcy[i][0];
            // int prev = hierarcy[i][1];
            // int childIdx = hierarcy[i][2];
            int parent = hierarcy[i][3];
            if (parent >= 0)
            {
                double area = cv::contourArea(contours[i]);
                cv::RotatedRect min_area_rect = cv::minAreaRect(contours[i]);
                double width = min_area_rect.size.width;
                double height = min_area_rect.size.height;
                // if (area < 20 || (area < 80 && width < 40 && height < 40)) // 40 == 40*0.05 = 2m
                if ((area < 80 && width < 40 && height < 40)) // 40 == 40*0.05 = 2m
                {
                    std::vector<std::vector<cv::Point>> vvContour{contours[i]};
                    cv::fillPoly(binary, vvContour, cv::Scalar(255));
                }
                if (save_image_)
                {
                    cv::putText(result, std::to_string(int(area)), min_area_rect.center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1);
                    cv::drawContours(result, contours, i, cv::Scalar(0, 0, 255), 1);
                }
            } else {
                if (save_image_)
                {
                    cv::drawContours(result, contours, i, cv::Scalar(0, 255, 0), 1);
                }
            }
        }
        
        if (save_image_)
        {
            cv::imwrite("debug/2_11_raw_sub_contour.png", result);
        }

        // 计算距离变换
        cv::Mat dist;
        cv::distanceTransform(binary, dist, cv::DIST_L2, 5);

        // 显示结果
        if (save_image_)
        {
            cv::imwrite("debug/3_1_binary.png", binary);
            cv::imwrite("debug/3_2_distance_transform.png", dist);
        }

        // step3: 提取骨骼线
        // 3.1 第一次提取骨骼线，并统计车道线宽度，并过滤掉一些严重不符合的线段
        cv::Mat skeleton = ExtractSkeleton(binary);

        // 3.2 第二次
        if (save_image_)
        {
            cv::imwrite("debug/3_3_0_binary.png", binary);
            cv::imshow("Skeleton", skeleton);
            cv::imwrite("debug/3_3_skeleton.png", skeleton);
        }

        // step4: 提取端点和交叉点
        std::vector<cv::Point> junctionPoints, endPoints;
        int ret1 = SkeletonGetEndPt(skeleton, endPoints);
        int ret2 = SkeletonGetJunctionPt(skeleton, junctionPoints, endPoints);
        std::cout << "junction size: " << junctionPoints.size() << ", end size: " << endPoints.size() << std::endl;
        if (junctionPoints.size() == 0)
        {
            return false;
        }

        if (save_image_)
        {
            // // 创建一个三通道的 RGB 图像，初始化为黑色
            // cv::Mat rgbImage(skeleton.size(), CV_8UC3, cv::Scalar(0, 0, 0));
            // // 遍历二值图像的每个像素
            // for (int i = 0; i < skeleton.rows; i++) {
            //     for (int j = 0; j < skeleton.cols; j++) {
            //         // 获取二值图像中的当前像素值
            //         uchar pixelValue = skeleton.at<uchar>(i, j);

            //         // 如果像素值是 255 (白色)，将其转换为红色 (255, 0, 0)
            //         if (pixelValue == 255) {
            //             // rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
            //             rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255);  // 白色
            //         }
            //         else {
            //             rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);  // 黑色
            //         }
            //     }
            // }

            // for (auto& ptPair : junctionPoints) {
            //     // 修改像素为红色 (BGR模式：Blue, Green, Red)
            //     // int col = ptPair.x;
            //     // int row = ptPair.y;
            //     // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
            //     rgbImage.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255);  // 红色 = (B, G, R)
            // }
            // cv::imshow("skeleton_junction", rgbImage);
            // cv::imwrite("debug/4_1_skeleton_junction.png", rgbImage);


            // 创建一个三通道的 RGB 图像，初始化为黑色
            cv::Mat rgbImage2(binary.size(), CV_8UC3, cv::Scalar(0, 0, 0));

            // 遍历二值图像的每个像素
            for (int i = 0; i < binary.rows; i++)
            {
                for (int j = 0; j < binary.cols; j++)
                {
                    // 获取二值图像中的当前像素值
                    uchar pixelValue = binary.at<uchar>(i, j);

                    // 如果像素值是 255 (白色)，将其转换为红色 (255, 0, 0)
                    if (pixelValue == 255)
                    {
                        // rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
                        rgbImage2.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255); // 白色
                    }
                    else
                    {
                        rgbImage2.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // 黑色
                    }
                }
            }

            for (auto &ptPair : endPoints)
            {
                // 修改像素为红色 (BGR模式：Blue, Green, Red)
                // int col = ptPair.x;
                // int row = ptPair.y;
                // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
                // rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255);  // 红色 = (B, G, R)
                rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 255, 0); // 红色 = (B, G, R)
            }

            for (auto &ptPair : junctionPoints)
            {
                // 修改像素为红色 (BGR模式：Blue, Green, Red)
                // int col = ptPair.x;
                // int row = ptPair.y;
                // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
                // rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255);  // 红色 = (B, G, R)
                rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(255, 0, 0); // 红色 = (B, G, R)
            }
            cv::imwrite("debug/5_1_binary_junction.png", rgbImage2);
        }

        // step5: 根据端点或交叉点, 进行线段分割
        // 如果有一个 Y + I 形的两条线, 估计还是要从端点开始遍历
        // 分割骨骼线
        std::vector<std::vector<cv::Point>> segments = SplitSkeleton(skeleton, endPoints, junctionPoints);
        std::cout << "segments size: " << segments.size() << std::endl;
        if (save_image_)
        {
            // // 绘制结果
            // cv::Mat result = cv::Mat::zeros(skeleton.size(), CV_8UC3);
            // int cnt = 0;
            // for (const auto& segment : segments) {
            //     std::cout << "segment: " << cnt << " size: " << segment.size() << std::endl;
            //     auto color = GetCvColor(segments.size(), cnt++);
            //     for (size_t i = 0; i < segment.size() - 1; ++i) {
            //         cv::line(result, segment[i], segment[i + 1], color, 1);
            //     }
            // }
            // cv::imshow("skeleton_segment", result);
            // cv::imwrite("debug/5_1_skeleton_segment.png", result);
        }

        // step6: 计算交叉点处的线宽(或者用经验值),切割原始的车道线
        // if (save_image_)

        // 统计车道线的平均宽度
        pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        std::vector<float> half_avg_lane_widths;
        std::vector<cv::Mat> cluster_segments;
        int cnt = 0;
        for (const auto &segment : segments)
        {
            if (segment.size() == 0)
            {
                half_avg_lane_widths.push_back(0.f);
                cnt++;
                continue;
            }

            // 太短的线，直接丢弃
            // if (segment.size() * resolution_ < 2)
            // {
            //     // half_avg_lane_widths.push_back(0.f);
            //     continue;
            // }

            cv::Mat binary_segment = cv::Mat::zeros(binary.size(), CV_8UC1);
            float sum_lane_width = 0;
            for (size_t i = 0; i < segment.size(); ++i)
            {
                sum_lane_width += dist.at<float>(segment[i]);

                binary_segment.at<uint8_t>(segment[i]) = 255;

                // debug
                if (save_image_) {
                    int x_index = segment[i].x;
                    int y_index = y_bins_ - segment[i].y - 1;

                    pcl::PointXYZI p;
                    // p.x = x_index * resolution_ + g_x_min_;
                    // p.y = y_index * resolution_ + g_y_min_;
                    auto xy = index_xy_to_position_xy(x_index + min_index_xy_.x(), y_index + min_index_xy_.y(), 50000, 50000);
                    p.x = xy.x();
                    p.y = xy.y();
                    p.z = bev_img_z_.at<float>(y_index, x_index);
                    p.intensity = cnt;

                    skeleton_cloud->points.push_back(p);
                }
            }

            float half_avg_lane_width = sum_lane_width / segment.size();
            half_avg_lane_widths.push_back(half_avg_lane_width);
            // std::cout << "segment: " << cnt << " size: " << segment.size()
            //           << " sum_lane_width: " << sum_lane_width << " half_avg_lane_width:" << half_avg_lane_width << std::endl;

            if (half_avg_lane_width < 2)
            {
                cnt++;
                continue;
            }

            if (segment.size() <= 5 || (segment.size() * resolution_ < 2 && half_avg_lane_widths[cnt] <= 2))
            {
                cnt++;
                continue;
            }

            // step7: 对原始车道线进行分段
            // 根据线宽进行膨胀
            // 定义结构元素
            int lane_width_int = std::ceil(half_avg_lane_width);
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(lane_width_int, lane_width_int));

            // 膨胀操作
            cv::Mat dilated_binary_segment;
            cv::dilate(binary_segment, dilated_binary_segment, kernel, cv::Point(-1, -1), 1);

            // 查找所有非零点
            cv::Mat nonZeroPoints;
            cv::findNonZero(dilated_binary_segment, nonZeroPoints);
            cluster_segments.push_back(nonZeroPoints);
            // for (int i = 0; i < nonZeroPoints.rows; i++) {
            //     std::cout << nonZeroPoints.at<cv::Point>(i) << std::endl;
            // }

            cnt++;
        }

        {
            // 创建一个三通道的 RGB 图像，初始化为黑色
            cv::Mat rgbImage2(binary.size(), CV_8UC3, cv::Scalar(0, 0, 0));

            // 遍历二值图像的每个像素
            for (int i = 0; i < binary.rows; i++)
            {
                for (int j = 0; j < binary.cols; j++)
                {
                    // 获取二值图像中的当前像素值
                    uchar pixelValue = binary.at<uchar>(i, j);

                    // 如果像素值是 255 (白色)，将其转换为红色 (255, 0, 0)
                    if (pixelValue == 255)
                    {
                        // rgbImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 255);  // 红色
                        rgbImage2.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 255, 255); // 白色
                    }
                    else
                    {
                        rgbImage2.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0); // 黑色
                    }
                }
            }

            int cnt = -1;
            for (const auto &segment : segments)
            {
                cnt++;
                
                // 太短的线，直接丢弃
                // if (segment.size() <= 5)
                // if (segment.size() * resolution_ < 2)
                if (segment.size() <= 5 || (segment.size() * resolution_ < 2 && half_avg_lane_widths[cnt] <= 2))
                {
                    continue;
                }

                // std::cout << "segment: " << cnt << " size: " << segment.size() << " width: " << half_avg_lane_widths[cnt] << std::endl;
                auto color = GetCvColor(segments.size(), cnt);
                for (size_t i = 0; i < segment.size() - 1; ++i)
                {
                    cv::line(rgbImage2, segment[i], segment[i + 1], color, 1);
                    if (i == 0)
                    {
                        cv::putText(rgbImage2, std::to_string(cnt), segment[i], cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1);
                    }
                    
                }
            }

            for (auto &ptPair : endPoints)
            {
                // 修改像素为红色 (BGR模式：Blue, Green, Red)
                // int col = ptPair.x;
                // int row = ptPair.y;
                // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
                // rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255);  // 红色 = (B, G, R)
                rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 255, 0); // 红色 = (B, G, R)
            }

            for (auto &ptPair : junctionPoints)
            {
                // 修改像素为红色 (BGR模式：Blue, Green, Red)
                // int col = ptPair.x;
                // int row = ptPair.y;
                // 直接用 cv::Point的话, 默认会调换顺序为<point.y, point.x>
                // rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(0, 0, 255);  // 红色 = (B, G, R)
                rgbImage2.at<cv::Vec3b>(ptPair) = cv::Vec3b(255, 0, 0); // 红色 = (B, G, R)
            }
            // std::cout << " " << output_dir_ + "/cluster_"+cluster_index_+".png" << std::endl;
            cv::imwrite(output_dir_ + "/cluster_"+cluster_index_+".png", rgbImage2);
        }

        // step8: 转换为点云格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr skeleton_cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>());
        cnt = 0;
        for (auto &segment : cluster_segments)
        {
            std::vector<int> cluster_index;
            for (int i = 0; i < segment.rows; i++)
            {
                int x_index = segment.at<cv::Point>(i).x;
                int y_index = y_bins_ - segment.at<cv::Point>(i).y - 1;

                pcl::PointXYZI p;
                // p.x = x_index * resolution_ + g_x_min_;
                // p.y = y_index * resolution_ + g_y_min_;
                auto xy = index_xy_to_position_xy(x_index + min_index_xy_.x(), y_index + min_index_xy_.y(), 50000, 50000);
                p.x = xy.x();
                p.y = xy.y();
                p.z = bev_img_z_.at<float>(y_index, x_index);
                p.intensity = cnt;

                if (save_image_)
                {
                    skeleton_cluster_cloud->points.push_back(p);
                }

                cv::Point xy_index(y_index, x_index);
                if (m_xy_index_to_cloud_index_.find(xy_index) != m_xy_index_to_cloud_index_.end())
                {
                    cluster_index.push_back(m_xy_index_to_cloud_index_[xy_index]);
                }
            }
            cluster_indexs.push_back(cluster_index);
            cnt++;
        }

        if (save_image_)
        {
            if (skeleton_cluster_cloud->size() > 0)
            {
                skeleton_cluster_cloud->height = 1;
                skeleton_cluster_cloud->width = skeleton_cluster_cloud->size();
                std::string base_dir("/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross5/v1.00.25/batch-10010/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/9500026273967/laneBoundaryFromSeg/");
                std::string file_path(base_dir + "cluster_1_skeleton_cluster.pcd");
                if (fs::exists(base_dir))
                {
                    if (pcl::io::savePCDFile<pcl::PointXYZI>(file_path, *skeleton_cluster_cloud) == -1)
                    {
                        std::cerr << "Failed to load point cloud." << std::endl;
                        return false;
                    }
                }
            }

            if (skeleton_cloud->size() > 0)
            {
                skeleton_cloud->height = 1;
                skeleton_cloud->width = skeleton_cloud->size();
                std::string base_dir("/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross5/v1.00.25/batch-10010/model/auto_label/output/fsd_mapbuild_out_cloud_bev_label/9500026273967/laneBoundaryFromSeg/");
                std::string file_path(base_dir + "cluster_1_skeleton.pcd");
                if (fs::exists(base_dir))
                {
                    if (pcl::io::savePCDFile<pcl::PointXYZI>(file_path, *skeleton_cloud) == -1)
                    {
                        std::cerr << "Failed to load point cloud." << std::endl;
                        return false;
                    }
                }
            }
        }
        // cv::waitKey(0);

        return true;
    }

    bool SkeletonCluster::Process()
    {
        // step1: 读取pcd 并 生成BEV图像
        // GenBEV();
        GenBEV_V2();

        // step2: 进行基础的膨胀和腐蚀操作
        // 显示原始BEV图像
        if (save_image_)
        {
            cv::imshow("Original BEV Image", bev_img_);
            cv::imwrite("debug/1_origin_bev_image.png", bev_img_);
        }

        cv::Mat binary;
        cv::threshold(bev_img_, binary, 1, 255, cv::THRESH_BINARY);
        if (save_image_)
        {
            cv::imwrite("debug/2_1_binary.png", binary);
        }

        std::vector<std::vector<cv::Point>> contours;
        std::vector<cv::Vec4i> hierarcy;
        // cv::findContours(skeleton, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        cv::findContours(binary, contours, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
        // std::cout << " contour.size: " << contours.size() << " hierarcy.size: " << hierarcy.size() << std::endl;

        cv::Mat result;
        if (save_image_)
        {
            cv::cvtColor(binary, result, cv::COLOR_GRAY2BGR);
        }
        for (int i = 0; i < contours.size(); i++)
        {
            // 检查是否有子轮廓（空洞）
            // int next = hierarcy[i][0];
            // int prev = hierarcy[i][1];
            // int childIdx = hierarcy[i][2];
            int parent = hierarcy[i][3];
            if (parent >= 0)
            {
                double area = cv::contourArea(contours[i]);
                cv::RotatedRect min_area_rect = cv::minAreaRect(contours[i]);
                double width = min_area_rect.size.width;
                double height = min_area_rect.size.height;
                // if (area < 20 || (area < 80 && width < 40 && height < 40)) // 40 == 40*0.05 = 2m
                if ((area < 80 && width < 40 && height < 40)) // 40 == 40*0.05 = 2m
                {
                    std::vector<std::vector<cv::Point>> vvContour{contours[i]};
                    cv::fillPoly(binary, vvContour, cv::Scalar(255));
                }
                if (save_image_)
                {
                    cv::putText(result, std::to_string(int(area)), min_area_rect.center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1);
                    cv::drawContours(result, contours, i, cv::Scalar(0, 0, 255), 1);
                }
            } else {
                if (save_image_)
                {
                    cv::drawContours(result, contours, i, cv::Scalar(0, 255, 0), 1);
                }
            }
        }
        
        if (save_image_)
        {
            cv::imwrite("debug/2_11_raw_sub_contour.png", result);
        }
        
        return true;

    }




    // 计算两点之间的距离
    double distance(const cv::Point2f& a, const cv::Point2f& b) {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    // 计算圆心
    cv::Point2f calculateCenter(const cv::Point2f& a, const cv::Point2f& b, double radius) {
        double dx = b.x - a.x;
        double dy = b.y - a.y;
        double midpointX = (a.x + b.x) / 2.0;
        double midpointY = (a.y + b.y) / 2.0;
        double length = sqrt(dx * dx + dy * dy);
        double perpendicularSlope = -dx / dy;
        double c = sqrt(radius * radius - (length / 2.0) * (length / 2.0));
        double centerX = midpointX + c * perpendicularSlope / sqrt(1 + perpendicularSlope * perpendicularSlope);
        double centerY = midpointY + c / sqrt(1 + perpendicularSlope * perpendicularSlope);
        return cv::Point2f(centerX, centerY);
    }

    // 计算两个向量之间的夹角
    double getAngle(const cv::Point2f& origin, const cv::Point2f& p1, const cv::Point2f& p2) {
        cv::Point2f v1 = p1 - origin;
        cv::Point2f v2 = p2 - origin;
        double dot = v1.x * v2.x + v1.y * v2.y;
        double cross = v1.x * v2.y - v1.y * v2.x;
        return std::atan2(cross, dot) * 180.0 / CV_PI;
    }

    std::vector<cv::Point2f> rollingBall(const std::vector<cv::Point2f>& points, double radius) {
        std::vector<cv::Point2f> result;
        int n = points.size();
        std::vector<bool> visited(n, false);

        // 找到起始点（最下方的点）
        int startIdx = std::min_element(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
            return a.y > b.y || (a.y == b.y && a.x < b.x);
        }) - points.begin();

        int currentIdx = startIdx;
        result.push_back(points[currentIdx]);
        visited[currentIdx] = true;

        while (true) {
            double minAngle = 360.0;
            int nextIdx = -1;

            for (int i = 0; i < n; ++i) {
                if (visited[i]) continue;

                cv::Point2f center = calculateCenter(points[currentIdx], points[i], radius);
                // double angle = getAngle(center, points[currentIdx], points[i]);
                double angle = getAngle(points[currentIdx], center, points[i]);
                if (angle < minAngle) {
                    minAngle = angle;
                    nextIdx = i;
                }
            }

            if (nextIdx == -1 || nextIdx == startIdx) break;

            result.push_back(points[nextIdx]);
            visited[nextIdx] = true;
            currentIdx = nextIdx;
        }

        return result;
    }


    // 计算圆心
    std::vector<cv::Point2f> findCircleCenters(const cv::Point2f& A, const cv::Point2f& B, double r) {
        std::vector<cv::Point2f> centers;
        double d = distance(A, B); // 两点之间的距离

        // 如果两点之间的距离大于直径，无解
        if (d > 2 * r) {
            // std::cerr << "No solution: distance between points is greater than diameter." << std::endl;
            return centers;
        }

        // 如果两点重合且半径为零，圆心即为该点
        if (d == 0 && r == 0) {
            centers.push_back(A);
            return centers;
        }

        // 如果两点重合且半径不为零，无解
        if (d == 0 && r != 0) {
            std::cerr << "No solution: points are identical and radius is non-zero." << std::endl;
            return centers;
        }

        // 计算中点
        cv::Point2f mid = { (A.x + B.x) / 2, (A.y + B.y) / 2 };

        // 计算垂直平分线的方向向量
        cv::Point2f dir = { B.y - A.y, A.x - B.x };

        // 计算垂直平分线的方向向量长度
        double dir_len = std::sqrt(dir.x * dir.x + dir.y * dir.y);
        if (dir_len == 0) {
            std::cerr << "Error: direction vector length is zero." << std::endl;
            return centers;
        }

        // 单位化方向向量
        dir.x /= dir_len;
        dir.y /= dir_len;

        // 计算圆心到中点的距离
        double h = std::sqrt(r * r - (d * d) / 4);

        // 计算两个可能的圆心
        cv::Point2f center1 = { mid.x + h * dir.x, mid.y + h * dir.y };
        cv::Point2f center2 = { mid.x - h * dir.x, mid.y - h * dir.y };

        centers.push_back(center1);
        if (d < 2 * r) { // 如果两点之间的距离小于直径，有两个解
            centers.push_back(center2);
        }

        return centers;
    }


    // 计算两点之间的距离
    double distance2(const cv::Point2f& a, const cv::Point2f& b) {
        return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }

    // 判断点是否在圆内
    bool isPointInCircle(const cv::Point2f& point, const cv::Point2f& center, double radius) {
        return radius - distance2(point, center) > 1e-6;
        // return radius - distance2(point, center) > 1;
    }


    // 计算三角形的外接圆半径
    double calculateCircumradius(const cv::Point2f& a, const cv::Point2f& b, const cv::Point2f& c) {
        double ax = a.x, ay = a.y;
        double bx = b.x, by = b.y;
        double cx = c.x, cy = c.y;

        double d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
        double ux = ((ax * ax + ay * ay) * (by - cy) + (bx * bx + by * by) * (cy - ay) + (cx * cx + cy * cy) * (ay - by)) / d;
        double uy = ((ax * ax + ay * ay) * (cx - bx) + (bx * bx + by * by) * (ax - cx) + (cx * cx + cy * cy) * (bx - ax)) / d;

        return sqrt((ax - ux) * (ax - ux) + (ay - uy) * (ay - uy));
    }

    bool comparePoints(const cv::Point2f &a, const cv::Point2f &b)  {
        // return (a.x<b.x) || (((a.x-b.x<1e-6) && (a.y<b.y)));
        return (a.x<b.x) || ((a.x-b.x<1e-6) && a.y<b.y);
        // return (a.x<b.x) || (a.x==b.x && a.y<b.y);
    }
    
    // Alpha Shapes 算法
    std::vector<cv::Point2f> alphaShape(const std::vector<cv::Point2f>& points, double alpha) {
        std::vector<cv::Point2f> result;

        // 构建 Delaunay 三角剖分
        cv::Subdiv2D subdiv(cv::Rect(0, 0, 1000, 1000));
        for (const auto& pt : points) {
            subdiv.insert(pt);
        }

        // 获取所有三角形
        std::vector<cv::Vec6f> triangles;
        subdiv.getTriangleList(triangles);

        // 筛选符合条件的三角形
        for (const auto& t : triangles) {
            cv::Point2f a(t[0], t[1]);
            cv::Point2f b(t[2], t[3]);
            cv::Point2f c(t[4], t[5]);

            double r = calculateCircumradius(a, b, c);
            if (r <= alpha) {
                result.push_back(a);
                result.push_back(b);
                result.push_back(c);
            }
        }

        // 去重
        std::sort(result.begin(), result.end(), comparePoints);
        result.erase(unique(result.begin(), result.end(), comparePoints), result.end());

        return result;
    }


    // Alpha Shape算法实现
    std::vector<std::vector<cv::Point2f>> alphaShape2(const std::vector<cv::Point2f>& points, double alpha) {
        std::vector<std::vector<cv::Point2f>> edges;

        // 遍历所有点对
        for (size_t i = 0; i < points.size(); ++i) {
            for (size_t j = i + 1; j < points.size(); ++j) {
                // 计算两点之间的中点和距离
                cv::Point2f mid = (points[i] + points[j]) * 0.5;
                double dist = distance(points[i], points[j]);

                // 计算外接圆半径
                double radius = dist / 2.0;

                // 如果半径大于alpha值，则跳过
                if (radius > alpha) continue;

                // 判断是否所有其他点都在该圆外
                bool validEdge = true;
                for (size_t k = 0; k < points.size(); ++k) {
                    if (k == i || k == j) continue;
                    if (isPointInCircle(points[k], mid, radius)) {
                        validEdge = false;
                        break;
                    }
                }

                // 如果有效，则添加边
                if (validEdge) {
                    edges.push_back({points[i], points[j]});
                }
            }
        }

        return edges;
    }

    // Alpha Shape算法实现
    std::vector<std::vector<cv::Point>> alphaShape3(const std::vector<cv::Point>& points, double alpha) {
        std::vector<std::vector<cv::Point>> edges;
        int contour_point_size = points.size();
        // debug
        // std::cout << "points.size: " << contour_point_size  << " alpha: " << alpha << std::endl;

        // 遍历所有点对
        // TODO:qzc 剪枝
        int last_j = -1;
        for (int i = 0; i < contour_point_size; ++i) {
            if (i < last_j) {
                continue;
            }
            
            for (size_t j = i + 1; j < contour_point_size; ++j) {
                std::vector<cv::Point2f> centers = findCircleCenters(points[i], points[j], alpha);
                int center_size = centers.size();
                if (center_size == 0) {
                    continue;
                }
                

                // 判断是否所有其他点都在该圆外
                int debug_k = 0;
                std::vector<bool> validEdges = {true, true};
                for (int center_i = 0; center_i < center_size; center_i++) {
                    for (size_t k = 0; k < contour_point_size; ++k) {
                        if (k == i || k == j) continue;
                        if (isPointInCircle(points[k], centers[center_i], alpha)) {
                            validEdges[center_i] = false;
                            debug_k = 10000+k;
                            break;
                        }
                        debug_k = k;
                    }
                    // debug
                    // std::cout << " distance("<< i << ", "<< j << "):" << distance2(points[i], points[j]) << " center_i: " << center_i   << " debug_k: " << debug_k  << " center_size:" << center_size<< std::endl;
                }

                // 如果两个都为0，则说明

                if (center_size == 2 && validEdges[0] == true && validEdges[1] == true)
                {
                    std::cerr << COLOR_RED << "!!!!!!!! both true, i: " << i  << " j: " << j << std::endl;
                    // continue;
                }
                

                // 如果有效，则添加边
                if (validEdges[0] || validEdges[1]) {
                    edges.push_back({points[i], points[j]});
                    // debug
                    // std::cout << "valid:" << validEdges[0] << " " << validEdges[1] << " i: " << i  << " j: " << j << std::endl;

                    last_j = j;
                    break;
                }
            }
        }

        // debug
        // std::cout << "edges.size: " << edges.size() << std::endl;
        if (edges.size()>2) {
            auto& first_point = edges[0][0]; // first
            auto& last_point = edges[edges.size()-1][1]; // last

            edges.push_back({last_point, first_point});
        }


        std::vector<std::vector<cv::Point>> edges_expand;
        int edges_size = edges.size();
        for (size_t i = 0; i < edges_size-1; i++) {
            auto& cur_edge = edges[i];
            auto& next_edge = edges[i+1];

            edges_expand.push_back(cur_edge);
            if (cur_edge[1] != next_edge[0]) {
                edges_expand.push_back({cur_edge[1], next_edge[0]});
            }
        }

        // 塞入最后一个末尾-起始边
        edges_expand.push_back(edges[edges.size()-1]);
        
        
        return edges_expand;
    }


    bool IsIsland(const std::vector<cv::Point>& contour, std::vector<std::vector<cv::Point>>& edges, std::vector<cv::Point>& triangle) {
        edges.clear();
        triangle.clear();

        double area = cv::contourArea(contour);
        cv::RotatedRect min_area_rect = cv::minAreaRect(contour);
        double width = min_area_rect.size.width;
        double height = min_area_rect.size.height;
        if ((area < 300 && width < 60 && height < 60)) // 60 == 60*0.05 = 3m
        {
            return false;
        }

        std::vector<cv::Point> points;
        for (const auto& pt : contour) {
            points.push_back(pt);
        }

        // 应用 Alpha Shapes 算法
        double alpha = 200.0;  // 调整 alpha 参数 : 10m = 200 * 0.05
        // double alpha = 100.0;  // 调整 alpha 参数 : 5m = 100 * 0.05
        edges = alphaShape3(points, alpha);

        std::vector<cv::Point> edges_contours;
        for (size_t i = 0; i < edges.size(); i++) {
            edges_contours.push_back(edges[i][0]);
        }

        double area_triangle = cv::minEnclosingTriangle(edges_contours, triangle);
        std::vector<double> sides;
        for (int i = 0; i < triangle.size(); i++) {
            double d = distance(triangle[i], triangle[(i+1)%3]);
            sides.push_back(d);
        }
        std::sort(sides.begin(), sides.end(), std::greater<double>());
        double len_mid = sides[1];
        double len_min = sides[2];
        // std::cout << " " << sides[0]<< " " << sides[1]<< " " << sides[2] << std::endl;
        // alg::calc_theta1();

        double area2 = 0;
        double ratio_rect_to_external = 0.0;
        double half_rect_area = len_mid*len_min/2;
        if (edges.size() > 0) {
            area2 = cv::contourArea(edges_contours);
            ratio_rect_to_external = half_rect_area / area2;
        }

        // float len_mid = height > width ? height : width;
        // float len_min = height > width ? width : height;
        double ratio_max_len_to_min_len = len_mid / len_min;

        std::cout << "ratio_rect_to_external:(1, 2): " << ratio_rect_to_external << " (1,2): " << ratio_max_len_to_min_len 
                << " area: " << area << " width*height:" << len_mid*len_min << " area2: " << area2 << " are_triangle: " << area_triangle 
                << " width(180,600): " << len_mid << " height: " << len_min << std::endl;

        if (1.0 < ratio_rect_to_external && ratio_rect_to_external < 2 && 
            1.0 < ratio_max_len_to_min_len && ratio_max_len_to_min_len < 2 &&
            180 < len_mid  && len_mid < 600 // 10-30m
            ) 
        {
            return true;
        }
        
        return false;
    }

    bool SkeletonCluster::RefineEdge(int refine_mode, const std::vector<std::vector<cv::Point>>& edges_in, std::vector<std::vector<cv::Point>>& edges, cv::Point2f& centroid_out) {
        bool ret = false;

        cv::Mat binary = cv::Mat::zeros(y_bins_, x_bins_, CV_8UC1);
        for (const auto& edge : edges_in) {
            line(binary, edge[0], edge[1], 255, 1);
        }

        cv::Mat stats, centroids, labelImage;
        int nLabels = cv::connectedComponentsWithStats(binary, labelImage, stats, centroids, 8);
        if (nLabels == 2)
        {
            // for (size_t i = 1; i < nLabels; i++)  {
                // int stats_area = stats.at<int>(i, cv::CC_STAT_AREA);
                cv::Point2f centroid(centroids.at<double>(1, 0), centroids.at<double>(1, 1));
                centroid_out = centroid;
                std::cout << "connected component: " << 1 << " centroid : (" << centroid.x << ", "<< centroid.y << ")" << std::endl;
            // }
        }
        

        if (refine_mode == 1) // 外轮廓，重新提取内轮廓
        {
            int shift_pixel = 6;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(shift_pixel*2, shift_pixel*2));
            cv::Mat binary_dilate;
            cv::dilate(binary, binary_dilate, kernel, cv::Point(-1, -1), 1);

            std::vector<std::vector<cv::Point>> contours_inner;
            std::vector<cv::Vec4i> hierarcy;
            // cv::findContours(binary, contours_inner, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
            cv::findContours(binary_dilate, contours_inner, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
            if (contours_inner.size() == 2) {
                for (int i = 0; i < contours_inner.size(); i++) {
                    // 检查是否有子轮廓（空洞）
                    // int next = hierarcy[i][0];
                    // int prev = hierarcy[i][1];
                    // int childIdx = hierarcy[i][2];
                    int parent = hierarcy[i][3];
                    if (parent >= 0) {
                        int point_size = contours_inner[i].size();
                        std::vector<cv::Point> points;
                        for (size_t j = 0; j < point_size-1; j++) {
                            edges.push_back({contours_inner[i][j], contours_inner[i][j+1]});
                        }

                        // 塞入最后一个末尾-起始边
                        if (point_size>2) {
                            auto& first_point = contours_inner[i][0]; // first
                            auto& last_point = contours_inner[i][point_size-1]; // last
                            edges.push_back({last_point, first_point});
                        }

                        ret = true;
                    }
                }
                // ret = true;
            }

        } else if (refine_mode == 2) {  // 内轮廓，重新提取外轮廓
            int shift_pixel = 4;
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(shift_pixel*2, shift_pixel*2));
            cv::Mat binary_dilate;
            cv::dilate(binary, binary_dilate, kernel, cv::Point(-1, -1), 1);

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary_dilate, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            if (contours.size() == 1) {
                int point_size = contours[0].size();
                for (size_t i = 0; i < point_size-1; i++) {
                    edges.push_back({contours[0][i], contours[0][i+1]});
                }

                // 塞入最后一个末尾-起始边
                if (point_size>2) {
                    auto& first_point = contours[0][0]; // first
                    auto& last_point = contours[0][point_size-1]; // last
                    edges.push_back({last_point, first_point});
                }

                ret = true;
            }
        }

        return ret;

    }

    pcl::PointXYZI SkeletonCluster::BevPixelToPoint(cv::Point& p_in, int index)
    {
        int x_index = p_in.x;
        int y_index = y_bins_ - p_in.y - 1;
        pcl::PointXYZI p;
        auto xy = index_xy_to_position_xy(x_index + min_index_xy_.x(), y_index + min_index_xy_.y(), 50000, 50000);
        p.x = xy.x();
        p.y = xy.y();
        p.z = bev_img_z_.at<float>(y_index, x_index);
        p.intensity = index;

        return p;
    }

    PointElement SkeletonCluster::BevPixelToPoint(cv::Point& p_in, int ele_type, int cluster_id, int point_index, bool use_z)
    {
        int x_index = p_in.x;
        int y_index = y_bins_ - p_in.y - 1;
        PointElement p;
        auto xy = index_xy_to_position_xy(x_index + min_index_xy_.x(), y_index + min_index_xy_.y(), 50000, 50000);
        p.x = xy.x();
        p.y = xy.y();
        p.z = use_z ? bev_img_z_.at<float>(y_index, x_index) : 0;
        // p.intensity = index;

        p.ele_type = ele_type;
        p.id = cluster_id;
        p.index = point_index;
        // p.heading = 0;
        // p.score = 0;

        return p;
    }

    bool SkeletonCluster::ProcessBoundary()
    {
        // step1: 读取pcd 并 生成BEV图像
        // GenBEV();
        GenBEV_V2();

        // step2: 进行基础的膨胀和腐蚀操作
        // 显示原始BEV图像
        if (save_image_) {
            // cv::imshow("Original BEV Image", bev_img_);
            std::string output_image(output_dir_ + "debug/" + cluster_index_ + "_1_origin_bev_image.png");
            cv::imwrite(output_image, bev_img_);
            // std::cout << "save: " << output_image<< std::endl;
        }

        cv::Mat kernel_2x2 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(2, 2));
        cv::Mat kernel_3x3 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        // cv::Mat kernel_5x5 = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
        cv::morphologyEx(bev_img_, bev_img_, cv::MORPH_OPEN, kernel_3x3);
        cv::morphologyEx(bev_img_, bev_img_, cv::MORPH_CLOSE, kernel_3x3);
        
        cv::Mat binary;
        cv::threshold(bev_img_, binary, 1, 255, cv::THRESH_BINARY);
        if (save_image_) {
            std::string output_image(output_dir_ + "debug/" + cluster_index_ + "_2_morphology_bev_image.png");
            cv::imwrite(output_image, binary);
        }

        // cv::Mat stats, centroids, labelImage;
        // int nLabels = cv::connectedComponentsWithStats(binary, labelImage, stats, centroids, 4);
        // // cv::Mat mask(labelImage.size(), CV_8UC1, cv::Scalar(0));
        // for (size_t i = 0; i < nLabels; i++)
        // {
        //     int stats_area = stats.at<int>(i, cv::CC_STAT_AREA);
        //     std::cout << "connected component: " << i << " : " << stats_area << " pixels"<< std::endl;
        // }
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        // cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        cv::Mat result;
        if (save_image_) {
            cv::cvtColor(binary, result, cv::COLOR_GRAY2BGR);
        }
        for (int i = 0; i < contours.size(); i++) {
            int refine_mode = 0;
            auto origin_contour = contours[i];

            // 选择一个轮廓
            std::vector<std::vector<cv::Point>> edges;
            std::vector<cv::Point> triangle;
            bool is_island = IsIsland(contours[i], edges, triangle);
            if (is_island) {
                refine_mode = 1; // 外轮廓
                
                std::cout << COLOR_BLUE << " is island " << "->outer "  << COLOR_DEFAULT << std::endl;

                // 重新再找一下内部轮廓线
                std::vector<std::vector<cv::Point>> contours_inner;
                std::vector<cv::Vec4i> hierarcy;
                cv::findContours(binary, contours_inner, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_NONE);
                // cv::findContours(binary, contours_inner, hierarcy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE);
                for (int j = 0; j < contours_inner.size(); j++) {
                    // 检查是否有子轮廓（空洞）
                    // int next = hierarcy[j][0];
                    // int prev = hierarcy[j][1];
                    // int childIdx = hierarcy[j][2];
                    int parent = hierarcy[j][3];
                    if (parent >= 0) {
                        std::vector<std::vector<cv::Point>> edges_inner;
                        std::vector<cv::Point> triangle_inner;
                        bool is_island_inner = IsIsland(contours_inner[j], edges_inner, triangle_inner);
                        if (is_island_inner) {
                            refine_mode = 2; // 内轮廓

                            std::cout << COLOR_GREEN << " is island " << "->inner " << COLOR_DEFAULT << std::endl;
                            contours[i] = contours_inner[j];
                            edges = edges_inner;
                            triangle = triangle_inner;
                        }
                    }
                }
            }

            cv::Point2f centroid(0, 0);
            std::vector<std::vector<cv::Point>> edges_refine;
            edges_refine.clear();
            bool is_refine_success = RefineEdge(refine_mode, edges, edges_refine, centroid);
            if (is_refine_success) {
                edges = edges_refine;
            }
            
            if (save_image_) {
                // cv::putText(result, std::to_string(int(area)), min_area_rect.center, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 255), 1);
                cv::drawContours(result, contours, i, cv::Scalar(0, 0, 255), 1);
                
                // 绘制结果
                for (const auto& edge : edges) {
                    line(result, edge[0], edge[1], cv::Scalar(255, 0, 0), 1);
                }
                std::cout << "size: " << edges.size() << std::endl;
                
                if (edges.size() > 0) {
                    circle(result, edges[0][0], 1, cv::Scalar(0, 255, 0), cv::FILLED);
                    // circle(result, edges[3][0], 1, cv::Scalar(255, 255, 0), cv::FILLED);
                    // circle(result, points[1408], 1, cv::Scalar(255, 255, 0), cv::FILLED);
                    // circle(result, points[1428], 1, cv::Scalar(255, 255, 0), cv::FILLED);
                    circle(result, centroid, 3, cv::Scalar(0, 0, 255), cv::FILLED);
                }

                for (int k = 0; k < triangle.size(); k++) {
                    cv::line(result, triangle[k], triangle[(k+1)%3], cv::Scalar(0, 255, 255), 2);
                }
                
            }

            // 转换为点云格式
            if (refine_mode > 0) {
                // 矢量化的边界线
                {
                    pcl::PointCloud<PointElement>::Ptr island_cluster_cloud(new pcl::PointCloud<PointElement>());
                    int cnt = 0;
                    int edges_size = edges.size();
                    for (int j = 0; j < edges_size; j++) {
                        auto& seg_start_point = edges[j][0];
                        auto p = BevPixelToPoint(seg_start_point, 10, std::stoi(cluster_index_), j);
                        p.type1 = 2; // 白线
                        island_cluster_cloud->points.push_back(p);

                        if (j == edges_size-1) {
                            auto& seg_end_point = edges[j][1];
                            auto p = BevPixelToPoint(seg_end_point, 10, std::stoi(cluster_index_), j+1);
                            p.type1 = 2; // 白线
                            island_cluster_cloud->points.push_back(p);
                        }
                    }

                    // 将质心写入
                    if (centroid.x > 1e-6 || centroid.y > 1e-6) {
                        cv::Point centroid(floor(centroid.x), floor(centroid.y));
                        auto p = BevPixelToPoint(centroid, 100, std::stoi(cluster_index_), 0, false);
                        island_cluster_cloud->points.push_back(p);
                    }
                    

                    if (island_cluster_cloud->size() > 0) {
                        pcl::io::savePCDFileBinary(output_dir_ + cluster_index_ + "_trjCloud.pcd", *island_cluster_cloud);
                    }
                }

                // 外轮廓
                {
                    pcl::PointCloud<PointElement>::Ptr island_cluster_cloud(new pcl::PointCloud<PointElement>());
                    int cnt = 0;
                    int edges_size = origin_contour.size();

                    for (int j = 0; j < edges_size; j++) {
                        auto& seg_start_point = origin_contour[j];
                        auto p = BevPixelToPoint(seg_start_point, 10, std::stoi(cluster_index_), j);
                        p.type1 = 2; // 白线
                        island_cluster_cloud->points.push_back(p);
                    }

                    if (island_cluster_cloud->size() > 0) {
                        pcl::io::savePCDFileBinary(output_dir_ + cluster_index_ + "_trjCloud_contour.pcd", *island_cluster_cloud);
                    }
                }
            }
        }
        
        if (save_image_) {
            std::string output_image(output_dir_ + "debug/" + cluster_index_ + "_3_contour.png");
            cv::imwrite(output_image, result);
        }
        
        return true;  
    }
   
}