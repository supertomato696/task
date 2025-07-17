

#include <gflags/gflags.h>
#include "utils/visualization_util.h"
#include "utils/algorithm_util.h"

// #include <pcl/visualization/cloud_viewer.h>   // visualization模块下的CloudViewer头文件。
#include <iostream>   // 标准输入输出头文件声明。
#include <pcl/io/io.h>  // IO相关头文件声明。
#include <pcl/io/pcd_io.h>

DEFINE_bool(pcd_display_use_scale, false, "pcd_display_use_scale");

namespace fsdmap {
namespace utils {

RGB trans_color(float value, Eigen::Vector3d min, Eigen::Vector3d max) {
    Eigen::Vector3d diff = max - min;
    diff *= value;
    diff += min;
    if (diff.x() < 0) {
        diff.x() += max.x() - min.x();
    }
    if (diff.y() < 0) {
        diff.y() += max.y() - min.y();
    }
    if (diff.z() < 0) {
        diff.z() += max.z() - min.z();
    }
    return {(int)diff.x(), (int)diff.y(), (int)diff.z()};
}

RGB trans_intensity(float intensity) {
    double value = intensity / 255;
    if (value <= 0.06535948) {
        return trans_color(value / 0.06535948, {0, 0, 255}, {0, 255, 0});
    } else if (value <= 0.13071895) {
        return trans_color((value - 0.06535948) / (0.13071895 - 0.06535948), 
                {0, 255, 0}, {255, 255, 0});
    } else if (value <= 0.19607843) {
        return trans_color((value - 0.13071895) / (0.19607843 - 0.13071895), 
                {255, 255, 0}, {239, 41, 41});
    } else if (value <= 1) {
        return trans_color((value - 0.19607843) / (1 - 0.19607843), {239, 0, 0}, {255, 0, 0});
    } else {
        return {0, 0, 255};
    }
}

void trans_display_local_pos(DisplayScope &box, Eigen::Vector3d &pos, bool need_scale) {
    Eigen::Vector3d local_pos = pos - box.center_pos;
    //TODO 目前只有2维，3维旋转待补充
    // if (box.dir.z() != 0.0) {
    //     alg::rotate_yaw(local_pos, local_pos, box.dir.z());
    // }
    if (need_scale) {
        local_pos /= box.resolution;
        local_pos.x() += box.x_radius + 0.5;
        local_pos.y() = box.y_radius - local_pos.y() - 0.5;
    }
    pos = local_pos;
}

void save_display_image(const char * file_name, DisplayScope &box, DisplayInfo* log) {
    std::vector<DisplayInfo*> log_list = {log};
    save_display_image(file_name, box, log_list);
}

void save_display_image(const char * file_name, DisplayScope &box, std::vector<DisplayInfo*> &log_list) {
    cv::Mat im;
    gen_display_image(im, box, log_list);
    boost::filesystem::path path(file_name);
    boost::filesystem::path p_path = path.parent_path();
    if (!boost::filesystem::exists(p_path)) {
        boost::filesystem::create_directories(p_path);
    }
    cv::imwrite(file_name, im);
}

void gen_display_image(cv::Mat &im, DisplayScope &box, DisplayInfo* log) {
    std::vector<DisplayInfo*> log_list = {log};
    gen_display_image(im, box, log_list);
}

void gen_display_image(cv::Mat &im, DisplayScope &box, std::vector<DisplayInfo*> &log_list) {
    // image limit 16384
    int limit = 16384;
    int width = (int)box.x_radius * 2;
    int height = (int)box.y_radius * 2;
    width = std::min(width, 16384);
    height = std::min(height, 16384);
    im = cv::Mat(height, width, 
            CV_8UC3, cv::Scalar(0, 0, 0));
    for (int j = log_list.size() - 1; j >= 0; --j) {
        auto &log = log_list[j];
        // if (!log->has_cache_img) {
            for (int i = 0; i < log->path.size(); ++i) {
                Eigen::Vector3d tar_pos = log->path[i].pos;
                trans_display_local_pos(box, tar_pos);
                log->path[i].tar_pt = cv::Point((int)tar_pos.x(), (int)tar_pos.y());
                auto &color = log->path[i].color;
                if (color == cv::Scalar(0, 0, 0)) {
                    color = log->color;
                }
            }
        // }
        // log->has_cache_img = true;
        if (log->type == DisplayInfo::LINE) {
            for (int i = 1; i < log->path.size(); ++i) {
                auto &pt1 = log->path[i - 1].tar_pt;
                auto &pt2 = log->path[i].tar_pt;
                auto &color = log->path[i].color;
                cv::arrowedLine(im, pt1, pt2, color, 1, 8, 0, 0.1);
            }
        } else if (log->type == DisplayInfo::LINE_INDEX) {
            for (int i = 1; i < log->path.size(); ++i) {
                auto &pt1 = log->path[i - 1].tar_pt;
                auto &pt2 = log->path[i].tar_pt;

                auto &color = log->path[i].color;
                //  在图像上绘制从 pt1 到 pt2 的箭头，表示路径的方向
                cv::arrowedLine(im, pt1, pt2, color, 1, 8, 0, 0.3);
                auto index_no = utils::fmt("{}", i);// 格式化索引号（i 表示当前点的索引，起始为1）
                //在当前点的位置（pt2）上绘制索引号文本，作为标记
                cv::putText(im, index_no.c_str(), pt2,
                        cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0), 1, 8);
            }
        }  else if (log->type == DisplayInfo::TEXT) {
            for (int i = 0; i < log->path.size(); ++i) {
                auto &pt1 = log->path[i].tar_pt;
                auto &color = log->path[i].color;
                //  在图像上绘制从 pt1 到 pt2 的箭头，表示路径的方向
                // cv::arrowedLine(im, pt1, pt2, color, 1, 8, 0, 0.3);
                auto index_desc = utils::fmt("{}", log->desc);// 格式化索引号（i 表示当前点的索引，起始为1）
                //在当前点的位置（pt2）上绘制索引号文本，作为标记
                cv::putText(im, index_desc.c_str(), pt1,
                        cv::FONT_HERSHEY_COMPLEX, 0.5, color, 1, 8);
                // cv::putText(im, index_desc.c_str(), pt1,
                //         cv::FONT_HERSHEY_COMPLEX, 0.3, color, 1, 20); //调试覆盖度补点
            }
        } else if (log->type == DisplayInfo::POINT) {
            int radius = (width + height) / 20000;
            for (int i = 0; i < log->path.size(); ++i) {
                auto &pt1 = log->path[i].tar_pt;
                auto &color = log->path[i].color;
                cv::circle(im, pt1, radius + 1, color, radius + 2);
                cv::circle(im, pt1, radius, color, radius + 1);//原来的
            }
        } else if (log->type == DisplayInfo::POLYGEN) {
            cv::Point root_points[log->path.size()];
            for (int i = 0; i < log->path.size(); ++i) {
                auto &pt = log->path[i].tar_pt;
                root_points[i] = pt;
            }
            const cv::Point* ppt[1] = { root_points };
            int npt[] = { log->path.size() };
            cv::fillPoly(im, ppt, npt, 1, log->color);
        }
    }
    cv::cvtColor(im, im, cv::COLOR_BGR2RGB);
}

float rgb_2_float(cv::Scalar &rgb) {
    std::uint32_t rgb1 = ((std::uint32_t)rgb[0] << 16 
            | (std::uint32_t)rgb[1] << 8 
            | (std::uint32_t)rgb[2]);
    float rgb2 = *reinterpret_cast<float*>(&rgb1);
    return rgb2;
}

void sample_line_to_cloud_point(CloudPtr &tar_cloud, Eigen::Vector3d &start_pt,
        Eigen::Vector3d& end_pt, LogElement &ele) {
        // Eigen::Vector3d& end_pt, cv::Scalar &rgb, PointLabel &label) {
    double pt_gap = 0.2;
    Eigen::Vector3d dir = alg::get_dir(end_pt, start_pt);
    double dis = alg::calc_dis(start_pt, end_pt);
    int sample_num = (int)(dis / pt_gap) + 1;
    int middle = sample_num / 2;
    for (int i = 0; i < sample_num; ++i) {
        double scale1 = (double) i / (sample_num - 1);
        double scale2 = i == 0 ?  0 : (pow(scale1, 0.7) / scale1);
        Eigen::Vector3d tmp = start_pt + scale2 * i * pt_gap * dir;
        if (i == sample_num - 1) {
            tmp = end_pt;
        }
        CloudPoint new_pt = ele.label;
        new_pt.x = tmp.x();
        new_pt.y = tmp.y();
        new_pt.z = tmp.z();
        new_pt.rgb = rgb_2_float(ele.color);
        tar_cloud->emplace_back(new_pt);
    }
}


void sample_cycle_to_cloud_point(CloudPtr &tar_cloud, Eigen::Vector3d &pt,
        double radius, LogElement &ele) {
        // double radius, cv::Scalar &rgb, PointLabel &label) {
    double pt_gap = 0.05;
    int sample_num = radius / pt_gap + 1;
    sample_num *= sample_num;
    int64_t l_radius = (int) (radius * 1000);
    for (int i = 0; i < sample_num; ++i) {
        int x = rand() % (2 * l_radius) - l_radius;
        int y = rand() % (2 * l_radius) - l_radius;
        if (pow(pow((double)x, 2) + pow((double)y, 2), 0.5) > l_radius) {
            continue;
        }
        CloudPoint new_pt = ele.label;
        new_pt.x = (double)x / 1000 + pt.x();
        new_pt.y = (double)y / 1000 + pt.y();
        new_pt.z = pt.z();
        new_pt.rgb = rgb_2_float(ele.color);
        tar_cloud->emplace_back(new_pt);
    }
}

int64_t save_display_pcd(CloudPtr &tar_cloud, DisplayScope &box, std::vector<DisplayInfo*> &log_list) {
    double pt_gap = 0.1;
    for (auto &log : log_list) {
        if (log->has_cache_pcd) {
            *tar_cloud += *log->cache_cloud;
            continue;
        }
        log->cache_cloud = CloudPtr(new pcl::PointCloud<CloudPoint>);
        log->has_cache_pcd = true;
        if (log->type == DisplayInfo::LINE || log->type == DisplayInfo::LINE_INDEX) {
            for (int i = 1; i < log->path.size(); ++i) {
                auto start_pt = log->path[i - 1].pos;
                auto end_pt = log->path[i].pos;
                auto &color = log->path[i - 1].color;
                if (color == cv::Scalar(0, 0, 0)) {
                    color = log->color;
                }
                sample_line_to_cloud_point(log->cache_cloud, start_pt, end_pt, 
                        log->path[i - 1]);
            }
        } else if (log->type == DisplayInfo::POINT) {
            double radius = 0.1;
            for (int i = 0; i < log->path.size(); ++i) {
                auto pt = log->path[i].pos;
                auto &color = log->path[i].color;
                if (color == cv::Scalar(0, 0, 0)) {
                    color = log->color;
                }
                sample_cycle_to_cloud_point(log->cache_cloud, pt, radius, log->path[i]);
            }
        } else if (log->type == DisplayInfo::POLYGEN) {

            for (int i = 1; i < log->path.size(); ++i) {
                auto start_pt = log->path[i - 1].pos;
                auto end_pt = log->path[i].pos;
                auto &color = log->path[i - 1].color;
                if (color == cv::Scalar(0, 0, 0)) {
                    color = log->color;
                }
                sample_line_to_cloud_point(log->cache_cloud, start_pt, end_pt, 
                        log->path[i - 1]);
            }
            
            // cv::Point root_points[log->path.size()];
            // for (int i = 0; i < log->path.size(); ++i) {
            //     auto pt = log->path[i];
            //     trans_display_local_pos(box, pt);
            //     root_points[i] = {(int)pt.x(), (int)pt.y()};
            // }
            // trans_display_local_pos(box, start_pt, FLAGS_pcd_display_use_scale);
            // trans_display_local_pos(box, end_pt, FLAGS_pcd_display_use_scale);
            // sample_line_to_cloud_point(tar_cloud, start_pt, end_pt);
        }
        *tar_cloud += *log->cache_cloud;
    }
    return fsdmap::SUCC;
}

int64_t save_display_pcd(const char * file_name, DisplayScope &box, DisplayInfo* log) {
    std::vector<DisplayInfo*> log_list = {log};
    return save_display_pcd(file_name, box, log_list);
}

int64_t save_display_pcd(const char * file_name, DisplayScope &box, std::vector<DisplayInfo*> &log_list) {
    CloudPtr cloud(new pcl::PointCloud<CloudPoint>);
    save_display_pcd(cloud, box, log_list);
    if (cloud->points.size() == 0) {
        return 0;
    }
    boost::filesystem::path path(file_name);
    boost::filesystem::path p_path = path.parent_path();
    if (!boost::filesystem::exists(p_path)) {
        boost::filesystem::create_directories(p_path);
    }
    pcl::io::savePCDFileBinary(file_name, *cloud);
    return cloud->points.size();
}

int64_t save_display_pcd(RGBCloudPtr &tar_cloud, DisplayScope &box, CloudPtr &src_cloud,
        utils::ThreadPoolProxy* pool) {
    int64_t curr_index = tar_cloud->points.size();
    tar_cloud->points.resize(curr_index + src_cloud->points.size());
    for (int i = 0; i < src_cloud->points.size(); ++i) {
        if (pool == NULL) {
            auto &raw_pt = src_cloud->points[i];
            Eigen::Vector3d pt = {raw_pt.x, raw_pt.y, raw_pt.z};
            // trans_display_local_pos(box, pt, FLAGS_pcd_display_use_scale);
            // RGB rgb = trans_intensity(raw_pt.intensity);
            tar_cloud->points[curr_index + i].x = pt.x();
            tar_cloud->points[curr_index + i].y = pt.y();
            tar_cloud->points[curr_index + i].z = pt.z();
            tar_cloud->points[curr_index + i].rgb = raw_pt.rgb;
        } else {
            pool->schedule([&, i](utils::ProcessBar *process_bar) {
                    auto &raw_pt = src_cloud->points[i];
                    Eigen::Vector3d pt = {raw_pt.x, raw_pt.y, raw_pt.z};
                    // trans_display_local_pos(box, pt, FLAGS_pcd_display_use_scale);
                    tar_cloud->points[curr_index + i].x = pt.x();
                    tar_cloud->points[curr_index + i].y = pt.y();
                    tar_cloud->points[curr_index + i].z = pt.z();
                    tar_cloud->points[curr_index + i].rgb = raw_pt.rgb;
                    //       tar_cloud->points[curr_index + i] = {pt.x(), pt.y(), pt.z(), rgb2};

                    });
        }
    }
    if (pool != NULL) {
        pool->wait();
    }
    return tar_cloud->points.size();
}

void trans_pcd_point(CloudPoint &src_pt, CloudPoint &tar_pt, DisplayScope &box) {
    // pcd无需计算中心
    tar_pt = src_pt;
    // Eigen::Vector3d pt = {src_pt.x, src_pt.y, src_pt.z};
    // trans_display_local_pos(box, pt, FLAGS_pcd_display_use_scale);
    // tar_pt = src_pt;
    // tar_pt.x = pt.x();
    // tar_pt.y = pt.y();
    // tar_pt.z = pt.z();
}

int64_t save_display_pcd(CloudPtr &tar_cloud, DisplayScope &box, CloudPtr &src_cloud,
        utils::ThreadPoolProxy* pool) {
    int64_t curr_index = tar_cloud->points.size();
    int64_t valid_size = 0;
    for (int i = 0; i < src_cloud->points.size(); ++i) {
        auto &src_pt = src_cloud->points[i];
        if (src_pt.status == 1) {
            ++valid_size;
        }
    }
    tar_cloud->points.resize(curr_index + valid_size);
    int64_t valid_index = 0;
    for (int i = 0; i < src_cloud->points.size(); ++i) {
        if (src_cloud->points[i].status != 1) {
            continue;
        }
        if (pool == NULL) {
            auto &src_pt = src_cloud->points[i];
            // auto &tar_pt = src_cloud->points[i];
            auto &tar_pt = tar_cloud->points[curr_index + valid_index++];
            trans_pcd_point(src_pt, tar_pt, box);
        } else {
            pool->schedule([&, i, valid_index](utils::ProcessBar *process_bar) {
                    auto &src_pt = src_cloud->points[i];
                    // auto &tar_pt = src_cloud->points[i];
                    auto &tar_pt = tar_cloud->points[curr_index + valid_index];
                    trans_pcd_point(src_pt, tar_pt, box);
                    });
            ++valid_index;
        }
    }
    if (pool != NULL) {
        pool->wait();
    }
    return valid_size;
}

}
}
