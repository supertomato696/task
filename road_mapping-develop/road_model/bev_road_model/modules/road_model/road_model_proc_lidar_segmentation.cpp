


#include "road_model_proc_lidar_segmentation.h"
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h> //边界提取

DECLARE_double(display_scope_buff);

DEFINE_bool(lidar_segmentation_enable, true, "lidar_segmentation_enable");
DEFINE_bool(lidar_segmentation_debug_pos_enable, true, "lidar_segmentation_debug_enable");
DEFINE_bool(lidar_segmentation_save_data_enable, false, "lidar_segmentation_save_data_enable");
DEFINE_bool(lidar_segmentation_save_data_pcd_all_enable, true, "lidar_segmentation_save_data_pcd_all_enable");

DEFINE_bool(lidar_segmentation_load_pcd_enable, true, "lidar_segmentation_load_pcd_enable");
DEFINE_bool(lidar_segmentation_pcd_display_use_ring, false, "lidar_segmentation_pcd_display_use_ring");
DEFINE_bool(lidar_segmentation_pcd_trans_intensity_enable, false, "lidar_segmentation_pcd_trans_intensity_enable");
DEFINE_bool(lidar_segmentation_get_label_enable, false, "lidar_segmentation_get_label_enable");

DEFINE_int32(lidar_segmentation_pcd_display_ring_max, 125, "lidar_segmentation_pcd_display_ring_max");
DEFINE_int32(lidar_segmentation_pcd_display_ring_min, 60, "lidar_segmentation_pcd_display_ring_min");
DEFINE_int32(lidar_segmentation_pcd_display_ring_time, 100, "lidar_segmentation_pcd_display_ring_time");
DEFINE_int32(lidar_segmentation_pcd_display_ring_time_max, 20000, "lidar_segmentation_pcd_display_ring_time_max");
DEFINE_int32(lidar_segmentation_pcd_display_ring_time_min, 20000, "lidar_segmentation_pcd_display_ring_time_min");
DEFINE_int32(lidar_segmentation_pcd_display_pcd_scope, 30, "lidar_segmentation_pcd_display_pcd_scope");

DECLARE_bool(gen_key_point_use_cache);

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcLidarSegmentation::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_lidar_segmentation_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    if (FLAGS_gen_key_point_use_cache) {
        return fsdmap::process_frame::PROC_STATUS_SUCC;
    }

    session->enable_debug_pos = FLAGS_lidar_segmentation_debug_pos_enable;

    CHECK_FATAL_PROC(preprocess_pcd(session), "preprocess_pcd");

    // CHECK_FATAL_PROC(boundary_segment(session), "boundary_segment");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcLidarSegmentation::boundary_segment(RoadModelSessionData* session) {
    // if (!FLAGS_lidar_segmentation_get_label_enable) {
    //     return fsdmap::SUCC;
    // }
    if (!session->opt_ground_pcd) {
        return fsdmap::SUCC;
    }
    //提取所有点
    pcl::PointCloud<pcl::PointXYZ>::Ptr road_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto pt : *session->opt_ground_pcd) {
        pcl::PointXYZ newpt(pt.x, pt.y, pt.z);
        road_ptr->push_back(newpt);
    }

    //------------------------半径滤波---------------------------
    LOG_INFO("->半径滤波");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;	//创建滤波器对象
    ror.setInputCloud(road_ptr);						//设置待滤波点云
    ror.setRadiusSearch(0.2);						//设置查询点的半径范围
    ror.setMinNeighborsInRadius(5);					//设置判断是否为离群点的阈值，即半径内至少包括的点数
    //ror.setNegative(true);						//默认false，保存内点；true，保存滤掉的外点
    ror.filter(*cloud_filtered);					//执行滤波，保存滤波结果于cloud_filtered

    //------------------------均匀下采样---------------------------
    LOG_INFO("->均匀下采样");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sample(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::UniformSampling<pcl::PointXYZ> us;	//创建滤波器对象
    us.setInputCloud(cloud_filtered);				//设置待滤波点云
    us.setRadiusSearch(0.1);				//设置滤波球体半径
    us.filter(*cloud_sample);				//执行滤波，保存滤波结果于cloud_filtered

    if(cloud_sample!=NULL && cloud_sample->size()>0)
    {
        std::string pcd_file = session->get_debug_dir("genkeypoint_road_deal.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud_sample);
    }

    //------------------------计算法向量---------------------------
    LOG_INFO("->计算法向量");
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud_sample));
    normEst.setRadiusSearch(0.5);
    normEst.compute(*normals);

    LOG_INFO("->点云边缘估计");
    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
    boundEst.setInputCloud(cloud_sample);
    boundEst.setInputNormals(normals);
    boundEst.setRadiusSearch(0.25);
    boundEst.setAngleThreshold(M_PI / 2);
    boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    boundEst.compute(boundaries);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < cloud_sample->points.size(); i++) {
        if (boundaries[i].boundary_point > 0) {
            cloud_boundary->push_back(cloud_sample->points[i]);
        }
    }

    if (!cloud_boundary->points.empty())
    {
        session->roadedge_pcd = cloud_boundary;
        std::string pcd_file = session->get_debug_dir("genkeypoint_road_edge.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud_boundary);
    }

    return fsdmap::SUCC;
}

int RoadModelProcLidarSegmentation::preprocess_pcd(RoadModelSessionData* session) {
    if (session->raw_ground_pcd.size() == 0) {
        return fsdmap::SUCC;
    }
    for (auto &key_pose : session->raw_ground_pcd) {
        // filter_intensity(session, key_pose->pcd, NULL, key_pose.get());
        session->opt_ground_pcd = key_pose->pcd;
    }
    // for (auto &pt : session->opt_ground_pcd->points) {
    //     if (pt.intensity_opt > 0) {
    //         pt.opt_label = 0;
    //     }
    // }
    
    // auto &box = session->_scope;
    // cv::Mat im = cv::Mat((int)(box.y_radius * 2), (int)(box.x_radius * 2), CV_8UC1, cv::Scalar(0));
    // using num_class = std::tuple<float, int, int>;
    // std::vector<std::vector<num_class>> cache_vec(im.cols);
    // for (auto &vecx : cache_vec) {
    //     vecx = std::vector<num_class>(im.rows); 
    //     for (auto &vecy : vecx) {
    //         vecy = std::make_tuple(0, 0, 0);
    //     }
    // }
    //     if (pt.intensity_opt <= 0) {
    //         continue;
    //     }
    //     Eigen::Vector3d xy = {pt.x, pt.y, 0};
    //     // utils::trans_display_local_pos(box, xy);
    //     int x = (int)round(xy.x());
    //     int y = (int)round(xy.y());
    //     if (x < 0 || x >= im.cols) {
    //         continue;
    //     }
    //     if (y < 0 || y >= im.rows) {
    //         continue;
    //     }
    //     auto &cache = cache_vec[x][y];
    //     if (std::get<2>(cache) == 0) {
    //         std::get<0>(cache) = pt.z;
    //         std::get<1>(cache) = pt.intensity_opt;
    //         std::get<2>(cache) = 1;
    //         continue;
    //     }
    //     std::get<1>(cache) += pt.intensity_opt;
    //     std::get<2>(cache) += 1;
    // }
    // for (int x = 0; x < cache_vec.size(); ++x) {
    //     auto &vecx = cache_vec[x];
    //     for (int y = 0; y < vecx.size(); ++y) {
    //         auto &cache = vecx[y];
    //         float avg_value = ((float)std::get<1>(cache) / std::get<2>(cache)) * 40;
    //         im.at<uchar>(y, x) = (int)avg_value;
    //     }
    // }
    // std::string image_file = session->get_debug_dir("0pcd_image.png");
    // cv::imwrite(image_file, im);
    return fsdmap::SUCC;
}

int64_t RoadModelProcLidarSegmentation::filter_intensity(RoadModelSessionData* session,
        utils::CloudPtr &src_cloud, Eigen::Vector3d* key_pos, KeyPose* poss) {
    int64_t valid_size = 0;
    for (int i = 0; i < src_cloud->points.size(); ++i) {
        auto &src_pt = src_cloud->points[i];
        if (FLAGS_lidar_segmentation_pcd_display_use_ring) {
            // if (src_pt.ring > FLAGS_lidar_segmentation_pcd_display_ring_max 
            //         || src_pt.ring < FLAGS_lidar_segmentation_pcd_display_ring_min) {
            //     continue;
            // }
            // if (src_pt.ring < FLAGS_lidar_segmentation_pcd_display_ring_time) {
            //     // if (src_pt.time < FLAGS_lidar_segmentation_pcd_display_ring_time_max
            //     //         && src_pt.time > FLAGS_lidar_segmentation_pcd_display_ring_time_min) {
            //     continue;
            //     // }
            // }
        }
        if (key_pos != NULL) {
            Eigen::Vector3d raw_pos = {src_pt.x, src_pt.y, src_pt.z};
            Eigen::Vector3d tar_pos = {src_pt.x, src_pt.y, src_pt.z};
            Eigen::Vector3d base = {0, 0, 0};

            double dis = alg::calc_dis(raw_pos, base);
            if (dis > FLAGS_lidar_segmentation_pcd_display_pcd_scope) {
                continue;
            }
            // alg::rotate_yaw(raw_pos, tar_pos, yaw);
            tar_pos = poss->r * raw_pos;
            tar_pos += *key_pos;
            src_pt.x = tar_pos.x();
            src_pt.y = tar_pos.y();
            src_pt.z = tar_pos.z();
        }

        ++valid_size;
        src_pt.status = 1;
    }
    pcl::KdTreeFLANN<utils::CloudPoint> kdtree;
    kdtree.setInputCloud(src_cloud);
    for (int i = 0; i < src_cloud->points.size(); ++i) {
        if (src_cloud->points[i].status != 1) {
            continue;
        }
        if (!FLAGS_lidar_segmentation_pcd_trans_intensity_enable) {
            utils::CloudPoint &src_pt = src_cloud->points[i];
            src_pt.intensity_opt = src_pt.intensity;
            continue;
        }
        // session->thread_pool->schedule([&, i](utils::ProcessBar *process_bar) {
        //     utils::CloudPoint &src_pt = src_cloud->points[i];
        //     std::vector<int> ptIdxByRadius;
        //     std::vector<float> ptRadius;
        //     kdtree.radiusSearch(src_pt, 1, ptIdxByRadius, ptRadius);
        //     if (ptIdxByRadius.size() == 0) {
        //         return;
        //         // continue;
        //     }
        //     float total1 = 0;
        //     float total2 = 0;
        //     int64_t get_num1 = 0;
        //     int64_t get_num2 = 0;
        //     for (int j = 0; j < ptIdxByRadius.size(); ++j) {
        //         auto &index = ptIdxByRadius[j];
        //         auto &tar_pt = src_cloud->points[index];
        //         if (tar_pt.ring != src_pt.ring) {
        //            continue;
        //         }
        //         if (tar_pt.intensity <= 0) {
        //            // continue;
        //         }
        //         // float dis = pow(ptRadius[j], 0.5);
        //         float dis = ptRadius[j];
        //         if (dis < 0.01) {
        //             total1 += tar_pt.intensity;
        //             ++get_num1;
        //         } else {
        //             total2 += tar_pt.intensity;
        //             ++get_num2;
        //         }

        //     }
        //     if (get_num2 == 0 || get_num1 == 0) {
        //         src_pt.intensity_opt = 0;
        //         LOG_DEBUG("intensity [size={}, raw={}, opt={}", get_num1, src_pt.intensity, 0);
        //         return;
        //         // continue;
        //     }
        //     total1 /= get_num1;
        //     total2 /= get_num2;
        //     total1 = total1 > 99 ? 99 : total1;
        //     total2 = total2 > 99 ? 99 : total2;
        //     float total = total1 - total2;
        //     total = total < 0 ? 0 : total;
        //     // total = pow(total, 2);
        //     total = total > 10 ? 10 : total;
        //     total = total < 8 ? 0 : total;
        //     src_pt.intensity_opt = total;
        //     // src_pt.info = (int)(total1 * 10) + total2 / 100;
        //     LOG_DEBUG("intensity [size1={}, size2={}, raw={}, opt={}, raw1={}, raw2={}",
        //             get_num1, get_num2, src_pt.intensity, total, total1, total2);
        // });
    }
    if (FLAGS_lidar_segmentation_pcd_trans_intensity_enable) {
        session->thread_pool->wait(1, "filter_intensity");
    }
    return valid_size;
}

int RoadModelProcLidarSegmentation::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_lidar_segmentation_save_data_enable) {
        return fsdmap::SUCC;
    }
    if (session->opt_ground_pcd && session->opt_ground_pcd->points.size() > 0) {
        utils::CloudPtr tar_cloud(new pcl::PointCloud<utils::CloudPoint>);
        utils::save_display_pcd(tar_cloud, session->_scope, session->opt_ground_pcd, session->thread_pool);
        std::string pcd_file = session->get_debug_dir("000raw_pcd.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *tar_cloud);
    }
    
    // session->set_display_name("lidar_segmentation");
    // auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
    // log->color = {223, 130, 154};

    // utils::CloudPtr cloud(new pcl::PointCloud<utils::CloudPoint>);
    // // utils::RGBCloudPtr cloud(new pcl::PointCloud<utils::RGBPoint>);
    // for (auto &trail : session->key_pose_map) {
    //     for (auto &key_pose : trail.second.list) {
    //         log->add(key_pose->pos);
    //         if (FLAGS_lidar_segmentation_save_data_pcd_all_enable && key_pose->pcd) {
    //             // utils::save_display_pcd(cloud, session->_scope, key_pose->pcd, session->thread_pool);
    //             session->thread_pool->schedule([&, key_pose](utils::ProcessBar *process_bar) {
    //                 utils::CloudPtr tar_cloud(new pcl::PointCloud<utils::CloudPoint>);
    //                 utils::save_display_pcd(tar_cloud, session->_scope, key_pose->pcd, 
    //                         &key_pose->pos, key_pose->yaw, NULL);
    //                 auto file_name = session->get_debug_dir("clip_pcd_{}.pcd", key_pose->frame_id);
    //                 pcl::io::savePCDFileBinary(file_name, *tar_cloud);
    //                 session->add_vec(cloud->points, tar_cloud->points, true);
    //                 process_bar->num_biz += cloud->points.size();
    //             });
    //         }
    //     }
    // }
    // session->thread_pool->wait(2, "process pcd");
    // if (cloud->points.size() > 0) {
    //     std::string pcd_file = session->get_debug_dir("0raw_pcd.pcd");
    //     pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud);
    // }

    // for (auto &it : session->lane_line_instance_map) {
    //     log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_lane_line[id={}]", it.first);
    //     auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(session->ele_type_map[first_feature->type]);
    //     // if (first_feature->attr.is_double_line) {
    //     //     log->color = {0, 255, 255};
    //     // } else if (first_feature->attr.color == 3) {
    //     //     log->color = {255, 255, 0};
    //     // }
    //     for (auto &feature : it.second.list) {
    //         log->add(feature->pos, -1);
    //     }
    // }
    // for (auto &it : session->barrier_instance_map) {
    //     log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_barrier[id={}]", it.first);
    //     // log->color = {152, 99, 60};
    //     auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(session->ele_type_map[first_feature->type]);
    //     for (auto &feature : it.second.list) {
    //         log->add(feature->pos, -1);
    //     }
    // }
    // for (auto &it : session->curb_instance_map) {
    //     log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_curb[id={}]", it.first);
    //     auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(session->ele_type_map[first_feature->type]);
    //     // log->color = {81, 89, 240};
    //     for (auto &feature : it.second.list) {
    //         log->add(feature->pos, -1);
    //     }
    // }
    // for (auto &it : session->lane_center_instance_map) {
    //     log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_curb[id={}]", it.first);
    //     auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(session->ele_type_map[first_feature->type]);
    //     for (auto &feature : it.second.list) {
    //         log->add(feature->pos, -1);
    //     }
    // }
    // for (auto &it : session->object_instance_map) {
    //     log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_object[id={}]", it.first);
    //     auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(session->ele_type_map[first_feature->type]);
    //     for (auto &pt : it.second.list) {
    //         log->add(pt->pos, -1);
    //     }
    // }
    // session->save_debug_info("lidar_segmentation");
    return fsdmap::SUCC;
}

}
}
