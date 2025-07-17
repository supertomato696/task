


#include "road_model_proc_trans_interface.h"
#include "dao/upload2RoadServer.h"
#include "dao/data_processer.h"


DEFINE_bool(trans_interface_upload_enable, true, "trans_interface_upload_enable");
DEFINE_bool(trans_interface_debug_pos_enable, true, "trans_interface_debug_enable");
DEFINE_bool(trans_interface_save_data_enable, true, "trans_interface_save_data_enable");

DEFINE_string(road_server_address, "172.21.207.124:8081", "road_server_address");
DEFINE_string(editor, "xxx", "bev_uploader_name");
DEFINE_string(road_branch, "bev_upload_test", "road_branch");
DEFINE_string(dataOut, "/home/xx/uploadtest/", "obj_output_file");

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcTransInterface::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_trans_interface_debug_pos_enable;

    if (!FLAGS_trans_interface_upload_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
     
    // 车道线
    // session->new_lane_groups
    // 道路边界
    // session->new_boundary_segment
    // 点要素
    // session->new_object_list;
    //路口
    //session->new_junction_list
    dao::DataProcessorBase* dp = session->data_processer;

    for (int i = 0; i < session->raw_object_ret_list.size(); i++)
    {
        if(session->raw_object_ret_list[i]->ele_type == 3 || session->raw_object_ret_list[i]->ele_type == 4)
        {
            upload2RoadServer::testCreateTrafficInfoProxy(*(session->raw_object_ret_list[i]), dp);
        }
        else if(session->raw_object_ret_list[i]->ele_type == 5 || session->raw_object_ret_list[i]->ele_type == 6)
        {
            upload2RoadServer::testCreatPositionProxy(*(session->raw_object_ret_list[i]), dp);
        }
        // else if(session->raw_object_ret_list[i]->ele_type == 7)
        // {
        //     upload2RoadServer::testCreatJunctionProxy(*(session->raw_object_ret_list[i]), dp);
        // }
    }
    for (int i = 0; i < session->new_junction_list.size(); i++)
    {
        upload2RoadServer::testCreatJunctionProxy(*(session->new_junction_list[i]), dp);
    }
    
    for (int i = 0; i < session->new_lane_groups.size(); ++i) {
        auto &lg = session->new_lane_groups[i];
        if (lg->lane_line_info.size() == 0) {
            VEC_ERASE(session->new_lane_groups, i);
        }
    }
    
    upload2RoadServer::testCreatRoadBoundaryProxyAll(session->new_boundary_segment, dp);
    upload2RoadServer::testCreatLaneGroupProxyAll(session->new_lane_groups, dp);
    
    
    upload2RoadServer::testupLoadTiles(FLAGS_road_server_address,FLAGS_road_branch, FLAGS_editor, FLAGS_dataOut);

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}



int RoadModelProcTransInterface::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_trans_interface_save_data_enable) {
        return fsdmap::SUCC;
    }
    // session->set_display_name("trans_interface");
    // auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
    // log->color = {223, 130, 154};

    // utils::CloudPtr cloud(new pcl::PointCloud<utils::CloudPoint>);
    // utils::CloudPtr cloud1(new pcl::PointCloud<utils::CloudPoint>);
    // // utils::RGBCloudPtr cloud(new pcl::PointCloud<utils::RGBPoint>);
    // for (auto &key_pose : session->raw_key_poses) {
    //     log->add(key_pose->pos);
    //     if (FLAGS_trans_interface_save_data_pcd_all_enable && key_pose->pcd) {
    //         // utils::save_display_pcd(cloud, session->_scope, key_pose->pcd, session->thread_pool);
    //         utils::save_display_pcd(cloud, session->_scope, key_pose->pcd, NULL);
    //     }
    // }
    // if (cloud->points.size() > 0) {
    //     pcl::KdTreeFLANN<utils::CloudPoint> kdtree;
    //     kdtree.setInputCloud(cloud);
    //     VEC_PUSH_ALL(cloud1->points, cloud->points);

    //     for (int i = 0; i < cloud->points.size(); ++i) {
    //         session->thread_pool->schedule([&, i, this](utils::ProcessBar *process_bar) {
    //         std::vector<int> ptIdxByRadius;   
    //         std::vector<float> ptRadius;
    //         utils::CloudPoint &pSearch = cloud->points[i];
    //         kdtree.radiusSearch(pSearch, 1, ptIdxByRadius, ptRadius);
    //         if (ptIdxByRadius.size() == 0) {
    //         return;
    //         }
    //         float total = 0;
    //         for (auto &j : ptIdxByRadius) {
    //             total += cloud->points[j].intensity;
    //         }
    //         total /= ptIdxByRadius.size();
    //         total -= pSearch.intensity;
    //         total = fabs(total) * 10;
    //         cloud1->points[i].intensity = total;
    //         });
    //     }
    //     session->thread_pool->wait(2, "process intensity");

    //     std::string pcd_file = session->get_debug_dir("raw_pcd.pcd");
    //     pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud1);
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
    // session->save_debug_info("trans_interface");
    return fsdmap::SUCC;
}

}
}
