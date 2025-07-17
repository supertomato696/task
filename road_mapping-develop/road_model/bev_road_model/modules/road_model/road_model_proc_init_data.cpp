


#define PCL_NO_PRECOMPILE
#include "road_model_proc_init_data.h"

#include <boost/algorithm/string.hpp>
//add  output origin shp
#include <Eigen/Eigenvalues>
#include <osqp/osqp.h>

#include <gdal_priv.h>
#include <ogrsf_frmts.h>
#include <random>

DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);

DEFINE_bool(init_data_debug_pos_enable, true, "init_data_debug_enable");
DEFINE_bool(init_data_save_data_enable, true, "init_data_save_data_enable");
DEFINE_bool(init_data_save_data_pcd_all_enable, true, "init_data_save_data_pcd_all_enable");
DEFINE_bool(init_data_save_data_save_frame_enable, false, "init_data_save_data_save_frame_enable");

DEFINE_bool(init_data_load_pcd_enable, false, "init_data_load_pcd_enable");
DEFINE_bool(init_data_ground_las, false, "init_data_ground_las");
DEFINE_bool(init_data_filter_key_pose_by_gap_enable, true, "init_data_filter_key_pose_by_gap_enable");
DEFINE_double(init_data_key_pose_min_gap, 4, "init_data_key_pose_min_gap");

DEFINE_bool(origin_data_format, true, "origin_data_format");

#define COLOR_GREEN  "\033[32m"
#define COLOR_RESET "\033[0m"

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcInitData::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_init_data_debug_pos_enable;

    if (session->data_processer == NULL) {
        return fsdmap::process_frame::PROC_STATUS_FAIL;
    }

    session->get_logger()->set_logid(session->data_processer->get_logid());

    if (FLAGS_origin_data_format == false) {
       // 从.json文件中读取车道线信息，存储Link关系上的点，点坐标从wgs转换到统一的（0,0,0)坐标系下，在
        // 统一的（0,0,0)坐标系下计算前后点之间的方向向量，以及点之间的前后连接关系，
        // 输出结果 ： session->raw_links、 session->raw_links_points

        CHECK_FATAL_PROC(session->data_processer->download_link_mnt(session->raw_links, 
                    session->raw_links_points), 
                "download_link");        

        CHECK_FATAL_PROC(session->data_processer->download_key_pose_mnt(session->raw_key_poses,
                    false), 
                "download_key_pose");
        // 从middle.json里面读取十字路口信息，转换中心点到统一的（0,0,0)坐标系下，
        // 输出结果 ：session->raw_intersections
        // CHECK_FATAL_PROC(session->data_processer->download_intersection_mnt(session->raw_intersections), 
        //         "download_intersection");                


        CHECK_FATAL_PROC(session->data_processer->download_feature_mnt(session->raw_features), 
                "download_feature");        
                
        // 读取道路边界
        CHECK_FATAL_PROC(session->data_processer->download_boundary_line_mnt(session->raw_boundary_feature),
                "download_boundary_line");        

        // 读取车道线信息
        CHECK_FATAL_PROC(session->data_processer->download_lane_line_mmt(session->raw_lane_feature),
                "download_lane_line");

        // 从人工标注的信息里面读取车道线信息和道路边界
        CHECK_FATAL_PROC(session->data_processer->download_feature_from_sem_client(
                    session->raw_client_lane_feature, session->raw_client_boundary_feature),
                "download_client_feature");

        // 读取目标物体，目标包含点云以及类型等属性
        CHECK_FATAL_PROC(session->data_processer->download_object_mmt(
                    session->raw_object_ret_list),
                "download_raw_object_ret_list");

        // 读取路口边界以及不可通行区域
        CHECK_FATAL_PROC(session->data_processer->download_junction_mmt(
                    session->new_junction_list, session->new_area_list),
                "download_raw_object_junction_list");  
    } else {
        // 从.json文件中读取车道线信息，存储Link关系上的点，点坐标从wgs转换到统一的（0,0,0)坐标系下，在
        // 统一的（0,0,0)坐标系下计算前后点之间的方向向量，以及点之间的前后连接关系，
        // 输出结果 ： session->raw_links、 session->raw_links_points
        CHECK_FATAL_PROC(session->data_processer->download_link(session->raw_links, 
                    session->raw_links_points, session->task_polygon, session->task_polygon_vec3d), 
                "download_link");

        // 从middle.json里面读取十字路口信息，转换中心点到统一的（0,0,0)坐标系下，
        // 输出结果 ：session->raw_intersections
        CHECK_FATAL_PROC(session->data_processer->download_intersection(session->raw_intersections), 
                "download_intersection");

        // CHECK_FATAL_PROC(session->data_processer->download_key_pose(session->raw_key_poses,
        //             false), 
        //         "download_key_pose");
        CHECK_FATAL_PROC(session->data_processer->download_key_pose_mnt(session->raw_key_poses,
                    false), 
                "download_key_pose");
            

        CHECK_FATAL_PROC(session->data_processer->download_feature(session->raw_features), 
                "download_feature");
      
        // 读取道路边界
        // CHECK_FATAL_PROC(session->data_processer->download_site_boundary_line(session->raw_site_boundary_feature),
        CHECK_FATAL_PROC(session->data_processer->download_site_boundary_line(session->raw_boundary_feature),
                "download_site_boundary_line");     

        CHECK_FATAL_PROC(session->data_processer->download_boundary_line(session->raw_boundary_feature),
                "download_boundary_line");     

        // 读取车道线信息
        CHECK_FATAL_PROC(session->data_processer->download_lane_line(session->raw_lane_feature),
                "download_lane_line");

        // 读取车道中心信息
        CHECK_FATAL_PROC(session->data_processer->download_lane_center(session->raw_features),
                "download_lane_center");

        // 从人工标注的信息里面读取车道线信息和道路边界
        CHECK_FATAL_PROC(session->data_processer->download_feature_from_sem_client(
                    session->raw_client_lane_feature, session->raw_client_boundary_feature),
                "download_client_feature");
        
        // 读取目标物体，目标包含点云以及类型等属性
        CHECK_FATAL_PROC(session->data_processer->download_object(
                    session->raw_object_ret_list),
                "download_raw_object_ret_list");

        // 读取路口边界以及不可通行区域
        CHECK_FATAL_PROC(session->data_processer->download_junction(
                    session->new_junction_list, session->new_area_list),
                "download_raw_object_junction_list");

        // 读取link  匹配关系
        CHECK_FATAL_PROC(session->data_processer->download_match_info(
                    session->link_info_ptr, session->link_infos),
                "download_link_info");
    }
    
    if (FLAGS_init_data_load_pcd_enable) {
        CHECK_FATAL_PROC(session->data_processer->download_ground_pcd(session->raw_ground_pcd,
                    FLAGS_init_data_ground_las), 
                "download_groud_pcd");
    }
    

    CHECK_FATAL_PROC(init_param(session), "init_param");

    CHECK_FATAL_PROC(init_scope_data(session), "init_scope_data");

    CHECK_FATAL_PROC(init_trail(session), "init_trail");

    CHECK_FATAL_PROC(init_link(session), "init_link");

    CHECK_FATAL_PROC(init_feature_line(session), "init_feature_line");

    CHECK_FATAL_PROC(init_semantic_line(session), "init_semantic_line");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");

    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcInitData::init_param(RoadModelSessionData* session) {
    session->double_geo_map[4] = 1;
    session->double_geo_map[5] = 1;
    session->double_geo_map[6] = 1;
    session->double_geo_map[7] = 1;
    return fsdmap::SUCC;
}

int RoadModelProcInitData::init_link(RoadModelSessionData* session) {
    for (auto &inter : session->raw_intersections) {
        session->intersections.push_back(inter.get());
    }
    return fsdmap::SUCC;
}

int RoadModelProcInitData::init_trail(RoadModelSessionData* session) {
    double min_gap = FLAGS_init_data_key_pose_min_gap;
    for (auto &key_pose : session->raw_key_poses) { // 把raw_key_poses的结果放入key_pose_map
        if (MAP_NOT_FIND(session->key_pose_map, key_pose->line_id)) {
            session->key_pose_map[key_pose->line_id] = KeyPoseLine();
        }
        // LOG_INFO("line id:{}",key_pose->line_id);
        session->key_pose_map[key_pose->line_id].list.push_back(key_pose.get());
    }
    // 按照FLAGS_init_data_key_pose_min_gap的间距做稀疏化,没有直接做稀疏，只是修改了filter_status状态位等于2 
    if (FLAGS_init_data_filter_key_pose_by_gap_enable) {
        for (auto &key_line : session->key_pose_map) {
            KeyPose* prev_poss = NULL;
            for (auto &poss : key_line.second.list) {
                if (prev_poss == NULL) {
                    prev_poss = poss;
                    continue;
                }
                double dis = alg::calc_dis(prev_poss->pos, poss->pos);
                if (dis < min_gap) {
                    poss->filter_status = 2;
                    continue;
                }
                prev_poss = poss;
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcInitData::init_feature_line(RoadModelSessionData* session) {
    for (auto &feature : session->raw_features) {
        // LOG_DEBUG("debug_feature [id={}]", feature->frame_id);
        // if (feature->key_pose != NULL && feature->key_pose->invalid()) {
        //     if (feature->frame_id == "6171675065805_1675065842684000") {
        //         int a = 1;
        //     }
        //     LOG_DEBUG("filter feature by key_pose[id={}]", feature->frame_id);
        //     continue;
        // }   
        auto &ele_meta = feature->ele_type;
        auto ele_type = std::get<0>(ele_meta); // bev感知类型
        if (ele_type == ELEMENT_LANE_LINE) {
            // 把feature信息放到lane_line_feature_ptr和映射到lane_line_instance_map里
            auto ele = feature_to_line(session, session->lane_line_feature_ptr, 
                    session->lane_line_instance_map, feature.get());
            ele->attr.color = std::get<1>(ele_meta);
            ele->attr.geo = std::get<2>(ele_meta);
            if (MAP_FIND(session->double_geo_map, ele->attr.geo)) {
                ele->attr.is_double_line = 1;
            }
            ele->src_status = 1;
        } else if (ele_type == ELEMENT_BARRIER) {
            // 栅栏信息放入boundary_feature_ptr和barrier_instance_map
            auto ele = feature_to_line(session, session->boundary_feature_ptr, 
                    session->barrier_instance_map, feature.get());
            ele->sub_type = std::get<1>(ele_meta);
            ele->src_status = 1;

        } else if (ele_type == ELEMENT_CURB) {
            // 路沿信息放入boundary_feature_ptr和curb_instance_map
            auto ele = feature_to_line(session, session->boundary_feature_ptr, 
                    session->curb_instance_map, feature.get());
            ele->sub_type = std::get<1>(ele_meta);
            ele->src_status = 1;
        } else if (ele_type == ELEMENT_LANE_CENTER) {
            // 车道中心线信息存入lane_center_feature_ptr和lane_center_instance_map
            auto ele = feature_to_line(session, session->lane_center_feature_ptr, 
                    session->lane_center_instance_map, feature.get());
        } else if (ele_type == ELEMENT_VIRTUAL_LANE_LINE) {
            // 存储虚线信息到virtual_lane_center_feature_ptr和virtual_lane_center_instance_map
            auto ele = feature_to_line(session, session->virtual_lane_center_feature_ptr, 
                    session->virtual_lane_center_instance_map, feature.get());
        } 
        else if (ele_type == ELEMENT_JUNCTION) {
            // 存储交叉入口信信息
            // TODO:qzc may wrong
            // auto ele = feature_to_line(session, session->raw_intersections, 
            auto ele = feature_to_line(session, session->boundary_feature_ptr, 
                    session->junction_instance_map, feature.get());
            ele->sub_type = std::get<1>(ele_meta);
            ele->src_status = 1;
        } else if (ele_type == ELEMENT_OBJECT) {
            auto &object = session->object_instance_map[feature->line_id];
            object.list.push_back(feature->pos);
            object.ele_type = ele_meta;
            object.type = std::get<1>(ele_meta);
            object.frame_id = feature->frame_id;
            object.key_pose = feature->key_pose;
            object.frame_id = feature->frame_id;
        }
    }
    for (auto &feature : session->raw_client_lane_feature) {
        if (feature->key_pose != NULL && feature->key_pose->invalid()) {
            LOG_DEBUG("filter feature by key_pose[id={}]", feature->frame_id);
            continue;
        }
        feature->src_status = 4;
        if (MAP_FIND(session->double_geo_map, feature->attr.geo)) {
            feature->attr.is_double_line = 1;
        }
        feature_to_line(session, session->label_lane_line_instance_map, feature.get());
    }
    for (auto &line : session->label_lane_line_instance_map) {
        SORT(line.second.list,
                [](const LaneFeature* l, const LaneFeature* r)->bool{
                    return l->line_index < r->line_index;
                });
        for (int i = 1; i < line.second.list.size(); ++i) {
            auto prev = line.second.list[i - 1];
            auto curr = line.second.list[i];
            prev->next = curr;
            curr->prev = prev;
            prev->dir = alg::get_dir(curr->pos, prev->pos);
            curr->dir = prev->dir;
        }
    }
    for (auto &feature : session->raw_client_boundary_feature) {
        if (feature->key_pose != NULL && feature->key_pose->invalid()) {
            LOG_DEBUG("filter feature by key_pose[id={}]", feature->frame_id);
            continue;
        }
        feature->src_status = 4;
        feature_to_line(session, session->label_boundary_instance_map, feature.get());
    }
    for (auto &line : session->label_boundary_instance_map) {
        SORT(line.second.list,
                [](const BoundaryFeature* l, const BoundaryFeature* r)->bool{
                    return l->line_index < r->line_index;
                });
        for (int i = 1; i < line.second.list.size(); ++i) {
            auto prev = line.second.list[i - 1];
            auto curr = line.second.list[i];
            prev->next = curr;
            curr->prev = prev;
            prev->dir = alg::get_dir(curr->pos, prev->pos);
            curr->dir = prev->dir;
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcInitData::init_semantic_line(RoadModelSessionData* session) {
    for (auto &feature : session->raw_lane_feature) {
        // LOG_INFO("lane init color:{} type:{}",feature->color,feature->type)
        auto ele = feature_to_line(session, session->lane_line_feature_ptr, 
                session->sem_lane_instance_map, feature.get());
        ele->src_status = 1;
    }
    for (auto &feature : session->raw_boundary_feature) {
        auto ele = feature_to_line(session, session->boundary_feature_ptr, 
                session->sem_curb_instance_map, feature.get());
        ele->src_status = 1;
    }
    // for (auto &feature : session->raw_site_boundary_feature) {
    //     auto ele = feature_to_line(session, session->site_boundary_feature_ptr, 
    //             session->site_boundary_instance_map, feature.get());
    //     ele->src_status = 1;
    // }
    return fsdmap::SUCC;
}

int RoadModelProcInitData::init_scope_data(RoadModelSessionData* session) {
    session->_scope = session->data_processer->get_display_scope("");
    return fsdmap::SUCC;
}
 uint8_t RoadModelProcInitData::convert_arrow_type(const std::string& str)
 {
    uint8_t max_type_size=7+1;
    char delimiter = ',';
    uint8_t ret=0;
    std::vector<std::string> tokens;
    // 使用Boost的split函数进行字符串分割
    boost::split(tokens, str, boost::is_any_of(&delimiter));
    // for (const std::string& token : tokens) {
    //     std::cout <<std::stoi( token) << std::endl;
    // }

    for(int i=0;i<tokens.size();i++)
    {
        // LOG_INFO("tokens[{}]:{}", i,  tokens[i]);
        if(tokens[i].empty()) {
            continue;
        }
        ret+=(max_type_size*i+std::stoi(tokens[i]));
    }
    return ret;
 }

int RoadModelProcInitData::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_init_data_save_data_enable) {
        return fsdmap::SUCC;
    }

    std::random_device rd;  // 随机数种子
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1000);

    double scope_buff = FLAGS_display_scope_buff;
    PTR_VEC<utils::DisplayInfo> log_ptr_list;
    UMAP<std::string, std::vector<utils::DisplayInfo*>> frame_map;
    session->set_display_name("raw_trail");
    UMAP<std::string, std::vector<utils::DisplayInfo*>> trail_map;
    for (auto &trail : session->key_pose_map) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        auto trail_ptr = &trail;
        // session->thread_pool->schedule(
        //         [log, trail_ptr, session, this](utils::ProcessBar *process_bar) {
        log->color = {223, 130, 154};
        auto &first_poss = trail_ptr->second.list.front();
        for (auto &key_pose : trail_ptr->second.list) {
            auto &ele = log->add(key_pose->pos, 1, key_pose->raw_no);
            ele.label.opt_label = key_pose->raw_file_no;
            int random_number = dis(gen);
            ele.label.intensity_opt =random_number ;
            ele.label.score=alg::calc_theta(key_pose->dir);
            // ele.label.time = key_pose->timestamp;
        }
        // std::vector<utils::DisplayInfo*> tmp_list;
        // tmp_list.push_back(log);

        // auto log_name = session->get_debug_dir("raw_trail_detail/raw_{}_{}.png", 
        //         first_poss->raw_file_no, first_poss->trail_id);
        // utils::save_display_image(log_name.c_str(), session->_scope, tmp_list);

        // auto pcd_name = session->get_debug_dir("raw_trail_detail/raw_{}_{}.pcd",
        //         first_poss->raw_file_no, first_poss->trail_id);
        // utils::save_display_pcd(pcd_name.c_str(), session->_scope, tmp_list);


        // });
    }

    session->thread_pool->wait(2, "save_raw_key_pose");
    session->save_debug_info("raw_trail");
    
    session->set_display_name("raw_link");
    for (auto &link : session->raw_links) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {223, 0, 154};
        auto log_point = session->add_debug_log(utils::DisplayInfo::POINT, "raw_key_pose_point");
        if(!link->list.empty())
        {
            auto &ele = log->add(link->list.front()->pos);
              // 生成随机数
            int random_number = dis(gen);
            ele.label.intensity_opt=random_number;
        }
        int i=0;
        for (auto &pt : link->list) {
            auto &ele = log->add(pt->pos);
            ele.label.label = i++;
              // 生成随机数
            int random_number = dis(gen);
            ele.label.intensity_opt=random_number;
            ele.label.score=alg::calc_theta(pt->dir);
        }
    }
    for (auto &inter : session->intersections) {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "inter");
        log->color = {255, 0, 0};
        log->add(inter->pos);
    }
    session->save_debug_info("raw_link");

    // session->set_display_name("raw_data_lane_line_bev");
    // for (auto &it : session->lane_line_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::POINT, "raw_lane_line[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     log->color = std::get<3>(first_feature->src_feature->ele_type);
    //     // std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     //
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         ele.label.cloud_bev_label = feature->type;
    //         ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
    //         ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         // auto log_frame = session->add_ptr(log_ptr_list);
    //         // log_frame->init(log);
    //         // frame_map[frame_id].push_back(log_frame.get());
    //         frame_map[frame_id].push_back(log);
    //     }
    // } 
    
    // for (auto &it : session->lane_center_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_curb[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     // auto &first_feature = it.second.list.front();
    //     log->color = std::get<3>(first_feature->src_feature->ele_type);
    //     // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.label = feature->type;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         double h_dis = alg::calc_hori_dis(feature->pos, 
    //                 feature->key_pose->pos, feature->key_pose->dir);
    //         ele.label.intensity_opt = h_dis;
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         //auto log_frame = session->add_ptr(log_ptr_list);
    //         //log_frame->init(log);
    //         // frame_map[frame_id].push_back(log_frame.get());
    //         frame_map[frame_id].push_back(log);
    //     }
    // }
    // session->thread_pool->wait(1, "save_raw_lane_line_bev");
    // session->save_debug_info("raw_data_lane_line_bev");
    
    session->set_display_name("raw_data_lane_line_sem");
    for (auto &it : session->sem_lane_instance_map) {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "raw_lane_line[id={}]", it.first);
        auto ins_ptr = &it.second;
        session->thread_pool->schedule(
                [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        auto &first_feature = ins_ptr->list.front();
        // auto &first_feature = it.second.list.front();
        // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
        log->color = {0, 255, 0};
        for (auto &feature : ins_ptr->list) {
            auto &ele = log->add(feature->pos, -1, feature->type);
            ele.label.score = feature->score;

            switch(feature->color)
            {
               case 2:
               {
                ele.color={255, 255, 255}; //白色
                break;
               }
               case 3:
               {
                ele.color={255, 140, 0}; //黄色
               
                break;
               }
               default:
               ele.color={0, 255, 0};
            }
            
            ele.label.cloud_pano_seg=feature->type;
            ele.label.cloud_line_seg=feature->color;
            // LOG_INFO("lane init display  color:{} type:{} [{} {}]",
            //     feature->attr.color,feature->attr.type,
            //     feature->color,feature->type
            // )
        }
        });
    }
    session->thread_pool->wait(1, "save_raw_lane_line_sem");
    session->save_debug_info("raw_data_lane_line_sem");

    // session->set_display_name("raw_data_intersection_lane_line_center_bev");
    // for (auto &it : session->virtual_lane_center_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_intersection_lane_line_center[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     log->color = {217, 77, 255};
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->type);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         //auto log_frame = session->add_ptr(log_ptr_list);
    //         //log_frame->init(log);
    //         // frame_map[frame_id].push_back(log_frame.get());
    //         frame_map[frame_id].push_back(log);
    //     }
    // }
    // session->thread_pool->wait(1, "save_raw_data_intersection_lane_line_center_bev");
    // session->save_debug_info("raw_data_intersection_lane_line_center_bev");

    // session->set_display_name("raw_data_lane_line_label");
    // for (auto &it : session->label_lane_line_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_lane_line[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     if (MAP_FIND(session->double_geo_map, first_feature->attr.geo)) {
    //         log->color = {100, 100, 100};
    //     // } else if (first_feature->attr.color == 3) {
    //     //    log->color = {255, 255, 100};
    //     }
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->attr.geo);
    //         ele.label.opt_label = feature->src_status;
    //         ele.label.score = feature->raw_pos.x();
    //     }
    //     });
    // }
    // session->thread_pool->wait(1, "save_raw_lane_line_label");
    // session->save_debug_info("raw_data_lane_line_label");
    // session->set_display_name("raw_data_boundary_bev");
    // for (auto &it : session->junction_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_barrier[id={}]", it.first);
    //     // log->color = {152, 99, 60};
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     log->color = std::get<3>(first_feature->src_feature->ele_type);
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         ele.label.cloud_bev_label = feature->type;
    //         ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
    //         ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         frame_map[frame_id].push_back(log);
    //     }
    // }
    // for (auto &it : session->barrier_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_barrier[id={}]", it.first);
    //     // log->color = {152, 99, 60};
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     log->color = std::get<3>(first_feature->src_feature->ele_type);
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         ele.label.cloud_bev_label = feature->type;
    //         ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
    //         ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         frame_map[frame_id].push_back(log);
    //     }
    // }

    // for (auto &it : session->curb_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_curb[id={}]", it.first);
    //     // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     log->color = std::get<3>(first_feature->src_feature->ele_type);
    //     // log->color = {81, 89, 240};
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         ele.label.cloud_bev_label = feature->type;
    //         ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
    //         ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         frame_map[frame_id].push_back(log);
    //     }
    // }
    // session->thread_pool->wait(1, "save_raw_boundary_bev");
    // session->save_debug_info("raw_data_boundary_bev");
     srand48(time(NULL));
    session->set_display_name("raw_data_boundary_sem");
    for (auto &it : session->sem_curb_instance_map) {
        auto log = session->add_debug_log(utils::DisplayInfo::POINT, "raw_sem_curb[id={}]", it.first);
        auto line_log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_sem_curb {}",it.first);
        // auto line_log = session->add_debug_log(utils::DisplayInfo::POINT, "raw_sem_curb {}",it.first);
        auto ins_ptr = &it.second;
        session->thread_pool->schedule(
                [log, ins_ptr, session, line_log,this](utils::ProcessBar *process_bar) {
        auto &first_feature = ins_ptr->list.front();
        // log->color = {81, 89, 240};
        int label= 1000*drand48();
        for (auto &feature : ins_ptr->list) {
            // auto &ele = log->add(feature->pos, -1, feature->type);
            // ele.label.label =label;
            // ele.label.score = feature->score;
            // ele.label.opt_label = feature->src_status;
        }
        BoundaryFeature* prev=NULL;
        // std::cout << "ins_ptr->boundary_type " << ins_ptr->boundary_type << std::endl;
        for (auto &feature : ins_ptr->list) {
            if(prev)
            {
                auto &ele = line_log->add(prev->pos, -1, prev->type);
                ele.label.label =label;
                ele.label.score = prev->score;
                ele.label.opt_label = prev->src_status;
                ele.label.cloud_pano_seg=prev->type;
                ele.label.cloud_line_seg=prev->color;
                auto &next_ele = line_log->add(feature->pos, -1, prev->type);

                switch(ins_ptr->boundary_type) {
                    case 1: {
                        ele.color={0, 255, 0}; // 绿色
                        next_ele.color={0, 255, 0}; // 绿色
                        break;
                    }
                    default:
                        ele.color={255, 255, 255};
                        next_ele.color={255, 255, 255};
                }
            }
            prev=feature;

        }
        });

    }
    session->thread_pool->wait(1, "save_raw_boundary_sem");
    session->save_debug_info("raw_data_boundary_sem");
    // session->set_display_name("raw_data_boundary_label");
    // for (auto &it : session->label_boundary_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_sem_curb[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     log->color = {81, 89, 240};
    //     for (auto &feature : ins_ptr->list) {
    //         auto &ele = log->add(feature->pos, 1, feature->src_status);
    //         ele.label.opt_label = feature->sub_type;
    //     }
    //     });
    // }
    // session->thread_pool->wait(1, "save_raw_boundary_label");
    // session->save_debug_info("raw_data_boundary_label");
    // session->set_display_name("raw_data_obj_bev");
    // for (auto &it : session->object_instance_map) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_object[id={}]", it.first);
    //     auto ins_ptr = &it.second;
    //     session->thread_pool->schedule(
    //             [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
    //     auto &first_feature = ins_ptr->list.front();
    //     // log->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
    //     log->color = std::get<3>(ins_ptr->ele_type);
    //     auto &feature = ins_ptr;
    //     for (auto &pos : feature->list) {
    //         auto &ele = log->add(pos, -1, feature->key_pose->raw_no);
    //         ele.label.score = feature->score;
    //         ele.label.opt_label = feature->key_pose->raw_file_no;
    //         ele.label.cloud_bev_label = feature->type;
    //         ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
    //         ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
    //     }
    //     });
    //     if (FLAGS_init_data_save_data_save_frame_enable) {
    //         auto &frame_id = ins_ptr->frame_id;
    //         frame_map[frame_id].push_back(log);
    //     }
    // }
    // session->thread_pool->wait(1, "save_raw_object_bev");
    // session->save_debug_info("raw_data_obj_bev");
    session->set_display_name("raw_data_obj_sem");
    
    for (auto &obj : session->raw_object_ret_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_object");
        // session->thread_pool->schedule(
        //         [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        if (obj->ele_type == 3 || obj->ele_type == 4) { // 箭头和其他地面标识
            log->color = {0, 0, 255};
        } else if (obj->ele_type == 5) { // 人行横道
            log->color = {0, 255, 0};
        } else if (obj->ele_type == 6) { // 停止线
            log->color = {255, 0, 0};
        } else {
            log->color = {255, 255, 0};
        }
        
        for (auto &pt : obj->list) {
            auto &ele = log->add(pt->pos, 1, obj->ele_type);
            ele.label.score = obj->score;

            // LOG_INFO("obj.type:{}", obj->type);
            ele.label.opt_label = convert_arrow_type(obj->type);
            // / ele.label.cloud_bev_label = feature->type;
            // / ele.label.distance_x_cm = int(feature->raw_pos.x() * 100);
            // / ele.label.distance_y_cm = int(feature->raw_pos.y() * 100);
        }
        // });
    }
    for (auto &obj : session->new_junction_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "raw_junction");
        // session->thread_pool->schedule(
        //         [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        log->color = {255, 255, 255};
        for (auto &pt : obj->point_info) {
            auto &ele = log->add(pt->pos, 1, obj->ele_type);
            // std::cout << COLOR_GREEN << "原始的" << pt->pos << COLOR_RESET << "\n" << std::endl;    
        }
        // });
    }
    for (auto &obj : session->new_area_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_area");
        // session->thread_pool->schedule(
        //         [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        log->color = {255, 0, 255};
        for (auto &pt : obj->list) {
            auto &ele = log->add(pt->pos, 1, obj->ele_type);
        }
        // });
    }
    // session->thread_pool->wait(1, "save_raw_object_bev");
    session->save_debug_info("raw_data_obj_sem");

    
    //add
    session->set_display_name("raw_data_intersection");//路口
    for (auto &obj : session->raw_intersections) {
        auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "raw_intersection");
        // session->thread_pool->schedule(
        //         [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        log->color = {255, 255, 255};
        for (auto &pt : obj->point_info) {
            auto &ele = log->add(pt->pos, 1, obj->ele_type);
            // std::cout << COLOR_GREEN << "原始的" << pt->pos << COLOR_RESET << "\n" << std::endl;    
        }
        // });
    }
    session->save_debug_info("raw_data_intersection");



    session->set_display_name("raw_data_junction");//路口
    for (auto &obj : session->new_junction_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::POLYGEN, "raw_junction");
        // session->thread_pool->schedule(
        //         [log, ins_ptr, session, this](utils::ProcessBar *process_bar) {
        log->color = {255, 255, 255};
        for (auto &pt : obj->point_info) {
            auto &ele = log->add(pt->pos, 1, obj->ele_type);
            // std::cout << COLOR_GREEN << "原始的" << pt->pos << COLOR_RESET << "\n" << std::endl;    
        }
        // });
    }
    session->save_debug_info("raw_data_junction");

    // session->set_display_name("raw_proc_polygen");
    // for (auto &poly : session->data_processer->_valid_polygen) {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "polygen");
    //     for (auto &pt_gis : poly.outer()) {
    //         Eigen::Vector3d pt = {pt_gis.get<0>(), pt_gis.get<1>(), 0};
    //         auto &ele = log->add(pt, -1);
    //     }
    // }
    // session->save_debug_info("raw_proc_polygen");
    // if (FLAGS_init_data_save_data_save_frame_enable) {
    //     // for (auto it = frame_map.begin(); it != frame_map.end(); ++it) {
    //     for (auto &[k, v] : frame_map) {
    //         auto key_pose = session->data_processer->get_key_pose(k);
    //         if (key_pose == NULL) {
    //             continue;
    //         }
    //         auto log_list_ptr = &v;
    //         // utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
    //         // box.set_resolution(FLAGS_display_scale_rate);
    //         // auto log_name = session->get_debug_dir("raw_bev_detail/raw_bev_{}_{}_{}.png", 
    //         //         key_pose->raw_file_no, key_pose->raw_no,
    //         //         key_pose->frame_id);
    //         // utils::save_display_image(log_name.c_str(), box, *log_list_ptr);
    //         session->thread_pool->schedule(
    //                [scope_buff, key_pose, log_list_ptr, session, this](utils::ProcessBar *process_bar) {
    //                 utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
    //                 box.set_resolution(FLAGS_display_scale_rate);
    //                 auto log_name = session->get_debug_dir("raw_bev_detail/raw_bev_{}_{}_{}.png", 
    //                         key_pose->raw_file_no, key_pose->raw_no,
    //                         key_pose->frame_id);
    //                 utils::save_display_image(log_name.c_str(), box, *log_list_ptr);
    //                 auto pcd_name = session->get_debug_dir("raw_bev_detail/raw_bev_{}_{}_{}.pcd", 
    //                         key_pose->raw_file_no, key_pose->raw_no,
    //                         key_pose->frame_id);
    //                 utils::save_display_pcd(pcd_name.c_str(), session->_scope, *log_list_ptr);
    //         });
    //     }
    // }
    // session->thread_pool->wait(2, "save_raw_bev_frame");
    return fsdmap::SUCC;
}
}
}
