

#include "road_model_proc_fix_object.h"

DEFINE_bool(fix_stopline_geom_enable, true, "fix_stopline_geom_enable");
DEFINE_bool(fix_object_save_data_enable, true, "fix_object_save_data_enable");
DEFINE_double(search_lane_dir_dis, 10, "search_lane_dir_dis"); //往后搜10m


namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcFixObject::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_fix_stopline_geom_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
 
    // CHECK_FATAL_PROC(fix_stopline_geom(session), "fix_stopline_geom");

    CHECK_FATAL_PROC(fix_stopline_geom2(session), "fix_stopline_geom");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}


//方案1：   TODO: cxf ，优化split obj， 绑定停止线的逻辑，用上轨迹穿过的比例
int RoadModelProcFixObject::fix_stopline_geom(RoadModelSessionData* session) {
    if (!FLAGS_fix_stopline_geom_enable) {
        return fsdmap::SUCC;
    }

    for (auto& object : session->raw_object_ret_list) {
        if (object->ele_type != 6) {
            continue;
        }

        // std::cout << "object->bind_links.size() : " << object->bind_links.size() << std::endl;

        if (object->bind_links.size() == 1) {
            auto& keypose = object->bind_links[0];

            // 遍历 keypose->road_segment 对应 20m 内的 keypose，从后往前遍历
            Eigen::Vector3d dir_avg(0, 0, 0);
            int count = 0;
            auto& pos_sample_list = keypose->road_segment->pos_sample_list;
            auto it = std::find(pos_sample_list.begin(), pos_sample_list.end(), keypose);
            if (it == pos_sample_list.end()){
                continue;
            }

            for (; it != pos_sample_list.begin(); --it) {
                double dis = alg::calc_dis(keypose->pos, (*it)->pos, true);
                // std::cout << "(*it)->filled_lane_sample.size() : " << (*it)->filled_lane_sample.size() << std::endl;
                if (dis <= FLAGS_search_lane_dir_dis) {
                    for (auto& lc : (*it)->filled_lane_sample) {
                        // std::cout << "lc->road_index : " << lc->road_index << std::endl;
                        if(lc->road_index == 0) {
                            dir_avg += lc->dir;
                            count++;
                        } 
                    }
                }
                else {
                    break;
                }
            }
   
            if (count > 0) {
                dir_avg /= count; 

                auto v_dir = alg::get_vertical_dir(dir_avg);
                bool flag_theta = (alg::calc_theta(v_dir, object->dir, true, true) > 90);
                bool flag_theta2 = (alg::calc_theta2(v_dir, object->dir, true, true) > 90);
                std::cout << "flag_theta : " << flag_theta << " flag_theta2: " << flag_theta2 << std::endl;

                if (alg::calc_theta(v_dir, object->dir, true, true) > 90) {
                    v_dir = -v_dir;
                }

                // 修改停止线几何
                if (object->list.size() == 2) {
                    Eigen::Vector3d center = object->pos;
                    Eigen::Vector3d start = object->list[0]->pos;
                    Eigen::Vector3d end = object->list[1]->pos;

                    double theta = -(alg::calc_theta2(object->dir, v_dir, false, true));
                    // double theta = alg::calc_theta1(v_dir, object->dir, false, true);
                    
                    //异常值，过滤
                    // if(theta > 90 || theta < -90){
                    //     continue;
                    // }

                    Eigen::Vector3d rotated_start = alg::rotate_vector(start - center, theta) + center;
                    Eigen::Vector3d rotated_end = alg::rotate_vector(end - center, theta) + center;
                    
                    if(1){
                        object->list[0]->pos = rotated_start;
                        object->list[1]->pos = rotated_end;
                        object->list[0]->raw_pos = rotated_start;
                        object->list[1]->raw_pos = rotated_end;

                        std::cout << "start : " << start << " rotated_start: "<< rotated_start << std::endl;
                        std::cout << "end : " << end << " rotated_end: "<< rotated_end << std::endl;

                        object->dir = v_dir;

                    }
                }
            }
        }
    }

    return fsdmap::SUCC;
}


//版本2
int RoadModelProcFixObject::fix_stopline_geom2(RoadModelSessionData* session) {
    if (!FLAGS_fix_stopline_geom_enable) {
        return fsdmap::SUCC;
    }

    // int index = 0;
    for (auto& object : session->raw_object_ret_list) {
        if (object->ele_type != 6) {
            continue;
        }
        // std::cout << "index : " << index << std::endl;
        // index++;

        double radius = (object->length/2) * 0.7;  //停止线一半 *0.7
        std::vector<LaneLineSample*> search_sec; 
        session->lane_line_sample_tree.search(object->pos, radius, search_sec);
        // std::cout << "search_sec.size() : " << search_sec.size() << std::endl;
        // std::cout << "radius : " << radius << std::endl;
        // std::cout << "object->length/2 : " << object->length/2 << std::endl;
        
        int count = 0;
        Eigen::Vector3d dir_avg(0, 0, 0);

        for(auto &lc : search_sec){
            dir_avg += lc->dir;
            count++;
        }
   
        if (count > 0) {
            dir_avg /= count; 

            // 获取车道线垂直方向
            auto v_dir = alg::get_vertical_dir(dir_avg);


            // std::cout << "object->dir : " << object->dir.transpose() << std::endl; 
            // std::cout << "原来v_dir : " << v_dir.transpose() << std::endl;
            // std::cout << "alg::calc_theta(object->dir, v_dir,  true, true) >90? : " << alg::calc_theta1(object->dir, v_dir,  false, true) << std::endl;
            
            // 如果夹角大于90度，取反垂直方向，保证夹角小于90度，保证和停止线方向为正向
            if (fabs(alg::calc_theta1(object->dir, v_dir,  false, true)) > 90) {
                v_dir = -v_dir;
            }
            // std::cout << "纠正v_dir : " << v_dir.transpose() << std::endl;
            
            // 修改停止线几何
            if (object->list.size() == 2) {
                Eigen::Vector3d center = object->pos;
                Eigen::Vector3d start = object->list[0]->pos;
                Eigen::Vector3d end = object->list[1]->pos;
                
                // 计算停止线和车道线垂线的夹角，取反是为了后面将停止线旋转到车道线垂线方向
                double theta = -alg::calc_theta1(object->dir, v_dir, false, true);
                //340
                // std::cout << "alg::calc_theta(object->dir, v_dir, false, true) : " << alg::calc_theta1(object->dir, v_dir, false, true) << std::endl;
   
                
                //异常值，过滤
                // std::cout << "修正theta : " << theta << std::endl;
                // std::cout << "--------------------------------" << std::endl;
                if(fabs(theta) > 60){
                    continue;
                }

                Eigen::Vector3d rotated_start = alg::rotate_vector(start - center, theta) + center;
                Eigen::Vector3d rotated_end = alg::rotate_vector(end - center, theta) + center;
                
                if(1){
                    object->list[0]->pos = rotated_start;
                    object->list[1]->pos = rotated_end;
                    object->list[0]->raw_pos = rotated_start;
                    object->list[1]->raw_pos = rotated_end;

                    // std::cout << "start : " << start << " rotated_start: "<< rotated_start << std::endl;
                    // std::cout << "end : " << end << " rotated_end: "<< rotated_end << std::endl;

                    object->dir = v_dir;

                }
            }
        }
        
    }

    return fsdmap::SUCC;
}



int RoadModelProcFixObject::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_fix_object_save_data_enable) {
        return fsdmap::SUCC;
    }

    session->set_display_name("fix_object");
     for (auto &road_segment : session->road_segment_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "road_segment");
        log->color = {255, 0, 0};
        for (auto &key_pose : road_segment->pos_sample_list) {
            if(key_pose->lane_center_list.size() == 0){
                log->color = {255, 255, 255};
            }
            log->add(key_pose->pos, 10);  
            // std::cout << "pos_link: " << key_pose->pos << std::endl;
        }
    }
    

    for (auto &road_segment : session->road_segment_list) {
        for (auto &key_pose : road_segment->pos_sample_list) {
            for (auto &object : key_pose->object_list) {
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_object");
                log->add(key_pose->pos, 10);
                log->add(object->pos, 10);
                log->color = {0, 255, 0};
                double dis = alg::calc_dis(key_pose->pos, object->pos, true);
                DLOG_POINT2(key_pose->pos, object->pos, "debug bind obj_dis={}", dis);

                
                //add

                auto log2 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "bind_object");
                log2->color = {255, 255, 0};

                if(object->ele_type == 6){
                    auto& pos_sample_list = key_pose->road_segment->pos_sample_list;
         
                    auto it = std::find(pos_sample_list.begin(), pos_sample_list.end(), key_pose);
                    if (it == pos_sample_list.end()){
                        continue;
                    }


                    for (; it != pos_sample_list.begin(); --it) {
                        double dis = alg::calc_dis(key_pose->pos, (*it)->pos, true);
                        // std::cout << "(*it)->filled_lane_sample.size() : " << (*it)->filled_lane_sample.size() << std::endl;

                        if (dis <= 20.0) {
                            // std::cout << "enter " << "pos: " << (*it)->pos << std::endl;
                            log2->add((*it)->pos, 10);
                            for (auto& lc : (*it)->filled_lane_sample) {
                                // std::cout << "lc->road_index : " << lc->road_index << std::endl;
                                if(lc->road_index == 0) {
                                    // log->add((*it)->pos, 10);
                                } 
                            }
                        }
                        else {
                            break;
                        }
                    }
                }
          
            }
        }
    }
    //停止线位置
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6){
            auto log = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            log->color = {255, 0, 255};
            for(auto& pt : object->list){
                log->add(pt->pos, 10);
            }
        }
    }
    
    //找出是第几个元素,标号
    // int count = 0;
    // auto log2 = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "stopline2");
    // for (auto& object : session->raw_object_ret_list) {
    //     if(object->ele_type == 6){
    //         count++;
    //         // auto log2 = session->add_debug_log(utils::DisplayInfo::POINT, "stopline2{}", count);
    //         log2->color = {255, 0, 0};
    //         log2->add(object->pos, 10);
    //     }
    // }




    session->save_debug_info("fix_object");

    return fsdmap::SUCC;

    
}


}
}
