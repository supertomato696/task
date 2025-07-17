


#include "road_model_proc_bind_trail2.h"
#include "utils/polyfit.h"
#include "utils/algorithm_util.h"

DEFINE_bool(bind_trail_enable2, true, "bind_trail_enable2");
DEFINE_bool(bind_trail_debug_pos_enable2, true, "bind_trail_debug_enable");
DEFINE_bool(bind_trail_save_data_enable2, true, "bind_trail_save_data_enable2");
// DEFINE_bool(bind_trail_save_data_save_detail_enable, false, "bind_trail_save_data_save_detail_enable");
// DEFINE_double(bind_trail_turn_theta_threshold, 20, "bind_trail_turn_theta_threshold");
// DEFINE_double(bind_trail_pos_match_lane_radius, 30, "bind_trail_pos_match_lane_radius");
// DEFINE_double(bind_trail_pos_match_lane_z_max, 6, "bind_trail_pos_match_lane_z_max");
// DEFINE_double(bind_trail_pos_match_lane_z_min, -2, "bind_trail_pos_match_lane_z_min");
// DEFINE_double(bind_trail_pos_match_lane_theta, 30, "bind_trail_pos_match_lane_theta");
DEFINE_double(bind_trail_pos_match_crosswalk_radius2, 15.0, "bind_trail_pos_match_crosswalk_radius2");
// DEFINE_double(bind_trail_pos_match_crosswalk_h_scope, 0.5, "bind_trail_pos_match_crosswalk_h_scope");
// DEFINE_double(bind_trail_pos_match_crosswalk_v_scope, 5, "bind_trail_pos_match_crosswalk_v_scope");
// DEFINE_double(bind_trail_bind_lc_width_buff, 0.8, "bind_trail_bind_lc_width_buff");
// DEFINE_double(bind_trail_pos_match_stopline_theta_thres, 60, "bind_trail_pos_match_stopline_theta_thres");
// DEFINE_double(bind_trail_pos_match_crosswalk_theta_thres, 60, "bind_trail_pos_match_crosswalk_theta_thres");
// DECLARE_double(display_scope_buff);
// DECLARE_double(display_scale_rate);


namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcBindTrail2::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_bind_trail_enable2) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_bind_trail_debug_pos_enable2;

    // CHECK_FATAL_PROC(get_poss_cross_point_mutil(session), "get_poss_cross_point_mutil");

    // CHECK_FATAL_PROC(sort_new_lc(session), "sort_new_lc");

    // // CHECK_FATAL_PROC(get_poss_cross_fls(session), "get_poss_cross_fls");

    // CHECK_FATAL_PROC(find_more_crosspoint_and_cut_link(session), "find_more_crosspoint");

    CHECK_FATAL_PROC(get_poss_cross_object2(session), "get_poss_cross_object");


    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

void get_point(std::shared_ptr<BoundaryGroupLine> line ,int index,int size,bool is_forward,double distance,std::deque<Eigen::Vector3d> &points)
{
    if(distance<=0)
    {
        return ;
    }
    int prev_index=is_forward?index-1:index+1;
    if(!line)
    {
        return ;
    }
    if(index>=size||index<0|| prev_index>=size||prev_index<0)
    {
        return ;
    }
    auto cur_pt=line->list[index];
    auto prev_pt=line->list[prev_index];
    if(!cur_pt|| !prev_pt)
    {
        return ;
    }
    if(is_forward)
    {
        points.push_back(cur_pt->pos);
    }
    else
    {
        points.push_front(cur_pt->pos);
    }
    int next_index=is_forward?index+1:index-1;
    double next_distance=distance-alg::calc_dis(cur_pt->pos,prev_pt->pos);
    get_point(line,next_index,size,is_forward,next_distance,points);
}

int RoadModelProcBindTrail2::find_more_crosspoint_and_cut_link(RoadModelSessionData* session){

    RTreeProxy<KeyPose *, float, 2> link_pos_tree;
    for (auto line : session->link_sample_list)
    {
        if (alg::match_any_with_forms(line->list.front()->from_raw_link->forms, {25, 26})){
            continue;
        }
        for (auto pt : line->list)
        {
            link_pos_tree.insert(pt->pos, pt);
        }
    }

    RTreeProxy<BoundaryFeature *, float, 2> road_boundary_tree;
    for (auto line : session->merge_boundary_line_list)
    {
        for (auto rb : line->list)
        {
            road_boundary_tree.insert(rb->pos, rb.get());
        }
    }

    std::vector<fsdmap::BreakInfo *> new_crosspoint;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_cp(new pcl::KdTreeFLANN<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cross_point_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    int split_merge_size = session->split_merge_break_points.size();
    if (split_merge_size > 0){
        for (int i = 0; i < split_merge_size; i++) {
            auto bkp = session->split_merge_break_points[i];
            if (bkp->valid == false) {
                continue;
            }
            
            cross_point_cloud->push_back(pcl::PointXYZI(bkp->pos.x(), bkp->pos.y(), 0, 0));
        }

        if(cross_point_cloud->points.size() > 0) {
            kdtree_cp->setInputCloud(cross_point_cloud);
        }
    }

    /*  --------------------------------------  search road_boundary  --------------------------------------*/
    std::vector<std::shared_ptr<BoundaryGroupLine>> tmp_boundary_line_ptr;
    std::vector<std::shared_ptr<BoundaryGroupLine>> tmp_boundary_line_list;

    auto get_right_boundary_rb = [&road_boundary_tree](KeyPose *poss) -> BoundaryFeature*
    {
        auto from_raw_link = poss->from_raw_link;
        if (!from_raw_link)
        {
            return NULL;
        }
        else
        {
            if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
            {
                return NULL;
            }
        }

        std::vector<BoundaryFeature *> bfs;
        road_boundary_tree.search(poss->pos, 10, bfs);
        Eigen::Vector3d p1 = poss->pos;
        Eigen::Vector3d p2 = alg::get_vertical_pos(poss->pos, poss->dir, 10);
        BoundaryFeature *min_dis_bf = NULL;
        double min_dis = 100;
        for (auto bf : bfs)
        {
            if (!bf->next)
            {
                continue;
            }
            Eigen::Vector3d cross;
            if (alg::findIntersection(p1, p2, bf->pos, bf->next->pos, cross))
            {
                // TODO CHECK
                if (alg::judge_left(cross, poss->pos, poss->dir) < 0)
                {
                    continue;
                }
                double dis = alg::calc_dis(cross, poss->pos);
                if (dis < min_dis)
                {
                    min_dis = dis;
                    min_dis_bf = bf;
                }
            }
        }

        return min_dis_bf;
    };

    auto get_boundary_line=[&,session]()
    {
        for (auto link : session->link_sample_list)
        {
              if(link->list.empty())
              {
                continue;
              }
              auto front=link->list.front();
              auto from_raw_link = front->from_raw_link;
              if (!from_raw_link)
              {
                 continue;
              }

              if (alg::match_any_except_forms(from_raw_link->forms, {25, 26}))
              {
                continue;
              }
              std::vector<KeyPose*> new_single_line;
              session->sample_line(0.5, link->list, 
              session->key_pose_ptr,new_single_line);
        
             for(auto &p:new_single_line){
              p->from_raw_link=front->from_raw_link;
              p->from_link=front->from_link;
            }

            auto new_line = session->add_ptr(tmp_boundary_line_ptr);

            std::shared_ptr<BoundaryFeature> prev_rb=NULL;
            for (auto poss : new_single_line)
            {
                auto rbp = get_right_boundary_rb(poss);
                if (rbp)
                {   
                    std::shared_ptr<BoundaryFeature> rb = std::make_shared<BoundaryFeature>();
                    rb->init(rbp);

                    Eigen::Vector3d dir={0,0,0};
                    if(prev_rb)
                    {
                         // 避免太近方向算出来误差太大
                        if(alg::calc_dis(prev_rb->pos,rb->pos)<0.25)
                        {
                            continue;
                        }
                        // 避免弯道交叉
                        if(!alg::judge_front(rb->pos,prev_rb->pos,prev_rb->dir))
                        {
                          continue;
                        }
                        dir=alg::get_dir(rb->pos,prev_rb->pos);
                    }
                    prev_rb=rb;
                    new_line->list.push_back(rb);
                    if(!new_line->list.empty())
                    {
                        auto &back=new_line->list.back();
                        back->dir = dir;
                    }
                    //    trc.push_back(new_turn_right_point);  
                }
                else  //lane bounary loss
                {

                }
            }
            
            if(new_line->list.size()>0){
                new_line->list.pop_back();
                tmp_boundary_line_list.push_back(new_line);
            }
        }
    };
    /*  --------------------------------------  search road_boundary  --------------------------------------*/

    /*  --------------------------------------  find_new_crosspoint  --------------------------------------*/
    auto set_calc_curvature = [](std::shared_ptr<BoundaryGroupLine> line, int index, const std::deque<Eigen::Vector3d> &points)
    {
        if (!line)
        {
            return;
        }
        auto pt = line->list[index];
        std::vector<Eigen::Vector3d> temp;
        temp.insert(temp.end(), points.begin(), points.end());
        // 计算曲率
        PolyFit::Problem problem;
        problem.degree=3;
        problem.points=temp;
        problem.center=pt->pos;
        problem.dir=pt->dir;
        PolyFit::Options option;
        option.use_calc_direction=true;
        option.use_calc_center=true;
        PolyFit fit=PolyFit(problem,option); 
        fit.solver();
        double curvature= fit.curvature(pt->pos);
        pt->curvature=curvature;
    };

    auto find_crosspoint_by_boundaryline=[&](){
        auto find_crosspoint=[&](std::shared_ptr<BoundaryGroupLine>& group_line, int startindex, int endindex, bool forward, std::shared_ptr<fsdmap::BoundaryFeature>& pt){
            const int size = group_line->list.size();
            std::vector<Eigen::Vector3d> dirs;
            std::vector<KeyPose *> kpfs;

            auto pt1 = !forward? group_line->list[startindex]: group_line->list[endindex-1];
            link_pos_tree.search(pt1->pos, 20, kpfs);
            if (!kpfs.empty()) {
                for (const auto kp : kpfs) {
                    dirs.push_back(kp->dir);
                }
            }
            auto link_dir = alg::get_direction(dirs).normalized();

            const double theta_thres=10;
            std::vector<double> theta_list;
            std::deque<int> indexs;
            for (int i = startindex; i < endindex; i++) {
                int j = forward? i:endindex-1-i;
                /////////////////////////////////////////
                auto pt1 = group_line->list[j];
                auto pt_dir = pt1->dir.normalized();
                double theta = alg::calc_theta(link_dir, pt_dir);
                theta = theta>90? 180-theta: theta;
                theta_list.push_back(theta);
                // LOG_INFO("{}th pt -- ({}, {}), ({}, {}), angle_theta: {}", j, pt_dir.x(), pt_dir.y(), link_dir.x(), link_dir.y(), theta);
                if (theta < theta_thres) {
                    indexs.push_back(i-startindex);
                }
            }
            while (indexs.size() > 0) {
                auto start = indexs.front();
                indexs.pop_front();
                if (start >= theta_list.size()) {
                    break;
                }

                int count=0, bins=10;
                auto end = start+bins;
                size_t validCount = (end <= theta_list.size()) ? bins : theta_list.size() - start;

                for (size_t i = 0; i < validCount; ++i) {
                    if (theta_list[start + i] < theta_thres) {
                        count++;
                    }
                }
                if (count > bins*0.8){
                    pt = group_line->list[start+startindex];
                    return true;
                }
            }

            return false;
        };
        
        auto boundary_size = tmp_boundary_line_list.size();
        for (int i2 = 0; i2 < boundary_size; i2++) {
            auto group_line = tmp_boundary_line_list[i2];
            const int size = group_line->list.size();
            double max_curvature = -1;
            int max_index = 0;

            for (int j = 0; j < size; j++) {
                std::deque<Eigen::Vector3d> points;
                get_point(group_line, j, size, true, 15, points);
                get_point(group_line, j, size, false, 15, points);
                set_calc_curvature(group_line, j, points);
                if (group_line->list[j]->curvature > max_curvature) {
                    max_curvature = group_line->list[j]->curvature;
                    max_index = j;
                }
                // LOG_INFO("the {}-{}th point, dir: ({}, {}), max_index: {} ....", i2, j, group_line->list[j]->dir.x(), group_line->list[j]->dir.y(), max_index);
            }

            std::shared_ptr<fsdmap::BoundaryFeature> pt1, pt2;
            if (find_crosspoint(group_line, 0, max_index, false, pt1)) {
                LOG_INFO("---------Find new crosspoint: ({}, {})---------", pt1->pos.x(), pt1->pos.y());
                // new_cross_point_cloud->push_back(pcl::PointXYZI(pos1.x(), pos1.y(), pos1.z(), 0));
                BreakInfo*bk = new BreakInfo(BreakStatus::SPLIT_MERGE_VECTORIZE, pt1->pos, true);
                bk->dir = pt1->dir;
                bk->front_or_back = true;
                new_crosspoint.push_back(bk);
                session->split_merge_break_points.push_back(bk);
            }
            
            if (find_crosspoint(group_line, max_index, size, true, pt2)) {
                LOG_INFO("---------Find new crosspoint: ({}, {})---------", pt2->pos.x(), pt2->pos.y());
                // new_cross_point_cloud->push_back(pcl::PointXYZI(pos2.x(), pos2.y(), pos2.z(), 0));
                BreakInfo* bk = new BreakInfo(BreakStatus::SPLIT_MERGE_VECTORIZE, pt2->pos, true);
                bk->dir = pt2->dir;
                bk->front_or_back = false;
                new_crosspoint.push_back(bk);
                session->split_merge_break_points.push_back(bk);
            }
        }
    };
    /*  --------------------------------------     find_new_crosspoint      --------------------------------------*/

    /*  --------------------------------------  cut_link_by_new_crosspoint  --------------------------------------*/
    auto cut_link_by_new_crosspoint=[&](){

        if (new_crosspoint.size() > 0){
            RTreeProxy<KeyPose*, float, 2> tmp_link_pos_tree;
            session->link_pos_tree.RemoveAll();

            // LOG_INFO(" [cut_link] starting cutting ...");
            for (auto bk: new_crosspoint){
                auto p1 = alg::get_vertical_pos(bk->pos, bk->dir, -20);
                auto p2 = bk->pos;
                // LOG_INFO(" [cut_link] crosspoint: pos-({}, {}, {}), dir-({}, {}, {}) ...", bk->pos.x(), bk->pos.y(), bk->pos.z(), bk->dir.x(), bk->dir.y(), bk->dir.z());
                // LOG_INFO(" [cut_link] ps-pe: ({}, {}, {}), ({}, {}, {}) ...", p1.x(), p1.y(), p1.z(), p2.x(), p2.y(), p2.z());
                for (auto& line: session->link_sample_list) {
                    if (alg::match_any_with_forms(line->list.front()->from_raw_link->forms, {25, 26})){

                        int index = 0;
                        for (auto kp : line->list)
                        {
                            if (!kp->next)
                            {
                                continue;
                            }
                            Eigen::Vector3d cross;
                            if (alg::findIntersection(p1, p2, kp->pos, kp->next->pos, cross)){
                                auto& vec = line->list;
                                if (bk->front_or_back){
                                    vec.erase(vec.begin(), vec.begin()+index);
                                } else {
                                    vec.erase(vec.begin()+index, vec.begin()+vec.size());
                                }
                                LOG_INFO(" [cut_link] finish cutting ...");
                                break;
                            }
                            index++;
                        }
                    }

                    for (auto &poss : line->list) {
                        session->link_pos_tree.insert(poss->pos, poss);//切断后link的pose
                    }
                }
            }
        }
    };
    /*  --------------------------------------  cut_link_by_new_crosspoint  --------------------------------------*/

    /*  --------------------------------------           main code          --------------------------------------*/
    LOG_INFO("[cut_link_by_new_crosspoint] get_boundary_line ....");
    get_boundary_line();
    LOG_INFO("[cut_link_by_new_crosspoint] find_crosspoint_by_boundaryline ....");
    find_crosspoint_by_boundaryline();
    LOG_INFO("[cut_link_by_new_crosspoint] cut_link_by_new_crosspoint ....");
    cut_link_by_new_crosspoint();

    return fsdmap::SUCC;
}


int RoadModelProcBindTrail2::get_poss_cross_object2(RoadModelSessionData* session) {
    // 定义匹配半径和方向角度差异阈值
    float radius = FLAGS_bind_trail_pos_match_crosswalk_radius2;  // 15.0
    RTreeProxy<KeyPose*, float, 2> keypose_tree;  // 创建空间索引树

    // 将道路段的位置信息插入到空间索引树中
    for (auto &line : session->link_sample_list) {
        for (auto &poss : line->list) {
            keypose_tree.insert(poss->pos, poss);
        }
    }

    // 遍历  原始特征数据  
    for (auto& object : session->raw_object_ret_list) {
        std::vector<KeyPose*> secs;  // 存储匹配的KeyPose
        // 遍历对象的所有位置点
        for (auto& object_pt : object->list) {
            std::vector<KeyPose*> secs_one_pt;
            keypose_tree.search(object_pt->get_pos(), radius, secs_one_pt);  // 搜索匹配位置
            // 计算距离并过滤
            for(auto& keypos : secs_one_pt) {
                session->debug_pos(keypos->pos);  // 调试输出
                double dis = alg::calc_dis(object_pt->get_pos(), keypos->pos, true);
                if(dis < radius) {
                    secs.push_back(keypos);
                }
            }
        }
        std::vector<KeyPose*> secs_pos;
        keypose_tree.search(object->pos, radius, secs_pos);  // 搜索匹配位置
        // 计算距离并过滤
        for(auto& keypos : secs_pos) {
            double dis = alg::calc_dis(object->pos, keypos->pos, true);
            if(dis < radius) {
                secs.push_back(keypos);
            }
        }

        // 如果没有匹配到位置，则跳过当前对象
        if (secs.empty()) {
            continue;
        }


        if(object->ele_type == 6){
            //处理停止线
            KeyPose* minKeyPose = nullptr;
            double mindis = DBL_MAX;
            for (int i = 0; i < secs.size(); i++)
            {
                double temp_dis = alg::calc_dis(object->pos, secs[i]->pos, true);
                if(temp_dis < mindis)
                {
                    mindis = temp_dis;
                    minKeyPose = secs[i];
                }
            }

            if (minKeyPose != nullptr){
                //2.方案2： 对于link弯的曲线，进一步找到最近的点
                if(1){
                    std::vector<Eigen::Vector3d> link_pts;
                    for(auto& keypose : minKeyPose->from_link->list){
                        link_pts.push_back(keypose->pos);
                    }

                    std::vector<UsefulPnt> cross_points;
                    bool is_intersect = alg::get_cross_point_with_curve_segment(object->list[0]->pos, object->list[1]->pos, link_pts, cross_points,
                                                                                false, 0);
                    if(is_intersect == true){
                        double min_dis = 1000000;
                        for(auto& keypose : minKeyPose->from_link->list){
                            double dis = alg::calc_dis(cross_points[0].pos, keypose->pos);
                            if(dis < min_dis){
                                min_dis = dis;
                                minKeyPose = keypose;
                            }
                        }
                    }
                } //TODO :tianfa 用next prev 补出中间的交点


                //3 .方案3：直接插值出最近的点
                // if(0){
                //     int index = 0;
                //     for(int i = 0; i < minKeyPose->from_link->list.size() - 1; i++){
                //         Eigen::Vector3d intersection_point; //交出的最近点 
                //         Eigen::Vector3d pt1_from = minKeyPose->from_link->list[i]->pos;
                //         Eigen::Vector3d pt1_to = minKeyPose->from_link->list[i + 1]->pos;
                //         bool ignore_z = true;
                //         bool is_intersect = get_cross_point_by_point(object->list[0]->pos, object->list[1]->pos, 
                //             pt1_from, pt1_to, intersection_point, ignore_z);
                //         if (is_intersect) {
                //             index =i;
                //             break
                //         }
                //     }
                // }

                //4.剔除右转绑到直线的
                if (minKeyPose != nullptr){
                    Eigen::Vector3d cross_point2;
                    Eigen::Vector3d pos_to = minKeyPose->pos + minKeyPose->dir;
                    bool is_intersect2 = alg::get_cross_point_by_point(minKeyPose->pos, pos_to, object->list[0]->pos, object->list[1]->pos, cross_point2,
                                                            false, 1);
                    if(is_intersect2){
                        minKeyPose->object_list.push_back(object.get());
                        object->bind_links.push_back(minKeyPose);
                    }
                }

                
            }
            
        }
    }

    return fsdmap::SUCC;
}

bool RoadModelProcBindTrail2::judge_keypose_cross_crosswalk(RoadModelSessionData* session, RoadObjectInfo* object, KeyPose* keypose){
    for (int i = 0; i < object->list.size() - 1; ++i) {  // 处理从 0 到 n-2 个顶点
        Eigen::Vector3d poly_from = object->list[i]->pos;
        Eigen::Vector3d poly_to = object->list[i + 1]->pos;  // 下一顶点，形成一条边

        Eigen::Vector3d cross_point;
        Eigen::Vector3d pos_to = keypose->pos + keypose->dir;
        bool is_intersect = alg::get_cross_point_by_point(keypose->pos, pos_to, poly_from, poly_to, cross_point, false, 1);
        if (is_intersect) {
            return true;
        }
    }
    return false;
}


int RoadModelProcBindTrail2::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_bind_trail_save_data_enable2) {
        return fsdmap::SUCC;
    }

    session->set_display_name("bind_obj");
    for (auto &line : session->link_sample_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "link");
        log->color = {255, 255, 255};
        for (auto &key_pose : line->list) {
            log->add(key_pose->pos, 10);  
        }
    }
    
    //指向连线
    for (auto &line : session->link_sample_list) {
        for (auto &key_pose : line->list) {
            for (auto &object : key_pose->object_list) {
                auto log = session->add_debug_log(utils::DisplayInfo::LINE, "bind_object");
                log->add(key_pose->pos, 10);
                log->add(object->pos, 10);
                log->color = {0, 255, 0};
            }
        }
    }

    //停止线位置
    for (auto& object : session->raw_object_ret_list) {
        if(object->ele_type == 6){
            auto log2 = session->add_debug_log(utils::DisplayInfo::LINE, "stopline");
            auto log2_link = session->add_debug_log(utils::DisplayInfo::POINT, "st_link");
            log2->color = {255, 0, 255};
            log2_link->color = {255, 0, 255};
            for(auto& pt : object->list){
                log2->add(pt->pos, 10);
            }

            //停止线对应的link点
            for(auto& keypose : object->bind_links){
                log2_link->add(keypose->pos, 10);
            }
        }
    }


    session->save_debug_info("bind_obj");

    return fsdmap::SUCC;
}

}
}
