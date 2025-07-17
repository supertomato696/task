


#include "road_model_proc_sample_line.h"
#include "utils/algorithm_util.h"
#include <random>
DECLARE_double(display_scope_buff);
DECLARE_double(display_scale_rate);


DEFINE_bool(sample_line_debug_pos_enable, true, "sample_line_debug_enable");
DEFINE_bool(sample_line_save_data_enable, true, "sample_line_save_data_enable");
DEFINE_bool(sample_line_save_data_save_detail_enable, false, "sample_line_save_data_save_detail_enable");
DEFINE_bool(sample_line_save_data_save_frame_enable, false, "sample_line_save_data_save_frame_enable");
DEFINE_double(sample_line_sample_pose_gap, 2, "sample_line_sample_pose_gap");
DEFINE_double(sample_line_sample_lane_line_gap, 4, "sample_line_sample_lane_line_gap");
DEFINE_double(sample_line_dir_radius1, 5, "sample_line_dir_radius1");
DEFINE_double(sample_line_dir_radius2, 10, "sample_line_dir_radius2");
DEFINE_double(sample_line_ll_dir_gap_threshold, 0.4, "sample_line_ll_dir_gap_threshold");
DEFINE_double(sample_line_lc_dir_gap_threshold, 0.4, "sample_line_lc_dir_gap_threshold");
DEFINE_double(sample_line_link_repair_radius, 20, "sample_line_link_repair_radius");
DEFINE_double(sample_line_link_repair_theta, 30, "sample_line_link_repair_theta");  // origin 30
DEFINE_double(sample_line_link_repair_threshold, 30, "sample_line_link_repair_threshold");
DEFINE_int32(sample_line_bev_frame_skip_num, 1, "sample_line_bev_frame_skip_num");
DECLARE_double(split_road_make_line_sample_gap);
DECLARE_bool(split_road_split_road_by_link_enable);

namespace fsdmap {
namespace road_model {


fsdmap::process_frame::PROC_STATUS RoadModelProcSampleLine::proc(
        RoadModelSessionData* session) {
    session->enable_debug_pos = FLAGS_sample_line_debug_pos_enable;

    // 取出session->key_pose_map.list中pose之间间距满足2m的
    // 放入session->key_pose_ptr和session->key_pose_list
    CHECK_FATAL_PROC(sample_key_pose(session), "sample_key_pose");

    // 把ins的pose映射到link每个点的pose
    // 遍历link中的点，间隔2m取点放入session->key_pose_ptr和session->key_pose_line_ptr
    CHECK_FATAL_PROC(sample_link(session), "sample_link");

    // 根据路口分叉点，切断link
    // CHECK_FATAL_PROC(cut_link_from_crosspoint(session), "cut_link_from_crosspoint");

    // 在整合了 session->sem_lane_instance_map
    // 中所有的车道线点后，在一起重新等间距采样
    CHECK_FATAL_PROC(smooth_lane_line(session), "smooth_lane_line");

    // 对车道中心线降采样
    CHECK_FATAL_PROC(smooth_lane_center(session), "smooth_lane_center");

    CHECK_FATAL_PROC(smooth_boundary(session), "smooth_boundary");

    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}
std::string RoadModelProcSampleLine::calc_trail_id(std::string str)
{
    char delimiter = '/';
    std::string ret_string;
    std::vector<std::string> tokens;
    // 使用Boost的split函数进行字符串分割
    boost::split(tokens, str, boost::is_any_of(&delimiter));
    const int j=tokens.size();
    if(j<=2)
    {
        return ret_string;
    }
    ret_string=tokens[j-2]+"_"+tokens[j-1];
    return ret_string;
}

double crossProduct(const Eigen::Vector3d& A, const Eigen::Vector3d& B, const Eigen::Vector3d& P) {
    // 向量AB和向量AP的叉积（二维情况，只考虑z分量）
    return (B.x() - A.x()) * (P.y() - A.y()) - (B.y() - A.y()) * (P.x() - A.x());
}

// 判断点P是否在凸多边形内部
bool isPointInConvexPolygon(const std::vector<Eigen::Vector3d>& polygon, const Eigen::Vector3d& P) {
    int n = polygon.size();
    if (n < 3) return false; // 多边形至少需要3个顶点

    bool isInside = true;
    double sign = 0;

    for (int i = 0; i < n; ++i) {
        // 当前边的两个顶点
        Eigen::Vector3d A = polygon[i];
        Eigen::Vector3d B = polygon[(i + 1) % n]; // 下一个顶点，注意循环

        // 计算叉积
        double cp = crossProduct(A, B, P);

        // 如果叉积为0，说明点在边上
        if (cp == 0) return true;

        // 如果第一次计算叉积，记录符号
        if (sign == 0) sign = cp > 0 ? 1 : -1;

        // 如果后续叉积符号与第一次不同，说明点在多边形外
        if ((cp > 0 && sign == -1) || (cp < 0 && sign == 1)) {
            isInside = false;
            break;
        }
    }

    return isInside;
}

void RoadModelProcSampleLine::calc_junction_score(RoadModelSessionData* session,KeyPoseLine*kl)
{
    auto mark_score = [&](std::pair<Eigen::Vector3d, std::vector<Eigen::Vector3d>> junction, KeyPoseLine *kl)
    {
        // if(kl->list.front()->from_raw_link->form =="25" ||kl->list.front()->from_raw_link->form == "26"){
        if(alg::match_any_with_forms(kl->list.front()->from_raw_link->forms, {25, 26})) {
            return;
        }
        int break_index = -1;
        int n = kl->list.size();
        Eigen::Vector3d center = junction.first;
        double max_dis = -DBL_MAX;
        for (auto p : junction.second)
        {
            max_dis = std::max(max_dis, alg::calc_dis(p, center));
        }
        // 向外扩展1.5倍宽度
        max_dis = 3 * max_dis;
        //
        for (int i = 0; i < n; i++)
        {
            bool is_in_polygon = alg::point_in_polygon(kl->list[i]->pos, junction.second);
            // bool is_in_polygon = isPointInConvexPolygon(junction.second, kl->list[i]->pos);
            if(is_in_polygon)
            {
                break_index = i;
                break;
            }
        }
        if (break_index < 0)
        {
            return;
        }
        for (int i = 0; i < n; i++)
        {
            int next = break_index + i;
            if (next >= n - 1)
            {
                break;
            }
            float score = alg::calc_dis(kl->list[next]->pos, center);
            if (score > max_dis)
            {
                break;
            }
            score = exp(-0.01 * score);
            auto &max_score = kl->list[next]->junction_score;
            max_score = std::max(max_score, score);
        }
        for (int i = 0; i < n; i++)
        {
            int next = break_index - i;
            if (next < 0)
            {
                break;
            }
            float score = alg::calc_dis(kl->list[next]->pos, center);
            if (score > max_dis)
            {
                break;
            }
            score = exp(-0.01 * score);
            auto &max_score = kl->list[next]->junction_score;
            max_score = std::max(max_score, score);
        }
    };

    for (auto &obj : session->raw_intersections)
    {
        std::vector<Eigen::Vector3d> one_junction;
        auto center = obj->pos;
        for (auto &pt : obj->point_info)
        {
            one_junction.push_back(pt->pos);
        }
        resize_junctions.emplace_back(center, one_junction);
    }
    for (auto p : resize_junctions)
    {
        mark_score(p, kl);
    }
}

 void RoadModelProcSampleLine::find_same_link(RTreeProxy<KeyPose*, float, 2> &link_pos_tree,
                   KeyPoseLine* link, std::deque<KeyPoseLine*> &one_link,
                   std::set<KeyPoseLine*>&hash_map,bool is_next, bool current_link){
    auto is_print=[](const Eigen::Vector3d &pos,const Eigen::Vector3d & cur_pos){
          return alg::calc_dis(pos,cur_pos)<1.5;
    };
    if(!hash_map.count(link))
     {
        hash_map.insert(link);
        if(is_next)
        {
            one_link.push_back(link); 
        }else
        {
            one_link.push_front(link); 
        }
     }else
     {
        // 如果是当前第一次遍历，所有的 第一个 节点都继续遍历，防止从中间开始，导致link 遍历不全
        if(!current_link) {
            return ;
        }
     }
     KeyPose* key_pos=is_next? link->list.back(): link->list.front();
     //
     std::vector<KeyPose*> tmp_secs;
     link_pos_tree.search(key_pos->pos, 0.5, tmp_secs); //10cm  
     KeyPose *min_pos=NULL;
     double min_theta=1000;
    //    if(is_print(key_pos->pos,{-38.4,-72.27,0}))
    //    {
    //     LOG_WARN(" secs:{}",tmp_secs.size());
    //    }
    
    // 从当前link的起始或结束位置，开始找角度最小的link进行dfs遍历
     for(auto next_pos:tmp_secs)
     {
        if(next_pos==key_pos)
        {
            continue;
        }
        // 优先扩展直行，左/右转、辅路另外扩展
        bool cond1 = alg::match_any_with_forms(key_pos->from_raw_link->forms, {21, 25, 26});
        bool cond2 = alg::match_any_with_forms(next_pos->from_raw_link->forms, {21, 25, 26});
        if((cond1 && !cond2) || (!cond1 && cond2))
        {
            continue;
        }
       double theta=alg::calc_theta1(next_pos->dir,key_pos->dir);
       theta=fabs(theta);
       if(theta<min_theta&& theta<30)
       {
        min_theta=theta;
        min_pos=next_pos;
       }
    //    if(is_print(key_pos->pos,{-38.4,-72.27,0}))
    //    {
    //     LOG_WARN(" min theta:{}  thetat:{}",min_theta,theta);
    //    }

     }
     if(min_pos==NULL)
     {
        return ;
     }   
    //
    auto from_link=min_pos->from_link;
    if(from_link==NULL)
    {
        LOG_WARN("from link null");
    }
    // if(hash_map.count(from_link))
    // {
    //     return;
    // }
     //  TODO 双端遍历
    find_same_link(link_pos_tree,from_link,one_link,hash_map,is_next, false);
     //
    // find_same_link(link_pos_tree,from_link,one_link,hash_map,!is_next);
 }
 std::vector< std::deque<KeyPoseLine*>> RoadModelProcSampleLine::cluster_link(RoadModelSessionData* session)
 {
    std::vector<std::deque<KeyPoseLine*>> vlink;
    std::set<KeyPoseLine*> next_cadidate; // 有 前驱 和 后继 的link
    std::set<KeyPoseLine*> priority_cadidate; // 无前驱 或 后继 的link
    // 1 将所有 原始link的前后节点，放入 kdtree 中
    // RTreeProxy<KeyPose*, float, 2> session->raw_link_pos_tree;
    for(auto link:session->raw_links)
    {
        if(link->list.size() >= 2)
        {
            auto &front=link->list.front();
            auto &back=link->list.back();
            session->raw_link_pos_tree.insert(front->pos,front);
            session->raw_link_pos_tree.insert(back->pos,back);
        }
    }

    // 2. 遍历所有 node 数大于2的link点，将 link 分为两类： 同时有前驱后继的link 和 无前驱或后继的link
    // 优先处理 无前驱或后继的link
    int i=100;
    for(auto link:session->raw_links)
    {
        if(link->list.size()>=2)
        {
            // for (const auto& element : link->forms) {
            //     LOG_INFO("--- {} ---", element);
            // }
            if(alg::match_any_with_forms(link->forms, {25, 26})){
                priority_cadidate.insert(link.get());  // 左/右转 的link
                continue;
            }

          auto front=link->list.front();
          std::vector<KeyPose*> tmp_secs;
          session->raw_link_pos_tree.search(front->pos, 0.5, tmp_secs); //10cm
          if(tmp_secs.size()!=2)
          {
            // link->seed_id=i++;
            if(tmp_secs.size()==1)
            {
                priority_cadidate.insert(link.get());  // 无前驱 或 后继 的link
            }
            else
            {
                next_cadidate.insert(link.get()); // 有 前驱 和 后继 的link
            }
          }else
          {
            for(auto p:tmp_secs)
            {
                if(p->from_link==link.get()) // from_link：这个信息是从 轨迹匹配模块获取到的
                {
                    continue;
                }
                // 如果两条link的方向大于30度，则该点也优先处理
                auto theta=alg::calc_theta1(p->dir,front->dir);
                if(fabs(theta)>30)
                {
                    // link->seed_id=i++;
                    priority_cadidate.insert(link.get());
                }
            }
          }
        }
    }
    srand48(time(NULL));
    int same_id=0;
    std::set<KeyPoseLine*>hash_map;  //3 优先从无前驱节点开始遍历
    for(auto cadidate:{priority_cadidate,next_cadidate})
    {
        for(auto p:cadidate)
        {
            std::deque<KeyPoseLine*> one_link;
            find_same_link(session->raw_link_pos_tree,p, one_link,hash_map,true, false);
            find_same_link(session->raw_link_pos_tree,p, one_link,hash_map,false, true);
            if(!one_link.empty())
            {
                // 将同一个朝向且连续的link，聚类为一个id（same_id)
                vlink.push_back(one_link);
                // int same_id=drand48()*1E3;
                same_id++;
                for(auto plink:one_link)
                {
                    plink->same_id=same_id;
                }
                // LOG_INFO("one link length [{}  {}]",one_link.size(),same_id);
            }
        }
    }
    return vlink;
}
void  RoadModelProcSampleLine::smooth_link_pose( std::vector< KeyPose*> &link_list)
{

    auto smoothness_term=[]( Eigen::Vector3d xp, Eigen::Vector3d xc,Eigen::Vector3d xn,double w)->Eigen::Vector3d
    {     
      return  w * (-4.0) * (xp - 2.0 * xc + xn);
    };
        // 
       const int n=link_list.size();
       if(n<3)
       {
        return ;
       }
       int max_iter=50;
       int iter=0;
       double update=0;
    while(iter++<max_iter)
       {
        
           double d1=0;
           double score;
           for(int i=1;i<n-1;i++)
           {
             auto prev=link_list[i-1];
             auto cur=link_list[i+0];
             auto next=link_list[i+1];
             auto delta=smoothness_term(prev->pos,cur->pos.eval(),next->pos,0.2);
             delta=(cur->smooth_score+cur->junction_score*0.1)*delta;
             cur->pos=cur->pos.eval()-delta;
            //  LOG_INFO("update :[{}  {} {} ]", delta.x(), delta.y(), delta.z());
            d1+=delta.norm();
            score+=cur->smooth_score;
           }
           update+=d1;
        //    LOG_INFO("iter[{}]:update[{}  |  {} | {} ]",iter,d1,update,score/n);
        }
        KeyPose* prev=NULL;
        for(auto p:link_list)
        {
            if(prev)
            {
                prev->dir=alg::get_dir(p->pos,prev->pos);
                p->dir=prev->dir;
            }
            prev=p;
        }
}

int RoadModelProcSampleLine::cut_link_from_crosspoint(RoadModelSessionData* session) {
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

    auto cal_junc_dist = [](std::pair<Eigen::Vector3d, std::vector<Eigen::Vector3d>> junction, Eigen::Vector3d pos)
    {
        Eigen::Vector3d center = junction.first;
        double max_dis = -DBL_MAX;
        for (auto p : junction.second)
        {
            max_dis = std::max(max_dis, alg::calc_dis(p, center));
        }
        // LOG_INFO(" [cut_link] junc_area: {} ...", max_dis);
        // 向外扩展1.5倍宽度
        max_dis = 3 * max_dis;
        //
        return alg::calc_dis(pos, center) < max_dis;
    };

    auto find_valid_poss = [cal_junc_dist, cross_point_cloud, kdtree_cp, &resize_junctions=this->resize_junctions, &smb_pts=session->split_merge_break_points](
                        KeyPoseLine* line, int start_index, int end_index, float& min_dist, int& index, int& bk_index)
    {
        for (int i=start_index; i < end_index; ++i) {
            std::vector<int> neighbors;
            std::vector<float> dist;

            auto poss = line->list[i];
            bool is_in_junc=false;
            for (auto p : resize_junctions)
            {
                is_in_junc = is_in_junc | cal_junc_dist(p, poss->pos);
            }

            if (is_in_junc) {
                pcl::PointXYZI tmppoint(poss->pos[0], poss->pos[1], 0, 0);
                kdtree_cp->nearestKSearch(tmppoint, 1, neighbors, dist);
                // min_dist = std::min(min_dist, dist[0]);
                if (dist[0] < min_dist && (bk_index==neighbors[0] || bk_index==-1)){
                    bk_index = neighbors[0];
                    min_dist = dist[0];
                    index = i;
                }

                if(bk_index!=neighbors[0] && bk_index!=-1){
                    for (auto it = smb_pts.begin(); it != smb_pts.end(); ++it) {
                        if ((*it)->pos.x() == cross_point_cloud->points[neighbors[0]].x && (*it)->pos.y() == cross_point_cloud->points[neighbors[0]].y) {
                            smb_pts.erase(it);  // 移除指针，但不释放内存
                            break;
                        }
                    }
                }
            }
        }
    };
    
    if (split_merge_size > 0){

        RTreeProxy<KeyPose*, float, 2> tmp_link_pos_tree;
        std::vector<KeyPoseLine*> tmp_link_sample_list;

        session->link_pos_tree.RemoveAll();
        for (auto& line: session->link_sample_list) {

            if (alg::match_any_with_forms(line->list.front()->from_raw_link->forms, {25})){
                float l_min_dist = 25.0, r_min_dist = 25.0;
                int l_index=-1, r_index=-1;

                // for (int i=0; i < line->list.size(); ++i) {
                //     std::vector<int> neighbors;
                //     std::vector<float> dist;

                //     auto poss = line->list[i];
                //     bool is_in_junc=false;
                //     for (auto p : resize_junctions)
                //     {
                //         is_in_junc = is_in_junc | cal_junc_dist(p, poss);
                //     }

                //     if (is_in_junc) {
                //         pcl::PointXYZI tmppoint(poss->pos[0], poss->pos[1], poss->pos[2], 0);
                //         kdtree_cp->nearestKSearch(tmppoint, 1, neighbors, dist);
                //         if (dist[0] < min_dist){
                //             min_dist = dist[0];
                //             index = i;
                //         }
                //     }
                // }
                int n = line->list.size();
                int bk_index1=-1, bk_index2=-1;
                find_valid_poss(line, 0, n/2+1, l_min_dist, l_index, bk_index1);
                find_valid_poss(line, n/2+1, n, r_min_dist, r_index, bk_index2);

                auto& vec = line->list;
                if (l_index!=-1 && r_index!=-1){
                    // vec.erase(vec.begin()+r_index+1, vec.begin()+vec.size());
                    // vec.erase(vec.begin(), vec.begin()+l_index+1);
                    vec = std::vector<KeyPose*>(vec.begin()+l_index, vec.begin()+r_index+1);
                } else if (l_index!=-1) {
                    vec.erase(vec.begin(), vec.begin()+l_index);
                } else if (r_index!=-1) {
                    vec.erase(vec.begin()+r_index, vec.begin()+vec.size());
                }

                LOG_INFO(" [cut_link] min_dist: {}, {}...", l_min_dist, r_min_dist);
                LOG_INFO(" [cut_link] finish cutting ...");
                // if (min_dist < 15.0) {
                //     auto& vec = line->list;
                //     std::vector<KeyPose*> tmp;
                //     int remove_start, remove_end;
                //     if (index+1 < vec.size()-index-1) {
                //         vec.erase(vec.begin(), vec.begin()+index+1);
                //     } else {
                //         vec.erase(vec.begin()+index+1, vec.begin()+vec.size());
                //     }
                //     LOG_INFO(" [cut_link] finish cutting ...");
                // }
            }

            for (auto &poss : line->list) {
                session->link_pos_tree.insert(poss->pos, poss);//切断后link的pose
            }
        }
    }

    // 清空矢量化的分合点
    // session->split_merge_break_points.clear();
    // std::vector<BreakInfo*> split_merge_break_points_tmp;
    // for (auto bp: session->split_merge_break_points)
    // {
    //     bool is_in_junc=false;
    //     for (auto p : resize_junctions) {
    //         is_in_junc = is_in_junc | cal_junc_dist(p, bp->pos);
    //     }
    //     if(is_in_junc) {
    //         split_merge_break_points_tmp.push_back(bp);
    //     }
    // }

    // session->split_merge_break_points.clear();
    // session->split_merge_break_points.swap(split_merge_break_points_tmp);


    return fsdmap::SUCC;
}

int RoadModelProcSampleLine::sample_link(RoadModelSessionData* session) {
    double min_gap = FLAGS_split_road_make_line_sample_gap;//采样的点之间的最小距离; 2m
    //  link 分组
    auto vlink=cluster_link(session);
    //  修改link pos 
    for (auto &link : session->raw_links) {
            double max_dis=DBL_MAX;
            // 增加采样点
            int lane_num=(link->lanenum_sum+1); // 为了保证后面根据轨迹平滑后的link，不出现大折角
            session->sample_line(min_gap*lane_num, link->list, session->key_pose_ptr, link->opt_list);

            for (auto &poss : link->opt_list) {
                session->debug_pos(poss->pos);
                // 把ins的pose映射到link每个点的pose
                get_link_pos_by_trail(session,link.get(), poss);
                max_dis=std::min(max_dis,static_cast<double>(poss->trajector_distribution));
            }
            //  保证是link上最近的一个点；
            for (auto &poss : link->opt_list) {
                poss->trajector_distribution = max_dis;
            }
    }

    for(auto qlink:vlink)
    {
        auto new_line = session->add_ptr(session->key_pose_line_ptr);
        // new_line->id = link->id;
        // 使用最小间隔（min_gap）对路段链接的点进行采样，并生成新的采样点列表
        double length=0;
        for(auto link:qlink)
        {
            length += alg::calc_dis(link->opt_list.front()->pos, link->opt_list.back()->pos);
            std::vector<KeyPose*> new_single_line;
            session->sample_line(min_gap, link->opt_list, session->key_pose_ptr,new_single_line);
            for(auto &p:new_single_line){
              p->from_raw_link = link;
              p->from_link = new_line.get();
            }
            //
            // if(!new_single_line.empty()&&  !new_line->list.empty())
            // {
            //     auto cur_back=new_single_line.back();
            //     auto base_back=new_line->list.back();
            //     cur_back->set_prev(base_back);
            // }
            // 
            new_line->list.insert(new_line->list.end(), new_single_line.begin(), new_single_line.end());
        }
        // 去掉三岔路口内的短线
        if(alg::match_any_with_forms(qlink.front()->forms, {16})&&length<21){
            continue;
        }
        if(qlink.empty())
        {
            continue;
        }
        calc_junction_score(session,new_line.get());
        // 
        // new_line->bind_trail_id=sample_raw_link->bind_trail_id;
        // new_line->link_direction=sample_raw_link->link_direction;
        new_line->same_id=qlink.front()->same_id;
        // new_line->seed_id=sample_raw_link->seed_id;
        // new_line->link_index=sample_raw_link->link_index;
        // 
        smooth_link_pose(new_line->list);
        // 
        // for(int i=1;i<new_line->list.size();i++)
        // {
        // LOG_INFO("dis:{}",alg::calc_dis(new_line->list[i-1]->pos,new_line->list[i]->pos));
        // }
        // TODO write it navate 
        KeyPose* prev_node = NULL;
        double total_length = 0;
        std::vector<KeyPose*> temp;  // TODO 
        for (auto &node : new_line->list) {
            node->line_length = total_length;
            if (prev_node != NULL) {
                if(alg::calc_dis(prev_node->pos,node->pos)<min_gap/4)
                {
                   continue;
                }
                node->set_prev(prev_node);
                prev_node->dir = alg::get_dir(node->pos, prev_node->pos);
                node->dir = prev_node->dir;
                total_length += alg::calc_dis(node->pos, prev_node->pos);
                node->line_length = total_length;
            }
            prev_node = node;
            temp.push_back(node);
        }
        std::swap(new_line->list,temp);
        // 
        session->link_sample_list.push_back(new_line.get());
        for (auto &poss : new_line->list) {
            session->link_pos_tree.insert(poss->pos, poss);//所有link的pose
        }
    }

    return fsdmap::SUCC;
}

// 在ins中搜索与当前link点半径20m最近邻的ins位置，
// 遍历这些最近邻ins点，计算当前link点poss位置处的poss->dir垂线与该link点附近ins位置行驶方向的交点,
// 取出这些交点中与poss->pos距离最小的ins点作为link的pose
int RoadModelProcSampleLine::get_link_pos_by_trail(RoadModelSessionData* session,KeyPoseLine*link,
        KeyPose* poss) {
    double radius = FLAGS_sample_line_link_repair_radius;
    double theta_threshold = FLAGS_sample_line_link_repair_theta;
    double min_dis_threshold = FLAGS_sample_line_link_repair_threshold;
    std::vector<KeyPose*> search_sec;
    session->key_pose_tree.search(poss->pos, radius, search_sec);
    double max_dis=-DBL_MAX;
    double min_dis = DBL_MAX;
    Eigen::Vector3d cross_point;
    KeyPose* min_poss = NULL;
    KeyPose* max_poss = NULL;
    auto v_pos = alg::get_vertical_pos(poss->pos, poss->dir, 50); //计算poss->dir垂线方向上离pos距离是50m的点
    //
    for(auto trail_id:link->bind_trail_id)
    {
        KeyPoseLine kl;
        bool find=false;
        for(auto raw_trail:session->key_pose_map)
        {
            auto  new_trail_id=calc_trail_id(raw_trail.first);
            if(trail_id==new_trail_id)
            {
                find=true;
                kl=raw_trail.second;
            }
            if (find)
            {
                double one_line_min_dis = DBL_MAX;
                KeyPose *one_line_min_poss = NULL;
                for (auto p : kl.list)
                {
                    double theta = alg::calc_theta1(p->dir, poss->dir); // 计算xy平面上两个向量的夹角
                    double dis = alg::calc_dis(p->pos, poss->pos);
                    //  LOG_INFO("theta:{}",theta);
                    theta = fabs(theta);
                    if (dis < one_line_min_dis && dis < min_dis_threshold && theta < theta_threshold)
                    {
                        one_line_min_dis = dis;
                        one_line_min_poss = p;
                        // LOG_INFO("theta:{}",theta);
                        // LOG_INFO("dis:{}  [{} {} {} ] [{} {} {}]",dis,
                        // p->pos.x(),p->pos.y(),p->pos.z(),
                        // poss->pos.x(),poss->pos.y(),poss->pos.z());
                    }
                }
                if(!one_line_min_poss)
                {
                    continue;
                }
                //
                if (one_line_min_dis < min_dis)
                {
                    min_dis = one_line_min_dis;
                    min_poss = one_line_min_poss;
                }
                if (one_line_min_dis > max_dis)
                {
                    max_dis = one_line_min_dis;
                    max_poss = one_line_min_poss;
                }
                //
            }
            //
        }
    }
    //
    if(min_poss)
    {
         poss->pos = min_poss->pos;
         poss->dir = min_poss->dir;
         poss->smooth_score = min_poss->smooth_score;
        //  LOG_INFO("smooth score:{}",min_poss->smooth_score)
    }
    if(min_poss&&max_poss)
    {
        poss-> trajector_distribution=alg::calc_dis(min_poss->pos,max_poss->pos);
    }
   
    // for (auto& tar_pos : search_sec) {
    //     if (tar_pos->next == NULL) {
    //         continue;
    //     }

    //     // bool trail_match=false;
    //     // const auto trail_id=calc_trail_id(tar_pos->line_id);
    //     // for(auto trail:link->bind_trail_id)
    //     // {
    //     //     if(trail_id==trail)
    //     //     {
    //     //         trail_match=true;
    //     //         break;
    //     //     }
    //     // }
    //     // if(!trail_match)
    //     // {
    //     //   continue;
    //     // }

    //     double theta = alg::calc_theta(tar_pos->dir, poss->dir); // 计算xy平面上两个向量的夹角
    //     if (theta > theta_threshold) { // 排除方向不一致的ins点
    //         continue;
    //     }
    //     // 计算当前link点poss位置处的poss->dir垂线与该link点附近ins位置行驶方向的交点
    //     if (!alg::get_cross_point_by_point(tar_pos->pos, tar_pos->next->pos,
    //                 poss->pos, v_pos, cross_point, false, 2, 1000)) {
    //         continue;
    //     }
    //     double dis = alg::calc_dis(cross_point, poss->pos);
    //     if (dis > min_dis_threshold) {
    //         continue;
    //     }
    //     if (min_dis > dis) { // 取出与poss->pos最近邻的ins点中，交点与poss->pos距离最小的ins点作为link的pose
    //         min_dis = dis;
    //         min_poss = tar_pos;
    //     }
    // }
    // if (min_poss != NULL) {
    //     poss->pos = min_poss->pos;
    // }

    return fsdmap::SUCC;
}
void RoadModelProcSampleLine::calc_smoth_score(KeyPoseLine&kl)
{
    auto curvature=[](const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) ->double
    {
        Eigen::Vector2d v1(p1.x() - p0.x(), p1.y() - p0.y());
        Eigen::Vector2d v2(p2.x() - p1.x(), p2.y() - p1.y());
        // 计算向量的叉积
        double crossProduct = v1.x() * v2.y() - v1.y() * v2.x();
        double denominator = std::pow(v1.norm(), 3);
        // 避免除以零
        if (std::abs(denominator) < std::numeric_limits<double>::epsilon()) {
            return 0.0;
        }
        return std::abs(crossProduct) / denominator;
    };
        const int n=kl.list.size();
        if (n <= 2){
            return;
        }
        
        double d=0;
        for(int i=0;i<n-2;i++)
        {
            auto p0=kl.list[i+0];
            auto p1=kl.list[i+1];
            auto p2=kl.list[i+2];
            d=curvature(p0->pos,p1->pos,p2->pos);
            d=alg::calc_score_by_gaussian(d,0.0125,0);
            p0->smooth_score=d;
            // LOG_INFO("curvature:{}",d);
        }

        kl.list[n-2]->smooth_score=d;
        kl.list[n-1]->smooth_score=d;
    
}
int RoadModelProcSampleLine::sample_key_pose(RoadModelSessionData* session) {
    // if (FLAGS_split_road_split_road_by_link_enable) {
    //     return fsdmap::SUCC;
    // }
    double min_gap = FLAGS_sample_line_sample_pose_gap;
    for (auto &it : session->key_pose_map) {
         calc_smoth_score(it.second);
    }

    for (auto &it : session->key_pose_map) {
        // session->sample_line(min_gap, it.second.list, 
        //         session->key_pose_ptr, road_segment->pos_sample_list);
        // 取出key_pose_map.list中pose之间间距满足min_gap的放入session->key_pose_ptr和session->key_pose_list
        session->sample_line(min_gap, it.second.list, 
                session->key_pose_ptr, session->key_pose_list);
        
    }
    for (auto &poss : session->key_pose_list) {
        session->key_pose_tree.insert(poss->pos, poss);
        // LOG_INFO("line id:{}",poss->line_id);
    }
   
    return fsdmap::SUCC;
}

int RoadModelProcSampleLine::smooth_lane_line(RoadModelSessionData* session) {
    double min_gap = FLAGS_sample_line_sample_lane_line_gap;
    int skip_num = FLAGS_sample_line_bev_frame_skip_num;
    double radius1 = FLAGS_sample_line_dir_radius1;
    double radius2 = FLAGS_sample_line_dir_radius2;
    double dir_gap_threshold = FLAGS_sample_line_ll_dir_gap_threshold;
    std::vector<double> radii = {radius1, radius2};
    std::vector<std::vector<std::shared_ptr<LaneFeature>>> tmp_lane_feature_ptr;
    std::vector<std::shared_ptr<LaneFeatureLine>> tmp_lane_line_ptr;
    std::vector<LaneFeatureLine*> tmp_lane_line_list;
    // 等间距采样
    tmp_lane_feature_ptr.reserve(session->sem_lane_instance_map.size());
    // 遍历session->sem_lane_instance_map中车道线的点，间隔4m取点放入tmp_lane_feature_ptr，和tmp_lane_line_ptr
    for (auto &line : session->sem_lane_instance_map) {
        auto new_lane_line = session->add_ptr(tmp_lane_line_ptr);
        tmp_lane_line_list.push_back(new_lane_line.get());
        new_lane_line->id = line.first;
        new_lane_line->src_status = 1;
        auto new_line = new_lane_line.get();
        auto raw_line = &line.second;
        tmp_lane_feature_ptr.resize(tmp_lane_feature_ptr.size() + 1);
        int ptr_index = tmp_lane_feature_ptr.size() - 1;

        session->thread_pool->schedule(
                [&, ptr_index, raw_line, new_line, session, this](
                   utils::ProcessBar *process_bar) {
             auto &tmp_ptr_list = tmp_lane_feature_ptr[ptr_index];
             session->sample_line(min_gap, raw_line->list, 
                 tmp_ptr_list, new_line->list);
            // session->no_sample_line(min_gap, raw_line->list, 
            //      tmp_ptr_list, new_line->list);
        });
    }

    session->thread_pool->wait(1, "sample_lane_line");

    smooth_line(session, tmp_lane_line_list, dir_gap_threshold, radii);
    
    // 在整合了所有的车道线点后，在一起重新等间距采样
    std::vector<LaneLineSampleLine*> tmp_lane_sample_line;
    std::vector<std::vector<std::shared_ptr<LaneLineSample>>> tmp_lane_sample_ptr;
    tmp_lane_sample_ptr.reserve(tmp_lane_line_list.size());
    // 重新采样
    int64_t start_index = 0;
    for (auto line : tmp_lane_line_list) {
        auto new_lane_line = session->add_ptr(session->lane_line_sample_line_ptr);
        tmp_lane_sample_line.push_back(new_lane_line.get());
        new_lane_line->id = line->id;
        new_lane_line->src_status = line->src_status;
        auto new_line = new_lane_line.get();
        auto raw_line = line;
        tmp_lane_sample_ptr.resize(tmp_lane_feature_ptr.size() + 1);
        int ptr_index = tmp_lane_sample_ptr.size() - 1;

        // session->thread_pool->schedule(
        //         [&, ptr_index, raw_line, new_line, session, this](
        //             utils::ProcessBar *process_bar) {
             auto &tmp_ptr_list = tmp_lane_sample_ptr[ptr_index];
             session->sample_line(min_gap, raw_line->list, 
                     tmp_ptr_list, new_line->list);
            // session->no_sample_line(min_gap, raw_line->list, 
            //          tmp_ptr_list, new_line->list);
            // 计算该车道线中车道线上的点方向与该点处轨迹行驶方向是反向的比例    
             float oppo_num = 0;
             if (new_line->list.size() == 0) {
                 // return;
                 continue;
             }
             for (auto &fls : new_line->list) { // Feature Line Segment  车道段的特征
                 fls->src = fls->src->src; // 该线段父节点的父节点
                 fls->score = 1;
                 auto &poss = fls->src->key_pose;
                 if (poss == NULL) {
                     continue;
                 }
                 // 判断车道线上的点方向与该点处轨迹行驶方向是否反向
                 if (alg::calc_theta(fls->dir, poss->dir) > 90) {
                     ++oppo_num;
                 }
             }
             float oppo_rate = oppo_num / new_line->list.size();
             // 比例超过0.5就调换下车道线上点的连接顺序，重新计算朝向dir
             if (oppo_rate > 0.5) {
                 reverse(new_line->list.begin(), new_line->list.end());
                 LaneLineSample* prev = NULL;
                 for (auto &fls : new_line->list) {
                     if (prev == NULL) {
                         prev = fls;
                         continue;
                     }
                     fls->dir = alg::get_dir(fls->pos, prev->pos);
                     prev->dir = fls->dir;

                     fls->set_prev(prev);
                    prev = fls;
                 }
             }
        // });
    }
    // session->thread_pool->wait(1, "sample_lane_line_1");
    for (auto line : tmp_lane_sample_line) {
        if (line->list.size() == 0) {
            continue;
        }
        lane_line_sample_list_opt.push_back(line);
        if (line->src_status == 1) { // 表示车道线来自于bev感知结果
            auto &feature = line->list.front();
            // LOG_INFO("{}", feature->src->frame_id);
            session->bev_frame_lane_line[feature->src->frame_id].push_back(line);
        }
        
    }
    for (int64_t i = start_index; i < tmp_lane_sample_ptr.size(); ++i) {
        auto &tmp_ptr_list = tmp_lane_sample_ptr[i];
        VEC_PUSH_ALL(session->lane_line_sample_ptr, tmp_ptr_list);
    }

    return fsdmap::SUCC;
}

int RoadModelProcSampleLine::smooth_lane_center(RoadModelSessionData* session) {
    double min_gap = FLAGS_sample_line_sample_lane_line_gap;
    int skip_num = FLAGS_sample_line_bev_frame_skip_num;
    double radius1 = FLAGS_sample_line_dir_radius1;
    double radius2 = FLAGS_sample_line_dir_radius2;
    double dir_gap_threshold = FLAGS_sample_line_lc_dir_gap_threshold;
    std::vector<double> radii = {radius1, radius2};
    std::vector<std::shared_ptr<LaneCenterFeature>> tmp_lane_center_feature_ptr;
    std::vector<std::shared_ptr<LaneCenterLine>> tmp_lane_center_line_ptr;
    std::vector<LaneCenterLine*> tmp_lane_center_line_list;

    // 等间距采样
    LOG_INFO("lane_center_instance_map size: {}", session->lane_center_instance_map.size());
    for (auto &line : session->lane_center_instance_map) {
        auto &feature = line.second.list.front();
        auto poss = feature->key_pose;
        // if (feature->key_pose != NULL && feature->key_pose->invalid()) {
        // TODO:qzc, comment in new pipeline
        // if (feature->key_pose == NULL) {
        //     LOG_DEBUG("filter lc line by key_pose[id={}]", feature->frame_id);
        //     continue;
        // }
        // if (poss->line_index % skip_num != 0) {
        //     continue;
        // }
        auto new_lane_line = session->add_ptr(tmp_lane_center_line_ptr);
        new_lane_line->id = line.first;
        session->sample_line(min_gap, line.second.list, 
                tmp_lane_center_feature_ptr, new_lane_line->list);

        tmp_lane_center_line_list.push_back(new_lane_line.get());
    }
    LOG_INFO("tmp_lane_center_line_list size: {}", tmp_lane_center_line_list.size());
    smooth_line(session, tmp_lane_center_line_list, dir_gap_threshold, radii);
    LOG_INFO("tmp_lane_center_line_list after smooth size: {}", tmp_lane_center_line_list.size());
    // 重新采样
    for (auto &line : tmp_lane_center_line_list) {
        auto new_lane_line = session->add_ptr(session->lane_center_line_ptr);
        new_lane_line->id = line->id;
        session->sample_line(min_gap, line->list, 
                session->lane_center_feature_ptr, new_lane_line->list);
        if (new_lane_line->list.size() == 0) {
            continue;
        }
        lane_center_line_sample_list_opt.push_back(new_lane_line.get());
        // if (new_lane_line->src_status == 1) {
            auto &feature = new_lane_line->list.front();
            // LOG_INFO("{}", feature->src->frame_id);
            session->bev_frame_lane_center_line[feature->frame_id].push_back(new_lane_line.get());
        // }
        float oppo_num = 0;
        for (auto &fls : new_lane_line->list) {
            fls->score = 1;
            auto &poss = fls->key_pose;
            if (poss == NULL) {
                continue;
            }
            if (alg::calc_theta(fls->dir, poss->dir) > 90) {
                ++oppo_num;
            }
        }
        float oppo_rate = oppo_num / new_lane_line->list.size();
        if (oppo_rate > 0.5) {
            reverse(new_lane_line->list.begin(), new_lane_line->list.end());
            LaneCenterFeature* prev = NULL;
            for (auto &fls : new_lane_line->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;

                fls->set_prev(prev);
                prev = fls;
            }
        }
    }
    LOG_INFO("lane_center_feature_ptr size: {}", session->lane_center_feature_ptr.size());

    return fsdmap::SUCC;
}


int RoadModelProcSampleLine::smooth_boundary(RoadModelSessionData* session) {
    double min_gap = FLAGS_sample_line_sample_lane_line_gap;
    int skip_num = FLAGS_sample_line_bev_frame_skip_num;
    double radius1 = FLAGS_sample_line_dir_radius1;
    double radius2 = FLAGS_sample_line_dir_radius2;
    std::vector<double> radii = {radius1, radius2};
    double dir_gap_threshold = FLAGS_sample_line_lc_dir_gap_threshold;
    
    std::vector<std::shared_ptr<BoundaryFeature>> tmp_boundary_feature_ptr;
    std::vector<std::shared_ptr<BoundaryLine>> tmp_boundary_line_ptr;
    std::vector<BoundaryLine*> tmp_boundary_line_list;

    // 等间距采样
    // BEV感知的栅栏信息
    // for (auto &line : session->barrier_instance_map) {
    //     auto &feature = line.second.list.front();
    //     auto poss = feature->key_pose;
    //     // if (feature->key_pose != NULL && feature->key_pose->invalid()) {
    //     if (feature->key_pose == NULL) {
    //         LOG_DEBUG("filter boundary line by key_pose[id={}]", feature->frame_id);
    //         continue;
    //     }

    //     if (poss->line_index % skip_num != 0) {
    //         continue;
    //     }
    //     auto new_line = session->add_ptr(tmp_boundary_line_ptr);
    //     new_line->id = line.first;
    //     new_line->src_status = 1;
    //     session->sample_line(min_gap, line.second.list, 
    //             tmp_boundary_feature_ptr, new_line->list);
    //     tmp_boundary_line_list.push_back(new_line.get());
    // }

    // 等间距采样
    // BEV感知的路沿信息
    // for (auto &line : session->curb_instance_map) {
    //     auto &feature = line.second.list.front();
    //     if (feature->key_pose == NULL) {
    //         LOG_DEBUG("filter boundary line by key_pose[id={}]", feature->frame_id);
    //         continue;
    //     }
    //     auto new_line = session->add_ptr(tmp_boundary_line_ptr);
    //     new_line->id = line.first;
    //     new_line->src_status = 1;
    //     session->sample_line(min_gap, line.second.list, 
    //             tmp_boundary_feature_ptr, new_line->list);
    //     tmp_boundary_line_list.push_back(new_line.get());
    // }
    // 等间距采样
    // BEV感知的boundary信息
    for (auto &line : session->sem_curb_instance_map) {
        auto new_line = session->add_ptr(tmp_boundary_line_ptr);
        new_line->id = line.first;
        new_line->src_status = 1;
        if (line.second.boundary_type == LaneType::ISLAND_RB) {
            new_line->boundary_type = line.second.boundary_type;
            session->no_sample_line(min_gap, line.second.list, 
                    tmp_boundary_feature_ptr, new_line->list);
        } else {
            session->sample_line(min_gap, line.second.list, 
                    tmp_boundary_feature_ptr, new_line->list);
        }
        tmp_boundary_line_list.push_back(new_line.get());
    }
    // 人工标记的boundary信息
    // for (auto &line : session->label_boundary_instance_map) {
    //     auto new_line = session->add_ptr(tmp_boundary_line_ptr);
    //     new_line->id = line.first;
    //     new_line->src_status = 4;
    //     session->sample_line(min_gap, line.second.list, 
    //             tmp_boundary_feature_ptr, new_line->list);
    //     tmp_boundary_line_list.push_back(new_line.get());
    // }
    
    smooth_line(session, tmp_boundary_line_list, dir_gap_threshold, radii);

    // 重新采样
    // smooth_line()函数会修改filter_status，sample_line会根据filter_status做一个滤除
    for (auto &line : tmp_boundary_line_list) {
        auto new_line = session->add_ptr(session->boundary_line_ptr);
        new_line->id = line->id;
        new_line->src_status = line->src_status;
        if (line->boundary_type == LaneType::ISLAND_RB) {
            new_line->boundary_type = line->boundary_type;
            session->no_sample_line(min_gap, line->list, 
                    session->boundary_feature_ptr, new_line->list);
        } else {
            session->sample_line(min_gap, line->list, 
                    session->boundary_feature_ptr, new_line->list);
        }
        if (new_line->list.size() == 0) {
            continue;
        }
        // LOG_ERROR("new_line->list.size:{} new_line->src_status:{}",  new_line->list.size(),new_line->src_status);
        if (new_line->src_status == 1) {
            auto &feature = new_line->list.front();
            session->bev_frame_boundary_line[feature->frame_id].push_back(new_line.get());
        }
        float oppo_num = 0;
        for (auto &fls : new_line->list) {
            fls->score = 1;
            auto &poss = fls->key_pose;
            if (poss == NULL) {
                continue;
            }
            if (alg::calc_theta(fls->dir, poss->dir) > 90) {
                ++oppo_num;
            }
        }
        float oppo_rate = oppo_num / new_line->list.size();
        if (oppo_rate > 0.5) {
            reverse(new_line->list.begin(), new_line->list.end());
            BoundaryFeature* prev = NULL;
            for (auto &fls : new_line->list) {
                if (prev == NULL) {
                    prev = fls;
                    continue;
                }
                fls->dir = alg::get_dir(fls->pos, prev->pos);
                prev->dir = fls->dir;

                fls->set_prev(prev);
                prev = fls;
            }
        }
    }
    LOG_INFO("session->bev_frame_boundary_line size:{}",  session->bev_frame_boundary_line.size());

    return fsdmap::SUCC;
}

int RoadModelProcSampleLine::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_sample_line_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("sample_link");
    std::random_device rd;  // 随机数种子
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, 1000);
    for (auto &link : session->raw_links) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "raw_key_pose");
        log->color = {223, 0, 154};
        int i=0;

        for (auto &pt : link->opt_list) {
            auto &ele = log->add(pt->pos);
            srand48(link->same_id);
            ele.label.label = 1E3*drand48();
              // 生成随机数
            int random_number = dis(gen);
            ele.label.intensity_opt=random_number;
            ele.label.score=alg::calc_theta(pt->dir)*57.3;
            ele.label.distance_x_cm=i++;
            ele.label.distance_y_cm=link->link_index;
            ele.label.intensity=pt->trajector_distribution;
            ele.label.opt_label = link->lanenum_sum;

            //form赋值
            auto from_raw_link=pt->from_raw_link;
            if(!from_raw_link)
            {
                continue;
            }
            if (from_raw_link->forms.size() == 1) {
                ele.label.cloud_bev_label = *from_raw_link->forms.begin();
            } 
            else if (from_raw_link->forms.size() >= 2) {
                auto it = from_raw_link->forms.begin();
                ele.label.cloud_bev_label = *it;  // 第一个元素赋值
                ++it; 
                ele.label.cloud_bev_label_score = *it;  // 第二个元素赋值
            }
        }
    }
    session->save_debug_info("sample_link");


    session->set_display_name("sample_trail");
    // auto t_log = session->add_debug_log(utils::DisplayInfo::LINE, "trail_list");
    // t_log->color = {223, 130, 154};
    // for (auto &poss : session->key_pose_list) {
    //     t_log->add(poss->pos);
    // }
    int i=0;
    for (auto &link : session->link_sample_list) {
        auto log = session->add_debug_log(utils::DisplayInfo::LINE, "trail_list{}",i++);
        log->color = {0, 255, 154};
        for (auto &poss : link->list) {
            auto &ele=log->add(poss->pos);
            ele.label.score=alg::calc_theta(poss->dir)*57.3;
            srand48(link->same_id);
            ele.label.label = 1E3*drand48();
            ele.label.intensity_opt=poss->smooth_score;
            ele.label.distance_x_cm=poss->junction_score*100;
            auto from_raw_link=poss->from_raw_link;
            if(!from_raw_link)
            {
                continue;
            }
            ele.label.opt_label = from_raw_link->lanenum_sum;
            ele.label.cloud_bev_shape = from_raw_link->kind;
            ele.label.cloud_bev_color = from_raw_link->link_direction;
            ele.label.cloud_bev_label_2 = from_raw_link->width;
            // ele.label.cloud_bev_label_1 = std::stoi( from_raw_link->form);
    
            //form赋值
            if (from_raw_link->forms.size() == 1) {
                ele.label.cloud_bev_label = *from_raw_link->forms.begin();
            } 
            else if (from_raw_link->forms.size() >= 2) {
                auto it = from_raw_link->forms.begin();
                ele.label.cloud_bev_label = *it;  // 第一个元素赋值
                ++it; 
                ele.label.cloud_bev_label_score = *it;  // 第二个元素赋值
            }
        }
    }
    // for(auto jc:resize_junctions)
    // {
    //     auto log = session->add_debug_log(utils::DisplayInfo::LINE, "junction{}",i++);
    //     log->color = {255, 255, 255};
    //     for(auto p:jc.second)
    //     {
    //        auto &ele=log->add(p);
    //        ele.label.distance_x_cm=0;
    //        ele.label.opt_label =0;
    //        ele.label.label=0;
    //        ele.label.intensity_opt=0;
    //        ele.label.score=0;
    //     }
    // }
    session->save_debug_info("sample_trail");


    // session->set_display_name("sample_sine");
    //     auto tlog = session->add_debug_log(utils::DisplayInfo::LINE, "sample_sine_pose");
    //     tlog->color = {223, 0, 154};
    //     for (auto &pt : test_trjectory) {
    //         auto &ele = tlog->add(pt);
    //     }
    // session->save_debug_info("sample_sine");


    double scope_buff = FLAGS_display_scope_buff;
    session->set_display_name("sample_line");

    
    PTR_VEC<utils::DisplayInfo> log_ptr_list;
    UMAP<std::string, std::vector<utils::DisplayInfo*>> frame_map;
    std::vector<utils::DisplayInfo*> line_id_log;
    for (auto &line : lane_line_sample_list_opt) {
        // TODO:qzc delete
        // continue;

        if (line->src_status != 2) {
            // continue;
        }
        auto &first_feature = line->list.front();
        auto &frame_id = first_feature->src->frame_id;
        if (MAP_NOT_FIND(frame_map, frame_id)) {
            frame_map[frame_id] = std::vector<utils::DisplayInfo*>();
        }
        auto log_ptr = session->add_ptr(log_ptr_list);
        log_ptr->init(utils::DisplayInfo::LINE, "lane_line_sample[id={}]", line->id);
        if (first_feature->src->src_feature != NULL) {
            log_ptr->color = std::get<3>(first_feature->src->src_feature->ele_type);
        } else {
            log_ptr->color = {255, 255, 255};
        }
        log_ptr->log_name = line->id;
        frame_map[frame_id].push_back(log_ptr.get());
        for (auto &feature : line->list) {
            if (feature->src->invalid()) {
                continue;
            }
            log_ptr->add(feature->pos, 1);
            // LOG_INFO("lane sample color:{} type:{}",feature->attr.color,feature->attr.type)
        }

        session->add_debug_log(log_ptr.get());
    }
    for (auto &line : lane_center_line_sample_list_opt) {
        auto &first_feature = line->list.front();
        auto &frame_id = first_feature->frame_id;
        if (MAP_NOT_FIND(frame_map, frame_id)) {
            frame_map[frame_id] = std::vector<utils::DisplayInfo*>();
        }
        auto log_ptr = session->add_ptr(log_ptr_list);
        log_ptr->init(utils::DisplayInfo::LINE, "lane_center_sample[id={}]", line->id);
        // log_ptr->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
        log_ptr->color = std::get<3>(first_feature->src_feature->ele_type);
        log_ptr->log_name = line->id;
        frame_map[frame_id].push_back(log_ptr.get());
        for (auto &feature : line->list) {
            if (feature->invalid()) {
                continue;
            }
            log_ptr->add(feature->pos, -1);
            // LOG_INFO("center sample color:{} type:{}",feature->attr.color,feature->attr.type)
        }

        session->add_debug_log(log_ptr.get());
    }

    for (auto &[frame_id, list] : session->bev_frame_boundary_line) {
        for (auto &line : list) {
            auto &first_feature = line->list.front();
            auto &frame_id = first_feature->frame_id;
            if (MAP_NOT_FIND(frame_map, frame_id)) {
                frame_map[frame_id] = std::vector<utils::DisplayInfo*>();
            }
            auto log_ptr = session->add_ptr(log_ptr_list);
            log_ptr->init(utils::DisplayInfo::LINE, "bev_frame_boundary_line[id={}]", line->id);
            // log_ptr->color = std::get<3>(session->data_processer->ele_type_map[first_feature->type]);
            if (first_feature->src_feature != NULL) {
                log_ptr->color = std::get<3>(first_feature->src_feature->ele_type);
            } else {
                log_ptr->color = {81, 89, 240};
            }
            // auto log_ptr = session->add_debug_log(utils::DisplayInfo::LINE, "bev_frame_boundary_line[id={}]", line->id);
            log_ptr->log_name = line->id;
            frame_map[frame_id].push_back(log_ptr.get());

            for (auto &feature : line->list) {
                if (feature->invalid()) {
                    continue;
                }
                log_ptr->add(feature->pos, -1);
            }
            session->add_debug_log(log_ptr.get());
        }
    }

    if (FLAGS_sample_line_save_data_save_frame_enable) {
        for (auto &it : frame_map) {
            auto key_pose = session->data_processer->get_key_pose(it.first);
            if (key_pose == NULL) {
                continue;
            }
            utils::DisplayScope box(scope_buff, scope_buff, key_pose->pos);
            box.set_resolution(FLAGS_display_scale_rate);
            // box.dir.z() = -key_pose->yaw;
            auto log_name = session->get_debug_dir("{}_{}.png", "lane_sample", it.first);
            utils::save_display_image(log_name.c_str(), box, it.second);

            auto pcd_name = session->get_debug_dir("{}_{}.pcd", "lane_sample", it.first);
            utils::save_display_pcd(pcd_name.c_str(), session->_scope, it.second);
        }
    }
    session->save_debug_info("sample_line");

    return fsdmap::SUCC;
}

}
}
