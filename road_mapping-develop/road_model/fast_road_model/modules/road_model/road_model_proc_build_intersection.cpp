#include "road_model_proc_build_intersection.h"
#include "utils/algorithm_util.h"

DEFINE_bool(build_intersection_enable, false, "build_intersection_enable");
namespace fsdmap
{
    namespace road_model
    {
        fsdmap::process_frame::PROC_STATUS RoadBuildIntersection::proc(
            RoadModelSessionData *session)
        {
            if(!FLAGS_build_intersection_enable)
            {
                 return fsdmap::process_frame::PROC_STATUS_SUCC;
            }
            get_in_out_lane_groups(session);
            //
            build_inter(session);
            //
            add_virture_lane_group(session);
            // 
            save_debug_info(session);
            //
            return fsdmap::process_frame::PROC_STATUS_SUCC;
        }
        void RoadBuildIntersection::get_in_out_lane_groups(RoadModelSessionData *session)
        {
            auto get_arrows = [&](RoadLaneInfo *lane_info)->std::vector<std::string> 
            {
                std::vector<std::string> turn_type;
                std::map<std::string ,std::vector<std::string>> hash_map;
                hash_map["0"]={"unknow"};
                hash_map["1"]={"forward"};
                hash_map["2"]={"turn_right"};
                hash_map["3"]={"turn_left"};
                hash_map["4"]={"turn_left","back"};
                hash_map["5"]={"turn_right","back"};
                const auto arrows = lane_info->bind_arrow_list;
                for (const auto arrow : arrows)
                {
                    if (arrow->ele_type != 3)
                    {
                        continue;
                    }
                    if (arrow)
                    {
                         char delimiter = ',';
                         std::vector<std::string> tokens;
                         boost::split(tokens, arrow->type, boost::is_any_of(&delimiter));
                         for(auto token:tokens)
                         {
                             if(hash_map.count(token))
                             {
                                for(auto type:hash_map.at(token))
                                {
                                    turn_type.push_back(type);
                                }
                               
                             }
                         }
                        //  LOG_INFO("arrow type: {}", arrow->type);
                    }
                }
                return turn_type;
            };

            std::shared_ptr<InterInfo> inter_info_ptr = std::make_shared<InterInfo>();
            for (auto &inter : session->raw_intersections)
            {
                std::set<RoadLaneGroupInfo *> in_lane_group;
                std::set<RoadLaneGroupInfo *> out_lane_group;
                for (auto p : inter->in_lanes)
                {
                    if (p)
                    {
                        in_lane_group.insert(p->lane_group_info);
                    }
                }
                for (auto p : inter->out_lanes)
                {
                    if (p)
                    {
                        out_lane_group.insert(p->lane_group_info);
                    }
                }
                //
                for (auto in_group : in_lane_group)
                {
                    InterInfo::LaneGroup lane_group;
                    // lane_group.praw_lane_group=in_group;
                    for (auto in : in_group->lane_line_info)
                    {
                        InterInfo::KeyPoint kp;
                        kp.praw_lane_line=in;
                        auto back_point = in->center_lane_boundary_info->line_point_info.back();
                        kp.pos = back_point->pos;
                        kp.dir = back_point->dir;
                        kp.turn_type=get_arrows(in);
                        lane_group.key_points.push_back(kp);
                    }

                    // if(!lane_group.key_points.empty())
                    // {
                    //     //  LOG_INFO(" hor_dis||{}",lane_group.key_points.size());
                    //     //  auto &front=lane_group.key_points.front();
                    //     //  std::sort(lane_group.key_points.begin(),lane_group.key_points.end(),
                    //     //           [&](const  InterInfo::KeyPoint &a, const InterInfo::KeyPoint&b)
                    //     //  {
                    //     //    auto  a_h_dis= alg::calc_hori_dis(a.pos, front.pos, front.dir);
                    //     //    auto  b_h_dis= alg::calc_hori_dis(b.pos, front.pos, front.dir);
                    //     // //    LOG_INFO(" hor_dis[{} {}]",a_h_dis,b_h_dis);
                    //     //    return a_h_dis<b_h_dis;
                    //     //  });
                    // }
                    if(!lane_group.key_points.empty())
                    {
                        // only for debug curve point genarate 
                        // auto &front=lane_group.key_points.front();
                        // auto &back=lane_group.key_points.back();
                        // front.turn_type.push_back("turn_left");
                        // back.turn_type.push_back("turn_right");
                    }
                    inter_info_ptr->in_groups.push_back(lane_group);
                }

                for (auto out_group : out_lane_group)
                {
                    InterInfo::LaneGroup lane_group;
                    // lane_group.praw_lane_group=out_group;
                    for (auto out : out_group->lane_line_info)
                    {
                        InterInfo::KeyPoint kp;
                        kp.praw_lane_line=out;
                        auto back_point = out->center_lane_boundary_info->line_point_info.front();
                        kp.pos = back_point->pos;
                        kp.dir = back_point->dir;
                        lane_group.key_points.push_back(kp);
                        // LOG_INFO("inter pos:[{} {} {} ] dir:[{} {} {}]",
                        //          kp.pos.x(), kp.pos.y(), kp.pos.z(),
                        //          kp.dir.x(), kp.dir.y(), kp.dir.z());
                    }
                    // if(!lane_group.key_points.empty())
                    // {
                    //      auto &front=lane_group.key_points.front();
                    //      std::sort(lane_group.key_points.begin(),lane_group.key_points.end(),[&](const InterInfo::KeyPoint a,const InterInfo::KeyPoint&b)
                    //      {
                    //        auto  a_h_dis= alg::calc_hori_dis(a.pos, front.pos, front.dir);
                    //        auto  b_h_dis= alg::calc_hori_dis(b.pos, front.pos, front.dir);
                    //        return a_h_dis<b_h_dis;
                    //      });
                    // }

                    inter_info_ptr->out_groups.push_back(lane_group);
                }
                if (!inter_info_ptr->out_groups.empty() && !inter_info_ptr->in_groups.empty())
                {
                    session->inter_info_ptr.push_back(inter_info_ptr);
                    session->inter_infos.push_back(inter_info_ptr.get());
                }
            }
        }
        void RoadBuildIntersection::build_inter(RoadModelSessionData *session)
        {
           auto inter_infos=session->inter_infos;
           for(auto inter:inter_infos)
           {
             build_one_inter(*inter);
           }
        }
        void RoadBuildIntersection::build_one_inter(InterInfo &inter)
        {
            auto &virtual_lane_groups = inter.virtual_groups;
            const std::string turn_types[] = {"turn_left", "forward", "turn_right"};
            for (auto turn_type : turn_types)
            {
                for (auto in_lane : inter.in_groups)
                {
                    const auto sub_lane_gropus = get_sub_lane_group(in_lane, turn_type);
                    for (auto out_lane : inter.out_groups)
                    {
                        // TODO
                        auto new_virtual_lane_group = genarate_virture_lane_group(sub_lane_gropus, out_lane, turn_type);
                        if (!new_virtual_lane_group.lane_centers.empty())
                        {
                            virtual_lane_groups.push_back(new_virtual_lane_group);
                        }
                    }
                }
            }
        }
        InterInfo::LaneGroup RoadBuildIntersection::genarate_virture_lane_group(const InterInfo::LaneGroup &in_groups, const InterInfo::LaneGroup &out_groups, const std::string &turn_type)
        {
            // row col
            Eigen::Matrix<int, -1, -1> connect_matrix = connect_rule(in_groups, out_groups, turn_type);
            const int rows = connect_matrix.rows();
            const int cols = connect_matrix.cols();
            const auto &in_key_points = in_groups.key_points;
            const auto &out_key_points = out_groups.key_points;
            InterInfo::LaneGroup virtual_lane_groups;
            // auto &key_points=virtual_lane_groups.key_points;
            auto &lane_centers = virtual_lane_groups.lane_centers;
            // LOG_INFO("connect_matrix[{} {}]",rows,cols);
            for (int i = 0; i < rows; i++)
            {
                for (int j = 0; j < cols; j++)
                {

                    if (0 == connect_matrix(i, j))
                    {
                        continue;
                    }
                    if (!is_access(in_key_points[i], out_key_points[j], turn_type))
                    {
                        continue;
                    }
                    auto virtual_lane = create_one_connect(in_key_points[i], out_key_points[j]);
                    if (!virtual_lane.point_list.empty())
                    {
                        lane_centers.push_back(virtual_lane);
                    }
                }
            }
            return virtual_lane_groups;
        }
        Eigen::Matrix<int, -1, -1> RoadBuildIntersection::connect_rule(const InterInfo::LaneGroup &in_groups, const InterInfo::LaneGroup &out_groups, const std::string &turn_type)
        {
            int n = in_groups.key_points.size();
            int m = out_groups.key_points.size();
            Eigen::Matrix<int, -1, -1> connect_info;
            if(n==0||m==0)
            {
                // LOG_WARN("matrix[{} {} ]",n,m);
                return connect_info;
            }
     
            connect_info.resize(n, m);
            connect_info.setZero();
            if (n == 1)
            {
                for (int j = 0; j < m; j++)
                    connect_info(0, j) = 1;
            }
            else if (n * 2 == m) // TODO ->3
            {
                for (int j = 0; j < n; j++)
                {
                    connect_info(j, 2 * j + 0) = 1;
                    connect_info(j, 2 * j + 1) = 1;
                }
            }
            else if (n == 2 * m)
            {
                for (int j = 0; j < m; j++)
                {
                    connect_info(2 * j + 0, j) = 1;
                    connect_info(2 * j + 1, j) = 1;
                }
            }
            else if (turn_type == "forward" || turn_type == "turn_left")
            {
                // TODO
                int min_num = std::min(n, m);
                for (int j = 0; j < min_num; j++)
                {
                    connect_info(j, j) = 1;
                }
                if (n < m)
                {
                    for (int j = n - 1; j < m; j++)
                        connect_info(n - 1, j) = 1;
                }
                else
                {
                    for (int j = m - 1; j < n; j++)
                        connect_info(j, m - 1) = 1;
                }
            }
            else if (turn_type == "turn_right")
            {
                int min_num = std::min(n, m);
                for (int j = 0; j < min_num; j++)
                {
                    connect_info(n - 1 - j, m - 1 - j) = 1;
                }
                if (n < m)
                {
                    for (int j = n - 1; j < m; j++)
                        connect_info(0, m - 1 - j) = 1;
                }
                else
                {
                    for (int j = m - 1; j < n; j++)
                        connect_info(n - 1 - j, 0) = 1;
                }
            }
            return connect_info;
        }

        Eigen::Vector3d RoadBuildIntersection::get_cross(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2, bool &status)
        {
            Eigen::Vector3d ret;
            status = alg::get_cross_point(p1.pos, p1.dir, p2.pos, p2.dir, ret);
            return ret;
        }
        Eigen::Vector3d RoadBuildIntersection::calc_center(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2)
        {
            return (p1.pos + p2.pos) / 2;
        }
        bool RoadBuildIntersection::in_middle(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2, const Eigen::Vector3d &center)
        {
            bool infront = alg::judge_front(center, p1.pos, p1.dir);
            bool inback = !alg::judge_front(center, p2.pos, p2.dir);
            return infront && inback;
        }
        std::vector<Eigen::Vector3d> RoadBuildIntersection::genarate_spline(const std::vector<Eigen::Vector3d> &ctrl, int max_num, double res)
        {
            // TODO 优化计算
            auto f = [=](const double &t, const int &i) -> double
            {
                const double scale = 1.0 / 6.0;
                const double c[4] = {
                    scale * (-std::pow(t, 3) + 3 * std::pow(t, 2) - 3 * t + 1),
                    scale * (3 * std::pow(t, 3) - 6 * std::pow(t, 2) + 4),
                    scale * (-3 * std::pow(t, 3) + 3 * std::pow(t, 2) + 3 * t + 1),
                    scale * std::pow(t, 3)};
                return c[i];
            };
            auto sparse=[=](const std::vector<Eigen::Vector3d>& curve_points)
            {
                std::vector<Eigen::Vector3d> ret_point;
                ret_point.push_back(curve_points.front());
                for(int i=1;i<curve_points.size();i++)
                {
                      if(alg::calc_dis(ret_point.back(),curve_points[i])>=res)
                      {
                          ret_point.push_back(curve_points[i]);
                      }
                }
                if(ret_point.size()>=2)
                {
                    if(alg::calc_dis(curve_points.back(), ret_point.back())<=res)
                    {
                        const int n=ret_point.size();
                        ret_point.resize(n-1);
                    }
                    ret_point.push_back(curve_points.back());
                }
                return ret_point;

            };
            //  TODO
            const int point_num = ctrl.size();
            std::vector<Eigen::Vector3d> new_ctrl;
            std::vector<Eigen::Vector3d> curve_points;
            new_ctrl.push_back(ctrl.front());
            new_ctrl.insert(new_ctrl.end(), ctrl.begin(), ctrl.end());
            new_ctrl.push_back(ctrl.back());
            new_ctrl.push_back(ctrl.back());
            double dt = 1.0 / static_cast<double>(max_num);
            for (int i = 0; i < point_num; i++)
            {
                Eigen::Vector3d p1 = new_ctrl[i + 0];
                Eigen::Vector3d p2 = new_ctrl[i + 1];
                Eigen::Vector3d p3 = new_ctrl[i + 2];
                Eigen::Vector3d p4 = new_ctrl[i + 3];
                for (int j = 0; j < max_num; j++)
                {
                    double t = dt * j;
                    Eigen::Vector3d p = f(t, 0) * p1 + f(t, 1) * p2 + f(t, 2) * p3 + f(t, 3) * p4;
                    curve_points.push_back(p);
                }
            }
            return sparse(curve_points);
        }
        InterInfo::LaneGroup RoadBuildIntersection::get_sub_lane_group(const InterInfo::LaneGroup &in_groups, std::string &turn_type)
        {
            InterInfo::LaneGroup sub_lane_group_ret;
            for (auto kp : in_groups.key_points)
            {
                if (kp.type_match(turn_type))
                {
                    sub_lane_group_ret.key_points.push_back(kp);
                }
            }
            // TODO  出现特殊位置属性未知进行 脑补 左一 右一  单车道
            //
            return sub_lane_group_ret;
        }
        bool RoadBuildIntersection::is_access(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2, const std::string &turn_type)
        {
            const double turn_angle = 45;
            double theta = alg::calc_theta(p1.dir, p2.dir);
            // LOG_INFO("angle:[ {}---{} ]",theta,turn_type);
            if (fabs(theta) < turn_angle && turn_type == "forward")
            {
                // LOG_INFO("pass angle:[ {} {} ]",theta,turn_type );
                return true;
            }
            if (theta > turn_angle && theta < (180 - turn_angle) && turn_type == "turn_left")
            {
                //  LOG_INFO("pass angle:[ {} {} ]",theta,turn_type );
                return true;
            }
            if (theta < -turn_angle && theta > -(180 - turn_angle) && turn_type == "turn_right")
            {
                //  LOG_INFO("pass angle:[ {} {} ]",theta,turn_type );
                return true;
            }
            // TODO  通过众包轨迹
            // LOG_INFO("not pass angle:[ {} {} ]",theta,turn_type );
            return false;
        }

        InterInfo::VirtualLane RoadBuildIntersection::create_one_connect(const InterInfo::KeyPoint &p1, const InterInfo::KeyPoint &p2)
        {
            // LOG_INFO("start calc connect ");
            int flags = 0;
            bool status;
            InterInfo::VirtualLane virtual_lane;
            std::vector<Eigen::Vector3d> points;
            Eigen::Vector3d center_point = get_cross(p1, p2, status);
            if (status != true)
            {
                // LOG_INFO("calc cross failed  ");
                flags = 1;
                center_point = calc_center(p1, p2);
            }
            status = in_middle(p1, p2, center_point);
            if (status != true)
            {
                // LOG_INFO("not in center  ");
                flags = 2;
                return virtual_lane;
            }
            std::vector<Eigen::Vector3d> ctrl = {p1.pos, center_point, p2.pos};
            points = genarate_spline(ctrl, 20, 1);
            auto &center_list = virtual_lane.point_list;
            // LOG_INFO("curve pints:  ",points.size());
            for (auto p : points)
            {
                InterInfo::VirtualLane::Point vp;
                vp.pos = p;
                vp.width = 1.5;
                center_list.emplace_back(vp);
            }
            virtual_lane.start_end_point={p1,p2};

            return virtual_lane;
        }
       std::vector<Eigen::Vector3d> RoadBuildIntersection::get_point(RoadLaneInfo* lane_info,bool is_in)
        {
                std::vector<Eigen::Vector3d > ret_points;
                if(lane_info==NULL)
                {
                    return ret_points;
                }
                auto left=lane_info->left_lane_boundary_info;
                auto right=lane_info->right_lane_boundary_info;
                if(NULL==left||NULL==right)
                {
                    return ret_points;
                }
                if(left->line_point_info.empty()||right->line_point_info.empty())
                {
                    return ret_points;
                }
                auto left_point=is_in?left->line_point_info.back():left->line_point_info.front();
                auto right_point=is_in?right->line_point_info.back():right->line_point_info.front();
                ret_points={left_point->pos,right_point->pos};
                
                return ret_points;
         } 
         std::vector<Eigen::Vector3d >RoadBuildIntersection:: gnarate_lane_boundary_geo(const InterInfo::VirtualLane&vl,bool is_left)
         {
             std::vector<Eigen::Vector3d > boundary_geo;
              if(vl.start_end_point.empty())
                {
                    return boundary_geo;
                }
               fsdmap::RoadLaneInfo* in_raw_lane_line= static_cast<RoadLaneInfo*>(vl.start_end_point.front().praw_lane_line);
               fsdmap::RoadLaneInfo* out_raw_lane_line=static_cast<RoadLaneInfo*>(vl.start_end_point.back().praw_lane_line);
               auto in_points=get_point(in_raw_lane_line,true);
               auto out_points=get_point(out_raw_lane_line,false);
               if(in_points.empty()||out_points.empty())
               {
                 return boundary_geo;
               }
               std::vector<double>  lane_widths;
               double d1=alg::calc_dis(in_points[0],in_points[1]);
               double d2=alg::calc_dis(out_points[0],out_points[1]);
               double num_point=vl.point_list.size();
               for(int i=0;i<num_point;i++)
               {
                double scale=1.0-static_cast<double>(i)/(num_point);
                double d=d1*scale+d2*(1.0-scale);
                lane_widths.push_back(d);
               }
              boundary_geo.push_back(in_points[is_left?0:1]);
              for(int i=1;i<num_point-1;i++)
              {
                     auto half_width=lane_widths[i]/2.0;
                     auto pos=vl.point_list[i].pos;
                     auto  dir=alg::get_dir(vl.point_list[i-1].pos,vl.point_list[i].pos);
                     Eigen::Vector3d left_point=alg::get_vertical_pos(pos,dir,half_width*(is_left?1.0:-1.0));
                    //  Eigen::Vector3d right_point=alg::get_vertical_pos(pos,dir,-width);
                     boundary_geo.push_back(left_point);
        
              }
            boundary_geo.push_back(out_points[is_left?0:1]);
           return boundary_geo;
         }

        RoadLaneInfo*  RoadBuildIntersection::creat_new_lane_line(RoadModelSessionData *session,  RoadLaneGroupInfo* lane_group,const InterInfo::VirtualLane&vl)
        {
            const int num_point= vl.point_list.size();
            auto left_boundary_geo=gnarate_lane_boundary_geo(vl,true);
            auto right_boundary_geo=gnarate_lane_boundary_geo(vl,false);
            if(left_boundary_geo.size()!=right_boundary_geo.size()||left_boundary_geo.size()<=2)
            {
                LOG_INFO("boundary geo size[{}  {}  {} ]",left_boundary_geo.size(),
                                                          right_boundary_geo.size(),
                                                          num_point);
                return NULL;
            }
                auto tar_lane = session->add_ptr(session->road_lane_line_ptr);
                tar_lane->lane_group_info=lane_group;
                do {//center
                    auto lane_boundary = session->add_ptr(session->lane_boundary_info_ptr);
                    for(int j=0;j<num_point;j++)
                    {
                            auto new_line_point = session->add_ptr(session->line_point_info_ptr);
                            new_line_point->pos = vl.point_list[j].pos;
                            new_line_point->src_status = 2;
                            lane_boundary->line_point_info.push_back(new_line_point.get());
                            new_line_point->lane_boundary = lane_boundary.get();
                            new_line_point->lane_group = lane_group;
                            // 
                            // todo  add dir 
                    }
                    tar_lane->center_lane_boundary_info=lane_boundary.get();
                    lane_group->lane_boundary_info.push_back(lane_boundary.get());
                }while(0);
            
                 
                do {//left
                    auto lane_boundary = session->add_ptr(session->lane_boundary_info_ptr);
                    for(int j=0;j<num_point;j++)
                    {
                            auto new_line_point = session->add_ptr(session->line_point_info_ptr);
                            new_line_point->pos =left_boundary_geo[j];
                            new_line_point->src_status = 2;
                            lane_boundary->line_point_info.push_back(new_line_point.get());
                            new_line_point->lane_boundary = lane_boundary.get();
                            new_line_point->lane_group = lane_group;
                            // 
                            // todo  add dir 
                    }
                     tar_lane->left_lane_boundary_info=lane_boundary.get();
                     lane_group->lane_boundary_info.push_back(lane_boundary.get());
                }while(0);

                 do {//right
                    auto lane_boundary = session->add_ptr(session->lane_boundary_info_ptr);
                    for(int j=0;j<num_point;j++)
                    {
                            auto new_line_point = session->add_ptr(session->line_point_info_ptr);
                            new_line_point->pos = right_boundary_geo[j];
                            new_line_point->src_status = 2;
                            lane_boundary->line_point_info.push_back(new_line_point.get());
                            new_line_point->lane_boundary = lane_boundary.get();
                            new_line_point->lane_group = lane_group;
                            // 
                            // todo  add dir 
                    }
                    tar_lane->right_lane_boundary_info=lane_boundary.get();
                    lane_group->lane_boundary_info.push_back(lane_boundary.get());
                }while(0);

                return  tar_lane.get();
            };

            void  RoadBuildIntersection::creat_new_virtual_group(RoadModelSessionData *session,const InterInfo::LaneGroup& lane_group)
            {
                auto add_new_lane_group=session->add_ptr(session->lane_group_info_ptr);
                //
                for(auto vl:lane_group.lane_centers)
                {
                   auto road_lane_line=creat_new_lane_line(session,add_new_lane_group.get(),vl);
                   if(road_lane_line)
                   {
                    add_new_lane_group->lane_line_info.push_back(road_lane_line);
                   }
                   
                }
                // LOG_INFO("new add lane {}",add_new_lane_group->lane_line_info.size());
                // 
                auto &new_lane_groups=session->new_lane_groups;
                new_lane_groups.push_back(add_new_lane_group.get());
               
            };

        void RoadBuildIntersection::add_virture_lane_group(RoadModelSessionData *session)
        {
            auto inter_infos=session->inter_infos;
            for (auto one_inter : inter_infos)
            {
                for(auto virtual_group:one_inter->virtual_groups)
                {
                  creat_new_virtual_group(session,virtual_group);
                }
            }

        };
        int RoadBuildIntersection::save_debug_info(RoadModelSessionData *session)
        {
            session->set_display_name("inter_info");
            const auto inter_infos = session->inter_infos;
            for (auto one_inter : inter_infos)
            {
                for (auto in_group : one_inter->in_groups)
                {
                    int i = 0;
                    for (auto kp : in_group.key_points)
                    {
                        auto point_log = session->add_debug_log(utils::DisplayInfo::POINT, "inter_points");
                        point_log->color = {255, 0, 0};
                        auto &ele = point_log->add(kp.pos);
                        ele.label.label = i++;
                        ele.label.intensity_opt = alg::calc_theta(kp.dir);
                        //  LOG_INFO("inter pos:[{} {} {} ] dir:[{} {} {}] index:[{}]",
                        //          kp.pos.x(), kp.pos.y(), kp.pos.z(),
                        //          kp.dir.x(), kp.dir.y(), kp.dir.z(),i);
                    }
                }
                for (auto out_group : one_inter->out_groups)
                {
                    int i = 0;
                    for (auto kp : out_group.key_points)
                    {
                        auto point_log = session->add_debug_log(utils::DisplayInfo::POINT, "inter_points");
                        point_log->color = {0, 255, 0};
                        point_log->add(kp.pos);
                        auto &ele = point_log->add(kp.pos);
                        ele.label.label = i++;
                        ele.label.intensity_opt = alg::calc_theta(kp.dir);
                        //  LOG_INFO("inter pos:[{} {} {} ] dir:[{} {} {}] index:[{}]",
                        //          kp.pos.x(), kp.pos.y(), kp.pos.z(),
                        //          kp.dir.x(), kp.dir.y(), kp.dir.z(),i);
                    }
                }
                int i = 0;
                for(auto virtual_group:one_inter->virtual_groups)
                {
                    for (auto lc : virtual_group.lane_centers)
                    {
                        i++;
                        auto line_log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "virtual lane center{}", i);
                        line_log->color = {0, 0, 255};
                        for(auto kp:lc.point_list)
                        {
                            line_log->add(kp.pos);
                        }
                        // LOG_INFO("virtual line :[{}-> {} ]",i,lc.point_list.size());
                    }
                }
                //   LOG_INFO("virtual line group:[{}]",one_inter->virtual_groups.size());
            }
            session->save_debug_info("inter_info");
            return fsdmap::SUCC;
        }

    }
}