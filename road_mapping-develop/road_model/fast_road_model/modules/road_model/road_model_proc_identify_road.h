

#pragma once

#include "road_model_session_data.h"

namespace fsdmap
{
        namespace road_model
        {

                class RoadModelProcIdentifyRoad : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
                {
                public:
                        RoadModelProcIdentifyRoad() {};
                        virtual ~RoadModelProcIdentifyRoad() {};

                        const char *name()
                        {
                                return "proc_identify_road";
                        }

                        virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session);

                private:
                        struct BreakCandiate
                        {
                                int priority; //
                                int type;                    //1:车道线末端点;  2:属性变化点;  3:路口分合流交叉点;  6:停止线
                                Eigen::Vector3d raw_dir;
                                Eigen::Vector3d raw_pos;
                                Eigen::Vector3d raw_ps;
                                Eigen::Vector3d raw_pe;
                                Eigen::Vector3d raw_cross_point; //原始交叉点
                                KeyPose *raw_cross_pose=NULL;    //原始最近pose
                                //
                                Eigen::Vector3d pos;  //最终合并后坐标
                                Eigen::Vector3d ps;  //方向起点
                                Eigen::Vector3d pe;  //方向终点
                                Eigen::Vector3d dir; //方向
                                KeyPoseLine *hit_link=NULL;
                                std::vector<RoadObjectInfo *> objs;
                                int same_id;
                                Eigen::Vector3d lane_dir;
                        };

                        int max_id = 0;
                        std::vector<BreakCandiate> break_candidates, break_candidates_type;
                        
                        int gen_stop_line_break_candidate(RoadModelSessionData *session);

                        int gen_lane_break_candidate(RoadModelSessionData *session);

                        bool get_refined_cross_link(RoadModelSessionData *session,
                                                RTreeProxy<KeyPose*, float, 2>& link_tree,
                                                RTreeProxy<LaneCenterFeature *, float, 2>& lane_center_tree,
                                                Eigen::Vector3d bk_pos);
                        // split and merge: 分合流点，此处会修改keypose的位姿
                        int gen_split_merge_candidate(RoadModelSessionData *session);
                        
                        bool check_bc_in_sm_range(RoadModelSessionData* session, BreakCandiate& break_candidate);

                        int merge_break_candidate(RoadModelSessionData *session);
                        int merge_break_candidate_type(RoadModelSessionData *session);

                        int gen_lane_directin(RoadModelSessionData *session);
                        
                        int  update_break_staus_to_pose(RoadModelSessionData* session);

                        int save_debug_info(RoadModelSessionData *session);


                };
        }
}
