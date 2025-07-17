

#pragma once

#include "road_model_session_data.h"

namespace fsdmap
{
    namespace road_model
    {

        /**
         * @brief ����ģ��
         */
        class RoadModelProcFormatNewRoad : public fsdmap::process_frame::ProcBase<RoadModelSessionData>
        {
        public:
            RoadModelProcFormatNewRoad() {};
            virtual ~RoadModelProcFormatNewRoad() {};

            const char *name()
            {
                return "proc_format_new_road";
            }

            /**
             * @brief ���нӿڣ�proc��ܽӿ�
             *
             * @param session_data session �����ݶ��󣬸ô�����˽��
             */
            virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData *session_data);

        private:
            int save_debug_info(RoadModelSessionData *session_data);

            int format_lane_group(RoadModelSessionData *session_data);

            int format_boundary(RoadModelSessionData *session);

            int match_ground(RoadModelSessionData *session);

            int build_lane_relation(RoadModelSessionData *session, bool next);

            utils::CloudPoint *get_ground_point(RoadModelSessionData *session,
                                                pcl::KdTreeFLANN<utils::CloudPoint> &kdtree, Eigen::Vector3d &pos);

            // int format_lane_line(
            //         RoadModelSessionData* session_data,
            //         RoadFormatInfo &road_format_info);
            int format_lane_line(
                RoadModelSessionData *session,
                RoadFormatInfo &road_format_info, int &g_lane_idx);

            RoadLaneBoundaryInfo *gen_lane_boundary(
                RoadModelSessionData *session_data,
                RoadFormatInfo &road_format_info);

            RoadLaneBoundaryInfo *gen_lane_center_line(
                RoadModelSessionData *session,
                RoadFormatInfo &road_format_info);

            int format_lane_line_point(
                RoadModelSessionData *session_data,
                RoadFormatInfo &road_format_info);

            int format_lane_center_point(
                RoadModelSessionData *session,
                RoadFormatInfo &road_format_info);

            int trans_feature_to_lane_line_point(RoadModelSessionData *session_data,
                                                 RoadFormatInfo &road_format_info);

            int format_barrier(RoadModelSessionData *session_data);

            int format_barrier_point(RoadModelSessionData *session_data, RoadFormatInfo &road_format_info);

            int format_curb(RoadModelSessionData *session_data);

            int format_curb_point(RoadModelSessionData *session_data, RoadFormatInfo &road_format_info);

            int bind_boundary_to_lane_group(RoadModelSessionData *session,
                                            RoadLaneGroupInfo *lane_group, RoadSegment *road_segment);

            // int gen_issue(RoadModelSessionData* session_data);

            int format_obj(RoadModelSessionData *session_data);

            // int get_boundary(RoadObjectInfo* road_obj_info, Feature* fp);

            // int get_boundary_other_obj(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp);
            //
            // int get_boundary_stop_obj(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp);

            // bool judge_stop_line_dir(RoadModelSessionData* session, RoadObjectInfo* road_obj_info);
            //
            // int get_boundary_other_obj_v(RoadModelSessionData* session, RoadObjectInfo* road_obj_info, Feature* fp);

            // int get_boundary_point(Feature* fp, RoadObjectInfo* road_obj_info, Eigen::Vector3d pt_11);
            //
            // int get_boundary_point_v(Feature* fp, RoadObjectInfo* road_obj_info, Eigen::Vector3d pt_11);
            //
            // std::string _get_speed_limit_value(int speed_type) const;

            // bool init_traffic_sign_infos(RoadModelSessionData* session, std::string traffic_sign_define_json_file);

            int format_obj_single(RoadObjectInfo *road_obj_info, Feature *fp);

            // void extract_boundary_for_new_boundary(RoadModelSessionData* session,
            //     RoadObjectInfo *obj_info, Feature* fp);
            int check_connect(RoadModelSessionData *session);
             std::vector<BindKeyPosePoint> debug_dis_connect_point;
            
            int is_center_line_missed(RoadModelSessionData *session, RoadLaneBoundaryInfo *prev_center_line, RoadLaneBoundaryInfo *current_center_line);
        };

    }
}
