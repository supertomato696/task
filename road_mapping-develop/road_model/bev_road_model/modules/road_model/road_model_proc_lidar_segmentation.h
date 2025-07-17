

#pragma once

#include "road_model_session_data.h"

namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcLidarSegmentation :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcLidarSegmentation() {};
    virtual ~RoadModelProcLidarSegmentation() {};

    const char * name() {
        return "proc_lidar_segmentation";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    int save_debug_info(RoadModelSessionData* session);

    int preprocess_pcd(RoadModelSessionData* session);

    int boundary_segment(RoadModelSessionData* session);

    int64_t filter_intensity(RoadModelSessionData* session,
        utils::CloudPtr &src_cloud, Eigen::Vector3d* key_pos, KeyPose* poss);
};

}
}
