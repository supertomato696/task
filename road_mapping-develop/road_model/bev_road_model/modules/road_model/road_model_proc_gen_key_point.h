

#pragma once

#include "road_model_session_data.h"


namespace fsdmap {
namespace road_model {

/**
 * @brief ����ģ��
 */
class RoadModelProcGenKeyPoint :
    public fsdmap::process_frame::ProcBase<RoadModelSessionData> {
public:
    RoadModelProcGenKeyPoint() {};
    virtual ~RoadModelProcGenKeyPoint() {};

    const char * name() {
        return "proc_gen_key_point";
    }

    /**
     * @brief ���нӿڣ�proc��ܽӿ�
     *
     * @param session_data session �����ݶ��󣬸ô�����˽��
     */
    virtual fsdmap::process_frame::PROC_STATUS proc(RoadModelSessionData* session);

private:
    struct LineSection{
        std::vector<Eigen::Vector3f> pts;
        Eigen::Vector3f leftPt;
        Eigen::Vector3f rightPt;
        Eigen::Vector3f leftTruncatePt;
        Eigen::Vector3f rightTruncatePt;
        bool flag_tracked = false;
    };

    struct Sector{
        std::map<int, LineSection> lineSections;
        std::vector<LineSection> mergedLineSections;
        std::vector<int> prevIndices;
        std::vector<int> nextIndices;
    };

    struct Segment{
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
        //std::vector<double> ts;
        std::vector<int> anchorIndices;  //这段segment里面anchor点在cloud的索引
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_x; //optimized x coeff
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_y; //optimized y coeff
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> mCoeffs_z; //optimized z coeff

        Segment():cloud(new pcl::PointCloud<pcl::PointXYZ>){}
        void DecideOrientation(const pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                               const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree);
        bool QpSpline();
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud();
        pcl::PointCloud<pcl::PointXYZ>::Ptr smoothTrjCloud2();

        void selectAnchor(int stride);
    };

    int get_lane_key_point(RoadModelSessionData* session);

    int get_roadboundary_key_point(RoadModelSessionData* session);

    int make_point_tree(RoadModelSessionData* session);

    int get_key_point(RoadModelSessionData* session, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_ptr, pcl::PointCloud<PointXYZ_OPT>::Ptr& key_points, std::string keystr);

    int save_debug_info(RoadModelSessionData* session);

    int label_ConditionalRemoval(const utils::CloudPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& labelCloud,uint16_t opt_label);

    int DBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float radius, int minPts,
               std::vector<std::vector<int>> &clusters);

    int cal_ref_info(const pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor);

    int ExtractSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Segment> &segments,
                                                       float radius, int minPts,float eigenvaluediff);
    int sort_segments(std::vector<std::pair<int, float>>& disp, std::vector<Segment> &segments,pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud,pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor);

    //������������
    int EigenSolverEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud, double radius, pcl::PointCloud <pcl::Normal>::Ptr& cloud_normals);
};

}
}
