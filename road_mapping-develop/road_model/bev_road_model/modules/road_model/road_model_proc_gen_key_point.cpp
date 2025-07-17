


#include "road_model_proc_gen_key_point.h"
#include <Eigen/Eigenvalues>
#include <osqp/osqp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/boundary.h> //边界提取


//lane 和 boundary采用不用的参数配置

DEFINE_bool(gen_key_point_enable, false, "gen_key_point_enable");
DEFINE_bool(gen_key_point_debug_pos_enable, true, "gen_key_point_debug_enable");
DEFINE_bool(gen_key_point_save_data_enable, true, "gen_key_point_save_data_enable");

DEFINE_bool(use_opt_groud_pcd_file, false, "use_opt_groud_pcd_file");
DEFINE_string(gen_key_point_input_pcd, "/home/biyadi/data/bevdata/genkeypoint_input.pcd", "gen_key_point_input_pcd");
DEFINE_bool(use_opt_road_edge_pcd_file, false, "use_opt_road_edge_pcd_file");
DEFINE_string(road_edge_input_pcd, "/home/biyadi/data/bevdata/genkeypoint_road_edge.pcd", "road_edge_input_pcd");
DEFINE_bool(gen_key_point_use_cache, false, "gen_key_point_use_cache");
DEFINE_string(gen_key_point_cache_lane_file, "../lane_cache.pcd", "gen_key_point_cache_lane_file");
DEFINE_string(gen_key_point_cache_boundary_file, "../boundary_cache.pcd", "gen_key_point_cache_boundary_file");
DEFINE_string(gen_key_point_cache_ground_file, "../ground_cache.pcd", "gen_key_point_cache_ground_file");
DEFINE_double(lane_dbscan_radius, 0.5, "lane_dbscan_radius");
DEFINE_int32(lane_dbscan_minPts, 10, "lane_dbscan_minPts");
DEFINE_double(lane_extract_segment_radius, 1.0, "lane_extract_segment_radius");
DEFINE_int32(lane_extract_segment_minPts, 3, "lane_extract_segment_minPts");
DEFINE_double(lane_eigenvaluediff, 25.0, "lane_eigenvaluediff");

DEFINE_double(boundary_dbscan_radius, 0.5, "boundary_dbscan_radius");
DEFINE_int32(boundary_dbscan_minPts, 3, "boundary_dbscan_minPts");
DEFINE_double(boundary_extract_segment_radius, 0.25, "boundary_extract_segment_radius");
DEFINE_int32(boundary_extract_segment_minPts, 3, "boundary_extract_segment_minPts");
DEFINE_double(boundary_eigenvaluediff, 5.0, "boundary_eigenvaluediff");

DEFINE_double(resolution, 3.0, "resolution");


DECLARE_double(display_scope_buff);

namespace fsdmap {
namespace road_model {

fsdmap::process_frame::PROC_STATUS RoadModelProcGenKeyPoint::proc(
        RoadModelSessionData* session) {
    if (!FLAGS_gen_key_point_enable) {
        return fsdmap::process_frame::PROC_STATUS_DISABLE;
    }
    session->enable_debug_pos = FLAGS_gen_key_point_debug_pos_enable;
    std::string cache_file_lane = session->get_debug_dir(FLAGS_gen_key_point_cache_lane_file);         
    std::string cache_file_boundary = session->get_debug_dir(FLAGS_gen_key_point_cache_boundary_file); 
    std::string cache_file_ground = session->get_debug_dir(FLAGS_gen_key_point_cache_ground_file); 
    
    if (FLAGS_gen_key_point_use_cache) {
        session->lane_line_lidar_pcd.reset(new pcl::PointCloud<PointXYZ_OPT>);
        session->boundary_lidar_pcd.reset(new pcl::PointCloud<PointXYZ_OPT>);
        session->opt_ground_pcd.reset(new utils::Cloud);
        // if (pcl::io::loadPCDFile<PointXYZ_OPT>(cache_file_lane.c_str(), 
        //             *session->lane_line_lidar_pcd) == -1) {
        //      LOG_ERROR("failed to read lane pcd");
        //      return fsdmap::process_frame::PROC_STATUS_FAIL;
        // }
        if (pcl::io::loadPCDFile<PointXYZ_OPT>(cache_file_boundary.c_str(), 
                    *session->boundary_lidar_pcd) == -1) {
             LOG_ERROR("failed to read boundary pcd");
             return fsdmap::process_frame::PROC_STATUS_FAIL;
        }
        if (pcl::io::loadPCDFile<utils::CloudPoint>(cache_file_ground.c_str(), 
                    *session->opt_ground_pcd) == -1) {
             LOG_ERROR("failed to read ground pcd");
             return fsdmap::process_frame::PROC_STATUS_FAIL;
        }
    } else {
        if (FLAGS_use_opt_groud_pcd_file)
        {
            //首先读取点云数据
            utils::CloudPtr all_pc_ptr(new pcl::PointCloud<utils::CloudPoint>);
            pcl::io::loadPCDFile(FLAGS_gen_key_point_input_pcd, *all_pc_ptr);
            session->opt_ground_pcd = all_pc_ptr;
        }
        

        CHECK_FATAL_PROC(get_lane_key_point(session), "get_lane_key_point");

        CHECK_FATAL_PROC(get_roadboundary_key_point(session), "get_roadboundary_key_point");
    }

    CHECK_FATAL_PROC(make_point_tree(session), "make_point_tree");
    if (!FLAGS_gen_key_point_use_cache) {
        boost::filesystem::path path(cache_file_lane);
        if (!boost::filesystem::exists(path.parent_path())) {
            boost::filesystem::create_directories(path.parent_path());
        }
        if (session->lane_line_lidar_pcd && session->lane_line_lidar_pcd->size() > 0) {
            pcl::io::savePCDFileBinary(cache_file_lane.c_str(), 
                    *session->lane_line_lidar_pcd);
        }

        if (session->boundary_lidar_pcd && session->boundary_lidar_pcd->size() > 0) {
            pcl::io::savePCDFileBinary(cache_file_boundary.c_str(), 
                    *session->boundary_lidar_pcd);
        }
        if (session->opt_ground_pcd != NULL && session->opt_ground_pcd->size() > 0) {
            // std::string pcd_file = session->get_debug_dir("genkeypoint_input.pcd");
            pcl::io::savePCDFileBinary(cache_file_ground.c_str(), *session->opt_ground_pcd);
        }
    }
    CHECK_FATAL_PROC(save_debug_info(session), "save_debug_info");
    
    return fsdmap::process_frame::PROC_STATUS_SUCC;
}

int RoadModelProcGenKeyPoint::make_point_tree(RoadModelSessionData* session) {
    if (session->lane_line_lidar_pcd) {
        for (auto &pt : session->lane_line_lidar_pcd->points) {
            auto lane_point = session->add_ptr(session->lane_line_feature_ptr);
            lane_point->pos = {pt.x, pt.y, 0};
            session->lane_line_lidar_tree.insert(lane_point->pos, lane_point.get());
        }
    }

    if (session->boundary_lidar_pcd) {
        for (auto &pt : session->boundary_lidar_pcd->points) {
            auto boundary_point = session->add_ptr(session->boundary_feature_ptr);
            boundary_point->pos = {pt.x, pt.y, 0};
            session->debug_pos(boundary_point->pos);
            session->boundary_lidar_tree.insert(boundary_point->pos, boundary_point.get());
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::get_lane_key_point(RoadModelSessionData* session)
{
//    //首先读取点云数据
//    utils::CloudPtr all_pc_ptr(new pcl::PointCloud<utils::CloudPoint>);
//    pcl::io::loadPCDFile(FLAGS_gen_key_point_input_pcd, *all_pc_ptr);

    //提取车道线的点
    // LOG_INFO("start get_lane_key_point cloud size {}",session->opt_label_lane_pcd->size());
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    label_ConditionalRemoval(session->opt_ground_pcd, pc_ptr, 1);  // 选出label是1的点
    session->opt_label_lane_pcd = pc_ptr;
    if(pc_ptr!=NULL && pc_ptr->size()>0){
        std::string pcd_file = session->get_debug_dir("genkeypoint_lanecloud.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *pc_ptr);
    }

    //提取关键点
    // pcl::PointCloud<pcl::PointXYZ>::Ptr keypts_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<PointXYZ_OPT>::Ptr keypts_ptr(new pcl::PointCloud<PointXYZ_OPT>);
    get_key_point(session, pc_ptr, keypts_ptr, "lane");

    session->lane_line_lidar_pcd = keypts_ptr;
    if(keypts_ptr!=NULL && keypts_ptr->size()>0)
    {
        std::string pcd_file = session->get_debug_dir("genkeypoint_lane_result.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *keypts_ptr);
    }
    // LOG_INFO("finish get_lane_key_point");
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::get_roadboundary_key_point(RoadModelSessionData *session)
{
 
    // if(FLAGS_use_opt_groud_pcd_file)
    // {
    //       //首先读取点云数据
    //     utils::CloudPtr all_pc_ptr(new pcl::PointCloud<utils::CloudPoint>);
    //     pcl::io::loadPCDFile(FLAGS_gen_key_point_input_pcd, *all_pc_ptr);
        
    //     //提取所有点
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr road_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (auto pt : *session->opt_ground_pcd) 
    //     {
    //     //        pcl::PointXYZ newpt(pt.x/10.0, pt.y/10.0, pt.z/10.0);
    //     pcl::PointXYZ newpt(pt.x, pt.y, pt.z);
    //     road_ptr->push_back(newpt);
    //     }

    //     //------------------------半径滤波---------------------------
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;	//创建滤波器对象
    //     ror.setInputCloud(road_ptr);						//设置待滤波点云
    //     ror.setRadiusSearch(0.2);						//设置查询点的半径范围
    //     ror.setMinNeighborsInRadius(5);					//设置判断是否为离群点的阈值，即半径内至少包括的点数
    //     //ror.setNegative(true);						//默认false，保存内点；true，保存滤掉的外点
    //     ror.filter(*cloud_filtered);					//执行滤波，保存滤波结果于cloud_filtered

    //     //------------------------均匀下采样---------------------------
    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sample(new pcl::PointCloud<pcl::PointXYZ>);
    //     pcl::UniformSampling<pcl::PointXYZ> us;	//创建滤波器对象
    //     us.setInputCloud(cloud_filtered);				//设置待滤波点云
    //     us.setRadiusSearch(0.1);				//设置滤波球体半径
    //     us.filter(*cloud_sample);				//执行滤波，保存滤波结果于cloud_filtered

    //     if(cloud_sample!=NULL && cloud_sample->size()>0)
    //     {
    //         std::string pcd_file = session->get_debug_dir("genkeypoint_road_deal.pcd");
    //         pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud_sample);
    //     }

    //     //------------------------计算法向量---------------------------
    //     std::cout << "->计算法向量" << std::endl;
    //     pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normEst;
    //     pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    //     normEst.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(cloud_sample));
    //     normEst.setRadiusSearch(0.5);
    //     normEst.compute(*normals);

    //     std::cout << "->点云边缘估计" << std::endl;
    //     pcl::PointCloud<pcl::Boundary> boundaries;
    //     pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary> boundEst;
    //     boundEst.setInputCloud(cloud_sample);
    //     boundEst.setInputNormals(normals);
    //     boundEst.setRadiusSearch(0.25);
    //     boundEst.setAngleThreshold(M_PI / 2);
    //     boundEst.setSearchMethod(pcl::search::KdTree<pcl::PointXYZ>::Ptr(new pcl::search::KdTree<pcl::PointXYZ>));
    //     boundEst.compute(boundaries);

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (int i = 0; i < cloud_sample->points.size(); i++) {
    //         if (boundaries[i].boundary_point > 0) {
    //             cloud_boundary->push_back(cloud_sample->points[i]);
    //         }
    //     }
    //     session->roadedge_pcd = cloud_boundary;
    // }
    if(FLAGS_use_opt_road_edge_pcd_file)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(FLAGS_road_edge_input_pcd, *cloud_boundary);
        session->roadedge_pcd = cloud_boundary;
    }
   
//    std::cout << "边界点个数:" << cloud_boundary->points.size() << std::endl;
    // LOG_INFO("start get_roadboundary_key_point cloud size {}",session->roadedge_pcd->points.size());
    if (!session->roadedge_pcd || session->roadedge_pcd->points.empty())
        return fsdmap::SUCC;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_boundary = session->roadedge_pcd;
    std::string pcd_file = session->get_debug_dir("genkeypoint_road_edge.pcd");
    pcl::io::savePCDFileBinary(pcd_file.c_str(), *cloud_boundary);
    session->roadedge_pcd = cloud_boundary;
    pcl::PointCloud<PointXYZ_OPT>::Ptr key_points(new pcl::PointCloud<PointXYZ_OPT>);
    get_key_point(session, cloud_boundary, key_points, "boundary");

    session->boundary_lidar_pcd = key_points;
    if(key_points!=NULL && key_points->size()>0)
    {
        std::string pcd_file = session->get_debug_dir("genkeypoint_road_result.pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *key_points);
    }
    // LOG_INFO("finish get_roadboundary_key_point");
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::get_key_point(RoadModelSessionData* session, const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc_ptr, pcl::PointCloud<PointXYZ_OPT>::Ptr& key_points, std::string keystr)
{
    //聚类
    std::vector<std::vector<int>> clusters;
    if(keystr == "lane")
    {
        DBSCAN(pc_ptr, FLAGS_lane_dbscan_radius, FLAGS_lane_dbscan_minPts,clusters);
    }
    else
    {
        DBSCAN(pc_ptr, FLAGS_boundary_dbscan_radius, FLAGS_boundary_dbscan_minPts,clusters);
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i = 0; i < clusters.size(); ++i) {
        for(int j = 0; j < clusters[i].size();j++)
        {
            pcl::PointXYZ curPXYZ = pc_ptr->at(clusters[i][j]);
            clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
        }
    }
    if(clustersCloudWithI!=NULL && clustersCloudWithI->size()>0){
        std::string pcd_file = session->get_debug_dir("genkeypoint_clusterCloud_"+keystr+".pcd");
        pcl::io::savePCDFileBinary(pcd_file.c_str(), *clustersCloudWithI);
    }

    //提取segment
    int nline = 0;
    for (int i = 0; i < clusters.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pc_ptr, clusters[i], *clusterCloud);
        // if(clusterCloud!=NULL && clusterCloud->size()>0){
        //     std::string pcd_file = session->get_debug_dir("genkeypoint_clusterCloud_{}.pcd",i);
        //     pcl::io::savePCDFileBinary(pcd_file.c_str(), *clusterCloud);
        // }
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        cal_ref_info(clusterCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);
        // std::cout<< i <<"点个数："<<clusterCloud->size()<<std::endl;
        std::vector<Segment> segments;
        if(keystr == "lane")
        {
            ExtractSegment(clusterCloud, segments, FLAGS_lane_extract_segment_radius, FLAGS_lane_extract_segment_minPts,FLAGS_lane_eigenvaluediff);
        }
        else
        {
            ExtractSegment(clusterCloud, segments, FLAGS_boundary_extract_segment_radius, FLAGS_boundary_extract_segment_minPts,FLAGS_boundary_eigenvaluediff);
        }
        
        // std::cout<< i <<"segments个数："<<segments.size()<<std::endl;
        std::vector<std::pair<int, float>> disp;
        sort_segments(disp, segments, refLaneAnchorCloud, kdtree_refLaneAnchor);
        for(int j=0; j<segments.size(); j++)
        {
            int index = disp[j].first;
            segments[index].QpSpline();  // OSQP拟合曲线
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[index].smoothTrjCloud(); //在拟合好的曲线上采样
            if(trjCloud!=NULL && trjCloud->size()>0)
            {
                for (size_t k = 0; k < trjCloud->size(); k++)
                {
                    pcl::PointXYZ pt = trjCloud->at(k);
                    PointXYZ_OPT newPt;
                    newPt.x = pt.x;
                    newPt.y = pt.y;
                    newPt.z = pt.z;
                    newPt.ins_id = nline;
                    newPt.index = k;
                    key_points->push_back(newPt);
                }
                nline++;
            }
        }
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::label_ConditionalRemoval(const utils::CloudPtr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& labelCloud,uint16_t opt_label)
{
    utils::CloudPtr cloud_filtered(new pcl::PointCloud<utils::CloudPoint>);;
    /*创建条件限定下的滤波器*/
    pcl::ConditionAnd<utils::CloudPoint>::Ptr range_cond(new pcl::ConditionAnd<utils::CloudPoint>());//创建条件定义对象range_cond
    //为条件定义对象添加比较算子
    range_cond->addComparison(pcl::FieldComparison<utils::CloudPoint>::ConstPtr(new pcl::FieldComparison<utils::CloudPoint>("opt_label", pcl::ComparisonOps::EQ, opt_label)));//添加在x字段上大于 -0.1 的比较算子
    pcl::ConditionalRemoval<utils::CloudPoint> cr;	//创建滤波器对象
    cr.setCondition(range_cond);				//用条件定义对象初始化
    cr.setInputCloud(cloud);					//设置待滤波点云
    //cr.setKeepOrganized(true);				//设置保持点云的结构
    //cr.setUserFilterValue(5);					//将过滤掉的点用（5，5，5）代替
    cr.filter(*cloud_filtered);

    for (auto pt : *cloud_filtered) {
//        pcl::PointXYZ newpt(pt.x/10.0, pt.y/10.0, pt.z/10.0);
        pcl::PointXYZ newpt(pt.x, pt.y, pt.z);
        labelCloud->push_back(newpt);
    }
}

int RoadModelProcGenKeyPoint::DBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float radius, int minPts,
               std::vector<std::vector<int>> &clusters)
{
    // LOG_INFO("DBSCAN---> dbscan_radius={} minPts={}", radius,minPts);
    // std::cout<<"input points number: "<<cloud->size()<<std::endl;
    if(cloud==NULL || cloud->size()==0){
        std::cout<<"input cloud is null at Inference::DBSCAN"<<std::endl;
        return fsdmap::SUCC;
    }
    //create kdtree for fast neighborhood search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    //neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);

    auto RangeQuery = [&](pcl::PointXYZ searchPoint){
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        kdtree.radiusSearch (searchPoint, radius, neighbors, pointRadiusSquaredDistance);
    };

    std::vector<int> label(cloud->size(), -1); //-1 undefined, 0 noise, 1-n class
    int clusterId = 0;
    for(int i=0; i<cloud->size(); i++){
        if(label[i] >= 0){
            continue;
        }
        RangeQuery(cloud->at(i)); //contains point i itself
        if(neighbors.size() < minPts){
            label[i] = 0; //label as noise
            continue;
        }
        label[i] = ++clusterId;
        std::vector<bool> inQueue(cloud->size(), false);
        std::queue<int> q;
        for(auto id : neighbors){
            if(id != i && label[id] <= 0){
                q.push(id);
                inQueue[id] = true;
            }
        }
        while(!q.empty()){
            int currentId = q.front();
            q.pop();
            //Change Noise to border point
            if(label[currentId]==0) label[currentId] = clusterId;
            //previously processed
            if(label[currentId]>=0) continue;
            label[currentId] = clusterId;
            RangeQuery(cloud->at(currentId));
            if(neighbors.size() >= minPts){
                for(auto id : neighbors){
                    if(label[id] < 0 && !inQueue[id]){
                        q.push(id);
                        inQueue[id] = true;
                    }
                }
            }
        }
    }
    clusters.resize(clusterId);
    int noiseNum = 0;
    for(int i=0; i<label.size(); i++){
        if(label[i]>0){
            clusters[label[i]-1].push_back(i);
        }else{
            noiseNum += 1;
        }
    }
    // std::cout<<"finish DBSCAN cluster with noise points number: "<<noiseNum<<std::endl;
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::cal_ref_info(const pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor)
{
    //计算中心点
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    for(int i=0; i<laneSegCloud->size(); i++){
        centroid += laneSegCloud->at(i).getVector3fMap();
    }
    centroid = centroid/laneSegCloud->size();

    //计算方向向量
    Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
    for(int i=0; i<laneSegCloud->size(); i++){
        Eigen::Vector3f posDiff = laneSegCloud->at(i).getVector3fMap() - centroid;
        cov += posDiff*posDiff.transpose();
    }
    cov = cov/laneSegCloud->size();
    cov = (cov + cov.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(cov);
    Eigen::Vector3f eigenvalues = es.eigenvalues();
    Eigen::Matrix3f U = es.eigenvectors();
    // std::cout<<"the cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
    double d0 = eigenvalues[0];
    double d1 = eigenvalues[1];
    Eigen::Vector3f refDirection = Eigen::Vector3f::Zero(); // 与主方向垂直的方向
    if (d0 < d1)
        refDirection = U.col(0);
    else
        refDirection = U.col(1);

    float minVal = 1e6, maxVal = -1e6;
    for(int i=0; i<laneSegCloud->size(); i++){
        float val = refDirection.dot(laneSegCloud->at(i).getVector3fMap()-centroid); // 点到直线的距离
        if(val<minVal) minVal = val;
        if(val>maxVal) maxVal = val;
    }
    // 在中心点沿与直线垂直的方向往外采样几个点
    float resolution = 3.f;
    int num = (maxVal - minVal)/resolution + 1;
    int num1 = maxVal/resolution + 1;
    int num2 = (fabs(minVal))/resolution + 1;
    for(int i=-num2; i<=num1; i++){
        Eigen::Vector3f pos = centroid + resolution*i*refDirection;
        refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    }
    kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::ExtractSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Segment> &segments,
                    float radius, int minPts,float eigenvaluediff)
{
    // LOG_INFO("ExtractSegment--> radius={} minPts={} eigenvaluediff={}", radius,minPts,eigenvaluediff);
    pcl::PointCloud<pcl::PointXY>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXY>);
    for(int i=0; i<cloud->size(); i++){
        pcl::PointXY pt_plane(cloud->at(i).x, cloud->at(i).y);
        planeCloud->push_back(pt_plane);
    }

    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(planeCloud);

    //neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);

    // 计算检查给定点周围的邻域是否符合线性特征的要求
    auto NeighborLineFit = [&](int queryId, Eigen::Vector2f &direction){
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        pcl::PointXY searchPoint = planeCloud->at(queryId);
        kdtree.radiusSearch (searchPoint, radius, neighbors, pointRadiusSquaredDistance);
        //std::cout<<"NeighborLineFit, queryId: "<<queryId<<" neighbors size: "<<neighbors.size()<<std::endl;
        if(neighbors.size()<minPts){
            //std::cout<<"in NeighborLineFit, insufficient neighbor points"<<std::endl;
            return false; //not enough neighborhood points
        }
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        for(auto id : neighbors){
            centroid += Eigen::Vector2f(planeCloud->at(id).x, planeCloud->at(id).y);
        }
        centroid = centroid/neighbors.size();
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for(auto id : neighbors){
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            Eigen::Vector2f diff = pos - centroid;
            cov += diff*diff.transpose();
        }
        cov = cov/neighbors.size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
        es.compute(cov);
        Eigen::Vector2f eigenvalues = es.eigenvalues();
        // std::cout<<"eigenvalues值："<<eigenvalues(0)<<" , "<< eigenvalues(1) << std::endl;
        if(eigenvalues(1) < eigenvaluediff*eigenvalues(0)){
            //std::cout<<"in NeighborLineFit, fail PCA check"<<std::endl;
            return false;
        }
        Eigen::Matrix2f eigenvectors = es.eigenvectors();
        direction = eigenvectors.col(1);
        return true;
    };

    std::vector<std::pair<int, float>> disps;
    disps.reserve(100);
    Eigen::Vector2f direction;

    struct Anchor{
        int index; //index in cloud
        Eigen::Vector2f direction; //direction in xy plane
        Anchor(int index, Eigen::Vector2f direction):index(index), direction(direction){}
    };
    std::list<Anchor> anchors; //list of anchors for expansion on both sides
    std::list<int> indices; //list of cloud point index for expansion on both sides
    std::vector<bool> labeled(cloud->size(), false);

    for(int i=0; i<cloud->size(); i++){
        if(labeled[i]) continue;
        anchors.clear();
        indices.clear();
        int anchorIndex = i;
        // 选取anchorIndex点半径1m范围内最近邻点，svd分解计算是否符合线性特性，并返回直线的主方向
        bool bFitSuccess = NeighborLineFit(anchorIndex, direction);
        for(auto id : neighbors){
            labeled[id] = true; //label as searched anchorIndex附近的点都标记，表示已经搜索过，不用重复搜索
        }
        if(!bFitSuccess) continue;  // 不是直线，跳过
        Eigen::Vector2f anchor(planeCloud->at(anchorIndex).x, planeCloud->at(anchorIndex).y);
        //calculate displacement in neighborhood
        disps.clear();
        // 计算每个近邻点在直线的方向的投影长度，有正负区分
        for(auto id : neighbors){
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            float disp = (pos - anchor).dot(direction);  // pos点在直线的方向的投影长度，有正负区分
            disps.push_back(std::make_pair(id, disp));
        }
        //sort points in direction
        // 按离锚点anchor从近到远排序
        std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a,
                                                 const std::pair<int, float>&b){
            return a.second < b.second;
        });
        //set the initial anchor
        //std::cout<<"set the initial anchor: "<<anchorIndex<<std::endl;
        anchors.push_back(Anchor(anchorIndex, direction));
        for(auto ele : disps){
            //std::cout<<"("<<ele.first<<", "<<ele.second<<") ";
            indices.push_back(ele.first);
        }
        //std::cout<<"initial segment indices size: "<<indices.size()<<std::endl;
        bool expandToLeft = (indices.front()!=anchorIndex),
                expandToRight = (indices.back()!=anchorIndex);
        while(expandToLeft || expandToRight){
            if(expandToLeft){  // 向左扩充线段
                Anchor leftMostNode = anchors.front();  // 锚点
                Eigen::Vector2f leftMostDirection = leftMostNode.direction; // 锚点方向
                int leftMostNewAnchorIndex = indices.front(); // 最左侧的点
                Eigen::Vector2f leftMostNewAnchor(planeCloud->at(leftMostNewAnchorIndex).x,
                                                  planeCloud->at(leftMostNewAnchorIndex).y);  // 以最左侧这个点作为新的锚点，用于后面最近邻搜索
                // 以最左侧这个点作为新的锚点,继续搜索最近点，用于扩充线段
                if(NeighborLineFit(leftMostNewAnchorIndex, direction)){
                    //adjust direction to coincide with leftMostDirection
                    // 当前锚点的方向向量与上一个锚点的方向向量
                    if(direction.dot(leftMostDirection) < 0.f) direction = -direction;
                    disps.clear();
                    for(auto id : neighbors){
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos-leftMostNewAnchor).dot(direction);  // pos点在直线的方向的投影长度，有正负区分
                        //std::cout<<"id: "<<id<<" "<<labeled[id]<<" disp: "<<disp<<std::endl;
                        if(labeled[id] || disp > 0.f) continue;  //往左侧扩充disp应该小于0
                        disps.push_back(std::make_pair(id, disp));
                    }
                    //std::cout<<"expand to the left: "<<disps.size()<<std::endl;
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a,
                                                             const std::pair<int, float>&b){
                        return a.second < b.second;
                    });
                    anchors.push_front(Anchor(leftMostNewAnchorIndex, direction));  // 把最新的最左侧锚点放到anchors的最前面
                    for(auto iter=disps.rbegin(); iter!=disps.rend(); iter++){  // 同时把新的最近邻点放入indices
                        indices.push_front(iter->first);
                    }
                    expandToLeft = indices.front()!=leftMostNewAnchorIndex;
                }else{
                    expandToLeft = false;
                }
                for(auto id : neighbors){
                    labeled[id] = true; //label as searched
                }
            }
            if(expandToRight){
                Anchor rightMostNode = anchors.back();
                Eigen::Vector2f rightMostDirection = rightMostNode.direction;
                int rightMostNewAnchorIndex = indices.back();
                Eigen::Vector2f rightMostNewAnchor(planeCloud->at(rightMostNewAnchorIndex).x,
                                                   planeCloud->at(rightMostNewAnchorIndex).y);
                if(NeighborLineFit(rightMostNewAnchorIndex, direction)){
                    //adjust direction to coincide with rightMostDirection
                    if(direction.dot(rightMostDirection) < 0.f) direction = -direction;
                    disps.clear();
                    for(auto id : neighbors){
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos-rightMostNewAnchor).dot(direction);
                        if(labeled[id] || disp < 0.f) continue;
                        disps.push_back(std::make_pair(id, disp));
                    }
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a,
                                                             const std::pair<int, float>&b){
                        return a.second < b.second;
                    });
                    anchors.push_back(Anchor(rightMostNewAnchorIndex, direction));
                    for(auto ele : disps){
                        indices.push_back(ele.first);
                    }
                    expandToRight = indices.back()!=rightMostNewAnchorIndex;
                }else{
                    expandToRight = false;
                }
                for(auto id : neighbors){
                    labeled[id] = true; //label as searched
                }
            }
            //std::cout<<"after expand segment nodes size: "<<anchors.size()<<" indices size: "<<indices.size()<<std::endl;
        }
        if(anchors.size() < 2) continue; //too short segment
        Segment segment;
        auto iter = anchors.begin();
        int segmentCloudIndex = 0;
        for(auto id : indices){
            segment.cloud->push_back(cloud->at(id));
            if(id==iter->index){ // 在indices中遍历到与anchors中的某个anchor
                segment.anchorIndices.push_back(segmentCloudIndex);  // 记录segment.cloud中属于anchor的index
                iter++;
            }
            segmentCloudIndex += 1;
        }
        segments.push_back(segment);
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::sort_segments(std::vector<std::pair<int, float>>& disp, std::vector<Segment> &segments,pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud,pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor)
{
    disp.resize(segments.size());
    for(int i=0; i<segments.size(); i++){
        disp[i] = std::make_pair(i, 10.f);
    }
    for(int i=0; i<segments.size(); i++){
        int ncount = 0;
        float accuDisp = 0.f;
        float radius = 20.f;
        for(auto anchorIndex : segments[i].anchorIndices){
            std::vector<int> neighbors;
            std::vector<float> pointRadiusSquaredDistance;
            kdtree_refLaneAnchor->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
            if(neighbors.empty()){
                continue;
            }
            int neighborIndex = neighbors.front();
            if(neighborIndex<1){
                continue;
            }
            Eigen::Vector3f direction = refLaneAnchorCloud->at(neighborIndex).getVector3fMap() -
                                        refLaneAnchorCloud->at(neighborIndex-1).getVector3fMap();
            Eigen::Vector2f nx = direction.head(2).normalized();
            Eigen::Vector2f ny(-nx(1), nx(0));
            accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() -
                         refLaneAnchorCloud->at(neighborIndex).getVector3fMap()).head(2).dot(ny);
            ncount++;
        }
        if(ncount > 0){
            disp[i].second = accuDisp/ncount;
        }
        // std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    }
    std::sort(disp.begin(), disp.end(), [](std::pair<int, float> a, std::pair<int, float> b){
        return a.second > b.second;
    });
    return fsdmap::SUCC;
}

//参考https://blog.csdn.net/nudt_zrs/article/details/124258999
bool RoadModelProcGenKeyPoint::Segment::QpSpline()
{
    if(anchorIndices.size() < 3){
        return false;
    }
    int interValNum = anchorIndices.size() - 1;
    c_int n = 4*interValNum;          //number of opt variables
    std::vector<c_float> coeffs_x(n, 0.f); //initial coeff for x variable
    std::vector<c_float> coeffs_y(n, 0.f); //initial coeff for y variable
    std::vector<c_float> coeffs_z(n, 0.f); //initial coeff for z variable
    c_int P_nnz = 10*interValNum;     //only save upper part
    std::vector<c_float> P_x(P_nnz);  //non zero element of P
    std::vector<c_int> P_i(P_nnz);    //row index per element of P
    std::vector<c_int> P_p(n+1);      //accumulated non zero element number of P by column
    std::vector<c_float> qx(n), qy(n), qz(n);   //q vector in optimization

    P_p[0] = 0;
    //ts = std::vector<double>(cloud->size(), 0.f);
    //fit cubic function at each interval initially
    // 对一个segment里面的每个小段各自做拟合
    for(int i=0; i<interValNum; i++){
        int index_0 = anchorIndices[i], index_1 = anchorIndices[i+1];
        Eigen::Vector3d basePos = cloud->at(index_0).getVector3fMap().cast<double>();
        Eigen::Vector3d disp = cloud->at(index_1).getVector3fMap().cast<double>() - basePos;
        double disp_squaredNorm = disp.squaredNorm();
        Eigen::Matrix4d P_interval = Eigen::Matrix4d::Zero();
        Eigen::Vector4d q_interval_x = Eigen::Vector4d::Zero(),
                q_interval_y = Eigen::Vector4d::Zero(),
                q_interval_z = Eigen::Vector4d::Zero();
        Eigen::Vector4d tmp;
        for(int k=index_0; k<=index_1; k++){
            Eigen::Vector3d currPos = cloud->at(k).getVector3fMap().cast<double>();
            double dt = (currPos - basePos).dot(disp)/disp_squaredNorm,
                    dt_2 = dt*dt, dt_3 = dt*dt_2; // dt: currPos在disp上的投影长度占disp长度的比例
            //ts[k] = dt;
            tmp<<1.f, dt, dt_2, dt_3;
            P_interval += tmp*tmp.transpose();
            q_interval_x -= tmp*currPos(0);
            q_interval_y -= tmp*currPos(1);
            q_interval_z -= tmp*currPos(2);
        }
        //P_interval += 1e-8*Eigen::Matrix4d::Identity();
        P_interval.block<2,2>(2,2) += Eigen::Matrix2d::Identity();
        Eigen::Vector4d coeff_x = -P_interval.inverse()*q_interval_x,
                coeff_y = -P_interval.inverse()*q_interval_y,
                coeff_z = -P_interval.inverse()*q_interval_z;

        int coeffsIndex = 4*i, PIndex = 10*i;
        for(int j=0; j<4; j++){
            coeffs_x[coeffsIndex+j] = coeff_x(j);
            coeffs_y[coeffsIndex+j] = coeff_y(j);
            coeffs_z[coeffsIndex+j] = coeff_z(j);
            qx[coeffsIndex+j] = q_interval_x(j);
            qy[coeffsIndex+j] = q_interval_y(j);
            qz[coeffsIndex+j] = q_interval_z(j);
            P_p[coeffsIndex+j+1] = P_p[coeffsIndex+j] + j + 1;
            PIndex += j;
            for(int k=0; k<=j; k++){
                P_x[PIndex+k] = P_interval(k,j);
                P_i[PIndex+k] = k+coeffsIndex;
            }
        }
    }
    c_int m = 3*(interValNum-1);    //number of constraint
    std::vector<c_float> l(m, -1e-6);
    std::vector<c_float> u(m, 1e-6);
    c_int A_nnz = 12*(interValNum-1); //number of nonzero element in A
    std::vector<c_float> A_x(A_nnz);  //all non zero elements in A
    std::vector<c_int> A_i(A_nnz);    //row index per element in A
    std::vector<c_int> A_p(n+1);      //accumulated non zero element number of A by column
    A_p[0] = 0;
    //assign the first block
    A_p[1] = 1;
    A_p[2] = 3;
    A_p[3] = 6;
    A_p[4] = 9;

    A_x[0] = 1.0;
    A_x[1] = 1.0;
    A_x[2] = 1.0;
    A_x[3] = 1.0;
    A_x[4] = 2.0;
    A_x[5] = 2.0;
    A_x[6] = 1.0;
    A_x[7] = 3.0;
    A_x[8] = 6.0;

    A_i[0] = 0;
    A_i[1] = 0;
    A_i[2] = 1;
    A_i[3] = 0;
    A_i[4] = 1;
    A_i[5] = 2;
    A_i[6] = 0;
    A_i[7] = 1;
    A_i[8] = 2;
    for(int i=1; i<interValNum-1; i++){
        int colStartIndex = 4*i;
        A_p[colStartIndex+1] = A_p[colStartIndex] + 2;
        A_p[colStartIndex+2] = A_p[colStartIndex+1] + 3;
        A_p[colStartIndex+3] = A_p[colStartIndex+2] + 4;
        A_p[colStartIndex+4] = A_p[colStartIndex+3] + 3;

        int nzStartIndex = 9 + 12*(i-1);
        int rowStartIndex = 3*(i-1);

        A_x[nzStartIndex] = -1.0;
        A_x[nzStartIndex+1] = 1.0;
        A_x[nzStartIndex+2] = -1.0;
        A_x[nzStartIndex+3] = 1.0;
        A_x[nzStartIndex+4] = 1.0;
        A_x[nzStartIndex+5] = -2.0;
        A_x[nzStartIndex+6] = 1.0;
        A_x[nzStartIndex+7] = 2.0;
        A_x[nzStartIndex+8] = 2.0;
        A_x[nzStartIndex+9] = 1.0;
        A_x[nzStartIndex+10] = 3.0;
        A_x[nzStartIndex+11] = 6.0;

        A_i[nzStartIndex] = rowStartIndex;
        A_i[nzStartIndex+1] = rowStartIndex+3;
        A_i[nzStartIndex+2] = rowStartIndex+1;
        A_i[nzStartIndex+3] = rowStartIndex+3;
        A_i[nzStartIndex+4] = rowStartIndex+4;
        A_i[nzStartIndex+5] = rowStartIndex+2;
        A_i[nzStartIndex+6] = rowStartIndex+3;
        A_i[nzStartIndex+7] = rowStartIndex+4;
        A_i[nzStartIndex+8] = rowStartIndex+5;
        A_i[nzStartIndex+9] = rowStartIndex+3;
        A_i[nzStartIndex+10] = rowStartIndex+4;
        A_i[nzStartIndex+11] = rowStartIndex+5;
    }
    //assign the last block
    int colStartIndex = 4*(interValNum-1);
    A_p[colStartIndex+1] = A_p[colStartIndex] + 1;
    A_p[colStartIndex+2] = A_p[colStartIndex+1] + 1;
    A_p[colStartIndex+3] = A_p[colStartIndex+2] + 1;
    A_p[colStartIndex+4] = A_p[colStartIndex+3];
    int nzStartIndex = 9 + 12*(interValNum-2);
    A_x[nzStartIndex] = -1.0;
    A_x[nzStartIndex+1] = -1.0;
    A_x[nzStartIndex+2] = -2.0;
    int rowStartIndex = 3*(interValNum-2);
    A_i[nzStartIndex] = rowStartIndex;
    A_i[nzStartIndex+1] = rowStartIndex+1;
    A_i[nzStartIndex+2] = rowStartIndex+2;

    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    // Populate data
    data->n = n;
    data->m = m;
    data->P = csc_matrix(data->n, data->n, P_nnz, P_x.data(), P_i.data(), P_p.data());
    data->A = csc_matrix(data->m, data->n, A_nnz, A_x.data(), A_i.data(), A_p.data());
    data->l = l.data();
    data->u = u.data();

    // Define solver settings as default
    osqp_set_default_settings(settings);
    settings->alpha = 1.0; // Change alpha parameter
    settings->verbose = 0;//not print out

    // cubic spline in x variable
    data->q = qx.data();
    exitflag = osqp_setup(&work, data, settings);
    osqp_warm_start_x(work, coeffs_x.data());
    osqp_solve(work);
    mCoeffs_x.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_x[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_x[i].transpose()<<std::endl;
    }

    // cubic spline in y variable
    osqp_update_lin_cost(work, qy.data());
    osqp_warm_start_x(work, coeffs_y.data());
    osqp_solve(work);
    mCoeffs_y.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_y[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }

    // cubic spline in z variable
    osqp_update_lin_cost(work, qz.data());
    osqp_warm_start_x(work, coeffs_z.data());
    osqp_solve(work);
    mCoeffs_z.resize(interValNum);
    for(int i=0; i<interValNum; i++){
        int index = 4*i;
        mCoeffs_z[i] = Eigen::Vector4d(work->solution->x[index], work->solution->x[index+1], work->solution->x[index+2], work->solution->x[index+3]);
        //std::cout<<i<<" "<<iniCoeff.transpose()<<" "<<mCoeffs_y[i].transpose()<<std::endl;
    }
    // Cleanup
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
    // std::cout<<"successfully run Segment::QpSpline"<<std::endl;

    return true;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RoadModelProcGenKeyPoint::Segment::smoothTrjCloud()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<mCoeffs_x.size(); i++){  //遍历每段样条
        double h = 1.0/10;     //采样10个点
        for(int j=0; j<10; j++){
            double dt = h*j, dt_2 = dt*dt, dt_3=dt*dt_2;
            double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
            double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
            double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
            trjCloud->push_back(pcl::PointXYZ(x, y, z));
        }
    }
    return trjCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr RoadModelProcGenKeyPoint::Segment::smoothTrjCloud2()
{
    auto cubicFunCal = [&](const Eigen::Vector4d &fun_params, const double &t){
        double res = fun_params[0] + fun_params[1]*t + fun_params[2]*pow(t, 2) + fun_params[3]*pow(t, 3);
        return res;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int i=0; i<mCoeffs_x.size(); i++){
        double XS = cubicFunCal(mCoeffs_x[i], 0.0);
        double YS = cubicFunCal(mCoeffs_y[i], 0.0);
        double ZS = cubicFunCal(mCoeffs_z[i], 0.0);

        double XE = cubicFunCal(mCoeffs_x[i], 1.0);
        double YE = cubicFunCal(mCoeffs_y[i], 1.0);
        double ZE = cubicFunCal(mCoeffs_z[i], 1.0);

        double len = std::sqrt(pow(XS-XE, 2) + pow(YS - YE, 2) + pow(ZS - ZE, 2));
        int num = int(len);
        double gap = 1.0;
        if(num > 1)
            gap = 1.0 / num;

        if(i == mCoeffs_x.size() - 1)
        {
            for(int j=0; j<= num; j++){
                double dt = gap*j, dt_2 = dt*dt, dt_3=dt*dt_2;
                double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
                double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
                double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }
        else
        {
            for(int j=0; j< num; j++){
                double dt = gap*j, dt_2 = dt*dt, dt_3=dt*dt_2;
                double x = mCoeffs_x[i](0)+mCoeffs_x[i](1)*dt+mCoeffs_x[i](2)*dt_2+mCoeffs_x[i](3)*dt_3;
                double y = mCoeffs_y[i](0)+mCoeffs_y[i](1)*dt+mCoeffs_y[i](2)*dt_2+mCoeffs_y[i](3)*dt_3;
                double z = mCoeffs_z[i](0)+mCoeffs_z[i](1)*dt+mCoeffs_z[i](2)*dt_2+mCoeffs_z[i](3)*dt_3;
                trjCloud->push_back(pcl::PointXYZ(x, y, z));
            }
        }

    }
    return trjCloud;
}

int RoadModelProcGenKeyPoint::EigenSolverEstimation(pcl::PointCloud<pcl::PointXYZ>::Ptr &Cloud, double radius, pcl::PointCloud <pcl::Normal>::Ptr& cloud_normals)
{
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_cloud->setInputCloud(Cloud); // 设置要搜索的点云，建立KDTree

    for (int i = 0; i < Cloud->size(); ++i) {
        std::vector<int> pointIdxRadiusSearch;
        std::vector<float> pointRadiusSquaredDistance;
        kdtree_cloud->radiusSearch(Cloud->at(i), radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        if (pointIdxRadiusSearch.empty()) {
            //找最近点
            int K = 1;
            kdtree_cloud->nearestKSearch(Cloud->at(i), K, pointIdxRadiusSearch, pointRadiusSquaredDistance);
        }

        //特征值计算
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
        int totalNum = 0;
        for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
            centroid += Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap();
            totalNum += 1;
        }

        centroid = centroid / totalNum;
        for (int j = 0; j < pointIdxRadiusSearch.size(); j++) {
            A += (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid) *
                 (Cloud->at(pointIdxRadiusSearch[j]).getVector3fMap() - centroid).transpose();
        }

        A = A / totalNum;
        A = (A + A.transpose()) / 2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(A);
        Eigen::Vector3f eigenvalues = es.eigenvalues();
        //std::cout<<"eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        Eigen::Matrix3f eigenvectors = es.eigenvectors();
        Eigen::Vector3f direction = eigenvectors.col(2);

        cloud_normals->push_back(pcl::Normal(direction[0], direction[1], direction[2]));
    }
    return fsdmap::SUCC;
}

int RoadModelProcGenKeyPoint::save_debug_info(RoadModelSessionData* session) {
    if (!FLAGS_gen_key_point_save_data_enable) {
        return fsdmap::SUCC;
    }
    session->set_display_name("gen_key_point");
    // if (session->lane_line_lidar_pcd) {
    //     UMAP<int, std::vector<int64_t>> cache;
    //     for (int64_t i = 0; i < session->lane_line_lidar_pcd->points.size(); ++i) {
    //         auto &pt = session->lane_line_lidar_pcd->points[i];
    //         auto &vec = cache[pt.ins_id];
    //         if (pt.index >= vec.size()) {
    //             vec.resize(pt.index + 1);
    //         }
    //         cache[pt.ins_id][pt.index] = i;
    //     }
    //     for (auto &lit : cache) {
    //         auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "laneline");
    //         for (auto &index : lit.second) {
    //             auto &pt = session->lane_line_lidar_pcd->points[index];
    //             log->add(pt.x, pt.y, pt.z);
    //         }
    //     }
    // }
    // if (session->boundary_lidar_pcd) {
    //     UMAP<int, std::vector<int64_t>> cache;
    //     for (int64_t i = 0; i < session->boundary_lidar_pcd->points.size(); ++i) {
    //         auto &pt = session->lane_line_lidar_pcd->points[i];
    //         auto &vec = cache[pt.ins_id];
    //         if (pt.index >= vec.size()) {
    //             vec.resize(pt.index + 1);
    //         }
    //         cache[pt.ins_id][pt.index] = i;
    //     }
    //     for (auto &lit : cache) {
    //         auto log = session->add_debug_log(utils::DisplayInfo::LINE_INDEX, "boundary");
    //         for (auto &index : lit.second) {
    //             auto &pt = session->boundary_lidar_pcd->points[index];
    //             log->add(pt.x, pt.y, pt.z);
    //         }
    //     }
    // }
    session->save_debug_info("gen_key_point");
    return fsdmap::SUCC;
}

}
}
