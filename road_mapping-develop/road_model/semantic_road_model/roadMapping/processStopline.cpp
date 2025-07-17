//
//
//

#include "processStopline.h"
#include "Utils.h"
#include "pclFilter.h"
#include "cluster.h"
#include "Algorithm/GeometricFit.h"
#include "./include/CloudAlgorithm.h"
#include "./include/CommonUtil.h"
#include <boost/filesystem.hpp>
#include <unordered_set>
#include <pcl/common/pca.h>
namespace RoadMapping
{

    void processStopline::fit_line(const Engine::Base::Array<Engine::Geometries::Coordinate> &vecPoints, Engine::Geometries::Coordinate &pntStart, Engine::Geometries::Coordinate &pntEnd)
    {
    }

    void processStopline::run_yxx_fin_in(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir)
    {
        // 1、读取label CLASS_TYPE_140 = 140,  //stop_line
        std::string lidarFile = pcdPath;
        std::cout << "lidarFile: " << lidarFile << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        // 将停止线源直接切换至cloud_pano_seg
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_pano_seg", 21);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label", 4);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            cloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }
        pcl::io::savePCDFileBinary(outputDir + "/" + "stopline_rawCloud.pcd", *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudStatistical(new pcl::PointCloud<pcl::PointXYZ>);
        pclFilter::StatisticalOutlierRemoval(cloud, cloudStatistical, 50, 1.0);
        pcl::io::savePCDFileBinary(outputDir + "/" + "stopline_cloudStatistical.pcd", *cloudStatistical);
        // 2、对点云进行聚类以及去噪处理
        float radius = 0.50f;
        int minPtsSerch = 10;
        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(cloudStatistical, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << std::endl;

        // 测试输出聚类后数据
        int minPts = 100;
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrArrCroods;
        for (int i = 0; i < clusters.size(); ++i)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate> arrPts;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloudStatistical->at(clusters[i][j]);
                clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
                arrPts.Add(Engine::Geometries::Coordinate(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
            arrArrCroods.Add(arrPts);
        }
        if (!clustersCloudWithI->empty())
            pcl::io::savePCDFileBinary(outputDir + "/" + "stopline_clustersCloud.pcd", *clustersCloudWithI);

        // 3、对点云进行直线拟合
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(cloudStatistical);

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrArrResCroods;
        for (int index = 0; index < arrArrCroods.GetCount(); index++)
        {
            std::cout << "index:" << index << std::endl;
            Engine::Base::Array<Engine::Geometries::Coordinate> arrCroods = arrArrCroods[index];
            Engine::Geometries::Coordinate ptStart, ptEnd;
            double dAvageThinness = -1.0;
            Engine::Algorithm::GeometricFit::LeastSquareLineFit(arrCroods, ptStart, ptEnd);
            if (ptStart.DistanceXY(ptEnd) < 3.0) // 小于一个车道认为是不合理的存在
                continue;

            // 重刷Z值
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            pcl::PointXYZ pclSpt(ptStart.x, ptStart.y, ptStart.z);
            kdtree_cloud->nearestKSearch(pclSpt, K, Idx, SquaredDistance);
            ptStart.z = cloudStatistical->at(Idx[0]).z;

            pcl::PointXYZ pclEpt(ptEnd.x, ptEnd.y, ptEnd.z);
            kdtree_cloud->nearestKSearch(pclEpt, K, Idx, SquaredDistance);
            ptEnd.z = cloudStatistical->at(Idx[0]).z;

            Engine::Base::Array<Engine::Geometries::Coordinate> arrLinePts;
            arrLinePts.Add(ptStart);
            arrLinePts.Add(ptEnd);
            arrArrResCroods.Add(arrLinePts);
        }

        // 输出测试数据
        if (!arrArrResCroods.IsEmpty())
        {
            hdmap_build::CommonUtil::WriteToOBJ(arrArrResCroods, outputDir, "stopline", true);
        }
    }

    void processStopline::run_extract_guide_line(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir)
    {
        std::cout << "提取引导线......" << std::endl;
        std::string lidarFile = pcdPath;
        std::cout << "lidarFile: " << lidarFile << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        // 提取引导线的点云
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_pano_seg", 19);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_pano_seg", 55);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto iter = cloudfilter->begin(); iter != cloudfilter->end(); iter++)
        {
            MyColorPointType &pcl_p = (*iter);
            cloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
        }

        // 聚类
        float radius = 2.0f;
        int minPtsSerch = 10;
        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(cloud, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << std::endl;

        // 测试输出聚类后数据
        int minPts = 100;
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> resultArrCroods;
        for (int i = 0; i < clusters.size(); ++i)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate> arrPts;
            Engine::Base::Array<Engine::Geometries::Coordinate> fitArr;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud->at(clusters[i][j]);
                arrPts.Add(Engine::Geometries::Coordinate(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
            Engine::Geometries::Coordinate ptStart, ptEnd;
            double dAvageThinness = -1.0;
            Engine::Algorithm::GeometricFit::LeastSquareLineFit(arrPts, ptStart, ptEnd);

            if (ptStart.DistanceXY(ptEnd) < 3.0)
            { // 小于一个车道认为是不合理的存在
                continue;
            }

            fitArr.Add(ptStart);
            fitArr.Add(ptEnd);
            resultArrCroods.Add(fitArr);
        }

        // 输出导流区obj
        if (!resultArrCroods.IsEmpty())
        {
            hdmap_build::CommonUtil::WriteToOBJ(resultArrCroods, outputDir, "guideline", true);
            // boost::filesystem::path filePath(dataDir);
            // std::string pcd_out_path = filePath.parent_path().string() + "/diversion.pcd";
            std::string pcd_out_path = outputDir + "/diversion.pcd";
            std::cout << "pcd_out_path: " << pcd_out_path << std::endl;
            objsToPointElementPCD(resultArrCroods, pcd_out_path, 9);
        }
    }

    void processStopline::objsToPointElementPCD(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrLines, std::string pcdpath, uint16_t ele_type, uint16_t type1, uint16_t type2, uint16_t type3, float heading, float score)
    {
        pcl::PointCloud<PointElement>::Ptr pointElement(new pcl::PointCloud<PointElement>);
        for (int i = 0; i < arrLines.GetCount(); i++)
        {
            for (int j = 0; j < arrLines[i].GetCount(); j++)
            {
                PointElement newPclPt;
                newPclPt.id = i;
                newPclPt.x = arrLines[i][j].x;
                newPclPt.y = arrLines[i][j].y;
                newPclPt.z = arrLines[i][j].z;
                newPclPt.index = j;
                newPclPt.ele_type = ele_type;
                newPclPt.type1 = type1;
                newPclPt.type2 = type2;
                newPclPt.type3 = type3;
                newPclPt.heading = heading;
                newPclPt.score = score;
                pointElement->points.push_back(newPclPt);
            }
        }

        if (pointElement == NULL || pointElement->points.empty())
        {
            return;
        }

        pcl::io::savePCDFileBinary(pcdpath, *pointElement);
    }

    /*
        breif: 获得聚类中点云最可能的形状(modified by ccj, 2024.12.24)
    */
    int processStopline::getClusterBevShape(const std::vector<int>& cluster, const std::map<int, int>& indexMap, const pcl::PointCloud<ConRmPointType>::Ptr& cloudfilter) 
    {
        if (cluster.empty()) {
            return -1; 
        }

        std::map<int, int> frequencyMap;
        int mostFrequentValue = -1;
        int maxCount = 0;

        for (const auto& roadmarkIndex : cluster) {
            auto cloudfilterIndexIt = indexMap.find(roadmarkIndex);
            
            if (cloudfilterIndexIt != indexMap.end()) {
                int cloudfilterIndex = cloudfilterIndexIt->second;
                int cloud_bev_shape = cloudfilter->at(cloudfilterIndex).cloud_bev_shape;
                frequencyMap[cloud_bev_shape]++;
            }
        }

        for (const auto& pair : frequencyMap) {
            if (pair.second > maxCount) {
                maxCount = pair.second;
                mostFrequentValue = pair.first;
            }
        }
        return mostFrequentValue;
    }


    /*
        breif: 打包输出点云(modified by ccj, 2024.12.24)
    */
    void processStopline::generateOutputCloud(int index, float yaw, int arrow_type, pcl::PointCloud<PointElement>::Ptr &outputCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &LinePts, float score, int ele_type)
    {
        PointElement start_point;
        start_point.x = LinePts[0].x;
        start_point.y = LinePts[0].y;
        start_point.z = LinePts[0].z;
        start_point.ele_type = ele_type;
        start_point.type1 = arrow_type;
        start_point.id = index;
        start_point.index = 0;
        start_point.heading = yaw;
        start_point.score = score;
        outputCloud->push_back(start_point);

        PointElement end_point;
        end_point.x = LinePts[1].x;
        end_point.y = LinePts[1].y;
        end_point.z = LinePts[1].z;
        end_point.ele_type = ele_type;
        end_point.type1 = arrow_type;
        end_point.id = index;
        end_point.index = 1;
        end_point.heading = yaw;
        end_point.score = score;
        outputCloud->push_back(end_point);
    }

    /*
        breif: 增加类型信息(modified by ccj, 2024.12.24)
    */
    void processStopline::run_yxx_fin_add_type(std::string middle_json_path, std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir, std::string data_type)
    {
        std::string lidarFile = pcdPath;
        std::cout << "lidarFile: " << lidarFile << std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        // 条件滤波
        pcl::PointCloud<ConRmPointType>::Ptr cloudfilter(new pcl::PointCloud<ConRmPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter, "cloud_bev_label", 4);
        if (cloudfilter->size() < 2) {
            std::cerr << "！！！！没有停止线点云！！！！" << std::endl;
            return;
        }

        // 统计滤波
        pcl::PointCloud<ConRmPointType>::Ptr cloudStatistical(new pcl::PointCloud<ConRmPointType>);
        pclFilter::StatisticalOutlierRemoval(cloudfilter, cloudStatistical, 50, 1.0);

        // 转换成标准点云, 并记录对应索引关系
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::map<int, int> indexMap; // 用于记录cloud中点的索引和cloudStatistical中点的索引
        int cloudStatisticalIndex = 0; // cloudStatistical的当前索引
        for (auto iter = cloudStatistical->begin(); iter != cloudStatistical->end(); iter++, cloudStatisticalIndex++)
        {
            ConRmPointType &pcl_p = (*iter);
            cloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
            indexMap[cloud->size()-1] = cloudStatisticalIndex;
        }
        pcl::io::savePCDFileBinary(outputDir + "/" + "stopline_cloudStatistical.pcd", *cloud);

        // 聚类
        float radius = 0.25f; 
        int minPtsSerch = 10;
        std::map<int, int> cluster_bev_shape_map; // 用于记录每个聚类中点云的形状
        std::vector<std::vector<int>> clusters;
        Inference::DBSCAN(cloud, radius, minPtsSerch, clusters);
        std::cout << "extract clusters size: " << clusters.size() << std::endl;

        // 数据类型转换
        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloudWithI(new pcl::PointCloud<pcl::PointXYZI>);
        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrArrCroods;
        Engine::Base::Array<int> arrTypes;
        for (int i = 0; i < clusters.size(); ++i)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate> arrPts;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                pcl::PointXYZ curPXYZ = cloud->at(clusters[i][j]);
                clustersCloudWithI->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
                arrPts.Add(Engine::Geometries::Coordinate(curPXYZ.x, curPXYZ.y, curPXYZ.z));
            }
            arrArrCroods.Add(arrPts);

            // 获得聚类中点云最可能的形状
            int cluster_bev_shape = getClusterBevShape(clusters[i], indexMap, cloudStatistical);
            arrTypes.Add(cluster_bev_shape);
        }
        if (!clustersCloudWithI->empty())
            pcl::io::savePCDFileBinary(outputDir + "/" + "stopline_clustersCloud.pcd", *clustersCloudWithI);

        // 对点云进行直线拟合
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_cloud->setInputCloud(cloud);

        Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> arrArrResCroods;
        Engine::Base::Array<int> arrResTypes;
        for (int index = 0; index < arrArrCroods.GetCount(); index++)
        {
            Engine::Base::Array<Engine::Geometries::Coordinate> arrCroods = arrArrCroods[index];
            Engine::Geometries::Coordinate ptStart, ptEnd;

            Engine::Algorithm::GeometricFit::LeastSquareLineFit(arrCroods, ptStart, ptEnd);
            if (ptStart.DistanceXY(ptEnd) < 2.0) // 小于一个车道认为是不合理的存在
            {
                std::cout << "[" << index << "]ptStart: " << ptStart.x << " " << ptStart.y << " " << ptStart.z << ", ptEnd: " << ptEnd.x << " " << ptEnd.y << " " << ptEnd.z << std::endl;
                continue;
            }

            // 重刷Z值
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            pcl::PointXYZ pclSpt(ptStart.x, ptStart.y, ptStart.z);
            kdtree_cloud->nearestKSearch(pclSpt, K, Idx, SquaredDistance);
            ptStart.z = cloudStatistical->at(Idx[0]).z;

            pcl::PointXYZ pclEpt(ptEnd.x, ptEnd.y, ptEnd.z);
            kdtree_cloud->nearestKSearch(pclEpt, K, Idx, SquaredDistance);
            ptEnd.z = cloudStatistical->at(Idx[0]).z;

            Engine::Base::Array<Engine::Geometries::Coordinate> arrLinePts;
            arrLinePts.Add(ptStart);
            arrLinePts.Add(ptEnd);
            arrArrResCroods.Add(arrLinePts);
            arrResTypes.Add(arrTypes[index]);
        }

        pcl::PointCloud<PointElement>::Ptr stopline(new pcl::PointCloud<PointElement>);
        std::cout << "arrArrResCroods size: " << arrArrResCroods.GetCount() << std::endl;
        for (int i = 0; i < arrArrResCroods.GetCount(); i++){
            generateOutputCloud(i, 0, arrResTypes[i], stopline, arrArrResCroods[i], 0, 6);
        }
        std::cout << "data_type: " << data_type << std::endl;
        if (data_type == "BYD_LIDAR_B" || data_type == "BYD_LIDAR_BEV_B") {
            // 输出点云
            if(stopline!=nullptr && stopline->size()>0) {
                pcl::io::savePCDFileBinary(outputDir + "/" + "object_stopline.pcd", *stopline);
            }             
        } else {
            pcl::PointCloud<PointElement>::Ptr stopline_final = processStopline::afterProcess(middle_json_path, dataDir, refFile, stopline);
            
            // 输出点云
            if(stopline_final!=nullptr && stopline_final->size()>0) {
                pcl::io::savePCDFileBinary(outputDir + "/" + "object_stopline.pcd", *stopline_final);
            }   
        }           
    }

    float processStopline::computeProjectionOverlap(float a1, float a2, float b1, float b2) {
        // 确保范围的起点小于终点
        float min1 = std::min(a1, a2);
        float max1 = std::max(a1, a2);
        float min2 = std::min(b1, b2);
        float max2 = std::max(b1, b2);

        // 计算重叠区间
        float overlap_start = std::max(min1, min2);
        float overlap_end = std::min(max1, max2);

        // 如果没有重叠，返回 0
        if (overlap_start >= overlap_end) {
            return 0.0f;
        }

        // 计算重叠长度
        float overlap_length = overlap_end - overlap_start;

        // 计算两条线段的总长度
        float total_length1 = max1 - min1;
        float total_length2 = max2 - min2;

        // 返回重叠程度（取最小的总长度作为分母）
        float total_length = std::min(total_length1, total_length2);
        return overlap_length / total_length;
    }

    float processStopline::computeOverlap(const Eigen::Vector3f& point1, const Eigen::Vector3f& point2,
                        const Eigen::Vector3f& other_point1, const Eigen::Vector3f& other_point2,
                        const Eigen::Vector3f& line_vector) {
        // 将每条线段的端点投影到 line_vector 上
        float proj_point1 = line_vector.dot(point1);
        float proj_point2 = line_vector.dot(point2);
        float proj_other_point1 = line_vector.dot(other_point1);
        float proj_other_point2 = line_vector.dot(other_point2);
        
        // 计算投影后的重叠程度
        return computeProjectionOverlap(proj_point1, proj_point2, proj_other_point1, proj_other_point2);
    }

    pcl::PointCloud<PointElement>::Ptr processStopline::afterProcess(std::string middle_json_path, std::string dataDir, std::string refFile, pcl::PointCloud<PointElement>::Ptr stopline) {
        // 读取车道线
        std::ifstream ifs_parseInfo(refFile);
        if(!ifs_parseInfo.is_open()){
            ifs_parseInfo.close();
            return pcl::PointCloud<PointElement>::Ptr(new pcl::PointCloud<PointElement>);
        }   

        nlohmann::json parseInfo = nlohmann::json::parse(ifs_parseInfo);
        std::vector<std::string> links = parseInfo["links"];

        std::vector<std::string> files;
        pcl::PointCloud<pcl::PointXYZI>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZI>);
        
        #pragma omp parallel for
        for (const auto& link : links) {
            std::string laneBoundaryFromSeg = dataDir+"/" + link + "/laneBoundaryFromSeg";
            std::vector<std::string> file_names;
            Utils::getFiles(laneBoundaryFromSeg, file_names, [](std::string str)->bool{
                return boost::algorithm::ends_with(str, "_trjCloud.pcd");
            });

            #pragma omp critical
            for (auto filename : file_names) {
                std::string file_path = laneBoundaryFromSeg + "/" + filename;
                files.push_back(file_path);
            }
        }

        int line_id = 0;
        for (const auto& file : files) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(file, *cloud);
            for (const auto& point : *cloud) {
                refLaneBoundaryCloud->push_back(pcl::PointXYZI(point.x, point.y, point.z, line_id));
            }
            line_id++;
        }

        std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> lane_clouds_cache;
        lane_clouds_cache.reserve(refLaneBoundaryCloud->size());
        for (const auto& pt : *refLaneBoundaryCloud) {
            auto& cloud = lane_clouds_cache.emplace(
                static_cast<int>(pt.intensity), std::make_shared<pcl::PointCloud<pcl::PointXYZ>>()
            ).first->second;

            cloud->emplace_back(pt.x, pt.y, 0.0f);
        }

        // 计算附近车道线主方向，对停止线进行矫正
        pcl::PointCloud<PointElement>::Ptr corrected_stopline(new pcl::PointCloud<PointElement>(*stopline));
        
        #pragma omp parallel for
        for (size_t i = 0; i < corrected_stopline->size(); i += 2) {
            pcl::PointXYZ center((corrected_stopline->at(i).x + corrected_stopline->at(i + 1).x) / 2.0,
                                  (corrected_stopline->at(i).y + corrected_stopline->at(i + 1).y) / 2.0, 0.0);
            
            auto nearby_lines = findLinesWithDynamicThreshold(center, 4.0f, 10.0f, lane_clouds_cache);
            auto main_direction = filterLinesByDirection(std::move(nearby_lines), 30.0f).main_direction;
            
            Eigen::Vector3f perpendicular(-main_direction.y(), main_direction.x(), 0.0f);
            perpendicular.normalize();            
            
            auto& p1 = corrected_stopline->at(i);
            auto& p2 = corrected_stopline->at(i + 1);
            Eigen::Vector3f stop_direction(p2.x - p1.x, p2.y - p1.y, 0.0f);
            if (stop_direction.norm() < 1e-6) continue;
            stop_direction.normalize();

            float length = std::sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y));
            float cos_theta = std::abs(stop_direction.dot(perpendicular));
            float theta_deg = std::acos(cos_theta) * 180.0f / M_PI;
            std::cout << "theta_deg: " << theta_deg << ", p1: "<< p1.x << " " << p1.y << " " << p1.z << ", p2: " << p2.x << " " << p2.y << " " << p2.z << std::endl;
            constexpr float ANGLE_THRESHOLD = 30.0f;
            if (theta_deg <= ANGLE_THRESHOLD) {
                // 保持原长度和中心点，调整方向
                Eigen::Vector3f half_vec = perpendicular  * (length / 2.0f);
                
                // 更新端点坐标（Z轴保持不变）
                corrected_stopline->at(i).x = center.x - half_vec.x();
                corrected_stopline->at(i).y = center.y - half_vec.y();
                corrected_stopline->at(i+1).x = center.x + half_vec.x();
                corrected_stopline->at(i+1).y = center.y + half_vec.y();
            }
   
        }
    
        std::set<size_t> indices_to_remove;

        #pragma omp parallel for
        for (size_t i = 0; i < corrected_stopline->size(); i += 2) {
            // 当前停止线的两个端点
            Eigen::Vector3f point1(corrected_stopline->at(i).x, corrected_stopline->at(i).y, 0);
            Eigen::Vector3f point2(corrected_stopline->at(i + 1).x, corrected_stopline->at(i + 1).y, 0);
            Eigen::Vector3f center((point1 + point2) / 2.0f);
            Eigen::Vector3f line_vector = point2 - point1;
            if (line_vector.norm() < 1e-6) continue;
            line_vector.normalize();

            // 遍历其他停止线以寻找相似平行线
            for (size_t j = i + 2; j < corrected_stopline->size(); j += 2) {
                if (i == j) continue;

                Eigen::Vector3f other_point1(corrected_stopline->at(j).x, corrected_stopline->at(j).y, 0.0f);
                Eigen::Vector3f other_point2(corrected_stopline->at(j + 1).x, corrected_stopline->at(j + 1).y, 0.0f);
                Eigen::Vector3f other_center((other_point1 + other_point2) / 2.0f);
                Eigen::Vector3f other_line_vector = other_point2 - other_point1;
                if (other_line_vector.norm() < 1e-6) continue; // 避免除以零
                other_line_vector.normalize();

                // 判断两条线段是否近似平行
                if (std::abs(line_vector.dot(other_line_vector)) > 0.98) {
                    // 计算两条线段在line_vector上的投影重叠程度
                    float overlap = computeOverlap(point1, point2, other_point1, other_point2, line_vector);
                    // 计算两条线段中心点在line_vector垂直方向上的投影
                    float distance = (center - other_center).cross(line_vector).norm();
                    if (overlap > 0.5 && distance < 1.5) { // 如果两条线段重叠程度大于 0.5，且在line_vector 垂直方向上距离小于1.5米，丢掉短的停止线
                        #pragma omp critical
                        const float len_current = (point2 - point1).norm();
                        const float len_other = (other_point2 - other_point1).norm();
                        if (len_current < len_other - 0.01f) {
                            indices_to_remove.insert(i / 2); // 标记被丢弃的线段
                        } else if (len_other < len_current - 0.01f) {
                            indices_to_remove.insert(j / 2);
                        }
                    }
                }
            }
        }

        pcl::PointCloud<PointElement>::Ptr new_stopline(new pcl::PointCloud<PointElement>);
        for (size_t i = 0; i < corrected_stopline->size(); i += 2) {
            if (indices_to_remove.find(i / 2) == indices_to_remove.end()) {
                new_stopline->push_back(corrected_stopline->at(i));
                new_stopline->push_back(corrected_stopline->at(i+1));
            }
        } 

        return new_stopline;
    }  


    processStopline::NearbyLinesResult processStopline::findNearbyLinesWithMinPoints(
        const pcl::PointXYZ& target_point,
        double threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache,
        int min_points)
    {
        NearbyLinesResult result;
        const double squared_threshold = threshold * threshold;
        const auto& tx = target_point.x;
        const auto& ty = target_point.y;
        const auto& tz = target_point.z;

        std::vector<std::pair<int, size_t>> line_sizes;

        for (const auto& [line_id, cloud] : lane_clouds_cache) {
            if (!cloud || cloud->empty()) continue;

            auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            for (const auto& pt : *cloud) {
                const double dx = pt.x - tx;
                const double dy = pt.y - ty;
                const double dz = pt.z - tz;
                if ((dx*dx + dy*dy + dz*dz) <= squared_threshold) {
                    filtered_cloud->push_back(pt);
                }
            }

            if (filtered_cloud->size() >= min_points) {
                result.line_ids.push_back(line_id);
                result.line_points[line_id] = filtered_cloud;
                line_sizes.emplace_back(line_id, filtered_cloud->size());
            }
        }

        // 如果找到的线条数量大于3，保留点数最多的前三条
        if (line_sizes.size() > 3) {
            std::sort(line_sizes.begin(), line_sizes.end(), [](const auto& a, const auto& b) {
                return a.second > b.second;
            });

            // 只保留前三条
            result.line_ids = {line_sizes[0].first, line_sizes[1].first, line_sizes[2].first};
            std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> new_line_points;
            for (const auto& [line_id, _] : line_sizes) {
                if (new_line_points.size() < 3) {
                    new_line_points[line_id] = result.line_points[line_id];
                }
            }
            result.line_points = std::move(new_line_points);
        }

        return result;
    }

    processStopline::NearbyLinesResult processStopline::findLinesWithDynamicThreshold(
        const pcl::PointXYZ& target_point,
        double initial_threshold,
        double max_threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache)
    {
        double current_threshold = initial_threshold;
        while (current_threshold <= max_threshold) {
            auto result = findNearbyLinesWithMinPoints(target_point, current_threshold, lane_clouds_cache, 2);
            if (result.line_ids.size() >= 3) return result;
            current_threshold *= 1.5;
        }
        return findNearbyLinesWithMinPoints(target_point, max_threshold, lane_clouds_cache, 2);
    }


    processStopline::DirectionFilterResult processStopline::filterLinesByDirection(
        const NearbyLinesResult& candidates,
        float angle_threshold_deg)
    {
        DirectionFilterResult result;
        if (candidates.line_ids.empty()) return result;


        std::vector<Eigen::Vector3f> directions;
        std::vector<int> valid_lines;

        for (const int line_id : candidates.line_ids) {
            const auto& cloud = candidates.line_points.at(line_id);
            if (cloud->size() < 2) continue;

            Eigen::Vector3f dir;
            if (cloud->size() == 2) {  // 两点直接计算
                const auto& p1 = cloud->at(0);
                const auto& p2 = cloud->at(1);
                dir = {p2.x - p1.x, p2.y - p1.y, p2.z - p1.z};
            } else {  // PCA主成分分析
                pcl::PCA<pcl::PointXYZ> pca;
                pca.setInputCloud(cloud);
                dir = pca.getEigenVectors().col(0);
            }

            if (dir.norm() < 1e-6) continue;
            directions.push_back(dir.normalized());
            valid_lines.push_back(line_id);
        }

        if (directions.empty()) return result;

        Eigen::Vector3f sum_dir = Eigen::Vector3f::Zero();
        for (const auto& dir : directions) {
            sum_dir += dir;
        }
        Eigen::Vector3f avg_dir = sum_dir / directions.size();
        avg_dir.normalize();

        const float cos_th = std::cos(angle_threshold_deg * M_PI / 180.0f);
        std::vector<Eigen::Vector3f> filtered_directions;

        for (size_t i = 0; i < directions.size(); ++i) {
            if (directions[i].dot(avg_dir) >= cos_th) {
                result.filtered_lines.push_back(valid_lines[i]);
                filtered_directions.push_back(directions[i]);
            }
        }

        if (!filtered_directions.empty()) {
            Eigen::Vector3f sum_dir = Eigen::Vector3f::Zero();
            for (const auto& dir : filtered_directions) {
                sum_dir += dir;
            }
            result.main_direction = sum_dir.normalized();
        }

        return result;
    }        

}
