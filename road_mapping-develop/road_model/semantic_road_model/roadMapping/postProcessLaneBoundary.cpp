#include "postProcessLaneBoundary.h"
#include "pclPtType.h"
#include "pclFilter.h"
#include <queue>
#include <map>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/keypoints/uniform_sampling.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "earth.hpp"
#include "Utils.h"
#include "units.hpp"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "cluster.h"
#include "json.hpp"

namespace RoadMapping{

float PostProcessLaneBoundary::quadArea(LineSection &a, LineSection &b)
{
    Eigen::Vector3f l1 = b.leftTruncatePt - a.leftTruncatePt,
                    l2 = b.rightTruncatePt - b.leftTruncatePt,
                    l3 = a.rightTruncatePt - b.rightTruncatePt,
                    l4 = a.leftTruncatePt - a.rightTruncatePt;
    float area = (l1.cross(l2)).norm() + (l3.cross(l4)).norm();
    return area/2;
}

bool PostProcessLaneBoundary::estimateDirection(LineSection &lineSection, int classId, Sector &prevSector, Sector &nextSector, Eigen::Vector3f refDirection)
{
    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Matrix3f A = Eigen::Matrix3f::Zero();
    int totalNum = 0;
    for(auto pt : lineSection.pts){
        centroid += pt;
        totalNum += 1;
    }    
    if(prevSector.lineSections.count(classId)){
        for(auto pt : prevSector.lineSections[classId].pts){
            centroid += pt;
            totalNum += 1;
        }
    }
    if(nextSector.lineSections.count(classId)){
        for(auto pt : nextSector.lineSections[classId].pts){
            centroid += pt;
            totalNum += 1;            
        }
    }
    centroid = centroid/totalNum;
    for(auto pt : lineSection.pts){
        A += (pt-centroid)*(pt-centroid).transpose();
    }    
    if(prevSector.lineSections.count(classId)){
        for(auto pt : prevSector.lineSections[classId].pts){
            A += (pt-centroid)*(pt-centroid).transpose();
        }
    }
    if(nextSector.lineSections.count(classId)){
        for(auto pt : nextSector.lineSections[classId].pts){
           A += (pt-centroid)*(pt-centroid).transpose();
        }
    }    
    A = A/totalNum;
    A = (A + A.transpose())/2;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
    es.compute(A);
    Eigen::Vector3f eigenvalues = es.eigenvalues();
    //std::cout<<"eigenvalues: "<<eigenvalues.transpose()<<std::endl;
    if(eigenvalues(2) < 10*eigenvalues(1)){
        return false;
    }    
    Eigen::Matrix3f eigenvectors = es.eigenvectors();
    Eigen::Vector3f direction = eigenvectors.col(2);
    if(direction.dot(refDirection) < 0.f){
        direction = -direction;
    }
    float minVal = 1e6, maxVal = -1e6;
    for(auto pt : lineSection.pts){
        float val = (pt - centroid).dot(direction);
        if(val<minVal) minVal = val;
        if(val>maxVal) maxVal = val;
    }
    lineSection.leftPt = centroid + minVal*direction;
    lineSection.rightPt = centroid + maxVal*direction;
    if(prevSector.lineSections.count(classId)){ // 目的：计算线段的两个端点的范围，如果后面的锚点也有这条线，那么可以加到当前的锚点统计中，计算最小的 minVal
        for(auto pt : prevSector.lineSections[classId].pts){
            float val = (pt - centroid).dot(direction);
            if(val<minVal) minVal = val;            
        }
    }    
    lineSection.leftTruncatePt = centroid + minVal*direction;
    if(nextSector.lineSections.count(classId)){ // 目的：计算线段的两个端点的范围，如果前面的锚点也有这条线，那么可以加到当前的锚点统计中，计算最大的 maxVal
        for(auto pt : nextSector.lineSections[classId].pts){
           float val = (pt - centroid).dot(direction);
           if(val>maxVal) maxVal = val;
        }
    }    
    lineSection.rightTruncatePt = centroid + maxVal*direction;
    //std::cout<<"leftTruncatePt: "<<lineSection.leftTruncatePt.transpose()
    //         <<" rightTruncatePt: "<<lineSection.rightTruncatePt.transpose()<<std::endl;
    return true;
}

void PostProcessLaneBoundary::MergeLineSection(Sector &sector, Eigen::Vector3f refDirection)
{
    auto &lineSections = sector.lineSections;
    std::vector<int> classIds;
    for(auto iter = lineSections.begin(); iter!=lineSections.end(); iter++){
        classIds.push_back(iter->first);
    }
    std::vector<bool> flag_used(classIds.size(), false);
    for(int i=0; i<classIds.size(); i++){
        if(flag_used[i]) continue;
        float minArea = 1e6;
        int minIndex = -1;
        for(int j=i+1; j<classIds.size(); j++){ // 每个seg 与 其他seg计算面积，选择面积最小的，如果满足阈值，后面会进行合并
            if(flag_used[j]) continue;
            float area = quadArea(lineSections[classIds[i]], lineSections[classIds[j]]);
            if(area < minArea){
                minArea = area;
                minIndex = j;
            }
        }
        flag_used[i] = true;
        if(minArea > 2.f){
            auto &lineSection = lineSections[classIds[i]];
            std::sort(lineSection.pts.begin(), lineSection.pts.end(), [&](Eigen::Vector3f pt_0, Eigen::Vector3f pt_1){
                return (pt_0-lineSection.leftPt).dot(refDirection) < (pt_1-lineSection.leftPt).dot(refDirection);
            });
            sector.mergedLineSections.push_back(lineSection); // 当前线段与其他seg离得比较远，直接生成一条独立得线段
            continue;
        }
        std::cout<<"merge sector: "<<classIds[i]<<" "<<classIds[minIndex]<<" "<<minArea<<std::endl;
        flag_used[minIndex] = true;
        LineSection &lineSection_0 = lineSections[classIds[i]];
        LineSection &lineSection_1 = lineSections[classIds[minIndex]];
        LineSection lineSection;
        if(refDirection.dot(lineSection_1.leftPt - lineSection_0.leftPt)>0.f){
            lineSection.pts.insert(lineSection.pts.end(), lineSection_0.pts.begin(), lineSection_0.pts.end());
            lineSection.pts.insert(lineSection.pts.end(), lineSection_1.pts.begin(), lineSection_1.pts.end());
            lineSection.leftPt = lineSection.leftTruncatePt = lineSection_0.leftPt;
            lineSection.rightPt = lineSection.rightTruncatePt = lineSection_1.rightPt;
        }else{
            lineSection.pts.insert(lineSection.pts.end(), lineSection_1.pts.begin(), lineSection_1.pts.end());
            lineSection.pts.insert(lineSection.pts.end(), lineSection_0.pts.begin(), lineSection_0.pts.end());  
            lineSection.leftPt = lineSection.leftTruncatePt = lineSection_1.leftPt;  
            lineSection.rightPt = lineSection.rightTruncatePt = lineSection_0.rightPt;        
        }
        std::sort(lineSection.pts.begin(), lineSection.pts.end(), [&](Eigen::Vector3f pt_0, Eigen::Vector3f pt_1){
            return (pt_0-lineSection.leftPt).dot(refDirection) < (pt_1-lineSection.leftPt).dot(refDirection);
        });
        sector.mergedLineSections.push_back(lineSection);
    }
    sector.nextIndices = std::vector<int>(sector.mergedLineSections.size(), -1);
}

cv::Mat PostProcessLaneBoundary::bin_img_to_skel_img(cv::Mat &bin_img)
{
    //输入输出 单通道 前景白色 背景黑色
    cv::Mat tmp = bin_img.clone();
    cv::Mat out(tmp.size(), CV_8UC1, cv::Scalar(0));
    {
        cv::Mat temp;
        cv::Mat eroded;
        cv::Mat element3 = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
        bool done;
        do
        {
            cv::erode(tmp, eroded, element3);
            cv::dilate(eroded, temp, element3); // temp = open(img)
            cv::subtract(tmp, temp, temp);
            cv::bitwise_or(out, temp, out);
            eroded.copyTo(tmp);
            done = (cv::countNonZero(tmp) == 0);
        } while (!done);
    }
    return out;
}

cv::Mat PostProcessLaneBoundary::bin_img_remove_small_region(cv::Mat &bin_img, int min_area)
{
	//输入输出 单通道 前景白色 背景黑色
	cv::Mat out(bin_img.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat stats, centroids, labelImage, imshow_mat;
	int nLabels = cv::connectedComponentsWithStats(
		bin_img, labelImage, stats, centroids, 8);
	cv::Mat mask(labelImage.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat surfSup = stats.col(4) >= min_area;
	int tmp_label;
	for (int i = 1; i < bin_img.rows; i++)
	{
		for (int j = 1; j < bin_img.cols; j++)
		{
			tmp_label = labelImage.at<int>(i, j);
			mask.at<char>(i, j) = (char)surfSup.at<char>(tmp_label, 0);
		}
	}
	bin_img.copyTo(out, mask);
	return out;
}

void PostProcessLaneBoundary::lane_extract_base(std::string outputDir,std::string outnamechar,pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor)
{
    int refLaneAnchorNum = refLaneAnchorCloud->size();
    if (refLaneAnchorNum < 1) {
        return;
    }

    //聚类
    std::vector<std::vector<int>> clusters;
    float radius = 0.5f;
    int minPts = 20;
    Inference::DBSCAN(laneSegCloud, radius, minPts, clusters);
    std::cout<<"lane clustered number: "<<clusters.size()<<std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
    std::map<int, int> classIdsMap; // 聚类后点云索引，聚类类别
    for (int i = 0; i < clusters.size(); ++i) {
        if(clusters[i].size() < 50) //按照1米20个点 要求长度最少5米 修改背景：发现很多被误删的 将条件放宽，尽可能都参与建模
            continue;
        for(int j = 0; j < clusters[i].size();j++)
        {
            classIdsMap.insert(std::make_pair(clusters[i][j],i));
            pcl::PointXYZ curPXYZ = laneSegCloud->at(clusters[i][j]);
            clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
        }
    }
    if(clustersCloud->empty() || clustersCloud->points.empty())
    {
        return;
    }
    pcl::io::savePCDFileBinary(outputDir+"/clustersCloud.pcd", *clustersCloud);

    std::vector<Sector> sectors(refLaneAnchorNum); // 根据 link anchor 点 划分为多个条形区域
    std::map<int, int>::iterator m1_Iter;
    // 遍历每个聚类的点，并通过 link anchor tree，找最近的1个点
    // 保存每个条形区域里面每个聚类的点
    for(m1_Iter = classIdsMap.begin(); m1_Iter != classIdsMap.end(); m1_Iter++ ){ 
        int indexPt = m1_Iter->first;
        pcl::PointXYZ pt = laneSegCloud->at(indexPt);
        int K = 1;
        std::vector<int> Idx;
        std::vector<float> SquaredDistance;
        kdtree_refLaneAnchor->nearestKSearch(pt, K, Idx, SquaredDistance); // 找最近的 link anchor 点
        if(Idx.empty()) continue; // TODO:qzc 为啥这里不判断距离？
        int index = Idx.front();
        int classId = m1_Iter->second;
        sectors[index].lineSections[classId].pts.push_back(pt.getVector3fMap()); // 保存每个条形区域里面每个聚类的点
        // 此处将第一个sectors[0]和最后一个sectors[n-1]的点放到 sectors[1] 和 sectors[n-2]
        if (index == 0) {
            sectors[index+1].lineSections[classId].pts.push_back(pt.getVector3fMap()); // 保存每个条形区域里面每个聚类的点
        }
        if (index == refLaneAnchorNum-1) {
             sectors[index-1].lineSections[classId].pts.push_back(pt.getVector3fMap()); // 保存每个条形区域里面每个聚类的点
        }
    }

    //estimate lineSections, 通过 sectors（每个锚点上，关联了每个聚类的segment），计算每个线段的方向
    for(int i=1; i+1<sectors.size(); i++){
        auto &lineSections = sectors[i].lineSections; // 每个锚点关联的聚类类别的segment
        std::vector<int> eraseIds;
        for(auto iter=lineSections.begin(); iter!=lineSections.end(); iter++){
            int classId = iter->first;
            Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
            if(!estimateDirection(iter->second, classId, sectors[i-1], sectors[i+1], refDirection)){
                eraseIds.push_back(classId);
            }
        }
        for(auto classId : eraseIds){
            lineSections.erase(classId);
            std::cout<<i<<" th sector erased "<<classId<<" LineSection"<<std::endl;
        }
    }

    // debug begin
    // std::cout << "sector.size: " << sectors.size() << std::endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr sectorsCloud(new pcl::PointCloud<pcl::PointXYZI>);
    // for(int i=0; i<sectors.size(); i++){
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    //     for(auto iter = sectors[i].lineSections.begin(); iter!=sectors[i].lineSections.end(); iter++){
    //         LineSection &lineSection = iter->second;
    //         for(auto pt : lineSection.pts){
    //             pcl::PointXYZI point(pt(0), pt(1), pt(2), i);
    //             // pcl::PointXYZI point(pt(0), pt(1), pt(2)+0.1, i);
    //             cloud->push_back(point);
    //             sectorsCloud->push_back(point);
    //         }
    //     }
    //     if (cloud->size() > 0) {
    //         pcl::io::savePCDFileBinary(outputDir+"/sector_"+std::to_string(i)+".pcd", *cloud);
    //     }
    // }
    // if(!sectorsCloud->empty() && !sectorsCloud->points.empty()) {
    //     int sector_size = sectors.size();
    //     pcl::io::savePCDFileBinary(outputDir+"/sector_all_"+std::to_string(sector_size)+".pcd", *sectorsCloud);
    //     // pcl::io::savePCDFileBinary(outputDir+"/clustersCloud.pcd", *clustersCloud);
    // }
    // debug end

    // 通过统计两两之间seg之间的面积，间接的反应两条线段是否离的很近，如果很近，就直接合并
    for(int i=1; i+1<sectors.size(); i++){
        Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
        MergeLineSection(sectors[i], refDirection); // 单sector里面的线段合并
    }
    // for(int i=0; i<sectors.size(); i++){
    //     Eigen::Vector3f refDirection;
    //     if(i==0) {
    //         refDirection = refLaneAnchorCloud->at(1).getVector3fMap() - refLaneAnchorCloud->at(0).getVector3fMap();
    //     } else{
    //         refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
    //     }
    //     MergeLineSection(sectors[i], refDirection); // 单sector里面的线段合并
    // }

    std::vector<Inference::Segment> segments;
    for(int i=0; i+1<sectors.size(); i++){ // 多 sector 之间的线段进行合并
        for(int j=0; j<sectors[i].mergedLineSections.size(); j++){
            if(sectors[i].mergedLineSections[j].flag_tracked) continue;

            std::vector<std::pair<int,int>> trackedIds; // 第 i 个锚点id 上的第 j 条线段id
            trackedIds.push_back(std::make_pair(i,j));
            int s, t, mint=-1;

            do{
                std::pair<int, int> currentId = trackedIds.back();
                float minVal = 1e6;
                for(s=currentId.first+1; s<=currentId.first+5 && s+1<sectors.size(); s++){ // 后面连续的5个锚点
                    for(t=0; t<sectors[s].mergedLineSections.size(); t++){ // 对应锚点上已经进行过合并的线段
                        if(sectors[s].mergedLineSections[t].flag_tracked) continue;

                        float area = quadArea(sectors[currentId.first].mergedLineSections[currentId.second],
                                              sectors[s].mergedLineSections[t]); // 计算当前锚点 与 其他5个锚点的距离
                        if(area < minVal){
                            minVal = area;
                            mint = t;
                        }
                    }
                    if(minVal<5.f) break; // 如果有面积比较小的，说明距离比较近，
                }

                if(minVal<5.f){
                    trackedIds.push_back(std::make_pair(s,mint));
                }else{
                    break;
                }
            }while(1); // 这里会将不同sector上的同一条线段的id合并为一条, id 先保存在 trackedIds， 合并动作在下面

            for(auto id : trackedIds){
                sectors[id.first].mergedLineSections[id.second].flag_tracked = true;
            }

            if(trackedIds.size() < 3){
                continue;
            }

            Inference::Segment segment;
            float anchorGap = 5.f;
            std::vector<int> anchorIndices;
            for(auto id : trackedIds){
                std::cout<<"("<<id.first<<" "<<id.second<<") "; // 哪个 sector 的 哪条线 被合并为了一条线(注意此处的second并不一定是聚类的id，可能会变)
                int prevNum = segment.cloud->size();
                for(auto pt : sectors[id.first].mergedLineSections[id.second].pts){ // 当前sec上，需要合并的 sections
                    segment.cloud->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
                }
                int currNum = segment.cloud->size();
                anchorIndices.push_back(prevNum);
                anchorIndices.push_back(currNum-1);
            }
            if(anchorIndices.empty()) continue;
            segment.anchorIndices.push_back(anchorIndices.front());
            for(int k=1;k<anchorIndices.size(); k++){
                Eigen::Vector3f currPt = segment.cloud->at(anchorIndices[k]).getVector3fMap();
                int prevIndex = segment.anchorIndices.back();
                Eigen::Vector3f prevPt = segment.cloud->at(prevIndex).getVector3fMap();
                // if((currPt-prevPt).norm() > anchorGap){
                if((currPt-prevPt).norm() > anchorGap || k == anchorIndices.size()-1){
                    segment.anchorIndices.push_back(anchorIndices[k]);
                }
            }
            std::cout<<std::endl;
            // for bebug begin
            pcl::io::savePCDFileBinary(outputDir+"/lane_line_"+std::to_string(trackedIds[0].second)+".pcd", *(segment.cloud));
            // for debug end

            segments.push_back(segment); // 合并后的某一条线段保存到 segments中
        }
    }

    std::vector<std::pair<int, float>> disp(segments.size()); // 车道线上的累计距离
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
        std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    }
    std::sort(disp.begin(), disp.end(), [](std::pair<int, float> a, std::pair<int, float> b){
        return a.second > b.second;
    });


//    nlohmann::json obj;
//    obj["base"]["lat"] = 0.0;
//    obj["base"]["lon"] = 0.0;
//    obj["base"]["alt"] = 0.0;
    for(int i=0; i<segments.size(); i++){
        int index = disp[i].first;

//        if(segments[index].cloud !=NULL && segments[index].cloud->size()>0){
//            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[index].cloud);
//        }

        segments[index].QpSpline();
        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[index].smoothTrjCloud();
        if(trjCloud!=NULL && trjCloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_" + outnamechar + ".pcd", *trjCloud);
        }

//        segments[index].addSegmentCoeff(obj, i);
        //segments[index].addSegmentCoeff_straightLine(obj, i);
    }

//    std::ofstream fid_laneBoundary(outputDir+"/laneBoundary.json");
//    //fid_laneBoundary << std::setw(4) << obj << std::endl;
//    fid_laneBoundary<< obj << std::endl;
}

void PostProcessLaneBoundary::get_ref_info(std::string parse_json,std::string outputDir, pcl::PointCloud<pcl::PointXYZ>::Ptr &laneSegCloud,pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneAnchorCloud, pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_refLaneAnchor)
{
    Utils::readRefLines_2(parse_json, refLaneBoundaryCloud); // link 信息
    if(refLaneBoundaryCloud->empty() || refLaneBoundaryCloud->points.empty())
    {
        return;
    }
    pcl::io::savePCDFileBinary(outputDir+"/refLink.pcd", *refLaneBoundaryCloud);

    //计算方向向量
    Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
    for(int i=1; i<refLaneBoundaryCloud->size(); i++){
        refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
    }
    refDirection.normalize();
    Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();

    float minVal = 1e6, maxVal = -1e6;
    for(int i=0; i<laneSegCloud->size(); i++){
        float val = refDirection.dot(laneSegCloud->at(i).getVector3fMap()-refCentroid);
        if(val<minVal) minVal = val;
        if(val>maxVal) maxVal = val;
    }
    float resolution = 3.f;
//    int num = (maxVal - minVal)/resolution + 1;
    int num1 = maxVal/resolution + 1;
    int num2 = (fabs(minVal))/resolution + 1;
    for(int i=-num2; i<=num1; i++){
        Eigen::Vector3f pos = refCentroid + resolution*i*refDirection;
        refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    }
    if(refLaneAnchorCloud->empty() || refLaneAnchorCloud->points.empty())
    {
        return;
    }
    pcl::io::savePCDFileBinary(outputDir+"/refLaneAnchorCloud.pcd", *refLaneAnchorCloud);
    kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree
}

void PostProcessLaneBoundary::run(std::string dataDir, std::string outputDir)
{
    std::string reconsMergeOutputDir = dataDir+"/reconstruction/output";
    std::string bevInfoJsonFile = reconsMergeOutputDir + "/bev_info.json";
    std::cout<<"bevInfoJsonFile: "<<bevInfoJsonFile<<std::endl;
    std::ifstream ifs_bevInfo(bevInfoJsonFile);
    if(!ifs_bevInfo.is_open()){
        ifs_bevInfo.close();
        return;
    }
    nlohmann::json bevInfo;
    bevInfo<<ifs_bevInfo;
    int utm_num = bevInfo["utm_num"];
    float pixelSize = bevInfo["pixel_size_m"];
    Eigen::Matrix3d T_utm_bev = Eigen::Matrix3d::Identity();
    T_utm_bev<<bevInfo["T_utm_bev"][0][0], bevInfo["T_utm_bev"][0][1], bevInfo["T_utm_bev"][0][2],
            bevInfo["T_utm_bev"][1][0], bevInfo["T_utm_bev"][1][1], bevInfo["T_utm_bev"][1][2],
            bevInfo["T_utm_bev"][2][0], bevInfo["T_utm_bev"][2][1], bevInfo["T_utm_bev"][2][2];
    std::cout<<"T_utm_bev:\n"<<T_utm_bev<<std::endl;
    Eigen::Vector2d UTM_base = T_utm_bev.block<2,1>(0,2);
//    projPJ g_pWGS84 = pj_init_plus("+proj=latlong +datum=WGS84");
//    std::string utm_param = "+proj=utm";
//    utm_param = utm_param + " +zone=" + std::to_string(utm_num) + "N +ellps=WGS84 +no_defs";
//    projPJ g_utm = pj_init_plus(utm_param.data());

//    auto UTM2llh = [&](Eigen::Vector3d utmPos){
//        Eigen::Vector3d llh = utmPos;
//        pj_transform(g_utm, g_pWGS84, 1, 1, &llh(0), &llh(1), &llh(2));
//        std::swap(llh(0), llh(1));
//        return llh;
//    };

    Eigen::Vector3d init_UTM = Eigen::Vector3d(UTM_base(0), UTM_base(1), 0.f);
    std::cout<<"init_llh: "<<init_UTM.transpose()<<std::endl;

    //输入参考线
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string  parse_json = dataDir + "/parse_json.json";
    Utils::readRefLines(parse_json, refLaneBoundaryCloud, init_UTM);
    pcl::io::savePCDFileBinary(outputDir+"/refLink.pcd", *refLaneBoundaryCloud);

    //计算方向向量
    Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
    for(int i=1; i<refLaneBoundaryCloud->size(); i++){
        refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
    }
    refDirection.normalize();
    Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();


    //"line_id": 255, "arrow_id": 200, "road_id": 150, "boundary_id": 100
    std::string roadSegImgFile = reconsMergeOutputDir + "/bev_road_seg.png";
    cv::Mat img_bevRoadSeg = cv::imread(roadSegImgFile, cv::IMREAD_GRAYSCALE);
    std::cout << "img rows: " << img_bevRoadSeg.rows << " cols: " << img_bevRoadSeg.cols << std::endl;

    //cluster laneSeg img
    std::vector<std::vector<bool>> visited(img_bevRoadSeg.rows, std::vector<bool>(img_bevRoadSeg.cols, false));
    std::vector<std::vector<std::pair<int,int>>> classes;
    for(int i=0; i<img_bevRoadSeg.rows; i++){
        for(int j=0; j<img_bevRoadSeg.cols; j++){
            if(img_bevRoadSeg.at<uchar>(i,j)!=255 || visited[i][j]) continue;
            std::vector<std::pair<int,int>> classElements;
            visited[i][j] = true;
            classElements.push_back(std::make_pair(i,j));
            std::queue<std::pair<int,int>> q;
            q.push(std::make_pair(i,j));
            while(!q.empty()){
                std::pair<int,int> pos = q.front();
                q.pop();
                int row = pos.first, col = pos.second;
                if(row>0 && img_bevRoadSeg.at<uchar>(row-1,col)==255 && !visited[row-1][col]){
                    visited[row-1][col] = true;
                    q.push(std::make_pair(row-1, col));
                    classElements.push_back(std::make_pair(row-1, col));
                }
                if(col>0 && img_bevRoadSeg.at<uchar>(row,col-1)==255 && !visited[row][col-1]){
                    visited[row][col-1] = true;
                    q.push(std::make_pair(row, col-1));
                    classElements.push_back(std::make_pair(row, col-1));
                }
                if(row+1<img_bevRoadSeg.rows && img_bevRoadSeg.at<uchar>(row+1,col)==255 && !visited[row+1][col]){
                    visited[row+1][col] = true;
                    q.push(std::make_pair(row+1, col));
                    classElements.push_back(std::make_pair(row+1, col));
                }
                if(col+1<img_bevRoadSeg.cols && img_bevRoadSeg.at<uchar>(row,col+1)==255 && !visited[row][col+1]){
                    visited[row][col+1] = true;
                    q.push(std::make_pair(row, col+1));
                    classElements.push_back(std::make_pair(row, col+1));
                }
            }
            if(classElements.size() < 100) continue;
            classes.push_back(classElements);
        }
    }
    std::cout<<"lane clustered number: "<<classes.size()<<std::endl;
    std::vector<std::vector<int>> classIds(img_bevRoadSeg.rows, std::vector<int>(img_bevRoadSeg.cols, -1));
    for(int i=0; i<classes.size(); i++){
        for(int j=0; j<classes[i].size(); j++){
            std::pair<int,int> pos = classes[i][j];
            classIds[pos.first][pos.second] = i;
        }
    }
    cv::Mat img_bevRoadSeg_bin = (img_bevRoadSeg==255);
    cv::Mat img_laneskel = bin_img_to_skel_img(img_bevRoadSeg_bin);
    cv::imwrite(outputDir+"/img_laneskel_beforeDenoise.png", img_laneskel);
    img_laneskel = bin_img_remove_small_region(img_laneskel, 3);
    cv::imwrite(outputDir+"/img_laneskel_afterDenoise.png", img_laneskel);

    pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegCloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<std::pair<int,int>> laneSegIndices;
    for(int i=0; i<img_bevRoadSeg.rows; i++){
        for(int j=0; j<img_bevRoadSeg.cols; j++){
            //if(classIds[i][j]<0 || img_laneskel.at<uchar>(i,j)==0) continue;
            if(classIds[i][j]<0) continue;
            Eigen::Vector3d utmPos(T_utm_bev(0,0)*j+T_utm_bev(0,1)*i+T_utm_bev(0,2),
                                   T_utm_bev(1,0)*j+T_utm_bev(1,1)*i+T_utm_bev(1,2), 0.f);
//            Eigen::Vector3d llh = UTM2llh(utmPos);
            Eigen::Vector3d pos = utmPos - init_UTM;
            laneSegCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            laneSegIndices.push_back(std::make_pair(i,j));
        }
    }
    pcl::io::savePCDFileBinary(outputDir+"/laneSegCloud.pcd", *laneSegCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    float minVal = 1e6, maxVal = -1e6;
    for(int i=0; i<laneSegCloud->size(); i++){
        float val = refDirection.dot(laneSegCloud->at(i).getVector3fMap()-refCentroid);
        if(val<minVal) minVal = val;
        if(val>maxVal) maxVal = val;
    }
    float resolution = 3.f;
//    int num = (maxVal - minVal)/resolution + 1;
    int num1 = maxVal/resolution + 1;
    int num2 = (fabs(minVal))/resolution + 1;
    for(int i=-num2; i<=num1; i++){
        Eigen::Vector3f pos = refCentroid + resolution*i*refDirection;
        refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    }
    pcl::io::savePCDFileBinary(outputDir+"/refLaneAnchorCloud.pcd", *refLaneAnchorCloud);

    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree

    std::vector<Sector> sectors(refLaneAnchorCloud->size());
    for(int i=0; i<laneSegCloud->size(); i++){
        pcl::PointXYZ pt = laneSegCloud->at(i);
        int K = 1;
        std::vector<int> Idx;
        std::vector<float> SquaredDistance;
        kdtree_refLaneAnchor->nearestKSearch(pt, K, Idx, SquaredDistance);
        if(Idx.empty()) continue;
        int index = Idx.front();
        std::pair<int,int> pos = laneSegIndices[i];
        int classId = classIds[pos.first][pos.second];
        sectors[index].lineSections[classId].pts.push_back(pt.getVector3fMap());
    }
    //estimate lineSections
    for(int i=1; i+1<sectors.size(); i++){
        auto &lineSections = sectors[i].lineSections;
        std::vector<int> eraseIds;
        for(auto iter=lineSections.begin(); iter!=lineSections.end(); iter++){
            int classId = iter->first;
            Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
            if(!estimateDirection(iter->second, classId, sectors[i-1], sectors[i+1], refDirection)){
                eraseIds.push_back(classId);
            }
        }
        for(auto classId : eraseIds){
            lineSections.erase(classId);
            std::cout<<i<<" th sector erased "<<classId<<" LineSection"<<std::endl;
        }
    }
    /*
    for(int i=0; i<sectors.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for(auto iter = sectors[i].lineSections.begin(); iter!=sectors[i].lineSections.end(); iter++){
            LineSection &lineSection = iter->second;
            for(auto pt : lineSection.pts){
                pcl::PointXYZ point(pt(0), pt(1), pt(2));
                cloud->push_back(point);
            }
        }
        pcl::io::savePCDFileBinary(outputDir+"/sector_"+std::to_string(i)+".pcd", *cloud);
    }
    */
    for(int i=1; i+1<sectors.size(); i++){
        Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
        MergeLineSection(sectors[i], refDirection);
    }

    std::vector<Inference::Segment> segments;
    for(int i=1; i+1<sectors.size(); i++){
        for(int j=0; j<sectors[i].mergedLineSections.size(); j++){
            if(sectors[i].mergedLineSections[j].flag_tracked) continue;
            std::vector<std::pair<int,int>> trackedIds;
            trackedIds.push_back(std::make_pair(i,j));
            int s, t, mint=-1;

            do{
                std::pair<int, int> currentId = trackedIds.back();
                float minVal = 1e6;
                for(s=currentId.first+1; s<=currentId.first+5 && s+1<sectors.size(); s++){
                    for(t=0; t<sectors[s].mergedLineSections.size(); t++){
                        if(sectors[s].mergedLineSections[t].flag_tracked) continue;

                        float area = quadArea(sectors[currentId.first].mergedLineSections[currentId.second],
                                              sectors[s].mergedLineSections[t]);
                        if(area < minVal){
                            minVal = area;
                            mint = t;
                        }
                    }
                    if(minVal<5.f) break;
                }
                if(minVal<5.f){
                    trackedIds.push_back(std::make_pair(s,mint));
                }else{
                    break;
                }
            }while(1);

            for(auto id : trackedIds){
                sectors[id.first].mergedLineSections[id.second].flag_tracked = true;
            }

            if(trackedIds.size() < 5){
                continue;
            }

            Inference::Segment segment;
            float anchorGap = 5.f;
            std::vector<int> anchorIndices;
            for(auto id : trackedIds){
                std::cout<<"("<<id.first<<" "<<id.second<<") ";
                int prevNum = segment.cloud->size();
                for(auto pt : sectors[id.first].mergedLineSections[id.second].pts){
                    segment.cloud->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
                }
                int currNum = segment.cloud->size();
                anchorIndices.push_back(prevNum);
                anchorIndices.push_back(currNum-1);
            }
            if(anchorIndices.empty()) continue;
            segment.anchorIndices.push_back(anchorIndices.front());
            for(int k=1;k<anchorIndices.size(); k++){
                Eigen::Vector3f currPt = segment.cloud->at(anchorIndices[k]).getVector3fMap();
                int prevIndex = segment.anchorIndices.back();
                Eigen::Vector3f prevPt = segment.cloud->at(prevIndex).getVector3fMap();
                if((currPt-prevPt).norm() > anchorGap){
                    segment.anchorIndices.push_back(anchorIndices[k]);
                }
            }
            std::cout<<std::endl;
            segments.push_back(segment);
        }
    }

    std::vector<std::pair<int, float>> disp(segments.size());
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
        std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
    }
    std::sort(disp.begin(), disp.end(), [](std::pair<int, float> a, std::pair<int, float> b){
        return a.second > b.second;
    });


    nlohmann::json obj;
    obj["base"]["lat"] = init_UTM(0);
    obj["base"]["lon"] = init_UTM(1);
    obj["base"]["alt"] = init_UTM(2);
    for(int i=0; i<segments.size(); i++){
        int index = disp[i].first;
        /*
        if(segments[index].cloud !=NULL && segments[index].cloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[index].cloud);
        }
        */
        segments[index].QpSpline();
        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[index].smoothTrjCloud();
        if(trjCloud!=NULL && trjCloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
        }

        segments[index].addSegmentCoeff(obj, i);
        //segments[index].addSegmentCoeff_straightLine(obj, i);
    }

    std::ofstream fid_laneBoundary(outputDir+"/laneBoundary.json");
    //fid_laneBoundary << std::setw(4) << obj << std::endl;
    fid_laneBoundary<< obj << std::endl;
}

void PostProcessLaneBoundary::run_yxx(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir)
    {
#if 0
        //假设utm_zone = 50
        int utm_num = 50;
        std::string parse_json = dataDir + "/parse_json.json";

        //读取参数中的度带号，偏转点的utm坐标
        Eigen::Vector3d offset_utm;
        Utils::readParse(parse_json, utm_num, offset_utm);

        std::string lidarFile = pcdPath;
//        std::cout<<"lidarFile: "<<lidarFile<<std::endl;
        pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
        pcl::io::loadPCDFile(lidarFile, *pc_ptr);

        if(pc_ptr->empty())
            return;

        //输入第一个点作为偏转点
        MyColorPointType pcl_p = (*pc_ptr->begin());
        Eigen::Vector3d firstPt = Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z);
        Eigen::Vector3d init_llh = tools::Earth::UTM2llh(Eigen::Vector3d(firstPt(0), firstPt(1), 0.f),utm_num);

        std::cout<<"init_llh: "<<init_llh.transpose()<<std::endl;

        Eigen::Matrix3d Cen = tools::Earth::Pos2Cne(init_llh).transpose();
        Eigen::Vector3d init_ecef = tools::Earth::LLH2ECEF(init_llh);

        //输入车道线点
        //label： "line": 80, "arrow_id": 100, "road_id": 0
        pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegCloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (auto iter = pc_ptr->begin(); iter != pc_ptr->end(); iter++){
            MyColorPointType &pcl_p = (*iter);
            if(pcl_p.label == 80)
            {
                Eigen::Vector3d llh = tools::Earth::UTM2llh(Eigen::Vector3d(pcl_p.x, pcl_p.y, pcl_p.z),utm_num);
                Eigen::Vector3d pos = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
                laneSegCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        }
        pcl::io::savePCDFileBinary(outputDir+"/laneSegCloud.pcd", *laneSegCloud);

        //输入参考线
        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);

        Utils::readRefLines(parse_json, refLaneBoundaryCloud, init_llh);
        pcl::io::savePCDFileBinary(outputDir+"/refLaneBoundaryCloud.pcd", *refLaneBoundaryCloud);

        //计算方向向量
        Eigen::Vector3f refDirection = Eigen::Vector3f::Zero();
        for(int i=1; i<refLaneBoundaryCloud->size(); i++){
            refDirection += refLaneBoundaryCloud->at(i).getVector3fMap() - refLaneBoundaryCloud->at(i-1).getVector3fMap();
        }
        refDirection.normalize();
        Eigen::Vector3f refCentroid = refLaneBoundaryCloud->at(refLaneBoundaryCloud->size()/2).getVector3fMap();

        pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
        float minVal = 1e6, maxVal = -1e6;
        for(int i=0; i<laneSegCloud->size(); i++){
            float val = refDirection.dot(laneSegCloud->at(i).getVector3fMap()-refCentroid);
            if(val<minVal) minVal = val;
            if(val>maxVal) maxVal = val;
        }
        float resolution = 3.f;
        int num = (maxVal - minVal)/resolution + 1;
        for(int i=-num/2; i<=num/2; i++){
            Eigen::Vector3f pos = refCentroid + resolution*i*refDirection;
            refLaneAnchorCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
        }
        pcl::io::savePCDFileBinary(outputDir+"/refLaneAnchorCloud.pcd", *refLaneAnchorCloud);

        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
        kdtree_refLaneAnchor->setInputCloud(refLaneAnchorCloud); // 设置要搜索的点云，建立KDTree

        //聚类
        std::vector<std::vector<int>> clusters;
        float radius = 1.f;
        int minPts = 20;
        Inference::DBSCAN(laneSegCloud, radius, minPts, clusters);
        std::cout<<"lane clustered number: "<<clusters.size()<<std::endl;

        pcl::PointCloud<pcl::PointXYZI>::Ptr clustersCloud(new pcl::PointCloud<pcl::PointXYZI>);
        std::map<int, int> classIdsMap;
        for (int i = 0; i < clusters.size(); ++i) {
            if(clusters[i].size() < 100) //按照1米20个点 要求长度最少5米
                continue;
            for(int j = 0; j < clusters[i].size();j++)
            {
                classIdsMap.insert(std::make_pair(clusters[i][j],i));
                pcl::PointXYZ curPXYZ = laneSegCloud->at(clusters[i][j]);
                clustersCloud->push_back(pcl::PointXYZI(curPXYZ.x, curPXYZ.y, curPXYZ.z, i));
            }
        }
        pcl::io::savePCDFileBinary(outputDir+"/clustersCloud.pcd", *clustersCloud);

        std::vector<Sector> sectors(refLaneAnchorCloud->size());
        for(int i=0; i<laneSegCloud->size(); i++){
            pcl::PointXYZ pt = laneSegCloud->at(i);
            int K = 1;
            std::vector<int> Idx;
            std::vector<float> SquaredDistance;
            kdtree_refLaneAnchor->nearestKSearch(pt, K, Idx, SquaredDistance);
            if(Idx.empty()) continue;
            int index = Idx.front();
            int classId = classIdsMap[i];
            sectors[index].lineSections[classId].pts.push_back(pt.getVector3fMap());
        }
        //estimate lineSections
        for(int i=1; i+1<sectors.size(); i++){
            auto &lineSections = sectors[i].lineSections;
            std::vector<int> eraseIds;
            for(auto iter=lineSections.begin(); iter!=lineSections.end(); iter++){
                int classId = iter->first;
                Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
                if(!estimateDirection(iter->second, classId, sectors[i-1], sectors[i+1], refDirection)){
                    eraseIds.push_back(classId);
                }
            }
            for(auto classId : eraseIds){
                lineSections.erase(classId);
                std::cout<<i<<" th sector erased "<<classId<<" LineSection"<<std::endl;
            }
        }
        /*
        for(int i=0; i<sectors.size(); i++){
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for(auto iter = sectors[i].lineSections.begin(); iter!=sectors[i].lineSections.end(); iter++){
                LineSection &lineSection = iter->second;
                for(auto pt : lineSection.pts){
                    pcl::PointXYZ point(pt(0), pt(1), pt(2));
                    cloud->push_back(point);
                }
            }
            pcl::io::savePCDFileBinary(outputDir+"/sector_"+std::to_string(i)+".pcd", *cloud);
        }
        */
        for(int i=1; i+1<sectors.size(); i++){
            Eigen::Vector3f refDirection = refLaneAnchorCloud->at(i).getVector3fMap() - refLaneAnchorCloud->at(i-1).getVector3fMap();
            MergeLineSection(sectors[i], refDirection);
        }

        std::vector<Inference::Segment> segments;
        for(int i=1; i+1<sectors.size(); i++){
            for(int j=0; j<sectors[i].mergedLineSections.size(); j++){
                if(sectors[i].mergedLineSections[j].flag_tracked) continue;
                std::vector<std::pair<int,int>> trackedIds;
                trackedIds.push_back(std::make_pair(i,j));
                int s, t, mint=-1;

                do{
                    std::pair<int, int> currentId = trackedIds.back();
                    float minVal = 1e6;
                    for(s=currentId.first+1; s<=currentId.first+5 && s+1<sectors.size(); s++){
                        for(t=0; t<sectors[s].mergedLineSections.size(); t++){
                            if(sectors[s].mergedLineSections[t].flag_tracked) continue;

                            float area = quadArea(sectors[currentId.first].mergedLineSections[currentId.second],
                                                  sectors[s].mergedLineSections[t]);
                            if(area < minVal){
                                minVal = area;
                                mint = t;
                            }
                        }
                        if(minVal<5.f) break;
                    }
                    if(minVal<5.f){
                        trackedIds.push_back(std::make_pair(s,mint));
                    }else{
                        break;
                    }
                }while(1);

                for(auto id : trackedIds){
                    sectors[id.first].mergedLineSections[id.second].flag_tracked = true;
                }

                if(trackedIds.size() < 5){
                    continue;
                }

                Inference::Segment segment;
                float anchorGap = 5.f;
                std::vector<int> anchorIndices;
                for(auto id : trackedIds){
                    std::cout<<"("<<id.first<<" "<<id.second<<") ";
                    int prevNum = segment.cloud->size();
                    for(auto pt : sectors[id.first].mergedLineSections[id.second].pts){
                        segment.cloud->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));
                    }
                    int currNum = segment.cloud->size();
                    anchorIndices.push_back(prevNum);
                    anchorIndices.push_back(currNum-1);
                }
                if(anchorIndices.empty()) continue;
                segment.anchorIndices.push_back(anchorIndices.front());
                for(int k=1;k<anchorIndices.size(); k++){
                    Eigen::Vector3f currPt = segment.cloud->at(anchorIndices[k]).getVector3fMap();
                    int prevIndex = segment.anchorIndices.back();
                    Eigen::Vector3f prevPt = segment.cloud->at(prevIndex).getVector3fMap();
                    if((currPt-prevPt).norm() > anchorGap){
                        segment.anchorIndices.push_back(anchorIndices[k]);
                    }
                }
                std::cout<<std::endl;
                segments.push_back(segment);
            }
        }

        std::vector<std::pair<int, float>> disp(segments.size());
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
            std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;
        }
        std::sort(disp.begin(), disp.end(), [](std::pair<int, float> a, std::pair<int, float> b){
            return a.second > b.second;
        });


        nlohmann::json obj;
        obj["base"]["lat"] = init_llh(0);
        obj["base"]["lon"] = init_llh(1);
        obj["base"]["alt"] = init_llh(2);
        for(int i=0; i<segments.size(); i++){
            int index = disp[i].first;
            /*
            if(segments[index].cloud !=NULL && segments[index].cloud->size()>0){
                pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[index].cloud);
            }
            */
            segments[index].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[index].smoothTrjCloud();
            if(trjCloud!=NULL && trjCloud->size()>0){
                pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
            }

            segments[index].addSegmentCoeff(obj, i);
            //segments[index].addSegmentCoeff_straightLine(obj, i);
        }

        std::ofstream fid_laneBoundary(outputDir+"/laneBoundary.json");
        //fid_laneBoundary << std::setw(4) << obj << std::endl;
        fid_laneBoundary<< obj << std::endl;
#endif
    }

void PostProcessLaneBoundary::run_yxx_2(std::string dataDir, std::string pcdPath, std::string refFile,std::string outputDir,std::string label) {
    //假设utm_zone = 50
    int utm_num = 50;
    std::string parse_json = dataDir + "/parse_json.json";

    //读取参数中的度带号，偏转点的utm坐标
    Eigen::Vector3d offset_utm;
    Utils::readParse(parse_json, utm_num, offset_utm);

    std::string lidarFile = pcdPath;
    std::cout << "lidarFile: " << lidarFile << std::endl;
    pcl::PointCloud<MyColorPointType>::Ptr pc_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::io::loadPCDFile(lidarFile, *pc_ptr);

    if (pc_ptr->empty())
        return;

    //输入车道线点
    //label： "line": 80, "arrow_id": 100, "road_id": 0
    pcl::PointCloud<MyColorPointType>::Ptr linecloudfilter(new pcl::PointCloud<MyColorPointType>);
    if (label == "label")
    {
        pclFilter::ConditionalRemoval(pc_ptr, linecloudfilter, label, 80);
        process_lane(outputDir, parse_json, "trjCloud", linecloudfilter);
    }
    else if (label == "cloud_line_seg")
    {
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter2(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter2, label, 2);
        if (!cloudfilter2->empty() && cloudfilter2->points.size() > 0)
        {
            *linecloudfilter += *cloudfilter2;
        }
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter3(new pcl::PointCloud<MyColorPointType>);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter3, label, 3);
        if (!cloudfilter3->empty() && cloudfilter3->points.size() > 0)
        {
            *linecloudfilter += *cloudfilter3;
        }
        process_lane(outputDir, parse_json, "trjCloud", linecloudfilter);
    }
    else if (label == "cloud_pano_seg")
    {
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter2(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter2, label, 15);
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter2, label, 1);
        if (!cloudfilter2->empty() && cloudfilter2->points.size() > 0)
        {
            *linecloudfilter += *cloudfilter2;
        }
        process_lane(outputDir, parse_json, "trjCloud", linecloudfilter);
    }
    else if (label == "cloud_bev_label")
    {
        //坤哥37类别转换映射
        // if (type >= 1 && type <= 13)
        // { // 白色线类
        //     return 1;
        // }
        // if (type >= 14 && type <= 20)
        // { // 黄色线类
        //     return 2;
        // }
        // if (type >= 21 && type <= 27)
        // { // 道路边界类
        //     return 3;
        // }
        // if (type == 28)
        // { // 停止线
        //     return 4;
        // }
        // if (type == 29)
        // { // 人行道
        //     return 5;
        // }
        /////**************************新cloud_bev_label***************************//////
        // int label_1 = 0;
        // {
        //     // clang-format off
        //     if (raw_id_37 >= 1 && raw_id_37 <= 20) label_1 = 1; // 车道线
        //     else if (raw_id_37 == 28)              label_1 = 2; // stopline
        //     else if (raw_id_37 == 29)              label_1 = 3; // crosswalk
        //     // clang-format on
        // }
        // int label_2 = 0;
        // {
        //     // clang-format off
        //     if (raw_id_37 >= 21 && raw_id_37 <= 27) label_2 = 1; // road_boundary
        //     else if (raw_id_37 == 36)               label_2 = 2; // lane_center
        //     // clang-format on
        // }
        // int color = 0;
        // {
        //     // clang-format off
        //     if  (raw_id_37 >= 7 && raw_id_37 <= 13)      color = 1; // white
        //     else if (raw_id_37 >= 14 && raw_id_37 <= 20) color = 2; // yellow
        //     // clang-format on
        // }
        // int shape = 0;
        // {
        //     // clang-format off
        //          if (raw_id_37 ==1  || raw_id_37 == 8  || raw_id_37 == 15) shape = 1; // solid
        //     else if (raw_id_37 ==2  || raw_id_37 == 9  || raw_id_37 == 16) shape = 2; // dashed
        //     else if (raw_id_37 ==3  || raw_id_37 == 10 || raw_id_37 == 17) shape = 3; // double_solid
        //     else if (raw_id_37 ==4  || raw_id_37 == 11 || raw_id_37 == 18) shape = 4; // double_dashed
        //     else if (raw_id_37 ==5  || raw_id_37 == 12 || raw_id_37 == 19) shape = 5; // left_dashed_right_solid
        //     else if (raw_id_37 ==6  || raw_id_37 == 13 || raw_id_37 == 20) shape = 6; // left_solid_right_dashed
        //     // clang-format on
        // }

        /////////////////////////////////根据cloud_bev_label字段提取///////////////////////////////////
        pcl::PointCloud<MyColorPointType>::Ptr cloudfilter2(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter2, label, 1); 
        pclFilter::ConditionalRemoval(pc_ptr, cloudfilter2, "cloud_bev_label_1", 1); 
        if (!cloudfilter2->empty() && cloudfilter2->points.size() > 0)
        {
            *linecloudfilter += *cloudfilter2;
        }
        // pcl::PointCloud<MyColorPointType>::Ptr cloudfilter3(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval(pc_ptr, cloudfilter3, label, 2);
        // if (!cloudfilter3->empty() && cloudfilter3->points.size() > 0)
        // {
        //     *linecloudfilter += *cloudfilter3;
        // }

        /////////////////////////////////根据cloud_bev_shape字段提取///////////////////////////////////
        // pcl::PointCloud<MyColorPointType>::Ptr cloudfilter_shape(new pcl::PointCloud<MyColorPointType>);
        // pclFilter::ConditionalRemoval_GE_LE(pc_ptr, cloudfilter_shape, "cloud_bev_shape", 1, 7); 
        // if (!cloudfilter_shape->empty() && cloudfilter_shape->points.size() > 0)
        // {
        //     *linecloudfilter += *cloudfilter_shape;
        // }
        process_lane(outputDir, parse_json, "trjCloud", linecloudfilter);
    }
}

void PostProcessLaneBoundary::process_lane(std::string outputDir,std::string parse_json,std::string outnamechar,pcl::PointCloud<MyColorPointType>::Ptr &linecloudfilter)
{
    if (linecloudfilter->empty() || linecloudfilter->points.empty())
    {
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr linecloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (auto iter = linecloudfilter->begin(); iter != linecloudfilter->end(); iter++) {
        MyColorPointType &pcl_p = (*iter);
        linecloud->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    }

    //对原始车道线进行统计滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr laneSegCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pclFilter::StatisticalOutlierRemoval(linecloud, laneSegCloud, 50, 1.0f);
    if(laneSegCloud->empty() || laneSegCloud->points.empty())
    {
        return;
    }
    pcl::io::savePCDFileBinary(outputDir + "/laneSegCloud_Statistical.pcd", *laneSegCloud);

    // //读入感知点云,并将Z值刷新
    // pcl::KdTreeFLANN<MyColorPointType>::Ptr kdtree_cloud(new pcl::KdTreeFLANN<MyColorPointType>);
    // kdtree_cloud->setInputCloud(pc_ptr); // 设置要搜索的点云，建立KDTree

    // std::string pp_file = dataDir + "/pp_transCloud.pcd";
    // std::ifstream infile(pp_file);

    // pcl::PointCloud<MyColorPointType>::Ptr pp_ptr(new pcl::PointCloud<MyColorPointType>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr pp_ptr_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    // if (infile.good()) {
    //      pcl::io::loadPCDFile(pp_file, *pp_ptr);

    //      for (auto iter = pp_ptr->begin(); iter != pp_ptr->end(); iter++) {
    //          MyColorPointType &pcl_p = (*iter);
    //          int K = 1;
    //          std::vector<int> Idx;
    //          std::vector<float> SquaredDistance;
    //          kdtree_cloud->nearestKSearch(pcl_p, K, Idx, SquaredDistance);
    //          if (Idx.empty())
    //              continue;
    //          pcl_p.z = pc_ptr->at(Idx[0]).z;

    //          pp_ptr_xyz->push_back(pcl::PointXYZ(pcl_p.x, pcl_p.y, pcl_p.z));
    //      }
    // //     pcl::io::savePCDFileBinary(pp_file, *pp_ptr);

    //     //将点云中无结果的感知数据加入
    //     pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_cloud_lane(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    //     kdtree_cloud_lane->setInputCloud(laneSegCloud); // 设置要搜索的点云，建立KDTree
    //     for (auto iter = pp_ptr_xyz->begin(); iter != pp_ptr_xyz->end(); iter++) {
    //         pcl::PointXYZ &pcl_p = (*iter);
    //         double R = 2.0;
    //         std::vector<int> Idx;
    //         std::vector<float> SquaredDistance;
    //         kdtree_cloud_lane->radiusSearch(pcl_p, R, Idx, SquaredDistance);
    //         if (Idx.size() < 3)
    //             laneSegCloud->push_back(pcl_p);
    //     }

//        pcl::io::savePCDFileBinary(outputDir+"/pp_StatisticalOutlierRemoval.pcd", *pp_ptr_filter);
        //感知结果加入车道线建模
//        *laneSegCloud += *pp_ptr_xyz;
    // }
    if(laneSegCloud->empty() || laneSegCloud->points.empty())
    {
        return;
    }
    pcl::io::savePCDFileBinary(outputDir + "/laneSegCloud.pcd", *laneSegCloud);

    //获取参考线相关信息
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneBoundaryCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr refLaneAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refLaneAnchor(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    get_ref_info(parse_json, outputDir, laneSegCloud, refLaneBoundaryCloud, refLaneAnchorCloud, kdtree_refLaneAnchor);

    //获取提取线信息
    lane_extract_base(outputDir, outnamechar, laneSegCloud, refLaneBoundaryCloud, refLaneAnchorCloud,
                      kdtree_refLaneAnchor);
}
} //namespace RoadMapping