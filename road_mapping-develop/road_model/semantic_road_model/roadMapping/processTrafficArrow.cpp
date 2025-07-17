#include "processTrafficArrow.h"
#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "json.hpp"
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>
#include "earth.hpp"
#include "cluster.h"

namespace RoadMapping{

void ProcessTrafficArrow::run(Eigen::Vector3d init_llh,
                              std::vector<KeyPose> &keyPoses,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                              pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_keyPose,
                              std::string inputDir, std::string outputDir)
{
    //generate trafficArrow cloud
    std::ifstream ifs_dataSet(inputDir+"/data_set.json");
    nlohmann::json dataSetObj;
    dataSetObj<<ifs_dataSet;
    ifs_dataSet.close();
    int width = dataSetObj["header"]["image_width"];
    int height = dataSetObj["header"]["image_height"];
    float fx = dataSetObj["header"]["cam_fx"];
    float fy = dataSetObj["header"]["cam_fy"];
    float cx = dataSetObj["header"]["cam_cx"];
    float cy = dataSetObj["header"]["cam_cy"];
    std::cout<<"image width: "<<width<<" image height: "<<height<<std::endl;  

    Eigen::Vector3d t_veh_cam;
    t_veh_cam<<dataSetObj["header"]["t_veh_cam"][0], 
               dataSetObj["header"]["t_veh_cam"][1],
               dataSetObj["header"]["t_veh_cam"][2];
    std::cout<<"t_veh_cam: "<<t_veh_cam.transpose()<<std::endl;
    Eigen::Quaterniond q_veh_cam;
    q_veh_cam.x() = dataSetObj["header"]["q_veh_cam"][0];
    q_veh_cam.y() = dataSetObj["header"]["q_veh_cam"][1];
    q_veh_cam.z() = dataSetObj["header"]["q_veh_cam"][2];
    q_veh_cam.w() = dataSetObj["header"]["q_veh_cam"][3];
    Eigen::Matrix3d R_veh_cam = q_veh_cam.toRotationMatrix();
    std::cout<<"R_veh_cam: \n"<<R_veh_cam<<std::endl;

    std::string imageSegDir = inputDir+"/image_seg";
    std::map<std::string, std::string> imageSegNames;
    std::vector<std::string> files;
    Utils::getFiles(imageSegDir, files, ".png");
    for(auto file : files){
        int pos = file.find(".");
        std::string frameIdStr = file.substr(0, pos);
        //std::cout<<"file: "<<file<<" frameIdStr: "<<frameIdStr<<std::endl;
        imageSegNames[frameIdStr] = file;
    }
    std::cout<<"imageSegNames size: "<<imageSegNames.size()<<std::endl;  

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(int index=0; index<keyPoses.size(); index++){
        auto keyPose = keyPoses[index];
        std::string frameIdStr = keyPose.frameIdStr;
        Eigen::Vector3d llh(keyPose.lat, keyPose.lon, keyPose.alt);
        Eigen::Vector3d position = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
        Eigen::Matrix3d R_enu_vel = keyPose.R;
        if(imageSegNames.find(frameIdStr)==imageSegNames.end()){
            std::cout<<"could not find frame_id: "<<frameIdStr<<std::endl;
            continue;
        }
        cv::Mat img = cv::imread(imageSegDir+"/"+imageSegNames[frameIdStr], cv::IMREAD_GRAYSCALE);
        for(int i=1; i+1<img.rows; i++){
            for(int j=1; j+1<img.cols; j++){
                if(img.at<uchar>(i,j) != 100){
                    continue;
                }
                int absDiff = std::abs((int)img.at<uchar>(i,j) - (int)img.at<uchar>(i+1,j))+
                              std::abs((int)img.at<uchar>(i,j) - (int)img.at<uchar>(i-1,j))+
                              std::abs((int)img.at<uchar>(i,j) - (int)img.at<uchar>(i,j+1))+
                              std::abs((int)img.at<uchar>(i,j) - (int)img.at<uchar>(i,j-1));
                //std::cout<<"index: "<<index<<" absDiff: "<<absDiff<<std::endl;
                if(absDiff == 0){
                    continue;
                }
                Eigen::Vector3d d_cam((j-cx)/fx, (i-cy)/fy, 1.f);
                Eigen::Vector3d d_veh = R_veh_cam*d_cam;
                double lambda = t_veh_cam(2)/d_veh(2);
                Eigen::Vector3d pos_veh = t_veh_cam - lambda*d_veh;
                if(pos_veh.x() > 3.f && pos_veh.x() < 15.f && pos_veh.y() < 6.5f && pos_veh.y() > -6.5f){
                    Eigen::Vector3d pos = position + R_enu_vel*pos_veh;
                    cloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
                }
            }
        }
    }    
    std::cout<<"traffic arrow total cloud size: "<<cloud->size()<<std::endl;
    if(cloud->size() > 0){
        pcl::io::savePCDFileBinary(outputDir+"/totalCloud.pcd", *cloud);
    }

    float radius = 0.5f;
    int minPts = 3;
    std::vector<std::vector<int>> clusters;
    Inference::DBSCAN(cloud, radius, minPts, clusters);
    std::cout<<"extract traffic arrows, clusters size: "<<clusters.size()<<std::endl;

    nlohmann::json obj;
    obj["base"]["lat"] = init_llh(0);
    obj["base"]["lon"] = init_llh(1);
    obj["base"]["alt"] = init_llh(2);

    for(int index=0; index<clusters.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, clusters[index], *clusterCloud);      
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_cluster.pcd", *clusterCloud); 
        if(clusterCloud->size() < 200){
            continue;
        }
        Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            centroid += clusterCloud->at(i).getVector3fMap();
        }
        centroid = centroid/clusterCloud->size();
        Eigen::Matrix3f cov = Eigen::Matrix3f::Zero();
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            cov += posDiff*posDiff.transpose();
        }
        cov = cov/clusterCloud->size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es;
        es.compute(cov);
        Eigen::Vector3f eigenvalues = es.eigenvalues();        
        std::cout<<index<<" th cluster, eigenvalues: "<<eigenvalues.transpose()<<std::endl;
        if(eigenvalues(0) > 1e-3 || eigenvalues(1) < 100*eigenvalues(0)){
            continue;
        }
        Eigen::Matrix3f U = es.eigenvectors();
        Eigen::Vector3f nz = U.col(0);
        if(nz.z() < 0.f) nz = -nz;
        std::cout<<"nz: "<<nz.transpose()<<std::endl;
        if(nz.z() < 0.7f){
            continue;
        }
        //determine heading direction
        std::vector<int> indices;
        std::vector<float> squaredDistances;
        pcl::PointXYZ searchPt(centroid.x(), centroid.y(), centroid.z());
        kdtree_keyPose->radiusSearch(searchPt, 20.f, indices, squaredDistances);
        std::cout<<"searched indices size: "<<indices.size()<<std::endl;
        if(indices.size() < 2){
            continue;
        }
        std::sort(indices.begin(), indices.end());
        Eigen::Vector3f direction = keyPoseCloud->at(indices.back()).getVector3fMap() -
                                        keyPoseCloud->at(indices.front()).getVector3fMap();
        std::cout<<"direction: "<<direction.transpose()<<std::endl;
        direction.normalize();
        Eigen::Vector3f nx = direction - direction.dot(nz)*nz;
        nx.normalize();
        Eigen::Vector3f ny = nz.cross(nx);
        float x_min = 1e6, x_max = -1e6, y_min = 1e6, y_max = -1e6;
        for(int i=0; i<clusterCloud->size(); i++){
            Eigen::Vector3f posDiff = clusterCloud->at(i).getVector3fMap() - centroid;
            float x = posDiff.dot(nx);
            if(x < x_min) x_min = x;
            if(x > x_max) x_max = x;
            float y = posDiff.dot(ny);
            if(y < y_min) y_min = y;
            if(y > y_max) y_max = y;            
        }
        std::cout<<"x length: "<<x_max - x_min<<" y length: "<<y_max - y_min<<std::endl;
        if(x_max - x_min > 12.f || y_max - y_min > 3.f || x_max - x_min < 5.f){
            continue;
        }

        Eigen::Vector3f LeftFront = centroid + x_max*nx + y_max*ny;
        Eigen::Vector3f RightFront = centroid + x_max*nx + y_min*ny;
        Eigen::Vector3f LeftRear = centroid + x_min*nx + y_max*ny;
        Eigen::Vector3f RightRear = centroid + x_min*nx + y_min*ny;
        pcl::PointCloud<pcl::PointXYZ>::Ptr boxCloud(new pcl::PointCloud<pcl::PointXYZ>);
        auto generateSegmentCloud = [&](Eigen::Vector3f p0, Eigen::Vector3f p1){
            Eigen::Vector3f dp = p1 - p0;
            float num = dp.norm()/0.1f;
            for(int i=0; i<num; i++){
                Eigen::Vector3f pos = p0 + (i/num)*dp;
                boxCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        };
        generateSegmentCloud(LeftFront, RightFront);
        generateSegmentCloud(RightFront, RightRear);
        generateSegmentCloud(RightRear, LeftRear);
        generateSegmentCloud(LeftRear, LeftFront);
        pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(index)+"_box.pcd", *boxCloud);

        nlohmann::json singleTrafficArrow;
        singleTrafficArrow["id"] = index;
        singleTrafficArrow["LeftFront"] = {LeftFront.x(), LeftFront.y(), LeftFront.z()};
        singleTrafficArrow["RightFront"] = {RightFront.x(), RightFront.y(), RightFront.z()};
        singleTrafficArrow["RightRear"] = {RightRear.x(), RightRear.y(), RightRear.z()};
        singleTrafficArrow["LeftRear"] = {LeftRear.x(), LeftRear.y(), LeftRear.z()};
        obj["trafficArrow"].push_back(singleTrafficArrow);
    }
    std::ofstream fid_trafficArrow(outputDir+"/trafficArrow.json");
    //fid_trafficArrow << std::setw(4) << obj << std::endl;
    fid_trafficArrow<< obj << std::endl;     
}

} //namespace RoadMapping