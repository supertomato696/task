#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include "Utils.h"
#include "json.hpp"
#include "earth.hpp"
#include "cluster.h"
#include <sys/stat.h>
#include <filesystem>
#include "processBoundary.h"
#include "processTrafficArrow.h"
#include "processRoadMark.h"

int main(int argc, char *argv[])
{
    //read keyPoses
    std::string poseFile = argv[1];
    std::vector<KeyPose> keyPoses;
    Utils::readKeyPose(poseFile, keyPoses);
    std::cout<<"keyPoses size: "<<keyPoses.size()<<std::endl;

    //read perception result
    std::string perceptionFile = argv[2];
    std::ifstream ifs_perception(perceptionFile);
    nlohmann::json perceptionResult;
    perceptionResult<<ifs_perception;

    //reconstruction directory
    std::string inputDir = argv[3];
    std::string outputDir = argv[4];

    //generate rough lane pointCloud
    Eigen::Vector3d init_llh(keyPoses[0].lat, keyPoses[0].lon, keyPoses[0].alt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_laneBoundary(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roadBoundary(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto &keyPose : keyPoses){
        std::string frame_id_Str = std::to_string(keyPose.frame_id);
        if(!perceptionResult.contains(frame_id_Str)){
            continue;
        }
        nlohmann::json linesObj = perceptionResult[frame_id_Str]["lane_data"][0]["lines"];
        
        Eigen::Vector3d llh(keyPose.lat, keyPose.lon, keyPose.alt);
        Eigen::Vector3d position = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
        keyPoseCloud->push_back(pcl::PointXYZ(position(0), position(1), position(2)));
        Eigen::Matrix3d R_enu_vel = keyPose.R;
        //static std::ofstream fid_pos("pos.txt");
        //fid_pos<<std::setprecision(13)<<keyPose.timestamp<<" "<<position.transpose()<<std::endl;
        for(int i=0; i<linesObj.size(); i++){
            std::string line_type = linesObj[i]["line_type"];

            Eigen::Vector3d endPoint_0, endPoint_1;
            nlohmann::json endPointObj = linesObj[i]["end_point"];
            endPoint_0<<endPointObj[1][0], endPointObj[1][1], endPointObj[1][2];
            endPoint_1<<endPointObj[0][0], endPointObj[0][1], endPointObj[0][2];
            nlohmann::json coeffsObj = linesObj[i]["coeffs"];
            Eigen::Vector4d coeffs;
            coeffs<<coeffsObj["0"], coeffsObj["1"], coeffsObj["2"], coeffsObj["3"];

            for(int step = 0; step<std::min((double)endPoint_1(0), (double)20.f)/0.2; step++){
                float x = 0.1*step, x2 = x*x, x3=x*x2;
                float y = coeffs(0) + coeffs(1)*x + coeffs(2)*x2 + coeffs(3)*x3;
                float z = endPoint_1(2);
                Eigen::Vector3d pos = position + R_enu_vel*Eigen::Vector3d(x,y,z);
                if(line_type == "FENCE"){
                    cloud_roadBoundary->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
                }else{
                    cloud_laneBoundary->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
                }                
            }
        }        
    }
    
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_keyPose(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_keyPose->setInputCloud(keyPoseCloud); // 设置要搜索的点云，建立KDTree    
    pcl::io::savePCDFileBinary(outputDir+"/keyPoseCloud.pcd", *keyPoseCloud);

    RoadMapping::ProcessBoundary::run(
        init_llh, cloud_laneBoundary, keyPoseCloud, kdtree_keyPose, outputDir+"/laneBoundary"
    );
    RoadMapping::ProcessBoundary::run(
        init_llh, cloud_roadBoundary, keyPoseCloud, kdtree_keyPose, outputDir+"/roadBoundary"
    );

    //get longest laneBoundary
    
    std::string laneBoundaryDir = outputDir+"/laneBoundary";
    std::vector<std::string> trjCloudFiles;
    Utils::getFiles(laneBoundaryDir, trjCloudFiles, [](std::string str)->bool{
        return str.find("trjCloud.pcd")!=std::string::npos;
    });
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr maintrjCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto trjCloudFile : trjCloudFiles){
        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(laneBoundaryDir+"/"+trjCloudFile, *trjCloud);
        if(trjCloud->size() > maintrjCloud->size()){
            maintrjCloud = trjCloud;
        }
    }
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_maintrjCloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_maintrjCloud->setInputCloud(maintrjCloud); // 设置要搜索的点云，建立KDTree
    

    RoadMapping::ProcessTrafficArrow::run(
        init_llh, keyPoses, maintrjCloud, kdtree_maintrjCloud, inputDir, outputDir+"/trafficArrow"
    );

    RoadMapping::ProcessRoadMark::run(
        init_llh, keyPoses, maintrjCloud, kdtree_maintrjCloud, inputDir, outputDir+"/roadMark"
    );

    std::cout<<"successfully run roadModel"<<std::endl;
}