#include <iostream>
#include <map>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Utils.h"
#include "json.hpp"
#include <Eigen/Geometry>
#include "earth.hpp"

int main(int argc, char *argv[])
{
    
    //read keyPoses
    std::string poseFile = argv[1];
    std::vector<KeyPose> keyPoses;
    Utils::readKeyPose(poseFile, keyPoses);
    std::cout<<"keyPoses size: "<<keyPoses.size()<<std::endl;
    Eigen::Vector3d init_llh(keyPoses[0].lat, keyPoses[0].lon, keyPoses[0].alt);
    

    std::string dataSetJsonFile = argv[2];
    std::ifstream ifs_dataSet(dataSetJsonFile);
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
    

    std::string imageSegDir = argv[3];
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

    for(int index=0; index<keyPoses.size(); index++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
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
                if(pos_veh.x() > 1.f && pos_veh.x() < 20.f && pos_veh.y() < 10.f && pos_veh.y() > -10.f){
                    Eigen::Vector3d pos = position + R_enu_vel*pos_veh;
                    cloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
                }
            }
        }
        if(cloud->size() > 0){
            pcl::io::savePCDFileBinary(frameIdStr+".pcd", *cloud);
        }
    }
    


    std::cout<<"successfully run trafficArrowTest"<<std::endl;
}