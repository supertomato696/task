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
#include "cluster.h"
#include "convexHull.h"

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
    std::string imageDir = argv[4];
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
        /*
        std::vector<std::vector<std::pair<int,int>>> clusters;
        Inference::BFS(img, clusters, 110);
        cv::Mat img_show = cv::imread(imageDir+"/"+frameIdStr+".jpg");
        for(int i=0; i<clusters.size(); i++){
            std::vector<Eigen::Vector2f> pts(clusters[i].size());
            std::vector<int> nextVertex;
            for(int j=0; j<clusters[i].size(); j++){
                pts[j] = Eigen::Vector2f(clusters[i][j].first, clusters[i][j].second);
            }
            CH::QuickHull(pts, nextVertex);
            for(int j=0; j+1<nextVertex.size(); j++){
                Eigen::Vector2f p0 = pts[nextVertex[j]], p1 = pts[nextVertex[j+1]];
                cv::Point2f pt_0(p0(1), p0(0));
                cv::Point2f pt_1(p1(1), p1(0));
                cv::line(img_show, pt_0, pt_1, cv::Scalar(0,0,255), 1);
            }            

        }       
        cv::imshow("img.jpg", img_show);
        cv::waitKey(0);  
        */
        cv::Mat img_show = cv::imread(imageDir+"/"+frameIdStr+".jpg");
        for(int i=0; i<img.rows; i++){
            for(int j=0; j<img.cols; j++){
                if(img.at<uchar>(i,j) == 100){
                    img_show.at<cv::Vec3b>(i,j) = cv::Vec3b(0,0,255);
                }
                if(img.at<uchar>(i,j) == 110){
                    img_show.at<cv::Vec3b>(i,j) = cv::Vec3b(0,255,0);
                }
            }
        }  
        cv::imshow("img.jpg", img_show);
        cv::waitKey(0);   
    }
    


    std::cout<<"successfully run roadMarkTest"<<std::endl;
}