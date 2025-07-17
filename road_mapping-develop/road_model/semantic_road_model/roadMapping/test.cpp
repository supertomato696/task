#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "basicStruct.h"
#include "Utils.h"
#include "units.hpp"
#include "json.hpp"
#include "earth.hpp"


int main()
{
    std::vector<KeyPose> keyPoses;
    std::string poseFile = "/home/shenxuefeng/Documents/roadMapping/data2/1650244207929.gps";
    Utils::readKeyPose(poseFile, keyPoses);
    std::cout<<"keyPoses size: "<<keyPoses.size()<<std::endl;

    std::ifstream ifs_perception("/home/shenxuefeng/Documents/roadMapping/data2/1650244207929.perception");
    nlohmann::json perceptionResult;
    perceptionResult<<ifs_perception;
    Eigen::Vector3d init_llh(keyPoses[0].lat, keyPoses[0].lon, keyPoses[0].alt);
    pcl::PointCloud<pcl::PointXYZ>::Ptr laneCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto &keyPose : keyPoses){
        std::string frame_id_Str = std::to_string(keyPose.frame_id);
        if(!perceptionResult.contains(frame_id_Str)){
            continue;
        }
        nlohmann::json linesObj = perceptionResult[frame_id_Str]["lane_data"][0]["lines"];
        //cv::Mat img(1000, 200, CV_8UC1, cv::Scalar(0));
        
        Eigen::Vector3d llh(keyPose.lat, keyPose.lon, keyPose.alt);
        Eigen::Vector3d position = -tools::Earth::DeltaPosEnuInFirstPoint(init_llh, llh);
        Eigen::Matrix3d R_enu_vel = keyPose.R;
        static std::ofstream fid_pos("pos.txt");
        fid_pos<<std::setprecision(13)<<keyPose.timestamp<<" "<<position.transpose()<<std::endl;
        for(int i=0; i<linesObj.size(); i++){
            std::string line_type = linesObj[i]["line_type"];
            Eigen::Vector3d endPoint_0, endPoint_1;
            nlohmann::json endPointObj = linesObj[i]["end_point"];
            endPoint_0<<endPointObj[1][0], endPointObj[1][1], endPointObj[1][2];
            endPoint_1<<endPointObj[0][0], endPointObj[0][1], endPointObj[0][2];
            nlohmann::json coeffsObj = linesObj[i]["coeffs"];
            Eigen::Vector4d coeffs;
            coeffs<<coeffsObj["0"], coeffsObj["1"], coeffsObj["2"], coeffsObj["3"];
            //std::cout<<frame_id_Str<<" "<<coeffs.transpose()<<std::endl;
            for(int step = 0; step<std::min((double)endPoint_1(0), (double)20.f)/0.1; step++){
                float x = 0.1*step, x2 = x*x, x3=x*x2;
                float y = coeffs(0) + coeffs(1)*x + coeffs(2)*x2 + coeffs(3)*x3;
                float z = endPoint_1(2);
                int xIndex = x/0.1, yIndex = y/0.1+100;
                //img.at<u_char>(xIndex, yIndex) = 255;
                Eigen::Vector3d pos = position + R_enu_vel*Eigen::Vector3d(x,y,z);
                laneCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }
        }
        //cv::imwrite(frame_id_Str+".jpg", img);
        
    }
    pcl::io::savePCDFileBinary("all.pcd", *laneCloud);

    std::cout<<"successfully run the program"<<std::endl;
}