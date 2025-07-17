#include <iostream>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "Utils.h"
#include "json.hpp"
#include "earth.hpp"

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string outputDir = argv[2];
    std::vector<std::string> files;
    Utils::getFiles(dataDir, files, [](std::string str)->bool{
        if(str.size()!=13){
            return false;
        }
        for(int i=0; i<13; i++){
            if(str.at(i)<'0' || str.at(i)>'9'){
                return false;
            }
        }
        return true;
    });
    bool setGlobalRef = false;
    Eigen::Vector3d GlobalRef = Eigen::Vector3d::Zero();
    for(auto file : files){
        std::cout<<"currently deal with: "<<file<<std::endl;
        //transform laneBoundary cloud
        std::string laneBoundaryDir = dataDir+"/"+file+"/roadMappingResult/laneBoundary";
        std::ifstream ifs(laneBoundaryDir + "/boundary.json");
        if(!ifs.is_open()){
            continue;
        }
        nlohmann::json boundaryJson;
        boundaryJson<<ifs;
        ifs.close();
        Eigen::Vector3d ref;
        ref<<boundaryJson["base"]["lat"], boundaryJson["base"]["lon"], boundaryJson["base"]["alt"];
        if(!setGlobalRef){
            GlobalRef = ref;
            setGlobalRef = true;
        }
        std::vector<std::string> trjCloudFiles;
        Utils::getFiles(laneBoundaryDir, trjCloudFiles, [](std::string str)->bool{
            return str.find("trjCloud.pcd")!=std::string::npos;
        });
        
        Eigen::Vector3f position = -tools::Earth::DeltaPosEnuInFirstPoint(GlobalRef, ref).cast<float>();
        for(auto trjCloudFile : trjCloudFiles){
            std::cout<<"deal with trjCloud file: "<<trjCloudFile<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(laneBoundaryDir+"/"+trjCloudFile, *trjCloud);
            for(int i=0; i<trjCloud->size(); i++){
                trjCloud->at(i).getVector3fMap() += position;
            }
            int pos = trjCloudFile.find("_");
            std::string outputFileName = file+"_"+trjCloudFile.substr(0,pos);
            pcl::io::savePCDFile(outputDir+"/"+outputFileName+"_trjCloud.pcd", *trjCloud);
        }
        //transform trafficArrow cloud
        std::string trafficArrowDir = dataDir+"/"+file+"/roadMappingResult/trafficArrow";
        pcl::PointCloud<pcl::PointXYZ>::Ptr trafficArrow_totalCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::io::loadPCDFile(trafficArrowDir+"/totalCloud.pcd", *trafficArrow_totalCloud);
        for(int i=0; i<trafficArrow_totalCloud->size(); i++){
            trafficArrow_totalCloud->at(i).getVector3fMap() += position;
        }
        pcl::io::savePCDFile(outputDir+"/"+file+"_trafficArrow_totalCloud.pcd", *trafficArrow_totalCloud);
        
        std::vector<std::string> trafficArrowBoxFiles;
        Utils::getFiles(trafficArrowDir, trafficArrowBoxFiles, [](std::string str)->bool{
            return str.find("box.pcd")!=std::string::npos;
        });  
        for(auto trafficArrowBoxFile:trafficArrowBoxFiles){
            std::cout<<"deal with trafficArrowBoxFile: "<<trafficArrowBoxFile<<std::endl;
            pcl::PointCloud<pcl::PointXYZ>::Ptr trafficArrowBoxCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::io::loadPCDFile(trafficArrowDir+"/"+trafficArrowBoxFile, *trafficArrowBoxCloud);
            for(int i=0; i<trafficArrowBoxCloud->size(); i++){
                trafficArrowBoxCloud->at(i).getVector3fMap() += position;
            }
            int pos = trafficArrowBoxFile.find("_");
            std::string outputFileName = file+"_"+trafficArrowBoxFile.substr(0,pos);
            pcl::io::savePCDFile(outputDir+"/"+outputFileName+"_box.pcd", *trafficArrowBoxCloud);
        }      
    }

    std::cout<<"successfully run multipleLanesTest"<<std::endl;
}