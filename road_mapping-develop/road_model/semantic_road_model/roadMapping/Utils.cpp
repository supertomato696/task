#include "Utils.h"
#include "units.hpp"
#include "earth.hpp"
#include <dirent.h>
#include "json.hpp"
#include <boost/geometry.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/operations.hpp>

namespace Utils{

void split (std::string s, std::string delimiter, std::vector<std::string> &vec) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    while ((pos_end = s.find (delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        vec.push_back (token);
    }
}

void readKeyPose(std::string file, std::vector<KeyPose> &keyPoses)
{
    std::ifstream ifs(file);
    if(!ifs.is_open()){
        ifs.close();
        return;
    }
    std::string line;
    //skip the first 4 lines, non data messages
    for(int i=0;i<4;i++){
        getline(ifs, line);
    }
    std::string delimiter = " ";
    while(getline(ifs, line)){
        std::vector<std::string> vec;
        Utils::split(line, delimiter, vec);
        if(vec.size()<5){
            continue;
        }
        KeyPose keyPose;
        keyPose.timestamp = std::stod(vec[0])/1e3;
        keyPose.frame_id = std::stoi(vec[1]);
        keyPose.frameIdStr = vec[0]+"_"+vec[1];
        keyPose.lat = std::stod(vec[2])*tools::gl_deg;
        keyPose.lon = std::stod(vec[3])*tools::gl_deg;
        keyPose.alt = std::stod(vec[4]);
        keyPose.R<<std::stod(vec[5]), std::stod(vec[6]), std::stod(vec[7]),
                   std::stod(vec[8]), std::stod(vec[9]), std::stod(vec[10]),
                   std::stod(vec[11]),std::stod(vec[12]),std::stod(vec[13]); 
        keyPoses.push_back(keyPose);
    }
    ifs.close();
}

void readRefLines(const std::string dirPath, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, Eigen::Vector3d init_llh)
{
    std::cout<<"parse_josn: "<<dirPath<<std::endl;
    std::ifstream ifs_parseInfo(dirPath);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::string center_line = parseInfo["link_geom"];
    int utm_num = parseInfo["utm_num"];

    GisPolyline new_line;
    boost::geometry::read_wkt(center_line, new_line);

    std::vector<double> p_utm_world = parseInfo["t_utm_world"];
    Eigen::Vector3d t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

    for (const auto &pt : new_line){
        double x = pt.get<0>() * tools::gl_deg;
        double y = pt.get<1>() * tools::gl_deg;
        double z = pt.get<2>();

        Eigen::Vector3d llh(x,y,z);
//        Eigen::Vector3d llh(y,x,z);
//        //转换为UTM坐标
//        Eigen::Vector3d xyz = tools::Earth::llh2UTM(llh, utm_num);
//        Eigen::Vector3d xyz_offset = xyz - t_utm_world;
//        llh = tools::Earth::UTM2llh(xyz_offset, utm_num);
        Eigen::Vector3d utm = tools::Earth::llh2UTM(llh, utm_num, false);
        Eigen::Vector3d pos = utm - init_llh;
        refLaneBoundaryCloud->push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
    }
}

void readRefLines_2(const std::string dirPath, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud)
{
    std::cout<<"parse_josn: "<<dirPath<<std::endl;
    std::ifstream ifs_parseInfo(dirPath);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::string center_line = parseInfo["link_geom"];
    int utm_num = parseInfo["utm_num"];

    GisPolyline new_line;
    boost::geometry::read_wkt(center_line, new_line);

    std::vector<double> p_utm_world = parseInfo["t_utm_world"];
    Eigen::Vector3d t_utm_world(p_utm_world[0], p_utm_world[1], p_utm_world[2]);

    for (const auto &pt : new_line){
        double x = pt.get<0>() * tools::gl_deg;
        double y = pt.get<1>() * tools::gl_deg;
        double z = pt.get<2>();

        Eigen::Vector3d llh(x,y,z);

        //转换为UTM坐标
        Eigen::Vector3d xyz = tools::Earth::llh2UTM(llh, utm_num);
        Eigen::Vector3d xyz_offset = xyz - t_utm_world;
        refLaneBoundaryCloud->push_back(pcl::PointXYZ(xyz_offset(0), xyz_offset(1), xyz_offset(2)));
    }
}
void readParse(const std::string parse_josn, int &utm_num, Eigen::Vector3d &t_utm_world)
{
    std::ifstream ifs_parseInfo(parse_josn);
    if(!ifs_parseInfo.is_open()){
        ifs_parseInfo.close();
        return;
    }

    nlohmann::json parseInfo;
    parseInfo<<ifs_parseInfo;
    std::vector<double> p_utm_world = parseInfo["t_utm_world"];
    utm_num = parseInfo["utm_num"];
    t_utm_world = Eigen::Vector3d(p_utm_world[0], p_utm_world[1], p_utm_world[2]);
}

void getFiles(const std::string dirPath, std::vector<std::string> &files, std::string keyWord){
    struct dirent *entry;
    DIR *dir = opendir(dirPath.c_str());
    if (dir == NULL) {
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        std::string fileName = entry->d_name;
        if(fileName.find(keyWord) == std::string::npos){
            continue;
        }
        files.push_back(fileName);
    }
    std::sort(files.begin(), files.end());
    closedir(dir);    
}

void getFiles(const std::string dirPath, std::vector<std::string> &files, pFunc Func)
{
    struct dirent *entry;
    DIR *dir = opendir(dirPath.c_str());
    if (dir == NULL) {
        return;
    }

    while ((entry = readdir(dir)) != NULL) {
        std::string fileName = entry->d_name;
        if(Func(fileName)){
            files.push_back(fileName);
        }
    }
    std::sort(files.begin(), files.end());
    closedir(dir);     
}


} //namespace Utils