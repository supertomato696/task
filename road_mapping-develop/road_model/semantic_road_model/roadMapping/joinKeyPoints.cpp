//
//
//
#include "json.hpp"
#include "pclPtType.h"

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

static std::vector<std::string> get_files_name(const std::string &folder_path)
{
    std::vector<std::string> files_name;
    struct dirent *entry;
    DIR *dir = opendir(folder_path.c_str());
    if (dir == nullptr)
    {
        return files_name;
    }
    while ((entry = readdir(dir)) != nullptr)
    {
        files_name.emplace_back(entry->d_name);
    }
    closedir(dir);
    return files_name;
}

std::string path_join(std::string t)
{
    return t;
}

template <typename... Args>
std::string path_join(std::string t, Args... args)
{
    if (t[t.length() - 1] != '/')
    {
        t.append("/");
    }
    return t + path_join(args...);
}

// 将点云进行转换 参数1：输入要合并的文件夹地址 参数2：输出文件全路径
int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];     // 合并文件地址
    std::string middlejson = argv[2];  // 建模输出的输入任务json文件，主要为了获取link信息
    std::string elementname = argv[3]; //"laneboundary"-车道线 "roadboundary"-道路边界
    std::string suffix = argv[4];      // 要合并的数据的文件名称匹配字符
    std::string outpcdPath = argv[5];  // 合并后pcd输出文件

    // std::string dataDir = "/home/lenovo/data/lidar_mapping_ws/55267/fsd_mapbuild_out";
    // std::string middlejson = "/home/lenovo/data/lidar_mapping_ws/55267/fsd_mapbuild_out/fsd_mapbuild_task.json";
    // std::string elementname = "laneboundary";
    // std::string suffix = "_trjCloud.pcd";
    // std::string outpcdPath = "/home/lenovo/data/lidar_mapping_ws/55267/fsd_mapbuild_out/laneboundary.pcd";

    nlohmann::json parseJosn;
    std::ifstream ifs_json(middlejson);
    if (!ifs_json.is_open())
    {
        ifs_json.close();
        std::cout << "参数文件打开失败！！！" << std::endl;
        return 0;
    }

    parseJosn << ifs_json;
    std::vector<std::string> links = parseJosn["links"];
    std::string dirname = "";
    int ele_type = 1;
    if (elementname == "laneboundary")
    {
        dirname = "laneBoundaryFromSeg";
        ele_type = 1;
    }
    else if (elementname == "lanecenter")
    {
        dirname = "laneCenterFromSeg";
        ele_type = 3;
    }
    else
    {
        dirname = "roadBoundaryFromSeg";
        ele_type = 2;
    }

    std::cout << "joinKeyPoints::elementname: " << elementname << std::endl;
    pcl::PointCloud<PointElement>::Ptr all_ptr(new pcl::PointCloud<PointElement>);
    int id = 0;
    for (auto link : links)
    {
        std::string linkMiddDataDir = path_join(dataDir, link, dirname);
        std::vector<std::string> all_file_names = get_files_name(linkMiddDataDir);
        for (int i = 0; i < all_file_names.size(); ++i)
        {
            if (boost::algorithm::ends_with(all_file_names[i], suffix))
            {
                std::string filepath = path_join(linkMiddDataDir, all_file_names[i]);
                std::cout << "合并数据：" << filepath << std::endl;
                pcl::PointCloud<PointElement>::Ptr in_ptr(new pcl::PointCloud<PointElement>);
                pcl::io::loadPCDFile(filepath, *in_ptr);
                int index = 0;
                for (auto &pt : in_ptr->points)
                {
                    PointElement newPointElement;
                    newPointElement.x = pt.x;
                    newPointElement.y = pt.y;
                    newPointElement.z = pt.z;
                    newPointElement.type1 = pt.type1;
                    newPointElement.type2 = pt.type2;
                    newPointElement.type3 = pt.type3;
                    newPointElement.id = id;
                    newPointElement.index = index;
                    newPointElement.ele_type = ele_type;
                    // newPointElement.intensity = std::stod(link);
                    newPointElement.score = std::stod(link);
                    all_ptr->points.push_back(newPointElement);
                    index++;
                }
                id++;
            }
        }
    }

    if (all_ptr->empty() || all_ptr->points.empty())
        return 0;

    pcl::io::savePCDFileBinary(outpcdPath, *all_ptr);
}
