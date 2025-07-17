//
//
//
#include "pclPtType.h"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/keypoints/uniform_sampling.h>

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
    std::string dataDir = argv[1]; // 合并文件地址
    std::string pcdFile = argv[2]; // 输出合并后的文件名称
    std::string prefix = argv[3];  // 合并点云标识符 link_id
    //    std::string dataDir = "/home/test/data/10229/fsd_mapbuild_out/378575772";
    //    std::string pcdFile = "/home/test/data/10229/fsd_mapbuild_out/378575772/pp_combinePcd.pcd";
    //    std::string prefix = "378575772";
    pcl::PointCloud<MyColorPointType>::Ptr in_ptr(new pcl::PointCloud<MyColorPointType>);
    pcl::PointCloud<MyColorPointType>::Ptr all_ptr(new pcl::PointCloud<MyColorPointType>);

    std::vector<std::string> all_file_names = get_files_name(dataDir);
    for (size_t i = 0; i < all_file_names.size(); ++i)
    {
        if (boost::algorithm::starts_with(all_file_names[i], prefix))
        {
            std::cout << path_join(dataDir, all_file_names[i]) << std::endl;
            pcl::io::loadPCDFile(path_join(dataDir, all_file_names[i]), *in_ptr);
            *all_ptr += *in_ptr;
        }
    }

    if (all_ptr->empty())
        return 0;

    // 对点云进行均匀下采样工作
    pcl::PointCloud<MyColorPointType>::Ptr cloud_filtered(new pcl::PointCloud<MyColorPointType>); // 滤波后点云
    pcl::UniformSampling<MyColorPointType> us;                                                    // 创建滤波器对象
    us.setInputCloud(all_ptr);                                                                    // 设置待滤波点云
    us.setRadiusSearch(0.05f);                                                                    // 设置滤波球体半径
    us.filter(*cloud_filtered);                                                                   // 执行滤波，保存滤波结果于cloud_filtered

    if (!cloud_filtered->empty())
        pcl::io::savePCDFileBinary(pcdFile, *cloud_filtered);
}
