#include <iostream>
#include <fstream>
#include <gflags/gflags.h>
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include "dao/data_processer.h"
#include "road_model/road_model.h"

DEFINE_int32(log_level_console, 2, "log_level_console");
DEFINE_int32(log_level_file, 1, "log_level_file");
DEFINE_string(output_path, "/home/test/output/test.obj", "output_path");

std::string get_bin_version()
{
#ifdef VERSION_NUM
    std::string g_version = VERSION_NUM;
#else
    std::string g_version = "N/A";
#endif
#ifdef __COMMIT_ID__
    std::string g_commit = __COMMIT_ID__;
#else
    std::string g_commit = "N/A";
#endif
#ifdef __BRANCH__
    std::string g_branch = __BRANCH__;
#else
    std::string g_branch = "N/A";
#endif
#if defined(__DATE__) && defined(__TIME__)
    std::string g_model_built = __DATE__ " " __TIME__;
#else
    std::string g_model_built = "N/A";
#endif
    std::string algorithm_version;
    algorithm_version += ": " + g_version + ";";
    algorithm_version += "git branch: " + g_branch + ";";
    algorithm_version += "git commit id: " + g_commit + ";";
    algorithm_version += "built date: " + g_model_built + ";";
    return algorithm_version;
}

int main(int argc, char *argv[])
{
    google::SetVersionString(get_bin_version());
    google::ParseCommandLineFlags(&argc, &argv, false);
    auto logger = fsdmap::utils::Logger::instance();
    // logger->setLogLevel(spdlog::level::info, spdlog::level::info);
    logger->setLogLevel(fsdmap::utils::LogLevel(FLAGS_log_level_console),
                        fsdmap::utils::LogLevel(FLAGS_log_level_file));
    logger->get_timer().set_unit_level(4);
    logger->get_timer().start();
    logger->setup("log/las_to_pcd.log");

    fsdmap::dao::BevDataProcessor dp;
    if (dp.init() != fsdmap::SUCC)
    {
        LOG_ERROR("failed to init data_processer");
        return -1;
    }
    std::vector<std::shared_ptr<fsdmap::KeyPose>> group_pcd;
    if (dp.download_ground_pcd(group_pcd, true) != fsdmap::SUCC)
    {
        LOG_ERROR("failed to download group las");
        return -1;
    }
    LOG_INFO("pcd size {}", group_pcd[0]->pcd->size());

    std::string output_file = FLAGS_output_path;
    boost::filesystem::path path(output_file);
    if (!boost::filesystem::is_directory(path.parent_path()))
    {
        boost::filesystem::create_directory(path.parent_path());
    }
    pcl::io::savePCDFileBinary(output_file.c_str(), *group_pcd[0]->pcd);
    return 0;
}
