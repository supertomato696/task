/***************************************************************
 * Copyright 2022 The biyadi Authors. All Rights Reserved.
 *
 * @author: biyadi@biyadi.com
 * @date: 2022-12-20
 *
 * @desc: bev roadmodel meta
 *
 ***************************************************************/

#include <iostream>
#include <gflags/gflags.h>
#include "utils/macro_util.h"
#include "utils/log_util.h"
#include "utils/common_util.h"
#include "dao/data_processer.h"
#include "road_model/road_model.h"
#include "version.h"

DEFINE_int32(log_level_console, 2, "log_level_console");
DEFINE_int32(log_level_file, 1, "log_level_file");

std::string get_bin_version() {
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


int main(int argc, char* argv[]) {
    google::SetVersionString(get_bin_version());
    google::ParseCommandLineFlags(&argc, &argv, false);
    // google::ParseCommandLineFlags(&argc, &argv, true);

    auto logger = fsdmap::utils::Logger::instance();
    // logger->setLogLevel(spdlog::level::info, spdlog::level::info);
    logger->setLogLevel(fsdmap::utils::LogLevel(FLAGS_log_level_console),
            fsdmap::utils::LogLevel(FLAGS_log_level_file));
    logger->get_timer().set_unit_level(3);
    logger->get_timer().start();

    // std::vector<std::shared_ptr<fsdmap::KeyPose>> key_poses;
    fsdmap::dao::BevDataProcessor dp;
    std::string log_dir = fsdmap::utils::fmt("{}/road_model.log", dp.get_log_dir());
    logger->setup(log_dir);

    LOG_INFO(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>");
    LOG_INFO("......start road_mapping .....");
    LOG_INFO("road_mapping_version:v{}.{}.{}", ROAD_MAPPING_MAJOR_VERSION, ROAD_MAPPING_MINOR_VERSION, ROAD_MAPPING_PATCH_VERSION);

    LOG_INFO("log_dir:{}", log_dir);

    if (dp.init() != fsdmap::SUCC) {
        CLOG_ERROR("failed to init road_model");
        return -1;
    }
    // dp.download_key_pos(key_poses);
    // LOG_DEBUG("key pose size {}", key_poses.size());
    
    fsdmap::road_model::RoadModelProcess road_model;

    road_model.set_logger(logger);
    road_model.set_data_processer(&dp);
    int ret = road_model.init();
    if (ret != fsdmap::SUCC) {
        CLOG_ERROR("failed to init road_model");
        return -1;
    }


    ret = road_model.road_model();

    logger->print();
    return ret;

    // return 0;
}
