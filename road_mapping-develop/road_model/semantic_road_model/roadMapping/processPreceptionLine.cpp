//
//
//
#include <iostream>
#include "processPreceptionLine.h"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include <filesystem>
#include <fstream>
#include "json.hpp"
#include <bitset>
namespace RoadMapping
{

    void processPPline::ReadPerceptionLinePts(std::string dataDir, std::string trailid, std::string outDir)
    {
        // 获取感知结果线
        std::string data_set_path = dataDir + "/" + trailid + "/data_set.json";
        std::string pp_laneline_path = dataDir + "/" + trailid + "/pp_laneline_info.json";

        nlohmann::json j_data_set;
        std::ifstream ifs(data_set_path);
        ifs >> j_data_set;

        nlohmann::json j_pp;
        std::ifstream ifs_pp(pp_laneline_path);
        ifs_pp >> j_pp;

        // 读入轨迹内所有参数信息
        Trail trail;
        int pp_index = 0;
        for (auto one : j_data_set["data"])
        {

            uint64_t stamp_ms = one["stamp_ms"];
            std::string link_id = to_string(one["link_id"]);
            Eigen::Vector4d q_world_veh;
            for (int i = 0; i < 4; i++)
            {
                q_world_veh[i] = one["opt_q_world_veh"][i];
            }
            Eigen::Vector3d t_world_veh;
            for (int i = 0; i < 3; i++)
            {
                t_world_veh[i] = one["opt_t_world_veh"][i];
            }
            trail.stamp_ms_link[stamp_ms] = link_id;
            Eigen::Quaterniond q_world_veh_trans = Eigen::Quaterniond(q_world_veh);
            Eigen::Matrix4d T_world_veh = qab_tab_to_Tab(q_world_veh_trans, t_world_veh);
            trail.stamp_ms_T_world_veh[stamp_ms] = T_world_veh;
            //            std::cout << T_world_veh <<std::endl;
        }

        // 读入每帧的感知车道线
        Frame frame;
        for (auto lines : j_pp["data"])
        {
            uint64_t stamp_ms = lines["stamp_ms"];
            std::string link_id = trail.stamp_ms_link[stamp_ms];
            int n = lines["pp_coeffs"].size();
            for (int i = 0; i < n; i++)
            {
                nlohmann::json coeffsObj = lines["pp_coeffs"][i];
                //                std::cout<<coeffsObj<<std::endl;
                Eigen::Vector4d coeffs;
                coeffs << coeffsObj[0], coeffsObj[1], coeffsObj[2], coeffsObj[3];

                nlohmann::json pp_typesObj = lines["pp_types"];
                //                std::cout<<pp_typesObj<<std::endl;

                int type = pp_typesObj[i];
                GetPoints(coeffs, type, trail.stamp_ms_T_world_veh[stamp_ms], frame.point_dict[link_id]);
            }
        }

        // 结果输出
        for (auto iter = frame.point_dict.begin(); iter != frame.point_dict.end(); iter++)
        {
            std::string outfile = outDir + "/" + iter->first + "_" + trailid + ".pcd";
            pcl::io::savePCDFileBinary(outfile, iter->second);
        }
    }
    void processPPline::GetPoints(Eigen::Vector4d coeffs, int type, Eigen::Matrix4d T_world_veh, pcl::PointCloud<MyColorPointType> &ptsCloud)
    {
        // 从三次多项式曲线插值获取数据
        // LANE_ROI_AREA = {(0, 3): [4, 14], (3, 6): [6, 16],(6, 9): [8, 18], (9, 12): [10, 20], (12, 22): [14, 24], (15, 10000): [16, 26]};
        double BUFFER_SIZE_Y = 6.0;
        double y = fabs(coeffs(0));
        if (y > BUFFER_SIZE_Y)
            return;

        double start_x = 0.0;
        double end_x = 0.0;
        if (y >= 0 && y < 3)
        {
            start_x = 4.0;
            end_x = 14.0;
        }
        else if (y >= 3 && y < 6)
        {
            start_x = 6.0;
            end_x = 16.0;
        }
        else if (y >= 6 && y < 9)
        {
            start_x = 8.0;
            end_x = 18.0;
        }
        else if (y >= 9 && y < 12)
        {
            start_x = 10.0;
            end_x = 20.0;
        }
        else if (y >= 12 && y < 22)
        {
            start_x = 14.0;
            end_x = 24.0;
        }
        else
        {
            start_x = 16.0;
            end_x = 26.0;
        }

        MyColorPointType p_world;
        // p_world.label = 80;
        p_world.r = 255;
        p_world.g = 255;
        p_world.b = 255;
        // 计算线的类型以及颜色
        LineType linetype;
        if (type & linetype.LineType_Line_Fence)
        {
            std::cout << "监测到fence，不加入标线模块中" << std::endl;
            return;
        }

        if (type & linetype.LineType_Line_Yellow)
        {
            p_world.r = 255;
            p_world.g = 255;
            p_world.b = 0;
        }

        if (type & linetype.LineType_Line_Ramp)
        {
            p_world.intensity = 1;
        }
        else if (type & linetype.LineType_Line_Double)
        {
            p_world.intensity = 2;
        }
        else if (type & linetype.LineType_Line_Dash)
        {
            p_world.intensity = 3;
        }
        else if (type & linetype.LineType_Line_Solid)
        {
            p_world.intensity = 4;
        }

        double step = start_x;
        while (step <= end_x)
        {
            float x = step, x2 = x * x, x3 = x * x2;
            float y = coeffs(0) + coeffs(1) * x + coeffs(2) * x2 + coeffs(3) * x3;
            float z = 0;

            Eigen::Vector4d pos_veh;
            pos_veh << x, y, z, 1;

            Eigen::Vector4d pos_world = T_world_veh * pos_veh;
            p_world.x = pos_world[0];
            p_world.y = pos_world[1];
            p_world.z = pos_world[2];
            ptsCloud.push_back(p_world);
            //            std::cout<<"step:"<<step<<std::endl;
            step += 0.05; // 每50厘米解析一个点
        }
    }
}