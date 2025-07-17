//
//
//
// 人形横道建模
#include "processCrosswalk.h"
#include "processStopline.h"
#include "processLuKouPly.h"
#include "processImpassableArea.h"
#include "processCrosswalk.h"
#include <boost/filesystem.hpp>

int main(int argc, char *argv[])
{
    std::string fsd_mapout = argv[1];
    std::string pcdPath = argv[2];
    std::string refFile = argv[3];
    std::string outputDir = argv[4];
    std::string middle_json_path = argv[5];
    std::string data_type = argv[6];

    // std::string fsd_mapout = "/home/test/data/lukou_test/fsd_mapbuild_out";
    // std::string pcdPath = "/home/test/data/lukou_test/global_map_ground.pcd";
    // std::string refFile = "/home/test/data/lukou_test/fsd_mapbuild_out/fsd_mapbuild_task.json";
    // std::string outputDir = "/home/lenovo/data/lidar_mapping_ws/55302/fsd_mapbuild_out/LuKou";

    // std::string bev_obj_dir = "/home/lenovo/data/lidar_mapping_ws/55302/bev_obj/";
    // std::string global_pcd_path = "/home/lenovo/data/lidar_mapping_ws/55302/model_pcd/global_cloud.pcd";

    std::cout << "start run postLuKou: processStopline" << std::endl;
    // RoadMapping::processStopline::run_yxx_fin_in(fsd_mapout, pcdPath, refFile, outputDir);
    RoadMapping::processStopline::run_yxx_fin_add_type(middle_json_path, fsd_mapout, pcdPath, refFile, outputDir, data_type);
    std::cout << "successfully run postLuKou: processStopline" << std::endl;

    // std::cout << "start run guideline: run_extract_guide_line" << std::endl;
    // RoadMapping::processStopline::run_extract_guide_line(fsd_mapout, pcdPath, refFile, outputDir);
    // std::cout << "successfully run guideline: run_extract_guide_line" << std::endl;

    std::cout << "start run postLuKou: processLuKouPly" << std::endl;
    // std::string bev_obj_dir = fsd_mapout + "/bev_obj";
    RoadMapping::processLuKouPly bevprocessLuKouPly;
    // bevprocessLuKouPly.run_bev_lukouboundary(fsd_mapout, pcdPath, bev_obj_dir, outputDir);
    // bevprocessLuKouPly.run_cloud_bev_lukoubd(pcdPath, outputDir);
    bevprocessLuKouPly.generate_lukou_boundary_manual(middle_json_path, pcdPath, outputDir);
    std::cout << "successfully run postLuKou: processLuKouPly" << std::endl;

    // std::cout << "start run postLuKou: processImpassableArea" << std::endl;
    // boost::filesystem::path filePath(pcdPath);
    // std::string model_pcd = filePath.parent_path().string();
    // std::string flat_global_pcd = model_pcd + "/flat_global_cloud.pcd";
    // RoadMapping::processImpassableArea bevprocessimpassablearea;
    // bevprocessimpassablearea.run_impassable_area_process(flat_global_pcd, outputDir);
    // std::cout << "successfully run postLuKou: processImpassableArea" << std::endl;

    std::cout << "start run postLuKou: processCrosswalk" << std::endl;
    std::cout << "data_type: " << data_type << std::endl;
    RoadMapping::processCrosswalk processCrosswalk;
    processCrosswalk.run_yxx_fin_in(middle_json_path, fsd_mapout, pcdPath, refFile, outputDir, data_type);
    std::cout << "successfully run postLuKou: processCrosswalk" << std::endl;
}