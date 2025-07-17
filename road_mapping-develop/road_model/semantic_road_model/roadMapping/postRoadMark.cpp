//
//
//

// #include "postProcessLaneBoundary.h"
#include "postProcessRoadMark.h"

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string pcdPath = argv[2];
    std::string refFile = argv[3];
    std::string outputDir = argv[4];
    //    std::string dataDir = "/home/test/lidar_mapping_ws/test";
    //    std::string pcdPath = "/home/test/lidar_mapping_ws/test/mapping_output/global_map_ground.pcd";
    //    std::string refFile = "/home/test/lidar_mapping_ws/test/fsd_mapbuild_out/fsd_mapbuild_task.json";
    //    std::string outputDir = "/home/test/lidar_mapping_ws/test/fsd_mapbuild_out/RoadMarkRes";
    // std::string fsd_mapout = dataDir + "/fsd_mapbuild_out_cloud_pano_seg";
    // std::string fsd_mapout = dataDir;
    RoadMapping::PostProcessRoadMark postmark;
    // postmark.run_yxx_fin_in(fsd_mapout, pcdPath, refFile, outputDir);
    postmark.run_yxx_fin_add_type(dataDir, pcdPath, refFile, outputDir);
    //    RoadMapping::PostProcessRoadMark::run_yxx_bev_road(dataDir, fsd_mapout, pcdPath, refFile, outputDir);
    std::cout << "successfully run postRoadMark" << std::endl;
}