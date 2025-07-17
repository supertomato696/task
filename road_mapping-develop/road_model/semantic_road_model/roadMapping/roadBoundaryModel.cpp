#include <iostream>
#include "processRoadBoundary.h"

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string pcdPath = argv[2];
    std::string refFile = argv[3];
    std::string outputDir = argv[4];
    std::string label = "label";
    if (argc > 5)
    {
        label = argv[5];
    }
    // std::string dataDir = "/home/test/data/102041/fsd_mapbuild_out/332491454";
    // std::string pcdPath = "/home/test/data/102041/fsd_mapbuild_out/332491454/transCloud.pcd";
    // std::string refFile = "/home/test/data/102041/fsd_mapbuild_out/332491454/parse_json.json";
    // std::string outputDir = "/home/test/data/102041/fsd_mapbuild_out/332491454/roadBoundaryFromSeg";
    // RoadMapping::ProcessRoadBoundary::run(dataDir, outputDir);
    // RoadMapping::ProcessRoadBoundary::run_yxx_2(dataDir, pcdPath, refFile, outputDir);
    // RoadMapping::ProcessRoadBoundary::run_yxx_fin_in_road(dataDir, pcdPath, refFile, outputDir, label);
    RoadMapping::ProcessRoadBoundary::run_yxx_fin_add_type(dataDir, pcdPath, refFile, outputDir, label);
    //    RoadMapping::ProcessRoadBoundary::run_yxx_all_road(dataDir, pcdPath, refFile, outputDir);
    std::cout << "successfully run roadBoundaryModel" << std::endl;

    return 0;
}