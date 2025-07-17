#include "postProcessLaneBoundary.h"
#include "postProcessRoadMark.h"

#include "processRoadBoundary.h"

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string pcdPath = argv[2];
    std::string refFile = argv[3];
    std::string outputDir = argv[4];
    std::string label = "label";
    std::string lane_type = "lane_line";
    if (argc > 5) {
        label = argv[5];
    }

    if (argc > 6) {
        lane_type = argv[6];
    }
    std::cout << "label: " << label << " lane_type: " << lane_type << std::endl; 

    // std::string dataDir = "/home/test/data/fsd_mapbuild_out/283138391/";
    // std::string pcdPath = "/home/test/data/fsd_mapbuild_out/283138391/transCloud.pcd";
    // std::string refFile = "/home/test/data/fsd_mapbuild_out/283138391/parse_json.json";
    // std::string outputDir = "/home/test/data/fsd_mapbuild_out/283138391/laneBoundaryFromSeg";
    // std::string label = "cloud_label";
    // RoadMapping::PostProcessLaneBoundary::run_yxx_2(dataDir, pcdPath, refFile, outputDir, label);
    if (lane_type == "lane_line") {
        RoadMapping::ProcessRoadBoundary::run_qzc_fin_in_road(dataDir, pcdPath, refFile, outputDir, label);
    } else if (lane_type == "lane_center") {
        RoadMapping::ProcessRoadBoundary::run_qzc_lane_center_in_road(dataDir, pcdPath, refFile, outputDir, label);
    }
    std::cout << "successfully run postRoadModel" << std::endl;
}