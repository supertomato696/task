//
//
//
#include "processPreceptionLine.h"

int main(int argc, char *argv[])
{
    std::string dataDir = argv[1];
    std::string trailid = argv[2];
    std::string outDir = argv[3];
    //    std::string dataDir = "/home/test/data/10229/data/";
    //    std::string trailid = "4431663971207";
    //    std::string outDir = "/home/test/data/10229/data/";
    std::cout << "start run processPerception: " << trailid << std::endl;
    RoadMapping::processPPline::ReadPerceptionLinePts(dataDir, trailid, outDir);
    std::cout << "successfully run processPerception: " << trailid << std::endl;
}
