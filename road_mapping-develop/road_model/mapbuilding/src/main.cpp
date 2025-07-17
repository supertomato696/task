#include "mapbuilding.h"
#include <thread>
#include <mutex>
mutex g_mtx;
using namespace std;

int main(int argc,char* argv[]) {
	logUtil::initLog(&g_mtx);
	logUtil::log("starting...");

	if (argc != 3) {
		logUtil::log("parameter error,please pass in two parameters,first parameter is input path, second parameter is ouput path");
		return 1;
	}
	const char* pszInDirectory = argv[1];
	const char* pszOutDirectory = argv[2];

	VSIStatBufL statInfo1;
	VSIStatBufL statInfo2;
	int res1 = VSIStatExL(pszInDirectory, &statInfo1, VSI_STAT_EXISTS_FLAG);
	int res2 = VSIStatExL(pszOutDirectory, &statInfo2, VSI_STAT_EXISTS_FLAG);
	if (res1 != 0 || res2 != 0) {
		char szMsg[512] = {0};
		sprintf(szMsg, "the input parameter: %s or output parameter: %s does not exist", pszInDirectory, pszOutDirectory);
		logUtil::log(szMsg);
		return 1;
	}

	if (!CPL_TO_BOOL(VSI_ISDIR(statInfo1.st_mode)) || !CPL_TO_BOOL(VSI_ISDIR(statInfo2.st_mode))) {
		char szMsg[512] = { 0 };
		sprintf(szMsg, "the input parameter:%s or output parameter:%s is not a directory", pszInDirectory, pszOutDirectory);
		logUtil::log(szMsg);
		return 1;
	}

	string outputPath = pszOutDirectory;
	outputPath += "/mapbuilding_output";
	VSIRmdirRecursive(outputPath.c_str());
	VSIMkdir(outputPath.c_str(), 0755);

	mapbuilding* pBuilding = new mapbuilding();
	pBuilding->buildMapping(pszInDirectory, outputPath.c_str());
	//pBuilding->getStopLine("D:\\zhongbao\\mmt\\8\\stopline.shp");
	//pBuilding->getCrosswalk("D:\\zhongbao\\mmt\\10\\crosswalk.shp");
	//pBuilding->addPolygonsToOsm(NULL);
	//pBuilding->addLinesToOsm("D:\\zhongbao\\mmt\\6\\lanemarking.shp");
	//pBuilding->kappa2osm("D:\\zhongbao\\mmt\\6\\kappa.shp");
	//pBuilding->export_gnss_rtk(pszDir, "D:\\zhongbao\\mmt\\8\\gnss.txt");
	//pBuilding->parse_mmt(pszInDirectory,"D:/zhongbao/mmt/10/mapbuilding_output/50");
	//pBuilding->getRoadMark("D:\\zhongbao\\mmt\\9\\roadmark.shp");
	//pBuilding->getLanemarking("/home/output/lanemarking.shp");
	//pBuilding->matching();
	//pBuilding->addFields("D:\\zhongbao\\mmt\\2\\lanemarking.shp");
	delete pBuilding;
	logUtil::log("finish");
	return 0;
}
