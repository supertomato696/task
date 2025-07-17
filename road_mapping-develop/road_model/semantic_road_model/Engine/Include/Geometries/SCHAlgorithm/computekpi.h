#include "NodeInfo.h"
#include <vector>
#include <string>

class XCalculateKPI
{
private:
	std::vector<std::string> Split(std::string str, std::string separator);

public:
	std::vector<std::vector<NodeInfo>> ReadMidFile(const char *pDirFile_Data);

	std::vector<std::vector<NodeInfo>> ReadMidFileByCurvature(const char *pDirFile_Data);

	void caculateSlope(std::vector<NodeInfo>& vec_copy);

	void calculateKPI(std::vector<NodeInfo>& vec_s);

	void calculateKPI(std::vector<std::vector<NodeInfo>>& vec_ss);

	void smoothKPI(std::vector<NodeInfo>& vec_ss);

	bool smoothKPI(std::vector<std::vector<NodeInfo>>& vec_ss);

	void WriteMidFile(const char *pPathFile_Result, std::vector<std::vector<NodeInfo>> vec_ssTaggedNodes);

	void WriteMidFileBySmoothCurvature(const char *pPathFile_Result, std::vector<std::vector<NodeInfo>> vec_ssTaggedNodes);
	
	void processKPI(std::string str_dataPathFile);

	void smoothKPITest(std::string str_dataPathFile);

	void CaculateAndSmooth(std::vector<std::vector<NodeInfo>>& vec_ss);
	
	void FilterByLength(std::string strSrc, std::string strDest);

	void CaculateAndSmooth(std::string strFile);

	void CaculateAndSmooth(std::vector<NodeInfo>& vec_s);

	void CheckOverLimitLink(std::string strFile);

	void ExportFile4lihua(std::string strSrcFile);

	void ExportPositionFile(std::vector<std::vector<NodeInfo>>& vecLine);

	void ReplaceLinkSCH(std::string strSrcFile, std::string strDestFile, std::vector<std::string> vecLinkIds);
};