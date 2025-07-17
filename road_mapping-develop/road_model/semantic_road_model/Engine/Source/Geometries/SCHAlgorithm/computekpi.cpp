#include "Geometries/SCHAlgorithm/computekpi.h"
#include "Geometries/SCHAlgorithm/fit.h"
#include <iostream>
#include <fstream>
//#include "gdalutil.h"
#include <time.h>
#include "Geometries/SCHAlgorithm/FuzzySegmentor.h"
#include "Geometries/SCHAlgorithm/CoordinateTransfer.h"
#include "Geometries/SCHAlgorithm/MultipleCircle.h"
#include "Geometries/SCHAlgorithm/SlopeSegmentor.h"
#include "Geometries/SCHAlgorithm/MultipleSlope.h"
#include <string.h>

#include "Base/CWinToLinux.h"

using namespace std;

vector<string> XCalculateKPI::Split(string str, string separator)
{
	vector<string> result;
	int cutAt;

	while ((cutAt = str.find_first_of(separator)) != str.npos)
	{
		if (cutAt > 0)
		{
			result.push_back(str.substr(0, cutAt));
		}

		str = str.substr(cutAt + 1);
	}

	if (str.length() > 0)
	{
		result.push_back(str);
	}

	return result;
}

vector<vector<NodeInfo>> XCalculateKPI::ReadMidFile(const char *pDirFile_Data)
{
	char szTest[1000] = { 0 };
	int len = 0;
	FILE *fp = NULL;
	vector<vector<NodeInfo>> vec_ss;

	//errno_t err = fopen_s(&fp, pDirFile_Data, "r");
	if (OpenFile(fp, pDirFile_Data, "r") != 0)
		return vec_ss;


	string str_Test;
	string str_row;
	vector<string> lst_str;
	string str_LinkID = "";
	
	vector<NodeInfo> vec_s;
	NodeInfo nodeinfo;
	double lon0 = 0.0;
	double lat0 = 0.0;

	while (!feof(fp))
	{
		memset(szTest, 0, sizeof(szTest));
		fgets(szTest, sizeof(szTest) - 1, fp);
		str_Test = szTest;

		if (str_Test.compare("") == 0)
		{
			break;
		}

		str_row = szTest;
		lst_str = Split(str_row, ",");

		if (str_LinkID.compare(lst_str[1]) != 0)
		{
			lon0 = atof(lst_str[3].substr(1, lst_str[3].length() - 1).c_str());
			lat0 = atof(lst_str[4].substr(1, lst_str[4].length() - 1).c_str());

			if (vec_s.size() > 0)
			{
				vec_ss.push_back(vec_s);
				vec_s.clear();
			}
		}

		nodeinfo.PostnID = lst_str[0];
		nodeinfo.LinkID = lst_str[1];
		nodeinfo.IDFlag = lst_str[2];
		double lon = atof(lst_str[3].substr(1, lst_str[3].length() - 2).c_str());
		double lat = atof(lst_str[4].substr(1, lst_str[4].length() - 2).c_str());
		double originX = 0.0, originY = 0.0;

		if (vec_s.size() > 0) {
			originX = CoordinateTransfer::calcP2PDistance(lon0, lat, lon, lat);
			originY = CoordinateTransfer::calcP2PDistance(lon, lat0, lon, lat);

			if (lon < lon0)
			{
				originX *= -1;
			}

			if (lat < lat0)
			{
				originY *= -1;
			}
		}

		/*if (vec_s.size() > 0)
		{
		originX = lon-lon0;
		originY = lat-lat0;
		}*/

		nodeinfo.setOrigin_X(originX);
		nodeinfo.setOrigin_Y(originY);
		nodeinfo.setWgs_Z(atof(lst_str[5].substr(1, lst_str[5].length() - 2).c_str()));
		nodeinfo.x = lst_str[3];
		nodeinfo.y = lst_str[4];
		nodeinfo.z = lst_str[5];
		vec_s.push_back(nodeinfo);
		str_LinkID = lst_str[1];
	}

	vec_ss.push_back(vec_s);
	fclose(fp);
	fp = NULL;

	return vec_ss;
}

vector<vector<NodeInfo>> XCalculateKPI::ReadMidFileByCurvature(const char *pDirFile_Data)
{
	char szTest[1000] = { 0 };
	int len = 0;
	FILE *fp = NULL;
	vector<vector<NodeInfo>> vec_ss;
	//errno_t err = fopen_s(&fp, pDirFile_Data, "r");
	if (OpenFile(fp, pDirFile_Data, "r") != 0)
		return vec_ss;

	
	string str_Test;
	string str_row;
	vector<string> lst_str;
	string str_LinkID = "";
	
	vector<NodeInfo> vec_s;
	NodeInfo nodeinfo;
	double lon0 = 0.0;
	double lat0 = 0.0;

	while (!feof(fp))
	{
		memset(szTest, 0, sizeof(szTest));
		fgets(szTest, sizeof(szTest) - 1, fp);
		str_Test = szTest;

		if (str_Test.compare("") == 0)
		{
			break;
		}

		str_row = szTest;
		lst_str = Split(str_row, ",");

		if (str_LinkID.compare(lst_str[1]) != 0)
		{
			lon0 = atof(lst_str[3].substr(1, lst_str[3].length() - 1).c_str());
			lat0 = atof(lst_str[4].substr(1, lst_str[4].length() - 1).c_str());

			if (vec_s.size() > 0)
			{
				vec_ss.push_back(vec_s);
				vec_s.clear();
			}
		}

		nodeinfo.PostnID = lst_str[0];
		nodeinfo.LinkID = lst_str[1];
		nodeinfo.IDFlag = lst_str[2];
		double lon = atof(lst_str[3].substr(1, lst_str[3].length() - 2).c_str());
		double lat = atof(lst_str[4].substr(1, lst_str[4].length() - 2).c_str());
		double originX = 0.0, originY = 0.0;

		if (vec_s.size() > 0) {
			originX = CoordinateTransfer::calcP2PDistance(lon0, lat, lon, lat);
			originY = CoordinateTransfer::calcP2PDistance(lon, lat0, lon, lat);

			if (lon < lon0)
			{
				originX *= -1;
			}

			if (lat < lat0)
			{
				originY *= -1;
			}
		}

		/*if (vec_s.size() > 0)
		{
		originX = lon-lon0;
		originY = lat-lat0;
		}*/

		nodeinfo.setOrigin_X(originX);
		nodeinfo.setOrigin_Y(originY);
		nodeinfo.setWgs_Z(atof(lst_str[5].substr(1, lst_str[5].length() - 2).c_str()));
		nodeinfo.x = lst_str[3];
		nodeinfo.y = lst_str[4];
		nodeinfo.z = lst_str[5];


		double slope = atof(lst_str[6].substr(1, lst_str[6].length() - 2).c_str());
		double heading = atof(lst_str[8].substr(1, lst_str[8].length() - 2).c_str());
		double curvature = atof(lst_str[9].substr(1, lst_str[9].length() - 2).c_str());
		nodeinfo.setHeading(heading);
		nodeinfo.setSlope(slope);

		//DONGJIAN
		nodeinfo.bNegtive = (curvature < 0);
		nodeinfo.setCurvature(curvature);

		vec_s.push_back(nodeinfo);
		str_LinkID = lst_str[1];
	}

	vec_ss.push_back(vec_s);
	fclose(fp);
	fp = NULL;

	return vec_ss;
}

void XCalculateKPI::caculateSlope(vector<NodeInfo>& vec_copy)
{
	double lastWgsX = 0;
	double lastWgsY = 0;
	double baseD = 0;

	for (int i = 0; i < vec_copy.size(); i++)
	{
		NodeInfo &node = vec_copy[i];
		double currentWgsX = node.getOrigin_X();
		double currentWgsY = node.getOrigin_Y();
		if (i > 0)
		{
			double dx = lastWgsX-currentWgsX;
			double dy = lastWgsY-currentWgsY;
			baseD += sqrt(dx*dx+dy*dy);
		}
		node.setOrigin_X(baseD);
		node.setReference_X(baseD);
		double baseZ = node.getWgs_Z() / 100.0;
		node.setOrigin_Y(baseZ);
		node.setReference_Y(baseZ);
		//////////////////////////////////////////////////////////////////////////
		lastWgsX = currentWgsX;
		lastWgsY = currentWgsY;
	}

	time_t t_start, t_end;
	t_start = time(NULL);
	SlopeSegmentor segmentorSlope;
	vec_copy = segmentorSlope.tagSegmentBelong(vec_copy);

	/*t_end = time(NULL);
	cout << "tag segment belong cost " << t_end - t_start << endl;
	t_start = time(NULL);*/

	if (vec_copy.size() > 80)
	{
		MultipleSlope multipleSlope;
		vec_copy = multipleSlope.tagPathCurveByDicho(vec_copy);
	}
	else
	{
		std::vector<double> vecX;
		std::vector<double> vecY;
		int n = vec_copy.size();
		for (int i = 0; i < n; i++)
		{
			NodeInfo node = vec_copy[i];
			vecX.push_back(node.getReference_X());
			vecY.push_back(node.getReference_Y());
		}
		czy::Fit fit;
		fit.polyfit(vecX, vecY, 2);

		vector<double> vecFactors;
		fit.getFactor(vecFactors);


		for (int i = 0; i < n; i++)
		{
			NodeInfo& node = vec_copy[i];
			double y_deriv = 2 * vecFactors[2] * node.getReference_X() + vecFactors[1];

			double dAngle = atan2(y_deriv,1);
			dAngle = dAngle * 180 / 3.141592653;
			node.setSlope(dAngle);
		}		
	}

	/*t_end = time(NULL);
	cout << "Multi slope cost " << t_end - t_start << endl;*/
}

void XCalculateKPI::calculateKPI(vector<NodeInfo>& vec_s)
{
	//cout << vec_s.front().LinkID << endl;
	vector<NodeInfo> vec_copy = vec_s;
	time_t t_start, t_end;
	t_start = time(NULL);

	FuzzySegmentor segmentorFuzzy;
	segmentorFuzzy.tagSegmentBelong(vec_s);

	/*t_end = time(NULL);
	cout << "FuzzySegmentor cost time : " << t_end - t_start << endl;
	t_start = time(NULL);*/

	MultipleCircle multipleCircle;
	multipleCircle.tagPathCurveByDicho(vec_s);
	
	/*t_end = time(NULL);
	cout << "multi circle cost time : " << t_end - t_start << endl;*/

	t_start = time(NULL);
	caculateSlope(vec_copy);

	if (vec_copy.size() != vec_s.size())
	{
		cout << "error ! 坡段计算结果 vector size 与 原始数据不一致" << endl;
		return;
	}

	//给结果赋值
	for (int i = 0; i < vec_copy.size(); i++)
	{
		NodeInfo& node = vec_copy[i];
		vec_s[i].setSlope(node.getSlope());
	}

// 	t_end = time(NULL);
// 	cout << "slope cost time " <<t_end - t_start<< endl;
}

void XCalculateKPI::calculateKPI(vector<vector<NodeInfo>>& vec_ss)
{
	//vector<vector<NodeInfo>> vec_ss_taggedNodes;
	int iNum_vec = 0;
	int tempindex = 0;

	for (vector<vector<NodeInfo>>::iterator it_vecs = vec_ss.begin(); it_vecs != vec_ss.end(); it_vecs++)
	{
		vector<NodeInfo>& vec_s = *it_vecs;
		calculateKPI(vec_s);
	}
}

void XCalculateKPI::smoothKPI(vector<NodeInfo>& vec_ss)
{
	//////////////////////////////////////////////////////////////////////////
	/// 曲率的平滑
	FuzzySegmentor segmentorFuzzy;
	//1.挑出异常区间
	map<int, int> mapAbnormalIndex;
	bool bSelectgFlag = segmentorFuzzy.selectAbnormalNodes2(vec_ss, mapAbnormalIndex);
	if (bSelectgFlag == false)
	{
		cout << "selectAbnormalNodes  error" << endl;
	}

	//bool bFlag = segmentorFuzzy.se;
	//2.对异常区间进行平滑操作
	MultipleCircle  multipleCircle;
	bool bInterpolationFlag = multipleCircle.smoothArrayNodeByLineInterpolation(vec_ss, mapAbnormalIndex, true);
	if (bInterpolationFlag == false)
	{
		cout << "smoothArrayNodeByLineInterpolation error" << endl;
	}

	//3.去除曲率变号导致的突变
	multipleCircle.removeSignJump(vec_ss, 15);
	multipleCircle.removeSignJump(vec_ss, 30);

	//4.找出曲率正负变号的位置并平滑
	multipleCircle.SmoothPositiveNegtiveJump(vec_ss, 10);

	//////////////////////////////////////////////////////////////////////////
	//坡度的平滑
	mapAbnormalIndex.clear();
	SlopeSegmentor::selectAbnormalNodes2(vec_ss, mapAbnormalIndex);
	SlopeSegmentor::smoothArrayNodeByLineInterpolation(vec_ss, mapAbnormalIndex);

	//拉平，暂时不必要

}

bool XCalculateKPI::smoothKPI(vector<vector<NodeInfo>>& vec_ss)
{
	int tempindex = 0;
	for (vector<vector<NodeInfo>>::iterator it_vecs = vec_ss.begin(); it_vecs != vec_ss.end(); it_vecs++)
	{
		NodeInfo node = it_vecs->front();
		cout << "node ID :\t" << node.LinkID << endl;

		smoothKPI(*it_vecs);
	}

	return true;
}

void XCalculateKPI::CaculateAndSmooth(vector<vector<NodeInfo>>& vec_ss)
{
	calculateKPI(vec_ss);
	smoothKPI(vec_ss);
}

void XCalculateKPI::CaculateAndSmooth(vector<NodeInfo>& vec_s)
{
	calculateKPI(vec_s);
	//smoothKPI(vec_s);
}

void XCalculateKPI::CaculateAndSmooth(std::string strFile)
{
	const char *pDirFile_Data = strFile.c_str();
	vector<vector<NodeInfo>> vec_ss = ReadMidFile(pDirFile_Data);
	calculateKPI(vec_ss);

	//WriteMidFile("d:/data/aodi/result.mid", vec_ss);
	//smoothKPI(vec_ss);

	string str_resultPathFile = strFile.substr(0, strFile.find_last_of("\\/"));
	str_resultPathFile += "\\smoot_result.mid";
	
	WriteMidFile(str_resultPathFile.c_str(), vec_ss);
}

void XCalculateKPI::FilterByLength(string strSrc, string strDest)
{
	vector<vector<NodeInfo>> vec_ss = ReadMidFileByCurvature(strSrc.c_str());
	const double INTERVAL = 0.95;
	const double INTERVAL1 = 1;
	const double zLimit = 6;

	for (int i = 0; i < vec_ss.size(); i++)
	{
		vector<NodeInfo>& vec_s = vec_ss[i];

		for (int j = 0; j < vec_s.size(); j++)
		{
			NodeInfo& currentNode = vec_s[j];
			int rIndex = 0;
			for (int k = j + 1; k < vec_s.size() - 1; k++)
			{
				NodeInfo& nextNode = vec_s[k];

				double distance = sqrt((currentNode.origin_X - nextNode.origin_X)*(currentNode.origin_X - nextNode.origin_X) +
					(currentNode.origin_Y - nextNode.origin_Y)*(currentNode.origin_Y - nextNode.origin_Y));

				double deltaZ = fabs(currentNode.wgs_Z - nextNode.wgs_Z);

				if (distance < INTERVAL && deltaZ <= 6)
					continue;

				if (distance > INTERVAL&& distance < INTERVAL1&&deltaZ <= 6)
				{
					if (k > j + 1)
					{
						vec_s.erase(vec_s.begin() + j + 1, vec_s.begin() + k);
						cout << "link:\t" << currentNode.LinkID << "id:\t" << j << "\tid:\t" << k << "\tdistance\t" << distance << endl;
					}
					break;
				}

				if (distance > INTERVAL1 || deltaZ > 6)
				{
					if (k - 1 > j + 1)
					{
						double distance1 = sqrt((currentNode.origin_X - vec_s[k - 1].origin_X)*(currentNode.origin_X - vec_s[k - 1].origin_X) +
							(currentNode.origin_Y - vec_s[k - 1].origin_Y)*(currentNode.origin_Y - vec_s[k - 1].origin_Y));

						vec_s.erase(vec_s.begin() + j + 1, vec_s.begin() + k - 1);
						cout << "distance :\t" << distance1 << endl;
					}
					break;
				}
			} //k
		} //j

	} //i
	WriteMidFileBySmoothCurvature(strDest.c_str(), vec_ss);
}

void XCalculateKPI::WriteMidFileBySmoothCurvature(const char *pPathFile_Result, vector<vector<NodeInfo>> vec_ssTaggedNodes)
{
	char szTest[1000] = { 0 };
	int len = 0;
	FILE *fp = NULL;
	//errno_t err = fopen_s(&fp, pPathFile_Result, "w+");

	if (OpenFile(fp, pPathFile_Result, "w+" )!= 0)
		return;


	NodeInfo nodeinfo;
	vector<NodeInfo> vec_sNodes;
	char chSlope[20];
	char chHeading[20];
	char chCurvature[20];
	string str_Slope = "";
	string str_Heading = "";
	string str_Curvature = "";
	string lineTxt = "";

	for (vector<vector<NodeInfo>>::iterator it_vecs = vec_ssTaggedNodes.begin(); it_vecs != vec_ssTaggedNodes.end(); it_vecs++)
	{
		vec_sNodes = *it_vecs;
		double dx = 0.0, dy = 0.0, ddis = 0.0;

		for (vector<NodeInfo>::iterator it = vec_sNodes.begin(); it != vec_sNodes.end(); it++)
		{
			nodeinfo = *it;
			lineTxt = nodeinfo.PostnID;
			lineTxt += ",";
			lineTxt += nodeinfo.LinkID;
			lineTxt += ",";
			lineTxt += nodeinfo.IDFlag;

			if (dx != 0.0 && dy != 0.0)
			{
				ddis = sqrt(pow(dx - nodeinfo.getOrigin_X(), 2) + pow(dy - nodeinfo.getOrigin_Y(), 2));
			}
			else
			{
				ddis = 0.0;
			}

			lineTxt += ",";
			lineTxt += nodeinfo.x;
			lineTxt += ",";
			lineTxt += nodeinfo.y;
			lineTxt += ",";
			lineTxt += nodeinfo.z;
			lineTxt += ",\"";
			snprintf(chSlope,20,"%d", (int)(nodeinfo.getSlope()));

			str_Slope = chSlope;
			lineTxt += str_Slope;
			lineTxt += "\",";
			lineTxt += "\"0\"";
			lineTxt += ",\"";
			snprintf(chHeading,20, "%d", (int)(nodeinfo.getHeading()));

			
			str_Heading = chHeading;
			lineTxt += str_Heading;
			lineTxt += "\",\"";
			//sprintf_s(chCurvature, "%d",std::abs((int)(nodeinfo.getCurvature()))/* * (nodeinfo.bNegtive ? -1 : 1)*/); //dongjian
			snprintf(chCurvature,20, "%d", (int)(nodeinfo.getCurvature())); //dongjian

			str_Curvature = chCurvature;
			lineTxt += str_Curvature;
			lineTxt += "\"";
			lineTxt += "\n";
			fputs(lineTxt.c_str(), fp);
			dx = nodeinfo.getOrigin_X();
			dy = nodeinfo.getOrigin_Y();
		}
	}

	fclose(fp);
	fp = NULL;
}

void XCalculateKPI::WriteMidFile(const char *pPathFile_Result, vector<vector<NodeInfo>> vec_ssTaggedNodes)
{
	char szTest[1000] = { 0 };
	int len = 0;
	FILE *fp = NULL;
	//errno_t err = fopen_s(&fp, pPathFile_Result, "w+");

	if (OpenFile(fp, pPathFile_Result, "w+" )!= 0)
		return;

	NodeInfo nodeinfo;
	vector<NodeInfo> vec_sNodes;
	char chSlope[20];
	char chHeading[20];
	char chCurvature[20];
	string str_Slope = "";
	string str_Heading = "";
	string str_Curvature = "";
	string lineTxt = "";

	for (vector<vector<NodeInfo>>::iterator it_vecs = vec_ssTaggedNodes.begin(); it_vecs != vec_ssTaggedNodes.end(); it_vecs++)
	{
		vec_sNodes = *it_vecs;
		double dx = 0.0, dy = 0.0, ddis = 0.0;

		for (vector<NodeInfo>::iterator it = vec_sNodes.begin(); it != vec_sNodes.end(); it++)
		{
			nodeinfo = *it;
			lineTxt = nodeinfo.PostnID;
			lineTxt += ",";
			lineTxt += nodeinfo.LinkID;
			lineTxt += ",";
			lineTxt += nodeinfo.IDFlag;

			if (dx != 0.0 && dy != 0.0)
			{
				ddis = sqrt(pow(dx - nodeinfo.getOrigin_X(), 2) + pow(dy - nodeinfo.getOrigin_Y(), 2));
			}
			else
			{
				ddis = 0.0;
			}

			lineTxt += ",";
			lineTxt += nodeinfo.x;
			lineTxt += ",";
			lineTxt += nodeinfo.y;
			lineTxt += ",";
			lineTxt += nodeinfo.z;
			lineTxt += ",\"";
			snprintf(chSlope, 20, "%d", (int)(10000 * nodeinfo.getSlope()));


			

			str_Slope = chSlope;
			lineTxt += str_Slope;
			lineTxt += "\",";

			//原本用来描述横坡 现用于描述 分段点 
			lineTxt += "\"0\"";

			lineTxt += ",\"";
			snprintf(chHeading, 20, "%d", (int)(10000 * nodeinfo.getHeading()));

			
			str_Heading = chHeading;
			lineTxt += str_Heading;
			lineTxt += "\",\"";
			snprintf(chCurvature,20, "%d", (int)(1000000 * nodeinfo.getCurvature()));//sprintf_s=>snprintf


			str_Curvature = chCurvature;
			lineTxt += str_Curvature;
			lineTxt += "\"";
			lineTxt += "\n";
			fputs(lineTxt.c_str(), fp);
			dx = nodeinfo.getOrigin_X();
			dy = nodeinfo.getOrigin_Y();
		}
	}

	fclose(fp);
	fp = NULL;
}

void XCalculateKPI::CheckOverLimitLink(std::string strFile)
{
	vector<vector<NodeInfo>> vecSRC = ReadMidFileByCurvature(strFile.c_str());
	for (int i = 0; i < vecSRC.size(); i++)
	{
		vector<NodeInfo>& vecNodeInfo = vecSRC[i];
		string strLinkId = vecNodeInfo.front().LinkID;

		for (int j = 0; j < vecNodeInfo.size(); j++)
		{
			NodeInfo& node = vecNodeInfo[j];
			if (fabs(node.getCurvature()) > 4e4)
			{
				cout << "strlineID:\t" << strLinkId << "\t曲率超限" << endl;
				break;
			}
		}

		for (int j = 0; j < vecNodeInfo.size(); j++)
		{
			NodeInfo& node = vecNodeInfo[j];
			if (fabs(node.getSlope()) > 3.5e4)
			{
				cout << "strlineID:\t" << strLinkId << "\t坡度超限" << endl;
				break;
			}
		}
		//////////////////////////////////////////////////////////////////////////
	}
}

void XCalculateKPI::ExportFile4lihua(string strSrcFile)
{
	vector<vector<NodeInfo>> vecNodes = ReadMidFile(strSrcFile.c_str());
	string strDir = strSrcFile.substr(0, strSrcFile.find_last_of("\\/") + 1);

	for (int i = 0; i < vecNodes.size();i++)
	{
		vector<NodeInfo>& vecNode = vecNodes[i];
		if (vecNode.empty())
			continue;
		NodeInfo& firstNode = vecNode.front();
		ofstream outFile(strDir + firstNode.LinkID.substr(1,firstNode.LinkID.rfind("\"")-1) + ".txt");

		outFile.precision(6);
		outFile.setf(ios::fixed);

		if (!outFile.is_open())
		{
			cout << "file not open !" << endl;
			continue;
		}
		outFile << vecNode.size() << endl;
		for (int j = 0; j < vecNode.size();j++)
		{
			NodeInfo& node = vecNode[j];
			//outFile << node.x.substr(1, node.x.rfind("\"") - 1) << " " << node.y.substr(1, node.y.rfind("\"") - 1) << " " << node.getWgs_Z() << endl;
			outFile << node.getOrigin_X() << " " << node.getOrigin_Y() << " " << node.getWgs_Z()/100.0 << endl;
		}
		outFile.close();
	}

}

void XCalculateKPI::ReplaceLinkSCH(string strSrcFile, string strDestFile, vector<string> vecStrings)
{
	XCalculateKPI  ComputeKpi; 
	vector<vector<NodeInfo>> vecNodeInfoDest = ComputeKpi.ReadMidFileByCurvature(strDestFile.c_str());
	vector<vector<NodeInfo>> vecNodeInfoSrc = ComputeKpi.ReadMidFileByCurvature(strSrcFile.c_str());

	vector < NodeInfo > vecSrc;

	/*vector<string> vecStrings = { "184748", "189854", "189873", "189900", "199579", "199804", "199819", "199829", "199846", "199869", "199870",
	"204216", "204230", "204311", "204315", "204343", "204382", "204508", "204517", "204567", "208183" };*/
	for (int k = 0; k < vecStrings.size(); k++)
	{
		for (int i = 0; i < vecNodeInfoSrc.size(); i++)
		{
			vecSrc = vecNodeInfoSrc[i];
			NodeInfo& node = vecSrc.front();
			if (node.getLinkId() == vecStrings[k])
				break;
		}

		if (vecSrc.empty())
		{
			cout << "can't find " << vecStrings[k] << "in src" << endl;
			continue;
		}

		for (int i = 0; i < vecNodeInfoDest.size(); i++)
		{
			vector < NodeInfo >& vecNode = vecNodeInfoDest[i];
			NodeInfo& node = vecNode.front();
			if (node.getLinkId() == vecStrings[k])
			{
				if (vecNode.size() != vecSrc.size())
				{
					cout << " size not equal " << endl;
					break;
				}

				for (int j = 0; j < vecNode.size(); j++)
				{
					vecNode[j].setSlope(vecSrc[j].getSlope());
				}

				cout << "node : " << node.getLinkId() << " finished" << endl;
				break;
			}
		}
	}

	ComputeKpi.WriteMidFileBySmoothCurvature(strDestFile.c_str(), vecNodeInfoDest);
}