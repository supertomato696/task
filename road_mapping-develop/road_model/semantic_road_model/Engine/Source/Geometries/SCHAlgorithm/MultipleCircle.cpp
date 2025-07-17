//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/MultipleCircle.h"
#include "Geometries/SCHAlgorithm/GParams.h"
#include "Geometries/SCHAlgorithm/fit.h"
#include "Geometries/SCHAlgorithm/FuzzySegment.h"
#include "Geometries/SCHAlgorithm/FuzzySegmentor.h"
//#include "gdalutil.h"
#include "Eigen/Dense"
#include "Geometries/SCHAlgorithm/LSF/circlefit.h"

#include <algorithm>
#include <map>
#include <iostream>
using namespace std;

const double C_PI = 3.1415926;
const double C_ANGLE_LIMIT = 20.0;
const double C_HCORRECT_VALUE = 60.0;
int C_ENLARGE_NUM = (int) (160/ GParams::C_GAP);
int C_MIN_NUM = (int) (25 / GParams::C_GAP);

MultipleCircle::MultipleCircle(void)
{
}

MultipleCircle::~MultipleCircle(void)
{
}

/*
* 计算两条线的夹角
* @param XS
* @param YS
* @param XM
* @param YM
* @param XE
* @param YE
* @return
*/
double MultipleCircle::calcAngel(double XS, double YS, double XM, double YM, double XE, double YE) {
	double FI = 0.0;
	double DSX = XS - XM;
	double DSY = YS - YM;
	double DEX = XE - XM;
	double DEY = YE - YM;
	double DOTVEC = DSX * DEX + DSY * DEY;
	double NORM  = (DSX * DSX + DSY * DSY) * (DEX * DEX + DEY * DEY);    //模乘积

	if(NORM != 0) {
		double COSVAL = DOTVEC / sqrt(NORM);
		FI = acos(COSVAL) * 180 / C_PI;
	}

	return FI;
}

//计算阈值
double MultipleCircle::getThreshold(int nCount) {
	double allowed_var = (2.5 / (double)GParams::C_GAP);

	if(nCount < (150 / (int)GParams::C_GAP)) {
		allowed_var = (10/ (double)GParams::C_GAP) - 0.05 * nCount;
	}

	return allowed_var;
}

/*
* 计算某条线的方位角
* @param A：起点X坐标
* @param B：起点Y坐标
* @param X：终点X坐标
* @param Y：终点Y坐标
* @return
*/
double MultipleCircle::calcAzimuth(double A, double B, double X, double Y) {
	double aziMuth = 0.0;
	aziMuth = atan2(X - A, Y - B);

	return aziMuth;
}

void MultipleCircle::getProperNodes(vector<NodeInfo>& vecNodes, vector<NodeInfo>& properNodes, int nIndex, int IntialExtendNum, int& nInnerIndex)
{
	int nLeftEnd = 0;
	int nRightEnd = vecNodes.size();

	NodeInfo& currentNode = vecNodes[nIndex];
	//两边都越界
	if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size()) 
	{
		nInnerIndex = nIndex;
	}
	//起点单侧越界
	else if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 <= vecNodes.size()) 
	{
		nRightEnd = nIndex + IntialExtendNum + 1;
		nInnerIndex = nIndex;
	}
	//终点单侧越界
	else if (nIndex >= IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size()) 
	{
		nLeftEnd = nIndex - IntialExtendNum;
		nIndex = IntialExtendNum;
	}
	//两侧都未越界
	else 
	{
		nLeftEnd = nIndex - IntialExtendNum;
		nRightEnd = nIndex + IntialExtendNum + 1;
		nIndex = IntialExtendNum;
	}

	//2016.7.4 以下代码 暂时屏蔽
	/*
	增加相邻分段夹角控制，如果相邻分段夹角超过限定值，则不启用当前
	*/
	//const double INCLUDE_ANGLE_LIMIT = 150;
	//
	//int  nR = nIndex+1;
	//FuzzySegment* pCurrentSegment = currentNode.pSegment;
	//
	//while (nR < nRightEnd)
	//{
	//	NodeInfo& nodeInfo = vecNodes[nR];
	//	if (pCurrentSegment == nodeInfo.pSegment)
	//	{
	//		nR++; continue;
	//	}

	//	double dAngle = pCurrentSegment->IncludeAngle(nodeInfo.pSegment,vecNodes);
	//	
	//	if (dAngle < INCLUDE_ANGLE_LIMIT)
	//	{
	//	//nodeInfo.bTurnFlag = true;
	//	//	cout << "区    间 " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex << endl;
	//	//	cout << "与区间 " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
	//	//	cout << "右侧角度超限:\t" << dAngle << endl;
	//		break;
	//	}

	//	pCurrentSegment = nodeInfo.pSegment;
	//	nR++;
	//}
	//

	//pCurrentSegment = currentNode.pSegment;
	//int nL = nIndex;
	//while (nL > nLeftEnd)
	//{
	//	NodeInfo& nodeInfo = vecNodes[nL];
	//	if (pCurrentSegment == nodeInfo.pSegment)
	//	{
	//		nL--;continue;
	//	}
	//
	//	double dAngle = pCurrentSegment->IncludeAngle(nodeInfo.pSegment,vecNodes);
	//
	//	if (dAngle < INCLUDE_ANGLE_LIMIT)
	//	{
	//		nodeInfo.bTurnFlag = true;
	//	//	cout << "区    间 " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex << endl;
	//	//	cout << "与区间 " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
	//	//	cout << "左侧角度超限:\t" << dAngle << endl;
	//		break;
	//	}

	//	pCurrentSegment = nodeInfo.pSegment;
	//	nL--;
	//}
	//
	//properNodes.assign(vecNodes.begin() + nL, vecNodes.begin() + nR);

	properNodes.assign(vecNodes.begin() + nLeftEnd, vecNodes.begin() + nRightEnd);
}

/*
* 功能：核心函数，计算扩展数为enlargeNum时的曲率，检测扩展数为enlargeNum时，均方差是否超限，是否有连续两点超过三倍均方差阈值
* @param nodes:      路链串
* @param index:      路链上第几个点
* @param enlargeNum: 扩展点数
* @return            :ResultSet  <isPermitted:是否未超阈值，curvature:曲率>
*/
 //ResultSet MultipleCircle::checkIsPermitted(vector<NodeInfo>& nodes, int index, int enlargeNum, bool isFinal)
 //{
 //	double X1=0;
 //	double Y1=0;
 //	double X2=0;
 //	double Y2=0;
 //	double X3=0;
 //	double Y3=0;
 //	double X1Y1=0;
 //	double X1Y2=0;
 //	double X2Y1=0;
 //	double calc_X = 0, calc_Y = 0;
 //
 //	vector<NodeInfo> properNodes;
 //
 //	/*取参与拟合的计算点,如果超过路链的起点或终点，就从起点或终点开始取*/
 //	getProperNodes(nodes,properNodes, index, enlargeNum);
 //
 //	NodeInfo& currentNode = nodes[index];
 //
 //	/*最小二乘法计算圆心坐标，圆半径*/
 //
 //	for (int i = 0;i < properNodes.size();i++)	 
 //	{
 //		NodeInfo& properNode = properNodes[i];
 //
 //		if(1 == properNode.getSingle_flag()) 
 //		{		    		
 //			calc_X = properNode.getReference_X();
 //			calc_Y = properNode.getReference_Y();
 //		}
 //		else 
 //		{
 //			calc_X = properNode.getReference_X();
 //			calc_Y = properNode.getReference_Y();
 //		}
 //
 //		X1 = X1 + calc_X;
 //		Y1 = Y1 + calc_Y;
 //		X2 = X2 + calc_X * calc_X;
 //		Y2 = Y2 + calc_Y * calc_Y;
 //		X3 = X3 + calc_X * calc_X * calc_X;
 //		Y3 = Y3 + calc_Y * calc_Y * calc_Y;
 //		X1Y1 = X1Y1 + calc_X * calc_Y;
 //		X1Y2 = X1Y2 + calc_X * calc_Y * calc_Y;
 //		X2Y1 = X2Y1 + calc_X * calc_X * calc_Y;
 //	}
 //
 //	double C,D,E,G,H;
 //	int N;
 //	double a,b,c;
 //	N = properNodes.size();
 //	C = N*X2 - X1*X1;
 //	D = N*X1Y1 - X1*Y1;
 //	E = N*X3 + N*X1Y2 - (X2+Y2)*X1;
 //	G = N*Y2 - Y1*Y1;
 //	H = N*X2Y1 + N*Y3 - (X2+Y2)*Y1;
 //	double curvature = 0.0;
 //	double heading = 0.0;
 //	ResultSet result;  
 //	bool isOnOneSeg = false;
 //
 //	if((properNodes.at(0).getSeg_belong() == properNodes.at(N-1).getSeg_belong()) && (properNodes.at(0).getSeg_seq() < properNodes.at(N-1).getSeg_seq())) {
 //		isOnOneSeg = true;
 //	}
 //
 //	/**/
 //
 //	if((C*G != D*D) && (!isOnOneSeg)) 
 //	{
 //		/*最小二乘法计算圆心坐标，圆半径*/
 //
 //		a = (H*D-E*G)/(C*G-D*D);
 //		b = (H*C-E*D)/(D*D-G*C);
 //		c = -(a*X1 + b*Y1 + X2 + Y2)/N;
 //
 //		//圆心坐标(A,B), 圆半径R
 //		double A,B,R;
 //		A = a/(-2);
 //		B = b/(-2);
 //		R = sqrt(a*a+b*b-4*c)/2;

	//	//double r = CalcRadius(properNodes);

	//	
 //
 //		/**/
 //
 //		/*计算每个点距圆心距离与半径的差值*/
 //
 //		vector<double> point_distances;
 //		double dis_point, dis_avg, dis_var;
 //		double dis_sum = 0.0;
 //
 //		for (int i = 0;i < properNodes.size();i++)	 
 //		{
 //			NodeInfo& properNode = properNodes[i];
 //
 //			if(1 == properNode.getSingle_flag()) 
	//		{
 //				calc_X = properNode.getReference_X();
 //				calc_Y = properNode.getReference_Y();
 //			}
 //			else
	//		{
 //				calc_X = properNode.getReference_X();
 //				calc_Y = properNode.getReference_Y();
 //			}
 //
 //			dis_point = abs(sqrt(pow(calc_X-A,2) + pow(calc_Y-B,2)) - R);
 //			dis_sum = dis_sum + pow(dis_point, 2);
 //			point_distances.push_back(dis_point);
 //		}
 //
 //		/**/
 //
 //		dis_avg = dis_sum / N;
 //
 //		/*均方差*/
 //
 //		double dis_var_sum = 0.0;
 //
 //		for (int i = 0;i < point_distances.size();i++)	 
	//	{
 //			double point_distance = point_distances[i];
 //			dis_var_sum = dis_var_sum + pow(point_distance - dis_avg, 2);
 //		}
 //
 //		dis_var = sqrt(dis_var_sum / N);
 //
 //		//计算扩点数为enLarge时的圆心角
 //		NodeInfo node1 = properNodes.at(0);
 //		NodeInfo node2 = properNodes.at(N - 1);
 //		double circleAngel = calcAngel(node1.getReference_X(), node1.getReference_Y(), A, B, node2.getReference_X(), node2.getReference_Y());
 //
 //		//阈值判定，一开始为允许		    
 //		double threshold = getThreshold(N);
 //
	//	if (dis_var > threshold /*|| circleAngel > C_ANGLE_LIMIT*/)
	//	{
	//		result.setPermitted(false);
	//		if(dis_var > threshold)
	//			result.errType = result.errType | 0x01;

	//		result.dAvg = dis_avg;
	//		result.dSD = dis_var;

	//		//cout << "SD\t" << dis_var << "\tAVG\t" << dis_avg << endl;
	//		/*	if (circleAngel > C_ANGLE_LIMIT)
	//				result.errType = result.errType | 0x02;*/
	//	}
 //		else 
 //		{
 //			for(int i = 0; i < point_distances.size()-1; i++) 
 //			{
 //				if(point_distances.at(i) > 3 * threshold && point_distances.at(i+1) > 3 * threshold) 
 //				{
 //					result.setPermitted(false);
	//				result.errType = result.errType | 0x04;
 //					break;
 //				}
 //			}
 //		}
 //
 //		//算曲率的正负方向
 //		double DXL = 0.0, DYL = 0.0, DXN = 0.0, DYN = 0.0;
 //		double DA, DB;	
 //
 //		double nextX, nextY, lastX, lastY;
 //
 //		//大致估计航向的范围，避免算反
 //		double tangentialAngle;
 //
 //		lastX = calc_X;
 //		lastY = calc_Y;
 //		nextX = calc_X;
 //		nextY = calc_Y;
 //
 //		//在阈值范围内才计算
 //		if(result.IsPermitted() || isFinal) 
 //		{
 //			if(1 == nodes.at(index).getSingle_flag()) 
	//		{
 //				calc_X = nodes.at(index).getReference_X();
 //				calc_Y = nodes.at(index).getReference_Y();	    	   
 //			}
 //			else 
	//		{
 //				calc_X = nodes.at(index).getReference_X();
 //				calc_Y = nodes.at(index).getReference_Y();
 //			}
 //
 //			lastX = calc_X;
 //			lastY = calc_Y;
 //			nextX = calc_X;
 //			nextY = calc_Y;
 //
 //			if(index >= 1) 
	//		{
 //				if(1 == nodes.at(index - 1).getSingle_flag()) 
	//			{
 //					lastX = nodes.at(index - 1).getReference_X();
 //					lastY = nodes.at(index - 1).getReference_Y();
 //					DXL = calc_X - lastX;
 //					DYL = calc_Y - lastY;
 //				}
 //				else 
	//			{
 //					lastX = nodes.at(index - 1).getReference_X();
 //					lastY = nodes.at(index - 1).getReference_Y();
 //					DXL = calc_X - lastX;
 //					DYL = calc_Y - lastY;
 //				}
 //			}
 //
 //			if(index < nodes.size() - 1) 
	//		{
 //				if(1 == nodes.at(index + 1).getSingle_flag()) 
	//			{
 //					nextX = nodes.at(index + 1).getReference_X();
 //					nextY = nodes.at(index + 1).getReference_Y();
 //					DXN = nextX - calc_X;
 //					DYN = nextY - calc_Y;
 //				}
 //				else 
	//			{
 //					nextX = nodes.at(index + 1).getReference_X();
 //					nextY = nodes.at(index + 1).getReference_Y();
 //					DXN = nextX - calc_X;
 //					DYN = nextY - calc_Y;
 //				}
 //			}		       
 //
 //			DA = calc_X - A;
 //			DB = calc_Y - B;
 //
 //			//曲率方向，若圆为向逆时针走则为负，方位角的弧度值不断增大，第二，三象限交界处需做特殊判断
 //
	//		NodeInfo& sNode = nodes[currentNode.pSegment->m_nStartIndex];
	//		NodeInfo& eNode = nodes[currentNode.pSegment->m_nEndIndex];

	//		double v0_x = sNode.getReference_X() - A;
	//		double v0_y = sNode.getReference_Y() - B;
	//		double v1_x = eNode.getReference_X() - A;
	//		double v1_y = eNode.getReference_Y() - B;

	//		double z = v0_x*v1_y - v0_y*v1_x;
	//		if (z < 0)
	//			curvature = -1.0 / R;
	//		else
	//			curvature = 1.0 / R;

	//		double  v2N_x = calc_Y - B;
	//		double v2N_y = A - calc_X;

	//		double v3_x = eNode.getReference_X() - sNode.getReference_X();
	//		double v3_y = eNode.getReference_Y() - sNode.getReference_Y();

	//		double dot23 = v2N_x*v3_x + v2N_y*v3_y;
	//		if (dot23 < 0)
	//		{
	//			v2N_x = -v2N_x;
	//			v2N_y = -v2N_y;
	//		}
	//		
	//		heading = calcAzimuth(0,0,v2N_x, v2N_y) * 180 / 3.141593653;
 //		}
 //	}
 //	//C*G == D*D 或在同一分段上
 //	else 
 //	{
 //		//计算航向
 //		double DXL = 0.0, DXN = 0.0;		
 //		double nextX = 0.0, nextY = 0.0, lastX = 0.0, lastY = 0.0;
 //
 //		//在阈值范围内才计算
 //		if(result.IsPermitted() || isFinal) {
 //			if(1 == nodes.at(index).getSingle_flag()) {
 //				calc_X = nodes.at(index).getReference_X();
 //				calc_Y = nodes.at(index).getReference_Y();	    	   
 //			}
 //			else {
 //				calc_X = nodes.at(index).getReference_X();
 //				calc_Y = nodes.at(index).getReference_Y();
 //			}
 //
 //			if(index >= 1) {
 //				if(1 == nodes.at(index - 1).getSingle_flag()) {
 //					lastX = nodes.at(index - 1).getReference_X();
 //					lastY = nodes.at(index - 1).getReference_Y();
 //					DXL = calc_X - lastX;
 //				}
 //				else {
 //					lastX = nodes.at(index - 1).getReference_X();
 //					lastY = nodes.at(index - 1).getReference_Y();
 //					DXL = calc_X - lastX;
 //				}
 //			}
 //
 //			if(index < nodes.size() - 1) {
 //				if(1 == nodes.at(index + 1).getSingle_flag()) {
 //					nextX = nodes.at(index + 1).getReference_X();
 //					nextY = nodes.at(index + 1).getReference_Y();
 //					DXN = nextX - calc_X;
 //				}
 //				else {
 //					nextX = nodes.at(index + 1).getReference_X();
 //					nextY = nodes.at(index + 1).getReference_Y();
 //					DXN = nextX - calc_X;
 //				}
 //			}		       
 //
 //			if(DXL >= 0 && DXN > 0) {
 //				heading = abs(calcAngel(calc_X, calc_Y + 1, calc_X, calc_Y, nextX, nextY));
 //			}
 //			else if(DXL > 0 && DXN >= 0) {
 //				heading = abs(calcAngel(lastX, lastY + 1, lastX, lastY, calc_X, calc_Y));
 //			}
 //			else if(DXL <= 0 && DXN < 0) {
 //				heading = 360 - abs(calcAngel(calc_X, calc_Y + 1, calc_X, calc_Y, nextX, nextY));
 //			}
 //			else {
 //				heading = 360 - abs(calcAngel(lastX, lastY + 1, lastX, lastY, calc_X, calc_Y));
 //			}
 //		}
 //	}
 //
 //	if(heading < 0) {
 //		heading = 360 + heading;
 //	}
 //
 //	if(heading >= 360) {
 //		heading = heading - 360;
 //	}
 //
 //	result.setCurvature(curvature);
 //	result.setHeading(heading);
 //
 //	return result;
 //}

/*
* Function_Name:getProperEnlarge
* 实现功能：通过用二分法判定是否超限，获得合适的扩展点的个数
* @param nodes
*/
int MultipleCircle::getProperEnlarge(vector<NodeInfo>& nodes, int index, int top, int low,ResultSet& result)
{
	int topCheck = top;
	int lowCheck = low;

	int nTab = 0;

	while(topCheck - lowCheck >= 2) {    //二分法终止条件
		result = checkIsPermitted(nodes, index, (topCheck + lowCheck)/2, false);	
		
	/*	nTab++;
		for (int i = 0; i < nTab; i++)
		{
			cout << "\t";
		}
		cout << "\tsd:\t" << result.dSD << endl;*/

		if(result.IsPermitted()) {
			lowCheck = (topCheck + lowCheck) / 2;      //没有超过阈值则下限上移
		}		    
		else {
			topCheck = (topCheck + lowCheck) / 2;      //超过阈值则上先下移
		}
	}

	return (topCheck + lowCheck - 1) / 2;              //返回	    
}

/*
* Function_Name: tagLongSegHeading
* 实现功能：对长线段中间点的航向特殊处理，取两端航向非0的平均值进行标注
* @param nodes
* @return
*/
void MultipleCircle::tagLongSegHeading(vector<NodeInfo>& nodes) 
{
	int index = 0, fromIndex = 0, endIndex = 0;
	double heading1 = 0.0, heading2 = 0.0;

	while(index < nodes.size() - 1) 
	{
		if(0 == nodes.at(index).getLongseg_flag() && 1 == nodes.at(index+1).getLongseg_flag()) 
		{
			fromIndex = index;
			heading1 = nodes.at(index).getHeading();
		}
		else if(1 == nodes.at(index).getLongseg_flag() && 0 == nodes.at(index+1).getLongseg_flag()) 
		{
			endIndex = index + 1;
			heading2 = nodes.at(index+1).getHeading();

			for(int i = fromIndex; i < endIndex; i++) 
			{
				if(1 == nodes.at(i).getLongseg_flag()) 
				{
					nodes.at(i).setHeading((heading1 + heading2) / 2);
				}
			}
		}

		index++;
	}
}

bool MultipleCircle::smoothArrayNodeByLineInterpolation(vector<NodeInfo> &vecNodeInfo, map<int, int> & mapIndex, bool absSmooth)
{
	if (vecNodeInfo.size() < 1 )
	{
		return false;
	}
	
	map<int,int>::iterator  itmap;
	for (itmap = mapIndex.begin(); itmap != mapIndex.end(); itmap++)
	{
		int nStartIndex = itmap->first;
		int nEndIndex = itmap->second;

		int nNodeSize = vecNodeInfo.size();
		if(!(nStartIndex >= 0 && nStartIndex < nEndIndex && nStartIndex < nNodeSize && nEndIndex >= 0 && nEndIndex < nNodeSize))
			return false;

		double dLittleValue = vecNodeInfo[nStartIndex].getCurvature();
		double dTopValue = vecNodeInfo[nEndIndex].getCurvature();

		if (absSmooth)
		{
			//dongjian 
			dLittleValue = std::abs(dLittleValue);
			dTopValue = std::abs(dTopValue);
		}
		
		for (int index = nStartIndex + 1; index < nEndIndex ; index++)
		{
			double dCurvatureValue = dLittleValue + (double)(index - nStartIndex) / (nEndIndex - nStartIndex) * (dTopValue - dLittleValue);

			if (absSmooth)
			{
				//两边同号就修改为同号（防止符号跳变）
				//两边异号就采用自己本来符号
				if (vecNodeInfo[nStartIndex].bNegtive == vecNodeInfo[nEndIndex].bNegtive)
					vecNodeInfo[index].bNegtive = vecNodeInfo[nStartIndex].bNegtive;

				if (vecNodeInfo[index].bNegtive && dCurvatureValue > 0)
					dCurvatureValue = -dCurvatureValue;
			}
			else
			{
				vecNodeInfo[index].bNegtive = dCurvatureValue < 0;
			}
			
			vecNodeInfo[index].setCurvature(dCurvatureValue);
		}	
	}

	return true;
}

/*
* Function_Name:tagPathCurveByDicho
* 实现功能：用二分法对单条路链的曲率进行标注
* @param nodes
*/
//void MultipleCircle::tagPathCurveByDicho(vector<NodeInfo>& nodes)
//{
//	if (nodes.empty())
//		return;
//
//	cout << "Node:\t" << nodes.front().getLinkId() << endl;
//
//	vector<double > v_x;
//	vector<double> v_y;
//	vector<double> v_r;
//
//
//
//	int index = 0;
//
//	//////////////////////////////////////////////////////////////////////////
//	//Only 2 points 
//	if(nodes.size() == 2) 
//	{
//		double X1, X2, Y1, Y2;
//		double heading = 0.0;
//		X1 = nodes.at(0).getOrigin_X();
//		X2 = nodes.at(1).getOrigin_X();
//		Y1 = nodes.at(0).getOrigin_Y();
//		Y2 = nodes.at(1).getOrigin_Y();
//
//		if(X2-X1 > 0) 
//		{
//			heading = abs(calcAngel(X1, Y1 + 1, X1, Y1, X2, Y2));
//		}
//		else 
//		{
//			heading = 360 - abs(calcAngel(X1, Y1 + 1, X1, Y1, X2, Y2));
//		}
//
//		for (int i = 0;i < nodes.size();i++)	
//		{
//			NodeInfo& node = nodes[i];
//			node.setCurvature(0.0);
//			node.bNegtive = node.getCurvature() < 0;
//			node.setR_Value(99999999);
//			node.setHeading(heading);
//		}
//	}
//	//////////////////////////////////////////////////////////////////////////
//	// 3- 5 Points 
//	else if(nodes.size() >= 3 && nodes.size() < 5) 
//	{
//		index = 0;
//
//		for (int i = 0;i < nodes.size();i++)	 
//		{
//			NodeInfo& node = nodes[i];
//			ResultSet result;
//			result = checkIsPermitted(nodes, index, 5, true);
//			double curvature = result.getCurvature();
//			node.setCurvature(curvature);                //设定曲率为全段参与拟合计算时的曲率
//			node.bNegtive = node.getCurvature() < 0;
//			node.setHeading(result.getHeading());
//
//			if(abs(curvature) > 0.00000001) 
//			{
//				node.setR_Value(abs(1 / curvature));
//			}
//			else 
//			{
//				node.setR_Value(99999999);
//			}
//		}
//	}
//	//////////////////////////////////////////////////////////////////////////
//	// Greater than 5 points
//	//路链总点数大于等于5才参与计算
//	else if(nodes.size() >= 5) 
//	{                                
//		for (int i = 0;i < nodes.size();i++)	 
//		{
//			NodeInfo& node = nodes[i];
//
//			//非长直线点和非虚拟点才用计算曲率
//			if(0 == node.getLongseg_flag()) 
//			{                   	
//				double curvature = 0.0;
//				ResultSet result;
//
//				//如果扩32个点时阈值未超限
//				result = checkIsPermitted(nodes, i, C_ENLARGE_NUM, false);
//
//				if(result.IsPermitted()) 
//				{
//					//设置扩点数为32
//					node.setEnlarge_num(C_ENLARGE_NUM);
//
//					//设定曲率为32个点时的曲率
//					node.setCurvature(result.getCurvature());
//					node.bNegtive = node.getCurvature() < 0;
//
//					node.setHeading(result.getHeading());
//
//					cout << "index:\t" << i << "\tNum:\t" << node.getEnlarge_num() << "\tAvg:\t" << result.dAvg << "\tSD:\t" << result.dSD << endl;
//				}
//				else 
//				{
//					//找到合适的扩点数，初始上限32，下限0
//					//cout << "index " << i << "--------拟合过程---------------" << endl;
//
//					ResultSet result;
//					int properEnlarge = getProperEnlarge(nodes, i, C_ENLARGE_NUM, 0,result);   
//
//					//cout << "--------------------------------------------" << endl;
//
//					if(properEnlarge < 10/*C_MIN_NUM*/) {
//						//至少两边各向外扩5个点
//						properEnlarge = 10/*C_MIN_NUM*/;     
//						cout << " 点数较少" << i << endl;
//					}
//
//					//设置扩点数为合适的扩点数
//					node.setEnlarge_num(properEnlarge);
//
//					result = checkIsPermitted(nodes, i, properEnlarge, true);
//
//					cout << "index:\t" << i << "\tNum:\t" << node.getEnlarge_num() << "\tAvg:\t" << result.dAvg << "\tSD:\t" << result.dSD << endl;
//
//					if (!result.IsPermitted())
//					{
//						v_x.push_back(result.x);
//						v_y.push_back(result.y);
//						v_r.push_back(result.r);
//					}
//
//					//设定曲率为合适扩点时的曲率
//					node.setCurvature(result.getCurvature());
//					node.bNegtive = node.getCurvature() < 0;
//
//					node.setHeading(result.getHeading());
//				}
//
//				curvature = result.getCurvature();
//
//				if(abs(curvature) > 0.00000001) {
//					node.setR_Value(abs(1 / curvature));
//				}
//				else {
//					node.setR_Value(99999999);
//				}
//			}
//			else {
//				node.setR_Value(99999999);
//			}
//		}
//
//		tagLongSegHeading(nodes);
//	}
//	//////////////////////////////////////////////////////////////////////////
//	//
//	else 
//	{
//		std::cout << "we got an error here" << std::endl;
//	}
//
//	//if (!v_x.empty())
//		//gdalutil::ExportCircle(v_x, v_y, v_r, "d:/data/shp/" + nodes.front().getLinkId() + ".shp");
//}

bool SortListLengthCmp(const MultipleCircle::ITEM& item0, const MultipleCircle::ITEM& item1)
{
	return item0.nLength < item1.nLength;
}

void MultipleCircle::removeSignJump(vector<NodeInfo>& nodes,int  C_LENGTH_LIMIT)
{
	if (nodes.empty())
		return;

	typedef struct  
	{
		int nStartIndex;
		int nEndIndex;
		bool bNegetive;
	} SignInterval;

	NodeInfo& FrontNode = nodes.front();

	vector<SignInterval> vecSignIntervals;
	SignInterval interval;
	interval.nStartIndex = 0;
	interval.nEndIndex = 0;
	interval.bNegetive = FrontNode.bNegtive;
	
	//////////////////////////////////////////////////////////////////////////
	//SUMMARY ALL NODES
	for (int i = 1; i < nodes.size();i++)
	{
		NodeInfo& node = nodes[i];
		//NODE 与 区间同向
		if (node.bNegtive == interval.bNegetive)
		{
			interval.nEndIndex = i;
		}
		else
		{
			///// SUMMARY 
			vecSignIntervals.push_back(interval);
			//RESET
			interval.nStartIndex = i;
			interval.nEndIndex = i;
			interval.bNegetive = node.bNegtive;
		}
	}

	//LAST INTERVAL
	vecSignIntervals.push_back(interval);

	vector<ITEM> sortList;
	for (int i = 0; i < vecSignIntervals.size();i++)
	{
		int nLength = vecSignIntervals[i].nEndIndex - vecSignIntervals[i].nStartIndex;
		ITEM item;
		item.nIndex = i;
		item.nLength = nLength;
		sortList.push_back(item);
	}

	std::sort(sortList.begin(), sortList.end(), SortListLengthCmp);

	//const int  = 15;
	for (int i = 0; i < sortList.size();i++)
	{
		ITEM item = sortList[i];
		if (item.nLength > C_LENGTH_LIMIT)
			break;

		bool sign = vecSignIntervals[item.nIndex].bNegetive;
		
		bool rightSign;
		bool hasRight = item.nIndex + 1 < vecSignIntervals.size();
		if (hasRight)
			rightSign = vecSignIntervals[item.nIndex + 1].bNegetive;
		
		bool leftSign;
		bool hasLeft = item.nIndex > 0;
		if (hasLeft)
			leftSign = vecSignIntervals[item.nIndex - 1].bNegetive;
		
		if (hasLeft && hasRight && (sign != leftSign && sign != rightSign))
		{
			vecSignIntervals[item.nIndex].bNegetive = leftSign;
		}

	}

	//////////////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////
	for (int i = 0; i < vecSignIntervals.size(); i++)
	{
		SignInterval& interval = vecSignIntervals[i];
		for (int j = interval.nStartIndex; j <= interval.nEndIndex;j++)
		{
			NodeInfo& node = nodes[j];
			if (node.bNegtive != interval.bNegetive)
			{
				node.bNegtive = interval.bNegetive;
				node.setCurvature(node.getCurvature()*-1);
			}
		}
	}
}

int MultipleCircle::FindStablePosition(vector<NodeInfo>& nodes, int nIndex, int MaxExtend, bool bLeft /* = true */)
{
	vector<bool> vecMonotone;
	vector<double> vecValue;
	if (bLeft)
	{
		int nLeft = nIndex;
		while (nLeft >= 0 && --MaxExtend >= 0)
		{
			double d1 = abs(nodes[nLeft].getCurvature());
			int nPre = nIndex == nLeft ? nIndex : nLeft - 1;
			double d0 = abs(nodes[nPre].getCurvature());

			vecValue.push_back(d1 - d0);
			nLeft--;
		}
		int i = 0;
		for (i = vecValue.size() - 1; i > 0; i--)
		{
			if (abs(vecValue[i]) > 10)
				break;
		}

		return nIndex -i;
	}
	else
	{
		int nRight = nIndex;
		while (nRight < nodes.size() && --MaxExtend >=0)
		{
			double d1 = abs(nodes[nRight].getCurvature());
			int nPre = nIndex == nRight ? nIndex : nRight - 1;
			double d0 = abs(nodes[nPre].getCurvature());

			vecValue.push_back(d1 - d0);
			nRight++;
		}

		int i = 0;
		for (i = vecValue.size() - 1; i > 0; i--)
		{
			if (abs(vecValue[i]) > 10)
				break;
		}

		return nIndex+i;
	}
}

void MultipleCircle::SmoothPositiveNegtiveJump(vector<NodeInfo>& nodes, int SmoothPoints)
{
	map<int, int> mapJumpIntervals;
	for (int i = 0; i < nodes.size()-1; i++)
	{
		if (nodes[i].bNegtive == nodes[i+1].bNegtive)
			continue;

		//2016.7.4 
		/*int nStart = FindStablePosition(nodes, i, SmoothPoints, true);
		int nEnd = FindStablePosition(nodes, i, SmoothPoints, false);*/
		int nStart = i - SmoothPoints > 0 ? i - SmoothPoints : 0;
		int nEnd = i + SmoothPoints > nodes.size() ? nodes.size() - 1 : i + SmoothPoints;
		/*
		二次插值
		*/
		std::vector<double> vecX;
		std::vector<double> vecY;

		//vecX.push_back(nStart - 1);
		vecX.push_back(nStart);
		vecX.push_back(nStart + 1);
		vecX.push_back(nEnd - 1);
		vecX.push_back(nEnd);
		
		/*if (nEnd +1 < nodes.size())
			vecX.push_back(nEnd+1);*/

		//vecY.push_back(nodes[nStart - 1].getCurvature());
		vecY.push_back(nodes[nStart].getCurvature());
		vecY.push_back(nodes[nStart + 1].getCurvature());
		vecY.push_back(nodes[nEnd - 1].getCurvature());
		vecY.push_back(nodes[nEnd].getCurvature());

		/*	if (nEnd + 1 < nodes.size())
				vecY.push_back(nodes[nEnd + 1].getCurvature());*/

		czy::Fit fit;
		fit.polyfit(vecX, vecY, 3);

		for (int index = nStart + 1; index < nEnd; index++)
		{
			NodeInfo& node = nodes[index];
			double dCurvature = fit.getY(index);
			node.setCurvature(dCurvature);
			node.bNegtive = dCurvature < 0;
		}

		//mapJumpIntervals.insert(std::make_pair(nStart, nEnd));
	}
	//smoothArrayNodeByLineInterpolation(nodes, mapJumpIntervals,false);
}


void MultipleCircle::CalcRadius(vector<NodeInfo>& vecNodes, int nIndex, double& X_R, double& Y_R, double& R, double& SD)
{
	if (!CalcR(vecNodes, X_R, Y_R,R,SD))
	{
		cout << "index:\t"<<nIndex<<"\t引进算法失效，试用其他方法计算" << endl;
		//CalcRadius_WrongLSF(vecNodes, nIndex, X_R, Y_R, R, SD);
		SD = numeric_limits<double>::max();
	}

	//CalcStandardDeviation(vecNodes, nIndex, X_R,Y_R,R,SD);
}

void MultipleCircle::CalcRadius_WrongLSF(vector<NodeInfo>& vecNodes,int nIndex,double& X_R, double& Y_R, double& R,double& SD)
{
	double x2 = 0;
	double xy = 0;
	double y2 = 0;
	double x_s = 0;
	double y_s = 0;
	double c = 0;
	double x3 = 0;
	double x2y = 0;
	double xy2 = 0;
	double y3 = 0;

	int n = vecNodes.size();
	int nLongAxis = std::max(nIndex, n - nIndex);
	for (int i = 0; i < n;i++)
	{
		NodeInfo& node = vecNodes[i];
		double x = node.getReference_X();
		double y = node.getReference_Y();
		double radio = 1.0 - (abs(i - nIndex) / (double)(nLongAxis));
		double w = 1;

		if (radio > 0.6)
			w = 2;

		if (radio > 0.9)
			w = 3;

		if (radio > 0.95)
			w = 10;

		x2 += x*x*w;
		xy += x*y*w;
		y2 += y*y*w;
		x_s += x*w;
		y_s += y*w;
		c += 1*w;
		x3 += x*x*x*w;
		x2y += x*x*y*w;
		xy2 += x*y*y*w;
		y3 += y*y*y*w;
	}

	Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, 3);
	A(0, 0) = x2;
	A(0, 1) = xy;
	A(0, 2) = x_s;
	A(1, 0) = xy;
	A(1, 1) = y2;
	A(1, 2) = y_s;
	A(2, 0) = x_s;
	A(2, 1) = y_s;
	A(2, 2) = c;

	Eigen::MatrixXd B = Eigen::MatrixXd::Zero(3, 1);
	B(0, 0) = -(x3 + xy2);
	B(1, 0) = -(x2y + y3);
	B(2, 0) = -(x2 + y2);

	Eigen::MatrixXd X = A.lu().solve(B);

	double p = X(0, 0);
	double q = X(1, 0);
	double v = X(2, 0);

	R = sqrt(p*p + q*q - 4 * v) / 2;
	X_R = -p / 2.0;
	Y_R = -q / 2.0;
}

ResultSet MultipleCircle::checkIsPermitted(vector<NodeInfo>& nodes, int index, int enlargeNum, bool isFinal)
{
	double calc_X = 0, calc_Y = 0;

	vector<NodeInfo> properNodes;

	/*取参与拟合的计算点,如果超过路链的起点或终点，就从起点或终点开始取*/
	int nInnerIndex = -1;
	getProperNodes(nodes, properNodes, index, enlargeNum,nInnerIndex);

	NodeInfo& currentNode = nodes[index];

	/*最小二乘法计算圆心坐标，圆半径*/

	for (int i = 0; i < properNodes.size(); i++)
	{
		NodeInfo& properNode = properNodes[i];
		calc_X = properNode.getReference_X();
		calc_Y = properNode.getReference_Y();
	}

	double curvature = 0.0;
	double heading = 0.0;
	ResultSet result;
	bool isOnOneSeg = false;
	int N = properNodes.size();

	if ((properNodes.at(0).getSeg_belong() == properNodes.at(N - 1).getSeg_belong()) && 
		(properNodes.at(0).getSeg_seq() < properNodes.at(N - 1).getSeg_seq())) 
	{
		isOnOneSeg = true;
	}

	/**/

	if ((!isOnOneSeg))
	{
		/*最小二乘法计算圆心坐标，圆半径*/
		double A, B, R,SD;
		CalcRadius(properNodes,nInnerIndex, A, B, R,SD);

		result.x = A;
		result.y = B;
		result.r = R;
		result.dSD = SD;
		
		NodeInfo node1 = properNodes.at(0);
		NodeInfo node2 = properNodes.at(N - 1);
		double circleAngel = calcAngel(node1.getReference_X(), node1.getReference_Y(), A, B, node2.getReference_X(), node2.getReference_Y());

		
		//阈值判定，一开始为允许		    
		double threshold = getThreshold(N);

		if (SD > threshold )
		{
			result.setPermitted(false);
			if (SD > threshold)
				result.errType = result.errType | 0x01;
		//	cout << "平均距离:\t" << dis_avg << "\t均方差:\t" << dis_var << endl;
		}

		//算曲率的正负方向
		double DXL = 0.0, DYL = 0.0, DXN = 0.0, DYN = 0.0;
		double DA, DB;

		double nextX, nextY, lastX, lastY;

		//大致估计航向的范围，避免算反
		double tangentialAngle;

		lastX = calc_X;
		lastY = calc_Y;
		nextX = calc_X;
		nextY = calc_Y;

		//在阈值范围内才计算
		if (result.IsPermitted() || isFinal)
		{
			if (1 == nodes.at(index).getSingle_flag())
			{
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}
			else
			{
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}

			lastX = calc_X;
			lastY = calc_Y;
			nextX = calc_X;
			nextY = calc_Y;

			if (index >= 1)
			{
				if (1 == nodes.at(index - 1).getSingle_flag())
				{
					lastX = nodes.at(index - 1).getReference_X();
					lastY = nodes.at(index - 1).getReference_Y();
					DXL = calc_X - lastX;
					DYL = calc_Y - lastY;
				}
				else
				{
					lastX = nodes.at(index - 1).getReference_X();
					lastY = nodes.at(index - 1).getReference_Y();
					DXL = calc_X - lastX;
					DYL = calc_Y - lastY;
				}
			}

			if (index < nodes.size() - 1)
			{
				if (1 == nodes.at(index + 1).getSingle_flag())
				{
					nextX = nodes.at(index + 1).getReference_X();
					nextY = nodes.at(index + 1).getReference_Y();
					DXN = nextX - calc_X;
					DYN = nextY - calc_Y;
				}
				else
				{
					nextX = nodes.at(index + 1).getReference_X();
					nextY = nodes.at(index + 1).getReference_Y();
					DXN = nextX - calc_X;
					DYN = nextY - calc_Y;
				}
			}

			DA = calc_X - A;
			DB = calc_Y - B;

			//曲率方向，若圆为向逆时针走则为负，方位角的弧度值不断增大，第二，三象限交界处需做特殊判断

			NodeInfo& sNode = nodes[currentNode.pSegment->m_nStartIndex];
			NodeInfo& eNode = nodes[currentNode.pSegment->m_nEndIndex];

			double v0_x = sNode.getReference_X() - A;
			double v0_y = sNode.getReference_Y() - B;
			double v1_x = eNode.getReference_X() - A;
			double v1_y = eNode.getReference_Y() - B;

			double z = v0_x*v1_y - v0_y*v1_x;
			if (z < 0)
				curvature = -1.0 / R;
			else
				curvature = 1.0 / R;

			double  v2N_x = calc_Y - B;
			double v2N_y = A - calc_X;

			double v3_x = eNode.getReference_X() - sNode.getReference_X();
			double v3_y = eNode.getReference_Y() - sNode.getReference_Y();

			double dot23 = v2N_x*v3_x + v2N_y*v3_y;
			if (dot23 < 0)
			{
				v2N_x = -v2N_x;
				v2N_y = -v2N_y;
			}

			heading = calcAzimuth(0, 0, v2N_x, v2N_y) * 180 / 3.141593653;
		}
	}
	//C*G == D*D 或在同一分段上
	else
	{
		//计算航向
		double DXL = 0.0, DXN = 0.0;
		double nextX = 0.0, nextY = 0.0, lastX = 0.0, lastY = 0.0;

		//在阈值范围内才计算
		if (result.IsPermitted() || isFinal) {
			if (1 == nodes.at(index).getSingle_flag()) {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}
			else {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}

			if (index >= 1) {
				if (1 == nodes.at(index - 1).getSingle_flag()) {
					lastX = nodes.at(index - 1).getReference_X();
					lastY = nodes.at(index - 1).getReference_Y();
					DXL = calc_X - lastX;
				}
				else {
					lastX = nodes.at(index - 1).getReference_X();
					lastY = nodes.at(index - 1).getReference_Y();
					DXL = calc_X - lastX;
				}
			}

			if (index < nodes.size() - 1) {
				if (1 == nodes.at(index + 1).getSingle_flag()) {
					nextX = nodes.at(index + 1).getReference_X();
					nextY = nodes.at(index + 1).getReference_Y();
					DXN = nextX - calc_X;
				}
				else {
					nextX = nodes.at(index + 1).getReference_X();
					nextY = nodes.at(index + 1).getReference_Y();
					DXN = nextX - calc_X;
				}
			}

			if (DXL >= 0 && DXN > 0) {
				heading = abs(calcAngel(calc_X, calc_Y + 1, calc_X, calc_Y, nextX, nextY));
			}
			else if (DXL > 0 && DXN >= 0) {
				heading = abs(calcAngel(lastX, lastY + 1, lastX, lastY, calc_X, calc_Y));
			}
			else if (DXL <= 0 && DXN < 0) {
				heading = 360 - abs(calcAngel(calc_X, calc_Y + 1, calc_X, calc_Y, nextX, nextY));
			}
			else {
				heading = 360 - abs(calcAngel(lastX, lastY + 1, lastX, lastY, calc_X, calc_Y));
			}
		}
	}

	if (heading < 0) {
		heading = 360 + heading;
	}

	if (heading >= 360) {
		heading = heading - 360;
	}

	result.setCurvature(curvature);
	result.setHeading(heading);

	return result;
}

void MultipleCircle::CalcCurvatureTwoPoints(vector<NodeInfo>& nodes)
{
	double X1, X2, Y1, Y2;
	double heading = 0.0;
	X1 = nodes.at(0).getOrigin_X();
	X2 = nodes.at(1).getOrigin_X();
	Y1 = nodes.at(0).getOrigin_Y();
	Y2 = nodes.at(1).getOrigin_Y();

	if (X2 - X1 > 0)
	{
		heading = abs(calcAngel(X1, Y1 + 1, X1, Y1, X2, Y2));
	}
	else
	{
		heading = 360 - abs(calcAngel(X1, Y1 + 1, X1, Y1, X2, Y2));
	}

	for (int i = 0; i < nodes.size(); i++)
	{
		NodeInfo& node = nodes[i];
		node.setCurvature(0.0);
		node.bNegtive = node.getCurvature() < 0;
		node.setR_Value(99999999);
		node.setHeading(heading);
	}
}

void MultipleCircle::tagPathCurveByDicho(vector<NodeInfo>& nodes)
{
	if (nodes.empty())
		return;

	int index = 0;
	//////////////////////////////////////////////////////////////////////////
	//Only 2 points 
	if(nodes.size() == 2) 
	{
		CalcCurvatureTwoPoints(nodes);
	}
	//////////////////////////////////////////////////////////////////////////
	// 3- 5 Points 
	else if(nodes.size() >= 3 && nodes.size() < 5) 
	{
		index = 0;

		for (int i = 0;i < nodes.size();i++)	 
		{
			NodeInfo& node = nodes[i];
			ResultSet result;
			result = checkIsPermitted(nodes, index, 5, true);
			double curvature = result.getCurvature();
			node.setCurvature(curvature);                //设定曲率为全段参与拟合计算时的曲率
			node.bNegtive = node.getCurvature() < 0;
			node.setHeading(result.getHeading());

			if(abs(curvature) > 0.00000001) 
			{
				node.setR_Value(abs(1 / curvature));
			}
			else 
			{
				node.setR_Value(99999999);
			}
		}
	}
	//////////////////////////////////////////////////////////////////////////
	// Greater than 5 points
	//路链总点数大于等于5才参与计算
	else if(nodes.size() >= 5) 
	{                                
		for (int i = 0;i < nodes.size();i++)	 
		{
			NodeInfo& node = nodes[i];

			//非长直线点和非虚拟点才用计算曲率
			if(0 == node.getLongseg_flag()) 
			{                   	
				double curvature = 0.0;
				ResultSet result;

				//如果扩32个点时阈值未超限
				result = checkIsPermitted(nodes, i, C_ENLARGE_NUM, false);

				if(result.IsPermitted()) 
				{
					//设置扩点数为32
					node.setEnlarge_num(C_ENLARGE_NUM);

					//设定曲率为32个点时的曲率
					node.setCurvature(result.getCurvature());
					node.bNegtive = node.getCurvature() < 0;

					node.setHeading(result.getHeading());

					//cout << "index:\t" << i << "\tNum:\t" << node.getEnlarge_num() << "\tSD:\t" << result.dSD << endl;
				}
				else 
				{
					//找到合适的扩点数，初始上限32，下限0
					//cout << "index " << i << "--------拟合过程---------------" << endl;
					int properEnlarge = getProperEnlarge(nodes, i, C_ENLARGE_NUM, 0,result);   

					//cout << "--------------------------------------------" << endl;

					if(properEnlarge < 10/*C_MIN_NUM*/) {
						//至少两边各向外扩5个点
						properEnlarge = 10/*C_MIN_NUM*/;     
						cout << " 点数较少" << i << endl;
					}

					//设置扩点数为合适的扩点数
					node.setEnlarge_num(properEnlarge);

					result = checkIsPermitted(nodes, i, properEnlarge, true);

					//cout << "index:\t" << i << "\tNum:\t" << node.getEnlarge_num() << "\tSD:\t" << result.dSD << endl;
					//设定曲率为合适扩点时的曲率
					node.setCurvature(result.getCurvature());
					node.bNegtive = node.getCurvature() < 0;

					node.setHeading(result.getHeading());
				}

				curvature = result.getCurvature();

				if(abs(curvature) > 0.00000001) 
				{
					node.setR_Value(abs(1 / curvature));
				}
				else 
				{
					node.setR_Value(99999999);
				}
			}
			else 
			{
				node.setR_Value(99999999);
			}
		} //all nodes and 

		tagLongSegHeading(nodes);
	}
	//////////////////////////////////////////////////////////////////////////
	//
	else 
	{
		std::cout << "we got an error here" << std::endl;
	}
}

void MultipleCircle::CalcStandardDeviation(vector<NodeInfo>& vecNodes, int nIndex, double a, double b, double r, double& sd)
{
	int n = vecNodes.size();
	if (n <= 0)
		return;

	int nLongAxis = std::max(nIndex, n - nIndex);
	double dSQ = 0;
	double dw = 0;

	for (int i = 0; i < n; i++)
	{
		NodeInfo& node = vecNodes[i];

		double x = node.getReference_X();
		double y = node.getReference_Y();

		double radio = 1.0 - (abs(i - nIndex) / (double)(nLongAxis));
		double w = 1;

		if (radio > 0.6)
			w = 2;

		if (radio > 0.9)
			w = 3;

		if (radio > 0.95)
			w = 5;

		double t = sqrt((x - a)*(x - a) + (y - b)*(y - b)) - r;

		dSQ += w *t*t;
		dw += w*(n - 1) / n;
	}

	sd = sqrt(dSQ / dw);
}