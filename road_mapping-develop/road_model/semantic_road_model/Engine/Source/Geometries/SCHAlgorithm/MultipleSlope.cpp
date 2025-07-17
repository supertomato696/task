//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/MultipleSlope.h"
#include "Geometries/SCHAlgorithm/GParams.h"
#include "Geometries/SCHAlgorithm/FuzzySegment.h"
#include <iostream>

#include "Geometries/SCHAlgorithm/LSF/circlefit.h"
#include <math.h>

#include <limits>
using namespace std;
//////////////////////////////////////////////////////////////////////////

const double C_PI = 3.1415926;
const double C_ANGLE_LIMIT = 15.0;
const double C_SLOPE_LIMIT = 10.0;
const double C_TOLLERATE_LIMIT = 15.0;
const int C_ENLARGE_NUM = (int)(80 / GParams::C_GAP);
const int C_MIN_NUM = (int)(15 / GParams::C_GAP);

MultipleSlope::MultipleSlope(void)
{
}

MultipleSlope::~MultipleSlope(void)
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
double MultipleSlope::calcAngel(double XS, double YS, double XM, double YM, double XE, double YE) {
	double FI = 0.0;
	double DSX = XS - XM;
	double DSY = YS - YM;
	double DEX = XE - XM;
	double DEY = YE - YM;
	double DOTVEC = DSX * DEX + DSY * DEY;
	double NORM = (DSX * DSX + DSY * DSY) * (DEX * DEX + DEY * DEY);    //模乘积

	if (NORM != 0) {
		double COSVAL = DOTVEC / sqrt(NORM);
		FI = acos(COSVAL) * 180 / C_PI;
	}

	return FI;
}

//计算阈值
double MultipleSlope::getThreshold(int nCount) {
	double allowed_var = 0.1;

	if (nCount < 15) {
		allowed_var = 0.25 - 0.01 * nCount;
	}

	return allowed_var;
}

void MultipleSlope::getProperNodes(vector<NodeInfo>& vecNodes, vector<NodeInfo>& properNodes, int nIndex, int IntialExtendNum)
{
	if (nIndex == 15)
	{
		int a = 0;
	}

	int nLeftEnd = 0;
	int nRightEnd = vecNodes.size();

	NodeInfo& currentNode = vecNodes[nIndex];
	//两边都越界
	if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size())
	{
	}
	//起点单侧越界
	else if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 <= vecNodes.size())
	{
		nRightEnd = nIndex + IntialExtendNum + 1;
	}
	//终点单侧越界
	else if (nIndex >= IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size())
	{
		nLeftEnd = nIndex - IntialExtendNum;
	}
	//两侧都未越界
	else
	{
		nLeftEnd = nIndex - IntialExtendNum;
		nRightEnd = nIndex + IntialExtendNum + 1;
	}

	//2016.7.4 以下代码 暂时屏蔽
	/*
	增加相邻分段夹角控制，如果相邻分段夹角超过限定值，则不启用当前
	*/
	const double INCLUDE_ANGLE_LIMIT = 140;

	int  nR = nIndex + 1;
	FuzzySegment* pCurrentSegment = currentNode.pSegment;

	while (nR < nRightEnd)
	{
		NodeInfo& nodeInfo = vecNodes[nR];
		if (pCurrentSegment == nodeInfo.pSegment)
		{
			nR++; continue;
		}

		double dAngle = pCurrentSegment->IncludeAngle(nodeInfo.pSegment, vecNodes, 100);

		/*cout << "区    间 " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex ;
		cout << " 与区间 " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
		cout << "右侧角度超限:\t" << dAngle << endl;*/

		if (dAngle < INCLUDE_ANGLE_LIMIT)
		{
			break;
		}

		pCurrentSegment = nodeInfo.pSegment;
		nR++;
	}


	pCurrentSegment = currentNode.pSegment;
	int nL = nIndex;
	while (nL > nLeftEnd)
	{
		NodeInfo& nodeInfo = vecNodes[nL];
		if (pCurrentSegment == nodeInfo.pSegment)
		{
			nL--; continue;
		}

		double dAngle = pCurrentSegment->IncludeAngle(nodeInfo.pSegment, vecNodes, 100);

		/*	cout << "区    间 " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex ;
			cout << " 与区间 " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
			cout << "左侧角度超限:\t" << dAngle << endl;*/

		if (dAngle < INCLUDE_ANGLE_LIMIT)
		{
			break;
		}

		pCurrentSegment = nodeInfo.pSegment;
		nL--;
	}

	const int  C_INFLATE_NUM = 10;

	nL = nL - C_INFLATE_NUM > 0 ? nL - C_INFLATE_NUM : 0;
	nR = nR + C_INFLATE_NUM >= vecNodes.size() ? vecNodes.size() - 1 : nR + C_INFLATE_NUM;

	properNodes.assign(vecNodes.begin() + nLeftEnd, vecNodes.begin() + nRightEnd);
	//properNodes.assign(vecNodes.begin() + nL, vecNodes.begin() + nR);
}

/*
* 功能：核心函数，计算扩展数为enlargeNum时的坡度，检测扩展数为enlargeNum时，均方差是否超限，是否有连续两点超过三倍均方差阈值
* @param nodes:      路链串
* @param index:      路链上第几个点
* @param enlargeNum: 扩展点数
* @return            :ResultSet  <isPermitted:是否未超阈值，slope:坡度>
*/
MultipleSlope::ResultSet MultipleSlope::checkIsPermitted(vector<NodeInfo>& nodes, int index, int enlargeNum, bool isFinal)
{
	double X1 = 0;
	double Y1 = 0;
	double X2 = 0;
	double Y2 = 0;
	double X3 = 0;
	double Y3 = 0;
	double X1Y1 = 0;
	double X1Y2 = 0;
	double X2Y1 = 0;
	double calc_X, calc_Y;
	vector<NodeInfo> properNodes;

	if (200 == index)
	{
		int x = 0;
	}

	/*取参与拟合的计算点,如果超过路链的起点或终点，就从起点或终点开始取*/
	getProperNodes(nodes, properNodes, index, enlargeNum);

	/*最小二乘法计算圆心坐标，圆半径*/
	int N;
	double a, b, c;
	N = properNodes.size();
	double slope = 0.0;
	MultipleSlope::ResultSet result;
	bool isOnOneSeg = false;

	if ((properNodes.at(0).getSeg_belong() == properNodes.at(N - 1).getSeg_belong()) && (properNodes.at(0).getSeg_seq() < properNodes.at(N - 1).getSeg_seq()))
	{
		isOnOneSeg = true;
	}

	/**/

	if (/*!isOnOneSeg*/true)
	{
		/*最小二乘法计算圆心坐标，圆半径*/
		double A, B, R,SD;
		if (!CalcR(properNodes, A, B, R, SD))
		{
			//SD = numeric_limits<double>::max();
			SD = (std::numeric_limits<double>::max)();
		}
	
		/**/

		//计算扩点数为enLarge时的圆心角
		NodeInfo node1 = properNodes.at(0);
		NodeInfo node2 = properNodes.at(N - 1);
		double circleAngel = calcAngel(node1.getOrigin_X(), node1.getOrigin_Y(), A, B, node2.getOrigin_X(), node2.getOrigin_Y());

		//阈值判定，一开始为允许
		double threshold = getThreshold(N);

		if (SD > threshold /*|| circleAngel > C_ANGLE_LIMIT*/)
		{
			result.setPermitted(false);
		}
		
		//算该点到下一点的坡度
		double DB;

		//在阈值范围内才计算
		if (result.IsPermitted() || isFinal)
		{
			//最后一点没有后面的点，在最后单独处理
			if (1 == nodes.at(index).getSingle_flag()) {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}
			else {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}

			DB = calc_Y - B;

			/*计算坡度*/

			//圆心在计算点的下方
			if (DB >= 0)
				slope = 90 - calcAngel(calc_X + 1, calc_Y, calc_X, calc_Y, A, B);
			//圆心在计算点的上方
			else
				slope = calcAngel(calc_X + 1, calc_Y, calc_X, calc_Y, A, B) - 90;
		}

		/**/
	}
	//C*G == D*D 或在同一分段上
	//else {
	//	int leftIndex = 0, rightIndex = nodes.size() - 1;

	//	if (index > 0) {
	//		leftIndex = index - 1;
	//	}

	//	if (index < nodes.size() - 1) {
	//		rightIndex = index + 1;
	//	}

	//	double DX = nodes.at(rightIndex).getOrigin_X() - nodes.at(leftIndex).getOrigin_X();
	//	double DY = nodes.at(rightIndex).getOrigin_Y() - nodes.at(leftIndex).getOrigin_Y();
	//	slope = 180 / C_PI * atan(DY / DX);
	//}

	//算出的坡度不合理（有可能是拟合圆弧为直线所致），尝试直接和下一点两点计算一下
	if (slope == 0 || fabs(slope) > C_SLOPE_LIMIT)
	{
		double calc_X1 = 0.0, calc_X2 = 0.0, calc_Y1 = 0.0, calc_Y2 = 0.0;

		if (index != nodes.size() - 1)
		{
			if (1 == nodes.at(index).getSingle_flag()) {
				calc_X1 = nodes.at(index).getReference_X();
				calc_Y1 = nodes.at(index).getReference_Y();
			}
			else {
				calc_X1 = nodes.at(index).getReference_X();
				calc_Y1 = nodes.at(index).getReference_Y();
			}

			if (1 == nodes.at(index + 1).getSingle_flag()) {
				calc_X2 = nodes.at(index + 1).getReference_X();
				calc_Y2 = nodes.at(index + 1).getReference_Y();
			}
			else {
				calc_X2 = nodes.at(index + 1).getReference_X();
				calc_Y2 = nodes.at(index + 1).getReference_Y();
			}
		}
		//为最后一个点，则和前面的点两点计算一下坡度
		else {
			if (1 == nodes.at(index - 1).getSingle_flag()) {
				calc_X1 = nodes.at(index - 1).getReference_X();
				calc_Y1 = nodes.at(index - 1).getReference_Y();
			}
			else {
				calc_X1 = nodes.at(index - 1).getReference_X();
				calc_Y1 = nodes.at(index - 1).getReference_Y();
			}

			if (1 == nodes.at(index).getSingle_flag()) {
				calc_X2 = nodes.at(index).getReference_X();
				calc_Y2 = nodes.at(index).getReference_Y();
			}
			else {
				calc_X2 = nodes.at(index).getReference_X();
				calc_Y2 = nodes.at(index).getReference_Y();
			}
		}

		double tempSlope = 180 / C_PI * atan((calc_Y2 - calc_Y1) / (calc_X2 - calc_X1));

		if (tempSlope != 0 && fabs(tempSlope) <= C_TOLLERATE_LIMIT)
		{
			slope = tempSlope;
		}
	}

	result.setSlope(slope);

	return result;
}

/*
* Function_Name: tagLongSegHeading
* 实现功能：对长坡中间点的坡度特殊处理，取一段坡两端计算坡度值标注整段
* @param nodes
* @return
*/
void MultipleSlope::tagLongSegSlope(vector<NodeInfo>& nodes)
{
	int index = 0, fromIndex = 0, endIndex = 0;
	double X1 = 0.0, X2 = 0.0, Y1 = 0.0, Y2 = 0.0;
	double slope = 0.0;

	while (index < nodes.size() - 1)
	{
		if (0 == nodes.at(index).getLongseg_flag()
			&& 1 == nodes.at(index + 1).getLongseg_flag())
		{
			fromIndex = index + 1;
			X1 = nodes.at(index + 1).getReference_X();
			Y1 = nodes.at(index + 1).getReference_Y();
		}
		else if (1 == nodes.at(index).getLongseg_flag()
			&& 0 == nodes.at(index + 1).getLongseg_flag())
		{
			endIndex = index + 1;
			X2 = nodes.at(index + 1).getReference_X();
			Y2 = nodes.at(index + 1).getReference_Y();
			slope = 180 / C_PI * atan((Y2 - Y1) / (X2 - X1));

			for (int i = fromIndex; i < endIndex; i++)
			{
				if (1 == nodes.at(i).getLongseg_flag())
				{
					nodes.at(i).setSlope(slope);
				}
			}
		}

		index++;
	}
}

/*
* Function_Name:getProperEnlarge
* 实现功能：通过用二分法判定是否超限，获得合适的扩展点的个数
* @param nodes
*/
int MultipleSlope::getProperEnlarge(vector<NodeInfo>& nodes, int index, int top, int low)
{
	int topCheck = top;
	int lowCheck = low;

	while (topCheck - lowCheck >= 2)
	{    //二分法终止条件
		MultipleSlope::ResultSet result;
		result = checkIsPermitted(nodes, index, (topCheck + lowCheck) / 2, false);

		if (result.IsPermitted()) {
			lowCheck = (topCheck + lowCheck) / 2;      //没有超过阈值则下限上移
		}
		else {
			topCheck = (topCheck + lowCheck) / 2;      //超过阈值则上先下移
		}
	}

	return (topCheck + lowCheck - 1) / 2;              //返回	    
}

/*
* 实现功能：对坡度进行简单平滑：第一个点超限第二个点不超，则首点赋第二个点坡度值
*                             中间点超限，两边点不超，则赋两边点平均值
*                             尾点超限，倒数第二点不超，则尾点赋倒数第二个点坡度值
* @return nodes:平滑完坡度后的点序列
*/
void MultipleSlope::smoothSlopeValue(vector<NodeInfo>& nodes)
{
	double modifiedSlope = 0.0;

	for (int i = 0; i < nodes.size(); i++)
	{
		if (0 == i && nodes.at(i).getSlope() > C_TOLLERATE_LIMIT && nodes.at(i + 1).getSlope() <= C_TOLLERATE_LIMIT)
		{
			modifiedSlope = nodes.at(i + 1).getSlope();
			nodes.at(i).setSlope(modifiedSlope);
		}

		if (i > 0 && i < nodes.size() - 1 &&
			nodes.at(i).getSlope() > C_TOLLERATE_LIMIT &&
			nodes.at(i + 1).getSlope() <= C_TOLLERATE_LIMIT &&
			nodes.at(i - 1).getSlope() <= C_TOLLERATE_LIMIT)
		{
			modifiedSlope = (nodes.at(i - 1).getSlope() + nodes.at(i + 1).getSlope()) / 2;
			nodes.at(i).setSlope(modifiedSlope);
		}

		if (i == nodes.size() - 1 &&
			nodes.at(i).getSlope() > C_TOLLERATE_LIMIT &&
			nodes.at(i - 1).getSlope() <= C_TOLLERATE_LIMIT)
		{
			modifiedSlope = nodes.at(i - 1).getSlope();
			nodes.at(i).setSlope(modifiedSlope);
		}
	}
}

/*
* Function_Name:tagPathCurveByDicho
* 实现功能：用二分法对单条路链的坡度进行标注
* @param nodes
*/
vector<NodeInfo> MultipleSlope::tagPathCurveByDicho(vector<NodeInfo> nodes)
{
	int index = 0;

	/*
	 *	只有两个点：直接用2点计算
	 */
	if (nodes.size() == 2)
	{
		double DX = nodes.at(1).getOrigin_X() - nodes.at(0).getOrigin_X();
		double DY = nodes.at(1).getOrigin_Y() - nodes.at(0).getOrigin_Y();
		double slope = 180 / C_PI * atan(DY / DX);
		nodes.at(0).setSlope(slope);
	}
	else if (nodes.size() >= 3 && nodes.size() < 5)
	{
		index = 0;

		for (int i = 0; i < nodes.size(); i++)
		{
			NodeInfo node = nodes[i];
			MultipleSlope::ResultSet result;
			result = checkIsPermitted(nodes, index, 5, true);
			double slope = result.getSlope();
			node.setSlope(slope);                               //设定坡度为全段参与拟合计算时的坡度
			index++;
		}
	}
	//路链总点数大于等于5才参与计算
	else if (nodes.size() >= 5)
	{
		index = 0;

		/*****************************
		* 计算长直坡上的点的坡度
		*****************************/

		tagLongSegSlope(nodes);

		for (int i = 0; i < nodes.size(); i++)
		{
			NodeInfo node = nodes[i];

			//非长直坡才需用最小二乘法计算坡度
			if (0 == node.getLongseg_flag())
			{
				MultipleSlope::ResultSet result;

				//如果扩16个点时阈值未超限
				result = checkIsPermitted(nodes, index, C_ENLARGE_NUM, false);

				if (result.IsPermitted()) {
					//设置扩点数为16
					node.setEnlarge_num(C_ENLARGE_NUM);

					//设定坡度为16个点时的坡度
					node.setSlope(result.getSlope());
				}
				else {
					//找到合适的扩点数，初始上限32，下限0
					int properEnlarge = getProperEnlarge(nodes, index, C_ENLARGE_NUM, 0);

					if (properEnlarge < C_MIN_NUM) {
						//至少两边各向外扩3个点
						properEnlarge = C_MIN_NUM;
					}

					//设置扩点数为合适的扩点数
					node.setEnlarge_num(properEnlarge);

					result = checkIsPermitted(nodes, index, properEnlarge, true);

					//设定坡度为合适扩点时的坡度
					node.setSlope(result.getSlope());
				}
			}

			nodes[index] = node;
			index++;
		}

		/**/
	}
	else {
	}

	smoothSlopeValue(nodes);

	return nodes;
}
