//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/SlopeSegmentor.h"
#include "Geometries/SCHAlgorithm/GParams.h"
#include <iostream>
#include "Geometries/SCHAlgorithm/FuzzySegment.h"
#include "Geometries/SCHAlgorithm/fit.h"

using namespace std;

//线段两端曲率不为0的点的个数
int SlopeSegmentor::C_ENDS_POINTNUM = (int) (60 /  GParams::C_GAP);

//中间点距两点连线超限阈值
double SlopeSegmentor::C_DISTANCE_LIMIT = 0.05 * GParams::C_GAP; //2016.7.5 by qiuli

//长线段点连续多少个以下需要平滑为0
int SlopeSegmentor::C_LONGNUM_LIMIT = 6;

SlopeSegmentor::SlopeSegmentor(void)
	{
	}

SlopeSegmentor::~SlopeSegmentor(void)
	{
	}

/*
* 计算平面坐标系点到线段的距离
* 坡度用平面坐标计算
*/
double SlopeSegmentor::calcOriP2LDistance(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode) {
	double distance = 0.0;
	double ref_x = 0.0, ref_y = 0.0;
	double S2E_X = eNode.getOrigin_X() - sNode.getOrigin_X();
	double S2E_Y = eNode.getOrigin_Y() - sNode.getOrigin_Y();
	double segLength = pow(S2E_X, 2) + pow(S2E_Y, 2);
	double d1 = cNode.getOrigin_X() * S2E_X + cNode.getOrigin_Y() * S2E_Y;
	double d2 = sNode.getOrigin_X() * eNode.getOrigin_Y() - sNode.getOrigin_Y() * eNode.getOrigin_X();

	//计算映射点坐标
	ref_x = (S2E_X * d1 + S2E_Y * d2) / segLength;
	ref_y = (S2E_Y * d1 - S2E_X * d2) / segLength;

	distance = sqrt(pow(fabs(cNode.getOrigin_X() - ref_x), 2) + pow(fabs(cNode.getOrigin_Y() - ref_y), 2));

	return distance;
	}

/*
* 
* @param nodes:一条线串
* @param startIndex:线段的起点索引
* @param endIndex:线段的终点索引
* @param segBelong:待标注的线段号
* @return 标注好某条线段信息的线串
*/
void SlopeSegmentor::tagOneSegment(vector<NodeInfo>& nodes, int startIndex, int endIndex, int segBelong)
{
	int seg_seq = 1;
	bool tagLongFlag = false;

	FuzzySegment* pSegment = new FuzzySegment;
	pSegment->m_nId = segBelong;
	pSegment->m_nStartIndex = startIndex;
	pSegment->m_nEndIndex = (endIndex == nodes.size() ? endIndex - 1 : endIndex);


	if (endIndex > startIndex + 2 * C_ENDS_POINTNUM + C_LONGNUM_LIMIT - 2)
	{
		tagLongFlag = true;
	}

	for (int i = startIndex; i < endIndex; i++)
	{
		nodes.at(i).setSeg_belong(segBelong);
		nodes.at(i).setSeg_seq(seg_seq);
		nodes[i].pSegment = pSegment; //segment 标记
		seg_seq++;

		if (tagLongFlag)
		{
			if (i >= startIndex + C_ENDS_POINTNUM && i <= endIndex - C_ENDS_POINTNUM)
			{
				nodes.at(i).setLongseg_flag(1);
			}
		}
	}
}

/*
* 标注单独三个点的坡度
* @param nodes: 待标注分段的点
* @param startIndex: 起始坐标
* @param segBelong:  所属分段
* @return
*/
vector<NodeInfo> SlopeSegmentor::tagThreePoints(vector<NodeInfo> nodes, int startIndex, int segBelong) {
	vector<NodeInfo> segmentedNodes = nodes;
	double checkDis = 0.0;
	NodeInfo sNode = nodes.at(startIndex);
	NodeInfo cNode = nodes.at(startIndex + 1);
	NodeInfo eNode = nodes.at(startIndex + 2);
	checkDis = calcOriP2LDistance(cNode, sNode, eNode);

	if (checkDis >= C_DISTANCE_LIMIT)
	{
		tagOneSegment(segmentedNodes, startIndex, startIndex + 1, segBelong);
		tagOneSegment(segmentedNodes, startIndex + 1, startIndex + 3, segBelong + 1);
	}
	else
	{
		tagOneSegment(segmentedNodes, startIndex, startIndex + 3, segBelong);
	}

	return segmentedNodes;
}

//计算点的映射坐标
SlopeSegmentor::RefResult SlopeSegmentor::calcPointRefOrdinate(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode) {
	SlopeSegmentor::RefResult refResult;
	double ref_x = 0.0, ref_y = 0.0;
	double S2E_X = eNode.getOrigin_X() - sNode.getOrigin_X();
	double S2E_Y = eNode.getOrigin_Y() - sNode.getOrigin_Y();
	double segLength = pow(S2E_X, 2) + pow(S2E_Y, 2);
	double d1 = cNode.getOrigin_X() * S2E_X + cNode.getOrigin_Y() * S2E_Y;
	double d2 = sNode.getOrigin_X() * eNode.getOrigin_Y() - sNode.getOrigin_Y() * eNode.getOrigin_X();

	//计算映射点坐标
	ref_x = (S2E_X * d1 + S2E_Y * d2) / segLength;
	ref_y = (S2E_Y * d1 - S2E_X * d2) / segLength;

	refResult.setRef_X(ref_x);
	refResult.setRef_Y(ref_y);

	return refResult;
	}

void SlopeSegmentor::tagSegRef(vector<NodeInfo>& referedNodes, int sIndex, int eIndex) 
{
	NodeInfo sNode = referedNodes.at(sIndex);
	NodeInfo eNode = referedNodes.at(eIndex);
	referedNodes.at(sIndex).setReference_X(sNode.getOrigin_X());
	referedNodes.at(sIndex).setReference_Y(sNode.getOrigin_Y());

	for(int j = sIndex + 1; j < eIndex; j++) 
	{
		NodeInfo cNode = referedNodes.at(j);
		SlopeSegmentor::RefResult refResult = calcPointRefOrdinate(cNode, sNode, eNode);
		referedNodes.at(j).setReference_X(refResult.getRef_X());
		referedNodes.at(j).setReference_Y(refResult.getRef_Y());
		}
	}

/*
* 标注一整段的映射坐标
* @param seggedNodes
* @return
*/
vector<NodeInfo> SlopeSegmentor::tagRefBySegResult(vector<NodeInfo> seggedNodes)
{
	vector<NodeInfo> referedNodes = seggedNodes;
	int  startIndex = 0, endIndex = referedNodes.size() - 1;
	long oldSegBelong, newSegBelong;
	oldSegBelong = referedNodes.at(0).getSeg_belong();

	for (int i = 0; i < referedNodes.size(); i++)
	{
		newSegBelong = referedNodes.at(i).getSeg_belong();

		if (newSegBelong != oldSegBelong)
		{
			tagSegRef(referedNodes, startIndex, i);
			startIndex = i;
			oldSegBelong = newSegBelong;
		}
	}

	tagSegRef(referedNodes, startIndex, endIndex);
	NodeInfo eNode = seggedNodes.at(endIndex);
	referedNodes.at(endIndex).setReference_X(eNode.getOrigin_X());
	referedNodes.at(endIndex).setReference_Y(eNode.getOrigin_Y());

	return referedNodes;
}

/*
* 标注单独四个点的曲率，一般用来标注尾点
* @param nodes
* @param startIndex
* @param segBelong
* @return
*/
vector<NodeInfo> SlopeSegmentor::tagFourPoints(vector<NodeInfo> nodes, int startIndex, int segBelong) {
	vector<NodeInfo> segmentedNodes = nodes;
	double checkDis1 = 0.0, checkDis2 = 0.0;
	NodeInfo sNode = nodes.at(startIndex);
	NodeInfo cNode1 = nodes.at(startIndex + 1);
	NodeInfo cNode2 = nodes.at(startIndex + 2);
	NodeInfo eNode = nodes.at(startIndex + 3);
	checkDis1 = calcOriP2LDistance(cNode1, sNode, eNode);
	checkDis2 = calcOriP2LDistance(cNode2, sNode, eNode);

	if (checkDis1 >= C_DISTANCE_LIMIT && checkDis2 >= C_DISTANCE_LIMIT)
	{
		double checkDis3 = calcOriP2LDistance(cNode1, sNode, cNode2);
		double checkDis4 = calcOriP2LDistance(cNode2, cNode1, eNode);

		if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT)
		{
			tagOneSegment(segmentedNodes, startIndex, startIndex + 1, segBelong);
			tagOneSegment(segmentedNodes, startIndex + 1, startIndex + 2, segBelong + 1);
			tagOneSegment(segmentedNodes, startIndex + 2, startIndex + 4, segBelong + 2);
		}
		else if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 < C_DISTANCE_LIMIT)
		{
			tagOneSegment(segmentedNodes, startIndex, startIndex + 1, segBelong);
			tagOneSegment(segmentedNodes, startIndex + 1, startIndex + 4, segBelong + 1);
		}
		else {
			tagOneSegment(segmentedNodes, startIndex, startIndex + 2, segBelong);
			tagOneSegment(segmentedNodes, startIndex + 2, startIndex + 4, segBelong + 1);
		}
	}
	else
	{
		tagOneSegment(segmentedNodes, startIndex, startIndex + 4, segBelong);
	}

	return segmentedNodes;
}

/*
* Function_Name:findNextEP
* 实现功能：最长模糊线段算法核心函数，找下一个线段端点，过程中标注单噪点
* @param endIndex:下一个端点索引
*/
int SlopeSegmentor::findNextEP(vector<NodeInfo>& nodes, int start)
{
	int endIndex = nodes.size() - 1;
	NodeInfo sNode = nodes.at(start);

	for (int i = start + 3; i < nodes.size() - 1; i++)
	{
		NodeInfo cNode1;
		NodeInfo cNode2;
		double checkDis1 = 0.0;
		double checkDis2 = 0.0;
		double checkDis3 = 0.0;
		double checkDis4 = 0.0;
		NodeInfo tempENode = nodes.at(i);

		if (i - start > 5)
		{                                                            //五个点以上进行圆弧检查，圆弧
			cNode1 = nodes.at((int)floor(0.5 * (i + start)));
			cNode2 = nodes.at((int)floor(0.5 * (i + start) + 1));               //通过向下取整取到中间两个点
			checkDis1 = calcOriP2LDistance(cNode1, sNode, tempENode);
			checkDis2 = calcOriP2LDistance(cNode2, sNode, tempENode);

			if (checkDis1 >= C_DISTANCE_LIMIT && checkDis2 >= C_DISTANCE_LIMIT)
			{
				tempENode = nodes.at(i + 1);
				checkDis3 = calcOriP2LDistance(cNode1, sNode, tempENode);
				checkDis4 = calcOriP2LDistance(cNode2, sNode, tempENode);

				if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT)
				{
					endIndex = (int)floor(0.5 * (i + start) - 1);

					break;
				}
			}
		}

		cNode1 = nodes.at(i - 2);
		cNode2 = nodes.at(i - 1);
		checkDis1 = calcOriP2LDistance(cNode1, sNode, tempENode);
		checkDis2 = calcOriP2LDistance(cNode2, sNode, tempENode);

		if (checkDis1 >= C_DISTANCE_LIMIT && checkDis2 >= C_DISTANCE_LIMIT)
		{
			tempENode = nodes.at(i + 1);
			checkDis3 = calcOriP2LDistance(cNode1, sNode, tempENode);
			checkDis4 = calcOriP2LDistance(cNode2, sNode, tempENode);
			bool isSinglePixel = true;

			if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT)
			{
				isSinglePixel = false;

				if (fabs(checkDis1 / checkDis2 - (i - 2 - start) / (i - 1 - start)) < 0.0002)
				{
					endIndex = i - 1;

					break;
				}
				else {
					endIndex = i - 2;

					break;
				}
			}
			else {
				cNode1 = nodes.at(i - 1);
				cNode2 = nodes.at(i);
				checkDis3 = calcOriP2LDistance(cNode1, sNode, tempENode);
				checkDis4 = calcOriP2LDistance(cNode2, sNode, tempENode);

				if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT) {
					endIndex = i - 2;

					break;
				}
			}

			if (isSinglePixel)
			{
				nodes.at(i).setSingle_flag(1);
			}
		}
	}

	return endIndex;
}

/*
* Function_Name:tagSegmentBelong
* 实现功能：对单条路链的分段进行标注
* @param nodes
*/
vector<NodeInfo> SlopeSegmentor::tagSegmentBelong(vector<NodeInfo> nodes){
	vector<NodeInfo> segmentedNodes = nodes;

	if (3 == nodes.size()) 
	{
		segmentedNodes = tagThreePoints(segmentedNodes, 0, 1);
		segmentedNodes = tagRefBySegResult(segmentedNodes);
	}
	else if (4 == nodes.size()) 
	{
		segmentedNodes = tagFourPoints(segmentedNodes, 0, 1);
		segmentedNodes = tagRefBySegResult(segmentedNodes);
	}
	else if (nodes.size() >= 5) 
	{		                              //对于点数大于等于5的线串才进行分段，后用最小二乘法计算曲率
		int startIndex = 0, endIndex = nodes.size() - 1;              //初始化终点为最后一个点
		int seg_belong = 1;

		while (startIndex < nodes.size() - 4) {                        //如果线段还有4个以上的点未参与分段
			endIndex = findNextEP(nodes, startIndex);                  //根据是否有连续两点超限找线段终点

			if (endIndex == nodes.size() - 1) {
				endIndex = nodes.size();
			}

		//	cout << "下一个分界区间:\t" << endIndex << endl;

			tagOneSegment(segmentedNodes, startIndex, endIndex, seg_belong);
			startIndex = endIndex;
			seg_belong++;
		}

		if (startIndex == nodes.size() - 4) {
			segmentedNodes = tagFourPoints(segmentedNodes, startIndex, seg_belong);
		}
		else if (startIndex != nodes.size()) {
			if (1 == segmentedNodes.at(nodes.size() - 2).getSingle_flag()) {
				tagOneSegment(segmentedNodes, startIndex, nodes.size(), seg_belong - 1);
			}
			else {
				NodeInfo sNode = segmentedNodes.at(nodes.size() - 3);
				NodeInfo cNode = segmentedNodes.at(nodes.size() - 2);
				NodeInfo eNode = segmentedNodes.at(nodes.size() - 1);
				double checkDis = calcOriP2LDistance(cNode, sNode, eNode);

				if (checkDis < C_DISTANCE_LIMIT) {
					tagOneSegment(segmentedNodes, startIndex, nodes.size(), seg_belong - 1);
				}
				else {
					 tagOneSegment(segmentedNodes, startIndex, nodes.size() - 2, seg_belong - 1);
					 tagOneSegment(segmentedNodes, nodes.size() - 2, nodes.size(), seg_belong);
				}
			}
		}

		segmentedNodes = tagRefBySegResult(segmentedNodes);
	}
	else if (nodes.size() == 1){
	}

	
	/*
	*	划分递增 递减区间
	*/
	//vector<FuzzySegment*> vecFuzzySegment;

	//for (int i = 0; i < segmentedNodes.size() - 1;)
	//{
	//	NodeInfo& currentNode = segmentedNodes[i];
	//	vecFuzzySegment.push_back(currentNode.pSegment);
	//	i = currentNode.pSegment->m_nEndIndex;
	//}

	//double nPoints = 0;
	//bool bLastAscend = true;
	//

	//vector<pair<int, bool>> vecIntervals;

	//for (int i = 1; i < vecFuzzySegment.size();i++)
	//{
	//	FuzzySegment* pCurrentSeg = vecFuzzySegment[i];
	//	FuzzySegment* pLastSeg = vecFuzzySegment[i - 1];

	//	bool bAscend = pCurrentSeg->AverageY(segmentedNodes) > pLastSeg->AverageY(segmentedNodes);
	//	if (i == 1)
	//	{
	//		bLastAscend = bAscend;
	//		nPoints += pLastSeg->Number();
	//	}

	//	if (bLastAscend == bAscend)
	//		nPoints += pCurrentSeg->Number();
	//	else
	//	{
	//		cout <<"index : "<< pCurrentSeg->m_nStartIndex<<"\tnumber:\t"<<nPoints << "\t:"<<((bLastAscend) ? "+" : "-")<<endl;
	//		pair<int, bool> pair(pCurrentSeg->m_nStartIndex,bLastAscend);
	//		vecIntervals.push_back(pair);
	//		bLastAscend = bAscend;
	//		nPoints = pCurrentSeg->Number();
	//	}
	//}

	//for (int i = 0; i < vecIntervals.size();i++)
	//{
	//	int nIndex = vecIntervals[i].first;
	//	bool bAscend = vecIntervals[i].second;

	//	int nPoints = 0;
	//	int j = i + 1;
	//	while (nPoints < 100 && j<vecIntervals.size())
	//	{
	//		int nNextIndex = vecIntervals[j].first;
	//		bool bNextAscend = vecIntervals[j].second;
	//		if (bAscend == bNextAscend)
	//		{
	//			bool bA = segmentedNodes[j].getReference_Y() - segmentedNodes[nIndex].getReference_Y();

	//		}
	//		else
	//		{
	//			j++;
	//			nPoints += nIndex - vecIntervals[j-1].first;
	//		}
	//	}

	//}

	return segmentedNodes;
}

void SlopeSegmentor::selectAbnormalNodes2(vector<NodeInfo>& nodes, map<int, int>& mapAbnormalIndex)
{
	const int MAXCHECKNUM = 100;
	double THRESHOLD = 100/10000.0;

	bool bFirstUse = true;

	int nCurrentPos = 0;
	for (int nPoints = nodes.size(); nCurrentPos < nPoints - 1;)
	{
		NodeInfo& ptCurrent = nodes[nCurrentPos];
		int nNextPos = nCurrentPos + 1;

		vector<double> vecValue;

		for (; nNextPos < nCurrentPos + MAXCHECKNUM && nNextPos <= nPoints - 1; nNextPos++)
		{
			NodeInfo& ptNextPoint = nodes[nNextPos];
			double dJumpValue = ptCurrent.getSlope() - ptNextPoint.getSlope();

			int nRadio = (nNextPos - nCurrentPos);
			/*	if (nRadio > 10)
			nRadio = 10;*/

			if (fabs(dJumpValue) < nRadio * THRESHOLD)
			{
				nNextPos = TryNextJumpPoint(nCurrentPos, nNextPos, THRESHOLD, nodes);

				if (nNextPos - nCurrentPos > 1)
				{
					mapAbnormalIndex.insert(std::make_pair(nCurrentPos, nNextPos));
				}

				break;
			}

			if (nNextPos == nPoints - 1 && nNextPos - nCurrentPos > 1)
			{
				mapAbnormalIndex.insert(std::make_pair(nCurrentPos, nNextPos));
				//cout << "catch tail jump" << endl;
			}

			vecValue.push_back(dJumpValue);
		} //next end

		nCurrentPos = nNextPos;
	} //1
}

void SlopeSegmentor::ProcessTailJumpInterval(vector<NodeInfo> &vecNodeInfo, int nStartIndex)
{
	if (nStartIndex <= 0)
	{
		cout << "start index <= 0 ,quit" << endl;
		return;
	}

	int nLeft = 50;
	while (nStartIndex - nLeft < 0)
	{
		nLeft = nLeft*0.9;
	}

	std::vector<double> vecX;
	std::vector<double> vecY;
	for (int i = nStartIndex - nLeft; i <= nStartIndex; i++)
	{
		vecX.push_back(i);
		vecY.push_back(vecNodeInfo[i].getSlope());
	}

	czy::Fit fit;
	fit.polyfit(vecX, vecY, 3);

	double dMaxSlope = -1e6;
	double dMinSlope = 1e6;

	/*for (int i = nStartIndex + 1; i < vecNodeInfo.size(); i++)
	{
	NodeInfo& node = vecNodeInfo[i];
	if (node.getSlope() > dMaxSlope)
	dMaxSlope = node.getSlope();
	if (node.getSlope() < dMinSlope)
	dMinSlope = node.getSlope();
	}*/

	dMaxSlope = 3.5;
	dMinSlope = -3.5;

	bool bFit = true;

	for (int i = nStartIndex + 1; i < vecNodeInfo.size(); i++)
	{
		double y = fit.getY(i);
		if (y > dMaxSlope || y < dMinSlope)
		{
			bFit = false;
			break;
		}
		vecNodeInfo[i].setSlope(y);
	}


	if (!bFit)
	{
		vecX.clear();
		vecY.clear();
		//cout << vecNodeInfo.front().getLinkId() << "\t坡度趋势拟合超限,使用其他方法" << endl;
		for (int i = nStartIndex + 1; i < vecNodeInfo.size(); i++)
		{
			vecX.push_back(i);
			vecY.push_back(vecNodeInfo[i].getSlope());
		}
		czy::Fit fit;
		fit.polyfit(vecX, vecY, 2);
		for (int i = nStartIndex + 1; i < vecNodeInfo.size(); i++)
		{
			double y = fit.getY(i);
			vecNodeInfo[i].setSlope(y);
		}

	}

}

void SlopeSegmentor::smoothArrayNodeByLineInterpolation(vector<NodeInfo> &nodes, map<int, int> & mapIndex)
{
	if (nodes.size() < 1)
	{
		return;
	}

	map<int, int>::iterator  itmap;
	for (itmap = mapIndex.begin(); itmap != mapIndex.end(); itmap++)
	{
		int nStartIndex = itmap->first;
		int nEndIndex = itmap->second;

		if (nStartIndex > 0 && nEndIndex == nodes.size() - 1)
		{
			ProcessTailJumpInterval(nodes, nStartIndex);
			continue;
		}

		int nNodeSize = nodes.size();
		if (!(nStartIndex >= 0 && nStartIndex < nEndIndex && nStartIndex < nNodeSize && nEndIndex >= 0 && nEndIndex < nNodeSize))
			return;

		double dLittleValue = nodes[nStartIndex].getSlope();
		double dTopValue = nodes[nEndIndex].getSlope();

		for (int index = nStartIndex + 1; index < nEndIndex; index++)
		{
			double dCurvatureValue = dLittleValue + (double)(index - nStartIndex) / (nEndIndex - nStartIndex) * (dTopValue - dLittleValue);
			nodes[index].setSlope(dCurvatureValue);
		}
	}
}

int SlopeSegmentor::TryNextJumpPoint(int nStartIndex, int nCurrentIndex, double dThreshold, std::vector<NodeInfo>& nodes)
{
	NodeInfo& sNode = nodes[nStartIndex];
	int nSize = nodes.size();

	int nNextPos = nCurrentIndex + 1;
	for (; nNextPos < nSize; nNextPos++)
	{
		NodeInfo& p0 = nodes[nNextPos - 1];
		NodeInfo& p1 = nodes[nNextPos];

		double dDelta0 = fabs(p1.getSlope()) - fabs(p0.getSlope());
		double dDelta1 = fabs(p1.getSlope()) - fabs(sNode.getSlope());

		dDelta0 = fabs(dDelta0);
		dDelta1 = fabs(dDelta1);

		if (dDelta0 > dThreshold && dDelta1 <= (nNextPos - nStartIndex)*dThreshold)
			continue;
		else
		{
			break;
		}

	}

	return nNextPos - 1;
}