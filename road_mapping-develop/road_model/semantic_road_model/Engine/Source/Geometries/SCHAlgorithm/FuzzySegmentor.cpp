//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/FuzzySegmentor.h"
#include "Geometries/SCHAlgorithm/GParams.h"
#include <iostream>
#include "Geometries/SCHAlgorithm/FuzzySegment.h"

#include <math.h>

int FuzzySegmentor::C_ENDS_POINTNUM  = (int) (60 / GParams::C_GAP);
double FuzzySegmentor::C_DISTANCE_LIMIT = 0.2 * GParams::C_GAP;
int FuzzySegmentor::C_LONGNUM_LIMIT = 6;

using namespace std;

FuzzySegmentor::FuzzySegmentor(void)
	{
	}

FuzzySegmentor::~FuzzySegmentor(void)
	{
	}

/*
* ����ƽ������ϵ�㵽�߶εľ���
* �����ڲ���,ʵ�������������������
*/
double FuzzySegmentor::calcOriP2LDistance(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode) {
	double distance = 0.0;
	double ref_x = 0.0, ref_y = 0.0;
	double S2E_X = eNode.getOrigin_X() - sNode.getOrigin_X();
	double S2E_Y = eNode.getOrigin_Y() - sNode.getOrigin_Y();
	double segLength = pow(S2E_X, 2) + pow(S2E_Y, 2);
	double d1 = cNode.getOrigin_X() * S2E_X + cNode.getOrigin_Y() * S2E_Y;
	double d2 = sNode.getOrigin_X() * eNode.getOrigin_Y() - sNode.getOrigin_Y() * eNode.getOrigin_X();

	//����ӳ�������
	ref_x = (S2E_X * d1 + S2E_Y * d2) / segLength;
	ref_y = (S2E_Y * d1 - S2E_X * d2) / segLength;

	distance = sqrt(pow(fabs(cNode.getOrigin_X() - ref_x), 2) + pow(fabs(cNode.getOrigin_Y() - ref_y), 2));

	return distance;
	}

/*
* @param nodes:һ���ߴ�
* @param startIndex:�߶ε��������
* @param endIndex:�߶ε��յ�����
* @param segBelong:����ע���߶κ�
* @return ��ע��ĳ���߶���Ϣ���ߴ�
*/
void FuzzySegmentor::tagOneSegment(vector<NodeInfo>& nodes, int startIndex, int endIndex, int segBelong)
{
	int seg_seq = 1;
	bool tagLongFlag = false;

	FuzzySegment* pSegment = new FuzzySegment;
	pSegment->m_nId = segBelong;
	pSegment->m_nStartIndex = startIndex;
	pSegment->m_nEndIndex = (endIndex == nodes.size() ? endIndex - 1 : endIndex);

	//////////////////////////////////////////////////////////////////////////
	//
	
	if (endIndex > startIndex + 2 * C_ENDS_POINTNUM + C_LONGNUM_LIMIT - 2)
	{
		tagLongFlag = true;
	}

	for (int i = startIndex; i < endIndex; i++)
	{
		nodes.at(i).setSeg_belong(segBelong);
		nodes.at(i).setSeg_seq(seg_seq);
		seg_seq++;

		nodes.at(i).pSegment = pSegment; //�ֶ���Ϣ

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
* ��ע���������������
* @param nodes: ����ע�ֶεĵ�
* @param startIndex: ��ʼ����
* @param segBelong:  �����ֶ�
* @return
*/
void FuzzySegmentor::tagThreePoints(vector<NodeInfo>& nodes, int startIndex, int segBelong)
{
	double checkDis = 0.0;
	NodeInfo sNode = nodes.at(startIndex);
	NodeInfo cNode = nodes.at(startIndex + 1);
	NodeInfo eNode = nodes.at(startIndex + 2);
	checkDis = calcOriP2LDistance(cNode, sNode, eNode);

	if (checkDis >= C_DISTANCE_LIMIT)
	{
		tagOneSegment(nodes, startIndex, startIndex + 1, segBelong);
		tagOneSegment(nodes, startIndex + 1, startIndex + 3, segBelong + 1);
	}
	else
	{
		tagOneSegment(nodes, startIndex, startIndex + 3, segBelong);
	}
}

//������ӳ������
RefResult FuzzySegmentor::calcPointRefOrdinate(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode) {
	RefResult refResult;
	double ref_x = 0.0, ref_y = 0.0;
	double S2E_X = eNode.getOrigin_X() - sNode.getOrigin_X();
	double S2E_Y = eNode.getOrigin_Y() - sNode.getOrigin_Y();
	double segLength = pow(S2E_X, 2) + pow(S2E_Y, 2);
	double d1 = cNode.getOrigin_X() * S2E_X + cNode.getOrigin_Y() * S2E_Y;
	double d2 = sNode.getOrigin_X() * eNode.getOrigin_Y() - sNode.getOrigin_Y() * eNode.getOrigin_X();

	//����ӳ�������
	ref_x = (S2E_X * d1 + S2E_Y * d2) / segLength;
	ref_y = (S2E_Y * d1 - S2E_X * d2) / segLength;

	refResult.setRef_X(ref_x);
	refResult.setRef_Y(ref_y);

	return refResult;
	}

void FuzzySegmentor::tagSegRef(vector<NodeInfo>& referedNodes, int sIndex, int eIndex)
{
	NodeInfo sNode = referedNodes.at(sIndex);
	NodeInfo eNode = referedNodes.at(eIndex);
	referedNodes.at(sIndex).setReference_X(sNode.getOrigin_X());
	referedNodes.at(sIndex).setReference_Y(sNode.getOrigin_Y());

	for (int j = sIndex + 1; j < eIndex; j++)
	{
		NodeInfo cNode = referedNodes.at(j);
		RefResult refResult = calcPointRefOrdinate(cNode, sNode, eNode);
		referedNodes.at(j).setReference_X(refResult.getRef_X());
		referedNodes.at(j).setReference_Y(refResult.getRef_Y());
	}
}

/*
* ��עһ���ε�ӳ������
* @param seggedNodes
* @return
*/
void FuzzySegmentor::tagRefBySegResult(vector<NodeInfo>& seggedNodes)
{
	int  startIndex = 0, endIndex = seggedNodes.size() - 1;
	long oldSegBelong, newSegBelong;
	oldSegBelong = seggedNodes.at(0).getSeg_belong();

	for (int i = 0; i < seggedNodes.size(); i++)
	{
		newSegBelong = seggedNodes.at(i).getSeg_belong();

		if (newSegBelong != oldSegBelong)
		{
			tagSegRef(seggedNodes, startIndex, i);
			startIndex = i;
			oldSegBelong = newSegBelong;
		}
	}

	tagSegRef(seggedNodes, startIndex, endIndex);
	NodeInfo eNode = seggedNodes.at(endIndex);
	seggedNodes.at(endIndex).setReference_X(eNode.getOrigin_X());
	seggedNodes.at(endIndex).setReference_Y(eNode.getOrigin_Y());
}

/**
* ��ע�����ĸ�������ʣ�һ��������עβ��
* @param nodes
* @param startIndex
* @param segBelong
* @return
*/
void FuzzySegmentor::tagFourPoints(vector<NodeInfo>& segmentedNodes, int startIndex, int segBelong)
{
	double checkDis1 = 0.0, checkDis2 = 0.0;
	NodeInfo sNode = segmentedNodes.at(startIndex);
	NodeInfo cNode1 = segmentedNodes.at(startIndex + 1);
	NodeInfo cNode2 = segmentedNodes.at(startIndex + 2);
	NodeInfo eNode = segmentedNodes.at(startIndex + 3);
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
		else
		{
			tagOneSegment(segmentedNodes, startIndex, startIndex + 2, segBelong);
			tagOneSegment(segmentedNodes, startIndex + 2, startIndex + 4, segBelong + 1);
		}
	}
	else
	{
		tagOneSegment(segmentedNodes, startIndex, startIndex + 4, segBelong);
	}
}

/*
* Function_Name:findNextEP
* ʵ�ֹ��ܣ��ģ���߶��㷨���ĺ���������һ���߶ζ˵㣬�����б�ע�����
* @param endIndex:��һ���˵�����
*/
int FuzzySegmentor::findNextEP(vector<NodeInfo>& nodes, int start)
{
	int endIndex = nodes.size() - 1;
	NodeInfo& sNode = nodes.at(start);

	for (int i = start + 3; i < nodes.size() - 1; i++) 
	{
		NodeInfo cNode1;
		NodeInfo cNode2;
		double checkDis1 = 0.0;
		double checkDis2 = 0.0;
		double checkDis3 = 0.0;
		double checkDis4 = 0.0;
		//
		NodeInfo tempENode = nodes.at(i);

		//������Բ������ȡ�м�����
		if (i - start > 5) 
		{
			cNode1 = nodes.at((int)floor(0.5 * (i + start)));
			cNode2 = nodes.at((int)floor(0.5 * (i + start) + 1));
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

		/*������ֱ�߷���ȡ�����ڶ��͵�����������*/

		cNode1 = nodes.at(i - 2);
		cNode2 = nodes.at(i - 1);
		checkDis1 = calcOriP2LDistance(cNode1, sNode, tempENode);
		checkDis2 = calcOriP2LDistance(cNode2, sNode, tempENode);

		if (checkDis1 >= C_DISTANCE_LIMIT && checkDis2 >= C_DISTANCE_LIMIT) {
			tempENode = nodes.at(i + 1);
			checkDis3 = calcOriP2LDistance(cNode1, sNode, tempENode);
			checkDis4 = calcOriP2LDistance(cNode2, sNode, tempENode);

			//�����
			bool isSinglePixel = true;

			if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT) {
				//�ǵ����
				isSinglePixel = false;

				//��Ϊ������С����С��7��������ϸ΢���ж�
				if (fabs(checkDis1 / checkDis2 - (i - 2 - start) / (i - 1 - start)) < 0.0002) {
					endIndex = i - 1;

					break;
				}
				else {
					endIndex = i - 2;
					break;
				}
			}
			else 
			{
				cNode1 = nodes.at(i - 1);
				cNode2 = nodes.at(i);
				checkDis3 = calcOriP2LDistance(cNode1, sNode, tempENode);
				checkDis4 = calcOriP2LDistance(cNode2, sNode, tempENode);

				if (checkDis3 >= C_DISTANCE_LIMIT && checkDis4 >= C_DISTANCE_LIMIT) 
				{
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

	/**/

	return endIndex;
}

/*
* Function_Name:tagSegmentBelong
* ʵ�ֹ��ܣ��Ե���·���ķֶν��б�ע
* @param nodes
*/
void FuzzySegmentor::tagSegmentBelong(vector<NodeInfo>& nodes)
{
	//��ע3��
	if (3 == nodes.size())
	{
		tagThreePoints(nodes, 0, 1);
		tagRefBySegResult(nodes);
	}
	//��ע4����
	else if (4 == nodes.size())
	{
		tagFourPoints(nodes, 0, 1);
		tagRefBySegResult(nodes);
	}
	//////////////////////////////////////////////////////////////////////////
	// 5 �㼰����
	else if (nodes.size() >= 5)
	{
		int startIndex = 0, endIndex = nodes.size() - 1;
		int seg_belong = 1;

		

		while (startIndex < nodes.size() - 4)
		{
// 			for (int i = 0; i < nodes.size(); i++)
// 				cout << nodes[i].PostnID << endl;

			//DONGJIAN
			endIndex = findNextEP(nodes, startIndex);

			if (endIndex == nodes.size() - 1)
			{
				endIndex = nodes.size();
			}

			
			if (endIndex == 31)
			{
				int x = 0;
			}

			//cout << "�ֶν��:\t" << endIndex << endl;
			tagOneSegment(nodes, startIndex, endIndex, seg_belong);
			startIndex = endIndex;
			seg_belong++;

			if (nodes.back().PostnID == nodes[nodes.size() - 2].PostnID)
			{
				int a = 0;
			}
		}


	

		if (startIndex == nodes.size() - 4)
		{
			tagFourPoints(nodes, startIndex, seg_belong);
		}
		else if (startIndex != nodes.size()) 
		{
			if (1 == nodes.at(nodes.size() - 2).getSingle_flag())
			{
				tagOneSegment(nodes, startIndex, nodes.size(), seg_belong - 1);
			}
			else 
			{
				NodeInfo sNode = nodes.at(nodes.size() - 3);
				NodeInfo cNode = nodes.at(nodes.size() - 2);
				NodeInfo eNode = nodes.at(nodes.size() - 1);
				double checkDis = calcOriP2LDistance(cNode, sNode, eNode);

				if (checkDis < C_DISTANCE_LIMIT) 
				{
					tagOneSegment(nodes, startIndex, nodes.size(), seg_belong - 1);
				}
				else 
				{
					tagOneSegment(nodes, startIndex, nodes.size() - 2, seg_belong - 1);
					tagOneSegment(nodes, nodes.size() - 2, nodes.size(), seg_belong);
				}
			}
		}

		tagRefBySegResult(nodes);
	}
}

bool FuzzySegmentor::getSingleAbnormalNodes(vector<NodeInfo> &lstNodes)
{
	int nNodes =  lstNodes.size();
		for(int i=1;i<nNodes-1;i++)
		{
			NodeInfo sNode = lstNodes[i-1];
			NodeInfo mNode = lstNodes[i];
			NodeInfo eNode = lstNodes[i+1];
			
			double dRs = 1e6 / fabs(sNode.getCurvature());
			double dRm = 1e6 / fabs(mNode.getCurvature());
			double dRe = 1e6 / fabs(eNode.getCurvature());
			
			//LogUtil.Debug("Rֵ "+ dRs + " " + dRm + " " + dRe);
			
			/* if(dRs < 1e3 && dRe < 1e3)
			{
				if(dRm > 2*dRs && dRm > 2*dRe ) 
					lstNodes.get(i).setSingle_flag(1);
			}else if(dRs < 2e3 && dRe < 2e3)
			{
				if(dRm > dRs + 2e3 && dRm > dRe + 2e3 )
					lstNodes.get(i).setSingle_flag(1);
			}else if(dRs < 3e3 && dRe < 3e3)
			{
				if(dRm > dRs + 3e3 && dRm > dRe + 3e3 )
					lstNodes.get(i).setSingle_flag(1);
			}else 
			{
				double dRaverage = ( dRs + dRe )/2.0;
				if(dRm > dRs + dRaverage && dRm > dRe + dRaverage)
					lstNodes.get(i).setSingle_flag(1);
			} */
			
			double dRAverage = (dRs + dRe) / 2.0;
			if((fabs(dRs - dRAverage) < 0.5*dRAverage) &&	( fabs(dRe - dRAverage) < 0.5*dRAverage) )
			{
				//int nScale = (int) Math.floor(dRAverage / 1e3);
				double dScale = 2;
				if(dRm > dScale *dRAverage || dRm < (1.0 /dScale) * dRAverage )
				{
					lstNodes[i].setSingle_flag(1);
				}
			}
		}
	
	return true;
}

double FuzzySegmentor::MaxCurvatureAbs(vector<NodeInfo>& nodes)
{
	double dMaxCurvature = 0;
	double dMinCurvature = 1e10;

	for (int i = 0, n = nodes.size(); i < n; i++)
	{
		NodeInfo& node = nodes[i];
		double dCurvatureAbs = fabs(node.getCurvature());

		if (dCurvatureAbs > dMaxCurvature)
			dMaxCurvature = dCurvatureAbs;

		if (dCurvatureAbs < dMinCurvature)
			dMinCurvature = dCurvatureAbs;
	}
	return dMaxCurvature - dMinCurvature;
}



int FuzzySegmentor::TryNextJumpPoint(int nStartIndex, int nCurrentIndex, double dThreshold, std::vector<NodeInfo>& nodes)
{
	NodeInfo& sNode = nodes[nStartIndex];
	int nSize = nodes.size();

	int nNextPos = nCurrentIndex + 1;
	for (; nNextPos < nSize;nNextPos++)
	{
		NodeInfo& p0 = nodes[nNextPos - 1];
		NodeInfo& p1 = nodes[nNextPos];
			
		double dDelta0 = fabs(p1.getCurvature()) - fabs(p0.getCurvature());
		double dDelta1 = fabs(p1.getCurvature()) - fabs(sNode.getCurvature());

		dDelta0 = fabs(dDelta0);
		dDelta1 = fabs(dDelta1);

		if (dDelta0 > dThreshold && dDelta1 <= (nNextPos - nStartIndex)*dThreshold)
			continue;
		else
		{
			break;
		}
			
	}
	
	return nNextPos-1;
}

void FuzzySegmentor::AverageAndStandDerivation(vector<double>& vecValue, double& dAverage, double& dStandDerviation)
{
	int n = vecValue.size();

	double sum = 0;
	for (int i = 0; i < n; i++)
		sum += vecValue[i];
	
	dAverage = sum / n;

	sum = 0;
	for (int i = 0; i < n; i++)
	{
		double x = (vecValue[i] - dAverage)*(vecValue[i] - dAverage);
		sum += x;
	}
	dStandDerviation = sqrt(sum / n);
}

bool FuzzySegmentor::isFareJump(vector<double>& vecValue)
{
	int n = vecValue.size();
	if (n < 2)
		return false;

	//����ƫ��ֵ�ñ�׼��
	{
		double average, dStandDeviation;
		AverageAndStandDerivation(vecValue, average, dStandDeviation);
		if (dStandDeviation < fabs(average)*0.15)
			return true;
	}
	
	{
		if (n< 3)
			return false;

		vector<double> vecDifference;
		for (int i = 0; i <n - 1;i++)
		{
			vecDifference.push_back(vecValue[i + 1] - vecValue[i]);
		}

		bool bMonotone = true;
		bool bLastBool = (vecDifference[0] < 0);
		for (int i = 1; i < n - 1;i++)
		{
			bool b = vecDifference[i] < 0;
			if (b != bLastBool)
			{
				bMonotone = false;
				break;
			}
		}
		
		double average, dStandDeviation;
		AverageAndStandDerivation(vecDifference, average, dStandDeviation);

		//cout << " average : " << average << endl;
		//cout << "standard d : " << dStandDeviation << endl;
	
		//cout << "radio:\t" << dStandDeviation / average << endl;
		//cout << (bMonotone ? "yes" : "no") << endl;
		//cout << "-------------------------------" << endl;

	}
	
	return false;
}


bool FuzzySegmentor::selectAbnormalNodes2(vector<NodeInfo>& nodes, map<int, int>& mapAbnormalIndex)
{
	const double inflateScale = 1 * 300; //BMW ���鳡���� ʹ�� ϵ�� Ϊ 1*300
	const int MAXCHECKNUM = 30;
	double dTemp = MaxCurvatureAbs(nodes) / 1e4;
	double THRESHOLD = dTemp * inflateScale;

	bool bFirstUse = true;
	double dMaxValidThreshold = 0;

	int nCurrentPos = 0;
	for (int nPoints = nodes.size(); nCurrentPos < nPoints - 1;)
	{
		NodeInfo& ptCurrent = nodes[nCurrentPos];

		if (nCurrentPos == 7)
		{
			int a = 0;
		}

		int nNextPos = nCurrentPos + 1;
		
		vector<double> vecValue;
		
		for (; nNextPos < nCurrentPos + MAXCHECKNUM && nNextPos < nPoints - 1; nNextPos++)
		{
			NodeInfo& ptNextPoint = nodes[nNextPos];
			double dJumpValue = fabs(ptCurrent.getCurvature()) - fabs(ptNextPoint.getCurvature());

			if (fabs(dJumpValue) < (nNextPos - nCurrentPos) * THRESHOLD)
			{
				//////////////////////////////////////////////////////////////////////////
				//���� ����
				if (isFareJump(vecValue))
					break;

				nNextPos = TryNextJumpPoint(nCurrentPos,nNextPos,THRESHOLD,nodes);

				if (nNextPos - nCurrentPos > 1)
				{
					mapAbnormalIndex.insert(std::make_pair(nCurrentPos, nNextPos));
					//test purpose
					//cout << "jump interval:\t" << nCurrentPos << "  " << nNextPos << endl;
				}

				break;
			}

			vecValue.push_back(dJumpValue);			
		} //next end

		nCurrentPos = nNextPos;
	} //1

	return true;
}


//��һ���߶��У�ѡ���쳣�㣬�����ж���쳣�����䣬ȷ��ÿ���쳣����Ŀ�ʼ�ڵ�ͽ����ڵ��������
//����: nodes, ���߶δ�
//����� vecmapAbnormalIndex�� �쳣����
//bool FuzzySegmentor::selectAbnormalNodes(vector<NodeInfo> &nodes, vector<map<int, int> > & vecmapAbnormalIndex)
bool FuzzySegmentor::selectAbnormalNodes(vector<NodeInfo> &nodes, map<int, int>  & mapAbnormalIndex)
{
	vector<NodeInfo> segmentedNodes = nodes;

	//bool flag = getSingleAbnormalNodes(nodes);
	//if (!flag )
	//{
	//	return false;
	//}

	//�����任����ֵ
	const double MODIFYSCALE = 0.5;
	//�쳣���䷶Χ�ڵ�������ֵ
	const int	NABNORMALNUM = 10;

	mapAbnormalIndex.clear();

	int nPairIndex = 0;
	int nStartIndex = -1;
	int nEndIndex = -1;
	for (int index = 1; index < nodes.size()-1; index++ )
	{

		string nlinkid = (nodes[index].LinkID.c_str());
		string nposid = (nodes[index].PostnID.c_str());

		double dPreCurvature = nodes[index-1].getCurvature();
		double dCurCurvature = nodes[index].getCurvature();
		double dNextCurvature = nodes[index+1].getCurvature();

		double dModifyScaleCur = (dCurCurvature - dPreCurvature) / dPreCurvature;
		double dModifyScaleNext = (dNextCurvature - dPreCurvature) / dPreCurvature;

		dModifyScaleCur = fabs(dModifyScaleCur);
		dModifyScaleNext = fabs(dModifyScaleNext);

		//�����任���Ʒǳ�����
		if (dModifyScaleCur <= MODIFYSCALE && dModifyScaleNext >= MODIFYSCALE)
		{
			nPairIndex++;
			if (nPairIndex == 1)
			{
				nStartIndex = index;
			}	

			if (nPairIndex == 2)
			{
				nEndIndex = index;
				if (nEndIndex - nStartIndex <NABNORMALNUM)
				{
					mapAbnormalIndex.insert(map<int, int>::value_type(nStartIndex, nEndIndex));
				}				
			}
			nPairIndex = nPairIndex % 2;
		}
		//if (dModifyScaleCur >= MODIFYSCALE && dModifyScaleNext <= MODIFYSCALE)
		//{
		//	nPairIndex++;
		//	if (nPairIndex == 2)
		//	{
		//		nEndIndex = index;
		//		if (nEndIndex - nStartIndex <NABNORMALNUM)
		//		{
		//			mapAbnormalIndex.insert(map<int, int>::value_type(nStartIndex, nEndIndex));
		//		}				
		//	}
		//	nPairIndex = nPairIndex % 2;
		//}
		
	}

	return true;


	//if(3 == nodes.size()) {
	//	segmentedNodes = tagThreePoints(segmentedNodes, 0, 1);
	//	segmentedNodes = tagRefBySegResult(segmentedNodes);
	//}
	//else if(4 == nodes.size()) {
	//	segmentedNodes = tagFourPoints(segmentedNodes, 0, 1);
	//	segmentedNodes = tagRefBySegResult(segmentedNodes);
	//}
	//else if(nodes.size() >= 5) {
	//	int startIndex = 0, endIndex = nodes.size() - 1;
	//	int seg_belong = 1;

	//	while(startIndex < nodes.size() - 4) {
	//		endIndex = findNextEP(nodes, startIndex);

	//		if(endIndex == nodes.size() - 1) {
	//			endIndex = nodes.size();
	//		}

	//		segmentedNodes = tagOneSegment(segmentedNodes, startIndex, endIndex, seg_belong);
	//		startIndex = endIndex;
	//		seg_belong++;
	//	}

	//	if(startIndex == nodes.size() - 4) {
	//		segmentedNodes = tagFourPoints(segmentedNodes, startIndex, seg_belong);
	//	}
	//	else if(startIndex != nodes.size()) {
	//		if(1 == segmentedNodes.at(nodes.size()-2).getSingle_flag()) {
	//			segmentedNodes = tagOneSegment(segmentedNodes, startIndex, nodes.size(), seg_belong - 1);
	//		}
	//		else {
	//			NodeInfo sNode = segmentedNodes.at(nodes.size()-3);
	//			NodeInfo cNode = segmentedNodes.at(nodes.size()-2);
	//			NodeInfo eNode = segmentedNodes.at(nodes.size()-1);
	//			double checkDis = calcOriP2LDistance(cNode, sNode, eNode);

	//			if(checkDis < C_DISTANCE_LIMIT) {
	//				segmentedNodes = tagOneSegment(segmentedNodes, startIndex, nodes.size(), seg_belong - 1);
	//			}
	//			else {
	//				segmentedNodes = tagOneSegment(segmentedNodes, startIndex, nodes.size() - 2, seg_belong - 1);
	//				segmentedNodes = tagOneSegment(segmentedNodes, nodes.size() - 2, nodes.size(), seg_belong);
	//			}
	//		}
	//	}

	//	segmentedNodes = tagRefBySegResult(segmentedNodes);
	//}

	//return segmentedNodes;
}
