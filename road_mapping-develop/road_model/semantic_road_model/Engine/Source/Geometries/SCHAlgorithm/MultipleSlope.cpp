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
* ���������ߵļн�
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
	double NORM = (DSX * DSX + DSY * DSY) * (DEX * DEX + DEY * DEY);    //ģ�˻�

	if (NORM != 0) {
		double COSVAL = DOTVEC / sqrt(NORM);
		FI = acos(COSVAL) * 180 / C_PI;
	}

	return FI;
}

//������ֵ
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
	//���߶�Խ��
	if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size())
	{
	}
	//��㵥��Խ��
	else if (nIndex < IntialExtendNum && nIndex + IntialExtendNum + 1 <= vecNodes.size())
	{
		nRightEnd = nIndex + IntialExtendNum + 1;
	}
	//�յ㵥��Խ��
	else if (nIndex >= IntialExtendNum && nIndex + IntialExtendNum + 1 > vecNodes.size())
	{
		nLeftEnd = nIndex - IntialExtendNum;
	}
	//���඼δԽ��
	else
	{
		nLeftEnd = nIndex - IntialExtendNum;
		nRightEnd = nIndex + IntialExtendNum + 1;
	}

	//2016.7.4 ���´��� ��ʱ����
	/*
	�������ڷֶμнǿ��ƣ�������ڷֶμнǳ����޶�ֵ�������õ�ǰ
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

		/*cout << "��    �� " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex ;
		cout << " ������ " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
		cout << "�Ҳ�Ƕȳ���:\t" << dAngle << endl;*/

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

		/*	cout << "��    �� " << currentNode.pSegment->m_nStartIndex << "  " << currentNode.pSegment->m_nEndIndex ;
			cout << " ������ " << nodeInfo.pSegment->m_nStartIndex << "  " << nodeInfo.pSegment->m_nEndIndex << endl;
			cout << "���Ƕȳ���:\t" << dAngle << endl;*/

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
* ���ܣ����ĺ�����������չ��ΪenlargeNumʱ���¶ȣ������չ��ΪenlargeNumʱ���������Ƿ��ޣ��Ƿ����������㳬��������������ֵ
* @param nodes:      ·����
* @param index:      ·���ϵڼ�����
* @param enlargeNum: ��չ����
* @return            :ResultSet  <isPermitted:�Ƿ�δ����ֵ��slope:�¶�>
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

	/*ȡ������ϵļ����,�������·���������յ㣬�ʹ������յ㿪ʼȡ*/
	getProperNodes(nodes, properNodes, index, enlargeNum);

	/*��С���˷�����Բ�����꣬Բ�뾶*/
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
		/*��С���˷�����Բ�����꣬Բ�뾶*/
		double A, B, R,SD;
		if (!CalcR(properNodes, A, B, R, SD))
		{
			//SD = numeric_limits<double>::max();
			SD = (std::numeric_limits<double>::max)();
		}
	
		/**/

		//����������ΪenLargeʱ��Բ�Ľ�
		NodeInfo node1 = properNodes.at(0);
		NodeInfo node2 = properNodes.at(N - 1);
		double circleAngel = calcAngel(node1.getOrigin_X(), node1.getOrigin_Y(), A, B, node2.getOrigin_X(), node2.getOrigin_Y());

		//��ֵ�ж���һ��ʼΪ����
		double threshold = getThreshold(N);

		if (SD > threshold /*|| circleAngel > C_ANGLE_LIMIT*/)
		{
			result.setPermitted(false);
		}
		
		//��õ㵽��һ����¶�
		double DB;

		//����ֵ��Χ�ڲż���
		if (result.IsPermitted() || isFinal)
		{
			//���һ��û�к���ĵ㣬����󵥶�����
			if (1 == nodes.at(index).getSingle_flag()) {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}
			else {
				calc_X = nodes.at(index).getReference_X();
				calc_Y = nodes.at(index).getReference_Y();
			}

			DB = calc_Y - B;

			/*�����¶�*/

			//Բ���ڼ������·�
			if (DB >= 0)
				slope = 90 - calcAngel(calc_X + 1, calc_Y, calc_X, calc_Y, A, B);
			//Բ���ڼ������Ϸ�
			else
				slope = calcAngel(calc_X + 1, calc_Y, calc_X, calc_Y, A, B) - 90;
		}

		/**/
	}
	//C*G == D*D ����ͬһ�ֶ���
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

	//������¶Ȳ������п��������Բ��Ϊֱ�����£�������ֱ�Ӻ���һ���������һ��
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
		//Ϊ���һ���㣬���ǰ��ĵ��������һ���¶�
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
* ʵ�ֹ��ܣ��Գ����м����¶����⴦��ȡһ�������˼����¶�ֵ��ע����
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
* ʵ�ֹ��ܣ�ͨ���ö��ַ��ж��Ƿ��ޣ���ú��ʵ���չ��ĸ���
* @param nodes
*/
int MultipleSlope::getProperEnlarge(vector<NodeInfo>& nodes, int index, int top, int low)
{
	int topCheck = top;
	int lowCheck = low;

	while (topCheck - lowCheck >= 2)
	{    //���ַ���ֹ����
		MultipleSlope::ResultSet result;
		result = checkIsPermitted(nodes, index, (topCheck + lowCheck) / 2, false);

		if (result.IsPermitted()) {
			lowCheck = (topCheck + lowCheck) / 2;      //û�г�����ֵ����������
		}
		else {
			topCheck = (topCheck + lowCheck) / 2;      //������ֵ����������
		}
	}

	return (topCheck + lowCheck - 1) / 2;              //����	    
}

/*
* ʵ�ֹ��ܣ����¶Ƚ��м�ƽ������һ���㳬�޵ڶ����㲻�������׵㸳�ڶ������¶�ֵ
*                             �м�㳬�ޣ����ߵ㲻���������ߵ�ƽ��ֵ
*                             β�㳬�ޣ������ڶ��㲻������β�㸳�����ڶ������¶�ֵ
* @return nodes:ƽ�����¶Ⱥ�ĵ�����
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
* ʵ�ֹ��ܣ��ö��ַ��Ե���·�����¶Ƚ��б�ע
* @param nodes
*/
vector<NodeInfo> MultipleSlope::tagPathCurveByDicho(vector<NodeInfo> nodes)
{
	int index = 0;

	/*
	 *	ֻ�������㣺ֱ����2�����
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
			node.setSlope(slope);                               //�趨�¶�Ϊȫ�β�����ϼ���ʱ���¶�
			index++;
		}
	}
	//·���ܵ������ڵ���5�Ų������
	else if (nodes.size() >= 5)
	{
		index = 0;

		/*****************************
		* ���㳤ֱ���ϵĵ���¶�
		*****************************/

		tagLongSegSlope(nodes);

		for (int i = 0; i < nodes.size(); i++)
		{
			NodeInfo node = nodes[i];

			//�ǳ�ֱ�²�������С���˷������¶�
			if (0 == node.getLongseg_flag())
			{
				MultipleSlope::ResultSet result;

				//�����16����ʱ��ֵδ����
				result = checkIsPermitted(nodes, index, C_ENLARGE_NUM, false);

				if (result.IsPermitted()) {
					//����������Ϊ16
					node.setEnlarge_num(C_ENLARGE_NUM);

					//�趨�¶�Ϊ16����ʱ���¶�
					node.setSlope(result.getSlope());
				}
				else {
					//�ҵ����ʵ�����������ʼ����32������0
					int properEnlarge = getProperEnlarge(nodes, index, C_ENLARGE_NUM, 0);

					if (properEnlarge < C_MIN_NUM) {
						//�������߸�������3����
						properEnlarge = C_MIN_NUM;
					}

					//����������Ϊ���ʵ�������
					node.setEnlarge_num(properEnlarge);

					result = checkIsPermitted(nodes, index, properEnlarge, true);

					//�趨�¶�Ϊ��������ʱ���¶�
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
