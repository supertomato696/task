#ifndef _H_FUZZY_SEGMENTOR_
#define _H_FUZZY_SEGMENTOR_
#pragma once
#include "NodeInfo.h"
#include <vector>
#include <map>

class RefResult	
{
public:
	RefResult(void)
		{
		ref_X = 0.0;
		ref_Y = 0.0;
		}
private:
	double ref_X;
	double ref_Y;
public:
	double getRef_X() {
		return ref_X;
		}

	void setRef_X(double x) {
		ref_X = x;
		}

	double getRef_Y() {
		return ref_Y;
		}

	void setRef_Y(double y) {
		ref_Y = y;
		}					
	};

class FuzzySegmentor
	{
	public:
		FuzzySegmentor(void);
		~FuzzySegmentor(void);
	public:
		static int C_ENDS_POINTNUM;
		static double C_DISTANCE_LIMIT;
		static int C_LONGNUM_LIMIT;
	private:
		double calcOriP2LDistance(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode);
		void tagOneSegment(std::vector<NodeInfo>& nodes, int startIndex, int endIndex, int segBelong);
		void tagThreePoints(std::vector<NodeInfo>& nodes, int startIndex, int segBelong);
		RefResult calcPointRefOrdinate(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode);
		void tagSegRef(std::vector<NodeInfo>& referedNodes, int sIndex, int eIndex);
		void tagRefBySegResult(std::vector<NodeInfo>& seggedNodes);
		void tagFourPoints(std::vector<NodeInfo>& nodes, int startIndex, int segBelong);
		int findNextEP(std::vector<NodeInfo>& nodes, int start);

		//void MinAndMaxCurvature(vector<NodeInfo>& nodes,double& minCurvature,double& maxCurvature);
		//���ʾ���ֵ �ķ�Χ
		double MaxCurvatureAbs(std::vector<NodeInfo>& nodes);

		int TryNextJumpPoint(int nStartIndex, int nCurrentIndex, double dThreshold,std::vector<NodeInfo>& nodes);

	public:
		void tagSegmentBelong(std::vector<NodeInfo>& nodes);

		//��һ���߶��У�ѡ���쳣�㣬�����ж���쳣�����䣬ȷ��ÿ���쳣����Ŀ�ʼ�ڵ�ͽ����ڵ��������
		//����: nodes, ���߶δ�
		//����� vecmapAbnormalIndex�� �쳣����
		//bool selectAbnormalNodes(vector<NodeInfo> &nodes, vector<map<int, int> > & vecmapAbnormalIndex);

		bool selectAbnormalNodes(std::vector<NodeInfo> &nodes, std::map<int, int>  & mapAbnormalIndex);

		//by qiuli's method 
		bool selectAbnormalNodes2(std::vector<NodeInfo>& nodes, std::map<int, int>& mapAbnormalIndex);

		bool getSingleAbnormalNodes(std::vector<NodeInfo> &nodes);

		bool isFareJump(std::vector<double>& vecValue);

		void AverageAndStandDerivation(std::vector<double>& vecValue, double& dAverage, double& dStandDerviation);
	};
#endif