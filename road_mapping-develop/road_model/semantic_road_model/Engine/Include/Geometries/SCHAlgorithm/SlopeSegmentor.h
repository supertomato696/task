#ifndef _H_SLOPE_SEGMENTOR
#define _H_SLOPE_SEGMENTOR
#pragma once

#include "NodeInfo.h"
#include <vector>
#include <map>

class SlopeSegmentor
	{
	class RefResult	{
	public:
		RefResult(void)
			{
			ref_X = 0.0;
			ref_Y = 0.0;
			}
	private:
		//中间点到线段的映射坐标X
		double ref_X;

		//中间点到线段的映射坐标Y
		double ref_Y;
	public:
		double getRef_X() {
			return ref_X;
			}

		void setRef_X(double ref_X1) {
			ref_X = ref_X1;
			}

		double getRef_Y() {
			return ref_Y;
			}

		void setRef_Y(double ref_Y1) {
			ref_Y = ref_Y1;
			}					
		};

	public:
		SlopeSegmentor(void);
		~SlopeSegmentor(void);
	public:
		static int C_ENDS_POINTNUM;
		static double C_DISTANCE_LIMIT;
		static int C_LONGNUM_LIMIT;
	private:
		double calcOriP2LDistance(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode);
		void tagOneSegment(std::vector<NodeInfo>& nodes, int startIndex, int endIndex, int segBelong);
		std::vector<NodeInfo> tagThreePoints(std::vector<NodeInfo> nodes, int startIndex, int segBelong);
		RefResult calcPointRefOrdinate(NodeInfo cNode, NodeInfo sNode, NodeInfo eNode);

		void tagSegRef(std::vector<NodeInfo>& referedNodes, int sIndex, int eIndex);
		std::vector<NodeInfo> tagRefBySegResult(std::vector<NodeInfo> seggedNodes);
		std::vector<NodeInfo> tagFourPoints(std::vector<NodeInfo> nodes, int startIndex, int segBelong);
		int findNextEP(std::vector<NodeInfo>& nodes, int start);

		static int TryNextJumpPoint(int nStartIndex, int nCurrentIndex, double dThreshold, std::vector<NodeInfo>& nodes);

		static void ProcessTailJumpInterval(std::vector<NodeInfo> &vecNodeInfo, int nStartIndex);

	public:
		std::vector<NodeInfo> tagSegmentBelong(std::vector<NodeInfo> nodes);

		static void selectAbnormalNodes2(std::vector<NodeInfo>& nodes, std::map<int, int>& mapAbnormalIndex);
		static void smoothArrayNodeByLineInterpolation(std::vector<NodeInfo> &nodes, std::map<int, int> & mapIndex);
	};

#endif //_H_SLOPE_SEGMENTOR