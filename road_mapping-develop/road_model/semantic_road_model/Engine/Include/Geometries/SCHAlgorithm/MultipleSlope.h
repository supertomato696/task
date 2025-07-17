#ifndef _H_MULTIPLESLOPE_
#define  _H_MULTIPLESLOPE_
#pragma once

#include "NodeInfo.h"
#include <vector>

class MultipleSlope
{
	class ResultSet
	{
	public:
		ResultSet(void)
		{
			//一开始设置为不超限
			isPermitted = true;

			//坡度初始化为0
			slope = 0.0;
		}
	private:
		bool isPermitted;
		double slope;
	public:
		bool IsPermitted() {
			return isPermitted;
		}

		void setPermitted(bool isPermitted1) {
			isPermitted = isPermitted1;
		}

		double getSlope() {
			return slope;
		}

		void setSlope(double slope1) {
			slope = slope1;
		}
	};

public:
	MultipleSlope(void);
	~MultipleSlope(void);
private:
	void getProperNodes(std::vector<NodeInfo>& vecNodes, std::vector<NodeInfo>& vecProperNodes, int nIndex, int IntialExtendNum);
	double calcAngel(double XS, double YS, double XM, double YM, double XE, double YE);
	double getThreshold(int nCount);
	ResultSet checkIsPermitted(std::vector<NodeInfo>& nodes, int index, int enlargeNum, bool isFinal);
	void tagLongSegSlope(std::vector<NodeInfo>& nodes);
	int getProperEnlarge(std::vector<NodeInfo>& nodes, int index, int top, int low);
	void smoothSlopeValue(std::vector<NodeInfo>& nodes);
public:
	std::vector<NodeInfo> tagPathCurveByDicho(std::vector<NodeInfo> nodes);
};
#endif //_H_MULTIPLESLOPE_