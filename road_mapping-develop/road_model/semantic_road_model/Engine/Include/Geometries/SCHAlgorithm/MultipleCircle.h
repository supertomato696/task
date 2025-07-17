#ifndef _H_MULTI_CIRCLE_
#define _H_MULTI_CIRCLE_
#pragma once

#include "NodeInfo.h"
#include <vector>
#include <map>

class ResultSet	
{
public:
	ResultSet(void)
	{
		isPermitted = true;
		curvature = 0.0;
		heading = 0.0;
		errType = 0;

		dSD = 0;
		x = 0;
		y = 0;
		r = 0;
	}

	ResultSet(const ResultSet& r)
	{
		isPermitted = r.isPermitted;
		curvature = r.curvature;
		heading = r.heading;
		errType = r.errType;
		dSD = r.dSD;
		x = r.x;
		y = r.y;
		this->r = r.r;
	}

/*private:*/
public:
	bool isPermitted;
	double curvature;
	double heading;
	unsigned int errType; // 0x00 : no error , 0x01 : 标准差错误 0x02:角度超限 0x04:拟合点超限
	
	double dSD;
	double x;
	double y;
	double r;

public:
	bool IsPermitted() {
		return isPermitted;
		}

	void setPermitted(bool isPermitted1) {
		isPermitted = isPermitted1;
		}

	double getCurvature() {
		return curvature;
		}

	void setCurvature(double curvature1) {
		curvature = curvature1;
		}

	double getHeading() {
		return heading;
		}

	void setHeading(double heading1) {
		heading = heading1;
		}
	};

class MultipleCircle
	{
	public :
		struct ITEM
		{
			int nIndex;
			int nLength;
		} ;

	public:
		MultipleCircle(void);
		~MultipleCircle(void);
	private:
		void getProperNodes(std::vector<NodeInfo>& vecNodes, std::vector<NodeInfo>& vecProperNodes, int nIndex, int IntialExtendNum, int& nInnerIndex);

		double calcAngel(double XS, double YS, double XM, double YM, double XE, double YE);
		double getThreshold(int nCount);
		double calcAzimuth(double A, double B, double X, double Y);
		ResultSet checkIsPermitted(std::vector<NodeInfo>& nodes, int index, int enlargeNum, bool isFinal);
		int getProperEnlarge(std::vector<NodeInfo>& nodes, int index, int top, int low, ResultSet& result);
		void tagLongSegHeading(std::vector<NodeInfo>& nodes);
	public:
		void tagPathCurveByDicho(std::vector<NodeInfo>& nodes);

		bool smoothArrayNodeByLineInterpolation(std::vector<NodeInfo> &nodes, std::map<int, int> & mapIndex, bool absSmooth);

		void removeSignJump(std::vector<NodeInfo>& nodes, int maxInterval);

		void SmoothPositiveNegtiveJump(std::vector<NodeInfo>& nodes, int SmoothPoints);

		int FindStablePosition(std::vector<NodeInfo>& nodes, int nIndex, int MaxExtend, bool bLeft = true);

		void CalcRadius(std::vector<NodeInfo>& vecNodes, int nIndex, double& X_R, double& Y_R, double& R, double& SD);

		void CalcRadius_WrongLSF(std::vector<NodeInfo>& vecNodes, int nIndex, double& X_R, double& Y_R, double& R, double& SD);

		void CalcCurvatureTwoPoints(std::vector<NodeInfo>& vecNodes);

		void CalcStandardDeviation(std::vector<NodeInfo>& vecNodes, int nIndex, double a, double b, double r, double& sd);
	};

#endif //_H_MULTI_CIRCLE_