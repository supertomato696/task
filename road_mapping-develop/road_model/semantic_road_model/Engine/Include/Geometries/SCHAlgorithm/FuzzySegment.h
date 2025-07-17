#ifndef _H_FUZZY_SEGMENT_
#define _H_FUZZY_SEGMENT_
#include <vector>
#include "NodeInfo.h"

class FuzzySegment
{
public:
	int m_nStartIndex;
	int m_nEndIndex;
	int m_nId;

	FuzzySegment* m_pLeftSegment;
	FuzzySegment* m_pRightSegment;

public:
	FuzzySegment();
	//Ω«∂»÷µ in degree
	double IncludeAngle(FuzzySegment* pSegment, std::vector<NodeInfo>& vecNodes,double yScale = 1.0);
	int Number();

	double AverageY(std::vector<NodeInfo>& vecNodes);
};

#endif //_H_FUZZY_SEGMENT_



