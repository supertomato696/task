#include "Geometries/SCHAlgorithm/FuzzySegment.h"
#include <math.h>

FuzzySegment::FuzzySegment() :
m_nStartIndex(0),
m_nEndIndex(0),
m_nId(0),
m_pLeftSegment(NULL),
m_pRightSegment(NULL)
{}

double FuzzySegment::IncludeAngle(FuzzySegment* pSegment, std::vector<NodeInfo>& vecNodes, double yScale)
{
	if (NULL == pSegment)
		return 0;

	const double PI = 3.14159265;

	NodeInfo& node0 = vecNodes[m_nStartIndex];
	NodeInfo& node1 = vecNodes[m_nEndIndex];

	double x0 = node0.getReference_X();
	double y0 = node0.getReference_Y() * yScale ;
	double x1 = node1.getReference_X();
	double y1 = node1.getReference_Y()*yScale;

	NodeInfo& node2 = vecNodes[pSegment->m_nStartIndex];
	NodeInfo& node3 = vecNodes[pSegment->m_nEndIndex];

	double x2 = node2.getReference_X();
	double y2 = node2.getReference_Y()*yScale;
	double x3 = node3.getReference_X();
	double y3 = node3.getReference_Y()*yScale;

	double dCos = ((x1 - x0)*(x3 - x2) + (y1 - y0)*(y3 - y2)) / (sqrt((x1 - x0)*(x1 - x0) + (y1 - y0)* (y1 - y0)) * sqrt((x3 - x2)*(x3 - x2) + (y3 - y2)*(y3 - y2)));
	double dAngle = acos(dCos);

	if (dAngle < PI / 2)
		dAngle = PI - dAngle;

	dAngle = dAngle*(180.0 / PI);

	return dAngle;
}

int FuzzySegment::Number()
{
	return m_nEndIndex - m_nStartIndex ;
}

double FuzzySegment::AverageY(std::vector<NodeInfo>& vecNodes)
{
	/*double dElev = 0;
	for (int i = m_nStartIndex; i <= m_nEndIndex;i++)
	{
	NodeInfo& node = vecNodes[i];
	dElev += node.getReference_Y();
	}

	dElev = dElev / (m_nEndIndex - m_nStartIndex + 1);*/

	NodeInfo& node = vecNodes[m_nEndIndex];
	return node.getReference_Y();

	//return dElev;
}