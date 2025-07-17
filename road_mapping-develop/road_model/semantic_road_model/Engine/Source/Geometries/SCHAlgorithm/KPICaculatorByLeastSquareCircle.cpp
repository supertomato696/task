#include "Geometries/SCHAlgorithm/KPICaculatorByLeastSquareCicle.h"
#include "Base/Math.h"
#include "Geometries/SCHAlgorithm/computekpi.h"
#include "Geometries/LineString.h"

using namespace Engine;
using namespace Engine::Geometries;
using namespace Engine::Base;

Void PositionlizeUtility::GeneratePosition(const Base::Array<Coordinate*>* pCoorArray, const Base::Double dInterval, std::vector<NodeInfo>& vecPositionNodes)
{
	if (dInterval <= 0 || pCoorArray->IsEmpty())
		return;

	SizeT nPoints = pCoorArray->GetCount();

	SizeT positionNumber;
	Double lineDistance, distResidue;
	distResidue = 0;
	
	for (SizeT i = 0; i < nPoints - 1; i++)
	{
		Coordinate& ptFirstPoint = *(pCoorArray->ElementAt(i));
		Vector3d vDelta = *(pCoorArray->ElementAt(i + 1)) - ptFirstPoint ;
		Double dLength = vDelta.GetLength(); //线段长度

		if (Math::Equal(0,dLength))
			continue;

		lineDistance = dLength + distResidue;
		positionNumber = Base::Math::Floor(lineDistance / dInterval);

		Double u0 = (1.0 / dLength)*(dInterval - distResidue);
		Double u = (1.0 / dLength)*dInterval;

		//节点
		NodeInfo vertex;
		vertex.bVertex = true;
		vertex.nVertexIndex = i;
		vertex.setOrigin_X(ptFirstPoint.x);
		vertex.setOrigin_Y(ptFirstPoint.y);
		vecPositionNodes.push_back(vertex);

		for (SizeT j = (Math::Equal(distResidue,0) ? 1:0); j < positionNumber; j++)
		{
			Coordinate c = vDelta*(u*j + u0) + (*pCoorArray->ElementAt(i));

			NodeInfo node;
			node.bVertex = false;
			node.setOrigin_X(c.x);
			node.setOrigin_Y(c.y);
			vecPositionNodes.push_back(node);
		}

		distResidue = lineDistance - positionNumber*dInterval;
	}

	//LAST POINT
	NodeInfo vertex;
	vertex.bVertex = true;
	vertex.nVertexIndex = nPoints - 1;

	Coordinate& ptLastPoint = *(pCoorArray->ElementAt(nPoints -1));
	vertex.setOrigin_X(ptLastPoint.x);
	vertex.setOrigin_Y(ptLastPoint.y);

	vecPositionNodes.push_back(vertex);
}

CurvatureByLeastSquareCicle::CurvatureByLeastSquareCicle() :
m_pLineString(NULL)
{

}

Void CurvatureByLeastSquareCicle::SetData(const LineString* pLineString)
{
	m_pLineString = pLineString;
}

Bool CurvatureByLeastSquareCicle::SCHAtPosition(std::vector<NodeInfo>& vecNodes)
{
	if (vecNodes.empty())
		return false;

	XCalculateKPI caculator;
	caculator.CaculateAndSmooth(vecNodes);

	return true;
}

Bool CurvatureByLeastSquareCicle::SCHAtVertex(std::vector<Double>& vecCurvatures)
{
	if (NULL == m_pLineString)
		return false;

	std::vector<NodeInfo> vecNodes;
	const Double dPositionInterval = 1.0;
	PositionlizeUtility::GeneratePosition(m_pLineString->GetCoordinates(), dPositionInterval, vecNodes);

	SCHAtPosition(vecNodes);

	vecCurvatures.clear();
	//search value;
	for (SizeT i = 0, n = vecNodes.size(); i < n;i++)
	{
		NodeInfo& node = vecNodes[i];
		if (node.bVertex )
		{
			Double dCurvature = node.getCurvature();
			vecCurvatures.push_back(dCurvature);
		}
	}
	
	return true;
}




