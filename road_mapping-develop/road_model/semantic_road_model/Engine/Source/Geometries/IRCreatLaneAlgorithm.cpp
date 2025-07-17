/******************************************************************
作者: test
日期: 2021-8-18 11:19
文件名称:IRCreatLaneAlgorithm.cpp
简要描述:
******************************************************************/

#include "Geometries/IRCreatLaneAlgorithm.h"
#include "Geometries/BaseAlgorithm.h"
#include "Geometries/BaseAlgorithm3D.h"
#include "Geometries/GeometryAlgorithm.h"
#include "Geometries/Coordinate.h"
#include "Geometries/LineString.h"
#include "Geometries/LinearRing.h"

#define val_fmax(a, b) (((a) > (b)) ? (a) : (b))
#define val_fmin(a, b) (((a) < (b)) ? (a) : (b))

using namespace Engine::Geometries;
using namespace Engine;

bool IRCreatLaneAlgorithm::IsPointPrjToMultLine(
	const Engine::Geometries::Coordinate *pHitPoint,
	const Engine::Geometries::LineString *pMultLine)
{
	if (pHitPoint == NULL || pMultLine == NULL || pMultLine->GetNumPoints() < 2)
	{
		return false;
	}

	bool bRst = false;
	int iCoorNum = pMultLine->GetNumPoints();
	Array<Coordinate *> *pArrCoors = pMultLine->GetCoordinates();
	for (int i = 0; i < iCoorNum - 1; i++)
	{
		bool bIsPrjToLine = BaseAlgorithm::IsProjectToLineset(pHitPoint, pArrCoors->GetAt(i), pArrCoors->GetAt(i + 1));
		if (bIsPrjToLine)
		{
			bRst = true;
			break;
		}
	}
	return bRst;
}

double IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(
	const Engine::Geometries::LineString *line1,
	const Engine::Geometries::LineString *line2,
	double &dMaxWidth, double &dMinWidth)
{
	if (line1 == NULL || line1->GetNumPoints() < 2 ||
		line2 == NULL || line2->GetNumPoints() < 2)
	{
		return -1.0;
	}

	dMaxWidth = 0.0;
	dMinWidth = 1000000.0;

	Array<Geometries::Coordinate *> *pCoors1 = line1->GetCoordinates();
	Array<Geometries::Coordinate *> *pCoors2 = line2->GetCoordinates();

	Int32 ncoords1 = pCoors1->GetCount();
	Int32 ncoords2 = pCoors2->GetCount();

	struct WidthInfo
	{
		Double MaxWidth;
		Double MinWidth;
		Double TotalWidth;
		Int32 IndexS;
		Int32 IndexE;
		WidthInfo()
		{
			MaxWidth = 0.0;
			MinWidth = 1000000.0;
			TotalWidth = 0.0;
			IndexS = -1;
			IndexE = -1;
		}
	};
	Array<WidthInfo *> arrWidthInfo1;
	Array<WidthInfo *> arrWidthInfo2;
	WidthInfo *pWidthInfoTmp = NULL;

	Engine::Geometries::Coordinate pntProject;
	Int32 nSegIndex = -1;
	Double curWidth = 0.0;

	Coordinate coord1;
	Coordinate coord2;
	Coordinate coord3;
	for (int i = 0; i < ncoords1; i++)
	{
		if (!BaseAlgorithm::GetNearestPntToLineset(pCoors1->GetAt(i), pCoors2, pntProject, nSegIndex))
		{
			continue;
		}
		bool bNeedJudgeDegree = false;
		if (nSegIndex == 0 && pntProject.DistanceXY(*(pCoors2->GetAt(0))) < Geometries_EP)
		{
			coord1 = Coordinate(pCoors1->GetAt(i)->x, pCoors1->GetAt(i)->y, 0.0);
			coord2 = Coordinate(pCoors2->GetAt(0)->x, pCoors2->GetAt(0)->y, 0.0);
			coord3 = Coordinate(pCoors2->GetAt(1)->x, pCoors2->GetAt(1)->y, 0.0);
			bNeedJudgeDegree = true;
		}
		if (nSegIndex == ncoords2 - 2 && pntProject.DistanceXY(*(pCoors2->GetAt(ncoords2 - 1))) < Geometries_EP)
		{
			coord1 = Coordinate(pCoors1->GetAt(i)->x, pCoors1->GetAt(i)->y, 0.0);
			coord2 = Coordinate(pCoors2->GetAt(ncoords2 - 1)->x, pCoors2->GetAt(ncoords2 - 1)->y, 0.0);
			coord3 = Coordinate(pCoors2->GetAt(ncoords2 - 2)->x, pCoors2->GetAt(ncoords2 - 2)->y, 0.0);
			bNeedJudgeDegree = true;
		}
		if (bNeedJudgeDegree && Math::Fabs(GeometryAlgorithm::RadianToDegree(GeometryAlgorithm::ComputeAngle(coord1, coord2, coord3)) - 90.0) > Geometries_EP)
		{
			continue;
		}
		if (pWidthInfoTmp == NULL || pWidthInfoTmp->IndexE != i - 1)
		{
			pWidthInfoTmp = new WidthInfo();
			pWidthInfoTmp->IndexS = i;
			arrWidthInfo1.Add(pWidthInfoTmp);
		}
		curWidth = pCoors1->GetAt(i)->DistanceXY(pntProject);
		if (curWidth > pWidthInfoTmp->MaxWidth)
		{
			pWidthInfoTmp->MaxWidth = curWidth;
		}
		if (curWidth < pWidthInfoTmp->MinWidth)
		{
			pWidthInfoTmp->MinWidth = curWidth;
		}
		pWidthInfoTmp->IndexE = i;
		pWidthInfoTmp->TotalWidth += curWidth;
	}

	for (int i = 0; i < ncoords2; i++)
	{
		if (!BaseAlgorithm::GetNearestPntToLineset(pCoors2->GetAt(i), pCoors1, pntProject, nSegIndex))
		{
			continue;
		}
		bool bNeedJudgeDegree = false;
		if (nSegIndex == 0 && pntProject.DistanceXY(*(pCoors1->GetAt(0))) < Geometries_EP)
		{
			coord1 = Coordinate(pCoors2->GetAt(i)->x, pCoors2->GetAt(i)->y, 0.0);
			coord2 = Coordinate(pCoors1->GetAt(0)->x, pCoors1->GetAt(0)->y, 0.0);
			coord3 = Coordinate(pCoors1->GetAt(1)->x, pCoors1->GetAt(1)->y, 0.0);
			bNeedJudgeDegree = true;
		}
		if (nSegIndex == line1->GetNumPoints() - 2 && pntProject.DistanceXY(*(pCoors1->GetAt(ncoords1 - 1))) < Geometries_EP)
		{
			coord1 = Coordinate(pCoors2->GetAt(i)->x, pCoors2->GetAt(i)->y, 0.0);
			coord2 = Coordinate(pCoors1->GetAt(ncoords1 - 1)->x, pCoors1->GetAt(ncoords1 - 1)->y, 0.0);
			coord3 = Coordinate(pCoors1->GetAt(ncoords1 - 2)->x, pCoors1->GetAt(ncoords1 - 2)->y, 0.0);
			bNeedJudgeDegree = true;
		}
		if (bNeedJudgeDegree && Math::Fabs(GeometryAlgorithm::RadianToDegree(GeometryAlgorithm::ComputeAngle(coord1, coord2, coord3)) - 90.0) > Geometries_EP)
		{
			continue;
		}
		if (pWidthInfoTmp == NULL || pWidthInfoTmp->IndexE != i - 1)
		{
			pWidthInfoTmp = new WidthInfo();
			pWidthInfoTmp->IndexS = i;
			arrWidthInfo2.Add(pWidthInfoTmp);
		}
		curWidth = pCoors2->GetAt(i)->DistanceXY(pntProject);
		if (curWidth > pWidthInfoTmp->MaxWidth)
		{
			pWidthInfoTmp->MaxWidth = curWidth;
		}
		if (curWidth < pWidthInfoTmp->MinWidth)
		{
			pWidthInfoTmp->MinWidth = curWidth;
		}
		pWidthInfoTmp->IndexE = i;
		pWidthInfoTmp->TotalWidth += curWidth;
	}

	while (arrWidthInfo1.GetCount() > 1)
	{
		if (arrWidthInfo1.GetAt(0) == NULL)
		{
			pWidthInfoTmp = arrWidthInfo1.GetAt(0);
			arrWidthInfo1.Delete(0);
			continue;
		}
		else if (arrWidthInfo1.GetAt(1) == NULL)
		{
			pWidthInfoTmp = arrWidthInfo1.GetAt(1);
			arrWidthInfo1.Delete(1);
			continue;
		}
		else if (arrWidthInfo1.GetAt(0)->TotalWidth / (arrWidthInfo1.GetAt(0)->IndexE - arrWidthInfo1.GetAt(0)->IndexS + 1) >
				 arrWidthInfo1.GetAt(1)->TotalWidth / (arrWidthInfo1.GetAt(1)->IndexE - arrWidthInfo1.GetAt(1)->IndexS + 1))
		{
			pWidthInfoTmp = arrWidthInfo1.GetAt(0);
			arrWidthInfo1.Delete(0);
		}
		else
		{
			pWidthInfoTmp = arrWidthInfo1.GetAt(1);
			arrWidthInfo1.Delete(1);
		}
		DELETE_PTR(pWidthInfoTmp);
	}

	while (arrWidthInfo2.GetCount() > 1)
	{
		if (arrWidthInfo2.GetAt(0) == NULL)
		{
			pWidthInfoTmp = arrWidthInfo2.GetAt(0);
			arrWidthInfo2.Delete(0);
			continue;
		}
		else if (arrWidthInfo2.GetAt(1) == NULL)
		{
			pWidthInfoTmp = arrWidthInfo2.GetAt(1);
			arrWidthInfo2.Delete(1);
			continue;
		}
		else if (arrWidthInfo2.GetAt(0)->TotalWidth / (arrWidthInfo2.GetAt(0)->IndexE - arrWidthInfo2.GetAt(0)->IndexS + 1) >
				 arrWidthInfo2.GetAt(1)->TotalWidth / (arrWidthInfo2.GetAt(1)->IndexE - arrWidthInfo2.GetAt(1)->IndexS + 1))
		{
			pWidthInfoTmp = arrWidthInfo2.GetAt(0);
			arrWidthInfo2.Delete(0);
		}
		else
		{
			pWidthInfoTmp = arrWidthInfo2.GetAt(1);
			arrWidthInfo2.Delete(1);
		}
		DELETE_PTR(pWidthInfoTmp);
	}

	Double dWidthDst = -1.0;
	pWidthInfoTmp = NULL;
	if (arrWidthInfo1.IsEmpty() && arrWidthInfo2.IsEmpty())
	{
		return -1.0;
	}
	else if (arrWidthInfo1.IsEmpty())
	{
		pWidthInfoTmp = arrWidthInfo2.GetAt(0);
		arrWidthInfo2.Clear();
	}
	else if (arrWidthInfo2.IsEmpty())
	{
		pWidthInfoTmp = arrWidthInfo1.GetAt(0);
		arrWidthInfo1.Clear();
	}
	else if (arrWidthInfo1.GetAt(0)->MaxWidth < arrWidthInfo2.GetAt(0)->MinWidth)
	{
		pWidthInfoTmp = arrWidthInfo1.GetAt(0);
		arrWidthInfo1.Clear();
	}
	else if (arrWidthInfo1.GetAt(0)->MinWidth > arrWidthInfo2.GetAt(0)->MaxWidth)
	{
		pWidthInfoTmp = arrWidthInfo2.GetAt(0);
		arrWidthInfo2.Clear();
	}
	else
	{
		dMaxWidth = arrWidthInfo1.GetAt(0)->MaxWidth > arrWidthInfo2.GetAt(0)->MaxWidth ? arrWidthInfo1.GetAt(0)->MaxWidth : arrWidthInfo2.GetAt(0)->MaxWidth;
		dMinWidth = arrWidthInfo1.GetAt(0)->MinWidth < arrWidthInfo2.GetAt(0)->MinWidth ? arrWidthInfo1.GetAt(0)->MinWidth : arrWidthInfo2.GetAt(0)->MinWidth;
		dWidthDst = (arrWidthInfo1.GetAt(0)->TotalWidth + arrWidthInfo2.GetAt(0)->TotalWidth) /
					(arrWidthInfo1.GetAt(0)->IndexE - arrWidthInfo1.GetAt(0)->IndexS + arrWidthInfo2.GetAt(0)->IndexE - arrWidthInfo2.GetAt(0)->IndexS + 2);
	}

	if (pWidthInfoTmp != NULL)
	{
		dMaxWidth = pWidthInfoTmp->MaxWidth;
		dMinWidth = pWidthInfoTmp->MinWidth;
		dWidthDst = pWidthInfoTmp->TotalWidth / (pWidthInfoTmp->IndexE - pWidthInfoTmp->IndexS + 1);
		DELETE_PTR(pWidthInfoTmp);
	}

	while (!arrWidthInfo1.IsEmpty())
	{
		pWidthInfoTmp = arrWidthInfo1.GetAt(0);
		arrWidthInfo1.Delete(0);
		DELETE_PTR(pWidthInfoTmp);
	}

	while (!arrWidthInfo2.IsEmpty())
	{
		pWidthInfoTmp = arrWidthInfo2.GetAt(0);
		arrWidthInfo2.Delete(0);
		DELETE_PTR(pWidthInfoTmp);
	}

	return dWidthDst;
}

double IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(
	const Engine::Geometries::LineString *line1,
	const Engine::Geometries::LineString *line2)
{
	Double dMaxWidth = 0.0;
	Double dMinWidth = 1000000.0;
	return CalShortLAvgDstToLongL(line1, line2, dMaxWidth, dMinWidth);
}

void IRCreatLaneAlgorithm::SortAndRemoveRepByPrjMeasure(
	Engine::Geometries::LineString *lineRefLine,
	Array<Geometries::LineString *> &pOrgLines)
{
	if (lineRefLine == NULL || lineRefLine->GetNumPoints() < 2 ||
		pOrgLines.GetCount() < 2)
	{
		return;
	}

	int iLineNum = pOrgLines.GetCount();
	Array<double> arrSPMeasure;
	Array<double> arrEPMeasure;
	arrSPMeasure.SetSize(iLineNum);
	arrEPMeasure.SetSize(iLineNum);
	Coordinate coorSP;
	Coordinate coorEP;
	Coordinate coorPrjS;
	Coordinate coorPrjE;
	int iPrjSIndex = -1;
	int iPrjEIndex = -1;
	double dMeasureS = 0.0;
	double dMeasureE = 0.0;
	LineString *pLine = NULL;

	// 计算所有线的首末点刻度
	for (int i = iLineNum - 1; i >= 0; i--)
	{
		pLine = pOrgLines.GetAt(i);
		if (pLine == NULL || pLine->GetNumPoints() < 2)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}
		coorSP = pLine->GetStartPoint()->ToCoordinate();
		coorEP = pLine->GetEndPoint()->ToCoordinate();
		bool bRstS = BaseAlgorithm::GetNearestPntToLineset(&coorSP, lineRefLine->GetCoordinates(), coorPrjS, iPrjSIndex);
		bool bRstE = BaseAlgorithm::GetNearestPntToLineset(&coorEP, lineRefLine->GetCoordinates(), coorPrjE, iPrjEIndex);
		if (!bRstS || !bRstE)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}
		bRstS = lineRefLine->GetMAtPoint(&coorPrjS, iPrjSIndex, dMeasureS);
		bRstE = lineRefLine->GetMAtPoint(&coorPrjE, iPrjEIndex, dMeasureE);
		if (!bRstS || !bRstE)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}

		arrSPMeasure.SetAt(i, dMeasureS);
		arrEPMeasure.SetAt(i, dMeasureE);
	}

	if (pOrgLines.IsEmpty() || pOrgLines.GetCount() < 2)
	{
		return;
	}

	// 按起始刻度排序
	iLineNum = pOrgLines.GetCount();
	for (int i = 0; i < iLineNum - 1; i++)
	{
		double dCurSMeasure = arrSPMeasure.GetAt(i);
		int iMinMeaInd = i;
		for (int j = i + 1; j < iLineNum; j++)
		{
			if (dCurSMeasure > arrSPMeasure.GetAt(j))
			{
				dCurSMeasure = arrSPMeasure.GetAt(j);
				iMinMeaInd = j;
			}
		}
		if (iMinMeaInd != i)
		{
			pLine = pOrgLines.GetAt(i);
			pOrgLines.SetAt(i, pOrgLines.GetAt(iMinMeaInd));
			pOrgLines.SetAt(iMinMeaInd, pLine);
			dMeasureS = arrSPMeasure.GetAt(i);
			arrSPMeasure.SetAt(i, arrSPMeasure.GetAt(iMinMeaInd));
			arrSPMeasure.SetAt(iMinMeaInd, dMeasureS);
			dMeasureE = arrEPMeasure.GetAt(i);
			arrEPMeasure.SetAt(i, arrEPMeasure.GetAt(iMinMeaInd));
			arrEPMeasure.SetAt(iMinMeaInd, dMeasureE);
		}
	}

	// 排序后若前线的末点刻度大于后线的首点刻度则重叠
	// 重叠处理方式暂定保留到参照线平均距离短的
	for (int i = 1; i < iLineNum;)
	{
		if (arrEPMeasure.GetAt(i - 1) <= arrSPMeasure.GetAt(i))
		{
			i++;
			continue;
		}
		pLine = pOrgLines.GetAt(i);
		double dAvgDisB = IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(pLine, lineRefLine);
		pLine = pOrgLines.GetAt(i - 1);
		double dAvgDisF = IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(pLine, lineRefLine);
		if (dAvgDisB < 0 || dAvgDisF < 0)
		{
			i++;
			continue;
		}

		int iDelIndex = i;
		double dLengthRatio = pOrgLines.GetAt(i)->GetLength() / pOrgLines.GetAt(i - 1)->GetLength();
		if (dLengthRatio >= 1.1 || (dLengthRatio < 1.1 && dLengthRatio > 0.9 && dAvgDisB < dAvgDisF))
		{
			iDelIndex = i - 1;
		}
		pLine = pOrgLines.GetAt(iDelIndex);
		DELETE_PTR(pLine);
		pOrgLines.Delete(iDelIndex);
		arrSPMeasure.Delete(iDelIndex);
		arrEPMeasure.Delete(iDelIndex);
		iLineNum = pOrgLines.GetCount();
	}
}

void IRCreatLaneAlgorithm::SortAndRemoveRepByPrjMeasure(
	Engine::Geometries::LineString *lineRefLine,
	Array<Geometries::LineString *> &pOrgLines,
	double &dPrjLength, double &dPrjScale)
{
	if (lineRefLine == NULL || lineRefLine->GetNumPoints() < 2 ||
		pOrgLines.GetCount() < 1)
	{
		return;
	}

	int iLineNum = pOrgLines.GetCount();
	Array<double> arrSPMeasure;
	Array<double> arrEPMeasure;
	arrSPMeasure.SetSize(iLineNum);
	arrEPMeasure.SetSize(iLineNum);
	Coordinate coorSP;
	Coordinate coorEP;
	Coordinate coorPrjS;
	Coordinate coorPrjE;
	int iPrjSIndex = -1;
	int iPrjEIndex = -1;
	double dMeasureS = 0.0;
	double dMeasureE = 0.0;
	LineString *pLine = NULL;

	// 计算所有线的首末点刻度
	for (int i = iLineNum - 1; i >= 0; i--)
	{
		pLine = pOrgLines.GetAt(i);
		if (pLine == NULL || pLine->GetNumPoints() < 2)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}
		coorSP = pLine->GetStartPoint()->ToCoordinate();
		coorEP = pLine->GetEndPoint()->ToCoordinate();
		bool bRstS = BaseAlgorithm::GetNearestPntToLineset(&coorSP, lineRefLine->GetCoordinates(), coorPrjS, iPrjSIndex);
		bool bRstE = BaseAlgorithm::GetNearestPntToLineset(&coorEP, lineRefLine->GetCoordinates(), coorPrjE, iPrjEIndex);
		if (!bRstS || !bRstE)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}
		bRstS = lineRefLine->GetMAtPoint(&coorPrjS, iPrjSIndex, dMeasureS);
		bRstE = lineRefLine->GetMAtPoint(&coorPrjE, iPrjEIndex, dMeasureE);
		if (!bRstS || !bRstE)
		{
			DELETE_PTR(pLine);
			pOrgLines.Delete(i);
			arrSPMeasure.Delete(i);
			arrEPMeasure.Delete(i);
			continue;
		}

		arrSPMeasure.SetAt(i, dMeasureS);
		arrEPMeasure.SetAt(i, dMeasureE);
	}

	if (pOrgLines.IsEmpty() || pOrgLines.GetCount() < 1)
	{
		return;
	}

	// 按起始刻度排序
	iLineNum = pOrgLines.GetCount();
	for (int i = 0; i < iLineNum - 1; i++)
	{
		double dCurSMeasure = arrSPMeasure.GetAt(i);
		int iMinMeaInd = i;
		for (int j = i + 1; j < iLineNum; j++)
		{
			if (dCurSMeasure > arrSPMeasure.GetAt(j))
			{
				dCurSMeasure = arrSPMeasure.GetAt(j);
				iMinMeaInd = j;
			}
		}
		if (iMinMeaInd != i)
		{
			pLine = pOrgLines.GetAt(i);
			pOrgLines.SetAt(i, pOrgLines.GetAt(iMinMeaInd));
			pOrgLines.SetAt(iMinMeaInd, pLine);
			dMeasureS = arrSPMeasure.GetAt(i);
			arrSPMeasure.SetAt(i, arrSPMeasure.GetAt(iMinMeaInd));
			arrSPMeasure.SetAt(iMinMeaInd, dMeasureS);
			dMeasureE = arrEPMeasure.GetAt(i);
			arrEPMeasure.SetAt(i, arrEPMeasure.GetAt(iMinMeaInd));
			arrEPMeasure.SetAt(iMinMeaInd, dMeasureE);
		}
	}

	// 排序后若前线的末点刻度大于后线的首点刻度则重叠
	// 重叠处理方式暂定保留到参照线平均距离短的
	for (int i = 1; i < iLineNum;)
	{
		if (arrEPMeasure.GetAt(i - 1) <= arrSPMeasure.GetAt(i))
		{
			i++;
			continue;
		}
		pLine = pOrgLines.GetAt(i);
		double dAvgDisB = IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(pLine, lineRefLine);
		pLine = pOrgLines.GetAt(i - 1);
		double dAvgDisF = IRCreatLaneAlgorithm::CalShortLAvgDstToLongL(pLine, lineRefLine);
		if (dAvgDisB < 0 || dAvgDisF < 0)
		{
			i++;
			continue;
		}

		int iDelIndex = i;
		double dLengthRatio = pOrgLines.GetAt(i)->GetLength() / pOrgLines.GetAt(i - 1)->GetLength();
		if (dLengthRatio >= 2 || (dLengthRatio < 2 && dLengthRatio > 0.5 && dAvgDisB < dAvgDisF))
		{
			iDelIndex = i - 1;
		}
		pLine = pOrgLines.GetAt(iDelIndex);
		DELETE_PTR(pLine);
		pOrgLines.Delete(iDelIndex);
		arrSPMeasure.Delete(iDelIndex);
		arrEPMeasure.Delete(iDelIndex);
		iLineNum = pOrgLines.GetCount();
	}

	dPrjLength = 0.0;
	dPrjScale = 0.0;
	for (int i = 0; i < pOrgLines.GetCount(); i++)
	{
		dPrjLength += arrEPMeasure.GetAt(i) - arrSPMeasure.GetAt(i);
	}
	dPrjScale = dPrjLength / lineRefLine->GetLength();
}

void IRCreatLaneAlgorithm::SortPrjPtByPrjMeasure(
	Engine::Geometries::LineString *lineRefLine,
	Array<Geometries::Coordinate *> &pPrjPts)
{
	if (lineRefLine == NULL || lineRefLine->GetNumPoints() < 2 || pPrjPts.GetCount() < 2)
	{
		return;
	}

	int iLinePtNum = lineRefLine->GetNumPoints();
	int iPrjPtNum = pPrjPts.GetCount();
	Array<double> arrPrjMeasure;
	arrPrjMeasure.SetSize(iPrjPtNum);
	Geometries::Coordinate *pPrjPtCur = NULL;
	int iPrjCurIndex = -1;
	double dPrjCurMeasure = 0.0;
	for (int i = 0; i < iPrjPtNum; i++)
	{
		pPrjPtCur = pPrjPts.GetAt(i);
		Geometries::Coordinate pCoorPrjCur;
		if (BaseAlgorithm::GetNearestPntToLineset(pPrjPtCur, lineRefLine->GetCoordinates(), pCoorPrjCur, iPrjCurIndex) &&
			lineRefLine->GetMAtPoint(&pCoorPrjCur, iPrjCurIndex, dPrjCurMeasure))
		{
			arrPrjMeasure.SetAt(i, dPrjCurMeasure);
		}
	}

	// 按投影点刻度排序
	for (int i = 0; i < iPrjPtNum - 1; i++)
	{
		double dCurMeasure = arrPrjMeasure.GetAt(i);
		int iMinMeaInd = i;
		for (int j = i + 1; j < iPrjPtNum; j++)
		{
			if (dCurMeasure > arrPrjMeasure.GetAt(j))
			{
				dCurMeasure = arrPrjMeasure.GetAt(j);
				iMinMeaInd = j;
			}
		}
		if (iMinMeaInd != i)
		{
			pPrjPtCur = pPrjPts.GetAt(i);
			pPrjPts.SetAt(i, pPrjPts.GetAt(iMinMeaInd));
			pPrjPts.SetAt(iMinMeaInd, pPrjPtCur);
			dCurMeasure = arrPrjMeasure.GetAt(i);
			arrPrjMeasure.SetAt(i, arrPrjMeasure.GetAt(iMinMeaInd));
			arrPrjMeasure.SetAt(iMinMeaInd, dCurMeasure);
		}
	}
}

// bool IRCreatLaneAlgorithm::InterceptLinesInPolygon(
//	Geometries::Polygon* polygon,
//	Geometries::LineString* pLineOrg,
//	Array<Geometries::LineString*> & pLinesInPlg)
//{
//	if (polygon == NULL || pLineOrg == NULL)
//	{
//		return false;
//	}
//
//	int nCoordCnt = pLineOrg->GetNumPoints();
//	Coordinate* pTmp = NULL;
//	Coordinate* pTmpSec = NULL;
//	Array<Coordinate*>* pArrCoorOrg = pLineOrg->GetCoordinates();
//	Array<Coordinate*>* pArrCoorPg = polygon->GetExteriorRing()->GetCoordinates();
//
//	LineString* pLineDst = new LineString();
//	Array<Coordinate*>* pArrCoorDst = pLineDst->GetCoordinates();
//
//	bool bHavePtInPg = false;
//	for (int i = 0; i < nCoordCnt; i++)
//	{
//		pTmp = pArrCoorOrg->GetAt(i);
//		if (i != 0)
//		{
//			pTmpSec = pArrCoorOrg->GetAt(i - 1);
//		}
//		//点在多边形内的判断
//		bool isInPolygon = GeometryAlgorithm::PtInPolygon(polygon, pTmp);
//
//		if (isInPolygon)
//		{
//			bHavePtInPg = true;
//			//首点在多边形内，加入点，继续遍历，直到线结束 或 线不在面内
//			if (i != 0 && pArrCoorDst->IsEmpty())
//			{
//				//前一点与当前点连线，与多边形算焦点，将焦点加入pInCoord
//				Coordinate ptrTmpCoord;
//				bool bIntersect = BaseAlgorithm3D::IntersectionLineSegmentPlanes(*pTmpSec, *pTmp, pArrCoorPg, ptrTmpCoord);
//				if (bIntersect)
//				{
//					pArrCoorDst->Add(new Coordinate(ptrTmpCoord));
//				}
//			}
//			pArrCoorDst->Add(new Coordinate(*pTmp));
//			continue;
//		}
//		if (!bHavePtInPg && i != 0)
//		{
//			LineString* pTmpLine = new LineString;
//			pTmpLine->GetCoordinates()->Add(new Coordinate(*pTmpSec));
//			pTmpLine->GetCoordinates()->Add(new Coordinate(*pTmp));
//			isInPolygon = GeometryAlgorithm::IntersectsLineSegmentPolygon(pTmpLine, polygon);
//			DELETE_PTR(pTmpLine);
//		}
//		if (isInPolygon)
//		{
//			for (int j = 0; j < pArrCoorPg->GetCount() - 1; j++)
//			{
//				Array<Coordinate*> arrCoorPgLineTmp;
//				arrCoorPgLineTmp.Add(pArrCoorPg->GetAt(j));
//				arrCoorPgLineTmp.Add(pArrCoorPg->GetAt(j + 1));
//				Coordinate ptrTmpCoord;
//				bool bIntersect = BaseAlgorithm3D::IntersectionLineSegmentPlanes(*pTmpSec, *pTmp, &arrCoorPgLineTmp, ptrTmpCoord);
//				if (bIntersect)
//				{
//					if (pArrCoorDst->IsEmpty())
//					{
//						pArrCoorDst->Add(new Coordinate(ptrTmpCoord));
//					}
//					else if (pArrCoorDst->GetAt(0)->Distance(ptrTmpCoord) > Geometries_EP)
//					{
//						pArrCoorDst->Add(new Coordinate(ptrTmpCoord));
//					}
//				}
//				arrCoorPgLineTmp.Clear();
//				if (pArrCoorDst->GetCount() > 1)
//				{
//					if (pTmpSec->DistanceXY(*pArrCoorDst->GetAt(0)) > pTmpSec->DistanceXY(*pArrCoorDst->GetAt(1)))
//					{
//						*pArrCoorDst->GetAt(1) = *pArrCoorDst->GetAt(0);
//						*pArrCoorDst->GetAt(0) = ptrTmpCoord;
//					}
//					break;
//				}
//			}
//			if (pArrCoorDst->GetCount() > 1 && pArrCoorDst->GetAt(0)->Distance(*pTmp) < pArrCoorDst->GetAt(1)->Distance(*pTmp))
//			{
//				Coordinate* pCoorTmp = pArrCoorDst->GetAt(0);
//				pArrCoorDst->SetAt(0, pArrCoorDst->GetAt(1));
//				pArrCoorDst->SetAt(1, pCoorTmp);
//				pCoorTmp = NULL;
//			}
//			//break;
//			pLinesInPlg.Add(pLineDst);
//			pLineDst = new LineString();
//			pArrCoorDst = pLineDst->GetCoordinates();
//			bHavePtInPg = false;
//		}
//		if (bHavePtInPg)
//		{
//			//算当前点与前一点连线，与多边形算焦点，加入pInCoord
//			Coordinate ptrTmpCoord;
//			bool bIntersect = BaseAlgorithm3D::IntersectionLineSegmentPlanes(*pTmpSec, *pTmp, pArrCoorPg, ptrTmpCoord);
//			if (bIntersect)
//			{
//				pArrCoorDst->Add(new Coordinate(ptrTmpCoord));
//			}
//			//break;
//			pLinesInPlg.Add(pLineDst);
//			pLineDst = new LineString();
//			pArrCoorDst = pLineDst->GetCoordinates();
//			bHavePtInPg = false;
//		}
//	}
//
//	if (pLineDst->GetNumPoints() > 1)
//	{
//		pLinesInPlg.Add(pLineDst);
//		pLineDst = NULL;
//	}
//
//	pArrCoorDst = NULL;
//	DELETE_PTR(pLineDst);
//
//	return !(pLinesInPlg.IsEmpty());
// }

bool IRCreatLaneAlgorithm::InterceptLinesInPolygon(
	Geometries::Polygon *polygon,
	Geometries::LineString *pLineOrg,
	Array<Geometries::LineString *> &pLinesInPlg)
{
	if (polygon == NULL || pLineOrg == NULL || pLineOrg->GetNumPoints() < 2)
	{
		return false;
	}
	pLinesInPlg.Clear();

	int iSegmentCount = (int)pLineOrg->GetNumPoints() - 1;

	bool bHavePtInPg = false;
	Geometries::Coordinate *curCoorS;
	Geometries::Coordinate *curCoorE;
	for (int i = 0; i < iSegmentCount; i++)
	{
		curCoorS = pLineOrg->GetCoordinateN(i);
		curCoorE = pLineOrg->GetCoordinateN(i + 1);
		Array<Geometries::LineString *> pInPolgLinesCur;
		GetLineSegmentInOrOutPolygonParts(polygon, curCoorS, curCoorE, true, pInPolgLinesCur);
		if (pInPolgLinesCur.IsEmpty())
		{
			continue;
		}
		if (pLinesInPlg.IsEmpty())
		{
			pLinesInPlg.Add(pInPolgLinesCur);
			pInPolgLinesCur.Clear();
			continue;
		}
		Geometries::LineString *pLineLast = pLinesInPlg.GetAt(pLinesInPlg.GetCount() - 1);
		Geometries::LineString *pLineCurFirst = pInPolgLinesCur.GetAt(0);
		curCoorS = pLineLast->GetCoordinateN(pLineLast->GetNumPoints() - 1);
		curCoorE = pLineCurFirst->GetCoordinateN(0);
		if (curCoorS->Distance(*curCoorE) < Geometries_EP)
		{
			DELETE_PTR(curCoorE);
			pLineCurFirst->GetCoordinates()->Delete(0);
			pLineLast->GetCoordinates()->Add(*pLineCurFirst->GetCoordinates());
			pLineCurFirst->GetCoordinates()->Clear();
			DELETE_PTR(pLineCurFirst);
			pInPolgLinesCur.Delete(0);
		}
		pLinesInPlg.Add(pInPolgLinesCur);
	}

	return !(pLinesInPlg.IsEmpty());
}

void IRCreatLaneAlgorithm::GetLineSegmentInOrOutPolygonParts(Geometries::Polygon *polygon,
															 Geometries::Coordinate *pCoorS, Geometries::Coordinate *pCoorE,
															 Base::Bool bInRing, Array<Geometries::LineString *> &pRstLines)
{
	pRstLines.Clear();
	if (polygon == NULL || pCoorS == NULL || pCoorE == NULL || pCoorS->DistanceXY(*pCoorE) < Geometries_EP)
	{
		return;
	}

	// 交点数组
	Array<Geometries::Coordinate *> coorIntercept;
	// 交点位置，排序用
	Array<Double> arrMeasure;

	// 计算外壳与线段交点
	GetInterceptLineSegmentWithMultLine(polygon->GetExteriorRing(), pCoorS, pCoorE, coorIntercept, arrMeasure);

	// 计算洞与线段交点
	for (int i = 0; i < polygon->GetNumInteriorRing(); i++)
	{
		GetInterceptLineSegmentWithMultLine(polygon->GetInteriorRingN(i), pCoorS, pCoorE, coorIntercept, arrMeasure);
	}

	Geometries::Coordinate coorTmp;
	if (coorIntercept.IsEmpty())
	{
		coorTmp = (*pCoorS + *pCoorE) / 2.0;
		bool bInPlgCur = GeometryAlgorithm::PtInPolygon(polygon, &coorTmp);
		if (bInRing == bInPlgCur)
		{
			Geometries::LineString *pLineTmp = new Geometries::LineString();
			pLineTmp->GetCoordinates()->Add(new Geometries::Coordinate(*pCoorS));
			pLineTmp->GetCoordinates()->Add(new Geometries::Coordinate(*pCoorE));
			pRstLines.Add(pLineTmp);
		}
		return;
	}

	Geometries::LineString *pLineTmp = new Geometries::LineString();
	pLineTmp->GetCoordinates()->Add(coorIntercept);
	coorIntercept.Clear();
	pLineTmp->GetCoordinates()->InsertAt(0, new Geometries::Coordinate(*pCoorS));
	pLineTmp->GetCoordinates()->Add(new Geometries::Coordinate(*pCoorE));
	pLineTmp->Distinct();

	Geometries::Coordinate *curCoorS;
	Geometries::Coordinate *curCoorE;
	Geometries::LineString *pLineCur = NULL;
	int iSegmentCount = (int)pLineTmp->GetNumPoints() - 1;
	for (int i = 0; i < iSegmentCount; i++)
	{
		curCoorS = pLineTmp->GetCoordinateN(i);
		curCoorE = pLineTmp->GetCoordinateN(i + 1);
		coorTmp = (*curCoorS + *curCoorE) / 2.0;
		bool bInPlgCur = GeometryAlgorithm::PtInPolygon(polygon, &coorTmp);
		if (bInRing != bInPlgCur)
		{
			continue;
		}
		if (!pRstLines.IsEmpty() && pRstLines.GetAt(pRstLines.GetCount() - 1)->GetCoordinateN(1)->Distance(*curCoorS) < Geometries_EP)
		{
			*(pRstLines.GetAt(pRstLines.GetCount() - 1)->GetCoordinateN(1)) = *curCoorE;
			continue;
		}
		pLineCur = new Geometries::LineString();
		pLineCur->GetCoordinates()->Add(new Coordinate(*curCoorS));
		pLineCur->GetCoordinates()->Add(new Coordinate(*curCoorE));
		pRstLines.Add(pLineCur);
	}
	DELETE_PTR(pLineTmp);
}

void IRCreatLaneAlgorithm::GetInterceptLineSegmentWithMultLine(Geometries::LineString *multLine,
															   Geometries::Coordinate *pCoorS, Geometries::Coordinate *pCoorE,
															   Array<Geometries::Coordinate *> &arrCoor, Array<Double> &arrMeasure)
{
	if (arrCoor.GetCount() != arrMeasure.GetCount())
	{
		return;
	}
	if (multLine == NULL || multLine->GetNumPoints() < 2 || pCoorS == NULL || pCoorE == NULL || pCoorS->DistanceXY(*pCoorE) < Geometries_EP)
	{
		return;
	}

	for (int i = 0; i < multLine->GetNumPoints() - 1; i++)
	{
		Array<Coordinate *> arrCoorPgLineTmp;
		arrCoorPgLineTmp.Add(multLine->GetCoordinateN(i));
		arrCoorPgLineTmp.Add(multLine->GetCoordinateN(i + 1));
		Coordinate coordTmp;
		bool bIntersect = BaseAlgorithm3D::IntersectionLineSegmentPlanes(*pCoorS, *pCoorE, &arrCoorPgLineTmp, coordTmp);
		if (!bIntersect)
		{
			continue;
		}
		Double dMeasureCur = pCoorS->DistanceXY(coordTmp);
		if (arrCoor.IsEmpty())
		{
			arrMeasure.Add(dMeasureCur);
			arrCoor.Add(new Geometries::Coordinate(coordTmp));
			continue;
		}
		for (int j = 0; j < arrCoor.GetCount(); j++)
		{
			if (arrMeasure.GetAt(j) > dMeasureCur)
			{
				arrMeasure.InsertAt(j, dMeasureCur);
				arrCoor.InsertAt(j, new Geometries::Coordinate(coordTmp));
				break;
			}
			else if (j == arrCoor.GetCount() - 1)
			{
				arrMeasure.Add(dMeasureCur);
				arrCoor.Add(new Geometries::Coordinate(coordTmp));
				break;
			}
		}
	}
}

Geometries::LineString *IRCreatLaneAlgorithm::MultLineTranslationXY(
	Geometries::LineString *pLineOrg, Base::Double dLength)
{
	if (pLineOrg == NULL || pLineOrg->GetNumPoints() < 2)
	{
		return NULL;
	}
	Array<Geometries::Coordinate *> *arrOrgCoors = pLineOrg->GetCoordinates();
	Base::Int32 iCoorNumOrg = arrOrgCoors->GetCount();
	for (int i = 0; i < iCoorNumOrg; i++)
	{
		if (arrOrgCoors->GetAt(i) == NULL)
		{
			return NULL;
		}
	}
	if (Math::Fabs(dLength) < Geometries_EP)
	{
		return (LineString *)pLineOrg->Clone();
	}

	// 计算首末点偏移向量
	Coordinate pCoorVS = *arrOrgCoors->GetAt(1) - *arrOrgCoors->GetAt(0);							  // 起始线段向量
	Coordinate pCoorVE = *arrOrgCoors->GetAt(iCoorNumOrg - 1) - *arrOrgCoors->GetAt(iCoorNumOrg - 2); // 起始线段向量
	pCoorVS.z = 0.0;
	pCoorVE.z = 0.0;
	pCoorVS.Normalize();
	pCoorVE.Normalize();
	pCoorVS = Coordinate(-pCoorVS.y, pCoorVS.x, 0.0);
	pCoorVE = Coordinate(-pCoorVE.y, pCoorVE.x, 0.0);

	Geometries::LineString *pLineDst = new Geometries::LineString();
	Array<Geometries::Coordinate *> *arrDstCoors = pLineDst->GetCoordinates();
	arrDstCoors->Add(new Coordinate(*arrOrgCoors->GetAt(0) + pCoorVS * dLength));
	Double dTotalLthOrg = pLineOrg->GetLength();
	Double dLengthCur = 0.0;

	for (int i = 1; i < iCoorNumOrg - 1; i++)
	{
		Coordinate *pCoorPre = arrOrgCoors->GetAt(i - 1);
		Coordinate *pCoorCur = arrOrgCoors->GetAt(i);
		Coordinate *pCoorNext = arrOrgCoors->GetAt(i + 1);
		dLengthCur += pCoorCur->Distance(*pCoorPre);
		Double dScaleCur = dLengthCur / dTotalLthOrg; // 当前点所在位置处相对整线比例

		Coordinate pCoorVPre = *pCoorCur - *pCoorPre;	// 当前点前一线段向量
		Coordinate pCoorVNext = *pCoorNext - *pCoorCur; // 当前点后一线段向量
		pCoorVPre.z = 0.0;
		pCoorVNext.z = 0.0;
		pCoorVNext.Normalize();
		pCoorVPre.Normalize();
		Coordinate pCoorVTCor;														 // 当前点所需横向平移方向
		int iPosTmp = BaseAlgorithm::PntMatchLine(*pCoorPre, *pCoorCur, *pCoorNext); // 当前点后一点偏移方向
		Double dCosTmp = Math::Cos(pCoorVPre.AngleWith(pCoorVNext) / 2.0);
		if (iPosTmp == 0 || Math::Fabs(dCosTmp) < Geometries_EP) // 当前点与前后点处于同一直线
		{
			pCoorVTCor.x = -pCoorVPre.y;
			pCoorVTCor.y = pCoorVPre.x;
			pCoorVTCor.Normalize();
			pCoorVTCor = pCoorVTCor * dLength;
		}
		else
		{
			pCoorVTCor = pCoorVNext - pCoorVPre;
			pCoorVTCor.Normalize();
			if ((dLength > 0 && iPosTmp == 2) || (dLength < 0 && iPosTmp == 1))
			{
				pCoorVTCor = pCoorVTCor * (-1.0);
			}
			pCoorVTCor = pCoorVTCor * Math::Fabs(dLength) / dCosTmp; // 当前点所需横向平移向量
		}

		arrDstCoors->Add(new Geometries::Coordinate(*pCoorCur + pCoorVTCor));
	}
	arrDstCoors->Add(new Coordinate(*arrOrgCoors->GetAt(iCoorNumOrg - 1) + pCoorVE * dLength));

	if (pLineDst->GetNumPoints() < 2)
	{
		DELETE_PTR(pLineDst);
		return NULL;
	}
	return pLineDst;
}

// Geometries::LineString* IRCreatLaneAlgorithm::MultLineSmothTranslationXY(
//	Geometries::Coordinate* pCoorDstS, Geometries::Coordinate* pCoorDstE,
//	Geometries::LineString* pLineOrg)
//{
//	if (pCoorDstS == NULL || pCoorDstE == NULL || pLineOrg == NULL || pLineOrg->GetNumPoints() < 2)
//	{
//		return NULL;
//	}
//
//	Geometries::LineString* pLineDst = new Geometries::LineString();
//	Array<Geometries::Coordinate*>* arrDstCoors = pLineDst->GetCoordinates();
//	Base::Int32 iCoorNum = pLineOrg->GetNumPoints();
//	Coordinate pCoorVS = *pCoorDstS - pLineOrg->GetStartPoint()->ToCoordinate();
//	Coordinate pCoorVE = *pCoorDstE - pLineOrg->GetEndPoint()->ToCoordinate();
//	Coordinate pCoorVM = pCoorVE - pCoorVS;
//
//	double dVSDis = pCoorVS.GetLength();
//	double dVEDis = pCoorVE.GetLength();
//	double dLineLeng = pLineOrg->GetLength();
//	double dCurMeasure = 0.0;
//	arrDstCoors->Add(new Coordinate(*pCoorDstS));
//	for (int j = 1; j < iCoorNum - 1; j++)
//	{
//		Coordinate* pCoorCur = pLineOrg->GetCoordinates()->GetAt(j);
//		if (pLineOrg->GetMAtPoint(pCoorCur, j, dCurMeasure))
//		{
//			double dScale = dCurMeasure / dLineLeng;
//			Coordinate pCoorDif = pCoorVS + pCoorVM * dScale;
//			pCoorDif.Normalize();
//			pCoorDif = pCoorDif * (dVSDis * (1 - dScale) + dVEDis * dScale);
//			arrDstCoors->Add(new Coordinate(*pCoorCur + pCoorDif));
//		}
//	}
//	arrDstCoors->Add(new Coordinate(*pCoorDstE));
//	if (pLineDst->GetNumPoints() < 2)
//	{
//		DELETE_PTR(pLineDst);
//		return NULL;
//	}
//	return pLineDst;
// }

Geometries::LineString *IRCreatLaneAlgorithm::MultLineSmothTranslationXY(
	const Geometries::Coordinate *pCoorDstS, const Geometries::Coordinate *pCoorDstE,
	Geometries::LineString *pLineOrg)
{
	if (pCoorDstS == NULL || pCoorDstE == NULL || pLineOrg == NULL || pLineOrg->GetNumPoints() < 2)
	{
		return NULL;
	}
	Array<Geometries::Coordinate *> *arrOrgCoors = pLineOrg->GetCoordinates();
	Base::Int32 iCoorNumOrg = arrOrgCoors->GetCount();
	for (int i = 0; i < iCoorNumOrg; i++)
	{
		if (arrOrgCoors->GetAt(i) == NULL)
		{
			return NULL;
		}
	}

	// 参数
	Coordinate pCoorVS = *pCoorDstS - *arrOrgCoors->GetAt(0);											 // 起点位移向量
	Coordinate pCoorVSL = *arrOrgCoors->GetAt(1) - *arrOrgCoors->GetAt(0);								 // 起始线段向量
	int iPosS = BaseAlgorithm::PntMatchLine(*arrOrgCoors->GetAt(0), *arrOrgCoors->GetAt(1), *pCoorDstS); // 起点目标点与起始线段左右位置关系
	Double dZDiffS = pCoorVS.z;																			 // 起点Z方向位移
	pCoorVS.z = 0.0;
	pCoorVSL.z = 0.0;
	pCoorVSL.Normalize();
	Double dAngelS = pCoorVSL.AngleWith(pCoorVS);							   // 起点位移向量与起始线段向量间的水平夹角
	Coordinate pCoorPVS = pCoorVSL * pCoorVS.GetLength() * Math::Cos(dAngelS); // 起点位移向量纵向分量
	Coordinate pCoorTVS = pCoorVS - pCoorPVS;								   // 起点位移向量横向分量
	Double dTDisS = pCoorTVS.GetLength();
	if (iPosS == 2)
	{
		dTDisS *= -1.0;
	}

	Coordinate pCoorVE = *pCoorDstE - *arrOrgCoors->GetAt(iCoorNumOrg - 1);															 // 终点位移向量
	Coordinate pCoorVEL = *arrOrgCoors->GetAt(iCoorNumOrg - 1) - *arrOrgCoors->GetAt(iCoorNumOrg - 2);								 // 末尾线段向量
	int iPosE = BaseAlgorithm::PntMatchLine(*arrOrgCoors->GetAt(iCoorNumOrg - 2), *arrOrgCoors->GetAt(iCoorNumOrg - 1), *pCoorDstE); // 终点目标点与末尾线段左右位置关系
	Double dZDiffE = pCoorVE.z;																										 // 终点Z方向位移
	pCoorVE.z = 0.0;
	pCoorVEL.z = 0.0;
	pCoorVEL.Normalize();
	Double dAngelE = pCoorVEL.AngleWith(pCoorVE);							   // 终点位移向量与末尾线段向量间的水平夹角
	Coordinate pCoorPVE = pCoorVEL * pCoorVE.GetLength() * Math::Cos(dAngelE); // 终点位移向量纵向分量
	Coordinate pCoorTVE = pCoorVE - pCoorPVE;								   // 终点位移向量横向分量
	Double dTDisE = pCoorTVE.GetLength();
	if (iPosE == 2)
	{
		dTDisE *= -1.0;
	}

	pCoorPVS.z = dZDiffS;
	pCoorPVE.z = dZDiffE;
	Coordinate pCoorPVM = pCoorPVE - pCoorPVS;
	Double dTDisM = dTDisE - dTDisS;

	Geometries::LineString *pLineDst = new Geometries::LineString();
	Array<Geometries::Coordinate *> *arrDstCoors = pLineDst->GetCoordinates();
	arrDstCoors->Add(new Geometries::Coordinate(*pCoorDstS));
	Double dTotalLthOrg = pLineOrg->GetLength();
	Double dLengthCur = 0.0;

	for (int i = 1; i < iCoorNumOrg - 1; i++)
	{
		Coordinate *pCoorPre = arrOrgCoors->GetAt(i - 1);
		Coordinate *pCoorCur = arrOrgCoors->GetAt(i);
		Coordinate *pCoorNext = arrOrgCoors->GetAt(i + 1);
		dLengthCur += pCoorCur->Distance(*pCoorPre);
		Double dScaleCur = dLengthCur / dTotalLthOrg;			 // 当前点所在位置处相对整线比例
		Coordinate pCoorPVCur = pCoorPVS + pCoorPVM * dScaleCur; // 当前点所需纵向平移向量
		Double dTDisCur = dTDisS + dTDisM * dScaleCur;			 // 当前点所需横向平移距离

		Coordinate pCoorVPre = *pCoorCur - *pCoorPre;	// 当前点前一线段向量
		Coordinate pCoorVNext = *pCoorNext - *pCoorCur; // 当前点后一线段向量
		pCoorVPre.z = 0.0;
		pCoorVNext.z = 0.0;
		pCoorVNext.Normalize();
		pCoorVPre.Normalize();
		Coordinate pCoorVTCor;														 // 当前点所需横向平移方向
		int iPosTmp = BaseAlgorithm::PntMatchLine(*pCoorPre, *pCoorCur, *pCoorNext); // 当前点后一点偏移方向
		Double dCosTmp = Math::Cos(pCoorVPre.AngleWith(pCoorVNext) / 2.0);
		if (iPosTmp == 0 || Math::Fabs(1 - dCosTmp) < Geometries_EP) // 当前点与前后点处于同一直线
		{
			pCoorVTCor.x = -pCoorVPre.y;
			pCoorVTCor.y = pCoorVPre.x;
			pCoorVTCor.Normalize();
			pCoorVTCor = pCoorVTCor * dTDisCur;
		}
		else
		{
			pCoorVTCor = pCoorVNext - pCoorVPre;
			pCoorVTCor.Normalize();
			if ((dTDisCur > 0 && iPosTmp == 2) || (dTDisCur < 0 && iPosTmp == 1))
			{
				pCoorVTCor = pCoorVTCor * (-1.0);
			}
			pCoorVTCor = pCoorVTCor * Math::Fabs(dTDisCur) / dCosTmp; // 当前点所需横向平移向量
		}

		arrDstCoors->Add(new Geometries::Coordinate(*pCoorCur + pCoorVTCor + pCoorPVCur));
	}

	arrDstCoors->Add(new Geometries::Coordinate(*pCoorDstE));

	if (pLineDst->GetNumPoints() < 2)
	{
		DELETE_PTR(pLineDst);
		return NULL;
	}
	return pLineDst;
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfStrtLineAndPlane(
	const Engine::Geometries::Coordinate pntLine,
	const Engine::Geometries::Coordinate coorLineV,
	const Engine::Geometries::Coordinate pntPlane,
	const Engine::Geometries::Coordinate coorPlaneV,
	Engine::Geometries::Coordinate &pntResult)
{
	Base::Bool bRst = false;
	Base::Double dDotProduct = coorLineV.DotProduct(coorPlaneV);
	if (dDotProduct > 0.0 || dDotProduct < 0.0)
	{
		Base::Double dVt = (pntPlane - pntLine).DotProduct(coorPlaneV);
		pntResult = pntLine + coorLineV * dVt;
		bRst = true;
	}
	return bRst;
}

Base::Double IRCreatLaneAlgorithm::DistancePtToStrtLineXY(
	const Coordinate pt, const Coordinate ptStart,
	const Coordinate ptEnd)
{
	Coordinate ptXY = Coordinate(pt.x, pt.y, 0.0);
	Coordinate ptStartXY = Coordinate(ptStart.x, ptStart.y, 0.0);
	Coordinate ptEndXY = Coordinate(ptEnd.x, ptEnd.y, 0.0);
	Coordinate coorV1 = ptXY - ptStartXY;
	Coordinate coorV2 = ptEndXY - ptStartXY;
	return (coorV1.GetLength() * Math::Sin(coorV1.AngleWith(coorV2)));
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfStrtLines(
	const Engine::Geometries::Coordinate pntLine1S,
	const Engine::Geometries::Coordinate pntLine1E,
	const Engine::Geometries::Coordinate pntLine2S,
	const Engine::Geometries::Coordinate pntLine2E,
	Engine::Geometries::Coordinate &pntResult)
{
	Coordinate coorV1 = pntLine1E - pntLine1S;
	Coordinate coorV2 = pntLine2E - pntLine2S;
	coorV1.z = 0.0;
	coorV2.z = 0.0;
	coorV1.Normalize();
	coorV2.Normalize();
	if ((coorV1 - coorV2).IsZeroVector() || (coorV1 + coorV2).IsZeroVector())
	{
		// 平行
		return false;
	}

	Base::Double d1dx = pntLine1E.x - pntLine1S.x;
	Base::Double d2dx = pntLine2E.x - pntLine2S.x;
	Base::Double d1dy = pntLine1E.y - pntLine1S.y;
	Base::Double d2dy = pntLine2E.y - pntLine2S.y;
	Base::Double d1a = 0.0;
	Base::Double d2a = 0.0;
	Base::Double d1b = 0.0;
	Base::Double d2b = 0.0;
	Base::Double d1c = 0.0;
	Base::Double d2c = 0.0;

	d1a = d1dy;
	d1b = -d1dx;
	d1c = -(d1a * pntLine1S.x + d1b * pntLine1S.y);
	d2a = d2dy;
	d2b = -d2dx;
	d2c = -(d2a * pntLine2S.x + d2b * pntLine2S.y);

	Base::Double dv = d1a * d2b - d2a * d1b; // 不平行的情况下，dv != 0
	pntResult.x = (d1b * d2c - d2b * d1c) / dv;
	pntResult.y = (d2a * d1c - d1a * d2c) / dv;
	pntResult.z = 0.0;
	return true;
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfStrtLineAndLineSeg(
	const Engine::Geometries::Coordinate pntStrtLineS,
	const Engine::Geometries::Coordinate pntStrtLineE,
	const Engine::Geometries::Coordinate pntLineSegS,
	const Engine::Geometries::Coordinate pntLineSegE,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	Coordinate coorRst = Coordinate(0.0, 0.0, 0.0);
	if (InterPtOfStrtLines(pntStrtLineS, pntStrtLineE, pntLineSegS, pntLineSegE, coorRst))
	{
		Coordinate coorSegSNZ = Coordinate(pntLineSegS.x, pntLineSegS.y, 0.0);
		Coordinate coorSegENZ = Coordinate(pntLineSegE.x, pntLineSegE.y, 0.0);
		Coordinate coorSegPtS = coorRst - coorSegSNZ;
		Coordinate coorSegPtE = coorRst - coorSegENZ;
		if (coorSegPtS.IsZeroVector())
		{
			coorRst.z = pntLineSegS.z;
			bRst = true;
		}
		else if (coorSegPtE.IsZeroVector())
		{
			coorRst.z = pntLineSegE.z;
			bRst = true;
		}
		else
		{
			if (coorSegPtS.DotProduct(coorSegPtE) < Geometries_NEP)
			{
				coorRst.z = pntLineSegS.z + (pntLineSegE.z - pntLineSegS.z) * coorSegPtS.GetLength() / (coorSegENZ - coorSegSNZ).GetLength();
				bRst = true;
			}
		}
	}
	if (bRst)
	{
		pntResult = coorRst;
	}
	return bRst;
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfRadLineAndLineSeg(
	const Engine::Geometries::Coordinate pntRadLineS,
	const Engine::Geometries::Coordinate pntRadLineN,
	const Engine::Geometries::Coordinate pntLineSegS,
	const Engine::Geometries::Coordinate pntLineSegE,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	Coordinate coorRst = Coordinate(0.0, 0.0, 0.0);
	if (InterPtOfStrtLines(pntRadLineS, pntRadLineN, pntLineSegS, pntLineSegE, coorRst))
	{
		Coordinate coorRadSNZ = Coordinate(pntRadLineS.x, pntRadLineS.y, 0.0);
		Coordinate coorRadNNZ = Coordinate(pntRadLineN.x, pntRadLineN.y, 0.0);
		Coordinate coorRadPtS = coorRst - coorRadSNZ;
		Coordinate coorRadPtN = coorRadNNZ - coorRadSNZ;
		if (coorRadPtS.DotProduct(coorRadPtN) < Geometries_NEP)
		{
			return bRst;
		}

		Coordinate coorSegSNZ = Coordinate(pntLineSegS.x, pntLineSegS.y, 0.0);
		Coordinate coorSegENZ = Coordinate(pntLineSegE.x, pntLineSegE.y, 0.0);
		Coordinate coorSegPtS = coorRst - coorSegSNZ;
		Coordinate coorSegPtE = coorRst - coorSegENZ;
		if (coorSegPtS.IsZeroVector())
		{
			coorRst.z = pntLineSegS.z;
			bRst = true;
		}
		else if (coorSegPtE.IsZeroVector())
		{
			coorRst.z = pntLineSegE.z;
			bRst = true;
		}
		else
		{
			if (coorSegPtS.DotProduct(coorSegPtE) < Geometries_NEP)
			{
				coorRst.z = pntLineSegS.z + (pntLineSegE.z - pntLineSegS.z) * coorSegPtS.GetLength() / (coorSegENZ - coorSegSNZ).GetLength();
				bRst = true;
			}
		}
	}
	if (bRst)
	{
		pntResult = coorRst;
	}
	return bRst;
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfLineSegs(
	const Engine::Geometries::Coordinate pntLineSegS1,
	const Engine::Geometries::Coordinate pntLineSegE1,
	const Engine::Geometries::Coordinate pntLineSegS2,
	const Engine::Geometries::Coordinate pntLineSegE2,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	Coordinate coorRst = Coordinate(0.0, 0.0, 0.0);
	if (InterPtOfStrtLines(pntLineSegS1, pntLineSegE1, pntLineSegS2, pntLineSegE2, coorRst))
	{
		Coordinate coorLineSegSNZ1 = Coordinate(pntLineSegS1.x, pntLineSegS1.y, 0.0);
		Coordinate coorLineSegNNZ1 = Coordinate(pntLineSegE1.x, pntLineSegE1.y, 0.0);
		Coordinate coorLineSegSNZ2 = Coordinate(pntLineSegS2.x, pntLineSegS2.y, 0.0);
		Coordinate coorLineSegNNZ2 = Coordinate(pntLineSegE2.x, pntLineSegE2.y, 0.0);
		Coordinate coorLineSegPtS1 = coorRst - coorLineSegSNZ1;
		Coordinate coorLineSegPtN1 = coorRst - coorLineSegNNZ1;
		Coordinate coorLineSegPtS2 = coorRst - coorLineSegSNZ2;
		Coordinate coorLineSegPtN2 = coorRst - coorLineSegNNZ2;
		Base::Double dotProd1 = coorLineSegPtS1.DotProduct(coorLineSegPtN1);
		Base::Double dotProd2 = coorLineSegPtS2.DotProduct(coorLineSegPtN2);
		if (dotProd1 < Geometries_NEP && dotProd2 < Geometries_NEP)
		{
			pntResult = coorRst;
			bRst = true;
		}
	}
	return bRst;
}
Base::Bool IRCreatLaneAlgorithm::IsLineSegsIntersect(
	const Engine::Geometries::Coordinate pntLineSegS1,
	const Engine::Geometries::Coordinate pntLineSegE1,
	const Engine::Geometries::Coordinate pntLineSegS2,
	const Engine::Geometries::Coordinate pntLineSegE2,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	Coordinate coorRst = Coordinate(0.0, 0.0, 0.0);
	if (InterPtOfStrtLines(pntLineSegS1, pntLineSegE1, pntLineSegS2, pntLineSegE2, coorRst))
	{
		Coordinate coorLineSegSNZ1 = Coordinate(pntLineSegS1.x, pntLineSegS1.y, 0.0);
		Coordinate coorLineSegNNZ1 = Coordinate(pntLineSegE1.x, pntLineSegE1.y, 0.0);
		Coordinate coorLineSegSNZ2 = Coordinate(pntLineSegS2.x, pntLineSegS2.y, 0.0);
		Coordinate coorLineSegNNZ2 = Coordinate(pntLineSegE2.x, pntLineSegE2.y, 0.0);
		Coordinate coorLineSegPtS1 = coorRst - coorLineSegSNZ1;
		Coordinate coorLineSegPtN1 = coorRst - coorLineSegNNZ1;
		Coordinate coorLineSegPtS2 = coorRst - coorLineSegSNZ2;
		Coordinate coorLineSegPtN2 = coorRst - coorLineSegNNZ2;
		Base::Double dotProd1 = coorLineSegPtS1.DotProduct(coorLineSegPtN1);
		Base::Double dotProd2 = coorLineSegPtS2.DotProduct(coorLineSegPtN2);
		if (dotProd1 < Geometries_EP && dotProd2 < Geometries_EP)
		{
			pntResult = coorRst;
			bRst = true;
		}
	}
	return bRst;
}
Base::Bool IRCreatLaneAlgorithm::InterPtOfStrtLineAndMultLine(
	const Engine::Geometries::Coordinate pntStrtLineS,
	const Engine::Geometries::Coordinate pntStrtLineE,
	const Engine::Geometries::LineString *pMultLine,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	if (pMultLine == NULL || pMultLine->GetNumPoints() < 2)
	{
		return bRst;
	}
	Array<Geometries::Coordinate *> *pArrCoors = pMultLine->GetCoordinates();
	for (Int32 i = 0; i < pArrCoors->GetCount() - 1; i++)
	{
		Geometries::Coordinate *pCoorCurSegS = pArrCoors->GetAt(i);
		Geometries::Coordinate *pCoorCurSegE = pArrCoors->GetAt(i + 1);
		if (InterPtOfStrtLineAndLineSeg(pntStrtLineS, pntStrtLineE, *pCoorCurSegS, *pCoorCurSegE, pntResult))
		{
			bRst = true;
			break;
		}
	}
	return bRst;
}

Base::Bool IRCreatLaneAlgorithm::InterPtOfRadLineAndMultLine(
	const Engine::Geometries::Coordinate pntRadLineS,
	const Engine::Geometries::Coordinate pntRadLineN,
	const Engine::Geometries::LineString *pMultLine,
	Engine::Geometries::Coordinate &pntResult)
{
	Bool bRst = false;
	if (pMultLine == NULL || pMultLine->GetNumPoints() < 2)
	{
		return bRst;
	}
	Array<Geometries::Coordinate *> *pArrCoors = pMultLine->GetCoordinates();
	for (Int32 i = 0; i < pArrCoors->GetCount() - 1; i++)
	{
		Geometries::Coordinate *pCoorCurSegS = pArrCoors->GetAt(i);
		Geometries::Coordinate *pCoorCurSegE = pArrCoors->GetAt(i + 1);
		if (InterPtOfRadLineAndLineSeg(pntRadLineS, pntRadLineN, *pCoorCurSegS, *pCoorCurSegE, pntResult))
		{
			bRst = true;
			break;
		}
	}
	return bRst;
}

Base::Bool IRCreatLaneAlgorithm::GenerateMultlineExternalBuffer(
	Engine::Geometries::LineString *pMultline,
	Engine::Geometries::Polygon *&pPgResult,
	Base::Double distanceL, Base::Double distanceR, Base::Double distanceSE)
{
	if (pMultline == NULL || pMultline->GetNumPoints() < 2)
	{
		return false;
	}
	Geometries::LineString *pLineLinkBuffer = dynamic_cast<Geometries::LineString *>(pMultline->Clone());
	if (distanceSE > Geometries_EP)
	{
		Geometries::Coordinate coorS = *pMultline->GetCoordinateN(0);
		Geometries::Coordinate coorE = *pMultline->GetCoordinateN(1);
		Geometries::Coordinate coorV = coorS - coorE;
		coorV.Normalize();
		pLineLinkBuffer->GetCoordinates()->InsertAt(0, new Coordinate(coorS + coorV * distanceSE));
		coorS = *pMultline->GetCoordinateN(pMultline->GetNumPoints() - 2);
		coorE = *pMultline->GetCoordinateN(pMultline->GetNumPoints() - 1);
		coorV = coorE - coorS;
		coorV.Normalize();
		pLineLinkBuffer->GetCoordinates()->Add(new Coordinate(coorE + coorV * distanceSE));
	}
	// 左边框线
	Geometries::LineString *pLeftLine = GeometryAlgorithm::GenerateOffsetLine(pLineLinkBuffer, distanceL);
	// 右边框线
	Geometries::LineString *pRightLine = GeometryAlgorithm::GenerateOffsetLine(pLineLinkBuffer, distanceR);

	DELETE_PTR(pLineLinkBuffer);

	// 左边框线点串
	Array<Coordinate *> *pLeft = pLeftLine->GetCoordinates();
	// 右边框线点串
	Array<Coordinate *> *pRight = pRightLine->GetCoordinates();

	DELETE_PTR(pPgResult);
	// 左边框线和右边框线组成的闭合环
	Geometries::LinearRing *shell = new Geometries::LinearRing;
	pPgResult = new Geometries::Polygon(shell);
	// 左边框线和右边框线组成的闭合环点串
	Engine::Base::Array<Geometries::Coordinate *> *coordsPg = pPgResult->GetExteriorRing()->GetCoordinates();

	Geometries::Coordinate *pCoorTmp = NULL;
	for (int i = 0; i < pLeft->GetCount(); i++)
	{
		pCoorTmp = new Coordinate(*pLeft->GetAt(i));
		pCoorTmp->z = 0.0;
		coordsPg->Add(pCoorTmp);
		pCoorTmp = NULL;
	}
	for (int i = pRight->GetCount() - 1; i >= 0; i--)
	{
		pCoorTmp = new Coordinate(*pRight->GetAt(i));
		pCoorTmp->z = 0.0;
		coordsPg->Add(pCoorTmp);
		pCoorTmp = NULL;
	}

	pCoorTmp = new Coordinate(*pLeft->GetAt(0));
	pCoorTmp->z = 0.0;
	coordsPg->Add(pCoorTmp);
	pCoorTmp = NULL;

	DELETE_PTR(pLeftLine);
	DELETE_PTR(pRightLine);
	return true;
}

Base::Bool IRCreatLaneAlgorithm::RefreshLineByAnotherThroughPrj(
	Engine::Geometries::LineString *&pLine1,
	Engine::Geometries::LineString *pLine2,
	Base::Double dThreshold)
{
	if (pLine1 == NULL || pLine1->GetNumPoints() < 2 ||
		pLine2 == NULL || pLine2->GetNumPoints() < 2)
	{
		return false;
	}

	Coordinate coorPtS2 = *pLine2->GetCoordinates()->GetAt(0);
	Coordinate coorPtE2 = *pLine2->GetCoordinates()->GetAt(pLine2->GetNumPoints() - 1);
	Coordinate coorPrjS2;
	Coordinate coorPrjE2;
	int iPrjS2Index = -1;
	int iPrjE2Index = -1;
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS2, pLine1->GetCoordinates(), coorPrjS2, iPrjS2Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE2, pLine1->GetCoordinates(), coorPrjE2, iPrjE2Index);
	if (iPrjS2Index > iPrjE2Index || (iPrjS2Index == iPrjE2Index && (coorPrjS2 - coorPrjE2).IsZeroVector()))
	{
		return false;
	}

	Coordinate coorPtS1 = *pLine1->GetCoordinates()->GetAt(0);
	Coordinate coorPtE1 = *pLine1->GetCoordinates()->GetAt(pLine1->GetNumPoints() - 1);
	Coordinate coorPrjS1;
	Coordinate coorPrjE1;
	int iPrjS1Index = -1;
	int iPrjE1Index = -1;
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS1, pLine2->GetCoordinates(), coorPrjS1, iPrjS1Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE1, pLine2->GetCoordinates(), coorPrjE1, iPrjE1Index);
	if (iPrjS1Index > iPrjE1Index)
	{
		return false;
	}

	LineString *pLineTmpS = NULL;
	LineString *pLineTmpE = NULL;
	LineString *pLineTmpM = NULL;
	if (coorPrjS1.DistanceXY(coorPtS2) < Geometries_EP)
	{
		pLineTmpE = (LineString *)pLine2->Clone();
	}
	else if (!GeometryAlgorithm::PtBreakLine(&coorPrjS1, pLine2, pLineTmpS, pLineTmpE))
	{
		return false;
	}
	DELETE_PTR(pLineTmpS);
	if (pLineTmpE == NULL)
	{
		return false;
	}
	pLineTmpS = pLineTmpE;
	pLineTmpE = NULL;
	if (coorPrjE1.DistanceXY(coorPtE2) < Geometries_EP)
	{
		pLineTmpM = (LineString *)pLineTmpS->Clone();
	}
	else if (!GeometryAlgorithm::PtBreakLine(&coorPrjE1, pLineTmpS, pLineTmpM, pLineTmpE))
	{
		return false;
	}
	DELETE_PTR(pLineTmpS);
	DELETE_PTR(pLineTmpE);
	if (pLineTmpM == NULL)
	{
		return false;
	}
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS1, pLineTmpM->GetCoordinates(), coorPrjS1, iPrjS1Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE1, pLineTmpM->GetCoordinates(), coorPrjE1, iPrjE1Index);
	if (iPrjS1Index > iPrjE1Index)
	{
		return false;
	}
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS2, pLine1->GetCoordinates(), coorPrjS2, iPrjS2Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE2, pLine1->GetCoordinates(), coorPrjE2, iPrjE2Index);
	if (iPrjS2Index > iPrjE2Index || (iPrjS2Index == iPrjE2Index && (coorPrjS2 - coorPrjE2).IsZeroVector()))
	{
		return false;
	}
	Coordinate *pTmpF = NULL;
	Coordinate *pTmpT = NULL;
	for (int i = 0; i < pLineTmpM->GetNumPoints(); i++)
	{
		Int32 iPrjIndex;
		Coordinate coorPrjTmp;
		Coordinate *pTmp = pLineTmpM->GetCoordinateN(i);
		BaseAlgorithm::GetNearestPntToLineset(pTmp, pLine2->GetCoordinates(), coorPrjTmp, iPrjIndex);
		pTmpF = pLine2->GetCoordinates()->GetAt(iPrjIndex);
		pTmpT = pLine2->GetCoordinates()->GetAt(iPrjIndex + 1);
		if (pTmpF->DistanceXY(*pTmp) < Geometries_EP)
		{
			pTmp->z = pTmpF->z;
			continue;
		}
		if (pTmpT->DistanceXY(*pTmp) < Geometries_EP)
		{
			pTmp->z = pTmpT->z;
			continue;
		}
		pTmp->z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(*pTmp) / pTmpF->DistanceXY(*pTmpT);
	}

	// 为投影点计算Z值
	pTmpF = pLineTmpM->GetCoordinates()->GetAt(iPrjS1Index);
	pTmpT = pLineTmpM->GetCoordinates()->GetAt(iPrjS1Index + 1);
	coorPrjS1.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjS1) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLineTmpM->GetCoordinates()->GetAt(iPrjE1Index);
	pTmpT = pLineTmpM->GetCoordinates()->GetAt(iPrjE1Index + 1);
	coorPrjE1.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjE1) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLine1->GetCoordinates()->GetAt(iPrjS2Index);
	pTmpT = pLine1->GetCoordinates()->GetAt(iPrjS2Index + 1);
	coorPrjS2.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjS2) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLine1->GetCoordinates()->GetAt(iPrjE2Index);
	pTmpT = pLine1->GetCoordinates()->GetAt(iPrjE2Index + 1);
	coorPrjE2.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjE2) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = NULL;
	pTmpT = NULL;

	// 处理弯曲较大线错误，应对bug3224	[3/21/2017 guohaiqiang]
	/*if (coorPtS1.DistanceXY(coorPrjS1) > dMaxWidthThreshold)
	{
		coorPrjS1 = coorPtS2;
		iPrjS1Index = 0;
	}
	if (coorPtS2.DistanceXY(coorPrjS2) > dMaxWidthThreshold)
	{
		coorPrjS2 = coorPtS1;
		iPrjS2Index = 0;
	}
	if (coorPtE1.DistanceXY(coorPrjE1) > dMaxWidthThreshold)
	{
		coorPrjE1 = coorPtE2;
		iPrjE1Index = pLine2->GetNumPoints() - 2;
	}
	if (coorPtS2.DistanceXY(coorPrjS2) > dMaxWidthThreshold)
	{
		coorPrjS2 = coorPtS1;
		iPrjS2Index = pLine1->GetNumPoints() - 2;
	}*/

	Geometries::LineString *pLineDst = new Geometries::LineString();
	Array<Geometries::Coordinate *> *pCoorDst = pLineDst->GetCoordinates();

	if (iPrjS1Index == 0 &&
		(coorPrjS1 - coorPtS2).GetLength() < dThreshold &&
		(coorPrjS2 - coorPtS1).GetLength() >= dThreshold)
	{
		for (int i = 0; i <= iPrjS2Index; i++)
		{
			pCoorDst->Add(new Coordinate(*pLine1->GetCoordinates()->GetAt(i)));
		}
		pCoorDst->Add(new Coordinate(coorPrjS2));
		pCoorDst->Add(new Coordinate(coorPrjS1));
	}
	else
	{
		pCoorDst->Add(new Coordinate(coorPrjS1));
	}

	for (int i = iPrjS1Index + 1; i <= iPrjE1Index; i++)
	{
		pCoorDst->Add(new Coordinate(*pLineTmpM->GetCoordinates()->GetAt(i)));
	}

	if (iPrjE1Index == pLineTmpM->GetNumPoints() - 2 &&
		(coorPrjE1 - coorPtE2).GetLength() < dThreshold &&
		(coorPrjE2 - coorPtE1).GetLength() >= dThreshold)
	{
		pCoorDst->Add(new Coordinate(coorPtE2));
		pCoorDst->Add(new Coordinate(coorPrjE2));
		for (int i = iPrjE2Index + 1; i < pLine1->GetNumPoints(); i++)
		{
			pCoorDst->Add(new Coordinate(*pLine1->GetCoordinates()->GetAt(i)));
		}
	}
	else
	{
		pCoorDst->Add(new Coordinate(coorPrjE1));
	}

	pLineDst->Distinct();
	pCoorDst = pLineDst->GetCoordinates();
	int iCoorNumDst = pCoorDst->GetCount();
	/*if (iCoorNumDst < 2)
	{
		DELETE_PTR(pLineDst);
		return false;
	}

	for (int i = iCoorNumDst - 1; i > 0; i--)
	{
		pTmpF = pCoorDst->GetAt(i);
		pTmpT = pCoorDst->GetAt(i - 1);
		if (pTmpF->Distance(*pTmpT) < Geometries_EP)
		{
			DELETE_PTR(pTmpF);
			pCoorDst->Delete(i);
		}
		pTmpF = NULL;
		pTmpT = NULL;
	}

	iCoorNumDst = pCoorDst->GetCount();*/
	if (iCoorNumDst < 2)
	{
		DELETE_PTR(pLineDst);
		return false;
	}

	DELETE_PTR(pLineTmpM);
	DELETE_PTR(pLine1);
	pLine1 = pLineDst;
	pLineDst = NULL;
	return true;
}

Int32 IRCreatLaneAlgorithm::RefreshLineByAnotherInOverlap(
	Engine::Geometries::LineString *pLine1,
	Engine::Geometries::LineString *pLine2,
	Engine::Geometries::LineString *&pLineRst,
	Int32 iRefreshType, Double dThreshold, Double dSmoothScale)
{
	if (pLine1 == NULL || pLine1->GetNumPoints() < 2 ||
		pLine2 == NULL || pLine2->GetNumPoints() < 2 ||
		iRefreshType < 0 || iRefreshType > 2)
	{
		return 3;
	}

	Coordinate coorPtS1 = *pLine1->GetCoordinates()->GetAt(0);
	Coordinate coorPtE1 = *pLine1->GetCoordinates()->GetAt(pLine1->GetNumPoints() - 1);
	Coordinate coorPrjS1;
	Coordinate coorPrjE1;
	int iPrjS1Index = -1;
	int iPrjE1Index = -1;
	GetNearestPtOnMultLine(&coorPtS1, pLine2, coorPrjS1, iPrjS1Index);
	GetNearestPtOnMultLine(&coorPtE1, pLine2, coorPrjE1, iPrjE1Index);
	Coordinate coorPtS2 = *pLine2->GetCoordinates()->GetAt(0);
	Coordinate coorPtE2 = *pLine2->GetCoordinates()->GetAt(pLine2->GetNumPoints() - 1);
	Coordinate coorPrjS2;
	Coordinate coorPrjE2;
	int iPrjS2Index = -1;
	int iPrjE2Index = -1;
	GetNearestPtOnMultLine(&coorPtS2, pLine1, coorPrjS2, iPrjS2Index);
	GetNearestPtOnMultLine(&coorPtE2, pLine1, coorPrjE2, iPrjE2Index);

	Double dS1Dis = coorPtS1.DistanceXY(coorPrjS1);
	Double dE1Dis = coorPtE1.DistanceXY(coorPrjE1);
	Double dS2Dis = coorPtS2.DistanceXY(coorPrjS2);
	Double dE2Dis = coorPtE2.DistanceXY(coorPrjE2);
	int iType = 0; // 最近端点判断 0：s1;1：e1;2：s2;3：e2;
	Double dMinDis = dS1Dis;

	if (dMinDis > dE1Dis)
	{
		iType = 1;
		dMinDis = dE1Dis;
	}
	if (dMinDis > dS2Dis)
	{
		iType = 2;
		dMinDis = dS2Dis;
	}
	if (dMinDis > dE2Dis)
	{
		iType = 3;
		dMinDis = dE2Dis;
	}

	if (iRefreshType == 1)
	{
		iType = 2;
	}
	else if (iRefreshType == 2)
	{
		iType = 3;
	}

	bool bIgnorS = (iPrjS2Index == 0 && coorPrjS2.DistanceXY(*pLine1->GetCoordinateN(0)) < dThreshold) || iRefreshType == 1;
	bool bIgnorE = ((iPrjE2Index == pLine1->GetNumPoints() - 2) && (coorPrjE2.DistanceXY(*pLine1->GetCoordinateN(pLine1->GetNumPoints() - 1)) < dThreshold)) || iRefreshType == 2;

	bool bPrjS1IsStart = (iPrjS1Index == 0 && coorPrjS1.DistanceXY(*pLine2->GetCoordinateN(0)) < Geometries_EP);
	bool bPrjE1IsEnd = ((iPrjE1Index == pLine2->GetNumPoints() - 2) && (coorPrjE1.DistanceXY(*pLine2->GetCoordinateN(pLine2->GetNumPoints() - 1)) < Geometries_EP));

	Array<Int32> iVerticalCornerIndex;
	Coordinate coorPrjTmp;
	int iPrjIndexTmp = -1;
	pLineRst = new Geometries::LineString();
	int iRefreshRst = 0;
	switch (iType)
	{
	case 0:
	{
		// 非重叠判断
		if (coorPrjS1.DistanceXY(coorPtE2) < Geometries_EP)
		{
			iRefreshRst = 2;
			break;
		}
		// 方向判断
		Coordinate coorV1 = *pLine1->GetCoordinateN(1) - coorPtS1;
		Coordinate coorV2 = *pLine2->GetCoordinateN(iPrjS1Index + 1) - *pLine2->GetCoordinateN(iPrjS1Index);
		coorV1.z = 0.0;
		coorV2.z = 0.0;
		if (GeometryAlgorithm::RadianToDegree(coorV1.AngleWith(coorV2)) > 90)
		{
			iRefreshRst = 1;
			break;
		}

		pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS1));
		bool bIsEnd = true; // pLine2结束
		int i = iPrjS1Index + 1;
		for (; i < pLine2->GetNumPoints(); i++)
		{
			if (!bPrjE1IsEnd && i > iPrjE1Index)
			{
				bIsEnd = false;
				break;
			}
			BaseAlgorithm::GetNearestPntToLineset(pLine2->GetCoordinateN(i), pLine1->GetCoordinates(), coorPrjTmp, iPrjIndexTmp);
			if (coorPrjTmp.DistanceXY(coorPtE1) < Geometries_EP)
			{
				bIsEnd = false;
				break;
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine2->GetCoordinateN(i)));
		}
		if (bIsEnd)
		{
			if (!bIgnorE)
			{
				if (pLineRst->GetNumPoints() > 1)
				{
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
				}
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE2));
				for (i = iPrjE2Index + 1; i < pLine1->GetNumPoints(); i++)
				{
					pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
				}
			}
		}
		else
		{
			if (i - 1 <= iPrjE1Index)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE1));
			}
		}
	}
	break;
	case 1:
	{
		// 非重叠判断
		if (coorPrjE1.DistanceXY(coorPtS2) < Geometries_EP)
		{
			iRefreshRst = 2;
			break;
		}
		// 方向判断
		Coordinate coorV1 = coorPtE1 - *pLine1->GetCoordinateN(pLine1->GetNumPoints() - 2);
		Coordinate coorV2 = *pLine2->GetCoordinateN(iPrjE1Index + 1) - *pLine2->GetCoordinateN(iPrjE1Index);
		coorV1.z = 0.0;
		coorV2.z = 0.0;
		if (GeometryAlgorithm::RadianToDegree(coorV1.AngleWith(coorV2)) > 90)
		{
			iRefreshRst = 1;
			break;
		}

		pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE1));
		bool bIsEnd = true; // pLine2结束
		int i = iPrjE1Index;
		for (; i >= 0; i--)
		{
			if (!bPrjS1IsStart && i <= iPrjS1Index)
			{
				bIsEnd = false;
				break;
			}
			BaseAlgorithm::GetNearestPntToLineset(pLine2->GetCoordinateN(i), pLine1->GetCoordinates(), coorPrjTmp, iPrjIndexTmp);
			if (coorPrjTmp.DistanceXY(coorPtS1) < Geometries_EP)
			{
				bIsEnd = false;
				break;
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine2->GetCoordinateN(i)));
		}
		if (bIsEnd)
		{
			if (!bIgnorS)
			{
				if (pLineRst->GetNumPoints() > 1)
				{
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
				}
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS2));
				for (i = iPrjS2Index; i >= 0; i--)
				{
					pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
				}
			}
		}
		else
		{
			if (i + 1 > iPrjS1Index)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS1));
			}
		}
		if (pLineRst != NULL && pLineRst->GetNumPoints() > 1)
		{
			Geometries::LineString *pLineRevTmp = (Geometries::LineString *)pLineRst->Reverse();
			DELETE_PTR(pLineRst);
			pLineRst = pLineRevTmp;
			pLineRevTmp = NULL;
			for (int i = 0; i < iVerticalCornerIndex.GetCount(); i++)
			{
				iVerticalCornerIndex.SetAt(i, pLineRst->GetNumPoints() - 1 - iVerticalCornerIndex.GetAt(i));
			}
			iVerticalCornerIndex.Reverse();
		}
	}
	break;
	case 2:
	{
		// 非重叠判断
		if (coorPrjS2.DistanceXY(coorPtE1) < Geometries_EP)
		{
			iRefreshRst = 2;
			break;
		}
		// 方向判断
		Coordinate coorV1 = *pLine2->GetCoordinateN(1) - coorPtS2;
		Coordinate coorV2 = *pLine1->GetCoordinateN(iPrjS2Index + 1) - *pLine1->GetCoordinateN(iPrjS2Index);
		coorV1.z = 0.0;
		coorV2.z = 0.0;
		if (GeometryAlgorithm::RadianToDegree(coorV1.AngleWith(coorV2)) > 90)
		{
			iRefreshRst = 1;
			break;
		}

		if (!bIgnorS)
		{
			for (int i = 0; i <= iPrjS2Index; i++)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS2));

			if (pLineRst->GetNumPoints() > 1)
			{
				iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
				iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
			}
		}

		bool bIsEnd = true; // pLine2结束
		int i = 0;
		for (; i < pLine2->GetNumPoints(); i++)
		{
			if (!bPrjE1IsEnd && i > iPrjE1Index)
			{
				bIsEnd = false;
				break;
			}
			BaseAlgorithm::GetNearestPntToLineset(pLine2->GetCoordinateN(i), pLine1->GetCoordinates(), coorPrjTmp, iPrjIndexTmp);
			if (coorPrjTmp.DistanceXY(coorPtE1) < Geometries_EP)
			{
				bIsEnd = false;
				break;
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine2->GetCoordinateN(i)));
		}
		if (bIsEnd)
		{
			if (!bIgnorE)
			{
				if (pLineRst->GetNumPoints() > 1)
				{
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
				}
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE2));
				for (i = iPrjE2Index + 1; i < pLine1->GetNumPoints(); i++)
				{
					pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
				}
			}
		}
		else
		{
			if (i - 1 <= iPrjE1Index)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE1));
			}
		}
	}
	break;
	case 3:
	{
		// 非重叠判断
		if (coorPrjE2.DistanceXY(coorPtS1) < Geometries_EP)
		{
			iRefreshRst = 2;
			break;
		}
		// 方向判断
		Coordinate coorV1 = coorPtE2 - *pLine2->GetCoordinateN(pLine2->GetNumPoints() - 2);
		Coordinate coorV2 = *pLine1->GetCoordinateN(iPrjE2Index + 1) - *pLine1->GetCoordinateN(iPrjE2Index);
		coorV1.z = 0.0;
		coorV2.z = 0.0;
		if (GeometryAlgorithm::RadianToDegree(coorV1.AngleWith(coorV2)) > 90)
		{
			iRefreshRst = 1;
			break;
		}

		if (!bIgnorE)
		{
			for (int i = pLine1->GetNumPoints() - 1; i > iPrjE2Index; i--)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjE2));
			if (pLineRst->GetNumPoints() > 1)
			{
				iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
				iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
			}
		}

		bool bIsEnd = true; // pLine2结束
		int i = pLine2->GetNumPoints() - 1;
		for (; i >= 0; i--)
		{
			if (!bPrjS1IsStart && i <= iPrjS1Index)
			{
				bIsEnd = false;
				break;
			}
			BaseAlgorithm::GetNearestPntToLineset(pLine2->GetCoordinateN(i), pLine1->GetCoordinates(), coorPrjTmp, iPrjIndexTmp);
			if (coorPrjTmp.DistanceXY(coorPtS1) < Geometries_EP)
			{
				bIsEnd = false;
				break;
			}
			pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine2->GetCoordinateN(i)));
		}
		if (bIsEnd)
		{
			if (!bIgnorS)
			{
				if (pLineRst->GetNumPoints() > 1)
				{
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints() - 1);
					iVerticalCornerIndex.Add(pLineRst->GetNumPoints());
				}
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS2));
				for (i = iPrjS2Index; i >= 0; i--)
				{
					pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(*pLine1->GetCoordinateN(i)));
				}
			}
		}
		else
		{
			if (i + 1 > iPrjS1Index)
			{
				pLineRst->GetCoordinates()->Add(new Geometries::Coordinate(coorPrjS1));
			}
		}
		if (pLineRst != NULL && pLineRst->GetNumPoints() > 1)
		{
			Geometries::LineString *pLineRevTmp = (Geometries::LineString *)pLineRst->Reverse();
			DELETE_PTR(pLineRst);
			pLineRst = pLineRevTmp;
			pLineRevTmp = NULL;
			for (int i = 0; i < iVerticalCornerIndex.GetCount(); i++)
			{
				iVerticalCornerIndex.SetAt(i, pLineRst->GetNumPoints() - 1 - iVerticalCornerIndex.GetAt(i));
			}
			iVerticalCornerIndex.Reverse();
		}
	}
	break;
	default:
		break;
	}

	if (pLineRst->GetNumPoints() < 2)
	{
		DELETE_PTR(pLineRst);
		if (iRefreshRst == 0)
		{
			iRefreshRst = 4;
		}
	}

	if (iRefreshRst != 0)
	{
		DELETE_PTR(pLineRst);
	}

	// 平滑投影点处直角拐角
	if (iRefreshRst == 0 && iVerticalCornerIndex.GetCount() > 0 && iVerticalCornerIndex.GetCount() % 2 == 0)
	{
		Double dLengthTotal;						// 平面投影长度
		Double dMidMeasure;							// 平滑线段中点刻度
		Int32 iSmoothIndexS;						// 平滑线起点所在线段起点编号
		Int32 iSmoothIndexE;						// 平滑线终点所在线段终点编号
		Geometries::Coordinate *coorSmoothS = NULL; // 平滑部分起点
		Geometries::Coordinate *coorSmoothE = NULL; // 平滑部分终点
		Geometries::LineString *pLineSmoothS = new Geometries::LineString();
		Geometries::LineString *pLineSmoothE = new Geometries::LineString();
		Geometries::LineString *pLineSmoothDst = NULL;
		Int32 iCoorNumber = pLineRst->GetNumPoints();
		pLineRst->GetMAtPoint(pLineRst->GetCoordinateN(iCoorNumber - 1), iCoorNumber - 2, dLengthTotal);
		for (int i = 0; i < iVerticalCornerIndex.GetCount() - 1; i += 2)
		{
			iCoorNumber = pLineRst->GetNumPoints();
			Int32 iIndexCornerS = iVerticalCornerIndex.GetAt(i) + i;
			Int32 iIndexCornerE = iVerticalCornerIndex.GetAt(i + 1) + i;
			if (pLineRst->GetCoordinateN(iIndexCornerS)->DistanceXY(*pLineRst->GetCoordinateN(iIndexCornerE)) < Geometries_EP)
			{
				coorSmoothS = new Geometries::Coordinate(*pLineRst->GetCoordinateN(iIndexCornerS));
				coorSmoothE = new Geometries::Coordinate(*pLineRst->GetCoordinateN(iIndexCornerE));
				pLineRst->GetCoordinates()->InsertAt(iIndexCornerE, coorSmoothE);
				pLineRst->GetCoordinates()->InsertAt(iIndexCornerE, coorSmoothS);
				coorSmoothS = NULL;
				coorSmoothE = NULL;
				continue;
			}
			coorSmoothS = new Geometries::Coordinate();
			coorSmoothE = new Geometries::Coordinate();
			Geometries::Coordinate coorMid = (*pLineRst->GetCoordinateN(iIndexCornerS) + *pLineRst->GetCoordinateN(iIndexCornerE)) / 2;
			pLineRst->GetMAtPoint(&coorMid, iIndexCornerS, dMidMeasure);
			Double dSmoothLength = pLineRst->GetCoordinateN(iIndexCornerS)->DistanceXY(*pLineRst->GetCoordinateN(iIndexCornerE)) * dSmoothScale;
			if (dMidMeasure - dSmoothLength < Geometries_EP)
			{
				iSmoothIndexS = 0;
				*coorSmoothS = *pLineRst->GetCoordinateN(iSmoothIndexS);
			}
			else
			{
				pLineRst->GetPointAtM(dMidMeasure - dSmoothLength, *coorSmoothS, iSmoothIndexS);
			}
			if (dMidMeasure + dSmoothLength > dLengthTotal - Geometries_EP)
			{
				iSmoothIndexE = iCoorNumber - 1;
				*coorSmoothE = *pLineRst->GetCoordinateN(iSmoothIndexE);
			}
			else
			{
				pLineRst->GetPointAtM(dMidMeasure + dSmoothLength, *coorSmoothE, iSmoothIndexE);
				iSmoothIndexE++;
			}

			if (i + 2 < iVerticalCornerIndex.GetCount() - 1 && iSmoothIndexE > iVerticalCornerIndex.GetAt(i + 2) + i)
			{
				iSmoothIndexE = iVerticalCornerIndex.GetAt(i + 2) + i;
				*coorSmoothE = *pLineRst->GetCoordinateN(iSmoothIndexE);
			}

			pLineSmoothS->GetCoordinates()->Add(coorSmoothS);
			for (int j = iSmoothIndexS + 1; j <= iIndexCornerS; j++)
			{
				pLineSmoothS->GetCoordinates()->Add(pLineRst->GetCoordinateN(j));
			}
			pLineSmoothDst = MultLineSmothTranslationXY(coorSmoothS, &coorMid, pLineSmoothS);
			if (pLineSmoothDst != NULL && pLineSmoothDst->GetNumPoints() == pLineSmoothS->GetNumPoints())
			{
				for (int j = 0; j < pLineSmoothDst->GetNumPoints(); j++)
				{
					*pLineSmoothS->GetCoordinateN(j) = *pLineSmoothDst->GetCoordinateN(j);
				}
			}
			DELETE_PTR(pLineSmoothDst);

			for (int j = iIndexCornerE; j < iSmoothIndexE; j++)
			{
				pLineSmoothE->GetCoordinates()->Add(pLineRst->GetCoordinateN(j));
			}
			pLineSmoothE->GetCoordinates()->Add(coorSmoothE);
			pLineSmoothDst = MultLineSmothTranslationXY(&coorMid, coorSmoothE, pLineSmoothE);
			if (pLineSmoothDst != NULL && pLineSmoothDst->GetNumPoints() == pLineSmoothE->GetNumPoints())
			{
				for (int j = 0; j < pLineSmoothDst->GetNumPoints(); j++)
				{
					*pLineSmoothE->GetCoordinateN(j) = *pLineSmoothDst->GetCoordinateN(j);
				}
			}
			DELETE_PTR(pLineSmoothDst);
			pLineSmoothS->GetCoordinates()->Clear();
			pLineSmoothE->GetCoordinates()->Clear();
			pLineRst->GetCoordinates()->InsertAt(iSmoothIndexE, coorSmoothE);
			pLineRst->GetCoordinates()->InsertAt(iSmoothIndexS + 1, coorSmoothS);
			coorSmoothS = NULL;
			coorSmoothE = NULL;
		}
	}

	if (pLineRst != NULL)
	{
		pLineRst->DistinctXY();
	}

	return iRefreshRst;
}

Base::Bool IRCreatLaneAlgorithm::MergeTowLinehroughPrj(
	Engine::Geometries::LineString *&pLine1,
	Engine::Geometries::LineString *pLine2)
{
	if (pLine1 == NULL || pLine1->GetNumPoints() < 2 ||
		pLine2 == NULL || pLine2->GetNumPoints() < 2)
	{
		return false;
	}

	Coordinate coorPtS2 = *pLine2->GetCoordinates()->GetAt(0);
	Coordinate coorPtE2 = *pLine2->GetCoordinates()->GetAt(pLine2->GetNumPoints() - 1);
	Coordinate coorPrjS2;
	Coordinate coorPrjE2;
	int iPrjS2Index = -1;
	int iPrjE2Index = -1;
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS2, pLine1->GetCoordinates(), coorPrjS2, iPrjS2Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE2, pLine1->GetCoordinates(), coorPrjE2, iPrjE2Index);
	if (iPrjS2Index > iPrjE2Index || (iPrjS2Index == iPrjE2Index && (coorPrjS2 - coorPrjE2).IsZeroVector()))
	{
		return false;
	}

	Coordinate coorPtS1 = *pLine1->GetCoordinates()->GetAt(0);
	Coordinate coorPtE1 = *pLine1->GetCoordinates()->GetAt(pLine1->GetNumPoints() - 1);
	Coordinate coorPrjS1;
	Coordinate coorPrjE1;
	int iPrjS1Index = -1;
	int iPrjE1Index = -1;
	BaseAlgorithm::GetNearestPntToLineset(&coorPtS1, pLine2->GetCoordinates(), coorPrjS1, iPrjS1Index);
	BaseAlgorithm::GetNearestPntToLineset(&coorPtE1, pLine2->GetCoordinates(), coorPrjE1, iPrjE1Index);
	if (iPrjS1Index > iPrjE1Index)
	{
		return false;
	}

	// 为投影点计算Z值
	Coordinate *pTmpF = pLine2->GetCoordinates()->GetAt(iPrjS1Index);
	Coordinate *pTmpT = pLine2->GetCoordinates()->GetAt(iPrjS1Index + 1);
	coorPrjS1.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjS1) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLine2->GetCoordinates()->GetAt(iPrjE1Index);
	pTmpT = pLine2->GetCoordinates()->GetAt(iPrjE1Index + 1);
	coorPrjE1.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjE1) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLine1->GetCoordinates()->GetAt(iPrjS2Index);
	pTmpT = pLine1->GetCoordinates()->GetAt(iPrjS2Index + 1);
	coorPrjS2.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjS2) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = pLine1->GetCoordinates()->GetAt(iPrjE2Index);
	pTmpT = pLine1->GetCoordinates()->GetAt(iPrjE2Index + 1);
	coorPrjE2.z = pTmpF->z + (pTmpT->z - pTmpF->z) * pTmpF->DistanceXY(coorPrjE2) / pTmpF->DistanceXY(*pTmpT);
	pTmpF = NULL;
	pTmpT = NULL;

	// 处理弯曲较大线错误，应对bug3224	[3/21/2017 guohaiqiang]
	if (coorPtS1.DistanceXY(coorPrjS1) > 5.0)
	{
		coorPrjS1 = coorPtS2;
		iPrjS1Index = 0;
	}
	if (coorPtS2.DistanceXY(coorPrjS2) > 5.0)
	{
		coorPrjS2 = coorPtS1;
		iPrjS2Index = 0;
	}
	if (coorPtE1.DistanceXY(coorPrjE1) > 5.0)
	{
		coorPrjE1 = coorPtE2;
		iPrjE1Index = pLine2->GetNumPoints() - 2;
	}
	if (coorPtS2.DistanceXY(coorPrjS2) > 5.0)
	{
		coorPrjS2 = coorPtS1;
		iPrjS2Index = pLine1->GetNumPoints() - 2;
	}

	Array<Geometries::Coordinate *> *pCoorDst = pLine1->GetCoordinates();

	if (iPrjS2Index == 0 && (coorPrjS2 - coorPtS1).IsZeroVector() && !(coorPrjS1 - coorPtS2).IsZeroVector())
	{
		// pCoorDst->InsertAt(0, new Coordinate(coorPrjS1));
		for (int i = iPrjS1Index; i >= 0; i--)
		{
			pCoorDst->InsertAt(0, new Coordinate(*pLine2->GetCoordinates()->GetAt(i)));
		}
	}

	if (iPrjE2Index == pLine1->GetNumPoints() - 2 && (coorPrjE2 - coorPtE1).IsZeroVector() && !(coorPrjE1 - coorPtE2).IsZeroVector())
	{
		// pCoorDst->Add(new Coordinate(coorPrjE1));
		for (int i = iPrjE1Index + 1; i < pLine2->GetNumPoints(); i++)
		{
			pCoorDst->Add(new Coordinate(*pLine2->GetCoordinates()->GetAt(i)));
		}
	}

	pLine1->Distinct();

	return true;
}

Geometries::Coordinate *IRCreatLaneAlgorithm::GetPtInLineByMeasure(
	Engine::Geometries::LineString *pLine,
	Base::Double dMeasure)
{
	if (pLine == NULL || pLine->GetNumPoints() < 2 || dMeasure < 0.0)
	{
		return NULL;
	}

	Base::Double dLineLength = pLine->GetLength();
	if (dLineLength <= 0.0 || dLineLength - dMeasure < Geometries_NEP)
	{
		return NULL;
	}

	Geometries::Coordinate *pCoorRst = NULL;
	Base::Int32 iCoorNum = pLine->GetNumPoints();
	if (dMeasure < Geometries_EP)
	{
		pCoorRst = new Geometries::Coordinate(*(pLine->GetCoordinates()->GetAt(0)));
	}
	else if (Math::Fabs(dLineLength - dMeasure) <= Geometries_EP)
	{
		pCoorRst = new Geometries::Coordinate(*(pLine->GetCoordinates()->GetAt(iCoorNum - 1)));
	}
	else
	{
		Base::Double dCurMeasure = 0.0;
		Array<Geometries::Coordinate *> *pArrCoors = pLine->GetCoordinates();
		int i;
		Geometries::Coordinate *pCoorS = NULL;
		Geometries::Coordinate *pCoorE = NULL;
		Base::Double dCurSegLength = 0.0;
		for (i = 0; i < iCoorNum - 1; i++)
		{
			pCoorS = pArrCoors->GetAt(i);
			pCoorE = pArrCoors->GetAt(i + 1);
			dCurSegLength = pCoorS->DistanceXY(*pCoorE);
			if (dMeasure - dCurMeasure < dCurSegLength)
			{
				break;
			}
			dCurMeasure += dCurSegLength;
		}
		Geometries::Coordinate pCurSeV = *pCoorE - *pCoorS;
		// pCurSeV.Normalize();
		pCoorRst = new Geometries::Coordinate(*pCoorS + pCurSeV * ((dMeasure - dCurMeasure) / dCurSegLength));
	}
	return pCoorRst;
}

Geometries::LineString *IRCreatLaneAlgorithm::GetCenterLineOfLRLine(
	Engine::Geometries::LineString *pLineStringL,
	Engine::Geometries::LineString *pLineStringR)
{
	if ((pLineStringL == NULL) || (pLineStringL->GetNumPoints() < 2) ||
		(pLineStringR == NULL) || (pLineStringR->GetNumPoints() < 2))
	{
		return NULL;
	}

	Array<Engine::Geometries::Coordinate *> *pCoorsL =
		pLineStringL->GetCoordinates();
	Array<Engine::Geometries::Coordinate *> *pCoorsR =
		pLineStringR->GetCoordinates();
	Int32 ncoordsL = pCoorsL->GetCount();
	Int32 ncoordsR = pCoorsR->GetCount();
	Double dLineLengthL = 0.0;
	Double dLineLengthR = 0.0;
	pLineStringL->GetMAtPoint(pLineStringL->GetCoordinateN(ncoordsL - 1), ncoordsL - 2, dLineLengthL);
	pLineStringR->GetMAtPoint(pLineStringR->GetCoordinateN(ncoordsR - 1), ncoordsR - 2, dLineLengthR);

	Array<Engine::Geometries::Coordinate *> *arrCentercoords =
		new Array<Engine::Geometries::Coordinate *>();

	// 添加起始处中点
	arrCentercoords->Add(new Geometries::Coordinate((*(pCoorsL->GetAt(0)) + *(pCoorsR->GetAt(0))) / 2));

	Int32 i = 1;
	Int32 j = 1;
	Double dDisL = -1.0;
	Double dDisR = -1.0;
	Double dMeasureL = -1.0;
	Double dMeasureR = -1.0;
	Engine::Geometries::Coordinate *pCoorCurL = NULL;
	Engine::Geometries::Coordinate *pCoorCurR = NULL;
	Engine::Geometries::Coordinate *pCoorCurM = NULL;
	Engine::Geometries::Coordinate *pCoorLTmp = NULL;
	Engine::Geometries::Coordinate *pCoorRTmp = NULL;

	Geometries::Coordinate coorInterPt = Geometries::Coordinate(0.0, 0.0, 0.0);
	Geometries::Coordinate coorPtLS = *(pCoorsL->GetAt(0));
	Geometries::Coordinate coorPtRS = *(pCoorsR->GetAt(0));
	Geometries::Coordinate coorPtLE = *(pCoorsL->GetAt(ncoordsL - 1));
	Geometries::Coordinate coorPtRE = *(pCoorsR->GetAt(ncoordsR - 1));

	// 此处暂不采用0、1、3方法  [12/27/2016 guohaiqiang]
	int iCalType = 2;
	// int iCalType = 0; // 计算方法 0:平行线裁切；1:圆半径裁切；2:等距离比例算中心点；3:最近点算中心点（算法组原始计算方法）
	// Double dWidthMax = 0.0;
	// Double dWidthMin = 0.0;
	// Double dWidthAvg = CalShortLAvgDstToLongL(pLineStringL, pLineStringR, dWidthMax, dWidthMin);
	// if (Math::Fabs(dWidthMax - dWidthMin) < dWidthAvg / 0.5)
	//{
	//	iCalType = 3;
	// }
	// else if ((coorPtLS - coorPtRS).IsZeroVector() || (coorPtLE - coorPtRE).IsZeroVector())
	//{
	//	iCalType = 2;
	// }
	// else
	//{
	//	Bool bHaveInterPt = InterPtOfStrtLines(coorPtLS, coorPtRS, coorPtLE, coorPtRE, coorInterPt);
	//	if (bHaveInterPt)
	//	{
	//		iCalType = 1;
	//	}
	// }

	if (iCalType == 0)
	{
		Coordinate coorV = coorPtRS - coorPtLS;
		while ((i < ncoordsL - 1 || j < ncoordsR - 1) && (i + j < ncoordsL + ncoordsR - 2))
		{
			pCoorCurL = pCoorsL->GetAt(i);
			pCoorCurR = pCoorsR->GetAt(j);
			pCoorCurM = arrCentercoords->GetAt(arrCentercoords->GetCount() - 1);

			if (pCoorLTmp == NULL)
			{
				coorInterPt = *pCoorCurL + coorV;
				pCoorLTmp = new Engine::Geometries::Coordinate(0.0, 0.0, 0.0);
				InterPtOfRadLineAndMultLine(*pCoorCurL, coorInterPt, pLineStringR, *pCoorLTmp);
				*pCoorLTmp = (*pCoorCurL + *pCoorLTmp) / 2;
			}

			if (pCoorRTmp == NULL)
			{
				coorInterPt = *pCoorCurR - coorV;
				pCoorRTmp = new Engine::Geometries::Coordinate();
				InterPtOfRadLineAndMultLine(*pCoorCurR, coorInterPt, pLineStringL, *pCoorRTmp);
				*pCoorRTmp = (*pCoorCurR + *pCoorRTmp) / 2;
			}

			dDisL = pCoorCurM->DistanceXY(*pCoorLTmp);
			dDisR = pCoorCurM->DistanceXY(*pCoorRTmp);

			if ((dDisL < dDisR || j == ncoordsR - 1) && i < ncoordsL - 1)
			{
				if (i < ncoordsL - 1)
				{
					if (dDisL > Geometries_EP)
					{
						arrCentercoords->Add(new Geometries::Coordinate(*pCoorLTmp));
					}
					DELETE_PTR(pCoorLTmp);
					i++;
					continue;
				}
			}

			if ((dDisL >= dDisR || i == ncoordsL - 1) && j < ncoordsR - 1)
			{
				if (j < ncoordsR - 1 || i == ncoordsR - 1)
				{
					if (dDisR > Geometries_EP)
					{
						arrCentercoords->Add(new Geometries::Coordinate(*pCoorRTmp));
					}
					DELETE_PTR(pCoorRTmp);
					j++;
					continue;
				}
			}
		}
	}
	else if (iCalType == 1)
	{
		while ((i < ncoordsL - 1 || j < ncoordsR - 1) && (i + j < ncoordsL + ncoordsR - 2))
		{
			pCoorCurL = pCoorsL->GetAt(i);
			pCoorCurR = pCoorsR->GetAt(j);
			pCoorCurM = arrCentercoords->GetAt(arrCentercoords->GetCount() - 1);

			if (pCoorLTmp == NULL)
			{
				pCoorLTmp = new Engine::Geometries::Coordinate(0.0, 0.0, 0.0);
				InterPtOfRadLineAndMultLine(coorInterPt, *pCoorCurL, pLineStringR, *pCoorLTmp);
				*pCoorLTmp = (*pCoorCurL + *pCoorLTmp) / 2;
			}

			if (pCoorRTmp == NULL)
			{
				pCoorRTmp = new Engine::Geometries::Coordinate();
				InterPtOfRadLineAndMultLine(coorInterPt, *pCoorCurR, pLineStringL, *pCoorRTmp);
				*pCoorRTmp = (*pCoorCurR + *pCoorRTmp) / 2;
			}

			dDisL = pCoorCurM->DistanceXY(*pCoorLTmp);
			dDisR = pCoorCurM->DistanceXY(*pCoorRTmp);

			if ((dDisL < dDisR || j == ncoordsR - 1) && i < ncoordsL - 1)
			{
				if (i < ncoordsL - 1)
				{
					if (dDisL > Geometries_EP)
					{
						arrCentercoords->Add(new Geometries::Coordinate(*pCoorLTmp));
					}
					DELETE_PTR(pCoorLTmp);
					i++;
					continue;
				}
			}

			if ((dDisL >= dDisR || i == ncoordsL - 1) && j < ncoordsR - 1)
			{
				if (j < ncoordsR - 1 || i == ncoordsR - 1)
				{
					if (dDisR > Geometries_EP)
					{
						arrCentercoords->Add(new Geometries::Coordinate(*pCoorRTmp));
					}
					DELETE_PTR(pCoorRTmp);
					j++;
					continue;
				}
			}
		}
	}
	else if (iCalType == 2)
	{
		while ((i < ncoordsL - 1 || j < ncoordsR - 1) && (i + j < ncoordsL + ncoordsR - 2))
		{
			pCoorCurL = pCoorsL->GetAt(i);
			pCoorCurR = pCoorsR->GetAt(j);
			pLineStringL->GetMAtPoint(pCoorCurL, i - 1, dMeasureL);
			pLineStringR->GetMAtPoint(pCoorCurR, j - 1, dMeasureR);
			pCoorCurM = arrCentercoords->GetAt(arrCentercoords->GetCount() - 1);

			if (pCoorLTmp == NULL)
			{
				pCoorLTmp = GetPtInLineByMeasure(pLineStringR, dLineLengthR * dMeasureL / dLineLengthL);
				if (pCoorLTmp == NULL)
				{
					pCoorLTmp = new Geometries::Coordinate(*(pLineStringR->GetCoordinates()->GetAt(ncoordsR - 1)));
				}
				*pCoorLTmp = (*pCoorCurL + *pCoorLTmp) / 2;
			}

			if (pCoorRTmp == NULL)
			{
				pCoorRTmp = GetPtInLineByMeasure(pLineStringL, dLineLengthL * dMeasureR / dLineLengthR);
				if (pCoorRTmp == NULL)
				{
					pCoorRTmp = new Geometries::Coordinate(*(pLineStringL->GetCoordinates()->GetAt(ncoordsL - 1)));
				}
				*pCoorRTmp = (*pCoorCurR + *pCoorRTmp) / 2;
			}

			dDisL = pCoorCurM->DistanceXY(*pCoorLTmp);
			dDisR = pCoorCurM->DistanceXY(*pCoorRTmp);

			if ((dDisL < dDisR || j == ncoordsR - 1) && i < ncoordsL - 1)
			{
				if (dDisL > Geometries_EP)
				{
					arrCentercoords->Add(new Geometries::Coordinate(*pCoorLTmp));
				}
				DELETE_PTR(pCoorLTmp);
				i++;
				continue;
			}

			if ((dDisL >= dDisR || i == ncoordsL - 1) && j < ncoordsR - 1)
			{
				if (dDisR > Geometries_EP)
				{
					arrCentercoords->Add(new Geometries::Coordinate(*pCoorRTmp));
				}
				DELETE_PTR(pCoorRTmp);
				j++;
				continue;
			}
		}
	}
	else if (iCalType == 3)
	{
		int nSegIndex = 0;
		while ((i < ncoordsL - 1 || j < ncoordsR - 1) && (i + j < ncoordsL + ncoordsR - 2))
		{
			pCoorCurL = pCoorsL->GetAt(i);
			pCoorCurR = pCoorsR->GetAt(j);
			pLineStringL->GetMAtPoint(pCoorCurL, i - 1, dMeasureL);
			pLineStringR->GetMAtPoint(pCoorCurR, j - 1, dMeasureR);
			pCoorCurM = arrCentercoords->GetAt(arrCentercoords->GetCount() - 1);

			if (pCoorLTmp == NULL)
			{
				pCoorLTmp = new Geometries::Coordinate();
				BaseAlgorithm::GetNearestPntToLineset(pCoorCurL, pLineStringR->GetCoordinates(), *pCoorLTmp, nSegIndex);
				*pCoorLTmp = (*pCoorCurL + *pCoorLTmp) / 2;
			}

			if (pCoorRTmp == NULL)
			{
				pCoorRTmp = new Geometries::Coordinate();
				BaseAlgorithm::GetNearestPntToLineset(pCoorCurR, pLineStringL->GetCoordinates(), *pCoorRTmp, nSegIndex);
				*pCoorRTmp = (*pCoorCurR + *pCoorRTmp) / 2;
			}

			dDisL = pCoorCurM->DistanceXY(*pCoorLTmp);
			dDisR = pCoorCurM->DistanceXY(*pCoorRTmp);

			if ((dDisL < dDisR || j == ncoordsR - 1) && i < ncoordsL - 1)
			{
				if (dDisL > Geometries_EP)
				{
					arrCentercoords->Add(new Geometries::Coordinate(*pCoorLTmp));
				}
				DELETE_PTR(pCoorLTmp);
				i++;
				continue;
			}

			if ((dDisL >= dDisR || i == ncoordsL - 1) && j < ncoordsR - 1)
			{
				if (dDisR > Geometries_EP)
				{
					arrCentercoords->Add(new Geometries::Coordinate(*pCoorRTmp));
				}
				DELETE_PTR(pCoorRTmp);
				j++;
				continue;
			}
		}
	}

	// 添加末尾处中点
	arrCentercoords->Add(new Geometries::Coordinate((*(pCoorsL->GetAt(ncoordsL - 1)) + *(pCoorsR->GetAt(ncoordsR - 1))) / 2));

	Engine::Geometries::LineString *pCenterLine =
		new Engine::Geometries::LineString(arrCentercoords);
	arrCentercoords = NULL;

	if (pCenterLine->GetNumPoints() < 2)
	{
		DELETE_PTR(pCenterLine);
		return NULL;
	}

	return pCenterLine;
}

int IRCreatLaneAlgorithm::PtPositionWithPolygon(
	const Engine::Geometries::Polygon *polygon,
	const Engine::Geometries::Coordinate *pCoor)
{
	int iResult = 0;

	// 交点个数
	int nCross = 0;
	Array<Geometries::Coordinate *> *arrCoors = polygon->GetExteriorRing()->GetCoordinates();

	for (int i = 0; i < arrCoors->GetCount() - 1; i++)
	{
		Geometries::Coordinate *p1 = arrCoors->GetAt(i);
		Geometries::Coordinate *p2 = arrCoors->GetAt(i + 1);

		if (p1->y == p2->y)
		{
			continue;
		}

		if (pCoor->y < std::min(p1->y, p2->y))
		{
			continue;
		}

		if (pCoor->y >= std::max(p1->y, p2->y))
		{
			continue;
		}

		// 求交点的x坐标
		double x = (double)(pCoor->y - p1->y) * (double)(p2->x - p1->x) / (double)(p2->y - p1->y) + p1->x;

		// 点在边上
		if (x == pCoor->x)
		{
			iResult = 1;
			break;
		}

		// 只统计p1p2与p向右射线的交点
		if (x > pCoor->x)
		{
			nCross++;
		}
	}

	// 交点为偶数，点在多边形之外
	if (iResult == 0 && nCross % 2 == 0)
	{
		iResult = 2;
	}

	return iResult;
}

Base::Bool IRCreatLaneAlgorithm::GetNearestPtOnMultLine(
	const Engine::Geometries::Coordinate *ptTest,
	const Engine::Geometries::LineString *ls,
	Engine::Geometries::Coordinate &coord,
	int &iIndex)
{
	if (ptTest == NULL || ls == NULL || ls->GetNumPoints() < 2)
	{
		return false;
	}

	Coordinate coorPrj;
	Base::Bool bRst = BaseAlgorithm::GetNearestPntToLineset(ptTest, ls->GetCoordinates(), coorPrj, iIndex);
	if (!bRst)
	{
		return false;
	}

	Coordinate *pCoorS = ls->GetCoordinateN(iIndex);
	Coordinate *pCoorE = ls->GetCoordinateN(iIndex + 1);
	Base::Double dDiffZ = pCoorE->z - pCoorS->z;
	if (dDiffZ < Geometries_EP && dDiffZ > Geometries_NEP)
	{
		coorPrj.z = (pCoorS->z + pCoorE->z) / 2.0;
	}
	else if (pCoorS->DistanceXY(*pCoorE) > Geometries_EP)
	{
		coorPrj.z = pCoorS->z + dDiffZ * pCoorS->DistanceXY(coorPrj) / pCoorS->DistanceXY(*pCoorE);
	}
	else
	{
		coorPrj.z = pCoorS->z < pCoorE->z ? pCoorS->z : pCoorE->z;
	}

	coord = coorPrj;

	return true;
}

Base::Bool IRCreatLaneAlgorithm::BreakMultLineByPlane(
	const Geometries::LineString *pMultLine,
	const Engine::Geometries::Coordinate ptInPlane,
	const Engine::Geometries::Coordinate normalVector,
	Geometries::LineString *&pLsS,
	Geometries::LineString *&pLsE)
{
	if (pMultLine == NULL || pMultLine->GetNumPoints() < 2 ||
		ptInPlane.IsZeroVector() || normalVector.IsZeroVector())
	{
		return false;
	}
	int iCoorNum = pMultLine->GetNumPoints();
	Coordinate *pCoorS = NULL;
	Coordinate *pCoorE = NULL;
	Coordinate *pCoorInter = new Coordinate();
	Base::Bool bHaveInter = false;
	int i = 0;
	for (; i < iCoorNum - 1; i++)
	{
		pCoorS = pMultLine->GetCoordinateN(i);
		pCoorE = pMultLine->GetCoordinateN(i + 1);

		if (BaseAlgorithm3D::IntersectionLineSegmentPlane(*pCoorS, *pCoorE, ptInPlane, normalVector, *pCoorInter))
		{
			bHaveInter = true;
			break;
		}
	}
	if (!bHaveInter)
	{
		return false;
	}
	DELETE_PTR(pLsS);
	DELETE_PTR(pLsE);
	pLsS = new Geometries::LineString();
	pLsE = new Geometries::LineString();
	GeometryAlgorithm::PtBreakLineSegments(pCoorInter, pMultLine->GetCoordinates(), pLsS, pLsE); // 一个交点将一条线打断生成两条线
	if (pLsS == NULL && pLsE == NULL)
	{
		return false;
	}
	return true;
}

// void IRCreatLaneAlgorithm::ClipLineByDistence(
//	Engine::Geometries::LineString* pLineRef,
//	const Engine::Geometries::LineString* pMultLine,
//	Array<Geometries::LineString*> & arrClipLines,
//	double dDistence)
//{
//	//arrClipLines.Clear();
//	if (pLineRef == NULL || pLineRef->GetNumPoints() < 2 ||
//		pMultLine == NULL || pMultLine->GetNumPoints() < 2 ||
//		dDistence <= 0.0)
//	{
//		return;
//	}
//
//	int iCoorNum = pMultLine->GetNumPoints();
//	int iPositionF = BaseAlgorithm::PntMatchLineSegments(pLineRef->GetCoordinates(), *pMultLine->GetCoordinateN(0));
//	for (int i = 0; i < iCoorNum; i++)
//	{
//		if (BaseAlgorithm::PntMatchLineSegments(pLineRef->GetCoordinates(), *pMultLine->GetCoordinateN(i)) != iPositionF)
//		{
//			return;//如果两条多段线相交，直接返回
//		}
//	}
//
//	double dWidthMax = 0.0;
//	double dWidthMin = 0.0;
//	double dWidthAvg = CalShortLAvgDstToLongL(pMultLine, pLineRef, dWidthMax, dWidthMin);
//	if (dWidthAvg < 0.0 || dWidthMax < dDistence)
//	{
//		return;
//	}
//
//	if (dWidthMin >= dDistence)
//	{
//		arrClipLines.Add((LineString*)pMultLine->Clone());
//		return;
//	}
//
//	Coordinate* pCoorCur = NULL;
//	Coordinate* pCoorCurN = NULL;
//	Coordinate pCoorPrj;
//	Coordinate pCoorPrjN;
//	int iPrjIndex = -1;
//	Coordinate pCoorInter = Coordinate(0.0, 0.0, 0.0);
//	double dDisCur = 0.0;
//	double dDisCurN = 0.0;
//	LineString* pLineTmp = NULL;
//	Array<Coordinate*>* pArrCoors = NULL;
//	for (int i = 0; i < iCoorNum; i++)
//	{
//		pCoorCur = pMultLine->GetCoordinateN(i);
//		if (!GetNearestPtOnMultLine(pCoorCur, pLineRef, pCoorPrj, iPrjIndex))
//		{
//			continue;
//		}
//
//		dDisCur = pCoorCur->DistanceXY(pCoorPrj);
//		if (dDisCur < dDistence)
//		{
//			if (pLineTmp != NULL)
//			{
//				pCoorInter = *pCoorCurN + (*pCoorCur - *pCoorCurN) * (dDistence - dDisCurN) / (dDisCur - dDisCurN);
//				pArrCoors->Add(new Coordinate(pCoorInter));
//				pLineTmp->Distinct();
//				arrClipLines.Add(pLineTmp);
//				pLineTmp = NULL;
//				pArrCoors = NULL;
//			}
//			pCoorCurN = pCoorCur;
//			pCoorPrjN = pCoorPrj;
//			dDisCurN = dDisCur;
//			continue;
//		}
//
//		if (pLineTmp == NULL)
//		{
//			pLineTmp = new LineString();
//			pArrCoors = pLineTmp->GetCoordinates();
//		}
//
//		if (i != 0 && dDisCurN < dDistence)
//		{
//			pCoorInter = *pCoorCurN + (*pCoorCur - *pCoorCurN) * (dDistence - dDisCurN) / (dDisCur - dDisCurN);
//			pArrCoors->Add(new Coordinate(pCoorInter));
//		}
//		pCoorCurN = pCoorCur;
//		pCoorPrjN = pCoorPrj;
//		dDisCurN = dDisCur;
//		pArrCoors->Add(new Coordinate(*pCoorCur));
//	}
//	if (pLineTmp != NULL)
//	{
//		pLineTmp->Distinct();
//		arrClipLines.Add(pLineTmp);
//		pLineTmp = NULL;
//		pArrCoors = NULL;
//	}
// }

void IRCreatLaneAlgorithm::ClipLineByDistence(
	Engine::Geometries::LineString *pLineRef,
	const Engine::Geometries::LineString *pMultLine,
	Array<Geometries::LineString *> &arrClipLines,
	double dDistence)
{
	if (pLineRef == NULL || pLineRef->GetNumPoints() < 2 ||
		pMultLine == NULL || pMultLine->GetNumPoints() < 2 ||
		dDistence <= 0.0)
	{
		return;
	}

	int iCoorNum = pMultLine->GetNumPoints();
	int iPositionF = LineOnWhichSideOfAnother(pLineRef, pMultLine);
	if (iPositionF < 0)
	{
		return;
	}
	if (iPositionF == 0)
	{
		arrClipLines.Add((LineString *)pMultLine->Clone());
		return;
	}

	double dWidthMax = 0.0;
	double dWidthMin = 0.0;
	double dWidthAvg = CalShortLAvgDstToLongL(pMultLine, pLineRef, dWidthMax, dWidthMin);
	if (dWidthAvg < 0.0 || dWidthMax < dDistence)
	{
		return;
	}

	if (dWidthMin >= dDistence)
	{
		arrClipLines.Add((LineString *)pMultLine->Clone());
		return;
	}

	Geometries::LineString *pOffsetLine = GeometryAlgorithm::GenerateOffsetLine(pLineRef, dDistence * (iPositionF == 1 ? 1.0 : -1.0));
	if (pOffsetLine == NULL || pOffsetLine->GetNumPoints() < 2)
	{
		DELETE_PTR(pOffsetLine);
		return;
	}

	Coordinate *pCoorCur = NULL;
	Coordinate *pCoorCurN = NULL;
	Coordinate coorInter;
	LineString *pLineTmp = NULL;
	Array<Coordinate *> *pArrCoors = NULL;
	for (int i = 0; i < iCoorNum - 1; i++)
	{
		pCoorCur = pMultLine->GetCoordinateN(i);
		pCoorCurN = pMultLine->GetCoordinateN(i + 1);

		Array<Coordinate *> arrIntercept;
		Array<Double> arrMeasure;
		GetInterceptLineSegmentWithMultLine(pOffsetLine, pCoorCur, pCoorCurN, arrIntercept, arrMeasure);

		if (pLineTmp == NULL)
		{
			pLineTmp = new LineString();
			pArrCoors = pLineTmp->GetCoordinates();
			pArrCoors->Add(new Coordinate(*pCoorCur));
		}

		for (int j = 0; j < arrIntercept.GetCount(); j++)
		{
			coorInter = *arrIntercept.GetAt(j);
			pArrCoors->Add(arrIntercept.GetAt(j));
			pLineTmp->DistinctXY();
			if (pLineTmp->GetNumPoints() >= 2)
			{
				arrClipLines.Add(pLineTmp);
				pLineTmp = new LineString();
				pArrCoors = pLineTmp->GetCoordinates();
				pArrCoors->Add(new Coordinate(coorInter));
			}
		}
		pArrCoors->Add(new Coordinate(*pCoorCurN));
	}
	if (pLineTmp != NULL)
	{
		pLineTmp->Distinct();
		if (pLineTmp->GetNumPoints() >= 2)
		{
			arrClipLines.Add(pLineTmp);
			pLineTmp = NULL;
			pArrCoors = NULL;
		}
		DELETE_PTR(pLineTmp);
	}

	DELETE_PTR(pOffsetLine);
}

int IRCreatLaneAlgorithm::LineOnWhichSideOfAnother(
	const Engine::Geometries::LineString *pLineRef,
	const Engine::Geometries::LineString *pLineTest,
	bool bJudgeOnce)
{
	if (pLineRef == NULL || pLineRef->GetNumPoints() < 2 || pLineTest == NULL || pLineTest->GetNumPoints() <= 0)
	{
		return -1;
	}

	if (GeometryAlgorithm::Intersects(pLineRef, pLineTest))
	{
		return -1;
	}
	Coordinate *pRefCoorS = pLineRef->GetCoordinateN(0);
	Coordinate *pRefCoorE = pLineRef->GetCoordinateN(pLineRef->GetNumPoints() - 1);
	Coordinate *pTestCoorS = pLineTest->GetCoordinateN(0);
	Coordinate *pTestCoorE = pLineTest->GetCoordinateN(pLineTest->GetNumPoints() - 1);
	LineString *pLineRefInter = GeometryAlgorithm::InterceptPartLine(pTestCoorS, pTestCoorE, const_cast<LineString *>(pLineRef));
	if (pLineRefInter == NULL)
	{
		pLineRefInter = GeometryAlgorithm::InterceptPartLine(pTestCoorE, pTestCoorS, const_cast<LineString *>(pLineRef));
	}
	LineString *pLineTestInter = GeometryAlgorithm::InterceptPartLine(pRefCoorS, pRefCoorE, const_cast<LineString *>(pLineTest));
	if (pLineTestInter == NULL)
	{
		pLineTestInter = GeometryAlgorithm::InterceptPartLine(pRefCoorE, pRefCoorS, const_cast<LineString *>(pLineTest));
	}
	if (pLineRefInter == NULL || pLineTestInter == NULL)
	{
		DELETE_PTR(pLineRefInter);
		DELETE_PTR(pLineTestInter);
		return -1;
	}

	int iPositionF = BaseAlgorithm::PntMatchLineSegments(pLineRefInter->GetCoordinates(), *pLineTestInter->GetCoordinateN(0));
	for (int i = 0; i < pLineTestInter->GetNumPoints(); i++)
	{
		int iPositionCur = BaseAlgorithm::PntMatchLineSegments(pLineRefInter->GetCoordinates(), *pLineTestInter->GetCoordinateN(i));
		if (iPositionCur == 0)
		{
			continue;
		}
		if (iPositionF == 0)
		{
			iPositionF = iPositionCur;
			continue;
		}
		if (iPositionCur != iPositionF)
		{
			DELETE_PTR(pLineRefInter);
			DELETE_PTR(pLineTestInter);
			return -1; // 如果两条多段线交叉，直接返回
		}
	}
	if (bJudgeOnce)
	{
		DELETE_PTR(pLineRefInter);
		DELETE_PTR(pLineTestInter);
		return iPositionF;
	}
	for (int i = 0; i < pLineRefInter->GetNumPoints(); i++)
	{
		int iPositionCur = BaseAlgorithm::PntMatchLineSegments(pLineTestInter->GetCoordinates(), *pLineRefInter->GetCoordinateN(i));
		if (iPositionCur == 0)
		{
			continue;
		}
		iPositionCur = 3 - iPositionCur;
		if (iPositionF == 0)
		{
			iPositionF = iPositionCur;
			continue;
		}
		if (iPositionCur != iPositionF)
		{
			DELETE_PTR(pLineRefInter);
			DELETE_PTR(pLineTestInter);
			return -1; // 如果两条多段线交叉，直接返回
		}
	}
	DELETE_PTR(pLineRefInter);
	DELETE_PTR(pLineTestInter);
	return iPositionF;
}

double IRCreatLaneAlgorithm::CalPrjMeasureScale(
	Engine::Geometries::LineString *pLineRef,
	Array<Geometries::LineString *> &arrClipLines)
{
	if (pLineRef == NULL || pLineRef->GetNumPoints() < 2 || arrClipLines.IsEmpty())
	{
		return -1.0;
	}

	double dPrjLengthTotal = 0.0;
	double dPrjLengthS = 0.0;
	double dPrjLengthE = 0.0;
	int iPrjIndex = -1;
	int iPrjIndexN = -1;
	Coordinate *pCoorPrjS = NULL;
	Coordinate *pCoorPrjE = NULL;
	Coordinate *pCoorLineS = NULL;
	Coordinate *pCoorLineE = NULL;
	LineString *pLineTmp = NULL;
	int iLineNum = arrClipLines.GetCount();
	for (int i = 0; i < iLineNum; i++)
	{
		pLineTmp = arrClipLines.GetAt(i);
		DELETE_PTR(pCoorPrjS);
		DELETE_PTR(pCoorPrjE);
		pCoorPrjS = new Coordinate(0.0, 0.0, 0.0);
		pCoorPrjE = new Coordinate(0.0, 0.0, 0.0);
		pCoorLineS = pLineTmp->GetCoordinateN(0);
		pCoorLineE = pLineTmp->GetCoordinateN(pLineTmp->GetNumPoints() - 1);
		bool bRstS = BaseAlgorithm::GetNearestPntToLineset(pCoorLineS, pLineRef->GetCoordinates(), *pCoorPrjS, iPrjIndex);
		bool bRstE = BaseAlgorithm::GetNearestPntToLineset(pCoorLineE, pLineRef->GetCoordinates(), *pCoorPrjE, iPrjIndexN);
		if (bRstS && bRstE)
		{
			bRstS = pLineRef->GetMAtPoint(pCoorPrjS, iPrjIndex, dPrjLengthS);
			bRstE = pLineRef->GetMAtPoint(pCoorPrjE, iPrjIndexN, dPrjLengthE);
			if (bRstS && bRstE && dPrjLengthE > dPrjLengthS)
			{
				dPrjLengthTotal += dPrjLengthE - dPrjLengthS;
			}
		}
	}
	return dPrjLengthTotal / pLineRef->GetLength();
}

bool IRCreatLaneAlgorithm::ClipLineByPts(
	Engine::Geometries::LineString *pLine,
	Array<Geometries::Coordinate *> arrCoors,
	Array<Geometries::LineString *> &arrLines)
{
	if (pLine == NULL || pLine->GetNumPoints() < 2 || arrCoors.IsEmpty())
	{
		return false;
	}

	int iCoorNum = arrCoors.GetCount();
	int iLineCoorNum = pLine->GetNumPoints();
	Geometries::Coordinate *pCoorCur = NULL;
	Geometries::Coordinate *pLineCoorCur = NULL;
	Geometries::Coordinate pCoorPrj;
	Geometries::Coordinate pCoorPrjN;
	int iPrjIndex = -1;
	int iPrjIndexN = 0;
	Geometries::LineString *pLineTmp = NULL;
	for (int i = 0; i < iCoorNum; i++)
	{
		pCoorCur = arrCoors.GetAt(i);
		if (!GetNearestPtOnMultLine(pCoorCur, pLine, pCoorPrj, iPrjIndex))
		{
			continue;
		}
		if (pLineTmp == NULL)
		{
			pLineTmp = new Geometries::LineString();
		}
		if (!pCoorPrjN.IsZeroVector())
		{
			pLineTmp->GetCoordinates()->Add(new Coordinate(pCoorPrjN));
		}
		else
		{
			pLineTmp->GetCoordinates()->Add(new Coordinate(*pLine->GetCoordinateN(0)));
		}
		for (int j = iPrjIndexN + 1; j <= iPrjIndex; j++)
		{
			pCoorCur = pLine->GetCoordinateN(j);
			pLineTmp->GetCoordinates()->Add(new Coordinate(*pCoorCur));
		}
		pLineTmp->GetCoordinates()->Add(new Coordinate(pCoorPrj));
		iPrjIndexN = iPrjIndex;
		pCoorPrjN = pCoorPrj;
		pLineTmp->Distinct();
		if (pLineTmp->GetNumPoints() >= 2 && pLineTmp->GetLength() > Geometries_EP)
		{
			arrLines.Add(pLineTmp);
		}
		else
		{
			DELETE_PTR(pLineTmp);
		}
		pLineTmp = NULL;
	}

	pLineTmp = new Geometries::LineString();
	if (!pCoorPrjN.IsZeroVector())
	{
		pLineTmp->GetCoordinates()->Add(new Coordinate(pCoorPrjN));
	}
	for (int j = iPrjIndexN + 1; j < iLineCoorNum; j++)
	{
		pCoorCur = pLine->GetCoordinateN(j);
		pLineTmp->GetCoordinates()->Add(new Coordinate(*pCoorCur));
	}
	pLineTmp->Distinct();
	if (pLineTmp->GetNumPoints() >= 2 && pLineTmp->GetLength() > Geometries_EP)
	{
		arrLines.Add(pLineTmp);
		pLineTmp = NULL;
	}
	else
	{
		DELETE_PTR(pLineTmp);
	}
	if (arrLines.IsEmpty())
	{
		return false;
	}
	return true;
}

bool IRCreatLaneAlgorithm::IsMultLineSelfIntersection(
	Engine::Geometries::LineString *pLine,
	Int32 &iInterIndex1, Int32 &iInterIndex2)
{
	if (pLine == NULL || pLine->GetNumPoints() < 4)
	{
		return false;
	}
	int iCoorNum = pLine->GetNumPoints();
	int i = 0;
	int j = 0;
	Geometries::Coordinate *pCoorLineSegS1 = NULL;
	Geometries::Coordinate *pCoorLineSegE1 = NULL;
	Geometries::Coordinate *pCoorLineSegS2 = NULL;
	Geometries::Coordinate *pCoorLineSegE2 = NULL;
	Geometries::Coordinate coorInterPt = Geometries::Coordinate(0.0, 0.0, 0.0);
	bool bIsIntersection = false;
	for (i = 0; i < iCoorNum - 2; i++)
	{
		pCoorLineSegS1 = pLine->GetCoordinateN(i);
		pCoorLineSegE1 = pLine->GetCoordinateN(i + 1);
		for (j = i + 2; j < iCoorNum - 1; j++)
		{
			pCoorLineSegS2 = pLine->GetCoordinateN(j);
			pCoorLineSegE2 = pLine->GetCoordinateN(j + 1);
			if (InterPtOfLineSegs(*pCoorLineSegS1, *pCoorLineSegE1,
								  *pCoorLineSegS2, *pCoorLineSegE2, coorInterPt))
			{
				iInterIndex1 = i;
				iInterIndex2 = j;
				bIsIntersection = true;
				break;
			}
		}
		if (bIsIntersection)
		{
			break;
		}
	}
	return bIsIntersection;
}

void IRCreatLaneAlgorithm::SimpleMultLine(
	Engine::Geometries::LineString *&pLine,
	double dSimpleLength)
{
	if (pLine == NULL || pLine->GetNumPoints() <= 2)
	{
		return;
	}
	int iCoorNum = pLine->GetNumPoints();
	Array<Geometries::Coordinate *> *pCoors = pLine->GetCoordinates();
	Geometries::Coordinate *pCoorLineSegS = NULL;
	Geometries::Coordinate *pCoorLineSegE = NULL;
	for (int i = 0; i < iCoorNum - 1; i++)
	{
		pCoorLineSegS = pCoors->GetAt(i);
		pCoorLineSegE = pCoors->GetAt(i + 1);
		if (pCoorLineSegS->DistanceXY(*pCoorLineSegE) < dSimpleLength)
		{
			delete pCoorLineSegE;
			pCoors->Delete(i + 1);
			iCoorNum--;
			i--;
		}
	}
}

bool IRCreatLaneAlgorithm::GetVerticalOffsetPoint(
	Engine::Geometries::Coordinate pOrgCoor,
	Engine::Geometries::Coordinate pOrgVector,
	Engine::Geometries::Coordinate &pDisCoor,
	Base::Double dOffsetDis)
{
	if (pOrgVector.IsZeroVector())
	{
		return false;
	}

	if (Math::Fabs(dOffsetDis) < Geometries_EP)
	{
		pDisCoor = pOrgCoor;
		return true;
	}

	Geometries::Coordinate pDisVector = Geometries::Coordinate(pOrgVector);
	pDisVector.z = 0.0;
	pDisVector.Normalize();

	Double dTmp = pDisVector.x;
	pDisVector.x = -(pDisVector.y);
	pDisVector.y = dTmp;

	pDisCoor = pOrgCoor + pDisVector * dOffsetDis;
	return true;
}

Geometries::Polygon *IRCreatLaneAlgorithm::GetExternalRectangle(
	const Array<Coordinate *> *pntsSource)
{
	if (pntsSource->GetCount() < 3)
	{
		return NULL;
	}

	Int32 nSourceCount = pntsSource->GetCount();
	Double dPntsMinX = pntsSource->GetAt(0)->x;
	Double dPntsMaxX = pntsSource->GetAt(0)->x;
	Double dPntsMinY = pntsSource->GetAt(0)->y;
	Double dPntsMaxY = pntsSource->GetAt(0)->y;
	Int32 i = 0;
	Geometries::Coordinate *pCoorTmp = NULL;
	for (i = 1; i < nSourceCount; i++)
	{
		pCoorTmp = pntsSource->GetAt(i);
		if (pCoorTmp->x < dPntsMinX)
		{
			dPntsMinX = pCoorTmp->x;
		}
		if (pCoorTmp->x > dPntsMaxX)
		{
			dPntsMaxX = pCoorTmp->x;
		}
		if (pCoorTmp->y < dPntsMinY)
		{
			dPntsMinY = pCoorTmp->y;
		}
		if (pCoorTmp->y > dPntsMaxY)
		{
			dPntsMaxY = pCoorTmp->y;
		}
	}
	pCoorTmp = NULL;

	if (Math::Fabs(dPntsMaxX - dPntsMinX) < Geometries_EP ||
		Math::Fabs(dPntsMaxY - dPntsMinY) < Geometries_EP)
	{
		return NULL;
	}

	Geometries::LinearRing *pLineRing = new Geometries::LinearRing();
	pLineRing->GetCoordinates()->Add(new Geometries::Coordinate(dPntsMinX, dPntsMinY, 0.0));
	pLineRing->GetCoordinates()->Add(new Geometries::Coordinate(dPntsMinX, dPntsMaxY, 0.0));
	pLineRing->GetCoordinates()->Add(new Geometries::Coordinate(dPntsMaxX, dPntsMaxY, 0.0));
	pLineRing->GetCoordinates()->Add(new Geometries::Coordinate(dPntsMaxX, dPntsMinY, 0.0));
	pLineRing->GetCoordinates()->Add(new Geometries::Coordinate(dPntsMinX, dPntsMinY, 0.0));
	Geometries::Polygon *pPlgRst = new Geometries::Polygon(pLineRing);
	return pPlgRst;
}

bool IRCreatLaneAlgorithm::JudgePrjLineDirection(
	Geometries::LineString *pLineRef, Geometries::LineString *pLineTest)
{
	if (pLineRef == NULL || pLineRef->GetNumPoints() < 2 ||
		pLineTest == NULL || pLineTest->GetNumPoints() < 2)
	{
		return false;
	}

	Geometries::Coordinate *pCoorS = pLineTest->GetCoordinateN(0);
	Geometries::Coordinate *pCoorE = pLineTest->GetCoordinateN(pLineTest->GetNumPoints() - 1);

	if (pCoorS == NULL || pCoorE == NULL)
	{
		return false;
	}

	Geometries::Coordinate pCoorSPrj;
	Geometries::Coordinate pCoorEPrj;
	Int32 iPrjIndexS = -1;
	Int32 iPrjIndexE = -1;

	if (!BaseAlgorithm::GetNearestPntToLineset(pCoorS, pLineRef->GetCoordinates(), pCoorSPrj, iPrjIndexS) ||
		!BaseAlgorithm::GetNearestPntToLineset(pCoorE, pLineRef->GetCoordinates(), pCoorEPrj, iPrjIndexE))
	{
		return false;
	}

	if ((pCoorSPrj - pCoorEPrj).IsZeroVector())
	{
		return false;
	}

	if ((iPrjIndexS - iPrjIndexE) < 0)
	{
		return true;
	}
	else if (iPrjIndexS == iPrjIndexE)
	{
		Geometries::Coordinate *pCoorTmp = pLineRef->GetCoordinateN(iPrjIndexS);
		if (pCoorTmp->DistanceXY(pCoorSPrj) < pCoorTmp->DistanceXY(pCoorEPrj))
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		return false;
	}
}

bool IRCreatLaneAlgorithm::GetAverageVector2DOfLineString(Engine::Geometries::LineString *pLine, Engine::Geometries::Coordinate &vector)
{
	vector = Coordinate(0.0, 0.0, 0.0);
	if (pLine == NULL || pLine->GetNumPoints() < 2)
	{
		return false;
	}
	// int iVectorCount = 0;
	for (int i = 1; i < pLine->GetNumPoints(); i++)
	{
		Coordinate *pCoorS = pLine->GetCoordinateN(i - 1);
		Coordinate *pCoorE = pLine->GetCoordinateN(i);
		if (pCoorS == NULL || pCoorE == NULL)
		{
			return false;
		}
		Coordinate vectorCur = *pCoorE - *pCoorS;
		vectorCur.z = 0.0;
		vectorCur.Normalize();
		vector = vector + vectorCur;
		// iVectorCount++;
	}
	if (!vector.IsZeroVector())
	{
		vector.Normalize();
	}
	return true;
}

bool IRCreatLaneAlgorithm::GetAverageZOfLineString(Engine::Geometries::LineString *pLine, Engine::Base::Double &valueZ)
{
	valueZ = 0.0;
	if (pLine == NULL || pLine->GetNumPoints() < 2)
	{
		return false;
	}
	int iVectorCount = 0;
	for (int i = 0; i < pLine->GetNumPoints(); i++)
	{
		Coordinate *pCoorCur = pLine->GetCoordinateN(i);
		if (pCoorCur == NULL)
		{
			return false;
		}
		valueZ += pCoorCur->z;
		iVectorCount++;
	}
	valueZ = valueZ / (Double)iVectorCount;
	return true;
}

bool IRCreatLaneAlgorithm::GetTowLineAverageVectorAngle2D(Engine::Geometries::LineString *pLine1, Engine::Geometries::LineString *pLine2, Engine::Base::Double &dAngle)
{
	Coordinate vector1;
	Coordinate vector2;
	if (!GetAverageVector2DOfLineString(pLine1, vector1) || !GetAverageVector2DOfLineString(pLine2, vector2))
	{
		return false;
	}
	dAngle = GeometryAlgorithm::RadianToDegree(vector1.AngleWith(vector2));
	return true;
}

void IRCreatLaneAlgorithm::DealHECornerInLine(Engine::Geometries::LineString *pLineD)
{
	if (pLineD == NULL || pLineD->GetNumPoints() < 3)
	{
		return;
	}

	Array<Coordinate *> *pArrCoors = new Array<Coordinate *>();
	pArrCoors->Add(*pLineD->GetCoordinates());

	Coordinate *pCoorDel = NULL;
	Coordinate *pCoorTmp = pArrCoors->GetAt(0);
	if (pCoorTmp == NULL)
	{
		return;
	}
	Coordinate coorPrj = Coordinate(0.0, 0.0, 0.0);
	Coordinate coord1;
	Coordinate coord2;
	Coordinate coord3;
	Double dAng = -1.0;
	Int32 nSegIndex = -1;
	pArrCoors->Delete(0);
	bool b = BaseAlgorithm::GetNearestPntToLineset(pCoorTmp, pArrCoors, coorPrj, nSegIndex);
	if (b && coorPrj.DistanceXY(*pArrCoors->GetAt(0)) >= Geometries_EP &&
		coorPrj.DistanceXY(*pArrCoors->GetAt(pArrCoors->GetCount() - 1)) >= Geometries_EP)
	{
		bool bNeedDel = true;
		while (bNeedDel)
		{
			bNeedDel = false;
			pCoorDel = pArrCoors->GetAt(0);
			pArrCoors->Delete(0);
			pLineD->GetCoordinates()->Delete(1);
			DELETE_PTR(pCoorDel);
			if (pArrCoors->GetCount() < 3)
			{
				break;
			}
			coord1 = *pCoorTmp;
			coord2 = *pArrCoors->GetAt(0);
			coord3 = *pArrCoors->GetAt(1);
			dAng = GeometryAlgorithm::RadianToDegree(GeometryAlgorithm::ComputeAngle(coord1, coord2, coord3));
			if (dAng <= 90)
			{
				bNeedDel = true;
			}
		}
	}
	if (pArrCoors->GetCount() < 3)
	{
		return;
	}
	pCoorTmp = pArrCoors->GetAt(pArrCoors->GetCount() - 1);
	if (pCoorTmp == NULL)
	{
		return;
	}
	pArrCoors->Delete(pArrCoors->GetCount() - 1);
	b = BaseAlgorithm::GetNearestPntToLineset(pCoorTmp, pArrCoors, coorPrj, nSegIndex);
	if (b && coorPrj.DistanceXY(*pArrCoors->GetAt(0)) >= Geometries_EP &&
		coorPrj.DistanceXY(*pArrCoors->GetAt(pArrCoors->GetCount() - 1)) >= Geometries_EP)
	{
		bool bNeedDel = true;
		while (bNeedDel)
		{
			bNeedDel = false;
			pCoorDel = pArrCoors->GetAt(pArrCoors->GetCount() - 1);
			pArrCoors->Delete(pArrCoors->GetCount() - 1);
			pLineD->GetCoordinates()->Delete(pLineD->GetNumPoints() - 2);
			DELETE_PTR(pCoorDel);
			if (pArrCoors->GetCount() < 3)
			{
				break;
			}
			coord1 = *pCoorTmp;
			coord2 = *pArrCoors->GetAt(pArrCoors->GetCount() - 1);
			coord3 = *pArrCoors->GetAt(pArrCoors->GetCount() - 2);
			dAng = GeometryAlgorithm::RadianToDegree(GeometryAlgorithm::ComputeAngle(coord1, coord2, coord3));
			if (dAng <= 90)
			{
				bNeedDel = true;
			}
		}
	}
}

Base::Bool Engine::Geometries::IRCreatLaneAlgorithm::IsLineSegsCrossed(
	const Engine::Geometries::Coordinate pntLineSegS1,
	const Engine::Geometries::Coordinate pntLineSegE1,
	const Engine::Geometries::Coordinate pntLineSegS2,
	const Engine::Geometries::Coordinate pntLineSegE2,
	Base::Double tolerance /* = Geometries_NEP*/)
{
	// 参数方程
	Engine::Geometries::Coordinate linePara1 = calcLineSegParameters(pntLineSegS1, pntLineSegE1);
	Engine::Geometries::Coordinate linePara2 = calcLineSegParameters(pntLineSegS2, pntLineSegE2);
	// 如果平行则返回false
	if (fabs(linePara1.x * linePara2.y - linePara1.y * linePara2.x) < tolerance)
	{
		return false;
	}
	// 线段2的两点是否在线段1直线方程的值
	Base::Double valS2 = linePara1.x * pntLineSegS2.x + linePara1.y * pntLineSegS2.y + linePara1.z;
	Base::Double valE2 = linePara1.x * pntLineSegE2.x + linePara1.y * pntLineSegE2.y + linePara1.z;
	if (fabs(valS2) < tolerance) // 线段2起点在线段1上
	{
		if ((pntLineSegS2.x > val_fmin(pntLineSegS1.x, pntLineSegE1.x) && pntLineSegS2.x < val_fmax(pntLineSegS1.x, pntLineSegE1.x)) || (pntLineSegS2.y > val_fmin(pntLineSegS1.y, pntLineSegE1.y) && pntLineSegS2.y < val_fmax(pntLineSegS1.y, pntLineSegE1.y)))
		{
			return true;
		}
	}
	if (fabs(valE2) < tolerance) // 线段2终点在线段1上
	{
		if ((pntLineSegE2.x >= val_fmin(pntLineSegS1.x, pntLineSegE1.x) && pntLineSegE2.x <= val_fmax(pntLineSegS1.x, pntLineSegE1.x)) || (pntLineSegE2.y >= val_fmin(pntLineSegS1.y, pntLineSegE1.y) && pntLineSegE2.y <= val_fmax(pntLineSegS1.y, pntLineSegE1.y)))
		{
			return true;
		}
	}
	// 线段1的两点是否在线段2直线方程的值
	Base::Double valS1 = linePara2.x * pntLineSegS1.x + linePara2.y * pntLineSegS1.y + linePara2.z;
	Base::Double valE1 = linePara2.x * pntLineSegE1.x + linePara2.y * pntLineSegE1.y + linePara2.z;
	if (fabs(valS1) < tolerance) // 线段1起点在线段2上
	{
		if ((pntLineSegS1.x >= val_fmin(pntLineSegS2.x, pntLineSegE2.x) && pntLineSegS1.x <= val_fmax(pntLineSegS2.x, pntLineSegE2.x)) || (pntLineSegS1.y >= val_fmin(pntLineSegS2.y, pntLineSegE2.y) && pntLineSegS1.y <= val_fmax(pntLineSegS2.y, pntLineSegE2.y)))
		{
			return true;
		}
	}
	if (fabs(valE1) < tolerance) // 线段1起点在线段2上
	{
		if ((pntLineSegE1.x >= val_fmin(pntLineSegS2.x, pntLineSegE2.x) && pntLineSegE1.x <= val_fmax(pntLineSegS2.x, pntLineSegE2.x)) || (pntLineSegE1.y >= val_fmin(pntLineSegS2.y, pntLineSegE2.y) && pntLineSegE1.y <= val_fmax(pntLineSegS2.y, pntLineSegE2.y)))
		{
			return true;
		}
	}
	// 线段交叉判断
	if (valE1 > 0 != valS1 > 0 && valE2 > 0 != valS2 > 0)
	{
		return true;
	}
	return false;
}

Engine::Geometries::Coordinate Engine::Geometries::IRCreatLaneAlgorithm::calcLineSegParameters(
	const Engine::Geometries::Coordinate pntLineSegS,
	const Engine::Geometries::Coordinate pntLineSegE)
{
	Engine::Geometries::Coordinate linePara(0.0, 0.0, 0.0);
	linePara.x = pntLineSegE.y - pntLineSegS.y;
	linePara.y = -(pntLineSegE.x - pntLineSegS.x);
	linePara.z = -(linePara.x * pntLineSegS.x + linePara.y * pntLineSegS.y);
	return linePara;
}

Base::Void Engine::Geometries::IRCreatLaneAlgorithm::LineSegsCrossPnt(
	const Engine::Geometries::Coordinate pntLineSegS1,
	const Engine::Geometries::Coordinate pntLineSegE1,
	const Engine::Geometries::Coordinate pntLineSegS2,
	const Engine::Geometries::Coordinate pntLineSegE2,
	Engine::Geometries::Coordinate &crossPnt)
{
	// 参数方程
	Engine::Geometries::Coordinate linePara1 = calcLineSegParameters(pntLineSegS1, pntLineSegE1);
	Engine::Geometries::Coordinate linePara2 = calcLineSegParameters(pntLineSegS2, pntLineSegE2);
	Base::Double valueAB = linePara1.x * linePara2.y - linePara1.y * linePara2.x;
	crossPnt.x = -(linePara1.z * linePara2.y - linePara2.z * linePara1.y) / valueAB;
	crossPnt.y = (linePara1.z * linePara2.x - linePara2.z * linePara1.x) / valueAB;
	crossPnt.z = 0;
}
