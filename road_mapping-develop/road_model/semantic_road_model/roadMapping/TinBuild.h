#ifndef __TINBUILD_H_ 
#define __TINBUILD_H_ 

#include "Geometries/Coordinate.h"
#include "Base/Types.h"
#include <unordered_map>

using namespace Engine::Geometries;
using namespace Engine::Base;

typedef struct struct_Triangle
{
	Int32 nIndex0;
	Int32 nIndex1;
	Int32 nIndex2;

	Coordinate Point0;
	Coordinate Point1;
	Coordinate Point2;

    Int32 nE1; //对应的第一条边所在index 0-1
    Int32 nE2;//对应的第二条边所在index 1-2
    Int32 nE3;//对应的第三条边所在index 0-2
}Triangle;

typedef struct struct_Point
{
    Int32 nIndex; //点在集合中的索引
    Coordinate Pt;
}Point;

typedef struct struct_Edge
{
    Int32 nP1; //边的第一个顶点对应数据下标
    Int32 nP2;  //边的第二个顶点对应数据下标

    Int32 nUsedTimes; //记录边使用次数
}Edge;



//������Ϣ�ṹ��
typedef struct struct_LineTwoPts
{
	Coordinate ptStart;  //�����
	Coordinate ptEnd;    //���յ�
}LineTwoPts;

class TinBuild
{
	typedef std::unordered_map<Int32, Int32> MapIndex;

public:
	//离散点创建TIN
	static Bool PointToTin(const Array<Coordinate>& vecPoints, Array<Triangle>& vecTriangles, Array<Coordinate>& edgepoints);

	//给定上下线创建TIN三角网
	static Bool PointToTin(const Array<Coordinate>& topLine, const Array<Coordinate>& lowerLine, const Array<Coordinate>& vecPoints,
		Array<Triangle>& vecTriangles);

	//提取三角网的边界
    static Bool ExtractTINBondaries(const Array<Coordinate>& vecPoints, const Array<Triangle>& vecTriangles, Array<Array<Coordinate>>& edgepoints);

    //���ɵȸ���
	static Bool GenerateContourLine(Double contourValue, const Array<Triangle>& vecTriangles, Array<Array<Coordinate>>& ContourLinePoints);

public:
	//���ط�λ�ǣ��Ի���Ϊ��λ(0-2pi)���������ˮƽ,�򷵻�ֵС��0
	static Double GetSlope(const Coordinate& pnt0, const Coordinate& pnt1, const Coordinate& pnt2);

	//������ǣ��Ի���Ϊ��λ[0-pi/2].
	static Double GetAspect(const Coordinate& pnt0, const Coordinate& pnt1, const Coordinate& pnt2);

private:
    static Void SameEdge(Edge newEdge, Array<Edge>& arrEdge);

	static Bool ValidTriangle(const Triangle& triangle);

	//�ж��Ƿ��п��ܴ��ڵȸ���.
	static Bool AdjustZ(Triangle curTriangle, Double contourValue);

	//���ز�ֵ��.
	static Bool GetInsertPoint(const Coordinate& pnt0, const Coordinate& pnt1, Double contourValue, Coordinate& resPnt);

	//����ÿ���������еĵȸ���.
	static Bool ContourLineInTriangle(Triangle curTriangle, Double contourValue, LineTwoPts& resLine);

	//��ÿ���������еĵȸ��ߴ�����һ��
	static Bool AppendContourLines(Array<LineTwoPts>& arrContourLines, Double contourValue, Array<Array<Coordinate>>& ContourLinesPoints);

	//���ӱպϵȸ���
	static Bool JointClosedLine(const Array<Coordinate>& contourPoints, const Array<Int32>& arrMatchIndexs, MapIndex& arrNotDealIndexs, Array<Array<Coordinate>>& ContourLinesPoints);

	//���ӱպϵȸ���
	static Bool JointOpenLine(const Array<Coordinate>& contourPoints, const Array<Int32>& arrMatchIndexs, MapIndex& arrNotDealIndexs, MapIndex& mapNotMatchIndexs, Array<Array<Coordinate >> &ContourLinesPoints);
};

#endif
