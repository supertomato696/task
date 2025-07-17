//#include "stdafx.h"
#include "Geometries/SCHAlgorithm/NodeInfo.h"

using namespace std;
//////////////////////////////////////////////////////////////////////////

NodeInfo::NodeInfo(void):
origin_X(0),
origin_Y(0),
wgs_Z(0),
seg_belong(0),
seg_seq(0),
longseg_flag(false),
single_flag(false),
reference_X(0),
reference_Y(0),
r_Value(0),
enlarge_num(0),
heading(0),
curvature(0),
slope(0),
bNegtive(false),
pSegment(NULL),
bVertex(false),
nVertexIndex(0)
	{
	}

NodeInfo::~NodeInfo(void)
	{
	}

double NodeInfo::getOrigin_X() {
	return origin_X;
	}

void NodeInfo::setOrigin_X(double origin_X1) {
	origin_X = origin_X1;
	}

double NodeInfo::getOrigin_Y() {
	return origin_Y;
	}

void NodeInfo::setOrigin_Y(double origin_Y1) {
	origin_Y = origin_Y1;
	}

double NodeInfo::getWgs_Z() {
	return wgs_Z;
	}

void NodeInfo::setWgs_Z(double wgs_Z1) {
	wgs_Z = wgs_Z1;
	}

long NodeInfo::getSeg_belong() {
	return seg_belong;
	}
void NodeInfo::setSeg_belong(long seg_belong1) {
	seg_belong = seg_belong1;
	}
long NodeInfo::getSeg_seq() {
	return seg_seq;
	}
void NodeInfo::setSeg_seq(long seg_seq1) {
	seg_seq = seg_seq1;
	}

int NodeInfo::getLongseg_flag() {
	return longseg_flag;
	}
void NodeInfo::setLongseg_flag(int longseg_flag1) {
	longseg_flag = longseg_flag;
	}

int NodeInfo::getSingle_flag() {
	return single_flag;
	}

void NodeInfo::setSingle_flag(int single_flag1) {
	single_flag = single_flag1;
	}

double NodeInfo::getReference_X() {
	return reference_X;
	}

void NodeInfo::setReference_X(double reference_X1) {
	reference_X = reference_X1;
	}

double NodeInfo::getReference_Y() {
	return reference_Y;
	}

void NodeInfo::setReference_Y(double reference_Y1) {
	reference_Y = reference_Y1;
	}

double NodeInfo::getR_Value() {
	return r_Value;
	}

void NodeInfo::setR_Value(double r_Value1) {
	r_Value = r_Value1;
	}	

int NodeInfo::getEnlarge_num() {
	return enlarge_num;
	}

void NodeInfo::setEnlarge_num(int enlarge_num1) {
	enlarge_num = enlarge_num1;
	}	

double NodeInfo::getHeading() {
	return heading;
	}

void NodeInfo::setHeading(double heading1) {
	heading = heading1;
	}

double NodeInfo::getCurvature() {
	return curvature;
	}

void NodeInfo::setCurvature(double curvature1) {
	curvature = curvature1;
	}

double NodeInfo::getSlope() {
	return slope;
	}

void NodeInfo::setSlope(double slope1) {
	slope = slope1;
	}

NodeInfo::NodeInfo(const NodeInfo& node)
{
	PostnID = node.PostnID;
	LinkID = node.LinkID;
	IDFlag = node.IDFlag;
	x = node.x;
	y = node.y;
	z = node.z;
	bNegtive = node.bNegtive;//是否为负值
	origin_X = node.origin_X;
	origin_Y = node.origin_Y;
	wgs_Z = node.wgs_Z;
	seg_belong = node.seg_belong;
	seg_seq = node.seg_seq;
	longseg_flag = node.longseg_flag;
	single_flag = node.single_flag;

	reference_X = node.reference_X;
	reference_Y = node.reference_Y;

	//映射直线段后的X坐标
	r_Value =node.r_Value;

	//最小二乘法向两端的扩点数
	enlarge_num = node.enlarge_num;

	//航向
	heading = node.heading;

	//曲率
	curvature = node.curvature;

	//纵坡
	slope = node.slope;

	pSegment = node.pSegment;

	bNegtive = node.bNegtive;

	bVertex = node.bVertex;

	nVertexIndex = node.nVertexIndex;
}

string NodeInfo::getLinkId() const
{
	return  LinkID.substr(1, LinkID.size() - 2);
}