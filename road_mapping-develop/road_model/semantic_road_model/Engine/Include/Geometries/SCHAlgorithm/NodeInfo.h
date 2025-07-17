#pragma once

#include <string>

class FuzzySegment;
class NodeInfo
	{
		friend class XCalculateKPI;
	public:
		NodeInfo(void);
		NodeInfo(const NodeInfo& node);
		~NodeInfo(void);
	public:
		std::string PostnID;
		std::string LinkID;
		std::string IDFlag;

		//原始的x坐标
		std::string x;
		//原始的t坐标
		std::string y;
		//原始的z坐标
		std::string z;

		bool bNegtive;//是否为负值

		FuzzySegment* pSegment; //分段信息

		bool bVertex; //是否是节点信息

		int nVertexIndex; //如果是节点，是第几个节点

	private:
		//球面转平面坐标系后的X坐标
		double origin_X;

		//球面转平面坐标系后的Y坐标
		double origin_Y;

		//WGS84的Z坐标
		double wgs_Z;

		//分段后属于第几段
		long seg_belong;

		//第几段的第几个点
		long seg_seq;

		//是否为长线段上的中间点（这样的点可以不用最小二乘法算曲率坡度）
		int longseg_flag;

		//是否为单噪点
		int single_flag;

		//映射直线段后的X坐标
		double reference_X;

		//映射直线段后的Y坐标
		double reference_Y;

		//曲率半径
		double r_Value;

		//最小二乘法向两端的扩点数
		int enlarge_num;

		//航向
		double heading;

		//曲率
		double curvature;
	
		//纵坡
		double slope;
	public:
		std::string getLinkId() const;

		double getOrigin_X();
		void setOrigin_X(double origin_X1);
		double getOrigin_Y();
		void setOrigin_Y(double origin_Y1);
		double getWgs_Z();
		void setWgs_Z(double wgs_Z1);
		long getSeg_belong();
		void setSeg_belong(long seg_belong1);
		long getSeg_seq();
		void setSeg_seq(long seg_seq1);
		int getLongseg_flag();
		void setLongseg_flag(int longseg_flag1);
		int getSingle_flag();
		void setSingle_flag(int single_flag1);
		double getReference_X();
		void setReference_X(double reference_X1);
		double getReference_Y();
		void setReference_Y(double reference_Y1);
		double getR_Value();
		void setR_Value(double r_Value1);
		int getEnlarge_num();
		void setEnlarge_num(int enlarge_num);
		double getHeading();
		void setHeading(double heading1);
		double getCurvature();
		void setCurvature(double curvature1);
		double getSlope();
		void setSlope(double slope1);
	};
