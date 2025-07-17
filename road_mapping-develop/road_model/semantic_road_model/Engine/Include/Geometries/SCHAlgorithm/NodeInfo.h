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

		//ԭʼ��x����
		std::string x;
		//ԭʼ��t����
		std::string y;
		//ԭʼ��z����
		std::string z;

		bool bNegtive;//�Ƿ�Ϊ��ֵ

		FuzzySegment* pSegment; //�ֶ���Ϣ

		bool bVertex; //�Ƿ��ǽڵ���Ϣ

		int nVertexIndex; //����ǽڵ㣬�ǵڼ����ڵ�

	private:
		//����תƽ������ϵ���X����
		double origin_X;

		//����תƽ������ϵ���Y����
		double origin_Y;

		//WGS84��Z����
		double wgs_Z;

		//�ֶκ����ڵڼ���
		long seg_belong;

		//�ڼ��εĵڼ�����
		long seg_seq;

		//�Ƿ�Ϊ���߶��ϵ��м�㣨�����ĵ���Բ�����С���˷��������¶ȣ�
		int longseg_flag;

		//�Ƿ�Ϊ�����
		int single_flag;

		//ӳ��ֱ�߶κ��X����
		double reference_X;

		//ӳ��ֱ�߶κ��Y����
		double reference_Y;

		//���ʰ뾶
		double r_Value;

		//��С���˷������˵�������
		int enlarge_num;

		//����
		double heading;

		//����
		double curvature;
	
		//����
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
