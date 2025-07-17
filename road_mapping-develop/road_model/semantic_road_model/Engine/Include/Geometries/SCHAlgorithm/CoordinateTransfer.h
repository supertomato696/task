#ifndef _H_COORDINATE_TRANSFER
#define _H_COORDINATE_TRANSFER
#pragma once

class CoordinateTransfer
	{
	public:
		CoordinateTransfer(void);
		~CoordinateTransfer(void);
	public:
		//static double PI;
		static double Earth_R_VALUE;
	public:
		static double calcP2PDistance(double X1, double Y1, double X2, double Y2);
		static double calcAngel(double XS, double YS, double XM, double YM, double XE, double YE);
	};

#endif //_H_COORDINATE_TRANSFER