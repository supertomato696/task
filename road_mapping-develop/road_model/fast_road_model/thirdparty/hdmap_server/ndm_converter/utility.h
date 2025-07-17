/**
 * @file utility.h
 * @brief utilities in geometry module, including various helper functions.
 *
 *  Created on: 2009-4-19
 *      Author: Jim Z. Shi
 *  Memo: 所有坐标系转换，都是基于bd09坐标进行，其中墨卡托投影都是其于bd09后的投影坐标
 */

#ifndef UTILITY_H_
#define UTILITY_H_

namespace nuts {

	/**
	 * @class dpoint_t
	 * @brief double point type.
	 */
	struct dpoint_t {
		double x;
		double y;
		dpoint_t() :
			x(0), y(0) {
		}
		dpoint_t(double xx, double yy) :
			x(xx), y(yy) {
		}
		dpoint_t(const dpoint_t& rh) :
			x(rh.x), y(rh.y) {
		}
		bool operator<(dpoint_t const &rhs) const
		{
			return x < rhs.x || (x == rhs.x && y < rhs.y);
		}
	};

	/**
	 * @brief gcj to baidu encrypt.
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int gcj2bd(const dpoint_t& pt, dpoint_t* res);
	/**
	 * @brief baidu to gcj decrypt.
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int bd2gcj(const dpoint_t& pt, dpoint_t* res);

	/**
	 * @brief mc to baidu decrypt.
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int mc2bd(const dpoint_t& pt, dpoint_t* res);

	/**
	 * @brief baidu to mc decrypt.
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int bd2mc(const dpoint_t& pt, dpoint_t* res);

	/**
	 * @brief transform coordinates from gcj system to mc system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int gcj2mc(const dpoint_t& pt, dpoint_t* res);

	/**
	 * @brief transform coordinates from mc system to gcj system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int mc2gcj(const dpoint_t& pt, dpoint_t* res);

	/**
	 * @brief transform coordinates from wgs84 system to gcj system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int wgs2gcj( const dpoint_t& pt, dpoint_t* res ) ;

	/**
	 * @brief transform coordinates from wgs84 system to ll system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int wgs2bd( const dpoint_t& pt, dpoint_t* res ) ;

	/**
	 * @brief transform coordinates from wgs84 system to mc system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int wgs2mc( const dpoint_t& pt, dpoint_t* res ) ;


	/**
	 * @brief transform coordinates from wgs84 system to gcj system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int gcj2wgs( const dpoint_t& pt, dpoint_t* res ) ;

	/**
	 * @brief transform coordinates from wgs84 system to ll system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int bd2wgs( const dpoint_t& pt, dpoint_t* res ) ;

	/**
	 * @brief transform coordinates from wgs84 system to mc system;
	 * @param pt point.
	 * @param res return point ptr.
	 * @return
	 * 		- 0		-	ok.
	 * 		- <0	-	failed.
	 */
	extern int mc2wgs( const dpoint_t& pt, dpoint_t* res ) ;
} ;

#endif /* UTILITY_H_ */
