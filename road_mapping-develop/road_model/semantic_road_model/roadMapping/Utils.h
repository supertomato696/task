#pragma once

#include <iostream>
#include <sstream>
#include <vector>
#include <fstream>
#include "basicStruct.h"
#include <dirent.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/polygon.hpp>
namespace Utils{

using GisPt        = boost::geometry::model::point<double, 3, boost::geometry::cs::cartesian>;
using GisPolygon   = boost::geometry::model::polygon<GisPt>;
using GisPolyline  = boost::geometry::model::linestring<GisPt>;

// for string delimiter
void split(std::string s, std::string delimiter, std::vector<std::string> &vec);
//read KeyPose from file
void readKeyPose(std::string file, std::vector<KeyPose> &keyPoses);
//get files in directory
void getFiles(const std::string dirPath, std::vector<std::string> &files, std::string keyWord);

void readParse(const std::string dirPath, int &utm_num, Eigen::Vector3d &t_utm_world);

void readRefLines(const std::string dirPath, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud, Eigen::Vector3d init_llh);

void readRefLines_2(const std::string dirPath, pcl::PointCloud<pcl::PointXYZ>::Ptr &refLaneBoundaryCloud);

typedef bool (*pFunc)(std::string str);
void getFiles(const std::string dirPath, std::vector<std::string> &files, pFunc Func);
} //namespace Utils

