#pragma once
#include <vector>
#include <Eigen/Dense>

namespace CH{

struct Polyhedron2d{
    std::vector<Eigen::Vector2f> vertices; //in clockwise order
    float area = 0.f;

    void calculateArea();
};



float signedDistanceFunc(Eigen::Vector2f pa, Eigen::Vector2f pb, Eigen::Vector2f pt);

void QuickHull(const std::vector<Eigen::Vector2f> &pts, std::vector<int> &nextVertex);

} //namespace CH