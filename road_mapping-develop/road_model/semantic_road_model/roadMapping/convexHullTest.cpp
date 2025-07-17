#include <iostream>
#include "convexHull.h"

int main()
{
    std::vector<Eigen::Vector2f> pts;
    std::vector<int> nextVertex;
    for(int i=0; i<10; i++){
        pts.push_back(Eigen::Vector2f(i,0));
    }
    CH::QuickHull(pts, nextVertex);
    for(auto index : nextVertex){
        std::cout<<index<<" ";
    }

    std::cout<<"successfully run convexHull Test"<<std::endl;
}