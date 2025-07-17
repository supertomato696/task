#include "convexHull.h"
#include <limits>
#include <list>
#include <iostream>

namespace CH{

float signedAreaFunc(Eigen::Vector2f pa, Eigen::Vector2f pb, Eigen::Vector2f pt)
{
    return (pb(0)-pa(0))*(pt(1)-pa(1)) - (pb(1)-pa(1))*(pt(0)-pa(0));
}

struct Node{
    Node(){}
    Node(int index_0, int index_1):index_0(index_0), index_1(index_1){}

    int index_0;
    int index_1;
    std::vector<int> indices;
};

void QuickHull(const std::vector<Eigen::Vector2f> &pts, std::vector<int> &nextVertex)
{
    //consider special case
    if(pts.empty()){
        return;
    }
    std::vector<std::pair<int, Eigen::Vector2f>> orderedPts(pts.size());
    for(int i=0; i<pts.size(); i++){
        orderedPts[i] = std::make_pair(i, pts[i]);
    }
    std::sort(orderedPts.begin(), orderedPts.end(), [](const std::pair<int, Eigen::Vector2f> &a, 
                                                       const std::pair<int, Eigen::Vector2f> &b){
            if(a.second(0) == b.second(0)){
                return a.second(1) < b.second(1);
            }else{
                return a.second(0) < b.second(0);
            }
    });
    int paIndex = orderedPts.front().first, pbIndex = orderedPts.back().first;
    if(paIndex == pbIndex){
        nextVertex.push_back(paIndex);
        nextVertex.push_back(pbIndex);
        return;
    }
    std::list<Node> lNodes;
    Node pab(paIndex, pbIndex);
    Node pba(pbIndex, paIndex);
    for(int i=0; i<pts.size(); i++){
        float val = signedAreaFunc(pts[paIndex], pts[pbIndex], pts[i]);
        if(val > 0.f){
            pab.indices.push_back(i);
        }else if(val < 0.f){
            pba.indices.push_back(i);
        }
    }
    lNodes.push_back(pab);
    lNodes.push_back(pba);
    auto iter = lNodes.begin();
    while(iter!=lNodes.end()){
        if(iter->indices.empty()){
            iter++;
        }else{
            int paIndex = iter->index_0, pbIndex = iter->index_1;
            std::vector<int> &indices = iter->indices;
            int maxIndex = -1;
            float maxVal = std::numeric_limits<float>::lowest();
            for(auto index : indices){
                float val = signedAreaFunc(pts[paIndex], pts[pbIndex], pts[index]);
                if(val > maxVal){
                    maxVal = val;
                    maxIndex = index;
                }
            }
            auto niter = lNodes.insert(iter, Node(maxIndex, pbIndex));
            for(auto index : indices){
                float val = signedAreaFunc(pts[maxIndex], pts[pbIndex], pts[index]);
                if(val > 0.f){
                    niter->indices.push_back(index);
                }
            }
            niter = lNodes.insert(niter, Node(paIndex, maxIndex));
            for(auto index : indices){
                float val = signedAreaFunc(pts[paIndex], pts[maxIndex], pts[index]);
                if(val > 0.f){
                    niter->indices.push_back(index);
                }
            }  
            lNodes.erase(iter);
            iter = niter;         
        }
    }
    for(iter = lNodes.begin(); iter!=lNodes.end(); iter++){
        nextVertex.push_back(iter->index_0);
    }
    nextVertex.push_back(lNodes.begin()->index_0);

}

void Polyhedron2d::calculateArea()
{
    if(vertices.size()<=2){
        return;
    }
    Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
    for(auto vertex : vertices){
        centroid += vertex;
    }
    centroid = centroid/vertices.size();
    area = 0.f;
    for(int i=0; i<vertices.size()-1; i++){
        area += std::fabs(signedAreaFunc(vertices[i], vertices[i+1], centroid));
    }
    area = area/2;
}

} //namespace CH