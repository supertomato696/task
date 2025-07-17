#include "cluster.h"
#include <queue>
#include <Eigen/Eigenvalues>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>

namespace Inference{

Eigen::Vector3f GetWLH(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    Eigen::Vector3f obLWH;
    obLWH.setZero();
    if (cloud->size() < 10) {
        return obLWH;
    }
    // 主成分分析（PCA）
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudProj(new pcl::PointCloud<pcl::PointXYZ>);
    pca.project(*cloud, *cloudProj); // 将点云投影到主成分特征空间

    // 计算投影后的点云的最小值和最大值
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*cloudProj, minPt, maxPt);

    // 将最小值和最大值点反算回原始空间
    pcl::PointXYZ reMinPt, reMaxPt;
    pca.reconstruct(minPt, reMinPt);
    pca.reconstruct(maxPt, reMaxPt);

    // 计算长宽高
    Eigen::Vector3f p1 = reMinPt.getVector3fMap();
    Eigen::Vector3f p2 = reMaxPt.getVector3fMap();
    obLWH = pca.getEigenVectors().transpose() * (p2 - p1);

    return obLWH;
}


void BFS(const cv::Mat &img, std::vector<std::vector<std::pair<int,int>>> &clusters, int typeId)
{
    std::vector<std::vector<bool>> visited(img.rows, std::vector<bool>(img.cols, false));
    for(int i=0; i<img.rows; i++){
        for(int j=0; j<img.cols; j++){
            if(img.at<uchar>(i,j)!=typeId || visited[i][j]){
                continue;
            }
            std::queue<std::pair<int,int>> q;
            q.push(std::make_pair(i,j));
            visited[i][j]= true;
            std::vector<std::pair<int,int>> cluster;
            while(!q.empty()){
                std::pair<int,int> pos = q.front();
                q.pop();
                cluster.push_back(pos);
                int rowIndex = pos.first, colIndex = pos.second;
                if(rowIndex>0 && img.at<uchar>(rowIndex-1, colIndex)==typeId &&
                                 visited[rowIndex-1][colIndex]==false){
                    visited[rowIndex-1][colIndex] = true;
                    q.push(std::make_pair(rowIndex-1, colIndex));
                }
                if(rowIndex+1<img.rows && img.at<uchar>(rowIndex+1, colIndex)==typeId &&
                                          visited[rowIndex+1][colIndex]==false){
                    visited[rowIndex+1][colIndex] = true;
                    q.push(std::make_pair(rowIndex+1, colIndex));                    
                }
                if(colIndex>0 &&img.at<uchar>(rowIndex, colIndex-1)==typeId &&
                                visited[rowIndex][colIndex-1]==false){
                    visited[rowIndex][colIndex-1] = true;
                    q.push(std::make_pair(rowIndex, colIndex-1));
                }
                if(colIndex+1<img.cols && img.at<uchar>(rowIndex, colIndex+1)==typeId &&
                                          visited[rowIndex][colIndex+1]==false){
                    visited[rowIndex][colIndex+1] = true;
                    q.push(std::make_pair(rowIndex, colIndex+1));                    
                }
            }
            clusters.push_back(cluster);
        }
    }

}


void DBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float radius, int minPts, 
                     std::vector<std::vector<int>> &clusters)
{
    if(cloud==NULL || cloud->size()==0){
        std::cout<<"input cloud is null at Inference::DBSCAN"<<std::endl;
        return;
    }
    //create kdtree for fast neighborhood search
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud);
    //neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);

    auto RangeQuery = [&](pcl::PointXYZ searchPoint){
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        kdtree.radiusSearch (searchPoint, radius, neighbors, pointRadiusSquaredDistance);
    };

    std::vector<int> label(cloud->size(), -1); //-1 undefined, 0 noise, 1-n class
    int clusterId = 0; 
    for(int i=0; i<cloud->size(); i++){
        if(label[i] >= 0){
            continue;
        }
        RangeQuery(cloud->at(i)); //contains point i itself
        if(neighbors.size() < minPts){
            label[i] = 0; //label as noise
            continue;
        }
        label[i] = ++clusterId;
        std::vector<bool> inQueue(cloud->size(), false);
        std::queue<int> q;
        for(auto id : neighbors){
            if(id != i && label[id] <= 0){
                q.push(id);
                inQueue[id] = true;
            }
        }
        while(!q.empty()){
            int currentId = q.front();
            q.pop();
            //Change Noise to border point
            if(label[currentId]==0) label[currentId] = clusterId;    
            //previously processed           
            if(label[currentId]>=0) continue;
            label[currentId] = clusterId;
            RangeQuery(cloud->at(currentId));
            if(neighbors.size() >= minPts){
                for(auto id : neighbors){
                    if(label[id] < 0 && !inQueue[id]){
                        q.push(id);
                        inQueue[id] = true;
                    }
                }
            }
        }
    }
    clusters.resize(clusterId);
    int noiseNum = 0;
    for(int i=0; i<label.size(); i++){
        if(label[i]>0){
            clusters[label[i]-1].push_back(i);
        }else{
            noiseNum += 1;
        }
    }
    std::cout<<"finish DBSCAN cluster with noise points number: "<<noiseNum<<std::endl;
}


pcl::PointCloud<pcl::PointXYZ>::Ptr SampleByGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float resolution)
{
    std::map<std::pair<int,int>, std::pair<Eigen::Vector3f, int>> grid;
    for(int i=0; i<cloud->size(); i++){
        Eigen::Vector3f pos = cloud->at(i).getVector3fMap();
        std::pair<int,int> index(pos(0)/resolution, pos(1)/resolution);
        if(grid.count(index)){
            grid[index].first += pos;
            grid[index].second +=1 ;
        }else{
            grid[index] = std::make_pair(pos, 1);
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>);
    for(auto iter=grid.begin(); iter!=grid.end(); iter++){
        Eigen::Vector3f pos = (iter->second).first/(iter->second).second;
        pcl::PointXYZ pt(pos(0), pos(1), pos(2));
        sampledCloud->push_back(pt);
    }      
    return sampledCloud;
}

void ExtractSegment(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::vector<Segment> &segments,
                    float radius, int minPts, int minAnchorNum)
{
    pcl::PointCloud<pcl::PointXY>::Ptr planeCloud(new pcl::PointCloud<pcl::PointXY>);
    for(int i=0; i<cloud->size(); i++){
        pcl::PointXY pt_plane(cloud->at(i).x, cloud->at(i).y);
        planeCloud->push_back(pt_plane);
    }  
    
    pcl::KdTreeFLANN<pcl::PointXY> kdtree;
    kdtree.setInputCloud(planeCloud);

    //neighbor vector
    std::vector<int> neighbors;
    std::vector<float> pointRadiusSquaredDistance;
    neighbors.reserve(100);
    pointRadiusSquaredDistance.reserve(100);
    
    auto NeighborLineFit = [&](int queryId, Eigen::Vector2f &direction){
        neighbors.clear();
        pointRadiusSquaredDistance.clear();
        pcl::PointXY searchPoint = planeCloud->at(queryId);
        kdtree.radiusSearch (searchPoint, radius, neighbors, pointRadiusSquaredDistance);
        //std::cout<<"NeighborLineFit, queryId: "<<queryId<<" neighbors size: "<<neighbors.size()<<std::endl;
        if(neighbors.size()<minPts){
            //std::cout<<"in NeighborLineFit, insufficient neighbor points"<<std::endl;
            return false; //not enough neighborhood points
        }
        Eigen::Vector2f centroid = Eigen::Vector2f::Zero();
        for(auto id : neighbors){
            centroid += Eigen::Vector2f(planeCloud->at(id).x, planeCloud->at(id).y);
        }
        centroid = centroid/neighbors.size();
        Eigen::Matrix2f cov = Eigen::Matrix2f::Zero();
        for(auto id : neighbors){
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            Eigen::Vector2f diff = pos - centroid;
            cov += diff*diff.transpose();
        }
        cov = cov/neighbors.size();
        cov = (cov + cov.transpose())/2;
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> es;
        es.compute(cov);
        Eigen::Vector2f eigenvalues = es.eigenvalues();
        // std::cout<<"eigenvalues值："<<eigenvalues(0)<<" , "<< eigenvalues(1) << std::endl;
        if(eigenvalues(1) < 25*eigenvalues(0)){
            //std::cout<<"in NeighborLineFit, fail PCA check"<<std::endl;
            return false;
        }
        Eigen::Matrix2f eigenvectors = es.eigenvectors();
        direction = eigenvectors.col(1);
        return true;
    };

    std::vector<std::pair<int, float>> disps;
    disps.reserve(100);
    Eigen::Vector2f direction;

    struct Anchor{
        int index; //index in cloud
        Eigen::Vector2f direction; //direction in xy plane
        Anchor(int index, Eigen::Vector2f direction):index(index), direction(direction){}
    };
    std::list<Anchor> anchors; //list of anchors for expansion on both sides
    std::list<int> indices; //list of cloud point index for expansion on both sides
    std::vector<bool> labeled(cloud->size(), false);

    for(int i=0; i<cloud->size(); i++){
        if(labeled[i]) continue;
        anchors.clear();
        indices.clear();
        int anchorIndex = i;
        bool bFitSuccess = NeighborLineFit(anchorIndex, direction); // 查找 radius 范围内的点，计算这些点云的朝向
        for(auto id : neighbors){
            labeled[id] = true; //label as searched
        }
        if(!bFitSuccess) continue;
        Eigen::Vector2f anchor(planeCloud->at(anchorIndex).x, planeCloud->at(anchorIndex).y);
        //calculate displacement in neighborhood
        disps.clear();
        for(auto id : neighbors){
            Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
            float disp = (pos - anchor).dot(direction);
            disps.push_back(std::make_pair(id, disp));
        }
        //sort points in direction
        std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a, 
                                                 const std::pair<int, float>&b){
                                                     return a.second < b.second;
        });
        //set the initial anchor
        //std::cout<<"set the initial anchor: "<<anchorIndex<<std::endl;
        anchors.push_back(Anchor(anchorIndex, direction));
        for(auto ele : disps){
            //std::cout<<"("<<ele.first<<", "<<ele.second<<") ";
            indices.push_back(ele.first);
        }
        //std::cout<<"initial segment indices size: "<<indices.size()<<std::endl;  
        bool expandToLeft = (indices.front()!=anchorIndex),
             expandToRight = (indices.back()!=anchorIndex);   
        while(expandToLeft || expandToRight){
            if(expandToLeft){
                Anchor leftMostNode = anchors.front(); // 最左侧的点 (第一次进来为 i=0 的点)
                Eigen::Vector2f leftMostDirection = leftMostNode.direction;
                int leftMostNewAnchorIndex = indices.front();
                Eigen::Vector2f leftMostNewAnchor(planeCloud->at(leftMostNewAnchorIndex).x, 
                                                  planeCloud->at(leftMostNewAnchorIndex).y);
                if(NeighborLineFit(leftMostNewAnchorIndex, direction)){
                    //adjust direction to coincide with leftMostDirection
                    if(direction.dot(leftMostDirection) < 0.f) direction = -direction;
                    disps.clear();
                    for(auto id : neighbors){
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos-leftMostNewAnchor).dot(direction);
                        //std::cout<<"id: "<<id<<" "<<labeled[id]<<" disp: "<<disp<<std::endl;
                        if(labeled[id] || disp > 0.f) continue;                        
                        disps.push_back(std::make_pair(id, disp));
                    }
                    //std::cout<<"expand to the left: "<<disps.size()<<std::endl;
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a, 
                                                             const std::pair<int, float>&b){
                                                        return a.second < b.second;
                    });
                    anchors.push_front(Anchor(leftMostNewAnchorIndex, direction)); // 比当前搜索的最左侧（leftMostNewAnchorIndex）还靠左，则继续添加到循环链表中
                    for(auto iter=disps.rbegin(); iter!=disps.rend(); iter++){
                        indices.push_front(iter->first);
                    }
                    expandToLeft = indices.front()!=leftMostNewAnchorIndex;
                }else{
                    expandToLeft = false;
                }
                for(auto id : neighbors){
                    labeled[id] = true; //label as searched
                }                
            }
            if(expandToRight){
                Anchor rightMostNode = anchors.back();
                Eigen::Vector2f rightMostDirection = rightMostNode.direction;
                int rightMostNewAnchorIndex = indices.back();
                Eigen::Vector2f rightMostNewAnchor(planeCloud->at(rightMostNewAnchorIndex).x, 
                                                   planeCloud->at(rightMostNewAnchorIndex).y);    
                if(NeighborLineFit(rightMostNewAnchorIndex, direction)){
                    //adjust direction to coincide with rightMostDirection
                    if(direction.dot(rightMostDirection) < 0.f) direction = -direction;
                    disps.clear();
                    for(auto id : neighbors){
                        Eigen::Vector2f pos(planeCloud->at(id).x, planeCloud->at(id).y);
                        float disp = (pos-rightMostNewAnchor).dot(direction);
                        if(labeled[id] || disp < 0.f) continue;
                        disps.push_back(std::make_pair(id, disp));
                    }
                    std::sort(disps.begin(), disps.end(), [](const std::pair<int, float>&a, 
                                                             const std::pair<int, float>&b){
                                                        return a.second < b.second;
                    });
                    anchors.push_back(Anchor(rightMostNewAnchorIndex, direction));
                    for(auto ele : disps){
                        indices.push_back(ele.first);
                    }
                    expandToRight = indices.back()!=rightMostNewAnchorIndex;
                }else{
                    expandToRight = false;
                }   
                for(auto id : neighbors){
                    labeled[id] = true; //label as searched
                }                       
            }
            //std::cout<<"after expand segment nodes size: "<<anchors.size()<<" indices size: "<<indices.size()<<std::endl;                          
        }
        // TODO:qzc change
        if(anchors.size() < minAnchorNum) continue; //too short segment
        Segment segment;
        segment.direction_rough = direction;
        auto iter = anchors.begin();
        int segmentCloudIndex = 0;
        for(auto id : indices){
            segment.segmentCloudIndexMap[segmentCloudIndex] = id;
            segment.cloud->push_back(cloud->at(id));
            if(id==iter->index){
                segment.anchorIndices.push_back(segmentCloudIndex);
                iter++;
            }
            segmentCloudIndex += 1;
        } 
        segments.push_back(segment);    
    }

}

} //namespace Inference