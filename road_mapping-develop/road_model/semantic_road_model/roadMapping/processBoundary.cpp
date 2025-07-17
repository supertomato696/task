#include "processBoundary.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace RoadMapping{

void ProcessBoundary::run(Eigen::Vector3d init_llh,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                          pcl::PointCloud<pcl::PointXYZ>::Ptr keyPoseCloud,
                          pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_keyPose,
                          std::string outputDir)
{
    //sample cloud
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    std::cout<<"before voxel filter cloud size: "<<cloud->size()<<std::endl;
    sor.filter(*cloud);
    std::cout<<"after voxel filter cloud size: "<<cloud->size()<<std::endl;
    if(cloud!=NULL && cloud->size() > 0){
        pcl::io::savePCDFileBinary(outputDir+"/totalCloud.pcd", *cloud);
    }    

    //cluster cloud
    std::vector<std::vector<int>> clusters;
    float radius = 0.3f;
    int minPts = 3;
    Inference::DBSCAN(cloud, radius, minPts, clusters);

    std::vector<Inference::Segment> segments;
    //generate segment
    for(int i=0; i<clusters.size(); i++){
        std::cout<<i<<" th cluster size: "<<clusters[i].size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);

        float resolution = 0.2f;
        clusterCloud = Inference::SampleByGrid(clusterCloud, resolution);
        std::cout<<"after sample by grid, clusterCloud size: "<<clusterCloud->size()<<std::endl; 
        int preSegmentsSize = segments.size();       
        Inference::ExtractSegment(clusterCloud, segments);
        int currentSegmentsSize = segments.size();
        std::cout<<i<<" th cluster has "<<currentSegmentsSize-preSegmentsSize<<" segments"<<std::endl;
    }
    //arrange lane boundary order
    
    if(segments.empty()){
        return;
    }
    
    for(int i=0; i<segments.size(); i++){
        if(segments[i].anchorIndices.size() > 10){
            segments[i].selectAnchor(5);
        }
        std::cout<<i<<" th segment anchorIndices size: "<<segments[i].anchorIndices.size()<<std::endl;
        //arrange direction to coincide with key pose trajectory
        segments[i].DecideOrientation(keyPoseCloud, kdtree_keyPose);        
    }

    int referenceIndex = 0, referenceAnchorNum = segments[0].anchorIndices.size();
    for(int i=1; i<segments.size(); i++){
        if(segments[i].anchorIndices.size() > referenceAnchorNum){
            referenceAnchorNum = segments[i].anchorIndices.size();
            referenceIndex = i;
        }
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr refAnchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*segments[referenceIndex].cloud, segments[referenceIndex].anchorIndices, *refAnchorCloud);
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree_refAnchorCloud(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    kdtree_refAnchorCloud->setInputCloud(refAnchorCloud); // 设置要搜索的点云，建立KDTree   
    std::vector<std::pair<int, float>> disp(segments.size());
    for(int i=0; i<segments.size(); i++){
        disp[i] = std::make_pair(i, 10.f);
    }
    for(int i=0; i<segments.size(); i++){
        int ncount = 0;
        float accuDisp = 0.f;
        float radius = 20.f;
        for(auto anchorIndex : segments[i].anchorIndices){
            std::vector<int> neighbors;
            std::vector<float> pointRadiusSquaredDistance;
            kdtree_refAnchorCloud->radiusSearch(segments[i].cloud->at(anchorIndex), radius, neighbors, pointRadiusSquaredDistance);
            if(neighbors.empty()){
                continue;
            }
            int neighborIndex = neighbors.front();
            if(neighborIndex<1){
                continue;
            }
            Eigen::Vector3f direction = refAnchorCloud->at(neighborIndex).getVector3fMap() - 
                                            refAnchorCloud->at(neighborIndex-1).getVector3fMap();
            Eigen::Vector2f nx = direction.head(2).normalized();
            Eigen::Vector2f ny(-nx(1), nx(0));
            accuDisp += (segments[i].cloud->at(anchorIndex).getVector3fMap() - 
                            refAnchorCloud->at(neighborIndex).getVector3fMap()).head(2).dot(ny);
            ncount++;
        }
        if(ncount > 0){
            disp[i].second = accuDisp/ncount;
        }      
        std::cout<<i<<" th segment, disp: "<<disp[i].second<<std::endl;  
    }
    std::sort(disp.begin(), disp.end(), [](std::pair<int, float> a, std::pair<int, float> b){
        return a.second > b.second;
    });    


    nlohmann::json obj;
    obj["base"]["lat"] = init_llh(0);
    obj["base"]["lon"] = init_llh(1);
    obj["base"]["alt"] = init_llh(2);
    for(int i=0; i<segments.size(); i++){
        int index = disp[i].first;
        if(segments[index].cloud !=NULL && segments[index].cloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_segmentCloud.pcd", *segments[index].cloud);
        }        

        pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);       
        pcl::copyPointCloud(*segments[index].cloud, segments[index].anchorIndices, *anchorCloud);
        if(anchorCloud!=NULL && anchorCloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_anchorCloud.pcd", *anchorCloud);
        }
        

        segments[index].QpSpline();
        pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[index].smoothTrjCloud();
        if(trjCloud!=NULL && trjCloud->size()>0){
            pcl::io::savePCDFileBinary(outputDir+"/"+std::to_string(i)+"_trjCloud.pcd", *trjCloud);
        }       

        segments[index].addSegmentCoeff(obj, i);
        //segments[index].addSegmentCoeff_straightLine(obj, i);
    }
    
    std::ofstream fid_laneBoundary(outputDir+"/boundary.json");
    //fid_laneBoundary << std::setw(4) << obj << std::endl;
    fid_laneBoundary<< obj << std::endl;   

}

} //RoadMapping