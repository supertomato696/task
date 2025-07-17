#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include "cluster.h"

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    std::string file = "/home/shenxuefeng/Documents/roadMapping/all.pcd";
    pcl::io::loadPCDFile<pcl::PointXYZ>(file, *cloud);
    //sample cluster
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.1f, 0.1f, 0.1f);
    std::cout<<"before voxel filter, cloud size: "<<cloud->size()<<std::endl;
    sor.filter(*cloud);
    std::cout<<"after voxel filter, cloud size: "<<cloud->size()<<std::endl;
    std::vector<std::vector<int>> clusters;
    float radius = 0.2f;
    int minPts = 3;
    Inference::DBSCAN(cloud, radius, minPts, clusters);

    for(int i=0; i<clusters.size(); i++){
        std::cout<<i<<" th cluster size: "<<clusters[i].size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud, clusters[i], *clusterCloud);

        float resolution = 0.2f;
        clusterCloud = Inference::SampleByGrid(clusterCloud, resolution);
        std::cout<<"after sample by grid, clusterCloud size: "<<clusterCloud->size()<<std::endl;
        std::vector<Inference::Segment> segments;
        Inference::ExtractSegment(clusterCloud, segments);
        for(int j=0; j<segments.size(); j++){
            pcl::io::savePCDFileBinary(std::to_string(i)+"_"+std::to_string(j)+"_segmentCloud.pcd", *segments[j].cloud);
            pcl::PointCloud<pcl::PointXYZ>::Ptr anchorCloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*segments[j].cloud, segments[j].anchorIndices, *anchorCloud);
            pcl::io::savePCDFileASCII(std::to_string(i)+"_"+std::to_string(j)+"_anchorCloud.pcd", *anchorCloud);
            segments[j].QpSpline();
            pcl::PointCloud<pcl::PointXYZ>::Ptr trjCloud = segments[j].smoothTrjCloud();
            pcl::io::savePCDFileBinary(std::to_string(i)+"_"+std::to_string(j)+"_trjCloud.pcd", *trjCloud);
        }
    }    

    std::cout<<"successfully run cluster program"<<std::endl;
}