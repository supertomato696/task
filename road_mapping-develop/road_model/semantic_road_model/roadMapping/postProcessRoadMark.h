#pragma once
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pclPtType.h"
#include "Geometries/Envelope.h"
#include "Geometries/Polygon.h"
#include "Vec.h"
#include <pcl/surface/convex_hull.h>

#include <pcl/sample_consensus/method_types.h>//随机参数估计方法
#include <pcl/sample_consensus/model_types.h>//模型定义
#include <pcl/segmentation/sac_segmentation.h>//基于采样一致性分割的类的头文件
#include <pcl/filters/voxel_grid.h>//基于体素网格化的滤波
#include <pcl/filters/extract_indices.h>//索引提取
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/registration/icp.h>


namespace RoadMapping{

class PostProcessRoadMark{
    struct RoadMarkGeo
    {
        Engine::Geometries::Envelope enve; //外包框2d
        vector<Eigen::Vector3f> plypts;
        bool flag; //是否处理
        int index;//原始index
//        double area;//面积
    };

     struct BevPolygon{
            bool isSave; //是否保留
            int index; //在所有聚类中的index
            Engine::Base::Array<Engine::Geometries::Coordinate> polygon_pts; //bev结果框
            // Engine::Geometries::Polygon polygon_geo;
            Engine::Base::Array<Engine::Geometries::Coordinate> in_pts; //bev结果框
            Engine::Geometries::Envelope poly_enve; //外包框
            Engine::Geometries::Coordinate middlept;
            Engine::Geometries::Coordinate fit_s;
            Engine::Geometries::Coordinate fit_e;
            Engine::Base::Array<Engine::Geometries::Coordinate> polygon_box;
            int num_crosswalk_pts_in;
            int subtype;
            double length;
            double width;
            double area;
            double score;
        };

    struct ArrowFeature {
        Eigen::Vector3f center;
        Eigen::Vector3f direction;
        int original_id;
        std::vector<Eigen::Vector3f> points;
        ArrowFeature(Eigen::Vector3f c, Eigen::Vector3f d, int id, const std::vector<Eigen::Vector3f>& pts)
            : center(std::move(c)), direction(std::move(d)), original_id(id), points(pts) {}
    };

    struct NearbyLinesResult {
        std::vector<int> line_ids;  // 符合条件的线段ID
        std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> line_points;  // 各线段在阈值内的点云
    };

    struct DirectionFilterResult {
        std::vector<int> filtered_lines;  // 过滤后的线段ID
        Eigen::Vector3f main_direction;   // 最终主方向（单位向量）
    };

public:
    void run(std::string dataDir, std::string outputDir);
    void run_yxx(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir);
    void run_yxx_2(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir);
    void run_yxx_fin_in(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir); 
    void run_yxx_fin_add_type(std::string dataDir,std::string pcdPath,std::string refFile,std::string outputDir);
    void run_yxx_bev_road(std::string dataDir,std::string fsd_mapbuildout,std::string pcdPath,std::string refFile,std::string outputDir);
    //不同定位要素通用函数 minPts cluster最小点数 minX包围框最小长度 maxY包围框最大宽度
    void road_mark_base(const pcl::PointCloud<pcl::PointXYZ>::Ptr &roadArrowCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY);
    void road_mark_add_type(const pcl::PointCloud<MyColorPointType>::Ptr &pcPtr, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY);    
    int getClusterBevShape(const std::vector<int>& cluster, const std::map<int, int>& indexMap, const pcl::PointCloud<ConRmPointType>::Ptr& cloudfilter);  
    void mix_diff_roadmark(std::string outputDir);
    void roadmark_json2RoadMarkGeo(std::string outputDir, std::string lable, std::vector<RoadMarkGeo>& resRoadMarkGeo);
    void roadmark_RoadMarkGeo2json(std::string outputDir, std::string lable, std::vector<RoadMarkGeo>& resRoadMarkGeo);
    void recalroadmark(const pcl::PointCloud<pcl::PointXYZI>::Ptr &Cloud100, const pcl::PointCloud<pcl::PointXYZI>::Ptr &Cloud110, int index_100, vector<int> mix110, std::vector<RoadMarkGeo>& resRoadMarkGeo_100, std::vector<RoadMarkGeo>& resRoadMarkGeo_110);
    
    double interior_ratio(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_src, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudPtr_ref,double max_range);
    void wq_WriteToOBJ(Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes, std::string strTilePath, std::string name);
    void wq_adjust_center(Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes, Eigen::Vector3f centroid, Engine::Base::Array<Engine::Geometries::Coordinate> &vecLanes_out);
    void wq_getConcaveHull(Engine::Base::Array<Engine::Geometries::Coordinate>& clusterpts, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull);
    void wq_fix_edge_icp(pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud ,pcl::PointCloud<pcl::PointXYZ>::Ptr &sampleCloud);
    void wq_road_mark_base(const pcl::PointCloud<pcl::PointXYZ>::Ptr &roadArrowCloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, std::string outputDir, int label, int minPts, double minX, double maxY, double minY);
    bool getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, std::vector<int>& ori_pts_index, double interval, int save_ori_pts);
    void rotate_yaw(Engine::Base::Array<Engine::Geometries::Coordinate> &input, Engine::Base::Array<Engine::Geometries::Coordinate> &output, double yaw);
    void wq_icp_type(  std::string outputDir,Eigen::Vector3f direction,Eigen::Vector3f centroid,pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud,Engine::Base::Array<Engine::Geometries::Coordinate> &arrow_edgePts, std::string arrow_type,std::map<std::string, double> &arrow_icpPts,int flag);
    double calu_yaw(Engine::Geometries::Coordinate& v1, Engine::Geometries::Coordinate& v2) ;
    double calu_yaw(Eigen::Vector3f& v1, Eigen::Vector3f& v2) ;
    void generateOutputCloud(int index, float yaw, int arrow_type, pcl::PointCloud<PointElement>::Ptr &outputCloud, std::vector<Eigen::Vector3f> &FourPointMap, float score, int ele_type);
    NearbyLinesResult findNearbyLinesWithMinPoints(const pcl::PointXYZ& target_point, double threshold,
        const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache, int min_points = 2);  
    NearbyLinesResult findLinesWithDynamicThreshold(const pcl::PointXYZ& target_point, double initial_threshold,
        double max_threshold, const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache);
    DirectionFilterResult filterLinesByDirection(const NearbyLinesResult& candidates, float angle_threshold_deg = 30.0f);
    
    void afterProcess(const std::map<int, std::vector<Eigen::Vector3f>>& cornerPointMap, const pcl::PointCloud<pcl::PointXYZI>::Ptr& refLaneBoundaryCloud, std::map<int, std::vector<Eigen::Vector3f>>& newCornerPointMap);

    ////////////////////////////////////采用bev结果//////////////////////////////////////////////////////////
    void run_bev_mark(std::string dataDir,std::string pcdPath,std::string obj_dir,std::string outputDir, pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud);
    void pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate>& pts, Engine::Geometries::Polygon& resPolygon);
    void cal_bev_polygon_core(Engine::Base::Array<BevPolygon>& bev_crosswalks, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_hull, pcl::PointCloud<pcl::PointXYZI>::Ptr &refLaneBoundaryCloud, double toler=0.0);//低于toler会被删除
    void ReadObjFromPcd(std::string bev_lukou_pcd, Engine::Base::Array<BevPolygon> &arrArrPts);
    void get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate>& pts, Engine::Geometries::Envelope& envelope);
    void filter_bev_polygons();
    void bev_lukou_obj(const std::string bev_obj_floder, const std::string out_obj_floder,Engine::Base::Array<BevPolygon>& bev_crosswalks);
    void GetAveragePt(const Engine::Base::Array<Engine::Geometries::Coordinate>& arrCloudPts, Engine::Geometries::Coordinate& midpt);
    void polygon_to_pcd(std::string outputDir);

    private:
        std::string _output;
        Engine::Base::Array<BevPolygon> _bev_lukou_bds_polys;
        float _VoxelGrid_size; //体素滤波值
};

} //namespace RoadMapping