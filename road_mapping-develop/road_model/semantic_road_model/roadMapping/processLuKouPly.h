//
//
//

#ifndef ROADMAPPING_PROCESSLUKOUPLY_H
#define ROADMAPPING_PROCESSLUKOUPLY_H
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include "pclPtType.h"
#include "Geometries/Envelope.h"
#include "Geometries/Polygon.h"

namespace RoadMapping
{

    class processLuKouPly
    {

        struct BevPolygon
        {
            bool isSave;                                                     // 是否保留
            int index;                                                       // 在所有聚类中的index
            Engine::Base::Array<Engine::Geometries::Coordinate> polygon_pts; // bev结果框
            // Engine::Geometries::Polygon polygon_geo;
            Engine::Base::Array<Engine::Geometries::Coordinate> in_pts; // bev结果框
            Engine::Geometries::Envelope poly_enve;                     // 外包框
            Engine::Geometries::Coordinate middlept;
            Engine::Geometries::Coordinate fit_s;
            Engine::Geometries::Coordinate fit_e;
            Engine::Base::Array<Engine::Geometries::Coordinate> polygon_box;
            int num_crosswalk_pts_in;
            double length;
            double width;
            double area;
            double score;
        };

    public:
        static void run_yxx_fin_in(std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir);
        static void ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts); // 将obj结果读入
        static void GetAvePnts(Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts, Engine::Base::Array<Engine::Geometries::Coordinate> &avePnts);
        static void GetPlyOffset(Engine::Base::Array<Engine::Geometries::Coordinate> &oriPlyPts, Engine::Base::Array<Engine::Geometries::Coordinate> &offsetPlyPts, double offsetDis); // 将多边形进行外扩
        static void RemovePntsByAngle(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, double angle);
        static void RemovePntsByDis(Engine::Base::Array<Engine::Geometries::Coordinate> &arrPts, double minDis);

        ////////////////////////////////////采用bev结果//////////////////////////////////////////////////////////
        void run_bev_lukouboundary(std::string dataDir, std::string pcdPath, std::string obj_dir, std::string outputDir);
        void pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &pts, Engine::Geometries::Polygon &resPolygon);
        void cal_bev_polygon_core(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull, double toler = 0.0); // 低于toler会被删除
        void ReadObjFromPcd(std::string bev_lukou_pcd, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts);
        void get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate> &pts, Engine::Geometries::Envelope &envelope);
        void filter_bev_polygons();
        void bev_lukou_obj(const std::string bev_obj_floder, const std::string out_obj_floder, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks);
        ////////////////////////////////////采用cloud_bev结果//////////////////////////////////////////////////////////
        void run_cloud_bev_lukoubd(std::string pcdPath, std::string outputDir);
        void get_lukou_by_cloudedge(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts);
        void get_lukou_by_opencvcontours(float voxelGrid_size, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_cluster, Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts, int index, std::string outputDir);

        void anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &edge_pts); // 将多边形转换为逆时针
        ////////////////////////////////////根据停止线和人行横道自己生成路口框//////////////////////////////////////////////////////////
        float pointToCenterDis(float x1, float y1, float x2, float y2);
        void generate_lukou_boundary_manual(std::string middle_json_path, std::string global_pcd_path, std::string outputDir);


    private:
        std::string _output;
        Engine::Base::Array<BevPolygon> _bev_lukou_bds_polys;
        float _VoxelGrid_size; // 体素滤波值
    };
} // namespace RoadMapping
#endif // ROADMAPPING_PROCESSLUKOUPLY_H
