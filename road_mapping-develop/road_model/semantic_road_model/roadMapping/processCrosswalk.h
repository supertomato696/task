//
//
//

#ifndef ROADMAPPING_PROCESSCROSSWALK_H
#define ROADMAPPING_PROCESSCROSSWALK_H

#include <string>
#include <algorithm>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "pclPtType.h"
#include <pcl/kdtree/kdtree_flann.h>
#include "Geometries/Envelope.h"
#include "Geometries/Polygon.h"

#include <ceres/ceres.h>

namespace RoadMapping
{
    struct MinAreaRect {
        std::array<Eigen::Vector2f, 4> vertices;
        float area;
    };

    // 优化参数结构体（中心x,y + 旋转角度theta + 半长a + 半宽b）
    struct BoxParams {
        double data[5]; // [x, y, theta, a, b]
    };

    // 自定义损失函数：面积相似性 + 点包容性
    class BoxCostFunction {
    public:
        BoxCostFunction(const Eigen::Vector2d& pt, double polyArea, double lambda)
            : pt_(pt), polyArea_(polyArea), lambda_(lambda) {}

        template <typename T>
        bool operator()(const T* const params, T* residual) const {
            // 解析参数（新参数顺序 [x, y, theta, b, delta]）
            const T x = params[0];
            const T y = params[1];
            const T theta = params[2];
            const T b = params[3];
            const T delta = params[4];
            const T a = b + delta; // 关键修改：a = b + delta

            // 坐标变换到矩形局部坐标系（保持不变）
            T dx = pt_.x() - x;
            T dy = pt_.y() - y;
            T rotX = cos(theta) * dx + sin(theta) * dy;
            T rotY = -sin(theta) * dx + cos(theta) * dy;

            // 计算面积差异项（使用新的a和b）
            T rectArea = T(4.0) * a * b; // 注意是a*b，不是(b+delta)*b
            T areaDiff = (rectArea - T(polyArea_)) / T(polyArea_);

            // 计算点包容项（使用新的a和b）
            T distX = ceres::abs(rotX) - a;
            T distY = ceres::abs(rotY) - b;
            T pointLoss = std::max(T(0.0), std::max(distX, distY));

            // 综合损失
            residual[0] = T(lambda_) * pointLoss + T(1.0 - lambda_) * areaDiff;
            return true;
        }

    private:
        Eigen::Vector2d pt_;
        double polyArea_;
        double lambda_;
    };
    class processCrosswalk
    {
        struct CrosswalkGeo
        {
            Engine::Geometries::Envelope enve; // 外包框2d
            vector<Eigen::Vector3f> plypts;
            bool flag; // 是否处理
            int index; // 原始index
            //        double area;//面积
        };

        // 起始点组成的线段
        struct Segment_Two_Points
        {
            //            Segment_Two_Points(Engine::Geometries::Coordinate start_pt,Engine::Geometries::Coordinate end_pt) {
            //                this->start_pt=start_pt;
            //                this->end_pt=end_pt;
            //            };
            Engine::Geometries::Coordinate start_pt;
            Engine::Geometries::Coordinate end_pt;
        };

        // 停止线和路口的关联关系
        struct Stopline_Info
        {
            int stopline_index; // 停止线在stopline中的index;
            Engine::Geometries::Coordinate start_pt;
            Engine::Geometries::Coordinate end_pt;
            Engine::Geometries::Coordinate middle_pt;

            int center_index;                         // 路口中心点在centerpts中的index;
            Engine::Geometries::Coordinate center_pt; // 路口点坐标

            Engine::Geometries::Coordinate stopline_Normal;   // 停止线xy平面单位向量
            Engine::Geometries::Coordinate center_pt_porject; // 路口点在停止线上|延长线上|反向延长线上的投影点
            Engine::Geometries::Coordinate to_lulou_normal;   // 停止线指向中心点的单位向量
        };

        struct ClustrCloud_Realtion_Stopline
        {
            int clusterIndex;                                               // 在所有聚类中的index
            Engine::Base::Array<Engine::Geometries::Coordinate> clusterpts; // 聚类的点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr concavehull_ptr;            // 聚类凹包点云
            Engine::Geometries::Coordinate middle_pt;                       // 此聚类的中心点
            int r_center_index;                                             // 此聚类关联的路口点
            Engine::Geometries::Coordinate center_pt;                       // 此聚类关联的路口点

            int r_stoplineIndex;                                        // 关联到的停止线
            Stopline_Info stopline_realtion;                            // 该聚类点云关联的停止线
            Segment_Two_Points fit_axis;                                // 根据点云最小二乘拟合的轴线
            Engine::Geometries::Coordinate fit_axis_normal;             // 轴线的单位向量
            Engine::Geometries::Coordinate move_normal;                 // 朝向路口的单位向量
            Engine::Base::Array<Engine::Geometries::Coordinate> boxpts; // 矩形框点 左上-左下-右下-右上逆时针
            Engine::Base::Array<Engine::Geometries::Coordinate> icpboxpts;
            Engine::Geometries::Coordinate start_pt; // 计算上下边界时的外部点
            bool isright;                            // 通过中心点计算聚类是否有问题 true无问题 false有问题
        };

        struct Bev_Crosswalk
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

        struct NearbyLinesResult {
            std::vector<int> line_ids;  // 符合条件的线段ID
            std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr> line_points;  // 各线段在阈值内的点云
        };

        struct DirectionFilterResult {
            std::vector<int> filtered_lines;  // 过滤后的线段ID
            Eigen::Vector3f main_direction;   // 最终主方向（单位向量）
        };

    public:
        // 根据停止线以及crosswalk的label点云进行人行横道的提取
        void get_crosswalk_by_stopline(std::string global_pcd_path, const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts, const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &stop_lines, std::string outputDir, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &edgePts, bool online = true);

        // 根据crosswalk的label点云进行人行横道的提取
        void run_yxx_fin_in(std::string middle_json_path, std::string dataDir, std::string pcdPath, std::string refFile, std::string outputDir, std::string data_type);

        // 根据crosswalk的label点云进行人行横道的提取
        void run_bev_label_crosswalk(std::string global_pcd_path, std::string bev_obj_dir, std::string outputDir, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &edgePts);

        void ExtractPointsPackage(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods);
        void ExtractBox(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods, double widthToler, double weightToler);
        void SACSegmentationLine(pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output);
        void anticlockwisePolygon(Engine::Base::Array<Engine::Geometries::Coordinate> &plyCroods); // 将多边形变化为逆时针序列
        void extractBoxOptimized(const pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud, Engine::Base::Array<Engine::Geometries::Coordinate>& plyCroods, double widthToler, double heightToler); 

    private:
        bool filter_label_pcd(std::string global_pcd_path, pcl::PointCloud<pcl::PointXYZ> &crosswalkCloud, std::string outputDir, std::string ziduan = "label", int value = 90);
        bool get_stop_line_driver_dir(const Engine::Base::Array<Engine::Geometries::Coordinate> &stop_line, const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts, Engine::Geometries::Coordinate &direction);
        void pcl_crosswalk(pcl::PointCloud<pcl::PointXYZ>::Ptr crosswalkCloud);
        void output_bev(pcl::PointCloud<pcl::PointXYZ> &point_cloud, double pixel_size_m, std::string output_folder, pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_cloud);
        void cluster_pts_realtion(const std::vector<std::vector<int>> &clustersnew,
                                  const pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_cloud);

        void check_edge(ClustrCloud_Realtion_Stopline &one_clustercloud);
        void line_inter_disp(const Engine::Geometries::Coordinate &ori_pt, const Engine::Geometries::Coordinate &direction, double disp, double length, Engine::Base::Array<Engine::Geometries::Coordinate> &res_pts);
        void get_diff_max_pts(const ClustrCloud_Realtion_Stopline &one_clustercloud, const Engine::Geometries::Coordinate &direction, double disp, double length, int neighborhoodCount, int &upindex, int &downindex);
        int diff_max(const Engine::Base::Array<int> &arrCount, int neighborhoodCount = 0);
        bool adjust_cluster(const pcl::PointCloud<pcl::PointXYZ>::Ptr &clusterCloud, const Engine::Geometries::Coordinate &midPt);
        //////////////////////////////////////////////////////////////////////
        void check_edge2(ClustrCloud_Realtion_Stopline &one_clustercloud); // 修改提取外包框策略
        void get_up_down_edge(ClustrCloud_Realtion_Stopline &one_clustercloud, const Engine::Geometries::Coordinate &direction, double disp, double length, int neighborhoodCount, int &upindex, int &downindex);
        int diff_min(const Engine::Base::Array<int> &arrCount, int neighborhoodCount);
        //////////////////////////////////////////////////////////////////////
        void inputLuKouCenterPts(const Engine::Base::Array<Engine::Geometries::Coordinate> &lukou_center_pts);
        void inputStoplines(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &stop_lines);
        void clipPtsByStopline(const Engine::Base::Array<Engine::Geometries::Coordinate> &ori_cluster_pts, Stopline_Info stopline, Engine::Base::Array<Engine::Geometries::Coordinate> &clipPts);
        void clipPtsByMiddleptStopline(const ClustrCloud_Realtion_Stopline &one_clustercloud, Stopline_Info stopline, Engine::Base::Array<Engine::Geometries::Coordinate> &clipPts);
        void getProjNormal(Engine::Geometries::Coordinate s_pt, Engine::Geometries::Coordinate e_pt, Engine::Geometries::Coordinate hit_pt, Engine::Geometries::Coordinate &proj_pt, Engine::Geometries::Coordinate &proj_normal);
        void deal_r_center_stopline(ClustrCloud_Realtion_Stopline &one_clustercloud);                                                                              // 处理既有中心点也有停止线相关联的人行横道
        void deal_r_center(ClustrCloud_Realtion_Stopline &one_clustercloud);                                                                                       // 处理只有中心点相关联的人行横道
        void deal_r_stopline(ClustrCloud_Realtion_Stopline &one_clustercloud);                                                                                     // 处理只有停止线关联的人行横道
        void deal_no_r(ClustrCloud_Realtion_Stopline &one_clustercloud);                                                                                           // 处理只有未有任何关联的人行横道
        void deal_wrong(ClustrCloud_Realtion_Stopline &one_clustercloud, Engine::Base::Array<ClustrCloud_Realtion_Stopline> &newCluster);                          // 处理中心点错误的人行横道
        void fix_edge_icp(ClustrCloud_Realtion_Stopline &one_clustercloud);                                                                                        // 根据生成矩形以及相关连点云调整矩形框位置
        bool adjust_box(const Engine::Base::Array<Engine::Geometries::Coordinate> &clusterpts, const Engine::Base::Array<Engine::Geometries::Coordinate> &boxpts); // 根据点云判断生成的框是否准确，覆盖点云少于50%，则认为错误
        void getConcaveHull(const Engine::Base::Array<Engine::Geometries::Coordinate> &clusterpts, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull);
        bool getInterpolationLine(Engine::Base::Array<Engine::Geometries::Coordinate> &coordinates, std::vector<int> &ori_pts_index, double interval = 1.0, int save_ori_pts = 0); // 对点按照一定距离加密，当保留原始点时，输出原始点在新插入点中的索引

        ///////////////////////////////////////////////////bev结果引入/////////////////////////////////
        void ReadObj(std::string filepath, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &arrArrPts);
        void bev_crosswalk_obj(const std::string bev_obj_floder, const std::string out_obj_path, Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks);
        void get_cloud_hull(const pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull);
        bool bev_obj_icp(Engine::Base::Array<Engine::Geometries::Coordinate> &bev_obj, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull);
        void pts_geo_polygon(const Engine::Base::Array<Engine::Geometries::Coordinate> &pts, Engine::Geometries::Polygon &resPolygon);
        void cal_bev_crosswalk_core(const Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>> &bev_crosswalks, const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_hull, double toler = 0.0); // 低于toler会被删除

        void get_Envelope(const Engine::Base::Array<Engine::Geometries::Coordinate> &pts, Engine::Geometries::Envelope &envelope);
        void filter_bev_polygons();
        bool Bev_Crosswalk_to_ClustrCloud_Realtion_Stopline(Bev_Crosswalk &bev_crosswalk, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pts_in_cloud, ClustrCloud_Realtion_Stopline &one_clustercloud);
        void cal_fitaxis_height_width(Bev_Crosswalk &cal_length_width);
        void crosswalk_cloud_to_png(pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd, cv::Mat &crosswalk_image, int &width, int &height);
        void image_to_cloud(const cv::Mat &crosswalk_image, pcl::PointCloud<pcl::PointXYZ>::Ptr &bev_crosswalk_pcd);
        void get_cloud_in_bevcrosswalk(Bev_Crosswalk &bev_crosswalk, const pcl::PointCloud<pcl::PointXYZ>::Ptr &pano_segCloud, const pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr &kdtree_cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pts_in_cloud);

        float computeAverageNeighborDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
        float computePolygonArea(const std::vector<Eigen::Vector2f>& points);
        MinAreaRect findMinAreaRect(const std::vector<Eigen::Vector2f>& points);
        MinAreaRect optimizeRectangle(const std::vector<Eigen::Vector2f>& hullPoints, float polyArea, float lambda = 0.3);
        bool getSingleSegmentIntersections(const Engine::Base::Array<Engine::Geometries::Coordinate>& rect, const Engine::Geometries::Coordinate& c1, const Engine::Geometries::Coordinate& c2, std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections);
        void collectIntersections(const Engine::Base::Array<Engine::Geometries::Coordinate>& rect, const pcl::PointCloud<PointElement>::Ptr& line_pcd, std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections);
        bool validateOppositeEdges(const std::vector<std::pair<Engine::Geometries::Coordinate, int>>& intersections, std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>& result);
        void computeEdgeUV(const Engine::Geometries::Coordinate& A, const Engine::Geometries::Coordinate& AB, const Engine::Geometries::Coordinate& AD, const Engine::Geometries::Coordinate& p, double& u, double& v);
        std::vector<Engine::Base::Array<Engine::Geometries::Coordinate>> splitWithGap(const Engine::Base::Array<Engine::Geometries::Coordinate>& originalRect, const std::vector<std::pair<Engine::Geometries::Coordinate, Engine::Geometries::Coordinate>>& intersectionPairs);
        void splitRect(const std::string dataDir, const std::string refFile, const Engine::Base::Array<Engine::Geometries::Coordinate>& inputRect, std::vector<Engine::Base::Array<Engine::Geometries::Coordinate>>& outputRects, double widthToler, double heightToler);
        NearbyLinesResult findNearbyLinesWithMinPoints(const pcl::PointXYZ& target_point, const pcl::PointXYZ& site_center, double threshold,
            const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache, int minPts = 2);  
        NearbyLinesResult findLinesWithDynamicThreshold(const Engine::Base::Array<Engine::Geometries::Coordinate> crosswalk, const pcl::PointXYZ& site_center, 
            const std::unordered_map<int, pcl::PointCloud<pcl::PointXYZ>::Ptr>& lane_clouds_cache);
        DirectionFilterResult filterLinesByDirection(const NearbyLinesResult& candidates, float angle_threshold_deg = 30.0f);              
        Engine::Geometries::Coordinate calculateCenter(const Engine::Base::Array<Engine::Geometries::Coordinate>& object);    
        Eigen::Vector3f calculateDirection(const std::pair<PointElement, PointElement>& stopline);       
        double calculateMinCornerDistanceToStopline(const Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline);        
        double calculateDistanceToStopline(const Engine::Geometries::Coordinate& point, const std::pair<PointElement, PointElement>& stopline);
        int find_closest_edge(const Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline);
        void rotatePoint(Engine::Geometries::Coordinate& point, const Engine::Geometries::Coordinate& center, double angle);        
        void correctCrosswalk(Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline);        
        void adjustCrosswalkDistance(Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline);
        void moveCrosswalk(Engine::Base::Array<Engine::Geometries::Coordinate>& crosswalk, const std::pair<PointElement, PointElement>& stopline, double offset);      
        void afterProcess(const std::string middle_json_path, const std::string& dataDir, const std::string& outputDir, const std::string& refFile,
                    Engine::Base::Array<Engine::Base::Array<Engine::Geometries::Coordinate>>& crosswalks);

    private:
        Engine::Base::Array<Engine::Geometries::Coordinate> _lukoupts;
        Engine::Base::Array<Stopline_Info> _stoplines;
        std::map<int, Engine::Base::Array<Stopline_Info>> _center_stoplines; // 创建路口点与人行横道的关系
        Engine::Base::Array<ClustrCloud_Realtion_Stopline> _crosswalk_info;
        std::string _output;
        Engine::Base::Array<Bev_Crosswalk> _bev_crosswalks_polygons;
        float _VoxelGrid_size; // 体素滤波值
        // cv::Mat _crosswalk_image; //将n人行横道点云拍成bev视角下的图片
        // double _pixel_size_m; //像素值 单位米
        // int _width; //图片宽
        // int _height; //图片高
        // int _min_x; //图像左下角x
        // int _min_y;  //图像左下角y
        Engine::Base::Array<Engine::Geometries::Coordinate> _crosswalkcloud;
    };
} // namespace RoadMapping
#endif // ROADMAPPING_PROCESSCROSSWALK_H
