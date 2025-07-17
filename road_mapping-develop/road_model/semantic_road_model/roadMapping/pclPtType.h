//
//
//

#ifndef ROADMAPPING_PCLPTTYPE_H
#define ROADMAPPING_PCLPTTYPE_H

#define PCL_NO_PRECOMPILE
#include <Eigen/Core>
#include <map>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

struct EIGEN_ALIGN16 MyColorPointType
{
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        PCL_ADD_INTENSITY;
        uint8_t cloud_bev_label;
        uint32_t cloud_bev_label_shape;
        uint8_t cloud_bev_label_color;    
        uint8_t cloud_bev_label_1;
        uint32_t cloud_bev_label_1_shape;
        uint8_t cloud_bev_label_1_color;
        uint8_t cloud_bev_label_2;
        uint32_t cloud_bev_label_2_shape;
        uint8_t cloud_bev_label_2_color;
        uint8_t cloud_bev_center_line;
        uint32_t cloud_bev_center_line_shape;
        uint8_t cloud_bev_center_line_color;
        // uint8_t cloud_bev_traffic_light;
        // uint8_t cloud_bev_traffic_light_shape;
        // uint8_t cloud_bev_traffic_light_color;
        uint8_t cloud_bev_label_score;
        uint8_t cloud_bev_center_line_score;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(MyColorPointType, // 注册点类型宏 XYZRGBI + "GPSTime" (as fields)
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint32_t, cloud_bev_label_shape, cloud_bev_label_shape)
                                  (uint8_t, cloud_bev_label_color, cloud_bev_label_color)
                                  (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
                                  (uint32_t, cloud_bev_label_1_shape, cloud_bev_label_1_shape)
                                  (uint8_t, cloud_bev_label_1_color, cloud_bev_label_1_color)
                                  (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
                                  (uint32_t, cloud_bev_label_2_shape, cloud_bev_label_2_shape)
                                  (uint8_t, cloud_bev_label_2_color, cloud_bev_label_2_color)
                                  (uint8_t, cloud_bev_center_line, cloud_bev_center_line)
                                  (uint32_t, cloud_bev_center_line_shape, cloud_bev_center_line_shape)
                                  (uint8_t, cloud_bev_center_line_color, cloud_bev_center_line_color)
                                //   (uint8_t, cloud_bev_traffic_light, cloud_bev_traffic_light)
                                //   (uint8_t, cloud_bev_traffic_light_shape, cloud_bev_traffic_light_shape)
                                //   (uint8_t, cloud_bev_traffic_light_color, cloud_bev_traffic_light_color)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
                                  (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))


struct EIGEN_ALIGN16 PointLabel // 强制SSE填充以获得正确的内存对齐
{
    PCL_ADD_POINT4D; // 添加XYZ
    PCL_ADD_RGB;
    PCL_ADD_INTENSITY; // 添加强度
    uint16_t label;
    uint8_t cloud_pano_seg;
    uint8_t cloud_line_seg;
    uint8_t cloud_bev_label;
    uint16_t distance_x_cm;
    uint16_t distance_y_cm;
    uint8_t cloud_bev_label_score;
    uint8_t cloud_bev_label_1;
    uint8_t cloud_bev_label_2;
    uint8_t cloud_bev_color;
    uint8_t cloud_bev_shape;
    uint8_t opt_label;
    float intensity_opt;
    float score;
    int8_t status = 0;
    PCL_MAKE_ALIGNED_OPERATOR_NEW // 确保新的分配器内存是对齐的
    friend std::ostream& operator << (std::ostream& os, const PointLabel& p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointLabel,
        (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)
        (float, intensity, intensity)
        (uint16_t, label, label)
        (uint8_t, cloud_pano_seg, cloud_pano_seg)
        (uint8_t, cloud_line_seg, cloud_line_seg)
        (uint8_t, cloud_bev_label, cloud_bev_label)
        (uint16_t, distance_x_cm, distance_x_cm)
        (uint16_t, distance_y_cm, distance_y_cm)
        (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
        (uint8_t, cloud_bev_label_1, cloud_bev_label_1)
        (uint8_t, cloud_bev_label_2, cloud_bev_label_2)
        (uint8_t, cloud_bev_color, cloud_bev_color)
        (uint8_t, cloud_bev_shape, cloud_bev_shape)
        (uint8_t, opt_label, opt_label)
        (float, intensity_opt, intensity_opt)
        (float, score, score)
        )


struct EIGEN_ALIGN16 ConRmPointType
{
        PCL_ADD_POINT4D;
        PCL_ADD_RGB;
        PCL_ADD_INTENSITY;
        uint8_t cloud_bev_label;
        uint32_t cloud_bev_shape;
        uint8_t cloud_bev_color;    
        uint8_t cloud_bev_label_score;
        uint8_t cloud_bev_center_line_score;
        PCL_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(ConRmPointType, // 注册点类型宏 XYZRGBI + "GPSTime" (as fields)
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, rgb, rgb)
                                  (float, intensity, intensity)
                                  (uint8_t, cloud_bev_label, cloud_bev_label)
                                  (uint32_t, cloud_bev_shape, cloud_bev_shape)
                                  (uint8_t, cloud_bev_color, cloud_bev_color)
                                  (uint8_t, cloud_bev_label_score, cloud_bev_label_score)
                                  (uint8_t, cloud_bev_center_line_score, cloud_bev_center_line_score))

struct EIGEN_ALIGN16 PointElement // 强制SSE填充以获得正确的内存对齐
{
        PCL_ADD_POINT4D; // 添加XYZ
        PCL_ADD_RGB;
        PCL_ADD_INTENSITY;            // 添加强度
        uint16_t ele_type;            // 要素类型
        uint32_t id;                  // 实例id
        uint32_t index;               // 同一个下面点的顺序
        uint32_t type1;               // 扩展类型1(存入形状属性，ccj，2024.12.18)
        uint32_t type2;               // 扩展类型2(存入颜色属性，ccj，2024.12.18)
        uint32_t type3;               // 扩展类型3
        float heading;                // yaw角
        float score;                  // 置信度
        PCL_MAKE_ALIGNED_OPERATOR_NEW // 确保新的分配器内存是对齐的
            friend std::ostream &
            operator<<(std::ostream &os, const PointElement &p);
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointElement,
                                  (float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(uint16_t, ele_type, ele_type)(uint32_t, id, id)(uint32_t, index, index)(uint32_t, type1, type1)(uint32_t, type2, type2)(uint32_t, type3, type3)(float, heading, heading)(float, score, score))


#endif // ROADMAPPING_PCLPTTYPE_H
