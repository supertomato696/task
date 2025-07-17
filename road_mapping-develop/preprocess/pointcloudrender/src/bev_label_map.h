#ifndef __BEV_LABEL_MAP__
#define __BEV_LABEL_MAP__

#include <iostream>

struct LabelLayerInfo
{
    uint8_t label;
    uint32_t shape;
    uint8_t color;

    // default constructor
    LabelLayerInfo() : label(0), shape(0), color(0) {}
};


enum class DataType {
    MMT_RC = 1,
    BYD_BEV = 2,
    BYD_LIDAR_B = 3,
    BYD_LIDAR_BEV_B = 4
};


void bev_label_map_mmt_new(int raw_bev_id, LabelLayerInfo &label_0, LabelLayerInfo &label_1, LabelLayerInfo &label_2, 
                      LabelLayerInfo &center_line, LabelLayerInfo &traffic_light)
{
    label_0.label = label_0.shape = label_0.color = 0;
    label_1.label = label_1.shape = label_1.color = 0;
    label_2.label = label_2.shape = label_2.color = 0;
    center_line.label = center_line.shape = center_line.color = 0;
    traffic_light.label = traffic_light.shape = traffic_light.color = 0;

    // label_0
    // if (raw_bev_id >= 10 && raw_bev_id <= 14) {
    //     label_0.label = 3; // 道路边界 road_boundary (和路口面有重叠，放到不同层)

    //     switch (raw_bev_id) {
    //         case 11: label_0.shape = 2; break; // 普通边界
    //         case 12: label_0.shape = 4; break; // 水马
    //         case 13: label_0.shape = 5; break; // 锥桶
    //         case 14: label_0.shape = 20; break; // 锥桶水马混合
    //     }
    // if (raw_bev_id == 16) {
    //     label_0.label = 3; // 人行道区域 crosswalk (和道路边界，车道线，路口面有重叠，放到不同层)
    // } else 
    if (raw_bev_id >= 37 && raw_bev_id <= 42){
        label_0.label = 4; // 停止线 stopline (和路口面有重叠，放到不同层)

        switch (raw_bev_id) {
            case 38: label_0.shape = 1; break; // 普通停止线 
            case 39: label_0.shape = 2; break; // 左转待行区停止线
            case 40: label_0.shape = 3; break; // 掉头待行区停止线
            case 41: label_0.shape = 4; break; // 右转待行区停止线
            case 42: label_0.shape = 5; break; // 直行待行区停止线
        }
    } else {
        // do nothing
    }                                               

    // label_1
    if (raw_bev_id >= 1 && raw_bev_id <= 9) {
        label_1.label = 1;// 车道线 (可能会和停止线/路口面有重叠，放到不同层)

        if (raw_bev_id == 2 || raw_bev_id == 5 || raw_bev_id == 8) {
            label_1.shape = 1; // dashed
        } else if (raw_bev_id == 3 || raw_bev_id == 6  || raw_bev_id == 9) {
            label_1.shape = 2; // solid
        } else {
            // do nothing
        }
    
        if (raw_bev_id >= 4 && raw_bev_id <= 6) {
            label_1.color = 1; // white
        } else if (raw_bev_id >= 7 && raw_bev_id <= 9) {
            label_1.color = 2; // yellow
        } else {
            // do nothing
        }
    // } else if (raw_bev_id == 16) {
    //     label_1.label = 3; // 人行道区域 crosswalk
    } else if (raw_bev_id >= 28 && raw_bev_id <= 35) {
        label_1.label = 4; // 箭头区域

        switch (raw_bev_id) {
            case 29: label_1.shape = 1; break; // 直行
            case 30: label_1.shape = 2; break; // 右转
            case 31: label_1.shape = 3; break; // 左转
            case 32: label_1.shape = 4; break; // 左转掉头
            case 33: label_1.shape = 5; break; // 右转掉头
            case 34: label_1.shape = 6; break; // 向左合流
            case 35: label_1.shape = 7; break; // 向右合流
        }
    } else if (raw_bev_id == 36) {
        label_1.label = 5; // 导流区
    } else { 
        // do nothing
    }

    // label_2
    if (raw_bev_id == 16) {
        label_2.label = 3; // 人行道区域 crosswalk (和道路边界，车道线，路口面有重叠，放到不同层)
    } else {
    // if (raw_bev_id == 17) {
    //     label_2.label = 3; // 路口区域 junction (几乎会和所有元素重叠)
    // } else {
        // do nothing
    }

    // center_line
    if (raw_bev_id >= 10 && raw_bev_id <= 14) {
       center_line.label = 2; // 道路边界 road_boundary (和人行横道，车道线，路口面有重叠，放到不同层)

        switch (raw_bev_id) {
            case 11: center_line.shape = 2; break; // 普通边界
            case 12: center_line.shape = 4; break; // 水马
            case 13: center_line.shape = 5; break; // 锥桶
            case 14: center_line.shape = 20; break; // 锥桶水马混合
        }
    } else if (raw_bev_id >= 18 && raw_bev_id <= 27) {
        center_line.label = 1; // lane_center (可能会和停止线/路口面/箭头有重叠，放到不同层)

        switch (raw_bev_id) {
            case 19: center_line.shape = 1; break; // 普通车道
            case 20: center_line.shape = 5; break; // 应急车道
            case 21: center_line.shape = 6; break; // 混合车道
            case 22: center_line.shape = 15; break; // 非机动车道
            case 23: center_line.shape = 20; break; // 收费站
            case 24: center_line.shape = 28; break; // 可变车道
            case 25: center_line.shape = 32; break; // 潮汐车道
            case 26: center_line.shape = 71; break; // 左右转或者掉头代转区
            case 27: center_line.shape = 73; break; // 不完整车道
        }
    } else {
        // do nothing
    }

    // traffic_light
    if (raw_bev_id >= 47 && raw_bev_id <= 49) {
        traffic_light.label = 1; // traffic_light

        if (raw_bev_id == 47) {
            traffic_light.shape = 1; // TRAFFIC_LIGHT_GROUP_SHAPE_HORIZONTAL
        }
        else if (raw_bev_id == 48) {
            traffic_light.shape = 2; // TRAFFIC_LIGHT_GROUP_SHAPE_VERTICAL
        }
        else if (raw_bev_id == 49) {
            traffic_light.shape = 3; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        } else {
            // do nothing
        }
    } else {
        // do nothing
    }

}


void bev_label_map_byd(int raw_bev_id, LabelLayerInfo &label_0, LabelLayerInfo &label_1, LabelLayerInfo &label_2, 
                      LabelLayerInfo &center_line, LabelLayerInfo &traffic_light)
{
    label_0.label = label_0.shape = label_0.color = 0;
    label_1.label = label_1.shape = label_1.color = 0;
    label_2.label = label_2.shape = label_2.color = 0;
    center_line.label = center_line.shape = center_line.color = 0;
    traffic_light.label = traffic_light.shape = traffic_light.color = 0;


    // if (raw_bev_id == 39) {
    //     label_0.label = 3; // 人行道区域 crosswalk (和道路边界，车道线，路口面有重叠，放到不同层)
    // } else if (raw_bev_id >= 63 && raw_bev_id <= 68){
    if (raw_bev_id >= 63 && raw_bev_id <= 68){
        label_0.label = 4; // 停止线 stopline (和路口面有重叠，放到不同层)

        switch (raw_bev_id) {
            case 64: label_0.shape = 1; break; // 普通停止线 
            // case 65: label_0.shape = 2; break; // 左转待行区停止线
            // case 66: label_0.shape = 3; break; // 掉头待行区停止线
            // case 67: label_0.shape = 4; break; // 右转待行区停止线
            // case 68: label_0.shape = 5; break; // 直行待行区停止线
        }
    } else {
        // do nothing
    }                                               

    // label_1
    if (raw_bev_id >= 1 && raw_bev_id <= 27) {
        label_1.label = 1;// 车道线 (可能会和停止线/路口面有重叠，放到不同层)

        if (raw_bev_id == 2 || raw_bev_id == 9  || raw_bev_id == 16) {
            label_1.shape = 1; // dashed
        } else if (raw_bev_id == 3 || raw_bev_id == 10 || raw_bev_id == 17) {
            label_1.shape = 2; // solid
        } else if (raw_bev_id == 4 || raw_bev_id == 11 || raw_bev_id == 18) {
            label_1.shape = 3; // double dashed
        } else if (raw_bev_id == 5 || raw_bev_id == 12 || raw_bev_id == 19) {
            label_1.shape = 4; // double solid
        } else if (raw_bev_id == 6 || raw_bev_id == 13 || raw_bev_id == 20) {
            label_1.shape = 5; // left_dashed_right_solid
        } else if (raw_bev_id == 7 || raw_bev_id == 14 || raw_bev_id == 21) {
            label_1.shape = 6; // left_solid_right_dashed
        } else {
            // do nothing
        }
    
        if (raw_bev_id >= 8 && raw_bev_id <= 14) {
            label_1.color = 1; // white
        } else if (raw_bev_id >= 15 && raw_bev_id <= 21) {
            label_1.color = 2; // yellow
        } else {
            // do nothing
        }
    // } else if (raw_bev_id == 16) {
    //     label_1.label = 3; // 人行道区域 crosswalk
    } else if (raw_bev_id >= 54 && raw_bev_id <= 61) {
        label_1.label = 4; // 箭头区域

        switch (raw_bev_id) {
            case 55: label_1.shape = 1; break; // 直行
            case 56: label_1.shape = 2; break; // 右转
            case 57: label_1.shape = 3; break; // 左转
            case 58: label_1.shape = 4; break; // 左转掉头
            // case 59: label_1.shape = 5; break; // 右转掉头
            // case 60: label_1.shape = 6; break; // 向左合流
            // case 61: label_1.shape = 7; break; // 向右合流
        }
    } else if (raw_bev_id == 62) {
        // label_1.label = 5; // 导流区
    } else { 
        // do nothing
    }

    // label_2
    if (raw_bev_id == 39) {
        label_2.label = 3; // 人行道区域 crosswalk (和道路边界，车道线，路口面有重叠，放到不同层)
    // if (raw_bev_id == 40) {
    //     label_2.label = 3; // 路口区域 junction (几乎会和所有元素重叠)
    } else if (raw_bev_id == 73) {
        label_2.label = 1; // 禁停区或路面标识（会和车道线、中心线、边界线、箭头重叠）
    // } else if (raw_bev_id == 150) {
    //     label_2.label = 4; // 分合流点
    } else {
        // do nothing
    }

    // center_line
    if (raw_bev_id >= 28 && raw_bev_id <= 37) {
       center_line.label = 2; // 道路边界 road_boundary (和人行横道，车道线，路口面有重叠，放到不同层)

        switch (raw_bev_id) {
            case 29: center_line.shape = 2; break; // 普通边界
            case 30: center_line.shape = 4; break; // 水马
            case 31: center_line.shape = 5; break; // 锥桶
            case 32: center_line.shape = 20; break; // 锥桶水马混合
        }
    } else if (raw_bev_id >= 41 && raw_bev_id <= 53) {
        center_line.label = 1; // lane_center (可能会和停止线/路口面/箭头有重叠，放到不同层)

        switch (raw_bev_id) {
            case 42: center_line.shape = 1; break; // 普通车道
            case 43: center_line.shape = 5; break; // 应急车道
            // case 44: center_line.shape = 6; break; // 混合车道
            // case 45: center_line.shape = 15; break; // 非机动车道
            // case 46: center_line.shape = 20; break; // 收费站
            // case 47: center_line.shape = 28; break; // 可变车道
            // case 48: center_line.shape = 32; break; // 潮汐车道
            // case 49: center_line.shape = 71; break; // 左右转或者掉头代转区
            case 50: center_line.shape = 73; break; // 不完整车道
            case 51: center_line.shape = 74; break; // blocked(byd)
        }
    } else {
        // do nothing
    }

    // traffic_light
    if (raw_bev_id >= 77 && raw_bev_id <= 90) {
        traffic_light.label = 1; // traffic_light
        if (raw_bev_id == 77) {
            traffic_light.shape = 1; // TRAFFIC_LIGHT_GROUP_SHAPE_HORIZONTAL
        }
        else if (raw_bev_id == 78) {
            traffic_light.shape = 2; // TRAFFIC_LIGHT_GROUP_SHAPE_VERTICAL
        }
        else if (raw_bev_id == 79) {
            traffic_light.shape = 3; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 80) {
            traffic_light.shape = 4; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 81) {
            traffic_light.shape = 5; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 82) {
            traffic_light.shape = 6; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 83) {
            traffic_light.shape = 7; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 84) {
            traffic_light.shape = 8; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 85) {
            traffic_light.shape = 9; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 86) {
            traffic_light.shape = 10; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 87) {
            traffic_light.shape = 11; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 88) {
            traffic_light.shape = 12; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 89) {
            traffic_light.shape = 13; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        }
        else if (raw_bev_id == 90) {
            traffic_light.shape = 14; // TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
        } else {
            // do nothing
        }
    } else {
        // do nothing
    }

}


void bev_label_map(std::string version, int raw_bev_id, int &label_0, int &label_1, int &label_2, int &color, int &shape,
                   int &center_line, int &traffic_light)
{
}

void bev_label_type_merge(std::vector<LabelLayerInfo>& input, LabelLayerInfo& output)
{
    output = LabelLayerInfo();

    if (input.empty()) {
        std::cerr << "label_type_merge: 输入为空" << std::endl;
        return;
    }

    int first_label = input[0].label;
    for (const auto& label_info : input) {
        if (label_info.label != 0 && label_info.label != first_label) {
            std::cerr << "label_type_merge: 输入的标签类型不一致" << std::endl;
            return;
        }
    }

    std::vector<int> shapes;
    for (const auto& label_info : input)
    {
        if (label_info.label != 0)
        {
            shapes.push_back(label_info.shape);
        }
    }

    if (shapes.empty()) {
        return;
    }

    std::sort(shapes.begin(), shapes.end());

    const int max_shape_count = 4;
    if (shapes.size() > max_shape_count) {// shape是int类型，最多4个
        std::cerr << "label_type_merge: 类型组合数量超过4个" << std::endl;
        return;
    }

    output = input[0];
    output.shape = 0;
    // std::cout << "label_type_merge[" << shapes.size() << "]: ";
    for (size_t i = 0; i < shapes.size(); ++i)
    {
        // std::cout << shapes[i] << " ";
        output.shape |= (shapes[i] << (8 * (shapes.size() - i - 1))); // 确保最小的在高位
    }
    // std::cout << "merge result: " << output.shape << std::endl;
}

void bev_label_map_new(const std::string version, const std::vector<int>& raw_bev_ids, LabelLayerInfo &label_0, LabelLayerInfo &label_1, LabelLayerInfo &label_2, 
                   LabelLayerInfo &center_line, LabelLayerInfo &traffic_light)
{
    std::vector<LabelLayerInfo> temp_label_0(raw_bev_ids.size());
    std::vector<LabelLayerInfo> temp_label_1(raw_bev_ids.size());
    std::vector<LabelLayerInfo> temp_label_2(raw_bev_ids.size());
    std::vector<LabelLayerInfo> temp_center_line(raw_bev_ids.size());
    std::vector<LabelLayerInfo> temp_traffic_light(raw_bev_ids.size());

    if ((version == "MMT_RC"))
    {
        for (size_t i = 0; i < raw_bev_ids.size(); i++)
        {
            bev_label_map_mmt_new(raw_bev_ids[i], temp_label_0[i], temp_label_1[i], temp_label_2[i], temp_center_line[i], temp_traffic_light[i]);
        }

        if (raw_bev_ids.size() > 1) {
            bev_label_type_merge(temp_label_0, label_0);
            bev_label_type_merge(temp_label_1, label_1);
            // std::cout << "label_1: " << label_1.shape << std::endl;
            bev_label_type_merge(temp_label_2, label_2);
            bev_label_type_merge(temp_center_line, center_line);
            // std::cout << "center_line: " << center_line.shape << std::endl;
            bev_label_type_merge(temp_traffic_light, traffic_light);
        } else {
            label_0 = temp_label_0[0];
            label_1 = temp_label_1[0];
            label_2 = temp_label_2[0];
            center_line = temp_center_line[0];
            traffic_light = temp_traffic_light[0];
        }
    } else if(version == "BYD_BEV") {
       for (size_t i = 0; i < raw_bev_ids.size(); i++)
        {
            bev_label_map_byd(raw_bev_ids[i], temp_label_0[i], temp_label_1[i], temp_label_2[i], temp_center_line[i], temp_traffic_light[i]);
        }

        if (raw_bev_ids.size() > 1) {
            bev_label_type_merge(temp_label_0, label_0);
            bev_label_type_merge(temp_label_1, label_1);
            // std::cout << "label_1: " << label_1.shape << std::endl;
            bev_label_type_merge(temp_label_2, label_2);
            bev_label_type_merge(temp_center_line, center_line);
            // std::cout << "center_line: " << center_line.shape << std::endl;
            bev_label_type_merge(temp_traffic_light, traffic_light);
        } else {
            label_0 = temp_label_0[0];
            label_1 = temp_label_1[0];
            label_2 = temp_label_2[0];
            center_line = temp_center_line[0];
            traffic_light = temp_traffic_light[0];
        }
    } else {
        std::cout << "未知的 BEV 版本号: " << version << std::endl;
    }
}

#endif