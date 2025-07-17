from enum import Enum

class DataType(Enum):
    MMT_RC = 1
    BYD_BEV = 2
    BYD_LIDAR_B = 3
    BYD_LIDAR_BEV_B = 4

class BevLineTrackCategory(Enum):
    other = 0
    lane_line = 1
    stop_line = 2
    center_line = 3
    road_boundary = 4
    crosswalk = 5
    junction = 6
    arrow = 7
    split_merge = 8

# other_scripts/msg/environment_model_msgs/local_map_info.proto
class LaneMarkerTypeByd(Enum):
    LM_TYPE_UNKNOWN = 0
    LM_TYPE_SOLID = 1
    LM_TYPE_DASHED = 2
    LM_TYPE_DASHED_SOLID = 3
    LM_TYPE_SOLID_DASHED = 4
    LM_TYPE_DOUBLE_DASHED = 5
    LM_TYPE_DOUBLE_SOLID = 6
    LM_TYPE_FISHBONE = 7
    LM_TYPE_WIDEDASHED = 8
    LM_TYPE_INVALID = 9
    RM_TYPE_CROSSWALK = 10
    RM_TYPE_STRAIGHT = 11
    RM_TYPE_LEFT = 12
    RM_TYPE_RIGHT = 13
    RM_TYPE_TURNING = 14
    RM_TYPE_STRAIGHT_LEFT = 15
    RM_TYPE_STRAIGHT_RIGHT = 16
    RM_TYPE_STRAIGHT_LEFT_RIGHT = 17
    RM_TYPE_LEFT_RIGHT = 18
    RM_TYPE_STRAIGHT_TURNING = 19
    RM_TYPE_LEFT_TURNING = 20
    RM_TYPE_IMPORT = 21
    RM_TYPE_EXPORT = 22
    RM_TYPE_STOPLINE = 23
    RM_TYPE_DECELERATION_ZONE = 24
    RM_TYPE_DIVERSION_ZONE = 25

class LaneTypeByd(Enum):
    LANE_TYPE_UNKNOWN = 0
    LANE_TYPE_MAIN = 1
    LANE_TYPE_SIDE = 2
    LANE_TYPE_EMERGENCY = 3
    LANE_TYPE_OTHER = 4
    LANE_TYPE_BLOCKED = 5
    LANE_TYPE_EMERGENCY_STOP = 6

class LaneMarkerColorByd(Enum):
    LM_COLOR_UNKNOWN = 0
    LM_COLOR_WHITE = 1
    LM_COLOR_YELLOW = 2
    LM_COLOR_BLUE = 3
    LM_COLOR_GREEN = 4
    LM_COLOR_RED = 5

class RoadEdgeTypeByd(Enum):
    ROADEDGE_TYPE_UNKNOWN = 0
    ROADEDGE_TYPE_FLAT = 1
    ROADEDGE_TYPE_LOW = 2
    ROADEDGE_TYPE_HIGH = 3
    ROADEDGE_TYPE_FENCE = 4

# 遍历枚举类并生成字典
g_lane_type_map = {lane_type.name: lane_type for lane_type in LaneTypeByd}
def get_lane_type(name):
    return g_lane_type_map.get(name, LaneTypeByd.LANE_TYPE_UNKNOWN)

g_lane_marker_type_map = {lane_marker_type.name: lane_marker_type for lane_marker_type in LaneMarkerTypeByd}
def get_lane_marker_type(name):
    return g_lane_marker_type_map.get(name, LaneMarkerTypeByd.LM_TYPE_UNKNOWN)

g_lane_color_map = {lane_color.name: lane_color for lane_color in LaneMarkerColorByd}
def get_lane_color(name):
    return g_lane_color_map.get(name, LaneMarkerColorByd.LM_COLOR_UNKNOWN)

g_road_edge_type_map = {road_edge_type.name: road_edge_type for road_edge_type in RoadEdgeTypeByd}
def get_road_edge_type(name):
    return g_road_edge_type_map.get(name, RoadEdgeTypeByd.ROADEDGE_TYPE_UNKNOWN)

g_type_map_byd = {
    # raw_bev_id: (类型，color， shape， 可视化颜色)
    1: ('ELEMENT_LANE_LINE', 0, 0, (255, 255, 255)), #车道线，UNKNOWN_COLOR，UNKNOWN
    2: ('ELEMENT_LANE_LINE', 0, 2, (255, 255, 255)), #车道线，UNKNOWN_COLOR，dashed
    3: ('ELEMENT_LANE_LINE', 0, 1, (255, 255, 255)), #车道线，UNKNOWN_COLOR，solid
    4: ('ELEMENT_LANE_LINE', 0, 5, (255, 255, 255)), #车道线，UNKNOWN，double dashed
    5: ('ELEMENT_LANE_LINE', 0, 6, (255, 255, 255)), #车道线，UNKNOWN，double solid
    6: ('ELEMENT_LANE_LINE', 0, 3, (255, 255, 255)), #车道线，UNKNOWN，left_dashed_right_solid
    7: ('ELEMENT_LANE_LINE', 0, 4, (255, 255, 255)), #车道线，UNKNOWN，left_solid_right_dashed

    8: ('ELEMENT_LANE_LINE', 1, 0, (255, 255, 255)),  #车道线，白色线类，UNKNOWN
    9: ('ELEMENT_LANE_LINE', 1, 2, (255, 255, 255)),  #车道线，白色线类，dashed
    10: ('ELEMENT_LANE_LINE', 1, 1, (255, 255, 255)), #车道线，白色线类，solid
    11: ('ELEMENT_LANE_LINE', 1, 5, (255, 255, 255)), #车道线，白色线类，double dashed
    12: ('ELEMENT_LANE_LINE', 1, 6, (255, 255, 255)), #车道线，白色线类，double solid
    13: ('ELEMENT_LANE_LINE', 1, 3, (255, 255, 255)), #车道线，白色线类，left_dashed_right_solid
    14: ('ELEMENT_LANE_LINE', 1, 4, (255, 255, 255)), #车道线，白色线类，left_solid_right_dashed

    15: ('ELEMENT_LANE_LINE', 2, 0, (255, 255, 0)), #车道线，黄色线类，UNKNOWN
    16: ('ELEMENT_LANE_LINE', 2, 2, (255, 255, 0)), #车道线，黄色线类，dashed
    17: ('ELEMENT_LANE_LINE', 2, 1, (255, 255, 0)), #车道线，黄色线类，solid
    18: ('ELEMENT_LANE_LINE', 2, 5, (255, 255, 0)), #车道线，黄色线类，double dashed
    19: ('ELEMENT_LANE_LINE', 2, 6, (255, 255, 0)), #车道线，黄色线类，double solid
    20: ('ELEMENT_LANE_LINE', 2, 3, (255, 255, 0)), #车道线，黄色线类，double dashed
    21: ('ELEMENT_LANE_LINE', 2, 4, (255, 255, 0)), #车道线，黄色线类，double solid

    28 : ('ELEMENT_BARRIER', 0, 0, (152, 99, 60)),   # 道路边界，UNKNOWN
    29 : ('ELEMENT_BARRIER', 0, 1, (152, 99, 60)),   # 道路边界，mmt:普通, byd: ROADEDGE_TYPE_FLAT
    30 : ('ELEMENT_BARRIER', 0, 3, (152, 99, 60)),   # 道路边界，mmt:水马, byd: ROADEDGE_TYPE_HIGH
    31 : ('ELEMENT_BARRIER', 0, 2, (152, 99, 60)),   # 道路边界，mmt:锥桶, byd: ROADEDGE_TYPE_LOW
    32 : ('ELEMENT_BARRIER', 0, 4, (152, 99, 60)),   # 道路边界，mmt: 锥桶水马混合/施工牌/防撞桶, byd: ROADEDGE_TYPE_FENCE

    39 : ('ELEMENT_OBJECT', 10, 0, (255, 0, 0)),      # 人行道区域, byd: 

    # 17 : ('ELEMENT_JUNCTION', 1, 0, (100, 255, 100)), # 路口区域

    # 车道中心线
    41 : ('ELEMENT_LANE_CENTER', 3, 0, (0, 255, 0)),  # 中心线，混合车道，unknown
    42 : ('ELEMENT_LANE_CENTER', 3, 1, (0, 255, 0)),  # 中心线，混合车道，普通车道
    43 : ('ELEMENT_LANE_CENTER', 3, 3, (0, 255, 0)),  # 中心线，混合车道，应急车道
    50 : ('ELEMENT_LANE_CENTER', 3, 4, (0, 255, 0)),  # 中心线，混合车道，不完整车道
    51 : ('ELEMENT_LANE_CENTER', 3, 5, (0, 255, 0)),  # 中心线，混合车道，blocked (byd)

    54 : ('ELEMENT_OBJECT', 7, 0, (255, 255, 0)),   # 地面标识，unknown
    55 : ('ELEMENT_OBJECT', 7, 11, (255, 255, 0)),  # 地面标识，直行
    56 : ('ELEMENT_OBJECT', 7, 13, (255, 255, 0)),  # 地面标识，右转
    57 : ('ELEMENT_OBJECT', 7, 12, (255, 255, 0)),  # 地面标识，左转
    58 : ('ELEMENT_OBJECT', 7, 14, (255, 255, 0)),  # 地面标识，左转掉头

    63 : ('ELEMENT_OBJECT', 8, 0, (255, 0, 0)),       # 停止线，unknown
    64 : ('ELEMENT_OBJECT', 8, 23, (255, 0, 0)),       # 停止线，普通停止线
    
    # other_scripts/msg/perception_msgs/dynamic_common.proto
    # 70 : ('ELEMENT_TRAFFICLIGHT', 0, 0, (255, 255, 0)),  # TRAFFIC_LIGHT_GROUP_SHAPE_HORIZONTAL
    # 71 : ('ELEMENT_TRAFFICLIGHT', 1, 0, (255, 255, 0)),  # TRAFFIC_LIGHT_GROUP_SHAPE_VERTICAL
    # 72 : ('ELEMENT_TRAFFICLIGHT', 2, 0, (255, 255, 0))  # TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
}

g_type_map_mmt_rc = {
    # raw_bev_id: (类型，color， shape， 可视化颜色)
    1: ('ELEMENT_LANE_LINE', 0, 0, (255, 255, 255)), #车道线，UNKNOWN_COLOR，UNKNOWN
    2: ('ELEMENT_LANE_LINE', 0, 1, (255, 255, 255)), #车道线，UNKNOWN_COLOR，dashed
    3: ('ELEMENT_LANE_LINE', 0, 2, (255, 255, 255)), #车道线，UNKNOWN_COLOR，solid
    4: ('ELEMENT_LANE_LINE', 1, 0, (255, 255, 255)), #车道线，白色线类，UNKNOWN
    5: ('ELEMENT_LANE_LINE', 1, 1, (255, 255, 255)), #车道线，白色线类，dashed
    6: ('ELEMENT_LANE_LINE', 1, 2, (255, 255, 255)), #车道线，白色线类，solid
    7: ('ELEMENT_LANE_LINE', 2, 0, (255, 255, 0)), #车道线，黄色线类，UNKNOWN
    8: ('ELEMENT_LANE_LINE', 2, 1, (255, 255, 0)), #车道线，黄色线类，dashed
    9: ('ELEMENT_LANE_LINE', 2, 2, (255, 255, 0)), #车道线，黄色线类，solid
    10 : ('ELEMENT_BARRIER', 0, 0, (152, 99, 60)),    # 道路边界，UNKNOWN
    11 : ('ELEMENT_BARRIER', 2, 0, (152, 99, 60)),   # 道路边界，普通
    12 : ('ELEMENT_BARRIER', 4, 0, (152, 99, 60)),   # 道路边界，水马
    13 : ('ELEMENT_BARRIER', 5, 0, (152, 99, 60)),   # 道路边界，锥桶
    14 : ('ELEMENT_BARRIER', 20, 0, (152, 99, 60)), # 道路边界，锥桶水马混合/施工牌/防撞桶
    # 15 : ('ELEMENT_OBJECT', 8, 1, (255, 0, 0)),       # 停止线
    16 : ('ELEMENT_OBJECT', 10, 0, (255, 0, 0)),      # 人行道区域

    17 : ('ELEMENT_JUNCTION', 1, 0, (100, 255, 100)), # 路口区域

    # 车道中心线
    18 : ('ELEMENT_LANE_CENTER', 3, 0, (0, 255, 0)),  # 中心线，混合车道，unknown
    19 : ('ELEMENT_LANE_CENTER', 3, 1, (0, 255, 0)),  # 中心线，混合车道，普通车道
    20 : ('ELEMENT_LANE_CENTER', 3, 5, (0, 255, 0)),  # 中心线，混合车道，应急车道
    21 : ('ELEMENT_LANE_CENTER', 3, 6, (0, 255, 0)),  # 中心线，混合车道，公交车道
    22 : ('ELEMENT_LANE_CENTER', 3, 15, (0, 255, 0)), # 中心线，混合车道，非机动车道
    23 : ('ELEMENT_LANE_CENTER', 3, 20, (0, 255, 0)), # 中心线，混合车道，收费站
    24 : ('ELEMENT_LANE_CENTER', 3, 28, (0, 255, 0)), # 中心线，混合车道，可变车道
    25 : ('ELEMENT_LANE_CENTER', 3, 32, (0, 255, 0)), # 中心线，混合车道，潮汐车道
    26 : ('ELEMENT_LANE_CENTER', 3, 71, (0, 255, 0)), # 中心线，混合车道，左右转或者掉头待转区
    27 : ('ELEMENT_LANE_CENTER', 3, 73, (0, 255, 0)), # 中心线，混合车道，不完整车道

    28 : ('ELEMENT_OBJECT', 7, 0, (255, 255, 0)),  # 地面标识，unknown
    29 : ('ELEMENT_OBJECT', 7, 1, (255, 255, 0)),  # 地面标识，直行
    30 : ('ELEMENT_OBJECT', 7, 2, (255, 255, 0)),  # 地面标识，右转
    31 : ('ELEMENT_OBJECT', 7, 3, (255, 255, 0)),  # 地面标识，左转
    32 : ('ELEMENT_OBJECT', 7, 4, (255, 255, 0)),  # 地面标识，左转掉头
    33 : ('ELEMENT_OBJECT', 7, 5, (255, 255, 0)),  # 地面标识，右转掉头
    34 : ('ELEMENT_OBJECT', 7, 6, (255, 255, 0)),  # 地面标识，向左合流
    35 : ('ELEMENT_OBJECT', 7, 7, (255, 255, 0)),  # 地面标识，向右合流
    36 : ('ELEMENT_OBJECT', 7, 13, (255, 255, 0)), # 地面标识，导流区

    37 : ('ELEMENT_OBJECT', 8, 0, (255, 0, 0)),       # 停止线，unknown
    38 : ('ELEMENT_OBJECT', 8, 1, (255, 0, 0)),       # 停止线，普通停止线
    39 : ('ELEMENT_OBJECT', 8, 2, (255, 0, 0)),       # 停止线，左转待行区停止线
    40 : ('ELEMENT_OBJECT', 8, 3, (255, 0, 0)),       # 停止线，掉头待行区停止线
    41 : ('ELEMENT_OBJECT', 8, 4, (255, 0, 0)),       # 停止线，右转待行区停止线
    42 : ('ELEMENT_OBJECT', 8, 5, (255, 0, 0)),       # 停止线，直行待行区停止线
    
    47 : ('ELEMENT_TRAFFICLIGHT', 0, 0, (255, 255, 0)),  # TRAFFIC_LIGHT_GROUP_SHAPE_HORIZONTAL
    48 : ('ELEMENT_TRAFFICLIGHT', 1, 0, (255, 255, 0)),  # TRAFFIC_LIGHT_GROUP_SHAPE_VERTICAL
    49 : ('ELEMENT_TRAFFICLIGHT', 2, 0, (255, 255, 0))  # TRAFFIC_LIGHT_GROUP_SHAPE_SINGLE
}

def get_raw_bev_id(type_map, element_type, subtype, other):
    for index, value in type_map.items():
        if value[0] == element_type and value[1] == subtype and value[2] == other:
            return index
    return None

def get_type_len(data_type):
    TYPE_NUM = 0
    if data_type == DataType.MMT_RC.name:
        TYPE_NUM = len(g_type_map_mmt_rc)
    elif data_type == DataType.BYD_BEV.name or data_type == DataType.BYD_LIDAR_B.name or data_type == DataType.BYD_LIDAR_BEV_B.name:
        TYPE_NUM = len(g_type_map_byd)
    return TYPE_NUM

def get_combine_name(type_id, data_type):
    type_name = ""
    if data_type == DataType.MMT_RC.name:
        type_name: str = str(g_type_map_mmt_rc[type_id][0])+"_"+ str(g_type_map_mmt_rc[type_id][1])+"_"+ str(g_type_map_mmt_rc[type_id][2]) if type_id in g_type_map_mmt_rc.keys() else "NO_DEFINE"
    elif data_type == DataType.BYD_BEV.name or data_type == DataType.BYD_LIDAR_B.name or data_type == DataType.BYD_LIDAR_BEV_B.name:
        type_name: str = str(g_type_map_byd[type_id][0])+"_"+ str(g_type_map_byd[type_id][1])+"_"+ str(g_type_map_byd[type_id][2]) if type_id in g_type_map_byd.keys() else "NO_DEFINE"

    return type_name

def get_bev_track_category(type_id, data_type) -> BevLineTrackCategory:
    # category : BevLineTrackCategory
    if data_type == DataType.MMT_RC.name:
        # category =  "" if type_id in g_type_map_mmt_rc.keys() else BevLineTrackCategory.other
        if type_id in (1, 2, 3, 4, 5, 6, 7, 8, 9):
            return BevLineTrackCategory.lane_line  # line
        elif type_id in (10, 11, 12, 13, 14):
            return BevLineTrackCategory.road_boundary  # road_boundary
        elif type_id in (37, 38, 39, 40, 41, 42):
            return BevLineTrackCategory.stop_line  # stop
        elif type_id in (16,):
            return BevLineTrackCategory.crosswalk  # corsswalk
        elif type_id in (28, 29, 30, 31, 32, 33, 34, 35, 36):
            return BevLineTrackCategory.arrow  # arrow
        elif type_id in (18, 19, 20, 21, 22, 23, 24, 25, 26, 27):
            return BevLineTrackCategory.center_line  # lane_center
        elif type_id in (17,):
            return BevLineTrackCategory.junction  # junction
        else:
            return BevLineTrackCategory.other
    
    elif data_type == DataType.BYD_BEV.name or data_type == DataType.BYD_LIDAR_B.name or data_type == DataType.BYD_LIDAR_BEV_B.name:
        # category = "" if type_id in g_type_map_byd.keys() else BevLineTrackCategory.other
        # TODO:qzc ziyan
        if 1<= type_id <= 27:
            return BevLineTrackCategory.lane_line  # line
        elif 28<= type_id <= 37:
            return BevLineTrackCategory.road_boundary  # road_boundary
        elif type_id == 39:
            return BevLineTrackCategory.crosswalk  # corsswalk
        # elif type_id == 40:
        #     return BevLineTrackCategory.junction  # junction
        elif 41<= type_id <= 50:
            return BevLineTrackCategory.center_line  # lane_center
        elif 51 <= type_id <= 55:
            return BevLineTrackCategory.arrow  # arrow
        elif 60 <= type_id <= 61:
            return BevLineTrackCategory.stop_line  # stop
        elif type_id == 150:
            return BevLineTrackCategory.split_merge  # split_merge
        else:
            return BevLineTrackCategory.other
        
    # return category
