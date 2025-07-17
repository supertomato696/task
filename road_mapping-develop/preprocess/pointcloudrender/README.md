# veh_scan 点云字段说明

geo_type:
0: 非路面
100: 路面-本向道路
200: 路面-对向道路

forward_distance: 点在 lidar 系的 x 值
side_distance: 点在 lidar 系的 y 值

label: 车端全景分割

semantic_label: 云端全景分割
255: 空类别

cloud_label: 云端车道线分割
255: 空类别


----------------------------------

                                           up z
                            front x           ^
                                 /            |
                                /             |
                             p7+ -----------  + p5
                              /|            / |
                             / |p6         /  |
                         p3 + -+--------- +p1-+ p4
                            |  /      .   |  /
                            | / origin    | /
                          p2+ ----------- + p0


------------------------------------------

# bev label 37 

FSDMAP_TYPE_INDEX_DICT = {
    0: 'other_unset_type',
    1: "other_solid",
    2: "other_dashed",
    3: 'other_double_solid',
    4: 'other_double_dashed',
    5: 'other_left_dashed_right_solid',
    6: 'other_left_solid_right_dashed',
    7: 'white_unset_type',
    8: 'white_solid',
    9: 'white_dashed',
    10: 'white_double_solid',
    11: 'white_double_dashed',
    12: 'white_left_dashed_right_solid',
    13: 'white_left_solid_right_dashed',
    14: 'yellow_unset_type',
    15: 'yellow_solid',
    16: 'yellow_dashed',
    17: 'yellow_double_solid',
    18: 'yellow_double_dashed',
    19: 'yellow_left_dashed_right_solid',
    20: 'yellow_left_solid_right_dashed',
    21: 'unknow_road_boundary',
    22: 'road_boundary_curb',
    23: 'road_boundary_fence',
    24: 'road_boundary_wall',
    25: 'road_boundary_surface',
    26: 'road_boundary_ditch',
    27: 'road_boundary_obstacle',
    28: 'stopline',
    29: 'crosswalk',
    30: 'polygon_other',
    31: 'polygon_triIsland',
    32: 'polygon_bed',
    33: 'polygon_booth',
    34: 'polygon_5',
    35: 'polygon_6',
    36: 'lane_center',
    37: 'junction',
    38: 'arrow_unknown',
    39: 'arrow_left',
    40: 'arrow_forward',
    41: 'arrow_right',
    42: 'arrow_left_and_forward',
    43: 'arrow_right_and_forward',
    44: 'arrow_left_and_right',
    45: 'arrow_u_turn',
    46: 'arrow_u_turn_and_forward',
    47: 'arrow_u_turn_and_left',
    48: 'arrow_merge_left',
    49: 'arrow_merge_right',
    50: 'crosswalk_notice',
    51: 'speed_limit_low',
    52: 'speed_limit_high',
    53: 'arrow_no_left_turn',
    54: 'arrow_no_right_turn',
    55: 'arrow_no_u_turn',
    56: 'arrow_forward_and_left_and_right',
    57: 'arrow_forward_and_u_turn_and_left',
    58: 'arrow_right_and_u_turn',
    59: 'marking_text',
    60: 'marking_time',
    61: 'check_following_distance',
    62: 'stopto_give_way',
    63: 'slowdown_to_give_way',
    64: 'marking_nets',
}