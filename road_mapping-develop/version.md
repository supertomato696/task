## 版本发布说明
v1.00.01 2024-12-26
1. road mapping 初步版本

v1.00.02 2024-12-28
1. road mapping 初步版本

v1.00.03 2024-12-30
1. 添加intersection shp   和用回ele_type和type保持统一、id用int 

v1.00.04 2024-12-31
1. 修改全部路面元素绑定的车道的结构体为RoadLaneInfo 
2. 添加路口进入退出的可视化，车道添加标志位判断进入退出路口is_entrance，is_exit;
3. 添加路口和人形横道绑定逻辑 
3. 优化路口画图速度，修改成从元素找车道；之前从车道找元素 会导致元素每次都画一遍，太慢了

v1.00.05 2024-12-31
1. fix 人行横道与路口绑定

v1.00.06 2024-01-01
1. 直接gcj02->utm，不经过 wgs84 转换
2. 给shp输出添加 crs

v1.00.07 2025-01-01
1. 修改人形横道 长边计算方法，适配不规则多边形情况

v1.00.09 2024-01-01
1. 添加路面元素的所有字段，车道id待取值

v1.00.10 2024-01-02
1. 修复crash
2. 修复 aois.shp 输出缺失问题

v1.00.11 2024-01-04
1. 修复路面拓扑丢失问题

v1.00.12 2024-01-05
1. 打开 多线程
2. 合并交通灯到主流程

v1.00.13 2024-01-06
1. 修复交通灯坐标转换问题
2. 更新最新版交通灯矢量化

v1.00.14 2024-01-07
1. 修改路面元素字段同步到最新规格

v1.00.15 2024-01-07
1. 增加交通灯拓扑关联

v1.00.16 2024-01-07
1. 去掉一些无用代码
2. 添加 mmt 中的 track_id 和 缩小拼图范围为x=[0,10],y=[-10,10]

v1.00.17 2024-01-09
1. 过滤掉一些地面点
2. 默认开启 使用 bev 优化的 pose 结果

v1.00.18 2024-01-13
1. 增加车道以及车道线的ID分组
2. 修改shp的字段以及文件名

v1.00.19 2024-01-13
1. 添加docker的debug和release两个版本，具体运行参考README.md

v1.00.20 2024-01-13
1. 添加version字段内容
2. 修改路面元素id
3. fix docker release和debug

v1.00.21 2024-01-14
1. 修改roadshp的关联车道ID索引
2. 适配ins修正link的距离阈值

v1.00.22 2024-01-14
1. 适配云端数据
2. 修复一些路径问题

v1.00.23 2024-01-14
1. 修改最终状态"message"为str
2. 打通联调，适配数据解析最新的 output.json, docker基础使用v3，
3. 修改version.sh  和version.h

v1.00.24 2024-01-15
1. 优化交通灯矢量化的若干问题
2. 适配新shp属性的交通灯拓扑关联

v1.00.25 2024-01-15
1. 分别输出orign mid 和拓扑 的shp

v1.00.26 2024-01-17
1. 修正link pos失效导致频繁打断的问题

v1.00.28 2024-01-18
1. dockerfile添加ldconfig
2. 推送test的docker的时候加用户和时间区分
3. 添加路面元素绑定的车道id 、  先用set容器存路口绑定道路id，后续会优化
4. 添加ckeck_results, 会在new-6lukou生成check_results.txt
5. 添加upload
6. 添加从0开始跑会删除output

v1.00.29 2024-01-21
1. 添加任务框切割

v1.00.30 2024-01-21
1. 增加任务框过滤
2. 解决关联后信号灯id重复问题
3. 清理非关键print输出
4. 增大关联路口面的距离阈值

v1.00.31 2024-01-22
1. 补全矢量化的车道线以及道路边界线的shp相关字段

v1.00.32 2024-01-22
1. 增加link 位置优化以及合并分组

v1.00.33 2024-01-23
1. 添加线和框找交点逻辑，修复任务框边界点
2. 修复部分线的边界点生成异常问题

v3.00.00 2024-01-23
1. 保持和项目组版本号一致

v3.00.01 2024-01-23
1. 预处理通过任务框，过滤轨迹，降低耗时和CPU

v3.00.02 2024-01-25
1. 合并3.00.02到 develop上
2. 删除交通灯mid输出

v3.00.03 2024-01-25
1. 修复边界框切割缺失

v3.00.04 2024-02-11
1. 修复道路边界线误连接

v3.00.05 2024-02-12
1. 非法属性置为unknown

v3.00.06 2024-02-13
1. link 优化位置
2. link 增加sd附着属性
3. link 优化增加路口范围限制动态调整权重

v3.00.07 2024-02-17
1. 适配C平台自研数据
2. 修复车道线的虚实属性反了的问题

v3.00.08 2024-02-17
1. 修复mach_line部分无法匹配到正确车道边线问题
2. fix输出lane boundary color； type字段有效值[1,2,3]； 
3. mute line_id & trail_id
4. 优化match_line车道线匹配错误问题

v3.00.09 2024-02-16
1. 解决矢量化和拓扑几何和属性不一致
2. 拓扑输出增加车道线黄白/虚实属性

v3.00.10 2024-02-18
1. 在拓扑修正停止线几何 和打断垂直
2. 修改link 绑定路面元素

v3.00.11 2024-02-18
1. fix停止线几何buf，正确修正方向
2. 修改原始的路面原始 dir 方向计算

v3.01.00 2024-02-19
1. 修复部分分合流
2. 默认开启拓扑后处理的几何

v3.01.01 2024-02-28 : (new: v3.01.01_qzc_03-01_09_47_42, base: v3.01.00)
1. 集成测试评估流程

v3.01.02 2024-03-03
1. 修改downloaddir方向计算
2. 修改交点计算函数，加了3种模式控制
3. 剔除停止线绑定到错误的link上，修复错误打断
4. 添加路口半径，路口进入退出动态搜索范围

v3.01.03 2024-03-04:(new:v3.01.03_cxf_03-04_17_06_12 base: v3.01.01_qzc_03-01_09_47_42)
1. 完成断点识别
2. 路口绑定进入退出link，剔除右转车道

v3.01.05 2024-03-04:(new:v3.01.05_huna_03-04_21_10_42 base: v3.01.03_cxf_03-04_17_06_12)
1. 修复平滑车道中心特征点的道路索引值问题
2. 修复filter_lane的代码问题
3. 优化黄线补充构建道路边界线
4. 优化lc的road_index的滤波逻辑

v3.01.06 2024-03-06:(new:v3.01.05_qzc_03-13_09_07_07 base: v3.01.05_huna_03-04_21_10_42)
1. 更新 bev pose

v3.01.07 2024-03-14:(new:v3.01.07_yx_03-14_09_07_07 base: v3.01.05_qzc_03-13_09_07_07)
1. 修复交通灯适配bug

v3.01.10 2024-03-18:(new:v3.01.10_huna_03-18_09_31_07 base: v3.01.07_yx_03-14_09_07_07)
1. 修复车道线无效拓扑ID
2. 修复车道线拓扑挂接错误
3. 修复因车道中心线丢失导致的单侧车道线丢失的问题

v3.01.11 2024-03-18
1. 停止线，人行横道，箭头预校正
2. 停止线双线处理
3. 人行横道粘连分割，矩形化
4. 箭头对齐
v3.01.10 2024-03-18:(new: v3.01.18_fa_03-18_20_12_53 base: v3.01.05_qzc_03-13_09_07_07)
1. fast_road_model 上线测试

v3.01.12 2024-03-17:(new:xx base:v3.01.07_yx_03-14_09_07_07)
1. 添加三角岛
2. 去除大部分重复车道线

v3.01.13 2024-03-16:
1. cxf 添加len参数控制get_cross_point_by_point、get_cross_point_with_polygon 线段选择延长的距离
2. 完成车道组覆盖度计算模块开发
3. 完成前后推 计算缺失点
4. 扩展第一遍while的时候， 车道线有，则补点

v3.01.14 2024-03-20:
1. 增加车道线虚实属性的拓扑打断

v3.01.16 2024-03-20:
1. 合并所有的代码

v3.03.00 2024-03-21:
1. 发布版本3.03.00

v3.03.01 2024-03-21:
1. 发布版本3.03.01,默认开启 fast_road_model

v3.03.02 2024-03-21:(new: v3.03.00_qzc_03-21_05_41_38 base: v3.01.10_qzc_03-19_00_15_54)
1. 自验证的时候，默认开启 评估，添加docker 版本

v3.03.03 2024-03-21:(new: v3.03.00_qzc_03-21_05_41_38 base: v3.01.10_qzc_03-19_00_15_54)
1. 修复三角岛link bug

v3.03.04 2025-03-22:(new: V3.3.0_20250330_mapping_autolabeling_v3.03.04 base:v3.03.00_qzc_03-21_05_41_38)
1. 修复线上shp lane缺失左右边界和，左右车道

v3.03.05 2025-03-25:(new: V3.3.0_20250330_mapping_autolabeling_v3.03.05 base:v3.03.00_qzc_03-21_05_41_38)
1. 新增道路边界type属性写入

v3.03.06 2025-03-24:(new: v3.03.06_ccj_03-27_03_19_25 base:v3.03.00_qzc_03-21_05_41_38)
1. 修复箭头旋转错误问题
2. 过滤不满足要求的停止线
3. 人行横道对齐停止线并修订距离停止线最近边到停止线距离到1~3m范围内

v3.03.07 2025-03-27:(new: v3.03.07_qzc_03-28_02_35_24 base:v3.03.06_ccj_03-27_03_19_25)
1. 添加右转专用道优化
2. 修复非主路口，车道线穿过停止线
3. 更新测试docker 到 v1.1.0

v3.03.8 2025-03-27:(new:v3.03.9_qzc_03-29_23_03_29  base:v3.03.07_qzc_03-28_02_35_24)
1. 修复拼图bug：导致很多右转中心线缺失 （定量的指标：和下面一个commit一起跑的结果）

v3.03.9 2025-03-27:(new:v3.03.9_qzc_03-29_23_03_29  base:v3.03.07_qzc_03-28_02_35_24)
1. 根据拼图bug修复后：重新调整右转策略
2. 更新 测试 docker 挂在路径
3. 修复crash
4. 优化右转2

v3.03.11 2025-03-31:(new:v3.03.12_qzc_04-02_23_14_20 base:v3.03.9_qzc_03-29_23_03_29)
1. 添加frame_id拼接输出shp
2. 优化export_shp.cpp，合并一些函数，obj创建字段重写，并精简代码
3. 修改交通灯顺序
4. 精简shp.cpp中的矢量化部分代码

v3.03.12 2025-04-02:(new:v3.03.12_qzc_04-02_23_14_20 base:v3.03.9_qzc_03-29_23_03_29)
1. 关闭一些日志
2. 打包建模的中间结果到平台，辅助分析问题
3. 检查建模的输入，如果缺数据，直接反馈信息到平台

v3.03.13 2025-04-02:(new:v3.03.13_ccj_04-09_17_50_39 base:v3.03.12_qzc_04-02_23_14_20)
1. 修复B平台数据人行横道无矢量化结果问题
2. 优化人行横道相对于停止线旋转和平移问题(功能完善)
3. 人行横道聚类之前先做体素滤波过滤重复点
4. 人行横道第二次聚类最小点数修改
5. road_model模块编译remove bev_road_model的依赖

v3.03.14 2025-04-10:(new:v3.03.15_cxf_04-10_15_58_43 base:v3.03.13_ccj_04-09_17_50_39)
1. 校验 通过车道组生成的停止线端点是否异常
2. 预防crash

v3.03.15 2025-04-10:(new:V3.3.0_20250330_mapping_autolabeling_v3.03.15 base:v3.03.13_ccj_04-09_17_50_39)
1. 发布新版本，上一个版本未编译拼图和矢量化

v3.03.16 2025-04-10:(new:v3.03.16_cxf_04-10_18_21_51 base:v3.03.13_ccj_04-09_17_50_39)
1. fix停止线端点矫正计算bug

v3.04.00 2025-04-02:(new:v3.04.00_yx_04-15_16_19_21 base:v3.03.16_cxf_04-10_18_21_51)
1. 增加对b平台lidar数据的适配

v3.04.01 2025-04-02:(new:v3.04.00_yx_04-15_16_19_21 base:v3.03.16_cxf_04-10_18_21_51)
1. 发布项目版本 3.4.0
0. =======下个版本需要添加的===========
1. 修改覆盖度结构体，指回原来线对象, 及add dir
2. 修复286,312 cal_cover crash
3. 添加第一次横推补点校验

v3.05.01 2025-05-07:(new: base:v3.04.00_yx_04-15_16_19_21)
1. 发布项目版本 3.5.0
2. 路口构组优化
3. crash 修复
4. 分合流优化v1

v3.05.02 2025-05-07:(new: base:v3.04.00_yx_04-15_16_19_21)
1. 关闭分合流

v3.06.00 2025-05-23:(newv3.06.00_qzc_05-23_02_17_59: base:V3.5.0_20250514_mapping_autolabeling_v3.05.02)
1. 优化右转
2. 分合流宽度不准
3. 分合流横向错位
4. 连8米碎线（中心线）
5. 连断裂的道路边界线（4m）
6. shp格式修改：FRAME_ID 為 ALG_ID

v3.06.01 2025-05-24:(new:V3.6.0_20250514_mapping_autolabeling_v3.06.01: base:v3.06.00_qzc_05-23_02_17_59)
1. 更新 bev pose

v3.06.02 2025-05-24:(new: base:V3.6.0_20250514_mapping_autolabeling_v3.06.01)
1. 添加分合流区间过滤其他断点
2. 修正分合流端点的朝向

v3.06.03 2025-05-24:(new: base:V3.6.0_20250514_mapping_autolabeling_v3.06.01)
1. 修正分合流端点的朝向(和原始点过近)

v3.06.04 2025-05-28:(new: base:V3.6.0_20250514_mapping_autolabeling_v3.06.01)
1. 添加车道线补线
2. 添加smooth操作

v3.06.05 2025-05-28:(new:V3.6.0_20250530_mapping_autolabeling_v3.06.05 base:V3.5.0_20250514_mapping_autolabeling_v3.05.02)
1. fix crash inner_link_poly_50m
2. 加强前后推逻辑，只连直线，分合流处不连

v3.06.06 2025-05-28:(new:V3.6.0_20250530_mapping_autolabeling_v3.06.05 base:V3.5.0_20250514_mapping_autolabeling_v3.05.02)
1. 修复人行横道生成路口框bug

v3.06.07 2025-06-04:(new:v3.06.07_qzc_06-10_17_13_46 base:V3.6.0_20250530_mapping_autolabeling_v3.06.05)
1. 更新评估docker
2. 更新评估数据集为：自研bev和激光

v3.06.10 2025-06-10:(new:v3.06.10_cxf_06-12_09_04_47 base:v3.06.07_qzc_06-10_17_13_46)
1. 重构车道组覆盖度

v3.06.11 2025-06-12:(new:v3.06.11_qzc_06-12_11_42_54 base:v3.06.10_cxf_06-12_09_04_47)
1. 过滤杂线

v3.06.12 2025-06-12:(new:v3.06.12_qzc_06-13_11_45_21 base:v3.06.11_qzc_06-12_11_42_54)
1. 融合BEV&激光

v3.06.13 2025-06-12:(new:v3.06.13_qzc_06-16_13_14_15 base:v3.06.11_qzc_06-12_11_42_54)
1. 修复地图缺失
2. 适配 byd_bd

v3.06.14 2025-06-18:(new: base:v3.06.13_qzc_06-16_13_14_15)
1. 停止线、人行横道、箭头使用激光的结果，其余使用融合的结果
2. 生成底图

v3.06.15 2025-06-18:(new: base:v3.06.13_qzc_06-16_13_14_15)
1. 生成底图(带颜色)

v3.06.16 2025-06-19:(new: base:v3.06.13_qzc_06-16_13_14_15)
1. 全局底图去掉中心线
2. 单帧bev的高度设置为0

v3.06.17 2025-06-19:(new: base:v3.06.13_qzc_06-16_13_14_15)
1. 为了方便后续作业，bev矢量和底图颜色进行互补

v3.06.18 2025-06-19:(new: base:v3.06.13_qzc_06-16_13_14_15)
1. 降低拼图点云的大小

v3.06.19 2025-06-19:(new:v3.06.19_qzc_06-20_20_56_44 base:v3.06.13_qzc_06-16_13_14_15)
1. 修复激光点云加载失败

v3.06.22 2025-06-21:(new:v3.06.22_qzc_06-21_09_45_20 base:v3.06.19_qzc_06-20_20_56_44)
1. 完成前后推转线， 
2. 前后推完之后， 在做分合流识别

1. 优化前后推命中点数
2. 优化真实车道宽度替代经验值计算
3. 删除断线，前后推完再做分合流
4. 解决候选点校验bug
5. 解决fix_disconnect把横推的短线误删
6. fix可视化bug

1. 优化起点，不用经验值50m
2. 修复横推地方前后推补点失败bug
3. 前后推优化到能连30m
4. 剔除分合流误识别，导致的车道线挂接不上

v3.06.23 2025-06-19:(new:v3.06.23_yx_06-21_19_43_48 base:v3.06.22_qzc_06-21_09_45_20)
1. 右转重构
2. 修复边界线/车道线的属性

v3.06.24 2025-06-22:(new:v3.06.24_qzc_06-22_16_25_53 base:v3.06.23_yx_06-21_19_43_48)
1. 解决夜间数据权重系数问题
2. 解决箭头矢量化coredump问题导致矢量化结果缺失问题
3. 选元素附近距离最近的10条车道线主方向对元素进行旋转,并优化相关参数
4. 元素(停止线,人行横道)与附近车道线主方向在[25, 35]度范围的时候不做校正
5. 优化停止线参数，输出阶梯停止线

v3.06.25 2025-06-23:(new: base:v3.06.24_qzc_06-22_16_25_53)
1. 修复右转异常卡死
2. 去除掉头部分的轨迹
3. 去掉重复帧
4. 加载感知分合流（关闭）
5. 关闭分合流

v3.8.00 2025-06-23:(new:v3.8.00_qzc_06-23_03_52_13 base:v3.06.24_qzc_06-22_16_25_53)
1. 发布版本 3.8.0

v3.8.01 2025-06-23:(new:v3.8.01_qzc_06-23_09_31_30 base:v3.06.19_qzc_06-20_20_56_44)
1. 回退人行横道误删

v3.8.02 2025-06-23:(new:V3.8.0_20250630_mapping_autolabeling_v3.8.02 base:v3.06.19_qzc_06-20_20_56_44)
1. 激光数据人行横道不做矫正

v3.8.03 2025-06-23:(new:V3.8.0_20250630_mapping_autolabeling_v3.8.03 base:v3.06.07_qzc_06-10_17_13_46)
1. 人行横道缺失

v3.8.04 2025-06-23:(new:v3.8.04_yx_06-24_15_32_34 base:V3.8.0_20250630_mapping_autolabeling_v3.8.03)
1. 右转和直行的重叠
2. longer link迭代bug修复
3. boundary找端点bug修复
4. 过滤异常过长link

v3.8.05 2025-06-25:(new:v3.8.05_yx_06-25_20_20_20 base:v3.8.04_yx_06-24_15_32_34)
1. 修复右转内部构组重叠
2. 开启右转-直行重叠打断功能

v3.8.06 2025-06-27:(new: base:v3.8.05_yx_06-25_20_20_20)
1. 分合流打断对齐
2. 分合流初始点优化，并增加拟合点

v3.8.07 2025-06-27:(new:v3.8.07_cxf_07-07_04_29_42 base:v3.8.05_yx_06-25_20_20_20)
1. 属性映射
2. 分合流检测：车道宽度检查3米 和 移动初始位置到车道数变化处
3. 通过右转link点，剔除无匹配的分合流
4. 重构分合流：适配分合流内的车道线折线
5. 修复分合流 point_status = 1的bug
6. 剔除20米内长度的中心线：该线起始终点和其他中心线相距2米内
待办项
1. 直行没有和分合流对齐
2. 相邻两处分合流重叠需要移动前一个pe点到后一个的ps点 
3. 匹配投影需要加保护，防止丢线
4. 剔除20米内长度的中心线：该线起始终点和其他中心线相距2米内 或者 2米内的比例为10%也删除
5. 剔除路中右转豁口
6. 中心线超出道路边界，需要切割道路边界线

v3.8.8 2025-07-08:(new: base:v3.8.07_cxf_07-07_04_29_42)
1. 新增加的车道线、边界线、中心线属性和类型错误修复
2. 重构分合流部分函数和结构体定义：get_cross_point_with_curve_segment 和 CrossPoint

v3.8.9 2025-07-09:(new:v3.8.9_qzc_07-09_10_45_53 base:v3.8.07_cxf_07-07_04_29_42)
1. 修复：直行没有和分合流对齐
2. 修复双向道车道线反向问题，需等双向道模型开发完才好调试

v3.9.0 2025-07-09:(new:V3.9.0_20250715_mapping_autolabeling_v3.9.0 base:v3.8.9_qzc_07-09_10_45_53)
1. 修复：直行没有和分合流对齐
2. 修复双向道车道线反向问题，需等双向道模型开发完才好调试