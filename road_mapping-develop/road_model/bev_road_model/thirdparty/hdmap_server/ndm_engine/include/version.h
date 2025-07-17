#ifndef __VERSION_H_IN__
#define __VERSION_H_IN__

#define VERSION_MAJOR 3
#define VERSION_MINOR 3
#define VERSION_PATCH 7

/*
# v 3.3.1
data: 20210222
(1)接入高德地图，将高频输出改为INFO
(2)ndm_proto:section添加special situation 属性（包括收费站、检查站等类型）
(3)ndm_proto:添加DataHolder定义

# v 3.3.2
date: 20210225
(1)地图加载时判断是否存在bounding polygon
(2)修复ID索引失败时，使用野指针的问题

# v 3.3.3
date: 20210227
(1)component中的cpolygon、ccylinder、ccurveLine改为智能指针，解决内存上涨问题

# v 3.3.4
date: 20210303
(1)map_param改为ndm_engine

# v 3.3.5
date: 20210312
(1)ndm_proto:添加ODD,动态信息
(2)ndm_proto:添加应用导航信息
(3)ndm_proto:添加导航推荐标志位
(4)ndm_proto:添加数字导航相关信息
(5)ndm_proto:标志牌类型中添加注意故障车辆
(6)兼顾离线/在线加载地图，打通导航功能。

# v 3.3.6
date: 20210316
(1)修复导航目的地错误导致的异常崩溃问题
(2)新增点到点生成导航路径工具
(3)修复城区路段，因junction长度计算过大导致stubmap车前地图丢失问题
(4)修复UnpackComponent导致crash问题
(5)修改引擎与EHR的通信方式;
(6)ndm_proto:Location中更新error code的定义

# v 3.3.7
date: 20210324
(1)ndm_proto:添加曲线的高度参数方程, 用于表示如栅栏等有高度属性的曲线要素;
(2)Transform API中添加曲线高度参数的Transform;
(3)支持离线加载地图，在线加载导航模式;
(4)ndm_proto:添加锥柱桶的类型, Odometry中速度添加单位的备注;
(5)ndm_proto:ODD, TrafficEvent和TrafficFlow中添加ID, link改成数组的形式; 
(6)ndm_proto:section中新增长度属性;
(7)ndm_proto:LocationMsg中加入relative position;
(8)lane route更新时, 同步更新NavigationMsg中header的时间戳, 以及Navigation的时间戳;
(9)当逻辑关系跟丢时，通过空间搜索地图给定位输出数据
(10)RelativePosition变量名修正

# v 3.3.8
date: 20210401
(1)ndm_proto:RelativePosition中增加current_section信息, 以及current_lane改为index的形式; 
(2)ndm_proto:修改车道限速信息定义;
(3)ndm_proto:GlobalData中添加OTA状态位;
(4)ndm_proto:修改Location消息的定义;
*/

#endif // __VERSION_H_IN__




