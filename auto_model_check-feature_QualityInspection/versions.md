1. 2025/2/27 geoeva_mapping_autolabeling v1.0.0
- 该工具能够对具有真值信息的6类道路要素（车道线、道路边界、人行横道、停止线、地面箭头、信号灯）进行几何精度评测。
- 可自动计算准确率、召回率等相关指标，分别反应输入数据命中真值的比例和真值数据被正确识别的比例。
- 所有指标评测结果汇总至表格：version_evaluate_results.csv

2. 2025/3/5 geoeva_mapping_autolabeling v1.0.5
- 更新：./src/topo_test/line_topo.py，用于检测车道线要素拓扑：
    （1）交叉车道线要素及交叉点位；
    （2）未正常挂接的悬挂点；
    （3）前驱后继属性挂接错误；
    （4）疑似应挂接但未挂接的前驱后继车道线。
- 拓扑检测结果追加至version_evaluate_results.csv表格中。

3. 2025/3/17 geoeva_mapping_autolabeling v1.0.7
- 更新：无实质代码更新，修改本文件，以测试harbor是否能够同步到腾讯镜像仓库

4.  2025/3/18 geoeva_mapping_autolabeling v1.0.9
- 更新：修复了linux环境中中文字体警告的问题

5. 2025/3/18 geoeva_mapping_autolabeling v2.0.0
- 使用：
    1. docker中自带一份config.yaml，用作参考，main.py中从挂载的data/input下寻找config.yaml
    2. ./data/algorithm_boxes中存储用以裁切待评测数据的凸包文件
    3. ./data/truth中存储v2真值，2/3数据保存在docker中，无需改动
- 更新：
    1. 适配了基于v2真值的评测，由于真值制作的规格及数据制作范围与v1真值相比差异较大，重新绘制了凸包
    2. 信号灯要素评测暂未实现
    3. 增添了车道相对宽度统计功能
    4. 暂不支持一次性输出mmt_rc和byd两组的结果，暂不支持统一的全要素csv统计输出

6. 2025/4/09 geoeva v2.0.5
为适配合规云argo流水线中集成特化
- 更新：
    1. 不再带真值。届时与输入值一起挂载。
    2. 信号灯要素评测实现
    3. 仅矢量化道路边界评测实现
    4. 车道相对宽度统计的柱状图样式调整
    5. 评测结果输出：json文件以及同名的csv
    6. 在算法框内，临时使用真值凸包再次剪裁输入的待评测要素 