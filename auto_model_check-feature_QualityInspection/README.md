# GeoEva QA 工具使用说明

---

## 项目传送门
[https://devops.byd.com/DNT/qa_team/geoeva.git](https://devops.byd.com/DNT/qa_team/geoeva.git)


## 版本列表

| 版本号 | 功能 | 备注 |
| --- | --- | --- |
| v1.3 | - GeoEva内部错误时仍然透传bev_cover_cal的分数。<br>- 配置文件都改为pydantic对象，测试ruff格式化插件。<br>- 确保无论如何评测步骤不会直接fail | |
| v1.2 | 适配建图平台输入<br>- 和覆盖度计算共用一个镜像，重构启动方式，适配建图覆盖度计算的输入。<br>- 基于覆盖度计算的基础镜像重新组织Dockerfile。<br>- 新增dotnet_runner负责调起覆盖度计算任务并获取其stdout。<br>- 调整匹配文件名的正则表达式。<br>- GeoEva本身的阻塞不导致任务流程失败，只是透传覆盖度计算的结果。 | |
| v1.1 | 主要提升了性能和可用性<br>- 自适应并行执行：根据项目数量实现了单进程/多进程切换，最大并发数为5，以充分利用资源。<br>- 支持“All”关键字：为 inspect_items 添加了“All”关键字，可快速运行所有已注册的任务。<br>- 扩展了一些错误码。<br>- 优化模块导入：在主函数和工作函数中实现了延迟导入，以减少内存占用和启动时间。<br>- 绘图器 DPI 调整：将Plotter保存图像时使用的DPI从300更改为150，有效减小文件大小并加快生成速度。<br>- 300秒超时控制：单个质检项执行超时时间设置为300秒。 | |
| v1.0 | 初版镜像，包含了6月初第一批质检项 | |

## 如何运行

1. **准备输入数据**  
   在您的本地机器（宿主机）上，创建一个工作目录，并按照以下结构组织您的输入文件。例如，在 `/path/to/your/data` 目录下：
   ```
   /path/to/your/data/
   └── geoEva/
       ├── input/
       │   ├── map_shp_v3.05.02.zip  # 输入的地图数据压缩包
       │   ├── tasks.json            # 定义QA任务的配置文件
       │   └── sd_link.geojson        # 包含SD-Link信息的地理数据文件
       └── output/                   # output目录，用于存放结果，如果没有也会自动创建
   ```

2. **获取 Docker 镜像**  
   从我们的仓库拉取最新的QA镜像：
   ```sh
   # GeoEvaQa Standalone
   docker pull 10.0.117.68/byd_geoeva/geoeva_qa:latest

   # GeoEvaQa ft. BevCoverCalc
   docker pull 10.0.117.68/byd_test/bev_cover:latest
   ```

3. **启动容器**  
   使用以下 `docker run` 命令启动容器，替换 `/path/to/your/data/geoEva/` 为您实际的路径：
   ```sh
   docker run -it --rm \
     -v /path/to/your/data/geoEva/:/mnt/tasks/geoEva \
     10.0.117.68/byd_geoeva/geoeva_qa:latest
   ```

   **命令参数解释**:
   - `-it`: 交互模式，方便查看实时日志。
   - `--rm`: 容器退出后自动删除。
   - `-v /path/to/your/data/geoEva/:/mnt/tasks/geoEva`: 挂载宿主机的 `geoEva` 目录到容器内部的 `/mnt/tasks/geoEva`。

## 输入数据结构详解

- **input/tasks.json**: JSON格式，指定需要执行的QA检查项。
- **input/*.zip**: 包含SHP格式的建图数据压缩包。
- **input/*.geojson**: 其他辅助地理空间数据。

## 预期输出

程序运行后，结果将生成在挂载的 `/path/to/your/data/geoEva/output/` 目录下，例如：
- `{{任务ID}}_{{算法框ID}}_log_count.csv`
- `{{任务ID}}_{{算法框ID}}_log_polygon_道路面自相交检测.gpkg`
- `output.json`

## 更改EntryPoint的用法

镜像的默认 `ENTRYPOINT` 是 `["python3", "main.py"]`，默认 `CMD` 是 `["-t", /mnt/tasks/geoEva]`。需要覆盖默认参数时，在 `docker run` 命令末尾添加参数：
```sh
docker run -it --rm \
  -v /path/to/your/data/geoEva/:/mnt/tasks/geoEva \
  10.0.117.68/byd_geoeva/geoeva_qa:v1.0 \
  -p /mnt/tasks/geoEva
```

## 标准输入示例

```json
{
    "expected_score": 80,
    "build_task_id": 9258,
    "check_integrity": false,
    "algorithm_box_id": 503276,
    "algorithm_polygon": "MULTIPOLYGON(((114.2846709556 22.6274311192,...,114.2846709556 22.6274311192)))",
    "algorithm_version": "V3.5.0",
    "algorithm_result": "503276_map_shp_v3.05.02_ID9258",
    "inspect_items": [
        "车道线自相交检测",
        ...
        "All"
    ]
}
```

## 标准输出示例

```json

{
    "error_code": 0,
    "build_task_id": "63762",
    "algo_box_id": "5661615",
    "algo_version": "v3.07.00",
    "expect_score": 0.8,
    "actual_score": 0,
    "message": "质检项配置数量为[],权重和为零。请检查",
    "bev_score": 0.8249,
    "status": 1
}
```

## 错误码一览

| 名称 | 值 |
| --- | --- |
| `SUCCESS` | 0 |
| `INIT` | 400 |
| `INTERNAL_ERROR` | 500 |
| `INVALID_PARAMETER` | 600 |
| `TIMEOUT` | 700 |
| `BAD_INTEGRITY` | 10001 |
| `PARTIAL_FAIL` | 20001 |

## 质检项一览

```json

# src.quality_inspector.__init__.py

DEFAULT_INSPECT_ITEMS = [
            "车道线悬挂点检测",
            "道路边界交叉检测",
            "车道线交叉检测",
            "停止线交叉检测",
            "车道线前驱后继关系检测",
            "道路边界前驱后继关系检测",
            "地面箭头自相交检测",
            "道路边界自相交检测",
            "人行横道自相交检测",
            "车道面自相交检测",
            "路口面自相交检测",
            "车道线自相交检测",
            "道路面自相交检测",
            "停止线自相交检测",
            "车道线折角检测",
            "道路边界折角检测",
            "道路面挂接车道缺失检测",
            "道路边界与道路面交叉检测",
            "车道左右边线长度差异过大检测",
            "道路面内车道与车道线数量差异过大检测"
        ]
```