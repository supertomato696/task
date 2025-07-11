# -*- coding: utf-8 -*-
# @Time    : 2025/6/5 21:07
# @Author  : StephenLeung
# @Email   : liang.yuhao1@byd.com
# @File    : line_process.py


from typing import List, Union, Tuple, Dict, Any

import numpy as np
from geopandas import GeoDataFrame
from numpy import ndarray
from shapely.geometry import LineString, MultiLineString, Point, Polygon
from shapely.strtree import STRtree
from shapely.wkb import loads as wkb_loads

from src.logger import logger
from src.quality_inspector.utils.utils import extract_line_intersection_points


class LinePreprocessor:
    """
    负责线几何对象的预处理，核心功能是将输入的线要素切分成指定长度的小段，并输出为gdf
    """

    @staticmethod
    def turn_line_to_segments(line_gdf: GeoDataFrame, segment_length: float = 1.0) -> GeoDataFrame:
        """
        将 GeoDataFrame 中的每条线（LineString 或 MultiLineString）切分为近似
        定长的小段。

        此方法要求输入的 `line_gdf` 必须包含一个名为 'ID' 的列，用作原始线要素的
        稳定标识符。

        切分后的子线段会保留原始 GeoDataFrame 的所有属性列（包括 'ID'），
        并额外添加一个 'segment_index' 列。该列表示此子段是其来源原始特征
        (无论 LineString 或 MultiLineString) 切分出的第几个子段 (0-based index)。
        对于 MultiLineString，索引会在其所有部分之间连续递增。
        非线状几何类型或空几何将被自动忽略。

        Args:
            line_gdf (GeoDataFrame): 输入的包含线要素的 GeoDataFrame。
                                     **必须包含 'ID' 列。**
                                     其 geometry 列应包含 LineString 或 MultiLineString 对象。
            segment_length (float): 目标切分长度（单位与 line_gdf 的 CRS 单位一致，
                                     通常是米）。默认为 1.0。

        Returns:
            GeoDataFrame: 一个新的 GeoDataFrame，包含所有切分后的 LineString 子段。
                          列结构为原始列 (包含 'ID') + 'segment_index'。
                          每个子段的 'ID' 列值与其来源的原始线要素的 'ID' 值相同。
                          如果没有任何线段可以被切分，将返回一个具有相同列结构但没有行的空 GeoDataFrame。
                          CRS 与输入的 line_gdf 保持一致。

        Raises:
            ValueError: 如果输入的 `line_gdf` 中不存在 'ID' 列。
        """
        if 'ID' not in line_gdf.columns:
            raise ValueError("输入的 GeoDataFrame 必须包含一个名为 'ID' 的列。")

        all_split_rows: List[dict] = []

        for idx, row in line_gdf.iterrows():
            geom = row.geometry
            original_id = row['ID']
            # --- 新增：为当前原始特征（行）初始化 segment 计数器 ---
            segment_counter_for_row = 0

            lines_to_process: List[LineString] = []
            # 统一处理 LineString 和 MultiLineString，得到 parts 列表
            if isinstance(geom, LineString):
                if not geom.is_empty:
                    lines_to_process = [geom]
            elif isinstance(geom, MultiLineString):
                # 过滤掉空的 parts
                logger.debug("捕捉到输入Multilinestring")
                lines_to_process = [part for part in geom.geoms if not part.is_empty]
            else:
                # logger.debug(f"ID {original_id} (行索引 {idx}) geometry 类型 {type(geom)} 非线状，跳过")
                continue  # 跳过非线状几何

            # 遍历该原始特征的所有有效 LineString parts
            for line_part in lines_to_process:
                try:
                    segments = LinePreprocessor.__split_line_high_performance(line_part, segment_length)
                except (ValueError, TypeError) as e:
                    # logger.error(f"ID {original_id} (行索引 {idx})： 切分部分时出错: {e}，跳过该部分")
                    continue  # 跳过切分失败的部分

                # --- 修改点：遍历 segments，使用并递增行级计数器 ---
                for seg in segments:
                    new_row = row.to_dict()
                    new_row['geometry'] = seg
                    # --- 修改点：使用行级计数器，并重命名列 ---
                    new_row['segment_index'] = segment_counter_for_row
                    all_split_rows.append(new_row)
                    # --- 修改点：递增行级计数器 ---
                    segment_counter_for_row += 1

        # 如果没有有效子段，返回空的 GeoDataFrame，保留列结构
        if not all_split_rows:
            # logger.warning("没有生成任何切分后的线段。")
            # --- 修改点：更新列定义 ---
            cols = list(line_gdf.columns) + ['segment_index']
            return GeoDataFrame([], columns=cols, crs=line_gdf.crs)

        # 构建新的gdf并且继承原始crs
        return GeoDataFrame(all_split_rows, crs=line_gdf.crs)

    @classmethod
    def __split_line_high_performance(
            cls, line: Union[LineString, MultiLineString],
            segment_length: float,
            min_segment_length: float = 1e-8
    ) -> List[LineString]:
        """
        将单个LineString或MultiLineString对象切分为多个LineString子段的高性能实现。

        核心优化思路:
        1. 使用 NumPy 进行向量化计算，避免 Python 循环处理坐标点。
        2. 批量计算线段长度和累积长度。
        3. 通过 `np.searchsorted` 快速定位切分点所在的原线段索引。
        4. 计算切分点精确坐标。
        5. 收集坐标并构建新的 LineString 对象。

        Args:
            line (Union[LineString, MultiLineString]): 输入的线几何对象。
            segment_length (float): 目标子段长度。
            min_segment_length (float): 允许的最小子段长度。用于避免处理过短的线
                                        或产生无效的切分结果。默认为 1e-8。

        Returns:
            List[LineString]: 切分后得到的 LineString子段列表。如果输入线过短或为空，
                              可能返回空列表。

        Raises:
            ValueError: 如果segment_length小于等于min_segment_length。
            TypeError: 如果输入line不是 LineString或 MultiLineString类型。
        """
        # 参数验证
        if segment_length <= min_segment_length:
            raise ValueError(f"切分长度({segment_length}) 必须大于{min_segment_length}米")

        # 内部处理函数，针对NumPy坐标数组
        def process_line(coords: np.ndarray) -> List[LineString]:
            if len(coords) < 2:
                return []

            # 计算各段向量和长度
            vectors = np.diff(coords, axis=0)
            segment_lengths = np.hypot(vectors[:, 0], vectors[:, 1])  # 等价于 np.sqrt(vectors[:, 0]**2 + vectors[:, 1]**2)
            # 过滤掉长度过小的段，避免后续计算问题
            valid_mask = segment_lengths > min_segment_length
            if not np.any(valid_mask):  # 如果所有段都过短
                if LineString(coords).length > min_segment_length:  # 但总长不短，则整体返回
                    return [LineString(coords)]
                else:
                    return []

            # 只处理有效段
            coords = np.vstack((coords[:-1][valid_mask], coords[-1]))  # 保留有效段的起点和最后一个点
            vectors = np.diff(coords, axis=0)
            segment_lengths = np.hypot(vectors[:, 0], vectors[:, 1])

            if len(coords) < 2:  # 过滤后可能少于2个点
                return []

            cum_lengths = np.insert(np.cumsum(segment_lengths), 0, 0)  # 累积长度，首位补0
            total_length = cum_lengths[-1]

            if total_length <= segment_length:  # 如果总长不足一个切分段
                return [LineString(coords)]

            # 预计算所有切分点应该在的距离值
            split_distances: ndarray = np.arange(segment_length, total_length, segment_length)
            split_points_data = []  # 存储 (原顶点索引, 切分点坐标)

            # 批量查找切分点落在哪个原始线段上
            # 'right' 表示如果距离等于某个累积长度，则认为它落在下一个线段的起点
            indices: ndarray = np.searchsorted(cum_lengths, split_distances, side='right') - 1

            # 计算每个切分点的精确坐标
            for i, dist in enumerate(split_distances):
                idx = indices[i]
                if idx >= len(segment_lengths):  # 超出范围则跳过（理论上arange已处理，增加健壮性）
                    continue

                # 计算切分点距离该段起点的长度
                remaining_dist = dist - cum_lengths[idx]

                # 防止除以零（虽然前面过滤了，但浮点数精度可能导致问题）
                if segment_lengths[idx] < min_segment_length:
                    continue

                # 计算插值比例
                ratio = remaining_dist / segment_lengths[idx]
                # 计算切分点坐标：起点坐标 + 方向向量 * 比例
                split_point_coords = coords[idx] + ratio * vectors[idx]
                split_points_data.append((idx, split_point_coords))

            # 构建结果线段
            segments = []
            start_coord_idx = 0  # 当前子段的起始坐标在原始 coords 中的索引
            current_coords = [coords[0]]  # 当前正在构建的子段的坐标列表

            for original_segment_idx, point_coords in split_points_data:
                # 添加从上一个切分点（或起点）到当前切分点之间的所有原始顶点
                current_coords.extend(coords[start_coord_idx + 1: original_segment_idx + 1].tolist())
                # 添加切分点
                current_coords.append(point_coords.tolist())
                # 确保至少有两个点才能构成 LineString
                if len(current_coords) >= 2:
                    segments.append(LineString(current_coords))

                # 开始构建下一个子段
                current_coords = [point_coords.tolist()]  # 新子段的起点是上一个子段的终点（切分点）
                start_coord_idx = original_segment_idx  # 更新下一个子段查找原始顶点的起始索引

            # 添加最后一段（从最后一个切分点到原始线的终点）
            current_coords.extend(coords[start_coord_idx + 1:].tolist())
            if len(current_coords) >= 2:
                segments.append(LineString(current_coords))

            # 过滤掉长度过短的最终结果段
            return [seg for seg in segments if seg.length > min_segment_length]

        # --- 主处理流程 ---
        if isinstance(line, LineString):
            if line.is_empty or line.length <= min_segment_length:
                return []
            return process_line(np.array(line.coords))

        elif isinstance(line, MultiLineString):
            all_segments = []
            for part in line.geoms:  # 遍历 MultiLineString 中的每个 LineString
                if not part.is_empty and part.length > min_segment_length:
                    all_segments.extend(process_line(np.array(part.coords)))
            return all_segments
        else:
            raise TypeError(f"输入几何类型必须是 LineString 或 MultiLineString，而非 {type(line)}")


class LineProcessor:

    @staticmethod
    def check_self_intersect(input: GeoDataFrame) -> Tuple[GeoDataFrame, GeoDataFrame]:
        if 'ID' not in input.columns:
            raise ValueError("输入的 GeoDataFrame 必须包含一个名为 'ID' 的列。")

        lines = input.copy()
        # 添加自相交标记
        lines['self_intersect'] = ~lines.geometry.is_simple
        # 过滤出自相交的线
        self_intersect_lines = lines[lines.self_intersect]
        # 提取自相交的交点
        records: List[Dict[str, Any]] = []

        for _, row in self_intersect_lines.iterrows():
            line_id = row['ID']
            line_geom: LineString = row.geometry

            if not line_geom or line_geom.is_empty:
                continue

            coords = list(line_geom.coords)

            # 一条线至少需要4个点（3个线段）才可能形成自相交点（例如最简单的"X"的一个分支）
            # s0, s1, s2 -> s0可以和s2相交
            if len(coords) < 4:
                continue

            segments: List[LineString] = []
            for i in range(len(coords) - 1):
                p1 = coords[i]
                p2 = coords[i+1]
                if p1 != p2:  # 避免创建零长度线段
                    segments.append(LineString([p1, p2]))

            # 至少需要两条非相邻线段才可能相交。
            # s0, s1, s2. s0 与 s2 是非相邻的。所以至少需要3个线段。
            if len(segments) < 3:
                continue

            # 使用集合存储当前线的交点WKB，以确保唯一性
            cur_line_interect_pts_wkb: set[bytes] = set()

            # 使用STRtree加速非相邻线段的相交查询
            tree = STRtree(segments)

            for i, seg1 in enumerate(segments):
                # 查询与seg1包围盒相交的线段索引
                # tree.query(seg1) 返回的是 segments 列表中的索引
                candidate_indices = tree.query(seg1)

                for j in candidate_indices:
                    # 条件确保我们只检查非相邻且唯一的线段对:
                    # j > i + 1  =>  seg2在seg1之后，并且seg2不与seg1相邻
                    if j <= i + 1:
                        continue

                    seg2 = segments[j]

                    tmp_pts = extract_line_intersection_points(seg1, seg2)
                    # 合并点集
                    cur_line_interect_pts_wkb = cur_line_interect_pts_wkb | tmp_pts

            # 将收集到的唯一交点（从WKB转换回Point）添加到总列表
            for point_wkb in cur_line_interect_pts_wkb:
                point_geom = wkb_loads(point_wkb)
                records.append({
                    'ID': line_id,
                    'geometry': point_geom
                })

        # 保存结果
        if records:
            self_intersect_pts = GeoDataFrame(records, geometry="geometry", crs=self_intersect_lines.crs)
        else:
            self_intersect_pts = GeoDataFrame(columns=['ID', 'geometry'], geometry="geometry", crs=self_intersect_lines.crs)
        
        return lines, self_intersect_pts
    
    @staticmethod
    def check_line_intersection(input: GeoDataFrame):
        if 'ID' not in input.columns:
            raise ValueError("输入的 GeoDataFrame 必须包含一个名为 'ID' 的列。")
        
        lines = input.copy()
        # 建立R-tree空间索引
        sindex = lines.sindex
        # 拆分提高效率
        geoms = lines.geometry.to_list()
        ids = lines['ID'].to_list()

        intersect_records = []
        intersect_flags = [False for _ in ids]

        for idx, (geom, pid) in enumerate(zip(geoms, ids)):
            # 筛选可能相交的线
            potential_intersects_idx = list(sindex.query(geom))
            # 过滤，已经对比过的线段
            potential_intersects_idx = [i for i in potential_intersects_idx if i > idx]
            
            for j in potential_intersects_idx:
                geom2 = geoms[j]
                # 获取这两根线的交点（已过滤正常相交的端点）
                tmp_intersect_pts = extract_line_intersection_points(geom, geom2)
                # 排除前驱后继挂接导致的合理相交
                endpoints = [geom.coords[0], geom.coords[-1], geom2.coords[0], geom2.coords[-1]] 

                for pts_wkb in tmp_intersect_pts:
                    pt_geom = wkb_loads(pts_wkb)
                    # 排除端点
                    if any(pt_geom.equals(Point(e)) for e in endpoints):
                        continue
                    intersect_records.append({
                        'id1': pid,
                        'id2': ids[j],
                        'geometry': pt_geom
                    })
                    intersect_flags[idx] = True
                    intersect_flags[j] = True
        
        lines['is_intersection'] = intersect_flags
        if intersect_records:
            intersect_pts = GeoDataFrame(intersect_records, geometry="geometry", crs=input.crs)
        else:
            intersect_pts = GeoDataFrame(columns=['id1', 'id2', 'geometry'], geometry="geometry", crs=input.crs)

        return lines, intersect_pts


    @staticmethod
    def validate_relation(input: GeoDataFrame,
                          line: LineString,
                          current_id: str,
                          current_point: Point,
                          related_id_list: str,
                          related_type: str,
                          buffer: Polygon):
        # 建立R-tree空间索引
        sindex = input.sindex
        exist_disconnect = []
        potential_connection = []

        if related_id_list is not None:
            # 查询相关要素，关联前驱后继可能有多个
            related_ids = related_id_list.split(',')
            for related_id in related_ids:
                related_row = input[input['ID'] == related_id]
                if not related_row.empty:
                    related_geom = related_row.geometry.iloc[0]
                    # 根据相关类型获取相关要素的端点
                    related_point = Point(related_geom.coords[-1]) if related_type == 'predecessor' else Point(
                        related_geom.coords[0])
                    # 检查几何关系
                    if current_point == related_point:
                        continue
                    elif not buffer.contains(related_point):
                        exist_disconnect.append({
                            'line_id': current_id,
                            'geometry': line,
                            'detail': f"要素 {current_id} 的 {related_type} 关系错误：{related_id} 要素的端点未连接到该要素的端点。"
                        })
                else:
                    exist_disconnect.append({
                        'line_id': current_id,
                        'geometry': line,
                        'detail': f"要素 {current_id} 的 {related_type} 要素 {related_id} 不存在。"
                    })
        else:
            # 如果没有相关 ID，使用空间索引查询缓冲区内的相交要素
            potential_matches_index = list(sindex.intersection(buffer.bounds))
            potential_matches = input.iloc[potential_matches_index]

            # 进一步筛选实际相交的要素，并排除自身
            potential_connect_lines = potential_matches[
                (potential_matches.geometry.intersects(buffer)) &
                (potential_matches['ID'] != current_id)  # 排除自身
                ]['ID'].tolist()
            if potential_connect_lines != []:
                potential_connection.append({
                    'line_id': current_id,
                    'geometry': line,
                    'potential_connect_lines': potential_connect_lines
                })    
        result = {
            'exist_disconnect': exist_disconnect,
            'potential_connection': potential_connection
        }
        return result


