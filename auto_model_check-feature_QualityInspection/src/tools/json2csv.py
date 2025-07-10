import os
import json
import csv
from src.logger import logger
from pathlib import Path


def json2csv(output_path, json_file_path):
    with open(json_file_path, 'r', encoding='utf-8') as f:
        data = json.load(f)

    # 获取json文件时提取文件名，在同样路径下创建其csv
    csv_file_name = f"{Path(json_file_path).stem}.csv"
    csv_file_path = os.path.join(output_path, csv_file_name)
    # 获取所有评估类型
    first_batch = next(iter(data["result"].items()))[1]
    all_eval_types = first_batch.keys()

    # 准备 CSV 文件头
    csv_headers = ["算法框ID"]

    # 添加评估类型到文件头
    for eval_type in all_eval_types:
        if eval_type == "crosswalk_eval":
            csv_headers.append(f"{eval_type}_输入面积准确率")
            csv_headers.append(f"{eval_type}_输入面积召回率")
        else:
            csv_headers.append(f"{eval_type}_准确率")
            csv_headers.append(f"{eval_type}_召回率")
        if eval_type == "crosswalk_eval":
            csv_headers.extend(
                [f"{eval_type}_整体平均H距离(米)", f"{eval_type}_被匹配真值个数/输入个数/真值个数", f"{eval_type}_整体IOU"])
        elif eval_type == "arrow_eval":
            csv_headers.append(f"{eval_type}_F1分数")
            csv_headers.extend(
                [f"{eval_type}_整体平均H距离(米)", f"{eval_type}_被匹配真值个数/输入个数/真值个数", f"{eval_type}_整体IOU"])
        else:
            csv_headers.append(f"{eval_type}_F1分数")

    # 写入 CSV 文件
    with open(csv_file_path, 'w', newline='', encoding='utf-8') as csvfile:
        writer = csv.writer(csvfile, quoting=csv.QUOTE_NONNUMERIC)
        writer.writerow(csv_headers)

        # 遍历结果数据
        for batch, evaluations in data["result"].items():
            row = [batch]
            for eval_type in all_eval_types:
                eval_data = evaluations.get(eval_type, {})
                if eval_type == "crosswalk_eval":
                    row.append(eval_data.get("输入面积准确率", ""))
                    row.append(eval_data.get("输入面积召回率", ""))
                else:
                    row.append(eval_data.get("准确率", ""))
                    row.append(eval_data.get("召回率", ""))
                if eval_type == "crosswalk_eval":
                    row.append(eval_data.get("整体平均H距离(米)", ""))  # 假设这是整体平均H距离
                    row.append('="' + eval_data.get("被匹配真值个数/输入个数/真值个数", "") + '"')
                    row.append(eval_data.get("整体IOU", ""))
                elif eval_type == "arrow_eval":
                    row.append(eval_data.get("F1分数", ""))
                    row.append(eval_data.get("整体平均H距离(米)", ""))  # 假设这是整体平均H距离
                    row.append(eval_data.get("被匹配真值个数/输入个数/真值个数", "") + '"')
                    row.append(eval_data.get("整体IOU", ""))
                else:
                    row.append(eval_data.get("F1分数", ""))
            writer.writerow(row)

    logger.info(f"CSV 文件 {csv_file_name} 已生成。")


if __name__ == "__main__":
    json_file_path = r'D:\geoeva_develop\data\output\v3.03.00_byd_168.93s.json'
    csv_file_root = r'D:\geoeva_develop\data\output'
    json2csv(csv_file_root, json_file_path)
