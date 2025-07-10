import argparse
from pathlib import Path

def rename_files(mapping: dict, root: Path, mode: str) -> None:
    """
    遍历 root 目录及其所有子目录，查找文件名中包含 mapping 中 key 的文件，
    根据 mode 重命名文件：
      - partial 模式：只重命名文件基本名称部分，保留原始扩展名。
      - full 模式：全替换整个文件名（包含扩展名）。
    如果某个文件名匹配了多个 key，则只应用第一个匹配规则。
    """
    if not root.exists():
        raise FileNotFoundError(f"指定目录不存在: {root}")

    for filepath in root.rglob("*"):
        if filepath.is_file():
            for key, new_value in mapping.items():
                if key in filepath.name:
                    if mode == "partial":
                        # 仅重命名名称部分，保留扩展名
                        original_ext = filepath.suffix
                        # 如果映射值中已经包含扩展名，则提取其 stem 部分
                        new_base = Path(new_value).stem
                        target_name = new_base + original_ext
                    else:
                        # full 模式，完全使用映射值重命名文件
                        target_name = new_value
                    target = filepath.with_name(target_name)
                    print(f"Renaming: {filepath} -> {target}")
                    try:
                        filepath.rename(target)
                    except Exception as e:
                        print(f"Error renaming {filepath} to {target}: {e}")
                    # 每个文件只处理一次（第一个匹配的 key 后退出循环）
                    break

def main():
    parser = argparse.ArgumentParser(
        description=(
            "根据给定的映射字典对指定目录下的所有文件进行重命名。\n"
            "当文件名中包含映射字典的 key 时，根据 --mode 参数决定重命名方式：\n"
            "  partial (默认)：仅重命名文件名部分，保留原扩展名；\n"
            "  full：全替换文件名，包括扩展名。"
        ),
        formatter_class=argparse.RawTextHelpFormatter
    )
    parser.add_argument("root", help="待遍历的根目录路径")
    parser.add_argument(
        "--mode",
        choices=["partial", "full"],
        default="partial",
        help="重命名模式：'partial' 表示只替换文件名部分（保留原扩展名），'full' 表示全替换。默认为 partial。"
    )
    args = parser.parse_args()

    # 示例映射字典，请根据需求修改
    mapping = {
        "boundary": "boundary",
        "lane_marking": "lane_marking",
        "laneline": "laneline",
        "lane": "lane",  # 不能放前边，会和lane_marking、laneline冲突
        "link": "link",
        "node": "node",
        "stopline": "stopline",
        "trafficlight": "trafficlight",
    }

    root_path = Path(args.root)
    rename_files(mapping, root_path, args.mode)

if __name__ == '__main__':
    main()