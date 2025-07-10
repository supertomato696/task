import pytest
from src.tools.common import lat_lon_to_tile_id  # 导入要测试的函数


def test_lat_lon_to_tile_id():
    # 常规案例
    assert lat_lon_to_tile_id(30.23384, 120.14515, 13) == "6829_3373"

    # 边界条件
    assert lat_lon_to_tile_id(0, 0, 13) == "8192_4096"  # 原点
    assert lat_lon_to_tile_id(90, 180, 13) == "16383_8191"  # 北极
    assert lat_lon_to_tile_id(-90, -180, 13) == "0_16383"  # 南极

    # 错误输入（如果有相应的错误处理）
    with pytest.raises(ValueError):
        lat_lon_to_tile_id("invalid", "invalid")


if __name__ == "__main__":
    pytest.main()
