#!/bin/bash
# 这段脚本放在镜像里面编译

set -e  # 遇到错误就退出

# # 虚拟环境和可执行工具路径
# VENV_PATH=ven
# PYTHON="$VENV_PATH/bin/python"
# PYINSTALLER="$VENV_PATH/bin/pyinstaller"

# 如果不用虚拟环境，选择 Python（优先 python3），以及全局安装的 pyinstaller.
PYTHON=python3
if ! command -v "$PYTHON" &> /dev/null; then
  PYTHON=python
fi

if ! command -v pyinstaller &> /dev/null; then
  echo "ERROR: pyinstaller not found in PATH"
  exit 1
fi
PYINSTALLER=pyinstaller


# 输出目录
OUTPUT_DIR=bin

VERSION=$("$PYTHON" -c "import main; print(main.VERSION)" | tail -n1)

# 可执行文件名
BIN_NAME="GeoEva_${VERSION}_linux"

# 确保输出目录存在
mkdir -p "$OUTPUT_DIR"

HIDDEN_IMPORTS=$(awk -F'[=<>]' '/^[a-zA-Z0-9_\-]/ {print "--hidden-import="$1}' requirements.txt | xargs)


# 调用 PyInstaller
"$PYINSTALLER" \
    --onefile \
    --console \
    --noconfirm \
    "$HIDDEN_IMPORTS" \
    --hidden-import fiona \
    --hidden-import geojson \
    --collect-submodules matplotlib \
    --collect-submodules src \
    --add-data "src:src" \
    --exclude-module pyinstaller \
    --name "$BIN_NAME" \
    --distpath "$OUTPUT_DIR" \
    main.py

# 检查打包结果
if [ $? -eq 0 ]; then
    echo "BINARY_NAME: $BIN_NAME"
    echo "GeoEva BuildSuccess!"
    # 删除 PyInstaller 自动生成的 build 目录
    rm -rf build
else
    echo "GeoEva BuildFailed!"
    exit 1
fi