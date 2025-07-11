FROM 10.0.117.68/byd_test/bev_cover:v24

# 步骤1: 创建一个虚拟环境
RUN python3 -m venv /opt/venv

# 步骤2: 将venv的bin目录永久性地添加到 PATH
ENV PATH="/opt/venv/bin:$PATH"


# 安装所需的 Python 库
# 个安装，也可以用 pip install -r requirements.txt 的方式。
RUN pip install --no-cache-dir \
    loguru==0.7.2 \
    numpy==2.0.1 \
    pyproj==3.6.1 \
    geopandas==1.0.1 \
    shapely==2.0.4 \
    pydantic==2.8.2 \
    jira==3.8.0 \
    pandas==2.2.3 \
    GeoAlchemy2==0.15.1 \
    SQLAlchemy==1.4.54 \
    defusedxml~=0.7.1 \
    pillow~=11.0.0 \
    zipp~=3.20.2 \
    setuptools~=70.0.0 \
    pytz~=2024.2 \
    cffi~=1.17.1 \
    six~=1.16.0 \
    python-dateutil~=2.9.0.post0 \
    contourpy~=1.3.0 \
    fonttools~=4.54.1 \
    scipy~=1.13.1 \
    packaging~=24.1 \
    pyparsing~=3.2.0 \
    cycler~=0.12.1 \
    kiwisolver~=1.4.7 \
    fiona==1.10.1 \
    geojson~=3.1.0 \
    tqdm==4.67.1 \
    PyYAML~=6.0.2 \
    matplotlib~=3.9.0 \
    pandera~=0.24.0

# 设置工作目录为 /geoeva
WORKDIR /geoeva

# 将代码添加到镜像，放在最后方便后续修改
COPY . /geoeva

# 统一main.py作为入口
ENTRYPOINT ["python3", "main.py"]
CMD ["-t", "/mnt/tasks/model/label_cover"]



# docker build -t geoeva_qa:v1.5 .
# docker tag geoeva_qa:v1.5 10.0.117.68/byd_test/bev_cover:v21
# sudo docker push 10.0.117.68/byd_test/bev_cover:v21