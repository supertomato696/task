## 编译第三方库
执行sh build_3rd.sh，编译结束后，三方库的头文件和lib文件会保存在3rd_output目录内，如果部署的机器操作系统版本没有发生变化，可以不用重新编译，把3rd_output复制到对应机器上即可

## 编译可执行文件
执行sh build.sh得到可执行文件

## Docker
生成镜像：sudo docker build -t mapbuilding .
进入镜像：sudo docker run -it mapbuilding /bin/bash

## 调用镜像
sudo docker run -it -v /home:/home/task_demo_20241210 -e "in_path=/home/task_demo_20241210/task_demo_20241210" -e "out_path=/home/task_demo_20241210/output" mapbuilding
