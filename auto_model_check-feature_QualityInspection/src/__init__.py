# import platform
# import matplotlib.pyplot as plt
#
# # 根据操作系统设置字体
# system = platform.system()  # 获取操作系统名称
#
# if system == "Windows":
#     # Windows系统字体设置
#     plt.rcParams['font.sans-serif'] = [
#         'Microsoft YaHei',  # 微软雅黑
#         'SimHei'  # 黑体
#     ]
# elif system == "Linux":
#     # Linux 系统字体设置
#     plt.rcParams['font.sans-serif'] = [
#         'WenQuanYi Zen Hei',  # 文泉驿正黑
#         'Noto Sans CJK SC',  # 谷歌思源字体（需系统安装）
#         'AR PL UMing CN',  # AR PL 字体（部分 Linux 发行版预装）
#         'SimSun',  # 宋体，可能需要手动安装
#         'DejaVu Sans'  # Matplotlib 默认字体
#     ]
# else:
#     # 默认字体设置（其他未知系统）
#     plt.rcParams['font.sans-serif'] = ['DejaVu Sans']
