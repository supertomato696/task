from dateutil import tz
import calendar
import shapely.geometry
import shapely
import datetime
from typing import Optional
import pyproj
import pymap3d
import hashlib
import json
import math
import os
import shutil
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from multiprocessing import cpu_count
from typing import *
import math
from threading import Timer
import psutil
import cv2
import numpy as np
import rosbag
import rospy
import sensor_msgs
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image, Imu
import scipy
from PIL import Image
import shapely.wkt
import shapely.ops
from shapely.geometry import Polygon, LineString
import pyproj
Image.MAX_IMAGE_PIXELS = None

project_root_folder = os.path.abspath(__file__+"/../../../")
sys.path.append(project_root_folder)
from common.util.log import log


# 飞书机器人
robot_url = "https://open.feishu.cn/open-apis/bot/v2/hook/508b719a-6f6a-4311-b198-574246505b65"  # 正式
# robot_url = "https://open.feishu.cn/open-apis/bot/v2/hook/ed8e8b25-45b6-453e-b532-f01b8c7657b8"  # 自测
zb_auto_robot_url = "https://open.feishu.cn/open-apis/bot/v2/hook/bc886b89-aa01-452c-aa71-30e7da8466dd"


def feishu_publish(task_folder: str):
    import requests
    import urllib3
    urllib3.disable_warnings()

    base_id = "bascneIVfkveLPLnzlHDMqRkUFe"
    table_id = "tblwIdeEKEMRyihW"
    app_token = ""

    def get_app_token():
        try:
            res = requests.post(url="https://open.feishu.cn/open-apis/auth/v3/tenant_access_token/internal",
                                headers={'Content-Type': 'application/json; charset=utf-8'},
                                data=json.dumps({"app_id": "cli_a200f14eef785013",  # app id
                                                "app_secret": "yoIyAbrEDXRYMNS8M3clZc26dwI1Rexg"}),  # app 密码
                                verify=False, timeout=(1, 10))
            res_data = json.loads(res.text)
            app_token = res_data["tenant_access_token"]
            return app_token
        except Exception as e:
            print("error in get_app_token: " + str(e))
            return ""

    def get_feishu_robot_img_key(image_path: str):
        try:
            import requests_toolbelt
            url = 'https://open.feishu.cn/open-apis/im/v1/images'
            form = {'image_type': 'message', 'image': (open(f'{image_path}', 'rb'))}
            multi_form = requests_toolbelt.MultipartEncoder(form)
            headers = {'Authorization': f'Bearer {app_token}', 'Content-Type': multi_form.content_type}
            response = requests.request("POST", url, headers=headers, data=multi_form, timeout=(1, 10))
            img_key = response.json()['data']['image_key']
            return img_key
        except Exception as e:
            print("error in get_feishu_robot_img_key: " + str(e))
            return ""

    def make_data_payload(task_info_dict: dict):
        data_dict = {"fields": {}}
        data_dict["fields"]["task_info_json"] = json.dumps(task_info_dict, ensure_ascii=False)
        data_dict["fields"]["task_id"] = task_info_dict["task_id"]
        data_dict["fields"]["version"] = task_info_dict["version"]
        data_dict["fields"]["tile_branch"] = task_info_dict["tile_branch"]
        data_dict["fields"]["road_branch"] = task_info_dict["road_branch"]
        data_dict["fields"]["tile_id_list"] = str(task_info_dict["tile_id_list"])
        data_dict["fields"]["error_code_text"] = str(task_info_dict["error_code"])+"_"+str(task_info_dict["error_text"])
        try:
            data_dict["fields"]["cost_minute"] = task_info_dict["log"]["cost_minute"]
            data_dict["fields"]["task_length_m"] = task_info_dict["log"]["task_length_m"]
            data_dict["fields"]["process_km_h"] = task_info_dict["log"]["process_km_h"]
            data_dict["fields"]["trail_num"] = task_info_dict["log"]["trail_num"]
            data_dict["fields"]["link_num"] = task_info_dict["log"]["link_num"]
            data_dict["fields"]["ram_max_g"] = task_info_dict["log"]["ram_max_g"]
            data_dict["fields"]["backload_cmd"] = task_info_dict["log"]["backload_cmd"]
            data_dict["fields"]["数据传输用时_m"] = task_info_dict["log"]["数据传输用时_m"]
            data_dict["fields"]["算法处理用时_m"] = task_info_dict["log"]["算法处理用时_m"]
            data_dict["fields"]["数据传输用时占比"] = task_info_dict["log"]["数据传输用时占比"]
            data_dict["fields"]["download_size_g"] = task_info_dict["log"]["download_size_g"]
            data_dict["fields"]["download_time_s"] = task_info_dict["log"]["download_time_s"]
        except Exception as e:
            print("error in download_size_g: " + str(e))
            pass
        payload = json.dumps(data_dict)
        return payload

    def get_table_id():
        try:
            url = f"https://open.feishu.cn/open-apis/bitable/v1/apps/{base_id}/tables?page_size=100"
            headers = {'Authorization': f'Bearer {app_token}'}
            payload = ''
            response = requests.request("GET", url, headers=headers, data=payload)
            res_dict = json.loads(response.text)
            for one_table in res_dict["data"]["items"]:
                if one_table["name"] == "最新":
                    return one_table["table_id"]
            return ""
        except Exception as e:
            print("error in get_tables: " + str(e))
            return ""

    app_token = get_app_token()
    if app_token == "":
        print("app_token 获取失败, 放弃飞书消息发送")
        return

    task_info_dict = json_file_to_dict(os.path.join(task_folder, "result.json"))
    image_list = [os.path.join(task_folder, "mapping_output/result.png"),
                  os.path.join(task_folder, "mapping_output/result_small.jpg"),
                  os.path.join(task_folder, "mapping_output/bev_road_seg.png")]

    try:
        getted_table_id = get_table_id()
        if getted_table_id != "":
            print("获取table id 成功", getted_table_id)
            table_id = getted_table_id
        else:
            print("获取table id 失败")
        headers = {'Authorization': f'Bearer {app_token}', 'Content-Type': 'application/json'}
        payload = make_data_payload(task_info_dict)
        tabel_url = f"https://open.feishu.cn/open-apis/bitable/v1/apps/{base_id}/tables/{table_id}/records"
        response = requests.request("POST", tabel_url, headers=headers, data=payload, timeout=(1, 10))
        print("更新任务信息库完成")
    except Exception as e:
        print("error in 更新任务信息库: " + str(e))

    try:
        data = {"msg_type": "post", "content": {"post": {"zh_cn": {"title": "", "content": []}}}}
        try:
            msg_image = []
            for one_path in image_list:
                print("上传机器人消息图片", one_path)
                image_token = get_feishu_robot_img_key(one_path)
                if image_token != "":
                    print("上传成功", image_token)
                    img = Image.open(one_path)
                    w = int(img.width/(img.height/100))
                    h = 100
                    msg_image.append({"tag": "img", "image_key": image_token, "width": w, "height": h})
                    break
                else:
                    print("上传失败")
                    time.sleep(2)
            data["content"]["post"]["zh_cn"]["content"].append(msg_image)
        except Exception as e:
            pass

        msg_text = [{"tag": "text", "text": json.dumps(task_info_dict, ensure_ascii=False)}]
        data["content"]["post"]["zh_cn"]["content"].append(msg_text)

        requests.post(robot_url,  # 机器人网址
                      headers={'Content-Type': 'application/json; charset=utf-8'},
                      data=json.dumps(data), verify=False, timeout=(1, 10))
        print("发送机器人消息完成")
    except Exception as e:
        print("error in 发送机器人消息: " + str(e))


def thread_pool(cpu: int, params: list):
    if cpu <= 0:
        cpu = cpu_count()

    with ThreadPoolExecutor(cpu) as executor:
        future_list = []
        for param in params:
            while True:
                try:
                    future = executor.submit(*param)
                except RuntimeError as err:
                    log.info("wait to start thread.")
                    time.sleep(5)
                    continue
                break
            future_list.append(future)


def qab_tab_to_Tab(qab: np.ndarray, tab: np.ndarray) -> np.ndarray:
    if (len(qab) == 0) or (len(tab) == 0):
        print("qab_tab_to_Tab 输入长度为0")
        return np.array([])
    Rab: np.ndarray = Rotation.from_quat(qab).as_matrix()
    Tab: np.ndarray = np.c_[Rab, tab]
    Tab: np.ndarray = np.r_[Tab, np.array([[0, 0, 0, 1]])]
    return Tab


def qab_to_Rab(qab: np.ndarray) -> np.ndarray:
    Rab: np.ndarray = Rotation.from_quat(qab).as_matrix()
    return Rab


def Rab_tab_to_Tab(Rab: np.ndarray, tab: np.ndarray) -> np.ndarray:
    Tab: np.ndarray = np.c_[Rab, tab]
    Tab: np.ndarray = np.r_[Tab, np.array([[0, 0, 0, 1]])]
    return Tab


def qab_to_Rba(qab: np.ndarray) -> np.ndarray:
    r: Rotation = Rotation.from_quat(qab)
    rinv: Rotation = r.inv()
    return rinv.as_matrix()


def Tab_to_qab(Tab: np.ndarray) -> np.ndarray:
    return np.array(Rotation.from_matrix(Tab[0:3, 0:3]).as_quat())


def Tab_to_tab(Tab: np.ndarray) -> np.ndarray:
    return Tab[0:3, 3:4].reshape(3)


def Tab_to_Rab(Tab: np.ndarray) -> np.ndarray:
    return Tab[0:3, 0:3]


def Tab_pbc_to_pac(Tab: np.ndarray, pbc: np.ndarray) -> np.ndarray:
    Rab: np.ndarray = Tab[0:3, 0:3]
    tab: np.ndarray = Tab[0:3, 3:4].reshape(3)
    return Rab.dot(pbc) + tab


def Tab_to_Tba(Tab: np.ndarray) -> np.ndarray:
    Rab: np.ndarray = Tab[0:3, 0:3]
    tab: np.ndarray = Tab[0:3, 3:4].reshape(3)
    Rba: np.ndarray = Rab.T
    tba: np.ndarray = -Rab.T.dot(tab)
    Tba: np.ndarray = np.c_[Rba, tba]
    Tba: np.ndarray = np.r_[Tba, np.array([[0, 0, 0, 1]])]
    return Tba


def Rab_to_qab(Rab: np.ndarray) -> np.ndarray:
    return np.array(Rotation.from_matrix(Rab).as_quat())


def q_slerp(q1: list, q2: list, rate: float) -> list:
    # rate 0 ~ 1，0 则输出 q1, 1 则输出q2
    q1 = np.array(q1)
    q1 = q1/np.linalg.norm(q1)
    q2 = np.array(q2)
    q2 = q2/np.linalg.norm(q2)
    if q1[3] < 0:
        q1 = -q1
    if q2[3] < 0:
        q2 = -q2
    cos_theta: float = q1.dot(q2)
    if cos_theta < 0:
        q1 = -q1
        cos_theta = q1.dot(q2)
    if cos_theta < -1:
        cos_theta = -1
    if cos_theta > 1:
        cos_theta = 1
    theta = math.acos(cos_theta)
    sin_theta = math.sin(theta)
    if sin_theta == 0:
        return q2
    # print("sin_theta {}".format(sin_theta))
    sin_1_t_theta = math.sin((1-rate)*theta)
    sin_t_theta = math.sin(rate*theta)
    q3: np.ndarray = (sin_1_t_theta/sin_theta)*q1+(sin_t_theta/sin_theta)*q2
    n = np.linalg.norm(q3)
    if n != 0:
        q3 = q3/n
    # print("q3 {}".format(q3))
    return q3.tolist()

def move_file(src='', dst=''):
    if not os.path.exists(src):
        log.error('copy_file(), no {}'.format(src))
        return
    folder = os.path.dirname(dst)
    if not os.path.exists(folder):
        os.makedirs(folder)
    shutil.move(src, dst)
    
def read_lines_from_file(filepath: str) -> List[str]:
    # 从文件中读所有行，写入字符串list中
    if not os.path.exists(filepath):
        return []
    lines = []
    file = open(filepath, 'r')
    lines0 = file.readlines()

    for line in lines0:
        lines.append(line.strip('\n'))
    return lines


def get_folder_path_in_file_path(file_path=''):
    return os.path.dirname(file_path)


def get_folder_name_in_path(path: str) -> str:
    if not os.path.exists(path):
        print('error, get_folder_name_in_path(), no {}'.format(path))
        return ''
    if os.path.isdir(path):
        return path.split('/')[-1]
    else:
        return path.split('/')[-2]


def write_lines_to_file_override(file_path: str, lines: list):
    folder = os.path.dirname(file_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    s = ''
    for l in lines:
        s += (str(l).strip('\n') + '\n')
    open(file_path, 'w').write(s.strip('\n'))


def json_file_to_dict(json_path: str) -> dict:
    if not os.path.exists(json_path):
        print("🔴 error 路径不存在", json_path)
        return {}
    data = {}
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            print('Reason: ' + str(e))
            return {}
    return data


def dict_to_json_file(json_path: str, dict: dict):
    def convert(x):
        if hasattr(x, "tolist"):
            return x.tolist()
        raise TypeError(x)
    folder = os.path.dirname(json_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    b = json.dumps(dict, default=convert, ensure_ascii=False)
    with open(json_path, 'w') as f:
        f.write(b)
        f.close()


def dict_to_json_file_format(json_path: str, dict: dict):
    def convert(x):
        if hasattr(x, "tolist"):
            return x.tolist()
        raise TypeError(x)
    folder = os.path.dirname(json_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    b = json.dumps(dict, default=convert, ensure_ascii=False,  indent=4, separators=(',', ":"))
    with open(json_path, 'w') as f:
        f.write(b)
        f.close()


def json_str_to_dict(json_str: str) -> dict:
    return json.loads(json_str)


def dict_to_json_str(dict: dict) -> str:
    def convert(x):
        if hasattr(x, "tolist"):
            return x.tolist()
        raise TypeError(x)
    return json.dumps(dict, default=convert)


def getFootPoint(point, line_p1, line_p2):
    """
    @point, line_p1, line_p2 : [x, y, z]
    """
    x0 = point[0]
    y0 = point[1]
    z0 = point[2]

    x1 = line_p1[0]
    y1 = line_p1[1]
    z1 = line_p1[2]

    x2 = line_p2[0]
    y2 = line_p2[1]
    z2 = line_p2[2]

    k = -((x1 - x0) * (x2 - x1) + (y1 - y0) * (y2 - y1) + (z1 - z0) * (z2 - z1)) / \
        ((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)*1.0

    xn = k * (x2 - x1) + x1
    yn = k * (y2 - y1) + y1
    zn = k * (z2 - z1) + z1

    return (xn, yn, zn)


def get_subfolders_path_with_prefix(root_folder_path='', prefix='') -> List[str]:
    subfolders_path = []
    if not os.path.exists(root_folder_path):
        log.error('get_subfolders_path_with_prefix(), no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        if name.startswith(prefix):
            fp = os.path.join(root_folder_path, name)
            if os.path.isdir(fp):
                subfolders_path.append(fp)
    return sorted(subfolders_path)


def get_subfolders_path_only_number_name(root_folder_path='') -> List[str]:
    subfolders_path = []
    if not os.path.exists(root_folder_path):
        log.error('get_subfolders_path_with_prefix(), no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        if name.isdigit():
            fp = os.path.join(root_folder_path, name)
            if os.path.isdir(fp):
                subfolders_path.append(fp)
    return sorted(subfolders_path)


def get_files_path_recursion_with_prefix(root='', prefix='') -> List[str]:
    if not os.path.exists(root):
        log.error('get_files_path_recursion_with_prefix(), no {}'.format(root))
        return []
    files_path = []
    for main_dir, dirs, file_name_list in os.walk(root):
        for name in file_name_list:
            if name.startswith(prefix):
                file_path = os.path.join(main_dir, name)
                files_path.append(file_path)
    return sorted(files_path)


def get_files_path_recursion_with_suffix(root='', suffix='') -> List[str]:
    if not os.path.exists(root):
        log.error('get_files_path_recursion_with_suffix(), no {}'.format(root))
        return []
    files_path = []
    for main_dir, dirs, file_name_list in os.walk(root):
        for name in file_name_list:
            if name.endswith(suffix):
                file_path = os.path.join(main_dir, name)
                files_path.append(file_path)
    return sorted(files_path)


def get_files_path_recursion(root='') -> List[str]:
    "递归的"
    if not os.path.exists(root):
        log.error('get_files_path_recursion(), no {}'.format(root))
        return []
    files_path = []
    for main_dir, dirs, file_name_list in os.walk(root):
        for name in file_name_list:
            files_path.append(os.path.join(main_dir, name))
    return sorted(files_path)


def get_files_path(root='') -> List[str]:
    if not os.path.exists(root):
        log.error('get_files_path(), no {}'.format(root))
        return []
    files_path = []
    file_list = os.listdir(root)
    for file_address in file_list:
        file_address = os.path.join(root, file_address)
        if os.path.isfile(file_address):
            files_path.append(file_address)
    return files_path


def get_subfolders_path(root_folder_path='') -> List[str]:
    subfolders_path = []
    if not os.path.exists(root_folder_path):
        log.error('get_folders_path, no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        fp = os.path.join(root_folder_path, name)
        if os.path.isdir(fp):
            subfolders_path.append(fp)
    return sorted(subfolders_path)


def run_a_process(cmd: str, timeout_s=None):
    import subprocess
    import shlex
    print("============================================")
    print(cmd)
    print("============================================")
    try:
        subprocess.run(shlex.split(cmd), timeout=timeout_s)
        return None  # 正常返回 None
    except Exception as e:
        print(e)
        return e  # 异常返回异常原因


def thread_pool(cpu_num: int, func_name_param_list: list):
    if cpu_num < 1:
        cpu_num = cpu_count()

    with ThreadPoolExecutor(cpu_num) as executor:
        future_list = []
        for param in func_name_param_list:
            while True:
                try:
                    future = executor.submit(*param)
                except Exception as err:
                    print("🔴 Error in thread_pool, wait to start thread."+err)
                    time.sleep(5)
                    continue
                break
            future_list.append(future)


def get_files_path_with_prefix(folder_path='', prefix='') -> List[str]:
    files_path = []
    if not os.path.exists(folder_path):
        print('error  get_files_path_with_prefix(), no {}'.format(folder_path))
        return []
    for name in os.listdir(folder_path):
        if not name.startswith(prefix):
            continue
        fp = os.path.join(folder_path, name)
        if os.path.isfile(fp):
            files_path.append(fp)
    return sorted(files_path)


def get_files_path_with_suffix(folder_path='', suffix='') -> List[str]:
    if not os.path.exists(folder_path):
        log.error('get_files_path_with_suffix(), no {}'.format(folder_path))
        return []
    files_path = []
    for name in os.listdir(folder_path):
        if name.endswith(suffix):
            fp = os.path.join(folder_path, name)
            if os.path.isfile(fp):
                files_path.append(fp)
    return sorted(files_path)


def get_file_name_in_file_path(filepath=''):
    return os.path.basename(filepath)


def get_relpath(abspath='', relto='') -> str:
    if len(relto) == 0:
        return ''
    return os.path.relpath(abspath, relto)


def get_str_md5_int64(string: str) -> int:
    md5_hex_str = hashlib.md5(string.encode()).hexdigest()
    md5_int128 = int(md5_hex_str, 16)
    md5_int64 = md5_int128 % (2 ** 64)
    return md5_int64


def add_folder_in_file_path_if_no_exist(filepath=''):
    folder = os.path.dirname(filepath)
    if not os.path.exists(folder):
        os.makedirs(folder)


def lonlat_to_tile_id(lon: float, lat: float) -> int:
    ix = int(lon/(90./float(1 << 30)))
    iy = int(lat/(90./float(1 << 30)))
    tile_id = int((ix >> 31) & 1)
    for i in range(30, 15, -1):
        tile_id = tile_id << 2
        tile_id = tile_id | ((((iy >> i) & 1) << 1) | ((ix >> i) & 1))
    return tile_id


def copy_file(src='', dst=''):
    if not os.path.exists(src):
        log.error('copy_file(), no {}'.format(src))
        return
    folder = os.path.dirname(dst)
    if not os.path.exists(folder):
        os.makedirs(folder)
    shutil.copyfile(src, dst)


def lonlatalt_utm_num_to_utm_xyz(lon, lat, alt, utm_num) -> List[float]:
    if not -80.0 <= lat <= 84.0:
        print('error lat is:' + str(lat))
        return [np.nan, np.nan]
    if not -180.0 <= lon <= 180.0:
        print('error lon is:' + str(lon))
        return [np.nan, np.nan]
    if not 1 <= utm_num <= 60:
        print('utm number out of range (must be between 1 and 60)')
        return [np.nan, np.nan]
    K0 = 0.9996
    E = 0.00669438
    E_P2 = 0.006739496752268451
    M1 = 0.9983242984503243
    M2 = 0.002514607064228144
    M3 = 2.6390466021299826e-06
    M4 = 3.418046101696858e-09
    R = 6378137
    lat_rad = np.radians(lat)
    lat_sin = np.sin(lat_rad)
    lat_cos = np.cos(lat_rad)
    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2
    lon_rad = np.radians(lon)
    central_lon = (utm_num - 1) * 6 - 180 + 3
    central_lon_rad = np.radians(central_lon)
    n = R / np.sqrt(1 - E * lat_sin ** 2)
    c = E_P2 * lat_cos ** 2
    a = lat_cos * ((lon_rad - central_lon_rad + np.pi) % (2 * np.pi) - np.pi)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a
    m = R * (M1 * lat_rad - M2 * np.sin(2 * lat_rad) +
             M3 * np.sin(4 * lat_rad) - M4 * np.sin(6 * lat_rad))
    utm_x = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) +
                      a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
    utm_y = K0 * (m + n * lat_tan *
                  (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) +
                   a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))
    if lat < 0:
        utm_y += 10000000
    return [utm_x, utm_y, alt]


def utm_xyz_num_to_lonlatalt(utm_x, utm_y, utm_z, utm_num) -> List[float]:
    if not 1 <= utm_num <= 60:
        print('zone number out of range (must be between 1 and 60)')
        return [np.nan, np.nan]
    K0 = 0.9996
    E = 0.00669438
    E_P2 = 0.006739496752268451
    M1 = 0.9983242984503243
    P2 = 0.00251882658811959
    P3 = 3.7009490465577744e-06
    P4 = 7.447770302991058e-09
    P5 = 1.7035993339288026e-11
    R = 6378137
    mu = (utm_y / K0) / (R * M1)
    p_rad = (mu + P2 * np.sin(2 * mu) + P3 * np.sin(4 * mu) +
             P4 * np.sin(6 * mu) + P5 * np.sin(8 * mu))
    p_sin = np.sin(p_rad)
    p_sin2 = p_sin * p_sin
    p_cos = np.cos(p_rad)
    p_tan = p_sin / p_cos
    p_tan2 = p_tan * p_tan
    p_tan4 = p_tan2 * p_tan2
    ep_sin = 1 - E * p_sin2
    ep_sin_sqrt = np.sqrt(1 - E * p_sin2)
    n = R / ep_sin_sqrt
    r = (1 - E) / ep_sin
    c = E_P2 * p_cos ** 2
    c2 = c * c
    d = (utm_x - 500000) / (n * K0)
    d2 = d * d
    d3 = d2 * d
    d4 = d3 * d
    d5 = d4 * d
    d6 = d5 * d
    lat = (p_rad - (p_tan / r) * (d2 / 2 - d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * E_P2)) +
           d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * E_P2 - 3 * c2))
    lon = (d - d3 / 6 * (1 + 2 * p_tan2 + c) +
           d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * E_P2 + 24 * p_tan4)) / p_cos
    lon = (lon + np.radians((utm_num - 1) * 6 - 180 + 3) + np.pi) % (2 * np.pi) - np.pi
    return [np.degrees(lon), np.degrees(lat), utm_z]


def np_right_ext_0(a) -> np.ndarray:
    a = np.array(a)
    return np.c_[a, np.zeros((a.shape[0], 1))]


def np_right_ext_1(a) -> np.ndarray:
    a = np.array(a)
    return np.c_[a, np.ones((a.shape[0], 1))]


def np_div_by_right_col(a: np.ndarray) -> np.ndarray:
    a = np.array(a)
    col_num = a.shape[1]
    b = a/a[:, col_num-1:col_num]
    return b


def get_folders_path_only_number_name(root_folder_path='') -> List[str]:
    subfolders_path = []
    if not os.path.exists(root_folder_path):
        log.error('get_subfolders_path_with_prefix(), no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        if name.isdigit():
            fp = os.path.join(root_folder_path, name)
            if os.path.isdir(fp):
                subfolders_path.append(fp)
    return sorted(subfolders_path)


def get_folders_int_number_only_number_name(root_folder_path='') -> List[str]:
    folders_int_number = []
    if not os.path.exists(root_folder_path):
        log.error('get_subfolders_path_with_prefix(), no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        if name.isdigit():
            folders_int_number.append(int(name))
    return sorted(folders_int_number)


def copy_folder(src_dir: str, dst_dir: str):
    if not os.path.exists(src_dir):
        log.error('copy_folder(), no {}'.format(src_dir))
        return
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)
    shutil.copytree(src_dir, dst_dir)


def add_folder_if_no_exist(folder_path: str):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)


def read_first_line_from_file(file_path: str) -> str:
    if not os.path.exists(file_path):
        return ''
    file = open(file_path, 'r')
    return file.readline().strip('\n')


def write_image(to_path: str, image: np.ndarray):
    folder = os.path.dirname(to_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    cv2.imwrite(to_path, image)


def gps_distance(lon1, lat1, lon2, lat2):
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])
    dlon = lon2 - lon1
    dlat = lat2 - lat1
    a = np.sin(dlat / 2) ** 2 + np.cos(lat1) * np.cos(lat2) * np.sin(dlon / 2) ** 2
    c = 2 * np.arcsin(np.sqrt(a))
    r = 6371  # 地球平均半径，单位为公里
    return c * r * 1000


def lonlat_to_utm_num(lon: float, lat: float) -> int:
    if 56 <= lat < 64 and 3 <= lon < 12:
        return 32
    if 72 <= lat <= 84 and lon >= 0:
        if lon < 9:
            return 31
        elif lon < 21:
            return 33
        elif lon < 33:
            return 35
        elif lon < 42:
            return 37
    return int((lon + 180) / 6) + 1


def undistort_many_image_with_new_K(raw_image_paths: List[str],
                                    un_image_paths: List[str],
                                    raw_k_fxycxy: List[float],
                                    raw_dist_k12p12k3456: List[float],
                                    new_w: int, new_h: int,
                                    new_k_fxycxy: List[float]):
    if len(raw_image_paths) == 0:
        print("输入图片路径个数为0")
        return

    if len(raw_image_paths) != len(un_image_paths):
        print("输入图片路径个数不等于输出图片路径个数")
        return

    raw_k: np.ndarray = np.array([[raw_k_fxycxy[0], 0.0, raw_k_fxycxy[2]],
                                  [0.0, raw_k_fxycxy[1], raw_k_fxycxy[3]],
                                  [0.0, 0.0, 1.0]])
    new_k: np.ndarray = np.array([[new_k_fxycxy[0], 0.0, new_k_fxycxy[2]],
                                  [0.0, new_k_fxycxy[1], new_k_fxycxy[3]],
                                  [0.0, 0.0, 1.0]])
    map_x, map_y = cv2.initUndistortRectifyMap(cameraMatrix=raw_k,
                                               distCoeffs=np.array(raw_dist_k12p12k3456),
                                               R=None,
                                               newCameraMatrix=new_k,
                                               size=(int(new_w), int(new_h)),
                                               m1type=5)
    done_num = [0]
    total_num = len(raw_image_paths)

    def _func(_map_x, _map_y, _raw_path, _un_path):
        if os.path.exists(_un_path):
            return
        if not os.path.exists(_raw_path):
            print("不存在图片 {}".format(_raw_path))
            return
        _raw_image = cv2.imread(_raw_path)
        if _raw_image is None:
            print("图片打开失败 {}".format(_raw_path))
            return
        _un_image = cv2.remap(_raw_image, _map_x, _map_y, cv2.INTER_LINEAR)
        write_image(_un_path, _un_image)
        done_num[0] += 1
        if done_num[0] % 100 == 0:
            print('图像去畸变进度 {}/{}'.format(done_num, total_num))

    thread_param = []
    for i in range(len(raw_image_paths)):
        thread_param.append([_func, map_x, map_y, raw_image_paths[i], un_image_paths[i]])
    thread_pool(-1, thread_param)


def cv_keep_max_region(mask: np.ndarray):
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_NONE)
    area = []
    for j in range(len(contours)):
        area.append(cv2.contourArea(contours[j]))
    max_idx = np.argmax(area)
    max_area = cv2.contourArea(contours[max_idx])
    for k in range(len(contours)):
        if k != max_idx:
            cv2.fillPoly(mask, [contours[k]], 0)
    return mask


def utm_xy_num_to_lonlat(utm_x, utm_y, utm_num) -> List[float]:
    if not 1 <= utm_num <= 60:
        print('zone number out of range (must be between 1 and 60)')
        return [np.nan, np.nan]
    K0 = 0.9996
    E = 0.00669438
    E_P2 = 0.006739496752268451
    M1 = 0.9983242984503243
    P2 = 0.00251882658811959
    P3 = 3.7009490465577744e-06
    P4 = 7.447770302991058e-09
    P5 = 1.7035993339288026e-11
    R = 6378137
    mu = (utm_y / K0) / (R * M1)
    p_rad = (mu + P2 * np.sin(2 * mu) + P3 * np.sin(4 * mu) +
             P4 * np.sin(6 * mu) + P5 * np.sin(8 * mu))
    p_sin = np.sin(p_rad)
    p_sin2 = p_sin * p_sin
    p_cos = np.cos(p_rad)
    p_tan = p_sin / p_cos
    p_tan2 = p_tan * p_tan
    p_tan4 = p_tan2 * p_tan2
    ep_sin = 1 - E * p_sin2
    ep_sin_sqrt = np.sqrt(1 - E * p_sin2)
    n = R / ep_sin_sqrt
    r = (1 - E) / ep_sin
    c = E_P2 * p_cos ** 2
    c2 = c * c
    d = (utm_x - 500000) / (n * K0)
    d2 = d * d
    d3 = d2 * d
    d4 = d3 * d
    d5 = d4 * d
    d6 = d5 * d
    lat = (p_rad - (p_tan / r) * (d2 / 2 - d4 / 24 * (5 + 3 * p_tan2 + 10 * c - 4 * c2 - 9 * E_P2)) +
           d6 / 720 * (61 + 90 * p_tan2 + 298 * c + 45 * p_tan4 - 252 * E_P2 - 3 * c2))
    lon = (d - d3 / 6 * (1 + 2 * p_tan2 + c) +
           d5 / 120 * (5 - 2 * c + 28 * p_tan2 - 3 * c2 + 8 * E_P2 + 24 * p_tan4)) / p_cos
    lon = (lon + np.radians((utm_num - 1) * 6 - 180 + 3) + np.pi) % (2 * np.pi) - np.pi
    return [np.degrees(lon), np.degrees(lat)]


def numpy_to_ros_image_msg(arr: np.ndarray, encoding: str) -> sensor_msgs.msg.Image:
    name_to_dtypes = {
        "rgb8":    (np.uint8,  3),
        "rgba8":   (np.uint8,  4),
        "rgb16":   (np.uint16, 3),
        "rgba16":  (np.uint16, 4),
        "bgr8":    (np.uint8,  3),
        "bgra8":   (np.uint8,  4),
        "bgr16":   (np.uint16, 3),
        "bgra16":  (np.uint16, 4),
        "mono8":   (np.uint8,  1),
        "mono16":  (np.uint16, 1),

        # for bayer image (based on cv_bridge.cpp)
        "bayer_rggb8":	(np.uint8,  1),
        "bayer_bggr8":	(np.uint8,  1),
        "bayer_gbrg8":	(np.uint8,  1),
        "bayer_grbg8":	(np.uint8,  1),
        "bayer_rggb16":	(np.uint16, 1),
        "bayer_bggr16":	(np.uint16, 1),
        "bayer_gbrg16":	(np.uint16, 1),
        "bayer_grbg16":	(np.uint16, 1),

        # OpenCV CvMat types
        "8UC1":    (np.uint8,   1),
        "8UC2":    (np.uint8,   2),
        "8UC3":    (np.uint8,   3),
        "8UC4":    (np.uint8,   4),
        "8SC1":    (np.int8,    1),
        "8SC2":    (np.int8,    2),
        "8SC3":    (np.int8,    3),
        "8SC4":    (np.int8,    4),
        "16UC1":   (np.uint16,   1),
        "16UC2":   (np.uint16,   2),
        "16UC3":   (np.uint16,   3),
        "16UC4":   (np.uint16,   4),
        "16SC1":   (np.int16,  1),
        "16SC2":   (np.int16,  2),
        "16SC3":   (np.int16,  3),
        "16SC4":   (np.int16,  4),
        "32SC1":   (np.int32,   1),
        "32SC2":   (np.int32,   2),
        "32SC3":   (np.int32,   3),
        "32SC4":   (np.int32,   4),
        "32FC1":   (np.float32, 1),
        "32FC2":   (np.float32, 2),
        "32FC3":   (np.float32, 3),
        "32FC4":   (np.float32, 4),
        "64FC1":   (np.float64, 1),
        "64FC2":   (np.float64, 2),
        "64FC3":   (np.float64, 3),
        "64FC4":   (np.float64, 4)
    }
    if not encoding in name_to_dtypes:
        raise TypeError('Unrecognized encoding {}'.format(encoding))

    im = Image(encoding=encoding)

    # extract width, height, and channels
    dtype_class, exp_channels = name_to_dtypes[encoding]
    dtype = np.dtype(dtype_class)
    if len(arr.shape) == 2:
        im.height, im.width, channels = arr.shape + (1,)
    elif len(arr.shape) == 3:
        im.height, im.width, channels = arr.shape
    else:
        raise TypeError("Array must be two or three dimensional")

    # check type and channels
    if exp_channels != channels:
        raise TypeError("Array has {} channels, {} requires {}".format(
            channels, encoding, exp_channels))

    if dtype_class != arr.dtype.type:
        raise TypeError("Array is {}, {} requires {}".format(
            arr.dtype.type, encoding, dtype_class))

    # make the array contiguous in memory, as mostly required by the format
    contig = np.ascontiguousarray(arr)
    im.data = contig.tostring()
    im.step = contig.strides[0]
    im.is_bigendian = (
        arr.dtype.byteorder == '>' or
        arr.dtype.byteorder == '=' and sys.byteorder == 'big')
    return im


def creat_bag(bag_path: str) -> rosbag.Bag:
    bag = rosbag.Bag(bag_path, 'w')
    return bag


def reindex_and_close_bag(bag: rosbag.Bag):
    for offset in bag.reindex():
        pass
    bag.close()


def add_rgb_image_to_bag(bag: rosbag.Bag, topic: str, stamp_s: float, image_path: str):
    cv_image: np.ndarray = cv2.imread(image_path)
    image_msg: sensor_msgs.msg.Image = numpy_to_ros_image_msg(cv_image, "bgr8")
    image_msg.header.frame_id = "camera"
    image_msg.header.stamp = rospy.Time.from_sec(stamp_s)

    bag.write(topic, image_msg, t=image_msg.header.stamp)


def add_rgb_image_json_to_bag(bag: rosbag.Bag, topic: str, stamp_s: float, image_path: str, json: str):
    cv_image: np.ndarray = cv2.imread(image_path)
    image_msg: sensor_msgs.msg.Image = numpy_to_ros_image_msg(cv_image, "bgr8")
    image_msg.header.frame_id = json
    image_msg.header.stamp = rospy.Time.from_sec(stamp_s)

    bag.write(topic, image_msg, t=image_msg.header.stamp)


def tmp_add_rgb_image_json_to_bag(bag: rosbag.Bag, topic: str, stamp_s: float,
                                  image_path: str, seg_path: str, json: str):
    cv_image: np.ndarray = cv2.imread(image_path)
    seg_image: np.ndarray = cv2.imread(seg_path)
    cv_image[seg_image == 60] = 0

    image_msg: sensor_msgs.msg.Image = numpy_to_ros_image_msg(cv_image, "bgr8")
    image_msg.header.frame_id = json
    image_msg.header.stamp = rospy.Time.from_sec(stamp_s)

    bag.write(topic, image_msg, t=image_msg.header.stamp)


def add_imu_to_bag(bag: rosbag.Bag, topic: str, stamp_s: float, acc: list, gyro: list):
    imu_msg = Imu()
    imu_msg.header.frame_id = "imu"
    imu_msg.header.stamp = rospy.Time.from_sec(stamp_s)
    imu_msg.orientation.x = 0
    imu_msg.orientation.y = 0
    imu_msg.orientation.z = 0
    imu_msg.orientation.w = 1
    imu_msg.linear_acceleration.x = acc[0]*10
    imu_msg.linear_acceleration.y = acc[1]*10
    imu_msg.linear_acceleration.z = acc[2]*10
    imu_msg.angular_velocity.x = gyro[0]
    imu_msg.angular_velocity.y = gyro[1]
    imu_msg.angular_velocity.z = gyro[2]

    bag.write(topic, imu_msg, t=imu_msg.header.stamp)


def add_cam_info_to_bag(bag: rosbag.Bag, topic: str, stamp_s: float,
                        width: int, height: int,
                        fx: float, fy: float, cx: float, cy: float):
    cam_info_msg = CameraInfo()
    cam_info_msg.header.frame_id = "camera"
    cam_info_msg.header.stamp = rospy.Time.from_sec(stamp_s)
    cam_info_msg.height = int(height)
    cam_info_msg.width = int(width)
    cam_info_msg.K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    bag.write(topic, cam_info_msg, t=cam_info_msg.header.stamp)


def tolist(a):
    return np.array(a).tolist()


def qab_to_euler_fix_yaw_deg(qab: np.ndarray) -> float:
    fix_z2y2x = Rotation.from_quat(qab).as_euler('zyx', degrees=True)
    return fix_z2y2x[0]


def make_homography(cam1_plane_d: float, cam1_plane_n: np.ndarray, T_cam1_cam2: np.ndarray,
                    K_cam1: np.ndarray,  K_cam2: np.ndarray):
    T_cam1_cam2 = np.array(T_cam1_cam2)
    t_cam1_cam2 = T_cam1_cam2[0:3, 3]
    R_cam2_cam1 = T_cam1_cam2[0:3, 0:3].transpose()
    K_cam1_inv = np.linalg.inv(np.array(K_cam1))
    t_cam1_cam2 = t_cam1_cam2.reshape(3, 1)
    cam1_plane_n = np.array(cam1_plane_n).reshape(1, 3)
    A = np.eye(3)+(1/cam1_plane_d)*(t_cam1_cam2.dot(cam1_plane_n))
    H_image2_image1 = np.array(K_cam2).dot(R_cam2_cam1).dot(A).dot(K_cam1_inv)
    H_image1_image2 = np.linalg.inv(H_image2_image1)
    H_image1_image2 = H_image1_image2.astype(np.float32)
    H_image1_image2 = H_image1_image2 / H_image1_image2[2, 2]
    return H_image1_image2


def rotation_matrix_from_vectors(from_vec1: np.ndarray,
                                 to_vec2: np.ndarray) -> np.ndarray:
    norm_v1 = np.linalg.norm(from_vec1)
    norm_v2 = np.linalg.norm(to_vec2)
    if norm_v1 == 0:
        return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])
    if norm_v2 == 0:
        return np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    a, b = (from_vec1 / norm_v1).reshape(3), (to_vec2 / norm_v2).reshape(3)
    v = np.cross(a, b)
    c = np.dot(a, b)
    s = np.linalg.norm(v)
    kmat = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    rotation_matrix = np.eye(3) + kmat + kmat.dot(kmat) * ((1 - c) / (s ** 2))
    return rotation_matrix


def make_bev_K_T(world_w, world_h, pixel_size, height):
    # world_w, world_h 地面范围
    # height 相机高度
    # pixel_size 一个像素对应地面尺寸
    cam_w = world_w/pixel_size
    cam_h = world_h/pixel_size
    fx = fy = height/pixel_size
    cx = cam_w/2
    cy = cam_h/2
    T_world_cam = np.array([[1, 0, 0, 0],
                            [0, -1, 0, 0],
                            [0, 0, -1, height],
                            [0, 0, 0, 1]])
    return (cam_w, cam_h, fx, fy, cx, cy, T_world_cam)


def fxycxy_to_K(fxycxy):
    if len(fxycxy) == 0:
        return np.array([])
    K = np.array([[fxycxy[0], 0, fxycxy[2]],
                  [0, fxycxy[1], fxycxy[3]],
                  [0, 0, 1]])
    return K


def del_folder_if_exist(dir=''):
    if os.path.exists(dir):
        shutil.rmtree(dir, ignore_errors=True)


def del_file_if_exist(file_path: str):
    if not os.path.exists(file_path):
        return
    os.remove(file_path)


def skel_cloud_to_distance_image(cloud_m_list: List[np.ndarray]):
    bin_img = np.zeros((10000, 10000)).astype(np.uint8)
    for cloud_m in cloud_m_list:
        b_cloud_px = np.rint(((cloud_m*20)+5000)).astype(int)
        for p in b_cloud_px:
            if (p[0] < 0) or (p[0] >= 10000) or (p[1] < 0) or (p[1] >= 10000):
                continue
            bin_img[p[1], p[0]] = 255

    bin_img = 255 - bin_img
    dt_image = cv2.distanceTransform(bin_img, 2, 3)
    dt_image[dt_image > 25] = 25
    dt_image = dt_image*10
    dt_image = np.rint(dt_image).astype(np.uint8)
    dt_image = 255-dt_image
    return dt_image


def qt_world_veh_to_qt_flat_veh(q_world_veh, t_world_veh):
    t_flat_veh = np.array([t_world_veh[0], t_world_veh[1], 0])
    yaw = qab_to_euler_fix_yaw_deg(q_world_veh)
    r = scipy.spatial.transform.Rotation.from_euler('z', yaw, degrees=True)
    q_flat_veh = r.as_quat()
    return (q_flat_veh, t_flat_veh)


def tab_to_Tab(tab: np.ndarray) -> np.ndarray:
    Tab: np.ndarray = np.eye(4)
    Tab[0, 3] = tab[0]
    Tab[1, 3] = tab[1]
    Tab[2, 3] = tab[2]
    return Tab


def qt_chazhi(older_stamp, older_q, older_t,
              newer_stamp, newer_q, newer_t,
              current_stamp):
    rate = ((current_stamp)-older_stamp)/(newer_stamp-older_stamp)
    current_t = np.array(older_t)+(np.array(newer_t)-np.array(older_t))*rate
    current_q = q_slerp(older_q, newer_q, rate)
    return (current_q, current_t)


def xyz_weighted(xyz1, w1, xyz2, w2) -> list:
    x3 = (xyz1[0] * w1 + xyz2[0] * w2) / (w1 + w2)
    y3 = (xyz1[1] * w1 + xyz2[1] * w2) / (w1 + w2)
    z3 = (xyz1[2] * w1 + xyz2[2] * w2) / (w1 + w2)
    return [x3, y3, z3]


def qxyzw_weighted(qxyzw1: list, w1, qxyzw2: list, w2) -> list:
    rate = w2 / (w1 + w2)
    return q_slerp(qxyzw1, qxyzw2, rate)


x_pi = 3.14159265358979324 * 3000.0 / 180.0
pi = 3.1415926535897932384626  # π
# a = 6378245.0  # 长半轴
wgs84_ep2 = 0.0067394967407662064  # WGS84椭球第二偏心率平方，wgs84_ep2 = ep * ep = (a*a - b*b)/b*b
wgs84_e2 = 0.006694379988651241  # WGS84椭球第一偏心率平方，wgs84_e2 = e * e = (a * a - b * b) / a * a
wgs84_c = 6399593.6257536924  # c是为简化书写引入的符号，c = a*a/b;
krasowski_a = 6378245.0  # 克拉索夫斯基椭球参数长半轴a
krasowski_e2 = 0.00669342162296594323  # 克拉索夫斯基椭球参数第一偏心率平方


def gcj2wgs(lng: Optional[float], lat: Optional[float]):
    """
    GCJ02(火星坐标系)转WGS84
    :param lng:火星坐标系的经度
    :param lat:火星坐标系纬度
    :return:
    """
    if out_of_china(lng, lat):
        return lng, lat
    dlat = transform_lat(lng - 105.0, lat - 35.0)
    dlng = transform_lon(lng - 105.0, lat - 35.0)
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - krasowski_e2 * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((krasowski_a * (1 - krasowski_e2)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (krasowski_a / sqrtmagic * math.cos(radlat) * pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return [lng * 2 - mglng, lat * 2 - mglat]


def wgs2gcj(lon, lat):
    dLat = transform_lat(lon - 105.0, lat - 35.0)
    dLon = transform_lon(lon - 105.0, lat - 35.0)
    radLat = lat / 180.0 * pi
    magic = math.sin(radLat)
    magic = 1 - krasowski_e2 * magic * magic
    sqrtMagic = math.sqrt(magic)
    dLat = (dLat * 180.0) / ((krasowski_a * (1 - krasowski_e2)) / (magic * sqrtMagic) * pi)
    dLon = (dLon * 180.0) / (krasowski_a / sqrtMagic * math.cos(radLat) * pi)
    mgLat = lat + dLat
    mgLon = lon + dLon
    return mgLon, mgLat


def transform_lat(lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
        0.1 * lng * lat + 0.2 * math.sqrt(math.fabs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 *
            math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 *
            math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret


def transform_lon(lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
        0.1 * lng * lat + 0.1 * math.sqrt(math.fabs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 *
            math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 *
            math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 *
            math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret


def out_of_china(lng, lat):
    """
    判断是否在国内，不在国内不做偏移
    :param lng:
    :param lat:
    :return:
    """
    if lng < 72.004 or lng > 137.8347:
        return True
    if lat < 0.8293 or lat > 55.8271:
        return True
    return False


def degree_2_radian(degree):
    """
    将角度变为弧度
    :param degree: 角度
    :return: 弧度
    """
    return degree * np.pi / 180


def radian_2_degree(radian):
    """
    将弧度变为角度
    :param radian: 弧度
    :return: 角度
    """
    return radian * 180 / np.pi


def get_n(radian_b):
    """
    获得某纬度处的卯酉圈曲率半径N
    :param radian_b: 度的弧度表达
    :return: float, 卯酉圈曲率半径N
    """

    cos_b = np.cos(radian_b)
    v = np.sqrt(1 + wgs84_ep2 * cos_b * cos_b)
    n = wgs84_c / v
    return n

# 经纬度转世界坐标(地形坐标系)


def wgs84_to_ecef(lon, lat, alt):
    """
    经纬高转空间直角坐标系
    :param lon: 经度
    :param lat: 纬度
    :param alt: 高度
    :return: List[float, float, float]
    """
    transformer = pyproj.Transformer.from_crs(
        {"proj": 'latlong', "ellps": 'WGS84', "datum": 'WGS84'},
        {"proj": 'geocent', "ellps": 'WGS84', "datum": 'WGS84'},
    )

    return transformer.transform(lon, lat, alt, radians=False)

# 世界坐标(地形坐标系)转经纬度


def ecef_to_wgs84(x, y, z):
    """
    :param x: ECEF坐标系下的X坐标
    :param y: ECEF坐标系下的Y坐标
    :param z: ECEF坐标系下的Z坐标
    :return: List[float, float, float]
    """
    transformer = pyproj.Transformer.from_crs(
        {"proj": 'geocent', "ellps": 'WGS84', "datum": 'WGS84'},
        {"proj": 'latlong', "ellps": 'WGS84', "datum": 'WGS84'},
    )

    return transformer.transform(x, y, z, radians=False)

# orign_pt:(lon, lat, alt);  input_pt:(lon, lat, alt)


def wgs84_to_enu(input_pt, origin_pt):
    # 注意纬度在前
    input_lon = input_pt[0]
    input_lat = input_pt[1]
    input_alt = 0
    if len(input_pt) >= 3:
        input_alt = input_pt[2]
    origin_lon = origin_pt[0]
    origin_lat = origin_pt[1]
    origin_alt = 0
    if len(origin_pt) >= 3:
        origin_alt = origin_pt[2]
    enu_e, enu_n, enu_u = pymap3d.geodetic2enu(input_lat, input_lon, input_alt, origin_lat, origin_lon, origin_alt)
    return (enu_e, enu_n, enu_u)

# orign_pt:(lon, lat, alt) ;     input:(e,n,u)


def enu_to_wgs84(input_pt, origin_pt):
    input_x = input_pt[0]
    input_y = input_pt[1]
    input_z = 0
    if len(input_pt) >= 3:
        input_z = input_pt[2]
    origin_lon = origin_pt[0]
    origin_lat = origin_pt[1]
    origin_alt = 0
    if len(origin_pt) >= 3:
        origin_alt = origin_pt[2]
    lat, lon, alt = pymap3d.enu2geodetic(input_x, input_y, input_z, origin_lat, origin_lon, origin_alt)
    if len(origin_pt) == 2:
        return (lon, lat)
    else:
        return (lon, lat, alt)


def feishu_robot_send_text(text: str, group_url=robot_url):
    try:
        import requests
        import urllib3
        urllib3.disable_warnings()
        requests.post(group_url,
                      headers={'Content-Type': 'application/json; charset=utf-8'},
                      data=json.dumps(
                          {"msg_type": "text", "at": {}, "content": {"text": text}}),
                      verify=False, timeout=(1, 1))
    except Exception as e:
        print("feishu_robot_send error: " + str(e))


def feishu_robot_send_img_text(image_path: str, text: str):
    import requests
    import urllib3
    urllib3.disable_warnings()

    def get_img_key(image_path: str):
        try:
            import requests_toolbelt
            res = requests.post(url="https://open.feishu.cn/open-apis/auth/v3/tenant_access_token/internal",
                                headers={'Content-Type': 'application/json; charset=utf-8'},
                                data=json.dumps({"app_id": app_id,  # app id
                                                 "app_secret": app_secret}),  # app 密码
                                verify=False, timeout=(1, 1))
            res_data = json.loads(res.text)
            token = res_data["tenant_access_token"]
            url = 'https://open.feishu.cn/open-apis/im/v1/images'
            form = {'image_type': 'message', 'image': (open(f'{image_path}', 'rb'))}
            multi_form = requests_toolbelt.MultipartEncoder(form)
            headers = {'Authorization': f'Bearer {token}', 'Content-Type': multi_form.content_type}
            response = requests.request("POST", url, headers=headers, data=multi_form)
            img_key = response.json()['data']['image_key']
            return img_key
        except Exception as e:
            return ""
    try:
        img_key = get_img_key(image_path)
        if img_key == "":
            print("png 上传失败，试试 jpg")
            time.sleep(1)
            img_key = get_img_key(image_path.rstrip(".png")+".jpg")
            if img_key == "":
                print("jpg 也上传失败了", image_path.rstrip(".png")+".jpg", "等10s再尝试...")
                time.sleep(10)
                img_key = get_img_key(image_path.rstrip(".png")+".jpg")
                if img_key == "":
                    print("jpg 等待10s也上传失败了", image_path.rstrip(".png")+".jpg", "试试小图片")
                    time.sleep(1)
                    img_key = get_img_key(image_path.rstrip(".png")+"_small.jpg")
                    if img_key == "":
                        print("小图片也上传失败了", image_path.rstrip(".png")+"_small.jpg", "放弃图片")
        if img_key == "":
            print("图片上传失败，只发送文本")
            data = {"msg_type": "text", "at": {}, "content": {"text": text}}
        else:
            img = Image.open(image_path)
            w = int(img.width/(img.height/100))
            data = {"msg_type": "post", "content": {"post": {"zh_cn": {"title": "", "content": [[{
                "tag": "img", "image_key": img_key, "width": w, "height": 100, }],
                [{"tag": "text", "text": text}]]}}}}
        requests.post(robot_url,  # 机器人网址
                      headers={'Content-Type': 'application/json; charset=utf-8'},
                      data=json.dumps(data), verify=False, timeout=(1, 1))
        print("完成消息发送")
    except Exception as e:
        print("feishu_robot_send_img_text error: " + str(e))
        return


def read_json(json_path: str) -> dict:
    if not os.path.exists(json_path):
        return {}
    data = {}
    with open(json_path, 'r') as f:
        try:
            data = json.load(f)
        except Exception as e:
            print('Reason: ' + str(e))
            return {}
    return data


def lonlat_utm_num_to_utm_xy(lon, lat, utm_num) -> List[float]:
    if not -80.0 <= lat <= 84.0:
        print('error lat is:' + str(lat))
        return [np.nan, np.nan]
    if not -180.0 <= lon <= 180.0:
        print('error lon is:' + str(lon))
        return [np.nan, np.nan]
    if not 1 <= utm_num <= 60:
        print('utm number out of range (must be between 1 and 60)')
        return [np.nan, np.nan]
    K0 = 0.9996
    E = 0.00669438
    E_P2 = 0.006739496752268451
    M1 = 0.9983242984503243
    M2 = 0.002514607064228144
    M3 = 2.6390466021299826e-06
    M4 = 3.418046101696858e-09
    R = 6378137
    lat_rad = np.radians(lat)
    lat_sin = np.sin(lat_rad)
    lat_cos = np.cos(lat_rad)
    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2
    lon_rad = np.radians(lon)
    central_lon = (utm_num - 1) * 6 - 180 + 3
    central_lon_rad = np.radians(central_lon)
    n = R / np.sqrt(1 - E * lat_sin ** 2)
    c = E_P2 * lat_cos ** 2
    a = lat_cos * ((lon_rad - central_lon_rad + np.pi) % (2 * np.pi) - np.pi)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a
    m = R * (M1 * lat_rad - M2 * np.sin(2 * lat_rad) +
             M3 * np.sin(4 * lat_rad) - M4 * np.sin(6 * lat_rad))
    utm_x = K0 * n * (a + a3 / 6 * (1 - lat_tan2 + c) +
                      a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000
    utm_y = K0 * (m + n * lat_tan *
                  (a2 / 2 + a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c ** 2) +
                   a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))
    if lat < 0:
        utm_y += 10000000
    return [utm_x, utm_y]


def cal_iou(box1, box2):
    """
    :param box1: = [xmin1, ymin1, xmax1, ymax1]
    :param box2: = [xmin2, ymin2, xmax2, ymax2]
    :return:
    """
    [xmin1, ymin1], [xmax1, ymax1] = box1
    [xmin2, ymin2], [xmax2, ymax2] = box2
    # 计算每个矩形的面积
    s1 = (xmax1 - xmin1) * (ymax1 - ymin1)  # C的面积
    s2 = (xmax2 - xmin2) * (ymax2 - ymin2)  # G的面积

    # 计算相交矩形
    xmin = max(xmin1, xmin2)
    ymin = max(ymin1, ymin2)
    xmax = min(xmax1, xmax2)
    ymax = min(ymax1, ymax2)

    w = max(0, xmax - xmin)
    h = max(0, ymax - ymin)
    area = w * h  # C∩G的面积
    iou = area / (s1 + s2 - area)
    return iou


def cal_iou_2(box1, box2):
    """
    :param box1: = [xmin1, ymin1, xmax1, ymax1]
    :param box2: = [xmin2, ymin2, xmax2, ymax2]
    :return:
    """
    if len(box1) == 0 or len(box2) == 0:
        return 0
    [xmin1, ymin1, z], [xmax1, ymax1, z] = box1
    [xmin2, ymin2, z], [xmax2, ymax2, z] = box2
    # 计算每个矩形的面积
    s1 = (xmax1 - xmin1) * (ymax1 - ymin1)  # C的面积
    s2 = (xmax2 - xmin2) * (ymax2 - ymin2)  # G的面积

    # 计算相交矩形
    xmin = max(xmin1, xmin2)
    ymin = max(ymin1, ymin2)
    xmax = min(xmax1, xmax2)
    ymax = min(ymax1, ymax2)

    w = max(0, xmax - xmin)
    h = max(0, ymax - ymin)
    area = w * h  # C∩G的面积
    iou = area / (s1 + s2 - area)
    return iou


def get_utm_num_from_gps_wkt(gps_polygon_wkt: str):
    g1 = shapely.wkt.loads(gps_polygon_wkt)
    utm_num = lonlat_to_utm_num(g1.centroid.x, g1.centroid.y)
    return utm_num


def get_utm_box_from_gps_polygon_wkt(gps_polygon_wkt: str, utm_num: int, old_box: list = []):
    gps_polygon_json = shapely.geometry.mapping(shapely.wkt.loads(gps_polygon_wkt))
    utm_xy_list = []
    for one_lon_lat in gps_polygon_json['coordinates'][0]:
        utm_xy_list.append(lonlat_utm_num_to_utm_xy(*one_lon_lat, utm_num))
    utm_xy_list = np.array(utm_xy_list)
    utm_box = [utm_xy_list.min(axis=0).tolist(),
               utm_xy_list.max(axis=0).tolist()]
    if len(old_box) != 0:
        utm_box[0][0] = min(old_box[0][0], utm_box[0][0])
        utm_box[0][1] = min(old_box[0][1], utm_box[0][1])
        utm_box[1][0] = max(old_box[1][0], utm_box[1][0])
        utm_box[1][1] = max(old_box[1][1], utm_box[1][1])
    return utm_box


def gps_line_wkt_to_world_xy_list(gps_line_wkt: str, utm_num, t_utm_world) -> list:
    gps_line_json = shapely.geometry.mapping(shapely.wkt.loads(gps_line_wkt))
    world_xy_list = []
    for one_lon_lat in gps_line_json['coordinates']:
        utm_xy = lonlat_utm_num_to_utm_xy(*one_lon_lat, utm_num)
        world_xy = [int((utm_xy[0]-t_utm_world[0])*100)/100,
                    int((utm_xy[1]-t_utm_world[1])*100)/100]
        world_xy_list.append(world_xy)
    return world_xy_list


def gps_polygon_wkt_to_world_xy_list(gps_polygon_wkt: str, utm_num, t_utm_world) -> list:
    gps_polygon_json = shapely.geometry.mapping(shapely.wkt.loads(gps_polygon_wkt))
    world_xy_list = []
    for one_lon_lat in gps_polygon_json['coordinates'][0]:
        utm_xy = lonlat_utm_num_to_utm_xy(*one_lon_lat, utm_num)
        world_xy = [int((utm_xy[0]-t_utm_world[0])*100)/100,
                    int((utm_xy[1]-t_utm_world[1])*100)/100]
        world_xy_list.append(world_xy)
    return world_xy_list


def angle_distance_deg(deg_1, deg_2):
    b = (deg_1 - deg_2 + 180) % 360
    if b < 0:
        b += 360
    return b - 180


def best_fit_transform(p_A_points: np.ndarray,
                       p_B_points: np.ndarray) -> np.ndarray:
    """
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
    """
    assert len(p_A_points) == len(p_B_points)
    # translate points to their centroids
    centroid_A = np.mean(p_A_points, axis=0)
    centroid_B = np.mean(p_B_points, axis=0)
    AA = p_A_points - centroid_A
    BB = p_B_points - centroid_B
    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)
    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = np.dot(Vt.T, U.T)
    # translation
    t = centroid_B.T - np.dot(R, centroid_A.T)
    # homogeneous transformation
    T_B_A = np.identity(4)
    T_B_A[0:3, 0:3] = R
    T_B_A[0:3, 3] = t
    return T_B_A


def get_filesize_B(filePath) -> int:
    if not os.path.exists(filePath):
        log.error('get_filesize_MB(), no {}'.format(filePath))
        return 0
    size_ = os.path.getsize(filePath)
    return size_


def rename_file(old_path, new_path):
    if not os.path.exists(old_path):
        return
    if os.path.exists(new_path):
        os.remove(new_path)
    os.rename(old_path, new_path)


class RepeatedTimer(object):
    def __init__(self, interval, function, *args, **kwargs):
        self._timer = None
        self.interval = interval
        self.function = function
        self.args = args
        self.kwargs = kwargs
        self.is_running = False
        self.start()

    def _run(self):
        self.is_running = False
        self.start()
        self.function(*self.args, **self.kwargs)

    def start(self):
        if not self.is_running:
            self._timer = Timer(self.interval, self._run)
            self._timer.start()
            self.is_running = True

    def stop(self):
        self._timer.cancel()
        self.is_running = False


def singleton(cls):
    _instance = {}

    def inner():
        if cls not in _instance:
            _instance[cls] = cls()
        return _instance[cls]
    return inner


@singleton
class SystemMonitor():
    class StageSystemInfo():
        def __init__(self):
            self.stage_name = ""
            self.stage_start_stamp_ms = int(round(time.time() * 1000))
            self.stage_time_use_ms = 0
            self.cpu_percent_mean = 0
            self.cpu_percent_max = 0
            self.cpu_percent_cnt = 0
            self.ram_gb_mean = 0
            self.ram_gb_max = 0
            self.ram_gb_cnt = 0

    def __init__(self) -> None:
        self.stage_file_path = ""
        self.log_file_path = ""
        self.start_stamp_ms = int(round(time.time() * 1000))
        self.container_cpu_max = 0
        self.container_ram_max = 0
        self.host_cpu_max = 0
        self.host_ram_max = 0
        self.stage_list = []
        self.prevUsageUser = 0
        self.host_cpu_count = psutil.cpu_count()
        self.host_ram_total = psutil.virtual_memory().total/1072693248
        self.every_n_second = 2
        try:
            self.container_cpu_count = len(open("/sys/fs/cgroup/cpu/cpuacct.usage_percpu", 'r').readline().strip('\n').strip(" ").split(" "))
            self.container_ram_total = int(open("/sys/fs/cgroup/memory/memory.limit_in_bytes", 'r').readline().strip('\n'))/1072693248
        except Exception as e:
            log.warning("读取容器参数失败")

    def start(self, stage_file_path: str, log_file_path: str):
        try:
            self.stage_file_path = stage_file_path
            self.log_file_path = log_file_path
            output_info = {
                "realtime": {
                    "container_cpu": 0,
                    "container_ram": 0,
                    "host_cpu": 0,
                    "host_ram": 0,
                },
                "container": {
                    "cpu_max": round(self.container_cpu_max, 1),
                    "ram_max": round(self.container_ram_max, 1),
                    "cpu_count": self.container_cpu_count,
                    "ram_total": round(self.container_ram_total, 1),
                },
                "host": {
                    "cpu_max": round(self.host_cpu_max, 1),
                    "ram_max": round(self.host_ram_max, 1),
                    "cpu_count": self.host_cpu_count,
                    "ram_total": round(self.host_ram_total, 1),
                },
                "stages": ["环节名称 环节用时 CPU均值 CPU峰值 内存均值 内存峰值"]
            }
            dict_to_json_file_format(self.log_file_path, output_info)
            self.rt = RepeatedTimer(self.every_n_second, self.run_once)
        except Exception as e:
            log.warning("任务监控启动失败")

    def stop(self):
        try:
            self.rt.stop()
        except Exception as e:
            log.warning("任务监控结束失败")

    def get_container_cpu(self):
        currentUsageUser = 0
        try:
            currentUsageUser = int(open("/sys/fs/cgroup/cpuacct/cpuacct.usage_user", 'r').readline().strip('\n'))
        except Exception as e:
            pass
        if self.prevUsageUser == 0:
            self.prevUsageUser = currentUsageUser
        current_cpu = float(currentUsageUser-self.prevUsageUser) / (self.every_n_second*10000000) / self.container_cpu_count
        self.prevUsageUser = currentUsageUser
        return current_cpu

    def get_container_ram(self):
        current_ram_gb = 0
        try:
            current_ram_b1 = int(open("/sys/fs/cgroup/memory/memory.usage_in_bytes", 'r').readline().strip('\n'))
            current_ram_b2 = int(open("/sys/fs/cgroup/memory/memory.stat", 'r').readline().strip('\n').split(" ")[1])
            current_ram_gb = float((current_ram_b1-current_ram_b2)/1072693248)
        except Exception as e:
            pass
        return current_ram_gb

    def get_host_cpu(self):
        return psutil.cpu_percent(0)

    def get_host_ram(self):
        current_mem_used_GB = psutil.virtual_memory().used/1072693248
        return current_mem_used_GB

    def run_once(self):
        current_stage_name = "空闲"
        try:
            current_stage_name = open(self.stage_file_path, 'r').readline().strip('\n')
        except Exception as e:
            return
        try:
            if (len(self.stage_list) == 0) or (current_stage_name != self.stage_list[-1].stage_name):
                stage_info: self.StageSystemInfo = self.StageSystemInfo()
                stage_info.stage_name = current_stage_name
                stage_info.stage_start_stamp_ms = int(round(time.time() * 1000))
                self.stage_list.append(stage_info)

            current_stamp_ms = int(round(time.time() * 1000))
            container_cpu = self.get_container_cpu()
            container_ram = self.get_container_ram()
            host_cpu = self.get_host_cpu()
            host_ram = self.get_host_ram()

            stage_info: self.StageSystemInfo = self.stage_list[-1]
            stage_info.stage_time_use_ms = current_stamp_ms - stage_info.stage_start_stamp_ms
            stage_info.cpu_percent_mean = (stage_info.cpu_percent_mean*stage_info.cpu_percent_cnt+container_cpu)/(stage_info.cpu_percent_cnt+1)
            stage_info.cpu_percent_cnt += 1
            stage_info.cpu_percent_max = max(stage_info.cpu_percent_max, container_cpu)
            stage_info.ram_gb_mean = (stage_info.ram_gb_mean*stage_info.ram_gb_cnt+container_ram)/(stage_info.ram_gb_cnt+1)
            stage_info.ram_gb_cnt += 1
            stage_info.ram_gb_max = max(stage_info.ram_gb_max, container_ram)

            self.container_cpu_max = max(self.container_cpu_max, container_cpu)
            self.container_ram_max = max(self.container_ram_max, container_ram)
            self.host_cpu_max = max(self.host_cpu_max, host_cpu)
            self.host_ram_max = max(self.host_ram_max, host_ram)

            output_info = {
                "realtime": {
                    "container_cpu": round(container_cpu, 1),
                    "container_ram": round(container_ram, 1),
                    "host_cpu": round(host_cpu, 1),
                    "host_ram": round(host_ram, 1),
                },
                "container": {
                    "cpu_max": round(self.container_cpu_max, 1),
                    "ram_max": round(self.container_ram_max, 1),
                    "cpu_count": self.container_cpu_count,
                    "ram_total": round(self.container_ram_total, 1),
                },
                "host": {
                    "cpu_max": round(self.host_cpu_max, 1),
                    "ram_max": round(self.host_ram_max, 1),
                    "cpu_count": self.host_cpu_count,
                    "ram_total": round(self.host_ram_total, 1),
                },
                "stages": ["环节名称 环节用时 CPU均值 CPU峰值 内存均值 内存峰值"]
            }
            stage_info: self.StageSystemInfo
            for stage_info in self.stage_list:
                output_info["stages"].append(
                    f"{stage_info.stage_name} {round(stage_info.stage_time_use_ms/60000,1)}m {round(stage_info.cpu_percent_mean,1)}% {round(stage_info.cpu_percent_max,1)}% {round(stage_info.ram_gb_mean,1)}G {round(stage_info.ram_gb_max,1)}G")
            dict_to_json_file_format(self.log_file_path, output_info)
        except Exception as e:
            return


def get_current_time_str_DHMS():
    utc_dt = datetime.datetime.utcnow().replace(tzinfo=datetime.timezone.utc)  # 构建了 UTC 的当前时间
    bj_dt = utc_dt.astimezone(datetime.timezone(datetime.timedelta(hours=8)))  # 将时区转化为东八区的时间
    finish_time = bj_dt.strftime('%d %H:%M:%S')
    return finish_time


def get_filesize_MB(filePath) -> float:
    if not os.path.exists(filePath):
        log.error('get_filesize_MB(), no {}'.format(filePath))
        return 0
    size_ = os.path.getsize(filePath)
    size_ = size_ / float(1024 * 1024)
    return round(size_, 2)


def make_link(fake_link, true_link):
    add_folder_in_file_path_if_no_exist(fake_link)
    rel_path = get_relpath(true_link, fake_link)
    if rel_path.startswith("../"):
        rel_path = rel_path[3:]
    log.info(rel_path)
    # 删除可能占据虚拟文件的东西
    if os.path.exists(fake_link):
        return
    try:
        del_folder_if_exist(fake_link)
        del_file_if_exist(fake_link)
        os.unlink(fake_link)
    except Exception as e:
        pass
    # 创建虚拟链接文件
    os.symlink(rel_path, fake_link)


def line_list_to_obj_file(line_list: List[List[List[float]]], obj_path: str):
    vs = []
    ls = []
    for one_line in line_list:
        ls.append("l")
        for one_point in one_line:
            if len(one_point) == 2:
                one_point.append(0)
            vs.append("v {} {} {}".format(*one_point))
            ls[-1] += f" {len(vs)}"
    vs.extend(ls)
    write_lines_to_file_override(obj_path, vs)


def densify_2d_line(old_line: List[Tuple[float, float]], step: float):
    if len(old_line) == 0:
        return []
    if len(old_line) == 1:
        return [old_line[0]]
    new_line = [old_line[0]]
    for one in old_line[1:]:
        while np.linalg.norm([one[0]-new_line[-1][0], one[1]-new_line[-1][1]]) > step:
            yaw = np.arctan2(one[1]-new_line[-1][1], one[0]-new_line[-1][0])
            new_line.append([new_line[-1][0]+step*np.cos(yaw), new_line[-1][1]+step*np.sin(yaw)])
        new_line.append(one)
    return new_line


def get_obj_from_gps_polygon_wkt(gps_polygon_wkt: str, utm_num: int, t_utm_world: list, obj_path: str):
    gps_polygon_json = shapely.geometry.mapping(shapely.wkt.loads(gps_polygon_wkt))
    world_xy_list = []
    for one_lon_lat in gps_polygon_json['coordinates'][0]:
        utm_xy = lonlat_utm_num_to_utm_xy(*one_lon_lat, utm_num)
        world_xy = [utm_xy[0]-t_utm_world[0], utm_xy[1]-t_utm_world[1]]
        world_xy_list.append(world_xy)
    line_list_to_obj_file([world_xy_list], obj_path)


def line_to_obj_file(line: List[List[float]], obj_path: str):
    vs = []
    ls = []
    ls.append("l")
    for one_point in line:
        vs.append("v {} {} {}".format(*one_point, 0, 0))
        ls[-1] += f" {len(vs)}"
    vs.extend(ls)
    folder = os.path.dirname(obj_path)
    if not os.path.exists(folder):
        os.makedirs(folder)
    s = ''
    for l in vs:
        s += (str(l).strip('\n') + '\n')
    open(obj_path, 'w').write(s.strip('\n'))


def densify_geometry(line_geometry, step):
    length_m = line_geometry.length  # get the length
    if length_m <= step:
        return line_geometry
    xy = []  # to store new tuples of coordinates
    for distance_along_old_line in np.arange(0, int(length_m), step):
        point = line_geometry.interpolate(distance_along_old_line)  # interpolate a point every step along the old line
        xp, yp = point.x, point.y  # extract the coordinates
        xy.append((xp, yp))  # and store them in xy list
    new_line = LineString(xy)  # Here, we finally create a new line with densified points.
    return new_line


def shapely_to_obj(shapely_geom, obj_path):
    def line_list_to_obj_file(line_list: List[List[List[float]]], obj_path: str):
        lines = []
        vs = []
        ls = []
        for one_line in line_list:
            ls.append("l")
            for one_point in one_line:
                vs.append("v {} {} {}".format(*one_point, 0, 0))
                ls[-1] += f" {len(vs)}"
        lines.extend(vs)
        lines.extend(ls)
        folder = os.path.dirname(obj_path)
        if not os.path.exists(folder):
            os.makedirs(folder)
        s = ''
        for l in lines:
            s += (str(l).strip('\n') + '\n')
        open(obj_path, 'w').write(s.strip('\n'))
    if shapely_geom.geometryType() == "LineString":
        line_list_to_obj_file([shapely_geom.coords[:]], obj_path)
    elif shapely_geom.geometryType() == "Polygon":
        line_list_to_obj_file([shapely_geom.exterior.coords[:]], obj_path)
    elif shapely_geom.geometryType() == "MultiLineString":
        line_list_to_obj_file([one.coords[:] for one in shapely_geom.geoms], obj_path)
    elif shapely_geom.geometryType() == "MultiPolygon":
        line_list_to_obj_file([one.exterior.coords[:] for one in shapely_geom.geoms], obj_path)
    else:
        print("不支持的 shapely gemo " + shapely_geom.geometryType())


def R_TN_body_to_R_GN_body(utm_num, lon, lat, R_TN_body):
    proj = pyproj.Proj(pyproj.CRS(f"EPSG:326{'%02d'%utm_num}"))
    yaw_GN_TN = proj.get_factors(lon, lat).meridian_convergence
    R_GN_TN = scipy.spatial.transform.Rotation.from_euler('Z', yaw_GN_TN, degrees=True).as_matrix()
    R_GN_body = R_GN_TN.dot(R_TN_body)
    return R_GN_body


def R_GN_body_to_R_TN_body(utm_num, lon, lat, R_GN_body):
    proj = pyproj.Proj(pyproj.CRS(f"EPSG:326{'%02d'%utm_num}"))
    yaw_GN_TN = proj.get_factors(lon, lat).meridian_convergence
    R_TN_GN = scipy.spatial.transform.Rotation.from_euler('Z', -yaw_GN_TN, degrees=True).as_matrix()
    R_TN_body = R_TN_GN.dot(R_GN_body)
    return R_TN_body


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    return x, y, z, w


def q_GN_body_to_q_TN_body(utm_num, lon, lat, q_GN_body):
    proj = pyproj.Proj(pyproj.CRS(f"EPSG:326{'%02d'%utm_num}"))
    yaw_GN_TN = proj.get_factors(lon, lat).meridian_convergence
    q_TN_GN = scipy.spatial.transform.Rotation.from_euler('Z', -yaw_GN_TN, degrees=True).as_quat()
    q_GN_body = quaternion_multiply(q_TN_GN, q_GN_body)
    return q_GN_body


def get_folder_size_GB(dir):
    size = 0
    for root, dirs, files in os.walk(dir):
        size += sum([os.path.getsize(os.path.join(root, name)) for name in files])
    return size/(1024**3)


class SunTimeException(Exception):

    def __init__(self, message):
        super(SunTimeException, self).__init__(message)


class Sun:
    """
    Approximated calculation of sunrise and sunset datetimes. Adapted from:
    https://stackoverflow.com/questions/19615350/calculate-sunrise-and-sunset-times-for-a-given-gps-coordinate-within-postgresql
    """

    def __init__(self, lat, lon):
        self._lat = lat
        self._lon = lon

    def get_sunrise_time(self, date=None):
        """
        Calculate the sunrise time for given date.
        :param lat: Latitude
        :param lon: Longitude
        :param date: Reference date. Today if not provided.
        :return: UTC sunrise datetime
        :raises: SunTimeException when there is no sunrise and sunset on given location and date
        """
        date = datetime.date.today() if date is None else date
        sr = self._calc_sun_time(date, True)
        if sr is None:
            raise SunTimeException('The sun never rises on this location (on the specified date)')
        else:
            return sr

    def get_local_sunrise_time(self, date=None, local_time_zone=tz.tzlocal()):
        """
        Get sunrise time for local or custom time zone.
        :param date: Reference date. Today if not provided.
        :param local_time_zone: Local or custom time zone.
        :return: Local time zone sunrise datetime
        """
        date = datetime.date.today() if date is None else date
        sr = self._calc_sun_time(date, True)
        if sr is None:
            raise SunTimeException('The sun never rises on this location (on the specified date)')
        else:
            return sr.astimezone(local_time_zone)

    def get_sunset_time(self, date=None):
        """
        Calculate the sunset time for given date.
        :param lat: Latitude
        :param lon: Longitude
        :param date: Reference date. Today if not provided.
        :return: UTC sunset datetime
        :raises: SunTimeException when there is no sunrise and sunset on given location and date.
        """
        date = datetime.date.today() if date is None else date
        ss = self._calc_sun_time(date, False)
        if ss is None:
            raise SunTimeException('The sun never sets on this location (on the specified date)')
        else:
            return ss

    def get_local_sunset_time(self, date=None, local_time_zone=tz.tzlocal()):
        """
        Get sunset time for local or custom time zone.
        :param date: Reference date
        :param local_time_zone: Local or custom time zone.
        :return: Local time zone sunset datetime
        """
        date = datetime.date.today() if date is None else date
        ss = self._calc_sun_time(date, False)
        if ss is None:
            raise SunTimeException('The sun never sets on this location (on the specified date)')
        else:
            return ss.astimezone(local_time_zone)

    def _calc_sun_time(self, date, isRiseTime=True, zenith=90.8):
        """
        Calculate sunrise or sunset date.
        :param date: Reference date
        :param isRiseTime: True if you want to calculate sunrise time.
        :param zenith: Sun reference zenith
        :return: UTC sunset or sunrise datetime
        :raises: SunTimeException when there is no sunrise and sunset on given location and date
        """
        # isRiseTime == False, returns sunsetTime
        day = date.day
        month = date.month
        year = date.year

        TO_RAD = math.pi/180.0

        # 1. first calculate the day of the year
        N1 = math.floor(275 * month / 9)
        N2 = math.floor((month + 9) / 12)
        N3 = (1 + math.floor((year - 4 * math.floor(year / 4) + 2) / 3))
        N = N1 - (N2 * N3) + day - 30

        # 2. convert the longitude to hour value and calculate an approximate time
        lngHour = self._lon / 15

        if isRiseTime:
            t = N + ((6 - lngHour) / 24)
        else:  # sunset
            t = N + ((18 - lngHour) / 24)

        # 3. calculate the Sun's mean anomaly
        M = (0.9856 * t) - 3.289

        # 4. calculate the Sun's true longitude
        L = M + (1.916 * math.sin(TO_RAD*M)) + (0.020 * math.sin(TO_RAD * 2 * M)) + 282.634
        L = self._force_range(L, 360)  # NOTE: L adjusted into the range [0,360)

        # 5a. calculate the Sun's right ascension

        RA = (1/TO_RAD) * math.atan(0.91764 * math.tan(TO_RAD*L))
        RA = self._force_range(RA, 360)  # NOTE: RA adjusted into the range [0,360)

        # 5b. right ascension value needs to be in the same quadrant as L
        Lquadrant = (math.floor(L/90)) * 90
        RAquadrant = (math.floor(RA/90)) * 90
        RA = RA + (Lquadrant - RAquadrant)

        # 5c. right ascension value needs to be converted into hours
        RA = RA / 15

        # 6. calculate the Sun's declination
        sinDec = 0.39782 * math.sin(TO_RAD*L)
        cosDec = math.cos(math.asin(sinDec))

        # 7a. calculate the Sun's local hour angle
        cosH = (math.cos(TO_RAD*zenith) - (sinDec * math.sin(TO_RAD*self._lat))) / (cosDec * math.cos(TO_RAD*self._lat))

        if cosH > 1:
            return None     # The sun never rises on this location (on the specified date)
        if cosH < -1:
            return None     # The sun never sets on this location (on the specified date)

        # 7b. finish calculating H and convert into hours

        if isRiseTime:
            H = 360 - (1/TO_RAD) * math.acos(cosH)
        else:  # setting
            H = (1/TO_RAD) * math.acos(cosH)

        H = H / 15

        # 8. calculate local mean time of rising/setting
        T = H + RA - (0.06571 * t) - 6.622

        # 9. adjust back to UTC
        UT = T - lngHour
        UT = self._force_range(UT, 24)   # UTC time in decimal format (e.g. 23.23)

        # 10. Return
        hr = self._force_range(int(UT), 24)
        min = round((UT - int(UT))*60, 0)
        if min == 60:
            hr += 1
            min = 0

        # 10. check corner case https://github.com/SatAgro/suntime/issues/1
        if hr == 24:
            hr = 0
            day += 1

            if day > calendar.monthrange(year, month)[1]:
                day = 1
                month += 1

                if month > 12:
                    month = 1
                    year += 1

        return datetime.datetime(year, month, day, hr, int(min), tzinfo=tz.tzutc())

    @staticmethod
    def _force_range(v, max):
        # force v to be >= 0 and < max
        if v < 0:
            return v + max
        elif v >= max:
            return v - max

        return v


def is_day_time(lon, lat, timestamp_s):
    sun = Sun(lat, lon)
    data = datetime.date.fromtimestamp(timestamp_s)
    sr_ts = calendar.timegm(sun.get_sunrise_time(data).timetuple())
    ss_ts = calendar.timegm(sun.get_sunset_time(data).timetuple())
    if (ss_ts < sr_ts):
        ss_ts += 3600*24
    if(abs((timestamp_s % (3600*24))-((sr_ts+ss_ts)/2) % (3600*24)) < ((ss_ts-sr_ts)/2+(60*72))):
        return True
    else:
        return False


def wgs_wkt_simple(raw_wkt: str) -> str:
    try:
        utm_num = get_utm_num_from_gps_wkt(raw_wkt)
        proj_transform = pyproj.Transformer.from_crs("EPSG:4326", f"EPSG:326{utm_num}", always_xy=True).transform
        inv_proj_transform = pyproj.Transformer.from_crs(f"EPSG:326{utm_num}", "EPSG:4326", always_xy=True).transform
        utm_task_poly: Polygon = shapely.ops.transform(proj_transform, shapely.wkt.loads(raw_wkt)).simplify(1)
        wgs_task_poly = shapely.ops.transform(inv_proj_transform, utm_task_poly)
        new_wkt = shapely.wkt.dumps(wgs_task_poly, rounding_precision=6)
        return new_wkt
    except Exception as _:
        return raw_wkt


def json_wgs_wkt_simple(obj: dict):
    if isinstance(obj, dict):
        for k, v in obj.items():
            if k == "task_geom" or k == "link_geom" or k == "geometry":
                obj[k] = wgs_wkt_simple(obj[k])
            json_wgs_wkt_simple(v)
    elif isinstance(obj, list):
        for _, v in enumerate(obj):
            json_wgs_wkt_simple(v)


if __name__ == '__main__':
    feishu_publish("/home/test/code/lidar-mapping/task_info_demo.json")
