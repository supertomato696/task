'''
Description:
Author: yxx
Date: 2022-08-26 14:36:17
Reference:
'''
import argparse
import copy
import datetime
import os,sys
import time
import subprocess
import json
from shapely.wkt import dumps, loads
from multiprocessing import cpu_count, Pool
import shutil

fsd_mapbuild_root_folder = os.path.dirname(os.path.abspath(__file__))  # 顶层目录加入到python的环境变量中
print(("fsd_mapbuild_root_folder is {}").format(fsd_mapbuild_root_folder))

road_model_folder = os.path.abspath(fsd_mapbuild_root_folder + "../../")
print(("road_model_folder is {}").format(road_model_folder))

sys.path.append(os.path.join(fsd_mapbuild_root_folder, "scripts"))
from process_link_node import ProcessLinkNode

def get_files_path_with_prefix(dir='', prefix=''):
    files_path = []
    for name in os.listdir(dir):
        if name.startswith(prefix):
            fp = os.path.join(dir, name)
            if os.path.isfile(fp):
                files_path.append(fp)
    return sorted(files_path)

# 获取所有子文件夹的名字
def get_folders_name(dir=''):
    subfolders_name = []
    for name in os.listdir(dir):
        fp = os.path.join(dir, name)
        if os.path.isdir(fp):
            subfolders_name.append(name)
    return sorted(subfolders_name)

# 仅获取数字文件夹的名字
def get_subfolders_only_number_name(root_folder_path=''):
    subfolders_name = []
    if not os.path.exists(root_folder_path):
        print('get_subfolders_only_number_name(), no {}'.format(root_folder_path))
        return []
    for name in os.listdir(root_folder_path):
        if name.isdigit():
            fp = os.path.join(root_folder_path, name)
            if os.path.isdir(fp):
                subfolders_name.append(name)
    return sorted(subfolders_name)

def move_files_with_prefix(src_folder='', dst_folder='', prefix=''):
    if not os.path.exists(dst_folder):
        os.makedirs(dst_folder)

    for main_dir, dirs, file_name_list in os.walk(src_folder):
        if len(dirs) == 0:
            if len(file_name_list) == 0:
                continue
            for name in file_name_list:
                if not name.startswith(prefix):
                    continue
                src_file = os.path.join(src_folder, name)
                dst_file = os.path.join(dst_folder, name)
                if not os.path.isfile(src_file):
                    continue
                shutil.move(src_file, dst_file)
        else:
            for name in dirs:
                move_files_with_prefix(os.path.join(src_folder, name), dst_folder, prefix)

def add_folder_in_file_path_if_no_exist(filepath=''):
    folder = os.path.dirname(filepath)
    if not os.path.exists(folder):
        os.makedirs(folder)
        
def copy_file(src='', dst=''):
    add_folder_in_file_path_if_no_exist(dst)
    if os.path.abspath(src) == os.path.abspath(dst):
        return
    shutil.copyfile(src, dst)

def empty_folder_if_exist(dir=''):
    if not os.path.exists(dir):
        return
    shutil.rmtree(dir, ignore_errors=True)
    os.makedirs(dir)

def del_file_if_exist(dir: str):
    if not os.path.exists(dir):
        return
    os.remove(dir)
            
def run_preception_once(taskdata_dir, trail_id, outputDir_pp):#运行一个trail的感知结果解析
    print(".......解析感知结果的轨迹："+trail_id)
    if not os.path.exists(os.path.join(taskdata_dir, trail_id)):
        return
    if not os.path.exists(os.path.join(taskdata_dir, trail_id, "data_set.json")):
        return
    if not os.path.exists(os.path.join(taskdata_dir, trail_id, "pp_laneline_info.json")):
        return
    pp_exe_path = fsd_mapbuild_root_folder +"/roadMapping/bin/processPerception"
    os.system(pp_exe_path + " "+taskdata_dir+" "+trail_id+" "+outputDir_pp)

def pp_cloud_cut(outputDir_pp, link_id, link_out_dir):
    print("......感知结果点云合并与截取："+link_id)
    #搜索所有的link_id前缀的感知结果点云
    move_files_with_prefix(outputDir_pp,link_out_dir, link_id)

    #将所有点云数据进行合并
    combine_pcd_file = os.path.join(link_out_dir, 'pp_combinePcd.pcd')
    combinePcdFiles_exe_path = fsd_mapbuild_root_folder + "/roadMapping/bin/combinePcdFiles"
    os.system(combinePcdFiles_exe_path+" "+link_out_dir+" "+combine_pcd_file+" "+link_id)

    if not os.path.exists(combine_pcd_file):
        return

    parse_json_path = os.path.join(link_out_dir, 'parse_json.json')
    out_pcd_path = os.path.join(link_out_dir, 'pp_transCloud.pcd')
    pcl_cloud_prcess_path = fsd_mapbuild_root_folder + "/roadMapping/bin/pclcloudprocess"
    os.system(pcl_cloud_prcess_path+" "+link_out_dir+" "+combine_pcd_file+" "+parse_json_path+" "+out_pcd_path)


def run_preception_jiexi(workspace_folder, task_id, fsd_map_output_dir, link_list):
    taskdata_dir = os.path.join(workspace_folder, task_id, "data")
    if not os.path.exists(taskdata_dir): #感知结果文件夹不存在，不处理感知数据
        return
    outputDir_pp = fsd_map_output_dir+"/preception" #所有感知结果放入一个文件夹中
    os.system("mkdir "+outputDir_pp)

    trail_list = get_subfolders_only_number_name(taskdata_dir)
    
    #多线程处理
    p = Pool(10)
    for trail_id in trail_list:
        p.apply_async(run_preception_once, args=(taskdata_dir, trail_id, outputDir_pp))

    p.close()
    p.join()

    #将数据移动到相应的link中
    pp = Pool(2)
    for link_id in link_list:
        link_out_dir = os.path.join(fsd_map_output_dir, link_id)
        pp.apply_async(pp_cloud_cut, args=(outputDir_pp, link_id, link_out_dir))

    pp.close()
    pp.join()

    #将感知文件夹删除
    if os.path.exists(outputDir_pp):
        shutil.rmtree(outputDir_pp, ignore_errors=True)

def parse_output(fsd_map_output_dir,workspace_folder,task_id,utm_num,t_utm_world,road_branch,info_json_path)->int:
    #将参数写入文件公用
    parse_json_path = os.path.join(fsd_map_output_dir, 'parse_json.json')
    parser_dict = {}
    parser_dict['workspace_folder'] = workspace_folder
    parser_dict['task_id'] = task_id
    parser_dict['utm_num'] = utm_num
    parser_dict['t_utm_world'] = t_utm_world
    parser_dict['road_branch'] = road_branch

    #读取任务信息
    with open(info_json_path,'r',encoding='utf8')as fp:
        json_data = json.load(fp)
    fp.close()

    parser_dict['task_geom'] = json_data['middle']['task_geom']
    parser_dict['link_geom'] = json_data['middle']['link_geom']

    #获取轨迹
    trail_list = []
    for trail_json in json_data['middle']['tracks']:
        trail_list.append(trail_json['trail_id'])
    trail_list_set = list(set(trail_list))
    parser_dict['trail_ids'] = trail_list_set

    #将link_geom内数据进行偏转
    # link_geom = loads(json_data['middle']['link_geom'])
    # link_geom_coords = list(link_geom.coords)
    # print(link_geom_coords)

    #输出任务信息
    with open(parse_json_path,'w',encoding='utf8')as fp:
        json.dump(parser_dict,fp,ensure_ascii=False)
    fp.close()

    return 0


def parse_output_multi_links(fsd_map_output_dir,workspace_folder,task_id,road_branch,info_json_path):

    #读取任务信息
    with open(info_json_path,'r',encoding='utf8')as fp:
        json_data = json.load(fp)
    fp.close()

    #获取所有的link_id
    utm_num = json_data['utm_num']
    t_utm_world = json_data['t_utm_world']
    link_list = []
    for link in json_data['middle']['links']:
        parser_dict = {}
        parser_dict['workspace_folder'] = workspace_folder
        parser_dict['task_id'] = task_id
        parser_dict['utm_num'] = utm_num
        parser_dict['t_utm_world'] = t_utm_world
        parser_dict['road_branch'] = road_branch
        parser_dict['link_id'] = link['link_id']
        parser_dict['task_geom'] = link['task_geom']
        parser_dict['link_geom'] = link['link_geom']
        parser_dict['link_direction'] = link['link_direction']
        if str(link['link_id']) in  link_list:
            print(str(link['link_id'])+" 找到重复link，跳出")
            continue
        link_list.append(str(link['link_id']))
        #获取轨迹
        trail_list = []
        for trail_json in link['tracks']:
            trail_list.append(trail_json['trail_id'])
        trail_list_set = list(set(trail_list))
        parser_dict['trail_ids'] = trail_list_set

        #将link_geom内数据进行偏转
        # link_geom = loads(json_data['middle']['link_geom'])
        # link_geom_coords = list(link_geom.coords)
        # print(link_geom_coords)

        #输出任务信息
        outputDir = os.path.join(fsd_map_output_dir,str(parser_dict['link_id']))
        os.system("mkdir "+outputDir)
        parse_json_path = os.path.join(outputDir, 'parse_json.json')
        with open(parse_json_path,'w',encoding='utf8')as fp:
            json.dump(parser_dict,fp,ensure_ascii=False)
        fp.close()

    precess_link_nodes = ProcessLinkNode(fsd_map_output_dir, workspace_folder, task_id, info_json_path)

    #重写一份任务信息进入建模阶段
    all_dict = {}
    all_dict['data_folder'] = fsd_map_output_dir
    all_dict['workspace_folder'] = workspace_folder
    all_dict['task_id'] = task_id
    all_dict['utm_num'] = utm_num
    all_dict['t_utm_world'] = t_utm_world
    all_dict['road_branch'] = road_branch
    all_dict['links'] = link_list

    if 'task_type' in json_data['middle'].keys():
        all_dict['task_type'] = json_data['middle']['task_type']
    if 'task_geom' in json_data['middle'].keys():
        all_dict['task_geom'] = json_data['middle']['task_geom']

    all_dict['custom_site_poly_50m'] = precess_link_nodes.get_inner_link_poly_50m_wkt()

    #解析中心点坐标
    center_pt_list = []
    if 'sub_crosses' in json_data['middle'].keys():
        print("sub_crosses:",json_data['middle']['sub_crosses'])
        if len(json_data['middle']['sub_crosses']) > 0:
            for one_crosses in json_data['middle']['sub_crosses']:
                if 'center' in one_crosses.keys():
                    center_pt_list.append(one_crosses['center'])
            all_dict['sub_crosses_center'] = center_pt_list

    task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
    with open(task_json_path,'w',encoding='utf8')as fp:
        json.dump(all_dict,fp,ensure_ascii=False)
    fp.close()

    return link_list


def run_mapbuild_singleLink(linkid:str, fsd_map_output_dir:str, mono_link_mapping_dir:str, fsd_mapbuild_root_folder, label)->int:
    """
    Arguments:单link自动化建模
    ---------
    Returns
    -------
    """
    print("==================点云范围截取==========================")
    #pcd_file_path = os.path.join(mono_link_mapping_dir, linkid+'.pcd'
    pcd_file_path = os.path.join(mono_link_mapping_dir, "global_cloud.pcd")
    # pcd_file_path = os.path.join(mono_link_mapping_dir, 'global_cloud.pcd')
    print(pcd_file_path)
    if not os.path.exists(pcd_file_path):
        print("!没有点云请检查："+pcd_file_path)
        return

    link_out_dir = os.path.join(fsd_map_output_dir, linkid)

    parse_json_path = os.path.join(link_out_dir, 'parse_json.json')
    out_pcd_path = os.path.join(link_out_dir, 'transCloud.pcd')
    pcl_cloud_prcess_path = fsd_mapbuild_root_folder + "/roadMapping/bin/pclcloudprocess"
    cmd = pcl_cloud_prcess_path+" "+link_out_dir+" "+pcd_file_path+" "+parse_json_path+" "+out_pcd_path
    print(cmd)
    os.system(cmd)

    trans_cloud_pcd_path = link_out_dir + "/transCloud.pcd"
    if not os.path.exists(trans_cloud_pcd_path):
        return 

    print("==================车道线矢量化=====================")
    lane_type = "lane_line"
    outputDir_laneBoundary = link_out_dir+"/laneBoundaryFromSeg"
    os.system("mkdir "+outputDir_laneBoundary)

    postRoadModel_path = fsd_mapbuild_root_folder +"/roadMapping/bin/postRoadModel"
    cmd = postRoadModel_path + " "+link_out_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_laneBoundary+" "+label+" "+lane_type
    print(cmd)
    os.system(cmd)

    print("==================车道中心线矢量化==========================")
    lane_type = "lane_center"
    outputDir_laneCenter = link_out_dir+"/laneCenterFromSeg"
    os.system("mkdir "+outputDir_laneCenter)

    postRoadModel_path = fsd_mapbuild_root_folder +"/roadMapping/bin/postRoadModel"
    cmd = postRoadModel_path + " "+link_out_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_laneCenter+" "+label+" "+lane_type
    print(cmd)
    os.system(cmd)

    print("==================道路边界矢量化==========================")
    outputDir_roadBoundary = link_out_dir+"/roadBoundaryFromSeg"
    os.system("mkdir "+outputDir_roadBoundary)

    roadBoundaryModel_path = fsd_mapbuild_root_folder + "/roadMapping/bin/roadBoundaryModel"
    cmd = roadBoundaryModel_path+" "+link_out_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_roadBoundary+" " + label
    print(cmd)
    os.system(cmd)
    #os.system(roadBoundaryModel_path+" "+link_out_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_roadBoundary+" "+label)


def run_temp_road_bdr(workspace_folder,task_id,fsd_map_output_dir) -> int:

    outputDir_roadBoundary = fsd_map_output_dir+"/roadBoundaryFromSeg"
    os.system("mkdir "+outputDir_roadBoundary)

    parse_json_path = os.path.join(fsd_map_output_dir, 'parse_json.json')
    pcd_file_path = os.path.join(workspace_folder, task_id, 'mapping_output', 'global_map_ground.pcd')

    # print("==================感知数据读入==========================")
    # cp_preception_files(fsd_map_output_dir, workspace_folder,task_id)

    print("==================点云范围截取==========================")
    pcl_cloud_prcess_path = fsd_mapbuild_root_folder + "/roadMapping/bin/pclcloudprocess"
    os.system(pcl_cloud_prcess_path+" "+fsd_map_output_dir+" "+pcd_file_path)

    trans_cloud_pcd_path = fsd_map_output_dir + "/transCloud.pcd"
    if not os.path.exists(trans_cloud_pcd_path):
        os.system("cp "+pcd_file_path + " "+trans_cloud_pcd_path)

    print("==================道路边界矢量化==========================")
    roadBoundaryModel_path = fsd_mapbuild_root_folder + "/roadMapping/bin/roadBoundaryModel"
    os.system(roadBoundaryModel_path+" "+fsd_map_output_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_roadBoundary)

    outputDir_laneBoundary = fsd_map_output_dir+"/laneBoundaryFromSeg"
    os.system("mkdir "+outputDir_laneBoundary)

    print("==================车道线、地面标识矢量化=====================")
    postRoadModel_path = fsd_mapbuild_root_folder +"/roadMapping/bin/postRoadModel"
    os.system(postRoadModel_path + " "+fsd_map_output_dir+" "+trans_cloud_pcd_path+" "+parse_json_path+" "+outputDir_laneBoundary)

    outputDir_roadMappingResult = fsd_map_output_dir+"/roadMappingResult"
    os.system("mkdir "+outputDir_roadMappingResult)

    print("=======================矢量化结果输出==========================")
    os.system("cp "+outputDir_roadBoundary+"/roadBoundary.json "+outputDir_roadMappingResult)
    os.system("cp "+outputDir_laneBoundary+"/laneBoundary.json "+outputDir_roadMappingResult)
    # os.system("cp "+outputDir_laneBoundary+"/trafficArrow.json "+outputDir_roadMappingResult)

    print("=======================结果上传==========================")
    upload_path = fsd_mapbuild_root_folder +"/roadMapping/bin/uploadLane"
    os.system(upload_path + " "+fsd_map_output_dir+" "+parse_json_path)
    return 0

def run_a_process(cmd: str, timeout_s=None):
    print("run " + cmd)
    try:
        subprocess.run(cmd.split(" "), timeout=timeout_s)
        return None
    # 超时会触发异常
    except Exception as e:
        print("🔴 Error in run_a_process "+e)
        return e


def run_upload_proxy(workspace_folder,task_id,info_json_path,utm_num,road_branch) -> int:
    inputDir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out')
    # outputDir = os.path.join(inputDir, 'mapfusion')
    # if not os.path.exists(outputDir):
    #     os.mkdir(outputDir)

    error_code = 0
    match_fusion_entry = fsd_mapbuild_root_folder + "/bundlematch/script/upload_map.py"
    match_fusion_cmd = "python3 {} --data_path={} --json_path={} --utm_zone={} --road_branch={}".format(
        match_fusion_entry,
        inputDir,
        info_json_path,
        utm_num,
        road_branch)

    # os.system(match_fusion_cmd)
    ret = run_a_process(match_fusion_cmd, 3600)
    ret = None

    if ret is not None:
        error_code = 501  # 超时退出
        return error_code

    return error_code

# def run(workspace_folder,task_id,utm_num,t_utm_world,road_branch,info_json_path) -> int:
#     print("============================================")
#     print("             lidar要素矢量化建模              ")
#     print("============================================")
#     fsd_map_output_dir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out')
#     if not os.path.exists(fsd_map_output_dir):
#         os.makedirs(fsd_map_output_dir)

#     error_code = parse_output(fsd_map_output_dir,workspace_folder,task_id,utm_num,t_utm_world,road_branch,info_json_path)
#     if error_code != 0:
#         return error_code

#     error_code = run_temp_road_bdr(workspace_folder,task_id,fsd_map_output_dir)
#     if error_code != 0:
#         return error_code

#     # parse_json_path = os.path.join(fsd_map_output_dir, 'parse_json.json')
#     # print("============================================")
#     # print("             结果上传和输出                   ")
#     # print("============================================")
#     # error_code = run_upload_proxy(workspace_folder,task_id,parse_json_path,utm_num,road_branch)
#     return error_code

def run_one_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type)->int:
    """
    Arguments:所有link进行自动化建模
    ---------
    Returns
    -------
    """
    #根据label确认不同处理的global_pcd_path
    global_pcd_path:str = ''
    # if label == 'label':
    #     global_pcd_path = os.path.join(workspace_folder, task_id, "mapping_output", "global_map_ground.pcd")
    # else:
    global_pcd_path = os.path.join(workspace_folder, task_id, "model_pcd", "global_cloud.pcd")
       
    if not os.path.exists(global_pcd_path):
        print("没有找到global点云："+global_pcd_path)
        return 0

    print("============================================")
    print("             lidar要素矢量化建模              ")
    print("============================================")
    if label == 'label':
        fsd_map_output_dir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out')
        if not os.path.exists(fsd_map_output_dir):
            os.makedirs(fsd_map_output_dir)
    elif label == 'cloud_line_seg':
        fsd_map_output_dir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out_cloud_line_seg')
        if not os.path.exists(fsd_map_output_dir):
           os.makedirs(fsd_map_output_dir)
    elif label == 'cloud_pano_seg':
        fsd_map_output_dir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out_cloud_pano_seg')
        if not os.path.exists(fsd_map_output_dir):
            os.makedirs(fsd_map_output_dir)
    elif label == 'cloud_bev_label':
        fsd_map_output_dir = os.path.join(workspace_folder, task_id, 'fsd_mapbuild_out_cloud_bev_label')
        if not os.path.exists(fsd_map_output_dir):
            os.makedirs(fsd_map_output_dir)

    print("==============分解参数文件===========")
    link_list = parse_output_multi_links(fsd_map_output_dir,workspace_folder,task_id,road_branch,info_json_path)

    print("==============解析感知结果===========")
    run_preception_jiexi(workspace_folder, task_id, fsd_map_output_dir, link_list)

    # # 处理路口附近的边界线
    run_boundary_extract = True
    if run_boundary_extract:
        print("start to processSiteBoundary")
        outputDir_site_boundary = fsd_map_output_dir+"/site_boundary"
        os.system("mkdir " + outputDir_site_boundary)
        os.system("mkdir -p " + outputDir_site_boundary+"/debug")
        task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
        processSiteBoundary_exe = fsd_mapbuild_root_folder + "/roadMapping/bin/processSiteBoundary"
        cmd = processSiteBoundary_exe+" "+global_pcd_path+" "+task_json_path+" "+outputDir_site_boundary
        print(cmd)
        os.system(cmd)

    # #获取global点云
    # mono_link_mapping_dir = os.path.join(workspace_folder,task_id,'monolink')
    # if not os.path.exists(mono_link_mapping_dir):
    #     print("没有找到monolink点云文件夹："+mono_link_mapping_dir)
    #     return 0

    #多线程处理
    # p = Pool(12)
    p = Pool(6)
    # p = Pool(1)
    for linkid in link_list:
        p.apply_async(run_mapbuild_singleLink, args=(linkid, fsd_map_output_dir, os.path.join(workspace_folder, task_id, "model_pcd"), fsd_mapbuild_root_folder, label))
    p.close()
    p.join()

    # 处理路口附近的边界线
    # run_boundary_extract = True
    # if run_boundary_extract:
    #     print("start to processSiteBoundary")
    #     outputDir_site_boundary = fsd_map_output_dir+"/site_boundary"
    #     os.system("mkdir " + outputDir_site_boundary)
    #     os.system("mkdir -p " + outputDir_site_boundary+"/debug")
    #     task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
    #     processSiteBoundary_exe = fsd_mapbuild_root_folder + "/roadMapping/bin/processSiteBoundary"
    #     cmd = processSiteBoundary_exe+" "+global_pcd_path+" "+task_json_path+" "+outputDir_site_boundary
    #     print(cmd)
    #     os.system(cmd)
    
    #进行车道线和道路边界的合并
    print("==============合并车道线关键点===========")
    joinKeyPoints_exe = fsd_mapbuild_root_folder + "/roadMapping/bin/joinKeyPoints"
    task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
    suffix = "_trjCloud.pcd"
    outpcdPath_lanebd = os.path.join(fsd_map_output_dir, "laneboundary.pcd")
    os.system(joinKeyPoints_exe+" "+fsd_map_output_dir+" "+task_json_path+" "+"laneboundary"+" "+suffix+" "+outpcdPath_lanebd)
    
    print("==============合并车道中心线关键点===========")
    outpcdPath_lanecenter = os.path.join(fsd_map_output_dir, "lanecenter.pcd")
    os.system(joinKeyPoints_exe+" "+fsd_map_output_dir+" "+task_json_path+" "+"lanecenter"+" "+suffix+" "+outpcdPath_lanecenter)
    

    print("==============合并道路边界关键点===========")
    outpcdPath_roadbd = os.path.join(fsd_map_output_dir, "roadboundary.pcd")
    os.system(joinKeyPoints_exe+" "+fsd_map_output_dir+" "+task_json_path+" "+"roadboundary"+" "+suffix+" "+outpcdPath_roadbd)
    
    #进行路口面的处理
    with open(task_json_path,'r',encoding='utf8')as fp:
        json_data = json.load(fp)
    fp.close()

    if (label == 'cloud_pano_seg'):            
        print("------>接入bev结果：")
        bev_out_path = os.path.join(fsd_map_output_dir, "bev_obj")
        json_to_obj_exe = road_model_folder + "/bev_road_model/bin/json_to_obj"
        cmd_str = "{} --flagfile={} --base_dir={} --output_path={} --middle_json_path={}".format(
        json_to_obj_exe,
        road_model_folder + "/bev_road_model/conf/json_to_obj.ini",
        os.path.join(workspace_folder, task_id, "tile_download"),
        bev_out_path + " --key_pose_file=imgpos_gcj.txt --use_middle_enable=true",
        info_json_path
        )
        print(cmd_str)
        os.system(cmd_str)
            
    #地面标识矢量化
    if (label == 'cloud_pano_seg' or label == 'cloud_bev_label'):
        print("==============地面标识矢量化===========")
        outputDir_roadMark = fsd_map_output_dir+"/RoadMarkRes"
        os.system("mkdir "+outputDir_roadMark)

        roadmark_exe = fsd_mapbuild_root_folder + "/roadMapping/bin/postRoadMark"

        # task_path = os.path.join(workspace_folder, task_id)
        task_path = fsd_map_output_dir
        task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
        cmd = roadmark_exe+" "+task_path+" "+global_pcd_path+" "+task_json_path+" "+outputDir_roadMark
        print(cmd)
        os.system(cmd)
            
    if ('task_type' in json_data.keys()) and (label == 'cloud_pano_seg' or label == 'cloud_bev_label'): #说明此任务为路口任务,目前停止线采用车端label 人行横道采用云端bev_label+全景分割 结果都输出在fsd_mapbuild_out文件夹中
        if (upload_lane == '0') or json_data['task_type'] == "lk2" :
            print("------此任务为离散任务，需执行路口建模------")
            outputDir_lukou = fsd_map_output_dir+"/LuKou"
            os.system("mkdir "+outputDir_lukou)
        
            lukou_exe = fsd_mapbuild_root_folder + "/roadMapping/bin/postLuKou"
            # fin_in_pcd_file_path = os.path.join(workspace_folder, task_id, 'mapping_output', 'global_map_ground.pcd')
            # task_path = os.path.join(workspace_folder, task_id)
            task_path = fsd_map_output_dir
            task_json_path = os.path.join(fsd_map_output_dir, 'fsd_mapbuild_task.json')
            cmd = lukou_exe+" "+fsd_map_output_dir+" "+global_pcd_path+" "+task_json_path+" "+outputDir_lukou+" "+info_json_path+" "+data_type
            print(cmd)
            os.system(cmd)
    
    if label == 'cloud_pano_seg' or label == 'cloud_bev_label':
        print("=======================后处理与结果上传==========================")
        outputDir_roadMappingResult = fsd_map_output_dir+"/roadMappingResult"
        os.system("mkdir "+outputDir_roadMappingResult)
        upload_path = fsd_mapbuild_root_folder +"/roadMapping/bin/uploadLane"
        cmd = upload_path + " "+fsd_map_output_dir+" "+task_json_path+" "+global_pcd_path+" "+upload_lane
        print(cmd)
        os.system(cmd)

    if label == 'cloud_pano_seg' or label == 'cloud_bev_label':
        print("=======================将obj结果整理到一个文件夹==========================")
        object_arrow = os.path.join(fsd_map_output_dir, "RoadMarkRes", "object_arrow.pcd")
        dst_arrow = os.path.join(fsd_map_output_dir, "object_arrow.pcd")
        del_file_if_exist(dst_arrow)
        if os.path.exists(object_arrow):
            copy_file(object_arrow, dst_arrow)

        object_other_mark = os.path.join(fsd_map_output_dir, "RoadMarkRes", "object_other_mark.pcd")
        dst_other_mark = os.path.join(fsd_map_output_dir, "object_other_mark.pcd")
        del_file_if_exist(dst_other_mark)
        if os.path.exists(object_other_mark):
            copy_file(object_other_mark, dst_other_mark)
        
        object_stopline = os.path.join(fsd_map_output_dir, "LuKou", "object_stopline.pcd")
        dst_stopline = os.path.join(fsd_map_output_dir, "object_stopline.pcd")
        del_file_if_exist(dst_stopline)
        if os.path.exists(object_stopline):
            copy_file(object_stopline, dst_stopline)
        
        object_crosswalk = os.path.join(fsd_map_output_dir, "LuKou", "object_crosswalk.pcd")
        dst_crosswalk = os.path.join(fsd_map_output_dir, "object_crosswalk.pcd")
        del_file_if_exist(dst_crosswalk)
        if os.path.exists(object_crosswalk):
            copy_file(object_crosswalk, dst_crosswalk)
        
        object_lukoubd = os.path.join(fsd_map_output_dir, "LuKou", "junction_lukoubd.pcd")
        dst_lukoubd = os.path.join(fsd_map_output_dir, "junction_lukoubd.pcd")
        del_file_if_exist(dst_lukoubd)
        if os.path.exists(object_lukoubd):
            copy_file(object_lukoubd, dst_lukoubd)
            
        object_area = os.path.join(fsd_map_output_dir, "LuKou", "junction_impassablearea.pcd")
        dst_area = os.path.join(fsd_map_output_dir, "junction_impassablearea.pcd")
        del_file_if_exist(dst_area)
        if os.path.exists(object_area):
            copy_file(object_area, dst_area)
        
    result_json:str = ''
    if label == 'label':
        result_json =  os.path.join(workspace_folder, task_id,'fsd_mapbuild_out','result.json')
    elif label == 'cloud_pano_seg':
        result_json =  os.path.join(workspace_folder, task_id,'fsd_mapbuild_out_cloud_pano_seg','result.json')
    elif label == 'cloud_bev_label':
        result_json =  os.path.join(workspace_folder, task_id,'fsd_mapbuild_out_cloud_bev_label','result.json')    
    else:
        result_json =  os.path.join(workspace_folder, task_id,'fsd_mapbuild_out_cloud_line_seg','result.json')
    with open(result_json,'w',encoding='utf8') as fp:
        print(fp)
    fp.close()
    return 0

def run_all_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type):
    error_code:int = 0
    if label == 'label': #仅跑车端数据 建模 
        print("**********仅进行车端数据建模**********")
        error_code = run_one_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type)
    elif label == 'cloud_line_seg': #仅云端数据建模
        print("**********仅进行云端数据建模**********")
        error_code = run_one_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type)
    elif label == 'cloud_pano_seg':  # 仅云端全景分割建模
        print("**********仅进行云端数据建模**********")
        error_code = run_one_label(workspace_folder, task_id, road_branch, info_json_path, label, upload_lane,data_type)
    elif label == 'cloud_bev_label':  # 仅云端bev_label数据建模
        print("**********仅进行云端数据建模**********")
        error_code = run_one_label(workspace_folder, task_id, road_branch, info_json_path, label, upload_lane,data_type)
    else:
        print("**********先进行车端数据建模，再进行云端数据建模**********")
        run_one_label(workspace_folder,task_id,road_branch,info_json_path,"label",upload_lane,data_type)
        run_one_label(workspace_folder,task_id,road_branch,info_json_path,"cloud_line_seg",upload_lane,data_type)
        run_one_label(workspace_folder,task_id,road_branch,info_json_path,"cloud_pano_seg",upload_lane,data_type)
        run_one_label(workspace_folder,task_id,road_branch,info_json_path,"cloud_bev_label",upload_lane,data_type)
        error_code = 0
    return error_code

parser = argparse.ArgumentParser()
parser.add_argument('--workspace_folder', type=str, default=os.environ['HOME'] + "/lidar_mapping_ws")
parser.add_argument('--task_id', type=str, default="39942")
parser.add_argument('--road_branch', type=str, default='yxx2')
parser.add_argument('--info_json_path', type=str, default="/home/test/lidar_mapping_ws/39942/task_info.json")
parser.add_argument('--label', type=str, default='both') #云端label开关cloud_label
parser.add_argument('--upload_lane', type=str, default='0')
parser.add_argument('--data_type', type=str, default="", help="数据类型")

if __name__ == "__main__":
    args = parser.parse_args()
    workspace_folder = str(args.workspace_folder)
    task_id = str(args.task_id)
    road_branch = str(args.road_branch)
    info_json_path = str(args.info_json_path)
    label = str(args.label)
    upload_lane=str(args.upload_lane)
    data_type=str(args.data_type)
    error_code = run_all_label(workspace_folder,task_id,road_branch,info_json_path,label,upload_lane,data_type)
    print(error_code)
    
