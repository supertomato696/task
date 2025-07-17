import os, sys
import shutil

import json

def load_json(file_name):
    # 打开并读取 JSON 文件
    with open(file_name, 'r') as file:
        data = json.load(file)
    return data

def copy_file(src_file: str, dst_file: str):
    if not os.path.exists(src_file):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    # if not os.path.exists(os.path.dirname(dst_file)):
    #     # shutil.rmtree(dst_dir, ignore_errors=True)
    #     os.makedirs(os.path.dirname(dst_file))

    shutil.copyfile(src_file, dst_file)

def copy_file_with_metadata(src_file: str, dst_file: str):
    if not os.path.exists(src_file):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    # if not os.path.exists(os.path.dirname(dst_file)):
    #     # shutil.rmtree(dst_dir, ignore_errors=True)
    #     os.makedirs(os.path.dirname(dst_file))

    shutil.copy2(src_file, dst_file)

def remove_folder(src_dir: str):
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)

def copy_folder(src_dir: str, dst_dir: str):
    if not os.path.exists(src_dir):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)
    shutil.copytree(src_dir, dst_dir)

def copy_folder_without_remove(src_dir: str, dst_dir: str):
    if not os.path.exists(src_dir):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return

    shutil.copytree(src_dir, dst_dir)


def add_folder_if_no_exist(folder_path: str):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def del_file_if_exist(file_path: str):
    if not os.path.exists(file_path):
        return
    os.remove(file_path)

def parse_cover_output(file_name):
    json_data = load_json(file_name)
    bev_unmet_link_list =  json_data['bev_unmet_link_list']

def mkdir_auto_model_dir(output_dir):
    src_files = {
        # bev mapping
        os.path.join(output_dir, "model/source/bev_mapping/output"),

        # parse
        os.path.join(output_dir, "model/source/data_engine/parse"),

        # cover
        os.path.join(output_dir, "model/source/data_engine/cover"),
        
        # match
        os.path.join(output_dir, "model/source/data_engine/match"),

        os.path.join(output_dir, "model/autolabel/input"),
    }

    for dir in src_files:
        add_folder_if_no_exist(dir)


if __name__ == "__main__":

    skip_pcd  = True
    skip_image = True
    # raw_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/new"
    raw_cross_dir_base = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ziyan"
    raw_cross_dir = os.path.join(raw_cross_dir_base, "ziyan_4lukou_origin")
    out_raw_cross_dir = os.path.join(raw_cross_dir_base, "ziyan_4lukou")

    if os.path.exists(out_raw_cross_dir):
        shutil.rmtree(out_raw_cross_dir, ignore_errors=True)
    add_folder_if_no_exist(out_raw_cross_dir)

    # 1. 路口
    cross_dirs = os.listdir(raw_cross_dir)
    for cross_dir_name in cross_dirs:
        cross_dir = os.path.join(raw_cross_dir, cross_dir_name) # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/batch-195"
        if os.path.isdir(cross_dir) and cross_dir_name.startswith("batch-"):
            # if cross_dir_name not in ["batch-312", "batch-290","batch-10017","batch-10010"]:
            # if cross_dir_name not in ["batch-10010"]:
            #     continue

            # 路口里的包
            cross_dir_for_cover = os.path.join(cross_dir, "cover/output")
            cross_dir_for_match = os.path.join(cross_dir, "cover/input/match")
            cross_dir_for_parse = os.path.join(cross_dir, "cover/input/parse")
            bag_dirs = os.listdir(cross_dir_for_match)
            for bag_dir_name in bag_dirs: 
                bag_dir = os.path.join(cross_dir_for_match, bag_dir_name) # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/batch-195/PL0169_event_ld"
                if os.path.isdir(bag_dir) and bag_dir_name.startswith("2025"):

                    # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/output/batch-195/PL0169_event_ld"
                    input_dir_match = os.path.join(cross_dir_for_match, bag_dir_name) 
                    input_dir_parse = os.path.join(cross_dir_for_parse, bag_dir_name) 
                    output_dir = os.path.join(out_raw_cross_dir, cross_dir_name) 
                    src_files = {
                        # 给自动建模
                        # platform
                        os.path.join(cross_dir_for_cover, "../input/tasks.json") : os.path.join(output_dir, "model/auto_label/input/tasks.json"),
                        os.path.join(cross_dir_for_cover, "../input/sd_link.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_link.geojson"),
                        os.path.join(cross_dir_for_cover, "../input/sd_node.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_node.geojson"),
                        
                        # cover
                        os.path.join(cross_dir_for_cover, "output.json") : os.path.join(output_dir, "model/source/data_engine/cover/output.json"),
                        
                        # match
                        input_dir_match : os.path.join(output_dir, "model/source/data_engine/match/{}".format(bag_dir_name)),
                         
                        # parse
                        input_dir_parse : os.path.join(output_dir, "model/source/data_engine/parse/{}".format(bag_dir_name)),
                    }

                    for key, value in src_files.items():
                        # print(key, value)
                        if not os.path.exists(key):
                            continue

                        if os.path.isdir(key):
                            # pass
                            add_folder_if_no_exist(value)
                            copy_folder(key, value)
                        else:
                            dst_dir = os.path.dirname(value)
                            add_folder_if_no_exist(dst_dir)

                            copy_file_with_metadata(key, value)

    print()