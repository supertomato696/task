import os, sys
import shutil

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

def copy_folder(src_dir: str, dst_dir: str):
    if not os.path.exists(src_dir):
        # log.error('copy_folder(), no {}'.format(src_dir))
        return
    if os.path.exists(dst_dir):
        shutil.rmtree(dst_dir, ignore_errors=True)
    shutil.copytree(src_dir, dst_dir)


def add_folder_if_no_exist(folder_path: str):
    if not os.path.exists(folder_path):
        os.makedirs(folder_path)

def del_file_if_exist(file_path: str):
    if not os.path.exists(file_path):
        return
    os.remove(file_path)


if __name__ == "__main__":

    skip_pcd  = True
    skip_image = True
    # raw_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/new"
    raw_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/new/site_11002458234858"
    
    src_files = {
        os.path.join(raw_dir, "task.json") : "model/auto_label/input/task.json",
        os.path.join(raw_dir, "sd_link.geojson") : "model/auto_label/input/sd_link.geojson",
        os.path.join(raw_dir, "sd_node.geojson") : "model/auto_label/input/sd_node.geojson",

        os.path.join(raw_dir, "parse") : "model/source/data_engine/parse",
        os.path.join(raw_dir, "match") : "model/source/data_engine/match",
        os.path.join(raw_dir, "cover") : "model/source/data_engine/cover",

        os.path.join(raw_dir, "bev_mapping/output") : "model/source/bev_mapping/output",
        os.path.join(raw_dir, "lidar_mapping/output") : "model/source/lidar_mapping/output",
    }

    for key, value in src_files.items():
        # print(key, value)
        if not os.path.exists(key):
            continue

        if os.path.isdir(key):
            dst_dir = os.path.join(raw_dir, value)
            add_folder_if_no_exist(dst_dir)
            copy_folder(key, dst_dir)
        else:
            dst_dir = os.path.join(raw_dir, os.path.dirname(value))
            add_folder_if_no_exist(dst_dir)

            dst_file = os.path.join(raw_dir,value)
            copy_file_with_metadata(key, dst_file)

    print()