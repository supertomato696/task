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

def del_folder_if_exists(file_folder: str):
    if not os.path.exists(file_folder):
        return
    shutil.rmtree(file_folder)


if __name__ == "__main__":
    raw_cross_dir = "/mnt/d/04_dataset/1_dilabel/crowd_source/new_6lukou"
    input_bev_pose_dir="/mnt/d/04_dataset/1_dilabel/crowd_source/new_6lukou/bev_pose/20250107"

    # 1. 路口
    cross_dirs = os.listdir(raw_cross_dir)
    for cross_dir_name in cross_dirs:
        cross_dir = os.path.join(raw_cross_dir, cross_dir_name) # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/batch-195"
        if os.path.isdir(cross_dir) and cross_dir_name.startswith("batch-"):
            # if cross_dir_name not in ["batch-312", "batch-290","batch-10017","batch-10010"]:
            #     continue

            # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/output/batch-195/PL0169_event_ld"
            input_dir = os.path.join(input_bev_pose_dir, cross_dir_name+"_refine_pose") 
            output_dir = os.path.join(raw_cross_dir, cross_dir_name) 
            src_files = {
                os.path.join(input_dir, "multi_mapping") : os.path.join(output_dir, "model/source/bev_mapping/output/multi_mapping"),
            }

            for key, value in src_files.items():
                # print(key, value)
                if not os.path.exists(key):
                    continue

                if os.path.isdir(key):
                    del_folder_if_exists(value)
                    # dst_dir = os.path.dirname(value)
                    dst_dir = value
                    add_folder_if_no_exist(dst_dir)
                    copy_folder(key, dst_dir)
    print()