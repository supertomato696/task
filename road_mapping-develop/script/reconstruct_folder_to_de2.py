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
    raw_cross_dir = "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2"
    out_raw_cross_dir = os.path.join(raw_cross_dir, "output")
    raw_cross_dir = os.path.join(raw_cross_dir, "task_engine")

    # if os.path.exists(out_raw_cross_dir):
    #     shutil.rmtree(out_raw_cross_dir, ignore_errors=True)

    # 1. 路口
    cross_dirs = os.listdir(raw_cross_dir)
    for cross_dir_name in cross_dirs:
        cross_dir = os.path.join(raw_cross_dir, cross_dir_name) # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/batch-195"
        if os.path.isdir(cross_dir) and cross_dir_name.startswith("batch-"):
            # if cross_dir_name not in ["batch-312", "batch-290","batch-10017","batch-10010"]:
            if cross_dir_name not in ["batch-10010"]:
                continue

            # 路口里的包
            cross_dir_for_cover = os.path.join(cross_dir, "cover/output")
            cross_dir = os.path.join(cross_dir, "cover/input/match")
            bag_dirs = os.listdir(cross_dir)
            for bag_dir_name in bag_dirs: 
                bag_dir = os.path.join(cross_dir, bag_dir_name) # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/batch-195/PL0169_event_ld"
                if os.path.isdir(bag_dir) and bag_dir_name.startswith("PL"):

                    # "/mnt/d/01_code/04_dataset/dilabel/crowd_source/ten_cross2/output/batch-195/PL0169_event_ld"
                    input_dir = os.path.join(cross_dir, bag_dir_name) 
                    output_dir = os.path.join(out_raw_cross_dir, cross_dir_name) 
                    src_files = {
                        # 给匹配
                        # os.path.join(raw_cross_dir, "task.json") : os.path.join(output_dir, "data_engine/match/input/task.json"),
                        # os.path.join(raw_cross_dir, "sd_link.geojson") : os.path.join(output_dir, "data_engine/match/input/sd_link.geojson"),
                        # os.path.join(raw_cross_dir, "sd_node.geojson") : os.path.join(output_dir, "data_engine/match/input/sd_node.geojson"),
                        # os.path.join(input_dir, "_sensor_gnss_rtk.json") : os.path.join(output_dir, "data_engine/parse/output/{}/position/_sensor_gnss_rtk.json".format(bag_dir_name)),
                        # os.path.join(input_dir, "_mla_egopose.json") : os.path.join(output_dir, "data_engine/parse/output/{}/position/_mla_egopose.json".format(bag_dir_name)),
                        
                        # 给自动建模
                        # os.path.join(raw_cross_dir, "task.json") : os.path.join(output_dir, "model/auto_label/input/task.json"),
                        # os.path.join(raw_cross_dir, "sd_link.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_link.geojson"),
                        # os.path.join(raw_cross_dir, "sd_node.geojson") : os.path.join(output_dir, "model/auto_label/input/sd_node.geojson"),
                        # os.path.join(input_dir, "_sensor_gnss_rtk.json") : os.path.join(output_dir, "model/source/data_engine/parse/{}/position/_sensor_gnss_rtk.json".format(bag_dir_name)),
                        # os.path.join(input_dir, "_mla_egopose.json") : os.path.join(output_dir, "model/source/data_engine/parse/{}/position/_mla_egopose.json".format(bag_dir_name)),
                        # os.path.join(input_dir, "_ddld_landmark.json") : os.path.join(output_dir, "model/source/data_engine/parse/{}/bev_features/_ddld_landmark.json".format(bag_dir_name)),
                        # os.path.join(input_dir, "_perception_fusion_object.json") : os.path.join(output_dir, "model/source/data_engine/parse/{}/bev_features/_perception_fusion_object.json".format(bag_dir_name)),
                        # os.path.join(input_dir, "_worldmodel_traffic_light.json") : os.path.join(output_dir, "model/source/data_engine/parse/{}/bev_features/_worldmodel_traffic_light.json".format(bag_dir_name)),
                        
                        # # cover
                        os.path.join(cross_dir_for_cover, "output.json") : os.path.join(output_dir, "model/source/data_engine/cover/output.json"),

                        # # match
                        os.path.join(input_dir, "_mla_egopose_gps_matched.csv") : os.path.join(output_dir, "model/source/data_engine/match/{}/_mla_egopose_gps_matched.csv".format(bag_dir_name)),
                        os.path.join(input_dir, "_mla_egopose_gps_matched.json") : os.path.join(output_dir, "model/source/data_engine/match/{}/_mla_egopose_gps_matched.json".format(bag_dir_name)),
                        os.path.join(input_dir, "_sensor_gnss_rtk_gps_matched.csv") : os.path.join(output_dir, "model/source/data_engine/match/{}/_sensor_gnss_rtk_gps_matched.csv".format(bag_dir_name)),
                        os.path.join(input_dir, "_sensor_gnss_rtk_gps_matched.json") : os.path.join(output_dir, "model/source/data_engine/match/{}/_sensor_gnss_rtk_gps_matched.json".format(bag_dir_name)),
                        os.path.join(input_dir, "data_label.json") : os.path.join(output_dir, "model/source/data_engine/match/{}/data_label.json".format(bag_dir_name)),
                        os.path.join(input_dir, "trail_match.json") : os.path.join(output_dir, "model/source/data_engine/match/{}/trail_match.json".format(bag_dir_name)),
                    }

                    for key, value in src_files.items():
                        # print(key, value)
                        if not os.path.exists(key):
                            continue

                        if os.path.isdir(key):
                            pass
                            # dst_dir = os.path.join(raw_dir, value)
                            # add_folder_if_no_exist(dst_dir)
                            # copy_folder(key, dst_dir)
                        else:
                            dst_dir = os.path.dirname(value)
                            add_folder_if_no_exist(dst_dir)

                            copy_file_with_metadata(key, value)

    print()