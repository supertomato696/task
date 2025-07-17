import sys
import os
import argparse
import glob
import numpy as np
from scipy.spatial.transform import Rotation
fsd_mapbuild_root_folder = os.path.dirname(os.path.abspath(__file__))
sys.path.append(fsd_mapbuild_root_folder + "/../../script")

def qab_to_euler_fix_yaw_deg(qab: np.ndarray) -> float:
    fix_z2y2x = Rotation.from_quat(qab).as_euler('zyx', degrees=True)
    return fix_z2y2x[0]


parser = argparse.ArgumentParser()
parser.add_argument('--base_dir', type=str, default="./test_data/27544/tile_download")
parser.add_argument('--opt_dir', type=str, default="../opt_veh_pose")

fsd_mapbuild_root_folder = os.path.dirname(os.path.abspath(__file__))

def trans_ground_las_to_pcd(workspace_dir, to_path, utm_num, t_utm_world):
    bev_dir = fsd_mapbuild_root_folder + "/../fsd_mapbuild/bev_mapbuild"
    data_dir = os.path.join(workspace_dir, "tile_download")
    upload_path = bev_dir + "/build/las_to_pcd"
    upload_path += " --flagfile=" + bev_dir + "/conf/las_to_pcd.ini"
    upload_path += " --output_path=" + to_path
    upload_path += " --base_dir=" + data_dir
    upload_path += " --utm_zone=" + str(utm_num)
    upload_path += " --utm_center_x=" + str(t_utm_world[0])
    upload_path += " --utm_center_y=" + str(t_utm_world[1])
    upload_path += " --utm_center_z=" + str(t_utm_world[2])
    print(upload_path)
    return os.system(upload_path)

def trans_imgpos(base_dir, opt_dir):
    try:
        dir_path = "%s/*/*/%s" % (base_dir, "imgpos.txt")
        file_list = glob.glob(dir_path)
        for pos_file in file_list:
            trail_dir = os.path.dirname(pos_file)
            new_file = os.path.join(trail_dir, "imgpos_gcj.txt")
            trail_id = os.path.basename(trail_dir)
            tile_id = os.path.basename(os.path.dirname(trail_dir))
            opt_file = os.path.join(opt_dir, "opt_veh_pose-%s-%s.txt" % (tile_id, trail_id))
            opt_map = {}
            if os.path.exists(opt_file):
                with open(opt_file, 'r') as f:
                    lines = f.readlines()
                    title = True
                    for line in lines:
                        if title:
                            title = False
                            continue
                        line_data = line.strip().split(" ")
                        wgs_timestamp = line_data[0]
                        wgs_lon = float(line_data[1])
                        wgs_lat = float(line_data[2])
                        wgs_alt = line_data[3]
                        quat = np.array([float(line_data[4]), float(line_data[5]), float(line_data[6]), float(line_data[7])])
                        yaw = qab_to_euler_fix_yaw_deg(quat)
                        opt_map[wgs_timestamp] = (wgs_lon, wgs_lat, wgs_alt, yaw,
                                                  float(line_data[4]), float(line_data[5]), float(line_data[6]), float(line_data[7]))

            with open(pos_file, 'r') as f, open(new_file, 'w') as new_f:
                lines = f.readlines()
                for line in lines:
                    line_data = line.strip().split(" ")
                    wgs_timestamp = line_data[0]
                    wgs_lon = float(line_data[1])
                    wgs_lat = float(line_data[2])
                    wgs_alt = line_data[3]
                    wgs_yaw = line_data[4]
                    quat_0 = 0
                    quat_1 = 0
                    quat_2 = 0
                    quat_3 = 0
                    if wgs_timestamp in opt_map:
                        opt_obj = opt_map[wgs_timestamp]
                        gcj_lon = opt_obj[0]
                        gcj_lat = opt_obj[1]
                        wgs_alt = opt_obj[2]
                        wgs_yaw = opt_obj[3]
                        quat_0 = opt_obj[4]
                        quat_1 = opt_obj[5]
                        quat_2 = opt_obj[6]
                        quat_3 = opt_obj[7]
                    else:
                        continue
                        # gcj_lon, gcj_lat = wgs2gcj(wgs_lon, wgs_lat)
                    new_f.write(
                        wgs_timestamp + " " + str(gcj_lon) + " " + str(gcj_lat) + " " + str(wgs_alt) + " " + str(wgs_yaw)
                        + " " +str(quat_0) + " " + str(quat_1) + " " + str(quat_2) + " " + str(quat_3) + '\n')

    except Exception as e:
        print(e)
        return 1
    return 0


if __name__ == "__main__":
    args = parser.parse_args()
    trans_imgpos(args.base_dir, args.opt_dir)
 
