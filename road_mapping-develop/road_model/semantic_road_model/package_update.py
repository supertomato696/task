import platform
import subprocess
import os
from python.tools import file_tools


def get_exe_depend_so(exe_path: str) -> list:
    out_str = subprocess.Popen('ldd ' + exe_path,
                               stdout=subprocess.PIPE,
                               shell=True).communicate()[0].decode()
    out_lines = out_str.split('\n')
    so_paths = []
    for one_line_ in out_lines:
        splited = one_line_.split(' ')
        if len(splited) < 3:
            continue
        one_so_path = splited[2]
        if not one_so_path.startswith('/'):
            continue
        so_paths.append(one_so_path)
    return so_paths


def copy_so(bin_list: list, to_folder: str):
    so_paths = set()

    for one_bin in bin_list:
        so_paths.update(get_exe_depend_so(one_bin))

    good_so_paths = []

    one: str = str()
    for one in so_paths:
        if one.startswith('/lib/'):
            continue
        good_so_paths.append(one)

    for one in good_so_paths:
        print('{} ====> {}MB'.format(one, file_tools.get_filesize_MB(one)))
        file_name = file_tools.get_file_name_in_file_path(one)
        new_path = os.path.join(to_folder, file_name)
        file_tools.copy_file(src=one, dst=new_path)


def copy_bin(bin_list: list, from_proj_folder, to_proj_folder: str):
    for one_bin in bin_list:
        rel_path = file_tools.get_relpath(one_bin, from_proj_folder)
        new_path = os.path.join(to_proj_folder, rel_path)
        file_tools.copy_file(one_bin, new_path)


def copy_python(from_python_folder, to_python_folder):
    file_tools.copy_folder(from_python_folder, to_python_folder)


def package():
    develop_project_folder = os.path.abspath(__file__ + '/..') 
    develop_bin_list = [os.path.join(develop_project_folder, "hdmap_build/lib/hdmap_build")]

    exe_project_folder = os.path.join(develop_project_folder, '../mapbuild_exe/')
    exe_project_so_folder = os.path.join(exe_project_folder, 'so/')

    develop_python_folder1 = os.path.join(develop_project_folder, 'hdmapbase/')
    exe_project_python_folder1 = os.path.join(exe_project_folder, 'hdmapbase/')
    copy_python(develop_python_folder1, exe_project_python_folder1)

    develop_python_folder = os.path.join(develop_project_folder, 'python/')
    exe_project_python_folder = os.path.join(exe_project_folder, 'python/')
    copy_python(develop_python_folder, exe_project_python_folder)

    copy_so(develop_bin_list, exe_project_so_folder)
    copy_bin(develop_bin_list, develop_project_folder, exe_project_folder)

    run_from = os.path.join(develop_project_folder, 'update_run.py')
    run_to = os.path.join(exe_project_folder, 'update_run.py')
    file_tools.copy_file(run_from, run_to)


if __name__ == '__main__':
    package()
