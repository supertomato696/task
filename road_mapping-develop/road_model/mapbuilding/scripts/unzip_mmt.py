import os
import zipfile
import shutil

def get_files_in_directory(directory):
    return [f for f in os.listdir(directory) if f.endswith('zip')]

def unzip():
    base_directory = 'D:/zhongbao/mmt/6'
    files = get_files_in_directory(base_directory)
    password = '20241023'

    for file in files:
        unzip_dir = '{}/{}'.format(base_directory,file.split('.')[0])
        zip_path = '{}/{}'.format(base_directory,file)
        if os.path.exists(unzip_dir):
            shutil.rmtree(unzip_dir)
        os.makedirs(unzip_dir)

        with zipfile.ZipFile(zip_path, 'r') as zFile:
            zip_names = zFile.namelist()
            target_files = [name for name in zip_names if name.count('/') == 2 and ((name.endswith('/') or name.count('_sensor_gnss_rtk') == 1 or name.count('_mla_egopose') == 1 or name.count('_ddld_landmark') == 1))]
            for target_file in target_files:
                zFile.extract(target_file,'D:/zhongbao/mmt/6/output')
                # zFile.extract(target_file,unzip_dir,pwd=password.encode('utf-8'))
        zFile.close()

if __name__ == '__main__':
    unzip()
    print('done')