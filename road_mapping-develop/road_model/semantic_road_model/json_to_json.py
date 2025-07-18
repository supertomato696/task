'''
Description:
Author: ccj
Date: 2024-12-05 11:46:00
Reference:
'''

import json
import argparse

def load_json_file(file_path):
    """加载 JSON 文件"""
    with open(file_path, 'r', encoding='utf-8') as file:
        return json.load(file)

def save_json_file(file_path, data):
    """保存 JSON 数据到文件"""
    with open(file_path, 'w', encoding='utf-8') as file:
        json.dump(data, file, ensure_ascii=False, indent=4)

def fill_template(template, data):
    """根据数据填充模板"""
    if template is None or data is None:
        return template
    
    template['tile_branch'] = ""  
    template['road_branch'] = ""  
    template.get('tile_id_list', []).clear()
    template['utm_num'] = 50
    template['t_utm_world'] = [213767.755871096, 2516133.087930442, 0]

    # 将log中的每个字段设置为空
    if 'log' in template:
        for key in template['log'].keys():
            if isinstance(template['log'][key], str):
                template['log'][key] = ""  
            elif isinstance(template['log'][key], (int, float)):
                template['log'][key] = 0  
            elif isinstance(template['log'][key], dict):
                template['log'][key] = {}  
            elif isinstance(template['log'][key], list):
                template['log'][key] = []  
            elif isinstance(template['log'][key], bool):
                template['log'][key] = False  

    links = []
    if 'features' not in data:
        raise ValueError("数据中缺少 'features' 字段")

    link_template = template.get('middle', {}).get('links', [{}])[0]
    for feature in data['features']:
        link = link_template.copy()

        link['link_id'] = feature['properties']['id']
        link['link_geom'] = ""

        if 'coordinates' in feature.get('geometry', {}):
            lines = feature['geometry']['coordinates']
            wkt_linestring = "LINESTRING (" + ", ".join(
                    f"{coord[0]} {coord[1]}" for line in lines for coord in line
                ) + ")"  
            link['link_geom'] = wkt_linestring              

        link.update({
            'link_ids': [],
            'tile_ids': [],
            'tracks': [],
            'task_geom': "",
            'link_length': 0,
            'link_direction': int(feature['properties']['direction']),
            'crossing_id': []
        })

        links.append(link)

    template['middle']['links'] = links

    # 将sys_log字段中每一个字段都置空
    if 'sys_log' in template:
        if 'container' in template['sys_log']:
            for key in template['sys_log']['container'].keys():
                template['sys_log']['container'][key] = 0
        if 'host' in template['sys_log']:
            for key in template['sys_log']['host'].keys():
                template['sys_log']['host'][key] = 0
        template['sys_log']['stages'] = []  
    return template

parser = argparse.ArgumentParser()
parser.add_argument('template_file', type=str, default='', help='模板文件路径')
parser.add_argument('data_file', type=str, default='', help='数据文件路径')
parser.add_argument('output_file', type=str, default='', help='输出文件路径')

if __name__ == "__main__":
    args = parser.parse_args()

    template = load_json_file(args.template_file)
    data = load_json_file(args.data_file)

    filled_template = fill_template(template, data)

    save_json_file(args.output_file, filled_template)

    print(f"{args.output_file} 文件已生成。")

                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                