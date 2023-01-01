"""
Gerard Harkema

based on source from roboflow
"""

import requests
import os
import json
from pip._internal.utils.appdirs import user_cache_dir
import sys

output_dir = "." #user_cache_dir('pip') + "/roboflow"

def download_blob(project, version, api_key, device_id, api_endpoint):

    endpoint = api_endpoint+"/oak/"+project+"/"+version+"?api_key="+api_key+"&device="+device_id
    api_data = requests.get(endpoint)

    if api_data.status_code != 200:
        raise Exception(str(api_data.json()))
    api_data = api_data.json()
    if 'warning' in api_data.keys():
	pass
        #print(api_data['warning'], flush=True)

    model_objects = {}
    model_objects['class_names'] = api_data['oak']['classes']
    model_objects['colors'] = api_data['oak']['colors']
    model_objects['environment'] = requests.get(api_data['oak']['environment']).json()
    r = requests.get(api_data['oak']['model'])
    file_name_base = project + '_V' + version
    file_name_blob = file_name_base + '.blob'
    file_name_config = file_name_base + '.txt'
    try:
        os.makedirs(os.path.join(output_dir, project))
    except OSError as error:
        print(error)
    with open(os.path.join(output_dir, project, file_name_blob), 'wb') as f:
        f.write(r.content)
    with open(os.path.join(output_dir, project, file_name_config), 'w') as f:
        f.write(json.dumps(model_objects))

    return os.path.join(output_dir, project)

if __name__ == '__main__':
    #print 'Number of arguments:', len(sys.argv), 'arguments.'
    #print 'Argument List:', str(sys.argv)
    #download_blob("simplefruits", "1", "K3sks4IiHf1jC7nMw6YN", "device", "https://api.roboflow.com")
    if len(sys.argv) != 4:
        print 'usage: ', str(sys.argv[0]), ' project version api_key'
    else:
        print download_blob(str(sys.argv[1]), str(sys.argv[2]), str(sys.argv[3]), "device", "https://api.roboflow.com")
