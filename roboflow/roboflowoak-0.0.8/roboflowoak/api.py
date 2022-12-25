import requests
import os
import json
from pip._internal.utils.appdirs import user_cache_dir

cache_dir = user_cache_dir('pip') + "/roboflow"

def download_blob(project, version, api_key, device_id, api_endpoint):

    endpoint = api_endpoint+"/oak/"+project+"/"+version+"?api_key="+api_key+"&device="+device_id
    api_data = requests.get(endpoint)

    if api_data.status_code != 200:
        raise Exception(str(api_data.json()))
    api_data = api_data.json()
    if 'warning' in api_data.keys():
        print(api_data['warning'], flush=True)

    model_objects = {}
    model_objects['class_names'] = api_data['oak']['classes']
    model_objects['colors'] = api_data['oak']['colors']
    model_objects['environment'] = requests.get(api_data['oak']['environment']).json()
    r = requests.get(api_data['oak']['model'])
    try:
        os.makedirs(os.path.join(cache_dir, project, version))
    except OSError as error:
        print(error)
    with open(os.path.join(cache_dir, project, version, 'roboflow.blob'), 'wb') as f:
        f.write(r.content)
    with open(os.path.join(cache_dir, project, version, 'config.txt'), 'w') as f:
        f.write(json.dumps(model_objects))

    return os.path.join(cache_dir, project, version)
