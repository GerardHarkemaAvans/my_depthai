from roboflowoak.pipe import DepthAIPipeline, list_devices
from roboflowoak.api import download_blob
import os
import json
import threading
from pip._internal.utils.appdirs import user_cache_dir
import shutil

cache_dir = user_cache_dir('pip') + "/roboflow"

class RoboflowOak:
    def __init__(self, model, version, api_key, confidence=0.2, overlap=0.5, advanced_config={}, rgb=False, depth=False, device=None, device_name="device", blocking=True, legacy=False, api_endpoint="https://api.roboflow.com"):
        self.project = model
        self.version = version
        self.api_key = api_key
        self.blocking = blocking
        self.depth = depth
        self.confidence = confidence
        self.overlap = overlap
        self.device_name = device_name
        self.dev = device
        self.api_endpoint = api_endpoint
        self.advanced_config = advanced_config
        self.fast = not len(self.advanced_config.keys()) > 0
        self.sensor_mode = "THE_1080_P"
        self.wide_fov = False
        if "sensor_mode" in self.advanced_config.keys():
            self.sensor_mode = advanced_config["sensor_mode"]
        if "wide_fov" in self.advanced_config.keys():
            self.wide_fov = advanced_config["wide_fov"]

        if self.dev is None:
            available_devices = list_devices()
            if len(available_devices) == 0:
                raise Exception("No available OAK devices detected.")
                default_device = None
            else:
                self.dev = available_devices[0]

        self.cache_path = self.find_weights()
        with open(os.path.join(self.cache_path, 'config.txt'), 'r') as f:
            self.model_objects = json.loads(f.read())
            self.size = (int(self.model_objects["environment"]["RESOLUTION"]), int(self.model_objects["environment"]["RESOLUTION"]))
            self.resolution = self.size
            self.class_names = self.model_objects["class_names"]
            self.colors = self.model_objects["colors"]
            #print("Size",self.size,"Resolution",self.resolution)
        try:
            self.dai_pipe = DepthAIPipeline(self.cache_path+"/roboflow.blob", self.size, self.resolution, self.class_names, rgb, self.colors, self.fast, self.confidence, self.overlap, self.sensor_mode, self.wide_fov, depth, device, legacy)
        except:
            try:
                shutil.rmtree(cache_dir)
                self.cache_path = self.find_weights()
                with open(os.path.join(self.cache_path, 'config.txt'), 'r') as f:
                    self.model_objects = json.loads(f.read())
                    self.size = (int(self.model_objects["environment"]["RESOLUTION"]), int(self.model_objects["environment"]["RESOLUTION"]))
                    self.resolution = self.size
                    self.class_names = self.model_objects["class_names"]
                    self.colors = self.model_objects["colors"]
                self.dai_pipe = DepthAIPipeline(self.cache_path+"/roboflow.blob", self.size, self.resolution, self.class_names, rgb, self.colors, self.fast, self.confidence, self.overlap, sensor_mode, wide_fov, depth, device, legacy)
            except:
                raise Exception("Failure while retrying load weights - does this model, version, api key exist? can you curl api.roboflow.com, and can your device download files from google cloud storage? have you hit your device limit?")

    def find_weights(self):
        if os.path.exists(os.path.join(cache_dir, self.project, self.version, "roboflow.blob")) and os.path.exists(
                        os.path.join(cache_dir, self.project, self.version, "config.txt")):
            return os.path.join(cache_dir, self.project, self.version)

        return download_blob(self.project, self.version, self.api_key, self.dev, self.api_endpoint)

    def detect(self):
        ret = self.dai_pipe.get()

        predictions = []
        dets = ret[0]
        for det in dets:
            if len(det) > 6:
                predictions.append(Prediction((det[0]+det[2])/2, (det[1]+det[3])/2, det[2]-det[0], det[3]-det[1], det[5], det[4], det[6]))
            else:
                predictions.append(Prediction((det[0]+det[2])/2, (det[1]+det[3])/2, det[2]-det[0], det[3]-det[1], det[5], det[4]))

        ret = list(ret)
        ret[0] = {"predictions": predictions}
        return ret

    def camera_control(self, control):
        self.dai_pipe.camera_control(control)

class OAKThread(threading.Thread, RoboflowOak):
    def __init__(self, threadID, name, counter, onDetect, project, version, api_key, confidence=0.5, overlap=0.5, advanced_config={}, rgb=True, depth=False, device=None, device_name="device", blocking=True, legacy=False):
        RoboflowOak.__init__(self, project, version, api_key, confidence, overlap, advanced_config, rgb, depth, device, device_name, blocking, legacy)
        threading.Thread.__init__(self)
        self.threadID = threadID
        self.name = name
        self.counter = counter
        self.onDetect = onDetect

    def run(self):
        while True:
            detections, frame, raw_frame, depth = self.detections()
            self.onDetect(self.threadID, detections, frame, raw_frame, depth)



def devices():
    return list_devices()

class Prediction:
    def __init__(self, x, y, width, height, confidence, class_name, depth=None):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.depth = depth
        self.confidence = confidence
        self.class_name = class_name

    def json(self):
        return {
            "x": self.x,
            "y": self.y,
            "width": self.width,
            "height": self.height,
            "depth": self.depth,
            "confidence": self.confidence,
            "class": self.class_name
        }