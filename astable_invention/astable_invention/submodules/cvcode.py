import numpy as np
import cv2 as cv

import os

def GetPath(filename):
	package_path = "src/astable_invention/astable_invention/submodules/"
	return os.path.join(package_path, filename)
	
class Detection:
    def __init__(self, thresh, draw, objects):

        self.thresh = thresh
        self.draw = draw
        self.objects = objects

        # Load the pre-trained model
        self.classNames = []
        self.classFile = GetPath("coco.names")
        with open(self.classFile, "rt") as f:
            self.classNames = f.read().rstrip("\n").split("\n")

        configPath = GetPath("ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt")
        weightsPath = GetPath("frozen_inference_graph.pb")

        self.net = cv.dnn_DetectionModel(weightsPath, configPath)
        self.net.setInputSize(320, 320)
        self.net.setInputScale(1.0 / 127.5)
        self.net.setInputMean((127.5, 127.5, 127.5))
        self.net.setInputSwapRB(True)

    def getObjects(self, frame):
        classIds, confs, bbox = self.net.detect(frame, confThreshold=self.thresh, nmsThreshold=0.1)
        objects = self.classNames if len(self.objects) == 0 else self.objects

        # Our operations on the frame come here
        objInfo = []
        frame_height, frame_width, _ = frame.shape

        if len(classIds) != 0:
            for classId, confidence, box in zip(classIds.flatten(), confs.flatten(), bbox):
                className = self.classNames[classId - 1]

                if className in objects:
                    #objInfo.append([box, className])
                    # [className, distance, angle]

                    object_width = box[2]
                    #print(object_width)
                    focal_length = 698.47570031
                    object_width_inrl = 0.6

                    distance = (object_width_inrl * focal_length) / object_width

                    posx = box[0]+(box[1]/2)
                    camera_angle = 30
                    length_x = frame_width / 2
                    bias = posx - length_x
                    angle = (bias / length_x) * camera_angle * (-1)

                    objInfo.append([className, distance, angle])

        return frame, objInfo
##############################################################################################
