import cv2 as cv
from .cvcode import Detection
from . import Parameters

def main(communication):
	d = Detection(Parameters.CvConfidenceThreshold,True,['dog','cat'])
	while True:
		frame = communication.PopFrame()
		if frame is not None:
			_, get = d.getObjects(frame)
			communication.PutResult(get)
