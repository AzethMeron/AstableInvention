
import multiprocessing
import cv2 as cv

class Camera:
	def __init__(self):
		self.cap = cv.VideoCapture(0)
		if not self.cap.isOpened():
			print("Cannot open camera")
	def GetFrame(self):
		ret, frame = self.cap.read()
		if not ret: return None
		return frame
	def __del__(self):
		self.cap.release()

class Communication:
	def __init__(self):
		self.__input_queue = multiprocessing.Queue()
		self.__output_queue = multiprocessing.Queue()
	def _PutSingle(queue, obj):
		while not queue.empty():
			try:
				queue.get_nowait()
			except:
				pass
		queue.put(obj)
	def PutResult(self, obj):
		Communication._PutSingle(self.__output_queue, obj)
	def PopResult(self):
		try:
			return self.__output_queue.get_nowait()
		except:
			return None
	def PopFrame(self):
		return self.__input_queue.get()
	def PutFrame(self, frame):
		Communication._PutSingle(self.__input_queue, frame)

# Temporary
def main(communication):
	while True:
		obj = communication.PopFrame()
		communication.PutResult(obj)

class CvAnchor:
	def __init__(self):
		self.Communication = Communication()
		self.Camera = Camera()
		self.Process = multiprocessing.Process(target=main, args=(self.Communication,))
		self.Process.start()
	def __del__(self):
		pass
		self.Process.terminate()
