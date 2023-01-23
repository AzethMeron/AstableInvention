
from . import Tools
from . import Parameters
from .GridMap import GridMap, UNEXPLORED, EMPTY, OBSTACLE
from .JobEngine import Job, JobEngine

class MotionPlanner:
	def __init__(self, robot):
		self.map = GridMap(Parameters.GridMapSize, Parameters.GridMapResolution)
		self.robot = robot
		# initialise position at the center of the gridmap
		(x,y) = self.map.Center()
		angle = self.robot.Position.Angle
		self.robot.Position.Update(x,y,angle)
	def ObstacleReached(self, type):
		(x,y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx,dy) = self.map.Facing(self.robot.Position.Angle)
		pos = (x+dx, y+dy)
		self.map.Set(pos, OBSTACLE)
	def Tick(self):
		pos = (self.robot.Position.X, self.robot.Position.Y)
		self.map.Set(pos, EMPTY)
	def ObstacleAfterReflex(self, type):
		pass
		
