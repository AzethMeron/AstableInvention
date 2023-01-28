
# Astable Invention project
from . import Tools
from . import Parameters
from .ModelMiddleLayer import IRReading, Bumper, Odometry, Position, Velocity, RoombaModel
from .CvLib import CvAnchor
from .JobEngine import Job, JobEngine
from .MotionPlanner import MotionPlanner

# python builtins
import math
import random

# Note about code
# If name of class/variable starts with _, it should be used ONLY within this class. Consider it private
# I might have forgot it requires double underscore to make "private" attribute

class AstableInvention(RoombaModel):
	# Inherited:
	# self.Velocity:
	#	self.Velocity.Linear 
	#	self.Velocity.Angular
	#	self.Velocity.Set # no need to use it, you can change linear and angular values, and roomba will automatically change speed
	# self.Bumper
	#	self.Bumper.State # State within last "tick"
	# self.Odometry
	#	self.Odometry.X, self.Odometry.Y, self.Odometry.Angle
	#	self.Odometry.dX, self.Odometry.dY, self.Odometry.dAngle - delta, change of value (from last "tick")
	# self.Position
	#	self.Position.X, self.Position.Y, self.Position.Angle
	#	self.Update()
	# self.IR
	# 	self.IR.front_center
	#	self.IR.front_left
	#	self.IR.front_right
	#	self.IR.left
	#	self.IR.right
	#	self.IR.side_right
	# 	IR has inverse values, f.e. 15 if it sees nothing and around 1800 if it's close
	# self.ObstacleReached(type) - function called once, IMMEDIATELY when obstacle is detected (bumper, more might be added)
	# self.ObstacleAfterReflex(type) - function called once, after finishing of reflex behaviour
	# type: string, "bump", ...
	# self.Loop() - loop, called every tick of timer, but only if there's no reflex going on right now
	def __init__(self, loop_interval):
		# call __init__() from RoombaModel
		super().__init__("AstableInvention", loop_interval)
		self.JobEngine = JobEngine(self, Parameters.AbsoluteToleranceXY, Parameters.AbsoluteToleranceAngle)
		self.CvAnchor = CvAnchor()
		self.MotionPlanner = MotionPlanner(self)
	def ObstacleReached(self, type):
		self.MotionPlanner.ObstacleReached(type)
	def ObstacleAfterReflex(self, type):
		self.MotionPlanner.ObstacleAfterReflex(type)
	def Tick(self):
		self.MotionPlanner.Tick()
	def Loop(self):
		self.get_logger().info(f"\n{self}\n\n")
		self.MotionPlanner.Loop()
		if self.JobEngine.IsRunning: print(self.JobEngine.IsRunning)
		if self.JobEngine.Run(): return None # Run the job. If there's a job in execution, break execution of loop. Otherwise, go further
		self.MotionPlanner.Run()

# Testing connection
#frame = self.CvAnchor.Camera.GetFrame()
#self.CvAnchor.Communication.PutFrame(frame)
#response = self.CvAnchor.Communication.PopResult()
