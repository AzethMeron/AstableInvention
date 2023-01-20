
# Astable Invention project
from . import Tools
from .ModelMiddleLayer import IRReading, Bumper, Odometry, Position, Velocity, RoombaModel
from .CvLib import CvAnchor

# python builtins
import math

# Note about code
# If name of class/variable starts with _, it should be used ONLY within this class. Consider it private
# I might have forgot it requires double underscore to make "private" attribute

class Job: # Composite, just to make JobEngine code more readable
	def __init__(self, x, y, angle = None, relative = False, distance = None):
		self.x = x
		self.y = y
		self.angle = angle # Set to None to ignore ginal rotation
		# specials - those are converted on-demand into absolute positions
		self.relative = relative
		self.translate = distance # This one overrides every other
	def Rotate(angle):
		return Job(0, 0, angle, True)
	def Translate(distance):
		r = Job(0,0)
		r.translate = abs(distance)
		return r
	def Absolute(x, y, angle = None):
		return Job(x,y,angle, relative=False, distance=None)
	def Relative(x,y, angle = None):
		return Job(x,y,angle, relative=True, distance=None)
		

class JobEngine:
	# Note: ToleranceRelative is pretty useless in this case (actually it's undesired) but i've realized this only AFTER implementing it
	def __init__(self, robot, ToleranceRelative, ToleranceAbsolute):
		self.IsRunning = None # None or Job obj
		self.Queue = []
		self.Robot = robot
		self.ToleranceRelative = ToleranceRelative
		self.ToleranceAbsolute = ToleranceAbsolute
	def _ConvertRelativeToAbsolute(self, job):
		x = self.Robot.Position.X + job.x
		y = self.Robot.Position.Y + job.y
		angle = (self.Robot.Position.Angle + job.angle) if job.angle else None
		return Job(x, y, angle=angle, relative=False)
	def _ConvertTranslateToRelative(self, job):
		angle = self.Robot.Position.Angle
		x = job.translate * math.sin(angle)
		y = job.translate * math.cos(angle)
		return Job(x,y, angle=0, relative=True)
	def Run(self): # Return True if Job is in progress, otherwise return False (nothing to do)
		# Feting next in queue
		if not self.IsRunning:  # if job isn't running
			if not len(self.Queue): return False # if there's no next in queue, return False
			self.IsRunning = self.Queue.pop(0) # otherwise, fetch
		# Conversion from special to absolute
		if self.IsRunning.translate: self.IsRunning = self._ConvertTranslateToRelative(self.IsRunning)
		if self.IsRunning.relative: self.IsRunning = self._ConvertRelativeToAbsolute(self.IsRunning)
		# References - original names too long
		posx = self.Robot.Position.X
		posy = self.Robot.Position.Y
		angle = self.Robot.Position.Angle
		job = self.IsRunning
		rt = self.ToleranceRelative
		at = self.ToleranceAbsolute
		# Check whether destination is reached
		if Tools.Compare(posx, job.x, rt, at) and Tools.Compare(posy, job.y, rt, at):
			if job.angle is None or Tools.Compare(angle, job.angle, rt, at):
				self.Abort()
				return True
			# Final rotation
			angle_to_rotate = Tools.PickShorterAngle(job.angle - angle)
			self.Robot.Velocity.Set(0, self.Robot.Velocity.DecideAngularSpeed(angle_to_rotate))
		else: # Destination isn't reached, we need to schedule move
			x = job.x - posx # X axis change
			y = job.y - posy # Y axis change
			angle_to_rotate = Tools.PickShorterAngle(math.atan2(y,x) - angle)
			distance = math.sqrt( (x)**2 + (y)**2 )
			if Tools.Compare(angle_to_rotate, 0, rt, at): # Perfect angle isn't important, robot can iteratively fix itself if error accumulate - atleast with reliable Odometry
				# Translation
				self.Robot.Velocity.MoveForward( self.Robot.Velocity.DecideLinearSpeed(distance) )
			# Special case - robot moves too far, so it needs to moonwalk a bit
			elif Tools.Compare(math.pi - abs(angle_to_rotate), 0, 0, Tools.DegToRad(15)) and Tools.Compare(distance, 0, 10*rt, 10*at): 
				self.Robot.Velocity.MoveBackward( self.Robot.Velocity.DecideLinearSpeed(distance) )
			else:
				# Rotation
				self.Robot.Velocity.Set(0, self.Robot.Velocity.DecideAngularSpeed(angle_to_rotate))
		return True
	def Schedule(self, job):
		self.Queue.append(job)
	def Abort(self):
		self.IsRunning = None
		self.Robot.Velocity.Stop()
		
	###################################################################################################

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
	# 	self.IR.front_center_left
	#	self.IR.front_center_right
	#	self.IR.front_left
	#	self.IR.front_right
	#	self.IR.left
	#	self.IR.right
	#	self.IR.side_left
	# 	IR has inverse values, f.e. 15 if it sees nothing and around 1800 if it's close
	# self.ObstacleReached(type) - function called once, IMMEDIATELY when obstacle is detected (bumper, more might be added)
	# self.ObstacleAfterReflex(type) - function called once, after finishing of reflex behaviour
	# type: string, "bump", ...
	# self.Loop() - loop, called every tick of timer, but only if there's no reflex going on right now
	def __init__(self, loop_interval):
		# call __init__() from RoombaModel
		super().__init__("AstableInvention", loop_interval)
		self.JobEngine = JobEngine(self, 0, 0.01)
		self.CvAnchor = CvAnchor()
	def Loop(self):
		self.get_logger().info(f"\n{self}\n\n")
		# Testing connection
		#frame = self.CvAnchor.Camera.GetFrame()
		#self.CvAnchor.Communication.PutFrame(frame)
		#response = self.CvAnchor.Communication.PopResult()
		if self.JobEngine.Run(): return None # Run the job. If there's a job in execution, break execution of loop. Otherwise, go further
		# slam i guess, find next objective (Job)
		#self.Velocity.MoveForward()
