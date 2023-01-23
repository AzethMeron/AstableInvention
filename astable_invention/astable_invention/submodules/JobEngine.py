
from . import Tools
from . import Parameters
import math

class Job: # Composite, just to make JobEngine code more readable
	# Don't use consturctor. Use Job.Rotate Job.Translate Job.Absolute Job.Relative
	def __init__(self, x, y, angle = None, relative = False, distance = None):
		self.x = x
		self.y = y
		self.angle = angle # Set to None to ignore ginal rotation
		# specials - those are converted on-demand into absolute positions
		self.relative = relative
		self.translate = distance # This one overrides every other
	def Rotate(angle):
		return Job(0, 0, angle, relative=True, distance=None)
	def Translate(distance):
		return Job(0, 0, None, relative=False, distance=distance)
	def Absolute(x, y, angle = None):
		return Job(x,y,angle, relative=False, distance=None)
	def Relative(x,y, angle = None):
		return Job(x,y,angle, relative=True, distance=None)
	def __str__(self):
		return f"{self.x} {self.y} {self.angle} {self.relative} {self.translate}"
		

class JobEngine:
	# Note: ToleranceRelative is pretty useless in this case (actually it's undesired) but i've realized this only AFTER implementing it
	def __init__(self, robot, ToleranceAbsoluteXY, AbsoluteToleranceAngle):
		self.IsRunning = None # None or Job obj
		self.Queue = []
		self.Robot = robot
		self.ToleranceAbsoluteXY = ToleranceAbsoluteXY
		self.AbsoluteToleranceAngle = AbsoluteToleranceAngle
	def _ConvertRelativeToAbsolute(self, job):
		x = self.Robot.Position.X + job.x
		y = self.Robot.Position.Y + job.y
		angle = Tools.SimplifyAngle(self.Robot.Position.Angle + job.angle) if job.angle else None
		return Job.Absolute(x,y,angle)
	def _ConvertTranslateToRelative(self, job):
		angle = self.Robot.Position.Angle
		x = job.translate * math.cos(angle)
		y = job.translate * math.sin(angle)
		return Job.Relative(x,y,None)
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
		at = self.ToleranceAbsoluteXY
		rt = self.AbsoluteToleranceAngle
		# Check whether destination is reached
		if Tools.Compare(posx, job.x, 0, at) and Tools.Compare(posy, job.y, 0, at):
			if job.angle is None or Tools.Compare(angle, job.angle, 0, rt):
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
			if Tools.Compare(angle_to_rotate, 0, 0, rt): # Perfect angle isn't important, robot can iteratively fix itself if error accumulate - atleast with reliable Odometry
				# Translation
				self.Robot.Velocity.MoveForward( self.Robot.Velocity.DecideLinearSpeed(distance) )
			# Special case - robot moves too far, so it needs to moonwalk a bit
			elif Tools.Compare(math.pi - abs(angle_to_rotate), 0, 0, Parameters.JobMoonwalkAngle) and Tools.Compare(distance, 0, 0, 10*at): 
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
