import rclpy
# import the ROS2 python libraries
from rclpy.node import Node as RosNode
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist as RosTwist
# import the LaserScan module from sensor_msgs interface
from nav_msgs.msg import Odometry as RosOdometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

# python builtins
import math

# irobot create imports
from irobot_create_msgs.msg import HazardDetectionVector
from irobot_create_msgs.msg import IrIntensityVector
from irobot_create_msgs.msg import IrOpcode

# temporary
import random

# Note about code
# If name of class/variable starts with _, it should be used ONLY within this class. Consider it private
# I might have forgot it requires double underscore to make "private" attribute

class Tools:
	def Constrain(input, low, high):
		if input < low:
			input = low
		elif input > high:
			input = high
		else:
			input = input
		return input
	def DegToRad(degrees):
		return math.radians(degrees)
	def RadToDeg(radians):
		return math.degrees(radians)
	def Compare(a, b, rel_tol, abs_tol):
		return abs(a-b) <= max( rel_tol*max(abs(a), abs(b)), abs_tol)
	def EulerFromQuaterion(x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		return roll_x, pitch_y, yaw_z # in radians
	def PickShorterAngle(angle_to_rotate):
		return angle_to_rotate if abs(angle_to_rotate) <= abs(2*math.pi - angle_to_rotate) else (2*math.pi - angle_to_rotate)

class IRReading:
	def __init__(self):
		self.front_center_left = None
		self.front_center_right = None
		self.front_left = None
		self.front_right = None
		self.left = None
		self.right = None
		self.side_left = None
	def ParseTopicMsg(self, msg):
		for reading in msg.readings:
			if reading.header.frame_id == "ir_intensity_front_center_left": self.front_center_left = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_front_center_right": self.front_center_right = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_front_left": self.front_left = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_front_right": self.front_right = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_left": self.left = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_right": self.right = IRReading._SimplifyIRReading(reading.value)
			elif reading.header.frame_id == "ir_intensity_side_left": self.side_left = IRReading._SimplifyIRReading(reading.value)
	def _SimplifyIRReading(value):
		min_value = 15
		max_value = 1800 # Those values need to be determined! This is placeholder!
		return  100.0 * (value - min_value) / (max_value - min_value) # Percentage, might be floating point
	def Reset(self):
		pass # Nothing
	def __str__(self):
		return f"Infrared readings:\n	front_center_left = {self.front_center_left}\n	front_center_right = {self.front_center_right}\n	front_left = {self.front_left}\n	front_right = {self.front_right}\n	left = {self.left}\n	right = {self.right}\n	side_left = {self.side_left}"			
		
class Bumper:
	def __init__(self):
		self.State = None
	def ParseTopicMsg(self, msg):
		for hazard in msg.detections:
			if hazard.type == 1:
				self.State = Bumper._SimplifyBumperState(hazard.header.frame_id)
	def _SimplifyBumperState(state):
		if state == "bump_front_center" or state == "bump_front_left" or "bump_front_right":
			return "bump_center"
		return state # bump_left, bump_right
	def Reset(self):
		self.State = None
	def __str__(self):
		return f"Bumper: {self.State}"

class Odometry:
	def __init__(self):
		self.X = 0.0; self.Y = 0.0#; self.PosZ = 0.0
		self.Angle = 0.0
		self.dX = 0.0; self.dY = 0.0
		self.dAngle = 0.0
		self._prevX = 0.0; self._prevY = 0.0
		self._prevAngle = 0.0
	def ParseTopicMsg(self, msg):
		self.X = msg.pose.pose.position.x
		self.Y = msg.pose.pose.position.y
		x = msg.pose.pose.orientation.x
		y = msg.pose.pose.orientation.y
		z = msg.pose.pose.orientation.z
		w = msg.pose.pose.orientation.w
		self.Angle = Tools.EulerFromQuaterion(x,y,z,w)[2]
	def Update(self):
		self.dX = self.X - self._prevX
		self.dY = self.Y - self._prevY
		self.dAngle = self.Angle - self._prevAngle
		self._prevX = self.X
		self._prevY = self.Y
		self._prevAngle = self.Angle
	def Reset(self):
		pass
	def __str__(self):
		pos = f"x = {self.X},	y = {self.Y},	Angle = {self.Angle}"
		dpos = f"dx = {self.dX},	dy = {self.dY},	Angle = {self.dAngle}"
		return f"Odometry:\n	{pos}\n	{dpos}"

class Position:
	def __init__(self):
		self.X = 0
		self.Y = 0
		self.Angle = 0
	def Update(self, x, y, angle, relative=False):
		self.X = (self.X if relative else 0) + x
		self.Y = (self.Y if relative else 0) + y
		self.Angle = (self.Angle if relative else 0) + angle
	def Reset(self):
		pass
	def __str__(self):
		return f"Position:\n	x = {self.X},	y = {self.Y},	Angle = {self.Angle}"

class Velocity:
	def __init__(self, publisher):
		self.Linear = 0.0
		self.Angular = 0.0
		# Make sure to set some sane limits
		self._vel_publisher = publisher
	def Set(self, linear, angular):
		max_linear = 2
		max_angular = 1
		linear = float(Tools.Constrain(linear, -max_linear, max_linear))
		angular = float(Tools.Constrain(angular, -max_angular, max_angular))
		self.Linear = linear
		self.Angular = angular
		command = RosTwist()
		command.linear.x = linear; command.linear.y = 0.0; command.linear.z = 0.0
		command.angular.x = 0.0; command.angular.y = 0.0; command.angular.z = angular
		self._vel_publisher.publish(command)
	def MoveForward(self, speed = 1): self.Set(speed,0)
	def MoveBackward(self, speed = 1): self.Set(-speed,0)
	def RotateLeft(self, speed = 1): self.Set(0,speed)
	def RotateRight(self, speed = 1): self.Set(0,-speed)
	def Stop(self): self.Set(0,0)
	def Reset(self):
		self.Set(self.Linear, self.Angular) 
	def __str__(self):
		return f"Velocity:\n	Linear: {self.Linear}\n	Angular: {self.Angular}"
	
	def DecideAngularSpeed(self, angle):
		return angle + (0.1 if angle>0 else -0.1)
	def DecideLinearSpeed(self, distance):
		return 5*distance

class RoombaModel(RosNode):
	# Class used to separate AstableInvention (which is main part of the project, the thinking brain of the robot) from hardware (which will be managed by RoombaModel and classes like Velocity, Odometry)
	###################################################################################################
	
	# I think mostly this code will be affected when migrating to physical device
	
	def __init__(self, name, loop_interval):
		super().__init__(name)
		# ROS, subscribers and publishers
		self._ir_intensity_subscriber = self.create_subscription( IrIntensityVector, 'ir_intensity', self._IrIntensityCallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) )
		self._odom_subscriber = self.create_subscription( RosOdometry, 'odom', self._OdomCallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) )
		self._hazard_subscriber = self.create_subscription( HazardDetectionVector, 'hazard_detection', self._HazardDetection, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) )
		self._vel_publisher = self.create_publisher( RosTwist, 'cmd_vel', 10 )
		self._timer = self.create_timer( loop_interval, self._loop )
		# creation & initialisation of attributes
		self.IR = IRReading()
		self.Odometry = Odometry()
		self.Position = Position()
		self.Bumper = Bumper()
		self.Velocity = Velocity(self._vel_publisher)
		# other, internal variables, used by features supplied by model
		self._ReflexTriggered = False
		# debug variable, set this to anything so it will be added by str(self._tmp) in __str__
		self._tmp = None
	
	def _IrIntensityCallback(self, msg):
		self.IR.ParseTopicMsg(msg)
		
	def _OdomCallback(self, msg):
		self.Odometry.ParseTopicMsg(msg)
		
	def _HazardDetection(self, msg):
		# Bumper
		self.Bumper.ParseTopicMsg(msg)
		self.TriggerHazard(self.Bumper.State, "bump")
		# more will be there probably
		
	###################################################################################################
		
	def _loop(self):
		# Pre-loop action: Position calculations
		self.Odometry.Update()
		self.Position.Update(self.Odometry.dX, self.Odometry.dY, self.Odometry.dAngle, True)
		# Loop itself
		if not self._ReflexLoop(): self.Loop() # Call overriden method
		# Post-loop action: Reset states
		self.ResetState() # Must be called LAST in loop
	def ResetState(self):
		self.Odometry.Reset()
		self.Velocity.Reset()
		self.Bumper.Reset()
		self.IR.Reset()
		self.Position.Reset()
	def __str__(self):
		output =  f"{str(self.Odometry)}\n{str(self.Position)}\n{str(self.Velocity)}\n{str(self.Bumper)}\n{str(self.IR)}"
		if self._tmp: output = f"{output}\n{self._tmp}" #append debug info, if any
		return output
	
	###################################################################################################
		
	# "Virtual" Methods
	def ObstacleReached(self, type): pass # Function to be overriden by AstableInvention
	def ObstacleAfterReflex(self, type): pass # Function to be overriden by AstableInvention
	def Loop(self): pass # Function to be overriden by AstableInvention
	
	###################################################################################################
	
	# Reflexes
	def HazardStatus(self):
		if self.Bumper.State: return "bump"
		return None
	def TriggerHazard(self, condition, type):
		if condition:
			self.Velocity.Stop() # Emergency stop
			if not self._ReflexTriggered: self.ObstacleReached(type)
			self._ReflexTriggered = True
	def _ReflexLoop(self): # False if no reflex going on, otherwise True
		type = self.HazardStatus()
		if not type and self._ReflexTriggered: # end of reflex 
			self._ReflexTriggered = False # reset flag
			self.ObstacleAfterReflex(type) # Call post-action
		if self._ReflexTriggered: 
			self.ObstacleReflexMoonwalk()
			return True
		return False
	def ObstacleReflexMoonwalk(self): self.Velocity.MoveBackward() # Get away from obstacle
	
	###################################################################################################

class Job: # Composite, just to make JobEngine code more readable
	def __init__(self, x, y, angle = None):
		self.x = x
		self.y = y
		self.angle = angle

class JobEngine:
	def __init__(self, robot, ToleranceRelative, ToleranceAbsolute):
		self.IsRunning = None # None or Job obj
		self.Queue = []
		self.Robot = robot
		self.ToleranceRelative = ToleranceRelative
		self.ToleranceAbsolute = ToleranceAbsolute
	def Run(self): # Return True if Job is in progress, otherwise return False (nothing to do)
		# Feting next in queue
		if not self.IsRunning:  # if job isn't running
			if not len(self.Queue): return False # if there's no next in queue, return False
			self.IsRunning = self.Queue.pop(0) # otherwise, fetch
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
			elif Tools.Compare(abs(angle_to_rotate), math.pi, 10*rt, 10*at) and Tools.Compare(distance, 0, 10*rt, 10*at): 
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
	def Loop(self):
		self.get_logger().info(f"\n{self}\n\n")
		if self.JobEngine.Run(): return None # Run the job. If there's a job in execution, break execution of loop. Otherwise, go further
		# slam i guess, find next objective (Job)
		self.Velocity.MoveForward()
		
# Commands (important)
# source install/local_setup.bash
# ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py
# colcon build --packages-select astable_invention
# ros2 run astable_invention main
# ros2 param set /motion_control safety_override full 

def main(args=None):
	rclpy.init(args=args)
	obj = AstableInvention(0.2)
	obj.Position.Update(obj.Odometry.X, obj.Odometry.Y, obj.Odometry.Angle)
	obj.JobEngine.Schedule( Job( -1, 1) )
	obj.JobEngine.Schedule( Job( -1, -1, 0) )
	rclpy.spin(obj)
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	obj.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
