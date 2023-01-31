
import rclpy
# import the ROS2 python libraries
from rclpy.node import Node as RosNode
# import the Twist module from geometry_msgs interface
from geometry_msgs.msg import Twist as RosTwist
# import the LaserScan module from sensor_msgs interface
from nav_msgs.msg import Odometry as RosOdometry
# import Quality of Service library, to set the correct profile and reliability in order to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

# Astable Invention project
from . import Tools
from . import Parameters

# python builtins
import math

from std_msgs.msg import Int32

def GetBits(value, id_min, bit_len):
	mask = 0
	for bit_num in range(id_min, id_min+bit_len):
		mask = mask | (1 << bit_num)
	value = value & mask
	value = value >> id_min
	return value

# Note about code
# If name of class/variable starts with _, it should be used ONLY within this class. Consider it private
# I might have forgot it requires double underscore to make "private" attribute

class IRReading:
	def __init__(self):
		self.front_center = 0
		self.front_left = 0
		self.front_right = 0
		self.left = 0
		self.right = 0
		self.side_right = 0 
	def ParseTopicMsg(self, msg):
		num = msg.data
		self.front_center = GetBits(num, 0, 4)
		self.front_left = GetBits(num, 4, 4)
		self.front_right = GetBits(num, 8, 4)
		self.left = GetBits(num, 12, 4)
		self.right = GetBits(num, 16, 4)
		self.side_right = GetBits(num, 20, 4)
	def _SimplifyIRReading(value):
		min_value = 0
		max_value = 15 
		return  100.0 * (value - min_value) / (max_value - min_value) # Percentage, might be floating point
	def Reset(self):
		pass # Nothing
	def __str__(self):
		return f"Infrared readings:\n	front_center = {self.front_center}\n	front_right = {self.front_right}\n	front_left = {self.front_left}\n	left = {self.left}\n	right = {self.right}\n	side_right = {self.side_right}"				
		
class Bumper:
	def __init__(self):
		self.State = None
	def ParseTopicMsg(self, msg):
		num = msg.data
		left_bumper = GetBits(num, 24, 1)
		right_bumper = GetBits(num, 25, 1)
		if left_bumper or right_bumper: 
			self.State = True
		else:
			self.State = None
	def Reset(self):
		pass #self.State = None
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
		linear = float(Tools.Constrain(linear, -Parameters.VelocityLinearLimit, Parameters.VelocityLinearLimit))
		angular = float(Tools.Constrain(angular, -Parameters.VelocityAngularLimit, Parameters.VelocityAngularLimit))
		self.Linear = linear
		self.Angular = angular
		command = RosTwist()
		command.linear.x = linear; command.linear.y = 0.0; command.linear.z = 0.0
		command.angular.x = 0.0; command.angular.y = 0.0; command.angular.z = angular
		self._vel_publisher.publish(command)
	def MoveForward(self, speed = Parameters.VelocityLinearDefault): self.Set(speed,0)
	def MoveBackward(self, speed = Parameters.VelocityLinearDefault): self.Set(-speed,0)
	def RotateLeft(self, speed = Parameters.VelocityAngularDefault): self.Set(0,speed)
	def RotateRight(self, speed = Parameters.VelocityAngularDefault): self.Set(0,-speed)
	def Stop(self): self.Set(0,0)
	def Reset(self):
		self.Set(self.Linear, self.Angular) 
	def __str__(self):
		return f"Velocity:\n	Linear: {self.Linear}\n	Angular: {self.Angular}"
	def DecideAngularSpeed(self, angle):
		return Parameters.DecideVelocityAngularParamA*angle + (Parameters.DecideVelocityAngularParamB if angle >= 0 else -Parameters.DecideVelocityAngularParamB)
	def DecideLinearSpeed(self, distance):
		return Parameters.DecideVelocityLinearParamA*distance + Parameters.DecideVelocityLinearParamA

class RoombaModel(RosNode):
	# Class used to separate AstableInvention (which is main part of the project, the thinking brain of the robot) from hardware (which will be managed by RoombaModel and classes like Velocity, Odometry)
	###################################################################################################
	
	# I think mostly this code will be affected when migrating to physical device
	
	def __init__(self, name, loop_interval):
		super().__init__(name)
		# ROS, subscribers and publishers
		self._odom_subscriber = self.create_subscription( RosOdometry, 'odom', self._OdomCallback, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) )
		self._bumper_subscriber = self.create_subscription( Int32, 'bumper', self._HazardDetection, QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT) )
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
		
	def _OdomCallback(self, msg):
		self.Odometry.ParseTopicMsg(msg)
		
	def _HazardDetection(self, msg):
		# Bumper
		int32data = msg.data
		self.Bumper.ParseTopicMsg(msg)
		self.TriggerHazard(self.Bumper.State, "bump")
		# more will be there probably
		
	###################################################################################################
		
	def PreLoopAction(self):
		self.Odometry.Update()
		self.Position.Update(self.Odometry.dX, self.Odometry.dY, self.Odometry.dAngle, True)
	
	def PostLoopAction(self):
		self.Odometry.Reset()
		self.Velocity.Reset()
		self.Bumper.Reset()
		#self.IR.Reset()
		self.Position.Reset()
	
	###################################################################################################
		
	def _loop(self):
		# Pre-loop action
		self.PreLoopAction()
		# Tick - which is the same as Loop, except it's (supposed to be) read-only
		self.Tick()
		# Loop itself
		if not self._ReflexLoop(): self.Loop() # Call overriden method
		# Post-loop action:
		self.PostLoopAction() # Must be called LAST in loop
	def __str__(self):
		output =  f"{str(self.Odometry)}\n{str(self.Position)}\n{str(self.Velocity)}\n{str(self.Bumper)}"
		if self._tmp: output = f"{output}\n{self._tmp}" #append debug info, if any
		return output
	
	###################################################################################################
		
	# "Virtual" Methods
	def ObstacleReached(self, type): pass # Function to be overriden by AstableInvention
	def ObstacleAfterReflex(self, type): pass # Function to be overriden by AstableInvention
	def Loop(self): pass # Function to be overriden by AstableInvention
	def Tick(self): pass # Function to be overriden by AstableInvention
	
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
