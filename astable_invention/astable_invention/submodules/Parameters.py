
from . import Tools

# Top layer
MainTimerInterval = 0.2 # In seconds
AbsoluteTolerance = 0.01 # absolute tolerance for X,Y and rotational position of robot - too small will make it never stop trying to reach any point
JobMoonwalkAngle = Tools.DegToRad(15) # In radians

# Middle layer - there're many hardcoded values still, because this layer is actually supposed to be replaced
VelocityLinearLimit = 2
VelocityAngularLimit = 1
VelocityLinearDefault = 1
VelocityAngularDefault = 1
# linear velocity =  a * distance + b
DecideVelocityLinearParamA = 5
DecideVelocityLinearParamB = 0
# angular velocity = a * angle +- b (sign depends on the angle, if it's negative then it's -b)
DecideVelocityAngularParamA = 1
DecideVelocityAngularParamB = 0.1

# Others
Infinity = 100000 
