
from . import Tools

# Top layer
MainTimerInterval = 0.2 # In seconds
AbsoluteToleranceXY = 0.05 # absolute tolerance for X,Y position of robot - too small will make it never stop trying to reach any point
AbsoluteToleranceAngle = 0.01 # absolute tolerance rotational position of robot - too small will make it never stop trying to reach any point
JobMoonwalkAngle = Tools.DegToRad(15) # In radians

# Gridmap parameters
GridMapSize = 1
GridMapResolution = 0.15 # should be greater than AbsoluteToleranceXY

# Threshold for IR readings (mapping)
MappingObstacleIRThreshold = 30.0

# Middle layer - there're many hardcoded values still, because this layer is actually supposed to be replaced
VelocityLinearLimit = 1
VelocityAngularLimit = 1
VelocityLinearDefault = 1
VelocityAngularDefault = 1
# linear velocity =  a * distance + b
DecideVelocityLinearParamA = 2
DecideVelocityLinearParamB = 0
# angular velocity = a * angle +- b (sign depends on the angle, if it's negative then it's -b)
DecideVelocityAngularParamA = 1
DecideVelocityAngularParamB = 0.1

# Others
Infinity = 100000 
