
import math

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
