
# Astable Invention project
from . import Tools
from . import Parameters

import numpy
import math

UNEXPLORED = 0
EMPTY = 1
OBSTACLE = 2

class GridMap:
	def __init__(self, size, resolution):
		self.grid = numpy.zeros( (round(size / resolution) , round(size / resolution)) )
		self.params = (size, resolution)
	def Discretize(self, pos): # pos = (x,y)
		(size, resolution) = self.params
		(x, y) = pos
		return ( round(x / resolution), round(y / resolution) )
	def DeDiscretize(self, discrete_pos):
		(size, resolution) = self.params
		(x, y) = discrete_pos
		return (x * resolution, y * resolution)
	def Facing(self, angle):
		# return relative (x,y) which shows which direction is being faced
		(size, resolution) = self.params
		if Tools.Compare(angle, math.pi / 2, 0, Tools.DegToRad(46)): return (0,resolution)
		if Tools.Compare(abs(angle), math.pi, 0, Tools.DegToRad(46)): return (-resolution, 0)
		if Tools.Compare(angle, -math.pi / 2, 0, Tools.DegToRad(46)): return (0,-resolution)
		if Tools.Compare(angle, 0, 0, Tools.DegToRad(46)): return (resolution,0)
	def GetNeighbours(self):
		(size, resolution) = self.params
		return [(-resolution,0), (0,resolution), (resolution,0), (0,-resolution)] 
	def Get(self, pos):
		pos = self.Discretize(pos)
		(x, y) = pos
		try:
			return self.grid[x,y]
		except:
			return OBSTACLE
	def Set(self, pos, value):
		pos = self.Discretize(pos)
		(x, y) = pos
		try:
			self.grid[x,y] = value
		except:
			pass
	def Center(self):
		(size, resolution) = self.params
		x = size/2
		y = size/2
		return (x,y)
	def Centrify(self, pos):
		(size, resolution) = self.params
		(x, y) = self.Discretize(pos)
		(x, y) = self.DeDiscretize((x,y))
		(x, y) = (x + resolution / 2, y + resolution / 2)
		return (x,y)
	def ManhattanDistance(pos1, pos2):
		(x1, y1) = pos1
		(x2, y2) = pos2
		return abs(x1-x2) + abs(y1-y2)
