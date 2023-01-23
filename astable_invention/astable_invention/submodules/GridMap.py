
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
		self.grid = numpy.zeros( int(size / resolution) , int(size / resolution) )
		self.params = (size, resolution)
	def Discretize(self, pos): # pos = (x,y)
		(size, resolution) = self.params
		(x, y) = pos
		return ( round(x / resolution), round(y, resolution) )
	def Facing(self, angle):
		# return relative (x,y) which shows which direction is being faced
		(size, resolution) = self.params
		if Tools.Compare(angle, math.pi / 2, 0, Tools.DegToRad(46)): return (0,resolution)
		if Tools.Compare(angle, math.pi, 0, Tools.DegToRad(46)): return (-resolution, 0)
		if Tools.Compare(angle, -math.pi / 2, 0, Tools.DegToRad(46)): return (0,-resolution)
		if Tools.Compare(angle, 0, 0, Tools.DegToRad(46)): return (resolution,0)
	def Get(self, pos):
		pos = self.Discretize(pos)
		(x, y) = pos
		return self.grid[x,y]
	def Set(self, pos, value):
		pos = self.Discretize(pos)
		(x, y) = pos
		self.grid[x,y] = value
	def Center(self):
		x = self.params[0]/2
		y = self.params[0]/2
		return (x,y)
	def __str__(self):
		# Not the best code bui it's only for debugging so... ANYWAY
		(size, resolution) = self.params
		lines = []
		for ix in range(size):
			line = ""
			for iy in range(size):
				pixel = UNEXPLORED
				for dx in range(1/resolution):
					for dy in range(1/resolution):
						pos = (ix/resolution + dx, iy/resolution + dy)
						val = self.Get(pos)
						if pixel < val: pixel = val
				if pixel == UNEXPLORED: line = f"{line} "
				if pixel == EMPTY: line = f"{line}o"
				if pixel == OBSTACLE: line = f"{line}X"
			lines.append(line)
		lines = lines[::-1]
		return "\n".join(lines)
