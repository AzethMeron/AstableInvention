
# Astable Invention project
from . import Tools
from . import Parameters

import numpy
import math

import traceback

UNEXPLORED = 0
EMPTY = 1
OBSTACLE = 2

class GridMap:
	def __init__(self, size, resolution):
		self.grid = numpy.zeros( (round(size / resolution) , round(size / resolution)) )
		self.representation = numpy.full( (round(size / resolution) , round(size / resolution)), ' ', dtype='U1' )
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
		angle = Tools.SimplifyAngle(angle)
		if Tools.Compare(angle, math.pi / 2, 0, Tools.DegToRad(46)): return (0,resolution)
		if Tools.Compare(abs(angle), math.pi, 0, Tools.DegToRad(46)): return (-resolution, 0)
		if Tools.Compare(angle, -math.pi / 2, 0, Tools.DegToRad(46)): return (0,-resolution)
		if Tools.Compare(angle, 0, 0, Tools.DegToRad(46)): return (resolution,0)
	def FacingExtended(self, angle):
		(size, resolution) = self.params
		angle = Tools.SimplifyAngle(angle)
		if angle >= Tools.DegToRad(-30) and angle <= Tools.DegToRad(30): return (resolution,0)
		if angle >= Tools.DegToRad(30) and angle <= Tools.DegToRad(60): return (resolution,resolution)
		if angle >= Tools.DegToRad(60) and angle <= Tools.DegToRad(120): return (0,resolution)
		if angle >= Tools.DegToRad(120) and angle <= Tools.DegToRad(150): return (-resolution,resolution)
		if angle >= Tools.DegToRad(150) and angle <= Tools.DegToRad(180): return (-resolution,0)
		if angle >= Tools.DegToRad(-180) and angle <= Tools.DegToRad(-150): return (-resolution,0)
		if angle >= Tools.DegToRad(-150) and angle <= Tools.DegToRad(-120): return (-resolution,-resolution)
		if angle >= Tools.DegToRad(-120) and angle <= Tools.DegToRad(-60): return (0,-resolution)
		if angle >= Tools.DegToRad(-60) and angle <= Tools.DegToRad(-30): return (resolution,-resolution)
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
		(x,y)= self.Discretize(pos)
		char_map = { UNEXPLORED: ' ', EMPTY: 'o', OBSTACLE : 'X' }
		inv_map = {v: k for k, v in char_map.items()}
		try:
			self.grid[x,y] = value
			if value > inv_map[self.representation[x,y]]:
				self.representation[x,y] = char_map[value]
		except Exception as e:
			pass
			#print(traceback.format_exc())
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
	def __str__(self):
		lines = []
		for line in self.representation:
			lines.append(' '.join(line))
		lines = lines[::-1]
		return "\n".join(lines)
	def ManhattanDistance(pos1, pos2):
		(x1, y1) = pos1
		(x2, y2) = pos2
		return abs(x1-x2) + abs(y1-y2)
