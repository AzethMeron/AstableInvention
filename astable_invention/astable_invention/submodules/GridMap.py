
# Astable Invention project
from . import Tools
from . import Parameters

import numpy

class GridMap:
	def __init__(self, size, resolution):
		self.grid = numpy.zeros( int(size / resolution) , int(size / resolution) )
		self.params = (size, resolution)
	def Discretize(pos): # pos = (x,y)
		(size, resolution) = self.params
		(x, y) = pos
		return ( round(x / resolution), round(y, resolution) )
	def Facing(angle):
		pass
		# return relative (x,y) which shows which direction is being faced
		#if Tools.BelongsToRange()
		
