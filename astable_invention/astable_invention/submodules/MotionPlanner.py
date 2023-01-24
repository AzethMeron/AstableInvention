
from . import Tools
from . import Parameters
from .GridMap import GridMap, UNEXPLORED, EMPTY, OBSTACLE
from .JobEngine import Job, JobEngine

from collections import deque

class MotionPlanner:
	def __init__(self, robot):
		self.map = GridMap(Parameters.GridMapSize, Parameters.GridMapResolution)
		self.robot = robot
		# initialise position at the center of the gridmap
		resolution = Parameters.GridMapResolution
		(x,y) = self.map.Center()
		(x,y) = (x+resolution/2,y+resolution/2)
		angle = self.robot.Position.Angle
		self.robot.Position.Update(x,y,angle)
		self.robot.JobEngine.Schedule( Job.Absolute(x,y,0) )
	def ObstacleReached(self, type):
		(x,y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx,dy) = self.map.Facing(self.robot.Position.Angle)
		self.map.Set((x,y), OBSTACLE)
		self.map.Set((x+dx, y+dy), OBSTACLE)
		self.robot.JobEngine.Clear()
	def Tick(self):
		pass
	def Loop(self):
		pos = (self.robot.Position.X, self.robot.Position.Y)
		self.map.Set(pos, EMPTY)
	def ObstacleAfterReflex(self, type):
		pass
	def ComputeWeight(self, dpos): 
		(x, y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx, dy) = dpos
		facing = self.map.Facing(self.robot.Position.Angle)
		weight = 0
		if facing == dpos: weight = weight + 1
		if self.map.Get((x+dx, y+dy)) == UNEXPLORED: weight = weight + 3
		return weight
	def StripInvalid(self, weighted):
		output = []
		(x, y) = (self.robot.Position.X, self.robot.Position.Y)
		for (dpos, weight, manhattan) in weighted:
			(dx, dy) = dpos
			pos = (x+dx, y+dy)
			if self.map.Get(pos) == OBSTACLE: continue
			output.append([dpos, weight, manhattan])
		return output
	def PathToNearestUnexplored(self):
		(x,y) = (self.robot.Position.X, self.robot.Position.Y)
		resolution = Parameters.GridMapResolution
		queue = deque( [(x,y)] )
		visited = set( (x,y) )
		parent = dict(); parent[ (x,y) ] = None
		while queue:
			curr = queue.popleft()
			if self.map.Get(curr) == UNEXPLORED:
				path = []
				while curr:
					path.append(curr)
					curr = parent[curr]
				return path[::-1]
			neighbours = [[resolution,0], [0,resolution], [-resolution,0], [0,-resolution]] 
			weighted = [ [dpos, self.ComputeWeight(dpos), GridMap.ManhattanDistance(self.map.Center(), (x+dpos[0],y+dpos[1]))] for dpos in neighbours ]
			weighted = self.StripInvalid(weighted)
			if weighted:
				#weighted = sorted(weighted, key=lambda x: -x[2])
				#weighted[0][1] = weighted[0][1] + 1
				weighted = sorted(weighted, key= lambda x: -x[1])
				for (dpos, weight, manhattan) in weighted:
					(dx,dy) = dpos
					visited.add((x+dx,y+dy))
					parent[(x+dx,y+dy)] = curr
					queue.append( (x+dx , y+dy) )
		return []
	def Run(self):
		to_visit = self.PathToNearestUnexplored()
		resolution = Parameters.GridMapResolution
		for pos in to_visit: 
			(x,y) = pos
			self.robot.JobEngine.Schedule( Job.Absolute(x,y) )
		
		
			
		
