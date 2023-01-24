
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
		(x,y) = self.map.Center()
		angle = self.robot.Position.Angle
		self.robot.Position.Update(x,y,angle)
	def ObstacleReached(self, type):
		(x,y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx,dy) = self.map.Facing(self.robot.Position.Angle)
		pos = (x+dx, y+dy)
		self.map.Set(pos, OBSTACLE)
		self.robot.JobEngine.Clear()
	def Tick(self):
		pos = (self.robot.Position.X, self.robot.Position.Y)
		self.map.Set(pos, EMPTY)
	def ObstacleAfterReflex(self, type):
		(x,y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx,dy) = self.map.Facing(self.robot.Position.Angle)
		pos = (x+dx, y+dy)
		if self.map.Get(pos) == OBSTACLE:
			resolution = Parameters.GridMapResolution
			queue = deque( [(x,y)] )
			visited = set( (x,y) )
			parent = dict(); parent[ (x,y) ] = None
			while queue:
				curr = queue.popleft()
				if self.map.Get(curr) == EMPTY:
					path = []
					while curr:
						path.append(curr)
						curr = parent[curr]
					for (x,y) in path[::-1]:
						self.robot.JobEngine.Schedule( Job.Absolute(x,y) )
					return None
				neighbours = [[resolution,0], [0,resolution], [-resolution,0], [0,-resolution]] 
				for (dpos, weight, manhattan) in weighted:
					(dx,dy) = dpos
					if self.map.Get((x+dx,y+dy)) == OBSTACLE: continue
					visited.add((x+dx,y+dy))
					parent[(x+dx,y+dy)] = curr
					queue.append( (x+dx , y+dy) )
	def ComputeWeight(self, dpos): 
		(x, y) = (self.robot.Position.X, self.robot.Position.Y)
		(dx, dy) = dpos
		facing = self.map.Facing(self.robot.Position.Angle)
		weight = 0
		if facing == dpos: weight = weight + 1
		if self.map.Get((x+dx, y+dy)) == UNEXPLORED: weight = weight + 3
		if self.map.Get((x+dx, y+dy)) == OBSTACLE or self.map.Get((x+dx,y+dy)) == EMPTY: weight = 0
		return weight
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
			weighted = [ i for i in weighted if i[1] > 0 ]
			if weighted:
				weighted = sorted(weighted, key=lambda x: -x[2])
				weighted[0][1] = weighted[0][1] + 1
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
		
		
			
		
