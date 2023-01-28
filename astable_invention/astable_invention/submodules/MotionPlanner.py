
from . import Tools
from . import Parameters
from .GridMap import GridMap, UNEXPLORED, EMPTY, OBSTACLE
from .JobEngine import Job, JobEngine

# here pos usually refers to (x,y)
# sometimes it's extended with angle, but that makes it incompatible with GridMap

class MotionPlanner:
	def __init__(self, robot):
		self.map = None
		self.robot = robot
		self.Reset()
	def GetPos(self):
		return (self.robot.Position.X, self.robot.Position.Y, self.robot.Position.Angle)
	def Reset(self):
		self.map = GridMap(Parameters.GridMapSize, Parameters.GridMapResolution)
		# initialise position at the center of the gridmap
		resolution = Parameters.GridMapResolution
		(x,y) = self.map.Centrify(self.map.Center())
		angle = self.robot.Position.Angle
		self.robot.Position.Update(x,y,angle)
	def ObstacleReached(self, type):
		if type == "bump":
			(x,y,angle) = self.GetPos()
			(dx,dy) = self.map.Facing(angle)
			self.map.Set((x, y), OBSTACLE) # Set current as obstacle
			self.map.Set((x+dx, y+dy), OBSTACLE) # Set facing as obstacle
			self.robot.JobEngine.Clear()
	def ObstacleAfterReflex(self, type):
		pass # Here we do nothing
	def Tick(self):
		pass # Here we do nothing
	def Loop(self):
		(x,y,angle) = self.GetPos()
		self.map.Set((x,y), EMPTY)
		(dx,dy) = self.map.Facing(angle)
	def ParseNeighbours(self, pos, facing, neighbours):
		output = []
		(px,py) = pos
		for (dx,dy) in neighbours:
			x = px + dx
			y = py + dy
			if self.map.Get((x,y)) == OBSTACLE: continue
			weight = 0
			if self.map.Get((x,y)) == UNEXPLORED: weight = weight + 5
			if facing == (dx,dy): weight = weight + 1
			output.append( ((x,y), weight) )
		output = sorted(output, key=lambda x: -x[1])
		return output
	def PathToNearestUnexplored(self):
		(x,y,angle) = self.GetPos()
		facing = self.map.Facing(angle)
		visited = set([ (x,y) ])
		parent = { (x,y) : None }
		queue = [ (x,y) ]
		while queue:
			(cx, cy) = queue.pop(0) # current x, current y
			if self.map.Get((cx,cy)) == UNEXPLORED:
				return self.GetPath( (cx,cy), parent )
			neighbours = self.map.GetNeighbours()
			parsed = self.ParseNeighbours((cx,cy), facing, neighbours)
			for ((ax, ay), weight) in parsed:
				if (ax, ay) in visited: continue
				visited.add( (ax, ay) )
				parent[(ax,ay)] = (cx, cy)
				queue.append( (ax,ay) )
		return []
	def GetPath(self, pos, parent):
		path = []
		curr = pos
		while curr:
			path.append(curr)
			curr = parent[curr]
		return path[::-1]
	def Run(self):
		print("Running motion planner")
		to_visit = self.PathToNearestUnexplored()
		print("Path found")
		for pos in to_visit:
			print(f"{self.GetPos()} -> {pos}")
			(x,y) = pos
			(x,y) = self.map.Centrify((x,y))
			self.robot.JobEngine.Schedule( Job.Absolute(x,y) )
		
		
			
		
