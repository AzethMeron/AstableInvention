
Installation
---

Code created for this project consists of three ROS packages: astableinvention, which is our code, and two external packages: create_3sim and aw-robomaker-small-house-world. Note that the model of iRobot create 3 was slightly modified by us, to disable bump reflexes.

Commands to create workspace, download repository, build and run the project:
```[bash]
source /opt/ros/galactic/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/AzethMeron/AstableInvention.git
cd ..
rosdep install -i --from-path src --rosdistro galactic -y
colcon build
source install/local_setup.bash
```

To run the simulation: ```ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py```
To run astableinvention node: ```ros2 run astable_invention main```

Note: branch master is for simulation, robot for real robot (supports topics programmed by us in the microcontroller) As of now, middle layer isn’t fully implemented in robot branch, because microcontroller doesn’t provide necessary data yet.

Model and simulation platform
---

Finding working model for our purposes was more difficult than anticipated. Either there's no working model for ROS 1 Noetic platform, or we failed to find one. The one I've found was 10 years old and required heavy modifications in URDF files... after doing which we realized it only offers odometry and differential drive, no bump sensors or IR. Our skills in this topic wasn't sufficent to attach those sensors to this model, so we abandoned this model and migrated to another, which we've found while researching: IRobot Create 3.

IRobot Create 3 is packed with features which were not required (and actually undesired) by us. Nevertheless, it worked and it pushed us towards ROS 2 Galactic platform. We had to disable safety features and reflexes (for example, bump reflexes) because our code is supposed to implement them on our own.

We've also used AWS Robomaker Small House to put our vacuuming robot inside some house-like environment.

![environment image](images/environment.png?raw=true "Robot in a house")


ROS 2 and abstraction layers
---

Programming has been done using ROS 2 framework. Our code has been implemented as a ros Node, with multiple layers of abstraction to separate higher functions, like movement to specified location, from messy real world stuff, like velocity control. 

**Bottom layer** of abstraction are ROS topics like Odometry, Velocity, or other provided by hardware manufacturer. Given I'm working with IRobot Create model for the simulation, the way of receiving and passing data to the physical robot might (and will likely) change upon switching to the one provided by Kacper, so i've decided to introduced one more, middle layer.

**Middle layer** of abstraction are Python classes implemented by me, to separate highest layer of abstraction from the lowest, which will require reimplementation once we migrate to physical robot. It provides API which will remain the same after reimplementation of Bottom layer. In code, classes: Odometry, Velocity, Bumper, IRReading and RoombaModel implement this layer.

**Top layer** of abstraction are Python classes implemented by me, being the actual brain of the project. It consists of JobEngine and AstableInvention classes, more on them later.


Middle layer of abstraction - models used
---

**Bumper** class models the physical bumpers of robot, it translates readings from sensor into following states: [ None, "bump_left", "bump_center", "bump_right" ]

**Odometry** class models position of the robot. It provides attributes: X, Y, Angle (in radians, from -π to π) and dX, dY, dAngle (deltas, change of value - this is actually calculated by remembering previous value of position and there's no need for hardware to supply those values)

**Position** class models position of the robot. It provides attributes: X, Y, Angle. It's different than Odometry, because Odometry comes from sensors directly - it's read-only information from bottom layer. Position on other hand CAN be changed. 

**Velocity** class models differential drive, allows to set two values: linear speed and angular speed. Both of them are signed, floating point numbers.

**IRReading** class models infrared sensors on our robot. It translates readings into percentage form (0 to 100, might be floating point, 0 for no reading, 100 for IT'S RIGHT IN FRONT OF ME SEND HELP). Sensors: [ side_left, left, front_left, front_center_left, front_center_right, front_right, right ]

**RoombaModel** class inherits from Node class (which is provided by ROS 2) and implements ROS subsribers, publishers, timers, as well as some (relatively) low-level features like bump reflexes.

All numeric values in project are either normalised (like in IRReading class) or decided within corresponding class (for example Velocity takes distance in and decides proper to traverse this distance) There are no hardcoded values for such things in top layer of abstraction.

Reflexes, protection from hazards
---

Reflexes are features of the middle layer. For now, there's only bump reflex, but the codebase for other types is there, it's pretty flexible.

Reflex is triggered when any hazard is detected (for example, when robots hits obstacle with a bumper) Reflex instantly stops the robot, then calls top layer function ObstacleReached to let programming know that collision occurred (so it can be taken into consideration in SLAM) but it also overrides top layer control flow - top layer can't control robot during reflex,

Behaviour of the robot during reflex is defined by function ObstacleReflexMoonwalk, which has access to all readings from the robot so it can take proper action to safely get away from the hazard. For now, it only moves backward, but more advanced algorithms can be implemented.

After reflex behaviour safely removes the robot from proximity of hazard (obstacle) top layer function ObstacleAfterReflex is called.

https://user-images.githubusercontent.com/41695668/212758125-14d32c59-cfb0-4e05-82e4-bdd48946b281.mp4

Movement to given point: Job and JobEngine
---

Python classes Job and JobEngine, operating in the top layer, implement the rotate-translate-rotate algorithm for robotic movement. Essentially, the robot is rotated to face the destination point, then it moves forward until it reaches the destination point, finally it’s rotated again to have the desired rotation.

Job class stores information about type and end-goal of movement, which is then executed by JobEngine, which also allows creation of a queue of Jobs. 

Jobs can be created with following syntax:  
  Job.Absolute(x,y,angle) - end position is given in absolute coordinates  
  Job.Relative(x,y,angle) - end position is given in relative coordinates  
  Job.Rotate(angle) - rotate by given angle  
  Job.Translate(distance) - move forward by given distance (in respect to current rotation)  
JobEngine works only with absolute coordinates, so all other syntaxes are eventually converted into absolute ones. Angle usually can be omitted (set to None) which allows robot to finish in any rotation it had once reaching given x and y.

Perfect localization given to Job may never be reached because of many reasons, chief among them is frequency rate of code execution - code executing the movement is called with every tick of ROS timer, which is set to small yet significant value. To solve this problem, we’ve introduced the parameter AbsoluteTolerance - which stands for maximum error that can appear on coordinates x y and angle.

https://user-images.githubusercontent.com/41695668/212758173-e14ef0d2-12c0-41d2-980b-7d5353ef684d.mp4

Autonomous mapping
---

We’ve implemented rudimentary algorithms for mapping, exploration, and pathfinding using only odometry data and (binary) bumper data. It’s not a SLAM, but at least it does work.

We couldn’t use any plug & play package for SLAM, nor exactly follow any article or tutorial for it, because pretty much every one we’ve found required a LIDAR or equivalent sensor, which our robot doesn’t have. Seems like light intensity sensors are not enough.

First challenge to overcome was: how to represent a map in the program. I’ve decided to use a simple grid map with two parameters: SIZE and RESOLUTION.

Grid map is created for 2D coordinate system. Variables X,Y are continuous in nature and require discretization to be useful. The main idea of the grid map is: divide the area of SIZE x SIZE into squares of size RESOLUTION x RESOLUTION. Then, by using discretization, we can find within which of those squares our robot is, based on values of X,Y and given resolution. Robot at the start is placed at the centre (SIZE/2, SIZE/2) to not get out of the boundaries of the grid map.

Value within the grid map may represent what is within a given square: empty area through which the robot may pass, obstacle, or unexplored area. 

In our implementation, robot is using Breadth First Search algorithm to find the nearest unexplored square and schedules movement to it using JobEngine, moving by a single neighbouring square at a time. Every program cycle when the robot isn’t in any danger, the current square is marked as empty. Once a robot hits an obstacle, the corresponding square is marked as an obstacle.
Like we’ve mentioned, this algorithm is very rudimentary. It uses only binary value of bumper reading (either hit an obstacle or not) and trusts odometry completely, which accumulates error over time that isn’t compensated for in any way. Furthermore, JobEngine doesn’t support continuous movement, so after reaching any square in a grid it stops, then moves to the next destination. In short, there’re many ways to improve  and optimise our algorithm, but it does work.

https://user-images.githubusercontent.com/41695668/214738519-8f1eee77-43a7-4978-b476-01a39e9437f9.mp4
