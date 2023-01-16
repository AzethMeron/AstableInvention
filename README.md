
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

Reflexes are features of middle layer. For now, there's only bump reflex, but the codebase for other types is there, it's pretty flexible.

Reflex is triggered when any hazard is detected (for example, when robots hits obstacle with a bumper) Reflex instantly stops the robot, then calls top layer function to let programming know that collision occured (so I can take it into consideration in SLAM) but it also overrides top layer control flow - **top layer can't control robot during reflex**,

Behaviour of robot during reflex is defined by function ObstacleReflexMoonwalk, which has access to all readings from robot so can take proper action to safely get away from hazard. For now, it only moves backward, but more advanced algorithm can be implemented.

![reflex](images/reflex.mp4)

Top layer of abstraction
---

Classes in top level of abstraction don't use hardcoded numerical values and use only pure-pythonic code, as well as tools supplied by middle layer. It should work as good in simulator as on physical robot, with only fine-tuning of some parameters.

**JobEngine** and Job classes are used to move robot to specified position, with specified final angle. It uses rotate-translate-rotate algorithm and takes absolute tolerance as parameter (it's pretty much impossible to reach position perfectly, even in simulator) However JobEngine doesn't deal with obstacles in the way, it's task for algorithm above.

**AstableInvention** is in highest level of abstraction, it inherits RoombaModel and utilitizes middle layer + JobEngine for SLAM (not implemented yet)
