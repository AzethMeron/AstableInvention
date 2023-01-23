import rclpy

# Astable Invention project
from .submodules.ModelTopLayer import AstableInvention, Job
from .submodules import Parameters
from .submodules import Tools
	
# Commands (important)
# source install/local_setup.bash
# ros2 launch irobot_create_gazebo_bringup create3_gazebo_aws_small.launch.py
# colcon build --packages-select astable_invention
# ros2 run astable_invention main
# ros2 param set /motion_control safety_override full 

def main(args=None):
	rclpy.init(args=args)
	obj = AstableInvention(Parameters.MainTimerInterval)
	rclpy.spin(obj)
	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	obj.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
    main()
