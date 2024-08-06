from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os.path
import yaml
from launch.substitutions import TextSubstitution, LaunchConfiguration, Command

def generate_launch_description():

	ld = LaunchDescription()

	parameters_file_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'co_lrio_params.yaml')

	robot_0_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = 'robot_0',
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_0_lidar_odometry_node)

	return ld