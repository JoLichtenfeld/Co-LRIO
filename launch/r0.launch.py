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
	xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'robot.urdf.xacro')
 
	robot_0_static_node = Node(
		package='tf2_ros',
		namespace='robot_0',
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_0/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_0_static_node)

	robot_0_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace='robot_0',
		name='robot_state_publisher',
		output='screen',
		parameters=[{'robot_description': Command(['xacro', ' ', xacro_path])}, {'frame_prefix' : 'robot_0/'}]
	)
	ld.add_action(robot_0_xacro_node)


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
 
    # centrailized mapping node
	centrailized_mapping_node = Node(
		package = 'co_lrio',
		namespace = '',
		executable = 'co_lrio_concentrated_mapping',
		name = 'co_lrio_concentrated_mapping',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(centrailized_mapping_node)

	return ld