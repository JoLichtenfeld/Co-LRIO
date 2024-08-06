from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os.path
import yaml
from launch.substitutions import TextSubstitution, LaunchConfiguration, Command

def generate_launch_description():

	ld = LaunchDescription()

	parameters_file_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'kitech.yaml')
	r0_xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'kitech_r0.xacro')
	r0_bag_path = '/media/changsik/T7 Shield/rosbag/kitech_r0'
	r0_name = 'robot_0'
	r1_xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'kitech_r1.xacro')
	r1_bag_path = '/media/changsik/T7 Shield/rosbag/kitech_r1'
	r1_name = 'robot_1'

	# Set env var to print message to stdout immediately
	# arg = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0')
	# ld.add_action(arg)

	# ******************************************************** #
	# single robot mapping node
	robot_0_static_node = Node(
		package='tf2_ros',
		namespace=r0_name,
		executable='static_transform_publisher',
		arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_0/odom'.split(' '),
		parameters=[parameters_file_path]
	)
	ld.add_action(robot_0_static_node)

	robot_0_xacro_node = Node(
		package='robot_state_publisher',
		executable='robot_state_publisher',
		namespace=r0_name,
		name='robot_state_publisher',
		output='screen',
		parameters=[
					{'robot_description': Command(['xacro', ' ', r0_xacro_path])}, 
					{'frame_prefix' : '%s/'%r0_name}
					]
	)
	ld.add_action(robot_0_xacro_node)

	robot_0_lidar_odometry_node = Node(
		package = 'co_lrio',
		namespace = r0_name,
		executable = 'co_lrio_lidar_odometry',
		name = 'co_lrio_lidar_odometry',
		parameters = [parameters_file_path],
		arguments = ['--ros-args', '--log-level', 'INFO'],
		output='screen'
	)
	ld.add_action(robot_0_lidar_odometry_node)

	# ******************************************************** #

	# single robot mapping node
	# robot_1_static_node = Node(
	# 	package='tf2_ros',
	# 	namespace=r1_name,
	# 	executable='static_transform_publisher',
	# 	arguments='0.0 0.0 0.0 0.0 0.0 0.0 world robot_1/odom'.split(' '),
	# 	parameters=[parameters_file_path]
	# )
	# ld.add_action(robot_1_static_node)

	# robot_1_xacro_node = Node(
	# 	package='robot_state_publisher',
	# 	executable='robot_state_publisher',
	# 	namespace=r1_name,
	# 	name='robot_state_publisher',
	# 	output='screen',
	# 	parameters=[
	# 				{'robot_description': Command(['xacro', ' ', r1_xacro_path])}, 
	# 				{'frame_prefix' : '%s/'%r1_name}
	# 				]
	# )
	# ld.add_action(robot_1_xacro_node)

	# robot_1_lidar_odometry_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = r1_name,
	# 	executable = 'co_lrio_lidar_odometry',
	# 	name = 'co_lrio_lidar_odometry',
	# 	parameters = [parameters_file_path],
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(robot_1_lidar_odometry_node)

	# robot_1_imu_compensation_node = Node(
	# 	package = 'imu_complementary_filter',
	# 	namespace = r1_name,
	# 	executable = 'complementary_filter_node',
	# 	name = 'complementary_filter_gain_node',
	# 	remappings=[
	# 		('/imu/data_raw', '/%s/camera/imu'%r1_name)
	# 	],
	# 	parameters=[
    #                 {'do_bias_estimation': True},
    #                 {'do_adaptive_gain': True},
    #                 {'use_mag': False},
    #                 {'gain_acc': 0.01},
    #                 {'gain_mag': 0.01},
    #     ],
	# 	output='screen'
	# )
	# ld.add_action(robot_1_imu_compensation_node)

	# ******************************************************** #

	# centrailized mapping node
	# centrailized_mapping_node = Node(
	# 	package = 'co_lrio',
	# 	namespace = '',
	# 	executable = 'co_lrio_concentrated_mapping',
	# 	name = 'co_lrio_concentrated_mapping',
	# 	parameters = [parameters_file_path],
	# 	arguments = ['--ros-args', '--log-level', 'INFO'],
	# 	output='screen'
	# )
	# ld.add_action(centrailized_mapping_node)

	# centrailized rviz
	rviz_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_kitech.rviz')
	rviz_node = Node(
		package = 'rviz2',
		namespace = '',
		executable = 'rviz2',
		name = 'co_lrio_rviz',
		respawn=True, 
		arguments = ['-d' + rviz_config_path]
	)
	ld.add_action(rviz_node)

	return ld