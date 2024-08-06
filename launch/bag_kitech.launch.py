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

	parameters_file_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'co_lrio_params.yaml')
	r0_xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'kitech_r0.xacro')
	r0_bag_path = '/media/changsik/T7 Shield/rosbag/kitech_r0'
	r0_name = 'robot_0'
	r1_xacro_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'kitech_r1.xacro')
	r1_bag_path = '/media/changsik/T7 Shield/rosbag/kitech_r1'
	r1_name = 'robot_1'

	## rosbag play
	robot_0_bag_node = ExecuteProcess(
		cmd=[
			'ros2', 'bag', 'play',
			r0_bag_path,
			'--clock',
			'--remap',
			'/velodyne_points:=' + r0_name + '/velodyne_points',
			'/fix:=' + r0_name + '/fix',
			'/zed2/zed_node/imu/data:=' + r0_name + '/zed2/zed_node/imu/data',
			'/zed2/zed_node/left/image_rect_color:=' + r0_name + '/zed2/zed_node/left/image_rect_color',
		],
		name='bag_r0',
		output='screen',
	)
	ld.add_action(robot_0_bag_node)

	## rosbag play
	# robot_1_bag_node = ExecuteProcess(
	# 	cmd=[
	# 		'ros2', 'bag', 'play',
	# 		r1_bag_path,
	# 		'--remap',
	# 		'/velodyne_points:=' + r1_name + '/velodyne_points',
	# 		'/fix:=' + r1_name + '/fix',
	# 		'/camera/imu:=' + r1_name + '/camera/imu',
	# 		'/camera/color/image_raw:=' + r1_name + '/camera/color/image_raw',
	# 	],
	# 	name='bag_r1',
	# 	output='screen',
	# )
	# ld.add_action(robot_1_bag_node)

	# centrailized rviz
	# rviz_config_path = os.path.join(get_package_share_directory('co_lrio'), 'config', 'rviz2_kitech.rviz')
	# rviz_node = Node(
	# 	package = 'rviz2',
	# 	namespace = '',
	# 	executable = 'rviz2',
	# 	name = 'co_lrio_rviz',
	# 	respawn=True, 
	# 	arguments = ['-d' + rviz_config_path]
	# )
	# ld.add_action(rviz_node)

	return ld