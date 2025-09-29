#!/usr/bin/env bash
set -e

SESSION="co_lrio"

source /opt/ros/jazzy/setup.bash
source ~/hector/install/setup.bash   # adjust if needed

tmux kill-session -t $SESSION 2>/dev/null || true

# Temporary URDF files
TMP_DIR="/tmp/co_lrio_urdfs"
mkdir -p $TMP_DIR

for robot in robot_0 robot_1 robot_2; do
    xacro $(ros2 pkg prefix co_lrio)/share/co_lrio/config/robot.urdf.xacro -o $TMP_DIR/${robot}.urdf
done

# Helper: creates panes for one robot in current window (2 columns layout)
create_robot_window() {
    local robot_ns=$1

    # Pane 1: static TF (left column, top)
    tmux send-keys "
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 world ${robot_ns}/odom
" C-m
    tmux rename-window "$robot_ns"

    # Pane 2: co_lrio_lidar_odometry (right column - split vertically to create right column)
    tmux split-window -h
    tmux send-keys "
ros2 run co_lrio co_lrio_lidar_odometry \
  --ros-args -r __ns:=/${robot_ns} \
  --params-file \$(ros2 pkg prefix co_lrio)/share/co_lrio/config/co_lrio_params.yaml
" C-m

    # Go back to left pane and split it vertically to add robot_state_publisher below static TF
    tmux select-pane -t 0
    tmux split-window -v
    tmux send-keys "
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -r __ns:=/${robot_ns} \
  -p frame_prefix:=${robot_ns}/ \
  -p robot_description:=\"\$(cat $TMP_DIR/${robot_ns}.urdf)\"
" C-m

    # Resize to make left column about 1/3 width (resize the right pane to be larger)
    tmux select-pane -t 2
    tmux resize-pane -L 30
}

# --- Robot windows ---
tmux new-session -d -s $SESSION -n robot_0
create_robot_window "robot_0"

tmux new-window -t $SESSION:1 -n robot_1
create_robot_window "robot_1"

tmux new-window -t $SESSION:2 -n robot_2
create_robot_window "robot_2"

# --- Window 4: Mapping ---
tmux new-window -t $SESSION:3 -n mapping
tmux send-keys "
ros2 run co_lrio co_lrio_concentrated_mapping \
  --ros-args --params-file \$(ros2 pkg prefix co_lrio)/share/co_lrio/config/co_lrio_params.yaml
" C-m

# --- Window 5: Visualization ---
tmux new-window -t $SESSION:4 -n viz
tmux send-keys "
rviz2 -d \$(ros2 pkg prefix co_lrio)/share/co_lrio/config/rviz2_three_robot.rviz
" C-m
tmux split-window -h

CMD_BAG_PLAY="ros2 bag play /home/jonathan/bags/s3e/S3E_Campus_Road_2"
CMD_BAG_PLAY="${CMD_BAG_PLAY} --remap"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/velodyne_points:=robot_0/velodyne_points"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/imu/data:=robot_0/imu/data"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/heading:=robot_0/heading"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/correct_imu:=robot_0/correct_imu"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/fix:=robot_0/fix"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/time_reference:=robot_0/time_reference"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/vel:=robot_0/vel"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Alpha/nlink_linktrack_nodeframe2:=robot_0/nlink_linktrack_nodeframe2"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/velodyne_points:=robot_1/velodyne_points"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/imu/data:=robot_1/imu/data"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/heading:=robot_1/heading"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/correct_imu:=robot_1/correct_imu"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/fix:=robot_1/fix"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/time_reference:=robot_1/time_reference"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/vel:=robot_1/vel"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Bob/nlink_linktrack_nodeframe2:=robot_1/nlink_linktrack_nodeframe2"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/velodyne_points:=robot_2/velodyne_points"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/imu/data:=robot_2/imu/data"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/heading:=robot_2/heading"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/correct_imu:=robot_2/correct_imu"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/fix:=robot_2/fix"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/time_reference:=robot_2/time_reference"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/vel:=robot_2/vel"
CMD_BAG_PLAY="${CMD_BAG_PLAY} /Carol/nlink_linktrack_nodeframe2:=robot_2/nlink_linktrack_nodeframe2"

tmux send-keys "
$CMD_BAG_PLAY
" C-m
tmux select-layout even-horizontal

# --- Attach ---
tmux attach -t $SESSION
