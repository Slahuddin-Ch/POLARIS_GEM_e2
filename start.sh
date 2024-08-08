#!/bin/bash

# Source the ROS setup files
source /opt/ros/noetic/setup.bash
source /root/ros_ws/devel/setup.bash

# Start tmux session
tmux new-session -d -s my_session

# Start roscore
tmux send-keys -t my_session.0 "roscore" C-m
sleep 5

# Launch Gazebo and RViz with the specified launch file
tmux new-window -t my_session -n gazebo_rviz
tmux send-keys -t my_session:1 "roslaunch polaris_gem_simulator gem_gazebo_rviz.launch velodyne_points:=true" C-m
sleep 10

# Launch the sensor management system
tmux new-window -t my_session -n sensor_management
tmux send-keys -t my_session:2 "roslaunch sensor_management_system sensor_management_system.launch" C-m
sleep 5

# Run the high level planner script
tmux new-window -t my_session -n high_level_planner
tmux send-keys -t my_session:3 "rosrun high_level_planner high_level_planner.py" C-m
sleep 5

# Start the Flask application
tmux new-window -t my_session -n flask_app
tmux send-keys -t my_session:4 "cd /root/ros_ws/src/control_app && python3 app.py" C-m

# Attach to the tmux session
tmux attach-session -t my_session
