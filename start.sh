#!/bin/bash

# Source the ROS setup files
source /opt/ros/noetic/setup.bash
source /home/user/gem_ws/devel/setup.bash

# Start tmux session
tmux new-session -d -s my_session

# Start roscore
tmux send-keys -t my_session.0 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; roscore" C-m
sleep 5

# Launch Gazebo and RViz with the specified launch file
tmux new-window -t my_session -n gazebo_rviz
tmux send-keys -t my_session:1 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; roslaunch gem_gazebo gem_gazebo_rviz.launch velodyne_points:=true" C-m

# # Run the high level planner script
tmux new-window -t my_session -n high_level_planner
tmux send-keys -t my_session:2 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; rosrun high_level_planner high_level_planner.py" C-m


# # Start the Flask application
tmux new-window -t my_session -n flask_app
tmux send-keys -t my_session:3 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; cd /home/user/gem_ws/src/POLARIS_GEM_e2/control_app; python3 app.py" C-m

# Run the Battery simulator
tmux new-window -t my_session -n battery_simulator
tmux send-keys -t my_session:4 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; roslaunch battery_simulation battery_simulation.launch" C-m

# Run the Temperature simulator
# tmux new-window -t my_session -n temperature_simulator
# tmux send-keys -t my_session:5 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; rosrun temperature_simulation temperature_simulator_node" C-m

# Run the GPS simulator
# tmux new-window -t my_session -n gps_simulator
# tmux send-keys -t my_session:6 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; rosrun gps_simulation gps_simulator_node" C-m

# Run the Signal simulator
# tmux new-window -t my_session -n signal_simulator
# tmux send-keys -t my_session:7 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; rosrun signal_simulation signal_simulator_node" C-m

# Launch the gem_sensor_info launch file
tmux new-window -t my_session -n sensor_info
tmux send-keys -t my_session:5 "source /opt/ros/noetic/setup.bash; source /home/user/gem_ws/devel/setup.bash; roslaunch gem_gazebo gem_sensor_info.launch" C-m



# Open sensor management system in a new gnome-terminal using /bin/bash
gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash && source /home/user/gem_ws/devel/setup.bash && roslaunch sensor_management_system sensor_management_system.launch; exec bash"

# Open the scenario_launcher in another new gnome-terminal using /bin/bash
gnome-terminal -- /bin/bash -c "source /opt/ros/noetic/setup.bash && source /home/user/gem_ws/devel/setup.bash && rosrun scenarios scenario_launcher.py; exec bash"


# Attach to the tmux session
tmux attach-session -t my_session  