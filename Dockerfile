# Use the official ROS Noetic base image
FROM ros:noetic-ros-core

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-rviz \
    ros-noetic-robot-state-publisher \
    ros-noetic-joint-state-publisher \
    ros-noetic-ackermann-msgs \
    ros-noetic-geometry2 \
    ros-noetic-hector-gazebo \
    ros-noetic-hector-models \
    ros-noetic-jsk-rviz-plugins \
    ros-noetic-ros-control \
    ros-noetic-ros-controllers \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# Install Flask
RUN pip3 install flask

# Install rosdep
RUN apt-get update && apt-get install -y python-rosdep

# Initialize rosdep
RUN rosdep init && rosdep update

# Set up environment variables
ENV ROS_WS=/root/ros_ws
ENV FLASK_APP=/root/flask_app

# Create workspace and Flask app directories
RUN mkdir -p $ROS_WS/src
RUN mkdir -p $FLASK_APP

# Copy all the ROS packages and Flask app to the container
COPY . $ROS_WS/src

# Install dependencies using rosdep
RUN rosdep install --from-paths $ROS_WS/src --ignore-src -r -y

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd $ROS_WS && catkin_make"

# Copy the launch script
COPY ./start.sh /root/start.sh
RUN chmod +x /root/start.sh

# Set the entry point to the launch script
ENTRYPOINT ["/root/start.sh"]
