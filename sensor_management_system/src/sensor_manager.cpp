#include "sensor_management_system/sensor_manager.hpp"

SensorManager::SensorManager()
    : battery_level_(100.0), temperature_(25.0), gps_accuracy_(0.0), signal_strength_(1), emergency_stop_(false), is_running_(false) {
    battery_sub_ = nh_.subscribe("battery_level", 10, &SensorManager::batteryCallback, this);
    temperature_sub_ = nh_.subscribe("temperature", 10, &SensorManager::temperatureCallback, this);
    gps_accuracy_sub_ = nh_.subscribe("gps_accuracy", 10, &SensorManager::gpsAccuracyCallback, this);
    signal_strength_sub_ = nh_.subscribe("signal_strength", 10, &SensorManager::signalStrengthCallback, this);
    emergency_stop_sub_ = nh_.subscribe("emergency_stop", 10, &SensorManager::emergencyStopCallback, this);
    robot_state_pub_ = nh_.advertise<std_msgs::String>("robot_state", 10);
    ROS_INFO("SensorManager initialized");
}

void SensorManager::start() {
    is_running_ = true;
    state_thread_ = std::thread(&SensorManager::publishRobotState, this);
    ROS_INFO("SensorManager started");
}

void SensorManager::stop() {
    is_running_ = false;
    if (state_thread_.joinable()) {
        state_thread_.join();
    }
    ROS_INFO("SensorManager stopped");
}

void SensorManager::batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
    battery_level_ = msg->data;
    ROS_INFO("Battery level updated: %.2f", battery_level_.load());
}

void SensorManager::temperatureCallback(const std_msgs::Float32::ConstPtr& msg) {
    temperature_ = msg->data;
    ROS_INFO("Temperature updated: %.2f", temperature_.load());
}

void SensorManager::gpsAccuracyCallback(const std_msgs::Float32::ConstPtr& msg) {
    gps_accuracy_ = msg->data;
    ROS_INFO("GPS accuracy updated: %.2f", gps_accuracy_.load());
}

void SensorManager::signalStrengthCallback(const std_msgs::Int32::ConstPtr& msg) {
    signal_strength_ = msg->data;
    ROS_INFO("Signal strength updated: %d", signal_strength_.load());
}

void SensorManager::emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg) {
    emergency_stop_ = msg->data;
    ROS_INFO("Emergency stop updated: %s", emergency_stop_.load() ? "true" : "false");
}

void SensorManager::publishRobotState() {
    ros::Rate rate(1); // 1 Hz
    std_msgs::String state_msg;
    while (is_running_) {
        if (emergency_stop_) {
            state_msg.data = "ERROR";
            ROS_WARN("Emergency stop detected! Entering ERROR state");
        } else if (battery_level_ < 50.0) {
            state_msg.data = "ERROR";
            ROS_WARN("Battery level critical! Entering ERROR state");
        } else if (temperature_ >= 55.0) {
            state_msg.data = "ERROR";
            ROS_WARN("Temperature exceeds safe range! Entering ERROR state");
        } else if (gps_accuracy_ >= 200.0) {
            static int low_accuracy_count = 0;
            low_accuracy_count++;
            if (low_accuracy_count > 15) {
                state_msg.data = "ERROR";
                ROS_WARN("GPS accuracy critical for too long! Entering ERROR state");
            }
        } else if (signal_strength_ == 0) {
            static int no_signal_count = 0;
            no_signal_count++;
            if (no_signal_count > 10) {
                state_msg.data = "ERROR";
                ROS_WARN("No signal detected for too long! Entering ERROR state");
            }
        } else if (signal_strength_ == 2) {
            static int low_signal_count = 0;
            low_signal_count++;
            if (low_signal_count > 20) {
                state_msg.data = "ERROR";
                ROS_WARN("Low signal detected for too long! Entering ERROR state");
            }
        } else {
            state_msg.data = "RUNNING";
            ROS_INFO("All systems nominal. State: RUNNING");
        }

        robot_state_pub_.publish(state_msg);
        ROS_INFO("Published robot state: %s", state_msg.data.c_str());
        rate.sleep();
    }
}
