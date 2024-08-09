#include "signal_simulation/signal_simulator.hpp"


SignalSimulator::SignalSimulator()
    : current_signal_strength_(1), is_running_(false) {
    signal_strength_pub_ = nh_.advertise<std_msgs::Float32>("signal_strength", 10);
    ROS_INFO("SignalSimulator initialized with default signal strength: %d", current_signal_strength_.load());
}

void SignalSimulator::startSimulation() {
    is_running_ = true;
    simulation_thread_ = std::thread(&SignalSimulator::simulateSignalStrength, this);
}

void SignalSimulator::stopSimulation() {
    is_running_ = false;
    if (simulation_thread_.joinable()) {
        simulation_thread_.join();
    }
}

void SignalSimulator::setSignalStrength(float strength) {
    current_signal_strength_ = strength;
    ROS_INFO("Signal strength set to: %f", current_signal_strength_.load());
}

float SignalSimulator::getSignalStrength() const {
    return current_signal_strength_.load();
}

void SignalSimulator::simulateSignalStrength() {
    ros::Rate rate(1); // 1 Hz
    while (is_running_) {
        publishSignalStrength();
        rate.sleep();
    }
}

void SignalSimulator::publishSignalStrength() {
    std_msgs::Float32 msg;
    msg.data = current_signal_strength_.load();
    signal_strength_pub_.publish(msg);
    ROS_INFO("Published signal strength: %f", msg.data);

    if (msg.data == 0) {
        ROS_WARN("No signal detected!");
    } else if (msg.data == 2) {
        ROS_WARN("Low signal detected!");
    }
}
