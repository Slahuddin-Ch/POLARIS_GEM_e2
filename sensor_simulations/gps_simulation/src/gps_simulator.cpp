#include "gps_simulation/gps_simulator.hpp"

GPSSimulator::GPSSimulator()
    : current_gps_accuracy_(100.0), critical_gps_accuracy_(200.0), is_running_(false) {
    gps_accuracy_pub_ = nh_.advertise<std_msgs::Float32>("gps_accuracy", 10);
    ROS_INFO("GPSSimulator initialized with critical GPS accuracy threshold: %.2f mm", critical_gps_accuracy_);
}

void GPSSimulator::startSimulation() {
    is_running_ = true;
    simulation_thread_ = std::thread(&GPSSimulator::simulateGPSAccuracy, this);
}

void GPSSimulator::stopSimulation() {
    is_running_ = false;
    if (simulation_thread_.joinable()) {
        simulation_thread_.join();
    }
}

void GPSSimulator::setGPSAccuracy(float accuracy) {
    current_gps_accuracy_ = accuracy;
    ROS_INFO("GPS accuracy set to: %.2f mm", current_gps_accuracy_.load());
}

float GPSSimulator::getGPSAccuracy() const {
    return current_gps_accuracy_.load();
}

void GPSSimulator::simulateGPSAccuracy() {
    ros::Rate rate(1); // 1 Hz
    while (is_running_) {
        publishGPSAccuracy();
        rate.sleep();
    }
}

void GPSSimulator::publishGPSAccuracy() {
    std_msgs::Float32 msg;
    msg.data = current_gps_accuracy_.load();
    gps_accuracy_pub_.publish(msg);
    ROS_INFO("Published GPS accuracy: %.2f mm", msg.data);

    if (msg.data > critical_gps_accuracy_) {
        ROS_WARN("GPS accuracy exceeds critical threshold!");
    }
}
