#include "battery_simulation/battery_simulator.hpp"
#include <thread>
#include <chrono>

BatterySimulator::BatterySimulator() : battery_level(100.0) {
    battery_pub = nh.advertise<std_msgs::Float32>("battery_level", 10);
    battery_status_pub = nh.advertise<std_msgs::Float32>("battery_status", 10);
    nh.param("critical_battery_level", critical_battery_level, 50.0f);  // Default to 50% if not set
    ROS_INFO("BatterySimulator initialized with critical battery level: %.2f%%", critical_battery_level);
}

void BatterySimulator::startSimulation() {
    ROS_INFO("Starting battery simulation...");
    std::thread(&BatterySimulator::simulateBattery, this).detach();
}

void BatterySimulator::simulateBattery() {
    while (ros::ok() && battery_level > 0) {
        std_msgs::Float32 msg;
        msg.data = battery_level;
        battery_pub.publish(msg);
        publishBatteryStatus();
        if (battery_level <= critical_battery_level) {
            ROS_WARN("Battery level critical: %.2f%%", battery_level);
        } else {
            ROS_INFO("Battery level: %.2f%%", battery_level);
        }
        battery_level -= 1.0;
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    ROS_WARN("Battery depleted.");
}

void BatterySimulator::publishBatteryStatus() {
    std_msgs::Float32 status_msg;
    status_msg.data = battery_level;
    battery_status_pub.publish(status_msg);
    ROS_INFO("Published battery status: %.2f%%", battery_level);
}

void BatterySimulator::setBatteryLevel(float level) {
    if (level < 0.0 || level > 100.0) {
        ROS_ERROR("Invalid battery level: %.2f. It must be between 0 and 100.", level);
        return;
    }
    battery_level = level;
    ROS_INFO("Battery level set to: %.2f%%", battery_level);
}

float BatterySimulator::getBatteryLevel() const {
    return battery_level;
}

#ifndef BATTERY_SIMULATOR_TEST
int main(int argc, char** argv) {
    ros::init(argc, argv, "battery_simulator");
    BatterySimulator battery_simulator;
    battery_simulator.startSimulation();
    ros::spin();
    return 0;
}
#endif