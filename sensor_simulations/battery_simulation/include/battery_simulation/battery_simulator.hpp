#ifndef BATTERY_SIMULATOR_HPP
#define BATTERY_SIMULATOR_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>

class BatterySimulator {
public:
    BatterySimulator();
    void startSimulation();
    void setBatteryLevel(float level);
    float getBatteryLevel() const;

private:
    void simulateBattery();
    void publishBatteryStatus();

    ros::NodeHandle nh;
    ros::Publisher battery_pub;
    ros::Publisher battery_status_pub;
    float battery_level;
    float critical_battery_level;
};

#endif
