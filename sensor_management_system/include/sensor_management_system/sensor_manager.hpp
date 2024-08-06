#ifndef SENSOR_MANAGER_HPP
#define SENSOR_MANAGER_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <atomic>
#include <thread>

class SensorManager {
public:
    SensorManager();
    void start();
    void stop();

private:
    void batteryCallback(const std_msgs::Float32::ConstPtr& msg);
    void temperatureCallback(const std_msgs::Float32::ConstPtr& msg);
    void gpsAccuracyCallback(const std_msgs::Float32::ConstPtr& msg);
    void signalStrengthCallback(const std_msgs::Int32::ConstPtr& msg);
    void emergencyStopCallback(const std_msgs::Bool::ConstPtr& msg);

    void publishRobotState();

    ros::NodeHandle nh_;
    ros::Subscriber battery_sub_;
    ros::Subscriber temperature_sub_;
    ros::Subscriber gps_accuracy_sub_;
    ros::Subscriber signal_strength_sub_;
    ros::Subscriber emergency_stop_sub_;
    ros::Publisher robot_state_pub_;

    std::atomic<float> battery_level_;
    std::atomic<float> temperature_;
    std::atomic<float> gps_accuracy_;
    std::atomic<int> signal_strength_;
    std::atomic<bool> emergency_stop_;

    std::atomic<bool> is_running_;
    std::thread state_thread_;
};

#endif // SENSOR_MANAGER_HPP
