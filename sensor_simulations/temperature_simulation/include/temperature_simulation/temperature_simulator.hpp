#ifndef TEMPERATURE_SIMULATOR_HPP
#define TEMPERATURE_SIMULATOR_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <atomic>

class TemperatureSimulator {
public:
    TemperatureSimulator();

    void startSimulation();
    void stopSimulation();
    void setTemperature(float temperature);
    float getTemperature() const;

private:
    void simulateTemperature();
    void publishTemperature();

    ros::NodeHandle nh_;
    ros::Publisher temperature_pub_;
    std::atomic<float> current_temperature_;
    const float safe_temperature_range_;
    std::atomic<bool> is_running_;
    std::thread simulation_thread_;
};

#endif // TEMPERATURE_SIMULATOR_HPP
