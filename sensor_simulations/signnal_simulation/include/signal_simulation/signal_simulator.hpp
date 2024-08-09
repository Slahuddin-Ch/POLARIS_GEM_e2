#ifndef SIGNAL_SIMULATOR_HPP
#define SIGNAL_SIMULATOR_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <atomic>

class SignalSimulator {
public:
    SignalSimulator();

    void startSimulation();
    void stopSimulation();
    void setSignalStrength(float strength);
    float getSignalStrength() const;

private:
    void simulateSignalStrength();
    void publishSignalStrength();

    ros::NodeHandle nh_;
    ros::Publisher signal_strength_pub_;
    std::atomic<float> current_signal_strength_;
    std::atomic<bool> is_running_;
    std::thread simulation_thread_;
};

#endif // SIGNAL_SIMULATOR_HPP
