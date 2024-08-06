#ifndef GPS_SIMULATOR_HPP
#define GPS_SIMULATOR_HPP

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <thread>
#include <atomic>

class GPSSimulator {
public:
    GPSSimulator();

    void startSimulation();
    void stopSimulation();
    void setGPSAccuracy(float accuracy);
    float getGPSAccuracy() const;

private:
    void simulateGPSAccuracy();
    void publishGPSAccuracy();

    ros::NodeHandle nh_;
    ros::Publisher gps_accuracy_pub_;
    std::atomic<float> current_gps_accuracy_;
    const float critical_gps_accuracy_;
    std::atomic<bool> is_running_;
    std::thread simulation_thread_;
};

#endif // GPS_SIMULATOR_HPP
