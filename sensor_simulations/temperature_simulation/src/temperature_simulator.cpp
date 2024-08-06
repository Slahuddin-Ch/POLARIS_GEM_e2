#include "temperature_simulation/temperature_simulator.hpp"

TemperatureSimulator::TemperatureSimulator()
    : current_temperature_(25.0), safe_temperature_range_(55.0), is_running_(false) {
    temperature_pub_ = nh_.advertise<std_msgs::Float32>("temperature", 10);
    ROS_INFO("TemperatureSimulator initialized with safe temperature range: %.2f°C", safe_temperature_range_);
}

void TemperatureSimulator::startSimulation() {
    is_running_ = true;
    simulation_thread_ = std::thread(&TemperatureSimulator::simulateTemperature, this);
}

void TemperatureSimulator::stopSimulation() {
    is_running_ = false;
    if (simulation_thread_.joinable()) {
        simulation_thread_.join();
    }
}

void TemperatureSimulator::setTemperature(float temperature) {
    current_temperature_ = temperature;
    ROS_INFO("Temperature set to: %.2f°C", current_temperature_.load());
}

float TemperatureSimulator::getTemperature() const {
    return current_temperature_.load();
}

void TemperatureSimulator::simulateTemperature() {
    ros::Rate rate(1); // 1 Hz
    while (is_running_) {
        publishTemperature();
        rate.sleep();
    }
}

void TemperatureSimulator::publishTemperature() {
    std_msgs::Float32 msg;
    msg.data = current_temperature_.load();
    temperature_pub_.publish(msg);
    ROS_INFO("Published temperature: %.2f°C", msg.data);

    if (msg.data > safe_temperature_range_) {
        ROS_WARN("Temperature exceeds safe range!");
    }
}
