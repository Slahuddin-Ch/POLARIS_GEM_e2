#include "temperature_simulation/temperature_simulator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "temperature_simulator_node");
    TemperatureSimulator temperature_simulator;
    temperature_simulator.startSimulation();
    ros::spin();
    return 0;
}
