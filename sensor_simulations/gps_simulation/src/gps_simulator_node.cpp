#include "gps_simulation/gps_simulator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "gps_simulator_node");
    GPSSimulator simulator;
    simulator.startSimulation();
    ros::spin();
    simulator.stopSimulation();
    return 0;
}
