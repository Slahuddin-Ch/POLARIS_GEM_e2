#include "signal_simulation/signal_simulator.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "signal_simulator_node");
    SignalSimulator simulator;
    simulator.startSimulation();
    ros::spin();
    simulator.stopSimulation();
    return 0;
}
