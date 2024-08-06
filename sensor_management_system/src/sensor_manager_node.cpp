#include "sensor_management_system/sensor_manager.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "sensor_manager_node");
    SensorManager manager;
    manager.start();
    ros::spin();
    manager.stop();
    return 0;
}
