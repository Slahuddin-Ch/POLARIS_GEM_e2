#include <ros/ros.h>
#include <gtest/gtest.h>
#include "temperature_simulation/temperature_simulator.hpp"

class TemperatureSimulatorTest : public ::testing::Test {
protected:
    TemperatureSimulator simulator;

    virtual void SetUp() {
        simulator.setTemperature(25.0);
        simulator.startSimulation();
        ros::Duration(1.0).sleep(); // Allow some time for the simulation to start
    }

    virtual void TearDown() {
        simulator.stopSimulation();
    }
};

TEST_F(TemperatureSimulatorTest, TemperatureLevelUpdate) {
    simulator.setTemperature(25.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_FLOAT_EQ(simulator.getTemperature(), 25.0);

    simulator.setTemperature(60.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_FLOAT_EQ(simulator.getTemperature(), 60.0);
}

TEST_F(TemperatureSimulatorTest, SafeTemperatureRange) {
    simulator.setTemperature(60.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_GT(simulator.getTemperature(), 55.0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_temperature_simulator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
