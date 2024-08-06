#include <ros/ros.h>
#include <gtest/gtest.h>
#include "gps_simulation/gps_simulator.hpp"

class GPSSimulatorTest : public ::testing::Test {
protected:
    GPSSimulator simulator;

    virtual void SetUp() {
        simulator.setGPSAccuracy(100.0);
        simulator.startSimulation();
        ros::Duration(1.0).sleep(); // Allow some time for the simulation to start
    }

    virtual void TearDown() {
        simulator.stopSimulation();
    }
};

TEST_F(GPSSimulatorTest, GPSAccuracyUpdate) {
    simulator.setGPSAccuracy(100.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_FLOAT_EQ(simulator.getGPSAccuracy(), 100.0);

    simulator.setGPSAccuracy(300.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_FLOAT_EQ(simulator.getGPSAccuracy(), 300.0);
}

TEST_F(GPSSimulatorTest, CriticalGPSAccuracy) {
    simulator.setGPSAccuracy(300.0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_GT(simulator.getGPSAccuracy(), 200.0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_gps_simulator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
