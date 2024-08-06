#include <ros/ros.h>
#include <gtest/gtest.h>
#include "signal_simulation/signal_simulator.hpp"

class SignalSimulatorTest : public ::testing::Test {
protected:
    SignalSimulator simulator;

    virtual void SetUp() {
        simulator.setSignalStrength(1);
        simulator.startSimulation();
        ros::Duration(1.0).sleep(); // Allow some time for the simulation to start
    }

    virtual void TearDown() {
        simulator.stopSimulation();
    }
};

TEST_F(SignalSimulatorTest, SignalStrengthUpdate) {
    simulator.setSignalStrength(1);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_EQ(simulator.getSignalStrength(), 1);

    simulator.setSignalStrength(2);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_EQ(simulator.getSignalStrength(), 2);
}

TEST_F(SignalSimulatorTest, NoSignalWarning) {
    simulator.setSignalStrength(0);
    ros::Duration(1.0).sleep(); // Allow some time for the simulator to process
    EXPECT_EQ(simulator.getSignalStrength(), 0);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_signal_simulator");
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
