#include <gtest/gtest.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "battery_simulation/battery_simulator.hpp"

class BatterySimulatorTest : public ::testing::Test {
protected:
    virtual void SetUp() {
        battery_simulator = new BatterySimulator();
        battery_simulator->setBatteryLevel(100.0);
        battery_level_received = false;
        battery_status_received = false;
        battery_sub = nh.subscribe("battery_level", 10, &BatterySimulatorTest::batteryCallback, this);
        battery_status_sub = nh.subscribe("battery_status", 10, &BatterySimulatorTest::batteryStatusCallback, this);
        battery_simulator->startSimulation();
        ROS_INFO("Test setup completed.");
    }

    virtual void TearDown() {
        battery_sub.shutdown();
        battery_status_sub.shutdown();
        delete battery_simulator;
        battery_simulator = nullptr;
    }

    void batteryCallback(const std_msgs::Float32::ConstPtr& msg) {
        battery_level_received = true;
        battery_level = msg->data;
        ROS_INFO("Battery level received: %.2f", battery_level);
    }

    void batteryStatusCallback(const std_msgs::Float32::ConstPtr& msg) {
        battery_status_received = true;
        battery_status = msg->data;
        ROS_INFO("Battery status received: %.2f", battery_status);
    }

    ros::NodeHandle nh;
    ros::Subscriber battery_sub;
    ros::Subscriber battery_status_sub;
    BatterySimulator* battery_simulator;
    bool battery_level_received;
    bool battery_status_received;
    float battery_level;
    float battery_status;
};

TEST_F(BatterySimulatorTest, BatteryLevelUpdate) {
    ROS_INFO("Running BatteryLevelUpdate test...");
    ros::AsyncSpinner spinner(1); // Use async spinner to handle callbacks
    spinner.start();
    ros::Rate rate(1);
    while (ros::ok() && !battery_level_received) {
        ros::spinOnce();
        rate.sleep();
    }

    EXPECT_TRUE(battery_level_received);
    EXPECT_LE(battery_level, 100.0);
    EXPECT_GE(battery_level, 0.0);
    ROS_INFO("BatteryLevelUpdate test completed.");
    spinner.stop();
}

TEST_F(BatterySimulatorTest, BatteryStatusUpdate) {
    ROS_INFO("Running BatteryStatusUpdate test...");
    ros::AsyncSpinner spinner(1); // Use async spinner to handle callbacks
    spinner.start();
    ros::Rate rate(1);
    while (ros::ok() && !battery_status_received) {
        ros::spinOnce();
        rate.sleep();
    }

    EXPECT_TRUE(battery_status_received);
    EXPECT_LE(battery_status, 100.0);
    EXPECT_GE(battery_status, 0.0);
    ROS_INFO("BatteryStatusUpdate test completed.");
    spinner.stop();
}

TEST_F(BatterySimulatorTest, SetBatteryLevel) {
    ROS_INFO("Running SetBatteryLevel test...");
    try {
        battery_simulator->setBatteryLevel(75.0);
        EXPECT_EQ(battery_simulator->getBatteryLevel(), 75.0);
        ROS_INFO("Set battery level to 75.0 and verified.");

        battery_simulator->setBatteryLevel(150.0);  // Invalid value
        EXPECT_NE(battery_simulator->getBatteryLevel(), 150.0);
        ROS_INFO("Attempted to set battery level to 150.0 and verified rejection.");
    } catch (const std::exception& e) {
        ROS_ERROR("Exception caught in SetBatteryLevel test: %s", e.what());
        FAIL() << "Exception caught in SetBatteryLevel test: " << e.what();
    }
    ROS_INFO("SetBatteryLevel test completed.");
}

TEST_F(BatterySimulatorTest, CriticalBatteryLevel) {
    ROS_INFO("Running CriticalBatteryLevel test...");
    battery_simulator->setBatteryLevel(49.0);
    ros::AsyncSpinner spinner(1); // Use async spinner to handle callbacks
    spinner.start();
    ros::Rate rate(1);
    while (ros::ok() && !battery_level_received) {
        ros::spinOnce();
        rate.sleep();
    }

    EXPECT_TRUE(battery_level_received);
    EXPECT_LE(battery_level, 49.0);
    ROS_INFO("CriticalBatteryLevel test completed.");
    spinner.stop();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_battery_simulator");
    ::testing::InitGoogleTest(&argc, argv);
    ROS_INFO("Starting Google Test...");
    return RUN_ALL_TESTS();
}
