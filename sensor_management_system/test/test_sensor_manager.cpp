#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>

class SensorManagerTest : public ::testing::Test {
protected:
    void SetUp() override {
        nh_ = ros::NodeHandle();
        state_sub_ = nh_.subscribe("robot_state", 10, &SensorManagerTest::stateCallback, this);
        battery_pub_ = nh_.advertise<std_msgs::Float32>("battery_level", 10);
        temperature_pub_ = nh_.advertise<std_msgs::Float32>("temperature", 10);
        gps_accuracy_pub_ = nh_.advertise<std_msgs::Float32>("gps_accuracy", 10);
        signal_strength_pub_ = nh_.advertise<std_msgs::Int32>("signal_strength", 10);
        emergency_stop_pub_ = nh_.advertise<std_msgs::Bool>("emergency_stop", 10);
        robot_state_ = "";
        ros::Duration(0.5).sleep(); // Give some time to establish connections
    }

    void stateCallback(const std_msgs::String::ConstPtr& msg) {
        robot_state_ = msg->data;
        ROS_INFO("Robot state updated: %s", robot_state_.c_str());
    }

    ros::NodeHandle nh_;
    ros::Subscriber state_sub_;
    ros::Publisher battery_pub_;
    ros::Publisher temperature_pub_;
    ros::Publisher gps_accuracy_pub_;
    ros::Publisher signal_strength_pub_;
    ros::Publisher emergency_stop_pub_;
    std::string robot_state_;
};

TEST_F(SensorManagerTest, BatteryLevelError) {
    std_msgs::Float32 battery_msg;
    battery_msg.data = 49.0;
    battery_pub_.publish(battery_msg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    EXPECT_EQ(robot_state_, "ERROR");
}

TEST_F(SensorManagerTest, TemperatureError) {
    std_msgs::Float32 temperature_msg;
    temperature_msg.data = 56.0;
    temperature_pub_.publish(temperature_msg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    EXPECT_EQ(robot_state_, "ERROR");
}

TEST_F(SensorManagerTest, GpsAccuracyError) {
    std_msgs::Float32 gps_msg;
    gps_msg.data = 199.0;
    gps_accuracy_pub_.publish(gps_msg);
    ros::spinOnce();
    ros::Duration(16.0).sleep(); // Simulate the delay
    EXPECT_EQ(robot_state_, "ERROR");
}

TEST_F(SensorManagerTest, SignalStrengthNoSignalError) {
    std_msgs::Int32 signal_msg;
    signal_msg.data = 0;
    signal_strength_pub_.publish(signal_msg);
    ros::spinOnce();
    ros::Duration(11.0).sleep(); // Simulate the delay
    EXPECT_EQ(robot_state_, "ERROR");
}

TEST_F(SensorManagerTest, SignalStrengthLowSignalError) {
    std_msgs::Int32 signal_msg;
    signal_msg.data = 2;
    signal_strength_pub_.publish(signal_msg);
    ros::spinOnce();
    ros::Duration(21.0).sleep(); // Simulate the delay
    EXPECT_EQ(robot_state_, "ERROR");
}

TEST_F(SensorManagerTest, EmergencyStopError) {
    std_msgs::Bool stop_msg;
    stop_msg.data = true;
    emergency_stop_pub_.publish(stop_msg);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
    EXPECT_EQ(robot_state_, "ERROR");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "test_sensor_manager");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
