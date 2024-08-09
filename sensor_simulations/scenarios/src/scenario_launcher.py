#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

# Import the scenarios
from battery_level_scenario import battery_level_scenario
from temperature_scenario import temperature_scenario
from gps_accuracy_scenario import gps_accuracy_scenario

class ScenarioLauncher:
    def __init__(self):
        rospy.init_node('scenario_launcher', anonymous=True)
        rospy.Subscriber('/button_topic', String, self.button_callback)
        self.rate = rospy.Rate(10)

    def button_callback(self, msg):
        if msg.data == "scenario1":
            rospy.loginfo("Scenario 1 pressed: Running Battery Level Scenario")
            battery_level_scenario()
        elif msg.data == "scenario2":
            rospy.loginfo("Scenario 2 pressed: Running Temperature Variation Scenario")
            temperature_scenario()
        elif msg.data == "scenario3":
            rospy.loginfo("Scenario 3 pressed: Running GPS Accuracy Fluctuation Scenario")
            gps_accuracy_scenario()
        else:
            rospy.logwarn("Unknown button pressed: No scenario assigned.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        launcher = ScenarioLauncher()
        launcher.run()
    except rospy.ROSInterruptException:
        pass
