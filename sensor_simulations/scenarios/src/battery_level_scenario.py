#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def battery_level_scenario():
    battery_pub = rospy.Publisher('battery_level', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Gradually decrease battery level
    for level in range(100, 50, -1):
        rospy.loginfo(f"Publishing battery level: {level}%")
        battery_pub.publish(Float32(level))
        rate.sleep()

    # Sudden drop to 49%
    rospy.loginfo("Publishing battery level: 49%")
    battery_pub.publish(Float32(49))
    rospy.sleep(1)
