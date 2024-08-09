#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def temperature_scenario():
    temp_pub = rospy.Publisher('temperature', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Gradually increase temperature
    for temp in range(30, 56):
        rospy.loginfo(f"Publishing temperature: {temp}° C")
        temp_pub.publish(Float32(temp))
        rate.sleep()

    # Spike to 60° C
    rospy.loginfo("Publishing temperature: 60° C")
    temp_pub.publish(Float32(60))
    rospy.sleep(1)
