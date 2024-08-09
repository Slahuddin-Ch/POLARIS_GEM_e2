#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

def gps_accuracy_scenario():
    gps_pub = rospy.Publisher('gps_accuracy', Float32, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    # Start with high accuracy
    rospy.loginfo("Publishing GPS accuracy: 100 mm")
    gps_pub.publish(Float32(100))
    rospy.sleep(10)

    # Intermittently drop accuracy
    rospy.loginfo("Dropping GPS accuracy to 150 mm for 20 seconds")
    gps_pub.publish(Float32(150))
    rospy.sleep(20)

    rospy.loginfo("Restoring GPS accuracy to 100 mm")
    gps_pub.publish(Float32(100))
    rospy.sleep(10)

    rospy.loginfo("Dropping GPS accuracy to 180 mm for 20 seconds")
    gps_pub.publish(Float32(180))
    rospy.sleep(20)
