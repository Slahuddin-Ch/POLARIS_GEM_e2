#!/usr/bin/env python3

# ================================================================
# File name: gem_sensor_info.py                                                                  
# Description: show sensor info in Rviz                                                              
# Author: Hang Cui
# Email: hangcui3@illinois.edu                                                                     
# Date created: 06/10/2021                                                                
# Date last modified: 07/02/2021                                                          
# Version: 0.1                                                                    
# Usage: rosrun gem_gazebo gem_sensor_info.py                                                                     
# Python version: 3.8                                                             
# ================================================================

# Python Headers
import numpy as np

# ROS Headers
import rospy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from jsk_rviz_plugins.msg import OverlayText
from std_msgs.msg import ColorRGBA, Float32, String, Bool
from tf.transformations import euler_from_quaternion


class GEMOverlay(object):

    def __init__(self):
          
        self.sensor_info_pub = rospy.Publisher("/gem/sensor_info", OverlayText, queue_size=1)
        self.gps_sub     = rospy.Subscriber("/gem/gps/fix", NavSatFix, self.gps_callback)
        self.imu_sub     = rospy.Subscriber("/gem/imu", Imu, self.imu_callback)
        self.odom_sub    = rospy.Subscriber("/gem/base_footprint/odom", Odometry, self.odom_callback)

        # New subscribers for additional sensor data
        self.battery_sub = rospy.Subscriber('/battery_level', Float32, self.battery_callback)
        self.temp_sub = rospy.Subscriber('/temperature', Float32, self.temperature_callback)
        self.gps_accuracy_sub = rospy.Subscriber('/gps_accuracy', Float32, self.gps_accuracy_callback)
        self.signal_sub = rospy.Subscriber('/signal_strength', Float32, self.signal_callback)
        self.emergency_sub = rospy.Subscriber('/emergency_stop', Bool, self.emergency_callback)
        

        self.rate        = rospy.Rate(10)

        self.sensor_overlaytext = self.default_overlaytext_style()

        self.lat         = 0.0
        self.lon         = 0.0
        self.alt         = 0.0
        self.imu_yaw     = 0.0

        self.x = 0.0
        self.y = 0.0
        self.x_dot = 0.0
        self.y_dot = 0.0
        self.gazebo_yaw = 0.0

        self.battery_level = 100.0
        self.temperature = 20.0
        self.gps_accuracy = 500.0
        self.signal_strength = 2.0
        self.emergency_stop = False

    def gps_callback(self, msg):
        self.lat = msg.latitude
        self.lon = msg.longitude
        self.alt = msg.altitude

    def imu_callback(self, msg):

        orientation_q      = msg.orientation
        angular_velocity   = msg.angular_velocity
        linear_accel       = msg.linear_acceleration

        orientation_list   = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        
        self.imu_yaw       = yaw

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.gazebo_yaw = yaw
        self.x_dot = msg.twist.twist.linear.x
        self.y_dot = msg.twist.twist.linear.y

    # Callbacks for additional sensor data
    def battery_callback(self, msg):
        self.battery_level = msg.data

    def temperature_callback(self, msg):
        self.temperature = msg.data

    def gps_accuracy_callback(self, msg):
        self.gps_accuracy = msg.data

    def signal_callback(self, msg):
        self.signal_strength = msg.data

    def emergency_callback(self, msg):
        self.emergency_stop = msg.data

    @staticmethod
    def default_overlaytext_style():

        text            = OverlayText()
        text.width      = 280
        text.height     = 400
        text.left       = 10
        text.top        = 10
        text.text_size  = 12
        text.line_width = 2
        text.font       = "DejaVu Sans Mono"
        text.fg_color   = ColorRGBA(25 / 255.0, 1.0, 240.0 / 255.0, 1.0)
        text.bg_color   = ColorRGBA(0.0, 0.0, 0.0, 0.2)
        return text

    def start_demo(self):
        
        while not rospy.is_shutdown():
            f_vel = np.sqrt(self.x_dot**2 + self.y_dot**2)

            self.sensor_overlaytext.text = """--------------------------
                                 Sensor (Measurement):
                                 Lat = %.6f
                                 Lon = %.6f
                                 Alt = %.6f
                                 Yaw = %.6f
                                 --------------------------
                                 Gazebo (Ground Truth):
                                 X_pos = %.3f
                                 Y_pos = %.3f
                                 X_dot = %.3f
                                 Y_dot = %.3f
                                 F_vel = %.3f
                                 Yaw   = %.3f
                                 --------------------------
                                 Battery Level =   %.2f
                                 Temperature =     %.2f C
                                 GPS Accuracy =    %.2f mm
                                 Signal Strength =  %.2f
                                 Emergency Stop =  %s                           
                              """ % (self.lat, self.lon, self.alt, self.imu_yaw,
                                     self.x, self.y, self.x_dot, self.y_dot, f_vel,
                                     self.gazebo_yaw,
                                     self.battery_level, self.temperature, self.gps_accuracy,
                                     self.signal_strength,
                                     "Activated" if self.emergency_stop else "Deactivated")

            self.sensor_info_pub.publish(self.sensor_overlaytext)

            self.rate.sleep()


def gem_overlay():

    rospy.init_node('gem_sensor_info', anonymous=True)

    gem_overlay_object = GEMOverlay()

    try:
        gem_overlay_object.start_demo()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    gem_overlay()
