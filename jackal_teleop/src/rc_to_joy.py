#!/usr/bin/env python3

import rospy
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import Joy

def rc_callback(msg):
    joy_msg = Joy()
    joy_msg.header.stamp = rospy.Time.now()

    # Normalize RC values (1000-2000) to joystick range (-1.0 to 1.0)
    def scale(value, min_in=1000, max_in=2000, min_out=-1.0, max_out=1.0):
        return min_out + (float(value - min_in) / (max_in - min_in) * (max_out - min_out))

    # Map RC channels to joystick axes (adjust mapping as needed)
    joy_msg.axes = [
        scale(int((msg.channels[0])/10) * 10),  # Roll
        scale(int(msg.channels[1]/10) * 10),  # Pitch
        scale(msg.channels[2]),  # Throttle
        scale(msg.channels[3]),  # Yaw
    ]

    # Map extra RC channels to joystick buttons (1 if >1500, else 0)
    joy_msg.buttons = [
        1 if msg.channels[4] > 1510 else 0,  # Button 1
        1 if msg.channels[5] > 1510 else 0,  # Button 2
        1 if msg.channels[6] > 1510 else 0,  # Button 3
        1 if msg.channels[7] > 1510 else 0,  # Button 4
    ]

    joy_pub.publish(joy_msg)

# Initialize ROS node
rospy.init_node('rc_to_joy', anonymous=True)
joy_pub = rospy.Publisher('/joy', Joy, queue_size=10)
rospy.Subscriber('/mavros/rc/in', RCIn, rc_callback)

rospy.spin()
