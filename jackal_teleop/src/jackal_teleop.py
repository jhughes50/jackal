#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class JackalTeleop:
    def __init__(self):
        self.twist_buf_ = Twist()
        self.is_auto_ = False
        self.trigger_ = False

        self.joy_sub_ = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.twist_sub_ = rospy.Subscriber('twist_auto', Twist, self.twist_cb)
        self.twist_pub_ = rospy.Publisher('twist_out', Twist, queue_size=1)
        self.is_auto_pub_ = rospy.Publisher('~is_auto', Bool, queue_size=1)
        self.trigger_pub_ = rospy.Publisher('~trigger', Bool, queue_size=1)

        self.pub_timer_ = rospy.Timer(rospy.Duration(0.1), self.pub_cb)

    def pub_cb(self, event):
        if not self.is_auto_:
            self.twist_pub_.publish(self.twist_buf_)

    def joy_cb(self, msg):
        self.twist_buf_.linear.x = msg.axes[3]*2
        self.twist_buf_.angular.z = msg.axes[2]*0.5
        self.is_auto_ = msg.axes[4] > 0
        self.trigger_ = msg.buttons[0] == 0

        # publish auto flag
        is_auto_msg = Bool()
        is_auto_msg.data = self.is_auto_
        self.is_auto_pub_.publish(is_auto_msg)
        
        trigger_msg = Bool()
        trigger_msg.data = self.trigger_
        self.trigger_pub_.publish(trigger_msg)

    def twist_cb(self, msg):
        if self.is_auto_:
            self.twist_pub_.publish(msg)

if __name__=='__main__':
    rospy.init_node('jackal_teleop')
    jackal_telep = JackalTeleop()
    rospy.spin()
