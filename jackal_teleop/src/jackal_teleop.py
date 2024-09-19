#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8, Bool

class JackalTeleop:
    def __init__(self):
        self.twist_buf_ = Twist()
        self.is_auto_ = False
        self.trigger_ = False
        self.prev_trigger_ = True
        self.trigger_count_ = 0

        self.num_views_ = 2

        self.joy_sub_ = rospy.Subscriber('joy', Joy, self.joy_cb)
        self.twist_sub_ = rospy.Subscriber('twist_auto', Twist, self.twist_cb)
        self.twist_pub_ = rospy.Publisher('twist_out', Twist, queue_size=1)
        self.is_auto_pub_ = rospy.Publisher('~is_auto', Bool, queue_size=1)
        self.trigger_pub_ = rospy.Publisher('~trigger', UInt8, queue_size=1)

        self.pub_timer_ = rospy.Timer(rospy.Duration(0.1), self.pub_cb)

    def pub_cb(self, event):
        if not self.is_auto_:
            self.twist_pub_.publish(self.twist_buf_)

    def joy_cb(self, msg):
        self.twist_buf_.linear.x = msg.axes[3]*2
        self.twist_buf_.angular.z = msg.axes[2]*0.5
        self.is_auto_ = msg.axes[4] > 0
        self.trigger_ = msg.buttons[0]

        # publish auto flag
        is_auto_msg = Bool()
        is_auto_msg.data = self.is_auto_
        self.is_auto_pub_.publish(is_auto_msg)

        if self.trigger_ == 1:
            trigger_msg = UInt8()
            trigger_msg.data = self.trigger_count_
            self.trigger_pub_.publish(trigger_msg)
            
            if self.trigger_count_ == self.num_views_:
                self.trigger_count_ = 0
            else:
                self.trigger_count_ += 1
        self.prev_trigger_ = self.trigger_

    def twist_cb(self, msg):
        if self.is_auto_:
            self.twist_pub_.publish(msg)

if __name__=='__main__':
    rospy.init_node('jackal_teleop')
    jackal_telep = JackalTeleop()
    rospy.spin()
