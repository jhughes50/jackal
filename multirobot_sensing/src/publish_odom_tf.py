#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PointStamped, Point
import tf2_ros
import ros_numpy
import tf.transformations as tr

import numpy as np
import os

NODE_NAME = 'publish_odom_tf'

class PublishOdomTf:
    def __init__(self):
        self.ns_ = rospy.get_param('~ns')
        self.odom_sub_ = rospy.Subscriber('/'+os.path.join(self.ns_, 'ground_truth/odom'), 
                                                Odometry, lambda odom : self.odom_cb(self.ns_, odom))
        #tf stuff
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)

    def odom_cb(self, robot, odom):
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "map"
        t.child_frame_id = str(robot)+"/base_link"
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation

        br.sendTransform(t)

if __name__ == '__main__':
   rospy.init_node(NODE_NAME) 
   multirobot_sensing = PublishOdomTf()
   rospy.spin()
