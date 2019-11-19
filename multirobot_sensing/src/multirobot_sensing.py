#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PointStamped, Point
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import tf2_ros
import ros_numpy
import tf.transformations as tr

import numpy as np
import os

NODE_NAME = 'multirobot_sensing_node'

class MultirobotSensing:
    def __init__(self):
        self.pc_subs_ = []
        self.pc_subs_.append(rospy.Subscriber(os.path.join(NODE_NAME, 'r1/pc'),
                                              PointCloud2, lambda pc : self.pc_cb(0, pc)))
        self.pc_subs_.append(rospy.Subscriber(os.path.join(NODE_NAME, 'r2/pc'),
                                              PointCloud2, lambda pc : self.pc_cb(1, pc)))

        self.rel_pos_pubs_ = []
        self.rel_pos_pubs_.append(rospy.Publisher(os.path.join(NODE_NAME, 'r1/r2_pos'),
                                                  PointStamped, queue_size=10))
        self.rel_pos_pubs_.append(rospy.Publisher(os.path.join(NODE_NAME, 'r2/r1_pos'),
                                                  PointStamped, queue_size=10))

        self.viz_pub_ = rospy.Publisher(os.path.join(NODE_NAME, 'viz'),
                                        Marker, queue_size=10)

        #tf stuff
        self.tf_buffer_ = tf2_ros.Buffer()
        self.tf_listener_ = tf2_ros.TransformListener(self.tf_buffer_)

    def pc_cb(self, robot, pc):
        pc_np = ros_numpy.point_cloud2.pointcloud2_to_array(pc).reshape([1024, 64]) #64x1024 matrix

        for ind in range(0,2):
            if ind == robot: #don't care about self transform
                continue
            try:
                trans = self.tf_buffer_.lookup_transform("r"+str(robot+1)+"/os1_lidar", 
                            "r"+str(ind+1)+"/base_link", rospy.Time());
            except (tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException, tf2_ros.LookupException):
                rospy.logwarn("tf not found")
                continue

            Trel = self.transform_to_np(trans)

            #determine ray direction
            ray = Trel[0:3, 3]
            ray_sp = self.xyz_to_spherical(ray)
            shape = np.array(pc_np.shape)
            ray_ind = (ray_sp[1:3]*shape/np.array([-np.pi*2, np.pi*30/180]) + shape/2).astype(int)

            try:
                raytrace = np.array([pc_np[ray_ind[0], ray_ind[1]]['x'],
                                     pc_np[ray_ind[0], ray_ind[1]]['y'],
                                     pc_np[ray_ind[0], ray_ind[1]]['z']])
            except IndexError:
                raytrace = np.zeros([3,1])
            if np.linalg.norm(raytrace) == 0:
                raytrace = np.zeros([3,1])

            good_ray = True
            if np.abs(np.linalg.norm(raytrace) - np.linalg.norm(ray)) < 0.5 and np.linalg.norm(raytrace) > 0:
                rel_pos = PointStamped()
                rel_pos.header.stamp = pc.header.stamp
                rel_pos.header.frame_id = "r"+str(robot+1)+"/os1_lidar"
                rel_pos.point.x = Trel[0, 3]
                rel_pos.point.y = Trel[1, 3]
                rel_pos.point.z = Trel[2, 3]
                self.rel_pos_pubs_[robot].publish(rel_pos)
            else:
                good_ray = False

            #draw viz ray
            raytrace_sp = self.xyz_to_spherical(raytrace)
            self.draw_line(raytrace, "r"+str(robot+1)+"/os1_lidar", "r"+str(robot+1)+"/raytrace", good_ray)

    def draw_line(self, endpt, frame, ns, good_ray):
        marker = Marker()
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.ns = ns
        marker.header.frame_id = frame
        marker.color.a = 1.0
        marker.color.r = 1.0
        if good_ray:
            marker.color.g = 1.0
            marker.color.b = 1.0
        marker.scale.x = 0.02
        marker.pose.orientation.w = 1.0
        marker.points.append(Point(0,0,0))
        marker.points.append(Point(endpt[0], endpt[1], endpt[2]))
        self.viz_pub_.publish(marker)

    def xyz_to_spherical(self, xyz):
        spherical = np.array([0.,0.,0.])
        spherical[0] = np.linalg.norm(xyz)
        spherical[1] = np.arctan2(xyz[1], xyz[0]) #theta
        spherical[2] = np.arctan2(np.linalg.norm(xyz[0:2]), xyz[2]) - np.pi/2 #phi (centered around horiz)

        return spherical
    
    def transform_to_np(self, trans):
        #convert from trans to homogeneous matrix
        quat = np.array([trans.transform.rotation.x,
                         trans.transform.rotation.y,
                         trans.transform.rotation.z,
                         trans.transform.rotation.w])
        T = tr.quaternion_matrix(quat)
        T[0, 3] = trans.transform.translation.x
        T[1, 3] = trans.transform.translation.y
        T[2, 3] = trans.transform.translation.z

        return T

if __name__ == '__main__':
   rospy.init_node(NODE_NAME) 
   multirobot_sensing = MultirobotSensing()
   rospy.spin()
