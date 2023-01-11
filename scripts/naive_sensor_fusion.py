#!/usr/bin/env python3

import rospy
import numpy as np
from threading import Lock
from constants import NODE_NAIVE_SENSOR_FUSION, NAIVE_SENSOR_FUSION_FREQ, NAIVE_SENSOR_FUSION_INPUT_IMU_TOPIC, NAIVE_SENSOR_FUSION_INPUT_TWIST_TOPIC, NAIVE_SENSOR_FUSION_TOPIC, XYZW, ODOM, NAIVE_SENSOR_FUSION_CHILD_FRAME_ID, Q_NWU2ENU
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, TransformStamped
import tf2_ros
import tf
from utils import ros_obj2np, np2ros_obj



class NaiveSensorFusion:

    def __init__(self, imu_topic, twist_topic, out_topic, freq, parent_frame_id, child_frame_id):
        self._freq = freq
        self._lck = Lock()
        self._odom_msg = Odometry()
        self._odom_msg.header.frame_id = parent_frame_id
        self._odom_msg.child_frame_id = child_frame_id
        
        self._broadcaster = tf2_ros.TransformBroadcaster()
        self._T_parent2child = TransformStamped()
        self._T_parent2child.header.frame_id = parent_frame_id
        self._T_parent2child.child_frame_id = child_frame_id
        
        self._publisher = rospy.Publisher(out_topic, Odometry, queue_size=20)
        rospy.Subscriber(imu_topic, Imu, self._on_imu, queue_size=10)
        rospy.Subscriber(twist_topic, TwistWithCovarianceStamped, self._on_twist, queue_size=10)
        rospy.Timer(rospy.Duration(1. / freq), self._do_work_publisher)


    def _on_imu(self, msg):
        with self._lck:
            self._odom_msg.pose.pose.orientation = msg.orientation
        q_parent2child_enu = ros_obj2np(msg.orientation, XYZW)
        q_parent2child_nwu = tf.transformations.quaternion_multiply(Q_NWU2ENU, q_parent2child_enu)
        self._T_parent2child.header.stamp = msg.header.stamp
        np2ros_obj(q_parent2child_nwu, self._T_parent2child.transform.rotation, XYZW)
        self._broadcaster.sendTransform(self._T_parent2child)


    def _on_twist(self, msg):
        with self._lck:
            self._odom_msg.twist.twist.linear = msg.twist.twist.linear


    def _do_work_publisher(self, event):
        with self._lck:
            self._odom_msg.header.stamp = rospy.Time.now()
            self._publisher.publish(self._odom_msg)


    @classmethod
    def GetDefault(cls):
        imu_topic = rospy.get_param(NAIVE_SENSOR_FUSION_INPUT_IMU_TOPIC)
        twist_topic = rospy.get_param(NAIVE_SENSOR_FUSION_INPUT_TWIST_TOPIC)
        out_topic = rospy.get_param(NAIVE_SENSOR_FUSION_TOPIC)
        freq = rospy.get_param(NAIVE_SENSOR_FUSION_FREQ)
        parent_frame_id = ODOM
        child_frame_id = rospy.get_param(NAIVE_SENSOR_FUSION_CHILD_FRAME_ID)

        return cls(imu_topic, twist_topic, out_topic, freq, parent_frame_id, child_frame_id)



if __name__ == '__main__':
    rospy.init_node(NODE_NAIVE_SENSOR_FUSION)
    naive_sensor_fusion = NaiveSensorFusion.GetDefault()
    rospy.spin()
