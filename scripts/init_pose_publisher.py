#!/usr/bin/env python3

import rospy
import numpy as np
from constants import VIRTUAL_EE_FRAME_ID, NODE_INIT_POSE_PUBLISHER, MADGWICK_TOPIC, VIRTUAL_EE_INIT_FRAME_ID, ODOM, XYZW
from utils import ParamRetriever, np2ros_obj, ros_obj2np
from sensor_msgs.msg import Imu
import tf
import tf2_ros
from geometry_msgs.msg import TransformStamped



class InitPosePublisher:
    
    def __init__(self):
        myo_topic = ParamRetriever(MADGWICK_TOPIC)()
        self._virtual_ee_frame_id = ParamRetriever(VIRTUAL_EE_INIT_FRAME_ID)()
        self._myo_subscriber = rospy.Subscriber(myo_topic, Imu, self._on_first_myo_imu, queue_size=1)
        self._myo_transform_broadcaster = tf2_ros.StaticTransformBroadcaster()


    def _on_first_myo_imu(self, msg):
        transform = TransformStamped()
        
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = ODOM 
        transform.child_frame_id = self._virtual_ee_frame_id

        q_odom2myo = ros_obj2np(msg.orientation, XYZW)
        np2ros_obj(q_odom2myo, transform.transform.rotation, XYZW)

        self._myo_transform_broadcaster.sendTransform(transform)
        
        self._myo_subscriber.unregister()



if __name__ == '__main__':
    rospy.init_node(NODE_INIT_POSE_PUBLISHER)
    init_pose_publisher = InitPosePublisher()
    rospy.spin()
