#!/usr/bin/env python3

import rospy
import sys
import tf
from sensor_msgs.msg import Imu
import numpy as np
from utils import ros_obj2np, np2ros_obj, qv_mult, ParamRetriever
from constants import NODE_MYO_RAW_PREPROCESS, MYO_TOPIC_IMU, M_EE2HUMAN, VIRTUAL_EE_FRAME_ID, MADGWICK_INPUT_TOPIC, MYO_FRAME_ID, XYZW, G
import tf2_ros
from geometry_msgs.msg import TransformStamped



Q_EE2HUMAN = tf.transformations.quaternion_from_matrix(M_EE2HUMAN)



def on_imu(msg):
    # --- preprocessing ---
    acc = ros_obj2np(msg.linear_acceleration) * G
    gyro = np.radians(ros_obj2np(msg.angular_velocity))
    # ---
    
    # --- myo frame to virtual end-effector frame rotation
    msg.header.frame_id = VIRTUAL_EE_FRAME_NAME

    acc_ee = qv_mult(Q_EE2HUMAN, acc)
    np2ros_obj(acc_ee, msg.linear_acceleration)

    gyro = qv_mult(Q_EE2HUMAN, gyro)
    np2ros_obj(gyro, msg.angular_velocity)
    # ---

    publisher.publish(msg)



if __name__ == '__main__':
    rospy.init_node(NODE_MYO_RAW_PREPROCESS)
    VIRTUAL_EE_FRAME_NAME = ParamRetriever(VIRTUAL_EE_FRAME_ID)()[1:]
    rospy.Subscriber(ParamRetriever(MYO_TOPIC_IMU)(), Imu, on_imu, queue_size=200)    
    publisher = rospy.Publisher(ParamRetriever(MADGWICK_INPUT_TOPIC)(), Imu, queue_size=200)

    broadcaster = tf2_ros.StaticTransformBroadcaster()
    t = TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = VIRTUAL_EE_FRAME_NAME
    t.child_frame_id = ParamRetriever(MYO_FRAME_ID)()[1:]

    np2ros_obj(Q_EE2HUMAN, t.transform.rotation)
    
    broadcaster.sendTransform(t)

    rospy.spin()
