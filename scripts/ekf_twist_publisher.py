#!/usr/bin/env python3

import rospy
import numpy as np
from constants import NODE_EKF_TWIST_PUBLISHER, EKF_TOPIC, EKF_TWIST_PUBLISHER_TOPIC, Q_NWU2ENU, XYZW
from geometry_msgs.msg import TwistStamped
from utils import qv_mult, ros_obj2np, np2ros_obj
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf



def on_odom(msg):
    twist_out = TwistStamped()
    twist_out.header = msg.header

    linear_child = ros_obj2np(msg.twist.twist.linear)
    angular_child = ros_obj2np(msg.twist.twist.angular)
    q_parent2child = ros_obj2np(msg.pose.pose.orientation, XYZW)
    q_parent2child = tf.transformations.quaternion_multiply(Q_NWU2ENU, q_parent2child) 

    linear_parent = qv_mult(q_parent2child, linear_child)
    angular_parent = qv_mult(q_parent2child, angular_child)

    np2ros_obj(linear_parent, twist_out.twist.linear)
    np2ros_obj(angular_parent, twist_out.twist.angular)

    twist_publisher.publish(twist_out)
    
    imu_out = Imu()
    imu_out.header = msg.header
    np2ros_obj(q_parent2child, imu_out.orientation, XYZW)
    imu_publisher.publish(imu_out)


if __name__ == '__main__':
    rospy.init_node(NODE_EKF_TWIST_PUBLISHER)
    rospy.Subscriber(EKF_TOPIC, Odometry, on_odom, queue_size=1000)
    twist_publisher = rospy.Publisher(EKF_TWIST_PUBLISHER_TOPIC, TwistStamped, queue_size=10)
    imu_publisher = rospy.Publisher('/ekf/imu', Imu, queue_size=10)
    rospy.spin()
    
    
