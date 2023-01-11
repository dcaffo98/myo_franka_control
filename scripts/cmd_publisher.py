#!/usr/bin/env python3

import rospy
import numpy as np
from myo_franka_control.msg import KCCartesianCmd
from utils import wait_gazebo, ros_obj2np, np2ros_obj, qv_mult
import tf
from constants import NODE_CMD_PUBLISHER, CMD_PUBLISHER_TOPIC, CMD_PUBLISHER_INPUT_TOPIC, XYZW, Q_NWU2ENU
from nav_msgs.msg import Odometry
from abc import ABC, abstractmethod 



def odom2pose(msg):
    position = ros_obj2np(msg.pose.pose.position)                         
    orientation = ros_obj2np(msg.pose.pose.orientation, XYZW)
    orientation_rpy = tf.transformations.euler_from_quaternion(orientation) 
    return np.concatenate([position, orientation_rpy])                  



def odom2twist(msg):
    twist_lin = ros_obj2np(msg.twist.twist.linear) 
    twist_ang = ros_obj2np(msg.twist.twist.angular)
    return np.concatenate([twist_lin, twist_ang])  



class IODOM2CmdStrategy(ABC):
    
    @abstractmethod
    def do_work(self, msg, *args, **kwargs):
        ...    

    def __call__(self, msg, *args, **kwargs):
        return self.do_work(msg, *args, **kwargs)



class SimpleODOM2CmdStrategy(IODOM2CmdStrategy):

    def do_work(self, msg, *args, **kwargs):
        # pose
        position = ros_obj2np(msg.pose.pose.position)
        orientation = ros_obj2np(msg.pose.pose.orientation, XYZW)
        orientation = tf.transformations.quaternion_multiply(Q_NWU2ENU, orientation)
        rpy_orientation = tf.transformations.euler_from_quaternion(orientation)
        tgt_pose = np.concatenate([position, rpy_orientation])
        
        # twist
        tgt_twist = odom2twist(msg)
        tgt_twist[:3] = qv_mult(orientation, tgt_twist[:3])
        tgt_twist[3:] = qv_mult(orientation, tgt_twist[3:])

        return tgt_pose, tgt_twist



class CmdPublisher():

    def __init__(self, odom_topic, odom2cmd_strategy):
        self._publisher = rospy.Publisher(rospy.get_param(CMD_PUBLISHER_TOPIC), KCCartesianCmd, queue_size=1000)
        self._subscriber = rospy.Subscriber(odom_topic, Odometry, self.on_odom, queue_size=1000)
        self._odom2cmd_strategy = odom2cmd_strategy


    def on_odom(self, msg):
        tgt_pose, tgt_twist = self._odom2cmd_strategy(msg)
        cmd = KCCartesianCmd()
        cmd.stamp = msg.header.stamp
        cmd.tgt_pose = tgt_pose
        cmd.tgt_twist = tgt_twist
        self._publisher.publish(cmd)



if __name__ == '__main__':
    rospy.init_node(NODE_CMD_PUBLISHER)
    wait_gazebo()
    odom2cmd_strategy = SimpleODOM2CmdStrategy()
    CmdPublisher(rospy.get_param(CMD_PUBLISHER_INPUT_TOPIC), odom2cmd_strategy)
    rospy.spin()
