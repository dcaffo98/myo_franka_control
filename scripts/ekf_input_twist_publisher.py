#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int8
from utils import np2ros_obj, ParamRetriever
from constants import NODE_EKF_TWIST_PUBLISHER, EKF_INPUT_TWIST_FRAME_NAME, EKF_INPUT_TWIST, ML_GESTURE_TOPIC, ML_REF_FORWARD_TWIST, PANDA_TWIST_COV, EKF_INPUT_TWIST_FREQ
from geometry_msgs.msg import TwistWithCovarianceStamped
from gesture_recognition import Gestures
from copy import deepcopy
import queue



class EKFInputTwistPublisher:

    def __init__(self):
        gesture_topic = ParamRetriever(ML_GESTURE_TOPIC)()
        self._subscriber = rospy.Subscriber(gesture_topic, Int8, self._on_gesture, queue_size=100)
        
        twist_topic = ParamRetriever(EKF_INPUT_TWIST)()
        self._publisher = rospy.Publisher(twist_topic, TwistWithCovarianceStamped, queue_size=100)

        twist_msg = TwistWithCovarianceStamped()
        twist_msg.header.frame_id = ParamRetriever(EKF_INPUT_TWIST_FRAME_NAME)()
        twist_msg.twist.covariance = PANDA_TWIST_COV.tolist()
        self._twist_msg = twist_msg

        self._ref_forward_twist = np.array(ParamRetriever(ML_REF_FORWARD_TWIST)())
        self._ref_backward_twist = - self._ref_forward_twist
        self._neutral_twist = [0., 0., 0.]
        
        self._Q = queue.Queue()

        f = ParamRetriever(EKF_INPUT_TWIST_FREQ)()
        rospy.Timer(rospy.Duration(1 / f), self._do_work_publisher)


    def _on_gesture(self, gesture):
        twist_msg = deepcopy(self._twist_msg)
        
        gesture = gesture.data

        if gesture == Gestures.NEUTRAL or gesture == Gestures.FIST:
            np2ros_obj(self._neutral_twist, twist_msg.twist.twist.linear)
        elif gesture == Gestures.EXTENSION:
            np2ros_obj(self._ref_forward_twist, twist_msg.twist.twist.linear)
        elif gesture == Gestures.FLEXION:
            np2ros_obj(self._ref_backward_twist, twist_msg.twist.twist.linear)

        self._Q.put(twist_msg)

    
    def _do_work_publisher(self, event):
        try:
            twist_msg = self._Q.get(block=False)
            self._twist_msg = twist_msg
        except queue.Empty:
            twist_msg = self._twist_msg

        twist_msg.header.stamp = event.current_real
        self._publisher.publish(twist_msg)


if __name__ == '__main__':
    rospy.init_node(NODE_EKF_TWIST_PUBLISHER)
    ekf_inptu_twist_publisher = EKFInputTwistPublisher() 
    rospy.spin()

