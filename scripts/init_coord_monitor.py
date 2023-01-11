#!/usr/bin/env python3

import rospy
from utils import ParamRetriever
from constants import NODE_INIT_COORD_MONITOR, EKF_INPUT_POSE_POSITION, EKF_INPUT_POSE_FIRST_MSG_PUBLISHED
from geometry_msgs.msg import PoseWithCovarianceStamped



def on_ekf_first_pose_msg(_):
    rospy.loginfo('EKF first pose input received')
    rospy.set_param(EKF_INPUT_POSE_FIRST_MSG_PUBLISHED, True)
    subscriber_ekf_first_pose.unregister()



if __name__ == '__main__':
    rospy.init_node(NODE_INIT_COORD_MONITOR)
    subscriber_ekf_first_pose = rospy.Subscriber(ParamRetriever(EKF_INPUT_POSE_POSITION)(), PoseWithCovarianceStamped, on_ekf_first_pose_msg, queue_size=1)
    rospy.spin()
