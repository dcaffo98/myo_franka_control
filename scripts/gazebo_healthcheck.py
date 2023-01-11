#!/usr/bin/env python3

import rospy
from constants import GAZEBO_FUZZY_HEALTHCHECK_SERVICE, GAZEBO_HEALTHCHECK_COOLDOWN, GAZEBO_HEALTHCHECK_READY, NODE_GAZEBO_HEALTHCHECK
from utils import ParamRetriever


if __name__ == '__main__':
    rospy.init_node(NODE_GAZEBO_HEALTHCHECK)
    service_name = ParamRetriever(GAZEBO_FUZZY_HEALTHCHECK_SERVICE)()
    cooldown = ParamRetriever(GAZEBO_HEALTHCHECK_COOLDOWN)()
    rospy.wait_for_service(service_name)
    rospy.sleep(cooldown)
    rospy.loginfo('Gazebo awaited')
    rospy.set_param(GAZEBO_HEALTHCHECK_READY, True)
    rospy.loginfo('Gazebo ready advertised!')

