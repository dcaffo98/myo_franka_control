#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt8
import argparse
from time import sleep



if __name__ == "__main__":
    rospy.init_node('myo_caffeine')
    sec = float(rospy.get_param('/myo_caffeine/sec', 10))
    pub = rospy.Publisher('/myo_raw/vibrate', UInt8, queue_size=1)
    while not rospy.is_shutdown():
        pub.publish(UInt8(1))
        sleep(sec)

