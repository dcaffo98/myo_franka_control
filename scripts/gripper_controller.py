#!/usr/bin/env python3

import rospy
import numpy as np
from gesture_recognition import Gestures
from panda_robot import PandaArm
from constants import NODE_GRIPPER_CONTROLLER, GRIPPER_COOLDOWN_INTERVAL, GRIPPER_MAX_X, GRIPPER_MIN_X, GRIPPER_REF_V, ML_GESTURE_TOPIC, GRIPPER_DELTA_X
from std_msgs.msg import Int8



class GripperController:

    def __init__(self, panda_arm, min_x, max_x, delta_x, reset_interval, gesture_topic):
        self._panda_arm = panda_arm
        self._min_x = min_x
        self._max_x = max_x
        self._delta_x = delta_x
        self._reset_interval = reset_interval
        self._current_gesture = Gestures.NEUTRAL

        panda_arm.exec_gripper_cmd(max_x)
        self._last_check_time = rospy.Time.now()
        self._elapsed = 0.0
        self._is_closing = False
        
        self._gesture_subscriber = rospy.Subscriber(gesture_topic, Int8, self._on_gesture, queue_size=200)


    @property
    def gripper_position(self):
        return self._panda_arm.angles(include_gripper=True)[-2:].sum()


    def _on_gesture(self, msg):
        self._current_gesture = msg.data
        if self._current_gesture == Gestures.FIST:
            self._is_closing = True
            tgt_x = max(self.gripper_position - self._delta_x, self._min_x)
            self._panda_arm.exec_gripper_cmd(tgt_x)
        else:
            now = rospy.Time.now()
            if self._is_closing:
                self._elapsed = 0.0
                self._is_closing = False
            else:
                self._elapsed += (now - self._last_check_time).to_sec()
                if self._elapsed >= self._reset_interval:
                    tgt_x = min(self.gripper_position + 1.5 * self._delta_x, self._max_x)
                    self._panda_arm.exec_gripper_cmd(tgt_x)
            self._last_check_time = now


    @classmethod
    def GetDefault(cls):
        panda_arm = PandaArm()
        min_x = rospy.get_param(GRIPPER_MIN_X)
        max_x = rospy.get_param(GRIPPER_MAX_X)
        delta_x = rospy.get_param(GRIPPER_DELTA_X)
        reset_interval = rospy.get_param(GRIPPER_COOLDOWN_INTERVAL)
        gesture_topic = rospy.get_param(ML_GESTURE_TOPIC)

        return cls(panda_arm, min_x, max_x, delta_x, reset_interval, gesture_topic)



if __name__ == '__main__':
    rospy.init_node(NODE_GRIPPER_CONTROLLER)
    gripper_controller = GripperController.GetDefault()
    rospy.spin()

