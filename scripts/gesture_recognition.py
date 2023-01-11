#!/usr/bin/env python3

import rospy
import joblib
import numpy as np
from constants import NODE_GESTURE_RECOGNITION, ML_TRAINED_MODEL_PATH, MYO_TOPIC_EMG, ML_SAMPLE_WINDOW_SIZE, ML_COOLDOWN_WINDOW_SIZE, ML_GESTURE_TOPIC
from utils import ParamRetriever, ros_obj2np, np2ros_obj
from enum import IntEnum
from ros_myo.msg import EmgArray
from std_msgs.msg import Int8, String



def load_model(path):
    # quick-n-dirty fix for pickle imports

    def _sequence_channel_median(x):
        return np.median(x, axis=1)
    import sys, types
    sys.modules['pipelines'] = types.ModuleType('pipelines')
    sys.modules['pipelines'].__dict__['_sequence_channel_median'] = _sequence_channel_median

    import warnings
    warnings.filterwarnings("ignore", category=UserWarning)
    pipeline = joblib.load(path)
    
    del sys.modules['pipelines']

    return pipeline



class Gestures(IntEnum):
    
    # see https://github.com/aljazfrancic/myo-readings-dataset
    # gestures follow labels obtained from kmeans clustering

    NEUTRAL = 0
    FIST = 1
    EXTENSION = 2
    FLEXION = 3



class MyoGestureRecognition:

    def __init__(self, emg_topic, window_size, cooldown_size, model, gesture_topic):
        self._subscriber = rospy.Subscriber(emg_topic, EmgArray, self._on_emg, queue_size=200)
        self._publisher = rospy.Publisher(gesture_topic, Int8, queue_size=100)
        self._publisher_str = rospy.Publisher('/gesture_str', String, queue_size=10)
        self._window_size = window_size
        self._cooldown_size = cooldown_size
        self._model = model
        self._emg_data = np.zeros((window_size, 8))
        self._i = 0
        self._cooldown_i = 0
        self._is_cooldown = False


    def _on_emg(self, msg):
        if not self._is_cooldown:
           sample = np.array(msg.data, dtype=np.float64)
           self._emg_data[self._i] = sample
           self._i += 1
    
           if self._i == self._window_size:
               gesture = self._predict().item()
               self._publisher.publish(gesture)
               self._publisher_str.publish(String(Gestures(gesture).name))
               self._i = 0
               self._is_cooldown = True
        else:
            self._cooldown_i += 1

        if self._cooldown_i >= self._cooldown_size:
            self._is_cooldown = False

    
    def _predict(self):
        return self._model.predict(self._emg_data[np.newaxis])



    @classmethod
    def GetDefault(cls):
        emg_topic = ParamRetriever(MYO_TOPIC_EMG)()
        window_size = ParamRetriever(ML_SAMPLE_WINDOW_SIZE)()
        cooldown_size = ParamRetriever(ML_COOLDOWN_WINDOW_SIZE)()
        model_path = ParamRetriever(ML_TRAINED_MODEL_PATH)()
        model = load_model(model_path)
        gesture_topic = ParamRetriever(ML_GESTURE_TOPIC)()

        return cls(emg_topic,
                window_size,
                cooldown_size,
                model,
                gesture_topic)



if __name__ == '__main__':
    rospy.init_node(NODE_GESTURE_RECOGNITION)
    gesture_recognizer = MyoGestureRecognition.GetDefault()
    rospy.spin()
