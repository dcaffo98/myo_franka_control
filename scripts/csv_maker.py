#!/usr/bin/env python3

import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from utils import ros_q2np_q, ros_xyz2np_array
import csv
import argparse
from constants import XYZW
from utils import ros_obj2np
import tf



parser = argparse.ArgumentParser()
parser.add_argument('--filename', type=str, default='imu.csv')
parser.add_argument('--msg_cls', type=str, default='Imu')
parser.add_argument('--topic', type=str, default='/ekf/input/imu')



class CSVWriter:
    
    fieldnames = []                              


    def __init__(self, filename, topic, cls) -> None:
        self._f = open(filename, 'w')
        self._writer = csv.DictWriter(self._f, self.fieldnames)
        self._writer.writeheader()
        rospy.Subscriber(topic, cls, self._on_msg, queue_size=1000)


    def __del__(self):
        self._f.close()


    def _preprocess(self, msg, *args, **kwargs):
        return msg

    
    def _datarow(self, msg, *args, **kwargs):
        return []


    def _on_msg(self, msg):
        msg = self._preprocess(msg)
        datarow = self._datarow(msg)
        dictrow = {k:v for k, v in zip(self.fieldnames, datarow)}
        self._writer.writerow(dictrow)
    


class IMUCSVWriter(CSVWriter):
    
    fieldnames = [                       
                    'timestamp',     
                    'quaternion_x',  
                    'quaternion_y',  
                    'quaternion_z',  
                    'quaternion_w',  
                    'acceleration_x',
                    'acceleration_y',
                    'acceleration_z',
                    'gyroscope_x',   
                    'gyroscope_y',   
                    'gyroscope_z'    
    ]


    def _datarow(self, msg, *args, **kwargs):
        t = msg.header.stamp.to_sec()                  
        q = ros_q2np_q(msg.orientation)                
        acc = ros_xyz2np_array(msg.linear_acceleration)
        gyro = ros_xyz2np_array(msg.angular_velocity)  
        return [t, *q, *acc, *gyro]



class IMURPYCSVWriter(CSVWriter):
    fieldnames = [                     
                'timestamp',       
                'roll',    
                'pitch',    
                'yaw',   
                'acceleration_x',  
                'acceleration_y',  
                'acceleration_z',  
                'gyroscope_x',     
                'gyroscope_y',     
                'gyroscope_z'      
    ]               


    def _datarow(self, msg, *args, **kwargs):
        q = ros_obj2np(msg.orientation, XYZW)
        acc = ros_obj2np(msg.linear_acceleration)
        gyro = ros_obj2np(msg.angular_velocity)
        rpy = tf.transformations.euler_from_quaternion(q)
        return [msg.header.stamp.to_sec(), *rpy, *acc, *gyro]



class TWISTCSVWriter(CSVWriter):

    fieldnames = [
        'timestamp',
        'linear_x',
        'linear_y',
        'linear_z',
        'angular_x',
        'angular_y',
        'angular_z'
    ]


    def _datarow(self, msg, *args, **kwargs):
        twist_linear = ros_obj2np(msg.twist.twist.linear)
        twist_angular = ros_obj2np(msg.twist.twist.angular)
        return [msg.header.stamp.to_sec(), *twist_linear, *twist_angular]



if __name__ == "__main__":
    rospy.init_node('csv_maker')
    args = parser.parse_args()
    filename = args.filename
    topic = args.topic
    msg_cls_str = args.msg_cls
    msg_cls = eval(msg_cls_str)
    
    if msg_cls is Imu:
        writer_cls = IMURPYCSVWriter
    elif msg_cls is TwistWithCovarianceStamped:
        writer_cls = TWISTCSVWriter
    else:
        raise ValueError(f"Unknown class: {msg_cls}")

    writer = writer_cls(filename, topic, msg_cls)

    rospy.spin()
    
