#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import rospy
from sensor_msgs.msg import Imu
from utils import ros_obj2np
from constants import MYO_TOPIC_IMU_MADGWICK, MYO_TOPIC_IMU_SMA
from threading import Lock


def on_imu(msg):
    global i, j, x, y
    acc = ros_obj2np(msg.linear_acceleration)
    j = i % N
    y[j] = acc
    x[j] = i
    i += 1


def plot():
    plt.ion()
    fig, ax = plt.subplots(3, 1)
    acc_x, = ax[0].plot(x, y[:, 0], 'r-')
    acc_y, = ax[1].plot(x, y[:, 1], 'g-')
    acc_z, = ax[2].plot(x, y[:, 2], 'b-')

    while not rospy.is_shutdown():
        with lock:
            data = np.concatenate([y[j:], y[0:j]])
            acc_x.set_ydata(data[:, 0])
            acc_y.set_ydata(data[:, 1])
            acc_z.set_ydata(data[:, 2])
        r.sleep()


if __name__ == '__main__':
    rospy.init_node('plotter')
    topic_name = rospy.get_param(MYO_TOPIC_IMU_MADGWICK)
    N = 500
    i = 0
    j = 0
    x = np.zeros(N)
    y = np.zeros((N, 3))
    lock = Lock()
    r = rospy.Rate(1)
    rospy.Subscriber(topic_name, Imu, on_imu, queue_size=2 * N)
    plot()
