#!/usr/bin/env python3

import numpy as np
from copy import deepcopy
from sensor_msgs.msg import Imu
import rospy
from imu_filter import IMUFilter, NOPIMUFilterStrategy, IIMUFilterStrategy, SHOEDetector, SWIMUFilter
from constants import NODE_EKF_INPUT_IMU_PUBLISHER, EKF_INPUT_IMU_INPUT_TOPIC, EKF_INPUT_IMU, EKF_INPUT_IMU_FRAME_NAME, ODOM, VIRTUAL_EE_INIT_FRAME_ID, EE_INIT_FRAME_ID, Q_NWU2ENU, XYZW, Q_ENU2NWU, EKF_INPUT_IMU_ORIENTATION_COV, MYO_IMU_FILTER_SHOE_GAMMA, MYO_IMU_FILTER_W_SIZE, MYO_IMU_FILTER_USE_SHOE
from utils import ros_obj2np, np2ros_obj
import tf
import tf2_ros



ZEROS3 = np.array([0., 0., 0.])



class SHOEImuFilterStrategy(IIMUFilterStrategy):

    def __init__(self, w_size, shoe_gamma):
        super().__init__(w_size)
        self._gamma = shoe_gamma
        self._shoe = SHOEDetector(w_size, use_g=False)
        self._last_q = None


    @classmethod
    def GetDefault(cls):
        w_size = rospy.get_param(MYO_IMU_FILTER_W_SIZE)
        shoe_gamma = rospy.get_param(MYO_IMU_FILTER_SHOE_GAMMA)
        return cls(w_size, shoe_gamma)


    def do_work(self, acc, gyro, quat, timestamp, dt, *args, **kwargs):
        msg = args[0]

        if self._shoe.SHOE(acc, gyro) < self._gamma:
            # motion detected
            self._last_q = ros_obj2np(msg.orientation, XYZW)
        else:
            # stance hp confirmed
            if self._last_q is not None:
                np2ros_obj(self._last_q, msg.orientation, XYZW)
            np2ros_obj(ZEROS3, msg.linear_acceleration)
            np2ros_obj(ZEROS3, msg.angular_velocity)

        return msg



class EKFInputImuPublisherFilter(SWIMUFilter):
    
    def __init__(self, w_size, filter_strategy, out_msg_frame_id, fixed_frame_id, virtual_ee_init_frame_id, ee_init_frame_id):
        super().__init__(w_size, filter_strategy, sort_msgs=False)
        self._frame_id = out_msg_frame_id
        self._listener = tf.TransformListener()                                                                                                          
        self._fixed_frame_id = fixed_frame_id                                                                                                            
        self._virtual_ee_frame_id = virtual_ee_init_frame_id                                                                                             
        self._ee_init_frame_id = ee_init_frame_id                                                                                                        
        self._wait_transforms()         


    @classmethod
    def GetDefault(cls):
        w_size = rospy.get_param(MYO_IMU_FILTER_W_SIZE)

        use_shoe = rospy.get_param(MYO_IMU_FILTER_USE_SHOE)
        filter_strategy = SHOEImuFilterStrategy.GetDefault() if use_shoe else NOPIMUFilterStrategy()

        out_msg_frame_id = rospy.get_param(EKF_INPUT_IMU_FRAME_NAME)
        fixed_frame_id = ODOM
        virtual_ee_init_frame_id = rospy.get_param(VIRTUAL_EE_INIT_FRAME_ID)
        ee_init_frame_id = rospy.get_param(EE_INIT_FRAME_ID)

        return cls(w_size, filter_strategy, out_msg_frame_id, fixed_frame_id, virtual_ee_init_frame_id, ee_init_frame_id)


    def _wait_transforms(self):      
        while not rospy.is_shutdown():                                                                                                                  
            try:                                                                                                                                        
                # target, source, ...                                                                                                                   
                self._listener.waitForTransform(self._ee_init_frame_id, self._fixed_frame_id, rospy.Time(), rospy.Duration(2))                          
                self._listener.waitForTransform(self._fixed_frame_id, self._virtual_ee_frame_id, rospy.Time(), rospy.Duration(2))                       
                                                                                                                                                        
                (_, self._q_fixed2ee_init_nwu) = self._listener.lookupTransform(self._ee_init_frame_id, self._fixed_frame_id, rospy.Time(0))   
                (_, self._q_virtual_ee_init2fixed_enu) = self._listener.lookupTransform(self._fixed_frame_id, self._virtual_ee_frame_id, rospy.Time(0)) 

                self._q_fixed2ee_init_enu = tf.transformations.quaternion_multiply(Q_ENU2NWU, self._q_fixed2ee_init_nwu)
                
                # self._q_ee_init2virtual_ee_init_enu = tf.transformations.quaternion_conjugate(
                #        tf.transformations.quaternion_multiply(self._q_virtual_ee_init2fixed_enu, self._q_fixed2ee_init_enu))
                return

            except (tf.LookupException, tf.ConnectivityException, tf2_ros.tf2.TransformException):                                                                                      
                continue                                                                                                                                
                                                       

    def _preprocess(self, msg, *args, **kwargs):
        q_fixed2virtual_ee_enu = ros_obj2np(msg.orientation, XYZW)                                                             
                                                                                                                             
        # get rotation relative to the initial myo orientation                                                                       
        q_virtual_ee_init2virtual_ee_enu = tf.transformations.quaternion_multiply(self._q_virtual_ee_init2fixed_enu, q_fixed2virtual_ee_enu) 

        # q_ee_init2virtual_ee_enu = tf.transformations.quaternion_multiply(self._q_ee_init2virtual_ee_init_enu, q_virtual_ee_init2virtual_ee_enu)
        
        # express the orientation difference between myo init and current pose wrt odom 
        # i.e. myo_init2current ~= ee_init2current
        q_fixed2ee_enu = tf.transformations.quaternion_multiply(self._q_fixed2ee_init_enu, q_virtual_ee_init2virtual_ee_enu)
        
        # TODO: (debug only): convert back to NWU to better visualize in rviz
        # q_fixed2ee_enu = tf.transformations.quaternion_multiply(Q_NWU2ENU, q_fixed2ee_enu)

        np2ros_obj(q_fixed2ee_enu, msg.orientation, attrs=XYZW)

        msg.header.frame_id = self._frame_id

        msg.orientation_covariance = EKF_INPUT_IMU_ORIENTATION_COV

        return msg
        


def on_imu(msg):                                               
    out_msg = imu_filter(msg)                                     
    if out_msg is not None:
        publisher.publish(out_msg)
                       

        
if __name__ == '__main__':                                     
    rospy.init_node(NODE_EKF_INPUT_IMU_PUBLISHER)                       
    imu_in_topic = rospy.get_param(EKF_INPUT_IMU_INPUT_TOPIC)
    imu_out_topic = rospy.get_param(EKF_INPUT_IMU)
    imu_filter = EKFInputImuPublisherFilter.GetDefault()
    rospy.Subscriber(imu_in_topic, Imu, on_imu, queue_size=200) 
    publisher = rospy.Publisher(imu_out_topic, Imu, queue_size=200)
    rospy.spin()                                               

