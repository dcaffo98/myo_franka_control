import numpy as np
from abc import ABC, abstractmethod
from constants import G
from utils import rearrange_array



class IMUFilter(ABC):                                                                                                                      
                                                                                                                                        
    def __init__(self, w_size, imu_filter_strategy):                                                                                    
        super().__init__()
        self._w_size = w_size                                                                                                           
        self._imu_filter_strategy = imu_filter_strategy                                                                                 
        self._acc = np.zeros((w_size, 3))                                                                                               
        self._gyro = np.zeros((w_size, 3))                                                                                              
        self._quat = np.zeros((w_size, 4))                                                                                              
        self._timestamp = np.zeros(w_size)                                                                                              
        self._dt = np.zeros(w_size)                                                                                                     
        self._count = 0                                                                                                                 
        self._last_msg = None                                                                                                           
                                                                                                                                        
              
    def _preprocess(self, msg, *args, **kwargs):
        return msg


    def process(self, msg, *args, **kwargs):                                                                                            
        out = None                                                                                                                      
        
        self._update_state(msg)                                                                                                         
                                                                                                                                        
        self._count += 1                                                                                                                
                                                                                                                                        
        if self._count == self._w_size:                                                                                                 
            out = self._imu_filter_strategy(self._acc, self._gyro, self._quat, self._timestamp, self._dt, msg)                          
            self._count = 0                                                                                                             
                                                                                                                                        
        return out                                                                                                                      
                                                                                                                                        
                                                                                                                                        
    def __call__(self, msg, *args, **kwargs):                                                                                           
        return self.process(msg, *args, **kwargs)                                                                                       
                                                                                                                                        
                                                                                                                                        
    def _update_state(self, msg):
        msg = self._preprocess(msg)

        self._timestamp[self._count] = msg.header.stamp.to_time()                                                                       
        self._dt[self._count] = 0 if self._last_msg is None else (msg.header.stamp.to_time() - self._last_msg.header.stamp.to_time())   
                                                                                                                                        
        self._acc[self._count][0] = msg.linear_acceleration.x                                                                           
        self._acc[self._count][1] = msg.linear_acceleration.y                                                                           
        self._acc[self._count][2] = msg.linear_acceleration.z                                                                           
                                                                                                                                        
        self._gyro[self._count][0] = msg.angular_velocity.x                                                                             
        self._gyro[self._count][1] = msg.angular_velocity.y                                                                             
        self._gyro[self._count][2] = msg.angular_velocity.z                                                                             
                                                                                                                                        
        self._quat[self._count][0] = msg.orientation.x                                                                                  
        self._quat[self._count][1] = msg.orientation.y                                                                                  
        self._quat[self._count][2] = msg.orientation.z                                                                                  
        self._quat[self._count][3] = msg.orientation.w                                                                                  
                                                                                                                                        
        self._last_msg = msg  


    def _rearrange(self):                                                                                       
        i = self._count + 1                                                                                     
        return (rearrange_array(x, i) for x in [self._acc, self._gyro, self._quat, self._timestamp, self._dt])  



class SWIMUFilter(IMUFilter):

    def __init__(self, w_size, imu_filter_strategy, sort_msgs=True):
        super().__init__(w_size, imu_filter_strategy)
        self._sort_msgs = sort_msgs


    def process(self, msg, *args, **kwargs):
        out = None                                                                                           
        
        self._update_state(msg)
        
        if self._sort_msgs:
            acc, gyro, quat, timestamp, dt = self._rearrange()
        else:
            acc, gyro, quat, timestamp, dt = self._acc, self._gyro, self._quat, self._timestamp, self._dt

        out = self._imu_filter_strategy(acc, gyro, quat, timestamp, dt, msg)                                                                       
                                                                                                     
        self._count += 1                                                                                     
                                                                                                     
        if self._count == self._w_size:                                                                      
            self._count = 0                                                                                  
                                                                                                     
        return out                                                                                           



class IIMUFilterStrategy(ABC):                                               
                                                                             
    def __init__(self, w_size):  
        super().__init__()
        self._w_size = w_size                                                
                                                                             
    @abstractmethod                                                          
    def do_work(self, acc, gyro, quat, timestamp, dt, *args, **kwargs):      
        ...                                                                  
                                                                             
    def __call__(self, acc, gyro, quat, timestamp, dt, *args, **kwargs):     
        return self.do_work(acc, gyro, quat, timestamp, dt, *args, **kwargs) 
                                                                             

class NOPIMUFilterStrategy(IIMUFilterStrategy):

    def __init__(self):
        super().__init__(0)

    def do_work(self, acc, gyro, quat, timestamp, dt, *args, **kwargs):
        return args[0]


class SHOEDetector():
    def __init__(self, w_size, use_g=False):
        self._w_size = w_size
        self._g_factor = G if use_g else 1

    def SHOE(self, acc, gyro):                                              
        channel_mean_acc = acc.mean(axis=0)                                   
        acc_comp = acc - self._g_factor * (channel_mean_acc / np.linalg.norm(channel_mean_acc))
        acc_term = np.linalg.norm(acc_comp, axis=1).sum() / acc.var()         
        gyro_term = np.linalg.norm(gyro, axis=1).sum() / gyro.var()           
        return (acc_term + gyro_term) / self._w_size


def hist_check(acc, th_low, th_high):                         
    out = False, None
    diff = np.abs(np.diff(np.sign(acc[-2:]), axis=0))
    axes_to_check = np.nonzero(np.any(diff == 2, axis=0))[0]
    if len(axes_to_check):                                          
      abs_acc = np.abs(acc)
      th_low = th_low[axes_to_check] 
      axes_low = np.nonzero(np.any(abs_acc[-2:, axes_to_check] >= th_low, axis=0))[0]
      if len(axes_low):
        th_high = th_high[axes_to_check[axes_low]]                     
        axes_high = np.any(abs_acc[:, axes_to_check[axes_low]] >= th_high, axis=0)
        if np.any(axes_high):
          out = True, axes_to_check[axes_low[np.nonzero(axes_high)[0]]]                    
    return out
