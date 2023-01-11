import numpy as np
import tf
import rospy
from constants import PCKG_NAME, GAZEBO_HEALTHCHECK_READY, MOVEIT_HEALTHCHECK_READY
import subprocess
from time import sleep
import quaternion



def get_pckg_abs_path(pckg=PCKG_NAME):
    cmd = ('rospack find ' + pckg).split()
    return subprocess.run(cmd, stdout=subprocess.PIPE).stdout[:-1].decode('utf-8')


def np2ros_obj(array, ros_obj, attrs=['x', 'y', 'z']):          
    for i, attr in enumerate(attrs):                            
        setattr(ros_obj, attr, array[i])                        
                

def ros_obj2np(ros_obj, attrs=['x', 'y', 'z']):                 
    return np.array([getattr(ros_obj, attr) for attr in attrs]) 


def rpy2omega(alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot):               
    omega_x = np.cos(beta) * np.cos(gamma) * alpha_dot - np.sin(gamma) * beta_dot
    omega_y = np.cos(beta) * np.sin(gamma) * alpha_dot + np.cos(gamma) * beta_dot
    omega_z = -np.sin(beta) * alpha_dot + gamma_dot                              
    return np.array([omega_x, omega_y, omega_z])                                 


# rotate vector v1 by quaternion q1 
def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]



def np_quat2array_quat(np_quat):
    return np.array([np_quat.x, np_quat.y, np_quat.z, np_quat.w])


def array_quat2np_quat(array_quat):
    return np.quaternion(array_quat[-1], *array_quat[:-1])


def rearrange_array(array, i):                     
    return np.concatenate([array[i:], array[0:i]]) 
             

def avg_quaternion(quat):                    
    A = quat.T @ quat                               
    A /= (len(quat) ** 2)              
    out = np.linalg.eigh(A)[1][:, -1]               
    return out                                      
    # wxyz --> xyzw                                 
    # return  np.array([*out[1:], out[0]])          



class ParamRetriever:
    """ Can also be used as quick and dirty healtchecks """

    def __init__(self, param_name, attempts=None, interval=1, list2numpy=True, raise_exception=False):
        self._param_name = param_name
        self._attempts = attempts
        self._run_indefinitely = True if attempts is None else False
        self._interval = interval
        self._list2numpy = True
        self._raise_exception = raise_exception
        self._retrieved = None
        self._do_work()


    def _do_work(self):
        while self._retrieved is None:
            try:
                if not self._run_indefinitely and self._attempts <= 0:
                    msg = f"Param \'{self._param_name}\' couldn't be retrieved"
                    rospy.logwarn(msg)
                    if self._raise_exception:
                        raise ValueError(msg)
                self._retrieved = rospy.get_param(self._param_name)
                if type(self._retrieved) == list and self._list2numpy:
                    self._retrieved = np.array(self._retrieved)
            except KeyError:
                sleep(self._interval)
                if not self._run_indefinitely:
                    self._attempts -= 1


    def get(self, *args, **kwargs):
        return self._retrieved


    def __call__(self, *args, **kwargs):
        return self.get(*args, **kwargs)



def wait_bool_param(param_name):
    r = rospy.Rate(1)
    while not rospy.get_param(param_name, False):
        r.sleep()


def wait_gazebo():
    wait_bool_param(GAZEBO_HEALTHCHECK_READY)


def cross(a, b):
    x = a[1] * b[2] - a[2] * b[1]
    y = a[2] * b[0] - a[0] * b[2]
    z = a[0] * b[1] - a[1] * b[0]
    return np.array([x, y, z])

