#!/usr/bin/env python3

from os.path import join
import rospy
import numpy as np
from panda_robot import PandaArm
from myo_franka_control.msg import KCCartesianCmd
from utils import wait_gazebo, np2ros_obj
import tf
from constants import NODE_CMD_EXECUTOR, CMD_EXECUTOR_GAIN_MATRIX, CMD_EXECUTOR_CMD_TIMEOUT, CMD_PUBLISHER_TOPIC, Q_MAX, Q_MIN, XYZW, EKF_INPUT_POSE_POSITION, EKF_INPUT_POSE_FRAME_NAME, EKF_INPUT_POSE_PUBLISHER_FREQ, EKF_INPUT_POSE_FRAME_NAME, FRANKA_CONTROL_DT, FRANKA_CONTROL_FREQ, Q_DOT_MAX, Q_DOT_MIN, EKF_FREQ, VELOCITY_LIMITS_C1, VELOCITY_LIMITS_C2, VELOCITY_LIMITS_C3, VELOCITY_LIMITS_C4, PANDA_POSE_COV, CMD_EXECUTOR_MODE, NAIVE_SENSOR_FUSION_FREQ, CMD_EXECUTOR_MOM_GAIN
from geometry_msgs.msg import PoseWithCovarianceStamped
from threading import Lock, Thread
import queue
from math import floor
from custom_panda_arm import CustomPandaArm
import qpsolvers as qp
from enum import Enum



class KCMode(Enum):
    simple = 'simple'
    mom = 'mom'



Q_DOT_MAX = Q_DOT_MAX * 0.99
Q_DOT_MIN = Q_DOT_MIN * 0.99
Q_DOT_MIN_MAX = (Q_DOT_MIN, Q_DOT_MAX)


def get_orientation_error_matrix(curr_R, tgt_R):
    error = np.zeros(3)
    for i in range(3):
        error += np.cross(curr_R[:-1, i], tgt_R[:-1, i])
    return 0.5 * error



def rpy2omega(alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot):
    omega_x = np.cos(beta) * np.cos(gamma) * alpha_dot - np.sin(gamma) * beta_dot
    omega_y = np.cos(beta) * np.sin(gamma) * alpha_dot + np.cos(gamma) * beta_dot
    omega_z = -np.sin(beta) * alpha_dot + gamma_dot
    return np.array([omega_x, omega_y, omega_z])



def rpy_orientation_error(curr_orientation_rpy, tgt_orientation_rpy):
    curr_R = tf.transformations.euler_matrix(*curr_orientation_rpy)
    tgt_R = tf.transformations.euler_matrix(*tgt_orientation_rpy)
    return get_orientation_error_matrix(curr_R, tgt_R)



def clip_joint_velocity(qdot, q, qdot_min_max=None, alpha=0.98):
    current_qdot_max = alpha * np.minimum(
            VELOCITY_LIMITS_C1, 
            VELOCITY_LIMITS_C1, 
            np.maximum(
                0., 
                VELOCITY_LIMITS_C2 + np.sqrt(np.maximum(
                                                0., 
                                                VELOCITY_LIMITS_C3 * (VELOCITY_LIMITS_C4 - q)
                                                )
                                            )
                )
            )
    
    clipped_qdot = np.clip(qdot, -current_qdot_max, current_qdot_max)

    if qdot_min_max is not None:
        qdot_min, qdot_max = qdot_min_max
        clipped_qdot = np.clip(clipped_qdot, qdot_min, qdot_max)

    return clipped_qdot


def joint_velocity_damper(q, ps=0.05, pi=0.1, n=7, gain=1.0, q_min=Q_MIN, q_max=Q_MAX):
    Ain = np.zeros((n, n))
    Bin = np.zeros(n)
    
    for i in range(n):
        if q[i] - q_min[i] <= np.pi:
            Bin[i] = -gain * (((q_min[i] - q[i]) + ps) / (pi - ps))
            Ain[i, i] = -1
        if q_max[i] - q[i] <= np.pi:
            Bin[i] = gain * ((q_max[i] - q[i]) - ps) / (pi - ps)
            Ain[i, i] = 1

    return Ain, Bin




class CmdExecutor:
    
    def __init__(self, arm):
        self._arm = CustomPandaArm(arm)

        mode = rospy.get_param(CMD_EXECUTOR_MODE)
        self._mode = KCMode(mode)
        
        try:
            cmd_pub_freq = rospy.get_param(EKF_FREQ)
        except KeyError:
            cmd_pub_freq = rospy.get_param(NAIVE_SENSOR_FUSION_FREQ)
        self._dt = 1 / cmd_pub_freq
        assert FRANKA_CONTROL_FREQ >= cmd_pub_freq

        self._n_offset = floor(FRANKA_CONTROL_FREQ / cmd_pub_freq)
        self._delta_cmd_ratio = cmd_pub_freq / FRANKA_CONTROL_FREQ

        self._K = np.array(rospy.get_param(CMD_EXECUTOR_GAIN_MATRIX)).reshape(6, 6)
        self._lck = Lock()

        # gain term for control minimization with MoM
        self._Y = rospy.get_param(CMD_EXECUTOR_MOM_GAIN)

        self._Q = queue.Queue(2000)
        self._curr_cmd = np.zeros(7)
        self._freq = FRANKA_CONTROL_FREQ
        rospy.Timer(rospy.Duration(FRANKA_CONTROL_DT), self._exec_cmd)

        self._last_stamp = rospy.Time.now()
        self._update_ee_pose()
         
        cmd_queue_size = 2 * cmd_pub_freq
        cmd_pub_topic = rospy.get_param(CMD_PUBLISHER_TOPIC)
        rospy.Subscriber(cmd_pub_topic, KCCartesianCmd, self._on_cmd, queue_size=cmd_queue_size)


    def _update_ee_pose(self):
        self._ee_position, self._ee_orientation = self._arm.ee_pose
        self._last_stamp = rospy.Time.now()
        self._ee_orientation_rpy = tf.transformations.euler_from_quaternion(self._ee_orientation)
        self._ee_pose = np.concatenate([self._ee_position, self._ee_orientation_rpy])


    def _exec_cmd(self, _):
        self._arm.exec_velocity_cmd(self._Q.get(True))


    def _joint_velocity_mom_constraint(self, v, total_error, J, q):
        n = self._arm.n

        # quadratic component for objective function
        Q = np.eye(n + 6)
        
        # joint velocity component of Q
        Q[:n, :n] *= self._Y

        # slack component of Q
        Q[n:, n:] = (1 / total_error) * np.eye(6)

        # equality constraints
        Aeq = np.c_[J, np.eye(6)]
        beq = v.reshape((6,))

        # inequality constraints for joint limit avoidance
        Ain = np.zeros((n + 6, n + 6))
        bin = np.zeros(n + 6)
        
        # min angle (rad) in which the joint is allowed to approach to its limit
        ps = 0.05

        # minimum angle (rad) in which the velocity damper becomes active
        pi = 0.9

        # form the joint velocity damper
        Ain[:n, :n], bin[:n] = joint_velocity_damper(q, ps, pi, n)

        # linear component of objective function
        jacobm = self._arm.jacobm(J)
        c = np.r_[-jacobm.reshape((n,)), np.zeros(6)]

        # lower and upper bounds on the joint velocity and slack variable
        lb = -np.r_[Q_DOT_MAX, 10 * np.ones(6)]
        ub = np.r_[Q_DOT_MAX, 10 * np.ones(6)]

        # solve for the joint velocity qd
        qd = qp.solve_qp(Q, c, Ain, bin, Aeq, beq, lb=lb, ub=ub, solver='cvxopt')

        if qd is not None:
            cmd = qd[:n]
        else:
            rospy.logwarn('Quadratic programming cannot be solved')
            cmd = np.zeros(n)

        return cmd


    def _exec_mode_mom(self, twist_error, pose_error, J, q):
        total_error = np.abs(pose_error).sum()
        joint_velocity_cmd = self._joint_velocity_mom_constraint(twist_error, total_error, J, q)
        # Thread(target=self._copy_cmd, args=(joint_velocity_cmd,)).start()
        self._Q.put(joint_velocity_cmd)


    def _exec_mode_simple(self, twist_error, J, q):
        J_inv = np.linalg.pinv(J)
        qdot = J_inv @ twist_error
        qdot = clip_joint_velocity(qdot, q)
        self._Q.put(qdot)


    def _exec(self, *args, **kwargs):
        if self._mode == KCMode.simple:
            self._exec_mode_simple(kwargs['twist_error'], kwargs['J'], kwargs['q'])
        elif self._mode == KCMode.mom:
            self._exec_mode_mom(kwargs['twist_error'], kwargs['pose_error'], kwargs['J'], kwargs['q'])
        else:
            raise NotImplementedError()


    def _on_cmd(self, msg):
        with self._lck:
            self._update_ee_pose()
        
        q = self._arm.q

        J = self._arm.jacobian
        
        tgt_twist = np.array(msg.tgt_twist)

        tgt_orientation_rpy = np.array(msg.tgt_pose[3:])
        
        # assuming the robot is a perfect integrator, the position error is 
        # (current_position + velocity * dt) - current_position = velocity * dt
        # note that this is a strong assumption and will rarely be verified in reality
        dt = abs((self._last_stamp - msg.stamp).to_sec())
        position_error = tgt_twist[:3] * dt

        orientation_error = rpy_orientation_error(self._ee_orientation_rpy, tgt_orientation_rpy)

        pose_error = np.concatenate([position_error, orientation_error])

        twist_error = tgt_twist + self._K @ pose_error

        self._exec(twist_error=twist_error, pose_error=pose_error, J=J, q=q)
    

    def _interpolate_cmd(self, cmd):
        delta_cmd = cmd - self._curr_cmd
        for i in range(self._n_offset):
            self._curr_cmd += delta_cmd * self._delta_cmd_ratio
            self._Q.put(self._curr_cmd)


    def _copy_cmd(self, cmd):
        for _ in range(self._n_offset):
            self._Q.put(cmd)
                        


if __name__ == '__main__':
    rospy.init_node(NODE_CMD_EXECUTOR)
    wait_gazebo()
    arm = PandaArm()
    arm.set_command_timeout(rospy.get_param(CMD_EXECUTOR_CMD_TIMEOUT))
    cmd_executor = CmdExecutor(arm)
    rospy.spin()

