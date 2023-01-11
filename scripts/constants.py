import numpy as np
import tf


# gravity
G = 9.80665                                                                    
ONE_OVER_G = 1 / G
G_NP = np.array([0., 0., G])                                                    
G_NP_OMG = np.array([0., 0., G, 1.])                                           

# rotation matrices
M_NED2ENU = np.array([[0., 1., 0., 0.], [1., 0., 0., 0.], [0., 0., -1., 0.], [0., 0., 0., 1.]]) 
M_ENU2NWU = tf.transformations.rotation_matrix(np.pi / 2, np.array([0., 0., 1.]))  
M_RAW2ACC = np.array([[0., 0., -1., 0.], [0., -1., 0., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]])
M_EE2HUMAN = np.array([[0., 0., 1., 0.], [0. ,-1., 0., 0.], [1., 0., 0., 0.], [0., 0., 0., 1.]])
M_NWU2NED = np.array([[1., 0., 0., 0.], [0., -1., 0., 0.], [0., 0., -1., 0.], [0., 0., 0., 1.]])
M_NWU2ENU = np.array([[0., 1., 0., 0.], [-1., 0., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])

# rotation quaternion
Q_NWU2NED = tf.transformations.quaternion_from_matrix(M_NWU2NED)
Q_NWU2ENU = tf.transformations.quaternion_from_matrix(M_NWU2ENU)
Q_ENU2NWU = tf.transformations.quaternion_conjugate(Q_NWU2ENU)
Q_EE2HUMAN = tf.transformations.quaternion_from_matrix(M_EE2HUMAN)

# utils
PCKG_NAME = 'myo_franka_control'
XYZ = tuple('x y z'.split())
XYZW = tuple('x y z w'.split())

# node names
NODE_CMD_PUBLISHER = 'cmd_publisher'
NODE_CMD_EXECUTOR = 'cmd_executor'
NODE_EKF_INPUT_IMU_PUBLISHER = 'ekf_input_imu_publisher'
NODE_MYO_LAUNCH = 'myo_launch'
NODE_GAZEBO_HEALTHCHECK = 'gazebo_healthcheck'
NODE_INIT_COORD_MONITOR = 'init_coord_monitor'
NODE_MYO_RAW_PREPROCESS = 'myo_raw_preprocess'
NODE_EKF_TWIST_PUBLISHER = 'ekf_twist_publisher'
NODE_INIT_POSE_PUBLISHER = 'pose_init_publisher'
NODE_GESTURE_RECOGNITION = 'gesture_recognition'
NODE_GRIPPER_CONTROLLER = 'gripper_controller'
NODE_NAIVE_SENSOR_FUSION = 'naive_sensor_fusion'

# franka params
FRANKA_WS_XY_RADIUS = '/franka/workspace/secondary/xy_radius'
FRANKA_WS_MAX_Z = '/franka/workspace/secondary/max_z'
FRANKA_CONTROL_FREQ = 250
FRANKA_CONTROL_DT = 1 / FRANKA_CONTROL_FREQ
Q_MAX = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
Q_MIN = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
Q_DOT_MAX = np.array([2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100])
# Q_DOT_MAX = np.array([1.8, 1.8, 1.8, 1.8, 2., 2., 2.])
Q_DOT_MIN = - Q_DOT_MAX.copy()

VELOCITY_LIMITS_C1 = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])
VELOCITY_LIMITS_C2 = np.array([-0.30, -0.20, -0.20, -0.30, -0.35, -0.35, -0.35])
VELOCITY_LIMITS_C3 = np.array([12.0, 5.17, 7.00, 8.00, 34.0, 11.0, 34.0])
VELOCITY_LIMITS_C4 = np.array([2.75010, 1.79180, 2.90650, -0.1458, 2.81010, 4.52050, 3.01960])

Q_MAX.setflags(write=False)
Q_MIN.setflags(write=False)
Q_DOT_MAX.setflags(write=False)
Q_DOT_MAX.setflags(write=False)
VELOCITY_LIMITS_C1.setflags(write=False)
VELOCITY_LIMITS_C2.setflags(write=False)
VELOCITY_LIMITS_C3.setflags(write=False)
VELOCITY_LIMITS_C4.setflags(write=False)

# frames
VIRTUAL_EE_INIT_FRAME_ID = '/frames/virtual_ee_init_frame_id'
ODOM = 'odom'
EE_INIT_FRAME_ID = '/frames/ee_init_frame_id'

# myo 
MYO_IMU_NOMINAL_FREQ = 200                                                      
MYO_FRAME_ID = '/myo/frame_id'
MYO_REFERENCE_FRAME_ID = '/myo/reference_frame_id'
MYO_TOPIC_IMU = '/myo/topic_imu'
MYO_TOPIC_EMG = '/myo/topic_emg'
MYO_TOPIC_ORI = '/myo/topic_ori'
MYO_TOPIC_ORI_DEG = '/myo/topic_ori_deg'
MYO_CALIBRATION_REF_ACC = '/myo_calibration/ref_acc'                            
MYO_IMU_FILTER_W_SIZE = '/myo/imu_filter/w_size'
MYO_IMU_FILTER_SHOE_GAMMA = '/myo/imu_filter/shoe_gamma'
MYO_IMU_FILTER_USE_SHOE = '/myo/imu_filter/use_shoe'

# madgwick
VIRTUAL_EE_FRAME_ID = '/myo/madgwick/child_frame_id'
MADGWICK_INPUT_TOPIC = '/myo/madgwick/input_topic'
MADGWICK_TOPIC = '/myo/madgwick/topic'
MADGWICK_CHILD_FRAME_ID = VIRTUAL_EE_FRAME_ID

# panda robot
PANDA_BASE_LINK_NAME = 'base_link'
PANDA_BASE_LINK_FRAME_ID = '/base_link'
PANDA_EE_LINK_NAME = 'panda_hand'
PANDA_EE_LINK_FRAME_ID = '/panda_hand'
PANDA_EE_INIT_POSE = '/cmd_publisher/ee_init_pose'
PANDA_MANIPULATOR_INIT_JOINT_CONFIG = '/panda_manipulator_init_joint_config'

# gazebo
WORLD_NAME = 'world'
WORLD_FRAME_ID = '/world'
GAZEBO_HEALTHCHECK_READY = '/gazebo/healthcheck/ready'
GAZEBO_HEALTHCHECK_COOLDOWN = '/gazebo/healthcheck/cooldown'
GAZEBO_FUZZY_HEALTHCHECK_SERVICE = '/gazebo/healthcheck/service'                                    
# kinematic control
CMD_PUBLISHER_TOPIC = '/cmd_publisher/topic'
CMD_PUBLISHER_INPUT_TOPIC = '/cmd_publisher/input_topic'
CMD_EXECUTOR_GAIN_MATRIX = '/cmd_executor/gain_matrix'
CMD_EXECUTOR_CMD_TIMEOUT = '/cmd_executor/cmd_timeout'
CMD_EXECUTOR_MODE = '/cmd_executor/mode'
CMD_EXECUTOR_MOM_GAIN = '/cmd_executor/mom_gain'

# ekf
EKF_BASE_LINK_FRAME = '/rl_ekf_odom/base_link_frame'
EKF_ODOM_FRAME = '/rl_ekf_odom/odom_frame'
EKF_FREQ = '/rl_ekf_odom/frequency'
EKF_TOPIC = '/odometry/filtered'
EKF_TOPIC_MAP = '/odometry/filtered_map'
EKF_INPUT_IMU = '/ekf/input/imu/topic'
EKF_INPUT_IMU_FRAME_NAME = '/ekf/input/imu/frame_name'
EKF_INPUT_IMU_INPUT_TOPIC = '/ekf/input/imu/input_topic'
EKF_INPUT_TWIST = '/ekf/input/twist/topic'
EKF_INPUT_TWIST_FRAME_NAME = '/ekf/input/twist/frame_name'
EKF_INPUT_TWIST_FREQ = '/ekf/input/twist/freq'
EKF_INPUT_POSE = '/ekf/input/pose/topic'
EKF_INPUT_POSE_POSITION = '/ekf/input/pose/position/topic'
EKF_INPUT_POSE_ORIENTATION = '/ekf/input/pose/orientation/topic'
EKF_INPUT_POSE_PUBLISHER_FREQ = '/ekf/input/pose/freq'
EKF_INPUT_POSE_FIRST_MSG_PUBLISHED = '/ekf/input/pose/first_msg_published'
EKF_INPUT_POSE_FRAME_NAME = '/ekf/input/pose/frame_name'
EKF_TWIST_PUBLISHER_TOPIC = '/ekf/twist'

# moveit
MOVEIT_HEALTHCHECK_TOPIC = '/moveit/healthcheck/topic'
MOVEIT_HEALTHCHECK_COOLDOWN = '/moveit/healthcheck/cooldown'
MOVEIT_HEALTHCHECK_READY = '/moveit/healthcheck/ready'


# covariance estimates
EKF_INPUT_IMU_ORIENTATION_COV = np.array([3.73196953, -0.28590121,  0.29335513, -0.28590121,  0.22353207,  0.09624419, 0.29335513, 0.09624419, 3.22322569])
EKF_INPUT_IMU_STANCE_ANGULAR_VELOCITY_COV = np.array([1.0252437, -0.14743304,  0.02783716, -0.14743304,  0.91713598,  0.00936624, 0.02783716, 0.00936624, 0.33719414])
# EKF_INPUT_IMU_STANCE_LINEAR_ACCELERATION_COV = np.array([1.2472364 ,  0.07391321,  0.39253612, 0.07391321,  0.46773469, -0.2113925, 0.39253612, -0.2113925, 1.35093137])
EKF_INPUT_IMU_STANCE_LINEAR_ACCELERATION_COV = np.array([2.5,  0.07391321,  0.39253612, 0.07391321,  1.1, -0.2113925, 0.39253612, -0.2113925, 2.7])

PANDA_POSE_COV = np.zeros((6, 6))
PANDA_POSE_COV[[0, 1, 2], [0, 1, 2]] = np.array([0.183184, 0.183184, 0.3249])
PANDA_POSE_COV = PANDA_POSE_COV.flatten()

PANDA_TWIST_COV = np.zeros((6, 6))
PANDA_TWIST_COV[[0, 1, 2, 3, 4, 5], [0, 1, 2, 3, 4, 5]] = np.array([0.7225, 0.7225, 0.7225, 1.5625, 1.5625, 1.5625])
PANDA_TWIST_COV = PANDA_TWIST_COV.flatten()

EKF_INPUT_IMU_ORIENTATION_COV.setflags(write=False)
EKF_INPUT_IMU_STANCE_ANGULAR_VELOCITY_COV.setflags(write=False)
EKF_INPUT_IMU_STANCE_LINEAR_ACCELERATION_COV.setflags(write=False)
PANDA_POSE_COV.setflags(write=False)
PANDA_TWIST_COV.setflags(write=False)

# machine learning
ML_GRIPPER_MODE = '/ml/gripper_mode'
ML_TWIST_MODE = '/ml/twist_mode'
ML_TRAINED_MODEL_PATH = '/ml/trained_model_path'
ML_SAMPLE_WINDOW_SIZE = '/ml/sample_window_size'
ML_COOLDOWN_WINDOW_SIZE = '/ml/cooldown_window_size'
ML_REF_FORWARD_TWIST = '/ml/ref_forward_twist'
ML_GESTURE_TOPIC = '/ml/gesture_topic'

# franka gripper
GRIPPER_MIN_X = '/franka/gripper/min_x'
GRIPPER_MAX_X = '/franka/gripper/max_x'
GRIPPER_REF_V = '/franka/gripper/ref_v'
GRIPPER_COOLDOWN_INTERVAL = '/franka/gripper/cooldown_interval'
GRIPPER_DELTA_X = '/franka/gripper/delta_x'

# naive sensor fusion
NAIVE_SENSOR_FUSION_FREQ = '/naive_sensor_fusion/freq'
NAIVE_SENSOR_FUSION_INPUT_IMU_TOPIC = '/naive_sensor_fusion/input/imu/topic'
NAIVE_SENSOR_FUSION_INPUT_TWIST_TOPIC = '/naive_sensor_fusion/input/twist/topic'
NAIVE_SENSOR_FUSION_TOPIC = '/naive_sensor_fusion/topic'
NAIVE_SENSOR_FUSION_CHILD_FRAME_ID = '/naive_sensor_fusion/child_frame_id'

