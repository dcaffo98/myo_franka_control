import numpy as np
import tf



cos = np.cos
sin = np.sin

cos2 = lambda x: np.cos(x) ** 2
sin2 = lambda x: np.sin(x) ** 2



def T_gamma_inv(_, beta, gamma):
    out = np.empty(9)
    
    C_beta = cos(beta)
    C_gamma = cos(gamma)
    S_beta = sin(beta)
    S_gamma = sin(gamma)

    den1 = C_beta * S_gamma ** 2 + C_beta * C_gamma ** 2

    out[0] = C_gamma / den1
    out[1] = S_gamma / den1
    out[2] = 0

    out[3] = -S_gamma
    out[4] = C_gamma
    out[5] = 0

    out[6] = S_beta * C_gamma / den1
    out[7] = S_beta * S_gamma / den1
    out[8] = 1

    return out.reshape(3, 3)


def rpy2omega(alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot):               
    omega_x = cos(beta) * cos(gamma) * alpha_dot - sin(gamma) * beta_dot
    omega_y = cos(beta) * sin(gamma) * alpha_dot + cos(gamma) * beta_dot
    omega_z = -np.sin(beta) * alpha_dot + gamma_dot                              
    return np.array([omega_x, omega_y, omega_z])   


def rotate_rpy_velocity_(alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot):
    omega = rpy2omega(alpha, beta, gamma, alpha_dot, beta_dot, gamma_dot)
    R = tf.transformations.euler_matrix(alpha, beta, gamma)
    rotated_omega = R[:3, :3] @ omega
    # alpha, beta, gamma = tf.transformations.euler_from_matrix(R)
    return T_gamma_inv(alpha, beta, gamma) @ rotated_omega


def rotate_rpy_velocity(quat, alpha_dot, beta_dot, gamma_dot):
    return rotate_rpy_velocity_(*tf.transformations.euler_from_quaternion(quat), alpha_dot, beta_dot, gamma_dot)



def debug_motion(tgt_twist, panda_arm):
    T_A_inv = np.eye(6)
    
    while True:
        J = panda_arm.zero_jacobian()
        _, ee_orientation_q = panda_arm.ee_pose()
        ee_orientation_rpy = tf.transformations.euler_from_quaternion(np.array([ee_orientation_q.x, ee_orientation_q.y, ee_orientation_q.z, ee_orientation_q.w]))
        T_A_inv[3:, 3:] = T_gamma_inv(*ee_orientation_rpy)
        J_A_inv = np.linalg.pinv(T_A_inv @ J)

        q_dot = J_A_inv @ tgt_twist
        panda_arm.exec_velocity_cmd(q_dot)

