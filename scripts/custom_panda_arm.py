from panda_robot import PandaArm
import numpy as np
from utils import cross



class CustomPandaArm:

    def __init__(self, panda_arm):
        self._arm = panda_arm
        self._n = 7

    
    @property
    def n(self):
        return self._n


    @property
    def panda_arm(self):
        return self._arm


    @property
    def jacobian(self):
        return self._arm.zero_jacobian()


    @property
    def q(self):
        return self._arm.angles()


    @property
    def ee_pose(self):
        ee_position, ee_orientation = self._arm.ee_pose()
        ee_orientation = np.array([ee_orientation.x, ee_orientation.y, ee_orientation.z, ee_orientation.w])
        return ee_position, ee_orientation


    def exec_velocity_cmd(self, qdot):
        self._arm.exec_velocity_cmd(qdot)

    
    def exec_position_cmd(self, q):
        self._arm.exec_position_cmd(q)


    def move_to_neutral(self):
        self._arm.move_to_neutral()

    
    def mom(self, J=None):
        """ Measure of Manipulability """
        if J is None:
            J = self.jacobian
        return np.linalg.det(J @ J.T) ** 0.5


    def hessian0(self, J=None):
        n = self.n
        
        if J is None:
            J = self.jacobian
        
        H = np.zeros((n, 6, n))

        for j in range(n):
            for i in range(j, n):
                H[j, :3, i] = cross(J[3:, j], J[:3, i])
                H[j, 3:, i] = cross(J[3:, j], J[3:, i])

                if i != j:
                    H[i, :3, j] = H[j, :3, i]

        return H


    def jacobm(self, J=None):
        if J is None:
            J = self.jacobian
        
        H = self.hessian0()
        
        J_JT = J @ J.T
        mom = np.linalg.det(J_JT) ** 0.5

        b = np.linalg.inv(J_JT)
        Jm = np.zeros((self.n ,1))

        for i in range(self.n):
            c = J @ H[i, :, :].T
            Jm[i, 0] = mom * (c.flatten('F') @ b.flatten('F')).T

        return Jm


