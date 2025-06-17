import numpy as np


from observers.eso import ESO
from .adrc_joint_controller import ADRCJointController
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class ADRFLController(Controller):
    def __init__(self, Tp, q0, Kp, Kd, p):
        self.model = ManiuplatorModel(Tp)
        self.Kp = Kp
        self.Kd = Kd
        self.L = np.array([[3 * p[0], 0], [0, 3 * p[1]], [3 * p[0] ** 2, 0], [0, 3 * p[1] ** 2], [p[0] ** 3, 0], [0, p[1] ** 3]])
        A = np.zeros((6, 6))
        B = np.zeros((6, 2))
        W = np.zeros((2, 6))
        A[np.arange(4), np.arange(2, 6)] = 1
        W[0, 0] = 1
        W[1, 1] = 1

        self.eso = ESO(A, B, W, self.L, q0, Tp)
        self.update_params(q0[:2], q0[2:])

    def update_params(self, q, q_dot):
        ### TODO Implement procedure to set eso.A and eso.B
        x = np.concatenate([q, q_dot], axis=0)
        M = self.model.M(x)
        C = self.model.C(x)
        M_inv = np.linalg.inv(M)

        A=np.zeros((6,6))
        A[0:2,2:4]=np.eye(2)
        A[2:4, 4:6] = np.eye(2)
        A[2:4, 2:4] = -M_inv @ C
        B=np.zeros((6,2))
        print(-M_inv@C)
        B[2:4, :] = M_inv

        self.eso.A = A
        self.eso.B = B

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement centralized ADRFLC
        #return NotImplementedError
        M = self.model.M(x)
        C = self.model.C(x)
        q1, q2, q1_dot, q2_dot = x
        q = np.array([q1, q2])
        q_hat, q_hat_dot, f_hat = np.split(self.eso.get_state(),[2,4])



        # 61 equation
        v = self.Kp @ (q_d - q) + self.Kd @ (q_d_dot - q_hat_dot) + q_d_ddot
        u = M @ (v - f_hat) + C @ q_hat_dot

        self.update_params(q_hat, q_hat_dot)
        self.eso.update(q[:,None], u[:,None])
        return u
