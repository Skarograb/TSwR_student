import numpy as np
from mpmath.functions.rszeta import z_half

from observers.eso import ESO
from .controller import Controller


class ADRCJointController(Controller):
    def __init__(self, b, kp, kd, p, q0, Tp):
        self.b = b
        self.kp = kp
        self.kd = kd

        A = np.array([
            [0, 1, 0],
            [0, 0, 1],
            [0, 0, 0]
        ])
        B = np.array([[0], [self.b], [0]])
        L = np.array([[3 * p], [3 * p ** 2], [p ** 3]])
        W = np.array([[1, 0, 0]])
        self.eso = ESO(A, B, W, L, q0, Tp)

    def set_b(self, b):
        ### TODO update self.b and B in ESO
        #return NotImplementedError
        self.b = b
        self.eso.set_B(np.array([[0], [self.b], [0]]))

    def calculate_control(self, x, q_d, q_d_dot, q_d_ddot):
        ### TODO implement ADRC
        #return NotImplementedError

        q_hat, q_hat_dot, f_hat = self.eso.get_state()
        q, q_dot=x
        e = q_d - q
        e_dot = q_d_dot - q_hat_dot
        v = self.kp * e + self.kd * e_dot + q_d_ddot
        u = (v - f_hat) / self.b
        self.eso.update(q,u)
        return u
