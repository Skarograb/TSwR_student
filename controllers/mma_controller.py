import numpy as np
from .controller import Controller
from models.manipulator_model import ManiuplatorModel


class MMAController(Controller):
    def __init__(self, Tp):
        # TODO: Fill the list self.models with 3 models of 2DOF manipulators with different m3 and r3
        # I:   m3=0.1,  r3=0.05
        # II:  m3=0.01, r3=0.01
        # III: m3=1.0,  r3=0.3
        self.Tp = Tp
        self.models = [ManiuplatorModel(Tp, m3=0.1, r3=0.05), ManiuplatorModel(Tp, m3=0.01, r3=0.01),
                       ManiuplatorModel(Tp, m3=1.0, r3=0.3)]
        self.i = 0
        self.Kp = 35
        self.Kd = 28
        self.u=np.zeros((2,1))
    def choose_model(self, x):
        # TODO: Implement procedure of choosing the best fitting model from self.models (by setting self.i)
        q1, q2, q1_dot, q2_dot = x
        error=None
        index=0
        for inx, model in enumerate(self.models):
            y = model.M(x) @ self.u + model.C(x)@[[q1_dot],[q2_dot]]
            curr_error = np.sum(np.abs([[q1], [q2]] - y))
            if error is None:
                error=curr_error
                index=inx
            elif error>curr_error:
                error=curr_error
                index=inx
            self.i=index


    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        self.choose_model(x)
        q = x[:2]
        q_dot = x[2:]
        #v = q_r_ddot # TODO: add feedback

        v = q_r_ddot + self.Kd * (-q_dot + q_r_dot) + self.Kp * (-q + q_r)
        M = self.models[self.i].M(x)
        C = self.models[self.i].C(x)
        u = M @ v[:, np.newaxis] + C @ q_dot[:, np.newaxis]
        return u
