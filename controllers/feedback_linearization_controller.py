import numpy as np
from models.manipulator_model import ManiuplatorModel
from .controller import Controller


class FeedbackLinearizationController(Controller):
    def __init__(self, Tp):
        self.model = ManiuplatorModel(Tp)
        self.Kp=34
        self.Kd=20
    def calculate_control(self, x, q_r, q_r_dot, q_r_ddot):
        """
        Please implement the feedback linearization using self.model (which you have to implement also),
        robot state x and desired control v.
        """
        v=q_r_ddot
        tau=self.model.M(x)@v+self.model.C(x)@q_r_dot
        q1, q2, q1_dot, q2_dot = x
        q = np.array([q1, q2])
        q_dot = np.array([q1_dot, q2_dot])
        v=q_r_ddot+self.Kd*(-q_dot+q_r_dot)+self.Kp*(-q+q_r)
        u = self.model.M(x) @ v + self.model.C(x) @ np.array([q1_dot, q2_dot])
        #return tau #zad2
        return u #zad8
