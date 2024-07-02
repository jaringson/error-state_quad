import numpy as np
from numpy.linalg import norm, inv
from copy import copy as cp
from utils import roll, pitch, yaw, exp, rota, rotp, from_euler, R_v_to_v1, R_v_to_b
from utils import skew, boxminus, from_two_unit_vectors

class CommandState:
    def __init__(self, params):

        self.gravity_ = params.gravity
        self.e3 = params.e3
        self.x0 = params.fig8x0
        self.dt = params.dt

        self.max_thrust_ = params.max_thrust
        self.mass_ = params.mass
        self.inertia_matrix_ = params.inertia_matrix

        self.fig8_period = params.fig8_period
        self.fig8_x_width = params.fig8_x_width
        self.fig8_y_width = params.fig8_y_width
        self.fig8_z_width = params.fig8_z_width


        self.angular_drag_ = params.angular_drag
        # self.linear_drag_ = params.linear_drag

    def get_p8(self, t):
        freq = self.fig8_period / (2.0 * np.pi)
        pos = self.x0 + np.array([[self.fig8_x_width * np.sin(t * freq)],
                                  [self.fig8_y_width * np.sin(2.0 * t * freq)],
                                  [self.fig8_z_width * np.sin(t * freq)]])
        return pos

    def get_pdot8(self, t):
        freq = self.fig8_period / (2.0 * np.pi)
        vel = np.array([[self.fig8_x_width * freq * np.cos(t * freq)],
                        [self.fig8_y_width * 2.0 * freq * np.cos(2.0 * t * freq)],
                        [self.fig8_z_width * freq * np.cos(t * freq)]])
        return vel

    def get_pddot8(self, t):
        freq = self.fig8_period / (2.0 * np.pi)
        acc = np.array([[-self.fig8_x_width * freq**2 * np.sin(t * freq)],
                        [-self.fig8_y_width * 4.0 * freq**2 * np.sin(2.0 * t * freq)],
                        [-self.fig8_z_width * freq**2 * np.sin(t * freq)]])
        return acc

    def get_commanded_state(self, state, t):

        x_ = cp(state[0:3])
        v_ = cp(state[3:6])
        q_ = cp(state[6:10])
        omega_ = cp(state[10:13])


        pos_c = self.get_p8(t)
        # pos_c = np.array([[1.0],[0.0],[0.0]])
        # pos_c = np.ones((3,1))

        pddot_I = self.get_pddot8(t)
        a_I = self.gravity_*self.e3 - pddot_I
        quat_c = from_two_unit_vectors(self.e3, a_I/norm(a_I))
        # quat_c = np.array([[1.0],[0.0],[0.0],[0.0]])

        v_c = rotp(q_, self.get_pdot8(t))
        # v_c = np.zeros((3,1))

        a_I_prev = self.gravity_*self.e3 - self.get_pddot8(t - self.dt)
        a_I_next = self.gravity_*self.e3 - self.get_pddot8(t + self.dt)
        q_prev = from_two_unit_vectors(self.e3, a_I_prev/norm(a_I_prev))
        q_next = from_two_unit_vectors(self.e3, a_I_next/norm(a_I_next))
        omega_c = boxminus(q_next, q_prev) / (2.0*self.dt)
        # omega_c = np.zeros((3,1))

        ## ----------------------------------- ##
        # u_s = norm(a_I) * self.mass_ / self.max_thrust_
        u_s = a_I[2,0] * self.mass_ / self.max_thrust_
        # u_s =

        u_tau = self.angular_drag_@omega_c + skew(omega_c)*self.inertia_matrix_@omega_c
        # u_tau = self.inertia_matrix_@omega_c + skew(omega_c)*self.inertia_matrix_@omega_c

        # set_trace()

        return pos_c, v_c, quat_c, omega_c, u_s, u_tau
