# Copyright 2024 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

from acados_template import AcadosModel
import casadi as ca
from casadi import cos, sin, vertcat
from casadi import SX
import numpy as np  # noqa: F401


class RRBotModel:
    def __init__(self):
        # Constants
        # ------------
        # gravity constant [m/s^2]
        self.gravity = 9.81

        # Casadi symbols
        # --------------
        self.sym_q = SX.sym('q', 2, 1)          # Joint position
        self.sym_q_dot = SX.sym('q_dot', 2, 1)  # Joint velocity
        self.sym_tau = SX.sym('tau', 2, 1)      # Joint torques  (controls)
        self.sym_algebraic_p = SX.sym('p_algebraic', 2, 1)          # Cart. position

        self.sym_l1 = SX.sym('l1', 1)  # length of the first link [m] : default = 1.0
        self.sym_l2 = SX.sym('l2', 1)  # length of the second link [m] : default = 1.0
        self.sym_m1 = SX.sym('m1', 1)  # mass of the first link [Kg] : default = 1.0
        self.sym_m2 = SX.sym('m2', 1)  # mass of the second link [Kg] : default = 1.0

        # CoG of the first link [m]
        self.sym_lc1 = self.sym_l1 / 2.
        # Inertia of the first link [Kg*m^2]
        self.sym_i1 = self.sym_m1 / 12. * self.sym_l1 * self.sym_l1
        # CoG of the second link [m]
        self.sym_lc2 = self.sym_l2 / 2.
        # Inertia of the second link [Kg*m^2]
        self.sym_i2 = self.sym_m2 / 12. * self.sym_l2 * self.sym_l2

        # Expressions
        # ------------
        B = SX.zeros(2, 2)
        B[0, 0] = self.sym_m1 * self.sym_lc1 * self.sym_lc1 + self.sym_m2 * (
            self.sym_l1 * self.sym_l1 + self.sym_lc2 * self.sym_lc2
            + 2 * self.sym_l1 * self.sym_lc2 * cos(self.sym_q[1])) + self.sym_i1 + self.sym_i2
        B[0, 1] = self.sym_m2 * (
                self.sym_lc2 * self.sym_lc2 + self.sym_l1 * self.sym_lc2 * cos(self.sym_q[1])
            ) + self.sym_i2
        B[1, 0] = B[0, 1]
        B[1, 1] = self.sym_m2 * self.sym_lc2 * self.sym_lc2 + self.sym_i2

        C_times_q_dot = SX.zeros(2, 1)
        C_times_q_dot[0] = (
            -self.sym_m2 * self.sym_l1 * self.sym_lc2
            * self.sym_q_dot[1] * sin(self.sym_q[1])) \
            * self.sym_q_dot[0] + (
                -self.sym_m2 * self.sym_l1 * self.sym_lc2
                * (self.sym_q_dot[0] + self.sym_q_dot[1]) * sin(self.sym_q[1])
            ) * self.sym_q_dot[1]
        C_times_q_dot[1] = (
                self.sym_m2 * self.sym_l1 * self.sym_lc2 * self.sym_q_dot[0]
                * sin(self.sym_q[1])) * self.sym_q_dot[0]

        G = SX.zeros(2, 1)
        G[0] = (self.sym_m1 * self.sym_lc1 * cos(self.sym_q[0])
                + self.sym_m2 * (self.sym_l1 * cos(self.sym_q[0])
                + self.sym_lc2 * cos(self.sym_q[0] + self.sym_q[1]))
                ) * self.gravity
        G[1] = self.sym_m2 * self.sym_lc2 * cos(
            self.sym_q[0] + self.sym_q[1]) * self.gravity

        # Forward dynamics: q_dot2 = f_dyn(q, q_dot, tau)
        self.expr_forward_dynamics = \
            ca.inv(B) @ (self.sym_tau - C_times_q_dot - G)

        # Forward kinematics: p = fk(q) and p_dot = J(q) * q_dot
        self.expr_fk = vertcat(
            self.sym_l1 * cos(self.sym_q[0])
            + self.sym_l2 * cos(self.sym_q[0] + self.sym_q[1]),
            self.sym_l1 * sin(self.sym_q[0])
            + self.sym_l2 * sin(self.sym_q[0] + self.sym_q[1])
        )
        self.expr_J = ca.jacobian(self.expr_fk, self.sym_q)  # Auto. diff

        self.sym_p = self.expr_fk
        self.sym_p_dot = self.expr_J @ self.sym_q_dot

        # Store joint inertia
        self.joint_inertia_matrix = B

    def export_acados_model(self) -> AcadosModel:
        # state & control variables
        x = vertcat(self.sym_q, self.sym_q_dot)
        xdot = SX.sym('x_dot', x.shape[0], 1)
        u = vertcat(self.sym_tau)

        # algebraic state variables
        z = vertcat(self.sym_algebraic_p)

        # dynamics (ODE) of the form: x_dot = ODE(x, u)
        function_ODE = vertcat(
            self.sym_q_dot,
            self.expr_forward_dynamics  # joint acc. from torques
        )

        # full DAE model (i.e., implicit)
        f_impl = vertcat(
            xdot - function_ODE,
            self.sym_algebraic_p - self.sym_p,  # for debugging
        )

        # Instantiate Acados model object and return
        model = AcadosModel()
        model.f_impl_expr = f_impl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.z = z
        model.p = vertcat(
            self.sym_l1,
            self.sym_l2,
            self.sym_m1,
            self.sym_m2
        )
        model.name = 'rrbot'

        return model
