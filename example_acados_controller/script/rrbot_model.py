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

        # mass of the first link [Kg]
        self.m1 = 1.0
        # length of the first link [m]
        self.l1 = 1.0
        # CoG of the first link [m]
        self.lc1 = self.l1/2
        # Inertia of the first link [Kg*m^2]
        self.i1 = self.m1 / 12 * self.l1 * self.l1

        # mass of the second link [Kg]
        self.m2 = 1.0
        # length of the second link [m]
        self.l2 = 1.0
        # CoG of the second link [m]
        self.lc2 = self.l2 / 2
        # Inertia of the second link [Kg*m^2]
        self.i2 = self.m2 / 12 * self.l2 * self.l2

        # Casadi symbols
        # --------------
        self.sym_q = SX.sym('q', 2)          # Joint position
        self.sym_q_dot = SX.sym('q_dot', 2)  # Joint velocity
        self.sym_tau = SX.sym('tau', 2)      # Joint torques  (controls)
        self.sym_p = SX.sym('p', 2)          # Cart. position
        self.sym_p_dot = SX.sym('p_dot', 2)  # Cart. velocity

        # Expressions
        # ------------
        B = SX.zeros(2, 2)
        B[0, 0] = self.m1 * self.lc1 * self.lc1 + self.m2 * (
            self.l1 * self.l1 + self.lc2 * self.lc2
            + 2 * self.l1 * self.lc2 * cos(self.sym_q[1])) + self.i1 + self.i2
        B[0, 1] = self.m2 * (
                self.lc2 * self.lc2 + self.l1 * self.lc2 * cos(self.sym_q[1])
            ) + self.i2
        B[1, 0] = B[0, 1]
        B[1, 1] = self.m2 * self.lc2 * self.lc2 + self.i2

        C_times_q_dot = SX.zeros(2, 1)
        C_times_q_dot[0] = (
            -self.m2 * self.l1 * self.lc2
            * self.sym_q_dot[1] * sin(self.sym_q[1])) \
            * self.sym_q_dot[0] + (
                -self.m2 * self.l1 * self.lc2
                * (self.sym_q_dot[0] + self.sym_q_dot[1]) * sin(self.sym_q[1])
            ) * self.sym_q_dot[1]
        C_times_q_dot[1] = (
                self.m2 * self.l1 * self.lc2 * self.sym_q_dot[0]
                * sin(self.sym_q[1])) * self.sym_q_dot[0]

        G = SX.zeros(2, 1)
        G[0] = (self.m1 * self.lc1 * cos(self.sym_q[0])
                + self.m2 * (self.l1 * cos(self.sym_q[0])
                + self.lc2 * cos(self.sym_q[0] + self.sym_q[1]))
                ) * self.gravity
        G[1] = self.m2 * self.lc2 * cos(
            self.sym_q[0] + self.sym_q[1]) * self.gravity

        # Forward dynamics: q_dot2 = f_dyn(q, q_dot, tau)
        self.expr_forward_dynamics = vertcat(
            B.inverse() * (self.sym_tau - C_times_q_dot - G)
        )

        # Forward kinematics: p = fk(q)
        self.expr_fk = vertcat(
            self.l1 * cos(self.sym_q[0])
            + self.l2 * cos(self.sym_q[0] + self.sym_q[1]),
            self.l1 * sin(self.sym_q[0])
            + self.l2 * sin(self.sym_q[0] + self.sym_q[1])
        )
        # Diff. kinematics
        self.expr_J = ca.jacobian(self.expr_fk, self.sym_q)  # Auto. diff

        # Store joint inertia
        self.joint_inertia_matrix = B

    def export_acados_model(self) -> AcadosModel:
        # state & control variables
        x = vertcat(self.sym_q, self.sym_q_dot)
        xdot = SX.sym('x_dot', x.shape[0], 1)
        u = vertcat(self.sym_tau)

        # algebraic state variables
        z = vertcat(self.sym_p, self.sym_p_dot)

        # dynamics (ODE) of the form: x_dot = ODE(x, u)
        function_ODE = vertcat(
            self.sym_q_dot,
            self.expr_forward_dynamics  # joint acc. from torques
        )

        # full DAE model (i.e., implicit)
        f_impl = vertcat(
            xdot - function_ODE,
            self.sym_p - self.expr_fk,
            self.sym_p_dot - self.expr_J @ self.sym_q_dot
        )

        # Instantiate Acados model object and return
        model = AcadosModel()
        model.f_impl_expr = f_impl
        model.x = x
        model.xdot = xdot
        model.u = u
        model.z = z
        model.p = vertcat([])  # unused at this point
        model.name = 'rrbot'

        return model
