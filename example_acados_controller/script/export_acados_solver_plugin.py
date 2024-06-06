#!/usr/bin/env python3

# Copyright 2024 ICUBE Laboratory, University of Strasbourg
# License: Apache License, Version 2.0
# Author: Thibault Poignonec (tpoignonec@unistra.fr)

import sys

from acados_solver_plugins import SolverPluginGenerator

from acados_template import AcadosOcp
import casadi as ca
import numpy as np  # noqa: F401

from rrbot_model import RRBotModel


def export_acados_ocp() -> AcadosOcp:
    # default state and control bounds
    tau_1_max = 25.0  # Nm
    tau_2_max = 10.0  # Nm
    q_dot_max = np.pi/4  # rad/s
    q_1_bounds = (- np.pi, 0.0)
    q_2_bounds = (-np.pi / 2, np.pi / 2)

    # setup base robot model
    rrbot_model = RRBotModel()
    model = rrbot_model.export_acados_model()

    # declare extra parameters
    sym_p_ref = ca.SX.sym('p_ref', 2)
    sym_p_dot_ref = ca.SX.sym('p_dot_ref', 2)
    sym_Q_pos_diag = ca.SX.sym('Q_pos_diag', 2)
    sym_Q_vel_diag = ca.SX.sym('Q_vel_diag', 2)
    sym_R_diag = ca.SX.sym('R_diag', 2)

    model.p = ca.vertcat(
        model.p,         # original model parameters (see rrbot_model.py)
        sym_p_ref,       # reference position
        sym_p_dot_ref,   # reference velocity
        sym_Q_pos_diag,  # weight for position error
        sym_Q_vel_diag,  # weight for velocity error
        sym_R_diag,      # weight for control (torques)
    )

    # setup OCP
    ocp = AcadosOcp()
    ocp.model = model
    Ts = 0.01
    N = 10
    Tf = N * Ts

    # set dimensions
    ocp.dims.N = N

    # set default parameters
    ocp.parameter_values = np.zeros((model.p.shape[0],))

    # set cost
    Q_err_p = ca.diag(sym_Q_pos_diag)
    Q_err_p_dot = ca.diag(sym_Q_vel_diag)
    R = ca.diag(sym_R_diag)

    err_p = sym_p_ref - rrbot_model.sym_p
    err_p_dot = sym_p_dot_ref - rrbot_model.sym_p_dot

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = \
        err_p.T @ Q_err_p @ err_p \
        + err_p_dot.T @ Q_err_p_dot @ err_p_dot \
        + rrbot_model.sym_tau.T @ R @ rrbot_model.sym_tau

    # Note: the terminal cost should be chosen more carefully in practice.
    # This is not very rigorous, but enough for the purpose of this example.
    ocp.model.cost_expr_ext_cost_e = \
        err_p.T @ Q_err_p @ err_p \
        + err_p_dot.T @ Q_err_p_dot @ err_p_dot

    # set constraints
    x0 = np.zeros((model.x.shape[0],))
    ocp.constraints.x0 = x0  # placeholder initial state

    # control constraints
    ocp.constraints.idxbu = np.array([0, 1])
    ocp.constraints.lbu = np.array([- tau_1_max, - tau_2_max])
    ocp.constraints.ubu = np.array([+ tau_1_max, + tau_2_max])

    # state constraints
    ocp.constraints.lbx = np.array(
        [q_1_bounds[0], q_2_bounds[0], -q_dot_max, -q_dot_max])
    ocp.constraints.ubx = np.array(
        [q_1_bounds[1], q_2_bounds[1], +q_dot_max, +q_dot_max])
    ocp.constraints.idxbx = np.array(range(model.x.shape[0]))

    # terminal (state) constraints
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx
    ocp.constraints.idxbx_e = ocp.constraints.idxbx

    # set solver options
    ocp.solver_options.tf = Tf
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.qp_solver_cond_N = int(N/4)
    ocp.solver_options.hpipm_mode = 'BALANCE'  # 'SPEED'  # 'ROBUST'
    ocp.solver_options.qp_solver_warm_start = 2  # 0 = None, 1 = warm, 2 =hot
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.integrator_type = 'IRK'

    return ocp


def main() -> int:
    # Create ocp
    acados_ocp = export_acados_ocp()

    # Define the index maps
    x_index_map = {
        'q': [0, 1],
        'q_dot': [2, 3],
    }
    z_index_map = {
        'p': [0, 1],
    }
    p_index_map = {
        'l0': [0],
        'l1': [1],
        'l2': [2],
        'm1': [3],
        'm2': [4],
        'p_ref': [5, 6],
        'p_dot_ref': [7, 8],
        'Q_pos_diag': [9, 10],
        'Q_vel_diag': [11, 12],
        'R_diag': [13, 14],
    }
    u_index_map = {
        'tau': [0, 1],
    }

    # Instantiate plugin generator
    solver_plugin_generator = SolverPluginGenerator()

    solver_plugin_generator.generate_solver_plugin(
        acados_ocp,
        plugin_class_name='RrbotCartesianTracking',
        solver_description='Acados solver plugin to track a cartesian trajectory with the RRBot planar robot',  # noqa: E501
        x_index_map=x_index_map,
        z_index_map=z_index_map,
        p_index_map=p_index_map,
        u_index_map=u_index_map,
    )
    return 0


if __name__ == '__main__':
    sys.exit(main())
