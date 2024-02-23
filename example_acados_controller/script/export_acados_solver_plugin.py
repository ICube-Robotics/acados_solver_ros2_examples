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
    # setup model
    rrbot_model = RRBotModel()
    model = rrbot_model.export_acados_model()

    # add extra model parameters
    sym_p_ref = ca.SX.sym('p_ref', 2)
    sym_p_dot_ref = ca.SX.sym('p_dot_ref', 2)
    model.p = ca.vertcat(model.p, sym_p_ref, sym_p_dot_ref)

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
    Q_err_p = 100 * np.eye(2)
    Q_err_p_dot = 10 * np.eye(2)
    R = 1 * np.eye(2)

    err_p = sym_p_ref - rrbot_model.sym_p
    err_p_dot = sym_p_ref - rrbot_model.sym_p_dot

    ocp.cost.cost_type = 'EXTERNAL'
    ocp.cost.cost_type_e = 'EXTERNAL'
    ocp.model.cost_expr_ext_cost = \
        err_p.T @ Q_err_p @ err_p \
        + err_p_dot.T @ Q_err_p_dot @ err_p_dot \
        + rrbot_model.sym_tau @ R @ rrbot_model.sym_tau
    ocp.model.cost_expr_ext_cost_e = \
        err_p.T @ Q_err_p @ err_p \
        + err_p_dot.T @ Q_err_p_dot @ err_p_dot
    # Note: the terminal cost should be chosen more carefully in practice.
    # This is not very rigorous, but enough for the purpose of this example.

    # set constraints
    x0 = np.zeros((model.x.shape[0],))
    ocp.constraints.x0 = x0  # placeholder initial state

    tau_max = 2.0
    ocp.constraints.lbu = np.array([-tau_max]*2)
    ocp.constraints.ubu = np.array([+tau_max]*2)
    ocp.constraints.idxbu = np.array([0, 1])

    q_min = - np.pi / 2
    q_max = + np.pi
    q_dot_max = np.pi  # rad/s

    ocp.constraints.lbx = np.array([q_min, q_min, -q_dot_max, -q_dot_max])
    ocp.constraints.ubx = np.array([q_max, q_max, q_dot_max, q_dot_max])
    ocp.constraints.idxbx = np.array(range(model.x.shape[0]))
    ocp.constraints.lbx_e = ocp.constraints.lbx
    ocp.constraints.ubx_e = ocp.constraints.ubx
    ocp.constraints.idxbx_e = ocp.constraints.idxbx

    # set options
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'EXACT'
    ocp.solver_options.integrator_type = 'IRK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI'
    ocp.solver_options.qp_solver_cond_N = int(N/4)

    # set prediction horizon
    ocp.solver_options.tf = Tf

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
        'p_dot': [2, 3],
    }
    p_index_map = {
        'p_ref': [0, 1],
        'p_dot_ref': [2, 3],
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
