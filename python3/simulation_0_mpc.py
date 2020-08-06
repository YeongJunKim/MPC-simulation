import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import do_mpc


def simulation_0_mpc(model, dt , _u_con):
    """

    :param model:
    :param dt:
    :return:
    """

    mpc = do_mpc.controller.MPC(model)

    setup_mpc = {
        'n_horizon': 10,
        'n_robust': 1,
        'open_loop': 0,
        't_step': dt,
        'state_discretization': 'discrete',
        'store_full_solution': True,
    }

    mpc.set_param(**setup_mpc)

    _x = model.x
    _u = model.u
    _p = model.p
    _z = model.z

    mterm = model.aux['2a']
    iterm = model.aux['2b']

    mpc.set_objective(mterm=mterm, lterm=iterm)
    mpc.set_rterm(v_l=0, v_a=0)

    max_v_l = _u_con[0]
    max_v_a = _u_con[1]

    mpc.bounds['lower','_u','v_l'] = -max_v_l
    mpc.bounds['lower','_u','v_a'] = -max_v_a
    mpc.bounds['upper','_u','v_l'] = max_v_l
    mpc.bounds['upper','_u','v_a'] = max_v_a

    mpc.setup()

    return mpc
