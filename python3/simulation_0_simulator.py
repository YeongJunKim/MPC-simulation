import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import do_mpc


def simulation_0_simulator(model, dt):



    simulator = do_mpc.simulator.Simulator(model)

    params_simulator = {
        't_step': dt
    }

    simulator.set_param(**params_simulator)

    simulator.setup()

    return simulator

