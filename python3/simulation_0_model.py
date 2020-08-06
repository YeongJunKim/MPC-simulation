# paper:
# author: Yeong Jun Kim(colson)
# email: colson@korea.ac.kr || dud3722000@naver.com
# date: 2020-08-06
# Utilezed code: https://www.github.com/YeongJunKim/do-mpc
# Using mobile robot's non holonomic model.

import numpy as np
from casadi import *
from casadi.tools import *
import pdb
import sys
import do_mpc


def simulation_0_model(dt,target):
    model_type = 'discrete'
    model = do_mpc.model.Model(model_type)

    # Certain parameters
    alpha = 0.7
    beta = 0.3


    # States struct (optimization variables):
    p_x =           model.set_variable('_x', 'p_x')
    p_y =           model.set_variable('_x', 'p_y')
    p_theta =       model.set_variable('_x', 'p_theta')

    # Input struct (optimzation variables):
    v_l =           model.set_variable('_u', 'v_l')
    v_a =           model.set_variable('_u', 'v_a')

    t_x =          target[0]
    t_y =          target[1]
    t_theta =      target[2]

    model.set_expression(expr_name='2a', expr=alpha * ((p_x-t_x) * (p_x-t_x) + (p_y-t_y) * (p_y-t_y) + (p_theta - t_theta) * (p_theta - t_theta)))
    model.set_expression(expr_name='2b', expr=beta * (v_l * v_l + v_a * v_a))

    model.set_rhs('p_x', p_x + v_l * cos(p_theta) * dt)
    model.set_rhs('p_y', p_y + v_l * sin(p_theta) * dt)
    model.set_rhs('p_theta', p_theta + v_a * dt)

    model.setup()

    return model
