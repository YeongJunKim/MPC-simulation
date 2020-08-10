# paper:
# author: Yeong Jun Kim(colson)
# email: colson@korea.ac.kr || dud3722000@naver.com
# date: 2020-08-06
# Utilezed code: https://www.github.com/YeongJunKim/do-mpc

import numpy as np
import matplotlib.pyplot as plt
from casadi import *
from casadi.tools import *
import pdb
import sys
sys.path.append('../../')
import do_mpc

import matplotlib.pyplot as plt
import pickle
import time

from simulation_0_model import simulation_0_model
from simulation_0_mpc import simulation_0_mpc
from simulation_0_simulator import simulation_0_simulator


dt = 1
init = [0, 0, 0]
target = [15, 1, 0]
_u_con = [0.2, 2.0]
model = simulation_0_model(dt, target)
mpc = simulation_0_mpc(model, dt, _u_con)
simulator = simulation_0_simulator(model, dt)
estimator = do_mpc.estimator.StateFeedback(model)

x0 = simulator.x0

x0['p_x'] = init[0]
x0['p_y'] = init[1]
x0['p_theta'] = init[2]

mpc.x0 = x0

mpc.set_initial_guess()

fig, ax, graphics = do_mpc.graphics.default_plot(mpc.data)
plt.ion()


iteration = 150
for k in range(iteration):
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)
    print(mpc.x0['p_x'])
    print(mpc.x0['p_y'])
    print(mpc.x0['p_theta'])

    graphics.plot_results(t_ind=k)
    graphics.plot_predictions(t_ind=k)
    graphics.reset_axes()
    plt.show()
    plt.pause(0.1)

x = mpc.data._x
x_axis = x[:,0]
y_axis = x[:,1]
plt.plot(x_axis,y_axis)
plt.xlabel('x_axis')
plt.ylabel('y_axis')
plt.title('Trajectory')
plt.show()

fig2, ax2 = plt.subplots(3, sharex=True, figsize=(9,9))
ax2[0].plot(x[:,0])
ax2[1].plot(x[:,1])
ax2[2].plot(x[:,2])
ax2[0].set_xlabel('iteration')
ax2[1].set_xlabel('iteration')
ax2[2].set_xlabel('iteration')
ax2[0].set_ylabel('x_position')
ax2[1].set_ylabel('y_position')
ax2[2].set_ylabel('heading angle')
