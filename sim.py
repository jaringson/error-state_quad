import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
from numpy.linalg import norm
from utils import roll, pitch, yaw

import params as P
from quadrotor import QuadDynamics

from dataPlotter import dataPlotter

# from quad_control_lqr import Controller
# from quad_control_pid import Controller
from quad_control_fpre import Controller


import json
import time
from tqdm import tqdm
from IPython.core.debugger import set_trace




# def run_sim():
quad = QuadDynamics(P)
control = Controller(P, quad)


dataPlot = dataPlotter()

pbar = tqdm(total = (P.sim_t - P.t_start)/P.dt)

t = P.t_start  # time starts at t_start
t_plot = P.t_plot
u = 0
fpreP = 0
j = 0

while t < P.sim_t:  # main simulation loop

    t_plot = t + P.t_plot

    # updates control and dynamics at faster simulation rate
    while t < t_plot:

        u, pos_c, J = control.computeControl(quad.state, t)

        y = quad.update(u)  # propagate system

        pbar.update(1)
        t = t + P.dt  # advance time by dt
    # update data plots
    dataPlot.update(t, quad.state, u, pos_c, J)

    # the pause causes the figure to display during simulation
    plt.pause(0.0001)
    # set_trace()
    # print(t)

# dataPlot.save('data/fpre.pickle')
# dataPlot.save('data/lqr.pickle')

pbar.close()
plt.waitforbuttonpress()

# out_file = open(filename, "w")
# json.dump(allStates, out_file, indent=3)
# out_file.close()
#
# out_file2 = open('data/kaldata.json', "w")
# json.dump(allKalStates, out_file2, indent=3)
# out_file2.close()
#
# out_file3 = open('data/kalcov.json', "w")
# json.dump(allKalCovariance, out_file3, indent=3)
# out_file3.close()
#
# out_file4 = open('data/kaltime.json', "w")
# json.dump(allKalTime, out_file4, indent=3)
# out_file4.close()


# if __name__ == "__main__":
#     run_sim(P.num_quads, P.collision_range, P.max_vel, 'data/change_vel_pi_2.json')
