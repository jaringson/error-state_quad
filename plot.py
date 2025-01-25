import pickle
import matplotlib.pyplot as plt
import numpy as np
# import param as P

from IPython.core.debugger import set_trace
from tqdm import tqdm

# plt.rcParams['text.usetex'] = True

# from os import listdir
# from os.path import isfile, join
#
# mypath = []
# onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]

plt.rcParams.update({'font.size': 20})
fig, axs = plt.subplots(4, 1)
# fig_x, axs_x = plt.subplots(2, 1)

# file_name = 'data/fpre.pickle'
file_name = 'data/lqr.pickle'

file = open(file_name, 'rb')
data = pickle.load(file)

# set_trace()

# set_trace()
axs[0].plot(data['t'], data['vx'], label='Trajectory')
axs[0].plot(data['t'], data['vxc'], label='Commanded')
axs[0].legend(loc='center right')
axs[0].set_ylabel(r'${v_x}$(m)')
axs[0].grid()
axs[0].tick_params(axis='x', which='both', bottom=False, labelbottom=False)

axs[1].plot(data['t'], data['vy'], label='Trajectory')
axs[1].plot(data['t'], data['vyc'], label='Commanded')
# axs[1].legend(loc='center right')
axs[1].set_ylabel(r'${v_y}$(m)')
axs[1].grid()
axs[1].tick_params(axis='x', which='both', bottom=False, labelbottom=False)

axs[2].plot(data['t'], data['vz'], label='Trajectory')
axs[2].plot(data['t'], data['vzc'], label='Commanded')
# axs[2].legend(loc='center right')
axs[2].set_ylabel(r'${v_z}$(m)')
axs[2].grid()
axs[2].tick_params(axis='x', which='both', bottom=False, labelbottom=False)
           
axs[3].plot(data['t'], data['J'])
# axs[3].legend(loc='center right')
axs[3].set_ylabel('J')
axs[3].set_xlabel('Time (s)')
axs[3].grid()

plt.show()
