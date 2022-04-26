# Imports
import os
import sys
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '.')))

from rocket import *
from controller import Controller
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import logging
import threading
from math import floor

logging.basicConfig(level=logging.INFO)

dt = 0.05
Tfinal = 20.0
t_step = Tfinal/dt
frames = int(t_step)

# Main Thread
def main():
    rocket = Rocket(dt)

    fig1 = plt.figure()
    ax = fig1.add_subplot(projection='3d')

    rocket._states = np.array([0, 0, -1000, 0, 0, 0, 0, 0, 0, 0, 0, 0])
    rocket._deltas = np.array([0, 0, 0])

    def update_vertices(i):
        vr, simplices = rocket.get_vertices()
        pos = rocket.get_pos()
        ax.cla()

        ax.set_title('UAV Simulation') 
        ax.set_xlabel('East Axis (m)') 
        ax.set_ylabel('North Axis (m)')
        ax.set_zlabel('Down Axis (m)')

        AXIS_LIMIT = 80

        ax.set_xlim3d(pos[0] - AXIS_LIMIT, pos[0] + AXIS_LIMIT)
        ax.set_ylim3d(pos[1] - AXIS_LIMIT, pos[1] + AXIS_LIMIT)
        ax.set_zlim3d(pos[2] - AXIS_LIMIT, pos[2] + AXIS_LIMIT)    
        #plane._update_state_array()

        ax.plot3D(vr[:, 0], vr[:, 1], vr[:, 2], 'g-') 
        
        
        # for i in simplices:
        #     i = np.append(i, i[0])
        #     ax.plot3D(vr[i, 0], vr[i, 1], vr[i, 2], 'r-')

    def animate_rocket():
        anim = FuncAnimation(fig1, update_vertices, blit=False, frames=int(t_step), interval=int(dt*1000))
        plt.show()
    
    commander = Controller(rocket)

    COMMAND_DELAY = 0.05

    x1 = threading.Thread(target=commander.cmd_loop, args=(COMMAND_DELAY,), daemon=True)
    logging.info('CMD thread initialized')
    x1.start()
    
    animate_rocket()
    
    data = rocket.get_state_array()
    titles = [
        'Pn',
        'Pe',
        'Pd',
        'u',
        'v',
        'w',
        'phi',
        'theta',
        'psi',
        'p',
        'q',
        'r'
        ]

    cols = 3
    fig2, ax = plt.subplots(floor(len(data)/cols), cols)
    for i in range(floor(len(data)/cols)):
        for j in range(cols):
            ax[i, j].plot(data[i*3 + j])
            ax[i, j].set_title(titles[i*3 + j])

    plt.tight_layout()
    plt.show()

# LONG explanation, but this is the clearest and best way to define the main function
if __name__ == "__main__":
    main()