##### Real-Time IMU Pitch & Roll #####
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../Hardware')))

import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

from Robot.Hardware.IMU_BNO085 import IMU_Initialization, Get_Quaternion, Quaternion_To_Euler
from Robot.Kinematics.Constant_Transforms import*


################## PARAMETERS ######################
# Parameters for the live feed plot
WINDOW_S = 10.0
PLOT_DT  = 0.05
MAXLEN   = int(WINDOW_S / PLOT_DT)

################## BUFFERS ######################
# Here we save the data points, for the 'WINDOW_S' seconds so we can see it on the screen
t_buf     = deque(maxlen=MAXLEN)
pitch_buf = deque(maxlen=MAXLEN)
roll_buf  = deque(maxlen=MAXLEN)

################## FIGURE ######################
fig, (ax_pitch, ax_roll) = plt.subplots(1, 2, figsize=(12, 4))
fig.suptitle('TAROK — Real-Time IMU Pitch & Roll', fontsize=14, fontweight='bold')
plt.subplots_adjust(left=0.07, right=0.97, top=0.88, bottom=0.12, wspace=0.30)

line_pitch, = ax_pitch.plot([], [], color='black',  lw=1.5, label='Pitch')
line_roll,  = ax_roll.plot([],  [], color='purple', lw=1.5, label='Roll')

for ax, title in [(ax_pitch, 'Pitch (deg)'), (ax_roll, 'Roll (deg)')]:
    ax.set_title(title)
    ax.set_xlabel('Time (s)')
    ax.set_ylabel('deg')
    ax.axhline(0, color='gray', lw=0.8, ls='--')
    ax.grid(True, ls='--', alpha=0.5)
    ax.legend(fontsize=8)

################## IMU INIT ######################
print("Initialising IMU...")
bno, i2c = IMU_Initialization()
print("IMU ready. Starting plot...")

t_start = time.time()








################## ANIMATION ######################
def update(_frame):
    try:
        quat  = Get_Quaternion(bno)
        
        print("Rotated")
        euler = Quaternion_To_Euler(quat)

    except Exception:
        return []

    now = time.time() - t_start
    t_buf.append(now)
    pitch_buf.append(euler[0])
    roll_buf.append(euler[1])

    t_arr = np.array(t_buf)
    t_min = t_arr[-1] - WINDOW_S

    for ax, line, buf in [(ax_pitch, line_pitch, pitch_buf),
                          (ax_roll,  line_roll,  roll_buf)]:
        line.set_data(t_arr, np.array(buf))
        ax.set_xlim(t_min, t_arr[-1] + 0.1)
        ax.relim()
        ax.autoscale_view(scalex=False, scaley=True)

    return []

ani = animation.FuncAnimation(fig, update, interval=int(PLOT_DT * 1000),
                               blit=False, cache_frame_data=False)

try:
    plt.show()
except KeyboardInterrupt:
    pass
finally:
    i2c.deinit()
    print("IMU disconnected.")