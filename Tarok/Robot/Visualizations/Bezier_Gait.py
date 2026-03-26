# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from Robot import*

import time
import matplotlib.animation as animation

Tarok = TarokDymensions

# ─────────────────────────────────────────────
#  Parameters
# ─────────────────────────────────────────────

L1 = Tarok.L1
L2 = Tarok.L2
L3 = Tarok.L3

Swing_Time = 3.0        # [s] swing phase duration
Swing_Time_Steps = 300  # Number of Steps in the Swing








Stand_Time  = 3 * Swing_Time # [s] stand phase duration
Stand_Time_Steps = 3 * Swing_Time_Steps





FL_c_k = Bezier_Control_Points()

t_Bezier, Trajectory_Swing, Velocity_Swing = Bezier_Time_Parameters()








