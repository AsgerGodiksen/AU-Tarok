# Imports
import sys
import os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../..')))


from Robot import*

import time
import matplotlib.animation as animation
import numpy as np

Tarok = TarokDymensions

# ─────────────────────────────────────────────
#  Parameters
# ─────────────────────────────────────────────

L1 = Tarok.L1
L2 = Tarok.L2
L3 = Tarok.L3

LEGS = Tarok.LEGS
COLORS = Tarok.COLORS

# Phase offsets for a crawl gait (fraction of full cycle, one leg at a time)
# FL=0, HR=0.25, FR=0.5, HL=0.75
PHASE_OFFSET = {
    'FL': 0.00,
    'HR': 0.25,
    'FR': 0.50,
    'HL': 0.75,
}


# ─────────────────────────────────────────────
#  Parameters - For Time related things
# ─────────────────────────────────────────────

Swing_Time_Scalar = 3.0        # [s] swing phase duration
Swing_Time_Steps = 300  # Number of Steps in the Swing

Stand_Time_Scalar  = 3 * Swing_Time_Scalar # [s] stand phase duration
Stand_Time_Steps = 3 * Swing_Time_Steps



# ─────────────────────────────────────────────
#  Building the Trajectories for the Legs
# ─────────────────────────────────────────────

print("Building trajectories...")
trajectories = {}
thetas       = {}

for leg in LEGS:
    Position_shifted, Velocity_shifted, _ = Building_Bezier_Trajectories(
                                    Swing_Time_Scalar,
                                    Stand_Time_Scalar,
                                    Swing_Time_Steps,
                                    Stand_Time_Steps,
                                    PHASE_OFFSET,
                                    leg)
    
    trajectories[leg] = Position_shifted
    
# ─────────────────────────────────────────────
#  Computing the Inverse Kinematics
# ─────────────────────────────────────────────     

Theta = np.zeros(Swing_Time_Steps + Stand_Time_Steps,3)

for i in range(len(Theta)):
    Inverse_Kinematics(Position_shifted, Leg):