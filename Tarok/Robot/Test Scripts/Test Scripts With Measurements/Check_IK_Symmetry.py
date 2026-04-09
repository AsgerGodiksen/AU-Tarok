# Check_IK_Symmetry.py
# Run on the RPi (or anywhere with the Robot package).
# No CAN, no motors — just prints the IK angles to check for J2/J3 swap.
#
# Place in: Tarok/Robot/Test Scripts/Test Scripts With Measurements/
# Run:      python Check_IK_Symmetry.py

import sys, os
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..')))

import numpy as np
from Robot import *

# ── Same parameters as Stand_Pose_Torque_3.py ──
l_k = 0.7048
w_k = 0.220

x_FL = x_FR =  l_k / 2
x_HL = x_HR = -l_k / 2
y_FL = y_HL =  w_k / 2 + 0.078
y_FR = y_HR = -w_k / 2 - 0.078
z = -0.41

P_FL_base = T0_B(np.array([x_FL, y_FL, z]).reshape((3, 1)), 'FL')
P_FR_base = T0_B(np.array([x_FR, y_FR, z]).reshape((3, 1)), 'FR')
P_HL_base = T0_B(np.array([x_HL, y_HL, z]).reshape((3, 1)), 'HL')
P_HR_base = T0_B(np.array([x_HR, y_HR, z]).reshape((3, 1)), 'HR')

Theta_FL = np.degrees(Inverse_Kinematics(P_FL_base, 'FL'))
Theta_FR = np.degrees(Inverse_Kinematics(P_FR_base, 'FR'))
Theta_HL = np.degrees(Inverse_Kinematics(P_HL_base, 'HL'))
Theta_HR = np.degrees(Inverse_Kinematics(P_HR_base, 'HR'))

# ── Print raw angles ──
print("=" * 60)
print(f"{'Leg':<6} {'J1 (hip)':>10} {'J2 (shoulder)':>14} {'J3 (knee)':>12}")
print("-" * 60)
for name, th in [("FL", Theta_FL), ("FR", Theta_FR),
                 ("HL", Theta_HL), ("HR", Theta_HR)]:
    print(f"{name:<6} {th[0]:>10.3f} {th[1]:>14.3f} {th[2]:>12.3f}")

# ── Symmetry comparison ──
pairs = [("FL vs FR", Theta_FL, Theta_FR),
         ("HL vs HR", Theta_HL, Theta_HR)]

print("\n" + "=" * 60)
print("SYMMETRY — |left| vs |right| per joint index")
print("-" * 60)
for label, L, R in pairs:
    print(f"\n  {label}:")
    for j in range(3):
        print(f"    J{j+1}:  |L| = {abs(L[j]):7.3f}   |R| = {abs(R[j]):7.3f}   diff = {abs(abs(L[j]) - abs(R[j])):7.3f}")

# ── The key check: does right J2 look like left J3? ──
print("\n" + "=" * 60)
print("SWAP CHECK — normal vs swapped J2/J3 match")
print("-" * 60)
for label, L, R in pairs:
    match_normal  = abs(abs(L[1]) - abs(R[1])) + abs(abs(L[2]) - abs(R[2]))
    match_swapped = abs(abs(L[1]) - abs(R[2])) + abs(abs(L[2]) - abs(R[1]))
    print(f"\n  {label}:")
    print(f"    Normal  (L_J2↔R_J2, L_J3↔R_J3): total diff = {match_normal:.3f}")
    print(f"    Swapped (L_J2↔R_J3, L_J3↔R_J2): total diff = {match_swapped:.3f}")
    if match_swapped < match_normal:
        print("    >>> SWAPPED match is closer — J2/J3 likely mixed in IK or transforms!")
    else:
        print("    >>> Normal match is closer — IK output order looks correct.")

print()