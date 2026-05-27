import numpy as np




X = np.array([-0.200, -0.2805, -0.300, -0.300, -0.300, 0, 0, 0, 0.3032, 0.3032, 0.2826, 0.200]) # [m]
Y = np.array([0.500,  0.500,   0.3611, 0.3611, 0.3611, 0.3611, 0.3611, 0.3214, 0.3214, 0.3214, 0.500, 0.500]) # [m]
Scaling_Factor_X = 0.15
Scaling_Factor_Y = 0.4
Offset = 0.24



X_scaled = X * Scaling_Factor_X
Y_scaled = Y * Scaling_Factor_Y + Offset


print("Scaled X:", X_scaled)
print("Scaled Y:", Y_scaled)