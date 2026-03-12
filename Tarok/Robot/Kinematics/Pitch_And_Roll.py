# In this document the math for Pitch and Roll are implemented.
# See Master Project for the math derivations and for an illustration

from math import pi, cos, sin, atan

L_Body = 0.3277 * 2
W_Body = 0.0729 * 2

def Inverse_Pitch(Theta_Pitch=None, Current_Foot_Positions=None):  
    """
    Inputs:
    Theta_Pitch:              Pitch angle in RADIANS
    Current_Foot_Positions:   4x3 list with current foot positions [x, y, z] in world frame

    Outputs:
    New_Foot_Positions:  4x3 list with updated foot positions [x, y, z]
    Shoulder_Heights:    List of 4 shoulder heights (z values), passed to Inverse_Roll

    Leg numbering:
        Front Left  : 0
        Front Right : 1
        Hind  Left  : 2
        Hind  Right : 3
    """
    
    New_Foot_Positions = [[None,None,None],
                          [None,None,None],
                          [None,None,None],
                          [None,None,None]]

    Shoulder_Heights =    [None,None,None,None]
    
    # Calculating the new foot positions for each leg for a loop
    # But first lets do it for the front left leg
    
    for i in range(4):
    
        # Extrating Foot Positions from the Current_Foot_Positions
        z = abs(Current_Foot_Positions[i][2])
        
        # Apply pitch sign per leg group, matching IMU_control convention
        if i == 0 or i == 1:   # Front legs
            pitch_signed = -Theta_Pitch
        else:                   # Hind legs
            pitch_signed = Theta_Pitch
        
        x_dist  = L_Body * cos(pitch_signed)     # Calculate the x distance
        dz      = L_Body * sin(pitch_signed)     # Calcualte the Change in z
        dx      = L_Body - x_dist               # Calculate the change in x
        
        
        Shoulder_Height = z + dz
        Shoulder_Heights[i] = Shoulder_Height

        if i == 0 or i == 1:   # Front legs: dx is negative
            dx_signed = -dx
        else:                   # Hind legs: dx is positive
            dx_signed = dx
            
        Shoulder_Angle = atan(dx_signed / Shoulder_Height)    
        
        if i == 0 or i == 1:
            Shoulder_Angle_signed = -Shoulder_Angle
        else:
            Shoulder_Angle_signed = Shoulder_Angle     
        
         # Leg length and new angles
        Leg_Length      = Shoulder_Height / cos(Shoulder_Angle_signed)
        Shoulder_Angle_2 = pitch_signed + Shoulder_Angle_signed

        z_new = Leg_Length * cos(Shoulder_Angle_2)
        x_new = Leg_Length * sin(Shoulder_Angle_2)

        if i == 0 or i == 1:
            x_new_signed = -x_new
        else:
            x_new_signed = x_new

        # Save new foot positions
        New_Foot_Positions[i][0] = x_new_signed + Current_Foot_Positions[i][0]
        New_Foot_Positions[i][1] = Current_Foot_Positions[i][1]
        New_Foot_Positions[i][2] = -z_new
        
    return New_Foot_Positions, Shoulder_Heights

def Inverse_Roll(Phi_Roll=None,Current_Foot_Positions=None,Shoulder_Heights=None):
    """
    Inputs:
    Phi_Roll:                 Roll angle in RADIANS
    Current_Foot_Positions:   4x3 list with current foot positions [x, y, z]
    Shoulder_Heights:         List of 4 shoulder heights from Inverse_Pitch

    Outputs:
    New_Foot_Positions:  4x3 list with updated foot positions [x, y, z]

    Leg numbering:
        Front Left  : 0
        Front Right : 1
        Hind  Left  : 2
        Hind  Right : 3
    """
           
    # Initiating the Array
    New_Foot_Positions = [[None,None,None],
                          [None,None,None],
                          [None,None,None],
                          [None,None,None]]
        
    for i in range(4):
        y = abs(Current_Foot_Positions[i][1])
        
        dz      = W_Body * sin(Phi_Roll)
        y_dist  = W_Body * cos(Phi_Roll)
        dy = y - y_dist 
        
        if i == 0 or i == 2:    # These are for the left legs
            Hip_Height = Shoulder_Heights[i] + dz
            dy = - dy

        else:
            Hip_Height = Shoulder_Heights[i] - dz
            dy = dy
        
        Hip_Angle   = atan(dy / Hip_Height)
        Leg_Length = Hip_Height/cos(Hip_Angle)
        Hip_Angle_2 = Phi_Roll + Hip_Angle
        
        z_new = Leg_Length * cos(Hip_Angle_2)
        y_new = Leg_Length * sin(Hip_Angle_2)
        
        if i == 0 or i == 2:
            y_final = - W_Body + y_new
        else:
            y_final = W_Body + y_new
            
        # Now Makring the new Foot Positions
        New_Foot_Positions[i][0] = Current_Foot_Positions[i][0]
        New_Foot_Positions[i][1] = y_final
        New_Foot_Positions[i][2] = - z_new  
            
    return New_Foot_Positions
    
    
if __name__ == "__main__":
    # Test with same initial foot positions as IMU_control.py
    shoulder_link = 0.078
    FR_Position = [ L_Body, -W_Body - shoulder_link, -0.4]
    FL_Position = [ L_Body,  W_Body + shoulder_link, -0.4]
    HR_Position = [-L_Body, -W_Body - shoulder_link, -0.4]
    HL_Position = [-L_Body,  W_Body + shoulder_link, -0.4]
    foot_positions = [FR_Position, FL_Position, HR_Position, HL_Position]

    # Test Inverse_Pitch with 10 degrees pitch
    test_pitch =0 * pi / 180
    new_positions, shoulder_heights = Inverse_Pitch(test_pitch, foot_positions)
    print("=== Inverse Pitch Test (10 deg) ===")
    for i, pos in enumerate(new_positions):
        print(f"  Leg {i}: {[round(v, 4) for v in pos]}")
    print(f"  Shoulder Heights: {[round(h, 4) for h in shoulder_heights]}")

    # Test Inverse_Roll with 10 degrees roll
    test_roll = 0 * pi / 180
    new_positions_roll = Inverse_Roll(test_roll, new_positions, shoulder_heights)
    print("\n=== Inverse Roll Test (10 deg) ===")
    for i, pos in enumerate(new_positions_roll):
        print(f"  Leg {i}: {[round(v, 4) for v in pos]}")