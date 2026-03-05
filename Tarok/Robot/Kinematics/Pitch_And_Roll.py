# In this document the math for Pitch and Roll are implemented.
# See Master Project for the math derivations and for an illustration

from math import pi, cos, sin, atan
from .. import Tarok_Dymensions


def Inverse_Pitch(Theta_Pitch=None, Current_Foot_Positions=None):  
    """
    Inputs: 
    Theta_Pitch:    Pitch Angle 
    Current Foot Positions in the world frame (x,y,z)
    
    Outputs:
    New Foot Positions in the world frame (x,y,z)
    Shoulder_Heights:     4, vector with z values.
    
    The Foots are numbered as follows:
    Front Left: 0
    Front Right: 1
    Hind Left: 2
    Hind Right: 3    
    """
    ### Values that is needed for the calculations
    L_Body = 0.3277 * 2 
    "W_Body = 0.07298 * 2       NOT USED"
    Theta_Pitch = 15
    ###
    
    New_Foot_Positions = [[None,None,None],
                          [None,None,None],
                          [None,None,None],
                          [None,None,None]]

    Shoulder_Heights =    [None,None,None,None]
    
    # Calculating the new foot positions for each leg for a loop
    # But first lets do it for the front left leg
    
    for i in range(4):
    
        # Extrating Foot Positions from the Current_Foot_Positions
        " x = abs(Current_Foot_Positions[i][0])   NOT USED"
        z = abs(Current_Foot_Positions[i][2])
        
        x_dist  = L_Body * cos(Theta_Pitch)     # Calculate the x distance
        dz      = L_Body * sin(Theta_Pitch)     # Calcualte the Change in z
        dx      = L_Body - x_dist               # Calculate the change in x
        
        # Now calculating the actuator highs for the hind and front
        # &
        # Now settting a sign of the dx. Since the dx values is measured from point under pitched actuator, 
        # to point under actuator, when Torso are in level  (DANSK: In level = I vater)
        
        if i == 0 or i == 1:    # Calculations for front
            Shoulder_Height = z + dz
            dx = -dx
            Shoulder_Angle = atan(dx/Shoulder_Height)
        else:                    # For Hind legs
            Shoulder_Height = z - dz    
            dx = dx
            Shoulder_Angle = atan(dx/Shoulder_Height)
        
        # Saving the Shoulder Heights, to be used for roll later    
        Shoulder_Heights[i] =  Shoulder_Height  
        
        # Now Calcualting the Leg Length
        Leg_Length = Shoulder_Height/cos(Shoulder_Angle)
        
        # Shoulder angle 2
        Shoulder_Angle_2 =  Theta_Pitch + Shoulder_Angle
        
        x_new = Leg_Length * cos(Shoulder_Angle_2)
        z_new = Leg_Length * sin(Shoulder_Angle_2)
        
        # Then Given the new x for either hind or front legs
        if i == 0 or i == 1:
            x_new = - x_new
        else: 
            x_new = x_new
            
        # Now making the new foot positions
        New_Foot_Positions[i][0] = x_new + Current_Foot_Positions[i][0]
        New_Foot_Positions[i][1] = Current_Foot_Positions[i][1]
        New_Foot_Positions[i][2] = - z_new
        
        return New_Foot_Positions, Shoulder_Heights

def Inverse_Roll(Phi_roll=None,Current_Foot_Positions=None,Shoulder_Heights=None):
    """
    Inputs: 
    Phi_Roll:    Roll Angle 
    Current Foot Positions in the world frame (x,y,z)
    
    Outputs:
    New Foot Positions in the world frame (x,y,z)
    
    The Foots are numbered as follows:
    Front Left: 0
    Front Right: 1
    Hind Left: 2
    Hind Right: 3    
    """
    
    # Values used in this Function
    W_Body = 0.07298 * 2
    Phi_Roll = 10
            
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
            Hip_Angle = atan(dy/Hip_Height) 
        else:
            Hip_Height = Shoulder_Heights[i] - dz
            dy = dy
            Hip_Angle = atan(dy/Hip_Height)
        
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
    
    print("Hello")

