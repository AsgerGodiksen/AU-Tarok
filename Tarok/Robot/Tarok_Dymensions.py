# In this File the dimensions of TAROK is found 

import numpy as np

class TarokDymensions:
    """ 
    Tarok Dynemsions
    
    Methods
    -------
    __init__():
        Gives Dymensions over the Physical Robot

    Shoulder_Positions():
        Gives Positions of the Shoulders in the Body frame
        
    Initial_Foot_Positions():
        Gives the inital Foot Positions that are just below the actuators
    """
    
    L1 = 0.078  
    L2 = 0.2
    L3 = 0.3
    
    # Leg Names
    LEGS = ['FL', 'FR', 'HL', 'HR']
    
    # Colors for plots and animations for the legs
    COLORS = {'FL': 'blue', 'FR': 'red', 'HL': 'green', 'HR': 'orange'}
    
    # Leg Offsets for Walking where Front Left is the inital used. 
    # Then Hind Right Leg Trajectory can be shifted with 25% and so on.
    CRAWL_OFFSETS= {
    'FL': 0.00,
    'HR': 0.25,
    'FR': 0.50,
    'HL': 0.75,}
    

    def __init__(self):
        """ Lengths of the different parts of the robot in meters"""
        self.Torso_Lenght = 0.479   # m - Torso Lenght
        self.Torso_Width = 0.3179   # m - Torso Width
        self.Torso_Height = 0.09919 # m - Torso Height
        self.Upper_Leg = 0.2        # m - Upper leg length
        self.Lower_Leg = 0.3        # m - Lower leg length
        self.L1 = 0.078             # m - Lenght of First segment in meters
        self.Foot_Radius = 0.02625  # [m] Radius of the Foot 
        

        """ Kinematics Lengths"""
        self.K_L = 0.7048  # [m] Lenght Between Actuator 2 rotation axis in front and hind legs 
        self.K_W = 0.220   # [m] Width  Between Actuator 1 rotation axis in left and right legs[ L_BODY/2,  W_BODY/2 + L1,  STAND_Z]
        self.Standing_Heights = 0.41

        
        """ Weights of the different parts of the robot in kg"""
        # Torso Parts
        self.Beam        = 0.5 # kg
        self.Motor_Mount = 0.2 # kg
        self.Torso = self.Beam * 4 + self.Motor_Mount * 2 # kg

        # Leg Parts
        self.Actuator       = 0.565 # kg
        self.Hip_Mount      = 0.1   # kg
        self.Hip_Knee       = 0.1   # kg
        self.Upper_Leg      = 0.3   # kg
        self.Axel_Connector = 0.1   # kg
        self.Axel_Slicer    = 0.1   # kg
        self.Leg_Rod        = 0.1   # kg
        self.Lower_Leg      = 0.2   # kg
        self.Foot           = 0.1   # kg    

        # Metal Parts
        self.Metal_Part = 0.5 # kg 

        self.Leg = 3 * self.Actuator + self.Hip_Mount + self.Hip_Knee + self.Upper_Leg + self.Axel_Connector + self.Axel_Slicer + self.Leg_Rod + self.Lower_Leg + self.Foot + self.Metal_Part # kg 

        # Total Weight of the Robot
        self.Tarok_Weight = self.Torso + 4 * self.Leg # kg
        
    def Shoulder_Positions(self):
        """ 
        Gives the Shoulder Positions for the Torso
        """
        Front_Left_Shoulder     = np.array([[ self.K_L/2], [ self.K_W/2 + self.L1], [0]])
        Front_Right_Shoulder    = np.array([[ self.K_L/2], [-self.K_W/2 - self.L1], [0]])
        Hind_Left_Shoulder      = np.array([[-self.K_L/2], [ self.K_W/2 + self.L1], [0]])
        Hind_Right_Shoulder     = np.array([[-self.K_L/2], [-self.K_W/2 - self.L1], [0]])
        
        return [Front_Left_Shoulder, Front_Right_Shoulder, Hind_Left_Shoulder, Hind_Right_Shoulder]
        
    def Initial_Foot_Positions(self):
        """ 
        Function From Tarok_Dymensions
              
            The Initial Foot Positions - The End effector 
        Returns:
            Returns the initial foot positions as (3,1) numpy arrays
            [Front_Left_Foot, Front_Right_Foot, Hind_Left_Foot, Hind_Right_Foot]
        """
        Front_Left_Foot  = np.array([[ self.K_L/2], [ self.K_W/2 + self.L1], [-self.Standing_Heights]])
        Front_Right_Foot = np.array([[ self.K_L/2], [-self.K_W/2 - self.L1], [-self.Standing_Heights]])
        Hind_Left_Foot   = np.array([[-self.K_L/2], [ self.K_W/2 + self.L1], [-self.Standing_Heights]])
        Hind_Right_Foot  = np.array([[-self.K_L/2], [-self.K_W/2 - self.L1], [-self.Standing_Heights]])
        
        return [Front_Left_Foot, Front_Right_Foot, Hind_Left_Foot, Hind_Right_Foot]