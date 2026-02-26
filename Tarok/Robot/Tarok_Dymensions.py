# In this File the dimensions of TAROK is found 

class Tarok_Dimensions_Class:
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
        self.K_W = 0.220   # [m] Width  Between Actuator 1 rotation axis in left and right legs

        
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

        

        