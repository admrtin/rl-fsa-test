class Environment():
    """The environment class and methods for linearizing density and wind"""
    def __init__(self,
                 ws, # wind speed, m/s
                 theta_ws, # angle of wind on vehicle from x-axis, ccw+
                 rho, # air density, kg/m^3
                 g0 # gravitational acceleration, m/s^2
                 ):
        self.ws = ws
        self.theta_ws = theta_ws 
        self.rho = rho
        self.g0 = g0
    
    def linearize_density(self, h):
        """
        Linearizes the air density with respect to height
        """
        # Constants
        h0 = 0 # m
        rho0 = 1.2 # kg/m^3
        h1 = 60000 # m
        rho1 = 0.02 # kg/m^3
        
        # Linearize the density with respect to height
        # y = mx + b
        rho = (rho1-rho0)/(h1-h0) * h + rho0
            
        return rho
    
    def linearize_wind(self, h):
        """
        Linearizes the wind speed with respect to height
        """
        # Constants
        h0 = 0
        ws0 = self.ws
        h1 = 60000
        ws1 = 0
        # Linearize the wind speed with respect to height
        # y = mx + b
        ws = (ws1-ws0)/(h1-h0) * h + ws0
  
        return ws