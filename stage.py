import matplotlib.pyplot as plt
import numpy as np

class Stage():
    """
    The stage class containing initial inputs, the computed state vectors,
    the parachute deployment method, and the initialization of the rigid 
    body method (if 5-dof)
    """
    def __init__(self,
                 m:float, # if 5dof this is overriden by volume-computed mass
                 Cd:float,
                 A:float,
                 h_deploy:float,
                 xyz0:list[float],
                 v_xyz0:list[float],
                 dof_model:str = "3dof",
                 ):
        
        # defaults
        self.dof_model = dof_model # either "3dof" or "5dof"
        self.m = m # mass, overriden if 5dof model (rigid body mass computation), kg
        self.Cd = Cd # setting initial drag coefficient, unitless
        self.A = A # setting area to initial reference area, m^2
        self.h_deploy = h_deploy # parachute deployment height
        self.is_parachute_deployed = False
        
        # 5dof variables (None when 3dof)
        self.r_outer = None # outer radius, m
        self.thickness = None # wall thickness, m
        self.r_inner = None # inner radius, m
        self.height = None # height of cylinder, m
        self.cg_location = None # center of gravity location, m
        self.cp_location = None # center of pressure location, m
        self.moment_arm = None # moment arm (r_cp-r_cg),m
        self.volume = None # volume of cylinder, m^3
        self.material_density = None # kg/m^3
        self.Ixx = None # moment of inertia about the x axis, kg*m^2
        self.Iyy = None # moment of inertia about the y axis, kg*m^2
        self.theta = None # initial vehicle pitch angle about (x-axis), deg
        self.psi = None # initial vehicle yaw angle (about y-axis), deg
        self.theta_dot = None # initial vehicle pitch rate, deg/s
        self.psi_dot = None # initial vehicle yaw rate, deg/s
        self.nvec = None # unit vector pointing normal to the cylinder body
        
        # states = list of state vectors as ndarrays, to be appended to after each integration step
        # state format changes between 3dof and 5dof
        if self.dof_model == "3dof": 
            self.states = [np.concatenate((
                np.array([0.0]),
                np.array(xyz0),
                np.array(v_xyz0))
                )]
        elif self.dof_model == "5dof":
            # determines the 5dof initial state variables
            self.include_rigid_body()
            self.states = [np.concatenate((
                np.array([0.0]),
                np.array(xyz0),
                np.array(v_xyz0),
                np.array([self.theta, self.psi]),
                np.array([self.theta_dot, self.psi_dot])
                ))]
    
    def deploy_parachute(self):
        self.Cd = 2.0 # parachute drag coefficient, unitless
        self.A = 18 # area of parachute, m^2
        self.is_parachute_deployed = True
        
    def include_rigid_body(self):
        # geometry, mass properties of the hollow cylinder vehicle (bonus question)
        self.r_outer = 0.6 # outer radius, m
        self.thickness = 0.008 # wall thickness, m
        self.r_inner = self.r_outer - self.thickness # inner radius, m
        self.height = 12 # height of cylinder, m
        self.cg_location = 2.0 # center of gravity location, m
        self.cp_location = 6.0 # center of pressure location, m
        self.moment_arm = self.cp_location - self.cg_location
        self.volume = np.pi * (self.r_outer**2 - self.r_inner**2) * self.height
        self.material_density = 1940 # kg/m^3
        self.m = self.material_density * self.volume # initial mass of the vehicle, kg
        self.Ixx = 1/12 * self.m * (self.height**2 + 2*(self.r_outer**2 + self.r_inner**2))
        self.Iyy = self.Ixx
        
        # assume the vehicle attitude is random at the start
        # I chose these values based on my own testing of what converged
        self.theta = np.deg2rad(np.random.uniform(-1.0,1.0)) # initial vehicle pitch angle about (x-axis), deg
        self.psi = np.deg2rad(np.random.uniform(-1.0, 1.0)) # initial vehicle yaw angle (about y-axis), deg
        self.theta_dot = np.deg2rad(np.random.uniform(-0.1,0.1)) # initial pitch velocity
        self.psi_dot = np.deg2rad(np.random.uniform(-0.1,0.1)) # initial yaw velocity 
        
        # compute initial unit normal vector pointing along the cylinder body
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(self.theta), -np.sin(self.theta)],
            [0, np.sin(self.theta), np.cos(self.theta)]
        ])
        
        Ry = np.array([
            [np.cos(self.psi), 0, np.sin(self.psi)],
            [0, 1, 0],
            [-np.sin(self.psi), 0, np.cos(self.psi)]
        ])
        
        # vertical normal vector (pointing into cylinder body)
        n_vert = np.array([1.0, 0.0, 0.0])
        
        # Rotate initial normal vector by pitch and yaw
        n = Rx @ Ry @ n_vert
        n = n / np.linalg.norm(n)  # ensure unit vector
        
        # Store initial rotated normal vector
        self.nvec = [n]