import numpy as np
from stage import Stage
from environment import Environment

class Solver():
    """
    The solver class containing the stage, its environments, 
    and the methods to compute the trajectory."""
    def __init__(
        self,
        dt: float, # time step of solver
        stage: Stage,
        env: Environment
        ):
        self.dt = dt # the time step
        self.stage = stage # the vehicle
        self.env = env # physical environment of the vehicle
        
    def solve_trajectory_RK4(self) -> np.ndarray:
        """Solves the trajectory via a Runge Kutta 4 numerical integrator"""
        dt = self.dt # the timestep
        current_state = self.stage.states[-1] # start with the initial state vector
        if self.stage.dof_model == "3dof":
            f = self.model_3dof
        elif self.stage.dof_model == "5dof":
            f = self.model_5dof
        else:
            raise ValueError("Incorrect model specified. Please choose 3dof or 5dof")
        
        # RK-4
        zero_out_angles = False 
        while current_state[3] > 0:
            t = current_state[0] # the current time
            y = current_state[1:] # the current position, velocity, attitude data (5-dof)
            
            if current_state[3] < self.stage.h_deploy: # if vehicle hits 10km
                self.stage.deploy_parachute()
                if self.stage.dof_model == '5dof':
                    y[6:9] = 0
            
            # RK4 integration
            # outputs the derivatives of the initial state vector
            k1 = f(t, y) 
            k2 = f(t + dt/2, y + dt*k1/2)
            k3 = f(t + dt/2, y + dt*k2/2)
            k4 = f(t + dt, y + dt*k3)
            # new state vector
            new_y =  y + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)
            
            new_state = np.concatenate(([t + dt], new_y))
            self.stage.states.append(new_state)
            current_state = self.stage.states[-1] # set to the newly computed state vector
        
        self.stage.states = np.vstack(self.stage.states) # stacks nicely into one ndarray
        return self.stage, self.env
    
    def model_5dof(self, t, y):
        """
        The 5-DOF motion model
        y[0] = current x position
        y[1] = current y position
        y[2] = current z position
        y[3] = current x velocity
        y[4] = current y velocity
        y[5] = current z velocity
        y[6] = current x (pitch) angle
        y[7] = current y (yaw) angle
        y[8] = current x (pitch) angular velocity
        y[9] = current y (yaw) angular velocity
        """
        # determine the air density and wind speed at the current height
        rho = self.env.linearize_density(y[2])
        ws = self.env.linearize_wind(y[2])
        
        # compute the rotation matricies
        # rotation about the x-axis
        Rx = np.array([
            [1, 0, 0],
            [0, np.cos(y[6]), -np.sin(y[6])],
            [0, np.sin(y[6]), np.cos(y[6])]
        ])
        # rotation about the y-axis
        Ry = np.array([
            [np.cos(y[7]), 0, np.sin(y[7])],
            [0, 1, 0],
            [-np.sin(y[7]), 0, np.cos(y[7])]
        ])
        
        # current unit normal vector pointing along the cylinder exterior
        n = Rx @ Ry @ self.stage.nvec[-1] # current unit normal vector
        # Safe normalization of normal vector
        n_norm = np.linalg.norm(n)
        if n_norm < 1e-10:
            n = self.stage.nvec[-1]
        else:
            n = n / n_norm
        
        # Compute relative velocity with wind
        theta = np.deg2rad(self.env.theta_ws)
        v_rel_vec = np.array([
            y[3] - ws*np.cos(theta),
            y[4] - ws*np.sin(theta),
            y[5]
        ])
        
        # Compute normal component safely
        v_rel_dot_n = np.dot(v_rel_vec, n)
        
        # Prevent overflow in drag calculation
        if abs(v_rel_dot_n) > 1e3:
            v_rel_dot_n = np.sign(v_rel_dot_n) * 1e3
        
        # Compute drag force with safety checks
        if abs(v_rel_dot_n) < 1e-10:
            D = np.zeros(3)
        else:
            drag_mag = 0.5 * rho * v_rel_dot_n**2 * self.stage.Cd * self.stage.A
            # Limit maximum drag magnitude
            if drag_mag > 1e5:
                drag_mag = 1e5
            D = -drag_mag * n  # Use normalized n directly
        
        # Compute moments safely
        d = self.stage.moment_arm * n
        M = np.cross(d, D)
        
        # Safe division for accelerations
        alpha_x = np.divide(M[0], self.stage.Ixx, where=abs(self.stage.Ixx)>1e-10)
        alpha_y = np.divide(M[1], self.stage.Iyy, where=abs(self.stage.Iyy)>1e-10)
        ax = np.divide(D[0], self.stage.m, where=abs(self.stage.m)>1e-10)
        ay = np.divide(D[1], self.stage.m, where=abs(self.stage.m)>1e-10)
        az = -self.env.g0 - np.divide(D[2], self.stage.m, where=abs(self.stage.m)>1e-10)

        derivatives = np.array([
            y[3], # current x velocity (dx/dt)
            y[4], # current y velocity (dy/dt)
            y[5], # current z velocity (dz/dt)
            ax, # current x acceleration (dVx/dt)
            ay, # current y acceleration (dVy/dt)
            az, # current z acceleration (dVz/dt)
            y[8], # current x angular velocity (dtheta/dt)
            y[9], # current y angular velocity (dpsi/dt)
            alpha_x, # current x angular acceleration (domega_x/dt)
            alpha_y # current y angular acceleration (domega_y/dt)
        ])
        return derivatives
    
    def model_3dof(self, t, y):
        """
        The 3-DOF motion model
        y[0] = current x position
        y[1] = current y position
        y[2] = current z position
        y[3] = current x velocity
        y[4] = current y velocity
        y[5] = current z velocity
        """
        # determine the air density and wind speed at the current height
        rho = self.env.linearize_density(y[2])
        ws = self.env.linearize_wind(y[2])
        
        # get the current values of the environment
        rho = self.env.rho # air density, kg/m^3 
        theta = np.deg2rad(self.env.theta_ws) # angle of action of wind measured from x axis, ccw+, deg
        g0 = self.env.g0 # gravitational acceleration m/s^2
        
        # relative velocities for drag computation
        v_rel_x = y[3] - ws*np.cos(theta)
        v_rel_y = y[4] - ws*np.sin(theta)
        
        # magnitude of the velocities (including wind)
        v_rel_mag = np.sqrt(v_rel_x**2 + v_rel_y**2 + y[5]**2)
        
        # acceleration
        ax = -0.5 * rho * v_rel_mag * v_rel_x * self.stage.Cd * self.stage.A/self.stage.m # ax
        ay = -0.5 * rho * v_rel_mag * v_rel_y * self.stage.Cd * self.stage.A/self.stage.m # ay
        az = -g0 - (0.5 * rho * v_rel_mag * y[5] * self.stage.Cd *  self.stage.A/self.stage.m) # az

        derivatives = np.array([
            y[3], # current x velocity (dx/dt)
            y[4], # current y velocity (dy/dt)
            y[5], # current z velocity (dz/dt)
            ax, # current x acceleration (dVx/dt)
            ay, # current y acceleration (dVy/dt)
            az # current z acceleration (dVz/dt)
        ])
        return derivatives