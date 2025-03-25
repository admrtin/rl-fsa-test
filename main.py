"""
Adam Martin
Flight Analysis Engineer I/II Interview
Started 03/15/2024 10:15AM MST, Completed 03/17/2024 1AM MST

Completed all of the parts and bonus points questions except for unit testing (ran out of time)

For the 5-DOF part 1 and 2 simulations, assumed:
- the initial omega_x, omega_y rotation angles were random: U(-1, 1)
- the initial omega_x_dot, omega_y_dot angular velocities were random U(-.1, .1)

If I had extra time, I would have implemented the following:
- "SimSpec" class that provides one interface for all possible input configurations
- "StateVector" class to write state vectors to the solver in a concise manner
- More thorough documentation & comments across the codebase
- unittesting across the board
- Input error checking across the board
- Time-adaptive numerical integrator
"""
import os
import time
from solver import Solver
from stage import Stage
from environment import Environment
from plotting import plot_trajectory, plot_trajectories
from trajectory_tools import parallel_solve_trajectories

def part_1_3dof(delta_t:float, plot_dir:str):
    """
    Part #1 (Reentry Simulation, no randomness, 3DOF)
    """
    print("-------------------------------------------------")
    print("Part #1 (Reentry Simulation, no randomness, 3DOF)")
    print("(Linearized air density and wind shear gradient)")
    print("-------------------------------------------------")
    # establish the initial inputs
    m = 700 # kg
    Cd = 1.4 # unitless
    A = 1.14 # m^2
    h_deploy = 10000 # parachute deployment height, m
    ws =  15 # m/s
    theta_ws = 270 # deg
    rho = 1.82 # kg/m^3
    g0 = 9.81 # m/s^2
    xyz0 = [0.0, 0.0, 60000.0] # initial position vector, m
    v_xyz0 = [70, 70, -280] # initial velocity vector, m/s
    
    # create a stage object (contains the states of the vehicle)
    stage = Stage(m, Cd, A, h_deploy, xyz0, v_xyz0, dof_model='3dof')
    # create an environment object (atmospheric properties, wind model, gravity model, etc.)
    env = Environment(ws, theta_ws, rho, g0)
    # create a solver object
    solver = Solver(delta_t, stage, env)
    print("Computing trajectory for part 1 of the test...")
    stage, _ = solver.solve_trajectory_RK4()
    print("Trajectory is computed.")
    del solver
    plot_trajectory(stage, plot_dir)
    print()

def part_1_5dof(delta_t, plot_dir:str):
    """
    Part #1 (Reentry Simulation, no randomness, 5DOF)
    """
    print("-------------------------------------------------")
    print("Part #1 (Reentry Simulation, no randomness, 5DOF)")
    print("(Linearized air density and wind shear gradient)")
    print("-------------------------------------------------")
    # establish the initial inputs
    m = 700 # kg
    Cd = 0.6 # unitless
    A = 14.4 # m^2
    h_deploy = 10000 # parachute deployment height, m
    ws =  15 # m/s
    theta_ws = 270 # deg
    rho = 1.82 # kg/m^3
    g0 = 9.81 # m/s^2
    xyz0 = [0.0, 0.0, 60000.0] # initial position vector, m
    v_xyz0 = [70, 70, -280] # initial velocity vector, m/s
    t0 = 0 # initial time, s
    
    # create a stage object (contains the states of the vehicle)
    stage = Stage(m, Cd, A, h_deploy, xyz0, v_xyz0, dof_model='5dof')
    # create an environment object (atmospheric properties, wind model, gravity model, etc.)
    env = Environment(ws, theta_ws, rho, g0)
    # create a solver object
    solver = Solver(delta_t, stage, env)
    print("Computing trajectory for part 1 of the test...")
    stage, _ = solver.solve_trajectory_RK4()
    print("Trajectory is computed.")
    del solver
    plot_trajectory(stage, plot_dir)
    
def part_2_3dof(delta_t:float, trajectory_count:int, plot_dir:str):
    """
    Part #2 (Reentry Simulation, with randomness, 3DOF)
    """
    # establish the initial inputs
    m = 700 # kg
    Cd = 1.4 # unitless
    A = 1.14 # m^2
    h_deploy = 10000 # parachute deployment height, m
    xyz0 = [0.0, 0.0, 60000.0] # initial position vector, m
    v_xyz0 = [70, 70, -280] # initial velocity vector, m/s
    dof_model = "3dof"
    stage_inputs = [m, Cd, A, h_deploy, xyz0, v_xyz0, dof_model]
    
    print()
    print("---------------------------------------------------")
    print("Part #2 (Reentry Simulation, with randomness, 3DOF)")
    print("(Linearized air density and wind shear gradient)")
    print("---------------------------------------------------")

    stages, envs = parallel_solve_trajectories(trajectory_count, delta_t, stage_inputs)
    print(f"All {trajectory_count} trajectories are computed.")
    plot_trajectories(stages, plot_dir)

def part_2_5dof(delta_t:float, trajectory_count:int, plot_dir:str):
    """
    Part #2 (Reentry Simulation, with randomness, 3DOF)
    """
    # establish the initial 3dof inputs
    m = 700 # kg
    Cd = 0.6 # unitless
    A = 14.4 # (normal reference area of the aero forces on the cylinder) m^2
    h_deploy = 10000 # parachute deployment height, m
    xyz0 = [0.0, 0.0, 60000.0] # initial position vector, m
    v_xyz0 = [70, 70, -280] # initial velocity vector, m/s
    dof_model = "5dof"
    stage_inputs = [m, Cd, A, h_deploy, xyz0, v_xyz0, dof_model]
    
    print()
    print("---------------------------------------------------")
    print("Part #2 (Reentry Simulation, with randomness, 5DOF)")
    print("(Linearized air density and wind shear gradient)")
    print("---------------------------------------------------")
    
    stages, envs = parallel_solve_trajectories(trajectory_count, delta_t, stage_inputs)
    print(f"All {trajectory_count} trajectories are computed.")
    plot_trajectories(stages, plot_dir)
    
if __name__ == "__main__":
    delta_t = 0.1 #s
    tic = time.time()
    part_1_3dof(delta_t,os.path.join("part_1_3dof"))
    part_1_5dof(delta_t,os.path.join("part_1_5dof"))

    trajectory_count = 200 # number of random trajectories to compute
    part_2_3dof(delta_t, trajectory_count, os.path.join("part_2_3dof"))
    part_2_5dof(delta_t, trajectory_count, os.path.join("part_2_5dof"))
    
    toc = time.time()
    print("-------------------------------------------------")
    print("End of Rocket Lab Flight Analysis Interview Test")
    print(f"Elapsed run time: {toc - tic:.2f} seconds")
    print("-------------------------------------------------")
    