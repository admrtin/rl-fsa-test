"""
The parallezation and trajectory randomizer functions
"""
import concurrent.futures as cf
import os
import numpy as np
from solver import Solver
from stage import Stage
from environment import Environment

def generate_random_trajectory(delta_t:float, stage_inputs:list):
    """Randomizes a trajectory, returns the corresponding solver"""
    # establish the initial, non-random inputs
    m = stage_inputs[0] # kg
    Cd = stage_inputs[1] # unitless
    A = stage_inputs[2] # m^2
    h_deploy = stage_inputs[3] # parachute deployment height, m
    xyz0 = stage_inputs[4] # initial position vector, m
    v_xyz0 = stage_inputs[5] # initial velocity vector, m/s
    dof_model = stage_inputs[6]
    
    # environment
    rho = 1.82 # kg/m^3
    g0 = 9.81 # m/s^2
    ws = np.random.uniform(0, 30) # from problem statement
    theta_ws = np.random.uniform(0, 360) # from problem statement
    
    # create a stage object (contains the states of the vehicle)
    stage = Stage(m, Cd, A, h_deploy, xyz0, v_xyz0, dof_model)
    # create an environment object (atmospheric properties, wind model, gravity model, etc.)
    env = Environment(ws, theta_ws, rho, g0)
    # create a solver object
    solver = Solver(delta_t, stage, env)
    return solver

def parallel_solve_trajectories(trajectory_count:int, delta_t:float, stage_inputs:list):
    """creates random trajectories, parallelizes & solves"""
    # vary the wind speed and angle with a uniform distribution
    # create {trajectory_count} stages and solvers, each with a different wind speed and angle
    stages = []
    envs = []
    # parallelize the solver computations
    futures = []
    print(f"{trajectory_count} randomized trajectories submitted to the ThreadPool queue...")
    max_workers = max(1, int(os.cpu_count() * 0.75))
    with cf.ThreadPoolExecutor(max_workers=max_workers) as executor:
        future_to_idx = {}
        for i in range(trajectory_count):
            solver = generate_random_trajectory(delta_t, stage_inputs)
            # solve the equations of motion
            future = executor.submit(solver.solve_trajectory_RK4)
            future_to_idx[future] = i+1
        
        
        # Process each future when it is completed
        completed_count = 0 # to track the number of completed trajectories
        for future in cf.as_completed(future_to_idx):
            idx = future_to_idx[future]

            try:
                stage, env = future.result()
                completed_count += 1
                # update the stage object with the computed states
                stages.append(stage)
                envs.append(env)
                print(f"Trajectory {idx} is computed. {completed_count}/{trajectory_count} complete.")
            except Exception as e:
                print(f"Trajectory {idx} generated an exception: {e}")
                completed_count += 1
                continue

    return stages, envs