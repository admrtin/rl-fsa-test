import matplotlib.pyplot as plt
from stage import Stage
import os

def plot_trajectories(stages:list[Stage], plot_dir:str):
    """Plots a list of solved stages"""
    # Create 3D figure for trajectories
    fig1, ax1 = plt.subplots(subplot_kw={'projection': '3d'})
    fig2, ax2 = plt.subplots()
    fig3, (ax3_theta, ax3_psi) = plt.subplots(2, 1, sharex=True, figsize=(10, 8))
    
    plot_attitude = False
    for stage in stages:
        # Convert states to km for better visualization
        x_km = stage.states[:,1] / 1000
        y_km = stage.states[:,2] / 1000
        z_km = stage.states[:,3] / 1000
         
        # Plot trajectory
        ax1.plot(x_km, y_km, z_km)
        
        # Plot impact location (last point)
        ax2.scatter(x_km[-1], y_km[-1], marker='x')
        
        if stage.dof_model == "5dof":
            plot_attitude = True
            
            time = stage.states[:,0]
            theta = stage.states[:,7]  # pitch angle (rad)
            psi = stage.states[:,8]    # yaw angle (rad)
            
            ax3_theta.plot(time, theta, label='omega_x')
            ax3_psi.plot(time, psi, label='omega_y')
            
    
    ax1.set_xlabel('x [km]')
    ax1.set_ylabel('y [km]')
    ax1.set_zlabel('z [km]')
    ax1.set_title('Randomized Trajectories')
    
    ax2.set_xlabel('x [km]')
    ax2.set_ylabel('y [km]')
    ax2.set_title('Impact locations of the trajectories')
    
    # Ensure there's enough space for labels
    fig1.tight_layout()
    fig2.tight_layout()
    
    # Save figures
    os.makedirs(plot_dir, exist_ok=True)
    fig1_path = os.path.join(plot_dir, "part_2_trajectories.png")
    fig2_path = os.path.join(plot_dir, "part_2_impact_locations.png")
    
    fig1.savefig(fig1_path)
    print(f"Trajectory visualization saved as {fig1_path}")
    
    fig2.savefig(fig2_path)
    print(f"Impact locations saved as {fig2_path}")
    
    if plot_attitude:
        ax3_theta.set_xlabel("Time [s]")
        ax3_theta.set_ylabel("omega_x [deg]")
        ax3_psi.set_xlabel('Time [s]')
        ax3_psi.set_ylabel('omega_y [deg]')

        fig3.suptitle('Attitude Evolution', fontsize=12)
        fig3.tight_layout()
        fig3_path = os.path.join(plot_dir, "part_2_attitude.png")
        fig3.savefig(fig3_path)
        print(f"Attitude evolution saved as {fig3_path}")
    
    plt.close(fig1)
    plt.close(fig2)
    plt.close(fig3)
    return

def plot_trajectory(stage:Stage, plot_dir:str):
    """Plots a solved stage"""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    # Convert to km for consistency with other plots
    x_km = stage.states[:,1] / 1000
    y_km = stage.states[:,2] / 1000
    z_km = stage.states[:,3] / 1000
    
    ax.plot(x_km, y_km, z_km)
    ax.set_xlabel('x [km]')
    ax.set_ylabel('y [km]')
    ax.set_zlabel('z [km]')
    ax.set_title('Single Trajectory')
    
    fig.tight_layout()
    os.makedirs(plot_dir, exist_ok=True)
    fig_path = os.path.join(plot_dir, "part_1_trajectory.png")
    fig.savefig(fig_path)
    print(f"Figure saved as {fig_path}")
    
    plt.close(fig)
    return