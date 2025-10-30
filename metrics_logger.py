import numpy as np
import matplotlib.pyplot as plt
import json
import os
from datetime import datetime

def save_simulation_metrics(vehicle, explorer, filename_prefix='simulation_metrics'):
    """
    Save comprehensive simulation metrics and plots for project documentation.
    
    Args:
        vehicle: AckermannVehicle instance with motion history
        explorer: Explorer instance with coverage metrics
        filename_prefix: Prefix for saved files
    """
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f'results_{timestamp}'
    os.makedirs(output_dir, exist_ok=True)
    
    # 1. Save motion data
    motion_data = {
        'time': vehicle.history['time'],
        'velocity': vehicle.history['v'],
        'angular_velocity': vehicle.history['omega'],
        'steering_left': vehicle.history['delta_left'],
        'steering_right': vehicle.history['delta_right'],
        'wheel_velocities': {
            'LF': vehicle.history['omega_LF'],
            'RF': vehicle.history['omega_RF'],
            'LR': vehicle.history['omega_LR'],
            'RR': vehicle.history['omega_RR']
        },
        'trajectory': {
            'x': vehicle.history['x'],
            'y': vehicle.history['y'],
            'theta': vehicle.history['theta']
        }
    }
    
    with open(f'{output_dir}/motion_data.json', 'w') as f:
        json.dump(motion_data, f, indent=2)
    
    # 2. Save exploration metrics
    metrics = explorer.get_coverage_metrics()
    with open(f'{output_dir}/exploration_metrics.txt', 'w') as f:
        f.write("Exploration Performance Metrics\n")
        f.write("============================\n\n")
        f.write(f"Final Coverage: {metrics['coverage']:.2f}%\n")
        f.write(f"Total Distance: {metrics['total_distance']:.2f}m\n")
        f.write(f"Frontiers Visited: {metrics['frontiers_visited']}\n")
        
        # Calculate additional metrics
        total_time = motion_data['time'][-1]
        avg_velocity = np.mean(motion_data['velocity'])
        max_velocity = np.max(motion_data['velocity'])
        
        f.write(f"\nDetailed Motion Analysis\n")
        f.write(f"Total Time: {total_time:.2f}s\n")
        f.write(f"Average Velocity: {avg_velocity:.2f}m/s\n")
        f.write(f"Maximum Velocity: {max_velocity:.2f}m/s\n")
    
    # 3. Generate and save plots
    # Velocity profile
    plt.figure(figsize=(10, 6))
    plt.plot(motion_data['time'], motion_data['velocity'])
    plt.title('Vehicle Velocity Profile')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.grid(True)
    plt.savefig(f'{output_dir}/velocity_profile.png')
    
    # Steering angles
    plt.figure(figsize=(10, 6))
    plt.plot(motion_data['time'], np.rad2deg(motion_data['steering_left']), 
             label='Left Wheel')
    plt.plot(motion_data['time'], np.rad2deg(motion_data['steering_right']), 
             label='Right Wheel')
    plt.title('Steering Angles')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.legend()
    plt.grid(True)
    plt.savefig(f'{output_dir}/steering_angles.png')
    
    # Coverage progress
    coverage_data = [(entry['time'], entry['coverage']) 
                    for entry in metrics['coverage_history']]
    times, coverages = zip(*coverage_data)
    
    plt.figure(figsize=(10, 6))
    plt.plot(times, coverages)
    plt.title('Exploration Coverage Progress')
    plt.xlabel('Time [s]')
    plt.ylabel('Coverage [%]')
    plt.grid(True)
    plt.savefig(f'{output_dir}/coverage_progress.png')
    
    # Trajectory plot
    plt.figure(figsize=(8, 8))
    plt.plot(motion_data['trajectory']['x'], 
            motion_data['trajectory']['y'])
    plt.title('Vehicle Trajectory')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.axis('equal')
    plt.grid(True)
    plt.savefig(f'{output_dir}/trajectory_2d.png')
    
    print(f"\nSimulation metrics and plots saved in: {output_dir}/")
    plt.close('all')