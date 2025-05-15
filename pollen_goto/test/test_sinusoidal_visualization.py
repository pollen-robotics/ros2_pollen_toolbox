import numpy as np
import matplotlib.pyplot as plt
from pollen_goto.interpolation import sinusoidal, linear, minimum_jerk

def visualize_interpolation():
    """Visualize and compare different interpolation methods."""
    # Test parameters
    start_pos = np.array([0.0])
    goal_pos = np.array([1.0])
    duration = 2.0  # 2 seconds
    
    # Create interpolation functions
    sin_func = sinusoidal(start_pos, goal_pos, duration)
    lin_func = linear(start_pos, goal_pos, duration)
    min_jerk_func = minimum_jerk(start_pos, goal_pos, duration)
    
    # Generate time points
    times = np.linspace(0, duration, 100)
    
    # Calculate positions for each method
    sin_positions = np.array([sin_func(t) for t in times])
    lin_positions = np.array([lin_func(t) for t in times])
    min_jerk_positions = np.array([min_jerk_func(t) for t in times])
    
    # Calculate velocities (numerical differentiation)
    sin_velocities = np.gradient(sin_positions, times)
    lin_velocities = np.gradient(lin_positions, times)
    min_jerk_velocities = np.gradient(min_jerk_positions, times)
    
    # Create plots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
    
    # Plot positions
    ax1.plot(times, sin_positions, 'b-', label='Sinusoidal')
    ax1.plot(times, lin_positions, 'r--', label='Linear')
    ax1.plot(times, min_jerk_positions, 'g:', label='Minimum Jerk')
    ax1.set_xlabel('Time (s)')
    ax1.set_ylabel('Position')
    ax1.set_title('Position vs Time')
    ax1.grid(True)
    ax1.legend()
    
    # Plot velocities
    ax2.plot(times, sin_velocities, 'b-', label='Sinusoidal')
    ax2.plot(times, lin_velocities, 'r--', label='Linear')
    ax2.plot(times, min_jerk_velocities, 'g:', label='Minimum Jerk')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Velocity')
    ax2.set_title('Velocity vs Time')
    ax2.grid(True)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    visualize_interpolation() 