#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import sys
import os


def load_log_file(filename):
    """Load data from the log file"""
    time = []
    reference = []
    current = []
    error = []
    effort = []
    
    with open(filename, 'r') as f:
        for line in f:
            # Skip comment lines
            if line.startswith('#'):
                print(line.strip())
                continue
            
            # Parse data
            values = line.strip().split(',')
            if len(values) == 5:
                time.append(float(values[0]))
                reference.append(float(values[1]))
                current.append(float(values[2]))
                error.append(float(values[3]))
                effort.append(float(values[4]))
    
    return (np.array(time), np.array(reference), np.array(current), 
            np.array(error), np.array(effort))


def calculate_metrics(time, reference, current, error):
    """Calculate performance metrics"""
    metrics = {}
    
    # Settling time (2% criterion)
    ref_change = abs(reference[-1] - reference[0])
    settling_threshold = 0.02 * ref_change
    settling_indices = np.where(np.abs(error) <= settling_threshold)[0]
    
    if len(settling_indices) > 0:
        metrics['settling_time'] = time[settling_indices[0]]
    else:
        metrics['settling_time'] = None
    
    # Overshoot
    if reference[-1] > reference[0]:
        overshoot = np.max(current) - reference[-1]
    else:
        overshoot = np.min(current) - reference[-1]
    
    metrics['overshoot'] = overshoot
    metrics['overshoot_percent'] = abs(overshoot) / ref_change * 100 if ref_change > 0 else 0
    
    # Steady-state error (average of last 100 samples)
    metrics['steady_state_error'] = np.mean(error[-100:])
    
    # RMS error
    metrics['rms_error'] = np.sqrt(np.mean(error**2))
    
    # Mean absolute error
    metrics['mean_abs_error'] = np.mean(np.abs(error))
    metrics['max_abs_error'] = np.max(np.abs(error))
    
    return metrics


def plot_results(time, reference, current, error, effort, metrics, filename):
    """Create plots for PD controller analysis"""
    
    fig, axes = plt.subplots(3, 1, figsize=(12, 10))
    fig.suptitle('PD Controller Performance Analysis', fontsize=16, fontweight='bold')
    
    # Subplot 1: Position tracking
    axes[0].plot(time, reference, 'r--', linewidth=2, label='Reference')
    axes[0].plot(time, current, 'b-', linewidth=1.5, label='Current Position')
    axes[0].grid(True, alpha=0.3)
    axes[0].set_xlabel('Time (s)', fontsize=12)
    axes[0].set_ylabel('Position (rad)', fontsize=12)
    axes[0].set_title('Position Tracking', fontsize=14, fontweight='bold')
    axes[0].legend(loc='best', fontsize=10)
    axes[0].set_xlim([time[0], time[-1]])
    
    # Subplot 2: Tracking error
    axes[1].plot(time, error, 'g-', linewidth=1.5)
    axes[1].axhline(y=0, color='k', linestyle='--', linewidth=1)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_xlabel('Time (s)', fontsize=12)
    axes[1].set_ylabel('Error (rad)', fontsize=12)
    axes[1].set_title('Position Error', fontsize=14, fontweight='bold')
    axes[1].set_xlim([time[0], time[-1]])
    
    # Add error statistics to plot
    textstr = f"Mean |Error|: {metrics['mean_abs_error']:.4f} rad\n"
    textstr += f"Max |Error|: {metrics['max_abs_error']:.4f} rad"
    props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
    axes[1].text(0.7, 0.95, textstr, transform=axes[1].transAxes, 
                fontsize=10, verticalalignment='top', bbox=props)
    
    # Subplot 3: Control effort
    axes[2].plot(time, effort, 'm-', linewidth=1.5)
    axes[2].grid(True, alpha=0.3)
    axes[2].set_xlabel('Time (s)', fontsize=12)
    axes[2].set_ylabel('Effort', fontsize=12)
    axes[2].set_title('Control Effort', fontsize=14, fontweight='bold')
    axes[2].set_xlim([time[0], time[-1]])
    
    plt.tight_layout()
    
    # Save figure
    base_name = os.path.splitext(filename)[0]
    output_filename = f'{base_name}_plot.png'
    plt.savefig(output_filename, dpi=300, bbox_inches='tight')
    print(f'\nFigure saved as: {output_filename}')
    
    plt.show()


def print_metrics(metrics):
    """Print performance metrics"""
    print('\n' + '='*40)
    print('Performance Metrics')
    print('='*40)
    
    if metrics['settling_time'] is not None:
        print(f"Settling time (2% criterion): {metrics['settling_time']:.3f} s")
    else:
        print("Settling time (2% criterion): Not achieved")
    
    print(f"Overshoot: {metrics['overshoot']:.4f} rad ({metrics['overshoot_percent']:.2f}%)")
    print(f"Steady-state error: {metrics['steady_state_error']:.6f} rad")
    print(f"RMS error: {metrics['rms_error']:.6f} rad")
    print(f"Mean absolute error: {metrics['mean_abs_error']:.6f} rad")
    print(f"Max absolute error: {metrics['max_abs_error']:.6f} rad")
    print('='*40 + '\n')


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 plot_pd_results.py <log_file>")
        print("Example: python3 plot_pd_results.py pd_control_log_20240101_120000.txt")
        sys.exit(1)
    
    filename = sys.argv[1]
    
    if not os.path.exists(filename):
        print(f"Error: File '{filename}' not found")
        sys.exit(1)
    
    print(f"Loading data from: {filename}")
    print('-'*40)
    
    # Load data
    time, reference, current, error, effort = load_log_file(filename)
    
    print(f"\nLoaded {len(time)} data points")
    print(f"Time range: {time[0]:.3f} to {time[-1]:.3f} seconds")
    print(f"Duration: {time[-1] - time[0]:.3f} seconds")
    
    # Calculate metrics
    metrics = calculate_metrics(time, reference, current, error)
    
    # Print metrics
    print_metrics(metrics)
    
    # Create plots
    plot_results(time, reference, current, error, effort, metrics, filename)


if __name__ == '__main__':
    main()
