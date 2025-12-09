#!/usr/bin/env python3

"""
Plot PD controller results from CSV files
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import glob
import os

def load_log_files():
    """Find and load all log files"""
    log_files = glob.glob('pd_controller_log_*.csv')
    if not log_files:
        print("No log files found!")
        return None
    
    # Sort by modification time to get most recent first
    log_files.sort(key=os.path.getmtime, reverse=True)
    return log_files

def plot_log_file(filename):
    """Plot a single log file"""
    try:
        # Read the CSV file
        data = pd.read_csv(filename)
        
        # Create plots
        fig, axes = plt.subplots(3, 1, figsize=(12, 10))
        fig.suptitle(f'PD Controller Results - {os.path.basename(filename)}', fontsize=14)
        
        # Plot 1: Position tracking
        axes[0].plot(data['time'], data['desired_position'], 'g-', label='Reference Position', linewidth=2)
        axes[0].plot(data['time'], data['current_position'], 'b-', label='Current Position', linewidth=2)
        axes[0].set_ylabel('Position (rad)')
        axes[0].set_title('Position Tracking')
        axes[0].legend()
        axes[0].grid(True)
        
        # Plot 2: Error
        axes[1].plot(data['time'], data['error'], 'r-', linewidth=2)
        axes[1].set_ylabel('Error (rad)')
        axes[1].set_title('Position Error')
        axes[1].grid(True)
        
        # Plot 3: Control effort
        axes[2].plot(data['time'], data['effort_command'], 'm-', linewidth=2)
        axes[2].set_ylabel('Effort Command')
        axes[2].set_xlabel('Time (s)')
        axes[2].set_title('Control Effort')
        axes[2].grid(True)
        
        plt.tight_layout()
        plt.show()
        
        # Print statistics
        print(f"\nFile: {filename}")
        print(f"Duration: {data['time'].max():.2f} seconds")
        print(f"Max error: {abs(data['error']).max():.6f} rad")
        print(f"Mean error: {abs(data['error']).mean():.6f} rad")
        print(f"Final error: {data['error'].iloc[-1]:.6f} rad")
        print(f"Max effort: {abs(data['effort_command']).max():.6f}")
        
    except Exception as e:
        print(f"Error plotting {filename}: {e}")

def main():
    log_files = load_log_files()
    if not log_files:
        print("No log files found. Run the controller first to generate data.")
        return
    
    print("Available log files:")
    for i, log_file in enumerate(log_files):
        print(f"{i+1}. {log_file}")
    
    if len(log_files) == 1:
        plot_log_file(log_files[0])
    else:
        while True:
            try:
                choice = input(f"\nSelect file to plot (1-{len(log_files)}) or 'q' to quit: ")
                if choice.lower() == 'q':
                    break
                choice = int(choice) - 1
                if 0 <= choice < len(log_files):
                    plot_log_file(log_files[choice])
                else:
                    print("Invalid choice!")
            except (ValueError, KeyboardInterrupt):
                break

if __name__ == '__main__':
    main()