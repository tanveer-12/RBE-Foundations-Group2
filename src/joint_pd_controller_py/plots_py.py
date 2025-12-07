import pandas as pd
import matplotlib.pyplot as plt

# Load your CSV file (replace with your actual filename)
df = pd.read_csv('pd_controller_log_YYYYMMDD_HHMMSS.csv')

# Plot current_position and desired_position vs time
plt.figure(figsize=(12, 8))
plt.subplot(2, 1, 1)
plt.plot(df['time'], df['current_position'], label='Current Position', linewidth=2)
plt.plot(df['time'], df['desired_position'], label='Desired Position', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Position (rad)')
plt.title('PD Controller Performance')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(df['time'], df['error'], label='Position Error', linewidth=2)
plt.xlabel('Time (s)')
plt.ylabel('Error (rad)')
plt.title('Position Error')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()