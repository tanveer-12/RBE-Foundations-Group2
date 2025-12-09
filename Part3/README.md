Build steps:

# 1. Build
cd ~/ros2_ws      #Change directory to relevant workspace here
colcon build --packages-select open_manipulator_pd_control
source install/setup.bash

# 2. Configure hardware
python3 set_current_mode.py

# 3. Start controller
ros2 run open_manipulator_pd_control pd_controller_direct.py --ros-args -p kp:=5.0 -p kd:=0.5    # Change values of kp and kd for tuning

# 4. Run tests
ros2 run open_manipulator_pd_control test_client.py start_log
ros2 run open_manipulator_pd_control test_client.py position 0.0
ros2 run open_manipulator_pd_control test_client.py stop_log

# 5. Plot results
python3 plot_pd_results.py pd_control_log_20251202_182405.txt   # Update the relevant log file here
