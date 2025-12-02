Build steps:

# 1. Build
cd ~/ros2_ws      #Change directory to relevant workspace here

colcon build --packages-select open_manipulator_pd_control

source install/setup.bash

# 2. Configure hardware
python3 set_current_mode.py

# 3. Launch robot
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

# 4. Start controller (in a different terminal)
ros2 run open_manipulator_pd_control pd_controller_node.py --ros-args -p kp:=5.0 -p kd:=0.5    # Change values of kp and kd for tuning

# 5. Run test 1 (in a 3rd terminal)
ros2 run open_manipulator_pd_control test_client.py start_log

ros2 run open_manipulator_pd_control test_client.py position 0.0

sleep 10

ros2 run open_manipulator_pd_control test_client.py stop_log

# 6. Run test 2
sleep 2

ros2 run open_manipulator_pd_control test_client.py start_log

ros2 run open_manipulator_pd_control test_client.py position 0.5

sleep 10

ros2 run open_manipulator_pd_control test_client.py stop_log

# 7. Run test 3
sleep 2

ros2 run open_manipulator_pd_control test_client.py start_log

ros2 run open_manipulator_pd_control test_client.py position -0.5

sleep 10

ros2 run open_manipulator_pd_control test_client.py stop_log
