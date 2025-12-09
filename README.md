# RBE-Foundations-Group2

# Terminal 1: Build your package
cd ~/copy-RBE-Foundations-Group2
colcon build --packages-select joint_pd_controller_py
source install/setup.bash

# Terminal 2: Start OpenMANIPULATOR-X controller (in new terminal)
cd ~/copy-RBE-Foundations-Group2
source install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

# Terminal 3: Start your PD controller (in new terminal)
cd ~/copy-RBE-Foundations-Group2
source install/setup.bash
ros2 run joint_pd_controller_py pd_controller_node

# Terminal 4: Test the controller (in new terminal)

# First, start logging
ros2 run joint_pd_controller_py test_client start_log

# Set position to 0.3 radians and wait 10 seconds
ros2 run joint_pd_controller_py test_client position 0.3
sleep 10

# Stop logging
ros2 run joint_pd_controller_py test_client stop_log

# Terminal 4: Continue testing

# Wait 2 seconds between tests
sleep 2

# Test 2: Position 0.8 radians
ros2 run joint_pd_controller_py test_client start_log
ros2 run joint_pd_controller_py test_client position 0.8
sleep 10
ros2 run joint_pd_controller_py test_client stop_log

sleep 2

# Test 3: Position -0.5 radians
ros2 run joint_pd_controller_py test_client start_log
ros2 run joint_pd_controller_py test_client position -0.5
sleep 10
ros2 run joint_pd_controller_py test_client stop_log


# Terminal 5: Plot results (in new terminal)
cd ~/copy-RBE-Foundations-Group2
source install/setup.bash
ros2 run joint_pd_controller_py plot_pd_results


===== check everything at once ====
# In Terminal 2: Start robot
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py

# In Terminal 3: Start controller
ros2 run joint_pd_controller_py pd_controller_node

# In Terminal 4: Run all tests automatically
ros2 run joint_pd_controller_py test_client start_log
ros2 run joint_pd_controller_py test_client position 0.3
sleep 10
ros2 run joint_pd_controller_py test_client stop_log

sleep 2

ros2 run joint_pd_controller_py test_client start_log
ros2 run joint_pd_controller_py test_client position 0.8
sleep 10
ros2 run joint_pd_controller_py test_client stop_log

sleep 2

ros2 run joint_pd_controller_py test_client start_log
ros2 run joint_pd_controller_py test_client position -0.5
sleep 10
ros2 run joint_pd_controller_py test_client stop_log