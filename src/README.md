# OpenManipulatorX_ROS2
ROS2 Humble packages to control Open Manipulator X

## Build
Copy the contents of the entire repository into the src/ directory of a new ROS2 workspace.
```
mkdir -p rbe500_ws/src
cd rbe500_ws
git clone https://github.com/yera217/OpenManipulatorX_ROS2.git ./src
```

Run the following to build the packages in your new workspace:
```
colcon build --symlink-install
```
You may get some warning mesages, ignore them for now as long as everything successfully builds with no errors.


Don't forget to source your workspace (and add source line with full path to ~/.bashrc if you want to automatically source the workspace for each new terminal instance):
```
source install/setup.bash
```

After connecting to the robot over USB, run the following to begin position control:
```
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py
```

Now run the example code to move the robot:
```
ros2 run rbe500-example basic_robot_control
```
Or for python:
```
ros2 run rbe500_example_py basic_robot_control
```


---
RBE 500 - Foundations of Robotics 2025 taught by Professor Haichong Zhang at Worcester Polytechnic Institute Robotics Engineering Department.

Forked from Steven Hyland's repo for RBE500 Fall2023 offering taught by Professor Berk Calli at Worcester Polytechnic Institute Robotics Engineering Department.
=================
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