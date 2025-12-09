# Service for converting joint velocities to end-effector velocities
# Forward velocity kinematics: V = J(q) * q_dot

# Request: Joint configuration and joint velocities
float64 q1          # Joint 1 position (rad)
float64 q2          # Joint 2 position (rad)
float64 q3          # Joint 3 position (rad)
float64 q4          # Joint 4 position (rad)
float64 q1_dot      # Joint 1 velocity (rad/s)
float64 q2_dot      # Joint 2 velocity (rad/s)
float64 q3_dot      # Joint 3 velocity (rad/s)
float64 q4_dot      # Joint 4 velocity (rad/s)

