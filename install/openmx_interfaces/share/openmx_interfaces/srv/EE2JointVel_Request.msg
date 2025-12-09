# Service for converting end-effector velocities to joint velocities
# Inverse velocity kinematics: q_dot = J^+(q) * V

# Request: Joint configuration and desired end-effector velocity
float64 q1          # Joint 1 position (rad)
float64 q2          # Joint 2 position (rad)
float64 q3          # Joint 3 position (rad)
float64 q4          # Joint 4 position (rad)
float64 vx          # Desired EE linear velocity x-component (m/s)
float64 vy          # Desired EE linear velocity y-component (m/s)
float64 vz          # Desired EE linear velocity z-component (m/s)
float64 wx          # Desired EE angular velocity x-component (rad/s)
float64 wy          # Desired EE angular velocity y-component (rad/s)
float64 wz          # Desired EE angular velocity z-component (rad/s)

