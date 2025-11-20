import math
import numpy as np


# Temp values to remove problems in VS Code
offset = math.radians(10.62)
q1 = 1
q2 = 1
q3 = 1
q4 = 1
q5 = 1


A1 = np.array([
	[math.cos(q1), 0, -math.sin(q1), 0],
   	[math.sin(q1), 0, math.cos(q1), 0],
   	[0, -1, 0, 96.326],
   	[0, 0, 0, 1]
    ])
        
A2 = np.array([
    [math.cos(-offset+q2), -math.sin(-offset+q2), 0, 130.23*math.cos(-offset+q2)],
    [math.sin(-offset+q2), math.cos(-offset+q2), 0, 130.23*math.sin(-offset+q2)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
        
A3 = np.array([
    [math.cos(offset+q3), -math.sin(offset+q3), 0, 124*math.cos(offset+q3)],
    [math.sin(offset+q3), math.cos(offset+q3), 0, 124*math.sin(offset+q3)],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
        
A4 = np.array([
   	[math.cos(q4), -math.sin(q4), 0, 133.4*math.cos(q4)],
    [math.sin(q4), math.cos(q4), 0, 133.4*math.sin(q4)],
   	[0, 0, 1, 0],
   	[0, 0, 0, 1]
])
        
        
  	#Defining the overall transform matrix for a given set of q values
    # A is the tranform matrix for each frame 
    # H is the homogeneous transform to get back to the base frame

H1 = A1
H2 = H1 @ A2
H3 = H2 @ A3
H4 = H3 @ A4

    # Finding the end effector Position
o_n = H4[0:3, 3]

    # Find z_i-1 vectors for jacobians
z_0 = np.array([
   	[0],
   	[0],
   	[1]
])
z_1 = H1[0:3, 2]
z_2 = H2[0:3, 2]
z_3 = H3[0:3, 2]

    # Finding the position vector for each transform

o1 = H1[0:3, 3]
o2 = H2[0:3, 3]
o3 = H3[0:3, 3]

    # calculate Linear portions of Jacobian

L1 = np.cross(z0, o_n)
L2 = np.cross(z1, o_n - o1)
L3 = np.cross(z2, o_n - o2)
L4 = np.cross(z3, o_n - o3)

    # Define the Jacobian

J = np.array([
   	[L1, L2, L3, L4],
   	[z_0, z_1, z_2, z_3],
])
