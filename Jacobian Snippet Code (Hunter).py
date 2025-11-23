import math
import numpy as np

    #==================================================#
    #       Take the q values from the robot here      #
    #==================================================#



    # Temp values to remove problems in VS Code
    # Values need to be pulled from the 

def compute_jacobian(self, q):
    offset = math.radians(10.62)
    
    # Get the joint position as an input
    q1, q2, q3, q4 = q

    # Define matrix transformations from Part 1
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
    z_0 = np.array([0, 0, 1])
    z_1 = H1[0:3, 2]
    z_2 = H2[0:3, 2]
    z_3 = H3[0:3, 2]

    # Finding the position vector for each transform
    o1 = H1[0:3, 3]
    o2 = H2[0:3, 3]
    o3 = H3[0:3, 3]


    # calculate Linear portions of Jacobian
    L1 = np.cross(z_0, o_n)
    L2 = np.cross(z_1, o_n - o1)
    L3 = np.cross(z_2, o_n - o2)
    L4 = np.cross(z_3, o_n - o3)

    # Define the Jacobian
    J = np.column_stack([
        np.concatenate([L1, z_0]).reshape(6,1),
        np.concatenate([L2, z_1]).reshape(6,1),
        np.concatenate([L3, z_2]).reshape(6,1),
        np.concatenate([L4, z_3]).reshape(6,1)
    ])

    return J


    #====================================================================================================#
    #                                         V = J * q_dot                                              #
    #           q_dot = ((J^T * J)^-1 * J^T) * V + (I - ((J^T * J)^-1 * J^T) * J) * z_hat                #
    #                   q_dot equation is taken from the notes for a generic system                      #
    #      Null space does not exist for thise overdefined system, no the q_dot equation reduces to      #
    #                               q_dot = ((J^T * J)^-1 * J^T) * V                                     #
    #====================================================================================================#


    # Need to figure out how q_dot can be defined or found here
    # I was thinking (q_f-q_i)/t where t is some sampling time between the two measurements
    # I dont know if joint velocities are something that we can measure directly from a topic

    # Define the joint velocity inputs

def joint_to_ee_vel_callback(self, request, response):

    #We need to figure out how to get the q_dot values somehow
    q = np.array([request.q1, request.q2, request.q3, request.q4])
    q_dot = np.array([request.q1_dot, request.q2_dot, request.q3_dot, request.q4_dot])

    # build the Jacobian J from q
    J = self.compute_jacobian(q)

    # Calculate EE velocity from equation
    V = J @ q_dot

    # Return linear and angular velocitites
    response.vx = float(V[0])
    response.vy = float(V[1])
    response.vz = float(V[2])
    response.wx = float(V[3])
    response.wy = float(V[4])
    response.wz = float(V[5])
    response.success = True

    return response

def ee_to_joint_vel_callback(self, request, response):
    q = np.array([request.q1, request.q2, request.q3, request.q4])
    V = np.array([request.vx, request.vy, request.vz, request.wx, request.wy, request.wz])

    J = self.compute_jacobian(q)

    # Calculate the matrix from the inverse calculations
    reverse_J = np.linalg.inv(J.T @ J) @ J.T

    # Calculate joint velocity from end effector Velocity
    q_dot = reverse_J @ V

    response.q1_dot = float(q_dot[0])
    response.q2_dot = float(q_dot[1])
    response.q3_dot = float(q_dot[2])
    response.q4_dot = float(q_dot[3])
    response.success = True

    return response
