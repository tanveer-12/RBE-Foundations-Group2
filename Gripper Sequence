import rclpy
from rclpy.node import Node
from open_manipulator_msgs.srv import SetJointPosition
import sys
import math
import time

Position = [125, 125, 60]
speed = 1.0 #Robot Path travel time (Sec)



class BasicRobotControl(Node):
    def __init__(self):
        super().__init__('basic_robot_control')
        self.joint_client = self.create_client(SetJointPosition, 'goal_joint_space_path')
        self.gripper_client = self.create_client(SetJointPosition, 'goal_tool_control')
        
        # Wait for services
        while not self.joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for joint movement service...")

        while not self.gripper_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for gripper service...")

        self.get_logger().info("All services ready.")
        
        
        
        self.base()
        #time.sleep(speed+0.1)
        #self.close_gripper()
        #self.open_gripper()
        #time.sleep(speed+0.1)
        #self.above()
        #time.sleep(speed+0.1)
        #self.location()
        #time.sleep(speed+0.1)
        #self.close_gripper()
        #time.sleep(speed+0.1)
        #self.above()
        #time.sleep(speed+0.1)
        #self.base()


    def base(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        
        request.joint_position.position = [0.0, 0.0-0.05, 0.0, 0.0, -0.05]
        request.path_time = speed

        self.future = self.joint_client.call_async(request)
        
    def above(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        
        x = Position[0]
        y = Position[1]
        z = Position[2]+75
        
        A = math.sqrt(x**2+y**2)
        B = z+37.074
        R1 = 130.23
        R2 = 124
        alpha = math.radians(10.62)
        M = math.sqrt(A**2+B**2)
        K = (A**2+B**2+R1**2-R2**2)/(2*R1)
        phi = math.atan2(B, A)
        
        u = math.asin(K/M) - phi
        
        v = math.atan2(R1*math.cos(u)-B, A-R1*math.sin(u))
        q2 = u - alpha
        q3 = v - u + alpha
        q4 = math.radians(90) - q2 - q3
        q1 = math.atan2(y,x)
	

        
        
        request.joint_position.position = [q1, q2, q3, q4, 0.0]
        request.path_time = speed

        self.future = self.joint_client.call_async(request)
        
        
    def location(self):
        request = SetJointPosition.Request()
        request.planning_group = ''
        request.joint_position.joint_name = ['joint1', 'joint2', 'joint3', 'joint4', 'gripper']
        
        x = Position[0]
        y = Position[1]
        z = Position[2]
        
        A = math.sqrt(x**2+y**2)
        B = z+37.074
        R1 = 130.23
        R2 = 124
        alpha = math.radians(10.62)
        M = math.sqrt(A**2+B**2)
        K = (A**2+B**2+R1**2-R2**2)/(2*R1)
        phi = math.atan2(B, A)
        
        u = math.asin(K/M) - phi
        
        v = math.atan2(R1*math.cos(u)-B, A-R1*math.sin(u))
        q2 = u - alpha
        q3 = v - u + alpha
        q4 = math.radians(90) - q2 - q3
        q1 = math.atan2(y,x)
	

        
        
        request.joint_position.position = [q1, q2, q3, q4, 0.0]
        request.path_time = speed

        self.future = self.joint_client.call_async(request)

    def open_gripper(self):
        req = SetJointPosition.Request()
        req.planning_group = 'gripper'
        req.joint_position.joint_name = ['gripper']
        req.joint_position.position = [0.01]  # open
        req.path_time = speed

        self.future = self.gripper_client.call_async(req)
        
    def close_gripper(self):
        req = SetJointPosition.Request()
        req.planning_group = 'gripper'
        req.joint_position.joint_name = ['gripper']
        req.joint_position.position = [0.0025]  # close
        req.path_time = speed

        self.future = self.gripper_client.call_async(req)


def main(args=None):
    rclpy.init(args=args)

    basic_robot_control = BasicRobotControl()

    while rclpy.ok():
        rclpy.spin_once(basic_robot_control)
        if basic_robot_control.future.done():
            try:
                response = basic_robot_control.future.result()
            except Exception as e:
                basic_robot_control.get_logger().error('Service call failed %r' % (e,))
            break

    basic_robot_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
