#include "rclcpp/rclcpp.hpp"
#include "open_manipulator_msgs/srv/set_joint_position.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <chrono>
#include <cstdlib>
#include <memory>
#include <cmath>
#include <vector>

using namespace std::chrono_literals;

class RobotControlWithFK : public rclcpp::Node {
private:
    // DH Parameters for OpenManipulator (in mm)
    const double L1 = 77.0;   // Base to joint2 (z-offset)
    const double L2 = 130.0;  // Joint2 to joint3 (link length)
    const double L3 = 124.0;  // Joint3 to joint4 (link length)
    const double L4 = 126.0;  // Joint4 to end-effector (link length)
    
    // ROS2 Client and Subscriber
    rclcpp::Client<open_manipulator_msgs::srv::SetJointPosition>::SharedPtr client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    // Current joint angles (rad)
    double theta1_, theta2_, theta3_, theta4_;
    bool have_joint_state_ = false;

    /**
     * @brief Calculate forward kinematics
     */
    std::vector<double> calculateFK(double t1, double t2, double t3, double t4) {
        // Compute cumulative angles for planar arm (joints 2, 3, 4)
        double theta234 = t2 + t3 + t4;
        
        // Calculate position in the plane of the arm
        double r = L2 * cos(t2) + L3 * cos(t2 + t3) + L4 * cos(theta234);
        double z = L1 + L2 * sin(t2) + L3 * sin(t2 + t3) + L4 * sin(theta234);
        
        // Rotate by theta1 to get 3D coordinates
        double x = r * cos(t1);
        double y = r * sin(t1);
        
        return {x, y, z};
    }

public:
    RobotControlWithFK() : Node("robot_control_with_fk") {
        // Create service client
        client_ = this->create_client<open_manipulator_msgs::srv::SetJointPosition>(
            "goal_joint_space_path");
        
        // Create joint state subscriber
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&RobotControlWithFK::jointCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Robot Control with FK Node Started");
        printDHParameters();
    }
    
    void printDHParameters() {
        std::cout << "\n=== DH Parameters ===" << std::endl;
        std::cout << "L1 (base height): " << L1 << " mm" << std::endl;
        std::cout << "L2 (link 1): " << L2 << " mm" << std::endl;
        std::cout << "L3 (link 2): " << L3 << " mm" << std::endl;
        std::cout << "L4 (link 3): " << L4 << " mm" << std::endl;
        std::cout << "=====================\n" << std::endl;
    }
    
    /**
     * @brief Callback for joint state updates
     */
    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (msg->position.size() < 4) {
            RCLCPP_WARN(this->get_logger(), "Not enough joint angles received");
            return;
        }
        
        // Update joint state
        theta1_ = msg->position[0];
        theta2_ = msg->position[1];
        theta3_ = msg->position[2];
        theta4_ = msg->position[3];
        have_joint_state_ = true;
        
        // Compute forward kinematics
        std::vector<double> ee_pos = calculateFK(theta1_, theta2_, theta3_, theta4_);
        
        // Display results
        std::cout << "===========================================\n";
        std::cout << "Joint Angles (rad):\n";
        std::cout << "  theta1 = " << theta1_ << "\n";
        std::cout << "  theta2 = " << theta2_ << "\n";
        std::cout << "  theta3 = " << theta3_ << "\n";
        std::cout << "  theta4 = " << theta4_ << "\n";
        std::cout << "-------------------------------------------\n";
        std::cout << "End Effector Position (mm):\n";
        std::cout << "  x = " << ee_pos[0] << "\n";
        std::cout << "  y = " << ee_pos[1] << "\n";
        std::cout << "  z = " << ee_pos[2] << "\n";
        std::cout << "===========================================\n\n";
    }
    
    /**
     * @brief Send joint position command to robot
     */
    bool sendJointCommand(const std::vector<double>& positions, double path_time = 5.0) {
        // Wait for service to be available
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return false;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting...");
        }
        
        // Create request
        auto request = std::make_shared<open_manipulator_msgs::srv::SetJointPosition::Request>();
        request->planning_group = "";
        request->joint_position.joint_name = {"joint1", "joint2", "joint3", "joint4", "gripper"};
        request->joint_position.position = positions;
        request->path_time = path_time;
        
        RCLCPP_INFO(this->get_logger(), "Sending joint command...");
        
        // Send request
        auto result = client_->async_send_request(request);
        
        // Wait for result
        if (rclcpp::spin_until_future_complete(this->shared_from_this(), result) ==
            rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Joint command sent successfully");
            
            // Calculate and display expected FK for the target position
            std::vector<double> expected_pos = calculateFK(
                positions[0], positions[1], positions[2], positions[3]);
            std::cout << "\nExpected End-Effector Position:\n";
            std::cout << "  x = " << expected_pos[0] << " mm\n";
            std::cout << "  y = " << expected_pos[1] << " mm\n";
            std::cout << "  z = " << expected_pos[2] << " mm\n\n";
            
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to send joint command");
            return false;
        }
    }
    
    /**
     * @brief Send multiple joint commands in sequence
     */
    void sendMultipleCommands(const std::vector<std::vector<double>>& command_sequence, 
                              double path_time = 5.0) {
        for (size_t i = 0; i < command_sequence.size(); ++i) {
            RCLCPP_INFO(this->get_logger(), "Executing command %zu of %zu", i+1, command_sequence.size());
            sendJointCommand(command_sequence[i], path_time);
            
            // Wait for the motion to complete
            rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int>(path_time * 1000)));
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<RobotControlWithFK>();
    
    // Example 1: Send a single joint command
    std::vector<double> target_positions = {1.0, 0.0, 0.0, 0.0, 0.0};
    node->sendJointCommand(target_positions, 5.0);
    
    // Example 2: Send multiple commands in sequence (commented out)
    // std::vector<std::vector<double>> command_sequence = {
    //     {0.0, 0.0, 0.0, 0.0, 0.0},   // Home position
    //     {1.0, 0.5, -0.5, 0.0, 0.0},  // Position 1
    //     {0.5, 1.0, 0.5, 0.5, 0.0},   // Position 2
    //     {0.0, 0.0, 0.0, 0.0, 0.0}    // Return home
    // };
    // node->sendMultipleCommands(command_sequence, 3.0);
    
    // Spin to receive joint state feedback and compute FK
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    return 0;
}