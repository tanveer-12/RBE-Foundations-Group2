#!/usr/bin/env python3
"""
Script to set OpenManipulator-X Actuator 4 to Current Control Mode
Uses Dynamixel SDK to directly communicate with the servo
"""

import sys
try:
    from dynamixel_sdk import *
except ImportError:
    print("Error: dynamixel_sdk not installed")
    print("Install with: pip3 install dynamixel-sdk")
    sys.exit(1)


# Control table addresses for XM430-W350-T (Protocol 2.0)
ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_CURRENT = 102
ADDR_PRESENT_CURRENT = 126

# Operating modes
OPERATING_MODE_CURRENT = 0
OPERATING_MODE_VELOCITY = 1
OPERATING_MODE_POSITION = 3
OPERATING_MODE_EXTENDED_POSITION = 4
OPERATING_MODE_CURRENT_BASED_POSITION = 5

# Communication settings
PROTOCOL_VERSION = 2.0
BAUDRATE = 1000000  # 1 Mbps (typical for OpenManipulator-X)
DEVICENAME = '/dev/ttyUSB0'  # Check your device name

# Dynamixel IDs for OpenManipulator-X
# Joint 1: ID 11
# Joint 2: ID 12
# Joint 3: ID 13
# Joint 4: ID 14 (Actuator 4)
# Gripper: ID 15

ACTUATOR_4_ID = 14


def initialize_port():
    """Initialize port and packet handlers"""
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not portHandler.openPort():
        print(f"Failed to open port {DEVICENAME}")
        print("Common solutions:")
        print("  1. Check if device exists: ls -l /dev/ttyUSB*")
        print("  2. Add user to dialout group: sudo usermod -aG dialout $USER")
        print("  3. Set permissions: sudo chmod 666 /dev/ttyUSB0")
        return None, None
    
    # Set baudrate
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate to {BAUDRATE}")
        return None, None
    
    print(f"Successfully opened port {DEVICENAME} at {BAUDRATE} bps")
    return portHandler, packetHandler


def set_operating_mode(portHandler, packetHandler, dxl_id, mode):
    """
    Set operating mode for a Dynamixel servo
    
    Args:
        portHandler: Port handler object
        packetHandler: Packet handler object
        dxl_id: Dynamixel ID
        mode: Operating mode (0=Current, 1=Velocity, 3=Position, etc.)
    """
    print(f"\nSetting Dynamixel ID {dxl_id} to mode {mode}...")
    
    # 1. Disable torque (required to change operating mode)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, 0
    )
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to disable torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    
    print("  Torque disabled")
    
    # 2. Set operating mode
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_OPERATING_MODE, mode
    )
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to set operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    
    print(f"  Operating mode set to {mode}")
    
    # 3. Re-enable torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
        portHandler, dxl_id, ADDR_TORQUE_ENABLE, 1
    )
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to enable torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return False
    elif dxl_error != 0:
        print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        return False
    
    print("  Torque enabled")
    return True


def verify_operating_mode(portHandler, packetHandler, dxl_id):
    """Read and verify the current operating mode"""
    dxl_present_mode, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
        portHandler, dxl_id, ADDR_OPERATING_MODE
    )
    
    if dxl_comm_result != COMM_SUCCESS:
        print(f"Failed to read operating mode: {packetHandler.getTxRxResult(dxl_comm_result)}")
        return None
    elif dxl_error != 0:
        print(f"Dynamixel error: {packetHandler.getRxPacketError(dxl_error)}")
        return None
    
    mode_names = {
        0: "Current Control",
        1: "Velocity Control",
        3: "Position Control",
        4: "Extended Position Control",
        5: "Current-based Position Control"
    }
    
    mode_name = mode_names.get(dxl_present_mode, f"Unknown ({dxl_present_mode})")
    print(f"Current operating mode: {mode_name}")
    return dxl_present_mode


def main():
    print("="*60)
    print("OpenManipulator-X Actuator 4 - Set to Current Control Mode")
    print("="*60)
    
    # Initialize communication
    portHandler, packetHandler = initialize_port()
    if portHandler is None:
        sys.exit(1)
    
    try:
        # Check current mode
        print("\nChecking current operating mode...")
        current_mode = verify_operating_mode(portHandler, packetHandler, ACTUATOR_4_ID)
        
        if current_mode == OPERATING_MODE_CURRENT:
            print("\nActuator 4 is already in Current Control Mode!")
            response = input("Do you want to reconfigure it anyway? (y/n): ")
            if response.lower() != 'y':
                print("Exiting...")
                portHandler.closePort()
                return
        
        # Set to current control mode
        print("\nConfiguring Actuator 4 for current control...")
        success = set_operating_mode(
            portHandler, 
            packetHandler, 
            ACTUATOR_4_ID, 
            OPERATING_MODE_CURRENT
        )
        
        if success:
            print("\n" + "="*60)
            print("SUCCESS: Actuator 4 is now in Current Control Mode!")
            print("="*60)
            print("\nYou can now run the PD controller:")
            print("  ros2 run open_manipulator_pd_control pd_controller_node.py")
            print("\nIMPORTANT: Other actuators should remain in Position Control Mode")
            print("           to hold their positions rigidly during testing.")
        else:
            print("\n" + "="*60)
            print("FAILED: Could not set operating mode")
            print("="*60)
            print("\nTroubleshooting:")
            print("  1. Check connections to the robot")
            print("  2. Verify Dynamixel ID (should be 14 for Actuator 4)")
            print("  3. Check power supply to the servos")
            print("  4. Try power cycling the robot")
        
        # Verify the change
        print("\nVerifying configuration...")
        verify_operating_mode(portHandler, packetHandler, ACTUATOR_4_ID)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Close port
        portHandler.closePort()
        print("\nPort closed")


if __name__ == '__main__':
    main()
