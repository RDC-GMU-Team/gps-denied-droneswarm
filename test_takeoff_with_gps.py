#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL 
import time

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        self.current_state = State()
        self.connected = False

        # Create subscriber for MAVROS state (which includes heartbeat info)
        self.subscription = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, 'mavros/cmd/takeoff')


        while not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Arming service not available, waiting...')
        while not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Set mode service not available, waiting...')
        while not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Takeoff service not available, waiting...')
        
        self.get_logger().info("Ardupilot Controller ready")

    def state_callback(self, msg):
        """Callback function that processes incoming state messages"""
        self.get_logger().info(f"MAVROS State Received:")
        self.get_logger().info(f"  Connected: {msg.connected}")
        self.get_logger().info(f"  Armed: {msg.armed}")
        self.get_logger().info(f"  Guided: {msg.guided}")
        self.get_logger().info(f"  Mode: {msg.mode}")
        self.get_logger().info(f"  System Status: {msg.system_status}")
        self.get_logger().info("---")

    def set_mode(self, mode):
        request = SetMode.Request()
        request.custom_mode = mode

        future = self.set_mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if future.result().mode_sent:
                self.get_logger().info(f"Flight mode changed to: {mode}" )
                return True
            else:
                self.get_logger().error("Failed to set flight mode")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False

    def arm_vehicle(self):
        """Arm the vehicle"""
        request = CommandBool.Request()
        request.value = True
        
        future = self.arming_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            if future.result().success:
                self.get_logger().info("Vehicle armed!")
                return True
            else:
                self.get_logger().error("Failed to arm vehicle")
                return False
        else:
            self.get_logger().error("Service call failed")
            return False

    def takeoff(self, altitude=5.0):
        """Execute takeoff sequence"""
        self.get_logger().info("Starting takeoff sequence...")
        request = CommandTOL.Request()
        request.altitude = altitude    # Altitude in meters
        request.latitude = float('nan')  # NaN indicates local frame takeoff
        request.longitude = float('nan') # NaN indicates local frame takeoff
        request.min_pitch = 0.0
        request.yaw = 0.0
        # Step 1: Set to GUIDED mode
        if not self.set_mode("GUIDED"):
            return False
        
        time.sleep(2)
        
        # Step 2: Arm the vehicle
        if not self.arm_vehicle():
            return False
        
        time.sleep(2)
        
        # Step 3: Run the take off mavros command (for ArduPilot)
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None and future.result().success:
            self.get_logger().info(f"local takeoff command sent")
            return True
        else:
            self.get_logger().error(f"Local takeoff command failed")
            return False

def main():
    rclpy.init()
    
    # Create the controller class
    controller = Controller()
    try:
        controller.takeoff(altitude=10.0)
        rclpy.spin(controller)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
