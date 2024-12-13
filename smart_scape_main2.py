# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import Movel
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
import json 
import serial
import time
import struct
from std_srvs.srv import SetBool
from std_srvs.srv import Empty


class RobotArmController(Node):
    def __init__(self):
        super().__init__('robot_arm_controller')
        
        # Subscriber to read current position
        self.subscription = self.create_subscription(
            Pose,
            '/rm_driver/udp_arm_position',
            self.position_callback,
            10
        )
        self.current_position = None

        # Publisher to send linear motion commands
        self.publisher = self.create_publisher(
            Movel,
            '/rm_driver/movel_cmd',
            10
        )
        
        # Create services for each position
        self.create_service(Empty, 'go_to_initial_position', self.go_to_initial_position_service)
        self.create_service(Empty, 'go_to_mid_belt_position', self.go_to_mid_belt_position_service)
        self.create_service(Empty, 'go_up_from_mid_belt_position', self.go_up_from_mid_belt_position_service)
        self.create_service(Empty, 'go_to_cup_A_position', self.go_to_cup_A_service)
        self.create_service(Empty, 'go_to_cup_B_position', self.go_to_cup_B_service)


    def position_callback(self, msg):
        """Callback to store the current position of the robot arm."""
        self.current_position = msg

    def get_current_position(self):
        """Get the current position of the robot arm."""
        while self.current_position is None:
            rclpy.spin_once(self)
        return self.current_position

    def send_linear_motion(self, pose, speed=20, block=True):
        """Send a linear motion command to the robot arm."""
        command = Movel()
        command.pose = pose
        command.speed = speed
        command.block = block
        self.publisher.publish(command)
        self.get_logger().info(f"Sent linear motion command to position: {pose.position}")
        
    # Define methods for each movement
    def go_to_initial_position(self):
        pose = Pose()
        pose.position.x = -0.2401369959115982
        pose.position.y = -0.04086500033736229
        pose.position.z = 0.4443739950656891
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=20, block=True)

    def go_to_mid_belt_position(self):
        pose = Pose()
        pose.position.x = -0.4330799877643585 + 0.034
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + 0.01
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=90, block=True)

    def go_up_from_mid_belt_position(self):
        pose = Pose()
        pose.position.x = -0.4330799877643585 + 0.034
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + 0.05
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=90, block=True)
        
    def go_to_cup_A_position(self):
        pose = Pose()
        pose.position.x = -0.418300986289978
        pose.position.y = 0.1383100003004074
        pose.position.z = 0.25126200914382935
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=90, block=True)
        
    def go_to_cup_B_position(self):
        pose = Pose()
        pose.position.x = -0.318300986289978
        pose.position.y = 0.1383100003004074
        pose.position.z = 0.25126200914382935
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=90, block=True)
        
    # Define service callbacks
    def go_to_initial_position_service(self, request, response):
        self.go_to_initial_position()
        return response

    def go_to_mid_belt_position_service(self, request, response):
        self.go_to_mid_belt_position()
        return response

    def go_up_from_mid_belt_position_service(self, request, response):
        self.go_up_from_mid_belt_position()
        return response
    
    def go_to_cup_A_service(self, request, response):
        self.go_to_cup_A_position()
        return response
    
    def go_to_cup_B_service(self, request, response):
        self.go_to_cup_B_position()
        return response

class BeltSpeedSubscriber(Node):
    def __init__(self):
        super().__init__('belt_speed_subscriber')
        
        # Subscriber for belt speed
        self.subscription = self.create_subscription(
            Float32,
            '/belt_speed',
            self.belt_speed_callback,
            10
        )
        self.belt_speed = None

    def belt_speed_callback(self, msg):
        """Callback to store the current belt speed."""
        self.belt_speed = msg.data
        #self.get_logger().info(f"Current belt speed: {self.belt_speed}")

    def get_belt_speed(self):
        """Retrieve the current belt speed."""
        return self.belt_speed
    

class GripperController(Node):
    def __init__(self):
        super().__init__('Gripper_controller')
        
        # Client to control the gripper
        self.gripper_client = self.create_client(SetBool, '/set_gripper_position')
        
        # Wait for the service to be available
        if not self.gripper_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service '/set_gripper_position' not available.")
            raise RuntimeError("Gripper service not available.")

    def send_gripper_command(self, open_gripper: bool):
        """Send a command to open or close the gripper."""
        request = SetBool.Request()
        request.data = open_gripper
        future = self.gripper_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f"Gripper command success: {response.success}, message: '{response.message}'")
        else:
            self.get_logger().error("Gripper command failed.")
            
class TCPPacketSubscriber(Node):
    def __init__(self):
        super().__init__('TCP_packet_subscriber')
        
        # Subscriber for TCP_packet
        self.subscription = self.create_subscription(
            String,
            '/rv_message_topic',
            self.TCP_packet_callback,
            10
        )
        self.TCP_packet = None

    def TCP_packet_callback(self, msg):
        """Callback to store the current TCP_packet."""
        #self.TCP_packet = msg.data
        
        # Parse the JSON string from the message
        self.TCP_packet = json.loads(msg.data)
        self.get_logger().info(f"Current TCP_packet: {self.TCP_packet}")

    def get_TCP_packet(self):
        """Retrieve the current TCP_packet."""
        return self.TCP_packet


def main():
    rclpy.init()

    # Create the robot arm controller node
    arm_controller = RobotArmController()
    belt_speed_subscriber = BeltSpeedSubscriber()
    gripper_controller = GripperController()
    TCP_packet_subscriber = TCPPacketSubscriber()
    
    
    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    executor.add_node(belt_speed_subscriber)
    executor.add_node(gripper_controller)
    executor.add_node(TCP_packet_subscriber)


    try:
        
        # Start the executor in a separate thread
        import threading
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        # Example: Control the arm in the main thread
        TCP_packet_subscriber.get_logger().info("packet...")
        packet = TCP_packet_subscriber.get_TCP_packet()
        print(packet)
        
        '''# Example: Control the arm in the main thread
        arm_controller.get_logger().info("Moving to initial position...")
        arm_controller.go_to_initial_position()
        
        time.sleep(2)

        arm_controller.get_logger().info("Moving to mid-belt position...")
        arm_controller.go_to_mid_belt_position()
        
        time.sleep(2)
        
        '''

    finally:
        arm_controller.destroy_node()
        belt_speed_subscriber.destroy_node()
        gripper_controller.destroy_node()
        TCP_packet_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()