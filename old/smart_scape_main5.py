#more simple without gripper and pos servises
# simple tcp

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

    def go_to_mid_belt_position(self,y_offset,rotation):
        pose = Pose()
        #pose.position.x = -0.4330799877643585 + 0.034 + y_offset
        pose.position.x = -0.4330799877643585 + y_offset
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + 0.01
        if rotation == 90 :
            pose.orientation.x = 0.9998310208320618
            pose.orientation.y = -0.010149000212550163
            pose.orientation.z = 0.012520000338554382
            pose.orientation.w = 0.00880299974232912
        else:
            pose.orientation.x = 0.7152109742164612
            pose.orientation.y = 0.6987419724464417
            pose.orientation.z = 0.015018999576568604
            pose.orientation.w = -0.002392000053077936
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
    
            
class TCPPacketSubscriber(Node):
    def __init__(self, arm_controller):
        super().__init__('TCP_packet_subscriber')
        
        # Subscriber for TCP_packet
        self.subscription = self.create_subscription(
            String,
            '/rv_message_topic',
            self.TCP_packet_callback,
            10
        )
        self.processed_ids = set()  # Keep track of processed IDs
        self.arm_controller = arm_controller  # Reference to the arm controller

    def TCP_packet_callback(self, msg):
        """Callback to handle incoming TCP packets."""
        # Parse the JSON string from the message
        try:
            packet = json.loads(msg.data)
            ids = packet.get('ids', [])
            centers = packet.get('centers', [])
            sizes = packet.get('sizes', [])
            labels = packet.get('labels', [])

            # Check for new IDs
            new_ids = [id_ for id_ in ids if id_ not in self.processed_ids]

            if new_ids:         
                # Print the data
                self.get_logger().info(f"New IDs: {new_ids}")
                self.get_logger().info(f"Centers: {centers}")
                self.get_logger().info(f"Sizes: {sizes}")
                self.get_logger().info(f"Labels: {labels}")

                # Update the processed IDs
                self.processed_ids.update(new_ids)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")


def main():
    rclpy.init()

    # Create the robot arm controller node
    arm_controller = RobotArmController()
    TCP_packet_subscriber = TCPPacketSubscriber(arm_controller)
    belt_speed_subscriber = BeltSpeedSubscriber()
    
    
    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    executor.add_node(TCP_packet_subscriber)
    executor.add_node(belt_speed_subscriber)


    try:
        executor.spin()

    finally:
        arm_controller.destroy_node()
        belt_speed_subscriber.destroy_node()
        TCP_packet_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()