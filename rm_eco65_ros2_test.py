# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from rm_ros_interfaces.msg import Movel
from std_msgs.msg import Bool
from std_msgs.msg import Float32 
from rclpy.executors import MultiThreadedExecutor
import serial
import time
import struct
from std_srvs.srv import SetBool


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
        self.get_logger().info(f"Current belt speed: {self.belt_speed}")

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


def main():
    rclpy.init()

    # Create the robot arm controller node
    arm_controller = RobotArmController()
    belt_speed_subscriber = BeltSpeedSubscriber()
    gripper_controller = GripperController()
    
    # Use a MultiThreadedExecutor to run both nodes concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    executor.add_node(belt_speed_subscriber)
    #while rclpy.ok():
    rclpy.spin_once(belt_speed_subscriber, timeout_sec=0.1)
    if belt_speed_subscriber.get_belt_speed() is not None:
        print(f"Current Belt Speed: {belt_speed_subscriber.get_belt_speed()}")

    try:
        
        current_position = arm_controller.get_current_position()
        print(current_position)

        ################ GO TO initial position ####################
        callib_pose = Pose()
        callib_pose.position.x = -0.2401369959115982
        callib_pose.position.y = -0.04086500033736229
        callib_pose.position.z = 0.4443739950656891
        
        callib_pose.orientation.x = 0.9998310208320618
        callib_pose.orientation.y = -0.010149000212550163
        callib_pose.orientation.z = 0.012520000338554382
        callib_pose.orientation.w = 0.00880299974232912
            # Send the linear motion command
        arm_controller.send_linear_motion(callib_pose, speed=20, block=True)

        time.sleep(4)

        # Open the gripper
        gripper_controller.send_gripper_command(open_gripper=True)
        time.sleep(1)

        # Close the gripper
        gripper_controller.send_gripper_command(open_gripper=False)
        time.sleep(1)

        ################ GO TO mid belt position ####################
        callib_pose = Pose()
        callib_pose.position.x = -0.4330799877643585 + 0.034
        callib_pose.position.y = -0.048670001327991486
        callib_pose.position.z = 0.2012850046157837 + 0.01
        
        callib_pose.orientation.x = 0.9998310208320618
        callib_pose.orientation.y = -0.010149000212550163
        callib_pose.orientation.z = 0.012520000338554382
        callib_pose.orientation.w = 0.00880299974232912
        
        arm_controller.send_linear_motion(callib_pose, speed=90, block=True)

        time.sleep(2)

        ################ GO up from mid belt position ####################
        callib_pose = Pose()
        callib_pose.position.x = -0.4330799877643585 + 0.034
        callib_pose.position.y = -0.048670001327991486
        callib_pose.position.z = 0.2012850046157837  + 0.05
        
        callib_pose.orientation.x = 0.9998310208320618
        callib_pose.orientation.y = -0.010149000212550163
        callib_pose.orientation.z = 0.012520000338554382
        callib_pose.orientation.w = 0.00880299974232912

        arm_controller.send_linear_motion(callib_pose, speed=90, block=True)

        time.sleep(2)
        
        # Open the gripper
        gripper_controller.send_gripper_command(open_gripper=True)
        time.sleep(1)

        # Close the gripper
        gripper_controller.send_gripper_command(open_gripper=False)
        time.sleep(1)

        ################ Cup A dropping position ####################
        #x=-0.38308998942375183, y=-0.04865499958395958, z=0.20129099488258362
        #-0.418300986289978, y=0.1383100003004074, z=0.25126200914382935
        dropA_pose = Pose()
        dropA_pose.position.x = -0.418300986289978 #
        dropA_pose.position.y = 0.1383100003004074
        dropA_pose.position.z = 0.25126200914382935 
        # Define orientation correctly as a Quaternion
        #x=0.9998319745063782, y=-0.010112999938428402, z=0.012378999963402748, w=0.008905000053346157
        dropA_pose.orientation.x = 0.9998310208320618
        dropA_pose.orientation.y = -0.010149000212550163
        dropA_pose.orientation.z = 0.012520000338554382
        dropA_pose.orientation.w = 0.00880299974232912
            
        arm_controller.send_linear_motion(dropA_pose, speed=90, block=True)

        time.sleep(2)

        ################ Cup B dropping position ####################
        #x=-0.38308998942375183, y=-0.04865499958395958, z=0.20129099488258362
        #-0.418300986289978, y=0.1383100003004074, z=0.25126200914382935
        dropB_pose = Pose()
        dropB_pose.position.x = -0.318300986289978 # -418 >>> -318
        dropB_pose.position.y = 0.1383100003004074
        dropB_pose.position.z = 0.25126200914382935 
        # Define orientation correctly as a Quaternion
        #x=0.9998319745063782, y=-0.010112999938428402, z=0.012378999963402748, w=0.008905000053346157
        dropB_pose.orientation.x = 0.9998310208320618
        dropB_pose.orientation.y = -0.010149000212550163
        dropB_pose.orientation.z = 0.012520000338554382
        dropB_pose.orientation.w = 0.00880299974232912
            
        arm_controller.send_linear_motion(dropB_pose, speed=90, block=True)

        time.sleep(2)

    finally:
        arm_controller.destroy_node()
        belt_speed_subscriber.destroy_node()
        gripper_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()