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
# Function to calculate CRC16
def calculate_crc(data):
    crc = 0xFFFF
    for pos in data:
        crc ^= pos
        for _ in range(8):
            if crc & 1:
                crc >>= 1
                crc ^= 0xA001
            else:
                crc >>= 1
    return crc

# Function to build the command with CRC
def build_command(address, function, register, value):
    command = struct.pack('>BBHH', address, function, register, value)
    crc = calculate_crc(command)
    command += struct.pack('<H', crc)
    return command

# Function to convert decimal to hexadecimal (with return as integer for struct)
def decimal_to_hex(value):
    if not isinstance(value, int):
        raise ValueError("Input must be an integer")
    if value < 0 or value > 0xFFFF:
        raise ValueError("Input must be between 0 and 65535")
    return value

# Serial configuration
ser = serial.Serial(
    port='/dev/ttyUSB0',
    baudrate=115200,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=2
)

# Set position using decimal input
def set_position(decimal_value):
    hex_value = decimal_to_hex(decimal_value)
    position_command = build_command(0x01, 0x06, 0x0103, hex_value)
    ser.write(position_command)
    response = ser.read(8)
    print(f"Position {decimal_value} Response:", response)

# Open the serial connection
if ser.isOpen():
    print("Connection established successfully")
else:
    print("Failed to open serial connection")
    exit()

# Initialization command
initialize_command = b'\x01\x06\x01\x00\x00\x01\x49\xF6'
reinitialize_command = b'\x01\x06\x01\x00\x00\x00\xA5\x48'

# Send initialization command
ser.write(initialize_command)
response = ser.read(8)
print("Initialization Response:", response)
time.sleep(1)

# Send reinitialization command
ser.write(reinitialize_command)
response = ser.read(8)
print("Reinitialization Response:", response)

time.sleep(1)

# Speed command (example, may need adjustment depending on your needs)
speed_command = build_command(0x01, 0x06, 0x0104, 0x0032)
ser.write(speed_command)
response = ser.read(8)
print("Speed Response:", response)
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

# Main function to execute the script

rclpy.init()

# Create the robot arm controller node
arm_controller = RobotArmController()
belt_speed_subscriber = BeltSpeedSubscriber()


# Use a MultiThreadedExecutor to run both nodes concurrently
executor = MultiThreadedExecutor()
executor.add_node(arm_controller)
executor.add_node(belt_speed_subscriber)
#while rclpy.ok():
rclpy.spin_once(belt_speed_subscriber, timeout_sec=0.1)
if belt_speed_subscriber.get_belt_speed() is not None:
    print(f"Current Belt Speed: {belt_speed_subscriber.get_belt_speed()}")
#try:
    # Get the current position
current_position = arm_controller.get_current_position()
print(current_position)
'''print("Current Position:")
print(f"x: {current_position.position.x}, y: {current_position.position.y}, z: {current_position.position.z}")
print(f"Orientation - x: {current_position.orientation.x}, y: {current_position.orientation.y}, "
        f"z: {current_position.orientation.z}, w: {current_position.orientation.w}")'''


set_position(1000) # open >>>>> 0(full close ) to 1000(full open)
################ GO TO initial position ####################
callib_pose = Pose()
callib_pose.position.x = -0.2401369959115982
callib_pose.position.y = -0.04086500033736229
callib_pose.position.z = 0.4443739950656891
# Define orientation correctly as a Quaternion
#new_pose.orientation = Quaternion()
callib_pose.orientation.x = 0.9998310208320618
callib_pose.orientation.y = -0.010149000212550163
callib_pose.orientation.z = 0.012520000338554382
callib_pose.orientation.w = 0.00880299974232912
    # Send the linear motion command
arm_controller.send_linear_motion(callib_pose, speed=20, block=True)

time.sleep(4)


'''################ GO TO calibration position ####################
callib_pose = Pose()
callib_pose.position.x = -0.4330799877643585 
callib_pose.position.y = -0.048670001327991486
callib_pose.position.z = 0.2012850046157837 + 0.01
# Define orientation correctly as a Quaternion
#new_pose.orientation = Quaternion()
callib_pose.orientation.x = 0.9998310208320618
callib_pose.orientation.y = -0.010149000212550163
callib_pose.orientation.z = 0.012520000338554382
callib_pose.orientation.w = 0.00880299974232912
    # Send the linear motion command
arm_controller.send_linear_motion(callib_pose, speed=20, block=True)'''


################ GO TO mid belt position ####################
callib_pose = Pose()
callib_pose.position.x = -0.4330799877643585 + 0.034
callib_pose.position.y = -0.048670001327991486
callib_pose.position.z = 0.2012850046157837 + 0.01
# Define orientation correctly as a Quaternion
#new_pose.orientation = Quaternion()
callib_pose.orientation.x = 0.9998310208320618
callib_pose.orientation.y = -0.010149000212550163
callib_pose.orientation.z = 0.012520000338554382
callib_pose.orientation.w = 0.00880299974232912
    # Send the linear motion command
arm_controller.send_linear_motion(callib_pose, speed=90, block=True)

time.sleep(2)

set_position(800) # close 0(full close ) to 1000(full open)
################ GO up from mid belt position ####################
callib_pose = Pose()
callib_pose.position.x = -0.4330799877643585 + 0.034
callib_pose.position.y = -0.048670001327991486
callib_pose.position.z = 0.2012850046157837  + 0.05
# Define orientation correctly as a Quaternion
#new_pose.orientation = Quaternion()
callib_pose.orientation.x = 0.9998310208320618
callib_pose.orientation.y = -0.010149000212550163
callib_pose.orientation.z = 0.012520000338554382
callib_pose.orientation.w = 0.00880299974232912

# Send the linear motion command
arm_controller.send_linear_motion(callib_pose, speed=90, block=True)

time.sleep(2)

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
    # Send the linear motion command
arm_controller.send_linear_motion(dropA_pose, speed=90, block=True)

time.sleep(2)

set_position(1000) # close 0(full close ) to 1000(full open)
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
    # Send the linear motion command
arm_controller.send_linear_motion(dropB_pose, speed=90, block=True)

time.sleep(2)

set_position(1000) # close 0(full close ) to 1000(full open)
#finally:

arm_controller.destroy_node()
belt_speed_subscriber.destroy_node()
rclpy.shutdown()


'''# Define a new position for linear motion (move slightly in the Z direction)
new_pose = Pose()
new_pose.position.x = current_position.position.x + 0.05
new_pose.position.y = current_position.position.y 
new_pose.position.z = current_position.position.z   # Move up by 5 cm
new_pose.orientation = current_position.orientation

    # Send the linear motion command
    #arm_controller.send_linear_motion(new_pose, speed=20, block=True)

#finally:
arm_controller.destroy_node()
rclpy.shutdown()'''


'''# Main function to execute the script
def main():
    rclpy.init()

    # Create the robot arm controller node
    arm_controller = RobotArmController()

    #try:
        # Get the current position
    current_position = arm_controller.get_current_position()
    print("Current Position:")
    print(f"x: {current_position.position.x}, y: {current_position.position.y}, z: {current_position.position.z}")
    print(f"Orientation - x: {current_position.orientation.x}, y: {current_position.orientation.y}, "
            f"z: {current_position.orientation.z}, w: {current_position.orientation.w}")

    # Define a new position for linear motion (move slightly in the Z direction)
    new_pose = Pose()
    new_pose.position.x = current_position.position.x
    new_pose.position.y = current_position.position.y + 0.05
    new_pose.position.z = current_position.position.z   # Move up by 5 cm
    new_pose.orientation = current_position.orientation

        # Send the linear motion command
        #arm_controller.send_linear_motion(new_pose, speed=20, block=True)

    #finally:
    arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()'''