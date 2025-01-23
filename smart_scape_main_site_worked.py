# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped, TransformStamped
from rm_ros_interfaces.msg import Movel
from std_msgs.msg import Bool, Float32, String
from sensor_msgs.msg import JointState
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool, Empty
import json
import tf2_ros
import time
import struct
import serial
import json

# Helper functions (Your existing functions)
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

def build_command(address, function, register, value):
    command = struct.pack('>BBHH', address, function, register, value)
    crc = calculate_crc(command)
    command += struct.pack('<H', crc)
    return command

def decimal_to_hex(value):
    if not isinstance(value, int):
        raise ValueError("Input must be an integer")
    if value < 0 or value > 0xFFFF:
        raise ValueError("Input must be between 0 and 65535")
    return value

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
        
        # Load parameters from JSON file
        self.positions = self.load_positions_from_json('/home/ubuntu22/rm_eco65_belt_ros2/parameters.json')

    def load_positions_from_json(self, file_path):
        """Load position data from a JSON file."""
        try:
            with open(file_path, 'r') as file:
                data = json.load(file)
            self.get_logger().info("Successfully loaded positions from JSON file.")
            return data.get("positions", {})
        except Exception as e:
            self.get_logger().error(f"Failed to load positions from JSON file: {e}")
            return {}

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

    def go_to_mid_belt_position(self, y_offset, z_offset, rotation):
        """Move to the position specified for belt_center_down ."""
        belt_center_down_pos_data = self.positions.get(f"belt_center_down", {})
        if not belt_center_down_pos_data:
            self.get_logger().error(f"No position data found for belt_center_down.")
            return
        
        pose = Pose()
        """ #Original
        pose.position.x = -0.4330799877643585 + y_offset
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + z_offset"""
        
        #pose.position.x = -0.4430799877643585 + y_offset ## edge value  -0.4330799877643585,
        pose.position.x = belt_center_down_pos_data.get("x", 0.0) + y_offset
        pose.position.y = belt_center_down_pos_data.get("y", 0.0)
        pose.position.z = belt_center_down_pos_data.get("z", 0.0) + z_offset
        
        if rotation == 90:
            pose.orientation.x = 0.9998310208320618
            pose.orientation.y = -0.010149000212550163
            pose.orientation.z = 0.012520000338554382
            pose.orientation.w = 0.00880299974232912
        else:
            pose.orientation.x = 0.7152109742164612
            pose.orientation.y = 0.6987419724464417
            pose.orientation.z = 0.015018999576568604
            pose.orientation.w = -0.002392000053077936
        self.send_linear_motion(pose, speed=100, block=True)
        
    def go_up_from_mid_belt_position(self):
        """Move to the position specified for belt_center_up ."""
        belt_center_up_pos_data = self.positions.get(f"belt_center_up", {})
        if not belt_center_up_pos_data:
            self.get_logger().error(f"No position data found for belt_center_up.")
            return
        
        pose = Pose()
        """ #Original
        pose.position.x = -0.4330799877643585 + 0.034
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + 0.05"""
        pose.position.x = belt_center_up_pos_data.get("x", 0.0)
        pose.position.y = belt_center_up_pos_data.get("y", 0.0)
        pose.position.z = belt_center_up_pos_data.get("z", 0.0)
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912
        self.send_linear_motion(pose, speed=100, block=True)

    def go_to_cup_position(self, cup_ID):
        """Move to the position specified for a cup (0, 1, 2 or 3)."""
        position_data = self.positions.get(f"cup_{cup_ID}", {})
        if not position_data:
            self.get_logger().error(f"No position data found for cup {cup_ID}.")
            return

        pose = Pose()
        pose.position.x = position_data.get("x", 0.0)
        pose.position.y = position_data.get("y", 0.0)
        pose.position.z = 0.25126200914382935
        # Set default orientation or load from JSON if needed
        pose.orientation.x = 0.9998310208320618
        pose.orientation.y = -0.010149000212550163
        pose.orientation.z = 0.012520000338554382
        pose.orientation.w = 0.00880299974232912

        self.send_linear_motion(pose, speed=100, block=True)

class TCPPacketSubscriber(Node):
    def __init__(self):
        super().__init__('TCP_packet_subscriber')
        
        # Subscriber for TCP data
        self.subscription = self.create_subscription(
            String,
            '/rv_message_topic',
            self.TCP_packet_callback,
            10
        )

        # Publisher for parsed block data
        self.block_data_pub = self.create_publisher(
            String,
            '/lego_blocks',
            10
        )

        self.processed_ids = set()  # Track processed IDs

    def TCP_packet_callback(self, msg):
        try:
            # Parse the JSON string from the message
            packet = json.loads(msg.data)
            #self.get_logger().info(f"res pack: {msg}")
            ids = packet.get('ids', [])
            centers = packet.get('centers', [])
            sizes = packet.get('sizes', [])
            labels = packet.get('labels', [])

            new_blocks = []
            for idx, id_ in enumerate(ids):
                if id_ not in self.processed_ids:
                    center = centers[idx] if idx < len(centers) else {"x": 0, "y": 0}
                    size = sizes[idx] if idx < len(sizes) else {"x": 0, "y": 0}
                    label = labels[idx] if idx < len(labels) else -1
                    new_blocks.append({
                        "block_id": id_,
                        "x_offset": center["x"],
                        "y_offset": center["y"],
                        "x_size": size["x"],
                        "y_size": size["y"],
                        "label": label
                    })
                    self.processed_ids.add(id_)

            # Publish new block data
            if new_blocks:
                self.block_data_pub.publish(String(data=json.dumps(new_blocks)))

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")


class LegoTracker(Node):
    def __init__(self, arm_controller,gripper_controller):
        super().__init__('lego_tracker')
        self.belt_speed_sub = self.create_subscription(Float32, '/belt_speed', self.belt_speed_callback, 10)
        self.block_data_sub = self.create_subscription(String, '/lego_blocks', self.block_data_callback, 10)
        self.position_pub = self.create_publisher(PointStamped, '/lego_position', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.arm_controller = arm_controller  # Reference to the arm controller
        self.arm_controller.go_up_from_mid_belt_position()
        
        self.gripper_controller = gripper_controller

        # Configurable parameters
        self.belt_speed = 0.0
        self.z_offset = 0.2
        self.y_offset = -0.5   ## arm base belt mid line dis on rviz
        self.block_lifetime = 0.3   # .6m  
        self.blocks = {}
        
        # Timer to periodically update blocks
        self.timer = self.create_timer(0.01, self.update_blocks)  # Update every 0.1 seconds


    def belt_speed_callback(self, msg):
        self.belt_speed = abs(msg.data)
        # self.belt_speed = 10.0 
        
        
    def block_data_callback(self, msg):
        """Handle incoming block data from TCPPacketSubscriber."""
        try:
            block_data = json.loads(msg.data)
            self.track_blocks(block_data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse block data: {e}")

    def track_blocks(self, tcp_data):
        current_time = self.get_clock().now()
        #self.get_logger().info(f" track_blocks: {tcp_data}")
        for block in tcp_data:
            block_id = block["block_id"]
            x_offset = block["x_offset"]
            y_offset = block["y_offset"]
            x_size = block["x_size"]
            y_size = block["y_size"]
            label = block["label"]
            self.get_logger().info(f" x_size: {x_size}")
            self.get_logger().info(f" y_size: {y_size}")
            if block_id not in self.blocks:
                self.blocks[block_id] = {"x": x_offset, "y": y_offset, "y_size": y_size,"label": label, "time": current_time}

    def update_blocks(self):
        current_time = self.get_clock().now()
        for block_id in list(self.blocks.keys()):
            block = self.blocks[block_id]
            start_x = block["x"]
            start_y = block["y"]
            y_size = block["y_size"]
            label = block["label"]
            time_elapsed = current_time - self.blocks[block_id]["time"]
            # Convert time_elapsed from Duration to seconds (float)
            time_elapsed_seconds = time_elapsed.nanoseconds / 1e9
            distance_moved = self.belt_speed / 1000.0 * time_elapsed_seconds
            
            #starting_x_shift = start_y / 8  #   original 10 >>> 8     pix val / 10 = mm >>>>>  (pix val / 10) /1000
            starting_x_shift_belt = start_y / 10 # beli wide effect
            starting_y_shift_belt = start_x / 10 # detectin time effect
            print("starting_y_shift_belt >> ",starting_y_shift_belt)
            
            if(starting_y_shift_belt>0.05):
                print("after line")
                #self.pickup_boundary = 0.295 - starting_y_shift_belt/5
                self.pickup_boundary = 0.29 - starting_y_shift_belt/5
                print("decreesd new pickup_boundary >> ",self.pickup_boundary)
            else:
                print("before line")
                #self.pickup_boundary = 0.295 + starting_y_shift_belt/5
                self.pickup_boundary = 0.29 + starting_y_shift_belt/5
                print("incressd new pickup_boundary >> ",self.pickup_boundary)
            
            
            
            if "processed" not in block:
                block["processed"] = False
            
            print("dis = ", distance_moved)
            
            #self.pickup_boundary = 0.295   ## main abjustment point  0.28>>
            print("size = ", y_size) 
            
            
            if not block["processed"] and self.pickup_boundary/2 < distance_moved:
                block["processed"] = True
                if y_size > 0.26 : # large
                    print("ready rotate")
                    self.arm_controller.go_to_mid_belt_position(starting_x_shift_belt,0.017,0) ##  original 0 >>> 90
                else:
                    print("ready no rotate")
                    self.arm_controller.go_to_mid_belt_position(starting_x_shift_belt,0.017,90)
            
            if self.pickup_boundary < distance_moved :
                if y_size > 0.26 : # large
                    print("go down rotate")
                    time.sleep(0.7)
                    self.arm_controller.go_to_mid_belt_position(starting_x_shift_belt,0.0,0)
                    time.sleep(0.1)
                    self.gripper_controller.set_position(1) #gripper close
                    time.sleep(0.3)
                    self.arm_controller.go_to_cup_position(label)
                    time.sleep(1)
                    self.gripper_controller.set_position(0) #gripper open
                    self.arm_controller.go_up_from_mid_belt_position()
                    del self.blocks[block_id]
                else:
                    ("go down no rotate")
                    self.arm_controller.go_to_mid_belt_position(starting_x_shift_belt,0.0,90)   ##(starting_x_shift,90)
                    time.sleep(0.1)
                    self.gripper_controller.set_position(1) #gripper close
                    time.sleep(0.3)
                    self.arm_controller.go_to_cup_position(label)
                    time.sleep(1)
                    self.gripper_controller.set_position(0) #gripper open
                    self.arm_controller.go_up_from_mid_belt_position()
                    del self.blocks[block_id]
                
            
            start_point_x = - 0.35
            
            #current_x = start_point_x + starting_y_shift_belt + distance_moved
            current_x = start_point_x + distance_moved
             
            point_msg = PointStamped()
            point_msg.header.stamp = self.get_clock().now().to_msg()
            point_msg.header.frame_id = "baselink"
            point_msg.point.x = -0.45  ## arm base belt mid line dis on rviz
            point_msg.point.y = current_x
            point_msg.point.z = 0.12   # belt height
            self.position_pub.publish(point_msg)
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = "baselink"
            tf_msg.child_frame_id = f"lego-{block_id} : label-{label}"
            
            tf_msg.transform.translation.x = -0.45 + starting_x_shift_belt    ## arm base belt mid line dis on rviz  -0.4 for mid belt /////  -0.4 - 0.05 = -0.45
            tf_msg.transform.translation.y = current_x 
            tf_msg.transform.translation.z = 0.12 # belt height
            tf_msg.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_msg)

    '''def timer_callback(self):
        mock_tcp_data = [{"block_id": "1", "x_offset": 0.0}]
        self.track_blocks(mock_tcp_data)
        self.update_blocks()'''

class GripperControlNode(Node):
    def __init__(self):
        super().__init__('gripper_control')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        # Initialize Serial Communication
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        try:
            self.ser = serial.Serial(port=serial_port, baudrate=baud_rate, timeout=2)
            if self.ser.isOpen():
                self.get_logger().info("Gripper serial connection established.")
            else:
                raise serial.SerialException("Failed to open serial port.")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection error: {e}")
            exit()

        # ROS Services and Publishers
        self.publisher = self.create_publisher(JointState, 'gripper_joint_states', 10)
        #self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.timer = self.create_timer(0.1, self.publish_joint_states)  # 50 Hz (every 20 ms)

        
        # Initialize Gripper
        self.gripper_position = 0  # Start with a default position (e.g., 0)

        # Initialize Gripper
        self.initialize_gripper()
        self.initialize_speed()
        

    def initialize_gripper(self):
        init_command = b'\x01\x06\x01\x00\x00\x01\x49\xF6'
        self.ser.write(init_command)
        response = self.ser.read(8)
        self.get_logger().info(f"Gripper Initialization Response: {response}")
        
    def initialize_speed(self):
        # Speed command (example, may need adjustment depending on your needs)
        speed_command = build_command(0x01, 0x06, 0x0104, 0x0032)
        max_speed_command = build_command(0x01, 0x06, 0x0104, 0x0064)
        speed_90_command = build_command(0x01, 0x06, 0x0104, 0x005A)
        self.ser.write(max_speed_command)
        response = self.ser.read(8)
        print("Speed Response:", response)

    def set_position(self, state):
        if state :  # 1 = true open
            decimal_value = 600
        else:
            decimal_value = 1000
        hex_value = decimal_to_hex(decimal_value)
        self.gripper_position = decimal_value
        position_command = build_command(0x01, 0x06, 0x0103, hex_value)
        self.ser.write(position_command)
        response = self.ser.read(8)
        self.get_logger().info(f"Gripper Position {decimal_value} Response: {response}")

    def publish_joint_states(self):
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['gripper_joint']
        joint_state.position = [self.gripper_position / 1.0]  # Normalize or adjust if necessary
        self.publisher.publish(joint_state)

    def destroy_node(self):
        self.ser.close()
        self.get_logger().info("Serial connection closed.")
        super().destroy_node()

def main():
    rclpy.init()

    arm_controller = RobotArmController()
    gripper_controller = GripperControlNode()
    lego_tracker = LegoTracker(arm_controller,gripper_controller)
    tcp_packet_subscriber = TCPPacketSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    executor.add_node(gripper_controller)
    executor.add_node(lego_tracker)
    executor.add_node(tcp_packet_subscriber)

    try:
        executor.spin()
    finally:
        gripper_controller.destroy_node()
        arm_controller.destroy_node()
        lego_tracker.destroy_node()
        tcp_packet_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
