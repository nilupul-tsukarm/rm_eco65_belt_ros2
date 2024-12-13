# Import necessary libraries
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped, TransformStamped
from rm_ros_interfaces.msg import Movel
from std_msgs.msg import Bool, Float32, String
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import SetBool, Empty
import json
import tf2_ros
import time


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

    # Define methods for specific movements
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

    def go_to_mid_belt_position(self, y_offset, rotation):
        pose = Pose()
        pose.position.x = -0.4330799877643585 + y_offset
        pose.position.y = -0.048670001327991486
        pose.position.z = 0.2012850046157837 + 0.01
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
                        "size": size,
                        "label": label
                    })
                    self.processed_ids.add(id_)

            # Publish new block data
            if new_blocks:
                self.block_data_pub.publish(String(data=json.dumps(new_blocks)))

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")


class LegoTracker(Node):
    def __init__(self, arm_controller):
        super().__init__('lego_tracker')
        self.belt_speed_sub = self.create_subscription(Float32, '/belt_speed', self.belt_speed_callback, 10)
        self.block_data_sub = self.create_subscription(String, '/lego_blocks', self.block_data_callback, 10)
        self.position_pub = self.create_publisher(PointStamped, '/lego_position', 10)
        
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.arm_controller = arm_controller  # Reference to the arm controller

        # Configurable parameters
        self.belt_speed = 0.0
        self.z_offset = 0.2
        self.y_offset = -0.5   ## arm base belt mid line dis on rviz
        self.block_lifetime = 0.6   # .6m 
        self.pickup_boundary = 0.4
        self.blocks = {}
        
        # Timer to periodically update blocks
        self.timer = self.create_timer(0.01, self.update_blocks)  # Update every 0.1 seconds


    def belt_speed_callback(self, msg):
        self.belt_speed = msg.data
        self.belt_speed = 10.0 
        
        
    def block_data_callback(self, msg):
        """Handle incoming block data from TCPPacketSubscriber."""
        #self.get_logger().info(f" XXXXX XXXX XXXX block_data_callback: {msg}")
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
            #self.get_logger().info(f" blocks: {block}")
            if block_id not in self.blocks:
                self.blocks[block_id] = {"x": x_offset, "y": y_offset, "time": current_time}

    def update_blocks(self):
        current_time = self.get_clock().now()
        for block_id in list(self.blocks.keys()):
            start_x = self.blocks[block_id]["x"]
            start_y = self.blocks[block_id]["y"]
            time_elapsed = current_time - self.blocks[block_id]["time"]
            # Convert time_elapsed from Duration to seconds (float)
            time_elapsed_seconds = time_elapsed.nanoseconds / 1e9
            distance_moved = self.belt_speed / 1000.0 * time_elapsed_seconds
            
            
            if distance_moved > self.block_lifetime :
                del self.blocks[block_id]
                continue
            
            if distance_moved > self.pickup_boundary :
                self.arm_controller.go_up_from_mid_belt_position()
                self.arm_controller.go_to_mid_belt_position(0,90)
                self.arm_controller.go_to_cup_A_position()
                self.arm_controller.go_up_from_mid_belt_position()
                del self.blocks[block_id]
                time.sleep(1)
            
            start_point_x = - 0.5
            
            #current_x = start_point_x + start_x + distance_moved
            current_x = start_point_x + distance_moved
            
            starting_x_shift = start_y / 10  # pix val / 10 = mm >>>>>  (pix val / 10) /1000
             
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
            tf_msg.child_frame_id = f"lego_{block_id}"
            
            tf_msg.transform.translation.x = -0.45 + starting_x_shift    ## arm base belt mid line dis on rviz  -0.4 for mid belt /////  -0.4 - 0.05 = -0.45
            tf_msg.transform.translation.y = current_x 
            tf_msg.transform.translation.z = 0.12 # belt height
            tf_msg.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tf_msg)

    '''def timer_callback(self):
        mock_tcp_data = [{"block_id": "1", "x_offset": 0.0}]
        self.track_blocks(mock_tcp_data)
        self.update_blocks()'''


def main():
    rclpy.init()

    arm_controller = RobotArmController()
    lego_tracker = LegoTracker(arm_controller)
    tcp_packet_subscriber = TCPPacketSubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(arm_controller)
    executor.add_node(lego_tracker)
    executor.add_node(tcp_packet_subscriber)

    try:
        executor.spin()
    finally:
        arm_controller.destroy_node()
        lego_tracker.destroy_node()
        tcp_packet_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
