import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from builtin_interfaces.msg import Duration
from robotis_interfaces.msg import MoveL

from rclpy.qos import qos_profile_sensor_data

class RobotIKControlNode(Node):
    def __init__(self):
        super().__init__('robot_ik_control_node')
        
        # Parameters
        self.declare_parameter('yolo_target_topic', '/target_workspace_coord')
        self.declare_parameter('omx_control_topic', '/omx_movel_controller/movel')
        self.declare_parameter('move_duration_sec', 3)
        self.declare_parameter('move_duration_nanosec', 0)

        # Get parameter values
        self.yolo_target_topic = self.get_parameter('yolo_target_topic').value
        self.omx_control_topic = self.get_parameter('omx_control_topic').value
        self.move_sec = self.get_parameter('move_duration_sec').value
        self.move_nanosec = self.get_parameter('move_duration_nanosec').value
        
        # Publisher for OpenManipulator MoveL command
        self.movel_pub = self.create_publisher(MoveL, self.omx_control_topic, 10)
        
        # Subscriber for YOLO transformed coordinates
        self.target_sub = self.create_subscription(
            PointStamped,
            self.yolo_target_topic,
            self.target_callback,
            qos_profile_sensor_data
        )
        
        self.get_logger().info(f"Robot IK Control Node started.")
        self.get_logger().info(f"Listening to YOLO coords on: {self.yolo_target_topic}")
        self.get_logger().info(f"Publishing MoveL commands to: {self.omx_control_topic}")

    def target_callback(self, msg: PointStamped):
        # Create MoveL message
        movel_msg = MoveL()
        
        # Set the target pose
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.header.frame_id = msg.header.frame_id  # Use the frame_id from YOLO coordinate
        
        # Set Position (x, y, z) from YOLO
        pose_stamped.pose.position.x = msg.point.x
        pose_stamped.pose.position.y = msg.point.y
        pose_stamped.pose.position.z = msg.point.z
        
        # Set Orientation (Assuming a fixed orientation pointing downward or forward)
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.7071
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 0.7071
        
        movel_msg.pose = pose_stamped
        
        # Set movement duration
        duration = Duration()
        duration.sec = self.move_sec
        duration.nanosec = self.move_nanosec
        movel_msg.time_from_start = duration
        
        # Publish the command
        self.movel_pub.publish(movel_msg)
        
        self.get_logger().info(
            f"Published MoveL -> Target: [x: {msg.point.x:.3f}, y: {msg.point.y:.3f}, z: {msg.point.z:.3f}], "
            f"Duration: {self.move_sec}s"
        )

def main(args=None):
    rclpy.init(args=args)
    node = RobotIKControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
