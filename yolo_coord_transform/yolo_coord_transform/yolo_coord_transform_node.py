import select
import sys
import termios
import threading
import tty

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from yolo_msgs.msg import DetectionArray


class YoloCoordTransformNode(Node):
    def __init__(self):
        super().__init__('yolo_coord_transform_node')

        # Declare parameters
        self.declare_parameter('yolo_topic', '/yolo/detections')
        self.declare_parameter('output_topic', '/target_workspace_coord')
        self.declare_parameter('place_output_topic', '/place_workspace_coord')
        self.declare_parameter(
            'target_class_ids',
            [-1],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter(
            'place_class_ids',
            [-1],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter('target_z_offset', 0.18)
        self.declare_parameter('place_z_offset', 0.18)
        self.declare_parameter('frame_id', 'link0')
        self.declare_parameter(
            'transform_matrix',
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        )
        self.declare_parameter('target_min_score', 0.0)
        self.declare_parameter('place_min_score', 0.0)
        self.declare_parameter('require_place_detection', False)
        self.declare_parameter('target_publish_key', 'a')
        self.declare_parameter('place_publish_key', 'd')
        self.declare_parameter(
            'target_publish_keys',
            ['a'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter(
            'target_key_class_ids',
            [-1],
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER_ARRAY)
        )
        self.declare_parameter(
            'target_key_labels',
            ['target'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY)
        )
        self.declare_parameter('reset_key', 'r')
        self.declare_parameter(
            'reset_position',
            [0.03, 0.0, 0.03],
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        )
        self.declare_parameter('reset_frame_id', 'base_link')

        # Get parameter values
        self.yolo_topic = self.get_parameter('yolo_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.place_output_topic = self.get_parameter('place_output_topic').value
        self.target_class_ids = self.get_parameter('target_class_ids').value
        self.place_class_ids = self.get_parameter('place_class_ids').value
        self.target_z_offset = self.get_parameter('target_z_offset').value
        self.place_z_offset = self.get_parameter('place_z_offset').value
        self.frame_id = self.get_parameter('frame_id').value
        matrix_flat = self.get_parameter('transform_matrix').value
        self.target_min_score = self.get_parameter('target_min_score').value
        self.place_min_score = self.get_parameter('place_min_score').value
        self.require_place_detection = self.get_parameter(
            'require_place_detection'
        ).value
        self.target_publish_key = self.get_parameter('target_publish_key').value
        self.place_publish_key = self.get_parameter('place_publish_key').value
        self.target_publish_keys = self.get_parameter('target_publish_keys').value
        self.target_key_class_ids = self.get_parameter('target_key_class_ids').value
        self.target_key_labels = self.get_parameter('target_key_labels').value
        self.reset_key = self.get_parameter('reset_key').value
        reset_position = self.get_parameter('reset_position').value
        self.reset_frame_id = self.get_parameter('reset_frame_id').value
        self.key_target_map = self._build_key_target_map()

        if len(reset_position) != 3:
            self.get_logger().error(
                f'Reset position must have exactly 3 elements. Got {len(reset_position)}'
            )
            self.reset_position = np.array([0.03, 0.0, 0.03], dtype=float)
        else:
            self.reset_position = np.array(reset_position, dtype=float)

        # Convert the flat list to a 3x3 numpy array
        if len(matrix_flat) != 9:
            self.get_logger().error(
                f'Transform matrix must have exactly 9 elements. Got {len(matrix_flat)}'
            )
            self.transform_matrix = np.eye(3)
        else:
            self.transform_matrix = np.array(matrix_flat).reshape(3, 3)

        self.get_logger().info(f'Target Class IDs: {self.target_class_ids}')
        self.get_logger().info(f'Place Class IDs: {self.place_class_ids}')
        self.get_logger().info(f'Target Z Offset: {self.target_z_offset}')
        self.get_logger().info(f'Place Z Offset: {self.place_z_offset}')
        self.get_logger().info(f'Target Min Score: {self.target_min_score}')
        self.get_logger().info(f'Place Min Score: {self.place_min_score}')
        self.get_logger().info(f'Require Place Detection: {self.require_place_detection}')
        self.get_logger().info(
            f'Target Publish Key: {self.target_publish_key} -> {self.output_topic}'
        )
        self.get_logger().info(
            f'Place Publish Key: {self.place_publish_key} -> {self.output_topic}'
        )
        self.get_logger().info(
            f'Target Key Map: {self._format_key_target_map()} -> {self.output_topic}'
        )
        self.get_logger().info(f'Reset Key: {self.reset_key}')
        self.get_logger().info(
            f'Reset Position: x={self.reset_position[0]:.3f}, '
            f'y={self.reset_position[1]:.3f}, z={self.reset_position[2]:.3f}'
        )
        self.get_logger().info(f'Reset Frame ID: {self.reset_frame_id}')
        self.get_logger().info(f'Transform Matrix:\n{self.transform_matrix}')

        self._missing_place_logged = False
        self._latest_target_point = None
        self._latest_place_point = None
        self._latest_points_by_class_id = {}

        # Publisher for workspace coordinates
        self.coord_pub = self.create_publisher(PointStamped, self.output_topic, 10)
        self.place_coord_pub = self.create_publisher(
            PointStamped,
            self.place_output_topic,
            10
        )

        # Subscriber to YOLO detections
        self.yolo_sub = self.create_subscription(
            DetectionArray,
            self.yolo_topic,
            self.yolo_callback,
            qos_profile_sensor_data
        )

        self._start_keyboard_listener()

    def yolo_callback(self, msg: DetectionArray):
        now = self.get_clock().now()
        target_detection = self._select_detection(
            msg,
            self.target_class_ids,
            self.target_min_score
        )
        place_detection = self._select_detection(
            msg,
            self.place_class_ids,
            self.place_min_score
        )
        selected_class_ids = set(self.key_target_map.values())

        for class_id in selected_class_ids:
            min_score = (
                self.place_min_score
                if class_id in self.place_class_ids
                else self.target_min_score
            )
            z_offset = (
                self.place_z_offset
                if class_id in self.place_class_ids
                else self.target_z_offset
            )
            detection = self._select_detection(msg, [class_id], min_score)
            if detection is None:
                continue
            point = self._detection_to_workspace_point(detection, z_offset)
            if point is not None:
                self._latest_points_by_class_id[class_id] = point

        if place_detection is not None:
            place_point = self._detection_to_workspace_point(
                place_detection,
                self.place_z_offset
            )
            if place_point is not None:
                self._latest_place_point = place_point
                self._missing_place_logged = False
                place_msg = self._make_point_msg(now, place_point)
                self.place_coord_pub.publish(place_msg)

        if target_detection is not None:
            target_point = self._detection_to_workspace_point(
                target_detection,
                self.target_z_offset
            )
            if target_point is not None:
                self._latest_target_point = target_point

        if self.require_place_detection and self._latest_place_point is None:
            if not self._missing_place_logged:
                self.get_logger().info('Waiting for place detection.')
                self._missing_place_logged = True

    def _build_key_target_map(self):
        if len(self.target_publish_keys) != len(self.target_key_class_ids):
            self.get_logger().error(
                'target_publish_keys and target_key_class_ids must have the same length. '
                'Falling back to target_publish_key/target_class_ids.'
            )
            first_target_class_id = (
                self.target_class_ids[0] if self.target_class_ids else -1
            )
            return {self.target_publish_key: first_target_class_id}

        key_target_map = {}
        for key, class_id in zip(self.target_publish_keys, self.target_key_class_ids):
            if not key:
                self.get_logger().warning(
                    f'Skipping empty publish key for class ID {class_id}.'
                )
                continue
            key_target_map[key[0]] = class_id

        if not key_target_map:
            self.get_logger().error(
                'No valid target publish keys configured. Falling back to target_publish_key.'
            )
            first_target_class_id = (
                self.target_class_ids[0] if self.target_class_ids else -1
            )
            key_target_map[self.target_publish_key] = first_target_class_id

        return key_target_map

    def _label_for_class_id(self, class_id):
        if len(self.target_key_labels) == len(self.target_key_class_ids):
            for configured_class_id, label in zip(
                self.target_key_class_ids,
                self.target_key_labels
            ):
                if configured_class_id == class_id:
                    return label
        return f'class {class_id}'

    def _format_key_target_map(self):
        entries = []
        for key, class_id in self.key_target_map.items():
            entries.append(f'"{key}"={self._label_for_class_id(class_id)}({class_id})')
        return ', '.join(entries)

    def _select_detection(self, msg: DetectionArray, class_ids, min_score):
        target_detections = [
            detection for detection in msg.detections
            if detection.class_id in class_ids and detection.score >= min_score
        ]
        if not target_detections:
            return None
        return max(target_detections, key=lambda detection: detection.score)

    def _detection_to_workspace_point(self, detection, z_offset):
        u = detection.bbox.center.position.x
        v = detection.bbox.center.position.y

        pixel_vector = np.array([u, v, 1.0])
        transformed_vector = np.dot(self.transform_matrix, pixel_vector)

        w_prime = transformed_vector[2]
        if abs(w_prime) < 1e-9:
            self.get_logger().warning(
                'Transform resulted in w=0, cannot normalize. Check your matrix.'
            )
            return None

        world_x = transformed_vector[0] / w_prime
        world_y = transformed_vector[1] / w_prime
        return np.array([world_x, world_y, z_offset], dtype=float)

    def _make_point_msg(self, stamp, point, frame_id=None):
        point_msg = PointStamped()
        point_msg.header.stamp = stamp.to_msg()
        point_msg.header.frame_id = frame_id or self.frame_id
        point_msg.point.x = point[0]
        point_msg.point.y = point[1]
        point_msg.point.z = point[2]
        return point_msg

    def _publish_latest_target(self):
        if self._latest_target_point is None:
            self.get_logger().warning(
                'No target detection has been cached yet. Check target_class_ids and camera view.'
            )
            return

        if self.require_place_detection and self._latest_place_point is None:
            self.get_logger().warning(
                'No place detection has been cached yet. Waiting before publishing target.'
            )
            return

        now = self.get_clock().now()
        point_msg = self._make_point_msg(now, self._latest_target_point)
        self.coord_pub.publish(point_msg)
        self.get_logger().info(
            f'Published cached target coord to target topic: '
            f'x={self._latest_target_point[0]:.3f}, '
            f'y={self._latest_target_point[1]:.3f}, '
            f'z={self._latest_target_point[2]:.3f}'
        )

    def _publish_latest_place_as_target(self):
        if self._latest_place_point is None:
            self.get_logger().warning(
                'No place detection has been cached yet. Check place_class_ids and camera view.'
            )
            return

        now = self.get_clock().now()
        point_msg = self._make_point_msg(now, self._latest_place_point)
        self.coord_pub.publish(point_msg)
        self.get_logger().info(
            f'Published cached place coord to target topic: '
            f'x={self._latest_place_point[0]:.3f}, '
            f'y={self._latest_place_point[1]:.3f}, '
            f'z={self._latest_place_point[2]:.3f}'
        )

    def _publish_latest_class_target(self, class_id):
        point = self._latest_points_by_class_id.get(class_id)
        label = self._label_for_class_id(class_id)
        if point is None:
            self.get_logger().warning(
                f'No {label} detection has been cached yet. Check camera view and class ID {class_id}.'
            )
            return

        if (
            self.require_place_detection
            and class_id not in self.place_class_ids
            and self._latest_place_point is None
        ):
            self.get_logger().warning(
                f'No place detection has been cached yet. Waiting before publishing {label}.'
            )
            return

        now = self.get_clock().now()
        point_msg = self._make_point_msg(now, point)
        self.coord_pub.publish(point_msg)
        self.get_logger().info(
            f'Published cached {label} coord to target topic: '
            f'x={point[0]:.3f}, y={point[1]:.3f}, z={point[2]:.3f}'
        )

    def _publish_reset_position(self):
        now = self.get_clock().now()
        point_msg = self._make_point_msg(
            now,
            self.reset_position,
            self.reset_frame_id
        )
        self.coord_pub.publish(point_msg)
        self.get_logger().info(
            f'Published reset position to target topic: '
            f'x={self.reset_position[0]:.3f}, '
            f'y={self.reset_position[1]:.3f}, '
            f'z={self.reset_position[2]:.3f}'
        )

    def _start_keyboard_listener(self):
        input_stream = sys.stdin
        close_input_stream = False

        if not input_stream.isatty():
            try:
                input_stream = open('/dev/tty', 'r')
                close_input_stream = True
            except OSError:
                self.get_logger().warning(
                    'Keyboard controls are enabled, but no terminal is available.'
                )
                return

        thread = threading.Thread(
            target=self._keyboard_listener,
            args=(input_stream, close_input_stream),
            daemon=True
        )
        thread.start()

    def _keyboard_listener(self, input_stream, close_input_stream):
        old_settings = termios.tcgetattr(input_stream)
        try:
            tty.setcbreak(input_stream.fileno())
            self.get_logger().info(
                self._keyboard_help_text()
            )
            while rclpy.ok():
                readable, _, _ = select.select([input_stream], [], [], 0.1)
                if not readable:
                    continue
                key = input_stream.read(1)
                if key == self.reset_key:
                    self._publish_reset_position()
                elif key in self.key_target_map:
                    self._publish_latest_class_target(self.key_target_map[key])
                elif key == self.target_publish_key:
                    self._publish_latest_target()
                elif key == self.place_publish_key:
                    self._publish_latest_place_as_target()
        finally:
            termios.tcsetattr(input_stream, termios.TCSADRAIN, old_settings)
            if close_input_stream:
                input_stream.close()

    def _keyboard_help_text(self):
        return (
            f'Press {self._format_key_target_map()} to publish cached coords. '
            f'Press "{self.reset_key}" to publish reset position.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloCoordTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
