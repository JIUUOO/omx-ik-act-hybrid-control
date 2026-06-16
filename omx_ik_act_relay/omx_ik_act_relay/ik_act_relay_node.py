import select
import sys
import termios
import threading
import tty

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import ParameterType
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class IkActRelayNode(Node):
    def __init__(self):
        super().__init__('ik_act_relay')

        self.declare_parameter(
            'input_names',
            ['ik', 'act'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'input_topics',
            ['/ik/joint_trajectory', '/act/joint_trajectory'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter('output_topic', '/leader/joint_trajectory')
        self.declare_parameter(
            'publish_keys',
            ['8', '0'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'publish_key_sources',
            ['ik', 'act'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )
        self.declare_parameter(
            'stop_keys',
            ['9'],
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY),
        )

        self.input_names = self.get_parameter('input_names').value
        self.input_topics = self.get_parameter('input_topics').value
        self.output_topic = self.get_parameter('output_topic').value
        self.publish_keys = self.get_parameter('publish_keys').value
        self.publish_key_sources = self.get_parameter('publish_key_sources').value
        self.stop_keys = self.get_parameter('stop_keys').value

        self.input_topic_by_name = self._build_input_topic_map()
        self.source_by_key = self._build_key_source_map()
        self.stop_key_set = self._build_stop_key_set()
        self.latest_msg_by_name = {}
        self.subscribers = []
        self.active_source_name = None

        self.output_pub = self.create_publisher(
            JointTrajectory,
            self.output_topic,
            10,
        )

        for name, topic in self.input_topic_by_name.items():
            sub = self.create_subscription(
                JointTrajectory,
                topic,
                lambda msg, source_name=name: self._trajectory_callback(
                    source_name,
                    msg,
                ),
                10,
            )
            self.subscribers.append(sub)
            self.get_logger().info(f'Subscribing {name}: {topic}')

        self.get_logger().info(f'Publishing selected command to: {self.output_topic}')
        self.get_logger().info(f'Keyboard map: {self._format_key_map()}')
        self.get_logger().info(f'Stop keys: {self._format_stop_keys()}')
        self.get_logger().info('Default state: relay stopped. No command is published.')

        self._start_keyboard_listener()

    def _build_input_topic_map(self):
        if len(self.input_names) != len(self.input_topics):
            self.get_logger().error(
                'input_names and input_topics must have the same length.'
            )
            return {}

        input_topic_by_name = {}
        for name, topic in zip(self.input_names, self.input_topics):
            if not name or not topic:
                self.get_logger().warning(
                    f'Skipping invalid input mapping name="{name}", topic="{topic}".'
                )
                continue
            input_topic_by_name[name] = topic
        return input_topic_by_name

    def _build_key_source_map(self):
        if len(self.publish_keys) != len(self.publish_key_sources):
            self.get_logger().error(
                'publish_keys and publish_key_sources must have the same length.'
            )
            return {}

        source_by_key = {}
        for key, source_name in zip(self.publish_keys, self.publish_key_sources):
            if not key or not source_name:
                self.get_logger().warning(
                    f'Skipping invalid key mapping key="{key}", source="{source_name}".'
                )
                continue
            source_by_key[key[0]] = source_name
        return source_by_key

    def _build_stop_key_set(self):
        return {key[0] for key in self.stop_keys if key}

    def _format_key_map(self):
        return ', '.join(
            f'"{key}" -> relay {source_name}'
            for key, source_name in self.source_by_key.items()
        )

    def _format_stop_keys(self):
        return ', '.join(f'"{key}"' for key in sorted(self.stop_key_set))

    def _trajectory_callback(self, source_name, msg):
        self.latest_msg_by_name[source_name] = msg
        if source_name == self.active_source_name:
            self.output_pub.publish(msg)

    def _set_active_source(self, source_name):
        if source_name not in self.input_topic_by_name:
            self.get_logger().warning(
                f'Key is mapped to unknown source "{source_name}". Check YAML.'
            )
            return

        self.active_source_name = source_name
        self.get_logger().info(
            f'Relay mode selected: "{source_name}" -> {self.output_topic}.'
        )

        msg = self.latest_msg_by_name.get(source_name)
        if msg is not None:
            self.output_pub.publish(msg)
        else:
            self.get_logger().info(
                f'Waiting for first JointTrajectory from "{source_name}".'
            )

    def _stop_relay(self):
        previous_source_name = self.active_source_name
        self.active_source_name = None
        self.latest_msg_by_name.clear()
        self.get_logger().info(
            f'Relay stopped and cached trajectories cleared. '
            f'Previous source: {previous_source_name or "none"}.'
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
            daemon=True,
        )
        thread.start()

    def _keyboard_listener(self, input_stream, close_input_stream):
        old_settings = termios.tcgetattr(input_stream)
        try:
            tty.setcbreak(input_stream.fileno())
            while rclpy.ok():
                readable, _, _ = select.select([input_stream], [], [], 0.1)
                if not readable:
                    continue
                key = input_stream.read(1)
                if key in self.stop_key_set:
                    self._stop_relay()
                    continue

                source_name = self.source_by_key.get(key)
                if source_name is not None:
                    self._set_active_source(source_name)
        finally:
            termios.tcsetattr(input_stream, termios.TCSADRAIN, old_settings)
            if close_input_stream:
                input_stream.close()


def main(args=None):
    rclpy.init(args=args)
    node = IkActRelayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
