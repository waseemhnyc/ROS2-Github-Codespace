import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtleDance(Node):
    def __init__(self):
        super().__init__('turtle_dance')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        self.commands = [
            {'linear': 2.0, 'angular': 0.0, 'duration': 2.0, 'desc': 'Move forward'},
            {'linear': 0.0, 'angular': 2.0, 'duration': 1.5, 'desc': 'Turn left'},
            {'linear': -2.0, 'angular': 0.0, 'duration': 2.0, 'desc': 'Move backward'},
            {'linear': 0.0, 'angular': -2.0, 'duration': 1.5, 'desc': 'Turn right'},
            {'linear': 0.0, 'angular': 0.0, 'duration': 0.5, 'desc': 'Stop'},
        ]

        self.current_cmd_index = 0
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)

    def update(self):
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9

        if self.current_cmd_index >= len(self.commands):
            self.get_logger().info("Finished all movements 🎉")
            self.destroy_node()
            return

        current_cmd = self.commands[self.current_cmd_index]

        if elapsed < current_cmd['duration']:
            msg = Twist()
            msg.linear.x = current_cmd['linear']
            msg.angular.z = current_cmd['angular']
            self.publisher_.publish(msg)
            self.get_logger().info(f"{current_cmd['desc']} - linear.x={msg.linear.x}, angular.z={msg.angular.z}")
        else:
            self.current_cmd_index += 1
            self.start_time = now  # Reset timer for next command


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDance()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
