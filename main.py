import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


# ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.8}}"

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_vel_publisher')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.publish_velocity)

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 2.0   # forward speed
        msg.angular.z = 1.8  # turn speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    node = TurtlePublisher()

    try:
        rclpy.spin(node)  # Keep the node alive and publishing
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
