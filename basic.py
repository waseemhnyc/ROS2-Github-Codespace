import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_vel_publisher')
        self.vcat_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.run_now)

    def run_now(self):
        msg = Twist()
        msg.linear.x = 5.0
        msg.angular.z = 0.0
        self.vcat_publisher.publish(msg)
        print(f'Published: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()

    node = TurtlePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
