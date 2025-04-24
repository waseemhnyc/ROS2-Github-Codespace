import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        self.takeoff_publisher = self.create_publisher(
            Empty, 
            '/simple_drone/takeoff', 
            10
        )
        self.land_publisher = self.create_publisher(
            Empty, 
            '/simple_drone/land', 
            10
        )
        self.cmd_vel_publisher = self.create_publisher(
            Twist, 
            '/simple_drone/cmd_vel', 
            10
        )
        self.get_logger().info('Drone controller initialized')

    def takeoff(self):
        self.get_logger().info('Taking off...')
        msg = Empty()
        self.takeoff_publisher.publish(msg)

    def land(self):
        self.get_logger().info('Landing...')
        msg = Empty()
        self.land_publisher.publish(msg)

    def move(self, x=0.0, y=0.0, z=0.0, angular_z=0.0):
        self.get_logger().info(f'Moving: x={x}, y={y}, z={z}, angular_z={angular_z}')
        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.z = float(angular_z)
        self.cmd_vel_publisher.publish(msg)

def main():
    rclpy.init()
    controller = DroneController()
    
    try:
        # Take off
        controller.takeoff()
        time.sleep(2.0)  # Wait for takeoff to complete
        
        # Hover for 3 seconds
        controller.get_logger().info('Hovering...')
        time.sleep(3.0)
        
        # Move up
        controller.get_logger().info('Moving up...')
        controller.move(z=0.5)  # Positive z is up
        time.sleep(2.0)  # Let the movement command complete
        
        # Hover at higher altitude for 5 seconds
        controller.get_logger().info('Hovering at higher altitude...')
        controller.move(z=0.0)  # Stop vertical movement
        time.sleep(5.0)
        
        # Land
        controller.land()
        time.sleep(3.0)  # Wait for landing to complete
        
        controller.get_logger().info('Flight completed!')
        
    finally:
        # Make sure we always clean up
        rclpy.shutdown()

if __name__ == '__main__':
    main()