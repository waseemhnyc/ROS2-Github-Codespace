import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class SimpleWallAvoidingRobot(Node):
    def __init__(self):
        super().__init__('simple_wall_avoiding_robot')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscriber = self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        
        # Add a state variable to track if we're rotating
        self.is_rotating = False
        # Add a counter to track how long we've been rotating
        self.rotation_counter = 0

    def pose_callback(self, msg):
        robots_x_position = msg.x
        robots_y_position = msg.y

        x_min = 0.5
        y_min = 0.5

        x_max = 10.5
        y_max = 10.5
        
        # Add a small threshold to detect approaching the wall before hitting it
        threshold = 0.5
        
        near_left_wall = robots_x_position < (x_min + threshold)
        near_right_wall = robots_x_position > (x_max - threshold)
        near_bottom_wall = robots_y_position < (y_min + threshold)
        near_top_wall = robots_y_position > (y_max - threshold)

        # Check if we're near any wall
        if (near_left_wall or near_right_wall or near_bottom_wall or near_top_wall):
            # Start rotating and reset counter
            self.is_rotating = True
            self.rotation_counter = 0
            
            # Log which wall we're near for debugging
            if near_left_wall:
                self.get_logger().info(f'Near left wall at x={robots_x_position}')
            if near_right_wall:
                self.get_logger().info(f'Near right wall at x={robots_x_position}')
            if near_bottom_wall:
                self.get_logger().info(f'Near bottom wall at y={robots_y_position}')
            if near_top_wall:
                self.get_logger().info(f'Near top wall at y={robots_y_position}')
        
        # Check if we're rotating
        if self.is_rotating:
            # Increment the rotation counter
            self.rotation_counter += 1
            
            # Create a Twist message for rotation
            rot_msg = Twist()
            rot_msg.linear.x = 0.0  # Stop forward motion
            rot_msg.angular.z = 1.5  # Rotate counter-clockwise
            
            # After rotating for some time, start moving forward again
            if self.rotation_counter > 20:  # Adjust this value to control rotation duration
                self.is_rotating = False
                self.get_logger().info('Rotation complete, moving forward')
                
                # Add a small forward movement to get away from the wall
                rot_msg.linear.x = 1.0
            
            self.publisher.publish(rot_msg)
        else:
            # If not rotating, move forward
            move_msg = Twist()
            move_msg.linear.x = 1.0  # Forward speed
            move_msg.angular.z = 0.0  # No rotation
            self.publisher.publish(move_msg)

def main():
    rclpy.init()

    node = SimpleWallAvoidingRobot()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    # Make sure to stop the turtle before shutting down
    stop_msg = Twist()
    node.publisher.publish(stop_msg)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
