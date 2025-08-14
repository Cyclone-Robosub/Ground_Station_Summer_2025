#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayLayout
from std_msgs.msg import MultiArrayDimension
import os

class PositionPublisher(Node):
    """
    A ROS2 node that reads position data from a text file and publishes it.

    The data is read from a file named 'position.txt' where each line
    contains three space-separated floating-point numbers (x, y, z).
    The data is published as a std_msgs/msg/Float32MultiArray message.
    """

    def __init__(self):
        # Initialize the Node with the name 'position_publisher'
        super().__init__('position_publisher')
        
        # Declare and get the file path from a parameter. 
        # This makes the file path configurable without changing the code.
        self.declare_parameter('file_path', 'position.txt')
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value
        
        # Create a publisher on the 'position_topic' with a queue size of 10.
        # The message type is std_msgs/msg/Float32MultiArray.
        self.publisher_ = self.create_publisher(Float32MultiArray, 'position_topic', 10)
        
        # A list to store the position data from the file.
        self.positions = []
        
        # Index to keep track of the current position to publish.
        self.current_index = 0

        # Read the data from the file once when the node starts.
        self.read_data_from_file()

        # Create a timer to call the timer_callback function every 1.0 seconds.
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('Position publisher node has been started. Publishing to "position_topic" every 1.0 seconds.')

    def read_data_from_file(self):
        """
        Reads the x, y, z data from the specified text file.
        """
        if not os.path.exists(self.file_path):
            self.get_logger().error(f"File not found: {self.file_path}")
            # Do not proceed if the file doesn't exist.
            self.destroy_node()
            rclpy.shutdown()
            return

        self.get_logger().info(f"Reading data from: {self.file_path}")
        try:
            with open(self.file_path, 'r') as file:
                for line in file:
                    try:
                        # Split the line by spaces, filter out empty strings, and convert to float.
                        parts = line.strip().split()
                        if len(parts) == 3:
                            x, y, z = map(float, parts)
                            self.positions.append((x, y, z))
                    except (ValueError, IndexError) as e:
                        self.get_logger().warn(f"Skipping malformed line: '{line.strip()}' - Error: {e}")
        except IOError as e:
            self.get_logger().error(f"Could not read file: {e}")
            self.destroy_node()
            rclpy.shutdown()
    
    def timer_callback(self):
        """
        Callback function for the timer.
        It publishes the next position from the list.
        """
        if not self.positions:
            self.get_logger().warn('No position data to publish. Shutting down.')
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()
            return
            
        if self.current_index < len(self.positions):
            # Get the current position data.
            x, y, z = self.positions[self.current_index]
            
            # Create a new Float32MultiArray message and populate it.
            msg = Float32MultiArray()
            
            # Populate the layout field to describe the data structure.
            # Here, it's a 1D array of 3 elements.
            msg.layout = MultiArrayLayout(
                dim=[MultiArrayDimension(label="x_y_z", size=3, stride=3)],
                data_offset=0
            )

            # Assign the data as a list of floats.
            msg.data = [float(x), float(y), float(z)]
            
            # Publish the message.
            self.publisher_.publish(msg)
            
            # Log the published data for verification.
            self.get_logger().info(f'Publishing: data={msg.data}')
            
            # Move to the next position in the list.
            self.current_index += 1
        else:
            self.get_logger().info('All positions have been published. Stopping the timer and shutting down.')
            # Stop the timer once all data is published.
            self.timer.cancel()
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)
    position_publisher = PositionPublisher()
    rclpy.spin(position_publisher)
    
    # After rclpy.spin() returns, the node is destroyed, so we shutdown.
    rclpy.shutdown()

if __name__ == '__main__':
    main()
