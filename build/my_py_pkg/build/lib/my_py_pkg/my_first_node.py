# import the python library for ROS2 which contains the node class
import rclpy
from rclpy.node import Node

#create a class that inherits from Node
class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_node') #name of the node
        self.get_logger().info('Hello, ROS2!') 
def main(args=None):
    rclpy.init(args=None) #initialize the ROS2 communication
    node=MyFirstNode() #create an instance of the class
    rclpy.spin(node) #keep the node alive to listen for messages
    rclpy.shutdown() #shutdown the ROS2 communication
if __name__ == '__main__':
    main()