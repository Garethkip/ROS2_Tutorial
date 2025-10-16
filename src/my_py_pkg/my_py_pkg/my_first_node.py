# import the python library for ROS2 which contains the node class
import rclpy
from rclpy.node import Node

#create a class that inherits from Node
class MyFirstNode(Node):
    def __init__(self):
        super().__init__('my_node') #name of the node
        self.counter_=0
        self.timer_=self.create_timer(1.0,self.printHello)
        self.get_logger().info('Hello, ROS2!') 
    def printHello(self):
        self.get_logger().info('Hello ' + str(self.counter_))
        self.counter_+=1
def main(args=None):
    rclpy.init(args=None) #initialize the ROS2 communication
    node=MyFirstNode() #create an instance of the class
    rclpy.spin(node) #keep the node alive to listen for messages
    rclpy.shutdown() #shutdown the ROS2 communication
if __name__ == '__main__':
    main()