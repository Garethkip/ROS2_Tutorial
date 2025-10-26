import rclpy
from rclpy.node import Node

class basicNode(Node):
    def __init__(self):
        super().__init__("Basic_node")
        self.get_logger().info("Node has been initialized")

def main(args=None):
    rclpy.init(args=None)
    basic_node = basicNode()
    rclpy.spin(basic_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()