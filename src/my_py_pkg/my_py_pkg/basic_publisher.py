import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64, String

class basicPub(Node):
    def __init__(self):
        super().__init__('Basic_Publisher')
        
        self.counter_= 0
        self.publish_String = self.create_publisher(String, 'counter_string', 10)
        self.publish_Number = self.create_publisher(Int64, 'counter_number', 10)
        self.timer = self.create_timer(1.0, self.pub)
        self.get_logger().info('Publisher Node has started')
        
    def pub(self):
        num = Int64()
        num.data = self.counter_
        msg = String()
        msg.data = "Number received is : "+ str(self.counter_)
        self.publish_String.publish(msg)
        self.publish_Number.publish(num)
        self.counter_+=1
    
def main(args=None):
        rclpy.init(args=None)
        node = basicPub()
        rclpy.spin(node)
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()