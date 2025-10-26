import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class basicSubscriber(Node):
    def __init__(self):
        super().__init__('number_counter')
        self.counter_ = 0
        self.subscriber = self.create_subscription(Int64, 'counter_number', self.counter_callback,10)
        self.get_logger().info('Number subscriber has started.')
    def counter_callback(self, msg: Int64):
        self.counter_ += msg.data
        self.get_logger().info('Counter Value: ' + str(self.counter_))
        
def main(args=None):
    rclpy.init(args=None)
    node = basicSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()