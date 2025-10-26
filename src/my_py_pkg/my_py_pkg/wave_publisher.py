import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
import math

class sineWave(Node):
    def __init__(self):
        super().__init__('Sine_Wave_Node')
        self.publisher = self.create_publisher(Float32, 'wave',10)
        self.timer = self.create_timer(0.1, self.publishWave)
        self.amplitude = 2
        self.frequency = 0.2
        self.time = 0
        self.get_logger().info('Wave publisher has started !')
        
    def publishWave(self):
        msg = Float32()
        msg.data = self.amplitude * math.sin(2*math.pi*self.frequency*self.time) # A*Sin(2*pi*f*t)
        self.publisher.publish(msg)
        self.time += 0.1

def main(args = None):
    rclpy.init(args = args)
    node = sineWave()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
