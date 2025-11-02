import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64, String
from msgs_pkg.msg import SineWave
import math

class customPub(Node):
    def __init__(self):
        super().__init__('Custom_msg_Publisher')
        self.pub = self.create_publisher(SineWave, 'sine_wave', 10)
        self.timer = self.create_timer(1.0, self.publish_sine_wave)
        self.get_logger().info('Custom Sine Wave Publisher Node has started')
        self.frequency = 1.0  # Frequency in Hz
        self.amplitude = 1.0  # Amplitude of the sine wave
        self.time_elapsed = 0.0
    def publish_sine_wave(self):
        msg = SineWave()
        msg.frequency.data = self.frequency
        msg.amplitude.data = self.amplitude
        msg.signal.data = self.amplitude * math.sin(2 * 3.14159 * self.frequency * self.time_elapsed)
        self.pub.publish(msg)
        self.get_logger().info(f'Published Sine Wave: Frequency={msg.frequency.data}, Amplitude={msg.amplitude.data}, Signal={msg.signal.data}')
        self.time_elapsed += 1.0  # Increment time by 1 second for each publish

def main(args=None):
        rclpy.init(args=None)
        node = customPub()
        rclpy.spin(node)
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()