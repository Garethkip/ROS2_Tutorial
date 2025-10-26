import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
import threading
import matplotlib.pyplot as plt

class SineWaveSub(Node):
    def __init__(self):
        super().__init__('Wave_Subscriber')
        self.subscribe = self.create_subscription(Float32, 'wave', self.sub_callback, 10)
        self.isdatareceived = False
        self.datareceived = None
        self.logging_time = 15
        
    def sub_callback(self, msg):
        self.datareceived = msg.data
        self.isdatareceived = True
          
    def data_logging(self):
        rate = self.create_rate(10)
        logged_data = []
        
        while (self.isdatareceived == False):
            rate.sleep()
        self.get_logger().info('Logging has Begun ! ')
        
        t = 0.0
        while(t<self.logging_time):
            logged_data.append(self.datareceived)
            t+=0.1
            rate.sleep()
        self.destroy_node
        
        plt.plot(logged_data)
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.show()
        rclpy.shutdown()
        
def main(args = None):
    rclpy.init(args = args)
    node = SineWaveSub()
    t = threading.Thread(target = node.data_logging)
    t.start()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
    
            