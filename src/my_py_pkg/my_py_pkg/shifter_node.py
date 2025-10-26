import rclpy
from rclpy.node import Node 

class Shifter(Node):
    def __init__(self):
        super().__init__("Shifter")
        self.get_logger().info('Shifter shifting')
        self.list = [[1,2,3,4,5],
                     [6,7,8,9,0]] 
        self.i = 0
        self.timer = self.create_timer(1.0,self.shift)
    def shift(self):
        self.list[0].append(self.list[1][-1])
        self.list[1].insert(0,self.list[0][0])
        self.list[0].pop(0)
        self.list[1].pop(-1)

        self.get_logger().info(str(self.list[0]))
        self.get_logger().info(str(self.list[1]))

def main(args = None):
    rclpy.init(args = None)
    node = Shifter()
    rclpy.spin(node)
    rclpy.shutdown()