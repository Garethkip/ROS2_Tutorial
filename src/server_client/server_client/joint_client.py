import sys
import rclpy
from rclpy.node import Node
from srv_pkg.srv import JointConversion
from std_msgs.msg import Float32

#A client behaves like an publisher
class JointConversionClient(Node):
    def __init__(self):
        super().__init__('Joint_Conversion_Client')
        self.client = self.create_client(JointConversion, 'Joints')
        print('Client Node has started ...')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again ...')
        
    def send_request(self, j_values):
        req = JointConversion.Request() # Create request object
        req.joint_input = [Float32(data=value) for value in j_values]
        # for i in range(len(j_values)):
        #     req.joint_input[i].data = j_values[i]
        self.future = self.client.call_async(req) # Send request asynchronously
        
        rclpy.spin_until_future_complete(self, self.future)
        # print(self.future) 
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    conversion_client = JointConversionClient()
    
    joint_values = [1.0, 0.78, 1.6, 0.0, 1.57, 0.3, 1.2]
    response = conversion_client.send_request(joint_values)
    
    print("Received converted joint values:")
    print([msg.data for msg in response.joint_output])
    
    conversion_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()