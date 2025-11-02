import rclpy
from rclpy.node import Node
import time
from srv_pkg.srv import JointConversion

#A service server is like a topic subscriber
class JointConversionServer(Node):
    def __init__(self):
        super().__init__('Joint_Conversion_Server')
        self.server = self.create_service(JointConversion, 'Joints' ,self.jointCallback)
        self.offset = [0.0, 0.2, 0.1, 0.5, 2.0, 0.3, 0.1]
        print("Server initialized ... ")
        
    def jointCallback(self, request, response):
        print("New Service request called ! ")
        # Extract joint data from request
        joints = [msg.data for msg in request.joint_input]
        # # Convert the request from a vector to a Float32 list
        # for i in range(len(request.joint_input)):
        #     # joints[i] = request.joint_input[i].data
        #     joints.append(request.joint_input[i].data[i]) #This was the issue
        
        #Apply the offset and convert to degrees
        result =[ x + y for x, y in zip(self.offset, joints) ]
        result = [(x*180.0)/3.1415 for x in result] 
          
        #Assign Vector to response
        for i in range(len(result)):
            response.joint_output[i].data = result[i] 
        time.sleep(2)
        print("Conversion Performed")
        return response

def main(args = None):
    rclpy.init(args = args)
    Conversion = JointConversionServer()
    rclpy.spin(Conversion)
    rclpy.shutdown()

if __name__ == '__main__':
    main()