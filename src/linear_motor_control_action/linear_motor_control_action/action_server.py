import rclpy
import math
import time
from rclpy.action import ActionServer
from rclpy.node import Node
from action_pkg.action import LinearControl

class LinearMotorControlActionServer(Node):
    def __init__(self):
        super().__init__('linear_motor_control_action_server')
        self._action_server = ActionServer(
            self,
            LinearControl,
            'linear_motor_control',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        current_position = goal_handle.request.initial_position
        goal_position = goal_handle.request.goal_position
        speed = goal_handle.request.linear_velocity

        dist = abs(goal_position - current_position)
        feedback_msg = LinearControl.Feedback()

        while dist > 0.01:
            if (current_position < goal_position) > 0:
                current_position += speed * 0.1  # Simulate movement
            else:
                current_position -= speed * 0.1  # Simulate movement
            dist = abs(goal_position - current_position)
            feedback_msg.distance = dist
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)

        goal_handle.succeed()

        result = LinearControl.Result()
        result.motion_done= True
        
        return result

def main(args=None):
    rclpy.init(args=args)

    action_server = LinearMotorControlActionServer()

    rclpy.spin(action_server)

if __name__ == '__main__':
    main()