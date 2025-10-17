#include "rclcpp/rclcpp.hpp"  // include the ROS 2 C++ client library

class MyNode : public rclcpp::Node
{
    public:
        MyNode() : Node("my_cpp_node") , counter_(0) // constructor initializes the node with the name "my_cpp_node"
        {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(1),  // set the timer to trigger every second
                std::bind(&MyNode::print_hello, this)  // bind the callback function
            );
        }
        void print_hello()
        {
            RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);  // log the message "Hello, ROS 2!"
            counter_++;
        }
    private:
    int counter_;
    rclcpp::TimerBase::SharedPtr timer_;  // timer to trigger the callback
};
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);  // initialize the ROS 2 client library
    auto node = std::make_shared<MyNode>();  // create an instance of MyNode
    rclcpp::spin(node);  // create and spin the node
    rclcpp::shutdown();  // shutdown the ROS 2 client library
    return 0;
}
