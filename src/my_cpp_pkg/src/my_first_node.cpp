#include "rclcpp/rclcpp.hpp"

class MyNode: public rclcpp::Node
{
    public:
        MyNode():Node("cpp_test"), counter_(0)
        {
            RCLCPP_INFO(this->get_logger(),"Hello ROS2 cpp");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
                                            std::bind(&MyNode::timerCallback, this));
        } 

    private:
        void timerCallback()
        {
            counter_++;
            RCLCPP_INFO(this->get_logger(),"Hello %d", counter_ );
        }

        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    // auto node = std::make_shared<rclcpp::Node>("cpp_test");
    auto node = std::make_shared<MyNode>();
    // RCLCPP_INFO(node->get_logger(),"Hello ROS2 cpp");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}