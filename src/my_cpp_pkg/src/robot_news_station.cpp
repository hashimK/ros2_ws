#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"

class RobotNewsStationNode: public rclcpp::Node
{
    public:
        RobotNewsStationNode():Node("robot_new_station"),robot_name_("hash")
        {
            RCLCPP_INFO(this->get_logger(),"Robot news station node created");
            timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&RobotNewsStationNode::timerCallback,this));
            publisher_ = this->create_publisher<example_interfaces::msg::String>("robot_news",10);
        }
    private:
        void timerCallback()
        {
            auto msg = example_interfaces::msg::String();
            msg.data = std::string("Hi this is ")+robot_name_+std::string(" from Mumbai");
            publisher_->publish(msg);
        }
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;
        std::string robot_name_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<RobotNewsStationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}