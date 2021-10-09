#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/manual_control_switches.hpp>

/**
 * @brief Manual Control Switches uORB topic data callback
 */
class ManualControlSwitchesListener : public rclcpp::Node
{
    public:
	explicit ManualControlSwitchesListener() : Node("manual_control_switches_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::ManualControlSwitches>(
			"fmu/manual_control_switches/out",
			10,
			[this](const px4_msgs::msg::ManualControlSwitches::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED manual_control_switches_listener DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			std::cout << "ts: "          << msg->timestamp    << std::endl;
			std::cout << "mode_slot: " << msg->mode_slot  << std::endl;
			std::cout << "arm_switch: " << msg->arm_switch  << std::endl;
			std::cout << "return_switch: " << msg->return_switch  << std::endl;
			std::cout << "loiter_switch: " << msg->loiter_switch << std::endl;
			std::cout << "offboard_switch: " << msg->offboard_switch << std::endl;
			std::cout << "kill_switch: " << msg->kill_switch << std::endl;
			std::cout << "gear_switch: " << msg->gear_switch << std::endl;
			std::cout << "transition_switch: " << msg->transition_switch << std::endl;
		});
	}
    private:
	rclcpp::Subscription<px4_msgs::msg::ManualControlSwitches>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting manual_control_switches listener node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ManualControlSwitchesListener>());

	rclcpp::shutdown();
	return 0;
}