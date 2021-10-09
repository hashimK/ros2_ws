#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <math.h>
#include <px4_ros_com/frame_transforms.h>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class ManualControl : public rclcpp::Node {
public:
	ManualControl() : Node("manual_control") {
		roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        throttle_ = 0.0f;
		arm_status_ = 0.0;
		prev_arm_status_ = 0.0;
		arm_transition_ = false;

		flight_mode_ = 0.0;
		prev_flight_mode_ = 0.0;
		flight_mode_transition_ = false;

		manual_control_setpoint_publisher_ =
			this->create_publisher<ManualControlSetpoint>("fmu/manual_control_setpoint/in", 10);

		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

        joystick_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cloud_commands",10,
                                    std::bind(&ManualControl::callbackJoystick, this,std::placeholders::_1));
		

		auto timer_callback = [this]() -> void {
			if (arm_transition_==true)
			{
				if (arm_status_>0.0)
				{
					this->arm();
				}
				else
				{
					this->disarm();
				}
				arm_transition_ = false;
			}

			if (flight_mode_transition_==true)
			{
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, flight_mode_);
				flight_mode_transition_ = false;
			}

			publish_manual_control_setpoint();
			
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_setpoint_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_subscriber_;

    double roll_;
    double pitch_;
    double yaw_;
    float throttle_;
	double arm_status_;
	double prev_arm_status_;
	bool arm_transition_;
	double flight_mode_ = 0.0;
	double prev_flight_mode_ = 0.0;
	bool flight_mode_transition_ = true;


	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	void publish_manual_control_setpoint() ;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0) const;


    void callbackJoystick(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pitch_ = msg->linear.x;
        roll_ = msg->linear.y;
        throttle_ = msg->angular.x;
		yaw_ = msg->angular.y;
		arm_status_ = msg->linear.z;
		if (arm_status_!=prev_arm_status_)
		{
			arm_transition_ = true;
			prev_arm_status_ = arm_status_;
		}
		flight_mode_ = msg->angular.z;
		if (flight_mode_!=prev_flight_mode_)
		{
			flight_mode_transition_ = true;
			prev_flight_mode_ = flight_mode_;
		}
    }
};

/**
 * @brief Send a command to Arm the vehicle
 */
void ManualControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void ManualControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void ManualControl::publish_manual_control_setpoint()  {
	ManualControlSetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.data_source = 2;

	// limit controls
	msg.y     = roll_;
	msg.x     = pitch_;
	msg.r     = yaw_;
	msg.z     = throttle_;
	msg.flaps = 0.f;
	msg.aux1  = 0.f;
	msg.aux2  = 0.f;
	msg.aux3  = 0.f;
	msg.aux4  = 0.f;
	msg.aux5  = 0.f;
	msg.aux6  = 0.f;
	manual_control_setpoint_publisher_->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void ManualControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	VehicleCommand msg{};
	msg.timestamp = timestamp_.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	vehicle_command_publisher_->publish(msg);
}



int main(int argc, char* argv[]) {
	std::cout << "Starting manual control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ManualControl>());

	rclcpp::shutdown();
	return 0;
}