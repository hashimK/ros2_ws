#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/manual_control_setpoint.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <px4_ros_com/frame_transforms.h>
#include <geometry_msgs/msg/twist.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class DroneController : public rclcpp::Node {
public:
	DroneController() : Node("drone_controller") {
		roll_pwm_ = 0.0;
        pitch_pwm_ = 0.0;
        yaw_pwm_ = 0.0;
        throttle_pwm_ = 0.0f;
		arm_status_ = 0.0;
		prev_arm_status_ = 0.0;
		arm_transition_ = false;

		flight_mode_ = 0.0;
		prev_flight_mode_ = 0.0;
		flight_mode_transition_ = false;

		speech_h_displacement_ = 0.0;
		speech_v_displacement_ = 0.0;
		speech_yaw_ = -1.0;
		speech_activation_status_ = false;
		prev_speech_h_displacement_ = 0.0;
		prev_speech_v_displacement_ = 0.0;
		prev_speech_yaw_ = -1.0;
		prev_speech_activation_status_ = false;

		flight_yaw_ = 0.0;

		pos_x_ = 0.0f;
		pos_y_ = 0.0f;
		pos_z_ = 0.0f;

		capture_once_ = false;

		manual_control_setpoint_publisher_ =
			this->create_publisher<ManualControlSetpoint>("fmu/manual_control_setpoint/in", 10);

		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);

		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);

		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

        web_app_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cloud_web_app_commands",10,
                                    std::bind(&DroneController::callbackWebApp, this,std::placeholders::_1));

		speech_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cloud_speech_commands",10,
                                    std::bind(&DroneController::callbackSpeech, this,std::placeholders::_1));

		odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out",10,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				if (capture_once_ == true)
				{
					pos_x_ = msg->x;
					pos_y_ = msg->y;
					pos_z_ = msg->z;

					// * @param q  Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
					tf2::Quaternion qt(
						msg->q[3],
						msg->q[0],
						msg->q[1],
						msg->q[2]);
					tf2::Matrix3x3 m(qt);
					double roll, pitch, yaw;
					m.getRPY(roll, pitch, yaw);
					flight_yaw_  = 180 - yaw*180/M_PI; // to get yaw in NED frame

					std::cout << "=================================="   << std::endl;
					std::cout << "flight_yaw_degrees: "      << flight_yaw_    << std::endl;
					std::cout << "pos_x_: "      << pos_x_    << std::endl;
					std::cout << "pos_y_: "      << pos_y_    << std::endl;
					std::cout << "pos_z_: "      << pos_z_    << std::endl;

					if (speech_activation_status_==false)
					{
						std::cout << "switching off offboard "   << std::endl;
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, flight_mode_);
					}

					capture_once_ = false;
				}
		});

		offboard_setpoint_counter_ = 0;
		

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

			if (speech_activation_status_== true)
			{
				if (offboard_setpoint_counter_ == 10) 
				{
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

					// Arm the vehicle if not previously armed
					if (arm_status_<1.0)
					{
						this->arm();
					}
				}

				// publish offboard setpoints
				publish_offboard_control_mode();
				publish_trajectory_setpoint();
			}

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm() const;
	void disarm() const;

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr web_app_subscriber_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr speech_subscriber_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;

	rclcpp::Publisher<ManualControlSetpoint>::SharedPtr manual_control_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;

	Eigen::Quaterniond quatd_;

    double roll_pwm_;
    double pitch_pwm_;
    double yaw_pwm_;
    float throttle_pwm_;
	double arm_status_;
	double prev_arm_status_;
	bool arm_transition_;
	double flight_mode_ ;
	double prev_flight_mode_;
	bool flight_mode_transition_;

	double speech_h_displacement_;
	double speech_v_displacement_;
	double speech_yaw_;
	bool speech_activation_status_;
	double prev_speech_h_displacement_;
	double prev_speech_v_displacement_;
	double prev_speech_yaw_;
	bool prev_speech_activation_status_;

	double flight_yaw_;

	float pos_x_;
	float pos_y_;
	float pos_z_;

	bool capture_once_;

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	void publish_offboard_control_mode() ;
	void publish_trajectory_setpoint() const;
	void publish_manual_control_setpoint() ;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,float param2 = 0.0) const;   


    void callbackWebApp(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pitch_pwm_ = msg->linear.x;
        roll_pwm_ = msg->linear.y;
        throttle_pwm_ = msg->angular.x;
		yaw_pwm_ = msg->angular.y;
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

	void callbackSpeech(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        speech_h_displacement_ = msg->linear.x;
		speech_yaw_ = msg->angular.z;
		speech_v_displacement_ = msg->linear.z;
		speech_activation_status_ = (msg->angular.x > 0) ? true : false;
		if (speech_activation_status_ == false)
		{
			offboard_setpoint_counter_=0;
		}

		if((prev_speech_h_displacement_!=speech_h_displacement_) || (prev_speech_v_displacement_!=speech_v_displacement_) || (prev_speech_yaw_!=speech_yaw_) || (prev_speech_activation_status_!=speech_activation_status_) )
		{
			capture_once_ = true;
			prev_speech_h_displacement_ = speech_h_displacement_;
			prev_speech_v_displacement_ = speech_v_displacement_;
			prev_speech_yaw_ = speech_yaw_;
			prev_speech_activation_status_ = speech_activation_status_;
		}
    }
};

/**
 * @brief Send a command to Arm the vehicle
 */
void DroneController::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void DroneController::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

void DroneController::publish_manual_control_setpoint()  {
	ManualControlSetpoint msg{};
	msg.timestamp = timestamp_.load();
	msg.data_source = 2;

	// limit controls
	msg.y     = roll_pwm_;
	msg.x     = pitch_pwm_;
	msg.r     = yaw_pwm_;
	msg.z     = throttle_pwm_;
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
void DroneController::publish_vehicle_command(uint16_t command, float param1,
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


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void DroneController::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint from the user's speech command
 */
void DroneController::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();

	msg.yaw = (speech_yaw_ == -1.0) ? flight_yaw_*M_PI/180.0 : speech_yaw_*M_PI/180.0 ;
	msg.x = pos_x_ + speech_h_displacement_ * cos( msg.yaw );
	msg.y = pos_y_+ speech_h_displacement_ * sin( msg.yaw );
	msg.z = pos_z_ - speech_v_displacement_; // -ve sign to transform ENU frame of ROS to NED frame of PX4
	trajectory_setpoint_publisher_->publish(msg);
}




int main(int argc, char* argv[]) {
	std::cout << "Starting drone_controller node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DroneController>());

	rclcpp::shutdown();
	return 0;
}