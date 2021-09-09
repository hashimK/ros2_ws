#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
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

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
        roll_ = 0.0;
        pitch_ = 0.0;
        yaw_ = 0.0;
        throttle_ = 0.0f;
        pitch_angle_limit_ = 45.0;
        roll_angle_limit_ = 45.0;
        yaw_angle_limit_ = 90.0;
		change_mode_ = 0.0;

		pos_x_ = 0.0f;
		pos_y_ = 0.0f;
		pos_z_ = 0.0f;

		offboard_control_mode_publisher_ =
			this->create_publisher<OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		vehicle_attitude_setpoint_publisher_ =
			this->create_publisher<VehicleAttitudeSetpoint>("fmu/vehicle_attitude_setpoint/in", 10);
		vehicle_command_publisher_ =
			this->create_publisher<VehicleCommand>("fmu/vehicle_command/in", 10);
		trajectory_setpoint_publisher_ =
			this->create_publisher<TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);


		// get common timestamp
		timesync_sub_ =
			this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					timestamp_.store(msg->timestamp);
				});

        joystick_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel_continous",10,
                                    std::bind(&OffboardControl::callbackJoystick, this,std::placeholders::_1));
		
		odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>("fmu/vehicle_odometry/out",10,
			[this](const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
				if (change_mode_== 0.0)
				{
					pos_x_ = msg->x;
					pos_y_ = msg->y;
					pos_z_ = msg->z;
				}
		});

		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

				// Arm the vehicle
				this->arm();
			}

            		// offboard_control_mode needs to be paired with vehicle_attitude_setpoint
			publish_offboard_control_mode();
			publish_vehicle_attitude_setpoint();
			publish_trajectory_setpoint();

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

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr vehicle_attitude_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr timesync_sub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr joystick_subscriber_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_subscription_;

    double roll_;
    double pitch_;
    double yaw_;
    float throttle_;
    double pitch_angle_limit_;
    double roll_angle_limit_;
    double yaw_angle_limit_;
	double change_mode_;

	float pos_x_;
	float pos_y_;
	float pos_z_;

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode() ;
	void publish_vehicle_attitude_setpoint() ;
	void publish_trajectory_setpoint() const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;

    void callbackJoystick(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        pitch_ = (msg->linear.x*pitch_angle_limit_)*-1.0*M_PI/180.0;
        roll_ = (msg->linear.y*roll_angle_limit_)*M_PI/180.0;
        throttle_ = msg->angular.x;
		change_mode_ = msg->linear.z;
		if (change_mode_==0.0) yaw_ = (msg->angular.y*yaw_angle_limit_)*M_PI/180.0;
    }
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() {
	OffboardControlMode msg{};
	msg.timestamp = timestamp_.load();
	if (change_mode_==1.0)
	{
		msg.position = true;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = false;
		msg.body_rate = false;
	}
	else
	{
		msg.position = false;
		msg.velocity = false;
		msg.acceleration = false;
		msg.attitude = true;
		msg.body_rate = false;
	}
	

	offboard_control_mode_publisher_->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint() const {
	TrajectorySetpoint msg{};
	msg.timestamp = timestamp_.load();
	// msg.x = 0.0;
	// msg.y = 0.0;
	// msg.z = -5.0;
	// msg.yaw = -3.14; // [-PI:PI]

	msg.x = pos_x_;
	msg.y = pos_y_;
	msg.z = pos_z_;
	msg.yaw = yaw_; // [-PI:PI]

	trajectory_setpoint_publisher_->publish(msg);
}


void OffboardControl::publish_vehicle_attitude_setpoint()  {
	VehicleAttitudeSetpoint msg{};
	msg.timestamp = timestamp_.load();
	// msg.x = 0.0;
	// msg.y = 0.0;
	// msg.z = -5.0;
	// msg.yaw = -3.14; // [-PI:PI]
    Eigen::Quaterniond quatd_ = px4_ros_com::frame_transforms::utils::quaternion::quaternion_from_euler(roll_, pitch_, yaw_);
    std::array<float, 4> qarray;
    px4_ros_com::frame_transforms::utils::quaternion::eigen_quat_to_array(quatd_,qarray);
    throttle_ = throttle_*-1.0f;
    std::array<float, 3> thrust_array = {0,0,throttle_};
    msg.q_d = qarray;
    msg.thrust_body = thrust_array;

	vehicle_attitude_setpoint_publisher_->publish(msg);
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
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
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}