#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_gps_position.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
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


class TelemetryInformer : public rclcpp::Node
{
public:
	explicit TelemetryInformer() : Node("telemetry_informer") {

		latitude_ = 0.0;
		longitude_ = 0.0;
		amsl_ = 0.0;
		groundspeed_ = 0.0;
		yaw_ = 0.0;

		timer_ = this->create_wall_timer(std::chrono::milliseconds(500),std::bind(&TelemetryInformer::timerCallback,this));
        telemetry_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cloud_telemetry",10);

		vehicle_position_speed_subscription_ = this->create_subscription<px4_msgs::msg::VehicleGpsPosition>(
			"fmu/vehicle_gps_position/out",10,
			[this](const px4_msgs::msg::VehicleGpsPosition::UniquePtr msg) {
			latitude_ = msg->lat/10000000.0; // converting from 1E-7 degrees to degrees
			longitude_ = msg->lon/10000000.0; // converting from 1E-7 degrees to degrees
			amsl_ = msg->alt/1000.0; // converting from mm to m
			groundspeed_ = msg->vel_m_s;
		});

		vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"fmu/vehicle_attitude/out",
			10,
			[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
			// frame conversion needed to convert from NED frame to ENU frame and then parse quaternion
			quatd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
			double roll, pitch, yaw;
			px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(quatd_,roll,pitch,yaw);
			roll = roll*180/M_PI;
			pitch = pitch*180/M_PI;
			yaw_ = yaw*180/M_PI;
		});
	}

private:
	void timerCallback()
	{
		auto msg = geometry_msgs::msg::Twist();
		msg.linear.x = latitude_;
		msg.linear.y = longitude_;
		msg.linear.z = amsl_;
		msg.angular.x = groundspeed_;
		msg.angular.y = yaw_;
		telemetry_publisher_->publish(msg);
	}

	double latitude_;
	double longitude_;
	double amsl_;
	double groundspeed_;
	double yaw_;

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr telemetry_publisher_;

	rclcpp::Subscription<px4_msgs::msg::VehicleGpsPosition>::SharedPtr vehicle_position_speed_subscription_;
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    Eigen::Quaterniond quatd_;
};

int main(int argc, char *argv[])
{
	std::cout << "Starting telemtery informer node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TelemetryInformer>());

	rclcpp::shutdown();
	return 0;
}