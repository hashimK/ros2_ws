#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <math.h>
#include <px4_ros_com/frame_transforms.h>

/**
 * @brief Sensor Combined uORB topic data callback
 */
class VehicleAttitudeListener : public rclcpp::Node
{
    public:
	explicit VehicleAttitudeListener() : Node("vehicle_attitude_listener") {
		subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
			"fmu/vehicle_attitude/out",
			10,
			[this](const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
			std::cout << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
			std::cout << "RECEIVED Attitude DATA"   << std::endl;
			std::cout << "============================="   << std::endl;
			// std::cout << "q[0]: "          << msg->q[0]   << std::endl;
            // std::cout << "q[1]: "          << msg->q[1]   << std::endl;
            // std::cout << "q[2]: "          << msg->q[2]   << std::endl;
            // std::cout << "q[3]: "          << msg->q[3]   << std::endl;
            // tf2::Quaternion q(
            //     msg->q[0],
            //     msg->q[1],
            //     msg->q[2],
            //     msg->q[3]);
            // tf2::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
            // roll = roll*180/M_PI;
            // pitch = pitch*180/M_PI;
            // yaw = yaw*180/M_PI;
            // std::cout << "roll: "          << roll   << std::endl;
            // std::cout << "pitch: "          << pitch   << std::endl;
            // std::cout << "yaw: "          << yaw   << std::endl;

            // frame conversion needed to convert from NED frame to ENU frame and then parse quaternion
            quatd_ = px4_ros_com::frame_transforms::utils::quaternion::array_to_eigen_quat(msg->q);
            double roll, pitch, yaw;
            px4_ros_com::frame_transforms::utils::quaternion::quaternion_to_euler(quatd_,roll,pitch,yaw);
            roll = roll*180/M_PI;
            pitch = pitch*180/M_PI;
            yaw = yaw*180/M_PI;
            std::cout << "roll: "          << roll   << std::endl;
            std::cout << "pitch: "          << pitch   << std::endl;
            std::cout << "yaw: "          << yaw   << std::endl;

            // std::cout << "q[0]: "          << quatd_.w()  << std::endl;
		});
	}
    private:
	rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_;
    Eigen::Quaterniond quatd_;
    
};

int main(int argc, char *argv[])
{
	std::cout << "Starting vehicle_attitude listener node..." << std::endl;
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<VehicleAttitudeListener>());

	rclcpp::shutdown();
	return 0;
}