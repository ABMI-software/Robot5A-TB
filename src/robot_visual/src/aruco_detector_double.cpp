#include "rclcpp/rclcpp.hpp"
#include <memory>

class ArucoDetectorDouble : public rclcpp::Node {
public:
    ArucoDetectorDouble() : Node("aruco_detector_double") {
        RCLCPP_INFO(this->get_logger(), "ArucoDetectorDouble node initialized.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorDouble>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
