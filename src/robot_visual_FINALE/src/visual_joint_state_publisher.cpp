#include "rclcpp/rclcpp.hpp"
#include <memory>

class VisualJointStatePublisher : public rclcpp::Node {
public:
    VisualJointStatePublisher() : Node("visual_joint_state_publisher") {
        RCLCPP_INFO(this->get_logger(), "VisualJointStatePublisher node initialized.");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VisualJointStatePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
