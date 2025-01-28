#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <unordered_map>

class ArucoDisplacement : public rclcpp::Node {
public:
    ArucoDisplacement() : Node("aruco_displacement") {
        RCLCPP_INFO(this->get_logger(), "ArucoDisplacement node initialized.");

        // Initialize the TF buffer and listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Timer to periodically check for transformations
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000), // Adjust the frequency as needed
            std::bind(&ArucoDisplacement::checkTransform, this));
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<std::string, geometry_msgs::msg::TransformStamped> last_transforms_;

    void checkTransform() {
        // List of ArUco markers to track
        std::vector<std::string> markers = {"aruco_1"}; // Add more as needed

        for (const auto &marker : markers) {
            try {
                // Get the latest transform for the marker
                geometry_msgs::msg::TransformStamped current_transform = tf_buffer_->lookupTransform("world", marker, tf2::TimePointZero);

                // Calculate movement if we have a previous transform
                if (last_transforms_.find(marker) != last_transforms_.end()) {
                    const auto &last_transform = last_transforms_[marker];

                    // Calculate the movement in x, y, z
                    double dx = current_transform.transform.translation.x - last_transform.transform.translation.x;
                    double dy = current_transform.transform.translation.y - last_transform.transform.translation.y;
                    double dz = current_transform.transform.translation.z - last_transform.transform.translation.z;

                    RCLCPP_INFO(this->get_logger(), "Movement of %s: dx: %.3f, dy: %.3f, dz: %.3f", marker.c_str(), dx, dy, dz);
                }

                // Update the last transform
                last_transforms_[marker] = current_transform;
            } catch (const tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "Could not get transform for %s: %s", marker.c_str(), ex.what());
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDisplacement>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}