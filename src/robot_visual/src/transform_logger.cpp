#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <fstream>
#include <iomanip>

class TransformLoggerNode : public rclcpp::Node
{
public:
    TransformLoggerNode()
        : Node("transform_logger_node"),
          tf_buffer_(this->get_clock()), // Pass the clock to the buffer
          tf_listener_(tf_buffer_)
    {
        // Open the CSV file
        csv_file_.open("/home/eliott-frohly/Robot5A_BT/src/robot_visual/logs/transform_log.csv", std::ios::out | std::ios::app);
        if (!csv_file_.is_open())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the CSV file for logging.");
            rclcpp::shutdown();
        }
        else
        {
            // Write CSV header if the file is empty
            csv_file_ << "timestamp,x,y,z\n";
        }

        // Timer to periodically query and log the transform
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // Log every 100ms
            std::bind(&TransformLoggerNode::logTransform, this));
    }

    ~TransformLoggerNode()
    {
        if (csv_file_.is_open())
        {
            csv_file_.close();
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::ofstream csv_file_;

    void logTransform()
    {
        try
        {
            // Lookup the transform from world to aruco_0
            geometry_msgs::msg::TransformStamped transform_stamped =
                tf_buffer_.lookupTransform("world", "aruco_19", tf2::TimePointZero);

            // Extract the timestamp
            rclcpp::Time timestamp = transform_stamped.header.stamp;

            // Extract x, y, z from the transform
            double x = transform_stamped.transform.translation.x;
            double y = transform_stamped.transform.translation.y;
            double z = transform_stamped.transform.translation.z;

            // Log to CSV
            csv_file_ << std::fixed << std::setprecision(6) << timestamp.seconds() << ","
                      << x << "," << y << "," << z << "\n";

            RCLCPP_INFO(this->get_logger(), "Logged transform: time=%f, x=%f, y=%f, z=%f",
                        timestamp.seconds(), x, y, z);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TransformLoggerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
