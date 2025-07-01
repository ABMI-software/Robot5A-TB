#include "rclcpp/rclcpp.hpp"
#include <opencv2/opencv.hpp>
#include <thread>

class CameraTestNode : public rclcpp::Node
{
public:
    CameraTestNode() : Node("camera_test_node")
    {
        this->declare_parameter<int>("camera_0_id", 4); // Default camera ID for /dev/video0
        

        int camera_0_id;
        this->get_parameter("camera_0_id", camera_0_id);
        

        // Open cameras
        cap_0.open(camera_0_id, cv::CAP_V4L2);
        

        if (!cap_0.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera 0 (ID: %d)", camera_0_id);
            rclcpp::shutdown();
        }


        // Set resolution and frame rate
        cap_0.set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        cap_0.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
        cap_0.set(cv::CAP_PROP_FPS, 30);
        cap_0.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // Use MJPG


        RCLCPP_INFO(this->get_logger(), "Successfully opened both cameras!");

        // Start a separate thread for displaying frames
        display_thread_ = std::thread(&CameraTestNode::displayFrames, this);
    }

    ~CameraTestNode()
    {
        if (display_thread_.joinable())
        {
            display_thread_.join();
        }
        cap_0.release();

        cv::destroyAllWindows();
    }

private:
    cv::VideoCapture cap_0;
    std::thread display_thread_;

    void displayFrames()
    {
        cv::Mat frame_0;

        while (rclcpp::ok())
        {
            cap_0 >> frame_0;


            if (!frame_0.empty())
            {
                cv::imshow("Camera 0 - 1600x1200 @ 30fps", frame_0);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Empty frame from camera 0");
            }



            // Break on key press
            if (cv::waitKey(10) == 27) // 27 is the ESC key
            {
                RCLCPP_INFO(this->get_logger(), "ESC pressed. Shutting down...");
                rclcpp::shutdown();
                break;
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CameraTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
