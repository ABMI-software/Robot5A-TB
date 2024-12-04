#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

class ArucoDetectorSingle : public rclcpp::Node
{
public:
    ArucoDetectorSingle() : Node("aruco_detector_single"), tf_broadcaster_(this)
    {
        // Get the package share directory
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_visual");

        // Construct paths to configuration files
        std::string camera_calibration_file = package_share_directory + "/config/camera_2_calibration.yaml";
        std::string camera_transform_file = package_share_directory + "/config/camera_transform.yaml";

        // Read camera calibration parameters
        readCameraCalibration(camera_calibration_file, camMatrix_, distCoeffs_);
        readTransforms(camera_transform_file, 2); // Use camera 1's transform

        // Set the marker length (in meters)
        marker_length_ = 0.03;

        // Initialize the camera
        cap_.open(0, cv::CAP_V4L2); // Use the first camera (/dev/video0)
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            rclcpp::shutdown();
        }

        // Configure the camera
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Timer to periodically process the camera frames
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), // Approx. 30fps
            std::bind(&ArucoDetectorSingle::processFrame, this));
    }

    ~ArucoDetectorSingle()
    {
        cap_.release();
        cv::destroyAllWindows();
    }

private:
    cv::VideoCapture cap_;
    cv::Mat camMatrix_, distCoeffs_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    Eigen::Matrix4d camera_transform_;
    double marker_length_;
    rclcpp::TimerBase::SharedPtr timer_;

    void readCameraCalibration(const std::string &filename, cv::Mat &camMatrix, cv::Mat &distCoeffs)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open camera calibration file: %s", filename.c_str());
            return;
        }
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
    }

    void readTransforms(const std::string &filename, int camera_id)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(filename);
            if (!config["camera"])
            {
                RCLCPP_ERROR(this->get_logger(), "No camera transforms found in the file: %s", filename.c_str());
                return;
            }

            for (const auto &camera : config["camera"])
            {
                if (camera["id"].as<int>() == camera_id)
                {
                    camera_transform_ = parseTransform(camera["transform"]);
                    return;
                }
            }
            RCLCPP_ERROR(this->get_logger(), "Transform for camera ID %d not found in file: %s", camera_id, filename.c_str());
        }
        catch (const YAML::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error in '%s': %s", filename.c_str(), e.what());
        }
    }

    Eigen::Matrix4d parseTransform(const YAML::Node &node)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < node.size(); ++i)
        {
            for (int j = 0; j < node[i].size(); ++j)
            {
                transform(i, j) = node[i][j].as<double>();
            }
        }
        return transform;
    }

    void processFrame()
    {
        cv::Mat frame;
        cap_ >> frame;

        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame received from the camera");
            return;
        }

        // Detect ArUco markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, detectorParams);

        if (!markerIds.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;

            // Estimate the pose of each marker
            cv::aruco::estimatePoseSingleMarkers(markerCorners, marker_length_, camMatrix_, distCoeffs_, rvecs, tvecs);

            for (size_t i = 0; i < markerIds.size(); ++i)
            {
                int marker_id = markerIds[i];

                // Convert rotation vector to rotation matrix
                cv::Mat rotation_matrix;
                cv::Rodrigues(rvecs[i], rotation_matrix);

                // Build the transformation matrix from the camera to the marker
                Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
                for (int row = 0; row < 3; ++row)
                {
                    for (int col = 0; col < 3; ++col)
                    {
                        camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
                    }
                    camera_to_marker(row, 3) = tvecs[i][row];
                }

                // Compute the transformation from the fixed frame to the marker
                Eigen::Matrix4d fixed_to_marker = camera_transform_ * camera_to_marker;

                // Publish the transform
                geometry_msgs::msg::TransformStamped transformStamped;
                transformStamped.header.stamp = this->now();
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);
                transformStamped.transform.translation.x = fixed_to_marker(0, 3);
                transformStamped.transform.translation.y = fixed_to_marker(1, 3);
                transformStamped.transform.translation.z = fixed_to_marker(2, 3);

                Eigen::Matrix3d rotation = fixed_to_marker.block<3, 3>(0, 0);
                Eigen::Quaterniond quaternion(rotation);
                transformStamped.transform.rotation.x = quaternion.x();
                transformStamped.transform.rotation.y = quaternion.y();
                transformStamped.transform.rotation.z = quaternion.z();
                transformStamped.transform.rotation.w = quaternion.w();

                tf_broadcaster_.sendTransform(transformStamped);

                // Draw marker and pose axis
                cv::aruco::drawAxis(frame, camMatrix_, distCoeffs_, rvecs[i], tvecs[i], marker_length_ * 1.5f);
            }
        }

        // Display the processed frame
        cv::imshow("ArUco Detector", frame);
        cv::waitKey(1);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArucoDetectorSingle>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
