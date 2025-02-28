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

        // Load calibration and transform files
        std::string camera_calibration_file = package_share_directory + "/config/camera_1_calibration.yaml";
        std::string camera_transform_file = package_share_directory + "/config/camera_transform.yaml";

        // Read camera calibration and transform parameters
        readCameraCalibration(camera_calibration_file, camMatrix_, distCoeffs_);
        readTransforms(camera_transform_file, 1); // Use camera 1's transform

        // Set marker parameters
        marker_length_ = 0.03;

        // Open camera
        cap_.open(0, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            rclcpp::shutdown();
        }

        // Camera settings
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1600);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
        cap_.set(cv::CAP_PROP_FPS, 30);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Configure ArUco detector parameters
        configureDetectorParameters();

        // Initialize the publisher for /tf_detected
        tf_detected_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tf_detected", 10);

        // Timer for frame processing
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
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_detected_publisher_; // Publisher for /tf_detected
    Eigen::Matrix4d camera_transform_;
    double marker_length_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;

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

    void configureDetectorParameters()
    {
        detectorParams_ = cv::aruco::DetectorParameters::create();

        // Dictionary for ArUco markers
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    }

    void processFrame()
    {
        cv::Mat frame, gray, filtered, enhanced;

        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame received from the camera");
            return;
        }

        // Convert to grayscale
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Reduce noise while preserving edges
        cv::bilateralFilter(gray, filtered, 3, 75, 20);

        // Adjust brightness and contrast using histogram equalization
        cv::equalizeHist(filtered, enhanced);

        // Optional: Apply adaptive thresholding for binary contrast enhancement
        cv::Mat binary;
        cv::adaptiveThreshold(enhanced, binary, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, 11, 2);

        // Detect markers
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f>> markerCorners;
        cv::aruco::detectMarkers(frame, dictionary_, markerCorners, markerIds, detectorParams_);

        if (!markerIds.empty())
        {
            std::vector<cv::Vec3d> rvecs, tvecs;

            for (size_t i = 0; i < markerIds.size(); ++i)
            {
                int marker_id = markerIds[i];

                // Define the 3D points of the marker corners in the marker's local coordinate system
                std::vector<cv::Point3f> markerPoints = {
                    cv::Point3f(-marker_length_ / 2, marker_length_ / 2, 0),  // Point 0
                    cv::Point3f(marker_length_ / 2, marker_length_ / 2, 0),   // Point 1
                    cv::Point3f(marker_length_ / 2, -marker_length_ / 2, 0),  // Point 2
                    cv::Point3f(-marker_length_ / 2, -marker_length_ / 2, 0)  // Point 3
                };

                // Solve PnP using SOLVEPNP_IPPE_SQUARE
                cv::solvePnP(markerPoints, markerCorners[i], camMatrix_, distCoeffs_, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE);

                // Publish transforms
                publishTransform(rvecs[i], tvecs[i], marker_id);

                // Draw marker and pose
                std::vector<cv::Point3f> axisPoints = {
                    cv::Point3f(0, 0, 0), // Origin
                    cv::Point3f(marker_length_ * 1.5f, 0, 0), // X-axis
                    cv::Point3f(0, marker_length_ * 1.5f, 0), // Y-axis
                    cv::Point3f(0, 0, marker_length_ * 1.5f)  // Z-axis
                };
                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix_, distCoeffs_, imagePoints);
                cv::line(frame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X - Red
                cv::line(frame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y - Green
                cv::line(frame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z - Blue

            }
        }

        // Start with the origin point (0, 0, 0) in the world frame
        Eigen::Vector4d origin_3d(0, 0, 0, 1); // Homogeneous coordinates

        // Transform the origin point to camera coordinates
        Eigen::Vector4d transformed_origin = camera_transform_ * origin_3d; // Perform the multiplication

        // Project the 3D point to 2D pixel coordinates
        Eigen::Vector3d projected_point = transformed_origin.head<3>(); // Get the 3D point

        // Convert camMatrix_ from cv::Mat to Eigen::Matrix3d
        Eigen::Matrix3d camMatrix_eigen;
        camMatrix_eigen << camMatrix_.at<double>(0, 0), camMatrix_.at<double>(0, 1), camMatrix_.at<double>(0, 2),
                        camMatrix_.at<double>(1, 0), camMatrix_.at<double>(1, 1), camMatrix_.at<double>(1, 2),
                        camMatrix_.at<double>(2, 0), camMatrix_.at<double>(2, 1), camMatrix_.at<double>(2, 2);

        // Project to pixel coordinates
        Eigen::Vector3d pixel_point_homogeneous = camMatrix_eigen * projected_point; // Perform the multiplication
        pixel_point_homogeneous /= pixel_point_homogeneous(2); // Normalize by the z-coordinate

        // Get the pixel coordinates
        int pixel_x = static_cast<int>(pixel_point_homogeneous(0));
        int pixel_y = static_cast<int>(pixel_point_homogeneous(1));

        // Draw the origin point on the frame
        cv::circle(frame, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(255, 0, 255), -1); // Draw a circle

        // Resize images
        int newWidth = 800; 
        int newHeight = 600; 
        cv::resize(frame, frame, cv::Size(newWidth, newHeight));
        cv::resize(enhanced, enhanced, cv::Size(newWidth, newHeight));
        cv::resize(binary, binary, cv::Size(newWidth, newHeight));

        // Visualize results
        cv::imshow("Original Frame", frame);
        // cv::imshow("Enhanced", enhanced);
        // cv::imshow("Binary", binary);
        cv::waitKey(1);
    }

    void publishTransform(const cv::Vec3d &rvec, const cv::Vec3d &tvec, int marker_id)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);

        Eigen::Matrix4d camera_to_marker = Eigen::Matrix4d::Identity();
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                camera_to_marker(row, col) = rotation_matrix.at<double>(row, col);
            }
            camera_to_marker(row, 3) = tvec[row];
        }

        Eigen::Matrix4d fixed_to_marker = camera_transform_ * camera_to_marker.inverse();

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
        if (marker_id==0 ){
        tf_detected_publisher_->publish(transformStamped); // Publish detected transform
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorSingle>());
    rclcpp::shutdown();
    return 0;
}
