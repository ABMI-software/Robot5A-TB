#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_map>

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
        cap_.open(4, cv::CAP_V4L2);
        if (!cap_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open the camera");
            rclcpp::shutdown();
        }

        // Camera settings
        // Set type (MJPG)
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));

        // Set resolution
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

        // Set camera properties
        cap_.set(cv::CAP_PROP_BRIGHTNESS, 0.5); // Reduce brightness to avoid overexposure
        cap_.set(cv::CAP_PROP_CONTRAST, 0.7);   // Higher contrast for clear edges
        cap_.set(cv::CAP_PROP_SATURATION, 0.6);
        cap_.set(cv::CAP_PROP_SHARPNESS, 4);    // Increase sharpness
        cap_.set(cv::CAP_PROP_GAMMA, 0.5);      // Adjust gamma to enhance details
        cap_.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 5000); // Set a neutral white balance (~5000-6000K)
        cap_.set(cv::CAP_PROP_AUTO_WB, 0);      // Lock white balance

        cap_.set(cv::CAP_PROP_EXPOSURE, -5);    // Lower exposure for better contrast

        // Set frame rate
        cap_.set(cv::CAP_PROP_FPS, 15);         // Higher FPS can cause motion blur, affecting accuracy

        // Check if the settings were applied
        double width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
        double height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap_.get(cv::CAP_PROP_FPS);

        std::cout << "Resolution: " << width << "x" << height << std::endl;
        std::cout << "FPS: " << fps << std::endl;

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
    

    // Store first detected values
    std::unordered_map<int, bool> first_detection;
    std::unordered_map<int, cv::Vec3d> first_tvecs;
    std::unordered_map<int, cv::Vec3d> first_rvecs;
    std::unordered_set<int> previously_detected_markers;


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
        cv::Mat frame, gray, filtered, enhanced, undistortedFrame;

        cap_ >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Empty frame received from the camera");
            return;
        }

        // Undistort the frame

        // Extract height and width from the image
        int height = frame.rows;
        int width = frame.cols;

        // Get the optimal new camera matrix
        cv::Mat camMatrixNew;
        cv::Rect roi;
        camMatrixNew = cv::getOptimalNewCameraMatrix(camMatrix_, distCoeffs_, cv::Size(width, height), 1, cv::Size(width, height), &roi);
        cv::undistort(frame, undistortedFrame, camMatrixNew, distCoeffs_);

        // Convert to grayscale
        cv::cvtColor(undistortedFrame, gray, cv::COLOR_BGR2GRAY);

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
        cv::aruco::detectMarkers(gray, dictionary_, markerCorners, markerIds, detectorParams_);


        // Refine corner locations to sub-pixel accuracy (optional)
        if (!markerCorners.empty()) {
        cv::TermCriteria criteria(
            cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
        for (auto &corners : markerCorners) {
            cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1),
                            criteria);
        }
        }

        std::unordered_set<int> currently_detected_markers(markerIds.begin(), markerIds.end());

        // Find markers that were previously detected but are now missing
        for (int prev_marker : previously_detected_markers)
        {
            if (currently_detected_markers.find(prev_marker) == currently_detected_markers.end())
            {
                RCLCPP_WARN(this->get_logger(), "Marker %d lost, reinitializing.", prev_marker);
                first_detection.erase(prev_marker);
                first_tvecs.erase(prev_marker);
                first_rvecs.erase(prev_marker);
            }
        }

        // Update previously detected markers
        previously_detected_markers = currently_detected_markers;


        if (!markerIds.empty())
        {
            std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());


            for (size_t i = 0; i < markerIds.size(); ++i)
            {
                int marker_id = markerIds[i];

                // Check if marker corners are valid
                if (markerCorners[i].size() != 4)
                {
                    RCLCPP_WARN(this->get_logger(), "Marker corners not valid for marker ID %d", marker_id);
                    continue; // Skip this marker
                }

                // Solve PnP using SOLVEPNP_IPPE_SQUARE
                // Define the 3D points of the marker corners in the marker's local coordinate system
                std::vector<cv::Point3f> markerPoints = {
                    cv::Point3f(-marker_length_ / 2, marker_length_ / 2, 0),  // Point 0
                    cv::Point3f(marker_length_ / 2, marker_length_ / 2, 0),   // Point 1
                    cv::Point3f(marker_length_ / 2, -marker_length_ / 2, 0),  // Point 2
                    cv::Point3f(-marker_length_ / 2, -marker_length_ / 2, 0)  // Point 3
                };

                if (!cv::solvePnP(markerPoints, markerCorners[i], camMatrixNew, distCoeffs_, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE))
                {
                    RCLCPP_WARN(this->get_logger(), "PnP failed for marker ID %d", marker_id);
                    continue; // Skip this marker
                }

                // if (marker_id == 0){
                // // Log the translation vector for the current marker
                // RCLCPP_INFO(this->get_logger(), "Translation vector for marker %d: [%f, %f, %f]",
                //     marker_id, tvecs[i][0], tvecs[i][1], tvecs[i][2]);}


                // // Log the rotation vector for the current marker
                // RCLCPP_INFO(this->get_logger(), "Rotation    vector for marker %d: [%f, %f, %f]",
                //     marker_id, rvecs[i][0], rvecs[i][1], rvecs[i][2]);
                // }

                // Check if this is the first time the marker is detected
                
                if (first_detection.count(marker_id) == 0) // Correct way to check if a key exists
                {
                    first_detection[marker_id] = true;
                    first_tvecs[marker_id] = tvecs[i];
                    first_rvecs[marker_id] = rvecs[i];
                }
                else
                {
                    // Apply filtering only for subsequent detections
                    // Define thresholds for translation and rotation
                    const double translation_threshold = 1.0; // threshold for translation
                    const double rotation_threshold = 1.0; // threshold for rotation (in radians)

                    cv::Vec3d previous_tvec = first_tvecs[marker_id];
                    cv::Vec3d previous_rvec = first_rvecs[marker_id];
                    
                    // // Check and update each translation component individually
                    // for (int j = 0; j < 3; j++)
                    // {
                    //     if (std::abs(tvecs[i][j] - previous_tvec[j]) >= translation_threshold)
                    //     {
                    //         tvecs[i][j] = -tvecs[i][j];  // Invert if exceeds threshold
                    //     }
                        
                    // }
                    
                    // Check and update each rotation component individually
                    for (int j = 0; j < 3; j++)
                    {
                        if (std::abs(rvecs[i][j] - previous_rvec[j]) >= rotation_threshold)
                        {
                            rvecs[i][j] = -rvecs[i][j];  // Invert if exceeds threshold
                        }
                        
                    }
                    
                    // Update the stored values
                    first_tvecs[marker_id] = tvecs[i];
                    first_rvecs[marker_id] = rvecs[i];
                }


                // Publish transforms
                publishTransform(rvecs[i], tvecs[i], marker_id);


                // draw axis and marker
                
                cv::aruco::drawDetectedMarkers(undistortedFrame, markerCorners, markerIds); // Draw all detected markers
                

                std::vector<cv::Point3f> axisPoints = {
                    cv::Point3f(0, 0, 0), // Origin
                    cv::Point3f(marker_length_ * 1.5f, 0, 0), // X-axis
                    cv::Point3f(0, marker_length_ * 1.5f, 0), // Y-axis
                    cv::Point3f(0, 0, marker_length_ * 1.5f)  // Z-axis
                };
                std::vector<cv::Point2f> imagePoints;
                cv::projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix_, distCoeffs_, imagePoints);
                cv::line(undistortedFrame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2); // X - Red
                cv::line(undistortedFrame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2); // Y - Green
                cv::line(undistortedFrame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2); // Z - Blue
            }
        }


        // Get the world-to-camera transform 
        Eigen::Matrix4d world_to_camera = camera_transform_.inverse();

        // Extract rotation matrix (3x3) and translation vector (3x1)
        Eigen::Matrix3d R = world_to_camera.block<3, 3>(0, 0);
        Eigen::Vector3d t = world_to_camera.block<3, 1>(0, 3);

        // Convert rotation matrix to Rodrigues rotation vector
        cv::Mat R_cv(3, 3, CV_64F);
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                R_cv.at<double>(i, j) = R(i, j);
        cv::Mat rvec_cv;
        cv::Rodrigues(R_cv, rvec_cv);
        cv::Vec3d rvec(rvec_cv.at<double>(0), rvec_cv.at<double>(1), rvec_cv.at<double>(2));

        // Convert translation to cv::Vec3d
        cv::Vec3d tvec(t(0), t(1), t(2));

        // Project the world origin (0, 0, 0) into the image
        std::vector<cv::Point3f> objectPoints = {cv::Point3f(0, 0, 0)};
        std::vector<cv::Point2f> imagePoints;
        cv::projectPoints(objectPoints, rvec, tvec, camMatrix_, distCoeffs_, imagePoints);
        int pixel_x = static_cast<int>(imagePoints[0].x);
        int pixel_y = static_cast<int>(imagePoints[0].y);

        // Draw the origin point on the frame
        cv::circle(undistortedFrame, cv::Point(pixel_x, pixel_y), 5, cv::Scalar(255, 0, 255), -1);

        // Project the world origin (0, 0, 0) into the image
        std::vector<cv::Point3f> objectPoints_end = {cv::Point3f(0.8, 0, 0)};
        std::vector<cv::Point2f> imagePoints_end;
        cv::projectPoints(objectPoints_end, rvec, tvec, camMatrix_, distCoeffs_, imagePoints_end);
        int pixel_x_end = static_cast<int>(imagePoints_end[0].x);
        int pixel_y_end = static_cast<int>(imagePoints_end[0].y);

        // // Draw a line from origin to 800mm endpoint
        // cv::line(undistortedFrame, cv::Point(pixel_x, pixel_y), cv::Point(pixel_x_end, pixel_y_end), cv::Scalar(0, 255, 0), 2); // Green color, thickness 2

       // Extract principal point from camMatrix_
        double cx = camMatrix_.at<double>(0, 2); // Principal point x-coordinate
        double cy = camMatrix_.at<double>(1, 2); // Principal point y-coordinate

        // Draw the principal point on the undistorted frame
        cv::circle(undistortedFrame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)), 5, cv::Scalar(0, 255, 255), -1); // Yellow filled circle

        // // Log for debugging
        // RCLCPP_INFO(this->get_logger(), "rvec: [%f, %f, %f]", rvec[0], rvec[1], rvec[2]);
        // RCLCPP_INFO(this->get_logger(), "tvec: [%f, %f, %f]", tvec[0], tvec[1], tvec[2]);
        // RCLCPP_INFO(this->get_logger(), "Projected pixel: [%d, %d]", pixel_x, pixel_y);

        
        // Resize images
        int newWidth = width; 
        int newHeight = height; 
        cv::resize(frame, frame, cv::Size(newWidth, newHeight));
        cv::resize(enhanced, enhanced, cv::Size(newWidth, newHeight));
        cv::resize(binary, binary, cv::Size(newWidth, newHeight));

        // Visualize results
        cv::imshow("Original Frame", undistortedFrame);
        cv::setMouseCallback("Original Frame", onMouse, this);
        // cv::imshow("Enhanced", enhanced);
        // cv::imshow("Binary", binary);
        cv::waitKey(1);
    }

    // Static mouse callback function
    static void onMouse(int event, int x, int y, int flags, void* userdata) {
        // Cast userdata back to the ArucoDetectorSingle instance
        ArucoDetectorSingle* self = static_cast<ArucoDetectorSingle*>(userdata);
        
        if (event == cv::EVENT_MOUSEMOVE) {
            // Display the cursor coordinates in the window title
            std::stringstream ss;
            ss << "Original Frame - Cursor: (" << x << ", " << y << ")";
            cv::setWindowTitle("Original Frame", ss.str());
        }
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

        Eigen::Matrix4d fixed_to_marker = camera_transform_ * camera_to_marker;

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
        if (marker_id == 0)
        {
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