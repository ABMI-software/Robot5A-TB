#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <Eigen/Dense>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <mutex>

class ArucoDetectorDouble : public rclcpp::Node
{
public:
    ArucoDetectorDouble() : Node("aruco_detector_double"), tf_broadcaster_(this),
                            mouse_data_1_{this, "Camera 1"}, mouse_data_2_{this, "Camera 2"}
    {
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("robot_visual");
        std::string calib_file_1 = package_share_directory + "/config/camera_1_calibration.yaml";
        std::string calib_file_2 = package_share_directory + "/config/camera_2_calibration.yaml";
        std::string transform_file = package_share_directory + "/config/camera_transform.yaml";

        readCameraCalibration(calib_file_1, camMatrix_1_, distCoeffs_1_);
        readCameraCalibration(calib_file_2, camMatrix_2_, distCoeffs_2_);
        readTransforms(transform_file, 1, camera_transform_1_);
        readTransforms(transform_file, 2, camera_transform_2_);

        marker_length_ = 0.03;

        cap_1_.open(6, cv::CAP_V4L2);
        cap_2_.open(4, cv::CAP_V4L2);
        if (!cap_1_.isOpened() || !cap_2_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open one or both cameras (1: %d, 2: %d)",
                         cap_1_.isOpened(), cap_2_.isOpened());
        }
        else
        {
            configureCamera(cap_1_);
            configureCamera(cap_2_);
        }

        configureDetectorParameters();
        tf_detected_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/tf_detected", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&ArucoDetectorDouble::processFrames, this));
    }

    ~ArucoDetectorDouble()
    {
        RCLCPP_INFO(this->get_logger(), "Shutting down ArucoDetectorDouble, closing windows");
        cap_1_.release();
        cap_2_.release();
        cv::destroyAllWindows();
    }

private:
    cv::VideoCapture cap_1_, cap_2_;
    cv::Mat camMatrix_1_, distCoeffs_1_, camMatrix_2_, distCoeffs_2_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr tf_detected_publisher_;
    Eigen::Matrix4d camera_transform_1_, camera_transform_2_;
    double marker_length_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Ptr<cv::aruco::DetectorParameters> detectorParams_;
    cv::Ptr<cv::aruco::Dictionary> dictionary_;
    std::mutex detection_mutex_;

    std::unordered_map<int, bool> first_detection_1_, first_detection_2_;
    std::unordered_map<int, cv::Vec3d> first_tvecs_1_, first_tvecs_2_;
    std::unordered_map<int, cv::Vec3d> first_rvecs_1_, first_rvecs_2_;
    std::unordered_set<int> previously_detected_1_, previously_detected_2_;

    struct MouseCallbackData {
        ArucoDetectorDouble* self;
        std::string window_name;
    };
    MouseCallbackData mouse_data_1_;
    MouseCallbackData mouse_data_2_;

    void configureCamera(cv::VideoCapture& cap)
    {
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(cv::CAP_PROP_BRIGHTNESS, 0.5);
        cap.set(cv::CAP_PROP_CONTRAST, 0.7);
        cap.set(cv::CAP_PROP_SATURATION, 0.6);
        cap.set(cv::CAP_PROP_SHARPNESS, 4);
        cap.set(cv::CAP_PROP_GAMMA, 0.5);
        cap.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 5000);
        cap.set(cv::CAP_PROP_AUTO_WB, 0);
        cap.set(cv::CAP_PROP_EXPOSURE, -5);
        cap.set(cv::CAP_PROP_FPS, 15);

        double width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        double height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        double fps = cap.get(cv::CAP_PROP_FPS);
        std::cout << "Camera Resolution: " << width << "x" << height << ", FPS: " << fps << std::endl;
    }

    void readCameraCalibration(const std::string& filename, cv::Mat& camMatrix, cv::Mat& distCoeffs)
    {
        cv::FileStorage fs(filename, cv::FileStorage::READ);
        if (!fs.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file: %s", filename.c_str());
            return;
        }
        fs["camera_matrix"] >> camMatrix;
        fs["distortion_coefficients"] >> distCoeffs;
        fs.release();
    }

    void readTransforms(const std::string& filename, int camera_id, Eigen::Matrix4d& transform)
    {
        try
        {
            YAML::Node config = YAML::LoadFile(filename);
            if (!config["camera"])
            {
                RCLCPP_ERROR(this->get_logger(), "No camera transforms found in: %s", filename.c_str());
                return;
            }
            for (const auto& camera : config["camera"])
            {
                if (camera["id"].as<int>() == camera_id)
                {
                    transform = parseTransform(camera["transform"]);
                    return;
                }
            }
            RCLCPP_ERROR(this->get_logger(), "Transform for camera %d not found in: %s", camera_id, filename.c_str());
        }
        catch (const YAML::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "YAML error in '%s': %s", filename.c_str(), e.what());
        }
    }

    Eigen::Matrix4d parseTransform(const YAML::Node& node)
    {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        for (int i = 0; i < node.size(); ++i)
            for (int j = 0; j < node[i].size(); ++j)
                transform(i, j) = node[i][j].as<double>();
        return transform;
    }

    void configureDetectorParameters()
    {
        detectorParams_ = cv::aruco::DetectorParameters::create();
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    }

    void processFrames()
    {
        if (!rclcpp::ok()) return;

        auto markers_1 = processSingleFrame(cap_1_, camMatrix_1_, distCoeffs_1_, camera_transform_1_, "Camera 1",
                                            first_detection_1_, first_tvecs_1_, first_rvecs_1_, previously_detected_1_);
        auto markers_2 = processSingleFrame(cap_2_, camMatrix_2_, distCoeffs_2_, camera_transform_2_, "Camera 2",
                                            first_detection_2_, first_tvecs_2_, first_rvecs_2_, previously_detected_2_);
        fuseAndPublishMarkers(markers_1, markers_2);
    }

    std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> processSingleFrame(
        cv::VideoCapture& cap, cv::Mat& camMatrix, cv::Mat& distCoeffs, Eigen::Matrix4d& camera_transform,
        const std::string& camera_name, std::unordered_map<int, bool>& first_detection,
        std::unordered_map<int, cv::Vec3d>& first_tvecs, std::unordered_map<int, cv::Vec3d>& first_rvecs,
        std::unordered_set<int>& previously_detected)
    {
        cv::Mat frame, gray, filtered, enhanced, undistortedFrame;
        std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> detected_markers;

        cap >> frame;
        if (frame.empty())
        {
            RCLCPP_WARN(this->get_logger(), "%s: Empty frame received", camera_name.c_str());
            undistortedFrame = cv::Mat::zeros(720, 1280, CV_8UC3);
            cv::putText(undistortedFrame, "No frame from " + camera_name, cv::Point(50, 360),
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        }
        else
        {
            int height = frame.rows;
            int width = frame.cols;
            cv::Mat camMatrixNew = cv::getOptimalNewCameraMatrix(camMatrix, distCoeffs, cv::Size(width, height), 1, cv::Size(width, height));
            cv::undistort(frame, undistortedFrame, camMatrixNew, distCoeffs);

            cv::cvtColor(undistortedFrame, gray, cv::COLOR_BGR2GRAY);
            // Invert grayscale image to handle inverted ArUco markers (white markers on black background)
            cv::bitwise_not(gray, gray);
            cv::bilateralFilter(gray, filtered, 3, 75, 20);
            cv::equalizeHist(filtered, enhanced);

            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            cv::aruco::detectMarkers(gray, dictionary_, markerCorners, markerIds, detectorParams_);

            if (!markerCorners.empty())
            {
                cv::TermCriteria criteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 100, 0.001);
                for (auto& corners : markerCorners)
                    cv::cornerSubPix(gray, corners, cv::Size(5, 5), cv::Size(-1, -1), criteria);
            }

            std::unordered_set<int> currently_detected(markerIds.begin(), markerIds.end());
            for (int prev_marker : previously_detected)
            {
                if (currently_detected.find(prev_marker) == currently_detected.end())
                {
                    // RCLCPP_WARN(this->get_logger(), "%s: Marker %d lost, reinitializing", camera_name.c_str(), prev_marker);
                    first_detection.erase(prev_marker);
                    first_tvecs.erase(prev_marker);
                    first_rvecs.erase(prev_marker);
                }
            }
            previously_detected = currently_detected;

            if (!markerIds.empty())
            {
                std::vector<cv::Vec3d> rvecs(markerIds.size()), tvecs(markerIds.size());
                std::vector<cv::Point3f> markerPoints = {
                    cv::Point3f(-marker_length_ / 2, marker_length_ / 2, 0),
                    cv::Point3f(marker_length_ / 2, marker_length_ / 2, 0),
                    cv::Point3f(marker_length_ / 2, -marker_length_ / 2, 0),
                    cv::Point3f(-marker_length_ / 2, -marker_length_ / 2, 0)
                };

                for (size_t i = 0; i < markerIds.size(); ++i)
                {
                    int marker_id = markerIds[i];
                    if (markerCorners[i].size() != 4 || !cv::solvePnP(markerPoints, markerCorners[i], camMatrixNew, distCoeffs, rvecs[i], tvecs[i], false, cv::SOLVEPNP_IPPE_SQUARE))
                        continue;

                    if (first_detection.count(marker_id) == 0)
                    {
                        first_detection[marker_id] = true;
                        first_tvecs[marker_id] = tvecs[i];
                        first_rvecs[marker_id] = rvecs[i];
                     }
                     else {
                        // Convert current and previous rvecs to rotation matrices
                        cv::Mat R_current, R_previous;
                        cv::Rodrigues(rvecs[i], R_current);
                        cv::Rodrigues(first_rvecs[marker_id], R_previous);

                        // Compute the relative rotation
                        cv::Mat R_diff = R_current * R_previous.t();
                        cv::Mat rvec_diff;
                        cv::Rodrigues(R_diff, rvec_diff);

                        // Check the angle of the relative rotation
                        double angle = cv::norm(rvec_diff);
                        const double angle_threshold = 0.02; // ~5.7 degrees

                        // If the angle is large, it may indicate a sign flip or significant change
                        if (angle > angle_threshold) {
                            // Try flipping the current rvec
                            cv::Vec3d flipped_rvec = -rvecs[i];
                            cv::Mat R_flipped;
                            cv::Rodrigues(flipped_rvec, R_flipped);
                            cv::Mat R_diff_flipped = R_flipped * R_previous.t();
                            cv::Mat rvec_diff_flipped;
                            cv::Rodrigues(R_diff_flipped, rvec_diff_flipped);
                            double angle_flipped = cv::norm(rvec_diff_flipped);

                            // Choose the rvec that results in a smaller rotation difference
                            if (angle_flipped < angle) {
                                rvecs[i] = flipped_rvec;
                            }
                        }
                    first_tvecs[marker_id] = tvecs[i];
                    first_rvecs[marker_id] = rvecs[i];
                    }

                    detected_markers[marker_id] = {rvecs[i], tvecs[i]};

                    // RCLCPP_INFO(this->get_logger(), "%s: Marker %d - rvec: [%f, %f, %f], tvec: [%f, %f, %f]",
                    //             camera_name.c_str(), marker_id,
                    //             rvecs[i][0], rvecs[i][1], rvecs[i][2],
                    //             tvecs[i][0], tvecs[i][1], tvecs[i][2]);

                    cv::aruco::drawDetectedMarkers(undistortedFrame, markerCorners, markerIds);
                    std::vector<cv::Point3f> axisPoints = {
                        cv::Point3f(0, 0, 0), cv::Point3f(marker_length_ * 1.5f, 0, 0),
                        cv::Point3f(0, marker_length_ * 1.5f, 0), cv::Point3f(0, 0, marker_length_ * 1.5f)
                    };
                    std::vector<cv::Point2f> imagePoints;
                    cv::projectPoints(axisPoints, rvecs[i], tvecs[i], camMatrix, distCoeffs, imagePoints);
                    cv::line(undistortedFrame, imagePoints[0], imagePoints[1], cv::Scalar(0, 0, 255), 2);
                    cv::line(undistortedFrame, imagePoints[0], imagePoints[2], cv::Scalar(0, 255, 0), 2);
                    cv::line(undistortedFrame, imagePoints[0], imagePoints[3], cv::Scalar(255, 0, 0), 2);
                }
            }
        }

        cv::imshow(camera_name, undistortedFrame);
        if (camera_name == "Camera 1") {
            cv::setMouseCallback(camera_name, onMouse, &mouse_data_1_);
        } else if (camera_name == "Camera 2") {
            cv::setMouseCallback(camera_name, onMouse, &mouse_data_2_);
        }
        cv::waitKey(1);

        return detected_markers;
    }

    void fuseAndPublishMarkers(const std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>>& markers_1,
                               const std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>>& markers_2)
    {
        std::lock_guard<std::mutex> lock(detection_mutex_);
        std::unordered_map<int, std::pair<cv::Vec3d, cv::Vec3d>> fused_markers;

        std::unordered_set<int> all_ids;
        for (const auto& [id, _] : markers_1) all_ids.insert(id);
        for (const auto& [id, _] : markers_2) all_ids.insert(id);

        for (int id : all_ids)
        {
            std::vector<cv::Vec3d> rvecs, tvecs;
            std::vector<Eigen::Matrix4d> camera_transforms;

            if (markers_1.count(id))
            {
                rvecs.push_back(markers_1.at(id).first);
                tvecs.push_back(markers_1.at(id).second);
                camera_transforms.push_back(camera_transform_1_);
            }
            if (markers_2.count(id))
            {
                rvecs.push_back(markers_2.at(id).first);
                tvecs.push_back(markers_2.at(id).second);
                camera_transforms.push_back(camera_transform_2_);
            }

            if (!rvecs.empty())
            {
                // Average translation with explicit cast to double
                cv::Vec3d tvec = {0, 0, 0};
                for (const auto& t : tvecs) tvec += t;
                tvec /= static_cast<double>(tvecs.size());

                // Convert rvecs to quaternions and average
                std::vector<Eigen::Quaterniond> quaternions;
                for (const auto& rvec : rvecs)
                {
                    cv::Mat rot_mat;
                    cv::Rodrigues(rvec, rot_mat);
                    Eigen::Matrix3d eigen_rot;
                    for (int i = 0; i < 3; ++i)
                        for (int j = 0; j < 3; ++j)
                            eigen_rot(i, j) = rot_mat.at<double>(i, j);
                    quaternions.emplace_back(eigen_rot);
                }

                // Simple quaternion averaging
                Eigen::Quaterniond avg_quat(0, 0, 0, 0);
                for (const auto& q : quaternions)
                {
                    avg_quat.coeffs() += q.coeffs();
                }
                avg_quat.coeffs() /= static_cast<double>(quaternions.size());
                avg_quat.normalize();

                // Convert back to rvec
                Eigen::Matrix3d avg_rot = avg_quat.toRotationMatrix();
                cv::Mat avg_rot_mat(3, 3, CV_64F);
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        avg_rot_mat.at<double>(i, j) = avg_rot(i, j);
                cv::Vec3d rvec;
                cv::Rodrigues(avg_rot_mat, rvec);

                fused_markers[id] = {rvec, tvec};
                publishTransform(rvec, tvec, id, rvecs.size() == 2 ? "both" : (markers_1.count(id) ? "camera_1" : "camera_2"));
            }
        }
    }

    void publishTransform(const cv::Vec3d& rvec, const cv::Vec3d& tvec, int marker_id, const std::string& source)
    {
        cv::Mat rotation_matrix;
        cv::Rodrigues(rvec, rotation_matrix);
        Eigen::Matrix4d marker_transform = Eigen::Matrix4d::Identity();
        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
                marker_transform(row, col) = rotation_matrix.at<double>(row, col);
            marker_transform(row, 3) = tvec[row];
        }

        // Use appropriate camera transform based on source
        Eigen::Matrix4d world_to_marker;
        if (source == "camera_2")
        {
            world_to_marker = camera_transform_2_ * marker_transform;
        }
        else if (source == "camera_1")
        {
            world_to_marker = camera_transform_1_ * marker_transform;
        }
        else // "both" - use camera_1 as default
        {
            world_to_marker = camera_transform_1_ * marker_transform;
        }

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "aruco_" + std::to_string(marker_id);
        transformStamped.transform.translation.x = world_to_marker(0, 3);
        transformStamped.transform.translation.y = world_to_marker(1, 3);
        transformStamped.transform.translation.z = world_to_marker(2, 3);

        Eigen::Matrix3d rotation = world_to_marker.block<3, 3>(0, 0);
        Eigen::Quaterniond quaternion(rotation);
        transformStamped.transform.rotation.x = quaternion.x();
        transformStamped.transform.rotation.y = quaternion.y();
        transformStamped.transform.rotation.z = quaternion.z();
        transformStamped.transform.rotation.w = quaternion.w();

        tf_broadcaster_.sendTransform(transformStamped);
        if (marker_id == 19)
            tf_detected_publisher_->publish(transformStamped);

        // RCLCPP_INFO(this->get_logger(), "Marker %d fused from %s - tvec: [%f, %f, %f], quat: [%f, %f, %f, %f]",
        //             marker_id, source.c_str(),
        //             transformStamped.transform.translation.x,
        //             transformStamped.transform.translation.y,
        //             transformStamped.transform.translation.z,
        //             quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());
    }

    static void onMouse(int event, int x, int y, int flags, void* userdata)
    {
        MouseCallbackData* data = static_cast<MouseCallbackData*>(userdata);
        if (event == cv::EVENT_MOUSEMOVE)
        {
            std::stringstream ss;
            ss << data->window_name << " - Cursor: (" << x << ", " << y << ")";
            cv::setWindowTitle(data->window_name, ss.str());
        }
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArucoDetectorDouble>());
    rclcpp::shutdown();
    return 0;
}