#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

class JointStateBridge : public rclcpp::Node
{
public:
    JointStateBridge()
        : Node("joint_state_bridge")
    {
        // Declare parameter for open-loop mode
        this->declare_parameter<bool>("open_loop", true); // Default to open-loop
        open_loop_ = this->get_parameter("open_loop").as_bool();

        // Define all joint names
        joint_names_ = {"R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch", "ServoGear"};

        // Initialize true_joint_states_ with default values
        true_joint_states_ = std::make_shared<sensor_msgs::msg::JointState>();
        true_joint_states_->header.frame_id = "";
        true_joint_states_->name = joint_names_;
        true_joint_states_->position.resize(joint_names_.size(), 0.0);
        true_joint_states_->velocity.resize(joint_names_.size(), 0.0);
        true_joint_states_->effort.resize(joint_names_.size(), 0.0);

        // Initialize subscribers
        if (!open_loop_) {
            visual_joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "/visual_joint_states",
                rclcpp::QoS(10).reliable(),
                std::bind(&JointStateBridge::visual_joint_state_callback, this, std::placeholders::_1));
        }

        arm_controller_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/arm_controller/controller_state",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::arm_controller_callback, this, std::placeholders::_1));

        gripper_controller_subscriber_ = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>(
            "/gripper_controller/controller_state",
            rclcpp::QoS(10).reliable(),
            std::bind(&JointStateBridge::gripper_controller_callback, this, std::placeholders::_1));

        // Initialize publishers
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states", rclcpp::QoS(10).reliable());

        true_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/true_joint_states", rclcpp::QoS(10).reliable());

        combined_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/combined_joint_states", rclcpp::QoS(10).reliable());

        robot_joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "robot/joint_states", rclcpp::QoS(10).reliable());

        // Timer to publish default joint states periodically until real data arrives
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), // 10 Hz
            std::bind(&JointStateBridge::publish_default_joint_states, this));
    }

private:
    void publish_default_joint_states()
    {
        // Update timestamp and publish default states if no real data yet
        true_joint_states_->header.stamp = this->now();
        joint_state_publisher_->publish(*true_joint_states_);

        // If we have controller data, stop the timer
        if (arm_joint_states_ != nullptr && gripper_joint_states_ != nullptr) {
            timer_->cancel();
        }
    }

    void visual_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        visual_joint_states_ = msg;
        if (!open_loop_ && true_joint_states_ != nullptr) {
            publish_combined_joint_states();
        }
    }

    void arm_controller_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        arm_joint_states_ = msg;
        recreate_joint_states();
    }

    void gripper_controller_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
    {
        gripper_joint_states_ = msg;
        recreate_joint_states();
    }

    void recreate_joint_states()
    {
        if (arm_joint_states_ == nullptr || gripper_joint_states_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "Arm or gripper joint states are null");
            return;
        }

        // Update true_joint_states_ with controller data
        true_joint_states_ = std::make_shared<sensor_msgs::msg::JointState>();
        true_joint_states_->header.stamp = this->now();
        true_joint_states_->header.frame_id = "";

        // Start with arm joint states
        true_joint_states_->name = arm_joint_states_->joint_names;
        true_joint_states_->position = arm_joint_states_->reference.positions;

        // Add gripper joint states
        for (size_t i = 0; i < gripper_joint_states_->joint_names.size(); ++i) {
            const auto& gripper_joint_name = gripper_joint_states_->joint_names[i];
            double gripper_position = gripper_joint_states_->reference.positions[i];

            auto it = std::find(true_joint_states_->name.begin(), true_joint_states_->name.end(), gripper_joint_name);
            if (it == true_joint_states_->name.end()) {
                true_joint_states_->name.push_back(gripper_joint_name);
                true_joint_states_->position.push_back(gripper_position);
            } else {
                size_t index = std::distance(true_joint_states_->name.begin(), it);
                true_joint_states_->position[index] = (true_joint_states_->position[index] + gripper_position) / 2.0;
            }
        }

        // Resize velocity and effort to match name size
        true_joint_states_->velocity.resize(true_joint_states_->name.size(), 0.0);
        true_joint_states_->effort.resize(true_joint_states_->name.size(), 0.0);

        // Publish the true joint states
        true_joint_state_publisher_->publish(*true_joint_states_);

        // In open-loop mode or if no visual data, publish true joint states to /joint_states
        if (open_loop_ || visual_joint_states_ == nullptr) {
            joint_state_publisher_->publish(*true_joint_states_);
        } else {
            publish_combined_joint_states();
        }
    }

    void publish_combined_joint_states()
    {
        if (true_joint_states_ == nullptr || visual_joint_states_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "True or visual joint states are null");
            return;
        }

        sensor_msgs::msg::JointState combined_joint_states;
        combined_joint_states.header = visual_joint_states_->header;

        // Start with true joint states
        combined_joint_states.name = true_joint_states_->name;
        combined_joint_states.position = true_joint_states_->position;
        combined_joint_states.velocity = true_joint_states_->velocity;
        combined_joint_states.effort = true_joint_states_->effort;

        // Overlay visual joint states
        for (size_t i = 0; i < visual_joint_states_->name.size(); ++i) {
            const auto& visual_joint_name = visual_joint_states_->name[i];
            double visual_position = visual_joint_states_->position[i];

            auto it = std::find(combined_joint_states.name.begin(), combined_joint_states.name.end(), visual_joint_name);
            if (it == combined_joint_states.name.end()) {
                combined_joint_states.name.push_back(visual_joint_name);
                combined_joint_states.position.push_back(visual_position);
                combined_joint_states.velocity.push_back(0.0);
                combined_joint_states.effort.push_back(0.0);
            } else {
                size_t index = std::distance(combined_joint_states.name.begin(), it);
                combined_joint_states.position[index] = visual_position; // Prefer visual data
            }
        }

        // Publish to all topics
        combined_joint_state_publisher_->publish(combined_joint_states);
        joint_state_publisher_->publish(combined_joint_states);
        // robot_joint_state_publisher_->publish(combined_joint_states);
    }

    // Member variables
    bool open_loop_;
    std::vector<std::string> joint_names_;
    sensor_msgs::msg::JointState::SharedPtr true_joint_states_;
    sensor_msgs::msg::JointState::SharedPtr visual_joint_states_;
    control_msgs::msg::JointTrajectoryControllerState::SharedPtr arm_joint_states_;
    control_msgs::msg::JointTrajectoryControllerState::SharedPtr gripper_joint_states_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr visual_joint_state_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr arm_controller_subscriber_;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr gripper_controller_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr combined_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr true_joint_state_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr robot_joint_state_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointStateBridge>());
    rclcpp::shutdown();
    return 0;
}