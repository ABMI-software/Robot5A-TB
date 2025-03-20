#include "robot_hardware_interface/SlushEngineHardware.hpp"

// Mock Slush implementations (replace with real Slush library if available)
namespace Slush {
sBoard::sBoard() {}
Motor::Motor(int id) { (void)id; }
void Motor::setMicroSteps(int steps) { (void)steps; }
void Motor::setCurrent(int a, int b, int c, int d) { (void)a; (void)b; (void)c; (void)d; }
void Motor::goTo(double position) { (void)position; }
}  // namespace Slush

namespace robot_hardware_interface {

SlushEngineHardware::SlushEngineHardware() = default;

hardware_interface::CallbackReturn SlushEngineHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        board_ = std::make_shared<Slush::sBoard>();
        RCLCPP_INFO(rclcpp::get_logger("SlushEngineHardware"), "SlushEngine board initialized.");
        for (size_t i = 0; i < joint_names_.size(); ++i) {
            joints_[joint_names_[i]] = std::make_shared<Slush::Motor>(i);
            position_commands_[joint_names_[i]] = 0.0;
            position_states_[joint_names_[i]] = 0.0;
            velocity_states_[joint_names_[i]] = 0.0;  // Initialize velocity
        }
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("SlushEngineHardware"), "Error initializing SlushEngine: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_configure(const rclcpp_lifecycle::State &) {
    try {
        node_ = std::make_shared<rclcpp::Node>("slush_engine_hardware_node");
        for (const auto &[name, motor] : joints_) {
            motor->setMicroSteps(16);
            motor->setCurrent(50, 50, 50, 50);
            RCLCPP_INFO(node_->get_logger(), "%s motor configured.", name.c_str());
        }

        visual_joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "visual_joint_states", 10,
            std::bind(&SlushEngineHardware::visual_joint_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to visual_joint_states topic.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Error configuring hardware: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SlushEngineHardware::visual_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (position_states_.count(msg->name[i])) {
            position_states_[msg->name[i]] = msg->position[i];
            if (i < msg->velocity.size()) {  // Check if velocity data is provided
                velocity_states_[msg->name[i]] = msg->velocity[i];
            }
        }
    }
}

std::vector<hardware_interface::StateInterface> SlushEngineHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto &name : joint_names_) {
        state_interfaces.emplace_back(name, "position", &position_states_[name]);
        state_interfaces.emplace_back(name, "velocity", &velocity_states_[name]);  // Export velocity
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SlushEngineHardware::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (const auto &name : joint_names_) {
        command_interfaces.emplace_back(name, "position", &position_commands_[name]);
    }
    return command_interfaces;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_activate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(node_->get_logger(), "Starting SlushEngine hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(node_->get_logger(), "Stopping SlushEngine hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SlushEngineHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    rclcpp::spin_some(node_);
    RCLCPP_DEBUG(node_->get_logger(), "Read joint states: %f, %f, ...", position_states_["R0_Yaw"], position_states_["R1_Pitch"]);
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SlushEngineHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    for (const auto &[name, motor] : joints_) {
        double command = position_commands_[name];
        motor->goTo(command);
        RCLCPP_DEBUG(node_->get_logger(), "Sent command %f to %s", command, name.c_str());
    }
    return hardware_interface::return_type::OK;
}

}  // namespace robot_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_hardware_interface::SlushEngineHardware, hardware_interface::SystemInterface)