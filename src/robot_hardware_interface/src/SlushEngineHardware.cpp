#include "robot_hardware_interface/SlushEngineHardware.hpp"

namespace robot_hardware_interface {

SlushEngineHardware::SlushEngineHardware() = default;

hardware_interface::CallbackReturn SlushEngineHardware::on_init(const hardware_interface::HardwareInfo &info) {
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
        return hardware_interface::CallbackReturn::ERROR;
    }

    try {
        joint_names_.clear();
        for (const auto &joint : info.joints) {
            joint_names_.push_back(joint.name);
            double steps_per_rad = std::stod(joint.parameters.at("steps_per_radian"));
            steps_per_radian_[joint.name] = steps_per_rad;
            position_commands_[joint.name] = 0.0;
            position_states_[joint.name] = 0.0;
            velocity_states_[joint.name] = 0.0;
        }
        RCLCPP_INFO(rclcpp::get_logger("SlushEngineHardware"), "Initialized %zu joints.", joint_names_.size());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("SlushEngineHardware"), "Error initializing: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_configure(const rclcpp_lifecycle::State &) {
    node_ = std::make_shared<rclcpp::Node>("slush_engine_hardware_node");
    command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/slush_commands", 10);
    RCLCPP_INFO(node_->get_logger(), "Configured SlushEngine hardware interface.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SlushEngineHardware::export_state_interfaces() {
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (const auto &name : joint_names_) {
        state_interfaces.emplace_back(name, "position", &position_states_[name]);
        state_interfaces.emplace_back(name, "velocity", &velocity_states_[name]);
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
    for (auto &[name, state] : position_states_) {
        state = 0.0;
        velocity_states_[name] = 0.0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(node_->get_logger(), "Stopping SlushEngine hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SlushEngineHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    // Real hardware: feedback would come from slush_engine_communication if supported
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SlushEngineHardware::write(const rclcpp::Time &, const rclcpp::Duration &) {
    sensor_msgs::msg::JointState command_msg;
    command_msg.header.stamp = node_->now();
    command_msg.name = joint_names_;
    command_msg.position.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
        const auto &name = joint_names_[i];
        double command_rad = position_commands_[name];
        command_msg.position[i] = command_rad * steps_per_radian_[name]; // Send steps
        // Mock feedback until real hardware
        position_states_[name] = command_rad;
        velocity_states_[name] = 0.0;
        RCLCPP_DEBUG(node_->get_logger(), "Commanded %s: %f rad (%f steps)", name.c_str(), command_rad, command_msg.position[i]);
    }

    command_publisher_->publish(command_msg);
    return hardware_interface::return_type::OK;
}

}  // namespace robot_hardware_interface

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(robot_hardware_interface::SlushEngineHardware, hardware_interface::SystemInterface)