#include "robot_hardware_interface/SlushEngineHardware.hpp"

// Mock Slush implementations (replace with real Slush library when available)
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

        // Populate joint_names_ from HardwareInfo
        joint_names_.clear();
        for (const auto &joint : info.joints) {
            joint_names_.push_back(joint.name);
        }

        // Initialize joints with optional IDs from URDF
        for (size_t i = 0; i < info.joints.size(); ++i) {
            const auto &joint = info.joints[i];
            int motor_id = i;  // Fallback to index if 'id' is missing
            if (joint.parameters.find("id") != joint.parameters.end()) {
                motor_id = std::stoi(joint.parameters.at("id"));
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SlushEngineHardware"), "Joint '%s' missing 'id' parameter; using index %zu", joint.name.c_str(), i);
            }
            joints_[joint.name] = std::make_shared<Slush::Motor>(motor_id);
            position_commands_[joint.name] = 0.0;
            position_states_[joint.name] = 0.0;
            velocity_states_[joint.name] = 0.0;
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

        joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&SlushEngineHardware::joint_state_callback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to joint_states topic.");
    } catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Error configuring hardware: %s", e.what());
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SlushEngineHardware::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
        if (position_states_.count(msg->name[i])) {
            position_states_[msg->name[i]] = msg->position[i];
            if (i < msg->velocity.size()) {
                velocity_states_[msg->name[i]] = msg->velocity[i];
            } else {
                velocity_states_[msg->name[i]] = 0.0;
            }
        }
    }
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
    for (const auto &[name, state] : position_states_) {
        position_states_[name] = 0.0;
        velocity_states_[name] = 0.0;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SlushEngineHardware::on_deactivate(const rclcpp_lifecycle::State &) {
    RCLCPP_INFO(node_->get_logger(), "Stopping SlushEngine hardware interface...");
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type SlushEngineHardware::read(const rclcpp::Time &, const rclcpp::Duration &) {
    rclcpp::spin_some(node_);  // Process /joint_states for visual feedback

    // Fallback for joints without feedback (R4_Pitch, ServoGear)
    position_states_["R4_Pitch"] = position_commands_["R4_Pitch"];
    position_states_["ServoGear"] = position_commands_["ServoGear"];
    velocity_states_["R4_Pitch"] = 0.0;
    velocity_states_["ServoGear"] = 0.0;

    RCLCPP_DEBUG(node_->get_logger(), "Read joint states: R0_Yaw=%f, R1_Pitch=%f, R4_Pitch=%f, ServoGear=%f",
                 position_states_["R0_Yaw"], position_states_["R1_Pitch"],
                 position_states_["R4_Pitch"], position_states_["ServoGear"]);
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