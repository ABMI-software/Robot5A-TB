#ifndef SLUSH_ENGINE_HARDWARE_HPP_
#define SLUSH_ENGINE_HARDWARE_HPP_

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <string>

namespace robot_hardware_interface {

class SlushEngineHardware : public hardware_interface::SystemInterface {
public:
    SlushEngineHardware();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    std::vector<std::string> joint_names_;
    std::map<std::string, double> steps_per_radian_;
    std::map<std::string, double> position_commands_;
    std::map<std::string, double> position_states_;
    std::map<std::string, double> velocity_states_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr command_publisher_;
};

}  // namespace robot_hardware_interface

#endif  // SLUSH_ENGINE_HARDWARE_HPP_