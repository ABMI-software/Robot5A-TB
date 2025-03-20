#ifndef SLUSH_ENGINE_HARDWARE_HPP_
#define SLUSH_ENGINE_HARDWARE_HPP_

#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// Hypothetical Slush C++ library (replace with actual API if available)
namespace Slush {
class sBoard {
public:
    sBoard();
};
class Motor {
public:
    Motor(int id);
    void setMicroSteps(int steps);
    void setCurrent(int a, int b, int c, int d);
    void goTo(double position);
};
}  // namespace Slush

namespace robot_hardware_interface {

class SlushEngineHardware : public hardware_interface::SystemInterface {
public:
    SlushEngineHardware();
    ~SlushEngineHardware() override = default;

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
    hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
    void visual_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

    std::shared_ptr<Slush::sBoard> board_;
    std::map<std::string, std::shared_ptr<Slush::Motor>> joints_;
    std::map<std::string, double> position_commands_;
    std::map<std::string, double> position_states_;
    std::map<std::string, double> velocity_states_;  // Added for velocity
    std::vector<std::string> joint_names_ = {"R0_Yaw", "R1_Pitch", "R2_Pitch", "R3_Yaw", "R4_Pitch", "ServoGear"};

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr visual_joint_state_subscriber_;
};

}  // namespace robot_hardware_interface

#endif  // SLUSH_ENGINE_HARDWARE_HPP_