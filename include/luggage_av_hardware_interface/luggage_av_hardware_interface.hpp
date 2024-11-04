#ifndef LUGGAGE_AV_HARDWARE_INTERFACE__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_
#define LUGGAGE_AV_HARDWARE_INTERFACE__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"

#include <poll.h>

namespace luggage_av_hardware_interface {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LuggageAVHardawreInterface : public hardware_interface::SystemInterface {
private:
    char* dev_;
    pollfd poll_fd_;

    int32_t hw_cmd_min_;
    int32_t hw_cmd_max_;

    double lin_vel_min_;
    double lin_vel_max_;

    struct Wheel {
        std::string velocity_command_interface_name;
        std::string position_state_interface_name;
        std::string velocity_state_interface_name;
    };

    Wheel wheel_L_;
    Wheel wheel_R_;
    
public:

    // from LifecycleNodeInterface
    hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);
    hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state);

    // from SystemInterface
    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& hardware_info);
    // CallbackReturn export_state_interfaces();
    // CallbackReturn export_command_interfaces();
    hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period);
    hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period);

};

}

#endif  // LUGGAGE_AV_HARDWARE_INTERFACE__LUGGAGE_AV_HARDWARE_INTERFACE_HPP_
