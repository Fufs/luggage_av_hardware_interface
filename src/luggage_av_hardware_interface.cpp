#include "luggage_av_hardware_interface/luggage_av_hardware_interface.hpp"

#include "hardware_interface/lexical_casts.hpp"

#include "defaults.hpp"
#include "proto/wheel_commands.pb.h"
#include "cobs.h"

#include <fcntl.h>
#include <unistd.h>
#include <cstdint>


namespace luggage_av_hardware_interface {

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_init(const hardware_interface::HardwareInfo& hardware_info) {
        if(hardware_interface::SystemInterface::on_init(hardware_info) != hardware_interface::CallbackReturn::SUCCESS) 
            return hardware_interface::CallbackReturn::ERROR;

        // Perform URDF checks
        //// Check if this system has exactly 2 joints
        if(get_hardware_info().joints.size() != 2) {
            RCLCPP_FATAL(
                get_logger(), "System '%s' has %zu joints. 2 expected.",
                get_hardware_info().name.c_str(), get_hardware_info().joints.size());

            return hardware_interface::CallbackReturn::ERROR;
        }

        for(hardware_interface::ComponentInfo& joint : info_.joints) {
            //// Check that only a single command interface exist for the joint
            if(joint.command_interfaces.size() != 1) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
                    joint.name.c_str(), joint.command_interfaces.size());

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the only existing command interface is a velocity interface
            if(joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
                    joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that there are exactly 2 state interfaces
            if(joint.state_interfaces.size() != 2) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
                    joint.state_interfaces.size());
            
                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the first state interface is a position interface
            if(joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as first state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                    hardware_interface::HW_IF_POSITION);

                return hardware_interface::CallbackReturn::ERROR;
            }

            //// Check that the second state interface is a velocity interface
            if(joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
                RCLCPP_FATAL(
                    get_logger(), "Joint '%s' have '%s' as second state interface. '%s' expected.",
                    joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                    hardware_interface::HW_IF_VELOCITY);
                
                return hardware_interface::CallbackReturn::ERROR;
            }
        }

        // Initialize class memebers
        auto it = get_hardware_info().hardware_parameters.find("device");
        if(it != get_hardware_info().hardware_parameters.end()) {
            dev_ = const_cast<char*>(it->second.c_str());
        }
        else {
            dev_ = luggage_av_hardware_interface_defaults.dev;
        }
        
        it = get_hardware_info().hardware_parameters.find("linear_velocity_min");
        if(it != get_hardware_info().hardware_parameters.end()) {
            lin_vel_min_ = hardware_interface::stod(it->second);
        }
        else {
            lin_vel_min_ = luggage_av_hardware_interface_defaults.lin_vel_min;
        }

        it = get_hardware_info().hardware_parameters.find("linear_velocity_max");
        if(it != get_hardware_info().hardware_parameters.end()) {
            lin_vel_max_ = hardware_interface::stod(it->second);
        }
        else {
            lin_vel_max_ = luggage_av_hardware_interface_defaults.lin_vel_max;
        }

        it = get_hardware_info().hardware_parameters.find("hardware_command_min");
        if(it != get_hardware_info().hardware_parameters.end()) {
            hw_cmd_min_ = hardware_interface::stod(it->second);
        }
        else {
            hw_cmd_min_ = luggage_av_hardware_interface_defaults.hw_cmd_min;
        }

        it = get_hardware_info().hardware_parameters.find("hardware_command_max");
        if(it != get_hardware_info().hardware_parameters.end()) {
            hw_cmd_max_ = hardware_interface::stod(it->second);
        }
        else {
            hw_cmd_max_ = luggage_av_hardware_interface_defaults.hw_cmd_max;
        }

        // TODO: Encoder steps per rotation parameter

        wheel_L_.velocity_command_interface_name = info_.joints[0].name + "/velocity";
        wheel_L_.position_state_interface_name = info_.joints[0].name + "/position";
        wheel_L_.velocity_state_interface_name = info_.joints[0].name + "/velocity";
        wheel_R_.velocity_command_interface_name = info_.joints[1].name + "/velocity";
        wheel_R_.position_state_interface_name = info_.joints[1].name + "/position";
        wheel_R_.velocity_state_interface_name = info_.joints[1].name + "/velocity";
        

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Setup communication with I/O processor

        poll_fd_.fd = open(dev_, O_RDWR);

        if(poll_fd_.fd < 0) {
            RCLCPP_ERROR(get_logger(), "Unable to open %s", dev_);
            
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Successfully opened %s", dev_);
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Unsetup communication with I/O processor
        if(poll_fd_.fd <= 0) return hardware_interface::CallbackReturn::SUCCESS; // The file descriptor was never opened

        if(close(poll_fd_.fd) < 0) {
            RCLCPP_ERROR(get_logger(), "Unable to close %s", dev_);

            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(get_logger(), "Successfully closed %s", dev_);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Enable the hardware

        return hardware_interface::CallbackReturn::SUCCESS;
        
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Disable the hardware

        return hardware_interface::CallbackReturn::SUCCESS;

    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Shutdown hardware gracefully

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn LuggageAVHardawreInterface::on_error(const rclcpp_lifecycle::State& /*previous_state*/) {
        // TODO: Handle errors

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type LuggageAVHardawreInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // TODO: Read from I/O processor and update state interfaces

        set_state(wheel_L_.velocity_state_interface_name, get_command(wheel_L_.velocity_command_interface_name));
        set_state(wheel_R_.velocity_state_interface_name, get_command(wheel_R_.velocity_command_interface_name));

        return hardware_interface::return_type::OK;

    }

    WheelCommands wc_msg;
    std::string wh_msg;
    uint8_t out_buf[256];

    hardware_interface::return_type LuggageAVHardawreInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
        // TODO: Get commands from commands interfaces and write to I/O processor

        // 1. Read the commands from WheelInterfaces.velocity_commands with get_command()
        // 2. Update msg object
        
        wc_msg.set_velocity_left((hw_cmd_max_-hw_cmd_min_)*(get_command(wheel_L_.velocity_command_interface_name)-lin_vel_min_)/(lin_vel_max_-lin_vel_min_)+hw_cmd_min_);
        wc_msg.set_velocity_right((hw_cmd_max_-hw_cmd_min_)*(get_command(wheel_R_.velocity_command_interface_name)-lin_vel_min_)/(lin_vel_max_-lin_vel_min_)+hw_cmd_min_);
        // wc_msg.set_velocity_left(get_command(wheel_L_.velocity_command_interface_name)*1000);
        // wc_msg.set_velocity_right(get_command(wheel_R_.velocity_command_interface_name)*1000);
        
        // // // 3. encode protobuf message
        wc_msg.SerializeToString(&wh_msg);
    
        // 4. TODO: CRC
        // 5. Encode cobs
                                                                // TODO: Find out why out_buf.size() is not available
        cobs_encode_result encode_result = cobs_encode(&out_buf, 256, wh_msg.c_str(), wh_msg.size());
        
        if(encode_result.status != cobs_encode_status::COBS_ENCODE_OK) {
            if(encode_result.status & cobs_encode_status::COBS_ENCODE_NULL_POINTER) {
                RCLCPP_ERROR(get_logger(), "A null pointer was passed to cobs_encode function");
            }
            if(encode_result.status & cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW) {
                RCLCPP_ERROR(get_logger(), "Buffer overflow detected when encoding COBS");
            }
            if(!(encode_result.status & (cobs_encode_status::COBS_ENCODE_NULL_POINTER | cobs_encode_status::COBS_ENCODE_OUT_BUFFER_OVERFLOW))) {
                RCLCPP_ERROR(get_logger(), "Unknown error when encoding COBS");
            }

            return hardware_interface::return_type::ERROR;
        }

        // 6. Add a final \0
        if(encode_result.out_len == 256) {
            RCLCPP_ERROR(get_logger(), "Buffer overflow detected when appending final \\0");

            return hardware_interface::return_type::ERROR;
        }

        out_buf[encode_result.out_len] = 0;
        
        // 7. Write to poll_fd_.fd
        // TODO: Check number of bytes written
        ::write(poll_fd_.fd, &out_buf, encode_result.out_len+1);

        return hardware_interface::return_type::OK;
    }
    
}  // namespace luggage_av_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(luggage_av_hardware_interface::LuggageAVHardawreInterface, hardware_interface::SystemInterface);
 