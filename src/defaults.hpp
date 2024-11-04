#include <cstdint>

namespace luggage_av_hardware_interface {

    struct {
        char* dev = const_cast<char*>("/dev/ttyACM0");
        double lin_vel_min = -1.0;
        double lin_vel_max = 1.0;
        int32_t hw_cmd_min = -8312;
        int32_t hw_cmd_max = 8312;
    } luggage_av_hardware_interface_defaults;

}
