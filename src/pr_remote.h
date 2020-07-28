#pragma once

#include <fmt/format.h>
#include <libserial/SerialPort.h>
#include <string>

namespace pr {

enum {
    STEERING_CHANNEL = 0,
    THROTTLE_CHANNEL,
    ESTOP_CHANNEL,
    TURBO_MODE_CHANNEL,
    PAN_CHANNEL,
    TILT_CHANNEL,
    NUM_CHANNELS
};

enum class TurboMode {
    FIRST_GEAR,
    SECOND_GEAR,
    THIRD_GEAR 
};

constexpr int ESTOP_MIN = 872;
constexpr int ESTOP_MAX = 2150;

constexpr int STEERING_MIN = 1195;
constexpr int STEERING_MAX = 1843;
constexpr int STEERING_CENTER = (STEERING_MIN + STEERING_MAX) / 2;

constexpr int THROTTLE_MIN = 1218;
constexpr int THROTTLE_MAX = 1814;
constexpr int THROTTLE_CENTER = (THROTTLE_MIN + THROTTLE_MAX) / 2;

constexpr int TURBO_MIN = 1218;
constexpr int TURBO_MAX = 1814;
constexpr int TURBO_CENTER = (TURBO_MIN + TURBO_MAX) / 2;

constexpr int PAN_MIN = 873;
constexpr int PAN_MAX = 2150;
constexpr int PAN_CENTER = (PAN_MIN + PAN_MAX) / 2;

constexpr int TILT_MIN = 873;
constexpr int TILT_MAX = 2150;
constexpr int TILT_CENTER = (TILT_MIN + TILT_MAX) / 2;

class RemoteControl {
    public:
        RemoteControl(const std::string& port_name) : port_name(port_name) {
            using namespace LibSerial;

            // Open the serial port.
            serial_port.Open(port_name);

            // Set the baud rate to 115200.
            serial_port.SetBaudRate(BaudRate::BAUD_115200);

            // Set the number of data bits.
            serial_port.SetCharacterSize(CharacterSize::CHAR_SIZE_8);

            // Turn off hardware flow control.
            serial_port.SetFlowControl(FlowControl::FLOW_CONTROL_NONE);

            // Disable parity.
            serial_port.SetParity(Parity::PARITY_NONE);

            // Set the number of stop bits.
            serial_port.SetStopBits(StopBits::STOP_BITS_1);
        }

        ~RemoteControl() {
            serial_port.Close();
        }

        struct Command {
            bool   estop         = false;
            double steering      = 0.0;
            double throttle      = 0.0;
            double pan_deg       = 0.0;
            double tilt_deg      = 0.0;
            TurboMode turbo_mode = TurboMode::FIRST_GEAR;
        };

        Command poll() {
            // Seek to start of a line.
            const int timeout_ms = 2000;
            {
                char c = 0;
                while(c != 'S') {
                    serial_port.ReadByte(c, timeout_ms);
                }
            }

            std::string line;
            {
                char c = 0;
                int i=64;
                while(i>0) {
                    serial_port.ReadByte(c, timeout_ms);
                    if(c == 'E') { break; }
                    line += c;
                    i--;
                }
            }

            // Parse numbers out of the command string.
            std::vector<int> raw_cmds(NUM_CHANNELS);
            {
                size_t prev_comma = -1;
                size_t curr_comma = 0;
                for(int chan=0; chan < NUM_CHANNELS; ++chan) {
                    curr_comma = line.find_first_of(",", prev_comma+1);
                    int chan_num = std::stoi(line.substr(prev_comma+1, curr_comma-prev_comma));
                    raw_cmds[chan] = chan_num;
                    prev_comma = curr_comma;
                }
            }

            fmt::print("{} {} {} {} {} {} {} {}\n",
                    raw_cmds[0],
                    raw_cmds[1],
                    raw_cmds[2],
                    raw_cmds[3],
                    raw_cmds[4],
                    raw_cmds[5],
                    raw_cmds[6],
                    raw_cmds[7]);

            Command cmd;
            cmd.steering = remap_steering(raw_cmds[STEERING_CHANNEL]);
            cmd.throttle = remap_throttle(raw_cmds[THROTTLE_CHANNEL]);
            cmd.estop = remap_estop(raw_cmds[ESTOP_CHANNEL]);
            cmd.turbo_mode = remap_turbo_mode(raw_cmds[TURBO_MODE_CHANNEL]);
            cmd.pan_deg = remap_pan(raw_cmds[PAN_CHANNEL]);
            cmd.tilt_deg = remap_tilt(raw_cmds[TILT_CHANNEL]);
            return cmd;
        }

    private:
        const std::string port_name;
        LibSerial::SerialPort serial_port;

        double remap_throttle(const int raw_throttle) const {
            return 2*(raw_throttle-THROTTLE_CENTER) / (double)(THROTTLE_MAX - THROTTLE_MIN);
        }
        double remap_steering(const int raw_steering) const {
            return -2*(raw_steering-STEERING_CENTER) / (double)(STEERING_MAX - STEERING_MIN);
        }
        bool remap_estop(const int raw_estop) const {
            return (raw_estop > (ESTOP_MIN+ESTOP_MAX)/2.0);
        }
        TurboMode remap_turbo_mode(const int raw_turbo) const {
            if(raw_turbo < (TURBO_CENTER + TURBO_MIN)/2) {
                return TurboMode::FIRST_GEAR;
            } else if(raw_turbo > (TURBO_CENTER + TURBO_MAX)/2) {
                return TurboMode::THIRD_GEAR;
            } else {
                return TurboMode::SECOND_GEAR;
            }
        }
        double remap_pan(const int raw_pan) const {
            return 2*(raw_pan-PAN_CENTER) / (double)(PAN_MAX - PAN_MIN) * 90.0;
        }
        double remap_tilt(const int raw_tilt) const {
            return -2*(raw_tilt-TILT_CENTER) / (double)(TILT_MAX - TILT_MIN) * 90.0;
        }
};

} // namespace pr
