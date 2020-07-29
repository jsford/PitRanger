#include "pr_ptu.h"
#include "pr_log.h"
#include "pr_time.h"
#include <Roboteq.h>
#include <cassert>

using namespace roboteq;

namespace pr {

PanTiltController::PanTiltController() {
    // Connect to the Roboteq.
    if( device.Connect(PTU_SERIAL_PORT) != RQ_SUCCESS ) {
        bool connected = false;
        int num_retries = 5;
        for(int retry = 1; retry <= num_retries; ++retry) {
            auto msg = fmt::format("Failed to connect to pan/tilt roboteq on port {}. Retrying [{}/5].", PTU_SERIAL_PORT, retry);
            pr::log_warn(msg);

            if( device.Connect(PTU_SERIAL_PORT) == RQ_SUCCESS ) {
                connected = true;
                break;
            }
            pr::time::sleep(1);
        }

        if(!connected) {
            auto msg = fmt::format("Failed to connect to pan/tilt roboteq on port {}.", PTU_SERIAL_PORT);
            throw std::runtime_error(msg);
        }
    }

    // Disable the Roboteq Watchdog.
    disable_watchdog();

    // Configure the tilt axis.
    use_position_mode(PTU_TILT_CHANNEL);
    set_position_mode_velocity(PTU_TILT_CHANNEL, PTU_TILT_MOTOR_RPM);
    config_absolute_encoder(PTU_TILT_CHANNEL, PTU_TILT_ENCODER_ANALOG_INPUT, PTU_TILT_ENCODER_MIN, PTU_TILT_ENCODER_CENTER, PTU_TILT_ENCODER_MAX);
    config_limit_switches(PTU_TILT_CHANNEL, PTU_TILT_FWD_LIMIT_DIGITAL_INPUT, PTU_TILT_REV_LIMIT_DIGITAL_INPUT);
    set_kp(PTU_TILT_CHANNEL, PTU_TILT_PID[0]);
    set_ki(PTU_TILT_CHANNEL, PTU_TILT_PID[1]);
    set_kd(PTU_TILT_CHANNEL, PTU_TILT_PID[2]);

    // Configure the pan axis.
    use_position_mode(PTU_PAN_CHANNEL);
    set_position_mode_velocity(PTU_PAN_CHANNEL, PTU_PAN_MOTOR_RPM);
    config_absolute_encoder(PTU_PAN_CHANNEL, PTU_PAN_ENCODER_ANALOG_INPUT, PTU_PAN_ENCODER_MIN, PTU_PAN_ENCODER_CENTER, PTU_PAN_ENCODER_MAX);
    config_limit_switches(PTU_PAN_CHANNEL, PTU_PAN_FWD_LIMIT_DIGITAL_INPUT, PTU_PAN_REV_LIMIT_DIGITAL_INPUT);
    set_kp(PTU_PAN_CHANNEL, PTU_PAN_PID[0]);
    set_ki(PTU_PAN_CHANNEL, PTU_PAN_PID[1]);
    set_kd(PTU_PAN_CHANNEL, PTU_PAN_PID[2]);

    // Drive the motors to their current position to avoid breaking anything.
    {
        auto  pan_deg = get_pan_deg();
        auto tilt_deg = get_tilt_deg();

        pan_cmd = pan_deg*1000/PTU_PAN_MAX_DEG;
        tilt_cmd = tilt_deg*1000/PTU_TILT_MAX_DEG;

        set_pan_deg(pan_deg);
        set_tilt_deg(tilt_deg);
    }
}

PanTiltController::~PanTiltController() {
    device.Disconnect();
}

void PanTiltController::use_position_mode(int chan) {
    int status = device.SetConfig(_MMOD, chan, 4);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set pan/tilt channel {} to closed-loop position tracking mode.", chan);
        throw std::runtime_error(msg);
    }
}
void PanTiltController::set_position_mode_velocity(int chan, int rpm) {
    int status = device.SetConfig(_MVEL, chan, rpm*10);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set pan/tilt channel {} velocity to {}.", chan, rpm);
        throw std::runtime_error(msg);
    }
}

void PanTiltController::config_absolute_encoder(int chan, int analog_in, int mv_min, int mv_mid, int mv_max) {
    int status = device.SetConfig(_AINA, chan, chan*16+2);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set AIN{} use to 'Feedback' for pan/tilt channel {}.", analog_in, chan);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_AMOD, chan, 1);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set AIN{} conversion type to 'Absolute' for pan/tilt channel {}.", analog_in, chan);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_AMIN, chan, mv_min);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set AIN{} min voltage to {} for pan/tilt channel {}.", mv_min, analog_in);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_ACTR, chan, mv_mid);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set AIN{} center voltage to {} for pan/tilt channel {}.", mv_mid, analog_in);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_AMAX, chan, mv_max);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set AIN{} max voltage to {} for pan/tilt channel {}.", mv_max, analog_in);
        throw std::runtime_error(msg);
    }
}

void PanTiltController::config_limit_switches(int chan, int fwd_in, int rev_in) {
    int status = device.SetConfig(_DINL, fwd_in, 0);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set DIN{} active level to 'High'.", fwd_in);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_DINL, rev_in, 0);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set DIN{} active level to 'High'.", rev_in);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_DINA, fwd_in, 4+2*chan);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set DIN{} use to 'Forward Limit Switch'.", fwd_in);
        throw std::runtime_error(msg);
    }

    status = device.SetConfig(_DINA, rev_in, 4+2*chan);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set DIN{} use to 'Reverse Limit Switch'.", rev_in);
        throw std::runtime_error(msg);
    }
}

void PanTiltController::disable_watchdog() {
    int status = device.SetConfig(_RWD, 0);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to disable Watchdog on Pan/Tilt Roboteq.");
        throw std::runtime_error(msg);
    }
}

void PanTiltController::disable_loop_error_detection(int chan) {
    int status = device.SetConfig(_CLERD, chan, 0);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to disable Loop Error Detection on Pan/Tilt Roboteq.");
        throw std::runtime_error(msg);
    }
}

void PanTiltController::set_kp(int chan, float kp) {
    int status = device.SetConfig(_KP, chan, kp*10);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set Kp for Pan/Tilt channel {}.", chan);
        throw std::runtime_error(msg);
    }
}
void PanTiltController::set_ki(int chan, float ki) {
    int status = device.SetConfig(_KI, chan, ki*10);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set Ki for Pan/Tilt channel {}.", chan);
        throw std::runtime_error(msg);
    }
}
void PanTiltController::set_kd(int chan, float kd) {
    int status = device.SetConfig(_KD, chan, kd*10);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set Kd for Pan/Tilt channel {}.", chan);
        throw std::runtime_error(msg);
    }
}


void PanTiltController::set_motor_cmd(int chan, int cmd) {
    cmd = std::clamp(cmd, -1000, 1000);

    if( chan == PTU_TILT_CHANNEL ) {
        tilt_cmd = cmd;
    } else if ( chan == PTU_PAN_CHANNEL ) {
        pan_cmd = cmd;
    } else {
        auto msg = fmt::format("Pan/Tilt set_motor_cmd received invalid channel id {}.", chan);
        throw std::runtime_error(msg);
    }

    int status = device.SetCommand(_MOTCMD, tilt_cmd, pan_cmd);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to set the motor command for the Pan/Tilt motor controller.");
        throw std::runtime_error(msg);
    }
}

int PanTiltController::get_encoder_value(int chan) {
    int pos;
    int status = device.GetValue(_F, chan, pos);
    if(status != RQ_SUCCESS) {
        auto msg = fmt::format("Failed to get the encoder value for Pan/Tilt channel {}.", chan);
        throw std::runtime_error(msg);
    }
    return pos;
}

void PanTiltController::set_pan_deg(int deg) {
    // Normalize to [-1000, +1000]
    pan_cmd = std::clamp<int>(deg*1000.0/PTU_PAN_MAX_DEG, -1000, 1000);
    set_motor_cmd(PTU_PAN_CHANNEL, pan_cmd);

    // Note(Jordan): Add some kind of a wait statement here...
}
void PanTiltController::set_tilt_deg(int deg) {
    // Normalize to [-1000, +1000]
    tilt_cmd = std::clamp<int>(deg*1000.0/PTU_TILT_MAX_DEG, -1000, 1000);
    set_motor_cmd(PTU_TILT_CHANNEL, tilt_cmd);

    // Note(Jordan): Add some kind of a wait statement here...
}

int PanTiltController::get_pan_deg() {
    return get_encoder_value(PTU_PAN_CHANNEL) * PTU_PAN_MAX_DEG/1000.0;
}

int PanTiltController::get_tilt_deg() {
    return get_encoder_value(PTU_TILT_CHANNEL) * PTU_TILT_MAX_DEG/1000.0;
}

} // namespace pr
