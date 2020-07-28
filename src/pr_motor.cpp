#include "pr_motor.h"
#include "pr_log.h"
#include "pr_time.h"

#include <Roboteq.h>
#include <fmt/format.h>
#include <stdexcept>

#include <experimental/filesystem>

using namespace roboteq;

namespace pr {

namespace {

constexpr int ENCODER_PPR = 7;
constexpr int MAX_WHEEL_RPM = 12;
constexpr int MOTOR_GEAR_RATIO = 188;
constexpr int MAX_MOTOR_RPM = MAX_WHEEL_RPM * MOTOR_GEAR_RATIO;

// Try to connect to a roboteq on the given port name.
// If successful, query and return the CAN ID of the roboteq.
// If unsuccessful, return -1.
int check_port_for_roboteq(const std::string& port_name) {
    RoboteqDevice dev;
    int status = dev.Connect(port_name);
    if(status != RQ_SUCCESS) {
        return -1;
    }

    int cnod = 0;
    if(dev.GetConfig(_CNOD, cnod) != RQ_SUCCESS) {
        return -1;
    }
    dev.Disconnect();
    return cnod;
}

// Search /dev/tty* for roboteq devices.
// Return: A hashmap mapping Roboteq CAN IDS to port names.
std::unordered_map<int, std::string> find_roboteq_ports() {
    namespace fs = std::experimental::filesystem;

    std::unordered_map<int, std::string> roboteq_ports;

    // Iterate over all files in /dev/
    for(const auto& p: fs::directory_iterator("/dev/")) {
        const std::string path  = p.path().string();

        // Search for paths matching /dev/ttyACM* or /dev/ttyTHS2.
        if(path.find("ttyACM") != std::string::npos ||
           path.find("ttyTHS2") != std::string::npos) {

            // Check candidate paths for roboteq devices.
            int can_id = check_port_for_roboteq(path);

            // If a roboteq is found, add it to the hashmap.
            if(can_id >= 0) {
                // If this roboteq CAN ID is not unique, raise an exception.
                if(roboteq_ports.find(can_id) != roboteq_ports.end()) {
                    throw std::runtime_error("ROBOTEQ Error: Your roboteq devices have not been assigned unique CAN IDs.");
                }
                roboteq_ports[can_id] = path;
            }
        }
    }
    return roboteq_ports;
}
    
} // anonymous namespace

constexpr double TILT_EPSILON = 2.0;
constexpr double PAN_EPSILON = 2.0;

void RQ_CHECK_STATUS(const int status, const std::string& msg) {
    if (status != RQ_SUCCESS) {
        std::string long_msg = fmt::format(msg + ": {}\n", status);
        pr::log_error(long_msg);
        throw std::runtime_error(long_msg);
    }
}

void Roboteq::connect(const std::string& port_name) {
    int status = device.Connect(port_name);
    RQ_CHECK_STATUS(status, fmt::format("Failed to connect to Roboteq on port: {}.", port_name));
}

Roboteq::~Roboteq() {
    device.Disconnect();
}

void Roboteq::use_position_mode(int chan) {
    int status = device.SetConfig(_MMOD, chan, 4);    
    RQ_CHECK_STATUS(status, "Failed to set Pan Motor Operating Mode to Closed-Loop Position Tracking.");
}

void Roboteq::set_position_mode_velocity(int chan, int rpm) {
    int status = device.SetConfig(_MVEL, 2, rpm*10);
    RQ_CHECK_STATUS(status, "Failed to set Pan Motor Position Mode Velocity.");
}

void Roboteq::config_absolute_encoder(int chan, int analog_in, int mv_min, int mv_mid, int mv_max) {
    int status = device.SetConfig(_AINA, chan, chan*16+2);
    RQ_CHECK_STATUS(status, "Failed to set AIN Use to Feedback for the Pan Motor");

    status = device.SetConfig(_AMOD, chan, 1);
    RQ_CHECK_STATUS(status, "Failed to set AIN Conversion Type to Absolute for the Pan Motor");

    status = device.SetConfig(_AMIN, chan, mv_min);
    RQ_CHECK_STATUS(status, "Failed to set AIN min voltage");

    status = device.SetConfig(_ACTR, chan, mv_mid);
    RQ_CHECK_STATUS(status, "Failed to set AIN center voltage");

    status = device.SetConfig(_AMAX, chan, mv_max);
    RQ_CHECK_STATUS(status, "Failed to set AIN max voltage");
}

void Roboteq::config_limit_switches(int chan, int fwd_in, int rev_in) {
    int status = device.SetConfig(_DINL, fwd_in, 0);
    RQ_CHECK_STATUS(status, "Failed to set DIN Active Level High.");

    status = device.SetConfig(_DINL, rev_in, 0);
    RQ_CHECK_STATUS(status, "Failed to set DIN Active Level High.");

    status = device.SetConfig(_DINA, fwd_in, 4+2*chan);
    RQ_CHECK_STATUS(status, "Failed to set DIN use to Forward Limit Switch.");

    status = device.SetConfig(_DINA, rev_in, 4+2*chan);
    RQ_CHECK_STATUS(status, "Failed to set DIN use to Reverse Limit Switch.");
}

int Roboteq::get_encoder_value(int chan) {
    int pos;
    int status = device.GetValue(_F, chan, pos);
    RQ_CHECK_STATUS(status, "Failed to get encoder value.");
    return pos;
}

void Roboteq::use_velocity_mode(int chan) {
    // Set Operating Mode to Closed-Loop Speed
    int status = device.SetConfig(_MMOD, chan, 1);
    RQ_CHECK_STATUS(status, "Failed to set Operating Mode to Closed-Loop Speed.");
}

void Roboteq::use_quadrature_encoder(int chan, int ppr) {
    // Set Encoder Mode to Feedback
    int status = device.SetConfig(_EMOD, chan, chan*16+2);
    RQ_CHECK_STATUS(status, "Failed to set Encoder Mode to Feedback.");

    // Set Encoder Pulses Per Revolution
    status = device.SetConfig(_EPPR, chan, ppr);
    RQ_CHECK_STATUS(status, "Failed to set Encoder Pulses per Revolution.");
}

void Roboteq::set_max_vel(int chan, int vel) {
    // Set Max. Motor RPM
    int status = device.SetConfig(_MXRPM, chan, vel);
    RQ_CHECK_STATUS(status, "Failed to set Max Velocity.");

    // Set Motor Accel. in 0.1 RPM/sec.
    status = device.SetConfig(_MAC, chan, vel*10);
    RQ_CHECK_STATUS(status, "Failed to set Motor Acceleration.");

    // Set Motor Decel. in 0.1 RPM/sec.
    status = device.SetConfig(_MDEC, chan, vel*10);
    RQ_CHECK_STATUS(status, "Failed to set Motor Deceleration.");
}

void Roboteq::disable_watchdog() {
    int status = device.SetConfig(_RWD, 0);
    RQ_CHECK_STATUS(status, "Failed to disable Watchdog on Left Roboteq.");
}
void Roboteq::disable_loop_error_detection(int chan) {
    int status = device.SetConfig(_CLERD, chan, 0);
    RQ_CHECK_STATUS(status, "Failed to disable Loop Error Detection.");
}

void Roboteq::set_motor_cmd(int chan, int cmd, bool force) {
    int& m_cmd = (chan == 1) ? m1_cmd : m2_cmd;
    bool dont_ignore = (m_cmd != 0 && cmd == 0);

    m1_cmd = std::clamp(m1_cmd, -1000, 1000);
    m2_cmd = std::clamp(m2_cmd, -1000, 1000);

    int status = device.SetCommand(_MOTCMD, m1_cmd, m2_cmd);
    RQ_CHECK_STATUS(status, "Failed to set Motor Command.");
    m_cmd = cmd;
}
void Roboteq::set_kp(int chan, float kp) {
    int status = device.SetConfig(_KP, chan, kp*10);
    RQ_CHECK_STATUS(status, "Failed to set Kp.");
}
void Roboteq::set_ki(int chan, float ki) {
    int status = device.SetConfig(_KI, chan, ki*10);
    RQ_CHECK_STATUS(status, "Failed to set Ki.");
}
void Roboteq::set_kd(int chan, float kd) {
    int status = device.SetConfig(_KD, chan, kd*10);
    RQ_CHECK_STATUS(status, "Failed to set Kd.");
}

MotorController::MotorController() {
    // Find the roboteq devices and connect to them.
    auto port_map = find_roboteq_ports();
    if(port_map.find(1) == port_map.end()) {
        throw std::runtime_error("Failed to connect to roboteq 1.\n");
    }
    if(port_map.find(2) == port_map.end()) {
        throw std::runtime_error("Failed to connect to roboteq 2.\n");
    }
    if(port_map.find(3) == port_map.end()) {
        throw std::runtime_error("Failed to connect to roboteq 3.\n");
    }

    left.connect(port_map.at(1));
    ptu.connect(port_map.at(2));
    right.connect(port_map.at(3));

    // Configure the Pan/Tilt Roboteq.
    {
        // Configure Roboteq
        ptu.disable_watchdog();

        // Configure Tilt Motor
        ptu.disable_loop_error_detection(1);
        ptu.use_position_mode(1);
        ptu.set_position_mode_velocity(1, 100);
        ptu.config_absolute_encoder(1, 1, 1400, 2300, 3200);
        ptu.config_limit_switches(1, 5,6);
        ptu.set_kp(1, 2.0);
        ptu.set_ki(1, 0.0);
        ptu.set_kd(1, 0.0);

        // Configure Pan Motor
        ptu.disable_loop_error_detection(2);
        ptu.use_position_mode(2);
        ptu.set_position_mode_velocity(2, 100);
        ptu.config_absolute_encoder(2, 2, 1400, 2300, 3200);
        ptu.config_limit_switches(2, 2,3);
        ptu.set_kp(2, 2.0);
        ptu.set_ki(2, 0.0);
        ptu.set_kd(2, 0.0);
    }

    // Configure the Right Roboteq.
    {
        // Configure Roboteq
        right.disable_watchdog();

        // Configure Front Right Motor.
        right.disable_loop_error_detection(1);
        right.use_velocity_mode(1);
        right.use_quadrature_encoder(1, ENCODER_PPR);
        right.set_max_vel(1, MAX_MOTOR_RPM);
        right.set_kp(1, 16.0);
        right.set_ki(1, 1.0);
        right.set_kd(1, 0.0);

        // Configure Rear Right Motor.
        right.disable_loop_error_detection(2);
        right.use_velocity_mode(2);
        right.use_quadrature_encoder(2, ENCODER_PPR);
        right.set_max_vel(2, MAX_MOTOR_RPM);
        right.set_kp(2, 16.0);
        right.set_ki(2, 1.0);
        right.set_kd(2, 0.0);
    }
    // Configure the Left Roboteq.
    {
        // Configure Roboteq
        left.disable_watchdog();

        // Configure Front Left Motor.
        left.disable_loop_error_detection(1);
        left.use_velocity_mode(1);
        left.use_quadrature_encoder(1, ENCODER_PPR);
        left.set_max_vel(1, MAX_MOTOR_RPM);
        left.set_kp(1, 16.0);
        left.set_ki(1, 1.0);
        left.set_kd(1, 0.0);

        // Configure Rear Left Motor.
        left.disable_loop_error_detection(2);
        left.use_velocity_mode(2);
        left.use_quadrature_encoder(2, ENCODER_PPR);
        left.set_max_vel(2, MAX_MOTOR_RPM);
        left.set_kp(2, 16.0);
        left.set_ki(2, 1.0);
        left.set_kd(2, 0.0);
    }
}

void MotorController::setPanDeg(const int deg) {
    target_pan_deg = deg;

    // Normalize to [-1000, +1000]
    int cmd = std::clamp<int>(target_pan_deg*1000.0/90.0, -1000, 1000);
    ptu.set_motor_cmd(2, cmd);
}

void MotorController::setTiltDeg(const int deg) {
    target_tilt_deg = deg;

    // Normalize to [-1000, +1000]
    int cmd = std::clamp<int>(target_tilt_deg*1000.0/90.0, -1000, 1000);
    ptu.set_motor_cmd(1, cmd);
}

[[nodiscard]] int MotorController::getPanDeg() {
    return ptu.get_encoder_value(2) * 90.0/1000.0;
}

[[nodiscard]] int MotorController::getTiltDeg() {
    return ptu.get_encoder_value(1) * 90.0/1000.0;
}

void MotorController::setLeftVelocity(const double vel) {
    velocity = vel;

    int cmd = vel*1000;
    left.set_motor_cmd(1, cmd);
    left.set_motor_cmd(2, cmd);
}
void MotorController::setRightVelocity(const double vel) {
    velocity = vel;

    int cmd = vel*1000;
    right.set_motor_cmd(1, -cmd);
    right.set_motor_cmd(2, -cmd);
}

} // namespace pr
