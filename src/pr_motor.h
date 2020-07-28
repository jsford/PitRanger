#pragma once

#include <Roboteq.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace pr {

class Roboteq {
    public:
    Roboteq() = default;
    ~Roboteq();

    void connect(const std::string& port_name);

    // Position Mode Configuration
    void use_position_mode(int chan); 
    void set_position_mode_velocity(int chan, int rpm);

    void config_absolute_encoder(int chan, int analog_in, int mv_min, int mv_mid, int mv_max);
    void config_limit_switches(int chan, int fwd_in, int rev_in);

    // Velocity Mode Configuration
    void use_velocity_mode(int chan); 
    void use_quadrature_encoder(int chan, int ppr); 
    void set_max_vel(int chan, int vel);

    // Position or Velocity Mode Commands
    void disable_watchdog();
    void disable_loop_error_detection(int chan); 

    void set_kp(int chan, float kp);
    void set_ki(int chan, float ki);
    void set_kd(int chan, float kd);

    void set_motor_cmd(int chan, int cmd, bool force=false);
    int get_encoder_value(int chan);

    private:
        // Keep track of motor commands and only update if they have changed.
        int m1_cmd = 0;
        int m2_cmd = 0;
        RoboteqDevice device;
};

class MotorController {
    public:
    MotorController();

    void setPanDeg(int deg);
    void setTiltDeg(int deg);

    int getPanDeg();
    int getTiltDeg();

    void setLeftVelocity(const double vel);
    void setRightVelocity(const double vel);

    Roboteq  left;
    Roboteq right;
    Roboteq   ptu;

    private:
        double curvature = 0.0;
        double velocity  = 0.0;
        int target_pan_deg  = 0;
        int target_tilt_deg = 0;
};

} // namespace pr
