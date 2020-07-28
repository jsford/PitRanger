#include "pr_imu.h"
#include "pr_time.h"
#include "pr_log.h"
#include "pr_motor.h"
#include "pr_remote.h"
#include <fmt/format.h>
#include <fmt/ostream.h>

constexpr double ROBOT_WHEEL_DIAMETER = 0.2;
constexpr double ROBOT_WHEELBASE_WIDTH = 0.8;
constexpr double PI = 3.1415926536;

int main(int argc, char* argv[])
{
    pr::RemoteControl rc("/dev/teensy");
    pr::MotorController motor_controller;

    motor_controller.setLeftVelocity(0);
    motor_controller.setRightVelocity(0);

    while(1) {
        // Get commands from the RC Reciever.
        auto cmd = rc.poll();

        // If ESTOP is enabled, set throttle to zero.
        if(cmd.estop) {
            cmd.throttle = 0.0;
            cmd.pan_deg = 0.0;
            cmd.tilt_deg = 0.0;

            motor_controller.setLeftVelocity(0);
            motor_controller.setRightVelocity(0);
        }

        // Set robot speed based on turbo mode.
        // NOTE(Jordan): The receiver is currently broken,
        // so turbo mode will always be FIRST_GEAR.
        double robot_vel = 0.0;
        switch(cmd.turbo_mode) {
            case pr::TurboMode::FIRST_GEAR:
                robot_vel = 1.00;
                break;
            case pr::TurboMode::SECOND_GEAR:
                robot_vel = 1.00;
                break;
            case pr::TurboMode::THIRD_GEAR:
                robot_vel = 1.00;
                break;
            default:
                break;
        }

        // Adjust speed based on throttle.
        cmd.throttle *= robot_vel;

        // Apply a deadband to the throttle to prevent twitching.
        if(std::abs(cmd.throttle) < 0.1) {
            cmd.throttle = 0.0;
        }

        fmt::print("STEER: {}\n", cmd.steering);
        fmt::print("THROTTLE: {}\n", cmd.throttle);
        fmt::print("ESTOP: {}\n", cmd.estop);
        fmt::print("TURBO: {}\n", cmd.turbo_mode);
        fmt::print("PAN: {}\n", cmd.pan_deg);
        fmt::print("TILT: {}\n", cmd.tilt_deg);
        fmt::print("----------------------\n");

        // Compute motor velocities.
        double  left_vel = 0.0;
        double right_vel = 0.0;
        if(cmd.throttle > 0.0) {
            left_vel = cmd.throttle + (cmd.steering<-0.2) * cmd.steering;
            right_vel  = cmd.throttle - (cmd.steering>0.2) * cmd.steering;
        } else {
            left_vel = cmd.throttle - (cmd.steering<-0.2) * cmd.steering;
            right_vel  = cmd.throttle + (cmd.steering>0.2) * cmd.steering;
        }

        // Apply motor velocities.
        motor_controller.setLeftVelocity(left_vel);
        motor_controller.setRightVelocity(right_vel);

        // Update Pan and Tilt
        motor_controller.setPanDeg(cmd.pan_deg);
        motor_controller.setTiltDeg(cmd.tilt_deg);
        pr::time::msleep(50);
    }

	return 0;
}
