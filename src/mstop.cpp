#include "pr_motor.h"
#include <fmt/format.h>

int main(int argc, char* argv[])
{
    pr::MotorController motor_controller;

    double left_vel = 0;
    double right_vel = 0;

    if (argc == 3) {
        left_vel  = std::stod(argv[1]);
        right_vel = std::stod(argv[2]);
    }

    fmt::print("Setting Motor Velocities\n");
    fmt::print("Left  {}\n", left_vel);
    fmt::print("Right {}\n", right_vel);

    motor_controller.setLeftVelocity(left_vel);
    motor_controller.setRightVelocity(right_vel);
    motor_controller.setLeftVelocity(left_vel);
    motor_controller.setRightVelocity(right_vel);

	return 0;
}
